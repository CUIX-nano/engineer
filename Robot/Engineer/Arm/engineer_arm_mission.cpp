#include "pyro_core_config.h"
#include "cmsis_os.h"

#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_algo_pid.h"

#include "pyro_rc_hub.h"
#include "engineer_arm_planning_mission.h"
#include "pyro_databoard.h"

#include "arm_rc_command.h"

#include <math.h>

extern "C" void engineer_arm_mission(void* args);
namespace pyro
{
    //定义一个关节旋转轴类，用来封装旋转轴的控制，减少代码量
class axis_control_t 
{ 
    //定义枚举，用来指示当前关节是否有限位
        enum constraint_t
    {
        NO_CONSTRAINT,//无限位，可绕旋转轴自由旋转
        CONSTRAINT//有限位
    };
    protected:
        //该旋转轴绑定的电机
        pyro::motor_base_t* _motor;
        //该旋转轴的位置环PID实例
        pyro::pid_t* _pos_pid;
        //该旋转轴的速度环PID实例
        pyro::pid_t* _rot_pid;

        //限位标志位变量
        constraint_t _constraint=NO_CONSTRAINT;

        //目标位置与目标旋转速度
        float _target_pos;
        float _target_rot;

        //反馈位置，反馈位置偏移量，反馈角速度
        float _feedback_pos;
        float _feedback_pos_offset;
        float _feedback_rot;

        //控制量
        float _control_value;

        //上限位
        float _upper_limit;
        //下限位
        float _lower_limit;

    public:
        //构造函数 需传入绑定的电机以及对应的位置速度环pid
        axis_control_t(pyro::motor_base_t* motor, pyro::pid_t* pos_pid, pyro::pid_t* rot_pid):_motor(motor),_pos_pid(pos_pid),_rot_pid(rot_pid)
        {
            _target_pos = 0.0f;
            _target_rot = 0.0f;
        }
        //由于单片机会一直运行，故生命周期没有结束的时候，不实现析构函数
        ~axis_control_t(){}

        //设定目标位置，传入的是一个引用
        //为什么使用引用? 
        //关节在NO_CONSTRAINT状态下会发生旋转绕圈现象，我们需要确保设定的目标位置始终在[-PI,PI]区间，因此我们传入一个变量的引用，对变量本身的值进行限制
        //关节在CONSTRAINT状态下，存在上下限位，我们需要对目标值进行钳位，将目标位置限制在上下界之间
        void set_target(float& target) 
        {
            _target_pos = target;
            if(_constraint==CONSTRAINT)
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_upper_limit;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_lower_limit;
                }
            }
            else
            {
                if(_target_pos>PI)
                {
                    _target_pos=_target_pos-2*pyro::PI;
                }
                else if(_target_pos<-PI)
                {
                    _target_pos=_target_pos+2*pyro::PI;
                }
            }
            target = _target_pos;
        }

        //更新反馈位置，反馈角速度
        //该处得到的逻辑位置是由实际反馈位置减去反馈位置偏移量得到的，只要偏移量设置合理，不存在逻辑位置在上界以上，下界以下的情况
        //同时在进行偏移量的算术运算过程中，可能会使逻辑位置超过PI或-PI,因此需要再对逻辑位置进行规范化
        void update() 
        {
            _motor->update_feedback();
            _feedback_pos = _motor->get_current_position()-_feedback_pos_offset;
            _feedback_rot = _motor->get_current_rotate();
            if(_feedback_pos>PI)
            {
                _feedback_pos-=2*PI;
            }
            else if(_feedback_pos<-PI)
            {
                _feedback_pos+=2*PI;
            }
        }

        //该函数主要解决在NO_CONSTRAINT状态下逻辑角度与目标角度在PI或-PI周围的情况，该函数会计算出一个能够使位置环PID无论从顺时针经过跳变点(feedback:170,target:-170,func_output:190)或者从逆时针经过跳变点(feedback:-170,target:170,func_output:-190)都会得到与实际经过角度相等的目标角度
        //说的有点不清楚，自己推一遍
        float rot_angle_correction_pi(float feedback,float target,float max,float min)
        {
            if(feedback-target>PI)
		        return target-min+max;
	        else if(target-feedback>PI)
		        return target-max+min;
	        else 
		        return target;
        }
        //实施控制
        //如果是CONSTRAINT状态,直接计算
        //如果是NO_CONSTRAINT状态，需使用本类的inline函数修正目标位置，产生正常平滑的输出角速度

        //很久以前的pid是需要外部传入dt的，现在已经不需要了，可以直接在控制函数中计算dt，懒得改了
        void control(float dt)
        {   
            // _control_value = 0.0f;
            if(_constraint==CONSTRAINT)
            {
                _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            else
            {
                _target_rot = _pos_pid->calculate(rot_angle_correction_pi(_feedback_pos,_target_pos,PI,-PI),_feedback_pos);//有蛊

                // _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            _control_value = _rot_pid->calculate(_target_rot,_feedback_rot);
            _motor->send_torque(_control_value);
        }
        //get方法
        float get_position(){return _feedback_pos;}
        //使能/禁用限位
        void enable_constraint(){_constraint=CONSTRAINT;}
        void disable_constraint(){_constraint=NO_CONSTRAINT;
         _upper_limit=pyro::PI;_lower_limit=-pyro::PI;}
         //手动设置上下限位
        void set_upper_limit(float upper_limit){ _upper_limit=upper_limit;}
        void set_lower_limit(float lower_limit){_lower_limit=lower_limit;}

        //设置反馈位置偏移量
        void set_feedback_pos_offset(float offset){_feedback_pos_offset=offset;}
        //重置位置速度环pid,清除积分
        void reset_pid()
        {
            _pos_pid->clear();
            _rot_pid->clear();
        }
    
};
};

extern pyro::databoard* global_databoard;

//一堆变量，太丑陋了
pyro::axis_control_t *axis1;
pyro::dm_motor_drv_t *axis1_motor;

pyro::axis_control_t *axis2;
pyro::dm_motor_drv_t *axis2_motor;

pyro::axis_control_t *axis3;
pyro::dm_motor_drv_t *axis3_motor;

pyro::axis_control_t *axis4;
pyro::dm_motor_drv_t *axis4_motor;

pyro::axis_control_t *axis5;
pyro::dm_motor_drv_t *axis5_motor;

pyro::axis_control_t *axis6;
pyro::pid_t *axis6_pos_pid ;
pyro::pid_t *axis6_rot_pid ;
pyro::dm_motor_drv_t *axis6_motor;


pyro::dm_motor_drv_t *end_motor;
pyro::pid_t *end_pos_pid;
pyro::pid_t *end_rot_pid;
pyro::axis_control_t *end_axis;

float motor_current_pos[7]={};
float motor_target_pos[7]={};
float motor_current_rot[7]={};
float motor_current_torque[7]={};

float axis_current_pos[6]={};
uint32_t axis_current_pos_id[6]={};

control_target_param_t local_control_target_param;

bool hold_gripper=false;
float gripper_pos = 0.0f;

control_mode_t control_mode;

void engineer_arm_init()
{
    //初始化R1
    //设定位置和速度pid参数
    //生成电机实例
    //设定达秒电机的参数(从达妙调试助手中读得)
    //生成旋转轴实例
    //启用限位
    //设定上下限位
    //设定反馈位置偏移量
    //后续的旋转轴流程类似
    pyro::pid_t *axis1_pos_pid = new pyro::pid_t(12,0,0.0,20,52);
    pyro::pid_t *axis1_rot_pid = new pyro::pid_t(8.8,12.0,0.0,10.0,27);
    axis1_motor = new pyro::dm_motor_drv_t(0x2, 0x1, pyro::bsp_can::can1);
    axis1_motor->set_position_range(-pyro::PI, pyro::PI);
    axis1_motor->set_rotate_range(-52, 52); 
    axis1_motor->set_torque_range(-27, 27);
    axis1 = new pyro::axis_control_t(axis1_motor,axis1_pos_pid,axis1_rot_pid);
    axis1->enable_constraint();
    axis1->set_upper_limit(pyro::PI);
    axis1->set_lower_limit(-2.1599);
    axis1->set_feedback_pos_offset(0.722468);

    //初始化R2
    pyro::pid_t *axis2_pos_pid = new pyro::pid_t(20,0,0.0,20.0,150);
    pyro::pid_t *axis2_rot_pid = new pyro::pid_t(20,0,0.0,20.0,150);
    // pyro::pid_t *axis2_pos_pid = new pyro::pid_t(8,0,0.0,20.0,150);//改pid时一般先复制原来不错的pid参数，然后调整参数
    // pyro::pid_t *axis2_rot_pid = new pyro::pid_t(15,0,0.0,20.0,150);
    axis2_motor = new pyro::dm_motor_drv_t(0x4, 0x3, pyro::bsp_can::can1);
    axis2_motor->set_position_range(-pyro::PI, pyro::PI);
    axis2_motor->set_rotate_range(-150, 150); 
    axis2_motor->set_torque_range(-150, 150);
    axis2 = new pyro::axis_control_t(axis2_motor,axis2_pos_pid,axis2_rot_pid);
    axis2->enable_constraint();
    axis2->set_upper_limit(1);
    axis2->set_lower_limit(-0.8);
    axis2->set_feedback_pos_offset(-2.3293);

    //初始化R3
    pyro::pid_t *axis3_pos_pid = new pyro::pid_t(20,0.00,0.0,5.0,160);
    pyro::pid_t *axis3_rot_pid = new pyro::pid_t(13,0.0,0.0,5.0,40);
    axis3_motor = new pyro::dm_motor_drv_t(0x6, 0x5, pyro::bsp_can::can3);
    axis3_motor->set_position_range(-pyro::PI, pyro::PI);
    axis3_motor->set_rotate_range(-160, 160); 
    axis3_motor->set_torque_range(-40, 40);
    axis3 = new pyro::axis_control_t(axis3_motor,axis3_pos_pid,axis3_rot_pid);
    axis3->enable_constraint();
    axis3->set_upper_limit(1.35);
    axis3->set_lower_limit(-1.2);
    axis3->set_feedback_pos_offset(-0.219);

    //初始化R4，注意R4没有限位是一个自由旋转关节
    pyro::pid_t *axis4_pos_pid = new pyro::pid_t(15.7,0,0.0,6,200);
    pyro::pid_t *axis4_rot_pid = new pyro::pid_t(1.0,0.2,0.001,3,7);
    axis4_motor = new pyro::dm_motor_drv_t(0x8, 0x7, pyro::bsp_can::can3);
    axis4_motor->set_position_range(-pyro::PI, pyro::PI);
    axis4_motor->set_rotate_range(-200, 200); 
    axis4_motor->set_torque_range(-7, 7);
    axis4 = new pyro::axis_control_t(axis4_motor,axis4_pos_pid,axis4_rot_pid);
    axis4->disable_constraint();
    // axis4->set_upper_limit(pyro::PI);
    // axis4->set_lower_limit(-pyro::PI);
    axis4->set_feedback_pos_offset(-0.809);


    // pyro::pid_t *axis5_pos_pid = new pyro::pid_t(6.3,0,0.0,10,30);
    // pyro::pid_t *axis5_rot_pid = new pyro::pid_t(1.0,0.1,0.00,4,8);

    //初始化R5
    pyro::pid_t *axis5_pos_pid = new pyro::pid_t(10.3,0.1,0.0,20,200);
    pyro::pid_t *axis5_rot_pid = new pyro::pid_t(1.0,0.01,0.00,4,7);
    axis5_motor = new pyro::dm_motor_drv_t(0xA, 0x9, pyro::bsp_can::can2);
    axis5_motor->set_position_range(-pyro::PI, pyro::PI);
    axis5_motor->set_rotate_range(-200, 200); 
    axis5_motor->set_torque_range(-7, 7);
    axis5 = new pyro::axis_control_t(axis5_motor,axis5_pos_pid,axis5_rot_pid);
    axis5->enable_constraint();
    axis5->set_upper_limit(pyro::PI/2);
    axis5->set_lower_limit(-pyro::PI/2);
    axis5->set_feedback_pos_offset(2.482);

    //初始化R6
    axis6_pos_pid = new pyro::pid_t(9,0.0,0.0,0.0,200);
    axis6_rot_pid = new pyro::pid_t(0.8,0.1,0.0,1,7);
    axis6_motor = new pyro::dm_motor_drv_t(0xC, 0xB, pyro::bsp_can::can2);
    axis6_motor->set_position_range(-pyro::PI, pyro::PI);
    axis6_motor->set_rotate_range(-200, 200); 
    axis6_motor->set_torque_range(-7, 7);
    axis6 = new pyro::axis_control_t(axis6_motor,axis6_pos_pid,axis6_rot_pid);
    axis6->enable_constraint();
    axis6->set_upper_limit(pyro::PI/2);
    axis6->set_lower_limit(-pyro::PI/2);
    axis6->set_feedback_pos_offset(2.7755);

    //初始化末端
    end_pos_pid = new pyro::pid_t(9,0.0,0.0,0,200);
    end_rot_pid = new pyro::pid_t(0.8,0.0,0.0,0,7);
    end_motor = new pyro::dm_motor_drv_t(0xE, 0xD, pyro::bsp_can::can2);
    end_motor->set_position_range(-pyro::PI, pyro::PI);
    end_motor->set_rotate_range(-200, 200); 
    end_motor->set_torque_range(-7, 7);
    end_axis = new pyro::axis_control_t(end_motor,end_pos_pid,end_rot_pid);
    end_axis->enable_constraint();
    end_axis->set_upper_limit(pyro::PI);
    end_axis->set_lower_limit(-pyro::PI);
    end_axis->set_feedback_pos_offset(-0.3560);

    while(rc_planning_sem == nullptr)
        vTaskDelay(1);

}

void enigneer_arm_update()
{
    // axis1_motor->update_feedback();
    // axis2_motor->update_feedback();
    // axis3_motor->update_feedback();
    // axis4_motor->update_feedback();
    // axis5_motor->update_feedback();
    // axis6_motor->update_feedback();
    // end_motor->update_feedback();

    //更新旋转轴
    axis1->update();
    axis2->update();
    axis3->update();
    axis4->update();
    axis5->update();
    axis6->update();
    end_axis->update();
    
    //读取电机的当前位置和旋转角度 方便调试
    motor_current_pos[0]=axis1_motor->get_current_position();
    motor_current_pos[1]=axis2_motor->get_current_position();
    motor_current_pos[2]=axis3_motor->get_current_position();
    motor_current_pos[3]=axis4_motor->get_current_position();
    motor_current_pos[4]=axis5_motor->get_current_position();
    motor_current_pos[5]=axis6_motor->get_current_position();
    motor_current_pos[6]=end_motor->get_current_position();

    motor_current_rot[0]=axis1_motor->get_current_rotate();
    motor_current_rot[1]=axis2_motor->get_current_rotate();
    motor_current_rot[2]=axis3_motor->get_current_rotate();
    motor_current_rot[3]=axis4_motor->get_current_rotate();
    motor_current_rot[4]=axis5_motor->get_current_rotate();
    motor_current_rot[5]=axis6_motor->get_current_rotate();
    motor_current_rot[6]=end_motor->get_current_rotate();

    motor_current_torque[0]=axis1_motor->get_current_torque();
    motor_current_torque[1]=axis2_motor->get_current_torque();
    motor_current_torque[2]=axis3_motor->get_current_torque();
    motor_current_torque[3]=axis4_motor->get_current_torque();
    motor_current_torque[4]=axis5_motor->get_current_torque();
    motor_current_torque[5]=axis6_motor->get_current_torque();
    motor_current_torque[6]=end_motor->get_current_torque();

    //获取旋转轴当前逻辑位置
    axis_current_pos[0]=axis1->get_position();
    axis_current_pos[1]=axis2->get_position();
    axis_current_pos[2]=axis3->get_position();
    axis_current_pos[3]=axis4->get_position();
    axis_current_pos[4]=axis5->get_position();
    axis_current_pos[5]=axis6->get_position();

    //发布旋转轴当前逻辑位置
    global_databoard->write_topic(axis_current_pos_id[0],
            *((pyro::genenral_data_t*)&(axis_current_pos[0])));
    global_databoard->write_topic(axis_current_pos_id[1],
            *((pyro::genenral_data_t*)&(axis_current_pos[1])));
    global_databoard->write_topic(axis_current_pos_id[2],
            *((pyro::genenral_data_t*)&(axis_current_pos[2])));
    global_databoard->write_topic(axis_current_pos_id[3],
            *((pyro::genenral_data_t*)&(axis_current_pos[3])));
    global_databoard->write_topic(axis_current_pos_id[4],
            *((pyro::genenral_data_t*)&(axis_current_pos[4])));
    global_databoard->write_topic(axis_current_pos_id[5],
            *((pyro::genenral_data_t*)&(axis_current_pos[5])));

    //将旋转轴当前逻辑位置拷贝到规划线程中的参数(当前逻辑位置)中，方便对机械臂进行规划
    //进行进程间的较大量，实时性要求较高的数据交互使用互斥锁
    xSemaphoreTake(rc_planning_sem, portMAX_DELAY);
    //获取锁
    memcpy(control_target_param->axis_current_pos,axis_current_pos,sizeof(float)*6);
    bool arm_is_ready = true;
    arm_is_ready &= axis1_motor->is_enable();
    arm_is_ready &= axis2_motor->is_enable();
    arm_is_ready &= axis3_motor->is_enable();
    arm_is_ready &= axis4_motor->is_enable();
    arm_is_ready &= axis5_motor->is_enable();
    arm_is_ready &= axis6_motor->is_enable();
    arm_is_ready &= end_motor->is_enable();
    control_target_param->arm_is_ready = arm_is_ready;
    xSemaphoreGive(rc_planning_sem);
    //释放锁
}

//const  pyro::dr16_drv_t::dr16_ctrl_t *rc_data;
void engineer_arm_set_control()
{
    //从规划线程中获取控制参数，对机械臂进行控制，由获取的参数决定对机械臂的控制方式
    xSemaphoreTake(rc_planning_sem, portMAX_DELAY);
    memcpy(&local_control_target_param,control_target_param,sizeof(control_target_param_t));
    xSemaphoreGive(rc_planning_sem);
    //持有互斥锁的时间要短，因此将控制参数拷贝到本地后直接释放互斥锁，直接对本地拷贝做后续操作
    control_mode = local_control_target_param.control_mode;
    //获取控制模式
    axis1->set_target(local_control_target_param.axis_target_pos[0]);
    axis2->set_target(local_control_target_param.axis_target_pos[1]);
    axis3->set_target(local_control_target_param.axis_target_pos[2]);
    axis4->set_target(local_control_target_param.axis_target_pos[3]);
    axis5->set_target(local_control_target_param.axis_target_pos[4]);
    axis6->set_target(local_control_target_param.axis_target_pos[5]);
    //设定关节的目标位置
    hold_gripper = local_control_target_param.hold_gripper;
    // gripper_pos += local_control_target_param.gripper_increment;
    // float temp = 0;
    
    //根据关节的控制模式来决定关节的控制位置
    if(local_control_target_param.gripper_mode == Biopolar)
    {
        //双极型控制
        //夹爪有打开和关闭两种状态
         if(hold_gripper)
            gripper_pos = 0;
        else
            gripper_pos = 1.6;
        //位置被硬编码进代码中，不太好理解
    }
    else if(local_control_target_param.gripper_mode == Analog)
    {
        //模拟控制，连续位置控制
        gripper_pos = local_control_target_param.gripper_pos;
    }
    end_axis->set_target(gripper_pos);
}

void engineer_arm_zeroforce()
{
    //将所有旋转轴的PID积分重置为0
    axis1->reset_pid();
    axis2->reset_pid();
    axis3->reset_pid();
    axis4->reset_pid();
    axis5->reset_pid();
    axis6->reset_pid();
    end_axis->reset_pid();

    //发送零力矩
    if(!axis1_motor->is_enable())
        axis1_motor->enable();
    else
        axis1_motor->send_torque(0);

    if(!axis2_motor->is_enable())
        axis2_motor->enable();
    else
        axis2_motor->send_torque(0);

    if(!axis3_motor->is_enable())
        axis3_motor->enable();
    else
        axis3_motor->send_torque(0);

    if(!axis4_motor->is_enable())
        axis4_motor->enable();
    else
        axis4_motor->send_torque(0);

    if(!axis5_motor->is_enable())
        axis5_motor->enable();
    else
        axis5_motor->send_torque(0);

    if(!axis6_motor->is_enable())
        axis6_motor->enable();
    else
        axis6_motor->send_torque(0);

    if(!end_motor->is_enable())
        end_motor->enable();
    else
        end_motor->send_torque(0);
}

void engineer_arm_control()
{
    if(!axis1_motor->is_enable())
        axis1_motor->enable();
    else
        axis1->control(0.005);
    
    if(!axis2_motor->is_enable())
        axis2_motor->enable();
    else
        axis2->control(0.005);

    if(!axis3_motor->is_enable())
        axis3_motor->enable();
    else
        axis3->control(0.005);

    if(!axis4_motor->is_enable())
        axis4_motor->enable();
    else
        axis4->control(0.005);

    if(!axis5_motor->is_enable())
        axis5_motor->enable();
    else
        axis5->control(0.005);
    
    if(!axis6_motor->is_enable())
        axis6_motor->enable();
    else
        axis6->control(0.005);

    if(!end_motor->is_enable())
        end_motor->enable();
    else
        end_axis->control(0.005);
}

//质量参数，该参数我当时调的时候没有做仿真去算，而是调试得来
constexpr float axis2_m = 10.5;
constexpr float axis3_m = 8;
constexpr float axis4_m = 0.05;
constexpr float axis5_m = 0.3;


float axis2_gravity_rad = 0;
float axis3_gravity_rad = 0;
float axis4_gravity_rad = 0;
float axis5_gravity_rad = 0;

float axis2_compensation_torque = 0;
float axis3_compensation_torque = 0;
float axis4_compensation_torque = 0;
float axis5_compensation_torque = 0;

//重力补偿模式，在该模式下，机械臂将会对关节进行重力补偿
//该模式主要用于用于调试机械臂的固定动作，减少改变机械臂位置所需要的力以及能够使机械臂静止在某个位置，方便进行调试
//该重力补偿在R2角度比较大时效果较好，能较好地停在某个位置，但R2离设定零点较近时，会向前倾，还需要进一步完善
void engineer_arm_gravcomp()
{
    //获取计算重力补偿所需要的弧度，需自行推导
    axis2_gravity_rad = axis2->get_position();
    axis3_gravity_rad = pyro::PI/2-axis2->get_position()+axis3->get_position();
    axis4_gravity_rad = axis4->get_position();
    axis5_gravity_rad = axis3_gravity_rad + axis5->get_position();

    //根据力学分解计算对应力矩
    axis5_compensation_torque = -sin(axis5_gravity_rad)*axis5_m*cos(axis4_gravity_rad);
    axis4_compensation_torque = -cos(axis4_gravity_rad)*axis4_m;
    axis3_compensation_torque = -sin(axis3_gravity_rad)*axis3_m;
    axis2_compensation_torque = -sin(axis2_gravity_rad)*axis2_m-0.1*axis3_compensation_torque;

    //发送力矩
    axis1_motor->send_torque(0);
    axis2_motor->send_torque(axis2_compensation_torque);
    axis3_motor->send_torque(axis3_compensation_torque);
    axis4_motor->send_torque(axis4_compensation_torque);
    axis5_motor->send_torque(axis5_compensation_torque);
    axis6_motor->send_torque(0);
    end_motor->send_torque(0);
}

void engineer_arm_mission(void* args)
{
    osDelay(10);

    engineer_arm_init();

    //等待数据板初始化完成
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }

    //获取旋转轴当前逻辑位置的topic id
    axis_current_pos_id[0] = global_databoard->get_topic_id("axis1_current_pos");
    axis_current_pos_id[1] = global_databoard->get_topic_id("axis2_current_pos");
    axis_current_pos_id[2] = global_databoard->get_topic_id("axis3_current_pos");
    axis_current_pos_id[3] = global_databoard->get_topic_id("axis4_current_pos");
    axis_current_pos_id[4] = global_databoard->get_topic_id("axis5_current_pos");
    axis_current_pos_id[5] = global_databoard->get_topic_id("axis6_current_pos");

    //固定延时，等达妙电机上电完毕之后再对电机进行使能
    

    for(;;)
    {
        enigneer_arm_update();
        engineer_arm_set_control();
        if(control_mode==ZERO_FORCE)
        {
            engineer_arm_zeroforce();
        }
        else if(control_mode==POSITION_CONTROL)
        {
            engineer_arm_control();
            // engineer_arm_zeroforce();
        }
        else if(control_mode == TORQUE_COMPENSATION)
        {
            engineer_arm_gravcomp();
        }
        vTaskDelay(1);
    }
}

