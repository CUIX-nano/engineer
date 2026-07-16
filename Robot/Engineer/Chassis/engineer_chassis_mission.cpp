#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_rc_hub.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_algo_pid.h"
#include "pyro_power_control_drv.h"
#include <math.h>
#include "pyro_dwt_drv.h"
#include "joint_ctrl.h"
#include "pyro_bsp_can.h"          // 新增：BSP CAN
#include "pyro_rc_core.h"      // 新增：RC 控制枚举（sw_pos_t）

extern pyro::databoard* global_databoard; 

namespace pyro
{
class wheel_control_t 
{ 
    
   typedef enum 
    {
        FORWARD,
        REVERSE
    }
    wheel_ward_t;
    protected:
        wheel_ward_t _ward;
        pyro::motor_base_t* _motor;
        pyro::pid_t* _rot_pid;

        float _target_rot;
        float _feedback_rot;
        float _control_value;
    public:
    
        wheel_control_t(pyro::motor_base_t* motor,  pyro::pid_t* rot_pid):_motor(motor),_rot_pid(rot_pid)
        {
            _target_rot = 0.0f;
            _ward = FORWARD;
        }
        ~wheel_control_t(){}
        void set_target(float& target) 
        {
            if(_ward == FORWARD){_target_rot = target;}
            else{_target_rot = -target;}
        }
        void update() 
        {
            _motor->update_feedback();
            _feedback_rot = _motor->get_current_rotate();
        }
        float control(float dt)
        {   
            float control_value = 0.0f;
            control_value = _rot_pid->calculate(_target_rot,_feedback_rot);
            // _motor->send_torque(control_value);
            return control_value;
        }
        void set_forward() { _ward = FORWARD;}
        void set_reverse() {_ward = REVERSE;}
};
class axis_control_t 
{ 
        enum constraint_t
    {
        NO_CONSTRAINT,
        CONSTRAINT
    };
    protected:
        pyro::motor_base_t* _motor;
        pyro::pid_t* _pos_pid;
        pyro::pid_t* _rot_pid;

        constraint_t _constraint=NO_CONSTRAINT;

        float _target_pos;
        float _target_rot;

        float _feedback_pos;
        float _feedback_pos_offset;
        float _feedback_rot;

        float _control_value;

        float _upper_limit;
        float _lower_limit;

    public:
        axis_control_t(pyro::motor_base_t* motor, pyro::pid_t* pos_pid, pyro::pid_t* rot_pid):_motor(motor),_pos_pid(pos_pid),_rot_pid(rot_pid)
        {
            _target_pos = 0.0f;
            _target_rot = 0.0f;
        }
        ~axis_control_t(){}
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
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_target_pos-2*pyro::PI;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_target_pos+2*pyro::PI;
                }
            }
            target = _target_pos;
        }
        void increment(float increment)
        {
            _target_pos = _target_pos+increment;
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
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_target_pos-2*pyro::PI;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_target_pos+2*pyro::PI;
                }
            }
        }
        void update() 
        {
            _motor->update_feedback();
            _feedback_pos = _motor->get_current_position()-_feedback_pos_offset;
            _feedback_rot = _motor->get_current_rotate();
            if(_feedback_pos>10)
            {
                _feedback_pos-=2*10;
            }
            else if(_feedback_pos<-10)
            {
                _feedback_pos+=2*10;
            }
        }
        inline float rot_angle_correction_pi(float feedback,float target,float max,float min)
        {
            if(feedback-target>PI)
		        return target-min+max;
	        else if(target-feedback>PI)
		        return target-max+min;
	        else 
		        return target;
        }
        void control(float dt)
        {   
            float control_value = 0.0f;
            if(_constraint==CONSTRAINT)
            {
                _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            else
            {
                _target_rot = _pos_pid->calculate(rot_angle_correction_pi(_feedback_pos,_target_pos,PI,-PI),_feedback_pos);
                // _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            control_value = _rot_pid->calculate(_target_rot,_feedback_rot);
            _motor->send_torque(control_value);
        }
        float get_position(){return _feedback_pos;}
        void enable_constraint(){_constraint=CONSTRAINT;}
        void disable_constraint(){_constraint=NO_CONSTRAINT;
         _upper_limit=pyro::PI;_lower_limit=-pyro::PI;}
        void set_upper_limit(float upper_limit){ _upper_limit=upper_limit;}
        void set_lower_limit(float lower_limit){_lower_limit=lower_limit;}

        void set_feedback_pos_offset(float offset){_feedback_pos_offset=offset;}
    
};



};

typedef enum
{
    ZERO_FORCE = 0,
    RC_CONTROL
}
chassis_mode_t;
chassis_mode_t chassis_mode = ZERO_FORCE,last_chassis_mode = ZERO_FORCE;

float joint_increment[4] = {0.0f,0.0f,0.0f,0.0f};

class joint_group_t
{
    private:
        joint_control_t* joints[4];//fl fr bl br
    public:
        joint_group_t()
        {
            // 修改 CAN 枚举为 bsp_can
            pyro::dji_m3508_motor_drv_t* jm_drv_fl = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1, pyro::bsp_can::can3);
            pyro::pid_t *rot_pid_fl = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,20.0f);
            pyro::pid_t *pos_pid_fl = new pyro::pid_t(9.5f,0.0f,0.0f,0.0f,80.0f);
            joints[0] = new joint_control_t(jm_drv_fl,pos_pid_fl,rot_pid_fl);
            joints[0]->set_direction(-1.0f);
            joints[0]->set_calibrate_parameters(-30.0f,2.0f,10.0f);
            joints[0]->set_upper_limit(80);
            joints[0]->set_lower_limit(0);

            pyro::dji_m3508_motor_drv_t* jm_drv_fr = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2, pyro::bsp_can::can3);
            pyro::pid_t *rot_pid_fr = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,20.0f);
            pyro::pid_t *pos_pid_fr = new pyro::pid_t(9.5f,0.0f,0.0f,0.0f,80.0f);
            joints[1] = new joint_control_t(jm_drv_fr,pos_pid_fr,rot_pid_fr);
            joints[1]->set_direction(1.0f);
            joints[1]->set_calibrate_parameters(-30.0f,2.0f,10.0f);
            joints[1]->set_upper_limit(80);
            joints[1]->set_lower_limit(0);

            pyro::dm_motor_drv_t* jm_drv_bl = new pyro::dm_motor_drv_t(0x02,0x03, pyro::bsp_can::can2);
            jm_drv_bl->set_position_range(-pyro::PI,pyro::PI);
            jm_drv_bl->set_rotate_range(-52,52);
            jm_drv_bl->set_torque_range(-27,27);
            pyro::pid_t *rot_pid_bl = new pyro::pid_t(2.0f, 0.005f, 0.0012f, 0.5f, 27.0f);
            pyro::pid_t *pos_pid_bl = new pyro::pid_t(14.0f, 0.005f, 0.0012f, 0.5f, 52.0f);
            joints[2] = new joint_control_t(jm_drv_bl,pos_pid_bl,rot_pid_bl);
            joints[2]->set_direction(-1.0f);
            joints[2]->set_upper_limit(8);
            joints[2]->set_lower_limit(0);

            pyro::dm_motor_drv_t* jm_drv_br = new pyro::dm_motor_drv_t(0x00,0x01, pyro::bsp_can::can2);
            jm_drv_br->set_position_range(-pyro::PI,pyro::PI);
            jm_drv_br->set_rotate_range(-52,52);
            jm_drv_br->set_torque_range(-27,27);
            pyro::pid_t *rot_pid_br = new pyro::pid_t(2.0f, 0.005f, 0.0012f, 0.5f, 27.0f);
            pyro::pid_t *pos_pid_br = new pyro::pid_t(14.0f, 0.005f, 0.0012f, 0.5f, 52.0f);
            joints[3] = new joint_control_t(jm_drv_br,pos_pid_br,rot_pid_br);
            joints[3]->set_upper_limit(8);
            joints[3]->set_lower_limit(0);
        }
        ~joint_group_t(){}
        void update()
        {
            for(int i=0;i<4;i++)
            {
                joints[i]->update();
            }
        }

        void set_increment(float target[4])
        {
            for(int i=0;i<4;i++)
            {
                joints[i]->increment(target[i]);
            }
        }

        void set_target(float target[4])
        {
            for(int i=0;i<4;i++)
            {
                joints[i]->set_target(target[i]);
            }
        }
        void zero_force()
        {
            for(int i=0;i<4;i++)
            {
                joints[i]->zero_force();
            }   
        }
        void control()
        {
            for(int i=0;i<4;i++)
            {
                if(joints[i]->is_calibrate())
                    joints[i]->calibrate_control();
                else
                    joints[i]->normal_control();
            }
        }
        void start_calibrate()
        {
            for(int i=0;i<4;i++)
            {
                joints[i]->start_calibrate();
            }
        }
};

joint_group_t* joint_group;



static uint32_t 
rc_sw_l_topic_id,
rc_sw_r_topic_id,
rc_ch_lx_topic_id,
rc_ch_ly_topic_id,
rc_ch_rx_topic_id,
rc_ch_ry_topic_id,
motor_torque_topic_id,
motor_rotate_topic_id;

typedef struct
{
    uint32_t sw_l;
    uint32_t sw_r;
    float ch_lx;
    float ch_ly;
    float ch_rx;
    float ch_ry;
}rc_data_t;

rc_data_t rc_data;

uint32_t timestamp;

pyro::dji_m3508_motor_drv_t *wheel_motor_fl,*wheel_motor_fr,*wheel_motor_br,*wheel_motor_bl;
pyro::pid_t *rot_pid_fl,*rot_pid_fr,*rot_pid_br,*rot_pid_bl;
pyro::wheel_control_t *wheel_control_fl,*wheel_control_fr,*wheel_control_br,*wheel_control_bl;

pyro::dm_motor_drv_t *left_side_joint_motor_drv,*right_side_joint_motor_drv;
pyro::pid_t *left_side_joint_pos_pid,*right_side_joint_pos_pid;
pyro::pid_t *left_side_joint_rot_pid,*right_side_joint_rot_pid;
pyro::axis_control_t *left_side_joint_control,*right_side_joint_control;


pyro::dm_motor_drv_t *left_side_front_joint_motor_drv,*right_side_front_joint_motor_drv;
pyro::pid_t *left_side_front_joint_pos_pid,*right_side_front_joint_pos_pid;
pyro::pid_t *left_side_front_joint_rot_pid,*right_side_front_joint_rot_pid;
joint_control_t *left_side_front_joint_control,*right_side_front_joint_control;

float wheel_motor_current_position[4];
float wheel_motor_current_rotate[4];
float wheel_motor_current_torque[4];

float joint_motor_current_position[2];// l r
float joint_motor_current_rotate[2];//l r
float joint_motor_current_torque[2];//l r


//解析遥控器数据并设置控制量
bool chassis_rc_solve()
{
    pyro::topic::data_status_t rc_data_status;
    rc_data_status = global_databoard->read(rc_sw_l_topic_id,(pyro::genenral_data_t*)&(rc_data.sw_l),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_sw_r_topic_id,(pyro::genenral_data_t*)&(rc_data.sw_r),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_ch_lx_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_lx),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_ch_ly_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_ly),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_ch_rx_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_rx),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_ch_ry_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_ry),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    return true;
}




//麦克纳姆轮逆解
int mecanum_inverse_kinematics(
    const float target_vel[3],
    float wheel_vel[4]) 
{
    // 参数检查
    if (target_vel == NULL || wheel_vel == NULL) {
        return -1;
    }
    float vx = target_vel[0];
    float vy = target_vel[1];
    float omega = target_vel[2];
    // 计算旋转半径
    float R = 0.21 + 0.21;
    // 逆运动学公式：
    // 左前轮：vx + vy + omega*R
    // 右前轮：vx - vy - omega*R
    // 右后轮：vx + vy - omega*R
    // 左后轮：vx - vy + omega*R
    //X型
    wheel_vel[0] = vx + vy + omega * R;  // 左前轮
    wheel_vel[1] = vx - vy - omega * R;  // 右前轮
    wheel_vel[2] = vx + vy - omega * R;  // 右后轮
    wheel_vel[3] = vx - vy + omega * R;  // 左后轮
    return 0;
}

float speed_vector[3];
float wheel_vel[4];


extern uint8_t overpass_pose;
//设置底盘控制量
void chassis_set_control()
{
    //设定控制模式
    if(!chassis_rc_solve())
        chassis_mode = ZERO_FORCE;
    // 将 rc_data.sw_r 转为 sw_pos_t 进行比较
    pyro::sw_pos_t sw_r = static_cast<pyro::sw_pos_t>(rc_data.sw_r);
    if (sw_r == pyro::sw_pos_t::MID || sw_r == pyro::sw_pos_t::DOWN)
    {
        chassis_mode = RC_CONTROL;
    }
    else
    {
        chassis_mode = ZERO_FORCE;
    }
    //设定目标速度矢量
    speed_vector[0] = rc_data.ch_ly*800;
    speed_vector[1] = rc_data.ch_lx*800;
    speed_vector[2] = rc_data.ch_rx*1200;

    //逆解出各轮电机速度
    mecanum_inverse_kinematics(speed_vector,wheel_vel);
    wheel_control_fl->set_target(wheel_vel[0]);
    wheel_control_fr->set_target(wheel_vel[1]);
    wheel_control_br->set_target(wheel_vel[2]);
    wheel_control_bl->set_target(wheel_vel[3]);

    pyro::sw_pos_t sw_l = static_cast<pyro::sw_pos_t>(rc_data.sw_l);
    if(sw_l == pyro::sw_pos_t::UP)
    {
        for(int i = 0; i < 4; i++)
        {
            joint_increment[i] = 0.0f;
        }
    }
    // else if(sw_l == pyro::sw_pos_t::MID)
    // {
    //     for(int i = 2; i < 4; i++)
    //     {
    //         joint_increment[i] = 0.0f;
    //     }
    //     joint_increment[0] = rc_data.ch_ry*0.05f;
    //     joint_increment[1] = rc_data.ch_ry*0.05f;
    // }
    // else if(sw_l == pyro::sw_pos_t::DOWN)
    // {
    //     for(int i = 0; i < 2; i++)
    //     {
    //         joint_increment[i] = 0.0f;
    //     }
    //     joint_increment[2] = rc_data.ch_ry*0.005f;
    //     joint_increment[3] = rc_data.ch_ry*0.005f;
    // }

    // joint_group->set_target(joint_increment);
    if(chassis_mode != ZERO_FORCE)
    {
        if(overpass_pose == 0)
        {
            joint_group->set_target(0);
        }
        else if(overpass_pose == 1)
        {
            float temp[4] = {6.0f,6.0,6.0,6.0};
            joint_group->set_target(temp);
        }
    }

    //当底盘从无力变为有力时，开始关节校准
    if(last_chassis_mode == ZERO_FORCE && chassis_mode != ZERO_FORCE){
        joint_group->start_calibrate();
    }
    
    
    last_chassis_mode = chassis_mode;
}

//校准时，关节电机发送恒扭矩



//底盘无力
void chassis_zero_force()
{
    wheel_motor_fl->send_torque(0.0f);
    wheel_motor_fr->send_torque(0.0f);
    wheel_motor_bl->send_torque(0.0f);
    wheel_motor_br->send_torque(0.0f);
}

pyro::power_control_drv_t& power_controller = pyro::power_control_drv_t::get_instance(4);

float control_torque[4]={};
pyro::power_control_drv_t::motor_data_t _motor_data[4];

//底盘功控
void chassis_power_control()
{
    //获取未进行功控前的扭矩电流
    _motor_data[0].torque_cmd = control_torque[0];
    _motor_data[1].torque_cmd = control_torque[1];
    _motor_data[2].torque_cmd = control_torque[2];
    _motor_data[3].torque_cmd = control_torque[3];

    //获取未进行功控前的扭矩转速
    _motor_data[0].gyro = wheel_motor_current_rotate[0];
    _motor_data[1].gyro = wheel_motor_current_rotate[1];
    _motor_data[2].gyro = wheel_motor_current_rotate[2];
    _motor_data[3].gyro = wheel_motor_current_rotate[3];

    for(int i = 1; i <= 4; i++)
    {
        //功率预测
        _motor_data[i-1].power_predict = power_controller.motor_power_predict(i,_motor_data[i-1].torque_cmd,_motor_data[i-1].gyro);
    }
    //计算功率控制后扭矩电流
    power_controller.calculate_restricted_torques(_motor_data,4,110);
    for(int i = 0; i < 4; i++)
    {
        control_torque[i] = _motor_data[i].restricted_torque;
    }
}
void chassis_rc_control()
{
    wheel_control_fl->update();
    wheel_control_fr->update();
    wheel_control_br->update();
    wheel_control_bl->update();

    control_torque[0] = wheel_control_fl->control(0.01f);
    control_torque[1] = wheel_control_fr->control(0.01f);
    control_torque[2] = wheel_control_br->control(0.01f);
    control_torque[3] = wheel_control_bl->control(0.01f);

    chassis_power_control();//不注释为限功率，注释为无工控

    wheel_motor_fl->send_torque(control_torque[0]);
    wheel_motor_fr->send_torque(control_torque[1]);
    wheel_motor_br->send_torque(control_torque[2]);
    wheel_motor_bl->send_torque(control_torque[3]);
}

extern "C" void engineer_chassis_mission(void const *argument)
{
    rc_sw_l_topic_id = global_databoard->get_topic_id("rc_sw_l");
    rc_sw_r_topic_id = global_databoard->get_topic_id("rc_sw_r");
    rc_ch_lx_topic_id = global_databoard->get_topic_id("rc_ch_lx");
    rc_ch_ly_topic_id = global_databoard->get_topic_id("rc_ch_ly");
    rc_ch_rx_topic_id = global_databoard->get_topic_id("rc_ch_rx");
    rc_ch_ry_topic_id = global_databoard->get_topic_id("rc_ch_ry");

    motor_torque_topic_id = global_databoard->get_topic_id("motor_torque");
    motor_rotate_topic_id = global_databoard->get_topic_id("motor_rotate");

    //设定各轮电机id及pid以及正反（修改 CAN 枚举）
    wheel_motor_fl = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1, pyro::bsp_can::can1);
    rot_pid_fl = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,10.0f);
    wheel_control_fl = new pyro::wheel_control_t(wheel_motor_fl,rot_pid_fl);
    wheel_control_fl->set_forward();

    wheel_motor_fr = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2, pyro::bsp_can::can1);
    rot_pid_fr = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,10.0f);
    wheel_control_fr = new pyro::wheel_control_t(wheel_motor_fr,rot_pid_fr);
    wheel_control_fr->set_reverse();

    wheel_motor_br = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3, pyro::bsp_can::can1);
    rot_pid_br = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,10.0f);
    wheel_control_br = new pyro::wheel_control_t(wheel_motor_br,rot_pid_br);
    wheel_control_br->set_reverse();

    wheel_motor_bl = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4, pyro::bsp_can::can1);
    rot_pid_bl = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,10.0f);
    wheel_control_bl = new pyro::wheel_control_t(wheel_motor_bl,rot_pid_bl);
    wheel_control_bl->set_forward();

    // 设置功率控制器的系数
    pyro::power_control_drv_t::motor_coefficient_t motor_coefficient_fl;
    motor_coefficient_fl.k1 = 0.0115f;
    motor_coefficient_fl.k2 = 0.0391f;
    motor_coefficient_fl.k3 = 0.2739f;
    motor_coefficient_fl.k4 = -3.2137f;
    power_controller.set_motor_coefficient(1,motor_coefficient_fl);

    pyro::power_control_drv_t::motor_coefficient_t motor_coefficient_fr;
    motor_coefficient_fr.k1 = 0.0114f;
    motor_coefficient_fr.k2 = 0.0214f;
    motor_coefficient_fr.k3 = 0.2181f;
    motor_coefficient_fr.k4 = 0.4862f;
    power_controller.set_motor_coefficient(2,motor_coefficient_fr);

    pyro::power_control_drv_t::motor_coefficient_t motor_coefficient_bl;
    motor_coefficient_bl.k1 = 0.0111f;
    motor_coefficient_bl.k2 = 0.0430f;
    motor_coefficient_bl.k3 = 0.4013f;
    motor_coefficient_bl.k4 = -11.0010f;
    power_controller.set_motor_coefficient(4,motor_coefficient_bl);

    pyro::power_control_drv_t::motor_coefficient_t motor_coefficient_br;
    motor_coefficient_br.k1 = 0.0120f;
    motor_coefficient_br.k2 = 0.0353f;
    motor_coefficient_br.k3 = 0.3015f;
    motor_coefficient_br.k4 = -4.8325f;
    power_controller.set_motor_coefficient(3,motor_coefficient_br);

    
    joint_group = new joint_group_t();
    for(;;)
    {
        chassis_set_control();
        //小丑代码
        wheel_motor_fl->update_feedback();
        wheel_motor_fr->update_feedback();
        wheel_motor_bl->update_feedback();
        wheel_motor_br->update_feedback();

        wheel_motor_current_position[0] = wheel_motor_fl->get_current_position();
        wheel_motor_current_position[1] = wheel_motor_fr->get_current_position();
        wheel_motor_current_position[2] = wheel_motor_br->get_current_position();
        wheel_motor_current_position[3] = wheel_motor_bl->get_current_position();

        wheel_motor_current_rotate[0] = wheel_motor_fl->get_current_rotate();
        wheel_motor_current_rotate[1] = wheel_motor_fr->get_current_rotate();
        wheel_motor_current_rotate[2] = wheel_motor_br->get_current_rotate();
        wheel_motor_current_rotate[3] = wheel_motor_bl->get_current_rotate();

        wheel_motor_current_torque[0] = wheel_motor_fl->get_current_torque();
        wheel_motor_current_torque[1] = wheel_motor_fr->get_current_torque();
        wheel_motor_current_torque[2] = wheel_motor_br->get_current_torque();
        wheel_motor_current_torque[3] = wheel_motor_bl->get_current_torque();

        // global_databoard->write_topic(motor_torque_topic_id,*((pyro::genenral_data_t*)&(wheel_motor_current_torque[3])));
        // global_databoard->write_topic(motor_rotate_topic_id,*((pyro::genenral_data_t*)&(wheel_motor_current_rotate[3])));

        joint_group->update();
        if(chassis_mode == ZERO_FORCE)
        {
            chassis_zero_force();
        }
        else if(chassis_mode == RC_CONTROL)
        {
            chassis_rc_control();
        }
        if(chassis_mode == ZERO_FORCE)
            joint_group->zero_force();
        else if(chassis_mode == RC_CONTROL)
            joint_group->control();
        vTaskDelay(1);
    }
}