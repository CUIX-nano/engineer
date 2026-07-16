#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "engineer_arm_planning_mission.h"
#include "pyro_rc_hub.h"
#include "timers.h"
#include <string.h>
#include "pyro_databoard.h"
#include "pyro_dwt_drv.h"

#include "arm_pose_def.h"
#include "arm_transition_component.h"
#include "arm_fixed_motion_hub.h"
#include "arm_rc_command.h"
#include "arm_self_control_command.h"

extern pyro::databoard* global_databoard;

control_target_param_t *control_target_param;
SemaphoreHandle_t rc_planning_sem;
control_target_param_t *control_target_param_buffer ;

specific_control_mode_t specific_control_mode = RESET_POSE;
transition_state_t transition_state = Not_transition;
arm_transition_t arm_transition_param;

float end_target_torque = 0.0f;

static pyro::rc_drv_t* dr16_drv;
// 移除旧的 dr16_ctrl_t 指针，新库使用 virtual_rc_t&
// static const pyro::dr16_drv_t::dr16_ctrl_t *rc_data;

class arm_planner_t
{
    public:
        void init();
        void update();
        void update_async();
        void transition_process();
        void fixed_motion_process();
        void planning_application();
        void application_async();
        arm_fixed_motion_group_t _arm_fixed_motion_group;
    private:
        float _axis_current_pos[6];
        float _axis_target_pos[6];

        specific_control_mode_t _specific_control_mode = RESET_POSE;
        transition_state_t _transition_state = Not_transition;
        arm_rc_command_t _arm_rc_command;
        arm_self_control_command _arm_self_control_command;
        motion_transition_t _motion_transition;
        

        user_command_t _user_command;

        uint32_t _axis_target_id[6];

        uint32_t _magazine_zero_force_id;
        uint32_t _magazine_angle_id;
};

void arm_planner_t::init()
{
    //绑定遥控器命令
    _arm_rc_command.bind_dr16(pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16));
    _arm_rc_command.bind_vt03(pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03));
    //等待数据面板初始化完成
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }
    _arm_self_control_command.bind(global_databoard);

    //临时添加的代码，用于调试
    _axis_target_id[0] = global_databoard->get_topic_id("axis1_target_pos");
    _axis_target_id[1] = global_databoard->get_topic_id("axis2_target_pos");
    _axis_target_id[2] = global_databoard->get_topic_id("axis3_target_pos");
    _axis_target_id[3] = global_databoard->get_topic_id("axis4_target_pos");
    _axis_target_id[4] = global_databoard->get_topic_id("axis5_target_pos");
    _axis_target_id[5] = global_databoard->get_topic_id("axis6_target_pos");

    _magazine_zero_force_id = global_databoard->get_topic_id("zero_force");
    _magazine_angle_id = global_databoard->get_topic_id("magazine_angle");
}

void arm_planner_t::update_async()
{
    //拷贝机械臂的当前位置
    xSemaphoreTake(rc_planning_sem, portMAX_DELAY);
    memcpy(control_target_param_buffer->axis_current_pos,control_target_param->axis_current_pos,sizeof(float)*6);
    control_target_param_buffer->arm_is_ready = control_target_param->arm_is_ready;
    xSemaphoreGive(rc_planning_sem);    
    memcpy(_axis_current_pos,control_target_param_buffer->axis_current_pos,sizeof(float)*6);
}
void arm_planner_t::update()
{
    update_async();
    //更新遥控器命令和自控命令
    _arm_rc_command.update(_specific_control_mode,_transition_state,_user_command);
    _arm_self_control_command.update(_user_command);
}

//处理转换
void arm_planner_t::transition_process()
{
    float time_spend = 3;

    //判断转换状态
    if(_transition_state == Transition_start && control_target_param_buffer->arm_is_ready)
    {
        //开始过渡
        //根据当前具体的控制状态确定过渡的结束位置
        switch(_specific_control_mode)
        {
            //向无力位置过渡
            case RESET_POSE_TRANSITION:
            _transition_state = Transition_running;
            time_spend = 2;
            _motion_transition.init(time_spend,_axis_current_pos, arm_reset_pose);
            break;
            //向通常位置过渡
            case NORMAL_POSE_TRANSITION:
            _transition_state = Transition_running;
            time_spend = 1;
            _motion_transition.init(time_spend,_axis_current_pos, arm_normal_pose);
            break;
            //向自控位置过渡
            case SELF_CONTROL_TRANSITION:
            _transition_state = Transition_running;
            time_spend = 2;
            _motion_transition.init(time_spend,_axis_current_pos, _arm_self_control_command.get_self_control_command());
            break;
            case CROSS_POSE_TRANSITION:
            _transition_state = Transition_running;
            time_spend = 2;
            _motion_transition.init(time_spend,_axis_current_pos, arm_cross_pose);
            break;
        }
    }
    else if(_transition_state == Transition_running)
    {
        //过渡中
        //更新插值
        _motion_transition.interpolation_update();
        if(_motion_transition.transition_timeout())//判断是否超时
        {
            //如果已超时，则结束过渡，进入下一状态
            _transition_state = Not_transition;
            switch(_specific_control_mode)
            {
                case RESET_POSE_TRANSITION:
                _specific_control_mode = RESET_POSE;//无力状态
                break;
                case NORMAL_POSE_TRANSITION:
                _specific_control_mode = NORMAL_POSE;//通常状态
                break;
                case CROSS_POSE_TRANSITION:
                _specific_control_mode = CROSS_POSE;//自控状态
                break;
                case SELF_CONTROL_TRANSITION:
                _specific_control_mode = SELF_CONTROL;//自控状态
                break;
            }
        }
    }
}

//处理固定动作
void arm_planner_t::fixed_motion_process()
{
    float slice[8];
    if(_specific_control_mode == MOTION_Start)
    {
        //动作开始

        //选择对应动作
        _arm_fixed_motion_group.select_motion(_user_command.selected_motion);
        //开始动作
        _arm_fixed_motion_group.start_motion(_axis_current_pos);
        //更新动作
        _arm_fixed_motion_group.update_motion(0);
        //获取动作切片
        _arm_fixed_motion_group.get_motion_slice(slice);
        //初始化向动作的下一阶段过渡的过渡参数
        _motion_transition.init(slice[0],_axis_current_pos,slice+1);
        //进入动作状态
        _specific_control_mode = MOTION;
    }
    else if(_specific_control_mode == MOTION)
    {
        //动作运行状态
        if(_arm_fixed_motion_group.motion_over())//判断动作是否完成
        {
            //如果完成则向通常位置进行过渡
            _specific_control_mode = NORMAL_POSE_TRANSITION;
            _transition_state = Transition_start;
        }
        if(_arm_fixed_motion_group.update_motion(_motion_transition.get_transition_current_period()))//判断当前动作的当前阶段是否完成
        {
            // 如果完成则获取动作下一阶段的切片，并初始化过渡参数
            _arm_fixed_motion_group.get_motion_slice(slice);
            _motion_transition.init(slice[0],_axis_target_pos,slice+1);
            if(slice[7] == 1.0f)
                _user_command.hold_gripper = true;
            else
                _user_command.hold_gripper = false;
        }
        else
        {
            //插值更新
            _motion_transition.interpolation_update();
        }
    }
    else if(_specific_control_mode == MOTION_REVERSE_Start)
    {
        //动作开始

        //选择对应动作
        // _arm_fixed_motion_group.select_motion(_user_command.selected_motion);
        //开始动作
        _arm_fixed_motion_group.start_motion_reverse(_axis_current_pos);
        //更新动作
        _arm_fixed_motion_group.update_motion(0);
        _motion_transition.init(slice[0],_axis_target_pos,slice+1);
        _specific_control_mode = MOTION_REVERSE;
        
    }
    else if(_specific_control_mode == MOTION_REVERSE)
    {
        //动作运行状态
        if(_arm_fixed_motion_group.motion_over())//判断动作是否完成
        {
            //如果完成则向通常位置进行过渡
            _specific_control_mode = NORMAL_POSE_TRANSITION;
            _transition_state = Transition_start;
        }
        if(_arm_fixed_motion_group.update_motion(_motion_transition.get_transition_current_period()))//判断当前动作的当前阶段是否完成
        {
            // 如果完成则获取动作下一阶段的切片，并初始化过渡参数
            _arm_fixed_motion_group.get_motion_slice(slice);
            _motion_transition.init(slice[0],_axis_target_pos,slice+1);
            if(slice[7] == 1.0f)
                _user_command.hold_gripper = true;
            else
                _user_command.hold_gripper = false;
        }
        else
        {
            //插值更新
            _motion_transition.interpolation_update();
        }
    }
    else if(_specific_control_mode == MOTION_PAUSE)
    {
        //动作暂停状态
        if(_arm_fixed_motion_group.current_step_over(_motion_transition.get_transition_current_period()))//不更新动作仅判断当前动作的当前阶段是否完成
        {
        }
        else
        {
            _motion_transition.interpolation_update();
        }
    }
    else if(_specific_control_mode == MOTION_CONTINUE)
    {
        //从暂停状态恢复
        _motion_transition.recover_from_pasuse();
        _specific_control_mode = MOTION;
    }
}

int get_mine_motion = 1;
uint8_t overpass_pose = 0;

//将机械臂的规划量应用到实际上的机械臂上
void arm_planner_t::planning_application()
{
    uint32_t zf = 1;//调试变量
    switch(_specific_control_mode)
    {
        case RESET_POSE://如果是无力状态
        // memcpy(control_target_param->axis_target_pos,arm_reset_pose,sizeof(float)*6);
        zf = 1;
        
        break;
        case NORMAL_POSE://如果是通常状态
        memcpy(_axis_target_pos,arm_normal_pose,sizeof(float)*6);
        zf = 0;
        break;
        case CROSS_POSE://如果是自控状态，实时跟随自控的位置
        memcpy(_axis_target_pos,arm_cross_pose,sizeof(float)*6);
        zf = 0;
        break;
        case SELF_CONTROL://如果是自控状态，实时跟随自控的位置
        memcpy(_axis_target_pos,_arm_self_control_command.get_self_control_command(),sizeof(float)*6);
        zf = 0;
        break;
        case RESET_POSE_TRANSITION:
        case NORMAL_POSE_TRANSITION:
        case SELF_CONTROL_TRANSITION:
        case CROSS_POSE_TRANSITION:
        case MOTION:
        case MOTION_REVERSE:
        case MOTION_PAUSE:
        //处于以上位置时，将插值应用到机械臂
        memcpy(_axis_target_pos,_motion_transition.get_transition_interpolation_value(),sizeof(float)*6);
        zf = 0;
        break;
        default:
        zf = 0;
        break;
    }

    switch(_user_command.get_mine_motion)
    {
        case arm_grip_energy_unit_0:
        get_mine_motion =1;
        break;
        case arm_grip_energy_unit_60:
        get_mine_motion =2;
        break;
        case arm_grip_energy_unit_120:
        get_mine_motion =3;
        break;
        case arm_grip_energy_unit_180:
        get_mine_motion =4;
        break;
        case arm_grip_energy_unit_240:
        get_mine_motion =5;
        break;
        case arm_grip_energy_unit_300:
        get_mine_motion =6;
        break;
        default:
        get_mine_motion=1;
    }
    overpass_pose = _user_command.overpass_pose;

    //调试用
    global_databoard->write_topic(_axis_target_id[0],*((pyro::genenral_data_t*)&_axis_target_pos[0]));
    global_databoard->write_topic(_axis_target_id[1],*((pyro::genenral_data_t*)&_axis_target_pos[1]));
    global_databoard->write_topic(_axis_target_id[2],*((pyro::genenral_data_t*)&_axis_target_pos[2]));
    global_databoard->write_topic(_axis_target_id[3],*((pyro::genenral_data_t*)&_axis_target_pos[3]));
    global_databoard->write_topic(_axis_target_id[4],*((pyro::genenral_data_t*)&_axis_target_pos[4]));    
    global_databoard->write_topic(_axis_target_id[5],*((pyro::genenral_data_t*)&_axis_target_pos[5]));

    global_databoard->write_topic(_magazine_zero_force_id,*((pyro::genenral_data_t*)&zf));
    global_databoard->write_topic(_magazine_angle_id,*((pyro::genenral_data_t*)&_user_command.magazine_target_pos));

   xSemaphoreTake(rc_planning_sem, portMAX_DELAY);
    application_async();
    xSemaphoreGive(rc_planning_sem);
}

void arm_planner_t::application_async()
{
    //设定控制参数
    switch(_specific_control_mode)
    {
        case RESET_POSE:
        control_target_param->control_mode = ZERO_FORCE;
        break;
        case NORMAL_POSE:
        control_target_param->control_mode = POSITION_CONTROL;
        memcpy(control_target_param->axis_target_pos,_axis_target_pos,sizeof(float)*6);
        control_target_param->hold_gripper = _user_command.hold_gripper;
        // control_target_param->gripper_increment = _user_command.gripper_increment;
        case CROSS_POSE:
        control_target_param->control_mode = POSITION_CONTROL;
        memcpy(control_target_param->axis_target_pos,_axis_target_pos,sizeof(float)*6);
        control_target_param->hold_gripper = _user_command.hold_gripper;
        break;
        case SELF_CONTROL:
        control_target_param->control_mode = POSITION_CONTROL;
        memcpy(control_target_param->axis_target_pos,_axis_target_pos,sizeof(float)*6);
        control_target_param->hold_gripper = _user_command.hold_gripper;
        // control_target_param->gripper_increment = _user_command.gripper_increment;
        break;
        case RESET_POSE_TRANSITION:
        case NORMAL_POSE_TRANSITION:
        case SELF_CONTROL_TRANSITION:
        case CROSS_POSE_TRANSITION:
        case MOTION:
        case MOTION_REVERSE:
        case MOTION_CONTINUE:
        case MOTION_PAUSE:
        control_target_param->control_mode = POSITION_CONTROL;
        memcpy(control_target_param->axis_target_pos,_axis_target_pos,sizeof(float)*6);
        control_target_param->hold_gripper = _user_command.hold_gripper;
        // control_target_param->gripper_increment = _user_command.gripper_increment;
        break;
        case GRAVITY_COMPENSATION:
        control_target_param->control_mode = TORQUE_COMPENSATION;
        break;
        default:
        control_target_param->control_mode = ZERO_FORCE;
    }
}

arm_planner_t arm_planner;
float now = 0;

extern "C" void engineer_arm_planning_mission(void* args)
{
    vTaskDelay(1500);
    control_target_param = new control_target_param_t;
    control_target_param->control_mode = ZERO_FORCE;
    // control_target_param->end_target_torque = 0;
    control_target_param->hold_gripper = false;
    for(int i=0;i<6;i++)
    {
        control_target_param->axis_target_pos[i]=0;
        control_target_param->axis_current_pos[i]=0;
    }
    control_target_param_buffer = new control_target_param_t;
    memcpy(control_target_param_buffer,control_target_param,sizeof(control_target_param_t));
    rc_planning_sem = xSemaphoreCreateMutex(); 
    
    arm_planner.init();//初始化
    //添加动作
    arm_planner._arm_fixed_motion_group.add_motion(arm_grip_energy_unit_0,arm_grip_energy_unit_0_motion,arm_grip_energy_unit_0_motion_stage_num);
    arm_planner._arm_fixed_motion_group.add_motion(arm_grip_energy_unit_60,arm_grip_energy_unit_60_motion,arm_grip_energy_unit_60_motion_stage_num);
    arm_planner._arm_fixed_motion_group.add_motion(arm_grip_energy_unit_120,arm_grip_energy_unit_120_motion,arm_grip_energy_unit_120_motion_stage_num);
    arm_planner._arm_fixed_motion_group.add_motion(arm_grip_energy_unit_180,arm_grip_energy_unit_180_motion,arm_grip_energy_unit_180_motion_stage_num);
    arm_planner._arm_fixed_motion_group.add_motion(arm_grip_energy_unit_240,arm_grip_energy_unit_240_motion,arm_grip_energy_unit_240_motion_stage_num);
    arm_planner._arm_fixed_motion_group.add_motion(arm_grip_energy_unit_300,arm_grip_energy_unit_300_motion,arm_grip_energy_unit_300_motion_stage_num);
    arm_planner._arm_fixed_motion_group.add_motion(arm_push_energy_unit,arm_push_energy_unit_motion,arm_push_energy_unit_motion_stage_num);
    arm_planner._arm_fixed_motion_group.add_motion(arm_pop_energy_unit,arm_pop_energy_unit_motion,arm_pop_energy_unit_motion_stage_num);

    for(;;)
    {
        //搞清楚调用关系
        now = pyro::dwt_drv_t::get_timeline_ms();
        arm_planner.update();
        arm_planner.fixed_motion_process();
        arm_planner.transition_process();
        arm_planner.planning_application();
        vTaskDelay(1);
    }
}