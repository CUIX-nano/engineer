#include "arm_rc_command.h"
#include "pyro_rc_core.h"   // 提供 sw_pos_t

arm_rc_command_t::arm_rc_command_t(){}
arm_rc_command_t::~arm_rc_command_t(){}

void arm_rc_command_t::bind_dr16(pyro::rc_drv_t* dr16_drv)
{
    this->dr16_drv = dr16_drv;
}

void arm_rc_command_t::bind_vt03(pyro::rc_drv_t* vt03_drv)
{
    this->vt03_drv = vt03_drv;
}

void arm_rc_command_t::dr16_update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state, user_command_t& user_command)
{
    // 静态变量记录上一次状态
    static pyro::sw_pos_t last_sw_r_state = pyro::sw_pos_t::UP;
    static pyro::sw_pos_t last_sw_l_state = pyro::sw_pos_t::UP;
    static uint8_t last_z_state = 0;
    static uint8_t last_x_state = 0;
    static uint8_t last_c_state = 0;
    static uint8_t last_v_state = 0;
    static uint8_t last_g_state = 0;
    static uint8_t last_r_state = 0;
    static uint8_t last_b_state = 0;
    static uint8_t last_f_state = 0;

    if(!dr16_drv->check_online())
        return;

    // 读取遥控数据（virtual_rc_t&）
    const auto &rc = dr16_drv->read();

    // 检查拨杆状态是否有效（UP/MID/DOWN）
    if(rc.switches.right.current_pos != pyro::sw_pos_t::UP &&
       rc.switches.right.current_pos != pyro::sw_pos_t::MID &&
       rc.switches.right.current_pos != pyro::sw_pos_t::DOWN)
        return;

    // 右拨杆从其他位置拨到上
    if(rc.switches.right.current_pos == pyro::sw_pos_t::UP && last_sw_r_state != pyro::sw_pos_t::UP)
    {
        specific_control_mode = RESET_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    // 右拨杆从其他位置拨到中
    else if(rc.switches.right.current_pos == pyro::sw_pos_t::MID && last_sw_r_state != pyro::sw_pos_t::MID)
    {
        specific_control_mode = NORMAL_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    // 右拨杆从其他位置拨到下
    else if(rc.switches.right.current_pos == pyro::sw_pos_t::DOWN && last_sw_r_state != pyro::sw_pos_t::DOWN)
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_300;
    }

    // 左拨杆从其他位置拨到中（当当前状态为正常姿态）
    if(rc.switches.left.current_pos == pyro::sw_pos_t::MID && last_sw_l_state != pyro::sw_pos_t::MID &&
       (specific_control_mode == NORMAL_POSE_TRANSITION || specific_control_mode == NORMAL_POSE))
    {
        specific_control_mode = GRAVITY_COMPENSATION;
    }
    // 左拨杆从其他位置拨到中（当当前状态为进行动作状态）
    else if(rc.switches.left.current_pos == pyro::sw_pos_t::MID && last_sw_l_state != pyro::sw_pos_t::MID &&
            specific_control_mode == MOTION)
    {
        specific_control_mode = MOTION_PAUSE;
    }
    // 左拨杆从其他位置拨到下（当当前状态为动作暂停）
    else if(rc.switches.left.current_pos == pyro::sw_pos_t::DOWN && last_sw_l_state != pyro::sw_pos_t::DOWN &&
            specific_control_mode == MOTION_PAUSE)
    {
        specific_control_mode = MOTION_CONTINUE;
    }
    // 左拨杆从其他位置拨到上（当当前状态为动作暂停）
    else if(rc.switches.left.current_pos == pyro::sw_pos_t::UP && last_sw_l_state != pyro::sw_pos_t::UP &&
            specific_control_mode == MOTION_PAUSE)
    {
        specific_control_mode = GRAVITY_COMPENSATION;
    }

    // 按键 z
    if(rc.keys.z.state && rc.keys.z.state != last_z_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_0;
    }

    // 按键 x
    if(rc.keys.x.state && rc.keys.x.state != last_x_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_60;
    }

    // 按键 c
    if(rc.keys.c.state && rc.keys.c.state != last_c_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_120;
    }

    // 按键 v
    if(rc.keys.v.state && rc.keys.v.state != last_v_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_180;
    }

    // 按键 g（夹爪切换）
    if(rc.keys.g.state && rc.keys.g.state != last_g_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        user_command.hold_gripper = !user_command.hold_gripper;
    }

    // 按键 r（矿仓角度递增）
    if(rc.keys.r.state && rc.keys.r.state != last_r_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        user_command.magazine_target_pos += pyro::PI/2;
        if(user_command.magazine_target_pos > pyro::PI)
            user_command.magazine_target_pos -= 2*pyro::PI;
        else if(user_command.magazine_target_pos < -pyro::PI)
            user_command.magazine_target_pos += 2*pyro::PI;
    }

    // 按键 b（推矿动作）
    if(rc.keys.b.state && rc.keys.b.state != last_b_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_push_energy_unit;
    }

    // 按键 f（拔矿动作）
    if(rc.keys.f.state && rc.keys.f.state != last_f_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_pop_energy_unit;
    }

    // 滚轮（用于夹爪微调）
    user_command.gripper_increment = rc.axes.wheel * 0.005f;

    // 记录上一次状态
    last_sw_r_state = rc.switches.right.current_pos;
    last_sw_l_state = rc.switches.left.current_pos;
    last_z_state = rc.keys.z.state;
    last_x_state = rc.keys.x.state;
    last_c_state = rc.keys.c.state;
    last_v_state = rc.keys.v.state;
    last_g_state = rc.keys.g.state;
    last_r_state = rc.keys.r.state;
    last_b_state = rc.keys.b.state;
    last_f_state = rc.keys.f.state;
}

void arm_rc_command_t::vt03_update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state, user_command_t& user_command)
{
    static pyro::sw_pos_t last_gear_state = pyro::sw_pos_t::UP;
    static uint8_t last_z_state = 0;
    static uint8_t last_x_state = 0;
    static uint8_t last_c_state = 0;
    static uint8_t last_v_state = 0;
    static uint8_t last_g_state = 0;
    static uint8_t last_r_state = 0;
    static uint8_t last_b_state = 0;
    static uint8_t last_f_state = 0;
    static uint8_t last_fn_l_state = 0;
    static uint8_t last_fn_r_state = 0;
    static uint8_t last_reflect_state = 0;
    static uint8_t last_trigger_state = 0;
    static uint8_t last_pause_state = 0;

    if(!vt03_drv->check_online())
        return;

    const auto &rc = vt03_drv->read();

    // 检查挡位是否有效
    if(rc.switches.gear.current_pos != pyro::sw_pos_t::UP &&
       rc.switches.gear.current_pos != pyro::sw_pos_t::MID &&
       rc.switches.gear.current_pos != pyro::sw_pos_t::DOWN)
        return;

    // 挡位从左到中
    if(rc.switches.gear.current_pos == pyro::sw_pos_t::UP && last_gear_state != pyro::sw_pos_t::UP)
    {
        specific_control_mode = RESET_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    // 挡位从中到右
    else if(rc.switches.gear.current_pos == pyro::sw_pos_t::MID && last_gear_state != pyro::sw_pos_t::MID)
    {
        specific_control_mode = NORMAL_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    // 挡位从右到中
    else if(rc.switches.gear.current_pos == pyro::sw_pos_t::DOWN && last_gear_state != pyro::sw_pos_t::DOWN)
    {
        specific_control_mode = SELF_CONTROL_TRANSITION;
        transition_state = Transition_start;
    }

    // 按键 z（切换取矿档位）
    if(rc.keys.z.state && rc.keys.z.state != last_z_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        switch(user_command.get_mine_motion)
        {
            case arm_motion_e::arm_grip_energy_unit_0:
                user_command.get_mine_motion = arm_motion_e::arm_grip_energy_unit_60;
                break;
            case arm_motion_e::arm_grip_energy_unit_60:
                user_command.get_mine_motion = arm_motion_e::arm_grip_energy_unit_120;
                break;
            case arm_motion_e::arm_grip_energy_unit_120:
                user_command.get_mine_motion = arm_motion_e::arm_grip_energy_unit_180;
                break;
            case arm_motion_e::arm_grip_energy_unit_180:
                user_command.get_mine_motion = arm_motion_e::arm_grip_energy_unit_240;
                break;
            case arm_motion_e::arm_grip_energy_unit_240:
                user_command.get_mine_motion = arm_motion_e::arm_grip_energy_unit_300;
                break;
            case arm_motion_e::arm_grip_energy_unit_300:
            default:
                user_command.get_mine_motion = arm_motion_e::arm_grip_energy_unit_0;
                break;
        }
    }

    // 按键 x（执行当前选中的取矿动作）
    if(rc.keys.x.state && rc.keys.x.state != last_x_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = user_command.get_mine_motion;
    }

    // 按键 c（反向执行动作）
    if(rc.keys.c.state && rc.keys.c.state != last_c_state &&
       specific_control_mode == MOTION)
    {
        specific_control_mode = MOTION_REVERSE_Start;
    }

    // 按键 v（穿越姿态切换）
    if(((rc.keys.v.state && rc.keys.v.state != last_v_state) || 
        (rc.buttons.fn_l.state && rc.buttons.fn_l.state != last_fn_l_state)) &&
        specific_control_mode == NORMAL_POSE)
    {
        specific_control_mode = CROSS_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    else if(((rc.keys.v.state && rc.keys.v.state != last_v_state) || 
             (rc.buttons.fn_l.state && rc.buttons.fn_l.state != last_fn_l_state)) &&
            specific_control_mode == CROSS_POSE && !user_command.overpass_pose)
    {
        specific_control_mode = NORMAL_POSE_TRANSITION;
        transition_state = Transition_start;
    }

    // 按键 v + shift 或 fn_r（切换穿越姿态）
    if(((rc.keys.v.state && rc.keys.v.state != last_v_state && rc.keys.shift.state) ||
        (rc.buttons.fn_r.state && rc.buttons.fn_r.state != last_fn_r_state)) &&
        specific_control_mode == CROSS_POSE)
    {
        user_command.overpass_pose = !user_command.overpass_pose;
    }
    else if(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION)
    {
        user_command.overpass_pose = 0;
    }

    // 按键 g 或 fn_r 切换夹爪
    if((rc.keys.g.state && rc.keys.g.state != last_g_state) ||
       (rc.buttons.fn_r.state && rc.buttons.fn_r.state != last_reflect_state))
    {
        if(!(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
        {
            user_command.hold_gripper = !user_command.hold_gripper;
        }
    }

    // 按键 r（矿仓角度递增）
    if(rc.keys.r.state && rc.keys.r.state != last_r_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        user_command.magazine_target_pos += pyro::PI/2;
        if(user_command.magazine_target_pos > pyro::PI)
            user_command.magazine_target_pos -= 2*pyro::PI;
        else if(user_command.magazine_target_pos < -pyro::PI)
            user_command.magazine_target_pos += 2*pyro::PI;
    }

    // 按键 b（推矿动作）
    if(rc.keys.b.state && rc.keys.b.state != last_b_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_push_energy_unit;
    }

    // 按键 f（拔矿动作）
    if(rc.keys.f.state && rc.keys.f.state != last_f_state &&
       !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_pop_energy_unit;
    }

    // 按键 pause（动作暂停/继续）
    if(rc.buttons.pause.state && rc.buttons.pause.state != last_pause_state)
    {
        if(specific_control_mode == MOTION)
            specific_control_mode = MOTION_PAUSE;
        else if(specific_control_mode == MOTION_PAUSE)
            specific_control_mode = MOTION_CONTINUE;
    }

    // 记录上一次状态
    last_gear_state = rc.switches.gear.current_pos;
    last_z_state = rc.keys.z.state;
    last_x_state = rc.keys.x.state;
    last_c_state = rc.keys.c.state;
    last_v_state = rc.keys.v.state;
    last_g_state = rc.keys.g.state;
    last_r_state = rc.keys.r.state;
    last_b_state = rc.keys.b.state;
    last_f_state = rc.keys.f.state;
    last_fn_l_state = rc.buttons.fn_l.state;
    last_fn_r_state = rc.buttons.fn_r.state;
    last_reflect_state = rc.buttons.fn_r.state;
    last_trigger_state = rc.buttons.trigger.state;
    last_pause_state = rc.buttons.pause.state;
}

void arm_rc_command_t::update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state, user_command_t& user_command)
{
    if(dr16_drv)
        dr16_update(specific_control_mode, transition_state, user_command);
    if(vt03_drv)
        vt03_update(specific_control_mode, transition_state, user_command);
}