#ifndef __ENGINEER_ARM_PLANNING_MISSION_H__
#define __ENGINEER_ARM_PLANNING_MISSION_H__

#include "semphr.h"
#include "arm_pose_def.h"


typedef enum
{
    ZERO_FORCE,
    POSITION_CONTROL,
    TORQUE_COMPENSATION,
}
control_mode_t;

typedef enum
{
    Biopolar,
    Analog
}
gripper_mode_e;

typedef struct{
    arm_motion_e selected_motion = none_motion;
    gripper_mode_e gripper_mode = Biopolar;
    bool hold_gripper;
    float gripper_increment;
    float gripper_target_pos;
    float magazine_target_pos;
    arm_motion_e get_mine_motion = arm_grip_energy_unit_0;
    uint8_t overpass_pose = 0;
}
user_command_t;

typedef struct 
{
    control_mode_t control_mode;
    float axis_current_pos[6];
    bool arm_is_ready;
    float axis_target_pos[6];
    gripper_mode_e gripper_mode;
    bool hold_gripper;
    float gripper_pos;
}
control_target_param_t;

typedef enum
{
    RESET_POSE_TRANSITION,
    RESET_POSE,
    NORMAL_POSE_TRANSITION,
    NORMAL_POSE,
    CROSS_POSE_TRANSITION,
    CROSS_POSE,
    SELF_CONTROL_TRANSITION,
    SELF_CONTROL,
    MOTION_Start,
    MOTION,
    MOTION_REVERSE_Start,
    MOTION_REVERSE,
    MOTION_PAUSE,
    MOTION_CONTINUE,
    GRAVITY_COMPENSATION,
    
}
specific_control_mode_t;

extern control_target_param_t *control_target_param;
extern SemaphoreHandle_t rc_planning_sem;

#endif