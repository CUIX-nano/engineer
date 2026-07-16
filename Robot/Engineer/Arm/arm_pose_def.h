#ifndef __ARM_POSE_DEF_H__
#define __ARM_POSE_DEF_H__

#include <stdint.h>

//姿态通过6个浮点数进行定义

extern float arm_reset_pose[6];
extern float arm_normal_pose[6];
extern float arm_cross_pose[6];

//取矿定义
//以最上方矿的中心轴线为起点，以补给站六边形的中心为圆心，顺时针旋转所得到的角度来对动作进行命名
typedef enum
{
    none_motion = -1,
    arm_grip_energy_unit_0 = 0,
    arm_grip_energy_unit_60,
    arm_grip_energy_unit_120,
    arm_grip_energy_unit_180,
    arm_grip_energy_unit_240,
    arm_grip_energy_unit_300,
    //将手中的矿放进矿仓顶部的动作
    arm_push_energy_unit,
    //将矿仓顶部的矿取到手中的动作
    arm_pop_energy_unit,


    arm_motion_max
}
arm_motion_e;

//动作通过二维数组来定义
//每新添加一个动作，就需要将其二维数组以及动作阶段数声明为全局变量
//具体定义见.cpp文件
extern float arm_grip_energy_unit_0_motion[][8];
extern uint32_t arm_grip_energy_unit_0_motion_stage_num;
extern float arm_grip_energy_unit_60_motion[][8];
extern uint32_t arm_grip_energy_unit_60_motion_stage_num;
extern float arm_grip_energy_unit_120_motion[][8];
extern uint32_t arm_grip_energy_unit_120_motion_stage_num;
extern float arm_grip_energy_unit_180_motion[][8];
extern uint32_t arm_grip_energy_unit_180_motion_stage_num;
extern float arm_grip_energy_unit_240_motion[][8];
extern uint32_t arm_grip_energy_unit_240_motion_stage_num;
extern float arm_grip_energy_unit_300_motion[][8];
extern uint32_t arm_grip_energy_unit_300_motion_stage_num;
extern float arm_push_energy_unit_motion[][8];
extern uint32_t arm_push_energy_unit_motion_stage_num;
extern float arm_pop_energy_unit_motion[][8];
extern uint32_t arm_pop_energy_unit_motion_stage_num;

#endif