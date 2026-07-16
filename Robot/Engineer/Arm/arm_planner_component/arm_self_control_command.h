#ifndef __ARM_SELF_CONTROL_COMMAND_H__
#define __ARM_SELF_CONTROL_COMMAND_H__

#include <string.h>
#include "pyro_databoard.h"

#include "engineer_arm_planning_mission.h"

// 该模块的目的为从图传链路获取自控数据
class arm_self_control_command
{
    public:
        arm_self_control_command();
        ~arm_self_control_command();
        void bind(pyro::databoard* databoard);
        void update(user_command_t& user_command);
        void get_self_control_command(float xdata[6]);
        float* get_self_control_command();
    private:
        pyro::databoard *_databoard;
        uint32_t _selfcontrol_axis1_id,_selfcontrol_axis2_id,_selfcontrol_axis3_id,_selfcontrol_axis4_id,_selfcontrol_axis5_id,_selfcontrol_axis6_id;
        float _self_control_command[6];
};

#endif