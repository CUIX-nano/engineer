#ifndef __ARM_RC_COMMAND_H__
#define __ARM_RC_COMMAND_H__

#include "pyro_rc_hub.h"
#include "arm_transition_component.h"
#include "engineer_arm_planning_mission.h"
#include "arm_pose_def.h"

// 该模块用于读取遥控器数据并进行对应的状态切换
class arm_rc_command_t
{
    public:
        arm_rc_command_t();
        ~arm_rc_command_t();
        void bind_dr16(pyro::rc_drv_t* dr16_drv);
        void bind_vt03(pyro::rc_drv_t* vt03_drv);
        void dr16_update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state,
            user_command_t& user_command);
        void vt03_update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state,
            user_command_t& user_command);
        void update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state,
            user_command_t& user_command);
    private:
        pyro::rc_drv_t* dr16_drv;
        pyro::rc_drv_t* vt03_drv;
        // 删除以下两行（旧版 RC 数据类型已不再使用）
        // const pyro::dr16_drv_t::dr16_ctrl_t *_dr16_rc_data;
        // const pyro::vt03_drv_t::vt03_ctrl_t *_vt03_rc_data;
};

#endif