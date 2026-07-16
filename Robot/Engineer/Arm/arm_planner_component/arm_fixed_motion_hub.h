#ifndef __ARM_FIXED_MOTION_HUB_H__
#define __ARM_FIXED_MOTION_HUB_H__

#include <stdint.h>
#include "arm_transition_component.h"
#include <string.h>
#include "arm_pose_def.h"

//单个动作类
class arm_fixed_motion_t
{
    public:
        arm_fixed_motion_t();
        ~arm_fixed_motion_t();
        //绑定动作切片和动作总阶段数
        void bind(float motion_slice[][8],uint32_t stage_num);
        //重置动作当前阶段
        void reset();
        //开始动作
        void start_motion();
        void start_motion_reverse();
        //传入一个时间长度，更新动作并判断当前阶段是否完成
        bool update_motion(float current_period);
        //传入一个时间长度，不更新动作并判断当前阶段是否完成
        bool current_step_over(float current_period);
        //获取当前动作切片
        void get_motion_slice(float dst[8]);
        //获取动作当前阶段状态
        transition_state_t get_motion_transition_state();
    private:
        //动作所对应的指针
        float (*_motion_slice)[8];
        //动作总阶段数
        uint32_t _motion_total_stage;
        //动作当前阶段
        uint32_t _motion_current_stage;
        //动作当前状态
        transition_state_t _motion_transition_state;
        //动作当前切片
        float _now_slice[8];
};

//动作组类
class arm_fixed_motion_group_t
{
    public:
        arm_fixed_motion_group_t();
        ~arm_fixed_motion_group_t();
        //根据传入的动作枚举选择动作
        void select_motion(arm_motion_e motion);
        //传入当前位置，开始动作
        void start_motion(float current_position[6]);
        void start_motion_reverse(float current_position[6]);
        //传入一个时间长度，更新动作并判断当前所选择的动作的当前阶段是否完成
        bool update_motion(float current_period);
        //传入一个时间长度，不更新动作并判断当前所选择的动作的当前阶段是否完成
        bool current_step_over(float current_period);
        //获取当前动作切片
        void get_motion_slice(float xdata[8]);
        //添加动作，添加动作时主要调用这个函数
        void add_motion(arm_motion_e motion_id,float motion_slice[][8],uint32_t stage_num);
        //判断当前所选择的动作是否完成
        bool motion_over();
    private:
        //当前所选择的动作的枚举
        arm_motion_e _now_motion_id;
        //当前所选择的动作
        arm_fixed_motion_t * _now_motion;
        //动作列表
        arm_fixed_motion_t _motion_list[16];
        //动作列表所对应的枚举
        arm_motion_e _motion_list_name[16];
        //动作列表长度
        uint32_t _motion_list_num;
        //动作所对应的过渡类
        motion_transition_t _motion_transition;
};

#endif