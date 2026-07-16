#include "arm_fixed_motion_hub.h"

arm_fixed_motion_t::arm_fixed_motion_t()
{
    _motion_total_stage = 0;
    _motion_current_stage = 0;
    _motion_transition_state = Not_transition;
}

arm_fixed_motion_t::~arm_fixed_motion_t(){}

void arm_fixed_motion_t::bind(float motion_slice[][8], uint32_t stage_num)
{
    _motion_slice = motion_slice;
    _motion_total_stage = stage_num;
}

void arm_fixed_motion_t::reset()
{
    _motion_current_stage = -1;//不在任意阶段
    _motion_transition_state = Not_transition;//动作未开始
}

void arm_fixed_motion_t::start_motion()
{
    _motion_current_stage = 0;
    _motion_transition_state = Transition_running;
    memcpy(_now_slice,_motion_slice[0],sizeof(float)*8);
}

void arm_fixed_motion_t::start_motion_reverse()
{
    _motion_current_stage = _motion_current_stage;
    if(_motion_current_stage==-1)
    {
        _motion_transition_state = Transition_end;
        return ;
    }
    _motion_transition_state = Transition_reverse_running;
    memcpy(_now_slice,_motion_slice[_motion_current_stage],sizeof(float)*8);
    // _now_slice[0] = _motion_slice[_motion_current_stage+1][0];
}

bool arm_fixed_motion_t::update_motion(float current_period)
{
    bool ret = false;//不需要切换阶段
    if(_motion_transition_state == Transition_running)
    {
        if(current_period>=_now_slice[0])//当前计时器时间大于当前切片的过渡时间
        {
            _motion_current_stage++;//阶段数加1
            if(_motion_current_stage>=_motion_total_stage)//当前阶段大于总阶段数
            {
                _motion_transition_state = Transition_end;//动作完成
            }
            else
            {
                ret = true;//动作未完成，进需要切换动作的下一个阶段
                memcpy(_now_slice,_motion_slice[_motion_current_stage],sizeof(float)*8);//拷贝切片
            }
        }
    }
    else if(_motion_transition_state == Transition_reverse_running)
    {
        if(current_period>=_now_slice[0])//当前计时器时间大于当前切片的过渡时间
        {
            _motion_current_stage--;//阶段数加1
            if(_motion_current_stage==-1)//当前阶段大于总阶段数
            {
                _motion_transition_state = Transition_end;//动作完成
            }
            else
            {
                ret = true;//动作未完成，进需要切换动作的下一个阶段
                memcpy(_now_slice,_motion_slice[_motion_current_stage],sizeof(float)*8);//拷贝切片
                // _now_slice[0] = _motion_slice[_motion_current_stage+1][0];
            }
        }
    }
    return ret;
}

bool arm_fixed_motion_t::current_step_over(float current_period)
{
    bool ret = false;//判断当前阶段是否完成
    if(_motion_transition_state == Transition_running || _motion_transition_state == Transition_reverse_running)
    {
        if(current_period>=_now_slice[0])
        {
            ret = true;//当前阶段完成
        }
    }
    return ret;
}

void arm_fixed_motion_t::get_motion_slice(float xdata[8])
{
    memcpy(xdata,_now_slice,sizeof(float)*8);
}

transition_state_t arm_fixed_motion_t::get_motion_transition_state()
{
    return _motion_transition_state;
}

arm_fixed_motion_group_t::~arm_fixed_motion_group_t(){}

void arm_fixed_motion_group_t::start_motion(float current_position[6])
{
    float slice[7];
    _now_motion ->start_motion();
    _now_motion ->get_motion_slice(slice);
    // _motion_transition.init(slice[0],current_position,slice+1);
    //开始动作，并获取切片，初始化动作过渡器(无用，现在实际上是用的同一个)
}

void arm_fixed_motion_group_t::start_motion_reverse(float current_position[6])
{
    float slice[7];
    _now_motion ->start_motion_reverse();
    _now_motion ->get_motion_slice(slice);
    // _motion_transition.init(slice[0],current_position,slice+1);
    //开始动作，并获取切片，初始化动作过渡器(无用，现在实际上是用的同一个)
}

bool arm_fixed_motion_group_t::update_motion(float current_period)
{
    return _now_motion ->update_motion(current_period);
    
}

bool arm_fixed_motion_group_t::current_step_over(float current_period)
{
    return _now_motion ->current_step_over(current_period);
}

void arm_fixed_motion_group_t::get_motion_slice(float xdata[8])
{
    _now_motion ->get_motion_slice(xdata);
}

arm_fixed_motion_group_t::arm_fixed_motion_group_t()
{ 
    _now_motion = nullptr;
    _motion_list_num = 0;
    _now_motion_id = none_motion;
}

void arm_fixed_motion_group_t::add_motion(arm_motion_e motion_id,float motion_slice[][8],uint32_t stage_num)
{
    if(_motion_list_num>=16)
    {
        return;
    }
    //可添加的动作数不超过16，超过16则无法添加
    _motion_list[_motion_list_num].bind(motion_slice,stage_num);
    _motion_list_name[_motion_list_num] = motion_id;
    _motion_list_num++;
    //添加动作
}

void arm_fixed_motion_group_t::select_motion(arm_motion_e motion)
{
    if(motion==none_motion)//不存在的动作
    {
        _now_motion = nullptr;
        _now_motion_id = none_motion;
        return;
    }
    for(uint32_t i=0;i<_motion_list_num;i++)
    {
        if(_motion_list_name[i]==motion)
        {
            _now_motion = &_motion_list[i];
            _now_motion_id = motion;
            return;
        }
    }
    //遍历列表选择动作
}

bool arm_fixed_motion_group_t::motion_over()
{
    return _now_motion->get_motion_transition_state()==Transition_end;
}