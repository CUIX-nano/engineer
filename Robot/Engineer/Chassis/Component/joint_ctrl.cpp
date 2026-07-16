#include "joint_ctrl.h"

joint_control_t::joint_control_t(pyro::motor_base_t* motor, pyro::pid_t* pos_pid, pyro::pid_t* rot_pid):_motor(motor),_pos_pid(pos_pid),_rot_pid(rot_pid)
{
    _target_pos = 0.0f;
    _target_rot = 0.0f;
    _pos_offset = 0.0f;
    _direction = 1.0f;
    _upper_limit=pyro::PI;
    _lower_limit=-pyro::PI;
    _cycle_counter = 0;
    _state = NORMAL;
    _block_moment = 0.00f;

    _calibrate_target_rot = -3.0f;
    _calibrate_threshold_rot = 0.5f;
    _max_calibrate_time = 3.0f;
}

void joint_control_t::set_target(float& target) 
{
    _target_pos = target;
        if(_target_pos>_upper_limit)
        {
            _target_pos=_upper_limit;
        }
        else if(_target_pos<_lower_limit)
        {
            _target_pos=_lower_limit;
        }
    target = _target_pos;
}


void joint_control_t::increment(float increment)
{
    _target_pos = _target_pos+increment;
    if(_target_pos>_upper_limit)
    {
        _target_pos=_upper_limit;
    }
    else if(_target_pos<_lower_limit)
    {
        _target_pos=_lower_limit;
    }
}

void joint_control_t::update() 
{
    float cur_pos,feedback_pos;
    _motor->update_feedback();
    cur_pos = _motor->get_current_position();
    if(cur_pos - _last_pos < -pyro::PI)
    {
        _cycle_counter++;
    }
    else if(cur_pos - _last_pos > pyro::PI)
    {
        _cycle_counter--;
    }
    _last_pos = cur_pos;

    feedback_pos = cur_pos - _pos_offset +2*pyro::PI * _cycle_counter;
    _feedback_pos = feedback_pos *_direction;
    _feedback_rot = _motor->get_current_rotate()*_direction;
}

void joint_control_t::zero_force()
{
    _target_pos = 0;
    _target_rot = 0;
    if(_motor->is_enable())
    {
        _motor->disable();
        return;
    }
    _motor->send_torque(0);
}
        
void joint_control_t::normal_control()
{   
    float control_value = 0.0f;

    if(!_motor->is_enable())
    {
        _motor->enable();
        return;
    }
    _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
    control_value = _rot_pid->calculate(_target_rot,_feedback_rot);
    _motor->send_torque(control_value*_direction);
}
void joint_control_t::calibrate_control()
{
    float control_value = 0.0f;
    if(!_motor->is_enable())
    {
        _motor->enable();
        return;
    }
    control_value = _rot_pid->calculate(_calibrate_target_rot,_feedback_rot);
    _motor->send_torque(control_value*_direction);

    if(fabs(_feedback_rot)<_calibrate_threshold_rot)
    {
        if(_is_block == false)
        {
            _is_block = true;
            _block_moment = pyro::dwt_drv_t::get_timeline_s();
        }
        else
        {
            if(pyro::dwt_drv_t::get_timeline_s()-_block_moment>_max_calibrate_time)
            {
                _state = NORMAL;
                this->set_pos_offset(this->get_motor_raw_pos());
            }
        }
    }
    else if(fabs(_feedback_rot)>_calibrate_threshold_rot)
    {
        _is_block = false;
    }
}