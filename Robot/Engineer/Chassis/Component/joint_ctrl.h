#ifndef JOINT_CTRL_H
#define JOINT_CTRL_H

#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include "pyro_dwt_drv.h"

class joint_control_t
{
       enum joint_state_t
       {
        NORMAL,
        CALIBRATING
       };

    protected:
        // motor and pid object attached
        pyro::motor_base_t* _motor;
        pyro::pid_t* _pos_pid;
        pyro::pid_t* _rot_pid;

        // target and feedback
        float _target_pos;
        float _target_rot;

        float _feedback_pos;
        float _feedback_rot;

        float _control_value;

        // upper and lower limit
        float _upper_limit;
        float _lower_limit;

        // the offset of the position and the counter of the cycle
        float _pos_offset;
        int32_t _cycle_counter;
        float _last_pos;

        // the direction of the joint
        float _direction;

        // calibartion variable
        joint_state_t _state;
        bool _is_block;
        float _block_moment;

        float _calibrate_target_rot;
        float _max_calibrate_time;
        float _calibrate_threshold_rot;
    public:
        joint_control_t(pyro::motor_base_t* motor, pyro::pid_t* pos_pid, pyro::pid_t* rot_pid);
        ~joint_control_t(){}
        void set_target(float& target);
        void increment(float increment);
        void update();
        
        float get_position(){return _feedback_pos;}
        void set_upper_limit(float upper_limit){ _upper_limit=upper_limit;}
        void set_lower_limit(float lower_limit){_lower_limit=lower_limit;}

        void set_pos_offset(float offset){_pos_offset = offset;
            _cycle_counter = 0;}
        void set_direction(float direction){_direction = direction<0?-1:1;}
        float get_motor_raw_pos(){return _motor->get_current_position();}
        void set_calibrate_parameters(float target_rot,float max_calibrate_time,float calibrate_threshold_rot)
        {
            _calibrate_target_rot = target_rot;
            _max_calibrate_time = max_calibrate_time;
            _calibrate_threshold_rot = calibrate_threshold_rot;
        }
        bool is_calibrate(){return _state == CALIBRATING;}
        void start_calibrate(){_state = CALIBRATING;_is_block = false;}
        void stop_calibrate(){_state = NORMAL;}
        void zero_force();
        void normal_control();
        void calibrate_control();
};

#endif