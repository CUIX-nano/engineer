#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_bsp_can.h"          // 新增：BSP CAN 头文件
#include "pyro_algo_pid.h"

class magazine_control_t 
{ 
        enum constraint_t
    {
        NO_CONSTRAINT,
        CONSTRAINT
    };
    protected:
        pyro::motor_base_t* _motor;
        pyro::pid_t* _pos_pid;
        pyro::pid_t* _rot_pid;

        constraint_t _constraint=NO_CONSTRAINT;

        float _target_pos;
        float _target_rot;

        float _feedback_pos;
        float _feedback_pos_offset;
        float _feedback_rot;

        float _control_value;

        float _upper_limit;
        float _lower_limit;

    public:
        magazine_control_t(pyro::motor_base_t* motor, pyro::pid_t* pos_pid, pyro::pid_t* rot_pid):_motor(motor),_pos_pid(pos_pid),_rot_pid(rot_pid)
        {
            _target_pos = 0.0f;
            _target_rot = 0.0f;
        }
        ~magazine_control_t(){}
        void set_target(float& target) 
        {
            _target_pos = target;
            if(_constraint==CONSTRAINT)
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_upper_limit;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_lower_limit;
                }
            }
            else
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_target_pos-2*pyro::PI;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_target_pos+2*pyro::PI;
                }
            }
            target = _target_pos;
        }
        void increment(float increment)
        {
            _target_pos = _target_pos+increment;
            if(_constraint==CONSTRAINT)
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_upper_limit;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_lower_limit;
                }
            }
            else
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_target_pos-2*pyro::PI;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_target_pos+2*pyro::PI;
                }
            }
        }
        void update() 
        {
            _motor->update_feedback();
            _feedback_pos = _motor->get_current_position()-_feedback_pos_offset;
            _feedback_rot = _motor->get_current_rotate();
            if(_feedback_pos>pyro::PI)
            {
                _feedback_pos-=2*pyro::PI;
            }
            else if(_feedback_pos<-pyro::PI)
            {
                _feedback_pos+=2*pyro::PI;
            }
        }
        inline float rot_angle_correction_pi(float feedback,float target,float max,float min)
        {
            if(feedback-target>pyro::PI)
		        return target-min+max;
	        else if(target-feedback>pyro::PI)
		        return target-max+min;
	        else 
		        return target;
        }
        void control(float dt)
        {   
            float control_value = 0.0f;
            if(_constraint==CONSTRAINT)
            {
                _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            else
            {
                _target_rot = _pos_pid->calculate(rot_angle_correction_pi(_feedback_pos,_target_pos,pyro::PI,-pyro::PI),_feedback_pos);
                // _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            control_value = _rot_pid->calculate(_target_rot,_feedback_rot);
            _motor->send_torque(control_value);
        }
        float get_position(){return _feedback_pos;}
        void enable_constraint(){_constraint=CONSTRAINT;}
        void disable_constraint(){_constraint=NO_CONSTRAINT;
         _upper_limit=pyro::PI;_lower_limit=-pyro::PI;}
        void set_upper_limit(float upper_limit){ _upper_limit=upper_limit;}
        void set_lower_limit(float lower_limit){_lower_limit=lower_limit;}

        void set_feedback_pos_offset(float offset){_feedback_pos_offset=offset;}
    
};



extern pyro::databoard* global_databoard;

static uint32_t 
zero_force_topic_id,
magazine_angle_topic_id;

uint32_t magazine_sw_l,magazine_sw_r;
magazine_control_t* magazine_control;


bool magazine_rc_solve()
{
    uint32_t timestamp;
    pyro::topic::data_status_t rc_data_status;
    rc_data_status = global_databoard->read(zero_force_topic_id,(pyro::genenral_data_t*)&(magazine_sw_l),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(magazine_angle_topic_id,(pyro::genenral_data_t*)&(magazine_sw_l),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    return true;
}

pyro::dji_gm_6020_motor_drv_t * magazine_motor_drv;
pyro::dji_m2006_motor_drv_t * lift_motor_drv;

uint32_t zero_force = 1 ;
float magazine_angle = 0;
extern "C" void magazine_mission(void *argument)
{

    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }

    zero_force_topic_id = global_databoard->get_topic_id("zero_force");
    magazine_angle_topic_id = global_databoard->get_topic_id("magazine_angle");

    // 修改 CAN 枚举：pyro::can_hub_t::can3 → pyro::bsp_can::can3
    magazine_motor_drv = new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1, pyro::bsp_can::can3);
    pyro::pid_t *pos_pid = new pyro::pid_t(10.0f, 0.01f, 0.2f, 3.0f, 10.0f);
    pyro::pid_t *rot_pid = new pyro::pid_t(0.50f, 0.2f, 0.00f, 1.0f, 3.0f);
    magazine_control = new magazine_control_t(magazine_motor_drv,pos_pid,rot_pid);
    magazine_control->disable_constraint();
    magazine_control->set_feedback_pos_offset(-1.635);

    // 修改 CAN 枚举：pyro::can_hub_t::can3 → pyro::bsp_can::can3
    lift_motor_drv = new pyro::dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3, pyro::bsp_can::can3);

    for(;;)
    {
        uint32_t timestamp;
        float t=pyro::PI/4;
        global_databoard->read(zero_force_topic_id,(pyro::genenral_data_t*)&(zero_force),timestamp);
        global_databoard->read(magazine_angle_topic_id,(pyro::genenral_data_t*)&(magazine_angle),timestamp);
        // magazine_motor_drv->update_feedback();
        magazine_control->update();
        lift_motor_drv->update_feedback();
        magazine_control->set_target(magazine_angle);
        if(zero_force)
        {
            magazine_motor_drv->send_torque(0.0f);
            lift_motor_drv->send_torque(0.0f);
        }
        else
        {
            magazine_control->control(0.01f);
            lift_motor_drv->send_torque(0.6f);
        }

        vTaskDelay(1);
    }
}