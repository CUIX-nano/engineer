#include "arm_transition_component.h"
#include "pyro_dwt_drv.h"

//初始化轴过渡参数
//三次多项式插值: sita(t)=a0+a1*t+a2*t^2+a3*t^3
//先验条件为:sita(0)=start sita(T)=end sita'(0)=0 sita'(T)=0
void axis_transition_init(transition_param_t* param,float transition_period,float start_angle,float end_angle)
{
    param->transition_start_pos = start_angle;
    param->transition_end_pos = end_angle;
    param->transition_current_pos = start_angle;
    param->transition_target_pos = start_angle;
    param->transition_coefficient[0] = start_angle;
    param->transition_coefficient[1] = 0.0f;
    param->transition_coefficient[2] = 3*(end_angle-start_angle)/transition_period/transition_period;
    param->transition_coefficient[3] = 2*(start_angle-end_angle)/transition_period/transition_period/transition_period;
}

//初始化六个旋转轴过渡参数，并记录时间戳
void arm_transition_init(arm_transition_t* param, float transition_period, float start_angle[6],float end_angle[6])
{
    param->transition_start_Tick = xTaskGetTickCount();
    param->transition_total_time = transition_period;
    param->transition_current_time = 0;
    for(int i=0;i<6;i++)
    {
        axis_transition_init(&param->axis_transition_param[i], transition_period, start_angle[i],end_angle[i]);
    }
}

void arm_transition_update(arm_transition_t* param)
{ 
    // param->transition_current_time = (xTaskGetTickCount()-param->transition_start_Tick)/1000.0f;
    param->transition_current_time = pyro::dwt_drv_t::get_timeline_ms()-param->transition_start_Tick;
    float _t = param->transition_current_time;
    float _t2 = _t*_t;
    float _t3 = _t2*_t;
    for(int i=0;i<6;i++)
    {
        param->axis_transition_param[i].transition_target_pos = param->axis_transition_param[i].transition_coefficient[0]+
        param->axis_transition_param[i].transition_coefficient[1]*_t+
        param->axis_transition_param[i].transition_coefficient[2]*_t2+
        param->axis_transition_param[i].transition_coefficient[3]*_t3;
    }
}

//构造函数
value_interpolation_t::value_interpolation_t()
{
    _interpolation_start_value = 0.0f;
    _interpolation_end_value = 0.0f;
    _interpolation_coefficient[0] = 0.0f;
    _interpolation_coefficient[1] = 0.0f;
    _interpolation_coefficient[2] = 0.0f;
    _interpolation_coefficient[3] = 0.0f;
    _interpolation_value = 0.0f;
}

value_interpolation_t::~value_interpolation_t(){}

//初始化插值参数
void value_interpolation_t::init(float transition_period,float start_value,float end_value)
{
    _interpolation_period = transition_period;
    _interpolation_start_value = start_value;
    _interpolation_end_value = end_value;
    _interpolation_value = start_value;
    _interpolation_coefficient[0] = start_value;
    _interpolation_coefficient[1] = 0.0f;
    _interpolation_coefficient[2] = 3*(end_value-start_value)/transition_period/transition_period;
    _interpolation_coefficient[3] = 2*(start_value-end_value)/transition_period/transition_period/transition_period;
}

void value_interpolation_t::interpolation_update(float xdata[3])
{
    _interpolation_value = _interpolation_coefficient[0]+
    _interpolation_coefficient[1]*xdata[0]+
    _interpolation_coefficient[2]*xdata[1]+
    _interpolation_coefficient[3]*xdata[2];
}

float value_interpolation_t::get_transition_interpolation_value() const
{
    return _interpolation_value;
}

motion_transition_t::motion_transition_t()
{
    _transition_start_Tick = 0;
    _transition_total_period = 0.0f;
    _transition_current_period = 0.0f;
}

motion_transition_t::~motion_transition_t(){}

void motion_transition_t::init(float transition_period, float start_angle[6],float end_angle[6])
{
    _transition_start_Tick = pyro::dwt_drv_t::get_timeline_ms();
    _transition_total_period = transition_period;
    _transition_current_period = 0;
    for(int i=0;i<3;i++)
    {
        _axis_transition[i].init(transition_period, start_angle[i],end_angle[i]);
    }
    if(((end_angle[3]-start_angle[3])>pyro::PI)&&start_angle[3]<0 && end_angle[3]>0)
    {
            end_angle[3]=start_angle[3] -(2*pyro::PI-(end_angle[3]-start_angle[3]));
    }
    else if(((end_angle[3]-start_angle[3])<-pyro::PI)&&start_angle[3]>0 && end_angle[3]<0)
    {
            end_angle[3] = start_angle[3]  +(2*pyro::PI+(end_angle[3]-start_angle[3]));
    }

    if(end_angle[3]>pyro::PI*3/2)
    {
        end_angle[3]-=2*pyro::PI;
    }
    else if(end_angle[3]<-pyro::PI*3/2)
    {
        end_angle[3]+=2*pyro::PI;
    }
    _axis_transition[3].init(transition_period, start_angle[3],end_angle[3]);
    for(int i=4;i<6;i++)
    {
        _axis_transition[i].init(transition_period, start_angle[i],end_angle[i]);
    }
}



void motion_transition_t::interpolation_update()
{
    float xdata[3];
    xdata[0] = (pyro::dwt_drv_t::get_timeline_ms()-_transition_start_Tick)/1000.0f;
    xdata[1] = xdata[0] * xdata[0];
    xdata[2] = xdata[1] * xdata[0];

    _transition_current_period = xdata[0];
    for(int i=0;i<6;i++)
    {
        _axis_transition[i].interpolation_update(xdata);
        _interpolation_value[i] = _axis_transition[i].get_transition_interpolation_value();
    }
}

void motion_transition_t::get_transition_interpolation_value(float dst[6])
{ 
    memcpy(dst,_interpolation_value,sizeof(float)*6);
}

float* motion_transition_t::get_transition_interpolation_value()
{
    return _interpolation_value;
}

float motion_transition_t::get_transition_current_period() const
{
    return _transition_current_period;
}

float motion_transition_t::get_transition_total_period() const
{
    return _transition_total_period;
}

bool motion_transition_t::transition_timeout() const
{
    return _transition_current_period>=_transition_total_period;
}

void motion_transition_t::recover_from_pasuse()//从暂停中恢复需重设一次时间戳
{
    _transition_start_Tick = pyro::dwt_drv_t::get_timeline_ms()-_transition_current_period*1000.0f;
}