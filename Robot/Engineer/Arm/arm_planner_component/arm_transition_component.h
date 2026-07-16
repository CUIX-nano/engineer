#ifndef __ARM_TRANSITION_COMPONENT_H__
#define __ARM_TRANSITION_COMPONENT_H__

#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"

#include "pyro_core_def.h"

//本模块用来定义一个过渡模块，目的是使用三次多项式插值的方式，为机械臂提供在两个点之间的平滑过渡
//本模块的命名不太好记忆

//过渡状态枚举
typedef enum
{
    Not_transition,
    Transition_start,
    Transition_running,
    Transition_reverse_running,
    Transition_end
}
transition_state_t;

/********************************* 老版本的插值过渡模块***********************************/

//单个过渡参数结构体
typedef struct
{ 
    //过渡开始位置
    float transition_start_pos;
    //过渡结束位置
    float transition_end_pos;
    //过渡系数(通过起始位置，结束位置以及过渡时间计算出，用于生成插值)
    float transition_coefficient[4];
    //过渡此时刻目标位置
    float transition_target_pos;
    //过渡此时刻当前位置
    float transition_current_pos;
}
transition_param_t;

//机械臂过渡参数结构体
typedef struct
{
    //过渡开始时间戳，在一段过渡过程开始时，记录此时时间戳作为起始时刻开始计时器
    uint32_t transition_start_Tick;
    //过渡的总时间(s)
    float transition_total_time;
    //过渡当前时间(s)
    float transition_current_time;
    //六个旋转轴的过渡参数
    transition_param_t axis_transition_param[6];
}
arm_transition_t;

//对一个旋转轴的过渡过程进行初始化
void axis_transition_init(transition_param_t* param,float transition_period,float start_angle,float end_angle);

//对整个机械臂的过渡过程进行初始化
void arm_transition_init(arm_transition_t* param, float transition_period, float start_angle[6],float end_angle[6]);

//更新机械臂的过渡过程
void arm_transition_update(arm_transition_t* param);


/********************************* 新版本的插值过渡模块***********************************/

//插值类
//使用类与对象封装插值函数，以减少代码量
class value_interpolation_t
{
    public:
        //构造函数
        value_interpolation_t();
        ~value_interpolation_t();
        //初始化插值参数
        void init(float transition_period,float start_value,float end_value);
        //传入浮点数数组[dt^3,dt^2,dt]，更新插值参数,目的是减少计算量，否则进传入dt需要重复计算6次影响效率
        void interpolation_update(float xdata[3]);
        //获取插值结果
        float get_transition_interpolation_value() const;
    private:
        //插值时长
        float _interpolation_period;
        //插值开始值
        float _interpolation_start_value;
        //插值结束值
        float _interpolation_end_value;
        //插值系数
        float _interpolation_coefficient[4];
        //插值结果
        float _interpolation_value;
};

//对整个动作进行插值
class motion_transition_t
{
    public:
        motion_transition_t();
        ~motion_transition_t();
        //初始化插值参数
        void init(float transition_period, float start_angle[6],float end_angle[6]);
        //对六个插值对象进行更新插值参数
        void interpolation_update();
        //获取插值结果，将结果存入外部数组
        void get_transition_interpolation_value(float xdata[6]);
        //获取插值结果，返回内部数组指针
        float* get_transition_interpolation_value();
        //获取当前插值时间
        float get_transition_current_period() const;
        //获取插值总时间
        float get_transition_total_period() const;
        //判断过渡是否超时
        bool transition_timeout() const;
        //从暂停状态恢复过渡
        void recover_from_pasuse();
    private:
        //六个轴的插值实例
        value_interpolation_t _axis_transition[6];
        //过渡开始时间戳
        float _transition_start_Tick;
        //过渡总时间
        float _transition_total_period;
        //过渡当前时间
        float _transition_current_period;
        //插值结果
        float _interpolation_value[6];
};

#endif