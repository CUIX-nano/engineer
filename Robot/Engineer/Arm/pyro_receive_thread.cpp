/**
 * @file pyro_receive_thread.cpp
 * @brief 遥控器接收任务，直接使用新版 RC 驱动（无 rc_hub）
 * @note global_databoard 是实例，使用 . 操作符
 * @note vx, vy, wz 为 FLOAT 类型
 */
#ifndef DR16_UART
#define DR16_UART PYRO_UART5
#endif
#ifndef VT03_UART
#define VT03_UART PYRO_UART1
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "pyro_databoard.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_core.h"   // sw_pos_t
#include "pyro_rw_lock.h"       // read_scope_lock

// 确保 DR16_UART 宏被定义，以便 instance() 方法可见


extern pyro::databoard global_databoard;

// 缓存话题 ID
static uint32_t s_topic_vx      = 0;
static uint32_t s_topic_vy      = 0;
static uint32_t s_topic_wz      = 0;
static uint32_t s_topic_enable  = 0;
static uint32_t s_topic_mode    = 0;

extern "C" void pyro_receive_thread(void *argument)
{
    // 等待 DataBoard 话题创建完成
    while (global_databoard.get_topic_id("chassis_ctrl_vx") == 0xFFFFFFFF) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 获取话题 ID
    s_topic_vx     = global_databoard.get_topic_id("chassis_ctrl_vx");
    s_topic_vy     = global_databoard.get_topic_id("chassis_ctrl_vy");
    s_topic_wz     = global_databoard.get_topic_id("chassis_ctrl_wz");
    s_topic_enable = global_databoard.get_topic_id("chassis_ctrl_enable");
    s_topic_mode   = global_databoard.get_topic_id("arm_ctrl_mode");

    pyro::genenral_data_t data;

    while (1) {
        // 默认值：速度为 0，使能关闭，模式为 0
        float vx = 0.0f, vy = 0.0f, wz = 0.0f;
        uint32_t enable = 0;
        uint32_t mode = 0;

        // 优先使用 DR16
        if (pyro::dr16_drv_t::instance().check_online()) {
            pyro::read_scope_lock lock(pyro::dr16_drv_t::instance().get_lock());
            const auto& vrc = pyro::rc_drv_t::read();

            // 摇杆映射：直接存储浮点数（范围 -1.0 ~ 1.0）
            vx = vrc.axes.ly;   // 左摇杆纵向
            vy = vrc.axes.lx;   // 左摇杆横向
            wz = vrc.axes.rx;   // 右摇杆横向

            // 左拨杆向上 → 使能，否则 0
            enable = (vrc.switches.left.current_pos == pyro::sw_pos_t::UP) ? 1 : 0;
            // 右拨杆向上 → 模式为 1，否则 0
            mode = (vrc.switches.right.current_pos == pyro::sw_pos_t::UP) ? 1 : 0;
        }
        else if (pyro::vt03_drv_t::instance().check_online()) {
            pyro::read_scope_lock lock(pyro::vt03_drv_t::instance().get_lock());
            const auto& vrc = pyro::rc_drv_t::read();

            // VT03 的摇杆映射（注意 VT03 的 lx/ly 可能与 DR16 相反）
            vx = vrc.axes.ly;
            vy = vrc.axes.lx;
            wz = vrc.axes.rx;

            // VT03 使用 gear 挡位模拟拨杆
            enable = (vrc.switches.gear.current_pos == pyro::sw_pos_t::UP) ? 1 : 0;
            mode   = (vrc.switches.gear.current_pos == pyro::sw_pos_t::MID) ? 1 : 0;
        }
        // 若都离线，则保持默认值（vx/vy/wz=0，enable=0，mode=0）

        // 写入 DataBoard（注意类型匹配）
        // chassis_ctrl_vx/vy/wz 为 FLOAT → 使用 data.data_f
        data.data_f = vx;
        global_databoard.write_topic(s_topic_vx, data);

        data.data_f = vy;
        global_databoard.write_topic(s_topic_vy, data);

        data.data_f = wz;
        global_databoard.write_topic(s_topic_wz, data);

        // chassis_ctrl_enable 为 SIGNED_INT → 使用 data.data_si
        data.data_si = (int32_t)enable;
        global_databoard.write_topic(s_topic_enable, data);

        // arm_ctrl_mode 为 UNSIGNED_INT → 使用 data.data_ui
        data.data_ui = mode;
        global_databoard.write_topic(s_topic_mode, data);

        // 100Hz 更新
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}