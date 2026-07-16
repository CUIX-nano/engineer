#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_dwt_drv.h"
#include "pyro_databoard.h"

extern "C"
{
    pyro::can_drv_t *can1_drv;
    pyro::can_drv_t *can2_drv;
    pyro::can_drv_t *can3_drv;

    pyro::databoard* global_databoard;

    void pyro_init_thread(void *argument)
    {
        pyro::dwt_drv_t::init(480); // Initialize DWT at 480 MHz

        // ========== 新库：通过 BSP 启用 UART DMA 接收 ==========
        pyro::bsp_uart::get_uart1().enable_rx_dma();
        pyro::bsp_uart::get_uart5().enable_rx_dma();
        pyro::bsp_uart::get_uart7().enable_rx_dma();
        pyro::bsp_uart::get_uart10().enable_rx_dma();

        // RC 初始化（暂时注释）
        // pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->enable();
        // pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->enable();

        // ========== 新库：通过 BSP 初始化 CAN ==========
        pyro::bsp_can::get_can1().init();
        pyro::bsp_can::get_can2().init();
        pyro::bsp_can::get_can3().init();
        pyro::bsp_can::get_can1().start();
        pyro::bsp_can::get_can2().start();
        pyro::bsp_can::get_can3().start();

        // 保留指针（供其他模块使用）
        can1_drv = &pyro::bsp_can::get_can1();
        can2_drv = &pyro::bsp_can::get_can2();
        can3_drv = &pyro::bsp_can::get_can3();

        // ========== DataBoard 初始化 ==========
        global_databoard = new pyro::databoard();
        global_databoard->create_topic("rc_sw_l", pyro::data_type_t::UNSIGNED_INT);
        global_databoard->create_topic("rc_sw_r", pyro::data_type_t::UNSIGNED_INT);
        global_databoard->create_topic("rc_ch_lx", pyro::data_type_t::FLOAT);
        global_databoard->create_topic("rc_ch_ly", pyro::data_type_t::FLOAT);
        global_databoard->create_topic("rc_ch_rx", pyro::data_type_t::FLOAT);
        global_databoard->create_topic("rc_ch_ry", pyro::data_type_t::FLOAT);

        global_databoard->create_topic("motor_torque", pyro::data_type_t::FLOAT);
        global_databoard->create_topic("motor_rotate", pyro::data_type_t::FLOAT);

        global_databoard->create_topic("zero_force", pyro::data_type_t::UNSIGNED_INT);
        global_databoard->create_topic("magazine_angle", pyro::data_type_t::FLOAT);

        vTaskDelete(nullptr);
    }
}