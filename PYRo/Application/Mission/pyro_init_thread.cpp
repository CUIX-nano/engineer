#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_dwt_drv.h"
#include "pyro_databoard.h"
extern "C"
{
    pyro::databoard global_databoard;
    pyro::can_drv_t *can1_drv;
    pyro::can_drv_t *can2_drv;
    pyro::can_drv_t *can3_drv;
    void globaldataboard_init();
    void pyro_init_thread(void *argument)
    {
        pyro::dwt_drv_t::init(480); // Initialize DWT at 480 MHz

        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart1)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart5)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart7)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart10)
            ->enable_rx_dma();

        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->enable();
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->enable();

        pyro::can_hub_t::get_instance();
        can1_drv = new pyro::can_drv_t(&hfdcan1);
        can2_drv = new pyro::can_drv_t(&hfdcan2);
        can3_drv = new pyro::can_drv_t(&hfdcan3);
        can1_drv->init();
        can2_drv->init();
        can3_drv->init();
        can1_drv->start();
        can2_drv->start();
        can3_drv->start();
        void globaldataboard_init();
        vTaskDelete(nullptr);
    }



    void globaldataboard_init()
    {
        /*******************标准化的控制信号**************************/
        //大概理解老登为啥写了那一坨了
        //模式
        global_databoard.create_topic("arm_ctrl_mode",      pyro::data_type_t::UNSIGNED_INT);
        //六个关节
        global_databoard.create_topic("arm_ctrl_joint0",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_joint1",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_joint2",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_joint3",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_joint4",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_joint5",    pyro::data_type_t::FLOAT);
        //笛卡尔坐标系
        global_databoard.create_topic("arm_ctrl_position_x",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_position_y",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_position_z",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_orientation_roll",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_orientation_pitch",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_ctrl_orientation_yaw",    pyro::data_type_t::FLOAT);
        //夹爪
        global_databoard.create_topic("arm_ctrl_gripper",    pyro::data_type_t::FLOAT);
        /********************用于执行层的指令******************** */
        //六个关节
        global_databoard.create_topic("arm_command_joint0",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_command_joint1",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_command_joint2",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_command_joint3",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_command_joint4",    pyro::data_type_t::FLOAT);
        global_databoard.create_topic("arm_command_joint5",    pyro::data_type_t::FLOAT);
        //夹爪
        global_databoard.create_topic("arm_command_gripper",    pyro::data_type_t::FLOAT);
        /*******************电机反馈量*********************** */
        //待填充
    }
}