#include "pyro_crc.h"
#include "pyro_uart_drv.h"
#include "pyro_bsp_uart.h"
#include "self_control_command.h"
#include "cmsis_os.h"
#include <cstring>
#include <string.h>
#include <cstdio>
#include <cstdlib>

// 原代码中计算了帧间隔和帧率，未知原因
// 自控接收串口实例
static pyro::uart_drv_t* self_control_uart = nullptr;
referee_datalink_frame_t self_control_command;
extern pyro::databoard global_databoard;

QueueSetHandle_t self_control_queue;

uint8_t self_control_buf[128];
float command_buf[6];           // 串口传入的命令
float filter_command[6];        // 经给低通滤波器，最终的目标命令
static float alpha = 0.02f;

uint32_t self_axis_id[6];

static bool self_control_callback(uint8_t *buf, uint16_t len,BaseType_t &xHigherPriorityTaskWoken)
{
    if( len == sizeof(referee_datalink_frame_t) )
    {
        xQueueSendFromISR(self_control_queue, buf,  &xHigherPriorityTaskWoken);
        return true;
    }
    else 
        return false;
}


extern "C" void self_command_updata_thread(void *arg)
{
    for(;;)
    {
        if(xQueueReceive(self_control_queue, self_control_buf, 0)==pdPASS)
        {
            if(verify_crc8_check_sum(self_control_buf,sizeof(pyro::frame_header_t))&&
            verify_crc16_check_sum(self_control_buf, sizeof(referee_datalink_frame_t))&&
            ((referee_datalink_frame_t*)self_control_buf)->header.sof == 0xA5&&
            ((referee_datalink_frame_t*)self_control_buf)->cmd_id == static_cast<uint16_t>(pyro::cmd_id::CUSTOM_CONTROLLER)&&
            ((referee_datalink_frame_t*)self_control_buf)->header.data_length == 30)
            {
                memcpy(&self_control_command, self_control_buf, sizeof(referee_datalink_frame_t));
                memcpy(command_buf,&self_control_command.data[0],sizeof(float)*6);
                for(int i=0; i<6; i++)
                {
                    filter_command[i] = filter_command[i]*(1-alpha) + command_buf[i]*alpha;
                }
                global_databoard.write_topic(self_axis_id[0],*((pyro::genenral_data_t*)&(filter_command[0])));
                global_databoard.write_topic(self_axis_id[1],*((pyro::genenral_data_t*)&(filter_command[1])));
                global_databoard.write_topic(self_axis_id[2],*((pyro::genenral_data_t*)&(filter_command[2])));
                global_databoard.write_topic(self_axis_id[3],*((pyro::genenral_data_t*)&(filter_command[3])));
                global_databoard.write_topic(self_axis_id[4],*((pyro::genenral_data_t*)&(filter_command[4])));
                global_databoard.write_topic(self_axis_id[5],*((pyro::genenral_data_t*)&(filter_command[5])));
            }
        }
        vTaskDelay(1);
    }
}

extern "C" void self_control_command_init(void *arg)
{
    self_control_queue = xQueueCreate(10, sizeof(referee_datalink_frame_t));
    self_control_uart = &pyro::bsp_uart::get_uart1();
    // 注册回调
    self_control_uart->add_rx_event_callback(self_control_callback, 0xA501);
    //显式开启 DMA 接收
    self_control_uart->enable_rx_dma();
    self_control_uart->write((uint8_t*)"ok", strlen("ok"), 100);

    self_axis_id[0] = global_databoard.get_topic_id("axis1_self_command");
    self_axis_id[1] = global_databoard.get_topic_id("axis2_self_command");
    self_axis_id[2] = global_databoard.get_topic_id("axis3_self_command");
    self_axis_id[3] = global_databoard.get_topic_id("axis4_self_command");
    self_axis_id[4] = global_databoard.get_topic_id("axis5_self_command");
    self_axis_id[5] = global_databoard.get_topic_id("axis6_self_command");

    xTaskCreate(self_command_updata_thread, "self_command_updata_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

    vTaskDelete(nullptr);
}