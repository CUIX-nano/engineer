#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"
#include "pyro_bsp_uart.h"          // 新增 BSP 头文件
#include <string.h>
#include "queue.h"

pyro::uart_drv_t* interboard_communication_uart;

extern pyro::databoard* global_databoard;
QueueHandle_t lowerboard_rx_queue;
uint8_t lowerboard_rx_buffer[64];
uint32_t tick_count = 0;

typedef struct __attribute__((packed))
{
    uint16_t frame_header;
    uint8_t sw_l;
    uint8_t sw_r;
    int16_t rc_ch_lx;
    int16_t rc_ch_ly;
    int16_t rc_ch_rx;
    int16_t rc_ch_ry;
    uint8_t zero_force;
    float magazine_angle;
    uint8_t which_motion;
    uint8_t which_mine;
    uint8_t overpass_pose;
    uint16_t crc16;
} upper_board_tx_frame_t;

typedef struct __attribute__((packed))
{
    uint16_t frame_header;
    uint16_t crc16;
} lower_board_tx_frame_t;

upper_board_tx_frame_t upper_board_tx_frame;
lower_board_tx_frame_t lower_board_tx_frame;
__attribute__((section(".dma_heap"))) uint8_t lower_board_tx_buffer[sizeof(lower_board_tx_frame_t)+1];

static uint16_t crc16_append(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for(uint8_t i = 0; i < len; i++)
    {
        crc = ((crc^data[i])&0x00FF)|(crc&0xff00);
        for(uint8_t j = 0; j < 8; j++)
        {
           if(crc&1)
           {
                crc >>= 1;
                crc ^= 0xA001;
           }
           else
           {
                crc >>= 1;
           }
        }
    }
    return crc;
}

// 【修改】回调签名：xHigherPriorityTaskWoken 改为引用类型
bool interboard_communication_callback(uint8_t *buf, uint16_t len, BaseType_t& xHigherPriorityTaskWoken)
{
    if( len != 0 )
    {
        tick_count ++;
        xQueueSendFromISR(lowerboard_rx_queue, buf, &xHigherPriorityTaskWoken);
        return true;
    }
    return false;
}
static uint32_t rc_sw_l_topic_id,rc_sw_r_topic_id,rc_ch_lx_topic_id,rc_ch_ly_topic_id,rc_ch_rx_topic_id,rc_ch_ry_topic_id;
static uint32_t zero_force_id,magazine_angle_id;

uint8_t which_mine=1;
uint8_t which_motion =1;
uint8_t overpass_pose =0;
extern "C" void interboard_communication_mission(void const *argument)
{
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }
    lowerboard_rx_queue = xQueueCreate(10,sizeof(upper_board_tx_frame_t));
    // 【修改】通过 BSP 获取 UART7 实例的指针
    interboard_communication_uart = &pyro::bsp_uart::get_uart7();
    interboard_communication_uart->add_rx_event_callback(interboard_communication_callback, 1);

    rc_sw_l_topic_id = global_databoard->get_topic_id("rc_sw_l");
    rc_sw_r_topic_id = global_databoard->get_topic_id("rc_sw_r");
    rc_ch_lx_topic_id = global_databoard->get_topic_id("rc_ch_lx");
    rc_ch_ly_topic_id = global_databoard->get_topic_id("rc_ch_ly");
    rc_ch_rx_topic_id = global_databoard->get_topic_id("rc_ch_rx");
    rc_ch_ry_topic_id = global_databoard->get_topic_id("rc_ch_ry");
    zero_force_id = global_databoard->get_topic_id("zero_force");
    magazine_angle_id = global_databoard->get_topic_id("magazine_angle");

    for(;;)
    {
        xQueueReceive(lowerboard_rx_queue,lowerboard_rx_buffer,portMAX_DELAY);
        uint16_t crc = crc16_append(((uint8_t*)lowerboard_rx_buffer)+2,sizeof(upper_board_tx_frame_t)-4);
        if(crc == ((upper_board_tx_frame_t*)(lowerboard_rx_buffer))->crc16)
        {
            memcpy(&upper_board_tx_frame,lowerboard_rx_buffer,sizeof(upper_board_tx_frame_t));

            uint32_t temp_i;
            float temp_f;
            temp_i = upper_board_tx_frame.sw_l;
            global_databoard->write_topic(rc_sw_l_topic_id,
            *((pyro::genenral_data_t*)&(temp_i)));
            temp_i = upper_board_tx_frame.sw_r;
            global_databoard->write_topic(rc_sw_r_topic_id,
            *((pyro::genenral_data_t*)&(temp_i)));
            temp_f = ((float)upper_board_tx_frame.rc_ch_lx)/1000.0f;
            global_databoard->write_topic(rc_ch_lx_topic_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)upper_board_tx_frame.rc_ch_ly)/1000.0f;
            global_databoard->write_topic(rc_ch_ly_topic_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)upper_board_tx_frame.rc_ch_rx)/1000.0f;
            global_databoard->write_topic(rc_ch_rx_topic_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)upper_board_tx_frame.rc_ch_ry)/1000.0f;
            global_databoard->write_topic(rc_ch_ry_topic_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_i = upper_board_tx_frame.zero_force;
            global_databoard->write_topic(zero_force_id,
            *((pyro::genenral_data_t*)&(temp_i)));
            temp_f = ((float)upper_board_tx_frame.magazine_angle);
            global_databoard->write_topic(magazine_angle_id,*((pyro::genenral_data_t*)&(temp_f)));

            which_mine = upper_board_tx_frame.which_mine;
            which_motion = upper_board_tx_frame.which_motion;
            overpass_pose = upper_board_tx_frame.overpass_pose;
        }

        vTaskDelay(1);
    }
}