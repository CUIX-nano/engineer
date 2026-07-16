#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"
#include "pyro_bsp_uart.h"          // 新增：新库 BSP 头文件
#include <string.h>
#include "queue.h"
#include "pyro_dwt_drv.h"

extern "C"
{
    #include "CRC8_CRC16.h"
}


typedef struct __attribute__((packed))
{
    uint8_t SOF;
    uint16_t dataLenth;
    uint8_t  seq;
    uint8_t  crc8;
}
tFrameHeader;

typedef struct __attribute__((packed))
{
    tFrameHeader frame_header;
    uint16_t cmd_id;
    float data[7];
    uint8_t used_data[2];
    uint16_t crc16;
}
referee_datalink_frame_t;

referee_datalink_frame_t self_control_frame;


extern pyro::databoard* global_databoard;

pyro::uart_drv_t* self_control_uart_drv;

uint8_t self_control_buf[128];

QueueSetHandle_t self_control_queue;

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
float time_stamp;
float last_time_stamp;
int frame_counter=0;
float frame_rate;

float t_old = 0;
float val_old[6];
float t_new = 0;
float val_new[6];
float t_cur = 0;
float val_cur[6];
float dt= 0.0f;

float new_val[6]={0.0};
float filtered_val[6]={0.0};

float time_stamps[20] = {};
float tsp = 0;
float last_tsp = 0;

// 【修改】回调函数签名：xHigherPriorityTaskWoken 改为引用类型
bool self_control_callback(uint8_t *buf, uint16_t len, BaseType_t& xHigherPriorityTaskWoken)
{
    if( len == sizeof(referee_datalink_frame_t) )
    {
        // 将队列发送的第三个参数改为 &xHigherPriorityTaskWoken
        xQueueSendFromISR(self_control_queue, buf, &xHigherPriorityTaskWoken);
        last_tsp = tsp;
        tsp = pyro::dwt_drv_t::get_timeline_ms();
        for(int i = 0; i < 19; i++)
            {
                time_stamps[i]=time_stamps[i+1];
            }
        time_stamps[19] = (tsp - last_tsp);
        if(pyro::dwt_drv_t::get_timeline_s() - time_stamp > 1.0f)
        {
            last_time_stamp = time_stamp;
            time_stamp = pyro::dwt_drv_t::get_timeline_s();
            // frame_counter = 0;
            frame_rate = frame_counter;
            frame_counter = 0;
            
        }
        else
        {
            frame_counter++;
        }
        return true;
    }
    return false;
}

static uint32_t selfcontrol_axis1_id,selfcontrol_axis2_id,selfcontrol_axis3_id,selfcontrol_axis4_id,selfcontrol_axis5_id,selfcontrol_axis6_id;

float alpha = 0.02;


extern "C" void self_control_mission(void* args)
{
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }

    self_control_queue = xQueueCreate(10, sizeof(referee_datalink_frame_t));

    // 【修改】通过 BSP 获取 UART1 实例的指针
    self_control_uart_drv = &pyro::bsp_uart::get_uart1();
    time_stamp = pyro::dwt_drv_t::get_timeline_s();
    self_control_uart_drv->add_rx_event_callback(self_control_callback, 2);

    selfcontrol_axis1_id = global_databoard->get_topic_id("selfcontrol axis1");
    selfcontrol_axis2_id = global_databoard->get_topic_id("selfcontrol axis2");
    selfcontrol_axis3_id = global_databoard->get_topic_id("selfcontrol axis3");
    selfcontrol_axis4_id = global_databoard->get_topic_id("selfcontrol axis4");
    selfcontrol_axis5_id = global_databoard->get_topic_id("selfcontrol axis5");
    selfcontrol_axis6_id = global_databoard->get_topic_id("selfcontrol axis6");

    for(;;)
    {   
        if(xQueueReceive(self_control_queue, self_control_buf, 0)==pdPASS)
        {
            ;
            // uint16_t crc = crc16_append(((uint8_t*)&self_control_buf)+2, sizeof(referee_datalink_frame_t)-4);
            if( verify_CRC8_check_sum(self_control_buf,sizeof(tFrameHeader))&& verify_CRC16_check_sum(self_control_buf,sizeof(referee_datalink_frame_t)) && ((referee_datalink_frame_t*)self_control_buf)->frame_header.SOF == 0xA5 && ((referee_datalink_frame_t*)self_control_buf)->cmd_id == 0x302 && ((referee_datalink_frame_t*)self_control_buf)->frame_header.dataLenth == 30 )
            {
                memcpy(&self_control_frame, self_control_buf, sizeof(referee_datalink_frame_t));
                memcpy(new_val,&self_control_frame.data[0],sizeof(float)*6);
                // t_old = t_new;
                // memcpy(val_old,val_cur,sizeof(float)*6);
                // t_new = pyro::dwt_drv_t::get_timeline_s();
                // memcpy(val_new,&self_control_frame.data[0],sizeof(float)*6);
                // dt = t_new - t_old;
            }
        }

        {
            // t_cur = pyro::dwt_drv_t::get_timeline_s();
            // if(t_cur > t_new + dt)
            // {
            //     // memcpy(val_cur,new_val,sizeof(float)*6);
            // }
            // else
            // {
            //     float ratio = (t_cur - t_new)/dt;
            //     for(int i = 0; i < 6; i++)
            //     {
                    
            //         val_cur[i] = val_old[i] + (val_new[i] - val_old[i]) * ratio;
            //     }
            // }
            for(int i = 0; i < 6; i++)
            {
                filtered_val[i] = filtered_val[i] * (1-alpha) + new_val[i] * alpha;
            }

            float temp_f;

            temp_f = ((float)filtered_val[0]);
            global_databoard->write_topic(selfcontrol_axis1_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)filtered_val[1]);
            global_databoard->write_topic(selfcontrol_axis2_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)filtered_val[2]);
            global_databoard->write_topic(selfcontrol_axis3_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)filtered_val[3]);
            global_databoard->write_topic(selfcontrol_axis4_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)filtered_val[4]);
            global_databoard->write_topic(selfcontrol_axis5_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)filtered_val[5]);
            global_databoard->write_topic(selfcontrol_axis6_id,*((pyro::genenral_data_t*)&(temp_f)));
        }
        
        vTaskDelay(1);
    }
}