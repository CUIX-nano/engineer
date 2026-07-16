#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_uart_drv.h"
#include "pyro_bsp_uart.h"          // 新增：新库 BSP 头文件
#include "pyro_core_dma_heap.h"
#include "string.h"
#include "pyro_powermeter.h"
#include "pyro_databoard.h"

extern pyro::databoard *global_databoard;
pyro::uart_drv_t *VOFA_uart;

class VOFA_Host
{
    private:
    pyro::uart_drv_t *_uart;
    float _justfloatbuffer[16];
    uint8_t _add_num;
    uint8_t *_send_buffer;
    public:
    VOFA_Host(pyro::uart_drv_t *uart):_uart(uart){
        _add_num = 0;
    }
    ~VOFA_Host(){}
    void addItem(float item)
    {
        _justfloatbuffer[_add_num] = item;
        _add_num++;
    }   
    void reset()
    {
        _add_num = 0;
    }
    void render_and_send()
    {
        static const uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
        if(_send_buffer != nullptr)
        {
            vPortDmaFree(_send_buffer);
        }
        _send_buffer = (uint8_t *)pvPortDmaMalloc(sizeof(float) * (_add_num+1));
        memcpy(_send_buffer,_justfloatbuffer,sizeof(float)*_add_num);
        memcpy(_send_buffer+(_add_num)*sizeof(float),tail,sizeof(tail));
        _uart->write(_send_buffer,(_add_num+1)*sizeof(float));
    }
};
VOFA_Host *vofa;

static uint32_t axis_current_pos_id[6]={};
static uint32_t _axis_target_id[6]={};
static uint32_t _selfcontrol_axis_id[6]={};
static float axis_current_pos[6]={};
static float axis_target_pos[6]={};
static float selfcontrol_axis_pos[6]={};
extern "C" void VOFA_Thread(void *argument)
{
    vTaskDelay(500);
    // 【修改】通过 BSP 获取 UART10 实例的指针
    VOFA_uart = &pyro::bsp_uart::get_uart10();
    vofa = new VOFA_Host(VOFA_uart);
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }
    axis_current_pos_id[0] = global_databoard->get_topic_id("axis1_current_pos");
    axis_current_pos_id[1] = global_databoard->get_topic_id("axis2_current_pos");
    axis_current_pos_id[2] = global_databoard->get_topic_id("axis3_current_pos");
    axis_current_pos_id[3] = global_databoard->get_topic_id("axis4_current_pos");
    axis_current_pos_id[4] = global_databoard->get_topic_id("axis5_current_pos");
    axis_current_pos_id[5] = global_databoard->get_topic_id("axis6_current_pos");

    _axis_target_id[0] = global_databoard->get_topic_id("axis1_target_pos");
    _axis_target_id[1] = global_databoard->get_topic_id("axis2_target_pos");
    _axis_target_id[2] = global_databoard->get_topic_id("axis3_target_pos");
    _axis_target_id[3] = global_databoard->get_topic_id("axis4_target_pos");
    _axis_target_id[4] = global_databoard->get_topic_id("axis5_target_pos");
    _axis_target_id[5] = global_databoard->get_topic_id("axis6_target_pos");

    _selfcontrol_axis_id[0] = global_databoard->get_topic_id("selfcontrol axis1");
    _selfcontrol_axis_id[1] = global_databoard->get_topic_id("selfcontrol axis2");
    _selfcontrol_axis_id[2] = global_databoard->get_topic_id("selfcontrol axis3");
    _selfcontrol_axis_id[3] = global_databoard->get_topic_id("selfcontrol axis4");
    _selfcontrol_axis_id[4] = global_databoard->get_topic_id("selfcontrol axis5");
    _selfcontrol_axis_id[5] = global_databoard->get_topic_id("selfcontrol axis6");
    for(;;)
    {
        uint32_t timestamp;
        for(int i=0;i<6;i++)
        {
            global_databoard->read(axis_current_pos_id[i],(pyro::genenral_data_t*)&(axis_current_pos[i]),timestamp);
            global_databoard->read(_axis_target_id[i],(pyro::genenral_data_t*)&(axis_target_pos[i]),timestamp);
            global_databoard->read(_selfcontrol_axis_id[i],(pyro::genenral_data_t*)&(selfcontrol_axis_pos[i]),timestamp);
        }
        vofa->reset();
        // for(int i=0;i<6;i++)
        //     vofa->addItem(selfcontrol_axis_pos[i]);
        // vofa->addItem(axis_current_pos[i]);
        // for(int i=3;i<6;i++)
        //     vofa->addItem(axis_target_pos[i]*57.32);
        for(int i=0;i<6;i++)
            vofa->addItem(axis_current_pos[i]);
        vofa->render_and_send();
        vTaskDelay(2);
    }
}