#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_rc_hub.h"
#include "pyro_bsp_uart.h"
#include "pyro_rc_core.h"   // 新增，解决 sw_pos_t 无法识别问题
#include "string.h"
#include "pyro_uart_drv.h"
#include "usart.h"

extern pyro::databoard* global_databoard;
static pyro::rc_drv_t* dr16_drv;
static pyro::rc_drv_t* vt03_drv;

// ... 其余代码不变 ...

typedef struct __attribute__((packed))
{
    uint16_t frame_header;
    uint8_t sw_l;
    uint8_t sw_r;
    int16_t chassis_vx;
    int16_t chassis_vy;
    int16_t chassis_wz;
    int16_t rc_ch_ry;
    uint8_t zero_force;
    float magazine_angle;
    uint8_t which_motion;
    uint8_t which_mine;
    uint8_t overpass_pose;
    uint16_t crc16;
} upper_board_tx_frame_t;

upper_board_tx_frame_t upper_board_tx_frame;
__attribute__((section(".dma_heap"))) uint8_t upper_board_tx_buffer[sizeof(upper_board_tx_frame_t)+1];

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

uint32_t zero_force_id = 0;
uint32_t magazine_angle_id = 0;
extern int get_mine_motion;
extern uint8_t overpass_pose;

void upper_board_tx_frame_update()
{
    // 优先使用 DR16，若在线则读取，否则使用 VT03
    if (dr16_drv->check_online())
    {
        const auto &rc = dr16_drv->read();  // virtual_rc_t&

        // 拨杆
        upper_board_tx_frame.sw_l = static_cast<uint8_t>(rc.switches.left.current_pos);
        upper_board_tx_frame.sw_r = static_cast<uint8_t>(rc.switches.right.current_pos);

        // 速度增益：ctrl 键按下时变为 0.05
        float speed_gain = rc.keys.ctrl.state ? 0.05f : 1.0f;

        // 底盘速度计算（与原有逻辑一致）
        if (rc.keys.a.state || rc.keys.d.state)
        {
            upper_board_tx_frame.chassis_vx = (int16_t)((rc.keys.d.state - rc.keys.a.state) * 1000 * speed_gain);
        }
        else
        {
            upper_board_tx_frame.chassis_vx = (int16_t)(rc.axes.lx * 1000);
        }

        if (rc.keys.w.state || rc.keys.s.state)
        {
            upper_board_tx_frame.chassis_vy = (int16_t)((rc.keys.w.state - rc.keys.s.state) * 1000 * speed_gain);
        }
        else
        {
            upper_board_tx_frame.chassis_vy = (int16_t)(rc.axes.ly * 1000);
        }

        if (rc.keys.q.state || rc.keys.e.state)
        {
            upper_board_tx_frame.chassis_wz = (int16_t)((rc.keys.e.state - rc.keys.q.state) * 1000 * speed_gain);
        }
        else
        {
            upper_board_tx_frame.chassis_wz = (int16_t)(rc.axes.rx * 1000);
        }

        upper_board_tx_frame.rc_ch_ry = (int16_t)(rc.axes.ry * 1000);
    }
    else if (vt03_drv->check_online())
    {
        const auto &rc = vt03_drv->read();
        // VT03 的挡位映射为 DR16 的拨杆
        if (rc.switches.gear.current_pos == pyro::sw_pos_t::UP)
            upper_board_tx_frame.sw_r = static_cast<uint8_t>(pyro::sw_pos_t::UP);
        else if (rc.switches.gear.current_pos == pyro::sw_pos_t::MID)
            upper_board_tx_frame.sw_r = static_cast<uint8_t>(pyro::sw_pos_t::MID);
        else if (rc.switches.gear.current_pos == pyro::sw_pos_t::DOWN)
            upper_board_tx_frame.sw_r = static_cast<uint8_t>(pyro::sw_pos_t::DOWN);
        // VT03 没有左拨杆，可置默认值
        upper_board_tx_frame.sw_l = static_cast<uint8_t>(pyro::sw_pos_t::UNKNOWN);

        float speed_gain = rc.keys.ctrl.state ? 0.05f : 1.0f;

        if (rc.keys.a.state || rc.keys.d.state)
        {
            upper_board_tx_frame.chassis_vx = (int16_t)((rc.keys.d.state - rc.keys.a.state) * 1000 * speed_gain);
        }
        else
        {
            upper_board_tx_frame.chassis_vx = (int16_t)(rc.axes.ly * 1000); // VT03 的 ly 映射为 x 轴
        }

        if (rc.keys.w.state || rc.keys.s.state)
        {
            upper_board_tx_frame.chassis_vy = (int16_t)((rc.keys.w.state - rc.keys.s.state) * 1000 * speed_gain);
        }
        else
        {
            upper_board_tx_frame.chassis_vy = (int16_t)(rc.axes.lx * 1000);
        }

        if (rc.keys.q.state || rc.keys.e.state)
        {
            upper_board_tx_frame.chassis_wz = (int16_t)((rc.keys.e.state - rc.keys.q.state) * 1000 * speed_gain);
        }
        else
        {
            upper_board_tx_frame.chassis_wz = (int16_t)(rc.axes.rx * 1000);
        }

        upper_board_tx_frame.rc_ch_ry = (int16_t)(rc.axes.ry * 1000);
    }

    // 读取 DataBoard 数据
    uint32_t zero_force = 0;
    uint32_t timestamp = 0;
    global_databoard->read(zero_force_id, (pyro::genenral_data_t*)&zero_force, timestamp);
    upper_board_tx_frame.zero_force = (uint8_t)zero_force;

    global_databoard->read(magazine_angle_id, (pyro::genenral_data_t*)(&upper_board_tx_frame.magazine_angle), timestamp);

    // 计算矿仓档位（使用 pyro::PI）
    {
        float angle = upper_board_tx_frame.magazine_angle;
        if (angle < pyro::PI/4 && angle > -pyro::PI/4)
            upper_board_tx_frame.which_mine = 1;
        else if (angle < pyro::PI*3/4 && angle > pyro::PI/4)
            upper_board_tx_frame.which_mine = 2;
        else if (angle > -pyro::PI*3/4 && angle < -pyro::PI/4)
            upper_board_tx_frame.which_mine = 4;
        else
            upper_board_tx_frame.which_mine = 3;
    }

    upper_board_tx_frame.which_motion = get_mine_motion;
    upper_board_tx_frame.overpass_pose = overpass_pose;

    // CRC
    upper_board_tx_frame.crc16 = crc16_append(((uint8_t*)&upper_board_tx_frame)+2, sizeof(upper_board_tx_frame_t)-4);
    memcpy(upper_board_tx_buffer, &upper_board_tx_frame, sizeof(upper_board_tx_frame_t));
}

pyro::uart_drv_t* interboard_communication_uart_drv;
extern "C" void interboard_communication_mission(void* args)
{
    dr16_drv = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
    vt03_drv = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03);

    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }

    zero_force_id = global_databoard->get_topic_id("zero_force");
    magazine_angle_id = global_databoard->get_topic_id("magazine_angle");

    upper_board_tx_frame.frame_header = 0xffA5;

    // 改用 BSP 获取 UART7
    interboard_communication_uart_drv = &pyro::bsp_uart::get_uart7();

    for(;;)
    {
        upper_board_tx_frame_update();
        interboard_communication_uart_drv->write(upper_board_tx_buffer, sizeof(upper_board_tx_frame_t));
        vTaskDelay(10);
    }
}