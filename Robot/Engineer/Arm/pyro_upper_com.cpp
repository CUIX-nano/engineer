#include "pyro_upper_com.h"
#include "pyro_bsp_uart.h"
#include "pyro_crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <cstring>

using namespace pyro;
/*******************************************
 * 这部分代码待修改
 * 回去等底盘出来了结合底盘的数据结构改
 * 注意,还没有加摇臂,考虑一下怎么加
 * 
 * 
 * 
 *****************************************/
// ==================== UART 配置宏（修改此处切换串口） ====================
// 可选值：1, 5, 7, 10
#define CHASSIS_UART_SELECT 1

// ==================== 常量 ====================
static constexpr uint16_t FRAME_HEADER = 0xA5A5;
static constexpr uint32_t CALLBACK_OWNER = 0x02;
static constexpr uint32_t TX_INTERVAL_MS = 10;      // 发送周期 10ms
static constexpr uint32_t RX_TIMEOUT_MS = 100;      // 通信超时 100ms
static constexpr uint8_t RX_QUEUE_SIZE = 10;

// ==================== 帧结构（与底盘协议严格一致） ====================
#pragma pack(push, 1)
struct upper_to_lower_frame_t {
    uint16_t frame_header;
    float vx;
    float vy;
    float wz;
    uint8_t enable;
    uint8_t magazine_pos;       // 固定 0
    uint8_t lift_mode;          // 固定 0
    uint8_t lift_auto_action;   // 固定 0
    uint8_t lift_manual;        // 固定 0
    uint8_t reserved[2];
    uint16_t crc16;
};

struct lower_to_upper_frame_t {
    uint16_t frame_header;
    uint8_t chassis_mode;
    int16_t current_vx;
    int16_t current_vy;
    uint8_t magazine_online;
    float magazine_angle;
    uint8_t lift_left_online;
    uint8_t lift_right_online;
    float lift_left_angle;
    float lift_right_angle;
    uint16_t chassis_power;
    uint8_t power_limited;
    uint8_t reserved[2];
    uint16_t crc16;
};
#pragma pack(pop)

// ==================== 静态变量 ====================
static uart_drv_t* s_uart = nullptr;
static databoard* s_db = nullptr;
static QueueHandle_t s_rx_queue = nullptr;
static TaskHandle_t s_rx_task = nullptr;
static TaskHandle_t s_tx_task = nullptr;

// DataBoard 话题 ID（下行控制）
static uint32_t s_topic_vx;
static uint32_t s_topic_vy;
static uint32_t s_topic_wz;
static uint32_t s_topic_enable;

// DataBoard 话题 ID（上行反馈）
static uint32_t s_topic_online;
static uint32_t s_topic_mode;
static uint32_t s_topic_cur_vx;
static uint32_t s_topic_cur_vy;
static uint32_t s_topic_power;
static uint32_t s_topic_limited;

// 通信在线状态
static TickType_t s_last_rx_tick = 0;
static bool s_is_online = false;

// ==================== 根据宏获取 UART 实例 ====================
static uart_drv_t* get_uart_by_select() {
#if CHASSIS_UART_SELECT == 1
    return &bsp_uart::get_uart1();
#elif CHASSIS_UART_SELECT == 5
    return &bsp_uart::get_uart5();
#elif CHASSIS_UART_SELECT == 7
    return &bsp_uart::get_uart7();
#elif CHASSIS_UART_SELECT == 10
    return &bsp_uart::get_uart10();
#else
    #error "Unsupported CHASSIS_UART_SELECT. Choose 1, 5, 7, or 10."
#endif
}

// ==================== 接收回调（ISR） ====================
static bool rx_callback(uint8_t *buf, uint16_t size, BaseType_t &xHigherPriorityTaskWoken) {
    if (size != sizeof(lower_to_upper_frame_t)) {
        return false;   // 不切换缓冲区
    }
    xQueueSendFromISR(s_rx_queue, buf, &xHigherPriorityTaskWoken);
    return true;        // 切换缓冲区
}

// ==================== 接收任务（解析并写入 DataBoard） ====================
static void rx_task(void *arg) {
    (void)arg;
    uint8_t rx_buf[sizeof(lower_to_upper_frame_t)];

    while (1) {
        if (xQueueReceive(s_rx_queue, rx_buf, portMAX_DELAY) == pdTRUE) {
            auto *frame = reinterpret_cast<lower_to_upper_frame_t*>(rx_buf);

            if (frame->frame_header != FRAME_HEADER) continue;
            if (!verify_crc16_check_sum(rx_buf, sizeof(lower_to_upper_frame_t))) continue;

            s_last_rx_tick = xTaskGetTickCount();

            if (!s_is_online) {
                s_is_online = true;
                genenral_data_t data;
                data.data_ui = 1;
                s_db->write_topic(s_topic_online, data);
            }

            // 写入反馈数据
            genenral_data_t data;
            data.data_ui = frame->chassis_mode;
            s_db->write_topic(s_topic_mode, data);

            data.data_f = (float)frame->current_vx;
            s_db->write_topic(s_topic_cur_vx, data);

            data.data_f = (float)frame->current_vy;
            s_db->write_topic(s_topic_cur_vy, data);

            data.data_ui = frame->chassis_power;
            s_db->write_topic(s_topic_power, data);

            data.data_ui = frame->power_limited;
            s_db->write_topic(s_topic_limited, data);
        }
    }
}

// ==================== 发送任务（定时读取 DataBoard 并发送） ====================
static void tx_task(void *arg) {
    (void)arg;
    upper_to_lower_frame_t tx_frame;
    TickType_t ts;

    while (1) {
        genenral_data_t data;

        // 读取控制指令
        auto read_float = [&](uint32_t id, float &val) {
            if (s_db->read(id, &data, ts) == topic::DATA_OK) {
                val = data.data_f;
            } else {
                val = 0.0f;
            }
        };

        read_float(s_topic_vx, tx_frame.vx);
        read_float(s_topic_vy, tx_frame.vy);
        read_float(s_topic_wz, tx_frame.wz);

        if (s_db->read(s_topic_enable, &data, ts) == topic::DATA_OK) {
            tx_frame.enable = (uint8_t)data.data_ui;
        } else {
            tx_frame.enable = 0;
        }

        // 未使用字段置 0
        tx_frame.magazine_pos = 0;
        tx_frame.lift_mode = 0;
        tx_frame.lift_auto_action = 0;
        tx_frame.lift_manual = 0;
        tx_frame.reserved[0] = 0;
        tx_frame.reserved[1] = 0;

        // 帧头 + CRC
        tx_frame.frame_header = FRAME_HEADER;
        append_crc16_check_sum(reinterpret_cast<uint8_t*>(&tx_frame), sizeof(tx_frame));

        if (s_uart) {
            s_uart->write(reinterpret_cast<uint8_t*>(&tx_frame), sizeof(tx_frame));
        }

        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}

// ==================== 超时检测任务 ====================
static void timeout_task(void *arg) {
    (void)arg;
    while (1) {
        if (s_is_online) {
            TickType_t now = xTaskGetTickCount();
            if (now - s_last_rx_tick > pdMS_TO_TICKS(RX_TIMEOUT_MS)) {
                s_is_online = false;
                genenral_data_t data;
                data.data_ui = 0;
                s_db->write_topic(s_topic_online, data);
                // 安全清零
                data.data_f = 0.0f;
                s_db->write_topic(s_topic_vx, data);
                s_db->write_topic(s_topic_vy, data);
                s_db->write_topic(s_topic_wz, data);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== 绑定 DataBoard 话题 ====================
static void databoard_topics_init() {
    s_topic_vx     = s_db->get_topic_id("chassis_ctrl_vx");
    s_topic_vy     = s_db->get_topic_id("chassis_ctrl_vy");
    s_topic_wz     = s_db->get_topic_id("chassis_ctrl_wz");
    s_topic_enable = s_db->get_topic_id("chassis_ctrl_enable");
    s_topic_online     = s_db->get_topic_id("chassis_feedback_online");
    s_topic_mode       = s_db->get_topic_id("chassis_feedback_mode");
    s_topic_cur_vx     = s_db->get_topic_id("chassis_feedback_current_vx");
    s_topic_cur_vy     = s_db->get_topic_id("chassis_feedback_current_vy");
    s_topic_power      = s_db->get_topic_id("chassis_feedback_power");
    s_topic_limited    = s_db->get_topic_id("chassis_feedback_power_limited");
}

// ==================== 公开初始化函数 ====================
void pyro::upper_com_init(databoard *db_ptr) {
    s_db = db_ptr;

    // 根据宏获取 UART 实例
    s_uart = get_uart_by_select();
    if (!s_uart) return;

    // 创建接收队列
    s_rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(lower_to_upper_frame_t));
    if (!s_rx_queue) return;

    // 注册 DMA 接收回调
    s_uart->add_rx_event_callback(rx_callback, CALLBACK_OWNER);
    s_uart->enable_rx_dma();

    // 绑定话题
    databoard_topics_init();

    // 创建任务
    xTaskCreate(rx_task, "chassis_rx", 512, nullptr, configMAX_PRIORITIES - 2, &s_rx_task);
    xTaskCreate(tx_task, "chassis_tx", 512, nullptr, configMAX_PRIORITIES - 3, &s_tx_task);
    xTaskCreate(timeout_task, "chassis_to", 256, nullptr, configMAX_PRIORITIES - 4, nullptr);
}