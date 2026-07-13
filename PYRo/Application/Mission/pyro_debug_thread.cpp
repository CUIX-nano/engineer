/***************************************************
 * CUIJ 调试任务：通过串口访问整个 DataBoard
 * 指令格式：CUIJ [子命令] [参数...]
 * 串口：UART10（通过 DMA + 回调接收，已验证发送正常）
 * 全局 DataBoard 为对象：global_databoard（定义在 pyro_typedef.h 中）
 ***************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "pyro_typedef.h"
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

// 外部引用全局 DataBoard 对象
extern pyro::databoard global_databoard;

// CUIJ 调试串口实例
static pyro::uart_drv_t* cuij_uart = nullptr;

// ==================== 环形接收缓冲区 ====================
#define RX_RING_SIZE 256
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;  // 写入位置（由回调更新）
static volatile uint16_t rx_tail = 0;  // 读取位置（由主循环更新）

// 从环形缓冲区读取一个字节，若无数据返回 -1
static int cuij_getchar_from_ring() {
    if (rx_head == rx_tail) return -1;
    uint8_t ch = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % RX_RING_SIZE;
    return ch;
}

// ==================== DMA 接收回调函数（ISR 上下文） ====================
// 该回调由 UART DMA 空闲中断触发，将接收到的数据存入环形缓冲区
static bool cuij_rx_callback(uint8_t* data, uint16_t len, BaseType_t xHigherPriorityTaskWoken) {
    if (len == 0) return false;

    // 将数据逐字节写入环形缓冲区（需防止覆盖）
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next_head = (rx_head + 1) % RX_RING_SIZE;
        if (next_head != rx_tail) {  // 缓冲区未满
            rx_ring[rx_head] = data[i];
            rx_head = next_head;
        } else {
            // 缓冲区已满，丢弃后续数据（可根据需求增加溢出计数）
            break;
        }
    }
    return true;  // 消费数据，驱动将切换 DMA 缓冲区
}

// ==================== 串口发送函数（与之前相同） ====================
static void cuij_send(const char* str) {
    if (cuij_uart) {
        cuij_uart->write((uint8_t*)str, strlen(str));
    }
}

// ==================== 辅助函数 ====================
static void cuij_format_value(pyro::genenral_data_t* data, char* buf, size_t buf_len) {
    snprintf(buf, buf_len, "float:%.6f  int:%d  uint:%u",
             data->data_f, data->data_si, data->data_ui);
}

// ==================== 命令处理函数（保持不变） ====================
static void cuij_cmd_help() {
    cuij_send("Commands:\n"
              "  CUIJ                     -> OK\n"
              "  CUIJ READ <topic>        -> print value\n"
              "  CUIJ WRITE <topic> <val> -> write value\n"
              "  CUIJ WATCH <topics...>   -> start monitoring (binary)\n"
              "  CUIJ WATCH CLOSE         -> stop monitoring\n");
}

static void cuij_cmd_read(const char* topic_name) {
    uint32_t id = global_databoard.get_topic_id(topic_name);
    if (id == 0xFFFFFFFF) {
        cuij_send("Topic not found.\n");
        return;
    }
    pyro::genenral_data_t data;
    TickType_t timestamp;
    auto status = global_databoard.read(id, &data, timestamp);
    if (status == pyro::topic::DATA_INVALID) {
        cuij_send("No valid data yet.\n");
        return;
    } else if (status != pyro::topic::DATA_OK) {
        cuij_send("Read error.\n");
        return;
    }
    char buf[80];
    cuij_format_value(&data, buf, sizeof(buf));
    cuij_send(buf);
    cuij_send("\n");
}

static void cuij_cmd_write(const char* topic_name, const char* value_str) {
    uint32_t id = global_databoard.get_topic_id(topic_name);
    if (id == 0xFFFFFFFF) {
        cuij_send("Topic not found.\n");
        return;
    }
    char* endptr;
    float fval = strtof(value_str, &endptr);
    if (*endptr == '\0') {
        pyro::genenral_data_t data;
        data.data_f = fval;
        auto ret = global_databoard.write_topic(id, data);
        if (ret == pyro::topic::DATA_OK)
            cuij_send("Write OK.\n");
        else
            cuij_send("Write failed (maybe type mismatch).\n");
        return;
    }
    int32_t ival = (int32_t)strtol(value_str, &endptr, 10);
    if (*endptr == '\0') {
        pyro::genenral_data_t data;
        data.data_si = ival;
        auto ret = global_databoard.write_topic(id, data);
        if (ret == pyro::topic::DATA_OK)
            cuij_send("Write OK.\n");
        else
            cuij_send("Write failed.\n");
        return;
    }
    uint32_t uval = (uint32_t)strtoul(value_str, &endptr, 10);
    if (*endptr == '\0') {
        pyro::genenral_data_t data;
        data.data_ui = uval;
        auto ret = global_databoard.write_topic(id, data);
        if (ret == pyro::topic::DATA_OK)
            cuij_send("Write OK.\n");
        else
            cuij_send("Write failed.\n");
        return;
    }
    cuij_send("Invalid number format.\n");
}

// ==================== 监视模式相关（保持不变） ====================
static bool cuij_watch_enabled = false;
static uint32_t cuij_watch_ids[10];
static uint8_t cuij_watch_count = 0;

static void cuij_cmd_watch(const char* arg) {
    if (!arg) {
        cuij_send("Missing arguments.\n");
        return;
    }
    if (strcmp(arg, "CLOSE") == 0) {
        cuij_watch_enabled = false;
        cuij_watch_count = 0;
        cuij_send("Watch closed.\n");
        return;
    }
    cuij_watch_count = 0;
    char* args_copy = strdup(arg);
    if (!args_copy) {
        cuij_send("Memory error.\n");
        return;
    }
    char* token = strtok(args_copy, " ");
    while (token && cuij_watch_count < 10) {
        uint32_t id = global_databoard.get_topic_id(token);
        if (id == 0xFFFFFFFF) {
            cuij_send("Topic not found: ");
            cuij_send(token);
            cuij_send("\n");
            free(args_copy);
            cuij_watch_count = 0;
            cuij_watch_enabled = false;
            return;
        }
        cuij_watch_ids[cuij_watch_count++] = id;
        token = strtok(nullptr, " ");
    }
    free(args_copy);
    if (cuij_watch_count == 0) {
        cuij_send("No valid topics.\n");
        cuij_watch_enabled = false;
        return;
    }
    cuij_watch_enabled = true;
    cuij_send("Watch started.\n");
}

// ==================== 命令解析入口（保持不变） ====================
static void cuij_process_line(char* line) {
    char* nl = strchr(line, '\n');
    if (nl) *nl = '\0';
    nl = strchr(line, '\r');
    if (nl) *nl = '\0';

    char* cmd = strtok(line, " ");
    if (!cmd) return;

    if (strcmp(cmd, "CUIJ") != 0) {
        cuij_send("Unknown command. Use 'CUIJ ...'\n");
        return;
    }

    char* sub = strtok(nullptr, " ");
    if (!sub) {
        cuij_send("OK\n");
        return;
    }

    if (strcmp(sub, "READ") == 0) {
        char* topic = strtok(nullptr, " ");
        if (topic) cuij_cmd_read(topic);
        else cuij_send("Missing topic name.\n");
    }
    else if (strcmp(sub, "WRITE") == 0) {
        char* topic = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (topic && val) cuij_cmd_write(topic, val);
        else cuij_send("Missing arguments.\n");
    }
    else if (strcmp(sub, "WATCH") == 0) {
        char* rest = strtok(nullptr, "");
        cuij_cmd_watch(rest);
    }
    else if (strcmp(sub, "HELP") == 0) {
        cuij_cmd_help();
    }
    else {
        cuij_send("Unknown subcommand. Use HELP.\n");
    }
}

// ==================== 监视模式发送函数（保持不变） ====================
static void cuij_send_watch_packet() {
    if (cuij_watch_count == 0 || !cuij_watch_enabled)
        return;

    uint8_t packet[1 + 4*10 + 1];
    uint8_t* p = packet;
    *p++ = 0xA5;

    for (uint8_t i = 0; i < cuij_watch_count; i++) {
        pyro::genenral_data_t data;
        TickType_t ts;
        auto status = global_databoard.read(cuij_watch_ids[i], &data, ts);
        if (status == pyro::topic::DATA_OK) {
            memcpy(p, &data, 4);
            p += 4;
        } else {
            memset(p, 0, 4);
            p += 4;
        }
    }
    *p++ = 0x5A;

    cuij_uart->write(packet, p - packet);
}

// ==================== 主任务 ====================
extern "C" void pyro_debug_task(void* argument) {
    // 1. 获取 UART10 实例
    cuij_uart = pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart10);
    if (!cuij_uart) {
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 2. 启用 DMA 接收（如果尚未启用）
    cuij_uart->enable_rx_dma();

    // 3. 注册接收回调
    cuij_uart->add_rx_event_callback(cuij_rx_callback, 0xDEADBEEF); // 任意 owner ID

    // 4. 初始化变量
    char rx_buffer[128];
    uint16_t idx = 0;
    TickType_t last_watch_time = 0;
    TickType_t last_heartbeat_time = 0;

    cuij_send("CUIJ Debug task started. Type 'CUIJ HELP' for commands.\n");

    while (1) {
        // 从环形缓冲区读取字符
        int ch = cuij_getchar_from_ring();

        if (ch >= 0) {
            if (ch == '\r' || ch == '\n') {
                rx_buffer[idx] = '\0';
                if (idx > 0) {
                    cuij_process_line(rx_buffer);
                }
                idx = 0;
            } else if (idx < sizeof(rx_buffer)-1) {
                rx_buffer[idx++] = (char)ch;
            }
        }

        // 监视模式发送（每 100ms）
        if (cuij_watch_enabled) {
            TickType_t now = xTaskGetTickCount();
            if (now - last_watch_time >= pdMS_TO_TICKS(100)) {
                cuij_send_watch_packet();
                last_watch_time = now;
            }
        }

        // 心跳：每秒发送一次
        TickType_t now = xTaskGetTickCount();
        if (now - last_heartbeat_time >= pdMS_TO_TICKS(1000)) {
            cuij_send("CUIJ heartbeat\r\n");
            last_heartbeat_time = now;
        }

        // 延时：若未收到字符且监视模式关闭，则延时 100ms，否则 1ms
        if (ch < 0 && !cuij_watch_enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}