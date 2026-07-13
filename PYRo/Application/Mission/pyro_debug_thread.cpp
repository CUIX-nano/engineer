/***************************************************
 * CUIJ 调试任务 - 精简重写版
 * 串口：UART10
 * 接收：DMA + IDLE 中断，固定缓冲区
 * 发送：阻塞轮询，避免 DMA BUSY
 * 命令：CUIJ, CUIJ READ, CUIJ WRITE
 ***************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "pyro_typedef.h"
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

extern pyro::databoard global_databoard;

// ==================== 接收缓冲区 ====================
#define RX_BUF_SIZE 256
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_len = 0;
static volatile bool rx_new_data = false;

// CUIJ 调试串口实例
static pyro::uart_drv_t* cuij_uart = nullptr;

// ==================== DMA 接收回调（ISR 上下文） ====================
static bool cuij_rx_callback(uint8_t* data, uint16_t len, BaseType_t xHigherPriorityTaskWoken) {
    if (len == 0) return false;
    // 拷贝数据到缓冲区（防止溢出）
    uint16_t copy_len = (len > RX_BUF_SIZE) ? RX_BUF_SIZE : len;
    memcpy((void*)rx_buf, data, copy_len);
    rx_len = copy_len;
    rx_new_data = true;
    // 重启 DMA 接收（由驱动在回调返回后自动执行）
    return true;
}

// ==================== 发送函数（阻塞轮询） ====================
static void cuij_send(const char* str) {
    if (!cuij_uart || !str) return;
    cuij_uart->write((uint8_t*)str, strlen(str), 100);  // 阻塞发送，超时100ms
}

static void cuij_send_binary(const uint8_t* data, uint16_t len) {
    if (!cuij_uart || !data || len == 0) return;
    cuij_uart->write(data, len, 100);
}

// ==================== 命令处理 ====================

// CUIJ READ <topic>
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
    // 打包为 A5 + 4字节数据 + 5A
    uint8_t packet[6];
    packet[0] = 0xA5;
    memcpy(&packet[1], &data, 4);
    packet[5] = 0x5A;
    cuij_send_binary(packet, sizeof(packet));
}

// CUIJ WRITE <topic> <value>
static void cuij_cmd_write(const char* topic_name, const char* value_str) {
    uint32_t id = global_databoard.get_topic_id(topic_name);
    if (id == 0xFFFFFFFF) {
        cuij_send("Topic not found.\n");
        return;
    }
    char* endptr;

    // 优先尝试整数解析（适合 UNSIGNED_INT 和 SIGNED_INT）
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

    // 如果整数解析失败，再尝试浮点
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

    // 尝试无符号整数
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
// ==================== 解析入口 ====================
static void cuij_process_line(const char* line) {
    // 去除换行符（已在外部去除）
    char* cmd = strtok((char*)line, " ");
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
    else {
        cuij_send("Unknown subcommand. Use READ or WRITE.\n");
    }
}

// ==================== 主任务 ====================
extern "C" void pyro_debug_task(void* argument) {
    // 1. 获取 UART10 实例
    cuij_uart = pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart10);
    if (!cuij_uart) {
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 2. 启用 DMA 接收并注册回调
    cuij_uart->enable_rx_dma();
    cuij_uart->add_rx_event_callback(cuij_rx_callback, 0xDEADBEEF);

    cuij_send("CUIJ Debug ready. Commands: CUIJ, CUIJ READ <topic>, CUIJ WRITE <topic> <value>\n");

    char line_buf[128];
    uint16_t line_idx = 0;

    while (1) {
        if (rx_new_data) {
            // 处理接收到的数据（可能包含多行或单行）
            // 由于 IDLE 中断触发时通常收到完整的一帧，但可能包含多个 '\n'，我们简单处理：
            // 将 rx_buf 当作字符串，逐行处理
            char* ptr = (char*)rx_buf;
            uint16_t remain = rx_len;
            while (remain > 0) {
                // 查找行结束符
                char* nl = (char*)memchr(ptr, '\n', remain);
                if (nl) {
                    uint16_t line_len = nl - ptr + 1; // 包含 '\n'
                    if (line_len > 0) {
                        // 复制到 line_buf，并替换 '\n' 为 '\0'
                        uint16_t copy_len = (line_len > sizeof(line_buf)-1) ? sizeof(line_buf)-1 : line_len-1;
                        memcpy(line_buf, ptr, copy_len);
                        line_buf[copy_len] = '\0';
                        // 去除 '\r' 如果存在
                        if (copy_len > 0 && line_buf[copy_len-1] == '\r') {
                            line_buf[copy_len-1] = '\0';
                        }
                        cuij_process_line(line_buf);
                    }
                    ptr += line_len;
                    remain -= line_len;
                } else {
                    // 没有换行，可能是数据不完整，忽略剩余
                    break;
                }
            }
            // 清空缓冲区，重置标志
            rx_len = 0;
            rx_new_data = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}