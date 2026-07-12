/***************************************************
 * CUIJ 调试任务：通过串口访问整个 DataBoard
 * 指令格式：CUIJ [子命令] [参数...]
 * 串口：通过 CUIJ_UART_SELECT 宏配置
 * 全局 DataBoard 为对象：global_databoard（定义在 pyro_typedef.h 中）
 ***************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "pyro_typedef.h"          // 包含 global_databoard 定义（对象）
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"
#include "stm32h7xx_hal_uart.h"    // 直接使用 HAL 接收函数
#include <cstring>
#include <cstdio>
#include <cstdlib>

// ==================== CUIJ 串口配置（修改此处切换串口） ====================
// 可选值：uart1, uart5, uart7, uart10
#define CUIJ_UART_SELECT uart1

// 根据选择自动映射到 HAL 句柄和枚举
#if (CUIJ_UART_SELECT == uart1)
    #define CUIJ_UART_HANDLE huart1
    #define CUIJ_UART_ENUM    pyro::uart_drv_t::uart1
#elif (CUIJ_UART_SELECT == uart5)
    #define CUIJ_UART_HANDLE huart5
    #define CUIJ_UART_ENUM    pyro::uart_drv_t::uart5
#elif (CUIJ_UART_SELECT == uart7)
    #define CUIJ_UART_HANDLE huart7
    #define CUIJ_UART_ENUM    pyro::uart_drv_t::uart7
#elif (CUIJ_UART_SELECT == uart10)
    #define CUIJ_UART_HANDLE huart10
    #define CUIJ_UART_ENUM    pyro::uart_drv_t::uart10
#else
    #error "Unsupported CUIJ_UART_SELECT. Choose uart1, uart5, uart7, or uart10."
#endif

// 外部引用全局 DataBoard 对象（在 pyro_typedef.h 中定义）
extern pyro::databoard global_databoard;

// 声明对应的 UART 句柄（由 CubeMX 生成）
extern UART_HandleTypeDef CUIJ_UART_HANDLE;

// CUIJ 调试串口实例，用于发送
static pyro::uart_drv_t* cuij_uart = nullptr;

// ==================== 串口收发基础函数 ====================

static void cuij_send(const char* str) {
    if (cuij_uart) {
        cuij_uart->write((uint8_t*)str, strlen(str));
    }
}

// 非阻塞读取一个字符（超时 10ms），返回 -1 表示无数据
static int cuij_getchar() {
    uint8_t ch;
    if (HAL_UART_Receive(&CUIJ_UART_HANDLE, &ch, 1, 10) == HAL_OK) {
        return ch;
    }
    return -1;
}

// ==================== 辅助函数 ====================

static void cuij_format_value(pyro::genenral_data_t* data, char* buf, size_t buf_len) {
    snprintf(buf, buf_len, "float:%.6f  int:%d  uint:%u",
             data->data_f, data->data_si, data->data_ui);
}

// ==================== 命令处理函数 ====================

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

// 监视模式相关
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

// ==================== 命令解析入口 ====================

static void cuij_process_line(char* line) {
    // 去除换行符
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

// ==================== 监视模式发送函数 ====================

static void cuij_send_watch_packet() {
    if (cuij_watch_count == 0 || !cuij_watch_enabled)
        return;

    uint8_t packet[1 + 4*10 + 1];   // 最大 1+40+1
    uint8_t* p = packet;
    *p++ = 0xA5;                    // 帧头

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
    *p++ = 0x5A;                    // 帧尾

    cuij_uart->write(packet, p - packet);
}

// ==================== 调试任务主函数（保持名为 pyro_debug_task） ====================

extern "C" void pyro_debug_task(void* argument) {
    // 获取对应 UART 实例（用于发送）
    cuij_uart = pyro::uart_drv_t::get_instance(CUIJ_UART_ENUM);
    if (!cuij_uart) {
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    char rx_buffer[128];
    uint16_t idx = 0;
    TickType_t last_watch_time = 0;

    cuij_send("CUIJ Debug task started. Type 'CUIJ HELP' for commands.\n");

    while (1) {
        int ch = cuij_getchar();
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

        if (cuij_watch_enabled) {
            TickType_t now = xTaskGetTickCount();
            if (now - last_watch_time >= pdMS_TO_TICKS(100)) {
                cuij_send_watch_packet();
                last_watch_time = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}