/***************************************************
 * 这个任务用来从串口访问整个databoard
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 ***************************************************/
/*
 #include "FreeRTOS.h"
#include "task.h"
#include "pyro_typedef.hpp"      // 包含 global_databoard 声明
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"       // 假设有串口驱动
#include <cstring>
#include <cstdio>
#include <cstdlib>

// 外部全局 DataBoard 指针（在 pyro_typedef.hpp 中定义为 extern）
extern pyro::databoard* global_databoard;

// 假设的调试串口发送函数（根据实际修改）
static void debug_send(const char* str) {
    // 示例：使用 uart1 发送
    pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart1)->send((uint8_t*)str, strlen(str));
}

// 获取 topic 类型名称
static const char* type_name(pyro::data_type_t type) {
    switch(type) {
        case pyro::UNSIGNED_INT: return "UINT32";
        case pyro::SIGNED_INT:   return "INT32";
        case pyro::FLOAT:        return "FLOAT";
        default: return "UNKNOWN";
    }
}

// 列出所有话题
static void cmd_list() {
    char buf[128];
    debug_send("ID  | Name                     | Type    | Valid | Timestamp\n");
    debug_send("----+--------------------------+---------+-------+-----------\n");
    for (uint32_t id = 1; id <= 48; id++) {
        // 直接访问 topic 对象（通过 databoard 的私有成员？不能直接访问）
        // 由于 databoard 不提供直接获取 topic 指针的接口，只能通过 get_topic_id 和 read 来获取信息
        // 这里需要扩展 databoard 接口，或者我们利用已知的命名规则来读取
        // 但为了演示，我们假设有一个 databoard 的公开方法：get_topic_info(id, ...)
        // 实际上，我们可以通过尝试读取来判断是否存在，但无法获取名称和类型。
        // 所以更合理的是修改 databoard 类添加一个遍历接口，或者我们维护一个外部映射表。
        // 这里我们做一个简化：只显示通过 get_topic_id 能找到的名称？
        // 可行方法是：创建一个名称数组，存储每个 ID 对应的名称（在创建时记录）。
        // 因为这是调试，我们可以在全局维护一个名称映射。
        // 或者我们直接使用 databoard 的内部数组（但不安全）。
        // 为了不修改原 databoard，我们换个方式：让调试任务在创建 topic 时也注册名称。
        // 但更简单：我们只支持按名称操作，list 可以输出已注册的 ID 和名称列表。
        // 因为 databoard 目前不提供遍历，我们暂时跳过 list 实现，或者扩展 databoard。
        // 这里我们假设已经扩展了 databoard 接口（如 get_topic_name(id) 和 get_topic_type(id)），
        // 但实际未提供。建议先不实现 list，或用其他方式。
    }
    debug_send("(List not fully implemented without databoard traversal API)\n");
}

// 读取并显示话题值
static void cmd_read(const char* arg) {
    uint32_t id;
    // 尝试作为 ID 解析
    id = (uint32_t)atoi(arg);
    if (id == 0 && arg[0] != '0') { // 不是数字，当作名称
        id = global_databoard->get_topic_id(arg);
        if (id == 0xFFFFFFFF) {
            debug_send("Topic not found.\n");
            return;
        }
    }
    // 读取数据
    pyro::genenral_data_t data;
    TickType_t timestamp;
    auto status = global_databoard->read(id, &data, timestamp);
    if (status == pyro::topic::DATA_INVALID) {
        debug_send("Topic has no valid data.\n");
        return;
    } else if (status != pyro::topic::DATA_OK) {
        debug_send("Read error.\n");
        return;
    }
    // 需要获取类型：因为 databoard 未提供获取类型接口，我们只能通过事先知道或从读取中推断？
    // 实际上 read 不会返回类型，我们只能假设已知类型。
    // 一个解决方法：让 read 返回类型，或者我们维护一个类型映射。
    // 同样，这里我们假设有一个 get_topic_type 方法，但未提供。
    // 为了演示，我们只能根据读取时填写的 union 猜测，但无法准确。
    // 所以我们需要扩展 databoard 或另外存储类型。
    // 这里就省略具体打印，仅演示框架。
    char buf[64];
    snprintf(buf, sizeof(buf), "Value: %u (as uint32) but type unknown\n", data.data_ui);
    debug_send(buf);
}

// 写入话题
static void cmd_write(const char* name_or_id, const char* value_str) {
    uint32_t id;
    id = (uint32_t)atoi(name_or_id);
    if (id == 0 && name_or_id[0] != '0') {
        id = global_databoard->get_topic_id(name_or_id);
        if (id == 0xFFFFFFFF) {
            debug_send("Topic not found.\n");
            return;
        }
    }
    // 需要知道类型才能正确设置联合体，但无法从 databoard 获取类型。
    // 所以我们只能尝试按浮点解析，如果失败则按整数。
    // 更好的方式：创建时记录类型，但这里先略。
    char* endptr;
    float fval = strtof(value_str, &endptr);
    if (*endptr == '\0') {
        // 是浮点数，但可能类型是整数，我们需根据实际类型转换。
        // 这里默认按浮点写入（若类型不匹配会报错？）
        pyro::genenral_data_t data;
        data.data_f = fval;
        auto ret = global_databoard->write_topic(id, data);
        if (ret == pyro::topic::DATA_OK) debug_send("Write OK.\n");
        else debug_send("Write failed (maybe type mismatch).\n");
    } else {
        // 尝试解析为整数
        int32_t ival = (int32_t)strtol(value_str, &endptr, 10);
        if (*endptr == '\0') {
            pyro::genenral_data_t data;
            data.data_si = ival; // 假设是有符号，若是无符号需区分。
            auto ret = global_databoard->write_topic(id, data);
            if (ret == pyro::topic::DATA_OK) debug_send("Write OK.\n");
            else debug_send("Write failed.\n");
        } else {
            debug_send("Invalid number format.\n");
        }
    }
}

// 命令解析入口
static void process_line(char* line) {
    // 移除换行符
    char* newline = strchr(line, '\n');
    if (newline) *newline = '\0';
    newline = strchr(line, '\r');
    if (newline) *newline = '\0';

    // 分割命令
    char* cmd = strtok(line, " ");
    if (!cmd) return;

    if (strcmp(cmd, "help") == 0) {
        debug_send("Commands:\n  list\n  read <name|id>\n  write <name|id> <value>\n");
    } else if (strcmp(cmd, "list") == 0) {
        cmd_list();
    } else if (strcmp(cmd, "read") == 0) {
        char* arg = strtok(nullptr, " ");
        if (arg) cmd_read(arg);
        else debug_send("Missing argument.\n");
    } else if (strcmp(cmd, "write") == 0) {
        char* arg1 = strtok(nullptr, " ");
        char* arg2 = strtok(nullptr, " ");
        if (arg1 && arg2) cmd_write(arg1, arg2);
        else debug_send("Missing arguments.\n");
    } else {
        debug_send("Unknown command. Type 'help'.\n");
    }
}

// 调试任务主函数
extern "C" void pyro_debug_task(void* argument) {
    // 假定 UART 已经初始化，使用 UART1 接收中断或轮询
    // 这里使用简单的轮询读取（示例）
    char rx_buffer[128];
    uint16_t idx = 0;
    while (1) {
        // 假设有一个非阻塞读取字符函数，返回 -1 若无数据
        int ch = debug_uart_getchar(); // 需要实现
        if (ch >= 0) {
            if (ch == '\r' || ch == '\n') {
                rx_buffer[idx] = '\0';
                if (idx > 0) {
                    process_line(rx_buffer);
                }
                idx = 0;
            } else if (idx < sizeof(rx_buffer)-1) {
                rx_buffer[idx++] = (char)ch;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
    */
   //待启用的部分