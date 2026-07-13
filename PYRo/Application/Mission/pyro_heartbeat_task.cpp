#include "FreeRTOS.h"
#include "task.h"
#include "pyro_uart_drv.h"
#include <cstring>

// 任务入口
extern "C" void pyro_heartbeat_task(void *argument) {
    // 获取 UART10 实例（用于发送）
    pyro::uart_drv_t *uart = pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart10);
    if (!uart) {
        // 若获取失败，死循环等待（或重启）
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    const char *msg = "Heartbeat from UART10\r\n";
    uint8_t *send_buf = (uint8_t *)msg;
    uint16_t len = strlen(msg);

    while (1) {
        uart->write(send_buf, len);
        vTaskDelay(pdMS_TO_TICKS(1000));   // 1 秒间隔
    }
}