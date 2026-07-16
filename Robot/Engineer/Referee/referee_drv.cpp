#include "cmsis_os.h"
#include "pyro_uart_drv.h"
#include "pyro_bsp_uart.h"          // 新增：BSP 头文件

#include <cstdio>
#include <cstring>

extern "C" void referee_usart_task(void *argument);
extern "C" void referee_rx_handler(uint8_t *buf, uint16_t Size);

// 【修改】回调签名：xHigherPriorityTaskWoken 改为引用类型
bool referee_uart_callback(uint8_t *data, uint16_t size,
                           BaseType_t& xHigherPriorityTaskWoken)
{

    referee_rx_handler(data, size);
    return true;
}

extern "C" void referee_init()
{
    // 【修改】通过 BSP 获取 UART1 实例的指针，并注册回调
    pyro::bsp_uart::get_uart1().add_rx_event_callback(referee_uart_callback, 0x20);
}

extern "C" void referee_task(void *arg)
{
    referee_usart_task(nullptr);
}