#ifndef __PYRO_UPPER_COM_H__
#define __PYRO_UPPER_COM_H__

#include "pyro_databoard.h"

namespace pyro {

/**
 * @brief 初始化上层板与底盘的板间通信（UART 可配置）
 * @param db_ptr 指向 DataBoard 对象的指针
 */
void upper_com_init(databoard *db_ptr);

} // namespace pyro

#endif