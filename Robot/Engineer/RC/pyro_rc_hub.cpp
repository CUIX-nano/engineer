/**
 * @file pyro_rc_hub.cpp
 * @brief Implementation file for the PYRO RC Hub.
 *
 * Implements the `get_instance` factory method which constructs
 * and returns the specific RC driver singletons (DR16, VT03).
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-11-14
 * @copyright [Copyright Information Here]
 */

#include "pyro_rc_hub.h"
#include "pyro_bsp_uart.h"

namespace pyro
{
rc_drv_t *rc_hub_t::get_instance(which_rc_t which_rc)
{
    switch (which_rc)
    {
        case DR16:
        {
#ifdef DR16_UART
            // 直接返回 DR16 单例的地址
            return &dr16_drv_t::instance();
#else
            return nullptr;
#endif
        }
        case VT03:
        {
#ifdef VT03_UART
            // 直接返回 VT03 单例的地址
            return &vt03_drv_t::instance();
#else
            return nullptr;
#endif
        }
        default:;
    }
    return nullptr;
}
}