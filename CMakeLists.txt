cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# 1. 编译器检测配置 (必须放在 project 命令之前)
# ==============================================================================
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

include(cmake/gcc-arm-none-eabi.cmake)

# ==============================================================================
# 2. 项目定义
# ==============================================================================
set(CMAKE_PROJECT_NAME PYRo)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# ==============================================================================
# 3. 硬件架构与编译优化 (STM32H723)
# ==============================================================================
set(CPU_FLAGS
    "-mcpu=cortex-m7"
    "-mthumb"
    "-mfpu=fpv5-d16"
    "-mfloat-abi=hard"
)

# 使用 add_compile_options 传递 CPU_FLAGS（列表会自动展开为多个 -m... 选项）
add_compile_options(
    ${CPU_FLAGS}
    -Ofast
    -Wdouble-promotion
    -funsafe-math-optimizations
    -Wall
    -fdata-sections
    -ffunction-sections
)

# 链接选项同样使用 add_link_options，并额外添加其他选项
add_link_options(
    ${CPU_FLAGS}
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--print-memory-usage
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})
# ==============================================================================
# 4. 源文件配置
# ==============================================================================
add_executable(${CMAKE_PROJECT_NAME}
    PYRo/Core/Memory/pyro_core_mem.cpp
    PYRo/Core/Memory/pyro_core_dma_heap.c
    PYRo/Core/Lock/pyro_rw_lock.cpp
    PYRo/Core/ETL/map.cpp
    PYRo/Core/Lock/pyro_rw_lock.cpp
    PYRo/Core/DataBoard/pyro_databoard.cpp

    PYRo/Peripheral/CAN/pyro_can_drv.cpp
    PYRo/Peripheral/UART/pyro_uart_drv.cpp
    PYRo/Peripheral/DWT/pyro_dwt_drv.cpp

    PYRo/Algorithm/OLS/pyro_algo_ols.cpp
    PYRo/Algorithm/PID/pyro_algo_pid.cpp

    PYRo/Component/RC/pyro_rc_base_drv.cpp
    PYRo/Component/RC/pyro_vt03_rc_drv.cpp
    PYRo/Component/RC/pyro_dr16_rc_drv.cpp
    PYRo/Component/RC/pyro_rc_hub.cpp

    PYRo/Component/Motor/pyro_dji_motor_drv.cpp
    PYRo/Component/Motor/pyro_dm_motor_drv.cpp
    PYRo/Component/Motor/pyro_motor_base.cpp

    PYRo/Component/CRC/PYRo_crc.cpp

    PYRo/Component/IMU/AHRS.c
    PYRo/Component/IMU/BMI088driver.c
    PYRo/Component/IMU/BMI088Middleware.c
    PYRo/Component/IMU/IMU_Base.c
    PYRo/Component/IMU/IMU_Ext.c
    PYRo/Component/IMU/MATH_LIB.c
    PYRo/Component/IMU/PID.c

    PYRo/Component/Shoot/pyro_fric_drv.cpp
    PYRo/Component/Shoot/pyro_trigger_drv.cpp
    PYRo/Component/Shoot/pyro_shoot_base.cpp
    PYRo/Component/Shoot/pyro_shoot_17mm_control.cpp

    PYRo/Debug/Debug_task.cpp
    PYRo/Debug/VOFA/pyro_vofa.cpp
    PYRo/Debug/JCOM/pyro_jcom.cpp

    PYRo/Application/Mission/pyro_mission_planer.cpp
    PYRo/Application/Mission/pyro_init_thread.cpp
    PYRo/Application/Mission/pyro_receive_thread.cpp
    PYRo/Application/Mission/pyro_processing_thread.cpp
    PYRo/Application/Mission/pyro_debug_thread.cpp
    PYRo/Application/Mission/pyro_heartbeat_task.cpp

    PYRo/Component/Referee/CRC8_CRC16.c
    PYRo/Component/Referee/fifo.c
    PYRo/Component/Referee/referee.c
    PYRo/Component/Referee/referee_drv.cpp
    PYRo/Component/Referee/referee_usart_task.c

    PYRo/Component/Powercontrol/pyro_power_control_drv.cpp
    PYRo/Component/Powermeter/pyro_powermeter.cpp

    PYRo/Moudle/Chassis/pyro_chassis_base.cpp
    PYRo/Algorithm/Kinematics/pyro_kin_mec.cpp
    PYRo/Moudle/Chassis/Mecanum/pyro_mec_chassis.cpp
    PYRo/Core/Lock/pyro_mutex.cpp
    PYRo/Algorithm/Kinematics/pyro_kin_rudder.cpp
    PYRo/Moudle/Chassis/Rudder/pyro_rud_chassis.cpp
    PYRo/Algorithm/Kinematics/pyro_kin_hybrid.cpp
    PYRo/Moudle/Chassis/Hybrid/pyro_hybrid_chassis.cpp
)

# ==============================================================================
# 5. STM32CubeMX 生成的源码 (必须在 add_executable 之后)
# ==============================================================================
add_subdirectory(cmake/stm32cubemx)

# ==============================================================================
# 6. 包含路径与链接
# ==============================================================================
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    # 用户自定义包含路径
    PYRo/Core/Def
    PYRo/Core/Memory
    PYRo/Core/Config
    PYRo/Core/ETL
    PYRo/Core/Lock
    PYRo/Core/FSM
    PYRo/Core/DataBoard

    PYRo/Peripheral/UART
    PYRo/Peripheral/CAN
    PYRo/Peripheral/DWT

    PYRo/Algorithm/OLS
    PYRo/Algorithm/PID
    PYRo/Algorithm/Kinematics

    PYRo/Component/RC
    PYRo/Component/Motor
    PYRo/Component/CRC
    PYRo/Component/Shoot
    PYRo/Component/IMU
    PYRo/Component/Referee
    PYRo/Component/Powermeter
    PYRo/Component/Powercontrol

    PYRo/Debug/VOFA

    PYRo/Moudle/Chassis
    PYRo/Moudle/Chassis/Mecanum
    PYRo/Moudle/Chassis/Rudder
    PYRo/Moudle/Chassis/Hybrid

    PYRo/Application/Mission
)

# 项目宏定义
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 可添加全局宏，如 STM32H723xx 等
)

# 移除错误的库依赖
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 链接库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)