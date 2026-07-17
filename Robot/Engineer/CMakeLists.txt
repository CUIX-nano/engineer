# =========================================================================
# 1. 显式定义当前编译的板子类型（在这里切换 GIMBAL_BOARD 或 CHASSIS_BOARD）
# =========================================================================
set(BOARD "GIMBAL_BOARD")
#set(BOARD "CHASSIS_BOARD")


# =========================================================================
# 2. 声明 Engineer 目标文件库（OBJECT库，仅编译目标文件不链接）
# =========================================================================
add_library(Engineer OBJECT)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Engineer PRIVATE
        BOARD=${BOARD}
        GIMBAL_BOARD=1
        CHASSIS_BOARD=2
)

# ===================== 所有源文件（严格匹配文件名/大小写/路径） =====================
# 定义 Engineer 专用的源文件列表（所有 Arm 控制代码）
set(ENGINEER_SOURCES
        Arm/pyro_mission_planer.cpp
        Arm/pyro_init_thread.cpp
        Arm/arm_control_execute.cpp
        Arm/pyro_debug_thread.cpp
        Arm/pyro_processing_thread.cpp
        Arm/pyro_receive_thread.cpp
        Arm/arm_control_execute.cpp
        Arm/pyro_upper_com.cpp
        Arm/self_control_command.cpp
        DataBoard/pyro_databoard.cpp
)

if (BOARD STREQUAL "GIMBAL_BOARD")
    target_sources(Engineer PRIVATE ${ENGINEER_SOURCES})

    # 针对 Gimbal 的特殊宏定义（作用于 PYRo 目标）
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
            VT03_UART=PYRO_UART1
            AUTOAIM_UART=PYRO_UART7
    )

elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Engineer PRIVATE ${ENGINEER_SOURCES})

    # 针对 Chassis 的特殊宏定义（作用于 PYRo 目标）
    target_compile_definitions(PYRo PUBLIC
            REFEREE_UART=PYRO_UART1
            SR05_UART=PYRO_UART5
            SR04_UART=PYRO_UART10
            SUPERCAP_UART=PYRO_UART7
    )

endif ()

# ===================== 头文件包含路径 =====================
target_include_directories(Engineer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Arm
        ${CMAKE_CURRENT_SOURCE_DIR}/DataBoard
)

# ===================== 链接依赖库 =====================
target_link_libraries(Engineer
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)