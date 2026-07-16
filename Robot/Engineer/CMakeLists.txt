# =========================================================================
# 1. 显式定义当前编译的板子类型（在这里切换 ARM_BOARD 或 CHASSIS_BOARD）
# =========================================================================
set(BOARD "ARM_BOARD")
#set(BOARD "CHASSIS_BOARD")

# =========================================================================
# 2. 声明 Engineer 目标文件库（OBJECT库，仅编译目标文件不链接）
# =========================================================================
add_library(Engineer OBJECT)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Engineer PRIVATE
        BOARD=${BOARD}
)

# ===================== 公共源文件（两块板子共用的模块） =====================
set(COMMON_SOURCES
        DataBoard/pyro_databoard.cpp
        RC/pyro_rc_hub.cpp
        CRC/pyro_crc.cpp
        # Referee 模块所有源文件
        Referee/CRC8_CRC16.c
        Referee/referee.c
        Referee/referee_drv.cpp
        Referee/referee_usart_task.c
        Powercontrol/pyro_power_control_drv.cpp
)

# ===================== ARM_BOARD 专用源文件 =====================
set(ARM_SOURCES
        Arm/arm_pose_def.cpp
        Arm/engineer_arm_mission.cpp
        Arm/engineer_arm_planning_mission.cpp
        Arm/interboard_communication_mission.cpp
        Arm/pyro_init_thread.cpp
        Arm/pyro_mission_planer.cpp
        Arm/self_control_mission.cpp
        Arm/VOFA_Thread.cpp
        Arm/arm_planner_component/arm_fixed_motion_hub.cpp
        Arm/arm_planner_component/arm_rc_command.cpp
        Arm/arm_planner_component/arm_self_control_command.cpp
        Arm/arm_planner_component/arm_transition_component.cpp
)

# ===================== CHASSIS_BOARD 专用源文件 =====================
set(CHASSIS_SOURCES
        Chassis/engineer_chassis_mission.cpp
        Chassis/interboard_communication_mission.cpp
        Chassis/Magazine_mission.cpp
        Chassis/pyro_init_thread.cpp
        Chassis/pyro_mission_planer.cpp
        Chassis/referee_system_mission.cpp
        Chassis/VOFA_Thread.cpp
        Chassis/Component/joint_ctrl.cpp
        Chassis/UI/UI.cpp
        Chassis/UI/UI_group.cpp
)

# ===================== 组合最终源文件列表 =====================
if (BOARD STREQUAL "ARM_BOARD")
    target_sources(Engineer PRIVATE ${COMMON_SOURCES} ${ARM_SOURCES})
    # 针对 ARM_BOARD 的特殊宏定义（作用于 PYRo 目标）
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
            VT03_UART=PYRO_UART1
            AUTOAIM_UART=PYRO_UART7
    )
elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Engineer PRIVATE ${COMMON_SOURCES} ${CHASSIS_SOURCES})
    # 针对 CHASSIS_BOARD 的特殊宏定义（作用于 PYRo 目标）
    target_compile_definitions(PYRo PUBLIC
            REFEREE_UART=PYRO_UART1
            SR05_UART=PYRO_UART5
            SR04_UART=PYRO_UART10
            SUPERCAP_UART=PYRO_UART7
    )
else()
    message(FATAL_ERROR "Unsupported BOARD type: ${BOARD}. Choose ARM_BOARD or CHASSIS_BOARD.")
endif()

# ===================== 头文件包含路径（包含所有可能的子目录） =====================
target_include_directories(Engineer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Arm
        ${CMAKE_CURRENT_SOURCE_DIR}/Arm/arm_planner_component
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis/Component
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis/UI
        ${CMAKE_CURRENT_SOURCE_DIR}/DataBoard
        ${CMAKE_CURRENT_SOURCE_DIR}/RC
        ${CMAKE_CURRENT_SOURCE_DIR}/CRC
        ${CMAKE_CURRENT_SOURCE_DIR}/Referee
        ${CMAKE_CURRENT_SOURCE_DIR}/Powercontrol
)

# ===================== 链接依赖库 =====================
target_link_libraries(Engineer
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)