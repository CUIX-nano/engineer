/***************************************************************************************
 *  这个线程会用来处理接收到的控制信号，指挥可能用到的控制器，把数据最终处理成电机的目标角度发给执行层
 *  
 * 
 * **************************************************************************************/
#include "FreeRTOS.h"      // 必须最先
#include <cstring>
#include "pyro_typedef.h"
#include "task.h"
#include "pyro_databoard.h"
PYRO_ArmControlSignal_t PYRO_ArmControlSignal;   // 规范信号的保存位置（输入）
PYRO_ArmJointCommand_t  PYRO_ArmJointCommand;    // 最终发给执行层的目标角度（输出）

extern "C"
{
    //线程初始化
    void pyro_processing_init()
    {
        std::memset(&PYRO_ArmControlSignal, 0, sizeof(PYRO_ArmControlSignal));
        std::memset(&PYRO_ArmJointCommand,  0, sizeof(PYRO_ArmJointCommand));  // 新增清零
    }

    //更新保存的机械臂控制信号（这里仅示意，实际应由接收任务写入）
    char pyro_processing_updataArmControlSignal()
    {
        // 实际使用时，PYRO_ArmControlSignal 应由其他任务（如遥控器接收任务）更新
        // 此处留空，保持原有逻辑，返回成功
        return 0;
    };

    void pyro_processing_thread(void *argument)
    {
        while(1)
        {
            pyro_processing_updataArmControlSignal();

            // 根据模式生成执行层目标
            switch (PYRO_ArmControlSignal.mode)
            {
                case PYRO_ARM_MODE_IDLE:
                    // 零力模式：可保持当前目标或置零（这里保持上一帧，即不做修改）
                    break;

                case PYRO_ARM_MODE_JOINT_DIRECT:
                    // 直接关节映射：从输入拷贝到输出
                    for (int i = 0; i < PYRO_ARM_JOINT_NUM; i++)
                        PYRO_ArmJointCommand.joint[i] = PYRO_ArmControlSignal.joint_direct.joint[i];
                        PYRO_ArmJointCommand.gripper = PYRO_ArmControlSignal.joint_direct.gripper;
                    break;

                case PYRO_ARM_MODE_CARTESIAN:
                    // 笛卡尔模式：待逆解算实现，暂时保持目标不变
                    break;

                case PYRO_ARM_MODE_AUTO_SEQUENCE:
                    // 一键/自动序列：待实现
                    break;

                default:
                    // 未知模式，可置零以防意外
                    std::memset(&PYRO_ArmJointCommand, 0, sizeof(PYRO_ArmJointCommand));
                    break;
            }

            // 循环延时，控制帧率（例如 50Hz）
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}