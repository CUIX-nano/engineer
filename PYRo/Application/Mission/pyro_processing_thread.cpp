/***************************************************************************************
 *  处理任务：从 DataBoard 读取控制信号（arm_ctrl_*），根据模式处理后写入执行指令（arm_command_*）
 *  当前支持：零力模式（IDLE）、直接关节映射（JOINT_DIRECT）
 *  频率：50Hz
 **************************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "pyro_typedef.h"          // 包含 global_databoard 声明
#include "pyro_databoard.h"
#include <cstring>

// 限幅参数（临时值，待实测后修改）
#define JOINT_MIN_LIMIT  -3.14159f   // -180°
#define JOINT_MAX_LIMIT   3.14159f   // +180°
#define GRIPPER_MIN_LIMIT 0.0f
#define GRIPPER_MAX_LIMIT 1.0f

// 零力标志值（超出机械限位，执行层识别后进入零力状态）
// 修改为 80.0f（更安全）
#define ZERO_FORCE_FLAG   80.0f

// 话题名称常量（避免拼写错误）
#define TOPIC_MODE          "arm_ctrl_mode"
#define TOPIC_CTRL_JOINT0   "arm_ctrl_joint0"
#define TOPIC_CTRL_JOINT1   "arm_ctrl_joint1"
#define TOPIC_CTRL_JOINT2   "arm_ctrl_joint2"
#define TOPIC_CTRL_JOINT3   "arm_ctrl_joint3"
#define TOPIC_CTRL_JOINT4   "arm_ctrl_joint4"
#define TOPIC_CTRL_JOINT5   "arm_ctrl_joint5"
#define TOPIC_CTRL_GRIPPER  "arm_ctrl_gripper"

#define TOPIC_CMD_JOINT0    "arm_command_joint0"
#define TOPIC_CMD_JOINT1    "arm_command_joint1"
#define TOPIC_CMD_JOINT2    "arm_command_joint2"
#define TOPIC_CMD_JOINT3    "arm_command_joint3"
#define TOPIC_CMD_JOINT4    "arm_command_joint4"
#define TOPIC_CMD_JOINT5    "arm_command_joint5"
#define TOPIC_CMD_GRIPPER   "arm_command_gripper"

// 外部全局 DataBoard 对象
extern pyro::databoard global_databoard;

// 本地缓存的话题 ID（避免每次循环都查名称）
static uint32_t ctrl_mode_id = 0;
static uint32_t ctrl_joint_id[6] = {0};
static uint32_t ctrl_gripper_id = 0;
static uint32_t cmd_joint_id[6] = {0};
static uint32_t cmd_gripper_id = 0;

// 辅助函数：限幅
static inline float clamp(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 初始化：获取所有话题 ID（只执行一次）
static void processing_init() {
    ctrl_mode_id = global_databoard.get_topic_id(TOPIC_MODE);
    ctrl_joint_id[0] = global_databoard.get_topic_id(TOPIC_CTRL_JOINT0);
    ctrl_joint_id[1] = global_databoard.get_topic_id(TOPIC_CTRL_JOINT1);
    ctrl_joint_id[2] = global_databoard.get_topic_id(TOPIC_CTRL_JOINT2);
    ctrl_joint_id[3] = global_databoard.get_topic_id(TOPIC_CTRL_JOINT3);
    ctrl_joint_id[4] = global_databoard.get_topic_id(TOPIC_CTRL_JOINT4);
    ctrl_joint_id[5] = global_databoard.get_topic_id(TOPIC_CTRL_JOINT5);
    ctrl_gripper_id = global_databoard.get_topic_id(TOPIC_CTRL_GRIPPER);

    cmd_joint_id[0] = global_databoard.get_topic_id(TOPIC_CMD_JOINT0);
    cmd_joint_id[1] = global_databoard.get_topic_id(TOPIC_CMD_JOINT1);
    cmd_joint_id[2] = global_databoard.get_topic_id(TOPIC_CMD_JOINT2);
    cmd_joint_id[3] = global_databoard.get_topic_id(TOPIC_CMD_JOINT3);
    cmd_joint_id[4] = global_databoard.get_topic_id(TOPIC_CMD_JOINT4);
    cmd_joint_id[5] = global_databoard.get_topic_id(TOPIC_CMD_JOINT5);
    cmd_gripper_id = global_databoard.get_topic_id(TOPIC_CMD_GRIPPER);
}

extern "C" void pyro_processing_thread(void *argument) {
    // 等待 DataBoard 完全初始化（话题已创建）
    while (1) {
        processing_init();
        // 检查是否所有 ID 都有效（非 0xFFFFFFFF）
        if (ctrl_mode_id != 0xFFFFFFFF &&
            ctrl_joint_id[0] != 0xFFFFFFFF && ctrl_joint_id[1] != 0xFFFFFFFF &&
            ctrl_joint_id[2] != 0xFFFFFFFF && ctrl_joint_id[3] != 0xFFFFFFFF &&
            ctrl_joint_id[4] != 0xFFFFFFFF && ctrl_joint_id[5] != 0xFFFFFFFF &&
            ctrl_gripper_id != 0xFFFFFFFF &&
            cmd_joint_id[0] != 0xFFFFFFFF && cmd_joint_id[1] != 0xFFFFFFFF &&
            cmd_joint_id[2] != 0xFFFFFFFF && cmd_joint_id[3] != 0xFFFFFFFF &&
            cmd_joint_id[4] != 0xFFFFFFFF && cmd_joint_id[5] != 0xFFFFFFFF &&
            cmd_gripper_id != 0xFFFFFFFF) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 本地缓存变量
    uint32_t mode;
    float joint_in[6], gripper_in;
    float joint_out[6], gripper_out;
    TickType_t timestamp;

    while (1) {
        // 1. 读取模式（UNSIGNED_INT）
        pyro::genenral_data_t data;
        auto status = global_databoard.read(ctrl_mode_id, &data, timestamp);
        if (status == pyro::topic::DATA_OK) {
            mode = data.data_ui;
        } else {
            // 读取失败，保持上一帧指令不变
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // 2. 读取所有输入关节和夹爪
        // 【修改1】读取失败时置为 ZERO_FORCE_FLAG（下力）
        for (int i = 0; i < 6; i++) {
            status = global_databoard.read(ctrl_joint_id[i], &data, timestamp);
            if (status == pyro::topic::DATA_OK) {
                joint_in[i] = data.data_f;
            } else {
                // 通信中断 → 下力（零力标志）
                joint_in[i] = ZERO_FORCE_FLAG;
            }
        }
        status = global_databoard.read(ctrl_gripper_id, &data, timestamp);
        if (status == pyro::topic::DATA_OK) {
            gripper_in = data.data_f;
        } else {
            // 通信中断 → 下力
            gripper_in = ZERO_FORCE_FLAG;
        }

        // 3. 根据模式生成输出
        switch (mode) {
            case PYRO_ARM_MODE_IDLE:
                // 零力模式：全部设为特殊标志值 80.0f
                for (int i = 0; i < 6; i++) {
                    joint_out[i] = ZERO_FORCE_FLAG;
                }
                gripper_out = ZERO_FORCE_FLAG;
                break;

            case PYRO_ARM_MODE_JOINT_DIRECT:
                // 【修改2】直接映射前先检查是否为零力标志，若是则直接透传，否则限幅
                for (int i = 0; i < 6; i++) {
                    if (joint_in[i] == ZERO_FORCE_FLAG) {
                        joint_out[i] = ZERO_FORCE_FLAG;
                    } else {
                        joint_out[i] = clamp(joint_in[i], JOINT_MIN_LIMIT, JOINT_MAX_LIMIT);
                    }
                }
                if (gripper_in == ZERO_FORCE_FLAG) {
                    gripper_out = ZERO_FORCE_FLAG;
                } else {
                    gripper_out = clamp(gripper_in, GRIPPER_MIN_LIMIT, GRIPPER_MAX_LIMIT);
                }
                break;

            case PYRO_ARM_MODE_CARTESIAN:
            case PYRO_ARM_MODE_AUTO_SEQUENCE:
                // 暂未实现，保持上一帧命令（不更新）
                goto skip_write;
                break;

            default:
                // 未知模式：安全起见置零力标志
                for (int i = 0; i < 6; i++) {
                    joint_out[i] = ZERO_FORCE_FLAG;
                }
                gripper_out = ZERO_FORCE_FLAG;
                break;
        }

        // 4. 写入执行指令
        for (int i = 0; i < 6; i++) {
            pyro::genenral_data_t out_data;
            out_data.data_f = joint_out[i];
            global_databoard.write_topic(cmd_joint_id[i], out_data);
        }
        {
            pyro::genenral_data_t out_data;
            out_data.data_f = gripper_out;
            global_databoard.write_topic(cmd_gripper_id, out_data);
        }

    skip_write:
        // 5. 循环延时，控制频率 50Hz
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}