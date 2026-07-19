/**
 * @file action_register.cpp
 * @brief 动作注册表实现
 */

#include "action_register.h"
#include <cstring>

namespace motion {

// 静态存储：动作表
static const Action* g_action_table[MAX_ACTIONS] = {nullptr};

// 静态存储：空动作对象（生命周期贯穿整个程序）
static Action g_empty_action;

// 静态存储：动作1对象
static Action g_action1;

// -------------------- 注册与查询 --------------------
bool registerAction(uint8_t id, const Action* action) {
    if (id >= MAX_ACTIONS) return false;
    g_action_table[id] = action;
    return true;
}

const Action* getAction(uint8_t id) {
    if (id >= MAX_ACTIONS) return nullptr;
    return g_action_table[id];
}

// -------------------- 初始化 --------------------
void initActionRegistry() {
    // 清空所有条目
    for (uint8_t i = 0; i < MAX_ACTIONS; ++i) {
        g_action_table[i] = nullptr;
    }

    // ---------- 注册编号0：空动作（极短时间，无任何位移） ----------
    Pose empty_poses[2];
    empty_poses[0].time = 0.0;
    empty_poses[0].gripper = 0.0;
    empty_poses[0].tolerance = -1.0;
    empty_poses[1].time = 0.001;
    empty_poses[1].gripper = 0.0;
    empty_poses[1].tolerance = -1.0;

    for (int j = 0; j < NUM_JOINTS; ++j) {
        empty_poses[0].joints[j] = 0.0;
        empty_poses[0].vel[j] = 0.0;
        empty_poses[0].acc[j] = 0.0;
        empty_poses[1].joints[j] = 0.0;
        empty_poses[1].vel[j] = 0.0;
        empty_poses[1].acc[j] = 0.0;
    }

    if (g_empty_action.registerFromPoses(empty_poses, 2)) {
        registerAction(0, &g_empty_action);
    }

    // ---------- 注册编号1：简单往返动作（无校验） ----------
    // 定义 5 个关键帧：时间 0, 1, 2, 3, 4 秒，所有关节同步运动
    Pose action1_poses[5];
    const double times[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
    // 关节位置序列：0 → 1 → 0 → -1 → 0
    const double positions[5][NUM_JOINTS] = {
        {0, 0, 0, 0, 0, 0},
        {1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0},
        {-1, -1, -1, -1, -1, -1},
        {0, 0, 0, 0, 0, 0}
    };

    for (int i = 0; i < 5; ++i) {
        action1_poses[i].time = times[i];
        action1_poses[i].gripper = 0.0;          // 夹爪保持 0
        action1_poses[i].tolerance = -1.0;       // 不校验
        for (int j = 0; j < NUM_JOINTS; ++j) {
            action1_poses[i].joints[j] = positions[i][j];
            action1_poses[i].vel[j] = 0.0;
            action1_poses[i].acc[j] = 0.0;
        }
    }

    if (g_action1.registerFromPoses(action1_poses, 5)) {
        registerAction(1, &g_action1);
    }

    // ---------- 其它编号暂不注册（留空） ----------
    // 后续可在外部调用 registerAction() 添加更多动作
}

} // namespace motion