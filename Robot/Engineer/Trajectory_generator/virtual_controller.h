/**
 * @file virtual_controller.h
 * @brief 虚拟控制器：轨迹执行器，管理时间推进与校验同步
 * @note 无动态内存分配，所有状态静态存储
 */

#ifndef VIRTUAL_CONTROLLER_H
#define VIRTUAL_CONTROLLER_H

#include "motion_planner.h"
#include <cstdint>

namespace motion {

/**
 * @brief 虚拟控制器状态枚举
 */
enum class ControllerStatus : uint8_t {
    IDLE = 0,       // 空闲，无动作加载
    READY,          // 已加载动作，等待启动
    RUNNING,        // 正在运行
    PAUSED,         // 已暂停
    FINISHED,       // 正常完成
    ERROR           // 错误（校验超时等）
};

/**
 * @brief 校验点信息（内部使用）
 */
struct Checkpoint {
    double time;        // 校验点时刻
    double tolerance;   // 允许误差（来自 Pose.tolerance）
};

/**
 * @brief 虚拟控制器类
 * @note 不拥有 Action 对象，只持有指针；调用者需保证 Action 生命周期长于控制器
 */
class VirtualController {
public:
    VirtualController();

    // -------------------- 生命周期管理 --------------------
    /**
     * @brief 加载动作
     * @param action 已注册的 Action 对象指针（不可为空）
     * @return true 加载成功
     */
    bool loadAction(const Action* action);

    /**
     * @brief 启动执行（从时间 0 开始）
     * @return true 启动成功（状态必须为 READY）
     */
    bool start();

    /**
     * @brief 暂停执行
     * @return true 暂停成功
     */
    bool pause();

    /**
     * @brief 恢复执行
     * @return true 恢复成功
     */
    bool resume();

    /**
     * @brief 停止并重置到 IDLE 状态
     */
    void stop();

    /**
     * @brief 完全重置（清空 Action 引用）
     */
    void reset();

    // -------------------- 实时更新接口 --------------------
    /**
     * @brief 推进时间增量
     * @param dt 时间增量（秒），必须 >= 0
     * @return 当前状态
     * @note 仅在 RUNNING 状态下有效推进；在 PAUSED 状态下调用无效
     */
    ControllerStatus update(double dt);

    /**
     * @brief 获取当前时刻的目标位姿
     * @param joints_out 输出 6 个关节角（数组长度至少为 NUM_JOINTS）
     * @param gripper_out 输出夹爪值
     * @return true 成功获取（如果无动作加载则返回 false）
     */
    bool getCurrentTarget(double* joints_out, double* gripper_out) const;

    // -------------------- 校验接口 --------------------
    /**
     * @brief 设置当前校验结果
     * @param verified true 表示校验通过
     * @return true 设置成功（当前确实在等待校验）
     * @note 若 verified = true，时间继续推进；
     *       若 verified = false，时间回滚到校验点并继续等待
     */
    bool setVerificationResult(bool verified);

    /**
     * @brief 是否正在等待校验
     */
    bool isWaitingForVerification() const;

    /**
     * @brief 设置校验超时时间（秒）
     * @param timeout 超时阈值，> 0 有效；默认 1.0 秒
     */
    void setVerificationTimeout(double timeout);

    /**
     * @brief 获取当前校验超时时间
     */
    double getVerificationTimeout() const;

    // -------------------- 状态查询 --------------------
    ControllerStatus getStatus() const { return status_; }
    double getCurrentTime() const { return current_time_; }
    double getTotalDuration() const;
    bool isFinished() const { return status_ == ControllerStatus::FINISHED; }
    bool isError() const { return status_ == ControllerStatus::ERROR; }
    bool hasAction() const { return action_ != nullptr; }

private:
    // -------------------- 内部辅助函数 --------------------
    /** @brief 构建校验点列表（从 Action 中提取 tolerance >= 0 的 Pose） */
    void buildCheckpoints();

    /** @brief 查找当前时间是否落在校验点上 */
    bool isAtCheckpoint(double time) const;

    /** @brief 获取当前校验点的容差（若无则返回 -1） */
    double getCurrentCheckpointTolerance() const;

    /** @brief 重置内部状态（保留 Action 指针） */
    void resetInternal();

    // -------------------- 数据成员 --------------------
    const Action* action_;                  // 当前动作（只读指针）
    ControllerStatus status_;               // 当前状态
    double current_time_;                   // 当前轨迹时刻（秒）
    double verification_timeout_;           // 校验超时阈值（秒）

    // 校验相关
    bool waiting_for_verification_;         // 是否正在等待校验
    double checkpoint_time_;                // 当前卡住的校验点时刻
    double checkpoint_tolerance_;           // 当前校验点的容差
    double time_at_checkpoint_start_;       // 进入等待校验的时刻（用于超时检测）
    uint32_t verification_fail_count_;      // 连续失败计数（防抖动）

    // 校验点列表（从 Action 中提取）
    static constexpr size_t MAX_CHECKPOINTS = 100;
    Checkpoint checkpoints_[MAX_CHECKPOINTS];
    size_t num_checkpoints_;
};

} // namespace motion

#endif // VIRTUAL_CONTROLLER_H