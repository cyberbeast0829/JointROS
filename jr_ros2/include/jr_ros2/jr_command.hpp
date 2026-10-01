/**
 * @file    jr_command.hpp
 * @brief   命令信箱的载荷（POD，定长，无分配）—— ROS 域写、RT 域读
 *
 * @par 语义（必须与文档 §6.1 一致）
 *  - **只更新被置位（mask）的关节**；未置位的关节保持上一次目标；
 *  - `seq` 单调递增（调用方维护），`stamp_ns` 是**写入时刻**的单调时间
 *    （用于度量"命令→发帧"延迟，不参与控制判定）；
 *  - `estop` 为真 → 本 tick 广播全总线急停（安全动作，优先于一切目标）；
 *  - 覆盖式：新命令直接替换旧的，**不排队**（积压回放对实时控制有害）。
 */

#ifndef JR_ROS2_JR_COMMAND_HPP
#define JR_ROS2_JR_COMMAND_HPP

#include <cstdint>

#include "jr_ros2/jr_config.hpp"

namespace jr {

/** 单个关节的本 tick 目标。 */
struct JointTarget {
    /**
     * 本条目标用哪种模式 —— **必须与该关节使能时选择的模式一致**。
     *
     * ⚠ 为什么不做“不一致就自动切模式”：切模式等效于 disable→enable，是**阻塞**动作
     *   （ADR-7 安全暂停窗口才能做），不可能也不应该发生在 RT 命令路径上。
     *   不一致的目标会被**丢弃并计数**（`JointStatePOD.cmd_rejected`），不静默照发。
     */
    CmdMode mode = CmdMode::kMit;
    double position = 0.0;   /**< rad（输出端）；MIT / CSP 使用 */
    double velocity = 0.0;   /**< rad/s；MIT / CSV 使用 */
    double kp = 0.0;         /**< MIT 线上值（si_gain=false 时使用） */
    double kd = 0.0;
    double stiffness = 0.0;  /**< N·m/rad（si_gain=true 时使用） */
    double damping = 0.0;    /**< N·m·s/rad */
    double torque = 0.0;     /**< N·m：MIT 前馈 / CST 目标（输出端） */
    double current = 0.0;    /**< A：CURRENT 目标（**电机端**） */
    /** 限制量（0 = 不限制）：CSP/CSV 用速度限制，CSP/CSV/CURRENT 用力矩上限；CST 不适用。
     *  ⚠ v0.22 语义修正（SDK F33）：第二个字段**不是**电流告警门限，而是固件的
     *  `torque_lim`（正常工作**力矩上限**，电机端 N·m）——
     *  固件在 CSP/CSV 帧里把它写成 `cur_limit_a × torque_constant`，一旦发 0
     *  电流环就被钳到 0（电机不出力、但不报错、`is_enabled()` 仍为 1）。 */
    double velocity_limit = 0.0;  /**< 输出端 rad/s */
    double torque_limit = 0.0;    /**< **电机端** N·m（0 = 用 SDK 默认） */
    bool   si_gain = false;  /**< true = 用 stiffness/damping（与 kp/kd 同值；仅 MIT 有意义） */
};

/** 一 tick 的目标集（全局关节索引）。 */
struct CommandSet {
    std::uint64_t seq = 0u;
    std::uint64_t stamp_ns = 0u;

    bool estop = false;          /**< 广播急停（整条总线、所有节点） */
    bool zero_torque_all = false;/**< 全部泄力（安全动作；不改变 mask 语义） */
    bool release_all = false;    /**< 释放"保持上一次目标"状态 → 未置位关节回到 hold */
    std::uint32_t mask = 0u;     /**< bit i = 全局关节 i 有目标（>32 关节时用 blocks） */

    /**
     * 全局关节数超过 32 时用分块掩码（当前 kMaxJoints=128）。
     * `mask` 是 block[0] 的别名，便于前 32 关节的常见场景少写代码。
     */
    std::uint32_t block[(kMaxJoints + 31u) / 32u] = {};

    JointTarget joint[kMaxJoints] = {};

    bool has(unsigned global_index) const noexcept
    {
        return (block[global_index / 32u] & (1u << (global_index % 32u))) != 0u;
    }
    void set_mask(unsigned global_index) noexcept
    {
        block[global_index / 32u] |= (1u << (global_index % 32u));
    }
    void clear_mask(unsigned global_index) noexcept
    {
        block[global_index / 32u] &= ~(1u << (global_index % 32u));
    }
    void clear_all_mask() noexcept
    {
        for (unsigned i = 0u; i < (kMaxJoints + 31u) / 32u; ++i) block[i] = 0u;
    }
};

}  // namespace jr

#endif /* JR_ROS2_JR_COMMAND_HPP */
