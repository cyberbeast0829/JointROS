/**
 * @file    jr_rt_hook.hpp
 * @brief   客户自己的控制器插件接口（进程内、零 DDS）—— DESIGN §7.4
 *
 * @par 适用人群
 *  人形/外骨骼客户把 WBC/RL 策略跑在 **我们的 tick 线程里**，端到端不经过 DDS：
 *  他们的 `step()` 与 `cycle_end()` 在同一个线程、同一个 tick 内完成，
 *  命令→发帧延迟 ≈ 0（不是"≤1 tick"）。
 *
 * @par 约束（写进头文件就是契约）
 *  - `step()` 在 RT 上下文调用：**禁止**分配、加锁、I/O、抛异常、调用任何 ROS API；
 *  - `step()` 必须**有界**：耗时会单独计入 `RtStats.hook_last_ns`，
 *    超预算会置 `feedback_stale` 级别的告警（不能让"我的算法慢"表现成"CAN 慢"）；
 *  - `init()/shutdown()` 在非 RT 路径调用，允许分配与阻塞。
 */

#ifndef JR_ROS2_RT_JR_RT_HOOK_HPP
#define JR_ROS2_RT_JR_RT_HOOK_HPP

#include <cstddef>
#include <cstdint>

#include "jr_ros2/jr_command.hpp"
#include "jr_ros2/jr_snapshot.hpp"

namespace jr {
namespace rt {

/**
 * 命令写入口（RT 内联，无分配）。
 *
 * 语义与 `jr_interfaces/msg/MitCommand` 一致：
 *  - `mit()`      → `kp/kd` 按**线上值**原样下发（SDK `jsdk_joint_set_mit`）；
 *  - `mit_si()`   → 按**输出端真实刚度/阻尼**下发（SDK `jsdk_joint_set_mit_stiffness`）；
 *  - `estop()`    → 本 tick 广播全总线急停（不返回、不需要确认）。
 *
 * ⚠ 全局关节索引由核心库分配（总线顺序 × 总线内顺序），启动后不变；
 *   可用 `jr::find_joint()` / 快照里的 `name` 建立自己的映射。
 */
class CommandWriter {
public:
    CommandWriter() noexcept = default;
    CommandWriter(CommandSet *set, std::uint64_t stamp_ns) noexcept
        : set_(set), stamp_ns_(stamp_ns) {}

    void mit(unsigned idx, double pos, double vel, double kp, double kd, double tau) noexcept
    {
        if (!valid(idx)) return;
        JointTarget &t = set_->joint[idx];
        t.position = pos;
        t.velocity = vel;
        t.kp = kp;
        t.kd = kd;
        t.torque = tau;
        t.si_gain = false;
        set_->set_mask(idx);
        set_->stamp_ns = stamp_ns_;
    }

    void mit_si(unsigned idx, double pos, double vel, double stiffness, double damping,
                double tau) noexcept
    {
        if (!valid(idx)) return;
        JointTarget &t = set_->joint[idx];
        t.position = pos;
        t.velocity = vel;
        t.stiffness = stiffness;
        t.damping = damping;
        t.torque = tau;
        t.si_gain = true;
        set_->set_mask(idx);
        set_->stamp_ns = stamp_ns_;
    }

    void zero_torque_all() noexcept
    {
        if (set_ != nullptr) set_->zero_torque_all = true;
    }

    void estop() noexcept
    {
        if (set_ != nullptr) set_->estop = true;
    }

    bool valid() const noexcept { return set_ != nullptr; }

private:
    bool valid(unsigned idx) const noexcept
    {
        return set_ != nullptr && idx < kMaxJoints;
    }

    CommandSet   *set_ = nullptr;
    std::uint64_t stamp_ns_ = 0u;
};

/** 插件接口。由客户实现；核心库每个 tick 调一次 `step()`。 */
class RtHook {
public:
    virtual ~RtHook() = default;

    /** 非 RT 路径调用；失败时把原因写进 `err`（保持 ASCII/UTF-8 均可）。 */
    virtual bool init(const char *config_text, char *err, std::size_t err_len) = 0;

    /** RT 路径每 tick 调用：读快照、写命令。**禁止**分配/加锁/IO/异常。 */
    virtual void step(const StateSnapshot &state, CommandWriter &out) noexcept = 0;

    /** 非 RT 路径调用（关停）。 */
    virtual void shutdown() noexcept = 0;
};

/** 默认空实现：不动任何关节（用于"只观察"的调试场景）。 */
class NullRtHook final : public RtHook {
public:
    bool init(const char *, char *, std::size_t) override { return true; }
    void step(const StateSnapshot &, CommandWriter &) noexcept override {}
    void shutdown() noexcept override {}
};

}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_RT_JR_RT_HOOK_HPP */
