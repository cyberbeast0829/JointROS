/**
 * @file    jr_rt_sched.hpp
 * @brief   实时调度与时钟（可移植封装：Linux 生产 / Windows 仅开发）
 *
 * @par 实现边界（不吹牛）
 *  - **Linux**：`SCHED_FIFO` + `sched_setaffinity` + `mlockall` + `clock_nanosleep(TIMER_ABSTIME)`；
 *    这是给出正式性能数据（DESIGN §7.5）的唯一组合。
 *  - **Windows**：高优先级线程 + 亲和性 + 高精度可等待定时器；**不承诺**抖动指标（仅开发/调试）。
 *  - **其它平台**：返回 `kNotSupported` 的降级路径，绝不做"假装绑核成功"。
 *
 * @warning 所有 `RtReport` 字段都是"**实测结果**"，不是"请求值"。
 *          提权失败必须让用户看见（`/diagnostics` 里会亮），否则客户会以为
 *          "我调了优先级就有实时性了"——这正是我们要避免的"说得比知道的多"。
 */

#ifndef JR_ROS2_RT_JR_RT_SCHED_HPP
#define JR_ROS2_RT_JR_RT_SCHED_HPP

#include <cstdint>

namespace jr {
namespace rt {

/** 单调时钟（ns）。**全包唯一**的单调时间源（度量/超时/看门狗都用它）。 */
std::uint64_t now_ns() noexcept;

/** 单调毫秒（给 SDK HAL 的 now_ms 用；回绕语义与 SDK 一致）。 */
std::uint32_t now_ms() noexcept;

struct RtSetup {
    bool enabled = true;
    int  priority = 80;
    bool mlock = true;
    int  cpu = -1;                /**< -1 = 不绑核 */
    bool warn_if_throttled = true;
};

struct RtReport {
    bool sched_ok = false;        /**< 实时调度策略是否**真的**生效 */
    bool affinity_ok = false;     /**< 绑核是否**真的**生效 */
    bool mlock_ok = false;        /**< 内存锁定是否**真的**生效 */
    bool high_res_timer = false;  /**< 高精度定时器是否可用（Windows） */
    bool throttled = false;       /**< 实时任务是否被内核节流（抖动会超预期） */
    int  applied_priority = 0;
    int  applied_cpu = -1;
    char text[256] = {};          /**< 人可读结论（含失败原因与建议） */
};

/** 进程级设置（mlockall / 高精度定时器）。在**任何** RT 线程创建前调用一次。 */
RtReport setup_process_rt(const RtSetup &setup) noexcept;

/** 线程级设置（调度策略/优先级/亲和性）。必须在 tick 线程内调用（作用于调用者）。 */
RtReport setup_thread_rt(const RtSetup &setup) noexcept;

/**
 * 无漂移睡眠器：按**绝对** deadline 睡眠（每 tick 重新计算，误差不累积）。
 * 构造在非 RT 路径（Windows 会创建可等待定时器句柄）。
 */
class Sleeper {
public:
    explicit Sleeper(std::uint64_t period_ns) noexcept;
    ~Sleeper();

    Sleeper(const Sleeper &) = delete;
    Sleeper &operator=(const Sleeper &) = delete;

    std::uint64_t period_ns() const noexcept { return period_ns_; }
    std::uint64_t start_ns() const noexcept { return start_ns_; }

    /** 第 n 个 tick 的绝对时刻（无漂移）。 */
    std::uint64_t deadline_of(std::uint64_t tick_index) const noexcept
    {
        return start_ns_ + tick_index * period_ns_;
    }

    /** 睡到绝对时刻（提前醒来会继续等；已过期立即返回）。 */
    void sleep_until(std::uint64_t deadline_ns) noexcept;

private:
    std::uint64_t period_ns_ = 0u;
    std::uint64_t start_ns_ = 0u;
    void *timer_ = nullptr;   /**< Windows: HANDLE；Linux: 不用 */
};

}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_RT_JR_RT_SCHED_HPP */
