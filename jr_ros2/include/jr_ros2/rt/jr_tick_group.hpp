/**
 * @file    jr_tick_group.hpp
 * @brief   一个 tick 组 = 一个 RT 线程 + 一个时间基准（可含多条总线）+ 命令信箱 + 状态快照
 *
 * @par 数据流（对应 DESIGN §5.3）
 *   ROS 域 --submit_command()--> [三缓冲信箱] --(RT 取最新)--> cycle_begin/目标/cycle_end
 *   RT 域 --填充并发布--> [三缓冲快照] --acquire_snapshot()--> ROS 域（发布/诊断/服务）
 *
 *  两域之间**只有**两个三缓冲，没有锁、没有队列积压，也没有 DDS 进入 RT 路径。
 *
 * @par 安全暂停协议（ADR-7）
 *  需要执行**阻塞型**配置/运维调用（下载描述符、读参数、标定、复位）时：
 *    ① 非 RT 侧 `pause()`：置请求 → 等 `pause_ready_`（tick 线程**先走 SDK 安全失能序列**，
 *       再把 context 所有权交出来，然后自己停在这个状态不再碰 context）；
 *    ② 非 RT 侧在此期间直接使用 `bus(i)` 的 SDK 句柄；
 *    ③ 完成后 `resume()`：tick 线程按需重新使能并恢复 tick。
 *  暂停期间**没有**控制帧发出 —— 这正是"先安全失能"的原因（否则设备侧若武装了超时就会故障）。
 */

#ifndef JR_ROS2_RT_JR_TICK_GROUP_HPP
#define JR_ROS2_RT_JR_TICK_GROUP_HPP

#include <atomic>
#include <cstdint>
#include <thread>

#include "jr_ros2/jr_command.hpp"
#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_snapshot.hpp"
#include "jr_ros2/jr_status.hpp"
#include "jr_ros2/rt/jr_bus_runtime.hpp"
#include "jr_ros2/rt/jr_rt_hook.hpp"
#include "jr_ros2/rt/jr_rt_sched.hpp"
#include "jr_ros2/rt/jr_triple_buffer.hpp"

namespace jr {
namespace rt {

/** 抖动直方图：0..639 µs 按 10 µs 分桶 + 1 个溢出桶（无分配，可算 p99）。 */
inline constexpr unsigned kJitterBuckets = 65u;
inline constexpr unsigned kJitterBucketNs = 10000u;

struct TickGroupReport {
    RtReport  process = {};
    RtReport  thread = {};
    double    period_ns = 0.0;
    std::uint64_t tick_count = 0u;   /**< 停止后可用（测试用：丢 tick 数在快照里） */
    char      text[384] = {};
};

class TickGroup {
public:
    TickGroup() noexcept;
    ~TickGroup();

    TickGroup(const TickGroup &) = delete;
    TickGroup &operator=(const TickGroup &) = delete;

    /* ---------------- 配置阶段（非 RT） ---------------- */

    /** 绑定配置里的一个 tick 组（不打开总线）。`cfg` 必须在整个生命周期内存活。 */
    Status init(const Config &cfg, unsigned group_index, Result *res) noexcept;

    /** 打开本组全部总线（HAL + context + 关节）。 */
    Status open_buses(const OpenOptions &opt, Result *res) noexcept;

    /** 配置本组全部总线（描述符 + configure + 读回量程）。 */
    Status configure_buses(Result *res) noexcept;

    /** 使能 / 失能全部关节（阻塞；不要在 tick 线程运行时直接调，用 pause/resume 包起来）。 */
    Status activate(bool require_calibrated, Result *res) noexcept;
    Status deactivate(Result *res) noexcept;

    /** 启动 RT 线程（做进程级 + 线程级实时设置）。`hook` 可为 nullptr。 */
    Status start(RtHook *hook, Result *res) noexcept;

    /** 外部驱动模式（ADR-6 的 `tick_source=controller_manager`）：**不创建线程**，
     *  由调用方用自己的线程调 `step()` 驱动周期。
     *
     * 为什么需要：让 CAN 周期跟着 ros2_control 的 update 线程走（时序单一，
     * 适合"要最简"的客户与调试）；与 `internal` 相对（后者 CAN 周期与 CM 解耦）。
     *
     * ⚠ 与 `internal` 模式的差异（头文件里写清楚，避免使用时才发现）：
     *   - 只做**进程级**实时设置（mlockall）；**线程级**（SCHED_FIFO/绑核）留给调用方 ——
     *     跑周期的是它的线程，优先级归 controller_manager 的配置管；
     *   - `pause()/resume()` **不支持**（返回 `kNotSupported`）：没有独立的 tick 线程可以
     *     "停在安全点"。阻塞型 SDK 调用请放在生命周期切换（`on_configure`/`on_activate`/
     *     `on_deactivate`）里做 —— CM 保证它们不与 `read()/write()` 并发；
     *   - 抖动/丢 tick 按本组配置的 `rate_hz` 作为期望周期统计（只如实记账）；
     *   - 调用方应保证 **只有一个线程** 调 `step()`（并发调用会被拒并报错）。
     */
    Status start_external(RtHook *hook, Result *res) noexcept;

    /** 跑一个周期（仅外部模式）。
     *
     * ⚠ RT 调用方请传 `res=nullptr`：非空时本方法会把每周期结果格式化成消息（不必要）。
     * @return `kOk`；非外部模式 / 未启动 / 已有并发 `step()` → `kInvalidState`。 */
    Status step(Result *res) noexcept;

    bool external_tick() const noexcept { return external_.load(std::memory_order_acquire); }

    /** 请求停止并等待线程退出（幂等）。 */
    void stop() noexcept;

    /** 按退出策略关闭本组全部总线（需先 stop；幂等）。 */
    void close_buses(ExitAction action, Result *res) noexcept;

    bool running() const noexcept { return running_.load(std::memory_order_acquire); }

    /* ---------------- ROS 域（非 RT） ---------------- */

    /** 提交命令（最新值胜出；覆盖次数会在快照里体现）。 */
    void submit_command(const CommandSet &cmd) noexcept;

    /** 取最新快照；指针在本线程**下次调用前**有效（每读线程一份暂存，多读者安全）。 */
    const StateSnapshot *acquire_snapshot(bool *got_new = nullptr) noexcept;

    /** 拷贝版快照（便利）；false = 还没有任何快照。 */
    bool copy_snapshot(StateSnapshot *out) noexcept;

    /** 暂停 tick 组（等 tick 线程完成安全失能并交出所有权）。 */
    Status pause(Result *res, unsigned timeout_ms = 3000u) noexcept;

    /** 恢复 tick（若暂停前是使能态，会重新走 SDK 使能序列）。 */
    void resume() noexcept;

    /** 排空故障事件环。@return 实际取出的条数。 */
    unsigned drain_faults(FaultEvent *out, unsigned cap) noexcept;

    /* ---------------- 查询 ---------------- */

    unsigned bus_count() const noexcept { return bus_count_; }
    BusRuntime &bus(unsigned i) noexcept { return *buses_[i]; }
    const BusRuntime &bus(unsigned i) const noexcept { return *buses_[i]; }
    const BusReport &bus_report(unsigned i) const noexcept { return buses_[i]->report(); }
    const TickGroupReport &report() const noexcept { return report_; }
    const Config &config() const noexcept { return *cfg_; }
    const TickGroupCfg &group_cfg() const noexcept { return cfg_->groups[group_index_]; }
    unsigned rate_hz() const noexcept { return cfg_->groups[group_index_].rate_hz; }
    std::uint32_t period_ns() const noexcept { return period_ns_; }

private:
    void thread_main() noexcept;
    /** 跑一个周期（**RT 线程与外部模式共用**）。
     *
     * 两条路径如果各写一份，迟早会漂移 —— 本项目的教训（两份 SDK 状态码映射自己漂了）。
     * 因此周期体（收帧→快照→命令→hook→下发→发帧→超时闸→发布）只此一份。 */
    void run_cycle(std::uint64_t t_start, std::uint32_t jitter_ns) noexcept;
    /** 由绝对 deadline 算抖动，并维护直方图 / max / 丢 tick 计数（两模式共用）。 */
    std::uint32_t jitter_from_deadline(std::uint64_t t_start, std::uint64_t deadline) noexcept;

    /** 阻塞型 SDK 调用（使能/失能/参数/标定…）此刻能不能安全执行？
     *
     *  两条路径的**所有权模型不同**，所以规则也不同：
     *   - `internal`：所有权在 tick 线程手里 → 必须先 `pause()`（它会先安全失能再交出所有权）；
     *   - 外部驱动：**没有** tick 线程，调用方就是所有者 → 只要没有另一个线程正在 `step()` 就允许。
     *
     *  ⚠ 踩过的坑：这里以前只写第一条规则，于是外部模式下 `activate()` **永远**被拒
     *  （而该模式下 `pause()` 又明确不支持）→ 死锁式的自相矛盾：能建起来、永远使不上能。 */
    bool blocking_op_allowed(Result *res, const char *what) noexcept;
    void fill_dynamic_stats(StateSnapshot &s) noexcept;
    /** 在暂停/恢复等"非 tick 时刻"也发布一份快照，否则读者看不到状态变化。 */
    void publish_status_snapshot(std::uint64_t t_ns) noexcept;
    void apply_command_timeout(std::uint64_t now_ns) noexcept;
    void enter_pause() noexcept;
    void leave_pause() noexcept;

    const Config *cfg_ = nullptr;
    unsigned group_index_ = 0u;
    std::uint32_t period_ns_ = 0u;

    BusRuntime *buses_[kMaxBuses] = {};
    unsigned bus_count_ = 0u;
    unsigned joint_base_[kMaxBuses] = {};
    unsigned total_joints_ = 0u;

    TripleBuffer<CommandSet>   commands_ = {};
    TripleBuffer<StateSnapshot> snapshots_ = {};

    RtHook *hook_ = nullptr;
    CommandSet hook_cmd_ = {};      /**< tick 线程私有：hook 写、本线程读 */

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> external_{false};   /**< true = 由调用方 step() 驱动（无 tick 线程） */
    std::atomic<bool> stepping_{false};   /**< 外部模式的并发守卫（一个周期只允许一个线程跑） */
    std::atomic<bool> thread_ready_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> pause_request_{false};
    std::atomic<bool> pause_ready_{false};
    std::atomic<bool> pause_release_{false};

    /* tick 线程私有状态 */
    std::uint64_t tick_index_ = 0u;
    /** 上一个周期的起始时刻（外部模式的抖动基准；也用于内部模式的坦白记账）。 */
    std::uint64_t last_tick_ns_ = 0u;
    std::uint64_t last_cmd_seq_ = 0u;
    std::uint64_t last_cmd_ns_ = 0u;
    /** 最近一次收到的命令（值语义：三缓冲读出来的副本）。命令保持生效直到超时。 */
    CommandSet    last_cmd_ = {};
    bool          timeout_fired_ = false;
    bool          estop_latched_ = false;
    std::uint64_t missed_ticks_ = 0u;
    std::uint64_t overwrites_ = 0u;
    std::uint32_t config_pauses_ = 0u;
    std::uint32_t jitter_max_ns_ = 0u;
    std::uint32_t cycle_max_ns_ = 0u;
    std::uint32_t cmd_to_tx_max_ns_ = 0u;
    /* min/mean 需要累加器（RT 线程私有；避免每周期除法的代价只算 sum） */
    std::uint32_t jitter_min_ns_ = 0xFFFFFFFFu;
    std::uint64_t jitter_sum_ns_ = 0u;
    std::uint32_t cycle_min_ns_ = 0xFFFFFFFFu;
    std::uint64_t cycle_sum_ns_ = 0u;
    std::uint64_t cmd_to_tx_sum_ns_ = 0u;
    std::uint32_t cmd_to_tx_samples_ = 0u;
    std::uint64_t stat_samples_ = 0u;
    std::uint32_t hook_last_ns_ = 0u;
    std::uint32_t histogram_[kJitterBuckets] = {};

    TickGroupReport report_ = {};
    char            last_note_[192] = {};   /**< 组级一般备注（RT 节流等） */
    char            timeout_note_[192] = {};/**< 命令超时的**粘滞**备注（收到新命令才清除） */
};

}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_RT_JR_TICK_GROUP_HPP */
