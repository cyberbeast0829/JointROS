/**
 * @file    jr_tick_group.cpp
 * @brief   TickGroup 实现（RT 线程主体 + 无锁交换 + 安全暂停协议）
 */

#include "jr_ros2/rt/jr_tick_group.hpp"

#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

namespace jr {
namespace rt {

TickGroup::TickGroup() noexcept
{
    last_note_[0] = '\0';
}

TickGroup::~TickGroup()
{
    stop();
    for (unsigned i = 0u; i < bus_count_ && i < kMaxBuses; ++i) {
        delete buses_[i];
        buses_[i] = nullptr;
    }
    bus_count_ = 0u;
}

Status TickGroup::init(const Config &cfg, unsigned group_index, Result *res) noexcept
{
    if (group_index >= cfg.group_count) {
        if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "tick_group index out of range");
        return Status::kInvalidArgument;
    }
    cfg_ = &cfg;
    group_index_ = group_index;

    const TickGroupCfg &g = cfg.groups[group_index];
    period_ns_ = static_cast<std::uint32_t>(1000000000ull / (g.rate_hz != 0u ? g.rate_hz : 1u));
    report_.period_ns = static_cast<double>(period_ns_);

    bus_count_ = 0u;
    total_joints_ = 0u;
    for (unsigned i = 0u; i < g.bus_count && i < kMaxBuses; ++i) {
        const unsigned bi = g.bus_index[i];
        if (bi >= cfg.bus_count) {
            if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "bus index out of range");
            return Status::kInvalidArgument;
        }
        buses_[bus_count_] = new (std::nothrow) BusRuntime();
        if (buses_[bus_count_] == nullptr) {
            if (res != nullptr) res->set(Status::kNoMemory, Advice::kNone, "out of memory (BusRuntime)");
            return Status::kNoMemory;
        }
        joint_base_[bus_count_] = bus_joint_base(cfg, bi);
        total_joints_ += cfg.buses[bi].joint_count;
        ++bus_count_;
    }

    std::snprintf(report_.text, sizeof(report_.text),
                  "tick_group '%s': %u bus(es), %u joint(s), %.3f Hz (period %u ns)", g.name,
                  bus_count_, total_joints_, static_cast<double>(g.rate_hz), period_ns_);
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "%s", report_.text);
    }
    return Status::kOk;
}

Status TickGroup::open_buses(const OpenOptions &opt, Result *res) noexcept
{
    for (unsigned i = 0u; i < bus_count_; ++i) {
        const unsigned bi = cfg_->groups[group_index_].bus_index[i];
        const Status st = buses_[i]->open(cfg_->buses[bi], period_ns_, opt, res);
        if (st != Status::kOk) return st;
        /* 插值开关是**全局**的（`command.interpolation`），但实现在每条总线的 RT 路径里 ——
           统一在这里接一次：节点、ros2_control 组件都经过 TickGroup，两处不用各写一遍。 */
        buses_[i]->set_interpolation(cfg_->command.interpolation);
    }
    if (res != nullptr) res->set(Status::kOk, Advice::kNone, "opened %u bus(es)", bus_count_);
    return Status::kOk;
}

Status TickGroup::configure_buses(Result *res) noexcept
{
    for (unsigned i = 0u; i < bus_count_; ++i) {
        BusReport rep;
        const Status st = buses_[i]->configure(&rep, res);
        if (st != Status::kOk) return st;
    }
    if (res != nullptr) res->set(Status::kOk, Advice::kNone, "configured %u bus(es)", bus_count_);
    return Status::kOk;
}

bool TickGroup::blocking_op_allowed(Result *res, const char *what) noexcept
{
    if (external_.load(std::memory_order_acquire)) {
        /* 外部驱动：调用方就是 context 的所有者（这正是 ros2_control 生命周期函数做的事），
           只要此刻没有**另一个线程**在 step() 里就行。 */
        if (stepping_.load(std::memory_order_acquire)) {
            if (res != nullptr) {
                res->set(Status::kInvalidState, Advice::kNone,
                         "external tick mode: another thread is inside step() - stop driving cycles "
                         "before %s", what);
            }
            return false;
        }
        return true;
    }
    /* ⚠ 内部模式的安全口子：tick 线程在跑且**未暂停**时，所有权在 RT 线程手里，
       此时做阻塞调用会与 tick 并发访问同一个 context（SDK 明确禁止）——
       症状不是崩溃，而是难查的错值/状态错乱。所以这里主动拦。 */
    if (running_.load(std::memory_order_acquire) &&
        !pause_ready_.load(std::memory_order_acquire)) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "tick group is running: call pause() first (it safely disables the joints and "
                     "hands the SDK context over), then %s, then resume()", what);
        }
        return false;
    }
    return true;
}

Status TickGroup::activate(bool require_calibrated, Result *res) noexcept
{
    if (!blocking_op_allowed(res, "activate()")) return Status::kInvalidState;
    for (unsigned i = 0u; i < bus_count_; ++i) {
        const Status st = buses_[i]->activate(require_calibrated, res);
        if (st != Status::kOk) {
            /* 一个总线失败：把**已经使能**的那些收回去，避免"半开半关"的现场。 */
            for (unsigned k = 0u; k < i; ++k) {
                Result ignored;
                (void)buses_[k]->deactivate(&ignored);
            }
            return st;
        }
    }
    if (res != nullptr) res->set(Status::kOk, Advice::kNone, "enabled %u bus(es)", bus_count_);
    return Status::kOk;
}

Status TickGroup::deactivate(Result *res) noexcept
{
    if (!blocking_op_allowed(res, "deactivate()")) return Status::kInvalidState;
    Status worst = Status::kOk;
    for (unsigned i = 0u; i < bus_count_; ++i) {
        Result sub;
        const Status st = buses_[i]->deactivate(&sub);
        if (st != Status::kOk && worst == Status::kOk) worst = st;
    }
    if (res != nullptr) {
        res->set(worst, Advice::kNone, "deactivated %u bus(es)", bus_count_);
    }
    return worst;
}

Status TickGroup::start(RtHook *hook, Result *res) noexcept
{
    if (running_.load(std::memory_order_acquire)) {
        if (res != nullptr) res->set(Status::kOk, Advice::kNone, "tick group already running");
        return Status::kOk;
    }

    hook_ = hook;

    const TickGroupCfg &g = group_cfg();
    RtSetup setup;
    setup.enabled = cfg_->rt.enabled;
    setup.priority = (g.priority >= 0) ? g.priority : cfg_->rt.priority;
    setup.mlock = cfg_->rt.mlock;
    setup.cpu = (g.cpu >= 0) ? g.cpu : cfg_->rt.cpu;
    setup.warn_if_throttled = cfg_->rt.warn_if_throttled;

    report_.process = setup_process_rt(setup);

    stop_requested_.store(false, std::memory_order_release);
    thread_ready_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);

    try {
        thread_ = std::thread(&TickGroup::thread_main, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        if (res != nullptr) {
            res->set(Status::kInternal, Advice::kCheckRtPermissions, "failed to create the tick thread");
        }
        return Status::kInternal;
    }

    /* 等线程完成实时设置（最多 ~500 ms），这样调用方拿到的是"实测结果"而不是"请求值"。 */
    for (unsigned i = 0u; i < 500u; ++i) {
        if (thread_ready_.load(std::memory_order_acquire)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (res != nullptr) {
        const bool rt_ok = report_.thread.sched_ok || !setup.enabled;
        res->set(rt_ok ? Status::kOk : Status::kInvalidArgument,
                 rt_ok ? Advice::kNone : Advice::kCheckRtPermissions,
                 "%s | thread: %s | process: %s", report_.text,
                 report_.thread.text[0] ? report_.thread.text : "(pending)",
                 (report_.process.mlock_ok || !setup.enabled) ? "memlocked"
                                                              : "mlock NOT applied (dev/unpriv?)");
    }
    return Status::kOk;
}

void TickGroup::stop() noexcept
{
    if (external_.load(std::memory_order_acquire)) {
        /* 外部模式没有线程要 join：只把状态清干净（幂等）。 */
        stop_requested_.store(true, std::memory_order_release);
        running_.store(false, std::memory_order_release);
        external_.store(false, std::memory_order_release);
        report_.tick_count = tick_index_;
        return;
    }
    if (!running_.load(std::memory_order_acquire) && !thread_.joinable()) return;
    stop_requested_.store(true, std::memory_order_release);
    /* 若正停在暂停态，先放它出来，否则永远等不到退出。 */
    pause_release_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
    running_.store(false, std::memory_order_release);
    report_.tick_count = tick_index_;
}

void TickGroup::close_buses(ExitAction action, Result *res) noexcept
{
    for (unsigned i = 0u; i < bus_count_; ++i) {
        Result sub;
        buses_[i]->close(action, &sub);
        if (i == 0u && res != nullptr) *res = sub;
    }
}

void TickGroup::submit_command(const CommandSet &cmd) noexcept
{
    commands_.store([&cmd](CommandSet &slot) { slot = cmd; });
}

const StateSnapshot *TickGroup::acquire_snapshot(bool *got_new) noexcept
{
    /* ⚠ 首次 tick 之前**没有任何有效快照**：返回 nullptr 而不是一份全零状态。
       否则上层会把"上电瞬间"读成"所有关节位置 = 0、无故障"。 */
    if (!snapshots_.has_value()) {
        if (got_new != nullptr) *got_new = false;
        return nullptr;
    }
    /* **每读线程一份**暂存：契约仍是"指针在下次调用前有效"，但多读者各自有独立副本。
       ⚠ 不能用一个共享成员暂存 —— ROS 多线程执行器下定时器回调与服务回调会互相踩
       （实测：4 读者 × 2400 次就能抓到一次撕裂读）。 */
    static thread_local StateSnapshot stage;
    snapshots_.read(&stage, got_new);   /* 没有新数据时 stage 保持本线程上次的副本 */
    return &stage;
}

bool TickGroup::copy_snapshot(StateSnapshot *out) noexcept
{
    if (out == nullptr) return false;
    const StateSnapshot *p = acquire_snapshot();
    if (p == nullptr) return false;   /* 还没有任何快照：不把"全零"当成状态 */
    *out = *p;
    return true;
}

Status TickGroup::pause(Result *res, unsigned timeout_ms) noexcept
{
    if (external_.load(std::memory_order_acquire)) {
        /* 外部模式没有 tick 线程可以"停在安全点" —— 不要做一个半成品的所有权协议，
           而是说清楚正确的做法（客户端拿它当依据）。 */
        if (res != nullptr) {
            res->set(Status::kNotSupported, Advice::kNone,
                     "external tick mode has no tick thread to park at a safe point: run blocking SDK "
                     "calls in lifecycle transitions (on_configure/on_activate/on_deactivate) instead - "
                     "controller_manager guarantees they do not run concurrently with read()/write()");
        }
        return Status::kNotSupported;
    }
    if (!running_.load(std::memory_order_acquire)) {
        /* 线程都没起：调用方可以直接用 context（所有权本来就在它手上）。 */
        if (res != nullptr) res->set(Status::kOk, Advice::kNone, "tick group not running: no pause needed");
        return Status::kOk;
    }
    if (pause_ready_.load(std::memory_order_acquire)) {
        if (res != nullptr) res->set(Status::kOk, Advice::kNone, "already paused");
        return Status::kOk;
    }

    pause_release_.store(false, std::memory_order_release);
    pause_request_.store(true, std::memory_order_release);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pause_ready_.load(std::memory_order_acquire)) {
            if (res != nullptr) {
                res->set(Status::kOk, Advice::kNone,
                         "paused: joints safely disabled and SDK context ownership handed over");
            }
            return Status::kOk;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    if (res != nullptr) {
        res->set(Status::kTimeout, Advice::kNone,
                 "pause timed out after %u ms: the tick thread did not hand over ownership",
                 timeout_ms);
    }
    return Status::kTimeout;
}

void TickGroup::resume() noexcept
{
    if (external_.load(std::memory_order_acquire)) return;   /* 外部模式：无暂停可恢复 */
    pause_release_.store(true, std::memory_order_release);
}

Status TickGroup::start_external(RtHook *hook, Result *res) noexcept
{
    if (running_.load(std::memory_order_acquire)) {
        if (res != nullptr) {
            res->set(Status::kOk, Advice::kNone, "tick group already started (%s tick mode)",
                     external_.load(std::memory_order_acquire) ? "external" : "internal");
        }
        return Status::kOk;
    }

    hook_ = hook;
    const TickGroupCfg &g = group_cfg();

    /* 只做**进程级**设置（mlockall 是进程级的）。线程级（SCHED_FIFO/绑核）留给调用方：
       在外部模式下跑周期的是**它的**线程（ros2_control 的 update 线程），
       优先级归 controller_manager 配置管 —— 我们不去改别人的线程，也不谎称设过。 */
    RtSetup setup;
    setup.enabled = cfg_->rt.enabled;
    setup.priority = (g.priority >= 0) ? g.priority : cfg_->rt.priority;
    setup.mlock = cfg_->rt.mlock;
    setup.cpu = (g.cpu >= 0) ? g.cpu : cfg_->rt.cpu;
    setup.warn_if_throttled = cfg_->rt.warn_if_throttled;
    report_.process = setup_process_rt(setup);
    std::snprintf(report_.text, sizeof(report_.text),
                  "external tick mode (the caller drives step(); no RT thread in this group)");
    std::snprintf(report_.thread.text, sizeof(report_.thread.text),
                  "thread-level RT settings NOT applied (the caller's thread drives the cycle)");

    stop_requested_.store(false, std::memory_order_release);
    thread_ready_.store(true, std::memory_order_release);
    last_tick_ns_ = 0u;
    external_.store(true, std::memory_order_release);
    running_.store(true, std::memory_order_release);

    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "%s | process: %s", report_.text,
                 (report_.process.mlock_ok || !setup.enabled) ? "memlocked"
                                                              : "mlock NOT applied (dev/unpriv?)");
    }
    return Status::kOk;
}

Status TickGroup::step(Result *res) noexcept
{
    if (!external_.load(std::memory_order_acquire)) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "step() needs external tick mode (start_external()); this group is driven by its own "
                     "RT thread - use submit_command()/acquire_snapshot() instead");
        }
        return Status::kInvalidState;
    }
    if (!running_.load(std::memory_order_acquire)) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "tick group is stopped");
        return Status::kInvalidState;
    }
    /* 并发守卫：SDK 的约束是"一个 context 一个线程"，所以一节周期只允许一个调用者。 */
    bool expected = false;
    if (!stepping_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "step() is already running on another thread (exactly one thread may drive a tick group)");
        }
        return Status::kInvalidState;
    }

    const std::uint64_t t_start = now_ns();
    std::uint32_t jitter = 0u;
    if (last_tick_ns_ != 0u && period_ns_ != 0u) {
        /* 用本组配置的 rate_hz 作为期望周期：如实统计抖动/丢 tick，不假装零抖动。 */
        jitter = jitter_from_deadline(t_start, last_tick_ns_ + period_ns_);
    }
    run_cycle(t_start, jitter);
    last_tick_ns_ = t_start;

    stepping_.store(false, std::memory_order_release);
    if (res != nullptr) {
        /* RT 调用方请传 nullptr：这里只是一句人类可读的说明，没必要每周期格式化。 */
        res->set(Status::kOk, Advice::kNone, "external tick %llu done",
                 static_cast<unsigned long long>(tick_index_ - 1u));
    }
    return Status::kOk;
}

unsigned TickGroup::drain_faults(FaultEvent *out, unsigned cap) noexcept
{
    unsigned n = 0u;
    for (unsigned i = 0u; i < bus_count_; ++i) {
        while (n < cap && buses_[i]->pop_fault(&out[n])) ++n;
    }
    return n;
}

/* ==========================================================================
 * RT 线程主体
 * ======================================================================== */

void TickGroup::thread_main() noexcept
{
    const TickGroupCfg &g = group_cfg();
    RtSetup setup;
    setup.enabled = cfg_->rt.enabled;
    setup.priority = (g.priority >= 0) ? g.priority : cfg_->rt.priority;
    setup.mlock = cfg_->rt.mlock;
    setup.cpu = (g.cpu >= 0) ? g.cpu : cfg_->rt.cpu;
    setup.warn_if_throttled = cfg_->rt.warn_if_throttled;

    report_.thread = setup_thread_rt(setup);
    if (report_.thread.throttled) {
        std::snprintf(last_note_, sizeof(last_note_),
                      "kernel RT throttling is ON (sched_rt_runtime_us != -1): expect periodic "
                      "jitter spikes. Set it to -1 for production.");
    }
    thread_ready_.store(true, std::memory_order_release);

    Sleeper sleeper(period_ns_);
    bool paused = false;

    while (!stop_requested_.load(std::memory_order_acquire)) {
        const std::uint64_t deadline = sleeper.deadline_of(tick_index_);
        sleeper.sleep_until(deadline);

        const std::uint64_t t_start = now_ns();

        /* ---- 暂停协议：先安全失能，再交出所有权，然后停在这里 ---- */
        if (pause_request_.load(std::memory_order_acquire) && !paused) {
            enter_pause();
            paused = true;
            pause_ready_.store(true, std::memory_order_release);
        }
        if (paused) {
            if (!pause_release_.load(std::memory_order_acquire)) continue;   /* 不碰 context */
            leave_pause();
            paused = false;
            pause_ready_.store(false, std::memory_order_release);
            pause_request_.store(false, std::memory_order_release);
            pause_release_.store(false, std::memory_order_release);
            continue;
        }

        run_cycle(t_start, jitter_from_deadline(t_start, deadline));
    }
}

std::uint32_t TickGroup::jitter_from_deadline(std::uint64_t t_start, std::uint64_t deadline) noexcept
{
    /* ---- 抖动统计（绝对 deadline，因此这里量的是"唤醒误差"） ---- */
    std::uint32_t jitter = 0u;
    if (t_start > deadline) {
        const std::uint64_t late = t_start - deadline;
        jitter = (late > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<std::uint32_t>(late);
        if (jitter > period_ns_) {
            missed_ticks_ += jitter / period_ns_;
        }
    }
    if (jitter > jitter_max_ns_) jitter_max_ns_ = jitter;
    if (jitter < jitter_min_ns_) jitter_min_ns_ = jitter;
    jitter_sum_ns_ += jitter;
    const unsigned bucket = jitter / kJitterBucketNs;
    ++histogram_[(bucket < kJitterBuckets) ? bucket : (kJitterBuckets - 1u)];
    return jitter;
}

void TickGroup::run_cycle(std::uint64_t t_start, std::uint32_t jitter_ns) noexcept
{
    /* ---- ① 收帧 / 解码（每总线） ---- */
    for (unsigned i = 0u; i < bus_count_; ++i) {
        buses_[i]->tick_begin(t_start);
    }

    /* ---- ② 填快照（写者私有槽：填完才发布，读者永远看不到半份数据） ---- */
    StateSnapshot &snap = snapshots_.write_slot();
    snap.tick = tick_index_;
    snap.t_ns = t_start;
    snap.joint_count = total_joints_;
    snap.bus_count = bus_count_;
    snap.mode = static_cast<std::uint32_t>(buses_[0]->mode());
    for (unsigned i = 0u; i < bus_count_; ++i) {
        buses_[i]->fill_snapshot(snap, i, joint_base_[i]);
    }

    /* ---- ③ 取命令（最新值） ---- */
    /* ⚠ v0.14：三缓冲读取改成**值语义**（认领 + 拷贝），所以这里多一次 CommandSet 拷贝。
       代价是几十 ns；换来的是"多读者不会把对方的槽位交还给写者"这个硬保证。 */
    bool got_new = false;
    CommandSet cmd{};
    if (commands_.read(&cmd, &got_new)) {
        if (last_cmd_seq_ != 0u && cmd.seq > last_cmd_seq_ + 1u) {
            overwrites_ += (cmd.seq - last_cmd_seq_ - 1u);
        }
        last_cmd_seq_ = cmd.seq;
        last_cmd_ns_ = t_start;
        last_cmd_ = cmd;           /* 值语义读出来的副本；后面每 tick 都拿它生效 */
        timeout_note_[0] = '\0';   /* 命令恢复正常 → 清掉"命令超时"的粘滞备注 */
        if (!cmd.estop) estop_latched_ = false;
    }

    /* ---- ④ RtHook（若启用）：读本 tick 快照，写自己的命令 ---- */
    hook_cmd_.clear_all_mask();
    hook_cmd_.estop = false;
    hook_cmd_.zero_torque_all = false;
    hook_cmd_.stamp_ns = 0u;   /* 必须复位：否则"上一 tick 的"时间戳会污染延迟统计 */
    hook_last_ns_ = 0u;
    if (hook_ != nullptr) {
        const std::uint64_t h0 = now_ns();
        CommandWriter writer(&hook_cmd_, t_start);
        hook_->step(snap, writer);
        hook_last_ns_ = static_cast<std::uint32_t>(now_ns() - h0);
    }

    /* ---- ⑤ 应用命令（信箱 + hook 覆盖） ---- */
    const bool fresh = (last_cmd_ns_ != 0u);
    for (unsigned i = 0u; i < bus_count_; ++i) {
        if (fresh) {
            buses_[i]->apply_command(last_cmd_, joint_base_[i]);
        }
        if (hook_cmd_.mask != 0u || hook_cmd_.estop || hook_cmd_.zero_torque_all) {
            buses_[i]->apply_command(hook_cmd_, joint_base_[i]);
        }
    }

    /* ---- ⑥ 广播下发（1 帧驱动多关节；不满足条件时 SDK 走单播） ---- */
    for (unsigned i = 0u; i < bus_count_; ++i) {
        const FeedbackPolicy fp = cfg_->buses[cfg_->groups[group_index_].bus_index[i]].feedback;
        if (fp == FeedbackPolicy::kBroadcastHeartbeat || fp == FeedbackPolicy::kHeartbeatOnly ||
            fp == FeedbackPolicy::kUnicastPoll) {
            (void)buses_[i]->try_broadcast_mit();
        }
    }

    /* ---- ⑦ 发帧（每总线） ---- */
    for (unsigned i = 0u; i < bus_count_; ++i) {
        buses_[i]->tick_end(t_start);
    }

    /* ---- ⑧ 命令超时闸（独立于设备看门狗） ---- */
    apply_command_timeout(t_start);

    /* ---- ⑨ 发布快照（含统计） ---- */
    const std::uint64_t t_end = now_ns();
    const std::uint32_t cycle_ns =
        (t_end - t_start > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<std::uint32_t>(t_end - t_start);
    if (cycle_ns > cycle_max_ns_) cycle_max_ns_ = cycle_ns;
    if (cycle_ns < cycle_min_ns_) cycle_min_ns_ = cycle_ns;
    cycle_sum_ns_ += cycle_ns;
    ++stat_samples_;
    /* 命令→发帧时延：两条路径（hook / 信箱）只取一个样本，不要重复计入统计。 */
    std::uint32_t cmd_lat = 0u;
    bool have_lat = false;
    if (hook_cmd_.stamp_ns != 0u) {
        cmd_lat = static_cast<std::uint32_t>(t_end - hook_cmd_.stamp_ns);
        have_lat = true;
    }
    if (got_new && cmd.stamp_ns != 0u && t_end > cmd.stamp_ns) {
        const std::uint64_t lat64 = t_end - cmd.stamp_ns;
        const std::uint32_t lat = (lat64 > 0xFFFFFFFFull) ? 0xFFFFFFFFu
                                                         : static_cast<std::uint32_t>(lat64);
        if (!have_lat || lat > cmd_lat) {
            cmd_lat = lat;
        }
        have_lat = true;
    }
    if (have_lat) {
        if (cmd_lat > cmd_to_tx_max_ns_) cmd_to_tx_max_ns_ = cmd_lat;
        cmd_to_tx_sum_ns_ += cmd_lat;
        ++cmd_to_tx_samples_;
    }
    snap.rt.jitter_last_ns = jitter_ns;
    snap.rt.hook_last_ns = hook_last_ns_;
    fill_dynamic_stats(snap);
    snapshots_.publish();

    ++tick_index_;
}

void TickGroup::enter_pause() noexcept
{
    /* 安全失能：SDK 的 deactivate 是"安全帧 → 等 2 周期 → STOP_MOTOR → 等 IDLE"。
       这一步必须在**交出所有权之前**完成，否则非 RT 侧的阻塞调用（如描述符下载）
       会让设备侧武装的超时先触发。 */
    for (unsigned i = 0u; i < bus_count_; ++i) {
        Result ignored;
        (void)buses_[i]->deactivate(&ignored);
        buses_[i]->set_mode(BusMode::kPaused);
    }
    ++config_pauses_;
    std::snprintf(last_note_, sizeof(last_note_),
                  "paused for a blocking operation (%u config pause(s) so far)", config_pauses_);
    publish_status_snapshot(now_ns());
}

void TickGroup::publish_status_snapshot(std::uint64_t t_ns) noexcept
{
    StateSnapshot &s = snapshots_.write_slot();
    s.tick = tick_index_;
    s.t_ns = t_ns;
    s.joint_count = total_joints_;
    s.bus_count = bus_count_;
    s.mode = (bus_count_ > 0u) ? static_cast<std::uint32_t>(buses_[0]->mode())
                               : static_cast<std::uint32_t>(BusMode::kIdle);
    for (unsigned i = 0u; i < bus_count_; ++i) {
        buses_[i]->fill_snapshot(s, i, joint_base_[i]);
    }
    fill_dynamic_stats(s);
    snapshots_.publish();
}

void TickGroup::leave_pause() noexcept
{
    /* 是否重新使能，由**各总线的实际模式**决定：暂停窗口里非 RT 侧可能已经
       主动 `activate()` 过（那就不用再动），也可能什么都没做（保持 READY）。
       ⚠ 不要用"暂停前的快照"来决定：暂停期间状态是被非 RT 侧改变的。 */
    bool any_active = false;
    for (unsigned i = 0u; i < bus_count_; ++i) {
        if (buses_[i]->mode() == BusMode::kActive) any_active = true;
    }

    if (any_active) {
        for (unsigned i = 0u; i < bus_count_; ++i) {
            Result ignored;
            const Status st = buses_[i]->activate(true, &ignored);
            buses_[i]->set_mode(st == Status::kOk ? BusMode::kActive : BusMode::kFault);
        }
        std::snprintf(last_note_, sizeof(last_note_), "resumed: joints re-enabled");
    } else {
        for (unsigned i = 0u; i < bus_count_; ++i) {
            buses_[i]->set_mode(BusMode::kReady);
        }
        std::snprintf(last_note_, sizeof(last_note_), "resumed in READY (joints stay disabled)");
    }
    publish_status_snapshot(now_ns());
}

void TickGroup::apply_command_timeout(std::uint64_t now_ns_in) noexcept
{
    const CommandCfg &cc = cfg_->command;
    if (cc.timeout_ms == 0u || last_cmd_ns_ == 0u) return;

    const std::uint64_t limit = static_cast<std::uint64_t>(cc.timeout_ms) * 1000000ull;
    const bool expired = (now_ns_in > last_cmd_ns_) && ((now_ns_in - last_cmd_ns_) > limit);
    if (!expired) {
        timeout_fired_ = false;
        return;
    }
    if (timeout_fired_) return;   /* 只动作一次，不每 tick 重复发 */
    timeout_fired_ = true;

    switch (cc.on_timeout) {
    case TimeoutAction::kHold:
        for (unsigned i = 0u; i < bus_count_; ++i) {
            buses_[i]->hold_all(cc.hold_stiffness, cc.hold_damping);
        }
        std::snprintf(timeout_note_, sizeof(timeout_note_),
                      "command timeout (>%u ms without a new command): holding position "
                      "(command.on_timeout=hold; send fresh commands to resume)",
                      cc.timeout_ms);
        break;
    case TimeoutAction::kZeroTorque:
        for (unsigned i = 0u; i < bus_count_; ++i) {
            buses_[i]->zero_torque_all();
        }
        std::snprintf(timeout_note_, sizeof(timeout_note_),
                      "command timeout (>%u ms): zero torque (limp)", cc.timeout_ms);
        break;
    case TimeoutAction::kDisable:
        for (unsigned i = 0u; i < bus_count_; ++i) {
            buses_[i]->request_disable_all();
        }
        std::snprintf(timeout_note_, sizeof(timeout_note_),
                      "command timeout (>%u ms): disabling the joints (request queued)",
                      cc.timeout_ms);
        break;
    case TimeoutAction::kEstop:
        for (unsigned i = 0u; i < bus_count_; ++i) {
            buses_[i]->estop_now();
        }
        std::snprintf(timeout_note_, sizeof(timeout_note_),
                      "command timeout (>%u ms): ESTOP broadcast on the whole bus",
                      cc.timeout_ms);
        estop_latched_ = true;
        break;
    }
}

void TickGroup::fill_dynamic_stats(StateSnapshot &s) noexcept
{
    RtStatsPOD &rt = s.rt;
    rt.period_ns = period_ns_;
    rt.jitter_max_ns = jitter_max_ns_;
    rt.cycle_max_ns = cycle_max_ns_;
    rt.cmd_to_tx_max_ns = cmd_to_tx_max_ns_;
    /* min/mean：用累加器算（每周期一次除法可以接受；RT 侧只做加法）。
       ⚠ 没有样本时保持 0 并让 `stat_samples == 0` 说明"还没有数据"，
          不要把"没样本"说成"抖动为 0"（那是两回事）。 */
    rt.stat_samples = static_cast<std::uint32_t>(stat_samples_);
    if (stat_samples_ != 0u) {
        rt.jitter_min_ns = jitter_min_ns_;
        rt.jitter_mean_ns = static_cast<std::uint32_t>(jitter_sum_ns_ / stat_samples_);
        rt.cycle_min_ns = cycle_min_ns_;
        rt.cycle_mean_ns = static_cast<std::uint32_t>(cycle_sum_ns_ / stat_samples_);
    }
    if (cmd_to_tx_samples_ != 0u) {
        rt.cmd_to_tx_mean_ns = static_cast<std::uint32_t>(cmd_to_tx_sum_ns_ / cmd_to_tx_samples_);
    }
    rt.tick_count = tick_index_ + 1u;   /* 1 基：本份快照 = 第 N 个已完成周期。
                                           与 TickGroupReport::tick_count（停止时的总数）
                                           口径一致，免得客户看到差 1 而怀疑丢 tick。 */
    rt.missed_ticks = missed_ticks_;
    rt.command_overwrites = overwrites_;
    rt.config_pauses = config_pauses_;
    rt.rt_sched_ok = report_.thread.sched_ok;
    rt.mlock_ok = report_.process.mlock_ok;
    rt.affinity_ok = report_.thread.affinity_ok;
    rt.rt_throttled = report_.thread.throttled;

    /* p99：从固定分桶直方图算（无分配、O(桶数)）。 */
    std::uint64_t total = 0u;
    for (unsigned i = 0u; i < kJitterBuckets; ++i) total += histogram_[i];
    if (total > 0u) {
        const std::uint64_t target = (total * 99u) / 100u;
        std::uint64_t acc = 0u;
        for (unsigned i = 0u; i < kJitterBuckets; ++i) {
            acc += histogram_[i];
            if (acc >= target && target > 0u) {
                rt.jitter_p99_ns = (i + 1u) * kJitterBucketNs;
                break;
            }
        }
        if (rt.jitter_p99_ns == 0u) rt.jitter_p99_ns = kJitterBuckets * kJitterBucketNs;
    }

    for (unsigned i = 0u; i < bus_count_ && i < kMaxBuses; ++i) {
        s.buses[i].degraded = s.buses[i].degraded || (buses_[i]->last_cycle_status() != 0);
    }
    /* 组级备注：命令超时是粘滞的（比 RT 节流警告更紧急），没超时才显示一般备注。 */
    std::snprintf(s.note, sizeof(s.note), "%s", (timeout_note_[0] != '\0') ? timeout_note_
                                                                            : last_note_);
}

}  // namespace rt
}  // namespace jr
