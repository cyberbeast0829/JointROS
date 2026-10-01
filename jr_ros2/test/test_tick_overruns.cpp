/**
 * @file    test_tick_overruns.cpp
 * @brief   丢拍计数（`missed_ticks` → `tick_overruns`）的**可证伪**用例
 *
 * @par 为什么要专门测这个
 *  真机 jog（内部就是 `pause()` → 保持 → `resume()` 窗口）之后，`tick_overruns`
 *  与窗口时长**超线性**增长：
 *      窗口 0.5 s → 增量 132,879   （按 1 kHz 折算只应是   500）
 *      窗口 1.0 s → 增量 519,093   （只应是 1,000）
 *      窗口 2.0 s → 增量 2,050,096 （只应是 2,000）
 *      纯待机 2 s（无暂停）→ 增量 **0**
 *  也就是说，这个数**只在暂停路径上**炸开，而且**超出期望 265x / 519x / 1025x**。
 *
 * @par 机制
 *  tick 线程里 `deadline = sleeper.deadline_of(tick_index_)` 是**每轮重算**的，
 *  而暂停分支只 `continue`（既不推进 `tick_index_`、也不重算基准）⇒ 停在暂停分支的
 *  **每一轮循环** 都量到同一个“迟到量”，并各自走一次
 *  `jitter_from_deadline()` → `missed_ticks_ += jitter / period_ns_`。
 *  重复次数 ≈ 暂停期间的循环圈数（2 s @1 kHz 就是上千次）。
 *
 * @par 本用例的判据
 *  ① 待机 1 s：增量必须是**个位数**量级（不是几百上千）；
 *  ② `pause()` + 停留 200 ms：增量**不得超过窗口量级**（≤ 1.5×窗口 + 100 拍）；
 *  ③ `pause()` + 200 ms + `resume()`（真机 jog 那条路）：同 ② 判据。
 *  三条同时成立才算“计数语义正确”；只让 ① 变绿而 ②③ 仍炸 = 没修。
 */

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "jr_ros2/jr_snapshot.hpp"
#include "jr_ros2/rt/jr_tick_group.hpp"
#include "jr_test.hpp"

namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t elapsed_ms(Clock::time_point t0)
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count());
}

/** 丢拍计数经 `fill_dynamic_stats()` 进快照，因此这里从快照取（与 CLI 同源）。 */
std::uint64_t missed(jr::rt::TickGroup &g)
{
    const jr::StateSnapshot *s = g.acquire_snapshot();
    return (s == nullptr) ? 0u : s->rt.missed_ticks;
}

/** 最小 Config：一个 tick 组 + 一条 `kVirtual` 总线（**不真开设备**，只跑时序）。
 *  ⚠ `bus_count`/`group_count` 必须显式写：`buses[]`/`groups[]` 是定长数组，
 *  不是 vector（这个项目里所有"计数 + 定长数组"的写法都是这个约定）。 */
jr::Config make_cfg(unsigned rate_hz)
{
    jr::Config cfg{};
    cfg.bus_count = 1u;
    cfg.group_count = 1u;

    jr::BusCfg &bus = cfg.buses[0];
    std::snprintf(bus.name, sizeof(bus.name), "vtest");
    bus.hal = jr::HalKind::kVirtual;
    bus.channel[0] = '\0';   /* 虚拟模型：空规格串 = SDK 内置默认设备 */
    bus.joint_count = 0u;
    bus.is_fd = false;

    jr::TickGroupCfg &grp = cfg.groups[0];
    std::snprintf(grp.name, sizeof(grp.name), "g0");
    grp.rate_hz = rate_hz;
    grp.bus_count = 1u;
    grp.bus_index[0] = 0u;

    return cfg;
}

/** 只跑时序：**不打开总线** ⇒ `run_cycle()` 里的总线循环为空，
 *  正好把“暂停窗口记账”这一项单独隔离出来。 */
class Probe {
public:
    explicit Probe(unsigned rate_hz) : cfg_(make_cfg(rate_hz)), g_() {}

    bool start()
    {
        jr::Result r{};
        if (g_.init(cfg_, 0u, &r) != jr::Status::kOk) {
            std::printf("  init 失败: %s\n", r.message);
            return false;
        }

        /* `joint_count == 0` 是**发现用空总线**，正常路径会被拒 —— 这里只跑时序，
         *  所以显式放行；同时 `enable_lock = false`：虚拟总线不需要独占物理通道。 */
        jr::rt::OpenOptions oo{};
        oo.allow_empty_scan_bus = true;
        oo.enable_lock = false;
        if (g_.open_buses(oo, &r) != jr::Status::kOk) {
            std::printf("  open_buses 失败: %s\n", r.message);
            return false;
        }
        if (g_.start(nullptr, &r) != jr::Status::kOk) {
            std::printf("  start 失败: %s\n", r.message);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        return true;
    }

    jr::rt::TickGroup &g() noexcept { return g_; }

private:
    jr::Config cfg_;
    jr::rt::TickGroup g_;
};

/** ②③ 共用的判据：增量必须是"暂停窗口折算值"的**同一个数量级**，
 *  绝不能是它的几十上百倍。
 *
 *  @par 为什么只判上界，不判下界
 *  记账发生在**恢复后的第一次迭代**：它读 `t_start - deadline` 并结清整个窗口。
 *  但"第一次迭代何时发生"取决于平台：
 *    - Linux：`clock_nanosleep` 按绝对时刻唤醒，`pause()` 前的 deadline 早已过期
 *      ⇒ 恢复后**第一拍立刻**结算 ⇒ 增量落在 `pause()` 内部，等测试线程
 *      再去读快照时已经是"结清后"的值（实测 0）；
 *    - Windows：`SetWaitableTimer` 等待 + `Sleep()` 恰好跨过下一拍
 *      ⇒ 结算发生在 `pause()` **返回之后**（实测 233）。
 *  两者都**符合**"窗口只计一次"的语义，只是观测点先后不同。
 *  因此下界取 0（容忍"已在 pause() 内结清"），上界卡死在 1.5 倍窗口 ——
 *  这正是重复累加会撞穿的地方（修前 20,706 ≈ 202 ms 窗口的 103 倍）。 */
bool within_window(std::uint64_t delta, std::uint64_t window_ms)
{
    const std::uint64_t expect = window_ms;                       /* 1 kHz → 1 拍/ms */
    const std::uint64_t limit = expect + expect / 2u + 100u;
    std::printf("  窗口≈%llu ms，增量=%llu，期望≈%llu，上限=%llu\n",
                static_cast<unsigned long long>(window_ms),
                static_cast<unsigned long long>(delta),
                static_cast<unsigned long long>(expect),
                static_cast<unsigned long long>(limit));
    if (delta > limit) {
        std::printf("  ^^^ 重复累加：delta/expect ≈ %llu 倍\n",
                    static_cast<unsigned long long>(delta / (expect == 0u ? 1u : expect)));
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    /* ---------------------------------------------------------------- ① 待机基线 */
    JR_CASE("idle 1 s @1 kHz 不应产生大量丢拍");
    {
        Probe p(1000u);
        if (p.start()) {
            jr::rt::TickGroup &g = p.g();
            const std::uint64_t before = missed(g);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            const std::uint64_t after = missed(g);
            g.stop();
            const std::uint64_t delta = after - before;
            std::printf("  1 s 增量=%llu\n", static_cast<unsigned long long>(delta));
            JR_CHECK_MSG(delta < 100u, "待机 1 s 就丢了几百拍：计数把噪声也算进去了");
        } else {
            JR_CHECK_MSG(false, "最小虚拟总线 tick 组起不来");
        }
    }

    /* ------------------------------------------------- ② pause 窗口（不 resume）：
     * 停住之后每一轮循环都会重复量到同一个迟到量 —— 这是重复累加的现场。 */
    JR_CASE("pause 200 ms 只应计一次窗口");
    {
        Probe p(1000u);
        if (p.start()) {
            jr::rt::TickGroup &g = p.g();
            const std::uint64_t before = missed(g);
            jr::Result r{};
            const Clock::time_point t0 = Clock::now();
            const jr::Status ps = g.pause(&r, 500u);
            const bool ok = (ps == jr::Status::kOk);
            const std::uint64_t paused_ms = elapsed_ms(t0);
            if (!ok) {
                JR_CHECK_MSG(false, "pause 握手失败（虚拟总线也该能停）");
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                const std::uint64_t after = missed(g);
                g.stop();
                JR_CHECK(within_window(after - before, paused_ms + 200u));
            }
        } else {
            JR_CHECK_MSG(false, "最小虚拟总线 tick 组起不来");
        }
    }

    /* ------------------------------------------- ③ resume 窗口（真机 jog 走这条）：
     * `pause()` → 保持 → `resume()`，与真机 jog 路径一致。 */
    JR_CASE("pause+resume 200 ms 只应计一次窗口（真机 jog 路径）");
    {
        Probe p(1000u);
        if (p.start()) {
            jr::rt::TickGroup &g = p.g();
            const std::uint64_t before = missed(g);
            jr::Result r{};
            const Clock::time_point t0 = Clock::now();
            const jr::Status ps = g.pause(&r, 500u);
            const bool ok = (ps == jr::Status::kOk);
            const std::uint64_t paused_ms = elapsed_ms(t0);
            if (!ok) {
                JR_CHECK_MSG(false, "pause 握手失败");
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                g.resume();
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                const std::uint64_t after = missed(g);
                g.stop();
                JR_CHECK(within_window(after - before, paused_ms + 200u));
            }
        } else {
            JR_CHECK_MSG(false, "最小虚拟总线 tick 组起不来");
        }
    }

    return jrtest::report();
}
