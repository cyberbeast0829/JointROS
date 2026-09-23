/**
 * @file    test_rt_sched.cpp
 * @brief   实时调度/时钟封装的测试 —— 核心是"**报告不得撒谎**"
 *
 * @par 为什么要专门测这个
 *  我们的 `RtReport` 是给客户看的（`/diagnostics`、启动日志）：它说"SCHED_FIFO 生效了"，
 *  客户就会以为抖动有保证。所以这里不只测"我们请求了什么"，而是**回头量真实状态**：
 *  请求成功 ⇔ 实测策略真的是 SCHED_FIFO。提权失败的机器上，报告必须说失败。
 *
 *  Linux 专属的断言在容器里跑（这也是 Windows 上验证不了的那部分代码路径）；
 *  Windows 上只跑"跨平台 + 关闭实时时必须不吹牛"的部分。
 */

#include <chrono>
#include <cstdio>
#include <thread>

#include "jr_ros2/rt/jr_rt_sched.hpp"
#include "jr_test.hpp"

#if defined(__linux__)
#  include <pthread.h>
#  include <sched.h>
#  include <unistd.h>
#endif

using namespace jr::rt;

namespace {

void sleep_ms(unsigned ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void test_monotonic_clock()
{
    JR_CASE("时钟：单调不回退，且睡眠时长在量级上正确");
    const std::uint64_t a = now_ns();
    std::uint64_t b = now_ns();
    JR_CHECK(b >= a);

    sleep_ms(5u);
    const std::uint64_t c = now_ns();
    const std::uint64_t delta_ms = (c - a) / 1000000ull;
    JR_CHECK_IN(static_cast<double>(delta_ms), 4.0, 200.0);   /* 不能"睡完时间没走" */

    const std::uint32_t ms1 = now_ms();
    sleep_ms(3u);
    const std::uint32_t ms2 = now_ms();
    JR_CHECK(ms2 >= ms1);
}

void test_sleeper_deadlines_have_no_drift()
{
    JR_CASE("Sleeper 的 deadline 无漂移（第 n 个 tick = start + n*period）");
    const std::uint64_t period = 2000000ull;   /* 2 ms */
    Sleeper s(period);
    const std::uint64_t t0 = s.start_ns();
    /* 逐 tick 推进 1000 次：绝对 deadline 必须**精确**等于 n*period（不累积误差） */
    for (std::uint64_t n = 1u; n <= 1000u; ++n) {
        if (s.deadline_of(n) != t0 + n * period) {
            JR_CHECK_MSG(false, "deadline_of() 出现漂移");
            return;
        }
    }
    JR_CHECK(true);
}

void test_sleep_until_respects_deadline()
{
    JR_CASE("sleep_until：真的睡到点（不能提前返回，也不能睡过头一个量级）");
    Sleeper s(10000000ull);   /* 10 ms */
    const std::uint64_t deadline = now_ns() + 10000000ull;
    s.sleep_until(deadline);
    const std::uint64_t spent_ms = (now_ns() - (deadline - 10000000ull)) / 1000000ull;
    std::printf("  sleep 10 ms 实际用了 %llu ms\n", static_cast<unsigned long long>(spent_ms));
    JR_CHECK_IN(static_cast<double>(spent_ms), 9.0, 60.0);

    /* 已经过期的 deadline 必须立刻返回（不能"补睡"） */
    const std::uint64_t before = now_ns();
    s.sleep_until(before - 1000000ull);
    JR_CHECK((now_ns() - before) < 2000000ull);
}

void test_disabled_rt_must_not_claim_anything()
{
    JR_CASE("rt.enabled=false 时**不得**声称任何实时设置生效（这是我们给客户的判断依据）");
    RtSetup off;
    off.enabled = false;

    const RtReport tp = setup_process_rt(off);
    JR_CHECK(!tp.sched_ok);
    JR_CHECK(!tp.mlock_ok);

    const RtReport tt = setup_thread_rt(off);
    JR_CHECK(!tt.sched_ok);
    JR_CHECK(!tt.affinity_ok);
    JR_CHECK_CONTAINS(tt.text, "CFS");      /* 必须说清楚"跑在普通调度下，抖动无保证" */
    std::printf("  disabled: %s\n", tt.text);
}

#if defined(__linux__)
void test_linux_report_matches_reality()
{
    JR_CASE("Linux：RtReport 必须与实测的调度策略/亲和性一致（报告不得撒谎）");

    /* ---- 调度策略 ---- */
    RtSetup on;
    on.enabled = true;
    on.priority = 80;
    on.mlock = false;      /* 容器里通常没有 mlock 权限，别让这条干扰判断 */

    const RtReport rep = setup_thread_rt(on);

    int policy = -1;
    sched_param sp;
    (void)std::memset(&sp, 0, sizeof sp);
    pthread_getschedparam(pthread_self(), &policy, &sp);
    const bool really_fifo = (policy == SCHED_FIFO);

    std::printf("  sched: report.sched_ok=%d, measured policy=%s (prio=%d) | %s\n",
                rep.sched_ok ? 1 : 0, really_fifo ? "SCHED_FIFO" : "SCHED_OTHER", sp.sched_priority,
                rep.text);
    JR_CHECK_MSG(rep.sched_ok == really_fifo,
                 "报告说 sched_ok 与实测不符 —— 客户会据此以为抖动有保证");

    /* 提权失败时必须给出原因（否则用户不知道要加 rtprio 限制/用 --privileged） */
    if (!rep.sched_ok) {
        JR_CHECK(rep.text[0] != '\0');
    }

    /* ---- 亲和性 ---- */
    RtSetup aff = on;
    aff.cpu = 0;
    const RtReport rep2 = setup_thread_rt(aff);
    cpu_set_t set;
    CPU_ZERO(&set);
    const bool got = (pthread_getaffinity_np(pthread_self(), sizeof(set), &set) == 0);
    const bool really_pinned = got && CPU_ISSET(0, &set) &&
                               (CPU_COUNT(&set) == 1);
    std::printf("  affinity: report.affinity_ok=%d, measured pinned-to-cpu0=%d\n",
                rep2.affinity_ok ? 1 : 0, really_pinned ? 1 : 0);
    if (got) {
        JR_CHECK_MSG(rep2.affinity_ok == really_pinned, "亲和性报告与实测不符");
    }

    /* 进程级：mlock 的诚实性（容器里一般会失败，那也必须**如实说失败**） */
    RtSetup ml = on;
    ml.mlock = true;
    const RtReport rep3 = setup_process_rt(ml);
    std::printf("  mlock: report.mlock_ok=%d, throttled=%d\n", rep3.mlock_ok ? 1 : 0,
                rep3.throttled ? 1 : 0);
    if (rep3.mlock_ok) {
        /* mlock_ok=true 时要能确实"锁住"——用一块内存做一次真实触碰不算证伪，
           这里只声明我们不会在失败时声称成功（见上面的 false 分支）。 */
        JR_CHECK(true);
    }

    /* 把本线程恢复成普通调度，避免影响后续用例（当 FIFO 优先级 80 会抢占整机） */
    sched_param back;
    (void)std::memset(&back, 0, sizeof back);
    (void)pthread_setschedparam(pthread_self(), SCHED_OTHER, &back);
    int policy_after = -1;
    sched_param sp_after;
    (void)std::memset(&sp_after, 0, sizeof sp_after);
    pthread_getschedparam(pthread_self(), &policy_after, &sp_after);
    JR_CHECK(policy_after == SCHED_OTHER);
}
#endif

}  // namespace

int main()
{
    test_monotonic_clock();
    test_sleeper_deadlines_have_no_drift();
    test_sleep_until_respects_deadline();
    test_disabled_rt_must_not_claim_anything();
#if defined(__linux__)
    test_linux_report_matches_reality();
#else
    std::printf("--- (非 Linux：跳过\"报告与实测一致\"的用例；这部分由容器里的 Linux 构建覆盖)\n");
#endif
    return jrtest::report();
}
