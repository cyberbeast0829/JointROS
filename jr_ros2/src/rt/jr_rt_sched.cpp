/**
 * @file    jr_rt_sched.cpp
 * @brief   实时调度与时钟实现
 */

#include "jr_ros2/rt/jr_rt_sched.hpp"

#include <cstdio>
#include <cstring>

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <cerrno>
#  include <ctime>
#  include <pthread.h>
#  include <sched.h>
#  include <unistd.h>
#  if defined(__linux__)
#    include <sys/mman.h>
#  endif
#endif

namespace jr {
namespace rt {

std::uint64_t now_ns() noexcept
{
#if defined(_WIN32)
    static const std::uint64_t freq = []() -> std::uint64_t {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        return (f.QuadPart > 0) ? static_cast<std::uint64_t>(f.QuadPart) : 1u;
    }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    const std::uint64_t ticks = static_cast<std::uint64_t>(c.QuadPart);
    /* 拆成商与余数，避免 ticks*freq 溢出（QPC 频率常见 10 MHz，1e9 相乘会溢出 64 位）。 */
    return (ticks / freq) * 1000000000ull + ((ticks % freq) * 1000000000ull) / freq;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull +
           static_cast<std::uint64_t>(ts.tv_nsec);
#endif
}

std::uint32_t now_ms() noexcept
{
    return static_cast<std::uint32_t>(now_ns() / 1000000ull);
}

RtReport setup_process_rt(const RtSetup &setup) noexcept
{
    RtReport rep;
    if (!setup.enabled) {
        std::snprintf(rep.text, sizeof(rep.text),
                      "rt.enabled=false: running in CFS (jitter NOT bounded)");
        return rep;
    }

#if defined(_WIN32)
    if (setup.mlock) {
        /* Windows 没有 mlockall 的等价物；用"最小工作集=最大工作集"减少换页。
           这里**明确**报告 false（不假装成功）。 */
        rep.mlock_ok = false;
    }
    rep.text[0] = '\0';
    return rep;
#else
#  if defined(__linux__)
    if (setup.mlock) {
        rep.mlock_ok = (mlockall(MCL_CURRENT | MCL_FUTURE) == 0);
    }
#  else
    rep.mlock_ok = false;
#  endif

#  if defined(__linux__)
    if (setup.warn_if_throttled) {
        std::FILE *f = std::fopen("/proc/sys/kernel/sched_rt_runtime_us", "r");
        if (f != nullptr) {
            long rt_us = -1;
            if (std::fscanf(f, "%ld", &rt_us) == 1) {
                /* 默认 950000 µs / 1000000 µs 周期 ⇒ 实时任务每秒只能跑 95%；
                   一旦跑满会被内核强制歇到下一周期（表现为周期性巨大抖动）。
                   设为 -1（无限）才不会被节流。 */
                rep.throttled = (rt_us != -1);
            }
            std::fclose(f);
        }
    }
#  endif
    return rep;
#endif
}

RtReport setup_thread_rt(const RtSetup &setup) noexcept
{
    RtReport rep;
    if (!setup.enabled) {
        std::snprintf(rep.text, sizeof(rep.text),
                      "rt.enabled=false: running in CFS (jitter NOT bounded)");
        return rep;
    }

#if defined(_WIN32)
    /* 线程优先级：TIME_CRITICAL（需进程优先级配合；失败不致命，报告实际结果）。 */
    if (SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) != 0) {
        rep.sched_ok = true;
        rep.applied_priority = static_cast<int>(THREAD_PRIORITY_TIME_CRITICAL);
    }
    if (setup.cpu >= 0) {
        const DWORD_PTR mask = (sizeof(DWORD_PTR) >= 8)
                                   ? (static_cast<DWORD_PTR>(1) << setup.cpu)
                                   : (static_cast<DWORD_PTR>(1) << (setup.cpu & 31));
        if (SetThreadAffinityMask(GetCurrentThread(), mask) != 0) {
            rep.affinity_ok = true;
            rep.applied_cpu = setup.cpu;
        }
    }
    std::snprintf(rep.text, sizeof(rep.text),
                  "windows: sched_ok=%d affinity_ok=%d (dev only, jitter targets are NOT met "
                  "here; use PREEMPT_RT Linux for production)",
                  rep.sched_ok ? 1 : 0, rep.affinity_ok ? 1 : 0);
    return rep;
#else
    sched_param param;
    std::memset(&param, 0, sizeof(param));
    param.sched_priority = setup.priority;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (rc == 0) {
        rep.sched_ok = true;
        rep.applied_priority = setup.priority;
    } else {
        std::snprintf(rep.text, sizeof(rep.text),
                      "SCHED_FIFO(prio=%d) failed: %s (need CAP_SYS_NICE / root, or a "
                      "rtprio limit in /etc/security/limits.conf)",
                      setup.priority, std::strerror(rc));
    }

    if (setup.cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(setup.cpu, &set);
        if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0) {
            rep.affinity_ok = true;
            rep.applied_cpu = setup.cpu;
        } else if (rep.text[0] == '\0') {
            std::snprintf(rep.text, sizeof(rep.text), "pthread_setaffinity_np(cpu=%d) failed",
                          setup.cpu);
        }
    }

    if (rep.sched_ok && rep.text[0] == '\0') {
        std::snprintf(rep.text, sizeof(rep.text), "SCHED_FIFO prio=%d, cpu=%d",
                      rep.applied_priority, rep.applied_cpu);
    }
    return rep;
#endif
}

Sleeper::Sleeper(std::uint64_t period_ns) noexcept
    : period_ns_(period_ns), start_ns_(now_ns())
{
#if defined(_WIN32)
    /* 高精度可等待定时器（Win10 1803+）；失败则退化为 Sleep 循环。 */
    HANDLE h = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                      TIMER_ALL_ACCESS);
    timer_ = reinterpret_cast<void *>(h);
#endif
}

Sleeper::~Sleeper()
{
#if defined(_WIN32)
    if (timer_ != nullptr) CloseHandle(reinterpret_cast<HANDLE>(timer_));
#endif
}

void Sleeper::sleep_until(std::uint64_t deadline_ns) noexcept
{
#if defined(_WIN32)
    const std::uint64_t kSpinNs = 200000ull;   /* 最后 200 µs 自旋，换更小的唤醒误差 */
    for (;;) {
        const std::uint64_t now = now_ns();
        if (now >= deadline_ns) return;
        const std::uint64_t remain = deadline_ns - now;
        if (remain <= kSpinNs) {
            YieldProcessor();
            continue;
        }
        const std::uint64_t wait_ns = remain - kSpinNs;
        if (timer_ != nullptr) {
            LARGE_INTEGER due;
            /* 负数 = 相对时间（100 ns 单位） */
            due.QuadPart = -static_cast<LONGLONG>(wait_ns / 100ull);
            if (due.QuadPart == 0) due.QuadPart = -1;
            if (SetWaitableTimer(reinterpret_cast<HANDLE>(timer_), &due, 0, nullptr, nullptr,
                                 FALSE) != 0) {
                WaitForSingleObject(reinterpret_cast<HANDLE>(timer_),
                                    static_cast<DWORD>(wait_ns / 1000000ull) + 2u);
                continue;
            }
        }
        Sleep(static_cast<DWORD>(wait_ns / 1000000ull));
    }
#else
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(deadline_ns / 1000000000ull);
    ts.tv_nsec = static_cast<long>(deadline_ns % 1000000000ull);
    /* TIMER_ABSTIME：内核按绝对时刻唤醒，不累积漂移。EINTR 时重新进入即可（仍是绝对时刻）。 */
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
        /* 继续等到同一绝对时刻 */
    }
#endif
}

}  // namespace rt
}  // namespace jr
