/**
 * @file    jr_test.hpp
 * @brief   极简测试骨架（**不依赖 gtest**）
 *
 * @par 为什么不用 gtest
 *  本仓库的核心库必须能在"只有编译器 + CMake"的机器上（Windows MinGW、客户的
 *  交叉编译环境、CI 容器）验证。gtest 会引入额外依赖与版本差异，而我们要的只是
 *  "断言 + 计数 + 非零退出码"。CTest 与 colcon test 都能直接跑。
 */

#ifndef JR_ROS2_TEST_JR_TEST_HPP
#define JR_ROS2_TEST_JR_TEST_HPP

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace jrtest {

inline int  checks = 0;
inline int  failures = 0;
/** 因**主机环境**而无法判决的检查数（与 failures 分开：它不是代码问题，但必须可见）。 */
inline int  env_skips = 0;
inline const char *current_case = "";

inline void begin(const char *name)
{
    current_case = name;
    std::printf("--- %s\n", name);
    std::fflush(stdout);
}

/** 报告一条"本轮无法判决"的检查（主机被抢占/过载导致前提不成立）。
 *
 * 为什么要有这个：本仓库有多处"睡了 N 毫秒后断言进度"的检查，在主机被抢占时
 * 会**假红**（实测：满载 20 核时相邻线程断言读到 0 次时间片）。既不希望假红，
 * 也不希望静静跳过 —— 所以单独计数、并在摘要里显式打印，永远不用失败来掩盖。 */
inline void note_env(const char *fmt, ...)
{
    char buf[512] = {};
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    ++env_skips;
    std::printf("[ENV] %s\n", buf);
    std::fflush(stdout);
}

/** 主机调度响应性探针：5 次 sleep(1ms) 的理论耗时≈5..10 ms。
 *  实测远大于此 = 本进程正在被抢占 → 本轮**所有时间敏感结论都不是证据**。 */
inline long host_pressure_ms()
{
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 5; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto t1 = std::chrono::steady_clock::now();
    return static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count());
}

inline void check(bool ok, const char *expr, const char *file, int line, const char *extra)
{
    ++checks;
    if (ok) return;
    ++failures;
    std::printf("FAIL [%s] %s:%d: %s%s%s\n", current_case, file, line, expr, extra ? " -- " : "",
                extra ? extra : "");
    /* ⚠ 必须立即 flush：测试输出重定向到文件/管道时 stdout 是**全缓冲**的，
       一旦后面段错误，缓冲区里的失败原因就跟着进程一起没了
       （实测：ctest 只报 "1 failure"，现场看不到是哪条断言、为什么）。 */
    std::fflush(stdout);
}

inline int report()
{
    const long pressure = host_pressure_ms();
    std::printf("\n[HOST] 5x sleep(1ms) took %ld ms%s\n", pressure,
                (pressure > 100l) ? "  <-- 主机被严重抢占：本轮时间敏感结论不可作为证据" : "");
    std::printf("%s: %d check(s), %d failure(s)\n", failures == 0 ? "PASS" : "FAIL", checks,
                failures);
    if (env_skips > 0) {
        std::printf("       %d env-skip(s) (主机原因无法判决，不是代码问题)\n", env_skips);
    }
    std::fflush(stdout);
    return failures == 0 ? 0 : 1;
}

}  // namespace jrtest

#define JR_CHECK(expr) ::jrtest::check((expr), #expr, __FILE__, __LINE__, nullptr)
#define JR_CHECK_MSG(expr, msg) ::jrtest::check((expr), #expr, __FILE__, __LINE__, (msg))
#define JR_CASE(name) ::jrtest::begin(name)

#define JR_CHECK_EQ(a, b)                                                                          \
    do {                                                                                           \
        const auto _a = (a);                                                                       \
        const auto _b = (b);                                                                       \
        if (!(_a == _b)) {                                                                          \
            char _buf[160];                                                                        \
            std::snprintf(_buf, sizeof(_buf), "got %.17g, expected %.17g", (double)_a, (double)_b); \
            ::jrtest::check(false, #a " == " #b, __FILE__, __LINE__, _buf);                        \
        } else {                                                                                   \
            ::jrtest::check(true, #a " == " #b, __FILE__, __LINE__, nullptr);                      \
        }                                                                                          \
    } while (0)

/** 浮点区间断言（模型类断言用它，避免把"估计值"写成精确值）。 */
#define JR_CHECK_IN(a, lo, hi)                                                                    \
    do {                                                                                           \
        const double _v = (double)(a);                                                             \
        if (!(_v >= (lo) && _v <= (hi))) {                                                         \
            char _buf[160];                                                                        \
            std::snprintf(_buf, sizeof(_buf), "value %.3f outside [%.3f, %.3f]", _v, (double)(lo),  \
                          (double)(hi));                                                           \
            ::jrtest::check(false, #a " in range", __FILE__, __LINE__, _buf);                      \
        } else {                                                                                   \
            ::jrtest::check(true, #a " in range", __FILE__, __LINE__, nullptr);                    \
        }                                                                                          \
    } while (0)

/** 字符串包含（错误信息必须"可操作"，所以测试要盯措辞里的关键信息）。 */
#define JR_CHECK_CONTAINS(haystack, needle)                                                       \
    do {                                                                                           \
        const char *_h = (haystack);                                                               \
        const char *_n = (needle);                                                                 \
        ::jrtest::check(_h != nullptr && std::strstr(_h, _n) != nullptr,                           \
                        "message contains \"" needle "\"", __FILE__, __LINE__,                     \
                        _h != nullptr ? _h : "(null)");                                            \
    } while (0)

#endif /* JR_ROS2_TEST_JR_TEST_HPP */
