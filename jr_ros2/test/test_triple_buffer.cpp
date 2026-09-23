/**
 * @file    test_triple_buffer.cpp
 * @brief   三缓冲（命令信箱/状态快照的底座）的单元与压力测试
 *
 * @par 这个测试必须"有牙齿"
 *  它的价值在于**能证伪**：如果哪天有人把三缓冲改成"两个缓冲 + 直接写就绪槽"
 *  或"发布前不清 NEW 位"，`撕裂读` 用例就会红 —— 而不是等现场出现"关节偶尔抖一下"。
 */

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "jr_ros2/rt/jr_triple_buffer.hpp"
#include "jr_test.hpp"

namespace {

struct Payload {
    std::uint32_t seq = 0u;
    std::uint32_t fill[16] = {};
    std::uint32_t checksum = 0u;

    void build(std::uint32_t s) noexcept
    {
        seq = s;
        std::uint32_t sum = s;
        for (unsigned i = 0u; i < 16u; ++i) {
            fill[i] = s + i;
            sum += fill[i];
        }
        checksum = sum;
    }
    bool consistent() const noexcept
    {
        std::uint32_t sum = seq;
        for (unsigned i = 0u; i < 16u; ++i) {
            if (fill[i] != seq + i) return false;
            sum += fill[i];
        }
        return sum == checksum;
    }
};

void test_basic_exchange()
{
    JR_CASE("基础交换：首次读无新数据，写后读拿到最新值，多次写只保留最后一份");

    jr::rt::TripleBuffer<Payload> tb;

    /* srcdoc 误注释修正：首次发布之前没有数据（读者不得把未初始化缓冲当数据）。 */
    Payload cur;
    bool got = true;
    JR_CHECK(!tb.has_value());
    JR_CHECK(!tb.read(&cur, &got));   /* 没有数据：既不写 out，也不报"新数据" */
    JR_CHECK(!got);

    tb.store([](Payload &s) { s.build(11u); });
    JR_CHECK(tb.has_value());
    JR_CHECK(tb.read(&cur, &got));
    JR_CHECK(got);
    JR_CHECK_EQ(cur.seq, 11u);
    JR_CHECK(cur.consistent());

    /* 连续发布三次、只读一次 → 必须拿到**最后**一份（覆盖式语义）。 */
    tb.store([](Payload &s) { s.build(12u); });
    tb.store([](Payload &s) { s.build(13u); });
    tb.store([](Payload &s) { s.build(14u); });
    JR_CHECK(tb.read(&cur, &got));
    JR_CHECK(got);
    JR_CHECK_EQ(cur.seq, 14u);

    /* 没有新发布时 got_new 必须为假（否则上层会把同一帧当成"新命令"重复计数），
       而且 `cur` 保持上次的副本不变（值语义：没有新数据就不动 out）。 */
    JR_CHECK(!tb.read(&cur, &got));
    JR_CHECK(!got);
    JR_CHECK_EQ(cur.seq, 14u);
}

void test_stress_no_torn_read()
{
    JR_CASE("压力：写线程 200k 次发布，读线程每次都必须看到**内部一致**的数据");

    jr::rt::TripleBuffer<Payload> tb;
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> writes{0};
    std::atomic<std::uint64_t> reads{0};
    std::atomic<std::uint64_t> torn{0};
    std::atomic<std::uint32_t> last_seq{0};
    std::atomic<std::uint64_t> regressions{0};
    std::atomic<std::uint64_t> skipped_no_value{0};
    std::atomic<std::uint64_t> reads_at_stop{0};

    /* ⭐ 会合（rendezvous）而不是"写侧跑固定次数"：
       写循环是紧循环，而 Windows 默认调度量子 ~15.6 ms —— 200k 次发布可以**在量子
       内全部跑完**，读线程一整个量子都拿不到时间片。实测（满载 20 核）
       `writes=200000 reads=0`，于是 `reads>1000` 假红。改成一个量子跨不过去的会合：
       写侧一直发到读侧取够 kMinReads 次为止（kMaxWrites 兑底，避免主机完全不给
       读机会时死循环），读侧取够后自然退出。 */
    constexpr std::uint64_t kMinReads = 1000u;
    constexpr std::uint64_t kMaxWrites = 20000000u;

    std::thread writer([&]() {
        std::uint32_t s = 1u;
        while (reads.load(std::memory_order_relaxed) < kMinReads &&
               writes.load(std::memory_order_relaxed) < kMaxWrites) {
            tb.store([s](Payload &slot) { slot.build(s); });
            ++s;   /* u32 绕回也无所谓：校验的是"同一次发布的内部一致"，不是序号本身 */
            writes.fetch_add(1u, std::memory_order_relaxed);
        }
        reads_at_stop.store(reads.load(std::memory_order_relaxed), std::memory_order_relaxed);
        stop.store(true, std::memory_order_release);
    });

    std::thread reader([&]() {
        unsigned dumped = 0u;
        /* 即使写侧先退出，也要取到 kMinReads 次（保证"读路径真的被测过"，
           否则 torn==0 是空洞的）。 */
        while (!stop.load(std::memory_order_acquire) ||
               reads.load(std::memory_order_relaxed) < kMinReads) {
            /* 首次发布前没有可比对的数据（这正是 has_value() 的用途）。 */
            if (!tb.has_value()) {
                skipped_no_value.fetch_add(1u, std::memory_order_relaxed);
                continue;
            }
            bool got = false;
            Payload cur;
            if (!tb.read(&cur, &got)) {
                /* 写侧已退出且没有新数据 ⇒ 再等也没用（防读不到时死循环）。 */
                if (stop.load(std::memory_order_acquire)) break;
                continue;
            }
            reads.fetch_add(1u, std::memory_order_relaxed);
            if (!cur.consistent()) {
                torn.fetch_add(1u, std::memory_order_relaxed);
                if (dumped < 5u) {
                    ++dumped;
                    std::printf("  TORN got_new=%d seq=%u fill[0]=%u fill[15]=%u checksum=%u\n",
                                got ? 1 : 0, cur.seq, cur.fill[0], cur.fill[15], cur.checksum);
                }
            }
            const std::uint32_t prev = last_seq.load(std::memory_order_relaxed);
            if (cur.seq < prev) regressions.fetch_add(1u, std::memory_order_relaxed);
            last_seq.store(cur.seq, std::memory_order_relaxed);
        }
    });

    writer.join();
    reader.join();

    std::printf("  writes=%llu reads=%llu (concurrent=%llu) torn=%llu regressions=%llu "
                "skipped(pre-publish)=%llu\n",
                (unsigned long long)writes.load(), (unsigned long long)reads.load(),
                (unsigned long long)reads_at_stop.load(), (unsigned long long)torn.load(),
                (unsigned long long)regressions.load(),
                (unsigned long long)skipped_no_value.load());
    JR_CHECK_EQ(torn.load(), 0u);
    JR_CHECK_EQ(regressions.load(), 0u);
    JR_CHECK(reads.load() >= kMinReads);   /* 会合设计保证成立；不成立=读路实现坏了 */
    /* 并发压力**质量**如实上报：主机没给读线程时间片时，"没撕裂"只覆盖到写侧停后的读。 */
    if (reads_at_stop.load() < kMinReads) {
        jrtest::note_env("writer 发了 %llu 次（上限 %llu）而读侧只拿到 %llu 次并发读取："
                         "主机没给第二个线程足够时间片，本轮**并发压力不足**"
                         "（torn/regression 不变量仍在通过）",
                         (unsigned long long)writes.load(), (unsigned long long)kMaxWrites,
                         (unsigned long long)reads_at_stop.load());
    }
}

/**
 * 多读者：**这是真实缺陷的回归用例**（v0.14）。
 *
 * 现场症状：`jr_bus` 节点用 `MultiThreadedExecutor(4)` 跑，状态发布定时器与阻塞服务回调
 * **并发**读同一个快照 ⇒ 两个读者线程用同一份 `read_` 槽位记账 ⇒ 彼此把对方的槽位交回
 * "就绪池" ⇒ 写者可能写进读者正在读的槽位 ⇒ **debug 下触发断言、release 下静默读到撕裂值**。
 *
 * 所以这里用 4 个读者线程做同一套内部一致性校验：修复前（读侧无串行化）它会红。
 * ⚠ v0.14 第二轮：光串行化"取槽位"**不过** —— 共享的 `read_` 会让读者 B 把读者 A
 *   正在读的槽位交还给写者（实测 4 读者 × 2400 次 → `torn=1`）。现在 `read()` 会
 *   **认领**槽位并在认领期间完成拷贝，写者写前必等认领归零 ⇒ 这个用例才能真正守住它。
 * ⚠ 注意这个用例在 `NDEBUG`（RelWithDebInfo）下也不会靠断言变红 —— 它靠的是**数据本身**。
 */
void test_multi_reader_no_torn()
{
    JR_CASE("多读者：4 个读者线程并发 read()，每个都必须看到内部一致的数据");

    jr::rt::TripleBuffer<Payload> tb;
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> writes{0};
    std::atomic<std::uint64_t> reads{0};
    std::atomic<std::uint64_t> torn{0};

    constexpr std::uint64_t kMinReadsPerReader = 500u;
    constexpr std::uint64_t kMaxWrites = 20000000u;
    constexpr unsigned      kReaders = 4u;

    std::thread writer([&]() {
        std::uint32_t s = 1u;
        while (reads.load(std::memory_order_relaxed) < kMinReadsPerReader * kReaders &&
               writes.load(std::memory_order_relaxed) < kMaxWrites) {
            tb.store([s](Payload &slot) { slot.build(s); });
            ++s;
            writes.fetch_add(1u, std::memory_order_relaxed);
        }
        stop.store(true, std::memory_order_release);
    });

    std::vector<std::thread> readers;
    readers.reserve(kReaders);
    for (unsigned r = 0u; r < kReaders; ++r) {
        readers.emplace_back([&]() {
            std::uint64_t mine = 0u;
            while (!stop.load(std::memory_order_acquire) || mine < kMinReadsPerReader) {
                Payload cur;
                bool got = false;
                if (!tb.read(&cur, &got)) {
                    /* 写侧已退出且没有新数据 ⇒ 退出（否则"只数成功读"会死循环）。 */
                    if (stop.load(std::memory_order_acquire)) break;
                    continue;
                }
                ++mine;
                reads.fetch_add(1u, std::memory_order_relaxed);
                if (!cur.consistent()) {
                    torn.fetch_add(1u, std::memory_order_relaxed);
                }
                if (stop.load(std::memory_order_acquire) && mine >= kMinReadsPerReader) break;
            }
        });
    }

    writer.join();
    for (auto &t : readers) t.join();

    std::printf("  multi-reader: writes=%llu reads=%llu torn=%llu\n",
                (unsigned long long)writes.load(), (unsigned long long)reads.load(),
                (unsigned long long)torn.load());
    JR_CHECK_EQ(torn.load(), 0u);
    JR_CHECK(reads.load() >= kMinReadsPerReader * kReaders);
}

void test_wait_consumed()
{
    JR_CASE("wait_consumed：写侧能看到'读者还没取走'，取走后立刻返回");

    jr::rt::TripleBuffer<Payload> tb;
    tb.store([](Payload &s) { s.build(1u); });
    JR_CHECK(!tb.wait_consumed(50u));   /* 读者没取 → 自旋耗尽 */

    bool got = false;
    Payload cur;
    JR_CHECK(tb.read(&cur, &got));
    JR_CHECK(got);
    JR_CHECK(tb.wait_consumed(50u));
}

}  // namespace

int main()
{
    test_basic_exchange();
    test_stress_no_torn_read();
    test_multi_reader_no_torn();
    test_wait_consumed();
    return jrtest::report();
}
