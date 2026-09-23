/**
 * @file    test_spsc_ring.cpp
 * @brief   SPSC 环（故障事件）测试：FIFO、绕回、**满时丢新并计数**、并发收发
 */

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "jr_ros2/rt/jr_spsc_ring.hpp"
#include "jr_test.hpp"

namespace {

struct Ev {
    std::uint32_t id = 0u;
    std::uint64_t t = 0u;
};

void test_fifo_and_wrap()
{
    JR_CASE("FIFO 与绕回：顺序不乱、容量上限正确、满时丢新且计数");
    jr::rt::SpscRing<Ev, 4u> ring;

    for (std::uint32_t i = 1u; i <= 4u; ++i) {
        Ev e;
        e.id = i;
        JR_CHECK(ring.push(e));
    }
    /* 满：第 5 条必须被丢（不是覆盖最早那条 —— 现场根因往往是第一条）。 */
    Ev e5;
    e5.id = 5u;
    JR_CHECK(!ring.push(e5));
    JR_CHECK_EQ(ring.dropped(), 1u);

    Ev out;
    for (std::uint32_t i = 1u; i <= 4u; ++i) {
        JR_CHECK(ring.pop(&out));
        JR_CHECK_EQ(out.id, i);
    }
    JR_CHECK(!ring.pop(&out));

    /* 空后可以再写（绕回），且 dropped 不再增长。 */
    Ev e6;
    e6.id = 6u;
    JR_CHECK(ring.push(e6));
    JR_CHECK(ring.pop(&out));
    JR_CHECK_EQ(out.id, 6u);
    JR_CHECK_EQ(ring.dropped(), 1u);
}

void test_concurrent()
{
    JR_CASE("并发：1 生产者 + 1 消费者；序号严格递增，且'断档数 == 丢弃数'");
    jr::rt::SpscRing<Ev, 64u> ring;

    constexpr std::uint32_t kCount = 200000u;
    std::atomic<bool>        produced_done{false};
    std::atomic<std::uint32_t> last_id{0u};
    std::atomic<std::uint64_t> got{0};
    std::atomic<std::uint64_t> gaps{0};
    std::atomic<std::uint64_t> order_errors{0};

    std::thread producer([&]() {
        for (std::uint32_t i = 1u; i <= kCount; ++i) {
            Ev e;
            e.id = i;
            (void)ring.push(e);   /* 允许丢（会计数） */
        }
        produced_done.store(true, std::memory_order_release);
    });

    std::thread consumer([&]() {
        Ev out;
        for (;;) {
            if (ring.pop(&out)) {
                const std::uint32_t prev = last_id.load(std::memory_order_relaxed);
                if (out.id <= prev) order_errors.fetch_add(1u, std::memory_order_relaxed);
                /* 每个被丢弃的条目会在接收序列里留下**恰好一个**断档 —— 这是个强不变量，
                   比"序号连续"更能证明环本身没弄丢/弄重数据。 */
                gaps.fetch_add((out.id > prev + 1u) ? (out.id - prev - 1u) : 0u,
                               std::memory_order_relaxed);
                last_id.store(out.id, std::memory_order_relaxed);
                got.fetch_add(1u, std::memory_order_relaxed);
                continue;
            }
            if (produced_done.load(std::memory_order_acquire) && ring.empty()) break;
            std::this_thread::yield();
        }
    });

    producer.join();
    consumer.join();

    const std::uint64_t produced = kCount;
    const std::uint64_t max_received = last_id.load();
    std::printf("  produced=%llu consumed=%llu dropped=%llu gaps=%llu tail=%llu order_errors=%llu\n",
                (unsigned long long)produced, (unsigned long long)got.load(),
                (unsigned long long)ring.dropped(), (unsigned long long)gaps.load(),
                (unsigned long long)(produced - max_received), (unsigned long long)order_errors.load());
    JR_CHECK_EQ(order_errors.load(), 0u);
    /* 收到的 + 丢掉的 = 总数（不会凭空出现也不会凭空消失）。 */
    JR_CHECK(got.load() + ring.dropped() == produced);
    /* 精确记账：丢掉的 = 中间断档 + 尾部（最后几个被丢的不会形成"断档"）。
       这个等价式比"断档数 == 丢弃数"更强：它把尾段也算进去了。 */
    JR_CHECK_EQ(ring.dropped(), gaps.load() + (produced - max_received));
    JR_CHECK(got.load() > 0u);
}

}  // namespace

int main()
{
    test_fifo_and_wrap();
    test_concurrent();
    return jrtest::report();
}
