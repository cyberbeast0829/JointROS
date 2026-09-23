/**
 * @file    jr_triple_buffer.hpp
 * @brief   无锁"最新值"交换原语（单写单读，三缓冲）—— 命令信箱与状态快照共用
 *
 * @par 为什么不用 mutex / 队列 / seqlock
 *  - `mutex`：RT 线程会被非 RT 线程阻塞（优先级反转）——设计禁则（§5.5）；
 *  - 队列：控制回路里"旧的命令/旧的状态"没有价值，积压只会增加延迟；
 *  - seqlock：读侧可能要重试，重试次数不确定（虽然通常 1 次就够），
 *    而且写侧连续写会让读侧反复失败 —— RT 路径要**确定的**工作量。
 *
 *  三缓冲的保证（v0.14 第二轮修订）：写者往**私有暂存**里填，`publish()` 在锁内
 *  一次拷贝交付；读者在**同一把锁**内拷贝 ⇒ 读者与写者在载荷访问上严格互斥，
 *  **可证明**拿不到半成品，也不需要重试。
 *
 *  ⚠ 名字里的 "triple" 是历史遗留：现在实际是"一份私有暂存 + 一份读者可见副本 + 一把
 *  有界自旋锁"（即双缓冲 + 锁）。保留类名是为了不动几十处调用点；语义以本节为准。
 *
 * @par 语义
 *  - 写入即可，无需等待；未发布前读者看不到；
 *  - 发布多次只保留**最后一次**（读侧可能跳过中间值，这正是我们要的"最新值"）；
 *  - `read()` 是**值语义**：把最新快照拷贝到调用方给出的 `out`，只在**确实有新数据**时写它
 *    （没有新数据时 `*out` 保持调用方上次拿到的副本 ⇒ "最后一份值"仍然可用）。 *
 * @warning **首次 publish() 之前没有任何有效数据**。`acquire()` 在那种情况下会返回
 *  读者自己的（未初始化）缓冲，且 `got_new == false`。要区分"还没有数据"与"没有新数据"，
 *  请先看 `has_value()`。（集成测试就是这样抓到过一次误用：读者把未初始化缓冲当成了
 *  真实数据 —— 在真实系统里它会表现成"上电瞬间状态全是 0 / 关节位置突然跳到 0"。）
 *
 * @par 线程约束（v0.14 修订）
 *  **一个**写线程 + **任意多个**读线程。
 *
 *  ⚠⚠ v0.14 实测踩过一串坑，值得记住（详见 §13.3）：
 *   ① 旧实现把 `read_`（读者私有槽位号）做成**共享**的 ⇒ 读者 B 的交换会把读者 A
 *      正在读的槽位**交还给写者** ⇒ 写者轮转进去 ⇒ 撕裂读（debug 下断言 abort /
 *      release 下静默错值；实测 4 读者 × 2400 次抓到 `torn=1`）；
 *   ② 改成"认领计数"后仍有窗口：读者可能拿着**过期的** `ready_` 快照去认领，
 *      而写者已经接手了那一格（先认领/先交付两种顺序都能复现 `torn`）。
 *  ⇒ 结论：**别自己发明无锁的 N 读者交换协议**；一次小拷贝 + 一把有界自旋锁就够了。
 *
 *  @warning `read()` 会**拷贝整个 T**。T 请保持小（快照是 KB 量级），否则写者的
 *  等待会变长；**大对象的便利读入口**请用 `TickGroup::acquire_snapshot()`（每线程暂存）。
 */

#ifndef JR_ROS2_RT_JR_TRIPLE_BUFFER_HPP
#define JR_ROS2_RT_JR_TRIPLE_BUFFER_HPP

#include <atomic>
#include <cassert>
#include <cstdint>
#include <thread>

namespace jr {
namespace rt {

template <typename T>
class TripleBuffer {
public:
    TripleBuffer() noexcept = default;

    TripleBuffer(const TripleBuffer &) = delete;
    TripleBuffer &operator=(const TripleBuffer &) = delete;

    /* ---------------- 写侧（一个线程） ---------------- */

    /**
     * 取私有暂存缓冲（内容未定义；**发布前对读者不可见**）。
     *
     * ⚠ 为什么写侧要有一份"私有暂存"：发布必须是一个**原子交付**。写者可能分很多段落力
     *   填一个大结构（例如 tick 里跨 SDK I/O 地填快照），中途不能被读者看到。
     *   所以：写进暂存 → `publish()` 在锁内一次拷贝交付 ⇒ 读者看到的永远是一份完整数据。
     */
    T &write_slot() noexcept
    {
        debug_claim_writer();
        return staging_;
    }

    /**
     * 发布（锁内交付）。
     *
     * ⚠ 这里持有 `payload_lock_` 并做**一次 T 拷贝**：代价是有界的（T 小 ⇒ 百纳秒级），
     *   换来的是"读者与写者在载荷访问上严格互斥 ⇒ 不存在撕裂读"这个**可证明**的保证。
     *   v0.14 实测教训：无锁的三缓冲在"多读者"下有很多微妙的窗口（先认领还是先交付、
     *   `ready_` 的可见性……），每种都能复现出 1/2400 量级的静默撕裂值；
     *   而 RT 侧真正不能接受的是**长**等待，不是一次小拷贝。
     */
    void publish() noexcept
    {
        debug_claim_writer();
        payload_lock_.acquire();
        live_ = staging_;
        fresh_.store(true, std::memory_order_release);
        payload_lock_.release();
        published_.fetch_add(1u, std::memory_order_release);
    }

    /** 是否**曾经**发布过（读侧用它区分"还没数据"与"没有新数据"）。 */
    bool has_value() const noexcept
    {
        return published_.load(std::memory_order_acquire) != 0u;
    }

    /** 已发布次数（诊断用；读者侧只读快照）。 */
    std::uint64_t publish_count() const noexcept
    {
        return published_.load(std::memory_order_acquire);
    }

    /** 便利入口：写 + 发布（w 为可调用对象，接收 T&）。 */
    template <typename Fn>
    void store(Fn &&fn) noexcept
    {
        fn(write_slot());
        publish();
    }

    /* ---------------- 读侧（另一个线程） ---------------- */

    /**
     * 取一份**私有副本**（值语义）。
     * @param out      目的地；**只在返回 true 时被写**（否则保持调用方上次的副本）
     * @param got_new  输出：本次是否拿到了**新发布**的数据
     * @return true = `*out` 是本次新数据的副本
     *
     * ⚠ 同一份数据只会被**一个**读者报成 `got_new == true`（新数据标志被第一个取走的读者清掉），
     *   其他读者这一轮返回 false 且 `*out` 不动 —— 对每个读者而言"最新值"仍然成立。
     *   这对 ROS 侧的"状态发布定时器 + 多个服务回调"是合适的：每份快照恰好被消费一次，
     *   而不需要所有读者都看见同一帧。
     */
    bool read(T *out, bool *got_new = nullptr) noexcept
    {
        debug_claim_reader();
        bool is_new = false;
        /* 载荷访问在锁内：与写者的 `publish()` 严格互斥 ⇒ 永远拿不到半成品。 */
        payload_lock_.acquire();
        if (fresh_.load(std::memory_order_acquire)) {
            is_new = true;
            if (out != nullptr) *out = live_;
            fresh_.store(false, std::memory_order_release);
        }
        payload_lock_.release();
        if (got_new != nullptr) *got_new = is_new;
        return is_new;
    }

    /**
     * 写侧便利：等待读者把当前已发布的数据取走（**仅供非 RT 路径**，例如测试/关停）。
     * @param max_spins 自旋上限，0 = 不限制
     * @return true = 已消费
     */
    bool wait_consumed(std::uint32_t max_spins = 0u) const noexcept
    {
        std::uint32_t n = 0u;
        for (;;) {
            if (!fresh_.load(std::memory_order_acquire)) return true;
            if (max_spins != 0u && ++n >= max_spins) return false;
        }
    }

private:
    static constexpr unsigned kYieldAfter = 64u;

    /**
     * 保护**载荷访问**的自旋锁（写者的 `publish()` 与读者的 `read()` 都拿它）。
     *
     * 为什么不是 `std::mutex`：读侧包含 `tick_source=controller_manager` 下由 CM 线程
     * 驱动的 `read()`；在那条路径上装 mutex 可能优先级反转且**不可界定**（§5.5 禁则的精神）。
     * 自旋锁的临界区只有一次 `T` 拷贝（无竞争时 1 次 `test_and_set`；竞争时自旋几十次后
     * `yield()` 让出），**上界明确** ⇒ RT 侧最坏只等一次拷贝的时间。
     */
    class SpinLock {
    public:
        void acquire() noexcept
        {
            unsigned spins = 0u;
            while (flag_.test_and_set(std::memory_order_acquire)) {
                if (++spins >= kYieldAfter) {
                    std::this_thread::yield();
                    spins = 0u;
                }
            }
        }
        void release() noexcept { flag_.clear(std::memory_order_release); }

    private:
        static constexpr unsigned kYieldAfter = 64u;
        std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
    };

    /*
     * 调试期守卫：本原语只允许**一个**写线程与**一个**读线程。
     * 误用（例如"订阅回调与 tick 线程同时写快照"）不会死锁，而是**静默错值**，
     * 所以这里主动在开发阶段把它變成硬失败（release 下完全无开销）。
     */
    void debug_claim_writer() noexcept
    {
#ifndef NDEBUG
        const std::thread::id me = std::this_thread::get_id();
        const std::thread::id cur = writer_tid_.load(std::memory_order_relaxed);
        if (cur == std::thread::id{}) {
            writer_tid_.store(me, std::memory_order_relaxed);
        } else {
            assert(cur == me && "TripleBuffer: only ONE writer thread is allowed");
        }
#endif
    }

    void debug_claim_reader() noexcept
    {
#ifndef NDEBUG
        /* v0.14：读侧允许多线程 ⇒ 这里不再要求"同一个线程 id"，只守**真正**的不变量：
           读线程不能是写线程（那意味着同一线程既当读者又当写者 ⇒ 槽位所有权混乱）。 */
        const std::thread::id me = std::this_thread::get_id();
        const std::thread::id w = writer_tid_.load(std::memory_order_relaxed);
        assert((w == std::thread::id{} || w != me) &&
               "TripleBuffer: the writer thread must not also read");
        (void)me;
#endif
    }

    T staging_ = {};   /**< 写侧私有暂存：只有写线程访问，读者永远看不到它 */
    T live_    = {};   /**< 读者可见的那一份（与 `staging_` 的交付在锁内完成） */
    std::atomic<bool> fresh_{false};           /**< 自上次 read() 以来有新发布 */
    std::atomic<std::uint64_t> published_{0u}; /**< 发布次数（0 = 还没有任何数据） */
    SpinLock payload_lock_ = {};

#ifndef NDEBUG
    std::atomic<std::thread::id> writer_tid_{};
#endif
};

}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_RT_JR_TRIPLE_BUFFER_HPP */
