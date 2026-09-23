/**
 * @file    jr_spsc_ring.hpp
 * @brief   无锁单生产者单消费者环（故障事件/日志用）；满时**丢新**并计数
 *
 * @par 为什么"丢新"而不是"丢旧"
 *  环里装的是**故障边沿事件**：宁可丢掉最新的若干条（计数器会暴露"丢了多少"），
 *  也不要覆盖掉"最早发生的那一条" —— 现场根因几乎总是第一条。
 *  非 RT 侧消费时会先看到丢帧计数（`dropped()`），不会静默。
 */

#ifndef JR_ROS2_RT_JR_SPSC_RING_HPP
#define JR_ROS2_RT_JR_SPSC_RING_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace jr {
namespace rt {

template <typename T, std::size_t N>
class SpscRing {
    static_assert(N >= 2u, "SpscRing: 容量至少 2");

public:
    /** 生产者（RT 或非 RT 线程之一）。满时丢弃并计数。@return true = 已入队 */
    bool push(const T &v) noexcept
    {
        const std::uint64_t w = write_.load(std::memory_order_relaxed);
        const std::uint64_t r = read_cache_;
        if (w - r >= N) {
            /* 重新读一次真实的读指针：可能消费者刚好取走了 */
            read_cache_ = read_.load(std::memory_order_acquire);
            if (w - read_cache_ >= N) {
                dropped_.fetch_add(1u, std::memory_order_relaxed);
                return false;
            }
        }
        buf_[static_cast<std::size_t>(w % N)] = v;
        write_.store(w + 1u, std::memory_order_release);
        return true;
    }

    /** 消费者。@return true = 取到一条 */
    bool pop(T *out) noexcept
    {
        const std::uint64_t r = read_.load(std::memory_order_relaxed);
        const std::uint64_t w = write_cache_.load(std::memory_order_acquire);
        if (r == w) {
            write_cache_ = write_.load(std::memory_order_acquire);
            if (r == write_cache_) return false;
        }
        if (out != nullptr) *out = buf_[static_cast<std::size_t>(r % N)];
        read_.store(r + 1u, std::memory_order_release);
        return true;
    }

    /** 已丢弃条数（满环时），给诊断用。 */
    std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

    bool empty() const noexcept
    {
        return read_.load(std::memory_order_acquire) == write_.load(std::memory_order_acquire);
    }

    constexpr std::size_t capacity() const noexcept { return N; }

private:
    T buf_[N] = {};
    std::atomic<std::uint64_t> write_{0u};
    std::atomic<std::uint64_t> read_{0u};
    std::atomic<std::uint64_t> dropped_{0u};
    /* 缓存对侧指针，减少 RT 侧的原子读（各自只被自己更新） */
    std::uint64_t read_cache_ = 0u;
    std::atomic<std::uint64_t> write_cache_{0u};
};

}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_RT_JR_SPSC_RING_HPP */
