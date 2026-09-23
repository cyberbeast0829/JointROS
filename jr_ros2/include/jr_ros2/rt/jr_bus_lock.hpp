/**
 * @file    jr_bus_lock.hpp
 * @brief   单 master 纪律：CAN 通道的跨进程排他锁（DESIGN ADR-9）
 *
 * @par 为什么必须有
 *  协议里设备"记住最后见到的 master_id"。同一条总线上跑两个主站（例如我们的节点 +
 *  `jsdk-cli`）时症状是：心跳乱跳、参数偶发写不进、控制断续 —— 现场几乎无法定位，
 *  而根因只是"多开了一个进程"。所以默认**拒绝启动**，并把持有者 PID 打印出来。
 *
 * @warning 锁只能防**本机**。跨机（两台 PC 接同一条 CAN）无法拦住 —— 靠文档（DESIGN §8.7）。
 */

#ifndef JR_ROS2_RT_JR_BUS_LOCK_HPP
#define JR_ROS2_RT_JR_BUS_LOCK_HPP

#include <cstddef>

#include "jr_ros2/jr_status.hpp"

namespace jr {

class BusLock {
public:
    BusLock() noexcept = default;
    ~BusLock();

    BusLock(const BusLock &) = delete;
    BusLock &operator=(const BusLock &) = delete;

    /**
     * 取排他锁。
     * @param bus_name  用于组成锁文件名（建议用 CAN 通道名，如 "can0"）
     * @param lock_dir  可为空 → 平台默认（Linux:/var/lock，Windows:%LOCALAPPDATA%）
     * @param allow_shared true = 只记录不阻塞（显式并行；DESIGN 要求此时诊断持续告警）
     * @param msg/msg_len 失败时写"谁持有锁/怎么办"
     */
    Status acquire(const char *bus_name, const char *lock_dir, bool allow_shared, char *msg,
                   std::size_t msg_len) noexcept;

    void release() noexcept;

    bool held() const noexcept { return held_; }
    bool shared_mode() const noexcept { return shared_mode_; }

    /** 锁文件里记录的持有者 PID（读不到返回 -1）。 */
    int owner_pid_on_disk() const noexcept;

    const char *path() const noexcept { return path_; }

    /** 持有人 PID 的伴随文件路径（POSIX/Windows 都存在；内容形如 "pid 1234"）。 */
    const char *owner_path() const noexcept { return owner_path_; }

private:
    void *handle_ = nullptr;    /**< Windows: HANDLE；POSIX: fd */
    char  path_[320] = {};      /**< 加锁对象（**不写 PID**，见 .cpp 里的 Windows 实测说明） */
    char  owner_path_[320] = {};/**< 写 PID 的伴随文件（**不加锁**，否则失败方读不到） */
    bool  held_ = false;
    bool  shared_mode_ = false;
};

}  // namespace jr

#endif /* JR_ROS2_RT_JR_BUS_LOCK_HPP */
