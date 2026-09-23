/**
 * @file    jr_snapshot.hpp
 * @brief   状态快照的载荷（POD，定长，无分配）—— RT 域写、ROS 域读
 *
 * 说明：这里刻意**不**用 ROS 消息类型。RT 路径写 POD，ROS 域再把 POD 投影成
 * `jr_interfaces/msg/...`。这样 RT 线程完全不依赖 rosidl（否则一个消息定义变化
 * 就可能改变 RT 侧的内存布局与代码路径）。
 */

#ifndef JR_ROS2_JR_SNAPSHOT_HPP
#define JR_ROS2_JR_SNAPSHOT_HPP

#include <cstdint>

#include "jr_ros2/jr_config.hpp"

namespace jr {

/** 逐关节状态（POD）。`status_flags` 位定义见 SDK `JSDK_JF_*`。 */
struct JointStatePOD {
    char          name[kJointNameLen] = {};
    std::uint8_t  bus_index = 0u;
    std::uint8_t  node_id = 0u;
    std::uint8_t  err_code = 0u;        /**< MIT 4-bit 摘要 */
    std::uint8_t  hb_error = 0u;        /**< 心跳 5-bit 子系统位图（原始位，优于摘要） */
    std::uint16_t mode_state = 0u;      /**< 固件原始 nibble */
    std::uint16_t status_flags = 0u;    /**< SDK JSDK_JF_* */
    std::uint32_t axis_error = 0u;      /**< 32-bit 子系统错误位（0 = 未查询或无错） */
    std::uint32_t age_ms = 0u;          /**< 距上次有效反馈 */
    std::uint32_t tx_rejected = 0u;     /**< 累计被拒绝的越界指令数（**设备**侧计数） */
    /**
     * 被**我们**丢弃的命令数（当前唯一原因：`JointTarget.mode` 与该关节使能时的模式不符）。
     * ⚠ 与 `tx_rejected` 分开：那个是设备说“你这帧我不收”，这个是“我们压根没发”。
     */
    std::uint32_t cmd_rejected = 0u;
    std::uint8_t  cmd_mode = 4u;        /**< 该关节使能时选择的模式（`CmdMode` 取值；4 = MIT） */
    std::uint32_t tx_frames = 0u;
    double        position = 0.0;       /**< rad（输出端） */
    double        velocity = 0.0;       /**< rad/s */
    double        effort = 0.0;         /**< N·m（SDK 估算：current × torque_constant × gear） */
    double        current = 0.0;        /**< 电机端 A */
    double        motor_temperature = 0.0;
    double        fet_temperature = 0.0;
    double        bus_voltage = 0.0;
    double        bus_current = 0.0;
    bool          online = false;
    bool          enabled = false;
    bool          calibrated = false;
    bool          valid_fresh = false;  /**< 本 tick 有新数据 */
    bool          target_rejected = false;
    bool          feedback_stale = false;
    bool          watchdog_unverified = false;
};

/** 总线级状态（POD）。 */
struct BusStatsPOD {
    char          name[kBusNameLen] = {};
    bool          link_up = false;
    bool          degraded = false;      /**< 有任何一种"需要用户知道"的降级（如广播降级单播） */
    std::uint32_t tx_frames = 0u;
    std::uint32_t rx_frames = 0u;
    std::uint32_t tx_failed = 0u;
    std::uint32_t rx_dropped = 0u;
    std::uint32_t keepalive_sent = 0u;
    std::uint32_t link_errors = 0u;
    std::uint32_t last_rx_age_ms = 0u;
    std::uint32_t hal_bus_flags = 0u;
    std::uint8_t  nodes_online = 0u;
    double        bus_load_estimate = 0.0; /**< 0..1+（按 bus_plan 模型估计，**声明为估计值**） */
    char          last_note[160] = {};     /**< 最近一次需要让人看到的文本（如广播降级原因） */
};

/** 实时性度量（POD）。 */
struct RtStatsPOD {
    std::uint32_t period_ns = 0u;
    std::uint32_t jitter_last_ns = 0u;
    std::uint32_t jitter_min_ns = 0u;   /**< 0 = 还没有样本（不要把它当成"抖动为 0"） */
    std::uint32_t jitter_mean_ns = 0u;
    std::uint32_t jitter_max_ns = 0u;
    std::uint32_t jitter_p99_ns = 0u;      /**< 由固定分桶直方图统计（无分配） */
    std::uint32_t cycle_last_ns = 0u;
    std::uint32_t cycle_min_ns = 0u;
    std::uint32_t cycle_mean_ns = 0u;
    std::uint32_t cycle_max_ns = 0u;
    std::uint32_t cmd_to_tx_last_ns = 0u;
    std::uint32_t cmd_to_tx_mean_ns = 0u;
    std::uint32_t cmd_to_tx_max_ns = 0u;
    std::uint64_t tick_count = 0u;         /**< 自启动以来**已完成**的周期数（首份快照 = 1） */
    std::uint32_t stat_samples = 0u;       /**< 统计样本数（min/mean 据此判断"有没有样本"） */
    std::uint64_t missed_ticks = 0u;
    std::uint64_t command_overwrites = 0u; /**< 命令在 RT 取走前被覆盖的次数 */
    std::uint32_t config_pauses = 0u;
    std::uint32_t hook_last_ns = 0u;       /**< RtHook::step 耗时（若启用） */
    bool          rt_sched_ok = false;     /**< SCHED_FIFO / 高优先级是否生效 */
    bool          mlock_ok = false;
    bool          affinity_ok = false;
    bool          rt_throttled = false;    /**< 内核节流实时任务（抖动会超预期） */
};

/** 一 tick 的完整快照（整体发布/读取；读者只会在 tick 边界看到一致状态）。 */
struct StateSnapshot {
    std::uint64_t tick = 0u;
    std::uint64_t t_ns = 0u;              /**< 本 tick 的单调时刻（时间基准 t0） */
    std::uint32_t joint_count = 0u;
    std::uint32_t bus_count = 0u;
    std::uint32_t mode = 0u;              /**< BusMode（见 jr_bus_runtime.hpp），用 uint32 避免头文件耦合 */
    char          note[192] = {};         /**< **组级**备注（RT 节流/暂停/命令超时等）；
                                               与 `buses[i].last_note`（总线级，如广播降级）分开，
                                               否则两个都会丢信息 */
    JointStatePOD joints[kMaxJoints] = {};
    BusStatsPOD   buses[kMaxBuses] = {};
    RtStatsPOD    rt = {};
};

}  // namespace jr

#endif /* JR_ROS2_JR_SNAPSHOT_HPP */
