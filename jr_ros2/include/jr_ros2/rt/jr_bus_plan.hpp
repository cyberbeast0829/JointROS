/**
 * @file    jr_bus_plan.hpp
 * @brief   总线预算模型：**先算再上机**（DESIGN §7.1）
 *
 * @par 它解决的问题
 *  "6 关节 @1 kHz 全单播" 在 CAN FD 上就要 ~96% 总线占用，在 Classic 上物理不可能。
 *  这类错误如果只在上机后表现为"掉帧/看门狗故障"，现场几乎无法定位。
 *  所以：① 启动时按配置算占用，超限**拒绝启动**；② 同一个模型做成 `jr_bus_plan` 工具，
 *  让客户在选型阶段就能算。
 *
 * @par 帧在网时间模型
 *  含 SOF/仲裁/CRC/ACK/EOF/IFS；填充位按**平均**情况估计（每 5 bit 约 1 个 stuff 位），
 *  所以结果是**工程估计值**，不是最坏情况上界。测试里锁死了几个典型值的区间
 *  （见 `test_bus_plan.cpp`），一旦模型被"优化"错就会红。
 */

#ifndef JR_ROS2_RT_JR_BUS_PLAN_HPP
#define JR_ROS2_RT_JR_BUS_PLAN_HPP

#include <cstdint>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_status.hpp"

namespace jr {
namespace rt {

/**
 * 一帧在网时间（微秒）。
 * @param payload_bytes 0..64
 * @param fd  true = CAN FD（数据段可变速）
 * @param nominal_bitrate 仲裁段速率（bps），如 1'000'000
 * @param data_bitrate FD 数据段速率（bps）；`fd=false` 时忽略
 */
double can_frame_us(unsigned payload_bytes, bool fd, std::uint32_t nominal_bitrate,
                    std::uint32_t data_bitrate) noexcept;

/** SDK 的 MIT 广播帧长 = (max_node_id + 1) * 8（槽位号 = node_id）。 */
unsigned broadcast_payload_bytes(unsigned max_node_id) noexcept;

struct BusPlanInput {
    unsigned      joint_count = 0u;
    double        rate_hz = 1000.0;
    bool          fd = true;
    std::uint32_t nominal_bitrate = 1000000u;
    std::uint32_t data_bitrate = 5000000u;
    std::uint8_t  max_node_id = 0u;      /**< 决定广播帧长（0 = 由 joint_count 推断） */
    FeedbackPolicy feedback = FeedbackPolicy::kBroadcastHeartbeat;
    std::uint32_t heartbeat_ms = 5u;     /**< 0 = 无心跳 */
    std::uint32_t poll_period_ms = 10u;  /**< 轮询策略下的轮询周期 */
    bool          use_broadcast = true;  /**< 是否可能用广播下发（关节数 ≤7 且都使能才行） */
    double        max_load = 0.60;       /**< 阈值 */
    /** 总线上"不受我们管理"的其它节点的帧率（估算用；例如客户还有别的设备）。 */
    double        extra_frames_per_sec = 0.0;
};

struct BusPlanResult {
    bool          feasible = false;      /**< 是否允许启动（load ≤ max_load） */
    bool          warn = false;          /**< 0.6*max ~ max 之间（允许但持续告警） */
    double        load = 0.0;            /**< 0..1+ */
    double        tx_frames_per_sec = 0.0;
    double        rx_frames_per_sec = 0.0;
    double        feedback_hz_per_joint = 0.0;
    double        frame_us_broadcast = 0.0;
    double        frame_us_unicast = 0.0;
    double        frame_us_heartbeat = 0.0;
    double        us_per_tick = 0.0;      /**< 每 tick 的发送+接收在网时间（含心跳均摊） */
    Advice        advice = Advice::kNone;
    /* 尺寸不是随手定的：GCC `-O2` 的 `-Wformat-truncation` 按**类型上界**估算最长输出
       （`%u` 10 位、`%s` 取实参数组长度），填出本结构体的那条建议原文有 3 个 `%u` +
       7 个 `%s`（见 jr_bus_plan.cpp），算出来上界 284 → 留到 320 才不报截断。
       同时要求**真实文本 ≤170 字符**：它会经 `Result::message`（192 B，含 "bus 'x': "
       前缀）回传，超了就会被静默截断（test_bus_plan 里有断言守着这条不变量）。 */
    char          text[320] = {};         /**< 人可读明细（含超限时的可操作建议） */
};

/** 按配置算一条总线的预算。 */
BusPlanResult plan_bus(const BusPlanInput &in) noexcept;

/** 用 BusCfg 直接构造（省得调用方自己推 node_id/策略）。 */
BusPlanResult plan_bus(const BusCfg &bus, double rate_hz) noexcept;

}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_RT_JR_BUS_PLAN_HPP */
