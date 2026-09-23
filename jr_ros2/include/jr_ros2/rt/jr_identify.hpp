/**
 * @file    jr_identify.hpp
 * @brief   总线**身份识别**：发现节点 + 读设备身份/量程（`jr_gen_config` / `jr_hw_verify` 共用）
 *
 * @par 为什么单独一层（而不是写进两个工具）
 *   两个工具问的是同一批问题（谁在线上、固件是什么、量程多少、标定没有），
 *   而回答它们的 SDK 调用顺序**有前置约束**（下一个注释里逐条写明）。
 *   写两份必然漂移；写一份，工具只负责"怎么问、怎么打印、退出码怎么给"。
 *
 * @par 与 `BusRuntime` 的关系（刻意复用，不重复实现）
 *   本层**不自己开总线**：调用方先用 `BusRuntime::open()` 拿到
 *   "锁 + ABI 自检 + HAL + 零初始化 context" 这四件容易写错的事
 *   （尤其 `jsdk_context_init()` 要求全新零存储，漏了就是偶发 INVALID_ARG）。
 *   本层只做 `BusRuntime` 没覆盖的那部分：**配置还没有关节时也能问总线**。
 *
 * @par 前置约束（SDK 语义，踩过才知道）
 *   1. `jsdk_context_discover()` **不能**在关节使能后调用（会争用响应）→ 拒绝并提示；
 *   2. `jsdk_joint_read_config_snapshot()` 读的是**本地缓存**（`j->gear_ratio` / `j->range`），
 *      **必须先 `jsdk_context_configure()`** 才有值 ⇒ `read_config=false` 时量程全 0 且
 *      `config_valid=false`（"没读到"必须能和"读到 0"区分开，所以用**标志位**而不是数值）；
 *   3. 主动探测（`probe_max>0`）会往总线上发 `QUERY_STATUS` ⇒ 真机上是有代价的动作
 *      （默认 16，`probe_max=0` = 仅被动听 200 ms 心跳）。
 */

#ifndef JR_ROS2_RT_JR_IDENTIFY_HPP
#define JR_ROS2_RT_JR_IDENTIFY_HPP

#include <cstdint>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_status.hpp"
#include "jr_ros2/rt/jr_bus_runtime.hpp"

namespace jr {
namespace rt {

/** 一个被发现的节点（`node_id` 来自总线上**它自己的应答**，不是我们猜的）。 */
struct IdentifyNode {
    std::uint8_t  node_id = 0u;
    DeviceInfoPOD device = {};      /**< fw/hw/serial/classic；`valid=false` = 没问到 */
    JointInfoPOD  config = {};      /**< 仅 `read_config=true` 且有值时才可信 */
    bool          in_config = false;/**< 配置里也声明了这个 node_id（用于交叉核对） */
    char          name[kJointNameLen] = {};  /**< 配置里的关节名（`in_config` 时有效） */
};

struct IdentifyReport {
    IdentifyNode node[kMaxJointsPerBus] = {};
    unsigned     count = 0u;            /**< 发现数（≤ kMaxJointsPerBus） */
    bool         config_read = false;   /**< configure() 成功 ⇒ `node[].config` 才有值 */
    unsigned     nodes_online = 0u;     /**< 设备侧统计窗口内的在线数（原始计数） */
    bool         link_up = false;       /**< HAL 自报链路可用 */
    DescInfoPOD  desc = {};             /**< 描述符信息（`config_read` 时才读） */
    char         text[768] = {};        /**< 逐行结论（含告警；工具直接打印） */
};

struct IdentifyOptions {
    /** 主动探测上限：**建议 16**；0 = 仅被动听心跳（真机上最省事、但可能漏掉没在发心跳的设备）。 */
    std::uint8_t probe_max = 16u;
    /** true = 追加 `configure()`（下描述符 + 标定）以读回量程/标定；代价是总线流量与时间。 */
    bool         read_config = true;
    std::uint32_t period_ns = 1000000u;   /**< 预算用（`configure()` 之后不再用） */
};

/**
 * 在**已打开**的总线上做一次身份识别。
 *
 * @param bus  已 `open()`（可尚未 `configure()`）的运行时；关节可以是 0 个。
 * @param opt  探测与读取策略。
 * @param out  输出报告（**先清零**；即使返回失败，也会带上已获得的部分）。
 * @param res  可读结论；失败时含 SDK 原文。
 *
 * @return `Status::kOk`        发现成功且（`read_config=false` 或 `configure()` 成功）
 *         `Status::kInvalidState` 总线未打开，或**有节点已使能**（discover 拒绝）
 *         `configure()` 的失败码  发现成功但配置/标定读取失败（`out` 里仍有设备身份）
 *         `Status::kNoMemory`    发现数超过 `kMaxJointsPerBus`（**不截断**，如实报）
 */
Status identify_bus(BusRuntime &bus, const IdentifyOptions &opt, IdentifyReport *out,
                    Result *res) noexcept;

}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_RT_JR_IDENTIFY_HPP */
