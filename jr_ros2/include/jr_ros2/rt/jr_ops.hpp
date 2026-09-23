/**
 * @file    jr_ops.hpp
 * @brief   运维/参数操作的请求与响应载体（**非 RT**；POD + 定长缓冲，无分配）
 *
 * @par 使用契约（很重要）
 *  这些操作会**阻塞**（等待设备应答），并且 SDK 明确禁止在关节使能时做描述符/参数/标定类操作
 *  （描述符与参数帧会挤掉控制帧 → 触发设备侧看门狗 → disarm）。因此：
 *    - 只允许在总线处于 **READY / PAUSED** 时调用（`mode == kActive` 一律拒绝）；
 *    - 若节点正在跑控制循环，必须先走 `TickGroup::pause()`（它会先做安全失能再交出所有权），
 *      操作完成后再 `resume()`。这是 ADR-7 的"安全暂停窗口"。
 *
 * @par 为什么请求里带 `joint_index`
 *  参数是**每关节**的（SDK 的 `jsdk_joint_param_*` 挂在关节上）。让调用方显式给出关节序号，
 *  避免这一层去猜"路径属于哪个关节"（那会变成一个隐式约定，出错时很难查）。
 *  关节序号 = `BusCfg.joints[]` 的下标（总线内序号），不是全局索引。
 */

#ifndef JR_ROS2_RT_JR_OPS_HPP
#define JR_ROS2_RT_JR_OPS_HPP

#include <cstddef>
#include <cstdint>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_param.hpp"
#include "jr_ros2/jr_status.hpp"

namespace jr {

/** 端点信息（`list_endpoints` / `lookup_endpoint` 的输出）。 */
struct EndpointInfo {
    char          path[kPathLen] = {};
    std::uint16_t id = 0u;
    ParamType     type = ParamType::kUnsupported;
    std::uint8_t  access = 0u;      /**< 位 0 = 可读，位 1 = 可写（与 SDK 的 ACCESS_* 一致） */

    bool readable() const noexcept { return (access & 0x01u) != 0u; }
    bool writable() const noexcept { return (access & 0x02u) != 0u; }
};

/** 一次参数读请求（就地填结果）。 */
struct ParamReadItem {
    unsigned      joint_index = 0u;   /**< 总线内关节序号 */
    const char   *path = nullptr;     /**< 完整路径，如 "axis0.motor.config.gear_ratio" */
    ParamType     declared_type = ParamType::kUnsupported; /**< 输出：描述符里的类型 */
    ParamValue    value = {};         /**< 输出：读到的值 */
    Status        status = Status::kOk; /**< 输出：本项结果 */
    char          message[128] = {};  /**< 输出：可读原因（含 SDK 原文） */
};

/** 一次参数写请求（就地填结果；**写后必须能分别看到"我请求的"与"读回的"**）。 */
struct ParamWriteItem {
    unsigned      joint_index = 0u;
    const char   *path = nullptr;
    ParamType     declared_type = ParamType::kUnsupported; /**< 输入/输出：必须与端点声明一致 */
    ParamValue    requested = {};     /**< 输入：打算写入的值 */
    ParamValue    value = {};         /**< 输出：**读回**的值（不可读回时 type=kUnsupported） */
    bool          verified = false;   /**< 输出：读回与请求一致 */
    /**
     * 输出：该端点是"写进去就被固件**立即消费**"的类型（如 `axis0.requested_state`）。
     * 这类端点的读回**本来就会与请求不同**（状态机把它收走并复位），因此
     * `verified=false` **不算失败** —— 请改看"结果状态"（如 `axis0.current_state`）。
     */
    bool          consumed_by_firmware = false;
    bool          persisted = false;  /**< 输出：是否已落 Flash（仅在 persist=true 时可能为真） */
    Status        status = Status::kOk; /**< 输出：本项结果（含 kUnverified） */
    char          message[160] = {};  /**< 输出：可读原因（含 SDK 原文） */
};

}  // namespace jr

#endif /* JR_ROS2_RT_JR_OPS_HPP */
