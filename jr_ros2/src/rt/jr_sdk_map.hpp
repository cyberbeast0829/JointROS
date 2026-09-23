/**
 * @file    jr_sdk_map.hpp
 * @brief   **内部**（不安装）：SDK 状态码 → jr::Status 的单一映射点
 *
 * @par 为什么单独一个内部头
 *  这段映射原本在 `jr_bus_runtime.cpp` 与 `jr_bus_ops.cpp` 里各写了一份 ——
 *  两份映射迟早会漂移（同一个 SDK 错误在两个文件里被归成不同状态码，现场看到的就是
 *  "同一件事有两种说法"）。集中一处，谁新增 SDK 错误码只改这里。
 */

#ifndef JR_ROS2_SRC_RT_JR_SDK_MAP_HPP
#define JR_ROS2_SRC_RT_JR_SDK_MAP_HPP

#include <cstring>

#include "joint_sdk/joint_sdk.h"

#include "jr_ros2/jr_status.hpp"

namespace jr {
namespace rt {
namespace internal {

inline Status map_sdk_status(jsdk_status_t st) noexcept
{
    switch (st) {
    case JSDK_OK:              return Status::kOk;
    case JSDK_ERR_INVALID_ARG: return Status::kInvalidArgument;
    case JSDK_ERR_NO_MEMORY:   return Status::kNoMemory;
    case JSDK_ERR_NOT_FOUND:   return Status::kNotFound;
    case JSDK_ERR_BAD_STATE:   return Status::kInvalidState;
    case JSDK_ERR_TRANSPORT:   return Status::kTransport;
    case JSDK_ERR_UNSUPPORTED: return Status::kNotSupported;
    case JSDK_ERR_TIMEOUT:     return Status::kTimeout;
    case JSDK_ERR_PROTOCOL:    return Status::kProtocol;
    case JSDK_ERR_BUSY:        return Status::kInvalidState;
    case JSDK_ERR_PARSE:       return Status::kProtocol;
    default:                   return Status::kInternal;
    }
}

/**
 * 由 SDK 的错误文本推断"下一步该做什么"。
 *
 * ⚠ 只做**粗分类**：最终结论必须看原始位与计数（见 DESIGN §8.6 —— 固件把 estop 与
 *   看门狗都报成 CAN_TIMEOUT，光看摘要会把两类完全不同的问题混成一个）。
 */
inline Advice advice_from_sdk_text(const char *sdk_text) noexcept
{
    if (sdk_text == nullptr) return Advice::kNone;
    if (std::strstr(sdk_text, "break_timeout") != nullptr) return Advice::kReduceRateOrRaiseWatchdog;
    if (std::strstr(sdk_text, "calibrat") != nullptr) return Advice::kCalibrationRequired;
    if (std::strstr(sdk_text, "ESTOP") != nullptr) return Advice::kNeedsDeviceReset;
    if (std::strstr(sdk_text, "bus-off") != nullptr ||
        std::strstr(sdk_text, "busoff") != nullptr) {
        return Advice::kCheckBusTermination;
    }
    if (std::strstr(sdk_text, "descriptor") != nullptr) return Advice::kCheckBusConfig;
    return Advice::kNone;
}

}  // namespace internal
}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_SRC_RT_JR_SDK_MAP_HPP */
