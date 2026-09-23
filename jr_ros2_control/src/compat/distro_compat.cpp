/**
 * @file    distro_compat.cpp
 * @brief   发行版差异实现（目前只有枚举→人话）
 */

#include "jr_ros2_control/compat/distro_compat.hpp"

namespace jr_ros2_control {
namespace compat {

const char *to_string(CallbackReturn v) noexcept
{
    switch (v) {
    case CallbackReturn::SUCCESS:
        return "SUCCESS";
    case CallbackReturn::FAILURE:
        return "FAILURE";
    case CallbackReturn::ERROR:
        return "ERROR";
    }
    return "?";
}

const char *to_string(return_type v) noexcept
{
    switch (v) {
    case return_type::OK:
        return "OK";
    case return_type::ERROR:
        return "ERROR";
    default:
        /* Jazzy 起 `return_type` 多了 DEACTIVATE（Humble 没有）→ 不能穷举，
           否则 -Wswitch 会一直挂在构建输出里。新增值由 detail 里的模板探测处理。 */
        break;
    }
    if (const char *extra = detail::extra_return_type_name(v)) {
        return extra;
    }
    return "?";
}

}  // namespace compat
}  // namespace jr_ros2_control
