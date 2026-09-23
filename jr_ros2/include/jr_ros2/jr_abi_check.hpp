/**
 * @file    jr_abi_check.hpp
 * @brief   启动期 SDK ABI 自检（**fail-fast**，沿用 Python 绑定 check_abi 的哲学）
 *
 * @par 为什么一定要做
 *  "链到了另一份/旧版本的 libjsdk_can"不会报错，只会**静默错值**或随机崩溃
 *  （结构体布局漂移 = 踩内存）。所以启动时用两个**独立**的数据源对拍：
 *    ① 本 TU 编译时看到的 `sizeof/alignof`（来自我们 include 的头文件）；
 *    ② 运行期库自报的 `jsdk_abi_types()` 表（来自**被链接进去的那份库**）。
 *  两者不一致 ⇒ 头文件与库不是同一版本 ⇒ 立刻失败。
 */

#ifndef JR_ROS2_JR_ABI_CHECK_HPP
#define JR_ROS2_JR_ABI_CHECK_HPP

#include <cstdint>

#include "jr_ros2/jr_status.hpp"

namespace jr {

/** 一条期望的类型尺寸/对齐。 */
struct AbiType {
    const char *name;
    std::uint32_t size;
    std::uint32_t align;
};

/** 本 TU 编译期看到的值（即本包编译时用的 joint_sdk.h 布局）。 */
const AbiType *expected_abi_types() noexcept;
unsigned expected_abi_type_count() noexcept;

struct AbiReport {
    bool     ok = false;
    unsigned checked = 0u;
    unsigned mismatches = 0u;
    char     detail[256] = {};   /**< 第一处不一致的可读描述（ok=false 时非空） */
};

/**
 * 与运行期库对拍。
 *
 * 若 `expected` 为 nullptr 则用 `expected_abi_types()`（生产路径）；
 * 传入自定义表用于**测试检查器本身**（变异测试：故意给错尺寸，必须红）。
 */
Status compare_abi_types(const AbiType *expected, unsigned count, AbiReport *out) noexcept;

/** 完整自检：后端名 + ABI 版本 + 类型表 + 上下文尺寸合理性。 */
Status check_sdk_abi(AbiReport *out) noexcept;

}  // namespace jr

#endif /* JR_ROS2_JR_ABI_CHECK_HPP */
