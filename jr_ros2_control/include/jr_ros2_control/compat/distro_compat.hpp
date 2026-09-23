/**
 * @file    distro_compat.hpp
 * @brief   发行版差异的**唯一**落脚点（ADR-11 / DESIGN §9.2）
 *
 * 规矩：`#if` 只允许出现在 `jr_ros2_control/src/compat/` 下。
 * 散到各处之后，每加一个发行版都要全局搜索、且很容易漏 —— 这个代价我们不接受。
 */

#ifndef JR_ROS2_CONTROL_COMPAT_DISTRO_COMPAT_HPP
#define JR_ROS2_CONTROL_COMPAT_DISTRO_COMPAT_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <type_traits>

namespace jr_ros2_control {
namespace compat {

using CallbackReturn = hardware_interface::CallbackReturn;
using return_type = hardware_interface::return_type;

/** `on_init` 的参数类型随发行版变化（DESIGN §9.2 / 风险 U3）：
 *
 *   - **Humble**：只有 `on_init(const HardwareInfo &)`；
 *   - **Jazzy**：两个都有（旧的标了 `[[deprecated]]`）；
 *   - **Lyrical 及更新**：旧签名已**删除**，只剩
 *     `on_init(const HardwareComponentInterfaceParams &)`。
 *
 *  ⚠ 为什么必须**精确匹配**签名，不能"两个重载都写"：
 *    基类的 `on_init` **不是纯虚**（有默认实现，只把 info 存进 `info_`）。
 *    如果我们多写一个对应不上基类虚函数的重载，它会**隐藏**基类那个虚函数 →
 *    框架通过基类接口调用时走的是基类默认实现 → **我们的初始化逻辑静默不执行**。
 *    "编得过、跑得起来、但什么都没做"是最危险的一类错，必须靠精确匹配 + 编译期守卫挡住。
 *
 *  ⚠ 判定方式：**用编译器判**（`__has_include` + 静态断言），**不用 CMake 探测**。
 *    教训（真实踩过）：早先版本用 `check_cxx_source_compiles` 探测"旧签名是否还能
 *    override"，只把 `hardware_interface` 目标**自己**的 include 目录给了迷你工程 →
 *    拿不到传递依赖的头（缺 `rcpputils/pointer_traits.hpp`）→ 探测在**三个发行版上
 *    全部失败**；而 `check_cxx_source_compiles` 失败时变量是**空串**（不是 FALSE），
 *    `#if` 把空串当 0 → 全走了"新签名"分支 → 直到 Humble 报
 *    "`HardwareComponentInterfaceParams` 不是类型"才暴露。
 *    ⇒ 探测必须**与真实编译同条件**，且失败要**响**（下面那道 static_assert 就是铃）。 */
#if defined(__has_include)
#  if __has_include(<hardware_interface/types/hardware_component_interface_params.hpp>)
#    define JR_COMPAT_HAS_MODERN_ON_INIT 1
#  else
#    define JR_COMPAT_HAS_MODERN_ON_INIT 0
#  endif
#else
#  error "本包要求支持 __has_include 的编译器（C++17 起）"
#endif

#if JR_COMPAT_HAS_MODERN_ON_INIT
#include <hardware_interface/types/hardware_component_interface_params.hpp>
using OnInitParams = hardware_interface::HardwareComponentInterfaceParams;
#else
using OnInitParams = hardware_interface::HardwareInfo;
#endif

/** 编译期守卫：确认我们选的这个参数类型**就是**基类上存在的那个虚函数。
 *  任何发行版再改签名/删重载，这里立刻带着人话报错 —— 而不是等到现场才发现
 *  "初始化逻辑根本没跑"。 */
namespace detail {
template <class P, class = void>
struct on_init_is_overridable : std::false_type {};
template <class P>
struct on_init_is_overridable<
    P, std::void_t<decltype(static_cast<CallbackReturn (
           hardware_interface::SystemInterface::*)(const P &)>(&hardware_interface::SystemInterface::on_init))>>
    : std::true_type {};
}  // namespace detail

static_assert(detail::on_init_is_overridable<OnInitParams>::value,
              "hardware_interface::SystemInterface 上没有 on_init(const <本发行版参数类型> &) "
              "这个虚函数：请更新 compat/distro_compat.hpp 的签名判定（适配层是唯一允许出现发行版差异的地方）");

/** 两种参数类型都取出 `HardwareInfo`（业务逻辑只认这一个）。 */
inline const hardware_interface::HardwareInfo &info_of(const hardware_interface::HardwareInfo &p)
{
    return p;
}
#if JR_COMPAT_HAS_MODERN_ON_INIT
inline const hardware_interface::HardwareInfo &
info_of(const hardware_interface::HardwareComponentInterfaceParams &p)
{
    return p.hardware_info;
}
#endif

/** 由 `HardwareInfo` 造出**本发行版** `on_init` 需要的参数对象。
 *  库与测试共用同一条路径：测试不需要自己再写一遍 `#if`（否则测试可能替错误的分支背书）。 */
inline OnInitParams make_params(const hardware_interface::HardwareInfo &info)
{
#if JR_COMPAT_HAS_MODERN_ON_INIT
    OnInitParams p;
    p.hardware_info = info;
    return p;
#else
    return info;
#endif
}

/** 本发行版用的是哪个签名（人话），现场/日志里一眼能看出走的哪条路。 */
inline const char *on_init_signature() noexcept
{
#if JR_COMPAT_HAS_MODERN_ON_INIT
    return "modern (HardwareComponentInterfaceParams)";
#else
    return "legacy (HardwareInfo)";
#endif
}

/** 人话版枚举（日志里不要只打数字 —— 现场排查的人看不懂 4 是哪个枚举）。 */
const char *to_string(CallbackReturn v) noexcept;
const char *to_string(return_type v) noexcept;

/** 发行版可能给 `return_type` **新增**枚举值：Jazzy 起有 `DEACTIVATE`，Humble 没有。
 *
 *  两个都不能接受：① 在 Humble 上写出 `return_type::DEACTIVATE`（那个名字不存在 → 编译失败）；
 *  ② 一直挂着 `-Wswitch`（我们的规矩是"新代码不允许带告警进来"）。
 *  所以用**模板**探测：`if constexpr` 的丢弃分支只在实例化时检查名字，
 *  于是"这个名字在不在"由编译器在**本发行版**上回答，不需要任何 `#if` 或版本号。
 */
namespace detail {
template <class T, class = void>
struct has_deactivate : std::false_type {};
template <class T>
struct has_deactivate<T, std::void_t<decltype(T::DEACTIVATE)>> : std::true_type {};

template <class T>
const char *extra_return_type_name(T v) noexcept
{
    if constexpr (has_deactivate<T>::value) {
        if (v == T::DEACTIVATE) {
            return "DEACTIVATE";
        }
    }
    (void)v;
    return nullptr;
}
}  // namespace detail

}  // namespace compat
}  // namespace jr_ros2_control

#endif /* JR_ROS2_CONTROL_COMPAT_DISTRO_COMPAT_HPP */
