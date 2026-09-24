/**
 * @file    jr_param.hpp
 * @brief   参数值的类型与文本编解码（**不依赖 SDK / ROS**）
 *
 * @par 为什么需要它（而不是直接用 SDK 的 `jsdk_value_t`）
 *  1. SDK 的 `jsdk_value_t.type` 用的是 `jsdk_ep_type_t`（含 object/json/endpoint_ref
 *     等不可读写的类型），直接暴露给 ROS 层与服务层会把这些内部概念带到客户面前；
 *  2. 客户/命令行给的是**文本**（`"100"`、`"true"`、`"-5.0"`），必须按**端点声明的类型**
 *     装箱 —— 这正是 SDK 侧踩过的坑：按 Python 类型猜宽度会把 u16 端点写成 u32，
 *     被设备以 `descriptor=uint16 given=uint32` 拒绝。规则固定为：
 *     **先查描述符 → 按声明类型解析 → 超值域/格式错就拒（绝不截断）**。
 *  3. 这一层是纯逻辑，可以在没有硬件、没有 ROS 的环境里做穷举单测。
 */

#ifndef JR_ROS2_JR_PARAM_HPP
#define JR_ROS2_JR_PARAM_HPP

#include <cstddef>
#include <cstdint>

#include "jr_ros2/jr_status.hpp"

namespace jr {

/** 可读写的标量参数类型（与 SDK 的 `JSDK_EP_*` 标量一一对应）。 */
enum class ParamType : std::uint8_t {
    kUnsupported = 0, /**< object / json / endpoint_ref / function：不可读写 */
    kBool,
    kU8, kI8, kU16, kI16, kU32, kI32, kU64, kI64,
    kF32, kF64
};

const char *param_type_name(ParamType t) noexcept;
ParamType   param_type_from_name(const char *name) noexcept;

/** 类型化值（定长、可比较；不持有指针）。 */
struct ParamValue {
    ParamType type = ParamType::kUnsupported;
    union {
        std::uint8_t  u8;  std::int8_t  i8;
        std::uint16_t u16; std::int16_t i16;
        std::uint32_t u32; std::int32_t i32;
        std::uint64_t u64; std::int64_t i64;
        float         f32; double       f64;
        bool          b;
    } v = {};

    bool is_integral() const noexcept;
    bool is_supported() const noexcept { return type != ParamType::kUnsupported; }

    /** 统一取数（用于比较/日志；不改变类型）。 */
    double       as_double() const noexcept;
    std::int64_t as_i64() const noexcept;
    std::uint64_t as_u64() const noexcept;
    bool         as_bool() const noexcept;
};

/** 同类型、同值的严格比较（写后校验用；不做"近似相等"这种会掩盖问题的判断）。 */
bool param_values_equal(const ParamValue &a, const ParamValue &b) noexcept;

/**
 * 值在**服务消息**里该落到哪个字段（`jr_interfaces/msg/ParamValue.msg` 的契约）。
 *
 * ⚠ 为什么要有一个共享函数：这份"类型 → 字段"的映射原先**在服务端和 CLI 各写了一份**
 *   （服务端 `switch (ParamType)`、CLI `switch (uint8 type_code)`），两份表对 **u32** 的说法
 *   不一致 —— 服务端按契约放进 `int64_value`，CLI 却按"码值 6"取 `uint64_value` ⇒
 *   **真机上所有 uint32 端点读出来都是 0**（`node_id`/`heartbeat_rate_ms`/`error`…
 *   而 f32 正常，因为 float 那一组两边恰好一致）。合成一个函数后，"两边说不一致"这种
 *   缺陷在结构上不再可能出现。
 */
enum class ValueField : std::uint8_t {
    kNone = 0, /**< 不可读写的类型：**不编造**字段，如实报 unsupported */
    kBool,     /**< → `bool_value` */
    kInt64,    /**< → `int64_value`（**含 u8/u16/u32/i8/i16/i32/i64**：u32 装得进 int64） */
    kUint64,   /**< → `uint64_value`（仅 u64：一律用它，即使值装得进 int64） */
    kDouble    /**< → `double_value`（f32/f64） */
};

/**
 * 单一映射点：**服务端（写消息）与 CLI（读消息）都调这个函数**。
 *
 * 做成 header-only（`inline`）：`jr_ctl` 设计上只链 `rclcpp` + `jr_interfaces`
 * （它"一个 CAN 帧都不发"、不依赖 jr_core），但又必须与节点用**同一张表** ——
 * 内联在头里同时满足这两条：既没有第二份表，也不用为一个枚举映射把整块 jr_core 链进工具。
 */
inline ValueField value_field_of(ParamType t) noexcept
{
    switch (t) {
    case ParamType::kBool: return ValueField::kBool;
    case ParamType::kU64:  return ValueField::kUint64;
    case ParamType::kF32:
    case ParamType::kF64:  return ValueField::kDouble;
    /* ⚠ u32 走 int64（契约见 ParamValue.msg）：u32 值域 0..4294967295 装得进 int64。
       真机踩过：CLI 曾把它按"码值 6"取 `uint64_value` ⇒ 所有 uint32 端点都显示 0。 */
    case ParamType::kU8: case ParamType::kI8:
    case ParamType::kU16: case ParamType::kI16:
    case ParamType::kU32: case ParamType::kI32:
    case ParamType::kI64:  return ValueField::kInt64;
    case ParamType::kUnsupported: break;
    }
    return ValueField::kNone;
}

/**
 * 文本 → 值（按**端点声明的类型**装箱）。
 *
 * 拒绝的情况（都返回非 kOk，并把原因写进 `err`）：
 *  - 类型不受支持（object/json/endpoint_ref）；
 *  - 格式非法（含尾随垃圾，如 `"12abc"`）；
 *  - **超出该类型的值域**（如给 u16 写 99999）—— 绝不静默截断。
 */
Status parse_param_text(ParamType type, const char *text, ParamValue *out, char *err,
                        std::size_t err_len) noexcept;

/** 值 → 文本（人类可读；诊断/服务响应用）。整数一律十进制，浮点用 %.9g。 */
void format_param_value(const ParamValue &value, char *out, std::size_t cap) noexcept;

}  // namespace jr

#endif /* JR_ROS2_JR_PARAM_HPP */
