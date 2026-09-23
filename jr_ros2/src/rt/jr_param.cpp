/**
 * @file    jr_param.cpp
 * @brief   参数值类型与文本编解码实现
 */

#include "jr_ros2/jr_param.hpp"

#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace jr {
namespace {

bool ieq(const char *a, const char *b) noexcept
{
    return a != nullptr && b != nullptr && std::strcmp(a, b) == 0;
}

/** 去掉首尾空白（就地读，不改输入）。 */
const char *skip_ws(const char *s) noexcept
{
    while (s != nullptr && (*s == ' ' || *s == '\t')) ++s;
    return s;
}

bool only_ws_left(const char *s) noexcept
{
    s = skip_ws(s);
    return s == nullptr || *s == '\0';
}

/**
 * 进制选择：只认 `0x`/`0X` 前缀，其余一律**十进制**。
 *
 * ⚠ 不能直接用 `base = 0`（自动进制）：那样 `"010"` 会被当成八进制 8，
 *   而客户输入里出现前导零太常见了（“010” 写的是十）。这类“聪明”行为在现场
 *   只会变成“我写 10，设备里却是 8”。
 */
int detect_base(const char *s) noexcept
{
    if (s == nullptr) return 10;
    if (*s == '+' || *s == '-') ++s;
    return (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) ? 16 : 10;
}

/**
 * 统一失败出口。⚠ 早期版本只转发两个参数，但下面多处格式串用了 3 个实参
 * （如"超值域"提示）—— 那是**未定义行为**。改成真变参，并用 format 属性让编译器盯着。
 */
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 3, 4)))
#endif
Status fail(char *err, std::size_t err_len, const char *fmt, ...) noexcept
{
    if (err != nullptr && err_len > 0u) {
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(err, err_len, fmt, ap);
        va_end(ap);
    }
    return Status::kInvalidArgument;
}

}  // namespace

const char *param_type_name(ParamType t) noexcept
{
    switch (t) {
    case ParamType::kBool: return "bool";
    case ParamType::kU8:   return "uint8";
    case ParamType::kI8:   return "int8";
    case ParamType::kU16:  return "uint16";
    case ParamType::kI16:  return "int16";
    case ParamType::kU32:  return "uint32";
    case ParamType::kI32:  return "int32";
    case ParamType::kU64:  return "uint64";
    case ParamType::kI64:  return "int64";
    case ParamType::kF32:  return "float";
    case ParamType::kF64:  return "double";
    case ParamType::kUnsupported: return "unsupported";
    }
    return "?";
}

ParamType param_type_from_name(const char *name) noexcept
{
    if (name == nullptr) return ParamType::kUnsupported;
    if (ieq(name, "bool") || ieq(name, "boolean")) return ParamType::kBool;
    if (ieq(name, "u8")   || ieq(name, "uint8"))   return ParamType::kU8;
    if (ieq(name, "i8")   || ieq(name, "int8"))    return ParamType::kI8;
    if (ieq(name, "u16")  || ieq(name, "uint16"))  return ParamType::kU16;
    if (ieq(name, "i16")  || ieq(name, "int16"))   return ParamType::kI16;
    if (ieq(name, "u32")  || ieq(name, "uint32"))  return ParamType::kU32;
    if (ieq(name, "i32")  || ieq(name, "int32"))   return ParamType::kI32;
    if (ieq(name, "u64")  || ieq(name, "uint64"))  return ParamType::kU64;
    if (ieq(name, "i64")  || ieq(name, "int64"))   return ParamType::kI64;
    if (ieq(name, "f32")  || ieq(name, "float"))   return ParamType::kF32;
    if (ieq(name, "f64")  || ieq(name, "double"))  return ParamType::kF64;
    return ParamType::kUnsupported;
}

bool ParamValue::is_integral() const noexcept
{
    switch (type) {
    case ParamType::kBool:
    case ParamType::kU8: case ParamType::kI8:
    case ParamType::kU16: case ParamType::kI16:
    case ParamType::kU32: case ParamType::kI32:
    case ParamType::kU64: case ParamType::kI64:
        return true;
    default:
        return false;
    }
}

double ParamValue::as_double() const noexcept
{
    switch (type) {
    case ParamType::kBool: return v.b ? 1.0 : 0.0;
    case ParamType::kU8:   return static_cast<double>(v.u8);
    case ParamType::kI8:   return static_cast<double>(v.i8);
    case ParamType::kU16:  return static_cast<double>(v.u16);
    case ParamType::kI16:  return static_cast<double>(v.i16);
    case ParamType::kU32:  return static_cast<double>(v.u32);
    case ParamType::kI32:  return static_cast<double>(v.i32);
    case ParamType::kU64:  return static_cast<double>(v.u64);
    case ParamType::kI64:  return static_cast<double>(v.i64);
    case ParamType::kF32:  return static_cast<double>(v.f32);
    case ParamType::kF64:  return v.f64;
    default:               return 0.0;
    }
}

std::uint64_t ParamValue::as_u64() const noexcept
{
    switch (type) {
    case ParamType::kBool: return v.b ? 1u : 0u;
    case ParamType::kU8:   return v.u8;
    case ParamType::kI8:   return (v.i8 < 0) ? 0u : static_cast<std::uint64_t>(v.i8);
    case ParamType::kU16:  return v.u16;
    case ParamType::kI16:  return (v.i16 < 0) ? 0u : static_cast<std::uint64_t>(v.i16);
    case ParamType::kU32:  return v.u32;
    case ParamType::kI32:  return (v.i32 < 0) ? 0u : static_cast<std::uint64_t>(v.i32);
    case ParamType::kU64:  return v.u64;
    case ParamType::kI64:  return (v.i64 < 0) ? 0u : static_cast<std::uint64_t>(v.i64);
    case ParamType::kF32:  return (v.f32 < 0.0f) ? 0u : static_cast<std::uint64_t>(v.f32);
    case ParamType::kF64:  return (v.f64 < 0.0) ? 0u : static_cast<std::uint64_t>(v.f64);
    default:               return 0u;
    }
}

std::int64_t ParamValue::as_i64() const noexcept
{
    switch (type) {
    case ParamType::kBool: return v.b ? 1 : 0;
    case ParamType::kU8:   return v.u8;
    case ParamType::kI8:   return v.i8;
    case ParamType::kU16:  return v.u16;
    case ParamType::kI16:  return v.i16;
    case ParamType::kU32:  return v.u32;
    case ParamType::kI32:  return v.i32;
    case ParamType::kU64:  return (v.u64 > 0x7FFFFFFFFFFFFFFFull) ? 0 : static_cast<std::int64_t>(v.u64);
    case ParamType::kI64:  return v.i64;
    case ParamType::kF32:  return static_cast<std::int64_t>(v.f32);
    case ParamType::kF64:  return static_cast<std::int64_t>(v.f64);
    default:               return 0;
    }
}

bool ParamValue::as_bool() const noexcept
{
    if (type == ParamType::kBool) return v.b;
    if (type == ParamType::kF32) return v.f32 != 0.0f;
    if (type == ParamType::kF64) return v.f64 != 0.0;
    return as_i64() != 0;
}

bool param_values_equal(const ParamValue &a, const ParamValue &b) noexcept
{
    if (a.type != b.type) return false;
    switch (a.type) {
    case ParamType::kBool: return a.v.b == b.v.b;
    case ParamType::kU8:   return a.v.u8 == b.v.u8;
    case ParamType::kI8:   return a.v.i8 == b.v.i8;
    case ParamType::kU16:  return a.v.u16 == b.v.u16;
    case ParamType::kI16:  return a.v.i16 == b.v.i16;
    case ParamType::kU32:  return a.v.u32 == b.v.u32;
    case ParamType::kI32:  return a.v.i32 == b.v.i32;
    case ParamType::kU64:  return a.v.u64 == b.v.u64;
    case ParamType::kI64:  return a.v.i64 == b.v.i64;
    /* 浮点用**精确比较**：写进去什么、读回来必须是同一个 f32/f64 位模式。
       用近似比较会把"设备做了量化"这种真实问题掩盖掉。 */
    case ParamType::kF32:  return a.v.f32 == b.v.f32;
    case ParamType::kF64:  return a.v.f64 == b.v.f64;
    default:               return false;
    }
}

Status parse_param_text(ParamType type, const char *text, ParamValue *out, char *err,
                        std::size_t err_len) noexcept
{
    if (err != nullptr && err_len > 0u) err[0] = '\0';
    if (out == nullptr) return Status::kInvalidArgument;
    if (type == ParamType::kUnsupported) {
        return fail(err, err_len,
                    "type is not a readable/writable scalar (object/json/endpoint_ref/function are "
                    "not supported)");
    }
    if (text == nullptr) return fail(err, err_len, "null text for type %s", param_type_name(type));

    const char *s = skip_ws(text);
    char *end = nullptr;
    errno = 0;

    switch (type) {
    case ParamType::kBool: {
        if (ieq(s, "true") || ieq(s, "1") || ieq(s, "on") || ieq(s, "yes")) {
            out->type = type;
            out->v.b = true;
            return Status::kOk;
        }
        if (ieq(s, "false") || ieq(s, "0") || ieq(s, "off") || ieq(s, "no")) {
            out->type = type;
            out->v.b = false;
            return Status::kOk;
        }
        return fail(err, err_len, "value '%s' is not a boolean (use true/false)", s);
    }

    case ParamType::kU8:
    case ParamType::kU16:
    case ParamType::kU32:
    case ParamType::kU64: {
        if (*s == '-') return fail(err, err_len, "value '%s' is negative but the endpoint is %s",
                                   s, param_type_name(type));
        const unsigned long long raw = std::strtoull(s, &end, detect_base(s));
        if (end == s || !only_ws_left(end)) return fail(err, err_len, "value '%s' is not an integer", s);
        if (errno == ERANGE) return fail(err, err_len, "value '%s' overflows %s", s, param_type_name(type));
        const unsigned long long limit =
            (type == ParamType::kU8)  ? 0xFFull
            : (type == ParamType::kU16) ? 0xFFFFull
            : (type == ParamType::kU32) ? 0xFFFFFFFFull
                                        : 0xFFFFFFFFFFFFFFFFull;
        if (raw > limit) return fail(err, err_len, "value '%s' exceeds %s max (0x%llX)", s,
                                     param_type_name(type), limit);
        out->type = type;
        if (type == ParamType::kU8) out->v.u8 = static_cast<std::uint8_t>(raw);
        else if (type == ParamType::kU16) out->v.u16 = static_cast<std::uint16_t>(raw);
        else if (type == ParamType::kU32) out->v.u32 = static_cast<std::uint32_t>(raw);
        else out->v.u64 = static_cast<std::uint64_t>(raw);
        return Status::kOk;
    }

    case ParamType::kI8:
    case ParamType::kI16:
    case ParamType::kI32:
    case ParamType::kI64: {
        const long long raw = std::strtoll(s, &end, detect_base(s));
        if (end == s || !only_ws_left(end)) return fail(err, err_len, "value '%s' is not an integer", s);
        if (errno == ERANGE) return fail(err, err_len, "value '%s' overflows %s", s, param_type_name(type));
        const long long lo = (type == ParamType::kI8)  ? -128ll
                             : (type == ParamType::kI16) ? -32768ll
                             : (type == ParamType::kI32) ? -2147483648ll
                                                         : (-9223372036854775807ll - 1ll);
        const long long hi = (type == ParamType::kI8)  ? 127ll
                             : (type == ParamType::kI16) ? 32767ll
                             : (type == ParamType::kI32) ? 2147483647ll
                                                         : 9223372036854775807ll;
        if (raw < lo || raw > hi) {
            return fail(err, err_len, "value '%s' exceeds %s range [%lld, %lld]", s,
                        param_type_name(type), lo, hi);
        }
        out->type = type;
        if (type == ParamType::kI8) out->v.i8 = static_cast<std::int8_t>(raw);
        else if (type == ParamType::kI16) out->v.i16 = static_cast<std::int16_t>(raw);
        else if (type == ParamType::kI32) out->v.i32 = static_cast<std::int32_t>(raw);
        else out->v.i64 = static_cast<std::int64_t>(raw);
        return Status::kOk;
    }

    case ParamType::kF32:
    case ParamType::kF64: {
        const double raw = std::strtod(s, &end);
        if (end == s || !only_ws_left(end)) return fail(err, err_len, "value '%s' is not a number", s);
        if (errno == ERANGE) return fail(err, err_len, "value '%s' out of range for %s", s, param_type_name(type));
        if (!std::isfinite(raw)) return fail(err, err_len, "value '%s' is not finite", s);
        if (type == ParamType::kF32) {
            const float f = static_cast<float>(raw);
            if (!std::isfinite(f)) return fail(err, err_len, "value '%s' overflows float", s);
            out->type = type;
            out->v.f32 = f;
        } else {
            out->type = type;
            out->v.f64 = raw;
        }
        return Status::kOk;
    }

    case ParamType::kUnsupported:
        break;
    }
    return fail(err, err_len, "unsupported parameter type (%s)", param_type_name(type));
}

void format_param_value(const ParamValue &value, char *out, std::size_t cap) noexcept
{
    if (out == nullptr || cap == 0u) return;
    switch (value.type) {
    case ParamType::kBool:
        std::snprintf(out, cap, "%s", value.v.b ? "true" : "false");
        return;
    case ParamType::kU8:  std::snprintf(out, cap, "%u", static_cast<unsigned>(value.v.u8)); return;
    case ParamType::kI8:  std::snprintf(out, cap, "%d", static_cast<int>(value.v.i8)); return;
    case ParamType::kU16: std::snprintf(out, cap, "%u", static_cast<unsigned>(value.v.u16)); return;
    case ParamType::kI16: std::snprintf(out, cap, "%d", static_cast<int>(value.v.i16)); return;
    case ParamType::kU32: std::snprintf(out, cap, "%u", value.v.u32); return;
    case ParamType::kI32: std::snprintf(out, cap, "%d", value.v.i32); return;
    case ParamType::kU64:
        std::snprintf(out, cap, "%llu", static_cast<unsigned long long>(value.v.u64));
        return;
    case ParamType::kI64:
        std::snprintf(out, cap, "%lld", static_cast<long long>(value.v.i64));
        return;
    case ParamType::kF32: std::snprintf(out, cap, "%.9g", static_cast<double>(value.v.f32)); return;
    case ParamType::kF64: std::snprintf(out, cap, "%.17g", value.v.f64); return;
    case ParamType::kUnsupported: break;
    }
    std::snprintf(out, cap, "%s", "(unsupported)");
}

}  // namespace jr
