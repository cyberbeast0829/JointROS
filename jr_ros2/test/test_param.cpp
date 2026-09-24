/**
 * @file    test_param.cpp
 * @brief   参数类型与文本编解码单测（纯逻辑，无总线/无硬件）
 *
 * 重点盯"会不会静默出错"：超值域**必须拒**（不截断）、格式错**必须拒**、
 * 类型不受支持**必须拒**；`format` → `parse` 必须能原值往返。
 */

#include <cstdio>
#include <cstring>

#include "jr_ros2/jr_param.hpp"
#include "jr_test.hpp"

using namespace jr;

namespace {

ParamValue parse_ok(ParamType t, const char *text)
{
    ParamValue v;
    char err[160] = {};
    const Status st = parse_param_text(t, text, &v, err, sizeof(err));
    JR_CHECK_MSG(st == Status::kOk, err);
    return v;
}

void test_type_names()
{
    JR_CASE("类型名与文本的对应关系");
    JR_CHECK_EQ(param_type_from_name("uint16"), ParamType::kU16);
    JR_CHECK_EQ(param_type_from_name("u16"), ParamType::kU16);
    JR_CHECK_EQ(param_type_from_name("float"), ParamType::kF32);
    JR_CHECK_EQ(param_type_from_name("bool"), ParamType::kBool);
    JR_CHECK_EQ(param_type_from_name("object"), ParamType::kUnsupported);
    JR_CHECK(std::strcmp(param_type_name(ParamType::kI32), "int32") == 0);
}

void test_integers()
{
    JR_CASE("整数：进进制前缀支持、边界值、超值域拒绝");
    JR_CHECK_EQ(parse_ok(ParamType::kU8, "255").v.u8, 255u);
    JR_CHECK_EQ(parse_ok(ParamType::kU16, "65535").v.u16, 65535u);
    JR_CHECK_EQ(parse_ok(ParamType::kU32, "0x10").v.u32, 16u);
    /* "010" 必须是十进制的 10（而不是八进制的 8）—— 客户输入里出现前导零太常见了 */
    JR_CHECK_EQ(parse_ok(ParamType::kU32, "010").v.u32, 10u);
    JR_CHECK_EQ(parse_ok(ParamType::kI32, "-5").v.i32, -5);
    JR_CHECK_EQ(parse_ok(ParamType::kI64, "-9223372036854775807").v.i64, -9223372036854775807ll);

    char err[160] = {};
    ParamValue v;
    JR_CHECK(parse_param_text(ParamType::kU16, "99999", &v, err, sizeof(err)) != Status::kOk);
    JR_CHECK_CONTAINS(err, "exceeds uint16");
    JR_CHECK(parse_param_text(ParamType::kU8, "256", &v, err, sizeof(err)) != Status::kOk);
    JR_CHECK(parse_param_text(ParamType::kU32, "-1", &v, err, sizeof(err)) != Status::kOk);
    JR_CHECK_CONTAINS(err, "negative");
    JR_CHECK(parse_param_text(ParamType::kI8, "128", &v, err, sizeof(err)) != Status::kOk);
    JR_CHECK(parse_param_text(ParamType::kU32, "12abc", &v, err, sizeof(err)) != Status::kOk);
    JR_CHECK_CONTAINS(err, "not an integer");
    JR_CHECK(parse_param_text(ParamType::kU32, "", &v, err, sizeof(err)) != Status::kOk);
}

void test_floats_and_bool()
{
    JR_CASE("浮点与布尔：非法输入拒绝（含 inf/nan）、布尔接受常见写法");
    const ParamValue f = parse_ok(ParamType::kF32, "-5.0");
    JR_CHECK_EQ(f.type, ParamType::kF32);
    JR_CHECK(f.v.f32 < -4.99f && f.v.f32 > -5.01f);

    char err[160] = {};
    ParamValue v;
    JR_CHECK(parse_param_text(ParamType::kF32, "inf", &v, err, sizeof(err)) != Status::kOk);
    JR_CHECK(parse_param_text(ParamType::kF32, "nan", &v, err, sizeof(err)) != Status::kOk);
    JR_CHECK(parse_param_text(ParamType::kF32, "1.0e999", &v, err, sizeof(err)) != Status::kOk);
    JR_CHECK(parse_param_text(ParamType::kF32, "abc", &v, err, sizeof(err)) != Status::kOk);

    JR_CHECK(parse_ok(ParamType::kBool, "true").v.b);
    JR_CHECK(parse_ok(ParamType::kBool, "1").v.b);
    JR_CHECK(!parse_ok(ParamType::kBool, "off").v.b);
    JR_CHECK(parse_param_text(ParamType::kBool, "maybe", &v, err, sizeof(err)) != Status::kOk);
}

void test_unsupported_type()
{
    JR_CASE("不可读写的类型必须明确拒绝（不做\"读一半\"）");
    ParamValue v;
    char err[160] = {};
    const Status st = parse_param_text(ParamType::kUnsupported, "1", &v, err, sizeof(err));
    JR_CHECK(st != Status::kOk);
    JR_CHECK_CONTAINS(err, "not a readable/writable scalar");
}

void test_roundtrip_and_equality()
{
    JR_CASE("format → parse 往返一致；相等比较必须严格（不做近似）");
    const ParamValue f = parse_ok(ParamType::kF32, "12.5");
    char text[64] = {};
    format_param_value(f, text, sizeof(text));
    const ParamValue back = parse_ok(ParamType::kF32, text);
    JR_CHECK(param_values_equal(f, back));

    const ParamValue u = parse_ok(ParamType::kU64, "18446744073709551615");
    format_param_value(u, text, sizeof(text));
    JR_CHECK(std::strcmp(text, "18446744073709551615") == 0);

    /* 不同类型即使数值相同也不相等（避免把 u32 与 f32 混为一谈）。 */
    const ParamValue a = parse_ok(ParamType::kU32, "1");
    const ParamValue b = parse_ok(ParamType::kF32, "1");
    JR_CHECK(!param_values_equal(a, b));

    const ParamValue c = parse_ok(ParamType::kF32, "1.0000001");
    const ParamValue d = parse_ok(ParamType::kF32, "1.0000002");
    JR_CHECK(!param_values_equal(c, d));   /* f32 精度内可区分 → 必须判不等 */

    const ParamValue t = parse_ok(ParamType::kBool, "true");
    const ParamValue fv = parse_ok(ParamType::kBool, "false");
    JR_CHECK(!param_values_equal(t, fv));
    JR_CHECK(t.as_bool());
    JR_CHECK_EQ(fv.as_i64(), 0);
}

/**
 * 类型 → 服务消息字段（`jr_interfaces/msg/ParamValue.msg` 的契约）。
 *
 * ⚠ 这不是"把常量再抄一遍"：真机上踩过一次 —— 服务端与 CLI **各写了一份**映射表，
 *   两边对 **u32** 的说法不同（服务端按契约放 `int64_value`，CLI 按码值 6 取 `uint64_value`），
 *   于是所有 uint32 端点（`node_id`/`heartbeat_rate_ms`/`error`…）经 CLI 读出来**恒为 0**，
 *   而 f32 正常（float 那一组两边恰好一致）。现在映射只有 `value_field_of()` 一份，
 *   这个用例把**契约本身**钉死：谁把 u32 挪去别的字段，这里立刻红。
 */
void test_value_field_mapping()
{
    JR_CASE("类型 → 服务消息字段（u32 必须走 int64_value）");
    JR_CHECK_EQ(value_field_of(ParamType::kBool), ValueField::kBool);
    JR_CHECK_EQ(value_field_of(ParamType::kF32), ValueField::kDouble);
    JR_CHECK_EQ(value_field_of(ParamType::kF64), ValueField::kDouble);
    JR_CHECK_EQ(value_field_of(ParamType::kU64), ValueField::kUint64);
    /* ↓ 回归点：u32 曾被显示层当成 `uint64_value`（那个字段没人写入 → 读出来恒 0）。 */
    JR_CHECK_EQ(value_field_of(ParamType::kU32), ValueField::kInt64);
    JR_CHECK_EQ(value_field_of(ParamType::kU8), ValueField::kInt64);
    JR_CHECK_EQ(value_field_of(ParamType::kI8), ValueField::kInt64);
    JR_CHECK_EQ(value_field_of(ParamType::kU16), ValueField::kInt64);
    JR_CHECK_EQ(value_field_of(ParamType::kI16), ValueField::kInt64);
    JR_CHECK_EQ(value_field_of(ParamType::kI32), ValueField::kInt64);
    JR_CHECK_EQ(value_field_of(ParamType::kI64), ValueField::kInt64);
    JR_CHECK_EQ(value_field_of(ParamType::kUnsupported), ValueField::kNone);

    /* 覆盖面：每个**受支持**的类型都必须落到一个真实字段上（不许留下"没人写"的洞）。 */
    const ParamType all[] = {ParamType::kBool, ParamType::kU8,  ParamType::kI8,  ParamType::kU16,
                             ParamType::kI16, ParamType::kU32, ParamType::kI32, ParamType::kU64,
                             ParamType::kI64, ParamType::kF32, ParamType::kF64};
    for (ParamType t : all) {
        JR_CHECK_MSG(value_field_of(t) != ValueField::kNone, param_type_name(t));
    }
}

}  // namespace

int main()
{
    test_type_names();
    test_integers();
    test_floats_and_bool();
    test_unsupported_type();
    test_roundtrip_and_equality();
    test_value_field_mapping();
    return jrtest::report();
}
