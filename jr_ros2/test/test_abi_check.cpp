/**
 * @file    test_abi_check.cpp
 * @brief   SDK ABI 自检测试（含**检查器自身**的变异测试）
 *
 * @par 为什么要测"检查器"
 *  上一轮在 Python 绑定上踩过：库比头文件旧 → 运行期静默错值。所以这里不仅验证
 *  "与当前 SDK 对拍通过"，还要验证"**故意给错尺寸时必须报错**"——否则那个检查
 *  可能只是一段永远返回 OK 的代码。
 */

#include <cstdio>
#include <cstring>

#include "jr_ros2/jr_abi_check.hpp"
#include "jr_test.hpp"

using namespace jr;

namespace {

void test_real_sdk_passes()
{
    JR_CASE("与真实 SDK 对拍：后端名/ABI 版本/结构体尺寸全部一致");
    AbiReport rep;
    const Status st = check_sdk_abi(&rep);
    std::printf("  %s\n", rep.detail);
    JR_CHECK_MSG(st == Status::kOk, rep.detail);
    JR_CHECK(rep.ok);
    JR_CHECK(rep.checked >= 15u);
    JR_CHECK_CONTAINS(rep.detail, "cyberbeast-can");
}

void test_mutated_table_must_fail()
{
    JR_CASE("变异测试：故意把某个结构体尺寸写错 → 检查器必须失败并指出类型名");
    AbiType bad[3] = {
        {"jsdk_value_t", 4u /* 真实值远大于 4 */, 4u},
        {"jsdk_bus_state_t", 44u, 4u},
        {"jsdk_can_frame_t", 68u, 4u},
    };
    AbiReport rep;
    const Status st = compare_abi_types(bad, 3u, &rep);
    std::printf("  %s\n", rep.detail);
    JR_CHECK(st != Status::kOk);
    JR_CHECK(!rep.ok);
    JR_CHECK_CONTAINS(rep.detail, "jsdk_value_t");
    JR_CHECK_CONTAINS(rep.detail, "linked library says");
}

void test_missing_type_must_fail()
{
    JR_CASE("变异测试：表里放一个 SDK 没有的类型名 → 必须报'缺类型'");
    AbiType bad[1] = {{"jsdk_does_not_exist_t", 8u, 8u}};
    AbiReport rep;
    const Status st = compare_abi_types(bad, 1u, &rep);
    JR_CHECK(st != Status::kOk);
    JR_CHECK_CONTAINS(rep.detail, "missing from the linked library");
}

void test_truncated_table_must_fail()
{
    JR_CASE("变异测试：只给了部分类型（库比头文件新）→ 表长不一致必须报错");
    AbiReport rep;
    const Status st = compare_abi_types(expected_abi_types(), 2u, &rep);
    JR_CHECK(st != Status::kOk);
    JR_CHECK_CONTAINS(rep.detail, "length mismatch");
}

void test_expected_table_covers_all()
{
    JR_CASE("期望表覆盖 SDK 公布的全部 15 个类型名（新增类型必须同步）");
    JR_CHECK_EQ(expected_abi_type_count(), 15u);
    const AbiType *t = expected_abi_types();
    for (unsigned i = 0u; i < expected_abi_type_count(); ++i) {
        JR_CHECK(t[i].name != nullptr);
        JR_CHECK(t[i].size > 0u);
        JR_CHECK(t[i].align > 0u);
    }
}

}  // namespace

int main()
{
    test_real_sdk_passes();
    test_mutated_table_must_fail();
    test_missing_type_must_fail();
    test_truncated_table_must_fail();
    test_expected_table_covers_all();
    return jrtest::report();
}
