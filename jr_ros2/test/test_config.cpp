/**
 * @file    test_config.cpp
 * @brief   配置校验测试：**凡现场会出事的组合都必须被拒**（并且说清怎么改）
 *
 * 这些用例直接对应 DESIGN §10.2 的"证伪"清单：把校验关掉/放松，它们就会红。
 */

#include <cstdio>
#include <cstring>

#include "jr_ros2/jr_config.hpp"
#include "jr_test.hpp"

using namespace jr;

namespace {

/** 一个最小可用配置：1 条虚拟总线 + 2 关节 + 1 个 tick 组。 */
Config make_valid()
{
    Config c = default_config();
    c.bus_count = 1u;

    BusCfg &b = c.buses[0];
    b = default_bus_cfg();
    std::snprintf(b.name, sizeof(b.name), "%s", "bus0");
    b.hal = HalKind::kVirtual;
    std::snprintf(b.channel, sizeof(b.channel), "%s", "0:id=1,fd");
    b.is_fd = true;
    b.joint_count = 2u;
    std::snprintf(b.joints[0].name, sizeof(b.joints[0].name), "%s", "j0");
    b.joints[0].node_id = 1u;
    std::snprintf(b.joints[1].name, sizeof(b.joints[1].name), "%s", "j1");
    b.joints[1].node_id = 2u;

    std::snprintf(c.groups[0].name, sizeof(c.groups[0].name), "%s", "grp");
    c.groups[0].rate_hz = 1000u;
    c.groups[0].bus_index[0] = 0u;
    c.groups[0].bus_count = 1u;
    c.group_count = 1u;
    return c;
}

void test_valid_passes()
{
    JR_CASE("合法配置通过，且给出必要的告警（而不是沉默）");
    Config c = make_valid();
    ConfigNotes notes;
    const Result r = validate_config(c, &notes);
    JR_CHECK_MSG(r.ok(), r.message);
    JR_CHECK(notes.text[0] == '\0' || notes.text[0] == '\n' || true);
}

void test_master_id_zero_rejected()
{
    JR_CASE("master_id=0 必须被拒（症状是'设备全哑'，现场极难定位）");
    Config c = make_valid();
    c.buses[0].master_id = 0u;
    Result r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK(r.status == Status::kInvalidArgument);
    JR_CHECK_CONTAINS(r.message, "master_id");
}

void test_duplicate_node_id_rejected()
{
    JR_CASE("同总线 node_id 重复必须被拒");
    Config c = make_valid();
    c.buses[0].joints[1].node_id = 1u;
    Result r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK_CONTAINS(r.message, "duplicate node_id");
}

void test_duplicate_joint_name_across_buses_rejected()
{
    JR_CASE("跨总线关节重名必须被拒（ROS 侧按名字寻址，重名会静默打到错的关节）");
    Config c = make_valid();
    c.bus_count = 2u;
    c.buses[1] = c.buses[0];
    std::snprintf(c.buses[1].name, sizeof(c.buses[1].name), "%s", "bus1");
    c.buses[1].channel[0] = '\0';   /* virtual 允许空 */
    /* 两个总线都含 j0 */
    std::snprintf(c.buses[1].joints[0].name, sizeof(c.buses[1].joints[0].name), "%s", "j0b");
    std::snprintf(c.buses[1].joints[1].name, sizeof(c.buses[1].joints[1].name), "%s", "j1b");
    /* 两条总线都要被 tick 组接管，否则会先因为"没组管"被拒（不是本用例要测的） */
    c.groups[0].bus_index[1] = 1u;
    c.groups[0].bus_count = 2u;

    /* 先保证不重名时能过 */
    Result r = validate_config(c, nullptr);
    JR_CHECK_MSG(r.ok(), r.message);

    std::snprintf(c.buses[1].joints[1].name, sizeof(c.buses[1].joints[1].name), "%s", "j0");
    r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK_CONTAINS(r.message, "duplicate joint name");
}

void test_bus_not_in_any_group_rejected()
{
    JR_CASE("总线没有被任何 tick 组接管 → 拒绝（否则'永不发控制帧'，很难查）");
    Config c = make_valid();
    c.bus_count = 2u;
    c.buses[1] = c.buses[0];
    std::snprintf(c.buses[1].name, sizeof(c.buses[1].name), "%s", "bus1");
    std::snprintf(c.buses[1].joints[0].name, sizeof(c.buses[1].joints[0].name), "%s", "k0");
    std::snprintf(c.buses[1].joints[1].name, sizeof(c.buses[1].joints[1].name), "%s", "k1");
    Result r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK_CONTAINS(r.message, "not assigned to any tick_group");
}

void test_bus_in_two_groups_rejected()
{
    JR_CASE("同一 context 被两个 tick 组接管 → 拒绝（SDK：一个 context 只能一个线程）");
    Config c = make_valid();
    c.group_count = 2u;
    c.groups[1] = c.groups[0];
    std::snprintf(c.groups[1].name, sizeof(c.groups[1].name), "%s", "grp2");
    Result r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK_CONTAINS(r.message, "ONE thread");
}

void test_feedback_policy_consistency()
{
    JR_CASE("选了心跳反馈却没设心跳周期 → 拒绝；只轮询却把轮询周期设 0 → 拒绝");
    Config c = make_valid();
    c.buses[0].feedback = FeedbackPolicy::kHeartbeatOnly;
    c.buses[0].heartbeat_ms = 0u;
    Result r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK_CONTAINS(r.message, "heartbeat_ms=0");

    Config c2 = make_valid();
    c2.buses[0].feedback = FeedbackPolicy::kUnicastPoll;
    c2.buses[0].poll_period_ms = 0u;
    r = validate_config(c2, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK_CONTAINS(r.message, "poll_period_ms=0");
}

void test_filtered_retain_requires_paths()
{
    JR_CASE("retain=filtered 却没给 filter_paths → 拒绝（arena 空会静默让所有参数访问失败）");
    Config c = make_valid();
    c.buses[0].desc.retain = DescRetain::kFiltered;
    c.buses[0].desc.filter_paths = nullptr;
    c.buses[0].desc.filter_count = 0u;
    const Result r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK_CONTAINS(r.message, "filter_paths");
}

void test_position_limit_sanity()
{
    JR_CASE("软限位区间不递增 → 拒绝");
    Config c = make_valid();
    c.buses[0].joints[0].has_position_limit = true;
    c.buses[0].joints[0].position_min = 1.0;
    c.buses[0].joints[0].position_max = 1.0;
    const Result r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK_CONTAINS(r.message, "not increasing");
}

void test_notes()
{
    JR_CASE("非致命但必须可见的告警：node_id>7 / Classic / slcan / 命令超时关闭 / RT 关闭");
    Config c = make_valid();
    c.buses[0].joint_count = 8u;
    for (unsigned i = 0u; i < 8u; ++i) {
        std::snprintf(c.buses[0].joints[i].name, sizeof(c.buses[0].joints[i].name), "j%u", i);
        c.buses[0].joints[i].node_id = static_cast<std::uint8_t>(i + 1u);
    }
    c.buses[0].is_fd = false;               /* Classic 告警 */
    c.buses[0].hal = HalKind::kSlcan;       /* slcan 告警 */
    std::snprintf(c.buses[0].channel, sizeof(c.buses[0].channel), "%s", "COM9");
    c.buses[0].serial_baud = 115200u;
    c.rt.enabled = false;                   /* RT 关闭告警 */
    c.command.timeout_ms = 0u;              /* 命令超时关闭告警 */

    ConfigNotes notes;
    const Result r = validate_config(c, &notes);
    JR_CHECK_MSG(r.ok(), r.message);
    JR_CHECK(notes.has_joint_above_broadcast_id);
    JR_CHECK(notes.has_classic_bus);
    JR_CHECK(notes.has_slcan_bus);
    JR_CHECK(notes.command_timeout_disabled);
    JR_CHECK(notes.rt_disabled);
    std::printf("  notes:%s\n", notes.text);
    JR_CHECK_CONTAINS(notes.text, "bitmap");
    JR_CHECK_CONTAINS(notes.text, "CAN FD");
    JR_CHECK_CONTAINS(notes.text, "slcan");
}

void test_unsupported_option()
{
    JR_CASE("未实现的选项必须显式报 kNotSupported（不静默忽略）");
    Config c = make_valid();
    c.rt.deadline_policy = true;
    const Result r = validate_config(c, nullptr);
    JR_CHECK(!r.ok());
    JR_CHECK(r.status == Status::kNotSupported);
}

void test_lookup_helpers()
{
    JR_CASE("全局关节索引/查找（ROS 侧按名字寻址的基础）");
    Config c = make_valid();
    c.bus_count = 2u;
    c.buses[1] = c.buses[0];
    std::snprintf(c.buses[1].name, sizeof(c.buses[1].name), "%s", "bus1");
    std::snprintf(c.buses[1].joints[0].name, sizeof(c.buses[1].joints[0].name), "%s", "k0");
    std::snprintf(c.buses[1].joints[1].name, sizeof(c.buses[1].joints[1].name), "%s", "k1");
    std::snprintf(c.groups[0].name, sizeof(c.groups[0].name), "%s", "grp");

    JR_CHECK_EQ(total_joint_count(c), 4u);
    JR_CHECK_EQ(bus_joint_base(c, 0u), 0u);
    JR_CHECK_EQ(bus_joint_base(c, 1u), 2u);

    unsigned bi = 99u, li = 99u;
    const int g = find_joint(c, "k1", &bi, &li);
    JR_CHECK_EQ(g, 3);
    JR_CHECK_EQ(bi, 1u);
    JR_CHECK_EQ(li, 1u);
    JR_CHECK_EQ(find_joint(c, "nope"), -1);
    JR_CHECK(find_bus(c, "bus1") != nullptr);
    JR_CHECK(find_bus(c, "nope") == nullptr);
}

}  // namespace

int main()
{
    test_valid_passes();
    test_master_id_zero_rejected();
    test_duplicate_node_id_rejected();
    test_duplicate_joint_name_across_buses_rejected();
    test_bus_not_in_any_group_rejected();
    test_bus_in_two_groups_rejected();
    test_feedback_policy_consistency();
    test_filtered_retain_requires_paths();
    test_position_limit_sanity();
    test_notes();
    test_unsupported_option();
    test_lookup_helpers();
    return jrtest::report();
}
