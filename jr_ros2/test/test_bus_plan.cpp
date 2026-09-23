/**
 * @file    test_bus_plan.cpp
 * @brief   总线预算模型测试
 *
 * @par 两个层次
 *  ① **量级**：帧在网时间必须落在工程上合理的区间（模型被"优化"错就会红）；
 *  ② **结论**：DESIGN §7.1 的四条场景结论必须复现（全单播 ≫ 广播；Classic 全单播
 *     物理不可能；8 关节没法广播 → 必须拆总线）。结论比精确数字更重要 ——
 *     这些结论正是我们要写进客户集成指南、并用来**拒绝启动**的依据。
 */

#include <cstddef>
#include <cstdio>
#include <cstring>

#include "jr_ros2/rt/jr_bus_plan.hpp"
#include "jr_test.hpp"

using namespace jr;
using namespace jr::rt;

namespace {

/* ---- 不变量：给用户看的建议必须**完整**到达 ----
   同一条文本会经 `Result::message`（192 B，前面还要加 "bus 'can0': "）回传；
   超了就会被 snprintf **静默截断** —— 用户拿到的是一句被砍断的排障建议。
   把"真实文本 ≤170"钉成断言：以后谁再往文本里加内容，先在这里红。 */
void check_text_fits_result(const char *what, const BusPlanResult &r)
{
    const std::size_t n = std::strlen(r.text);
    if (n > 170u) {
        std::printf("  [%s] plan text is %zu chars (>170): Result::message(192) would truncate "
                    "it -> '%s'\n", what, n, r.text);
    }
    JR_CHECK(n <= 170u);
}

void test_frame_times()
{
    JR_CASE("帧在网时间量级（FD 1M/5M 与 Classic 1M）");

    const double classic_8b = can_frame_us(8u, false, 1000000u, 0u);
    const double fd_8b = can_frame_us(8u, true, 1000000u, 5000000u);
    const double fd_hb = can_frame_us(18u, true, 1000000u, 5000000u);
    const double fd_bc56 = can_frame_us(56u, true, 1000000u, 5000000u);
    const double fd_bc64 = can_frame_us(64u, true, 1000000u, 5000000u);

    std::printf("  classic 8B=%.1f us, fd 8B=%.1f, fd 18B=%.1f, fd 56B=%.1f, fd 64B=%.1f\n",
                classic_8b, fd_8b, fd_hb, fd_bc56, fd_bc64);

    /* Classic 扩展帧 8B：131 bit 起，加填充位 → 125..165 µs */
    JR_CHECK_IN(classic_8b, 125.0, 165.0);
    /* FD 扩展帧 8B：仲裁段 40+stub bit @1M + 数据段 @5M → 60..95 µs */
    JR_CHECK_IN(fd_8b, 60.0, 95.0);
    /* FD 必须比 Classic 快（否则模型/参数搞反了：BRS 没生效） */
    JR_CHECK(fd_8b < classic_8b);
    /* 载荷单调：18B < 56B < 64B */
    JR_CHECK(fd_hb < fd_bc56 && fd_bc56 < fd_bc64);

    /* 广播帧长 = (max_node_id+1)*8（槽位号 = node_id；slot 0 不能省） */
    JR_CHECK_EQ(broadcast_payload_bytes(6u), 56u);
    JR_CHECK_EQ(broadcast_payload_bytes(7u), 64u);
}

void test_scenarios_6_joints_1khz()
{
    JR_CASE("6 关节 @1 kHz：全单播不可用、广播+心跳可用（DESIGN §7.1 结论）");

    BusPlanInput base;
    base.joint_count = 6u;
    base.rate_hz = 1000.0;
    base.fd = true;
    base.nominal_bitrate = 1000000u;
    base.data_bitrate = 5000000u;
    base.max_node_id = 6u;
    base.max_load = 0.60;

    BusPlanInput unicast = base;
    unicast.feedback = FeedbackPolicy::kUnicastOnly;
    BusPlanResult r_uni = plan_bus(unicast);
    std::printf("  unicast-only : load=%.1f%% (%.0f TX + %.0f RX frames/s) feasible=%d\n",
                r_uni.load * 100.0, r_uni.tx_frames_per_sec, r_uni.rx_frames_per_sec,
                r_uni.feasible ? 1 : 0);
    JR_CHECK(!r_uni.feasible);
    JR_CHECK(r_uni.load > 0.60);
    JR_CHECK(r_uni.advice == Advice::kFixBusPlanning);
    JR_CHECK_CONTAINS(r_uni.text, "Fix by");
    check_text_fits_result("unicast-only", r_uni);

    BusPlanInput poll = base;
    poll.feedback = FeedbackPolicy::kUnicastPoll;
    poll.heartbeat_ms = 10u;
    BusPlanResult r_poll = plan_bus(poll);
    std::printf("  bcast+poll-1 : load=%.1f%% feedback/joint=%.0f Hz feasible=%d\n",
                r_poll.load * 100.0, r_poll.feedback_hz_per_joint, r_poll.feasible ? 1 : 0);
    JR_CHECK(r_poll.feasible);
    JR_CHECK(r_poll.feedback_hz_per_joint > 100.0);

    BusPlanInput hb = base;
    hb.feedback = FeedbackPolicy::kBroadcastHeartbeat;
    hb.poll_period_ms = 0u;
    hb.heartbeat_ms = 5u;
    BusPlanResult r_hb = plan_bus(hb);
    std::printf("  bcast+hb5ms  : load=%.1f%% feedback/joint=%.0f Hz feasible=%d warn=%d\n",
                r_hb.load * 100.0, r_hb.feedback_hz_per_joint, r_hb.feasible ? 1 : 0,
                r_hb.warn ? 1 : 0);
    JR_CHECK(r_hb.feasible);
    JR_CHECK(r_hb.load < r_poll.load);           /* 不轮询更省 */
    JR_CHECK(r_hb.load < r_uni.load / 2.0);      /* 与全单播不是一个量级 */
    JR_CHECK_EQ(r_hb.feedback_hz_per_joint, 200.0);

    /* Classic 1 Mbps 全单播：物理不可能（>100%） */
    BusPlanInput cl = base;
    cl.fd = false;
    cl.heartbeat_ms = 0u;
    cl.feedback = FeedbackPolicy::kUnicastOnly;
    BusPlanResult r_cl = plan_bus(cl);
    std::printf("  classic 1M   : load=%.1f%% feasible=%d\n", r_cl.load * 100.0,
                r_cl.feasible ? 1 : 0);
    JR_CHECK(!r_cl.feasible);
    JR_CHECK(r_cl.load > 1.0);
}

void test_broadcast_addressing_limit()
{
    JR_CASE("超过 7 个关节：广播位图寻址不到 → 必须拆总线（拒绝并给出可操作建议）");

    BusPlanInput in;
    in.joint_count = 8u;
    in.rate_hz = 1000.0;
    in.fd = true;
    in.max_node_id = 8u;          /* 第 8 个关节 node_id = 8 > 7 */
    in.feedback = FeedbackPolicy::kHeartbeatOnly;
    in.heartbeat_ms = 5u;

    const BusPlanResult r = plan_bus(in);
    std::printf("  8 joints     : load=%.1f%% feasible=%d text=%s\n", r.load * 100.0,
                r.feasible ? 1 : 0, r.text);
    JR_CHECK_CONTAINS(r.text, "bitmap");
    JR_CHECK_CONTAINS(r.text, "Split this bus");
    check_text_fits_result("8-joint bitmap limit", r);
}

void test_invalid_input_text()
{
    JR_CASE("非法输入：文本必须说清是输入问题（不能渲染成 n/a 或吃掉负号）");

    BusPlanInput in;
    in.joint_count = 0u;
    in.rate_hz = 0.0;
    const BusPlanResult r = plan_bus(in);
    std::printf("  joints=0   : %s\n", r.text);
    JR_CHECK(!r.feasible);
    JR_CHECK_CONTAINS(r.text, "invalid plan input");
    JR_CHECK_CONTAINS(r.text, "0.000");      /* 定点渲染真输出了数字，而不是 n/a */
    check_text_fits_result("joints=0", r);

    BusPlanInput neg = in;
    neg.joint_count = 4u;
    neg.rate_hz = -250.0;
    const BusPlanResult rn = plan_bus(neg);
    std::printf("  rate=-250  : %s\n", rn.text);
    JR_CHECK(!rn.feasible);
    JR_CHECK_CONTAINS(rn.text, "-250.000"); /* 负号与第 3 位小数都不能丢 */
    check_text_fits_result("negative rate", rn);
}

void test_plan_from_bus_cfg()
{
    JR_CASE("plan_bus(BusCfg) 会自己取 max_node_id 与策略");

    BusCfg bus = default_bus_cfg();
    bus.hal = HalKind::kVirtual;
    bus.is_fd = true;
    bus.joint_count = 3u;
    bus.joints[0].node_id = 1u;
    bus.joints[1].node_id = 2u;
    bus.joints[2].node_id = 3u;
    bus.feedback = FeedbackPolicy::kBroadcastHeartbeat;
    bus.heartbeat_ms = 10u;
    bus.poll_period_ms = 0u;

    const BusPlanResult r = plan_bus(bus, 1000.0);
    JR_CHECK(r.feasible);
    check_text_fits_result("feasible 3-joint", r);
    JR_CHECK_EQ(r.frame_us_broadcast > 0.0, true);
    /* 3 关节 → max_id=3 → 广播帧 32 B */
    JR_CHECK_IN(r.frame_us_broadcast, 60.0, 165.0);
}

}  // namespace

int main()
{
    test_frame_times();
    test_scenarios_6_joints_1khz();
    test_broadcast_addressing_limit();
    test_invalid_input_text();
    test_plan_from_bus_cfg();
    return jrtest::report();
}
