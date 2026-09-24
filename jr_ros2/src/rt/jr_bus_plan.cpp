/**
 * @file    jr_bus_plan.cpp
 * @brief   总线预算模型实现
 */

#include "jr_ros2/rt/jr_bus_plan.hpp"

#include <cmath>
#include <cstdio>

namespace jr {
namespace rt {
namespace {

/** 平均填充位（每 5 个"可填充位"约 1 个 stuff 位）。 */
double stuff_bits(double stuffable_bits) noexcept
{
    return std::ceil(stuffable_bits / 5.0);
}

/** 定点渲染（**非负**量）：把 double 写成 `char[16]` 装得下的整数/小数文本。
 *
 * 为什么不用 `%f`：GCC 在 `-O2` 下按**类型上界**估算格式串最长输出（`double` 的
 * `%f` 上界约 309 位），只要理论最大值放不下就报 `-Wformat-truncation` —— 实测
 * **与是否使用返回值无关**（级别 1 一样报），所以无法靠"检查返回值"消音。
 * 改成整数运算渲染后编译器能算准上界（`%u` ≤10 位），同时把"这些量不可能长到
 * 20 位"变成**显式约束**：负数/NaN/超过 1e9 一律渲染成 `n/a` —— 是**可见的错**，
 * 不是悄悄截断。最大输出 14 字符（decimals=3），故配 `char[16]` 使用。 */
void fmt_num(char *dst, std::size_t cap, double v, int decimals) noexcept
{
    if (!(v >= 0.0) || v > 1e9) {   /* 负数 / NaN / 离谱值 */
        std::snprintf(dst, cap, "n/a");
        return;
    }
    unsigned p10 = 1u;
    for (int i = 0; i < decimals; ++i) p10 *= 10u;
    const unsigned long long scaled =
        static_cast<unsigned long long>(v * static_cast<double>(p10) + 0.5);
    const unsigned ip = static_cast<unsigned>(scaled / p10);
    const unsigned fp = static_cast<unsigned>(scaled % p10);
    switch (decimals) {
    case 3:  std::snprintf(dst, cap, "%u.%03u", ip, fp); break;
    case 1:  std::snprintf(dst, cap, "%u.%u", ip, fp); break;
    default: std::snprintf(dst, cap, "%u", ip); break;
    }
}

}  // namespace

double can_frame_us(unsigned payload_bytes, bool fd, std::uint32_t nominal_bitrate,
                    std::uint32_t data_bitrate) noexcept
{
    if (nominal_bitrate == 0u) return 0.0;
    const double n = static_cast<double>(payload_bytes);
    const double nominal = static_cast<double>(nominal_bitrate);

    if (!fd) {
        /* CAN 2.0B 扩展帧：SOF+SRR+IDE+ID(29)+RTR+r1+r0+DLC(4) = 39
           + data + CRC(15) + CRCdel+ACK+ACKdel+EOF(7) + IFS(3) = 28 → 67 + 8n */
        const double stuffable = 39.0 + 8.0 * n + 15.0;
        const double bits = 67.0 + 8.0 * n + stuff_bits(stuffable);
        return bits / nominal * 1e6;
    }

    const double data = (data_bitrate != 0u) ? static_cast<double>(data_bitrate) : nominal;

    /* 仲裁段（到 BRS 切换点）：SOF1+ID29+SRR1+IDE1+FDF1+res1+BRS1+ESI1+DLC4 = 40 */
    const double arb_bits = 40.0 + stuff_bits(40.0);
    /* 数据段：数据 + CRC（≤16B → 17 bit，否则 21 bit） */
    const double crc_bits = (payload_bytes <= 16u) ? 17.0 : 21.0;
    const double data_bits = 8.0 * n + crc_bits + stuff_bits(8.0 * n + crc_bits);
    /* 收尾（切换回仲裁速率）：CRCdel1+ACK1+ACKdel1+EOF7+IFS3 = 13 */
    const double tail_bits = 13.0;

    return (arb_bits / nominal + data_bits / data + tail_bits / nominal) * 1e6;
}

unsigned broadcast_payload_bytes(unsigned max_node_id) noexcept
{
    return (max_node_id + 1u) * 8u;
}

BusPlanResult plan_bus(const BusPlanInput &in) noexcept
{
    BusPlanResult r;

    const unsigned n = in.joint_count;
    if (n == 0u || in.rate_hz <= 0.0) {
        char rate[16] = {};
        fmt_num(rate, sizeof(rate), std::fabs(in.rate_hz), 3);
        r.feasible = false;
        r.advice = Advice::kFixBusPlanning;
        std::snprintf(r.text, sizeof(r.text), "invalid plan input: joints=%u rate_hz=%s%s", n,
                      (in.rate_hz < 0.0) ? "-" : "", rate);
        return r;
    }

    const unsigned max_id = (in.max_node_id != 0u) ? in.max_node_id : n;
    const unsigned bc_bytes = broadcast_payload_bytes(max_id);
    const unsigned hb_bytes = in.fd ? 18u : 8u;

    r.frame_us_broadcast = can_frame_us(bc_bytes, in.fd, in.nominal_bitrate, in.data_bitrate);
    r.frame_us_unicast = can_frame_us(8u, in.fd, in.nominal_bitrate, in.data_bitrate);
    r.frame_us_heartbeat = can_frame_us(hb_bytes, in.fd, in.nominal_bitrate, in.data_bitrate);

    /* 关节数 > 7 时广播位图寻址不到，必然退化为单播（SDK 的降级规则）。 */
    const bool can_broadcast = in.use_broadcast && (max_id <= kMaxBroadcastNodeId);

    double tx_per_tick = 0.0;
    double rx_per_tick = 0.0;
    double hb_frames_per_sec = 0.0;
    double poll_hz_per_joint = 0.0;

    switch (in.feedback) {
    case FeedbackPolicy::kUnicastOnly:
        /* 每个关节每 tick 一条单播 MIT，同时拿到该关节的 MIT 响应。 */
        tx_per_tick = static_cast<double>(n);
        rx_per_tick = static_cast<double>(n);
        break;

    case FeedbackPolicy::kUnicastPoll:
        /* 广播下发（1 帧）+ 每 tick 轮询 1 个关节（1 TX + 1 RX）。
           不能广播时，单播下发本身已带回反馈 → 等价于 unicast_only。 */
        if (can_broadcast) {
            tx_per_tick = 1.0 + 1.0;
            rx_per_tick = 1.0;
            poll_hz_per_joint = in.rate_hz / static_cast<double>(n);
        } else {
            tx_per_tick = static_cast<double>(n);
            rx_per_tick = static_cast<double>(n);
        }
        break;

    case FeedbackPolicy::kBroadcastHeartbeat:
        tx_per_tick = can_broadcast ? 1.0 : static_cast<double>(n);
        if (can_broadcast && in.poll_period_ms > 0u) {
            /* 轮询按周期折算：每 tick 的概率分摊（周期 > 1 帧时间的常见情况）。 */
            const double tick_ms = 1000.0 / in.rate_hz;
            const double poll_every = static_cast<double>(in.poll_period_ms) / tick_ms;
            tx_per_tick += 1.0 / poll_every;
            rx_per_tick += 1.0 / poll_every;
            poll_hz_per_joint = in.rate_hz / (static_cast<double>(n) * poll_every);
        }
        break;

    case FeedbackPolicy::kHeartbeatOnly:
        tx_per_tick = can_broadcast ? 1.0 : static_cast<double>(n);
        break;
    }

    if (in.heartbeat_ms != 0u) {
        hb_frames_per_sec = static_cast<double>(n) * 1000.0 / static_cast<double>(in.heartbeat_ms);
    }

    /* 帧数统计 */
    r.tx_frames_per_sec = tx_per_tick * in.rate_hz;
    r.rx_frames_per_sec = rx_per_tick * in.rate_hz + hb_frames_per_sec;
    const double total_frames = r.tx_frames_per_sec + r.rx_frames_per_sec + in.extra_frames_per_sec;

    /* 占用：按各帧型的在网时间加权（广播帧走 bc，其余单播/心跳） */
    const double unicast_frames = r.tx_frames_per_sec + rx_per_tick * in.rate_hz -
                                  (can_broadcast ? 1.0 * in.rate_hz : 0.0);
    const double bcast_frames = can_broadcast ? 1.0 * in.rate_hz : 0.0;
    const double bus_us_per_sec = bcast_frames * r.frame_us_broadcast +
                                  unicast_frames * r.frame_us_unicast +
                                  hb_frames_per_sec * r.frame_us_heartbeat +
                                  in.extra_frames_per_sec * r.frame_us_unicast;
    r.load = bus_us_per_sec / 1e6;

    /* 每 tick 的净占用（含心跳均摊），给"这个 tick 有多长"的直觉 */
    const double tick_ms = 1000.0 / in.rate_hz;
    r.us_per_tick = (r.frame_us_broadcast * bcast_frames +
                     r.frame_us_unicast * (unicast_frames)) / in.rate_hz +
                    r.frame_us_heartbeat * hb_frames_per_sec / in.rate_hz;

    /* 反馈率：取心跳与轮询中较快的那个（两者都开时以快的为准说明"最好情况"） */
    const double hb_hz = (in.heartbeat_ms != 0u) ? 1000.0 / static_cast<double>(in.heartbeat_ms)
                                                 : 0.0;
    switch (in.feedback) {
    case FeedbackPolicy::kUnicastOnly:
        r.feedback_hz_per_joint = in.rate_hz;
        break;
    case FeedbackPolicy::kUnicastPoll:
        r.feedback_hz_per_joint = std::fmax(poll_hz_per_joint, hb_hz);
        break;
    case FeedbackPolicy::kBroadcastHeartbeat:
        r.feedback_hz_per_joint = std::fmax(poll_hz_per_joint, hb_hz);
        break;
    case FeedbackPolicy::kHeartbeatOnly:
        r.feedback_hz_per_joint = hb_hz;
        break;
    }

    r.feasible = (r.load <= in.max_load);
    r.warn = r.feasible && (r.load > in.max_load * 0.75);

    if (!can_broadcast && in.use_broadcast && max_id > kMaxBroadcastNodeId) {
        char rate[16] = {};
        char frames[16] = {};
        char load[16] = {};
        fmt_num(rate, sizeof(rate), in.rate_hz, 0);
        fmt_num(frames, sizeof(frames), total_frames, 0);
        fmt_num(load, sizeof(load), r.load * 100.0, 1);
        r.advice = Advice::kFixBusPlanning;
        std::snprintf(r.text, sizeof(r.text),
                      "%u joints @%s Hz, %s %u/%u bps: node_id %u > %u breaks the broadcast "
                      "bitmap -> %s frames/s, %s%% load. Split this bus.",
                      n, rate, in.fd ? "FD" : "Classic", in.nominal_bitrate, in.data_bitrate,
                      static_cast<unsigned>(max_id), static_cast<unsigned>(kMaxBroadcastNodeId),
                      frames, load);
        return r;
    }

    char rate[16] = {};
    char tx[16] = {};
    char rx[16] = {};
    char load[16] = {};
    char limit[16] = {};
    char fb[16] = {};
    fmt_num(rate, sizeof(rate), in.rate_hz, 0);
    fmt_num(tx, sizeof(tx), r.tx_frames_per_sec, 0);
    fmt_num(rx, sizeof(rx), r.rx_frames_per_sec, 0);
    fmt_num(load, sizeof(load), r.load * 100.0, 1);
    fmt_num(limit, sizeof(limit), in.max_load * 100.0, 0);
    fmt_num(fb, sizeof(fb), r.feedback_hz_per_joint, 0);

    if (!r.feasible) {
        r.advice = Advice::kFixBusPlanning;
        const int wrote = std::snprintf(r.text, sizeof(r.text),
                      "%u joints @%s Hz, %s %u/%u bps: %s TX + %s RX frames/s = %s%% load "
                      "(limit %s%%), feedback %s Hz. Fix by: broadcast+heartbeat, or split "
                      "buses.",
                      n, rate, in.fd ? "FD" : "Classic", in.nominal_bitrate, in.data_bitrate, tx,
                      rx, load, limit, fb);
        /* ⚠ Classic 的拒绍多一条出路：配置**漏写 `is_fd`** 时我们按 Classic 起步
           （§13.3-46），而真有 FD 设备的客户会看到一句莫名其妙的“超预算”。
           这里明说“你是 FD 就写 is_fd: true” —— 否则他只是看到拒绍，不知道差在哪。
           ⚠ 分两次写（不用 `%s` 拼三元表达式）：GCC 的 `-Wformat-truncation` 按**类型上界**
           估算，把变量长串塞进同一个 format 会把它推过 text[320] 的界限。 */
        if (!in.fd && wrote > 0 && static_cast<std::size_t>(wrote) < sizeof(r.text)) {
            std::snprintf(r.text + wrote, sizeof(r.text) - static_cast<std::size_t>(wrote),
                          " If the device speaks CAN FD, set is_fd: true (Classic airtime is ~10x "
                          "tighter).");
        }
        return r;
    }

    std::snprintf(r.text, sizeof(r.text),
                  "%u joints @%s Hz, %s %u/%u bps, feedback=%u%s: %s TX + %s RX frames/s, %s%% "
                  "load (limit %s%%), feedback %s Hz",
                  n, rate, in.fd ? "FD" : "Classic", in.nominal_bitrate, in.data_bitrate,
                  static_cast<unsigned>(in.feedback), can_broadcast ? ", broadcast" : "", tx, rx,
                  load, limit, fb);
    r.advice = r.warn ? Advice::kFixBusPlanning : Advice::kNone;

    (void)tick_ms;
    return r;
}

BusPlanResult plan_bus(const BusCfg &bus, double rate_hz) noexcept
{
    BusPlanInput in;
    in.joint_count = bus.joint_count;
    in.rate_hz = rate_hz;
    in.fd = bus.is_fd;
    in.nominal_bitrate = bus.nominal_bitrate;
    in.data_bitrate = bus.data_bitrate;
    std::uint8_t max_id = 0u;
    for (unsigned j = 0u; j < bus.joint_count && j < kMaxJointsPerBus; ++j) {
        if (bus.joints[j].node_id > max_id) max_id = bus.joints[j].node_id;
    }
    in.max_node_id = max_id;
    in.feedback = bus.feedback;
    in.heartbeat_ms = bus.heartbeat_ms;
    in.poll_period_ms = bus.poll_period_ms;
    in.max_load = bus.max_bus_load;
    return plan_bus(in);
}

}  // namespace rt
}  // namespace jr
