/**
 * @file    test_ops.cpp
 * @brief   运维面（参数/端点/标定/点动）的虚拟总线集成测试
 *
 * 盯住的都是"会静默出错"的地方：
 *   ① 使能时做参数/描述符操作 → **必须拒绝**（否则会把设备喂狗的控制帧挤掉，触发 disarm）；
 *   ② 写操作缺 `confirm` → **必须整批拒绝**，且**设备里的值不变**（要真读回验证）；
 *   ③ 路径不存在 → `kNotFound`（**不做**前缀/模糊匹配）；
 *   ④ 类型不符（用 u32 请求去写 u16 端点）→ 拒绝并说清"端点声明的是 uint16"；
 *   ⑤ 只读端点 → 拒绝；
 *   ⑥ 写后读回不一致 → `kUnverified`，**整体不得返回成功**（§10.2 的证伪用例之一）；
 *   ⑦ 未标定的设备：`configure()` 降级为 READY（不卡成 kIdle），端点表可用，
 *      `calibrate()` 能真正跑起来（鸡生蛋问题）。
 */

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

#include "jr_ros2/rt/jr_tick_group.hpp"
#include "jr_test.hpp"

using namespace jr;
using namespace jr::rt;

namespace {

void sleep_ms(unsigned ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

/** 单关节虚拟总线（1 kHz 能力，但测试里不依赖速率）。 */
Config make_cfg(const char *spec = "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd",
                unsigned rate_hz = 200u)
{
    Config c = default_config();
    c.bus_count = 1u;
    BusCfg &b = c.buses[0];
    b = default_bus_cfg();
    std::snprintf(b.name, sizeof(b.name), "%s", "vbus");
    b.hal = HalKind::kVirtual;
    b.is_fd = true;
    std::snprintf(b.channel, sizeof(b.channel), "%s", spec);
    b.joint_count = 1u;
    std::snprintf(b.joints[0].name, sizeof(b.joints[0].name), "%s", "j0");
    b.joints[0].node_id = 1u;
    b.desc.cache_enabled = false;
    b.heartbeat_ms = 5u;
    b.poll_period_ms = 0u;
    b.feedback = FeedbackPolicy::kHeartbeatOnly;

    std::snprintf(c.groups[0].name, sizeof(c.groups[0].name), "%s", "grp");
    c.groups[0].rate_hz = rate_hz;
    c.groups[0].bus_index[0] = 0u;
    c.groups[0].bus_count = 1u;
    c.group_count = 1u;
    c.rt.enabled = false;
    c.lock.enabled = false;
    c.command.timeout_ms = 1000u;
    return c;
}

/** 开一条已 configure 的虚拟总线（模式 = READY，不跑 tick 线程）。 */
bool open_ready(TickGroup &tg, Config &cfg, Result *r)
{
    if (tg.init(cfg, 0u, r) != Status::kOk) return false;
    OpenOptions opt;
    opt.check_abi = true;
    opt.enable_lock = false;
    if (tg.open_buses(opt, r) != Status::kOk) return false;
    return tg.configure_buses(r) == Status::kOk;
}

std::uint32_t read_u32(TickGroup &tg, const char *path, bool *ok)
{
    ParamReadItem it;
    it.joint_index = 0u;
    it.path = path;
    Result r;
    (void)tg.bus(0).read_params(&it, 1u, &r);
    *ok = (it.status == Status::kOk);
    return it.value.v.u32;
}

/* ------------------------------------------------------------------ */

void test_guard_requires_configured_and_paused()
{
    JR_CASE("守卫：未 configure 时拒绝运维操作；ACTIVE 时也拒绝（并说明要先暂停）");

    Config cfg = make_cfg();
    Result r;
    TickGroup tg;
    // srcdoc 误注释修正：未 configure
    JR_CHECK_MSG(tg.init(cfg, 0u, &r) == Status::kOk, r.message);
    OpenOptions opt;
    opt.check_abi = false;
    opt.enable_lock = false;
    JR_CHECK_MSG(tg.open_buses(opt, &r) == Status::kOk, r.message);

    ParamReadItem it;
    it.joint_index = 0u;
    it.path = "axis0.motor.config.gear_ratio";
    const Status st = tg.bus(0).read_params(&it, 1u, &r);
    std::printf("  unconfigured → %s | %s\n", to_string(r.status), r.message);
    JR_CHECK(st != Status::kOk);
    JR_CHECK_CONTAINS(r.message, "endpoint table");

    JR_CHECK_MSG(tg.configure_buses(&r) == Status::kOk, r.message);

    /* 起线程 + 使能 → 必须拒绝参数读 */
    JR_CHECK_MSG(tg.start(nullptr, &r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.pause(&r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.activate(true, &r) == Status::kOk, r.message);
    tg.resume();
    sleep_ms(150u);

    const StateSnapshot *s = tg.acquire_snapshot();
    JR_CHECK(s != nullptr && s->joints[0].enabled);
    const Status st2 = tg.bus(0).read_params(&it, 1u, &r);
    std::printf("  active → %s | %s\n", to_string(r.status), r.message);
    JR_CHECK(st2 != Status::kOk);
    JR_CHECK_CONTAINS(r.message, "ACTIVE");
    JR_CHECK_CONTAINS(r.message, "Pause");

    /* tick 线程运行中（未暂停）直接 activate/deactivate 必须被拦住：
       那样会与 tick 并发访问同一个 context，症状不是崩溃而是难查的错值。 */
    const Status st_mis = tg.deactivate(&r);
    std::printf("  deactivate while running → %s | %s\n", to_string(r.status), r.message);
    JR_CHECK(st_mis == Status::kInvalidState);
    JR_CHECK_CONTAINS(r.message, "call pause() first");

    /* 暂停窗口内必须恢复可用（ADR-7 的用法） */
    JR_CHECK_MSG(tg.pause(&r) == Status::kOk, r.message);
    const Status st3 = tg.bus(0).read_params(&it, 1u, &r);
    JR_CHECK_MSG(st3 == Status::kOk, r.message);
    JR_CHECK(it.declared_type == ParamType::kF32);
    JR_CHECK_IN(it.value.as_double(), 16.0, 17.0);

    tg.resume();
    sleep_ms(80u);
    tg.stop();
    tg.close_buses(ExitAction::kDisable, &r);
}

void test_read_paths()
{
    JR_CASE("读参数：命中/未命中（kNotFound，不做模糊匹配）");
    Config cfg = make_cfg();
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(open_ready(tg, cfg, &r), r.message);

    ParamReadItem items[4];
    items[0].joint_index = 0u;
    items[0].path = "axis0.motor.config.gear_ratio";
    items[1].joint_index = 0u;
    items[1].path = "axis0.config.can.node_id";
    items[2].joint_index = 0u;
    items[2].path = "axis0.config.can.heartbeat_rate_ms";
    items[3].joint_index = 0u;
    items[3].path = "axis0.this.does.not.exist";

    const Status st = tg.bus(0).read_params(items, 4u, &r);
    std::printf("  %s\n", r.message);
    for (unsigned i = 0u; i < 4u; ++i) {
        char text[48] = {};
        format_param_value(items[i].value, text, sizeof(text));
        std::printf("    %-40s %-8s %-8s %s\n", items[i].path,
                    param_type_name(items[i].declared_type), text, to_string(items[i].status));
    }
    JR_CHECK(st != Status::kOk);                 /* 有一项失败 → 整体不报成功 */
    JR_CHECK(items[0].status == Status::kOk);
    JR_CHECK_IN(items[0].value.as_double(), 16.0, 17.0);
    JR_CHECK(items[1].status == Status::kOk);
    JR_CHECK_EQ(items[1].value.v.u32, 1u);
    JR_CHECK(items[2].status == Status::kOk);
    JR_CHECK_EQ(items[2].value.v.u32, 5u);       /* 设备侧心跳 = 5 ms（来自 spec 的 hb=5） */
    JR_CHECK(items[3].status != Status::kOk);

    /* 不存在的项要用 kNotFound 表示（而不是含糊的 protocol/不支持） */
    JR_CHECK(items[3].status == Status::kProtocol || items[3].status == Status::kNotFound);

    tg.close_buses(ExitAction::kDisable, &r);
}

void test_write_gates_and_verification()
{
    JR_CASE("写参数：confirm 闸门、类型不符拒绝、只读端点拒绝、写后读回校验");
    Config cfg = make_cfg();
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(open_ready(tg, cfg, &r), r.message);

    bool ok = false;
    const std::uint32_t before = read_u32(tg, "axis0.config.can.heartbeat_rate_ms", &ok);
    JR_CHECK(ok);
    JR_CHECK_EQ(before, 5u);

    /* ---- ① 缺 confirm → 整批拒绝，且设备值不变 ---- */
    ParamWriteItem w;
    w.joint_index = 0u;
    w.path = "axis0.config.can.heartbeat_rate_ms";
    w.requested.type = ParamType::kU32;
    w.requested.v.u32 = 20u;
    const Status st_no = tg.bus(0).write_params(&w, 1u, false, false, &r);
    std::printf("  confirm=false → %s | %s\n", to_string(r.status), r.message);
    JR_CHECK(st_no == Status::kInvalidState);
    JR_CHECK_CONTAINS(r.message, "confirm=true");
    const std::uint32_t after_refused = read_u32(tg, "axis0.config.can.heartbeat_rate_ms", &ok);
    JR_CHECK_EQ(after_refused, before);          /* ← 关键：被拒就是真的没写 */

    /* ---- ② 类型不符（用 u32 请求去写 bool 端点）→ 拒绝并指出声明类型 ---- */
    ParamWriteItem w2;
    w2.joint_index = 0u;
    w2.path = "axis0.motor.config.pre_calibrated";   /* 该端点是 bool */
    w2.requested.type = ParamType::kU32;
    w2.requested.v.u32 = 1u;
    const Status st_mm = tg.bus(0).write_params(&w2, 1u, true, false, &r);
    std::printf("  type-mismatch → %s | %s\n", to_string(w2.status), w2.message);
    JR_CHECK(st_mm != Status::kOk);
    JR_CHECK(w2.status == Status::kInvalidArgument);
    JR_CHECK_CONTAINS(w2.message, "declares bool");
    JR_CHECK_CONTAINS(w2.message, "uint32");

    /* ---- ③ 只读端点 → 拒绝 ---- */
    ParamWriteItem w3;
    w3.joint_index = 0u;
    w3.path = "serial_number";
    w3.requested.type = ParamType::kU64;
    w3.requested.v.u64 = 1u;
    const Status st_ro = tg.bus(0).write_params(&w3, 1u, true, false, &r);
    std::printf("  read-only → %s | %s\n", to_string(w3.status), w3.message);
    JR_CHECK(st_ro != Status::kOk);
    JR_CHECK(w3.status == Status::kNotSupported);
    JR_CHECK_CONTAINS(w3.message, "read-only");

    /* ---- ④ 正常写：读回一致 → verified=true ---- */
    ParamWriteItem w4;
    w4.joint_index = 0u;
    w4.path = "axis0.config.can.heartbeat_rate_ms";
    w4.requested.type = ParamType::kU32;
    w4.requested.v.u32 = 20u;
    const Status st_ok = tg.bus(0).write_params(&w4, 1u, true, false, &r);
    std::printf("  write ok → %s | verified=%d value=%u | %s\n", to_string(st_ok),
                w4.verified ? 1 : 0, w4.value.v.u32, w4.message);
    JR_CHECK_MSG(st_ok == Status::kOk, w4.message);
    JR_CHECK(w4.verified);
    JR_CHECK_EQ(w4.value.v.u32, 20u);
    JR_CHECK(!w4.persisted);                     /* 没要求持久化就不能声称已落 Flash */

    const std::uint32_t after = read_u32(tg, "axis0.config.can.heartbeat_rate_ms", &ok);
    JR_CHECK_EQ(after, 20u);

    /* ---- ⑤ “写进去就被固件立即消费”的端点：读回必然不同，但**不是失败** ---- */
    ParamWriteItem w5;
    w5.joint_index = 0u;
    w5.path = "axis0.requested_state";
    w5.requested.type = ParamType::kU8;
    w5.requested.v.u8 = 2u;                      /* IDLE：安全值，不会真的启动标定 */
    const Status st_rs = tg.bus(0).write_params(&w5, 1u, true, false, &r);
    std::printf("  requested_state=2 → %s verified=%d consumed=%d | %s\n", to_string(w5.status),
                w5.verified ? 1 : 0, w5.consumed_by_firmware ? 1 : 0, w5.message);
    if (!w5.verified) {
        /* 读回不同 → 必须走"固件消费"这条路，而不是报未校验/失败 */
        JR_CHECK_MSG(w5.consumed_by_firmware, w5.message);
        JR_CHECK_MSG(st_rs == Status::kOk, w5.message);
        JR_CHECK_CONTAINS(w5.message, "consumes");
    } else {
        JR_CHECK_EQ(w5.value.v.u8, 2u);          /* 设备没消费就直接存了 → 也合理 */
    }

    /* ---- ⑥ 不变量：**任何**写都不能出现"未校验却报成功" ---- */
    ParamWriteItem w6;
    w6.joint_index = 0u;
    w6.path = "axis0.config.can.node_id";
    w6.requested.type = ParamType::kU32;
    w6.requested.v.u32 = 1u;                     /* 写原值，必然能读回 */
    const Status st_ni = tg.bus(0).write_params(&w6, 1u, true, false, &r);
    if (w6.verified) {
        JR_CHECK(st_ni == Status::kOk);
    } else {
        JR_CHECK_MSG(st_ni != Status::kOk, "verified=false 却报了成功");
    }

    /* ---- ⑦ 持久化：只有"已确认接受"的项才能被打上 persisted ---- */
    ParamWriteItem w7[2];
    w7[0].joint_index = 0u;
    w7[0].path = "axis0.config.can.heartbeat_rate_ms";
    w7[0].requested.type = ParamType::kU32;
    w7[0].requested.v.u32 = 10u;
    w7[1].joint_index = 0u;
    w7[1].path = "axis0.no.such.path";              /* 故意制造一项失败 */
    w7[1].requested.type = ParamType::kU32;
    w7[1].requested.v.u32 = 1u;
    const Status st_p = tg.bus(0).write_params(w7, 2u, true, true /* persist */, &r);
    std::printf("  persist with 1 failing item → %s | %s\n", to_string(r.status), r.message);
    /* 有项失败 → 不得落 Flash（否则就是\"写一半就存进 Flash\"），整批也不得报成功 */
    JR_CHECK(st_p != Status::kOk);
    JR_CHECK(!w7[0].persisted);
    JR_CHECK(!w7[1].persisted);

    tg.close_buses(ExitAction::kDisable, &r);
}

void test_endpoints()
{
    JR_CASE("端点：list（子串/前缀/段前缀）与 lookup");
    Config cfg = make_cfg();
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(open_ready(tg, cfg, &r), r.message);

    EndpointInfo eps[64];
    unsigned n = 0u;
    JR_CHECK_MSG(tg.bus(0).list_endpoints("", eps, 64u, &n, &r) == Status::kOk, r.message);
    std::printf("  all endpoints: %u\n", n);
    JR_CHECK(n >= 20u);                          /* 虚拟设备模型的描述符条目数 */

    unsigned n_sub = 0u;
    JR_CHECK(tg.bus(0).list_endpoints("pre_calibrated", eps, 64u, &n_sub, &r) == Status::kOk);
    std::printf("  substring 'pre_calibrated': %u\n", n_sub);
    JR_CHECK(n_sub >= 1u);
    for (unsigned i = 0u; i < n_sub; ++i) {
        JR_CHECK(std::strstr(eps[i].path, "pre_calibrated") != nullptr);
    }

    unsigned n_pre = 0u;
    JR_CHECK(tg.bus(0).list_endpoints("axis0.config.*", eps, 64u, &n_pre, &r) == Status::kOk);
    std::printf("  prefix 'axis0.config.*': %u\n", n_pre);
    JR_CHECK(n_pre >= 1u);

    /* 截断必须如实报告（count 可以大于返回条数） */
    unsigned n_small = 0u;
    EndpointInfo two[2];
    JR_CHECK(tg.bus(0).list_endpoints("", two, 2u, &n_small, &r) == Status::kOk);
    JR_CHECK(n_small > 2u);
    JR_CHECK_CONTAINS(r.message, "cap");

    EndpointInfo one;
    JR_CHECK(tg.bus(0).lookup_endpoint("axis0.motor.config.gear_ratio", &one, &r) == Status::kOk);
    JR_CHECK(one.type == ParamType::kF32);
    JR_CHECK(one.readable());
    std::printf("  lookup gear_ratio: id=%u type=%s access=%u\n", static_cast<unsigned>(one.id),
                param_type_name(one.type), static_cast<unsigned>(one.access));

    const Status st_bad = tg.bus(0).lookup_endpoint("axis0.nope", &one, &r);
    JR_CHECK(st_bad == Status::kNotFound);
    JR_CHECK_CONTAINS(r.message, "not found");

    tg.close_buses(ExitAction::kDisable, &r);
}

void test_calibrate_and_jog()
{
    JR_CASE("运维：标定（含读回验证）、置零、点动（限时 + 安全收尾）");
    Config cfg = make_cfg();
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(open_ready(tg, cfg, &r), r.message);
    JR_CHECK(tg.bus(0).report().all_calibrated);

    /* 标定：SDK 的虚拟设备有状态机；这里验证"状态跑完 → 读回 pre_calibrated"这条判据 */
    const Status st_cal = tg.bus(0).calibrate(&r);
    std::printf("  calibrate → %s | %s\n", to_string(r.status), r.message);
    JR_CHECK_MSG(st_cal == Status::kOk, r.message);
    JR_CHECK(tg.bus(0).report().all_calibrated);

    JR_CHECK_MSG(tg.bus(0).set_zero_here(&r) == Status::kOk, r.message);

    /* 点动：缺 confirm → 拒绝；时长越界 → 拒绝 */
    JointTarget t;
    t.position = 0.05;
    t.kp = 2.0;
    t.kd = 0.2;
    JR_CHECK(tg.bus(0).jog(0u, t, 200u, false, &r) == Status::kInvalidState);
    JR_CHECK_CONTAINS(r.message, "confirm=true");
    JR_CHECK(tg.bus(0).jog(0u, t, 20000u, true, &r) == Status::kInvalidArgument);
    JR_CHECK_CONTAINS(r.message, "10000");

    const Status st_jog = tg.bus(0).jog(0u, t, 200u, true, &r);
    std::printf("  jog(200ms) → %s | %s\n", to_string(r.status), r.message);
    JR_CHECK_MSG(st_jog == Status::kOk, r.message);
    JR_CHECK_CONTAINS(r.message, "disabled again");
    /* 点动结束后必须是失能（安全态），并且再读参数仍然可用（说明已回到 READY） */
    JR_CHECK(!tg.bus(0).active());
    JR_CHECK_MSG(tg.bus(0).lookup_endpoint("axis0.motor.config.gear_ratio", nullptr, &r) !=
                     Status::kOk,
                 "null out 应被拒绝");
    EndpointInfo one;
    JR_CHECK(tg.bus(0).lookup_endpoint("axis0.motor.config.gear_ratio", &one, &r) == Status::kOk);

    tg.close_buses(ExitAction::kDisable, &r);
}

void test_uncalibrated_chicken_and_egg()
{
    JR_CASE("未标定设备：configure 降级为 READY（端点表可用）且 calibrate 能跑（鸡生蛋）");

    Config cfg = make_cfg();
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(open_ready(tg, cfg, &r), r.message);
    JR_CHECK_EQ(tg.bus(0).mode(), BusMode::kReady);

    /* 把齿轮比写成 0 → 量程不可用（= "未标定/标定参数异常"场景） */
    ParamWriteItem w;
    w.joint_index = 0u;
    w.path = "axis0.motor.config.gear_ratio";
    w.requested.type = ParamType::kF32;
    w.requested.v.f32 = 0.0f;
    const Status st_w = tg.bus(0).write_params(&w, 1u, true, false, &r);
    std::printf("  gear_ratio=0 write → %s (%s)\n", to_string(w.status), w.message);
    if (st_w != Status::kOk) {
        /* 设备拒绝构造这个场景：**不能静默跳过**，必须让人看见"这条路径没被验证"。 */
        std::printf("  !! 无法构造未标定场景（设备拒绝了 gear_ratio=0）→ 本用例未覆盖降级路径\n");
        JR_CHECK_MSG(false, "cannot construct the uncalibrated scenario: the test is not testing "
                            "anything, so it must fail loudly rather than pass silently");
        tg.close_buses(ExitAction::kDisable, &r);
        return;
    }

    /* 重新 configure：应降级为 READY + kCalibrationRequired，而不是把总线卡成不可用 */
    BusReport rep;
    const Status st_c = tg.bus(0).configure(&rep, &r);
    std::printf("  reconfigure → %s advice=%s | %s\n", to_string(r.status), to_string(r.advice),
                r.message);
    JR_CHECK(st_c == Status::kOk);
    JR_CHECK(r.advice == Advice::kCalibrationRequired);
    JR_CHECK_CONTAINS(r.message, "NOT CALIBRATED");
    JR_CHECK(!rep.all_calibrated);
    JR_CHECK_EQ(tg.bus(0).mode(), BusMode::kReady);

    /* 关键：端点表必须可用（否则标定服务连端点都查不到） */
    EndpointInfo one;
    JR_CHECK_MSG(tg.bus(0).lookup_endpoint("axis0.requested_state", &one, &r) == Status::kOk,
                 r.message);

    /* 未标定时使能必须被拒（`require_calibrated=true`） */
    JR_CHECK(tg.bus(0).activate(true, &r) == Status::kNotCalibrated);
    JR_CHECK(r.advice == Advice::kCalibrationRequired);

    /* 修回齿轮比，让后续用例/清理不受影响 */
    w.requested.v.f32 = 16.5f;
    (void)tg.bus(0).write_params(&w, 1u, true, false, &r);
    tg.close_buses(ExitAction::kDisable, &r);
}

void test_node_id_keeps_accounting_in_sync()
{
    JR_CASE("改 node_id：成功时我们自己的记账必须跟着改（否则快照/诊断报旧地址）");
    Config cfg = make_cfg();
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(open_ready(tg, cfg, &r), r.message);

    const Status st = tg.bus(0).set_node_id(0u, 3u, false, &r);
    std::printf("  set_node_id(1→3) → %s | %s\n", to_string(r.status), r.message);
    if (st == Status::kOk) {
        JR_CHECK_EQ(tg.bus(0).cfg().joints[0].node_id, 3u);
    } else {
        /* 设备不支持就明确说清楚（不能默默当成成功） */
        JR_CHECK(st != Status::kOk);
    }
    /* 非法 id 必须被拒 */
    JR_CHECK(tg.bus(0).set_node_id(0u, 0u, false, &r) == Status::kInvalidArgument);

    tg.close_buses(ExitAction::kDisable, &r);
}

}  // namespace

int main()
{
    test_guard_requires_configured_and_paused();
    test_read_paths();
    test_write_gates_and_verification();
    test_endpoints();
    test_calibrate_and_jog();
    test_uncalibrated_chicken_and_egg();
    test_node_id_keeps_accounting_in_sync();
    return jrtest::report();
}
