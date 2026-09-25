/**
 * @file    test_bus_runtime.cpp
 * @brief   虚拟总线端到端集成测试（本机无硬件即可验证实时链路与安全策略）
 *
 * 覆盖的**证伪用例**（DESIGN §10.2）：
 *   ① 未使能时 tick **不得**产生任何新的 TX 帧；
 *   ② 广播策略下 1 tick 只花 1 条帧（2 关节 → 帧数/‌tick ≈ 1，而不是 2）；
 *   ③ 命令超时后必须动作一次并在快照里可见；
 *   ④ 暂停协议：暂停时关节必须已安全失能且快照能反映 PAUSED；
 *   ⑤ 8 关节 @1 kHz 必须被**拒绝启动**（广播位图寻址不到 + 带宽不够）；
 *   ⑥ 同一条总线被第二个进程/对象抢占时必须报 kLocked。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <thread>

#include "jr_ros2/rt/jr_bus_lock.hpp"
#include "jr_ros2/rt/jr_tick_group.hpp"
#include "jr_test.hpp"

#include "joint_sdk/joint_sdk.h"   /* 只为 sizeof(jsdk_context_storage_t)：见脏堆回归用例 */
/* 虚拟后端：CURRENT 模式只能靠"验发出去的帧"来断言（设备不回响应，见 F19）。 */
#include "joint_sdk/jsdk_hal_builtin.h"

/* ==========================================================================
 * 测试内的**分配器钩子**（默认关闭，只在本文件的那个回归用例里打开）：
 * 打开时所有 `new` 返回**填满 0xAA 的脏内存**。
 *
 * 为什么要这一手：真实踩过一个 bug —— 上下文存储用 `::operator new` 拿（**未初始化**），
 * 而 `jsdk_context_init()` 的首字段 magic 守卫要求"全新/全零"存储（`magic != 0
 * && magic != JSDK_CTX_MAGIC` → INVALID_ARG）。命中脏堆块时就是**偶发**的
 * "context init failed: invalid-argument" —— 而它只报状态码，看着像玄学。
 *
 * 为什么不用"手工污染几个尺寸的堆块再 open"：那是拿分配器行为碰运气。本机试了三种
 * 网格（8 个整数尺寸 / 256 B 粗网格 / size±512 的 8 B 精确带）都**打不中**（假绿），
 * 换 ctest 的 cwd 又时红时不红。换成钩子后，"漏清零"这条路径**必然**暴露。
 *
 * ⚠ 必须是**全局作用域**的替换函数（放进匿名 namespace 就不是替换了）。
 * ========================================================================== */
std::atomic<bool> g_poison_alloc{false};

void *operator new(std::size_t n)
{
    void *p = std::malloc(n != 0u ? n : 1u);
    if (p == nullptr) throw std::bad_alloc();
    if (g_poison_alloc.load(std::memory_order_relaxed)) std::memset(p, 0xAA, n);
    return p;
}
void *operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }

void *operator new(std::size_t n, std::align_val_t al)
{
    const std::size_t a = static_cast<std::size_t>(al);
    void *base = std::malloc(n + a + sizeof(void *));
    if (base == nullptr) throw std::bad_alloc();
    const std::uintptr_t raw = reinterpret_cast<std::uintptr_t>(base) + sizeof(void *);
    void *p = reinterpret_cast<void *>((raw + (a - 1u)) & ~(a - 1u));
    reinterpret_cast<void **>(p)[-1] = base;   /* 释放时要用 */
    if (g_poison_alloc.load(std::memory_order_relaxed)) std::memset(p, 0xAA, n);
    return p;
}
void *operator new[](std::size_t n, std::align_val_t al) { return ::operator new(n, al); }
void operator delete(void *p, std::align_val_t) noexcept
{
    if (p != nullptr) std::free(reinterpret_cast<void **>(p)[-1]);
}
void operator delete[](void *p, std::align_val_t al) noexcept { ::operator delete(p, al); }
void operator delete(void *p, std::size_t, std::align_val_t al) noexcept { ::operator delete(p, al); }
void operator delete[](void *p, std::size_t, std::align_val_t al) noexcept
{
    ::operator delete(p, al);
}

using namespace jr;
using namespace jr::rt;

namespace {

void sleep_ms(unsigned ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/** 等到 tick 计数 ≥ n（上限 deadline_ms），返回是否达成。
 *
 * ⭐ 为什么不用 `sleep_ms(X); JR_CHECK(tick_count >= N)`：那是拿**墙钟当进度**。
 *   tick 线程正是被抢占的那个 —— 主机满载（编译器满载/Defender 扫描）时它可能整段
 *   窗口都拿不到时间片，于是"没跑够 tick"被误判成代码 bug。凡是"先要 tick 跑起来"
 *   的断言，一律改成**等到够为止**再断言。 */
bool wait_ticks(TickGroup &tg, std::uint64_t n, unsigned deadline_ms)
{
    const std::uint64_t t0 = now_ms();
    while (now_ms() - t0 < deadline_ms) {
        const StateSnapshot *s = tg.acquire_snapshot();
        if (s != nullptr && s->rt.tick_count >= n) return true;
        sleep_ms(1u);
    }
    return false;
}

/** 造一个虚拟总线配置：`joints` 个节点（node_id = 1..joints）。 */
Config make_cfg(unsigned joints, unsigned rate_hz)
{
    Config c = default_config();
    c.bus_count = 1u;

    BusCfg &b = c.buses[0];
    b = default_bus_cfg();
    std::snprintf(b.name, sizeof(b.name), "%s", "vbus");
    b.hal = HalKind::kVirtual;
    b.is_fd = true;
    b.master_id = 1u;
    b.joint_count = joints;

    /* 设备模型规格：SDK 的 virtual 后端自带描述符与状态机。
       ⚠ 每个节点的**起始数字是 nodes[] 下标**（0 开始），node_id 用 id= 指定。 */
    char spec[512] = {};
    for (unsigned i = 0u; i < joints; ++i) {
        char one[96] = {};
        std::snprintf(one, sizeof(one), "%s%u:id=%u,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd",
                      (i == 0u) ? "" : ";", i, i + 1u);
        std::strncat(spec, one, sizeof(spec) - std::strlen(spec) - 1u);
    }
    std::snprintf(b.channel, sizeof(b.channel), "%s", spec);

    for (unsigned i = 0u; i < joints; ++i) {
        std::snprintf(b.joints[i].name, sizeof(b.joints[i].name), "j%u", i);
        b.joints[i].node_id = static_cast<std::uint8_t>(i + 1u);
    }

    b.feedback = FeedbackPolicy::kBroadcastHeartbeat;
    b.heartbeat_ms = 5u;
    b.poll_period_ms = 0u;
    b.max_bus_load = 0.60;
    b.desc.cache_enabled = false;   /* 测试不写用户缓存目录 */

    std::snprintf(c.groups[0].name, sizeof(c.groups[0].name), "%s", "grp");
    c.groups[0].rate_hz = rate_hz;
    c.groups[0].bus_index[0] = 0u;
    c.groups[0].bus_count = 1u;
    c.group_count = 1u;

    c.rt.enabled = false;           /* 本机（Windows/CFS）不做实时性断言 */
    c.lock.enabled = false;         /* 锁单独测 */
    c.command.timeout_ms = 1000u;   /* 默认给宽，单测里按需改 */
    return c;
}

void test_end_to_end_virtual_bus()
{
    JR_CASE("端到端：open → configure → tick（未使能零控制帧）→ 暂停/使能/恢复 → 广播 → 关闭");

    Config cfg = make_cfg(2u, 200u);
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(tg.init(cfg, 0u, &r) == Status::kOk, r.message);

    OpenOptions opt;
    opt.check_abi = true;
    opt.enable_lock = false;
    JR_CHECK_MSG(tg.open_buses(opt, &r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.configure_buses(&r) == Status::kOk, r.message);
    std::printf("%s\n", tg.bus_report(0).text);
    JR_CHECK_EQ(tg.bus(0).mode(), BusMode::kReady);
    JR_CHECK(tg.bus(0).report().joint[0].calibrated);   /* 虚拟设备模型已标定 */

    JR_CHECK_MSG(tg.start(nullptr, &r) == Status::kOk, r.message);
    if (!wait_ticks(tg, 5u, 3000u)) {
        const StateSnapshot *sw = tg.acquire_snapshot();
        std::printf("  !! 3 s 内不到 5 个 tick（tick_count=%llu）：tick 线程根本没在跑\n",
                    sw != nullptr ? (unsigned long long)sw->rt.tick_count : 0ull);
    }
    const StateSnapshot *s = tg.acquire_snapshot();
    JR_CHECK(s != nullptr);
    JR_CHECK(s->rt.tick_count >= 5u);
    const std::uint32_t tx_before = s->buses[0].tx_frames;
    const std::uint64_t ticks_before = s->rt.tick_count;

    /* ---- 证伪用例 ①：未使能时 tick 不得发控制帧 ---- */
    JR_CHECK(wait_ticks(tg, ticks_before + 20u, 3000u));   /* 先让 tick 真的跑起来 */
    s = tg.acquire_snapshot();
    JR_CHECK(s->rt.tick_count > ticks_before);
    if (s->buses[0].tx_frames != tx_before) {
        std::printf("  !! tx_frames %u -> %u while disabled (SDK 的 tx_active 门失效？)\n",
                    tx_before, s->buses[0].tx_frames);
    }
    JR_CHECK_EQ(s->buses[0].tx_frames, tx_before);
    JR_CHECK(!s->joints[0].enabled);

    /* ---- 快照必须带关节名（ROS 侧按名字投影消息；漏填会发出**空名字**） ---- */
    JR_CHECK_MSG(std::strcmp(s->joints[0].name, "j0") == 0,
                 "快照里的关节名必须与配置一致（节点/工具都按名字对齐话题与诊断）");
    JR_CHECK_EQ(s->joint_count, 2u);

    /* ---- 证伪用例 ④：暂停协议（先安全失能，再交所有权） ---- */
    JR_CHECK_MSG(tg.pause(&r, 2000u) == Status::kOk, r.message);
    s = tg.acquire_snapshot();
    JR_CHECK_EQ(static_cast<unsigned>(s->mode), static_cast<unsigned>(BusMode::kPaused));
    JR_CHECK(!s->joints[0].enabled);

    /* 暂停期间：由**非 RT 侧**持有 context（这里只验证不崩且状态切换正确） */
    JR_CHECK_MSG(tg.activate(true, &r) == Status::kOk, r.message);
    tg.resume();
    /* 使能是**在 tick 里**落到设备上的：必须等 tick 跑起来再断言（不能拿墙钟当进度）。 */
    {
        const StateSnapshot *sp = tg.acquire_snapshot();
        const std::uint64_t tk = (sp != nullptr) ? sp->rt.tick_count : 0u;
        JR_CHECK(wait_ticks(tg, tk + 20u, 3000u));
    }
    sleep_ms(60u);
    s = tg.acquire_snapshot();
    JR_CHECK_EQ(static_cast<unsigned>(s->mode), static_cast<unsigned>(BusMode::kActive));
    JR_CHECK(s->joints[0].enabled);

    /* ---- 命令下发 + 广播下发：1 tick ≈ 1 帧 ---- */
    CommandSet cmd;
    cmd.seq = 1u;
    cmd.stamp_ns = now_ns();
    cmd.set_mask(0u);
    cmd.set_mask(1u);
    for (unsigned j = 0u; j < 2u; ++j) {
        cmd.joint[j].position = 0.05;
        cmd.joint[j].velocity = 0.0;
        cmd.joint[j].kp = 2.0;
        cmd.joint[j].kd = 0.2;
        cmd.joint[j].torque = 0.0;
        cmd.joint[j].si_gain = false;
    }
    s = tg.acquire_snapshot();
    const std::uint32_t tx0 = s->buses[0].tx_frames;
    const std::uint64_t tk0 = s->rt.tick_count;
    tg.submit_command(cmd);
    sleep_ms(300u);
    s = tg.acquire_snapshot();
    const std::uint32_t tx1 = s->buses[0].tx_frames;
    const std::uint64_t tk1 = s->rt.tick_count;
    const double frames_per_tick = (tk1 > tk0) ? static_cast<double>(tx1 - tx0) /
                                                    static_cast<double>(tk1 - tk0)
                                               : 0.0;
    std::printf("  %llu ticks, %u tx frames → %.2f frames/tick; note='%s'\n",
                (unsigned long long)(tk1 - tk0), tx1 - tx0, frames_per_tick, s->note);
    /* 广播成功时 ≈1.0；若降级为逐关节单播就是 ≈2.0（2 个关节）。 */
    JR_CHECK(frames_per_tick > 0.8 && frames_per_tick < 1.3);
    JR_CHECK_CONTAINS(s->buses[0].last_note, "broadcast");
    JR_CHECK(s->joints[0].online);
    /* 反馈新鲜度只有在"tick 真跑够"时才可判：设备心跳 5 ms，窗口内 tick 太少
       （主机抢占）时不能下结论。 */
    if (tk1 - tk0 >= 20u) {
        JR_CHECK_MSG(s->joints[0].valid_fresh || s->joints[0].age_ms < 1000u,
                     "tick 循环跑了 >=20 tick 后反馈必须新鲜（设备心跳 5 ms）");
    } else {
        jrtest::note_env("窗口内只跑了 %llu tick（<20，设备心跳 5 ms），反馈新鲜度本轮不可判",
                         (unsigned long long)(tk1 - tk0));
    }

    /* ---- 证伪用例 ③：命令超时（停止下发 → 必须动作一次且可见） ---- */
    tg.pause(&r);
    Config cfg_to = cfg;          /* 改超时只能改配置 → 重建一个组太麻烦；
                                     这里直接验证"当前配置下不会误动作"，再用
                                     command.timeout_ms 已为 1000 ms 的默认值验证"未超时不动"。 */
    (void)cfg_to;
    const std::uint64_t t_resume = now_ms();
    tg.resume();
    sleep_ms(100u);
    s = tg.acquire_snapshot();
    /* 这条断言有**前提**：距上次下发命令还不到 1 s 超时门。主机被抢占到 sleep 都被拖成
       几百毫秒时前提就不成立 —— 那时超时门动作是**正确行为**，不能判为失败。 */
    const std::uint64_t since_resume = now_ms() - t_resume;
    if (since_resume < 900u) {
        JR_CHECK(!std::strstr(s->note, "command timeout"));   /* 1 s 超时未到，不应动作 */
    } else {
        jrtest::note_env("resume 后实际过了 %llu ms（≥900 ms）：1 s 命令超时门可能已合法动作，"
                         "该断言本轮不适用",
                         (unsigned long long)since_resume);
    }

    /* ---- 关闭：暂停 → 失能 → 恢复 → 停止 → 按策略关闭 ---- */
    JR_CHECK_MSG(tg.pause(&r, 2000u) == Status::kOk, r.message);
    JR_CHECK(tg.deactivate(&r) == Status::kOk);
    tg.resume();
    sleep_ms(60u);
    tg.stop();
    tg.close_buses(ExitAction::kDisable, &r);
    JR_CHECK_EQ(tg.bus(0).mode(), BusMode::kIdle);
    JR_CHECK(!tg.bus(0).opened());
    std::printf("  closed ok (exit=%s)\n", to_string(r.status));
}

void test_command_timeout_fires()
{
    JR_CASE("命令超时：超时后动作一次、快照可见，且不重复刷帧");

    Config cfg = make_cfg(1u, 100u);
    cfg.command.timeout_ms = 60u;
    cfg.command.on_timeout = TimeoutAction::kHold;

    Result r;
    TickGroup tg;
    JR_CHECK(tg.init(cfg, 0u, &r) == Status::kOk);
    OpenOptions opt;
    opt.check_abi = false;
    opt.enable_lock = false;
    JR_CHECK_MSG(tg.open_buses(opt, &r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.configure_buses(&r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.start(nullptr, &r) == Status::kOk, r.message);

    JR_CHECK(tg.pause(&r) == Status::kOk);
    JR_CHECK_MSG(tg.activate(false, &r) == Status::kOk, r.message);
    tg.resume();
    sleep_ms(80u);

    CommandSet cmd;
    cmd.seq = 1u;
    cmd.stamp_ns = now_ns();
    cmd.set_mask(0u);
    cmd.joint[0].position = 0.1;
    cmd.joint[0].kp = 2.0;
    cmd.joint[0].kd = 0.2;
    tg.submit_command(cmd);
    sleep_ms(150u);   /* > 2 × timeout_ms */

    const StateSnapshot *s = tg.acquire_snapshot();
    std::printf("  note='%s'\n", s->note);
    JR_CHECK_CONTAINS(s->note, "command timeout");

    tg.stop();
    tg.close_buses(ExitAction::kDisable, &r);
}

void test_over_planning_rejected()
{
    JR_CASE("证伪用例⑤：8 关节 @1 kHz（广播寻址不到 + 带宽不足）必须被拒绝启动");

    Config cfg = make_cfg(8u, 1000u);
    Result r;
    TickGroup tg;
    JR_CHECK(tg.init(cfg, 0u, &r) == Status::kOk);

    OpenOptions opt;
    opt.check_abi = false;
    opt.enable_lock = false;
    const Status st = tg.open_buses(opt, &r);
    std::printf("  status=%s advice=%s msg=%s\n", to_string(r.status), to_string(r.advice),
                r.message);
    JR_CHECK(st != Status::kOk);
    JR_CHECK(r.advice == Advice::kFixBusPlanning);
    JR_CHECK_CONTAINS(r.message, "bitmap");
    JR_CHECK_CONTAINS(r.message, "Split this bus");
    tg.close_buses(ExitAction::kNone, &r);
}

void test_lock_excludes_second_owner()
{
    JR_CASE("证伪用例⑥：同一条总线的第二个 owner 必须拿到 kLocked（并告诉我们是谁占着）");

    BusLock a;
    BusLock b;
    char msg_a[320] = {};
    char msg_b[320] = {};

    const Status sa = a.acquire("jr-test-bus", ".", false, msg_a, sizeof(msg_a));
    const Status sb = b.acquire("jr-test-bus", ".", false, msg_b, sizeof(msg_b));

    std::printf("  first : %s | %s\n", to_string(sa), msg_a);
    std::printf("  second: %s | %s\n", to_string(sb), msg_b);
    std::printf("  owner pid on disk = %d\n", a.owner_pid_on_disk());

    JR_CHECK(sa == Status::kOk);
    JR_CHECK(sb == Status::kLocked);
    JR_CHECK_CONTAINS(msg_b, "jsdk-cli");   /* 提示必须点名最常见的误用 */
    JR_CHECK(a.owner_pid_on_disk() > 0);

    /* 允许共享时必须能拿到（显式并行），并且诊断里会持续告警 → 这里只验证能拿到 */
    BusLock c;
    char msg_c[320] = {};
    JR_CHECK(c.acquire("jr-test-bus", ".", true, msg_c, sizeof(msg_c)) == Status::kOk);

    a.release();
    /* 释放后应能被别人拿到 */
    BusLock d;
    char msg_d[320] = {};
    const Status sd = d.acquire("jr-test-bus", ".", false, msg_d, sizeof(msg_d));
    std::printf("  after release: %s | %s\n", to_string(sd), msg_d);
    JR_CHECK_MSG(sd == Status::kOk, msg_d);
}

/* ---- 回归：上下文存储必须**由我们清零**再交给 SDK ----
 * 真实踩过（只在特定时机的重建后首跑时红，看着像"负载抖动"）：
 *   `tg.open_buses(...)` 报 `context init failed: invalid-argument`，并级联到后续
 *   所有断言（整个 section 全红，10+ 条）。根因在 `jsdk_context_init()` 的首字段 magic
 *   守卫：`magic != 0 && magic != JSDK_CTX_MAGIC` → INVALID_ARG（含义是"调用方给了脏存储"）。
 *   而 `::operator new` 给的是**未初始化**内存 —— 复用到非零脏堆块就中招，所以是**偶发**。
 *   SDK 示例用静态数组（天然零初始化），所以这个坑只在动态分配时出现。
 * 本用例把分配器换成"返回脏内存"再 open：**修复前必然红**（已用变异测试验证）。 */
void test_open_after_dirty_heap()
{
    JR_CASE("脏堆下的 open：上下文存储必须清零（否则偶发 context init failed）");

    Config cfg = make_cfg(1u, 200u);
    OpenOptions opt;
    Result r;
    TickGroup tg;

    /* 打开钩子：从此每次 `new` 都是脏内存（0xAA）。
       漏清零的路径必然中招 —— 不靠分配器行为碰运气（试过三种堆污染网格都不可靠）。 */
    g_poison_alloc.store(true, std::memory_order_relaxed);
    const Status ii = tg.init(cfg, 0u, &r);
    const Status io = (ii == Status::kOk) ? tg.open_buses(opt, &r) : ii;
    g_poison_alloc.store(false, std::memory_order_relaxed);

    JR_CHECK_MSG(ii == Status::kOk, r.message);
    JR_CHECK_MSG(io == Status::kOk, r.message);
}

void test_external_tick_mode()
{
    JR_CASE("外部驱动 tick（tick_source=controller_manager）：只有 step() 才跑周期");

    Config cfg = make_cfg(2u, 200u);
    OpenOptions opt;
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(tg.init(cfg, 0u, &r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.open_buses(opt, &r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.configure_buses(&r) == Status::kOk, r.message);

    /* 未声明外部模式时 step() 必须被拒：
       否则同一份 context 可能被“内部线程 + 调用方线程”同时驱动（SDK 明确禁止）。 */
    JR_CHECK(tg.step(&r) == Status::kInvalidState);
    JR_CHECK_CONTAINS(r.message, "external tick mode");

    JR_CHECK_MSG(tg.start_external(nullptr, &r) == Status::kOk, r.message);
    JR_CHECK(tg.external_tick());
    JR_CHECK_CONTAINS(r.message, "external tick mode");
    /* 首个 tick 前没有快照 —— 这条既有语义在外部模式下必须一样。 */
    JR_CHECK(tg.acquire_snapshot() == nullptr);

    /* 外部模式没有线程：不调 step 就一个周期也不会跑（睡 200 ms = 40 个周期的时量）。 */
    sleep_ms(200u);
    JR_CHECK(tg.acquire_snapshot() == nullptr);

    JR_CHECK(tg.step(nullptr) == Status::kOk);
    const StateSnapshot *s = tg.acquire_snapshot();
    JR_CHECK(s != nullptr);
    if (s != nullptr) {
        JR_CHECK_EQ(s->rt.tick_count, 1u);
        JR_CHECK_EQ(static_cast<unsigned>(s->mode), static_cast<unsigned>(BusMode::kReady));
    }

    /* ADR-6：外部模式下命令由**调用方在 write() 里提交、紧接着 step()**，
       即“命令最多晚 1 tick 生效”。这里把“命令不会在 submit 的那一瞬间生效”钉住。 */
    CommandSet cmd;
    cmd.seq = 1u;
    cmd.stamp_ns = now_ns();
    cmd.set_mask(0u);
    cmd.set_mask(1u);
    for (unsigned j = 0u; j < 2u; ++j) {
        cmd.joint[j].position = 0.1;
        cmd.joint[j].velocity = 0.0;
        cmd.joint[j].kp = 2.0;
        cmd.joint[j].kd = 0.2;
        cmd.joint[j].torque = 0.0;
        cmd.joint[j].si_gain = false;
    }
    tg.submit_command(cmd);
    JR_CHECK(tg.step(nullptr) == Status::kOk);
    s = tg.acquire_snapshot();
    JR_CHECK(s != nullptr);
    if (s != nullptr) {
        JR_CHECK_EQ(s->rt.tick_count, 2u);   /* 每 step = 一个 tick，不多不少 */
    }

    /* 外部模式下 pause() 必须**明确地**说“不支持”并给出正确的替代做法，
       而不是做一个半成品的所有权移交（那会变成难查的竞态）。 */
    JR_CHECK(tg.pause(&r) == Status::kNotSupported);
    JR_CHECK_CONTAINS(r.message, "lifecycle");

    tg.stop();
    JR_CHECK(!tg.external_tick());
    JR_CHECK(tg.step(&r) == Status::kInvalidState);
    tg.close_buses(ExitAction::kDisable, &r);
}

/* ==========================================================================
 * 线性插值（`command.interpolation=linear`）
 * ======================================================================== */

void test_linear_interpolation()
{
    JR_CASE("command.interpolation=linear：运动量在 tick 上线性过渡（首条立即生效、增益不插值）");

    /* 外部驱动 tick ⇒ 我们能精确控制"发一条命令 → 跑几拍 → 再发一条"。 */
    Config cfg = make_cfg(1u, 200u);   /* 5 ms/拍 */
    cfg.command.interpolation = Interpolation::kLinear;

    OpenOptions opt;
    Result r;
    TickGroup tg;
    JR_CHECK_MSG(tg.init(cfg, 0u, &r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.open_buses(opt, &r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.configure_buses(&r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.start_external(nullptr, &r) == Status::kOk, r.message);

    /* 使能：外部模式下 `pause()` 不支持，但 `activate()` 是**允许**的 ——
       守卫的语义是"调用方就是所有者，只要没有并发 step()"（内部模式才要求先 pause）。 */
    JR_CHECK_MSG(tg.activate(true, &r) == Status::kOk, r.message);
    jsdk_hal_handle_t *hal = static_cast<jsdk_hal_handle_t *>(tg.bus(0).hal_handle_for_test());
    JR_CHECK(hal != nullptr);
    if (hal == nullptr) return;

    /* 设备量程（虚设备默认 12.5 rad；MIT 帧的定点映射要它才能反解）。 */
    const double pos_max = tg.bus(0).report().joint[0].mit_max_pos;
    JR_CHECK(pos_max > 1.0);
    if (!(pos_max > 1.0)) return;

    /* 提交一条 MIT 目标，把**本 tick 实际下发的目标**（插值后的生效值）读回来。
       ⚠ 为什么不去解线上帧：控制帧的载荷是 **16 字节**（实测 `len=16 id=0x0a000807`），
         不是 8 字节；按长度过滤会把参数帧当成控制帧（就是这么错过一轮）。
         帧格式是 SDK 的职责（它有自己的向量测试）；我们该验的是"交给 SDK 的目标"。 */
    const auto submit_and_applied_pos = [&](std::uint64_t seq, double pos, double kp,
                                            double *out) -> bool {
        CommandSet cmd;
        cmd.seq = seq;
        cmd.stamp_ns = now_ns();
        cmd.set_mask(0u);
        cmd.joint[0].mode = CmdMode::kMit;
        cmd.joint[0].position = pos;
        cmd.joint[0].kp = kp;
        cmd.joint[0].kd = 0.2;
        tg.submit_command(cmd);
        if (tg.step(nullptr) != Status::kOk) return false;
        *out = tg.bus(0).applied_target(0u).position;
        return true;
    };

    /* ---- ① 首条命令必须**立即**到位（没有历史区间可插）----
       这条是安全问题：从 0 慢慢爬到第一条目标，等于让关节"自己动起来"。 */
    double applied = 0.0;
    JR_CHECK(submit_and_applied_pos(1u, 0.30, 2.0, &applied));
    std::printf("  [interp] 首条命令发出去的目标 = %.4f（期望 0.3000）\n", applied);
    JR_CHECK_MSG(std::fabs(applied - 0.30) < 0.01,
                 "首条命令必须立即生效（不是从 0 开始爬）");

    /* ---- ② 第二条命令：给足 40 ms 的"控制器周期"，然后逐拍看它是不是**渐进**过去 ---- */
    /* 命令间隔取 120 ms：它同时决定"插值区间长度"。取大一点是为了**跨主机稳定** ——
       Windows 的 sleep 粒度约 15 ms、Linux 约 1 ms，区间太短会让"中间样本数"贴边。 */
    sleep_ms(120u);
    double samples[12] = {};
    unsigned n = 0u;
    bool got = submit_and_applied_pos(2u, -0.20, 2.0, &samples[n]);
    if (got) ++n;
    /* ⚠ 采样循环里**必须 sleep**：`step()` 本身不消耗时间（空转 10 拍只要几十微秒），
       而插值是按**真实时间**推进的 —— 不睡就永远看到起点值，看起来像"插值没生效"。
       （实测就是这么误判了一轮：目标一直是 0.300，其实是 f≈0.0003。） */
    for (unsigned i = 0u; i < 12u && n < 12u; ++i) {
        sleep_ms(5u);   /* ≈ 一个 tick 周期（本用例 200 Hz） */
        if (tg.step(nullptr) != Status::kOk) break;
        samples[n] = tg.bus(0).applied_target(0u).position;
        ++n;
    }
    JR_CHECK_MSG(n >= 4u, "至少抓到几个中间帧（窗口太窄就说明插值没生效）");

    /* 渐进：中间采样必须**严格落在**起点与终点之间，不能一步到位。 */
    std::printf("  [interp] 抓到 %u 帧，目标序列 = [%.3f %.3f %.3f %.3f %.3f]（期望从 0.30 渐到 -0.20）\n",
                n, samples[0], samples[1], samples[2], samples[3], samples[4]);
    unsigned intermediate = 0u;
    for (unsigned i = 0u; i < n; ++i) {
        const double v = samples[i];
        if (v < 0.29 && v > -0.19) ++intermediate;
    }
    std::printf("  [interp] 严格在中间的目标值 = %u 个（至少 2 个才算“真的在过渡”）\n",
                intermediate);
    JR_CHECK_MSG(intermediate >= 2u,
                 "linear：至少要看到 2 个**严格在中间**的目标值（说明真的在 tick 上过渡，"
                 "不是一步跳到位）");

    /* 终点：继续跑到插值区间结束，目标必须收敛到新值（插值只平滑过程，不改终点）。 */
    double last = samples[n > 0u ? n - 1u : 0u];
    for (unsigned i = 0u; i < 40u; ++i) {
        sleep_ms(5u);
        if (tg.step(nullptr) != Status::kOk) break;
        last = tg.bus(0).applied_target(0u).position;
        if (std::fabs(last - (-0.20)) < 0.005) break;
    }
    std::printf("  [interp] 插值区间结束后的目标 = %.4f（期望 -0.2000）\n", last);
    JR_CHECK_MSG(std::fabs(last - (-0.20)) < 0.02,
                 "最后必须收敛到新目标（插值只能平滑过程，不能改变终点）");

    /* ---- ③ 关掉插值后必须**一步到位**（否则"开关"是假的）---- */
    tg.bus(0).set_interpolation(Interpolation::kNone);
    sleep_ms(40u);
    JR_CHECK(submit_and_applied_pos(3u, 0.25, 2.0, &applied));
    std::printf("  [interp] 关掉插值后的目标 = %.4f（期望 0.2500）\n", applied);
    JR_CHECK_MSG(std::fabs(applied - 0.25) < 0.01,
                 "interpolation=none：目标必须立刻到位（不能残留上一段的插值状态）");

    tg.stop();
    tg.close_buses(ExitAction::kDisable, &r);
}

/* ==========================================================================
 * 模式化命令（ADR-4 二级接口）：CSP / CSV / CST / CURRENT
 * ======================================================================== */

/**
 * 一条总线上四种模式各来一个关节，验三件事：
 *   ① 命令**按各自模式**下发到设备（而不是被当成 MIT 发出去）；
 *   ② 模式不符的目标被**丢弃并计数**（不静默按 MIT 发 —— 那会改掉设备的输入模式）；
 *   ③ 混合模式下广播被降级为单播（MIT 广播帧会把非 MIT 关节换回 MIT 输入模式）。
 *
 * ⚠ 关于虚设备模型的**建模差异**（别把模型的简化当成固件语义）：
 *   - 位置/速度模式：模型真的会动（POS 用 `v = vel_ff + 2·err` 收敛；VEL 直接给速度）；
 *   - 力矩/电流模式：模型的“玩具物理”**不把力矩积分成运动**，只把它体现在
 *     `iq_measured`（= 反馈电流）上 ⇒ CST/CURRENT 的断言只能盯**反馈电流**（符号与量值），
 *     不能断言“关节被力矩推动了”（那是真机验收的事，§10.3）。
 *   - POS 帧里的 `vel_limit/cur_limit` 在协议注释里是**限制量**，而模型把它当速度前馈用
 *     ⇒ 本用例**只透传、不断言限制语义**（真机上才有效果）。
 */
/** 单个模式的用例：j0 = 被测模式，j1 = MIT 对照（证明混合模式下两条路都对）。
 *
 * ⚠ 为什么一个模式一条总线（而不是 4 关节挤一条）：虚设备的描述符下发是**串行泵**的，
 *   4 关节会超出 SDK 默认 5 s 的描述符超时，`configure_buses()` 直接失败
 *   （实测：descriptor download failed after 3 attempt(s): timeout）。
 *   这不是固件/我们代码的问题，但测试不能建在这种吞吐假设上。
 *
 * ⚠ 虚设备模型的**建模差异**（别把模型的简化当成固件语义）：
 *   - 位置/速度：模型真的会动（POS: v = vel_ff + 2·err；VEL: v = 目标速度）
 *     ⇒ 可以对**收敛量**下断言；
 *   - 力矩/电流：模型的“玩具物理”**不把力矩积分成运动**，只把它体现在 `iq_measured`
 *     ⇒ 只能盯**反馈电流**的符号/量值；“关节被力矩推动了”属真机验收（§10.3）；
 *   - POS 帧的 `vel_limit/cur_limit` 在协议注释里是**限制量**，模型却当速度前馈用
 *     ⇒ 本用例只透传、**不断言限制语义**。
 */
void run_mode_case(CmdMode mode, const char *label)
{
    JR_CASE(label);   /* 失败时能看出是哪个模式 */

    Config cfg = make_cfg(2u, 200u);
    cfg.buses[0].joints[0].mode = mode;
    cfg.command.timeout_ms = 5000u;   /* 本用例不测超时：别让 hold 把目标清掉 */

    Result r;
    TickGroup tg;
    JR_CHECK_MSG(tg.init(cfg, 0u, &r) == Status::kOk, r.message);

    OpenOptions opt;
    opt.enable_lock = false;
    JR_CHECK_MSG(tg.open_buses(opt, &r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.configure_buses(&r) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.start(nullptr, &r) == Status::kOk, r.message);

    /* 有界等待：断言一律等**快照上的事实**，不拿墙钟当进度。 */
    const auto wait_until = [&](auto pred, unsigned deadline_ms) {
        const std::uint64_t t0 = now_ms();
        while (now_ms() - t0 < deadline_ms) {
            const StateSnapshot *sp = tg.acquire_snapshot();
            if (sp != nullptr && pred(*sp)) return true;
            sleep_ms(2u);
        }
        const StateSnapshot *sp = tg.acquire_snapshot();
        return sp != nullptr && pred(*sp);
    };

    /* 使能：整总线走阻塞路径 → 每个关节按**自己配置的模式**进闭环。 */
    JR_CHECK_MSG(tg.pause(&r, 2000u) == Status::kOk, r.message);
    JR_CHECK_MSG(tg.activate(true, &r) == Status::kOk, r.message);
    tg.resume();
    JR_CHECK_MSG(wait_until([](const StateSnapshot &s) {
                     return s.joint_count == 2u && s.joints[0].enabled && s.joints[1].enabled;
                 }, 3000u),
                 "两个关节都应使能");

    /* ① 使能时选的模式必须真的落到设备上（配置 → 核心 → SDK → 帧型） */
    {
        const StateSnapshot *s = tg.acquire_snapshot();
        JR_CHECK(s != nullptr);
        if (s != nullptr) {
            JR_CHECK_EQ(static_cast<unsigned>(s->joints[0].cmd_mode), static_cast<unsigned>(mode));
            JR_CHECK_EQ(static_cast<unsigned>(s->joints[0].cmd_rejected), 0u);
            std::printf("  [mode] %s: cmd_mode=%u device mode_state=%u\n", label,
                        static_cast<unsigned>(s->joints[0].cmd_mode),
                        static_cast<unsigned>(s->joints[0].mode_state));
        }
    }

    std::uint64_t seq = 1u;
    const auto submit = [&](unsigned j, CmdMode m, double value) {
        CommandSet c;
        c.seq = seq++;
        c.stamp_ns = now_ns();
        c.joint[j].mode = m;
        switch (m) {
        case CmdMode::kCsp: c.joint[j].position = value; break;
        case CmdMode::kCsv: c.joint[j].velocity = value; break;
        case CmdMode::kCst: c.joint[j].torque = value; break;
        case CmdMode::kCurrent: c.joint[j].current = value; break;
        case CmdMode::kMit:
        default: c.joint[j].position = value; break;
        }
        c.set_mask(j);
        tg.submit_command(c);
    };

    /* ② 按模式下发自己的物理量，并在**该模式的观测量**上断言 */
    switch (mode) {
    case CmdMode::kCsp:
        submit(0u, mode, 0.25);
        JR_CHECK_MSG(wait_until([](const StateSnapshot &s) {
                         return std::fabs(s.joints[0].position - 0.25) < 0.02;
                     }, 3000u),
                     "CSP：位置应在 3 s 内收敛到 0.25 rad");
        break;
    case CmdMode::kCsv:
        submit(0u, mode, 2.0);
        JR_CHECK_MSG(wait_until([](const StateSnapshot &s) {
                         return std::fabs(s.joints[0].velocity - 2.0) < 0.2;
                     }, 3000u),
                     "CSV：速度应在 3 s 内跟到 2.0 rad/s（模型直接给速度 ⇒ 这是量值断言）");
        break;
    case CmdMode::kCst: {
        submit(0u, mode, 5.0);
        const bool pos =
            wait_until([](const StateSnapshot &s) { return s.joints[0].current > 0.05; }, 2000u);
        JR_CHECK_MSG(pos, "CST：+5 N·m 必须让反馈电流变正（力矩真的到了设备）");
        submit(0u, mode, -5.0);
        JR_CHECK_MSG(wait_until([](const StateSnapshot &s) { return s.joints[0].current < -0.05; },
                                2000u),
                     "CST：-5 N·m 必须让反馈电流变负（方向不能反）");
        break;
    }
    case CmdMode::kCurrent: {
        /* ⚠ CURRENT 的观测方式**与其它模式不同**：`CURRENT_CONTROL` 不是 `is_ctrl` 帧（F19），
           设备**不回任何应答**，所以拿不到"设备侧反馈电流"。SDK 自己的测试也是这个口径：
           **验发出去的帧**（`jsdk_hal_virtual_capture`）。
           断言：线上出现一个载荷 = 0.8 A（**电机端**，大端 f32）的 4 字节帧。
           这个用例里没有 CST 关节，所以 4 字节 + 该载荷只可能来自 CURRENT。 */
        const std::uint64_t t0 = now_ms();
        submit(0u, mode, 0.8);
        std::uint32_t hits = 0u;
        while (now_ms() - t0 < 2000u && hits == 0u) {
            jsdk_hal_handle_t *h =
                static_cast<jsdk_hal_handle_t *>(tg.bus(0).hal_handle_for_test());
            jsdk_can_frame_t f;
            while (h != nullptr && jsdk_hal_virtual_capture(h, &f) == 1) {
                if (f.len == 4u) {
                    const std::uint32_t raw = (static_cast<std::uint32_t>(f.data[0]) << 24) |
                                              (static_cast<std::uint32_t>(f.data[1]) << 16) |
                                              (static_cast<std::uint32_t>(f.data[2]) << 8) |
                                              static_cast<std::uint32_t>(f.data[3]);
                    float v = 0.0f;
                    std::memcpy(&v, &raw, sizeof v);
                    if (std::fabs(static_cast<double>(v) - 0.8) < 0.005) ++hits;
                }
            }
            sleep_ms(2u);
        }
        JR_CHECK_MSG(hits > 0u,
                     "CURRENT：线上必须出现载荷 = 0.8 A（**电机端**、大端 f32）的 4 字节帧 —— "
                     "单位/字节序搞错这里就红（设备不回响应，这是唯一能验的东西）");
        break;
    }
    case CmdMode::kMit:
    default:
        JR_CHECK(false);   /* 本用例不覆盖 MIT（一级接口由 e2e 用例负责） */
        break;
    }

    /* ③ 模式不符的目标必须被**丢弃并计数**（不静默按 MIT 发：那会改掉设备的输入模式） */
    {
        const StateSnapshot *s0 = tg.acquire_snapshot();
        const std::uint64_t t0 = (s0 != nullptr) ? s0->rt.tick_count : 0u;
        submit(0u, CmdMode::kMit, -0.90);
        JR_CHECK(wait_ticks(tg, t0 + 20u, 2000u));

        const StateSnapshot *s = tg.acquire_snapshot();
        JR_CHECK(s != nullptr);
        if (s != nullptr) {
            JR_CHECK_MSG(s->joints[0].cmd_rejected >= 1u,
                         "模式不符的目标必须被计数（cmd_rejected）—— 『命令没发出去』的唯一证据");
            /* 混合模式下广播必须降级：MIT 广播帧会把非 MIT 关节换回 MIT 输入模式。 */
            const std::string note(s->buses[0].last_note);
            std::printf("  [mode] %s: bus note='%s'\n", label, note.c_str());
            JR_CHECK_MSG(note.find("not in MIT mode") != std::string::npos,
                         "混合模式下广播必须被明确降级，且理由要写进备注");
        }
    }

    /* ④ MIT 对照关节照旧能动（回归：加模式没破坏一级接口） */
    {
        CommandSet c;
        c.seq = seq++;
        c.stamp_ns = now_ns();
        c.joint[1].mode = CmdMode::kMit;
        c.joint[1].position = 0.10;
        c.joint[1].kp = 2.0;
        c.joint[1].kd = 0.2;
        c.set_mask(1u);
        const StateSnapshot *s1 = tg.acquire_snapshot();
        const std::uint64_t t1 = (s1 != nullptr) ? s1->rt.tick_count : 0u;
        tg.submit_command(c);
        JR_CHECK(wait_ticks(tg, t1 + 20u, 2000u));
        JR_CHECK_MSG(wait_until([](const StateSnapshot &s) { return s.joints[1].position > 0.02; },
                                3000u),
                     "同一个 tick 组里的 MIT 关节（带 PD 增益）必须照旧能动");
    }

    tg.stop();
    tg.close_buses(ExitAction::kDisable, &r);
    JR_CHECK_EQ(static_cast<int>(r.status), static_cast<int>(Status::kOk));
}

void test_mode_commands()
{
    run_mode_case(CmdMode::kCsp, "模式化命令：csp");
    run_mode_case(CmdMode::kCsv, "模式化命令：csv");
    run_mode_case(CmdMode::kCst, "模式化命令：cst");
    run_mode_case(CmdMode::kCurrent, "模式化命令：current");
}

/**
 * 反馈"年龄"的诚实口径（F11，真机）：
 * SDK 会一边报 `FEEDBACK_STALE`（头文件原话“反馈超时（age_ms 超阈值）”）一边报 `age_ms=0`，
 * 于是**冻结在早先时刻的值看起来“刚刚才更新”**。这里钉住我们的口径：
 * 新鲜 ⇒ 用设备侧年龄；陈旧从未新鲜过 ⇒ `kFeedbackAgeUnknown`（不编数字）；
 * 陈旧但有历史 ⇒ 真实经过时间（并饱和）。
 */
void test_feedback_age_is_honest()
{
    JR_CASE("反馈年龄：陈旧时不许报 0（F11）");
    constexpr std::uint64_t kS = 1000000000ull;
    constexpr std::uint64_t kJan = 5ull * kS;   /* 进程起点附近的某个“从未”时刻 */

    /* ① 新鲜：直接用设备侧年龄（它比我们更有意义） */
    JR_CHECK_EQ(feedback_age_ms(kJan, 0u, false, 37u), 37u);

    /* ② 陈旧 + 从未见过新鲜帧 ⇒ 不许编一个数字（历史上这里报的是 0，看着像“刚刚”） */
    JR_CHECK_EQ(feedback_age_ms(kJan, 0u, true, 0u), kFeedbackAgeUnknown);

    /* ③ 陈旧 + 有历史 ⇒ 真实经过时间；**关键断言：不可能是 0** */
    const std::uint64_t fresh = 100ull * kS;
    const std::uint32_t age = feedback_age_ms(fresh + 2500ull * 1000000ull, fresh, true, 0u);
    JR_CHECK_EQ(age, 2500u);
    JR_CHECK_MSG(age != 0u, "陈旧帧的年龄不许是 0");

    /* ④ 饱和：极端经过时间不能溢出成“新鲜” */
    const std::uint32_t big = feedback_age_ms(~0ull, 1ull, true, 0u);
    JR_CHECK_EQ(big, kFeedbackAgeStaleCap);
    JR_CHECK_MSG(big != kFeedbackAgeUnknown, "饱和值必须与“未知”哨兵区分开");
}

}  // namespace

int main()
{
    test_open_after_dirty_heap();   /* 先跑：模拟"进程里第一次 open"（真实故障就是这般） */
    test_end_to_end_virtual_bus();
    test_mode_commands();
    test_external_tick_mode();
    test_linear_interpolation();
    test_command_timeout_fires();
    test_over_planning_rejected();
    test_lock_excludes_second_owner();
    test_feedback_age_is_honest();
    return jrtest::report();
}
