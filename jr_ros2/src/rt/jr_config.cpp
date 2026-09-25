/**
 * @file    jr_config.cpp
 * @brief   配置默认值与校验（唯一闸门）
 *
 * 校验原则：**不猜、不放行**。凡"现场一定会出事"的配置（master_id=0、
 * 关节重名、总线没被任何 tick 组接管、选了心跳反馈却把心跳周期设为 0……）
 * 一律拒绝启动，并把**怎么改**写进 message。
 */

#include "jr_ros2/jr_config.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace jr {
namespace {

void append_note(ConfigNotes *notes, const char *fmt, ...) noexcept
{
    if (notes == nullptr) return;
    const std::size_t used = std::strlen(notes->text);
    if (used + 2u >= sizeof(notes->text)) return;

    /* 每条提示独立一行：换行 + 内容（截断由 vsnprintf 保证，且不再写越界）。 */
    const std::size_t remain = sizeof(notes->text) - used;
    notes->text[used] = '\n';
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(notes->text + used + 1u, remain - 1u, fmt, ap);
    va_end(ap);
    notes->text[sizeof(notes->text) - 1u] = '\0';
}

bool non_empty(const char *s) noexcept { return s != nullptr && s[0] != '\0'; }

const char *hal_name(HalKind k) noexcept
{
    switch (k) {
    case HalKind::kVirtual:   return "virtual";
    case HalKind::kSocketCan: return "socketcan";
    case HalKind::kSlcan:     return "slcan";
    case HalKind::kPcan:      return "pcan";
    }
    return "?";
}

}  // namespace

BusCfg default_bus_cfg() noexcept
{
    BusCfg b;
#if defined(_WIN32) || defined(__APPLE__)
    b.hal = HalKind::kPcan;
    std::snprintf(b.channel, sizeof(b.channel), "%s", "PCAN_USBBUS1");
#elif defined(__linux__)
    b.hal = HalKind::kSocketCan;
    std::snprintf(b.channel, sizeof(b.channel), "%s", "can0");
#else
    b.hal = HalKind::kVirtual;
    b.channel[0] = '\0';
#endif
    return b;
}

Config default_config() noexcept
{
    Config c;
    c.bus_count = 0u;
    c.group_count = 0u;
    c.rt = RtCfg{};
    c.command = CommandCfg{};
    c.safety = SafetyCfg{};
    c.lock = LockCfg{};
    return c;
}

const BusCfg *find_bus(const Config &cfg, const char *name) noexcept
{
    if (!non_empty(name)) return nullptr;
    for (unsigned i = 0u; i < cfg.bus_count && i < kMaxBuses; ++i) {
        if (std::strcmp(cfg.buses[i].name, name) == 0) return &cfg.buses[i];
    }
    return nullptr;
}

unsigned bus_joint_base(const Config &cfg, unsigned bus_index) noexcept
{
    unsigned base = 0u;
    for (unsigned i = 0u; i < bus_index && i < cfg.bus_count && i < kMaxBuses; ++i) {
        base += cfg.buses[i].joint_count;
    }
    return base;
}

void lock_key(const BusCfg &bus, char *out, std::size_t cap) noexcept
{
    if (out == nullptr || cap == 0u) return;
    /* virtual：锁的是"同一个名字的那份仿真"，不是物理资源（见头文件注释）。 */
    if (bus.hal == HalKind::kVirtual || bus.channel[0] == '\0') {
        std::snprintf(out, cap, "%s:%s", to_string(bus.hal), bus.name);
        return;
    }
    std::snprintf(out, cap, "%s:%s", to_string(bus.hal), bus.channel);
}

unsigned total_joint_count(const Config &cfg) noexcept
{    unsigned n = 0u;
    for (unsigned i = 0u; i < cfg.bus_count && i < kMaxBuses; ++i) {
        n += cfg.buses[i].joint_count;
    }
    return n;
}

int find_joint(const Config &cfg, const char *name,
               unsigned *bus_index, unsigned *local_index) noexcept
{
    if (!non_empty(name)) return -1;
    for (unsigned b = 0u; b < cfg.bus_count && b < kMaxBuses; ++b) {
        const BusCfg &bus = cfg.buses[b];
        for (unsigned j = 0u; j < bus.joint_count && j < kMaxJointsPerBus; ++j) {
            if (std::strcmp(bus.joints[j].name, name) == 0) {
                if (bus_index != nullptr) *bus_index = b;
                if (local_index != nullptr) *local_index = j;
                return static_cast<int>(bus_joint_base(cfg, b) + j);
            }
        }
    }
    return -1;
}

Result validate_config(const Config &cfg, ConfigNotes *notes) noexcept
{
    Result r;

    if (notes != nullptr) {
        notes->text[0] = '\0';
        notes->has_joint_above_broadcast_id = false;
        notes->has_classic_bus = false;
        notes->has_slcan_bus = false;
        notes->command_timeout_disabled = false;
        notes->rt_disabled = false;
    }

    /* ---------------- 总览 ---------------- */
    if (cfg.bus_count == 0u) {
        r.set(Status::kInvalidArgument, Advice::kNone, "no bus configured");
        return r;
    }
    if (cfg.bus_count > kMaxBuses) {
        r.set(Status::kInvalidArgument, Advice::kNone, "bus_count=%u exceeds max %u",
              cfg.bus_count, kMaxBuses);
        return r;
    }
    if (cfg.group_count == 0u) {
        r.set(Status::kInvalidArgument, Advice::kNone,
              "no tick_group configured: every bus must be driven by exactly one tick group");
        return r;
    }
    if (cfg.group_count > kMaxTickGroups) {
        r.set(Status::kInvalidArgument, Advice::kNone, "group_count=%u exceeds max %u",
              cfg.group_count, kMaxTickGroups);
        return r;
    }
    if (total_joint_count(cfg) > kMaxJoints) {
        r.set(Status::kInvalidArgument, Advice::kNone, "total joints %u exceeds max %u",
              total_joint_count(cfg), kMaxJoints);
        return r;
    }

    /* ---------------- 实时 ---------------- */
    if (cfg.rt.enabled) {
        if (cfg.rt.priority < 1 || cfg.rt.priority > 99) {
            r.set(Status::kInvalidArgument, Advice::kNone,
                  "rt.priority=%d out of range 1..99", cfg.rt.priority);
            return r;
        }
        if (cfg.rt.deadline_policy) {
            r.set(Status::kNotSupported, Advice::kNone,
                  "rt.deadline_policy (SCHED_DEADLINE) is not implemented yet (P2)");
            return r;
        }
    } else {
        if (notes != nullptr) {
            notes->rt_disabled = true;
            append_note(notes, "[warn] rt.enabled=false: running in CFS, jitter is NOT bounded");
        }
    }

    /* ---------------- 每条总线 ---------------- */
    for (unsigned b = 0u; b < cfg.bus_count; ++b) {
        const BusCfg &bus = cfg.buses[b];

        if (!non_empty(bus.name)) {
            r.set(Status::kInvalidArgument, Advice::kNone, "bus[%u]: empty name", b);
            return r;
        }
        if (std::strlen(bus.name) >= kBusNameLen) {
            r.set(Status::kInvalidArgument, Advice::kNone, "bus[%u] '%s': name too long", b,
                  bus.name);
            return r;
        }
        if (bus.master_id == 0u || bus.master_id > 254u) {
            /* SDK 对 master_id = 0 的行为是"设备完全不回复"——现场表现为"设备全哑"。 */
            r.set(Status::kInvalidArgument, Advice::kCheckBusConfig,
                  "bus '%s': master_id=%u invalid (must be 1..254; 0 makes devices never reply)",
                  bus.name, static_cast<unsigned>(bus.master_id));
            return r;
        }
        if (!non_empty(bus.channel) && bus.hal != HalKind::kVirtual) {
            r.set(Status::kInvalidArgument, Advice::kCheckBusConfig,
                  "bus '%s': empty channel/interface for backend %s", bus.name,
                  hal_name(bus.hal));
            return r;
        }
        if (bus.hal == HalKind::kSlcan && bus.serial_baud == 0u) {
            r.set(Status::kInvalidArgument, Advice::kCheckBusConfig,
                  "bus '%s': slcan requires serial_baud (e.g. 115200); it is the SERIAL rate, "
                  "not the CAN rate",
                  bus.name);
            return r;
        }
        if (bus.nominal_bitrate == 0u) {
            r.set(Status::kInvalidArgument, Advice::kCheckBusConfig,
                  "bus '%s': nominal_bitrate=0 (only used for sanity-checking the link, but must "
                  "be set so we can check it)",
                  bus.name);
            return r;
        }
        if (bus.is_fd && bus.data_bitrate == 0u) {
            r.set(Status::kInvalidArgument, Advice::kCheckBusConfig,
                  "bus '%s': is_fd=true but data_bitrate=0", bus.name);
            return r;
        }
        if (bus.max_bus_load <= 0.0 || bus.max_bus_load > 1.0) {
            r.set(Status::kInvalidArgument, Advice::kNone, "bus '%s': max_bus_load=%f out of 0..1",
                  bus.name, bus.max_bus_load);
            return r;
        }

        if (!bus.is_fd) {
            if (notes != nullptr) {
                notes->has_classic_bus = true;
                append_note(notes,
                            "[warn] bus '%s' is Classic: bandwidth is ~10x tighter and the "
                            "descriptor needs ~6839 frames (1-2 s). Production should use CAN FD.",
                            bus.name);
            }
        }
        if (bus.hal == HalKind::kSlcan) {
            if (notes != nullptr) {
                notes->has_slcan_bus = true;
                append_note(notes,
                            "[warn] bus '%s' uses slcan: ASCII serial protocol, ~100-500 fps in "
                            "Classic. Configuration/monitoring only, NOT for high-rate control.",
                            bus.name);
            }
        }

        const bool uses_heartbeat = (bus.feedback == FeedbackPolicy::kBroadcastHeartbeat) ||
                                    (bus.feedback == FeedbackPolicy::kHeartbeatOnly);
        if (uses_heartbeat && bus.heartbeat_ms == 0u) {
            r.set(Status::kInvalidArgument, Advice::kFixBusPlanning,
                  "bus '%s': feedback policy needs heartbeats but heartbeat_ms=0 "
                  "(0 disables the device heartbeat)",
                  bus.name);
            return r;
        }
        if (bus.feedback == FeedbackPolicy::kUnicastPoll && bus.poll_period_ms == 0u) {
            r.set(Status::kInvalidArgument, Advice::kFixBusPlanning,
                  "bus '%s': feedback=unicast_poll but poll_period_ms=0", bus.name);
            return r;
        }

        if (bus.desc.retries > 10u) {
            r.set(Status::kInvalidArgument, Advice::kNone,
                  "bus '%s': desc.retries=%u is unreasonable (>10)", bus.name, bus.desc.retries);
            return r;
        }
        if (bus.desc.timeout_ms > 60000u) {
            r.set(Status::kInvalidArgument, Advice::kNone,
                  "bus '%s': desc.timeout_ms=%u is unreasonable (>60000)", bus.name,
                  bus.desc.timeout_ms);
            return r;
        }
        if (bus.desc.retain == DescRetain::kFiltered &&
            (bus.desc.filter_paths == nullptr || bus.desc.filter_count == 0u)) {
            r.set(Status::kInvalidArgument, Advice::kNone,
                  "bus '%s': desc.retain=filtered requires filter_paths/filter_count "
                  "(otherwise the arena stays empty and every parameter access fails)",
                  bus.name);
            return r;
        }

        /* -------- 关节 -------- */
        if (bus.joint_count == 0u) {
            r.set(Status::kInvalidArgument, Advice::kNone, "bus '%s': no joint configured",
                  bus.name);
            return r;
        }
        if (bus.joint_count > kMaxJointsPerBus) {
            r.set(Status::kInvalidArgument, Advice::kNone,
                  "bus '%s': joint_count=%u exceeds JR_MAX_JOINTS_PER_BUS=%u "
                  "(rebuild with a larger JR_MAX_JOINTS_PER_BUS; it must match the SDK's "
                  "JSDK_MAX_JOINTS_STATIC)",
                  bus.name, bus.joint_count, kMaxJointsPerBus);
            return r;
        }

        for (unsigned j = 0u; j < bus.joint_count; ++j) {
            const JointCfg &jc = bus.joints[j];
            if (!non_empty(jc.name)) {
                r.set(Status::kInvalidArgument, Advice::kNone, "bus '%s': joint[%u] has empty name",
                      bus.name, j);
                return r;
            }
            if (std::strlen(jc.name) >= kJointNameLen) {
                r.set(Status::kInvalidArgument, Advice::kNone, "joint '%s': name too long", jc.name);
                return r;
            }
            if (jc.node_id == 0u || jc.node_id > 254u) {
                r.set(Status::kInvalidArgument, Advice::kCheckBusConfig,
                      "joint '%s': node_id=%u invalid (must be 1..254)", jc.name,
                      static_cast<unsigned>(jc.node_id));
                return r;
            }
            for (unsigned k = 0u; k < j; ++k) {
                if (std::strcmp(bus.joints[k].name, jc.name) == 0) {
                    r.set(Status::kInvalidArgument, Advice::kNone,
                          "bus '%s': duplicate joint name '%s'", bus.name, jc.name);
                    return r;
                }
                if (bus.joints[k].node_id == jc.node_id) {
                    r.set(Status::kInvalidArgument, Advice::kCheckBusConfig,
                          "bus '%s': duplicate node_id %u ('%s' and '%s')", bus.name,
                          static_cast<unsigned>(jc.node_id), bus.joints[k].name, jc.name);
                    return r;
                }
            }
            for (unsigned b2 = 0u; b2 < b; ++b2) {
                for (unsigned j2 = 0u; j2 < cfg.buses[b2].joint_count; ++j2) {
                    if (std::strcmp(cfg.buses[b2].joints[j2].name, jc.name) == 0) {
                        r.set(Status::kInvalidArgument, Advice::kNone,
                              "duplicate joint name '%s' across buses '%s' and '%s'", jc.name,
                              cfg.buses[b2].name, bus.name);
                        return r;
                    }
                }
            }
            if (jc.node_id > kMaxBroadcastNodeId) {
                if (notes != nullptr) {
                    notes->has_joint_above_broadcast_id = true;
                    append_note(notes,
                                "[warn] joint '%s' node_id=%u > %u: it can NOT be addressed by "
                                "the broadcast bitmap, so this joint will always be a separate "
                                "unicast frame (and cannot join a synchronized group)",
                                jc.name, static_cast<unsigned>(jc.node_id),
                                static_cast<unsigned>(kMaxBroadcastNodeId));
                }
            }
            if (jc.has_position_limit && !(jc.position_min < jc.position_max)) {
                r.set(Status::kInvalidArgument, Advice::kNone,
                      "joint '%s': position limit [%f, %f] is not increasing", jc.name,
                      jc.position_min, jc.position_max);
                return r;
            }

            /* 命令模式（ADR-4 二级接口）：非法值拒绝；非 MIT 的后果**必须说出来** ——
               最贵的一条是"用 ~/cmd_mit 控这个关节会被拒"（不是静默不生效），
               以及 CURRENT 不喂设备看门狗（SDK F19）。 */
            switch (jc.mode) {
            case CmdMode::kMit:
                break;
            case CmdMode::kCsp:
            case CmdMode::kCsv:
            case CmdMode::kCst:
                if (notes != nullptr) {
                    notes->has_non_mit_joint = true;
                    /* ⚠ 两条合法路径都要报：`~/cmd`（自研控制器/节点）与 ros2_control
                       （v0.11 起按模式导出接口）。只说前者会让 ros2_control 的客户
                       去翻一个跟他无关的话题。 */
                    append_note(notes,
                                "[info] joint '%s' is configured in '%s' mode: drive it with "
                                "`~/cmd` (JointCommandArray), or through ros2_control (which exports "
                                "this mode's interface only). MIT targets on `~/cmd_mit` for this "
                                "joint are REFUSED — sending MIT frames would silently switch its "
                                "input mode back to MIT.",
                                jc.name, to_string(jc.mode));
                }
                break;
            case CmdMode::kCurrent:
                if (notes != nullptr) {
                    notes->has_non_mit_joint = true;
                    notes->has_current_mode_joint = true;
                    append_note(notes,
                                "[warn] joint '%s' is in CURRENT mode: (1) CURRENT frames do NOT "
                                "feed the device watchdog (SDK F19) — a stalled publisher can leave "
                                "the motor energised without any fault being raised; (2) `hold` / "
                                "`zero_torque` mean ZERO torque in this mode, so the joint will NOT "
                                "hold its position.",
                                jc.name);
                }
                break;
            default:
                r.set(Status::kInvalidArgument, Advice::kNone,
                      "joint '%s': unknown command mode %u (expected 4=mit / 8=csp / 9=csv / "
                      "10=cst / 11=current)",
                      jc.name, static_cast<unsigned>(jc.mode));
                return r;
            }
        }
    }

    /* ---------------- tick 组 ---------------- */
    for (unsigned g = 0u; g < cfg.group_count; ++g) {
        const TickGroupCfg &grp = cfg.groups[g];
        if (!non_empty(grp.name)) {
            r.set(Status::kInvalidArgument, Advice::kNone, "tick_group[%u]: empty name", g);
            return r;
        }
        if (grp.rate_hz == 0u || grp.rate_hz > 4000u) {
            r.set(Status::kInvalidArgument, Advice::kFixBusPlanning,
                  "tick_group '%s': rate_hz=%u out of range 1..4000", grp.name, grp.rate_hz);
            return r;
        }
        if (grp.bus_count == 0u) {
            r.set(Status::kInvalidArgument, Advice::kNone, "tick_group '%s': no bus assigned",
                  grp.name);
            return r;
        }
        if (grp.rate_hz > 2000u) {
            if (notes != nullptr) {
                append_note(notes,
                            "[warn] tick_group '%s': rate_hz=%u > 2000 — check the bus budget "
                            "and the device-side break_timeout before using this in production",
                            grp.name, grp.rate_hz);
            }
        }
        for (unsigned i = 0u; i < grp.bus_count; ++i) {
            if (grp.bus_index[i] >= cfg.bus_count) {
                r.set(Status::kInvalidArgument, Advice::kNone,
                      "tick_group '%s': bus_index[%u]=%u out of range (bus_count=%u)", grp.name, i,
                      grp.bus_index[i], cfg.bus_count);
                return r;
            }
            for (unsigned k = 0u; k < i; ++k) {
                if (grp.bus_index[k] == grp.bus_index[i]) {
                    r.set(Status::kInvalidArgument, Advice::kNone,
                          "tick_group '%s': bus '%s' listed twice", grp.name,
                          cfg.buses[grp.bus_index[i]].name);
                    return r;
                }
            }
        }
    }

    /* 每条总线必须**恰好**被一个组接管：0 次 = 永不 tick；>1 次 = 同 context 多线程（非法）。 */
    for (unsigned b = 0u; b < cfg.bus_count; ++b) {
        unsigned hits = 0u;
        for (unsigned g = 0u; g < cfg.group_count; ++g) {
            for (unsigned i = 0u; i < cfg.groups[g].bus_count; ++i) {
                if (cfg.groups[g].bus_index[i] == b) ++hits;
            }
        }
        if (hits == 0u) {
            r.set(Status::kInvalidArgument, Advice::kNone,
                  "bus '%s' is not assigned to any tick_group: it would never be ticked, so no "
                  "control frame would ever be sent",
                  cfg.buses[b].name);
            return r;
        }
        if (hits > 1u) {
            r.set(Status::kInvalidArgument, Advice::kNone,
                  "bus '%s' is assigned to %u tick_groups: a jsdk_context must be owned by "
                  "exactly ONE thread (SDK constraint)",
                  cfg.buses[b].name, hits);
            return r;
        }
    }

    /* ---------------- 命令与安全 ---------------- */
    if (cfg.command.timeout_ms > 60000u) {
        r.set(Status::kInvalidArgument, Advice::kNone, "command.timeout_ms=%u is unreasonable",
              cfg.command.timeout_ms);
        return r;
    }
    if (cfg.command.timeout_ms == 0u) {
        if (notes != nullptr) {
            notes->command_timeout_disabled = true;
            append_note(notes,
                        "[warn] command.timeout_ms=0: the node will KEEP the last target forever "
                        "if your controller dies. Device-side break_timeout is the only "
                        "remaining protection (and it is disabled by default on the device).");
        }
    }
    if (cfg.safety.on_exit == ExitAction::kNone) {
        if (notes != nullptr) {
            append_note(notes,
                        "[warn] safety.on_exit=none: on shutdown the joints may keep holding "
                        "torque, or a watchdog fault may be latched. Debug only.");
        }
    }
    if (cfg.safety.auto_enable && cfg.safety.on_exit == ExitAction::kNone) {
        r.set(Status::kInvalidArgument, Advice::kNone,
              "safety.auto_enable=true together with on_exit=none: refusing (this combination "
              "can energize joints and then leave them energized)");
        return r;
    }
    if (cfg.lock.allow_shared && notes != nullptr) {
        append_note(notes,
                    "[warn] lock.allow_shared=true: another master (e.g. jsdk-cli) may be on the "
                    "same bus. Devices remember the LAST master id they saw — expect interleaved "
                    "heartbeats and flaky writes.");
    }

    r.set(Status::kOk, Advice::kNone, "ok");
    return r;
}

}  // namespace jr
