/**
 * @file    jr_config_yaml.cpp
 * @brief   YAML → `jr::Config`（schema 见 DESIGN §6.4）
 *
 * 两条硬规则（都来自踩过的坑）：
 *  ① **未知键一律报错**，并且错误信息必须带**键路径**（`buses[0].is_FD` 这种拼错要能一眼看到）；
 *  ② 属于**节点层**的键（发布频率等）不是错误，但必须**报出来**（`YamlLoadReport`），
 *     不能让客户以为写了就生效了。
 */

#include "jr_ros2/jr_config_yaml.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

namespace jr {
namespace {

/** 加载上下文：错误（只留第一条，避免噪音）+ 节点层键与取值。 */
struct Loader {
    std::string err;
    std::vector<std::string> node_keys;
    NodeCfg node = {};
    bool failed = false;

    bool fail(const std::string &msg)
    {
        if (!failed) {
            err = msg;
            failed = true;
        }
        return false;
    }
    void node_key(const std::string &path) { node_keys.push_back(path); }
};

std::string idx(const std::string &base, std::size_t i)
{
    char buf[96] = {};
    std::snprintf(buf, sizeof(buf), "%s[%zu]", base.c_str(), i);
    return std::string(buf);
}

/** 键名白名单：不在名单里 → 报错（拼错不许静默忽略）。 */
bool keys_ok(Loader &L, const YAML::Node &n, const std::string &where,
             std::initializer_list<const char *> allowed)
{
    if (!n.IsMap()) return L.fail(where + ": expected a mapping (key: value)");
    for (auto it = n.begin(); it != n.end(); ++it) {
        const std::string k = it->first.as<std::string>();
        bool ok = false;
        for (const char *a : allowed) {
            if (k == a) {
                ok = true;
                break;
            }
        }
        if (!ok) return L.fail(where + ": unknown key '" + k + "'");
    }
    return true;
}

/* ---- 取值助手（缺键 = 用默认值；类型不符 = 报错） ---- */

bool need_bool(Loader &L, const YAML::Node &p, const std::string &where, const char *key, bool &out)
{
    const YAML::Node n = p[key];
    if (!n) return true;
    try {
        out = n.as<bool>();
    } catch (const YAML::Exception &e) {
        return L.fail(where + "." + key + ": expected bool (" + e.msg + ")");
    }
    return true;
}

bool need_u32(Loader &L, const YAML::Node &p, const std::string &where, const char *key,
              std::uint32_t &out)
{
    const YAML::Node n = p[key];
    if (!n) return true;
    try {
        const long long v = n.as<long long>();
        if (v < 0) return L.fail(where + "." + key + ": must be >= 0");
        out = static_cast<std::uint32_t>(v);
    } catch (const YAML::Exception &e) {
        return L.fail(where + "." + key + ": expected an integer (" + e.msg + ")");
    }
    return true;
}

bool need_int(Loader &L, const YAML::Node &p, const std::string &where, const char *key, int &out)
{
    const YAML::Node n = p[key];
    if (!n) return true;
    try {
        out = n.as<int>();
    } catch (const YAML::Exception &e) {
        return L.fail(where + "." + key + ": expected an integer (" + e.msg + ")");
    }
    return true;
}

bool need_double(Loader &L, const YAML::Node &p, const std::string &where, const char *key,
                 double &out)
{
    const YAML::Node n = p[key];
    if (!n) return true;
    try {
        out = n.as<double>();
    } catch (const YAML::Exception &e) {
        return L.fail(where + "." + key + ": expected a number (" + e.msg + ")");
    }
    return true;
}

bool need_str(Loader &L, const YAML::Node &p, const std::string &where, const char *key, char *out,
              std::size_t cap)
{
    const YAML::Node n = p[key];
    if (!n) return true;
    try {
        const std::string s = n.as<std::string>();
        if (s.size() >= cap) {
            return L.fail(where + "." + key + ": too long (" + std::to_string(s.size()) +
                          " chars, max " + std::to_string(cap - 1u) + ")");
        }
        std::snprintf(out, cap, "%s", s.c_str());
    } catch (const YAML::Exception &e) {
        return L.fail(where + "." + key + ": expected a string (" + e.msg + ")");
    }
    return true;
}

/** 字符串枚举：值不在表里 → 报错并列出**合法取值**（客户不用去翻文档）。 */
bool need_enum(Loader &L, const YAML::Node &p, const std::string &where, const char *key,
               std::initializer_list<const char *> allowed, std::string &out)
{
    const YAML::Node n = p[key];
    if (!n) return true;
    std::string v;
    try {
        v = n.as<std::string>();
    } catch (const YAML::Exception &e) {
        return L.fail(where + "." + key + ": expected a string (" + e.msg + ")");
    }
    std::string list;
    for (const char *a : allowed) {
        if (v == a) {
            out = v;
            return true;
        }
        if (!list.empty()) list += " | ";
        list += a;
    }
    return L.fail(where + "." + key + ": unknown value '" + v + "' (expected: " + list + ")");
}

std::string keys_text(const std::vector<std::string> &v)
{
    std::string s;
    for (const std::string &k : v) {
        if (!s.empty()) s += ", ";
        s += k;
    }
    return s;
}

/* ---- 各段 ---- */

bool read_rt(Loader &L, const YAML::Node &jr, Config &c)
{
    const YAML::Node n = jr["rt"];
    if (!n) return true;
    if (!keys_ok(L, n, "rt",
                 {"enabled", "policy", "priority", "mlock", "cpu_affinity", "warn_if_throttled"})) {
        return false;
    }
    if (!need_bool(L, n, "rt", "enabled", c.rt.enabled)) return false;
    if (!need_int(L, n, "rt", "priority", c.rt.priority)) return false;
    if (!need_bool(L, n, "rt", "mlock", c.rt.mlock)) return false;
    if (!need_bool(L, n, "rt", "warn_if_throttled", c.rt.warn_if_throttled)) return false;

    std::string policy;
    if (!need_enum(L, n, "rt", "policy", {"fifo", "other", "deadline"}, policy)) return false;
    if (policy == "other") {
        c.rt.enabled = false;      /* 普通调度：等价于"别做实时设置" */
    } else if (policy == "deadline") {
        c.rt.deadline_policy = true;   /* P2 未实现 → validate_config 会拒绝并说明 */
    }

    /* cpu_affinity: 每个 tick 组依次取一个（与 DESIGN §6.4 一致） */
    const YAML::Node aff = n["cpu_affinity"];
    if (aff) {
        if (!aff.IsSequence()) return L.fail("rt.cpu_affinity: expected a list, e.g. [3, 4]");
        for (std::size_t i = 0u; i < aff.size(); ++i) {
            if (i >= kMaxTickGroups) {
                return L.fail("rt.cpu_affinity: at most " + std::to_string(kMaxTickGroups) +
                              " entries (one per tick group)");
            }
            try {
                c.groups[i].cpu = aff[i].as<int>();
            } catch (const YAML::Exception &e) {
                return L.fail(idx("rt.cpu_affinity", i) + ": expected an integer (" + e.msg + ")");
            }
        }
    }
    return true;
}

/** 总线：先建名字→下标表（tick_groups / joints 都要按名字引用它）。 */
bool read_buses(Loader &L, const YAML::Node &jr, Config &c, std::vector<std::string> &names)
{
    const YAML::Node list = jr["buses"];
    if (!list) return L.fail("buses: missing (at least one bus is required)");
    if (!list.IsSequence()) return L.fail("buses: expected a list");
    if (list.size() > kMaxBuses) {
        return L.fail("buses: too many (" + std::to_string(list.size()) + ", max " +
                      std::to_string(kMaxBuses) + ")");
    }

    for (std::size_t i = 0u; i < list.size(); ++i) {
        const YAML::Node b = list[i];
        const std::string where = idx("buses", i);
        if (!keys_ok(L, b, where,
                     {"name", "type", "interface", "spec", "is_fd", "master_id", "bitrate",
                      "joints", "feedback", "heartbeat_ms", "poll_period_ms", "max_bus_load",
                      "serial_baud", "auto_keepalive", "clamp_target", "arm_device_watchdog",
                      "state_timeout_ms", "rx_burst_limit"})) {
            return false;
        }

        BusCfg &out = c.buses[i];
        out = default_bus_cfg();

        if (!need_str(L, b, where, "name", out.name, sizeof(out.name))) return false;
        if (out.name[0] == '\0') return L.fail(where + ".name: required");

        std::string type;
        if (!need_enum(L, b, where, "type", {"socketcan", "pcan", "slcan", "virtual"}, type)) {
            return false;
        }
        if (type.empty()) type = "socketcan";
        if (type == "socketcan") out.hal = HalKind::kSocketCan;
        else if (type == "pcan") out.hal = HalKind::kPcan;
        else if (type == "slcan") out.hal = HalKind::kSlcan;
        else out.hal = HalKind::kVirtual;

        if (out.hal == HalKind::kVirtual) {
            if (!need_str(L, b, where, "spec", out.channel, sizeof(out.channel))) return false;
            if (out.channel[0] == '\0') {
                return L.fail(where + ".spec: required for type 'virtual' (e.g. "
                              "\"0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd\")");
            }
        } else {
            if (!need_str(L, b, where, "interface", out.channel, sizeof(out.channel))) return false;
            if (out.channel[0] == '\0') {
                return L.fail(where + ".interface: required (e.g. can0 / PCAN_USBBUS1 / /dev/ttyACM0)");
            }
        }
        if (!need_u32(L, b, where, "serial_baud", out.serial_baud)) return false;
        if (!need_bool(L, b, where, "is_fd", out.is_fd)) return false;
        {
            std::uint32_t mid = out.master_id;
            if (!need_u32(L, b, where, "master_id", mid)) return false;
            if (mid == 0u || mid > 254u) {
                return L.fail(where + ".master_id: must be 1..254 (0 makes devices never reply)");
            }
            out.master_id = static_cast<std::uint8_t>(mid);
        }
        if (!need_bool(L, b, where, "auto_keepalive", out.auto_keepalive)) return false;
        if (!need_bool(L, b, where, "clamp_target", out.clamp_target)) return false;
        if (!need_bool(L, b, where, "arm_device_watchdog", out.arm_device_watchdog)) return false;
        if (!need_u32(L, b, where, "state_timeout_ms", out.state_timeout_ms)) return false;
        {
            /* rx_burst_limit 是 u8：先按 u32 读再查值域，拒绝的同时告诉客户合法范围。 */
            std::uint32_t burst = out.rx_burst_limit;
            if (!need_u32(L, b, where, "rx_burst_limit", burst)) return false;
            if (burst > 255u) return L.fail(where + ".rx_burst_limit: must be 0..255 (0 = SDK default)");
            out.rx_burst_limit = static_cast<std::uint8_t>(burst);
        }
        if (!need_double(L, b, where, "max_bus_load", out.max_bus_load)) return false;

        const YAML::Node br = b["bitrate"];
        if (br) {
            if (!keys_ok(L, br, where + ".bitrate", {"nominal", "data"})) return false;
            if (!need_u32(L, br, where + ".bitrate", "nominal", out.nominal_bitrate)) return false;
            if (!need_u32(L, br, where + ".bitrate", "data", out.data_bitrate)) return false;
        }

        /* 反馈策略可以在总线上覆盖全局 feedback（总线上没有就沿用全局）。 */
        std::string src;
        if (!need_enum(L, b, where, "feedback",
                       {"broadcast_plus_heartbeat", "unicast_poll", "heartbeat_only",
                        "unicast_only", "broadcast_heartbeat"},
                       src)) {
            return false;
        }
        if (!src.empty()) {
            if (src == "broadcast_plus_heartbeat" || src == "broadcast_heartbeat") {
                out.feedback = FeedbackPolicy::kBroadcastHeartbeat;
            } else if (src == "unicast_poll") {
                out.feedback = FeedbackPolicy::kUnicastPoll;
            } else if (src == "heartbeat_only") {
                out.feedback = FeedbackPolicy::kHeartbeatOnly;
            } else {
                out.feedback = FeedbackPolicy::kUnicastOnly;
            }
        }
        if (!need_u32(L, b, where, "heartbeat_ms", out.heartbeat_ms)) return false;
        if (!need_u32(L, b, where, "poll_period_ms", out.poll_period_ms)) return false;

        const YAML::Node js = b["joints"];
        if (js) {
            if (!js.IsSequence()) {
                return L.fail(where + ".joints: expected a list of joint names");
            }
            if (js.size() > kMaxJointsPerBus) {
                return L.fail(where + ".joints: too many (" + std::to_string(js.size()) +
                              ", max " + std::to_string(kMaxJointsPerBus) + ")");
            }
        }

        names.push_back(std::string(out.name));
    }

    c.bus_count = static_cast<unsigned>(list.size());
    return true;
}

/** 关节表（名字 → 总线 + node_id），并把每个关节挂到所属总线上。 */
bool read_joints(Loader &L, const YAML::Node &jr, Config &c, const std::vector<std::string> &bus_names)
{
    const YAML::Node list = jr["joints"];
    if (!list) return L.fail("joints: missing (each joint needs a bus and a node_id)");
    if (!list.IsSequence()) return L.fail("joints: expected a list");

    for (std::size_t i = 0u; i < list.size(); ++i) {
        const YAML::Node j = list[i];
        const std::string where = idx("joints", i);
        if (!keys_ok(L, j, where, {"name", "bus", "node_id", "mode"})) return false;

        char name[kJointNameLen] = {};
        char bus[kBusNameLen] = {};
        if (!need_str(L, j, where, "name", name, sizeof(name))) return false;
        if (!need_str(L, j, where, "bus", bus, sizeof(bus))) return false;
        if (name[0] == '\0') return L.fail(where + ".name: required");
        if (bus[0] == '\0') return L.fail(where + ".bus: required");

        std::uint32_t node_id = 0u;
        const YAML::Node nid = j["node_id"];
        if (!nid) return L.fail(where + ".node_id: required");
        if (!need_u32(L, j, where, "node_id", node_id)) return false;
        if (node_id == 0u || node_id > 254u) {
            return L.fail(where + ".node_id: must be 1..254 (0 makes devices never reply)");
        }

        unsigned bus_index = kMaxBuses;
        for (unsigned b = 0u; b < c.bus_count; ++b) {
            if (bus_names[b] == bus) {
                bus_index = b;
                break;
            }
        }
        if (bus_index == kMaxBuses) {
            return L.fail(where + ".bus: unknown bus '" + std::string(bus) + "' (known: " +
                          keys_text(bus_names) + ")");
        }

        BusCfg &b = c.buses[bus_index];
        if (b.joint_count >= kMaxJointsPerBus) {
            return L.fail(where + ": bus '" + bus + "' already has " +
                          std::to_string(b.joint_count) + " joints (max " +
                          std::to_string(kMaxJointsPerBus) + ")");
        }
        JointCfg &jc = b.joints[b.joint_count];
        jc = JointCfg{};
        std::snprintf(jc.name, sizeof(jc.name), "%s", name);
        jc.node_id = static_cast<std::uint8_t>(node_id);

        /* 命令模式（ADR-4 二级接口）。缺键 = MIT（一级接口，兼容存量配置）。
           `need_enum` 对缺键返回 true 且**不写 out** → 先给默认值。 */
        std::string mode_text = "mit";
        if (!need_enum(L, j, where, "mode", {"mit", "csp", "csv", "cst", "current"}, mode_text)) {
            return false;
        }
        if (mode_text == "csp") jc.mode = CmdMode::kCsp;
        else if (mode_text == "csv") jc.mode = CmdMode::kCsv;
        else if (mode_text == "cst") jc.mode = CmdMode::kCst;
        else if (mode_text == "current") jc.mode = CmdMode::kCurrent;
        else jc.mode = CmdMode::kMit;

        ++b.joint_count;
    }
    return true;
}

/** 软限位 / timeout-hold 的刚度阻尼（按关节名引用，必须已在上面的 joints 表里）。 */
bool read_limits(Loader &L, const YAML::Node &jr, Config &c)
{
    const YAML::Node lim = jr["limits"];
    if (!lim) return true;
    if (!lim.IsMap()) return L.fail("limits: expected a mapping of joint name -> limits");

    for (auto it = lim.begin(); it != lim.end(); ++it) {
        const std::string jname = it->first.as<std::string>();
        const YAML::Node l = it->second;
        const std::string where = "limits." + jname;
        if (!keys_ok(L, l, where, {"position", "velocity", "effort", "stiffness", "damping"})) {
            return false;
        }

        JointCfg *jc = nullptr;
        for (unsigned b = 0u; b < c.bus_count && jc == nullptr; ++b) {
            for (unsigned k = 0u; k < c.buses[b].joint_count; ++k) {
                if (std::strcmp(c.buses[b].joints[k].name, jname.c_str()) == 0) {
                    jc = &c.buses[b].joints[k];
                    break;
                }
            }
        }
        if (jc == nullptr) {
            return L.fail(where + ": unknown joint '" + jname + "' (declare it in 'joints:' first)");
        }

        const YAML::Node pos = l["position"];
        if (pos) {
            if (!pos.IsSequence() || pos.size() != 2u) {
                return L.fail(where + ".position: expected [min, max]");
            }
            try {
                jc->position_min = pos[0].as<double>();
                jc->position_max = pos[1].as<double>();
            } catch (const YAML::Exception &e) {
                return L.fail(where + ".position: expected numbers (" + e.msg + ")");
            }
            jc->has_position_limit = true;
        }
        double unused_velocity = 0.0;
        double unused_effort = 0.0;
        /* 只做**校验**：速度/力矩限位由设备量程与控制器共同决定（§8.3），不在这里覆盖。 */
        if (!need_double(L, l, where, "velocity", unused_velocity)) return false;
        if (!need_double(L, l, where, "effort", unused_effort)) return false;
        if (!need_double(L, l, where, "stiffness", jc->stiffness)) return false;
        if (!need_double(L, l, where, "damping", jc->damping)) return false;
    }
    return true;
}

bool read_tick_groups(Loader &L, const YAML::Node &jr, Config &c,
                      const std::vector<std::string> &bus_names)
{
    const YAML::Node list = jr["tick_groups"];
    if (!list) return L.fail("tick_groups: missing (at least one group is required)");
    if (!list.IsSequence()) return L.fail("tick_groups: expected a list");
    if (list.size() > kMaxTickGroups) {
        return L.fail("tick_groups: too many (" + std::to_string(list.size()) + ", max " +
                      std::to_string(kMaxTickGroups) + ")");
    }

    for (std::size_t i = 0u; i < list.size(); ++i) {
        const YAML::Node g = list[i];
        const std::string where = idx("tick_groups", i);
        if (!keys_ok(L, g, where, {"name", "rate_hz", "buses", "cpu", "priority"})) return false;

        TickGroupCfg &out = c.groups[i];
        const int keep_cpu = out.cpu;           /* 可能已被 rt.cpu_affinity 填过 */
        const int keep_prio = out.priority;
        out = TickGroupCfg{};
        out.cpu = keep_cpu;
        out.priority = keep_prio;

        if (!need_str(L, g, where, "name", out.name, sizeof(out.name))) return false;
        if (!need_u32(L, g, where, "rate_hz", out.rate_hz)) return false;
        if (!need_int(L, g, where, "cpu", out.cpu)) return false;
        if (!need_int(L, g, where, "priority", out.priority)) return false;

        const YAML::Node bs = g["buses"];
        if (!bs) return L.fail(where + ".buses: required (which buses share this time base)");
        if (!bs.IsSequence()) return L.fail(where + ".buses: expected a list of bus names");
        if (bs.size() > kMaxBuses) return L.fail(where + ".buses: too many");
        for (std::size_t k = 0u; k < bs.size(); ++k) {
            const std::string bname = bs[k].as<std::string>();
            unsigned bus_index = kMaxBuses;
            for (unsigned b = 0u; b < c.bus_count; ++b) {
                if (bus_names[b] == bname) {
                    bus_index = b;
                    break;
                }
            }
            if (bus_index == kMaxBuses) {
                return L.fail(idx(where + ".buses", k) + ": unknown bus '" + bname + "' (known: " +
                              keys_text(bus_names) + ")");
            }
            out.bus_index[k] = bus_index;
            out.bus_count = static_cast<unsigned>(k) + 1u;
        }
    }

    c.group_count = static_cast<unsigned>(list.size());
    return true;
}

bool read_feedback(Loader &L, const YAML::Node &jr, Config &c)
{
    const YAML::Node n = jr["feedback"];
    if (!n) return true;
    if (!keys_ok(L, n, "feedback",
                 {"source", "heartbeat_ms", "poll_period_ms", "publish_hz", "joint_state_hz"})) {
        return false;
    }

    std::string src;
    if (!need_enum(L, n, "feedback", "source",
                   {"broadcast_plus_heartbeat", "unicast_poll", "heartbeat_only", "unicast_only",
                    "broadcast_heartbeat"},
                   src)) {
        return false;
    }
    FeedbackPolicy fp = FeedbackPolicy::kBroadcastHeartbeat;
    if (src == "unicast_poll") fp = FeedbackPolicy::kUnicastPoll;
    else if (src == "heartbeat_only") fp = FeedbackPolicy::kHeartbeatOnly;
    else if (src == "unicast_only") fp = FeedbackPolicy::kUnicastOnly;
    if (!src.empty()) {
        for (unsigned b = 0u; b < c.bus_count; ++b) c.buses[b].feedback = fp;
    }

    std::uint32_t hb = 5u;
    std::uint32_t poll = 10u;
    if (!need_u32(L, n, "feedback", "heartbeat_ms", hb)) return false;
    if (!need_u32(L, n, "feedback", "poll_period_ms", poll)) return false;
    if (n["heartbeat_ms"]) {
        for (unsigned b = 0u; b < c.bus_count; ++b) c.buses[b].heartbeat_ms = hb;
    }
    if (n["poll_period_ms"]) {
        for (unsigned b = 0u; b < c.bus_count; ++b) c.buses[b].poll_period_ms = poll;
    }

    /* 发布频率属于**节点层**（ros2_control 组件不发布这些话题）→ 解析进 `NodeCfg`，
       同时把键名列出来（日志/诊断要能看到"哪些键属于节点"）。
       范围检查的理由：0 = 不发（那为什么写这个键？）不符预期；大于 5000 只会白烧 CPU。 */
    if (n["publish_hz"]) {
        L.node_key("feedback.publish_hz");
        if (!need_u32(L, n, "feedback", "publish_hz", L.node.publish_hz)) return false;
        if (L.node.publish_hz == 0u || L.node.publish_hz > 5000u) {
            return L.fail("feedback.publish_hz: must be in 1..5000 (got " +
                          std::to_string(L.node.publish_hz) + ")");
        }
    }
    if (n["joint_state_hz"]) {
        L.node_key("feedback.joint_state_hz");
        if (!need_u32(L, n, "feedback", "joint_state_hz", L.node.joint_state_hz)) return false;
        if (L.node.joint_state_hz == 0u || L.node.joint_state_hz > 5000u) {
            return L.fail("feedback.joint_state_hz: must be in 1..5000 (got " +
                          std::to_string(L.node.joint_state_hz) + ")");
        }
    }
    return true;
}

bool read_command(Loader &L, const YAML::Node &jr, Config &c)
{
    const YAML::Node n = jr["command"];
    if (!n) return true;
    if (!keys_ok(L, n, "command", {"interpolation", "timeout_ms", "timeout_action"})) return false;

    if (!need_u32(L, n, "command", "timeout_ms", c.command.timeout_ms)) return false;

    std::string act;
    if (!need_enum(L, n, "command", "timeout_action",
                   {"hold", "zero_torque", "disable", "estop"}, act)) {
        return false;
    }
    if (act == "zero_torque") c.command.on_timeout = TimeoutAction::kZeroTorque;
    else if (act == "disable") c.command.on_timeout = TimeoutAction::kDisable;
    else if (act == "estop") c.command.on_timeout = TimeoutAction::kEstop;
    else if (act == "hold") c.command.on_timeout = TimeoutAction::kHold;

    /* 插值器（v0.13 起由**核心库**在 tick 上实现，不再是“节点层键”）：
       —— `c.command.interpolation` 就是它的归属，节点/ros2_control 共用同一个开关。
       写成拼错的枚举（如 `linearr`）必须报错，不能静默不生效。 */
    if (n["interpolation"]) {
        std::string interp;
        if (!need_enum(L, n, "command", "interpolation", {"none", "linear"}, interp)) return false;
        if (interp == "linear") c.command.interpolation = Interpolation::kLinear;
        else if (!interp.empty()) c.command.interpolation = Interpolation::kNone;
    }
    return true;
}

bool read_safety(Loader &L, const YAML::Node &jr, Config &c)
{
    const YAML::Node n = jr["safety"];
    if (!n) return true;
    if (!keys_ok(L, n, "safety",
                 {"auto_enable", "require_calibrated", "arm_device_watchdog", "clamp_target",
                  "on_exit_action", "fault_action", "fault_auto_reset"})) {
        return false;
    }

    if (!need_bool(L, n, "safety", "auto_enable", c.safety.auto_enable)) return false;
    if (!need_bool(L, n, "safety", "require_calibrated", c.safety.require_calibrated)) return false;

    /* ⚠ `arm_device_watchdog` / `clamp_target` 是**每总线**的字段（SafetyCfg 里没有）：
       这里读全局值，只在 YAML 里写了该键时才覆盖到所有总线上（否则保留总线默认）。 */
    bool arm = false;
    bool clamp = false;
    const bool has_arm = static_cast<bool>(n["arm_device_watchdog"]);
    const bool has_clamp = static_cast<bool>(n["clamp_target"]);
    if (!need_bool(L, n, "safety", "arm_device_watchdog", arm)) return false;
    if (!need_bool(L, n, "safety", "clamp_target", clamp)) return false;
    for (unsigned b = 0u; b < c.bus_count; ++b) {
        if (has_arm) c.buses[b].arm_device_watchdog = arm;
        if (has_clamp) c.buses[b].clamp_target = clamp;
    }

    std::string ex;
    if (!need_enum(L, n, "safety", "on_exit_action",
                   {"disable", "hold", "zero_torque", "estop", "none"}, ex)) {
        return false;
    }
    if (ex == "hold") c.safety.on_exit = ExitAction::kHold;
    else if (ex == "zero_torque") c.safety.on_exit = ExitAction::kZeroTorque;
    else if (ex == "estop") c.safety.on_exit = ExitAction::kEstop;
    else if (ex == "none") c.safety.on_exit = ExitAction::kNone;
    else if (ex == "disable") c.safety.on_exit = ExitAction::kDisable;

    std::string fa;
    if (!need_enum(L, n, "safety", "fault_action", {"none", "disable_joint", "estop_bus"}, fa)) {
        return false;
    }
    if (fa == "disable_joint") c.safety.on_fault = FaultAction::kDisableJoint;
    else if (fa == "estop_bus") c.safety.on_fault = FaultAction::kEstopBus;

    const YAML::Node ar = n["fault_auto_reset"];
    if (ar) {
        if (!keys_ok(L, ar, "safety.fault_auto_reset", {"enabled", "max_attempts", "backoff_ms"})) {
            return false;
        }
        if (!need_bool(L, ar, "safety.fault_auto_reset", "enabled",
                       c.safety.fault_auto_reset.enabled)) {
            return false;
        }
        if (!need_u32(L, ar, "safety.fault_auto_reset", "max_attempts",
                      c.safety.fault_auto_reset.max_attempts)) {
            return false;
        }
        if (!need_u32(L, ar, "safety.fault_auto_reset", "backoff_ms",
                      c.safety.fault_auto_reset.backoff_ms)) {
            return false;
        }
    }
    return true;
}

bool read_params(Loader &L, const YAML::Node &jr, Config &c)
{
    const YAML::Node n = jr["params"];
    if (!n) return true;
    if (!keys_ok(L, n, "params", {"allow_write", "allow_flash_persist"})) return false;
    if (!need_bool(L, n, "params", "allow_write", c.safety.allow_param_write)) return false;
    if (!need_bool(L, n, "params", "allow_flash_persist", c.safety.allow_flash_persist)) {
        return false;
    }
    return true;
}

bool read_descriptor(Loader &L, const YAML::Node &jr, Config &c)
{
    const YAML::Node n = jr["descriptor"];
    if (!n) return true;
    if (!keys_ok(L, n, "descriptor",
                 {"cache_enabled", "cache_dir", "timeout_ms", "retries", "retry_backoff_ms",
                  "retain", "filter_paths"})) {
        return false;
    }

    bool cache = true;
    std::uint32_t timeout = 5000u;
    std::uint32_t retries = 3u;
    std::uint32_t backoff = 100u;
    char dir[kPathLen] = {};
    std::string retain;
    if (!need_bool(L, n, "descriptor", "cache_enabled", cache)) return false;
    if (!need_str(L, n, "descriptor", "cache_dir", dir, sizeof(dir))) return false;
    if (!need_u32(L, n, "descriptor", "timeout_ms", timeout)) return false;
    if (!need_u32(L, n, "descriptor", "retries", retries)) return false;
    if (!need_u32(L, n, "descriptor", "retry_backoff_ms", backoff)) return false;
    if (!need_enum(L, n, "descriptor", "retain", {"filtered", "all"}, retain)) return false;

    for (unsigned b = 0u; b < c.bus_count; ++b) {
        DescCfg &d = c.buses[b].desc;
        d.cache_enabled = cache;
        d.timeout_ms = timeout;
        d.retries = retries;
        d.retry_backoff_ms = backoff;
        if (retain == "filtered") d.retain = DescRetain::kFiltered;
        else if (retain == "all") d.retain = DescRetain::kAll;
        if (dir[0] != '\0') std::snprintf(d.cache_dir, sizeof(d.cache_dir), "%s", dir);
    }
    if (n["filter_paths"]) {
        /* `Config.desc.filter_paths` 是 `const char* const*`（借用调用方的字符串），
           从 YAML 读需要自己持有内存 —— 留给节点层实现，这里如实报出。 */
        L.node_key("descriptor.filter_paths");
    }
    return true;
}

bool read_bus_lock(Loader &L, const YAML::Node &jr, Config &c)
{
    const YAML::Node n = jr["bus_lock"];
    if (!n) return true;
    if (!keys_ok(L, n, "bus_lock", {"enabled", "lock_dir", "allow_shared"})) return false;
    if (!need_bool(L, n, "bus_lock", "enabled", c.lock.enabled)) return false;
    if (!need_bool(L, n, "bus_lock", "allow_shared", c.lock.allow_shared)) return false;
    if (!need_str(L, n, "bus_lock", "lock_dir", c.lock.lock_dir, sizeof(c.lock.lock_dir))) {
        return false;
    }
    return true;
}

bool read_all(Loader &L, const YAML::Node &root, Config &c)
{
    /* 允许两种文件形态：
       ① 我们的独立配置文件：`jr: {...}`
       ② ROS 参数文件：`jr: {ros__parameters: {...}}`（launch 里可以这么传） */
    YAML::Node jr = root["jr"];
    if (!jr) return L.fail("missing top-level key 'jr' (see DESIGN §6.4 for the schema)");
    if (jr["ros__parameters"]) jr = jr["ros__parameters"];

    if (!keys_ok(L, jr, "jr",
                 {"rt", "tick_groups", "buses", "joints", "limits", "feedback", "command", "safety",
                  "params", "descriptor", "bus_lock"})) {
        return false;
    }

    c = default_config();
    c.bus_count = 0u;
    c.group_count = 0u;

    std::vector<std::string> bus_names;
    if (!read_rt(L, jr, c)) return false;
    if (!read_buses(L, jr, c, bus_names)) return false;
    if (!read_joints(L, jr, c, bus_names)) return false;
    if (!read_limits(L, jr, c)) return false;
    if (!read_tick_groups(L, jr, c, bus_names)) return false;
    if (!read_feedback(L, jr, c)) return false;
    if (!read_command(L, jr, c)) return false;
    if (!read_safety(L, jr, c)) return false;
    if (!read_params(L, jr, c)) return false;
    if (!read_descriptor(L, jr, c)) return false;
    if (!read_bus_lock(L, jr, c)) return false;
    return true;
}

Status finish(Loader &L, Config *out, Result *res, YamlLoadReport *rep, const char *what) noexcept
{
    if (L.failed) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kFixBusPlanning, "%s: %s", what,
                     L.err.c_str());
        }
        return Status::kInvalidArgument;
    }

    ConfigNotes notes;
    const Result vr = validate_config(*out, &notes);
    if (vr.status != Status::kOk) {
        if (res != nullptr) *res = vr;
        return vr.status;
    }

    if (rep != nullptr) {
        rep->notes = notes;   /* 建议（如 node_id>7 无法广播寻址）带给调用方 */
        rep->node = L.node;   /* 节点层键的**取值**（已解析，节点直接用） */
        rep->node_keys = static_cast<unsigned>(L.node_keys.size());
        std::snprintf(rep->node_keys_text, sizeof(rep->node_keys_text), "%s",
                      L.node_keys.empty() ? "" : keys_text(L.node_keys).c_str());
    }
    if (res != nullptr) {
        if (!L.node_keys.empty()) {
            /* 这些键属于**节点层**：核心库不消费，但已解析进 report->node（不是"没人管"）。 */
            res->set(Status::kOk, Advice::kNone,
                     "loaded; %u node-level key(s) parsed for the node: %s",
                     static_cast<unsigned>(L.node_keys.size()), keys_text(L.node_keys).c_str());
        } else {
            res->set(Status::kOk, Advice::kNone, "loaded and validated");
        }
    }
    return Status::kOk;
}

}  // namespace

Status parse_config_yaml(const char *yaml_text, const char *what, Config *out, Result *res,
                         YamlLoadReport *rep) noexcept
{
    if (yaml_text == nullptr || out == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "null argument");
        return Status::kInvalidArgument;
    }
    const char *label = (what != nullptr) ? what : "<yaml>";
    Loader L;
    try {
        const YAML::Node root = YAML::Load(yaml_text);
        if (!root || !root.IsMap()) {
            if (res != nullptr) {
                res->set(Status::kInvalidArgument, Advice::kNone,
                         "%s: expected a YAML mapping at the top level", label);
            }
            return Status::kInvalidArgument;
        }
        if (!read_all(L, root, *out)) return finish(L, out, res, rep, label);
    } catch (const YAML::Exception &e) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kNone, "%s: YAML syntax error: %s", label,
                     e.msg.c_str());
        }
        return Status::kInvalidArgument;
    }
    return finish(L, out, res, rep, label);
}

Status load_config_yaml(const char *path, Config *out, Result *res, YamlLoadReport *rep) noexcept
{
    if (path == nullptr || out == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "null argument");
        return Status::kInvalidArgument;
    }
    Loader L;
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (!root || !root.IsMap()) {
            if (res != nullptr) {
                res->set(Status::kInvalidArgument, Advice::kNone,
                         "%s: expected a YAML mapping at the top level", path);
            }
            return Status::kInvalidArgument;
        }
        if (!read_all(L, root, *out)) return finish(L, out, res, rep, path);
    } catch (const YAML::BadFile &) {
        if (res != nullptr) {
            res->set(Status::kNotFound, Advice::kNone, "%s: cannot open the configuration file",
                     path);
        }
        return Status::kNotFound;
    } catch (const YAML::Exception &e) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kNone, "%s: YAML syntax error: %s", path,
                     e.msg.c_str());
        }
        return Status::kInvalidArgument;
    }
    return finish(L, out, res, rep, path);
}

}  // namespace jr
