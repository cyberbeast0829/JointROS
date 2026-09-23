/**
 * @file    test_config_yaml.cpp
 * @brief   YAML → Config 加载器测试（schema 见 DESIGN §6.4）
 *
 * 重点在**证伪**：客户配置文件写错时必须**报错并指出键路径**，不能静默忽略
 * （"写了没生效"是最难查的一类现场问题）；属于节点层的键要**如实报出**。
 */

#include <cstdio>
#include <cstring>
#include <string>

#include "jr_ros2/jr_config_yaml.hpp"
#include "jr_test.hpp"

using namespace jr;

namespace {

/** 覆盖 DESIGN §6.4 全部段落的完整示例（节点 / ros2_control / 工具共用同一份）。 */
const char *kFull = R"YAML(
jr:
  rt:
    policy: fifo
    priority: 80
    mlock: true
    cpu_affinity: [3, 4]
    warn_if_throttled: true
  tick_groups:
    - name: legs
      rate_hz: 1000
      buses: [can0, virt]
    - name: arms
      rate_hz: 500
      buses: [can1]
  buses:
    - name: can0
      type: socketcan
      interface: can0
      is_fd: true
      master_id: 1
      bitrate: {nominal: 1000000, data: 5000000}
      joints: [FL_hip, FL_knee]
    - name: can1
      type: pcan
      interface: PCAN_USBBUS1
      is_fd: false
      master_id: 2
      joints: [AR_elbow]
    - name: virt
      type: virtual
      spec: "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd"
      master_id: 3
      joints: [VIRT_ax]
  joints:
    - {name: FL_hip, bus: can0, node_id: 1}
    - {name: FL_knee, bus: can0, node_id: 2}
    - {name: AR_elbow, bus: can1, node_id: 1}
    - {name: VIRT_ax, bus: virt, node_id: 1}
  limits:
    FL_hip: {position: [-0.7, 0.7], velocity: 20.0, effort: 40.0, stiffness: 200.0, damping: 5.0}
  feedback:
    source: broadcast_plus_heartbeat
    heartbeat_ms: 5
    poll_period_ms: 10
    publish_hz: 500
    joint_state_hz: 100
  command:
    interpolation: none
    timeout_ms: 100
    timeout_action: hold
  safety:
    auto_enable: false
    require_calibrated: true
    arm_device_watchdog: false
    clamp_target: false
    on_exit_action: disable
    fault_action: none
    fault_auto_reset: {enabled: false, max_attempts: 1, backoff_ms: 1000}
  params:
    allow_write: false
    allow_flash_persist: false
  descriptor:
    cache_enabled: true
    cache_dir: ~/.cache/jr
    timeout_ms: 5000
    retries: 3
    retry_backoff_ms: 100
    retain: all
  bus_lock:
    enabled: true
    lock_dir: /tmp
    allow_shared: false
)YAML";

/** 最小可用配置（用于"只改一处"的证伪用例）。 */
const char *kMinimal = R"YAML(
jr:
  tick_groups:
    - {name: g0, rate_hz: 1000, buses: [can0]}
  buses:
    - {name: can0, type: virtual, spec: "0:id=1,gear=16.5,fd"}
  joints:
    - {name: j0, bus: can0, node_id: 1}
)YAML";

/** 只替换一处；替换不中 → 返回空串（用例会立刻红，避免"测试了个寂寞"）。 */
std::string variant(const std::string &base, const std::string &from, const std::string &to)
{
    const std::size_t p = base.find(from);
    if (p == std::string::npos) return std::string();
    std::string s = base;
    s.replace(p, from.size(), to);
    return s;
}

void test_full_example()
{
    JR_CASE("§6.4 完整示例：逐字段落地 + 节点层键如实报出");

    Config cfg;
    Result r;
    YamlLoadReport rep;
    const Status st = parse_config_yaml(kFull, "example", &cfg, &r, &rep);
    JR_CHECK_MSG(st == Status::kOk, r.message);
    if (st != Status::kOk) return;

    /* 总线 */
    JR_CHECK_EQ(cfg.bus_count, 3u);
    JR_CHECK_EQ(std::strcmp(cfg.buses[0].name, "can0"), 0);
    JR_CHECK_EQ(std::strcmp(cfg.buses[1].name, "can1"), 0);
    JR_CHECK_EQ(std::strcmp(cfg.buses[2].name, "virt"), 0);
    JR_CHECK(cfg.buses[0].hal == HalKind::kSocketCan);
    JR_CHECK(cfg.buses[1].hal == HalKind::kPcan);
    JR_CHECK(cfg.buses[2].hal == HalKind::kVirtual);
    JR_CHECK_EQ(std::strcmp(cfg.buses[0].channel, "can0"), 0);
    JR_CHECK(std::strstr(cfg.buses[2].channel, "id=1") != nullptr);   /* virtual: spec 进 channel */
    JR_CHECK_EQ(cfg.buses[1].is_fd, false);
    JR_CHECK_EQ(cfg.buses[0].master_id, 1u);
    JR_CHECK_EQ(cfg.buses[2].master_id, 3u);
    JR_CHECK_EQ(cfg.buses[0].nominal_bitrate, 1000000u);
    JR_CHECK_EQ(cfg.buses[0].data_bitrate, 5000000u);

    /* 关节挂载（按 joints: 表分配到对应总线） */
    JR_CHECK_EQ(cfg.buses[0].joint_count, 2u);
    JR_CHECK_EQ(cfg.buses[1].joint_count, 1u);
    JR_CHECK_EQ(cfg.buses[2].joint_count, 1u);   /* validate_config 要求每条总线都有关节 */
    JR_CHECK_EQ(std::strcmp(cfg.buses[0].joints[0].name, "FL_hip"), 0);
    JR_CHECK_EQ(cfg.buses[0].joints[1].node_id, 2u);

    /* 软限位 / hold 刚度 */
    JR_CHECK(cfg.buses[0].joints[0].has_position_limit);
    JR_CHECK_IN(cfg.buses[0].joints[0].position_min, -0.71, -0.69);
    JR_CHECK_EQ(cfg.buses[0].joints[0].stiffness, 200.0);

    /* tick 组：名字引用 → 下标；cpu_affinity 按组依次取 */
    JR_CHECK_EQ(cfg.group_count, 2u);
    JR_CHECK_EQ(std::strcmp(cfg.groups[0].name, "legs"), 0);
    JR_CHECK_EQ(cfg.groups[0].rate_hz, 1000u);
    JR_CHECK_EQ(cfg.groups[0].bus_count, 2u);
    JR_CHECK_EQ(cfg.groups[0].bus_index[0], 0u);      /* can0 */
    JR_CHECK_EQ(cfg.groups[0].bus_index[1], 2u);      /* virt */
    JR_CHECK_EQ(cfg.groups[1].bus_index[0], 1u);      /* can1 */
    JR_CHECK_EQ(cfg.groups[0].cpu, 3);
    JR_CHECK_EQ(cfg.groups[1].cpu, 4);

    /* 全局 feedback / command / safety / params / descriptor / lock */
    for (unsigned b = 0u; b < cfg.bus_count; ++b) {
        JR_CHECK(cfg.buses[b].feedback == FeedbackPolicy::kBroadcastHeartbeat);
        JR_CHECK_EQ(cfg.buses[b].heartbeat_ms, 5u);
        JR_CHECK_EQ(cfg.buses[b].desc.retries, 3u);
        JR_CHECK_EQ(std::strcmp(cfg.buses[b].desc.cache_dir, "~/.cache/jr"), 0);
        JR_CHECK(cfg.buses[b].desc.retain == DescRetain::kAll);
    }
    JR_CHECK_EQ(cfg.command.timeout_ms, 100u);
    JR_CHECK(cfg.command.on_timeout == TimeoutAction::kHold);
    JR_CHECK_EQ(cfg.safety.require_calibrated, true);
    JR_CHECK_EQ(cfg.safety.allow_param_write, false);
    JR_CHECK(cfg.safety.on_exit == ExitAction::kDisable);
    JR_CHECK_EQ(cfg.safety.fault_auto_reset.backoff_ms, 1000u);
    JR_CHECK_EQ(cfg.lock.enabled, true);
    JR_CHECK_EQ(std::strcmp(cfg.lock.lock_dir, "/tmp"), 0);
    JR_CHECK_EQ(cfg.lock.allow_shared, false);

    /* 节点层键：键名列出来 + **取值**已解析（不是只报个名字）
       ⚠ `command.interpolation` **不是**节点层键（v0.13 起由核心库在 tick 上实现）——
       它必须出现在 `cfg.command`，而不应该出现在 node_keys 里。 */
    JR_CHECK_EQ(rep.node_keys, 2u);
    JR_CHECK_CONTAINS(rep.node_keys_text, "feedback.publish_hz");
    JR_CHECK_CONTAINS(rep.node_keys_text, "feedback.joint_state_hz");
    JR_CHECK_EQ(rep.node.publish_hz, 500u);
    JR_CHECK_EQ(rep.node.joint_state_hz, 100u);
    JR_CHECK_EQ(cfg.command.interpolation, jr::Interpolation::kNone);
    JR_CHECK_CONTAINS(r.message, "node-level");

    /* 全部 node_id ≤ 7 → 不该有"无法广播寻址"的提示 */
    JR_CHECK_EQ(rep.notes.has_joint_above_broadcast_id, false);
}

void test_errors_point_at_the_key()
{
    JR_CASE("写错必须报错并指出键路径（静默忽略是最难查的现场问题）");

    const std::string base = kMinimal;
    Config cfg;
    Result r;

    /* ① 拼错键名 */
    {
        const std::string y = variant(base, "  joints:\n", "  rt: {priorityy: 80}\n  joints:\n");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "typo", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "rt: unknown key 'priorityy'");
    }
    /* ② 枚举值写错 → 必须列出合法取值 */
    {
        const std::string y = variant(base, "  joints:\n", "  rt: {policy: realtime}\n  joints:\n");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "bad-enum", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "unknown value 'realtime'");
        JR_CHECK_CONTAINS(r.message, "fifo");
    }
    /* ③ limits 引用了不存在的关节 */
    {
        const std::string y =
            variant(base, "  joints:\n    - {name: j0, bus: can0, node_id: 1}\n",
                    "  joints:\n    - {name: j0, bus: can0, node_id: 1}\n"
                    "  limits:\n    nosuch: {position: [0.0, 1.0]}\n");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "unknown-joint", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "unknown joint 'nosuch'");
    }
    /* ④ tick_groups 引用了不存在的总线 → 报错里要带**已知总线名** */
    {
        const std::string y = variant(base, "buses: [can0]", "buses: [canX]");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "unknown-bus", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "unknown bus 'canX'");
        JR_CHECK_CONTAINS(r.message, "known: can0");
    }
    /* ⑤ 缺必填键 */
    {
        const std::string y =
            variant(base, "- {name: j0, bus: can0, node_id: 1}", "- {name: j0, bus: can0}");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "missing-node-id", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "node_id: required");
    }
    /* ⑥ node_id 越界（0 会让设备完全不回复 → 必须早拦） */
    {
        const std::string y = variant(base, "node_id: 1}", "node_id: 0}");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "node-id-zero", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "must be 1..254");
    }
    /* ⑦ 语法错误也要报（而不是抛异常）
          ⚠ 注意长度：yaml-cpp 对明显畸形输入会在**parse** 阶段抛异常，被我们接住。 */
    {
        const Status st = parse_config_yaml("jr: [unclosed\n", "syntax", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "syntax");
    }
    /* ⑧ 缺 jr: 根键 */
    {
        const Status st = parse_config_yaml("other: {}\n", "no-root", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "missing top-level key 'jr'");
    }
}

void test_node_level_keys()
{
    JR_CASE("节点层键要解析取值，值非法要报错（写错不能静默不生效）");

    Config cfg;
    Result r;
    YamlLoadReport rep;
    const std::string base = kMinimal;

    /* ① 正常解析：覆盖默认值 */
    {
        /* `command.interpolation` 现在落到 **cfg.command**（核心库消费），不再是节点层键。 */
        std::string y = variant(base, "  joints:\n",
                                "  feedback: {publish_hz: 250, joint_state_hz: 50}\n"
                                "  command: {interpolation: linear}\n  joints:\n");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "node-keys", &cfg, &r, &rep);
        JR_CHECK_MSG(st == Status::kOk, r.message);
        JR_CHECK_EQ(rep.node.publish_hz, 250u);
        JR_CHECK_EQ(rep.node.joint_state_hz, 50u);
        JR_CHECK_EQ(cfg.command.interpolation, jr::Interpolation::kLinear);
        JR_CHECK_EQ(rep.node_keys, 2u);   /* 只剩两个节点层键 */
    }
    /* ② 默认值（没写这三个键时） */
    {
        YamlLoadReport rep2;
        const Status st = parse_config_yaml(base.c_str(), "node-defaults", &cfg, &r, &rep2);
        JR_CHECK_MSG(st == Status::kOk, r.message);
        JR_CHECK_EQ(rep2.node_keys, 0u);
        JR_CHECK_EQ(rep2.node.publish_hz, 500u);
        JR_CHECK_EQ(rep2.node.joint_state_hz, 100u);
        JR_CHECK_EQ(cfg.command.interpolation, jr::Interpolation::kNone);
    }
    /* ③ 插值器拼错 → 报错并给出合法取值 */
    {
        const std::string y = variant(base, "  joints:\n",
                                      "  command: {interpolation: quad}\n  joints:\n");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "bad-interp", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "command.interpolation");
        JR_CHECK_CONTAINS(r.message, "linear");
    }
    /* ④ publish_hz = 0 → 报错（0 = 不发，那写这个键干什么，大概率是想表达别的） */
    {
        const std::string y =
            variant(base, "  joints:\n", "  feedback: {publish_hz: 0}\n  joints:\n");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "pub-zero", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "feedback.publish_hz");
        JR_CHECK_CONTAINS(r.message, "1..5000");
    }
    /* ⑤ joint_state_hz 离谱大 → 报错 */
    {
        const std::string y =
            variant(base, "  joints:\n", "  feedback: {joint_state_hz: 99999}\n  joints:\n");
        JR_CHECK(!y.empty());
        const Status st = parse_config_yaml(y.c_str(), "js-hz", &cfg, &r);
        JR_CHECK(st == Status::kInvalidArgument);
        JR_CHECK_CONTAINS(r.message, "feedback.joint_state_hz");
    }
}

void test_validate_integration()
{
    JR_CASE("加载后必须过 validate_config（且把非致命提示带出来）");

    Config cfg;
    Result r;
    YamlLoadReport rep;

    /* node_id = 8 → 广播位图寻址不到：不是致命错误，但必须出现在 notes 里。 */
    const std::string base = kMinimal;
    const std::string y = variant(base, "node_id: 1}", "node_id: 8}");
    JR_CHECK(!y.empty());
    const Status st = parse_config_yaml(y.c_str(), "node8", &cfg, &r, &rep);
    JR_CHECK_MSG(st == Status::kOk, r.message);
    JR_CHECK_EQ(rep.notes.has_joint_above_broadcast_id, true);
    JR_CHECK_CONTAINS(rep.notes.text, "bitmap");

    /* 同一总线上两个关节用同一个 node_id → validate_config 必须拒绝加载。 */
    const std::string dup =
        variant(base, "    - {name: j0, bus: can0, node_id: 1}\n",
                "    - {name: j0, bus: can0, node_id: 1}\n"
                "    - {name: j1, bus: can0, node_id: 1}\n");
    JR_CHECK(!dup.empty());
    const Status st2 = parse_config_yaml(dup.c_str(), "dup-node", &cfg, &r);
    JR_CHECK(st2 != Status::kOk);
}

void test_file_forms()
{
    JR_CASE("文件形态：独立配置文件 / ROS 参数文件（jr: ros__parameters:）/ 文件不存在");

    const char *path = "jr_test_config_tmp.yaml";

    /* ① 独立配置文件 */
    {
        std::FILE *f = std::fopen(path, "wb");
        JR_CHECK(f != nullptr);
        if (f != nullptr) {
            std::fputs(kMinimal, f);
            std::fclose(f);
        }
        Config cfg;
        Result r;
        const Status st = load_config_yaml(path, &cfg, &r);
        JR_CHECK_MSG(st == Status::kOk, r.message);
        JR_CHECK_EQ(cfg.bus_count, 1u);
        std::remove(path);
    }
    /* ② ROS 参数文件形态（launch 里可以这么传） */
    {
        std::FILE *f = std::fopen(path, "wb");
        JR_CHECK(f != nullptr);
        if (f != nullptr) {
            std::fputs("jr:\n  ros__parameters:\n", f);
            /* 把最小配置的 jr: 之后的内容缩进两格塞进 ros__parameters 下（这里直接手写） */
            std::fputs("    tick_groups:\n      - {name: g0, rate_hz: 1000, buses: [can0]}\n"
                       "    buses:\n      - {name: can0, type: virtual, spec: \"0:id=1,gear=16.5,fd\"}\n"
                       "    joints:\n      - {name: j0, bus: can0, node_id: 1}\n",
                       f);
            std::fclose(f);
        }
        Config cfg;
        Result r;
        const Status st = load_config_yaml(path, &cfg, &r);
        JR_CHECK_MSG(st == Status::kOk, r.message);
        JR_CHECK_EQ(cfg.bus_count, 1u);
        std::remove(path);
    }
    /* ③ 文件不存在 → kNotFound（而不是 kInvalidArgument） */
    {
        Config cfg;
        Result r;
        const Status st = load_config_yaml("no_such_jr_config_12345.yaml", &cfg, &r);
        JR_CHECK(st == Status::kNotFound);
        JR_CHECK_CONTAINS(r.message, "cannot open");
    }
}

}  // namespace

int main()
{
    test_full_example();
    test_errors_point_at_the_key();
    test_node_level_keys();
    test_validate_integration();
    test_file_forms();
    return jrtest::report();
}
