/**
 * @file    jr_bus_node.cpp
 * @brief   `jr_bus` 实现（WP2 第一段：生命周期 + 命令 + 状态发布）
 *
 * 本文件只做"搬运"：配置 → 核心库，快照 → 消息，消息 → 命令信箱。
 * **没有任何实时逻辑**（那些在 `jr_core` 里，且被独立的无 ROS 测试覆盖）。
 */

#include "jr_ros2/ros/jr_bus_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstring>

#include "jr_ros2/jr_config_yaml.hpp"

namespace jr {
namespace ros {

namespace {

constexpr std::uint64_t kNsPerSec = 1000000000ull;
/** 故障事件环的抽稀周期：事件是边沿触发的，20 Hz 足够（且不占带宽）。 */
constexpr std::uint32_t kFaultDrainHz = 20u;

std::chrono::nanoseconds period_from_hz(std::uint32_t hz) noexcept
{
    if (hz == 0u) return std::chrono::nanoseconds(0);
    return std::chrono::nanoseconds(static_cast<std::int64_t>(kNsPerSec / hz));
}

/** 退出策略的人话名（核心库只给 `BusMode` 的人话名，退出策略这个枚举得自己转）。 */
const char *exit_action_name(ExitAction a) noexcept
{
    switch (a) {
    case ExitAction::kDisable:
        return "disable";
    case ExitAction::kHold:
        return "hold";
    case ExitAction::kZeroTorque:
        return "zero_torque";
    case ExitAction::kEstop:
        return "estop";
    case ExitAction::kNone:
        return "none";
    }
    return "?";
}

/** 把核心库的 Status 映射成消息里的稳定编号（一一对应，见 jr_interfaces/msg/JointResult.msg）。
 *
 *  ⚠ 服务层（本文件的下一段）会用到；这里保留在匿名命名空间里，
 *  并由 test_jr_bus_node 的静态断言守住映射（改一处不改另一处 → 编译期就红）。
 *  当前尚未使用 → 显式标记可能未使用，避免 -Wunused-function 噪声。 */
[[maybe_unused]] std::uint8_t status_code(Status s) noexcept
{
    return static_cast<std::uint8_t>(static_cast<std::int32_t>(s));
}

}  // namespace

JrBusNode::JrBusNode(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("jr_bus", options)
{    /* 参数只有两个（配置本身在共享 YAML 里，§6.4）：
       - `config_file`：**必填**，指向客户维护的那一份配置；
       - `bus`：本节点负责哪条总线（一个节点 = 一条总线，ADR-1 的话题命名）。
       ⚠ 节点名建议用 `__node:=<bus>` 覆盖，这样 `~/cmd_mit` 自然落在
         `/<robot>/jr/<bus>/cmd_mit` 上（见 launch 示例）。 */
    config_file_ = declare_parameter<std::string>("config_file", "");
    bus_name_ = declare_parameter<std::string>("bus", "");
    declare_parameter<std::string>("frame_id", bus_name_);
}

/* 析构：`services_` 是 `shared_ptr` → 不完整类型也能安全析构（见头文件注释）。 */
JrBusNode::~JrBusNode() = default;

/* ==========================================================================
 * 配置
 * ======================================================================== */

bool JrBusNode::load_and_validate(std::string *err)
{
    if (config_file_.empty()) {
        *err = "parameter 'config_file' is required (path to the shared jr YAML, DESIGN §6.4)";
        return false;
    }
    if (bus_name_.empty()) {
        *err = "parameter 'bus' is required (which bus this node owns; one node = one bus)";
        return false;
    }

    YamlLoadReport rep;
    Result r;
    const Status st = load_config_yaml(config_file_.c_str(), &cfg_, &r, &rep);
    if (st != Status::kOk) {
        *err = std::string("loading '") + config_file_ + "' failed: " + r.message;
        return false;
    }
    notes_ = rep.notes;
    node_cfg_ = rep.node;

    const BusCfg *bus = find_bus(cfg_, bus_name_.c_str());
    if (bus == nullptr) {
        std::string known;
        for (unsigned i = 0u; i < cfg_.bus_count; ++i) {
            if (!known.empty()) known += ", ";
            known += cfg_.buses[i].name;
        }
        *err = "bus '" + bus_name_ + "' is not in " + config_file_ +
               " (known: " + (known.empty() ? std::string("<none>") : known) + ")";
        return false;
    }
    bus_index_ = static_cast<unsigned>(bus - &cfg_.buses[0]);

    /* 这条总线属于哪个 tick 组（找不到 = 配置不自洽；不应该发生，因为 validate 过了）。 */
    bool found_group = false;
    for (unsigned g = 0u; g < cfg_.group_count; ++g) {
        for (unsigned b = 0u; b < cfg_.groups[g].bus_count; ++b) {
            if (cfg_.groups[g].bus_index[b] == bus_index_) {
                group_index_ = g;
                found_group = true;
                break;
            }
        }
        if (found_group) break;
    }
    if (!found_group) {
        *err = "internal: bus '" + bus_name_ + "' is not assigned to any tick group";
        return false;
    }

    local_joint_count_ = bus->joint_count;
    joint_names_.clear();
    for (unsigned j = 0u; j < local_joint_count_; ++j) joint_names_.emplace_back(bus->joints[j].name);
    cfg_loaded_ = true;
    return true;
}

/* ==========================================================================
 * lifecycle（§8.1 启动序列 / §8.2 退出序列 / §5.4 状态机）
 * ======================================================================== */

JrBusNode::CallbackReturn JrBusNode::on_configure(const rclcpp_lifecycle::State &)
{
    if (tg_ != nullptr) {
        RCLCPP_WARN(get_logger(), "already configured");
        return CallbackReturn::SUCCESS;
    }

    std::string err;
    if (!load_and_validate(&err)) {
        RCLCPP_FATAL(get_logger(), "%s", err.c_str());
        return CallbackReturn::FAILURE;
    }

    /* 时间基准偏置（§6.6）：控制/时间戳一律用单调时钟；ROS 时间只用于消息里的 stamp。
       `use_sim_time=true` 时这个偏置是没有意义的 → **明确告警**，不要假装时间是对的。 */
    sim_time_seen_ = has_parameter("use_sim_time") && get_parameter("use_sim_time").as_bool();
    steady_to_ros_ns_ = now().nanoseconds() - static_cast<std::int64_t>(monotonic_ns());
    if (sim_time_seen_) {
        RCLCPP_WARN(get_logger(),
                    "use_sim_time is true: message stamps will be offset by the node's startup "
                    "delta. Real-time control never uses ROS time (DESIGN §6.6) — this node keeps "
                    "using its monotonic clock, so the *control* loop is unaffected.");
    }

    if (notes_.text[0] != '\0') {
        RCLCPP_WARN(get_logger(), "config notes:\n%s", notes_.text);
    }

    tg_ = std::make_unique<rt::TickGroup>();
    Result r;
    if (tg_->init(cfg_, group_index_, &r) != Status::kOk) {
        RCLCPP_FATAL(get_logger(), "tick group init failed: %s", r.message);
        tg_.reset();
        return CallbackReturn::FAILURE;
    }

    /* open = 取 flock（单 master，ADR-9）+ 开 HAL + 建 context；
       ⚠ 这里失败最常见的原因是"同一条总线上已经有一个进程在跑"，
         所以必须把核心库的原话（含持有者 PID）打出来。 */
    rt::OpenOptions opt;
    /* ⚠⚠ 配置里的 `bus_lock` 必须**真的传下去**（踩过）：`OpenOptions` 有自己的一套默认值
       （enable_lock=true / lock_dir=nullptr），不传就等于「YAML 写了 `lock_dir` 是摆设、
       写了 `enabled:false` 是谎话」—— 而且症状很隐蔽：锁在**默认目录**里照样起作用，
       现场以为"我关了锁/我把锁放到别处了"。 */
    opt.enable_lock = cfg_.lock.enabled;
    opt.allow_shared_lock = cfg_.lock.allow_shared;
    opt.lock_dir = (cfg_.lock.lock_dir[0] != '\0') ? cfg_.lock.lock_dir : nullptr;
    if (tg_->open_buses(opt, &r) != Status::kOk) {
        RCLCPP_FATAL(get_logger(), "opening bus '%s' failed: %s | advice: %s", bus_name_.c_str(),
                     r.message, jr::to_string(r.advice));
        tg_.reset();
        return CallbackReturn::FAILURE;
    }

    /* configure = 描述符（缓存优先）+ 读回量程 + 预算检查。 */
    if (tg_->configure_buses(&r) != Status::kOk) {
        RCLCPP_FATAL(get_logger(), "configuring bus '%s' failed: %s | advice: %s",
                     bus_name_.c_str(), r.message, jr::to_string(r.advice));
        tg_->close_buses(ExitAction::kNone, nullptr);
        tg_.reset();
        return CallbackReturn::FAILURE;
    }

    /* 逐行结论（含"未标定""心跳现状"这类必须让人看到的事实）。 */
    RCLCPP_INFO(get_logger(), "bus '%s' configured:\n%s", bus_name_.c_str(), tg_->report().text);

    /* 发布率高于 tick 率没有意义（多出来的周期只会发重复数据）—— 说出来，不拦。 */
    const unsigned rate = tg_->rate_hz();
    if (node_cfg_.publish_hz > rate) {
        RCLCPP_WARN(get_logger(),
                    "feedback.publish_hz (%u) is above the tick rate (%u Hz): duplicate samples will "
                    "be published. Lower it or raise the tick rate.",
                    node_cfg_.publish_hz, rate);
    }
    if (cfg_.command.interpolation == Interpolation::kLinear) {
        /* v0.13 起插值由**核心库**在 tick 上做（见 `BusRuntime::apply_command()`）：
           这里只说明"发生了什么"，不再是"说了不做"的告警。
           说清插什么/不插什么，客户才不会以为增益也被平滑了。 */
        RCLCPP_INFO(get_logger(),
                    "command.interpolation=linear: the core ramps the motion quantity (position/"
                    "velocity/torque) between commands on the tick; gains and limits apply "
                    "immediately, and the first command after enable/estop applies immediately.");
    }

    RCLCPP_INFO(get_logger(),
                "configured (joints NOT enabled). Enable with the SetEnabled service%s.",
                cfg_.safety.auto_enable ? " (safety.auto_enable=true → will enable on activate)"
                                        : ", or set safety.auto_enable=true");
    return CallbackReturn::SUCCESS;
}

JrBusNode::CallbackReturn JrBusNode::on_activate(const rclcpp_lifecycle::State &)
{
    if (tg_ == nullptr) {
        RCLCPP_ERROR(get_logger(), "on_activate before a successful configure");
        return CallbackReturn::FAILURE;
    }

    Result r;
    if (tg_->start(nullptr, &r) != Status::kOk) {
        RCLCPP_FATAL(get_logger(), "starting the tick thread failed: %s", r.message);
        return CallbackReturn::FAILURE;
    }
    create_entities();
    create_services();
    create_diagnostics();

    if (!cfg_.safety.auto_enable) {
        RCLCPP_INFO(get_logger(),
                    "active: tick running, joints NOT enabled (cold start never drives - G6)");
        return CallbackReturn::SUCCESS;
    }

    /* auto_enable：先安全暂停（交出 context 所有权）→ 使能 → 恢复 tick。 */
    if (tg_->pause(&r) != Status::kOk) {
        RCLCPP_ERROR(get_logger(), "pause before enable failed: %s", r.message);
        return CallbackReturn::FAILURE;
    }
    const Status st = tg_->activate(cfg_.safety.require_calibrated, &r);
    tg_->resume();
    if (st != Status::kOk) {
        RCLCPP_ERROR(get_logger(), "enabling joints failed: %s | advice: %s", r.message,
                     jr::to_string(r.advice));
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "joints enabled (control frames start from the next cycle)");
    return CallbackReturn::SUCCESS;
}

JrBusNode::CallbackReturn JrBusNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    /* 先停止接受新命令，再停 tick，最后按退出策略收尾（§8.2）。幂等。 */
    destroy_diagnostics();
    destroy_services();
    destroy_entities();
    if (tg_ == nullptr) return CallbackReturn::SUCCESS;

    Result r;
    if (tg_->pause(&r) != Status::kOk) {
        RCLCPP_ERROR(get_logger(), "pause before disable failed: %s", r.message);
        return CallbackReturn::FAILURE;
    }
    const Status st = tg_->deactivate(&r);
    tg_->resume();   /* 恢复 tick 线程（此时总线是 READY，不会重新使能） */
    tg_->stop();

    if (st != Status::kOk) {
        RCLCPP_ERROR(get_logger(),
                     "safe disable reported a problem: %s (joints may still be enabled - check the "
                     "device state)",
                     r.message);
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "deactivated: safe disable done, tick stopped");
    return CallbackReturn::SUCCESS;
}

JrBusNode::CallbackReturn JrBusNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    destroy_diagnostics();
    destroy_services();
    destroy_entities();
    if (tg_ == nullptr) return CallbackReturn::SUCCESS;
    Result r;
    tg_->stop();
    tg_->close_buses(cfg_.safety.on_exit, &r);
    if (cfg_.safety.on_exit != ExitAction::kNone && !r.ok()) {
        RCLCPP_WARN(get_logger(), "exit action reported: %s", r.message);
    }
    tg_.reset();
    RCLCPP_INFO(get_logger(), "cleaned up (bus closed with exit action '%s', lock released)",
                exit_action_name(cfg_.safety.on_exit));
    return CallbackReturn::SUCCESS;
}

JrBusNode::CallbackReturn JrBusNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    /* SIGINT/SIGTERM 也走这条：必须与 cleanup 一样安全、且幂等（§8.2）。 */
    destroy_diagnostics();
    destroy_services();
    destroy_entities();
    if (tg_ == nullptr) return CallbackReturn::SUCCESS;
    Result r;
    tg_->stop();
    tg_->close_buses(cfg_.safety.on_exit, &r);
    tg_.reset();
    RCLCPP_INFO(get_logger(), "shutdown: bus closed with exit action '%s'",
                exit_action_name(cfg_.safety.on_exit));
    return CallbackReturn::SUCCESS;
}

JrBusNode::CallbackReturn JrBusNode::on_error(const rclcpp_lifecycle::State &)
{
    /* 出错时唯一正确的动作是**把关节放到安全状态**（不是继续尝试控制）。 */
    destroy_diagnostics();
    destroy_services();
    destroy_entities();
    if (tg_ == nullptr) return CallbackReturn::SUCCESS;
    Result r;
    if (tg_->pause(&r) != Status::kOk) {
        RCLCPP_ERROR(get_logger(), "on_error: cannot pause the tick group: %s", r.message);
        return CallbackReturn::FAILURE;
    }
    const Status st = tg_->deactivate(&r);
    tg_->resume();
    if (st != Status::kOk) {
        RCLCPP_ERROR(get_logger(), "on_error: safe disable failed: %s", r.message);
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "on_error: joints safely disabled");
    return CallbackReturn::SUCCESS;
}

/* ==========================================================================
 * ROS 实体
 * ======================================================================== */

void JrBusNode::create_entities()
{
    if (pub_feedback_ != nullptr) return;   /* 幂等 */

    ros_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = ros_cbg_;

    /* QoS（§6.3）：状态类 best_effort + depth 1（旧状态没价值，积压只会增加延迟）；
       命令类 reliable + depth 1（覆盖式语义，与信箱一致）。 */
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    pub_feedback_ = create_publisher<jr_interfaces::msg::JointFeedbackArray>("~/joint_feedback", state_qos);
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", state_qos);
    pub_bus_status_ = create_publisher<jr_interfaces::msg::BusStatus>("~/bus_status", status_qos);
    pub_rt_stats_ = create_publisher<jr_interfaces::msg::RtStats>("~/rt_stats", status_qos);
    pub_faults_ = create_publisher<jr_interfaces::msg::JointFault>("~/faults",
                                                                  rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    sub_cmd_mit_ = create_subscription<jr_interfaces::msg::MitCommandArray>(
        "~/cmd_mit", cmd_qos,
        [this](jr_interfaces::msg::MitCommandArray::ConstSharedPtr m) { on_cmd_mit(m); }, sub_opt);
    sub_cmd_mode_ = create_subscription<jr_interfaces::msg::JointCommandArray>(
        "~/cmd", cmd_qos,
        [this](jr_interfaces::msg::JointCommandArray::ConstSharedPtr m) { on_cmd_mode(m); }, sub_opt);
    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
        "~/estop", cmd_qos, [this](std_msgs::msg::Bool::ConstSharedPtr m) { on_estop_local(m); }, sub_opt);
    /* `/jr/estop_all`：所有总线节点都订阅（各总线依次广播，**不保证跨总线同时**，§8.4）。 */
    sub_estop_all_ = create_subscription<std_msgs::msg::Bool>(
        "/jr/estop_all", cmd_qos, [this](std_msgs::msg::Bool::ConstSharedPtr m) { on_estop_all(m); },
        sub_opt);

    /* 计时器（T4）：从**快照**读数据，绝不直接碰 SDK。 */
    const std::uint32_t pub_hz = node_cfg_.publish_hz;
    const std::uint32_t js_hz = node_cfg_.joint_state_hz;
    if (pub_hz != 0u) {
        tmr_feedback_ = create_wall_timer(period_from_hz(pub_hz), [this]() { publish_feedback(); }, ros_cbg_);
    }
    if (js_hz != 0u) {
        tmr_joint_states_ =
            create_wall_timer(period_from_hz(js_hz), [this]() { publish_joint_states(); }, ros_cbg_);
    }
    tmr_bus_status_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish_bus_status(); }, ros_cbg_);
    tmr_rt_stats_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish_rt_stats(); }, ros_cbg_);
    tmr_faults_ = create_wall_timer(period_from_hz(kFaultDrainHz), [this]() { drain_and_publish_faults(); }, ros_cbg_);

    RCLCPP_INFO(get_logger(), "publishers/subscribers up: joint_feedback=%u Hz, joint_states=%u Hz",
                pub_hz, js_hz);
}

void JrBusNode::destroy_entities()
{
    tmr_feedback_.reset();
    tmr_joint_states_.reset();
    tmr_bus_status_.reset();
    tmr_rt_stats_.reset();
    tmr_faults_.reset();
    sub_cmd_mit_.reset();
    sub_cmd_mode_.reset();
    sub_estop_.reset();
    sub_estop_all_.reset();
    pub_feedback_.reset();
    pub_joint_states_.reset();
    pub_bus_status_.reset();
    pub_rt_stats_.reset();
    pub_faults_.reset();
}

/* ==========================================================================
 * 命令
 * ======================================================================== */

std::uint64_t JrBusNode::monotonic_ns() noexcept
{
    /* Linux 上 steady_clock 就是 CLOCK_MONOTONIC，与核心库（clock_nanosleep/CLOCK_MONOTONIC）
       同源 —— 这是 §6.6 "度量用同一时钟" 能成立的前提，不要换成 system_clock。 */
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

builtin_interfaces::msg::Time JrBusNode::to_ros_time(std::uint64_t mono_ns) const noexcept
{
    const std::int64_t ros_ns = static_cast<std::int64_t>(mono_ns) + steady_to_ros_ns_;
    return rclcpp::Time(ros_ns, RCL_ROS_TIME);
}

int JrBusNode::global_index_of(const std::string &name) const noexcept
{
    unsigned bi = 0u;
    unsigned li = 0u;
    const int local = find_joint(cfg_, name.c_str(), &bi, &li);
    if (local < 0 || bi != bus_index_) return -1;
    return static_cast<int>(bus_joint_base(cfg_, bi) + li);
}

void JrBusNode::warn_unknown_joint_once(const std::string &name)
{
    if (std::find(warned_joints_.begin(), warned_joints_.end(), name) != warned_joints_.end()) return;
    warned_joints_.push_back(name);
    RCLCPP_WARN(get_logger(),
                "command for joint '%s' ignored: it is not on bus '%s' (this node owns only that "
                "bus). Each bus has its own node - publish to the right namespace.",
                name.c_str(), bus_name_.c_str());
}

void JrBusNode::submit(CommandSet *cmd)
{
    if (tg_ == nullptr) return;
    cmd->seq = ++cmd_seq_;
    cmd->stamp_ns = monotonic_ns();
    tg_->submit_command(*cmd);
}

bool JrBusNode::fill_mit_target(const jr_interfaces::msg::MitCommand &in, CommandSet *out) const noexcept
{
    const int g = global_index_of(in.name);
    if (g < 0) return false;
    JointTarget &t = out->joint[static_cast<unsigned>(g)];
    t.position = in.position;
    t.velocity = in.velocity;
    t.torque = in.torque;
    /* gain_mode 必须显式（ADR-5）：两种语义单位不同，混用会得到"能动但刚度量级不对"。 */
    if (in.gain_mode == jr_interfaces::msg::MitCommand::SI) {
        t.si_gain = true;
        t.stiffness = in.stiffness;
        t.damping = in.damping;
    } else {
        t.si_gain = false;
        t.kp = in.kp;
        t.kd = in.kd;
    }
    out->set_mask(static_cast<unsigned>(g));
    return true;
}

void JrBusNode::on_cmd_mit(const jr_interfaces::msg::MitCommandArray::ConstSharedPtr msg)
{
    if (tg_ == nullptr) return;

    /* ⚠ MIT 目标只能发给 **MIT 模式**的关节：给 CSP/CSV/CST/CURRENT 的关节发 MIT 帧
       会把它的输入模式**静默换回 MIT**（与广播路径同一条风险）。这里先按快照里记录的模式
       拦一道并说明原因，核心库还会再拦一道（两道防线）。 */
    const StateSnapshot *snap = tg_->acquire_snapshot();

    CommandSet cmd;
    unsigned used = 0u;
    unsigned unknown = 0u;
    unsigned want_enable = 0u;
    unsigned wrong_mode = 0u;
    for (const auto &c : msg->commands) {
        if (c.enable) ++want_enable;
        const int gl = global_index_of(c.name);
        if (gl >= 0 && snap != nullptr &&
            snap->joints[static_cast<unsigned>(gl)].cmd_mode !=
                static_cast<std::uint8_t>(CmdMode::kMit)) {
            ++wrong_mode;
            ++cmd_rejected_self_;   /* 节点侧就拒了：核心侧的计数器看不到这一笔 */
            const std::string key = c.name + "/mit";
            if (std::find(warned_modes_.begin(), warned_modes_.end(), key) == warned_modes_.end()) {
                warned_modes_.push_back(key);
                RCLCPP_WARN(get_logger(),
                            "MIT command for joint '%s' REFUSED (nothing was sent): it is enabled "
                            "in '%s' mode. Use the '~/cmd' topic with a matching target for this "
                            "joint, or set joints[].mode: mit in the config.",
                            c.name.c_str(),
                            jr::to_string(static_cast<CmdMode>(
                                snap->joints[static_cast<unsigned>(gl)].cmd_mode)));
            }
            continue;
        }
        if (fill_mit_target(c, &cmd)) {
            ++used;
        } else {
            ++unknown;
            warn_unknown_joint_once(c.name);
        }
    }
    if (unknown != 0u) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "%u/%zu command(s) ignored: joint not on bus '%s'", unknown,
                             msg->commands.size(), bus_name_.c_str());
    }
    if (want_enable != 0u) {
        /* ⚠ 本迭代**不**在命令路径里做使能（那需要暂停 tick 线程、属于配置类操作，ADR-7）。
           明说而不是静默忽略 —— 静默忽略会让人以为"发了 enable 命令所以使能了"。 */
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "%u command(s) had enable=true but enabling from the command path is "
                             "not implemented in this build: call the SetEnabled service (or set "
                             "safety.auto_enable=true).",
                             want_enable);
    }
    if (used != 0u) submit(&cmd);

    /* 拒绝的原因必须**看得见**（客户只会看到"关节不动"，日志里得有答案）。 */
    cmd_rejected_self_ += unknown + wrong_mode;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "'~/cmd_mit': %u accepted, %u refused (not on this bus %u, wrong mode "
                         "for a non-MIT joint %u) — see the per-joint warnings above",
                         used, unknown + wrong_mode, unknown, wrong_mode);
}

void JrBusNode::on_cmd_mode(const jr_interfaces::msg::JointCommandArray::ConstSharedPtr msg)
{
    if (tg_ == nullptr || msg->commands.empty()) return;

    /* 校验需要知道"每个关节**使能时**选的模式"——它在快照里（`cmd_mode`），
       而不是在配置里：配置能改，运行期只认当时真正使能用的那一个。 */
    const StateSnapshot *snap = tg_->acquire_snapshot();
    if (snap == nullptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "'~/cmd' dropped (%zu command(s)): no snapshot yet — the bus is not "
                             "running (activate the node and enable the joints first)",
                             msg->commands.size());
        return;
    }

    CommandSet cmd;
    unsigned used = 0u;
    unsigned bad_mode = 0u;
    unsigned mismatched = 0u;
    unsigned bad_value = 0u;

    for (const auto &c : msg->commands) {
        const int gl = global_index_of(c.name);
        if (gl < 0) {
            warn_unknown_joint_once(c.name);
            continue;
        }
        const unsigned g = static_cast<unsigned>(gl);
        if (g >= kMaxJoints) continue;

        /* ---- ① 模式值合法吗 ---- */
        CmdMode m = CmdMode::kMit;
        bool mode_ok = true;
        switch (c.mode) {
        case jr_interfaces::msg::JointCommand::CSP: m = CmdMode::kCsp; break;
        case jr_interfaces::msg::JointCommand::CSV: m = CmdMode::kCsv; break;
        case jr_interfaces::msg::JointCommand::CST: m = CmdMode::kCst; break;
        case jr_interfaces::msg::JointCommand::CURRENT: m = CmdMode::kCurrent; break;
        default: mode_ok = false; break;
        }
        if (!mode_ok) {
            ++bad_mode;
            continue;
        }
        /* ---- ② 与该关节使能时的模式一致吗（核心库还会再拦一道，这里是给人看的）---- */
        const JointStatePOD &j = snap->joints[g];
        if (j.cmd_mode != static_cast<std::uint8_t>(m)) {
            ++mismatched;
            const std::string key = c.name + "/" + std::to_string(c.mode);
            if (std::find(warned_modes_.begin(), warned_modes_.end(), key) == warned_modes_.end()) {
                warned_modes_.push_back(key);
                RCLCPP_WARN(get_logger(),
                            "command for joint '%s' REFUSED (nothing was sent): it is enabled in "
                            "'%s' mode but this command uses '%s'. Switching modes is a blocking "
                            "disable→enable and cannot happen on the command path — set "
                            "joints[].mode: %s in the config (then clean/configure/activate) and "
                            "publish the matching command type.",
                            c.name.c_str(), jr::to_string(static_cast<CmdMode>(j.cmd_mode)),
                            jr::to_string(m), jr::to_string(m));
            }
            continue;
        }

        /* ---- ③ 量值：非有限值会污染整条控制回路（NaN 比越界危险得多）---- */
        if (!std::isfinite(c.target) || !std::isfinite(c.velocity_limit) ||
            !std::isfinite(c.current_limit)) {
            ++bad_value;
            continue;
        }

        /* ---- ④ 组目标 ---- */
        jr::JointTarget t;
        t.mode = m;
        t.velocity_limit = (c.velocity_limit > 0.0) ? c.velocity_limit : 0.0;
        t.current_limit = (c.current_limit > 0.0) ? c.current_limit : 0.0;
        switch (m) {
        case CmdMode::kCsp: t.position = c.target; break;
        case CmdMode::kCsv: t.velocity = c.target; break;
        case CmdMode::kCst:
            t.torque = c.target;
            if (t.velocity_limit > 0.0 || t.current_limit > 0.0) {
                /* SDK 的限制量只对 CSP/CSV/CURRENT 有效；CST 下给了也没用 → 说一次。 */
                const std::string key = c.name + "/cst_limits";
                if (std::find(warned_modes_.begin(), warned_modes_.end(), key) ==
                    warned_modes_.end()) {
                    warned_modes_.push_back(key);
                    RCLCPP_WARN(get_logger(),
                                "joint '%s': velocity_limit/current_limit are ignored in CST mode "
                                "(the SDK applies limits to CSP/CSV/CURRENT only)",
                                c.name.c_str());
                }
            }
            break;
        case CmdMode::kCurrent: t.current = c.target; break;
        case CmdMode::kMit: default: continue;   /* 不可能到这里（上面 switch 已排掉） */
        }
        cmd.joint[g] = t;
        cmd.set_mask(g);
        ++used;
    }

    if (used != 0u) submit(&cmd);

    /* 拒绝的原因必须**看得见**（客户只会看到"关节不动"，日志里得有答案）。 */
    cmd_rejected_self_ += mismatched + bad_mode + bad_value;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "'~/cmd': %u accepted, %u refused (mode mismatch %u, bad mode value %u, "
                         "non-finite value %u) — see the per-joint warnings above",
                         used, mismatched + bad_mode + bad_value, mismatched, bad_mode, bad_value);
}

void JrBusNode::on_estop_local(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
    if (tg_ == nullptr) return;
    if (!msg->data) {
        RCLCPP_INFO(get_logger(), "estop=false received: nothing to do (recovery is FaultReset)");
        return;
    }
    CommandSet cmd;
    cmd.estop = true;
    submit(&cmd);
    estop_latched_ = true;
    /* estop 是**整条总线的全局广播**（Dest=0xFF）：这条线上不属于我们的设备也会收到并 disarm。
       README 首屏与 §8.4 都写了；日志里再说一次，因为代价是"别人也停了"。 */
    RCLCPP_WARN(get_logger(),
                "ESTOP broadcast requested on bus '%s': this is a bus-wide broadcast (Dest=0xFF) and "
                "will disarm every node on the bus, including devices that are not ours.",
                bus_name_.c_str());
}

void JrBusNode::on_estop_all(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
    on_estop_local(msg);
}

/* ==========================================================================
 * 状态发布
 * ======================================================================== */

void JrBusNode::publish_feedback()
{
    if (tg_ == nullptr || pub_feedback_ == nullptr) return;
    bool got_new = false;
    const StateSnapshot *snap = tg_->acquire_snapshot(&got_new);
    if (snap == nullptr) return;

    jr_interfaces::msg::JointFeedbackArray out;
    out.header.stamp = to_ros_time(snap->t_ns);
    out.header.frame_id = get_parameter("frame_id").as_string();
    out.joints.reserve(local_joint_count_);
    for (unsigned i = 0u; i < snap->joint_count; ++i) {
        const JointStatePOD &j = snap->joints[i];
        if (j.bus_index != bus_index_) continue;
        jr_interfaces::msg::JointFeedback f;
        f.header = out.header;
        f.name = j.name;
        f.position = j.position;
        f.velocity = j.velocity;
        f.effort = j.effort;
        f.current = j.current;
        f.motor_temperature = j.motor_temperature;
        f.fet_temperature = j.fet_temperature;
        f.bus_voltage = j.bus_voltage;
        f.bus_current = j.bus_current;
        f.age_ms = j.age_ms;
        f.status_flags = j.status_flags;
        f.mode_state = j.mode_state;
        f.error_code = j.err_code;
        f.heartbeat_error = j.hb_error;
        f.axis_error = j.axis_error;
        f.tx_rejected = j.tx_rejected;
        f.online = j.online;
        f.enabled = j.enabled;
        f.calibrated = j.calibrated;
        f.valid = j.valid_fresh;
        out.joints.push_back(std::move(f));
    }
    pub_feedback_->publish(out);
}

void JrBusNode::publish_joint_states()
{
    if (tg_ == nullptr || pub_joint_states_ == nullptr) return;
    const StateSnapshot *snap = tg_->acquire_snapshot();
    if (snap == nullptr) return;

    sensor_msgs::msg::JointState out;
    out.header.stamp = to_ros_time(snap->t_ns);
    out.name.reserve(local_joint_count_);
    out.position.reserve(local_joint_count_);
    out.velocity.reserve(local_joint_count_);
    out.effort.reserve(local_joint_count_);
    for (unsigned i = 0u; i < snap->joint_count; ++i) {
        const JointStatePOD &j = snap->joints[i];
        if (j.bus_index != bus_index_) continue;
        out.name.emplace_back(j.name);
        out.position.push_back(j.position);
        out.velocity.push_back(j.velocity);
        out.effort.push_back(j.effort);
    }
    pub_joint_states_->publish(out);
}

void JrBusNode::fill_bus_status(const StateSnapshot &snap, jr_interfaces::msg::BusStatus *out) const
{
    const BusStatsPOD &b = snap.buses[bus_index_];
    out->bus = b.name;
    out->link_up = b.link_up;
    out->nodes_online = b.nodes_online;
    out->tx_frames = b.tx_frames;
    out->rx_frames = b.rx_frames;
    out->tx_failed = b.tx_failed;
    out->rx_dropped = b.rx_dropped;
    out->keepalive_sent = b.keepalive_sent;
    out->link_errors = b.link_errors;
    out->last_rx_age_ms = b.last_rx_age_ms;
    out->hal_bus_flags = b.hal_bus_flags;
    out->bus_load_estimate = static_cast<float>(b.bus_load_estimate);
    out->tick_overruns = snap.rt.missed_ticks;
    out->command_overwrites = snap.rt.command_overwrites;
    /* 降级与原因**必须**出现在这里（SDK 文档明确要求"降级原因可见"）。
       快照里有两处备注，**两个都要带出来**（核心库的注释说得很清楚：
       `buses[i].last_note` 是总线级的（如广播降级），`snap.note` 是组级的
       （如命令超时）；只取其一就会丢信息）。 */
    const std::string bus_note(b.last_note);
    const std::string group_note(snap.note);
    if (!bus_note.empty() && !group_note.empty()) {
        out->note = bus_note + " | " + group_note;
    } else {
        out->note = bus_note.empty() ? group_note : bus_note;
    }
    out->degraded = b.degraded || !out->note.empty() || estop_latched_;
    if (estop_latched_) {
        if (out->note.empty()) {
            out->note = "ESTOP broadcast was sent from this node";
        } else {
            out->note += " | ESTOP broadcast was sent from this node";
        }
    }
}

void JrBusNode::publish_bus_status()
{
    if (tg_ == nullptr || pub_bus_status_ == nullptr) return;
    const StateSnapshot *snap = tg_->acquire_snapshot();
    if (snap == nullptr || bus_index_ >= snap->bus_count) return;
    jr_interfaces::msg::BusStatus out;
    fill_bus_status(*snap, &out);
    pub_bus_status_->publish(out);
}

void JrBusNode::publish_rt_stats()
{
    if (tg_ == nullptr || pub_rt_stats_ == nullptr) return;
    const StateSnapshot *snap = tg_->acquire_snapshot();
    if (snap == nullptr) return;
    const RtStatsPOD &r = snap->rt;

    jr_interfaces::msg::RtStats out;
    out.rate_hz = static_cast<double>(tg_->rate_hz());
    out.period_ns = r.period_ns;
    out.jitter_last_ns = r.jitter_last_ns;
    out.jitter_min_ns = r.jitter_min_ns;
    out.jitter_mean_ns = r.jitter_mean_ns;
    out.jitter_max_ns = r.jitter_max_ns;
    out.jitter_p99_ns = r.jitter_p99_ns;
    out.cycle_last_ns = r.cycle_last_ns;
    out.cycle_min_ns = r.cycle_min_ns;
    out.cycle_mean_ns = r.cycle_mean_ns;
    out.cycle_max_ns = r.cycle_max_ns;
    out.cmd_to_tx_last_ns = r.cmd_to_tx_last_ns;
    out.cmd_to_tx_mean_ns = r.cmd_to_tx_mean_ns;
    out.cmd_to_tx_max_ns = r.cmd_to_tx_max_ns;
    out.missed_ticks = r.missed_ticks;
    out.tick_count = r.tick_count;
    out.config_pauses = r.config_pauses;
    /* 被**我们**丢掉的命令（节点侧拒绝 + 核心侧丢弃）—— "关节不动"时先看这个：
       非零说明命令压根没到设备（节点拒了，或 RT 侧因模式不符丢了）。 */
    {
        std::uint32_t rejected = cmd_rejected_self_;
        for (unsigned i = 0u; i < snap->joint_count; ++i) rejected += snap->joints[i].cmd_rejected;
        out.cmd_rejected = rejected;
    }
    out.rt_sched_ok = r.rt_sched_ok;
    out.mlock_ok = r.mlock_ok;
    out.affinity_ok = r.affinity_ok;
    out.rt_throttled = r.rt_throttled;
    /* 快照生成 → 发布出去的时延：只能在这一侧量（RT 侧不知道我们什么时候发）。 */
    const std::uint64_t now_mono = monotonic_ns();
    if (now_mono > snap->t_ns) {
        const std::uint64_t d = now_mono - snap->t_ns;
        out.snapshot_to_pub_ns = (d > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<std::uint32_t>(d);
    } else {
        out.snapshot_to_pub_ns = 0u;
    }
    pub_rt_stats_->publish(out);
}

void JrBusNode::drain_and_publish_faults()
{
    if (tg_ == nullptr || pub_faults_ == nullptr) return;
    rt::FaultEvent ev[8];
    const unsigned n = tg_->drain_faults(ev, 8u);
    if (n == 0u) return;

    const BusCfg &bus = cfg_.buses[bus_index_];
    for (unsigned i = 0u; i < n; ++i) {
        jr_interfaces::msg::JointFault out;
        out.event = ev[i].event;
        out.error_code = ev[i].err_code;
        out.heartbeat_error = ev[i].hb_error;
        out.axis_error = ev[i].axis_error;
        out.motor_error = ev[i].motor_error;
        out.encoder_error = ev[i].encoder_error;
        out.sensorless_error = ev[i].sensorless_error;
        out.controller_error = ev[i].controller_error;
        out.system_error = ev[i].system_error;
        out.text = ev[i].text;
        out.advice = 0u;   /* 由诊断/恢复建议层填（WP5）；这里先给原始位 */
        for (unsigned j = 0u; j < bus.joint_count; ++j) {
            if (bus.joints[j].node_id == ev[i].node_id) {
                out.name = bus.joints[j].name;
                break;
            }
        }
        if (out.name.empty()) {
            char buf[32] = {};
            std::snprintf(buf, sizeof(buf), "node#%u", static_cast<unsigned>(ev[i].node_id));
            out.name = buf;
        }
        pub_faults_->publish(out);
        RCLCPP_WARN(get_logger(), "fault %s on '%s': %s (error_code=0x%02X heartbeat=0x%02X axis=0x%08X)",
                    ev[i].event == jr_interfaces::msg::JointFault::APPEARED ? "APPEARED" : "CLEARED",
                    out.name.c_str(), ev[i].text, ev[i].err_code, ev[i].hb_error, ev[i].axis_error);
    }
    if (tg_->bus(bus_index_).faults_dropped() != 0u) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "%llu fault event(s) were dropped (ring full): increase the drain rate "
                             "or fix the underlying fault storm",
                             static_cast<unsigned long long>(tg_->bus(bus_index_).faults_dropped()));
    }
}

/* ==========================================================================
 * 诊断（DESIGN §6.5）
 * --------------------------------------------------------------------------
 *  两个原则：
 *   ① hardware_id 分两层：`jr:<bus>` 与 `jr:<bus>/<joint>`（设计就是这么写的，
 *      所以总线级与关节级各用一个 updater，而不是硬塞进一个）；
 *   ② **每个结论都能追到原始值**（位图/计数/读回值），摘要文本一律附在原始值之后。
 * ======================================================================== */

void JrBusNode::create_diagnostics()
{
    if (!diags_.empty()) return;   /* 幂等 */

    const double period_s = 1.0;   /* 诊断 1 Hz：足够早发现，又不占带宽 */

    auto bus_up = std::make_shared<diagnostic_updater::Updater>(this, period_s);
    bus_up->setHardwareID("jr:" + bus_name_);
    bus_up->add("bus", [this](diagnostic_updater::DiagnosticStatusWrapper &stat) { diag_bus(stat); });
    diags_.push_back(bus_up);

    for (unsigned j = 0u; j < local_joint_count_; ++j) {
        auto up = std::make_shared<diagnostic_updater::Updater>(this, period_s);
        up->setHardwareID("jr:" + bus_name_ + "/" + joint_names_[j]);
        up->add("joint", [this, j](diagnostic_updater::DiagnosticStatusWrapper &stat) {
            diag_joint(j, stat);
        });
        diags_.push_back(up);
    }
    RCLCPP_INFO(get_logger(), "diagnostics up: %zu updater(s) (bus + %u joint(s))", diags_.size(),
                local_joint_count_);
}

void JrBusNode::destroy_diagnostics() { diags_.clear(); }

void JrBusNode::diag_bus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    const StateSnapshot *s = (tg_ != nullptr) ? tg_->acquire_snapshot() : nullptr;
    if (s == nullptr || bus_index_ >= s->bus_count) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "no snapshot yet");
        return;
    }
    const BusStatsPOD &b = s->buses[bus_index_];
    const BusCfg &bc = cfg_.buses[bus_index_];

    char flags[192] = {};
    rt::bus_flags_text(b.hal_bus_flags, flags, sizeof(flags));

    stat.add("link_up", b.link_up ? 1 : 0);
    stat.add("nodes_online", static_cast<int>(b.nodes_online));
    stat.add("tx_frames", b.tx_frames);
    stat.add("rx_frames", b.rx_frames);
    stat.add("tx_failed", b.tx_failed);
    stat.add("rx_dropped", b.rx_dropped);
    stat.add("keepalive_sent", b.keepalive_sent);
    stat.add("link_errors", b.link_errors);
    stat.add("last_rx_age_ms", b.last_rx_age_ms);
    stat.add("hal_bus_flags_raw", b.hal_bus_flags);
    stat.add("hal_bus_flags", flags);
    stat.add("bus_load_estimate", b.bus_load_estimate);
    stat.add("tick_overruns", s->rt.missed_ticks);
    stat.add("command_overwrites", s->rt.command_overwrites);
    stat.add("rt_throttled", s->rt.rt_throttled ? 1 : 0);
    stat.add("rt_sched_ok", s->rt.rt_sched_ok ? 1 : 0);
    stat.add("descriptor_complete", tg_->bus(bus_index_).report().desc.complete ? 1 : 0);
    stat.add("descriptor_from_cache", tg_->bus(bus_index_).report().desc.from_cache ? 1 : 0);
    stat.add("descriptor_endpoints", tg_->bus(bus_index_).report().desc.parsed_total);
    stat.add("lock_held", tg_->bus(bus_index_).lock().held() ? 1 : 0);
    stat.add("lock_owner_pid", tg_->bus(bus_index_).lock().owner_pid_on_disk());
    stat.add("lock_path", tg_->bus(bus_index_).lock().path());
    stat.add("degraded", b.degraded ? 1 : 0);
    stat.add("note", std::string(b.last_note));

    /* 等级与**原因**（不要只给一个 ERROR 不说为什么）。 */
    const bool bus_off = (b.hal_bus_flags & 0x08u) != 0u;         /* JSDK_HAL_BUS_OFF */
    const bool err_pass = (b.hal_bus_flags & 0x04u) != 0u;        /* JSDK_HAL_BUS_ERROR_PASS */
    const bool err_warn = (b.hal_bus_flags & 0x02u) != 0u;        /* JSDK_HAL_BUS_ERROR_WARN */
    if (!b.link_up) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "link is down");
    } else if (bus_off) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                     "bus-off: check wiring/termination/bitrate (the driver may need a restart)");
    } else if (err_pass || err_warn) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     err_pass ? "error-passive: the bus is degrading (termination/wiring/bitrate)"
                              : "error-warning: rising error counter");
    } else if (s->rt.rt_throttled) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     "RT throttling is active: jitter will exceed expectations (see PERF docs)");
    } else if (s->rt.missed_ticks != 0u) {
        stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN, "%llu tick(s) missed",
                      static_cast<unsigned long long>(s->rt.missed_ticks));
    } else if (!tg_->bus(bus_index_).report().desc.complete) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     "descriptor is incomplete: the endpoint table may be partial");
    } else if (b.degraded) {
        stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded: %s", b.last_note);
    } else if (bc.master_id == 0u) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "master_id is 0 (invalid)");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "bus healthy");
    }
}

void JrBusNode::diag_joint(unsigned local_index, diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    const StateSnapshot *s = (tg_ != nullptr) ? tg_->acquire_snapshot() : nullptr;
    if (s == nullptr) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "no snapshot yet");
        return;
    }
    /* 全局索引 = 总线基址 + 总线内序号（快照按全局索引排布）。 */
    const unsigned global = bus_joint_base(cfg_, bus_index_) + local_index;
    if (global >= s->joint_count) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "joint is missing from the snapshot");
        return;
    }
    const JointStatePOD &j = s->joints[global];

    char hb[192] = {};
    rt::joint_status_flags_text(j.status_flags, hb, sizeof(hb));
    char fault[192] = {};
    const bool has_fault = tg_->bus(bus_index_).joint_fault_text(local_index, fault, sizeof(fault));

    stat.add("online", j.online ? 1 : 0);
    stat.add("enabled", j.enabled ? 1 : 0);
    stat.add("calibrated", j.calibrated ? 1 : 0);
    stat.add("age_ms", j.age_ms);
    stat.add("position_rad", j.position);
    stat.add("velocity_rad_s", j.velocity);
    stat.add("effort_Nm", j.effort);
    stat.add("current_A_motor_side", j.current);
    stat.add("motor_temperature_C", j.motor_temperature);
    stat.add("fet_temperature_C", j.fet_temperature);
    stat.add("bus_voltage_V", j.bus_voltage);
    stat.add("mode_state_raw", static_cast<int>(j.mode_state));
    /* 使能时选的命令模式（`joints[].mode`）：与 `mode_state_raw` 对照看能一眼发现
       “配置说 CSP，设备却在 MIT”这种漂移。 */
    stat.add("cmd_mode", jr::to_string(static_cast<CmdMode>(j.cmd_mode)));
    stat.add("cmd_rejected", j.cmd_rejected);
    stat.add("error_code_raw", static_cast<int>(j.err_code));
    stat.add("error_code_note",
             "4-bit SUMMARY only (estop and watchdog both map to CAN_TIMEOUT) — see the raw bit fields");
    stat.add("heartbeat_error_raw", static_cast<int>(j.hb_error));
    stat.add("axis_error_raw", j.axis_error);
    stat.add("status_flags_raw", j.status_flags);
    stat.add("status_flags", hb);
    stat.add("tx_rejected", j.tx_rejected);
    stat.add("feedback_fresh", j.valid_fresh ? 1 : 0);
    /* 看门狗状态：三态（关闭 / 已武装 / 未校验）—— 这是现场最容易误判的一项。 */
    const std::uint32_t bt = tg_->bus(bus_index_).report().joint[local_index].break_timeout_ms;
    const bool armed_cfg = cfg_.buses[bus_index_].arm_device_watchdog;
    const char *wd = "disabled (device break_timeout = 0)";
    if (bt != 0u) {
        wd = j.watchdog_unverified ? "configured but NOT verified (firmware did not confirm)"
                                   : (armed_cfg ? "armed by this node" : "armed by the device config");
    }
    stat.add("watchdog_state", wd);
    stat.add("device_break_timeout_ms", bt);
    stat.add("fault_text", fault);

    if (!j.online) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "joint is offline");
    } else if (has_fault) {
        stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "fault: %s | %s", fault, hb);
    } else if (!j.calibrated) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     "not calibrated: physical-quantity commands are unavailable (run Calibrate)");
    } else if (j.feedback_stale) {
        stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN, "feedback stale (age_ms=%u)",
                      j.age_ms);
    } else if (j.target_rejected) {
        stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                      "a target was rejected as out of range (%u time(s) so far) — check limits",
                      j.tx_rejected);
    } else if (j.watchdog_unverified) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     "watchdog: configured but the device did not confirm it (WATCHDOG_UNVERIFIED)");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "joint healthy");
    }
}

}  // namespace ros
}  // namespace jr
