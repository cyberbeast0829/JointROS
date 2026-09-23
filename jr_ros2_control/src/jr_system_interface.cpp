/**
 * @file    jr_system_interface.cpp
 * @brief   `jr_ros2_control` 的 `SystemInterface` 实现
 */

#include "jr_ros2_control/jr_system_interface.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/logging.hpp>

#include <pluginlib/class_list_macros.hpp>

#include "jr_ros2/jr_config_yaml.hpp"

namespace jr_ros2_control {
namespace {

constexpr const char *kPosition = "position";
constexpr const char *kVelocity = "velocity";
constexpr const char *kEffort = "effort";
constexpr const char *kGainWireA = "kp";
constexpr const char *kGainWireB = "kd";
constexpr const char *kGainSiA = "stiffness";
constexpr const char *kGainSiB = "damping";

std::string param(const hardware_interface::HardwareInfo &info, const char *key)
{
    const auto it = info.hardware_parameters.find(key);
    return (it == info.hardware_parameters.end()) ? std::string() : it->second;
}

/* 按关节模式归并出**将要导出**的命令接口集合（同集合只报一次，带关节数，顺序固定）。
   为什么需要它：ready 日志曾经把接口集写死，纯 CSP 配置下报出一个设备根本不承载的集合，
   而 controller_manager 会拿 URDF 声明跟**实际导出集**逐一对账（对不上直接拒初始化）。 */
std::string describe_command_interfaces(const std::vector<std::uint8_t> &modes, bool gain_si)
{
    unsigned mit = 0u;
    unsigned csp = 0u;
    unsigned csv = 0u;
    unsigned cst = 0u;
    for (std::size_t i = 0u; i < modes.size(); ++i) {
        switch (static_cast<jr::CmdMode>(modes[i])) {
        case jr::CmdMode::kCsp: ++csp; break;
        case jr::CmdMode::kCsv: ++csv; break;
        case jr::CmdMode::kCst: ++cst; break;
        default: ++mit; break;
        }
    }
    std::string s;
    const auto add = [&s](const std::string &what, unsigned count) {
        if (count == 0u) return;
        if (!s.empty()) s += " | ";
        s += std::to_string(count) + "x " + what;
    };
    add(std::string("position/velocity/effort + ") + (gain_si ? "stiffness,damping" : "kp,kd"),
        mit);
    add("position", csp);
    add("velocity", csv);
    add("effort", cst);
    return s.empty() ? std::string("(none)") : s;
}

}  // namespace

JrSystemInterface::~JrSystemInterface()
{
    /* 析构里**必须**收尾：否则客户 Ctrl-C 之后可能留下抱力的关节。
       stop() + close(on_exit) 都是幂等的（重复调用安全）。 */
    if (tg_ != nullptr) {
        tg_->stop();
        jr::Result r;
        tg_->close_buses(cfg_.safety.on_exit, &r);
    }
}

/* ==========================================================================
 * ① 初始化（只解析与校验，不碰硬件）
 * ======================================================================== */

JrSystemInterface::CallbackReturn
JrSystemInterface::on_init(const compat::OnInitParams &params)
{
    /* 显式限定基类（**不会**虚派发到我们自己），让基类完成它的解析/校验。 */
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    /* 两种发行版的参数类型在兼容层里归一成 `HardwareInfo`（业务逻辑只认它）。 */
    const hardware_interface::HardwareInfo &info = compat::info_of(params);
    logger_ = rclcpp::get_logger(info.name.empty() ? std::string("jr_ros2_control") : info.name);

    config_file_ = param(info, "config_file");
    if (config_file_.empty()) {
        RCLCPP_FATAL(logger_,
                     "<param name=\"config_file\">/path/to/jr.yaml is required (same YAML schema as "
                     "the jr_bus node, DESIGN 6.4)");
        return CallbackReturn::ERROR;
    }

    const std::string ts = param(info, "tick_source");
    if (ts.empty() || ts == "internal") {
        tick_source_ = TickSource::kInternal;
    } else if (ts == "controller_manager") {
        tick_source_ = TickSource::kControllerManager;
    } else {
        RCLCPP_FATAL(logger_,
                     "tick_source: unknown value '%s' (expected: internal | controller_manager)",
                     ts.c_str());
        return CallbackReturn::ERROR;
    }

    const std::string gm = param(info, "gain_mode");
    if (gm.empty() || gm == "wire") {
        gain_si_ = false;
    } else if (gm == "si") {
        gain_si_ = true;
    } else {
        RCLCPP_FATAL(logger_, "gain_mode: unknown value '%s' (expected: wire | si)", gm.c_str());
        return CallbackReturn::ERROR;
    }

    jr::Result r;
    jr::YamlLoadReport rep;
    const jr::Status st = jr::load_config_yaml(config_file_.c_str(), &cfg_, &r, &rep);
    if (st != jr::Status::kOk) {
        RCLCPP_FATAL(logger_, "config '%s' rejected: %s", config_file_.c_str(), r.message);
        return CallbackReturn::ERROR;
    }
    if (rep.notes.text[0] != '\0') {
        /* 非致命但必须让用户看到的东西（例如某关节 node_id > 7 → 无法广播同步）。 */
        RCLCPP_WARN(logger_, "config '%s' notes:%s", config_file_.c_str(), rep.notes.text);
    }
    if (rep.node_keys > 0u) {
        RCLCPP_INFO(logger_,
                    "config has %u node-level key(s) not used by this component: %s",
                    rep.node_keys, rep.node_keys_text);
    }

    std::string err;
    if (!map_joints(&err)) {
        RCLCPP_FATAL(logger_, "%s", err.c_str());
        return CallbackReturn::ERROR;
    }

    const std::size_t n = info_.joints.size();
    state_position_.assign(n, 0.0);
    state_velocity_.assign(n, 0.0);
    state_effort_.assign(n, 0.0);
    cmd_position_.assign(n, 0.0);
    cmd_velocity_.assign(n, 0.0);
    cmd_effort_.assign(n, 0.0);
    cmd_gain_a_.assign(n, 0.0);
    cmd_gain_b_.assign(n, 0.0);

    /* 真正的导出集在 `export_command_interfaces()` 里才算得出来（本函数在它之前跑），
       先清成 "" —— 纯 CSP 配置下 `gain_interface_names()` 应当返回空串，不是 "kp,kd"。 */
    gain_iface_count_ = 0u;
    gain_iface_names_[0] = '\0';

    /* ⚠ 这行以前把命令接口集**写死**成 "position/velocity/effort + kp,kd"，在 CSP 配置下
       那是假话：实测客户照着它去写 URDF，controller_manager 直接拒绝初始化硬件
         "Discrepancy between robot description file (urdf) and actually exported HW interfaces"。
        ⇒ 日志必须由**当前模式**算出来。 */
    const std::string ifaces = describe_command_interfaces(joint_mode_, gain_si_);
    RCLCPP_INFO(logger_,
                "jr_ros2_control ready: %zu joint(s), tick_source=%s, gain_mode=%s, "
                "command interfaces: %s (URDF must declare exactly these), on_init=%s",
                n, (tick_source_ == TickSource::kInternal) ? "internal" : "controller_manager",
                gain_si_ ? "si" : "wire", ifaces.c_str(), compat::on_init_signature());
    return CallbackReturn::SUCCESS;
}

bool JrSystemInterface::map_joints(std::string *err)
{
    /* 配置里的全局关节顺序 = 总线顺序 × 总线内顺序（与 jr_core 的快照下标一致）。 */
    joint_name_.clear();
    for (unsigned b = 0u; b < cfg_.bus_count; ++b) {
        for (unsigned k = 0u; k < cfg_.buses[b].joint_count; ++k) {
            joint_name_.push_back(std::string(cfg_.buses[b].joints[k].name));
        }
    }

    joint_index_.assign(info_.joints.size(), 0u);
    joint_mode_.assign(info_.joints.size(), static_cast<std::uint8_t>(jr::CmdMode::kMit));
    for (std::size_t i = 0u; i < info_.joints.size(); ++i) {
        const std::string &need = info_.joints[i].name;
        std::size_t found = joint_name_.size();
        for (std::size_t g = 0u; g < joint_name_.size(); ++g) {
            if (joint_name_[g] == need) {
                found = g;
                break;
            }
        }
        if (found == joint_name_.size()) {
            std::string known;
            for (const std::string &j : joint_name_) {
                if (!known.empty()) known += ", ";
                known += j;
            }
            *err = "URDF joint '" + need + "' is not in the config file (configured joints: " +
                   (known.empty() ? std::string("<none>") : known) + ")";
            return false;
        }
        /* 显式窄化：`found < joint_name_.size()` 已在上面的检查里保证（关节数上限很小），
           但不写 cast 会触发 -Wconversion（跨发行版真的有告警）。 */
        joint_index_[i] = static_cast<unsigned>(found);
        /* 顺便把模式记下来（`export_command_interfaces()` / `write()` / `apply_default_gains()`
           都要用它；模式是**使能时**定下的，不再变）。 */
        {
            unsigned bus = 0u;
            unsigned local = 0u;
            if (locate_joint(joint_index_[i], &bus, &local)) {
                joint_mode_[i] = static_cast<std::uint8_t>(cfg_.buses[bus].joints[local].mode);
            }
        }
    }

    /* 反向检查：配置了但 URDF 里没有的关节 → 只是提示（可能是给 WP2 节点用的配置）。 */
    std::string unused;
    for (std::size_t g = 0u; g < joint_name_.size(); ++g) {
        bool used = false;
        for (std::size_t i = 0u; i < joint_index_.size(); ++i) {
            if (joint_index_[i] == g) {
                used = true;
                break;
            }
        }
        if (!used) {
            if (!unused.empty()) unused += ", ";
            unused += joint_name_[g];
        }
    }
    if (!unused.empty()) {
        RCLCPP_WARN(logger_, "configured but not in the URDF (ignored by ros2_control): %s",
                    unused.c_str());
    }

    /* ⚠ 模式一致性：ros2_control 这条路径发的是 **MIT 目标**（position/velocity/effort 都经
       MIT 力位混合下发）。若某个关节在 YAML 里配成了 CSP/CSV/CST/CURRENT，核心库会把
       MIT 目标**丢弃并计数**（这是对的：MIT 帧会静默改掉它的输入模式）——
       结果是"控制器在跑，关节不动"。与其让客户看这种哑谜，不如在这里就拒绝启动。 */
    for (std::size_t i = 0u; i < joint_index_.size(); ++i) {
        unsigned bus = 0u;
        unsigned local = 0u;
        if (!locate_joint(joint_index_[i], &bus, &local)) continue;
        const jr::JointCfg &jc = cfg_.buses[bus].joints[local];
        /* `current` 仍拒绝：不是“不行”，而是它的 `effort` 语义会从 N·m 变成**电机端 A**，
           PID/JTC 算出来的是 N·m ⇒ 差一个 gear×kt，等于在接口里撒谎（还叠加 F19 的看门狗缺口）。
           CSP/CSV/CST 都支持：它们各自的帧里本来就只有一种物理量，映射是诚实的。 */
        if (jc.mode == jr::CmdMode::kCurrent) {
            *err = std::string("joint '") + joint_name_[joint_index_[i]] +
                   "' is configured in 'current' mode. ros2_control's 'effort' interface is "
                   "torque in N·m, but a CURRENT frame carries **motor-side amps** — mapping it "
                   "would silently mis-scale every torque by gear×kt (and CURRENT frames don't "
                   "feed the device watchdog, SDK F19). Drive this joint from the '~/cmd' topic, "
                   "or use cst if you want torque in N·m.";
            return false;
        }
    }
    return true;
}

bool JrSystemInterface::locate_joint(unsigned global, unsigned *bus, unsigned *local) const noexcept
{
    unsigned seen = 0u;
    for (unsigned b = 0u; b < cfg_.bus_count; ++b) {
        for (unsigned k = 0u; k < cfg_.buses[b].joint_count; ++k, ++seen) {
            if (seen == global) {
                *bus = b;
                *local = k;
                return true;
            }
        }
    }
    return false;
}

void JrSystemInterface::set_command_for_test(unsigned i, double position, double velocity,
                                              double effort, double gain_a, double gain_b) noexcept
{
    if (i >= cmd_position_.size()) return;
    cmd_position_[i] = position;
    cmd_velocity_[i] = velocity;
    cmd_effort_[i] = effort;
    cmd_gain_a_[i] = gain_a;
    cmd_gain_b_[i] = gain_b;
}

void JrSystemInterface::apply_default_gains()
{
    const double two_pi = 6.283185307179586;
    for (std::size_t i = 0u; i < joint_index_.size(); ++i) {
        const unsigned g = joint_index_[i];

        /* 找到这个全局关节属于哪条总线、总线内的第几个关节（才能拿到 gear_ratio/量程）。 */
        unsigned bus = 0u;
        unsigned local = 0u;
        if (!locate_joint(g, &bus, &local)) continue;

        const jr::JointCfg &jc = cfg_.buses[bus].joints[local];
        if (jc.stiffness == 0.0 && jc.damping == 0.0) continue;   /* 没配默认值 → 不动 */

        /* ⚠ 非 MIT 模式的帧里**没有增益位置** ⇒ YAML 里写的 stiffness/damping 对它们无效。
           与其让它静默失效，不如说一句（客户会以为"我配了刚度"）。 */
        if (jc.mode != jr::CmdMode::kMit) {
            RCLCPP_WARN(logger_,
                        "joint '%s': limits.%s.stiffness/damping are ignored — the joint is in "
                        "'%s' mode and those frames carry no gains (the joint's own controller "
                        "parameters do the work; read/write them with the WriteParams/ReadParams "
                        "services, e.g. axis0.controller.config.*)",
                        joint_name_[g].c_str(), joint_name_[g].c_str(), jr::to_string(jc.mode));
            continue;
        }

        const jr::rt::JointInfoPOD &info = tg_->bus(bus).report().joint[local];
        double a = jc.stiffness;
        double b = jc.damping;
        if (!gain_si_) {
            if (info.gear_ratio <= 0.0) {
                RCLCPP_WARN(logger_,
                            "joint '%s': cannot convert the SI default stiffness/damping to wire "
                            "gains (gear_ratio=%.3f) - the defaults are ignored; write kp/kd from the "
                            "controller instead",
                            joint_name_[g].c_str(), info.gear_ratio);
                continue;
            }
            a = jc.stiffness * two_pi / info.gear_ratio;
            b = jc.damping * two_pi / info.gear_ratio;
        }

        /* 按设备量程夹紧：MIT 目标越界会被 SDK 拒绝（除非 clamp_target），
           与其让客户看到"部分帧被拒"，不如一开始就给合法值并告诉它被夹了。 */
        if (info.mit_max_kp > 0.0 && a > info.mit_max_kp) {
            RCLCPP_WARN(logger_, "joint '%s': default %s %.3f exceeds the device limit %.3f - clamped",
                        joint_name_[g].c_str(), gain_si_ ? "stiffness" : "kp", a, info.mit_max_kp);
            a = info.mit_max_kp;
        }
        if (info.mit_max_kd > 0.0 && b > info.mit_max_kd) {
            RCLCPP_WARN(logger_, "joint '%s': default %s %.3f exceeds the device limit %.3f - clamped",
                        joint_name_[g].c_str(), gain_si_ ? "damping" : "kd", b, info.mit_max_kd);
            b = info.mit_max_kd;
        }

        cmd_gain_a_[i] = a;
        cmd_gain_b_[i] = b;
        RCLCPP_INFO(logger_, "joint '%s': default MIT gains %s=%.3f %s=%.3f (from the YAML limits)",
                    joint_name_[g].c_str(), gain_si_ ? "stiffness" : "kp", a,
                    gain_si_ ? "damping" : "kd", b);
    }
}

/* ==========================================================================
 * ② 接口导出
 * ======================================================================== */

std::vector<hardware_interface::StateInterface> JrSystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> out;
    out.reserve(info_.joints.size() * 3u);
    for (std::size_t i = 0u; i < info_.joints.size(); ++i) {
        out.emplace_back(info_.joints[i].name, kPosition, &state_position_[i]);
        out.emplace_back(info_.joints[i].name, kVelocity, &state_velocity_[i]);
        out.emplace_back(info_.joints[i].name, kEffort, &state_effort_[i]);
    }
    return out;
}

std::vector<hardware_interface::CommandInterface> JrSystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> out;
    out.reserve(info_.joints.size() * 5u);
    unsigned gain_count = 0u;
    for (std::size_t i = 0u; i < info_.joints.size(); ++i) {
        /* ⚠ 只导出该模式**真正承载**的物理量：
           CSP/CSV/CST 的帧里没有增益位置，导出了就是“写了没用”（客户会在 YAML 里配刚度、
           控制器里写 kp/kd，然后什么也没发生）。模式在 `map_joints()` 里已经从配置读出。 */
        const jr::CmdMode m = (i < joint_mode_.size())
                                  ? static_cast<jr::CmdMode>(joint_mode_[i])
                                  : jr::CmdMode::kMit;
        const char *jn = info_.joints[i].name.c_str();
        switch (m) {
        case jr::CmdMode::kCsp:
            out.emplace_back(jn, kPosition, &cmd_position_[i]);
            break;
        case jr::CmdMode::kCsv:
            out.emplace_back(jn, kVelocity, &cmd_velocity_[i]);
            break;
        case jr::CmdMode::kCst:
            out.emplace_back(jn, kEffort, &cmd_effort_[i]);
            break;
        case jr::CmdMode::kMit:
        default:
            out.emplace_back(jn, kPosition, &cmd_position_[i]);
            out.emplace_back(jn, kVelocity, &cmd_velocity_[i]);
            out.emplace_back(jn, kEffort, &cmd_effort_[i]);
            /* ⚠ 两组增益**只导出其中一组**（单位搞错比"claim 失败"危险得多，见头文件说明）。 */
            out.emplace_back(jn, gain_si_ ? kGainSiA : kGainWireA, &cmd_gain_a_[i]);
            out.emplace_back(jn, gain_si_ ? kGainSiB : kGainWireB, &cmd_gain_b_[i]);
            ++gain_count;
            break;
        }
    }
    gain_iface_count_ = gain_count * 2u;
    /* 名字只在**真的导出增益**时才有意义（全 CSP 的配置下不该报 "kp,kd"）。 */
    std::snprintf(gain_iface_names_, sizeof(gain_iface_names_), "%s",
                  (gain_count == 0u) ? "" : (gain_si_ ? "stiffness,damping" : "kp,kd"));
    return out;
}

/* ==========================================================================
 * ③ 生命周期
 * ======================================================================== */

JrSystemInterface::CallbackReturn JrSystemInterface::on_configure(const rclcpp_lifecycle::State &)
{
    if (tg_ != nullptr) {
        RCLCPP_WARN(logger_, "already configured");
        return CallbackReturn::SUCCESS;
    }
    return configure_backend();
}

JrSystemInterface::CallbackReturn JrSystemInterface::configure_backend()
{
    tg_ = std::make_unique<jr::rt::TickGroup>();
    jr::Result r;

    jr::Status st = tg_->init(cfg_, 0u, &r);
    if (st != jr::Status::kOk) {
        RCLCPP_FATAL(logger_, "tick group init failed: %s", r.message);
        tg_.reset();
        return CallbackReturn::ERROR;
    }

    jr::rt::OpenOptions opt;
    st = tg_->open_buses(opt, &r);
    if (st != jr::Status::kOk) {
        RCLCPP_FATAL(logger_, "open failed: %s", r.message);
        tg_.reset();
        return CallbackReturn::ERROR;
    }

    st = tg_->configure_buses(&r);
    if (st != jr::Status::kOk) {
        RCLCPP_FATAL(logger_, "configure failed: %s", r.message);
        tg_->close_buses(jr::ExitAction::kNone, nullptr);
        tg_.reset();
        return CallbackReturn::ERROR;
    }

    if (tick_source_ == TickSource::kInternal) {
        st = tg_->start(nullptr, &r);
    } else {
        st = tg_->start_external(nullptr, &r);
    }
    if (st != jr::Status::kOk) {
        RCLCPP_FATAL(logger_, "starting the tick failed: %s", r.message);
        tg_->close_buses(jr::ExitAction::kNone, nullptr);
        tg_.reset();
        return CallbackReturn::ERROR;
    }

    std::string err;
    if (!prime_snapshot(&err)) {
        RCLCPP_FATAL(logger_, "%s", err.c_str());
        tg_->stop();
        tg_->close_buses(jr::ExitAction::kDisable, nullptr);
        tg_.reset();
        return CallbackReturn::ERROR;
    }

    /* 设备信息已就绪（gear_ratio / mit_max_kp/kd）→ 现在才能算默认 MIT 增益。 */
    apply_default_gains();

    RCLCPP_INFO(logger_, "%s", tg_->report().text);
    primed_ = true;
    return CallbackReturn::SUCCESS;
}

bool JrSystemInterface::prime_snapshot(std::string *err)
{
    if (tick_source_ == TickSource::kControllerManager) {
        /* 外部模式：**主动跑一拍**，否则第一次 read() 拿不到任何状态。
           这一拍也把"设备当前真实位置"带回来，控制器就不会以为初始位置是 0。 */
        jr::Result r;
        if (tg_->step(&r) != jr::Status::kOk) {
            *err = std::string("first cycle failed: ") + r.message;
            return false;
        }
    } else {
        /* 内部模式：等 tick 线程产生首份快照（有界等待，不假装"起来了就等于有数据"）。 */
        for (unsigned i = 0u; i < 1000u; ++i) {
            if (tg_->acquire_snapshot() != nullptr) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        *err = "no snapshot within 1000 ms after starting the tick thread";
        return false;
    }
    if (tg_->acquire_snapshot() == nullptr) {
        *err = "the first cycle produced no snapshot";
        return false;
    }
    return true;
}

JrSystemInterface::CallbackReturn
JrSystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
    if (tg_ == nullptr) {
        RCLCPP_ERROR(logger_, "on_activate before a successful configure");
        return CallbackReturn::ERROR;
    }

    /* SDK 明确禁止"在使能期间被另一个线程碰 context"：内部模式下先安全暂停 tick 线程，
       使能完成后再放它跑；外部模式下本线程就是驱动线程（CM 不会在 on_activate 期间
       调 read()/write()），所以不需要也不能 pause。 */
    jr::Result r;
    if (tick_source_ == TickSource::kInternal) {
        if (tg_->pause(&r) != jr::Status::kOk) {
            RCLCPP_ERROR(logger_, "pause before enable failed: %s", r.message);
            return CallbackReturn::ERROR;
        }
    }

    const jr::Status st = tg_->activate(cfg_.safety.require_calibrated, &r);

    if (tick_source_ == TickSource::kInternal) tg_->resume();

    if (st != jr::Status::kOk) {
        RCLCPP_ERROR(logger_, "enabling joints failed: %s | advice: %s", r.message,
                     jr::to_string(r.advice));
        return CallbackReturn::ERROR;
    }
    active_ = true;
    RCLCPP_INFO(logger_, "joints enabled (control frames start from the next cycle)");
    return CallbackReturn::SUCCESS;
}

JrSystemInterface::CallbackReturn
JrSystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    if (tg_ == nullptr) return CallbackReturn::SUCCESS;

    /* 安全失能序列（安全帧 → 等 2 周期 → STOP_MOTOR → 等 IDLE）。**幂等**：
       重复调用安全，且不会把已失能的关节"再动一下"。 */
    jr::Result r;
    if (tick_source_ == TickSource::kInternal) {
        if (tg_->pause(&r) != jr::Status::kOk) {
            RCLCPP_ERROR(logger_, "pause before disable failed: %s", r.message);
            return CallbackReturn::ERROR;
        }
    }
    const jr::Status st = tg_->deactivate(&r);
    if (tick_source_ == TickSource::kInternal) tg_->resume();

    active_ = false;
    if (st != jr::Status::kOk) {
        RCLCPP_ERROR(logger_, "safe disable reported a problem: %s (joints may still be enabled - "
                              "check the device state)",
                     r.message);
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

JrSystemInterface::CallbackReturn
JrSystemInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
    if (tg_ == nullptr) return CallbackReturn::SUCCESS;
    tg_->stop();
    jr::Result r;
    tg_->close_buses(cfg_.safety.on_exit, &r);
    tg_.reset();
    primed_ = false;
    active_ = false;
    return CallbackReturn::SUCCESS;
}

JrSystemInterface::CallbackReturn JrSystemInterface::on_error(const rclcpp_lifecycle::State &)
{
    /* 出错时唯一正确的动作是**尽量把关节放到安全状态**，而不是继续尝试控制。 */
    if (tg_ == nullptr) return CallbackReturn::SUCCESS;
    jr::Result r;
    if (tick_source_ == TickSource::kInternal) {
        if (tg_->pause(&r) != jr::Status::kOk) {
            RCLCPP_ERROR(logger_, "on_error: cannot pause the tick group: %s", r.message);
            return CallbackReturn::ERROR;
        }
    }
    const jr::Status st = tg_->deactivate(&r);
    if (tick_source_ == TickSource::kInternal) tg_->resume();
    active_ = false;
    if (st != jr::Status::kOk) {
        RCLCPP_ERROR(logger_, "on_error: safe disable failed: %s", r.message);
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

/* ==========================================================================
 * ④ 周期读写
 * ======================================================================== */

JrSystemInterface::return_type JrSystemInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (tg_ == nullptr || !primed_) return return_type::ERROR;

    /* `internal` 模式：这里**只取信箱**（常量时间、不做 I/O）—— 这正是"CAN 周期与
       controller_manager 周期解耦"的实现方式。
       `controller_manager` 模式：状态来自上一个 write() 里跑的那一拍（延迟 1 个 CM 周期）。 */
    const jr::StateSnapshot *s = tg_->acquire_snapshot();
    if (s == nullptr) {
        /* 正常不会发生（configure 里已 prime）；真发生也不能把 0 当成真实位置报出去。 */
        RCLCPP_ERROR_ONCE(logger_,
                          "no snapshot available yet: states keep their previous values");
        return return_type::OK;
    }

    for (std::size_t i = 0u; i < joint_index_.size(); ++i) {
        const jr::JointStatePOD &js = s->joints[joint_index_[i]];
        state_position_[i] = js.position;
        state_velocity_[i] = js.velocity;
        state_effort_[i] = js.effort;
        if (!js.online) {
            /* 掉线/陈旧**不**在这里自己动关节（§8.3：只如实标记，动作由安全策略决定）；
               但必须让客户在日志里看到，而不是悄悄把旧值当新值。
               （用 ONCE 而不是按频率节流：位置越界/抖动的日志本身就不该淹掉控制日志；
                 持续可见的实情在 WP2 的诊断里。） */
            RCLCPP_WARN_ONCE(logger_,
                             "joint '%s' is offline (age=%u ms, mode_state=%u, err=0x%x): reported "
                             "values are the last known ones",
                             joint_name_[joint_index_[i]].c_str(), js.age_ms, js.mode_state,
                             js.err_code);
        }
    }
    return return_type::OK;
}

JrSystemInterface::return_type JrSystemInterface::write(const rclcpp::Time &,
                                                        const rclcpp::Duration &)
{
    if (tg_ == nullptr || !primed_) return return_type::ERROR;

    jr::CommandSet c;
    c.seq = ++cmd_seq_;
    c.stamp_ns = jr::rt::now_ns();
    for (std::size_t i = 0u; i < joint_index_.size(); ++i) {
        const unsigned g = joint_index_[i];
        jr::JointTarget &t = c.joint[g];
        /* 按该关节的模式填目标（模式在 `map_joints()` 里从配置读出，与设备使能时的一致）。
           非 MIT 模式**不动增益**：帧里没有它们的位置。 */
        const jr::CmdMode m = (i < joint_mode_.size()) ? static_cast<jr::CmdMode>(joint_mode_[i])
                                                       : jr::CmdMode::kMit;
        switch (m) {
        case jr::CmdMode::kCsp:
            t.mode = m;
            t.position = cmd_position_[i];
            break;
        case jr::CmdMode::kCsv:
            t.mode = m;
            t.velocity = cmd_velocity_[i];
            break;
        case jr::CmdMode::kCst:
            t.mode = m;
            t.torque = cmd_effort_[i];
            break;
        case jr::CmdMode::kMit:
        default:
            t.mode = jr::CmdMode::kMit;
            t.position = cmd_position_[i];
            t.velocity = cmd_velocity_[i];
            t.torque = cmd_effort_[i];
            if (gain_si_) {
                t.stiffness = cmd_gain_a_[i];
                t.damping = cmd_gain_b_[i];
                t.si_gain = true;
            } else {
                t.kp = cmd_gain_a_[i];
                t.kd = cmd_gain_b_[i];
                t.si_gain = false;
            }
            break;
        }
        c.set_mask(g);
    }

    /* 覆盖式信箱（ADR-2）：不排队、不积压；被覆盖的次数在 RtStats 里可见。 */
    tg_->submit_command(c);

    if (tick_source_ == TickSource::kControllerManager) {
        /* 外部模式：命令提交后**立刻跑这一拍**，所以延迟比 internal 模式更小
           （internal 模式最多晚一个 tick）。 */
        jr::Result r;
        if (tg_->step(&r) != jr::Status::kOk) {
            RCLCPP_ERROR_ONCE(logger_, "cycle failed: %s", r.message);
            return return_type::ERROR;
        }
    }
    return return_type::OK;
}

}  // namespace jr_ros2_control

/* pluginlib 导出：与 jr_ros2_control.xml 里的 type 必须一致。 */
PLUGINLIB_EXPORT_CLASS(jr_ros2_control::JrSystemInterface, hardware_interface::SystemInterface)
