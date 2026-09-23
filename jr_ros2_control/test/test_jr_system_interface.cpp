/**
 * @file    test_jr_system_interface.cpp
 * @brief   WP3 组件测试：**不起 controller_manager** 地钉住组件的契约
 *
 * 覆盖四件事（都是客户会踩的）：
 *  ① 接口集合与**单位模式**（wire 的 kp/kd vs SI 的 stiffness/damping 二选一导出）；
 *  ② URDF ↔ 配置的映射（未知关节名必须报错并列出可用名字）；
 *  ③ 两种 tick 来源（internal / controller_manager）都能跑通完整生命周期；
 *  ④ 安全落点：`on_deactivate` 之后**不得**再发出控制帧。
 *
 * 用 `controller_manager` 模式做绝大部分检查：每一拍由测试自己 `step()`，
 * 因而**完全确定性**（不受主机抢占影响），这对"安全断言"尤其重要。
 */

#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "jr_ros2_control/jr_system_interface.hpp"
#include "jr_test.hpp"

using jr_ros2_control::JrSystemInterface;

namespace {

/** 虚拟总线 + 2 关节；`rt.enabled=false`（测试机不做实时设置）。 */
const char *kCfgYaml = R"YAML(
jr:
  rt:
    enabled: false
  tick_groups:
    - {name: g0, rate_hz: 1000, buses: [vbus]}
  buses:
    - name: vbus
      type: virtual
      spec: "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd;1:id=2,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd"
      is_fd: true
      master_id: 1
      joints: [j0, j1]
  joints:
    - {name: j0, bus: vbus, node_id: 1, mode: @MODE0@}
    - {name: j1, bus: vbus, node_id: 2, mode: @MODE1@}
  feedback:
    source: broadcast_plus_heartbeat
    heartbeat_ms: 5
  descriptor:
    cache_enabled: false
  bus_lock:
    enabled: false
)YAML";

const char *kCfgPath = "jr_test_wp3_config.yaml";

bool write_cfg(const char *path, const char *text)
{
    std::FILE *f = std::fopen(path, "wb");
    if (f == nullptr) return false;
    std::fputs(text, f);
    std::fclose(f);
    return true;
}

void replace_all(std::string *s, const char *from, const char *to)
{
    const std::string f = from;
    const std::string t = to;
    for (std::size_t p = s->find(f); p != std::string::npos; p = s->find(f, p + t.size())) {
        s->replace(p, f.size(), t);
    }
}

/** 写一份配置（可指定每个关节的模式）。模式是**配置项**，所以用例必须先落成文件。 */
bool write_cfg_modes(const char *path, const char *mode0, const char *mode1)
{
    std::string text = kCfgYaml;
    replace_all(&text, "@MODE0@", mode0);
    replace_all(&text, "@MODE1@", mode1);
    return write_cfg(path, text.c_str());
}

/** 按发行版构造 on_init 的参数：**直接用兼容层那一份**（测试不再自己写 `#if`，
 *  否则测试可能替错误的分支背书）。 */
jr_ros2_control::compat::OnInitParams make_params(const hardware_interface::HardwareInfo &info)
{
    return jr_ros2_control::compat::make_params(info);
}

hardware_interface::ComponentInfo make_joint(const char *name)
{
    hardware_interface::ComponentInfo j;
    j.name = name;
    /* URDF 里声明的接口（控制器能不能 claim 就靠它）：
       position/velocity/effort + 两个 MIT 增益（名字随 gain_mode 变）。 */
    for (const char *iface : {"position", "velocity", "effort", "kp", "kd", "stiffness", "damping"}) {
        hardware_interface::InterfaceInfo ii;
        ii.name = iface;
        j.command_interfaces.push_back(ii);
        j.state_interfaces.push_back(ii);
    }
    return j;
}

hardware_interface::HardwareInfo make_info(const char *config_file, const char *tick_source,
                                           const char *gain_mode, const char *joint0 = "j0",
                                           const char *joint1 = "j1")
{
    hardware_interface::HardwareInfo info;
    info.name = "jr_vbus";
    info.type = "system";
    if (config_file != nullptr) info.hardware_parameters["config_file"] = config_file;
    if (tick_source != nullptr) info.hardware_parameters["tick_source"] = tick_source;
    if (gain_mode != nullptr) info.hardware_parameters["gain_mode"] = gain_mode;
    info.joints.push_back(make_joint(joint0));
    info.joints.push_back(make_joint(joint1));
    return info;
}

void test_interface_contract()
{
    JR_CASE("接口集合：position/velocity/effort + **只导出一种**增益（单位二选一）");

    JR_CHECK(write_cfg_modes(kCfgPath, "mit", "mit"));

    /* ---- 默认（wire）：kp / kd ---- */
    {
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, nullptr, nullptr);
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::SUCCESS);
        const auto states = hw.export_state_interfaces();
        const auto cmds = hw.export_command_interfaces();
        JR_CHECK_EQ(states.size(), 6u);   /* 2 关节 × 3 */
        JR_CHECK_EQ(cmds.size(), 10u);    /* 2 关节 × (3 + 2 增益) */
        JR_CHECK_EQ(hw.exported_gain_interfaces(), 4u);
        JR_CHECK_EQ(std::strcmp(hw.gain_interface_names(), "kp,kd"), 0);

        /* 名字与绑定要对得上：逐个查一遍（防止"少导出一个接口"这类静默错误）。
           ⚠ `get_name()` 在各发行版里形式不同（Jazzy 带前缀），所以：
             - `get_interface_name()` **精确**比对（契约就是它）；
             - `get_name()` 只要求**包含**关节名。
           把实际名字打出来，出问题时不用再猜。 */
        static const char *want[] = {"position", "velocity", "effort", "kp", "kd"};
        unsigned k = 0u;
        for (const auto &c : cmds) {
            const std::string jn = c.get_name();
            const std::string want_joint = (k < 5u) ? "j0" : "j1";
            std::printf("    cmd[%u] name='%s' iface='%s'\n", k, jn.c_str(),
                        c.get_interface_name().c_str());
            JR_CHECK_MSG(jn.find(want_joint) != std::string::npos,
                         "the handle name should contain the joint name");
            JR_CHECK_EQ(std::strcmp(c.get_interface_name().c_str(), want[k % 5u]), 0);
            ++k;
        }
    }

    /* ---- SI：stiffness / damping（**不再**导出 kp/kd —— 单位搞错比 claim 失败危险） ---- */
    {
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, nullptr, "si");
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::SUCCESS);
        /* ⚠ 顺序有讲究：这两个访问器描述的是**导出结果**，导出之前它没有意义。
           `on_init` 现在只把它清空、不预填 —— 以前预填过，于是"纯 CSP 配置"的启动
           日志里也会出现 "kp,kd"（客户照着写 URDF → controller_manager 拒初始化）。 */
        JR_CHECK_EQ(std::strcmp(hw.gain_interface_names(), ""), 0);
        const auto cmds = hw.export_command_interfaces();
        JR_CHECK_EQ(std::strcmp(hw.gain_interface_names(), "stiffness,damping"), 0);
        JR_CHECK_EQ(cmds.size(), 10u);
        bool has_kp = false;
        bool has_stiffness = false;
        for (const auto &c : cmds) {
            const std::string n = c.get_interface_name();
            if (n == "kp" || n == "kd") has_kp = true;
            if (n == "stiffness" || n == "damping") has_stiffness = true;
        }
        JR_CHECK_EQ(has_kp, false);
        JR_CHECK_EQ(has_stiffness, true);
    }
}

void test_config_errors()
{
    JR_CASE("配置错误必须报错且可操作（缺 config_file / 未知 tick_source / 未知关节）");

    JR_CHECK(write_cfg_modes(kCfgPath, "mit", "mit"));

    /* 缺 config_file */
    {
        JrSystemInterface hw;
        const auto info = make_info(nullptr, nullptr, nullptr);
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::ERROR);
    }
    /* 未知 tick_source：必须列合法取值（客户拼错时要能自己改） */
    {
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, "cm", nullptr);
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::ERROR);
    }
    /* 未知 gain_mode */
    {
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, nullptr, "wire_plus");
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::ERROR);
    }
    /* URDF 里的关节名在配置里不存在 → 必须失败（否则控制的是空气，还看不出来） */
    {
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, nullptr, nullptr, "j0", "not_a_joint");
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::ERROR);
    }
    /* 配置文件不存在 */
    {
        JrSystemInterface hw;
        const auto info = make_info("no_such_wp3_config.yaml", nullptr, nullptr);
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::ERROR);
    }
}

/** 等外部模式下的第 n 拍（测试自己驱动，所以这里是确定性的）。 */
void pump(JrSystemInterface &hw, unsigned cycles)
{
    for (unsigned i = 0u; i < cycles; ++i) {
        hw.read(rclcpp::Time(0), rclcpp::Duration(0, 0u));
        hw.write(rclcpp::Time(0), rclcpp::Duration(0, 0u));
    }
}

void test_external_tick_lifecycle()
{
    JR_CASE("tick_source=controller_manager：生命周期 + 命令真的上线 + 失能后不再发控制帧");

    JR_CHECK(write_cfg_modes(kCfgPath, "mit", "mit"));

    JrSystemInterface hw;
    const auto info = make_info(kCfgPath, "controller_manager", nullptr);
    JR_CHECK_MSG(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::SUCCESS, "on_init failed");

    JR_CHECK_MSG(hw.on_configure(rclcpp_lifecycle::State()) ==
                     JrSystemInterface::CallbackReturn::SUCCESS,
                 "on_configure failed");
    JR_CHECK(hw.tick_group_ready());
    JR_CHECK(hw.tick_group().bus(0).mode() == jr::rt::BusMode::kReady);

    /* 外部模式下 pause 不被支持（要用生命周期切换做阻塞调用）—— 组件不该偷偷绕开它 */
    {
        jr::Result r;
        JR_CHECK(hw.tick_group().pause(&r) == jr::Status::kNotSupported);
    }

    /* 使能：冷启动后关节应是**失能**的（G6），on_activate 才使能 */
    {
        const jr::StateSnapshot *s = hw.tick_group().acquire_snapshot();
        JR_CHECK(s != nullptr);
        if (s != nullptr) JR_CHECK_EQ(s->joints[0].enabled, false);
    }
    JR_CHECK_MSG(hw.on_activate(rclcpp_lifecycle::State()) ==
                     JrSystemInterface::CallbackReturn::SUCCESS,
                 "on_activate failed");
    /* ⚠ 时序：`enabled` 是**设备反馈**里的位 —— `activate()` 返回只说明使能握手完成，
       快照要等**跑过周期**才会反映出来（这里 pump 会顺带下发命令）。 */
    hw.set_command_for_test(0u, 0.10, 0.0, 0.0, 5.0, 0.5);
    hw.set_command_for_test(1u, -0.10, 0.0, 0.0, 5.0, 0.5);
    pump(hw, 3u);
    JR_CHECK(hw.tick_group().bus(0).mode() == jr::rt::BusMode::kActive);
    {
        const jr::StateSnapshot *s = hw.tick_group().acquire_snapshot();
        JR_CHECK(s != nullptr);
        if (s != nullptr) JR_CHECK_EQ(s->joints[0].enabled, true);
    }

    /* ---- 状态映射：read() 之后，组件里的状态必须就是快照里的值（映射错位最容易静默出错） ---- */
    hw.read(rclcpp::Time(0), rclcpp::Duration(0, 0u));
    {
        const jr::StateSnapshot *s = hw.tick_group().acquire_snapshot();
        JR_CHECK(s != nullptr);
        if (s != nullptr) {
            JR_CHECK_EQ(hw.state_position(0u), s->joints[0].position);
            JR_CHECK_EQ(hw.state_position(1u), s->joints[1].position);
            JR_CHECK_EQ(hw.state_effort(0u), s->joints[0].effort);
        }
    }

    /* ---- 命令真的上线：write() 会把命令提交并在外部模式下立刻跑一拍 ---- */
    const std::uint32_t tx_before = hw.tick_group().acquire_snapshot()->buses[0].tx_frames;
    hw.set_command_for_test(0u, 0.20, 0.0, 0.0, 5.0, 0.5);
    hw.set_command_for_test(1u, -0.15, 0.0, 0.0, 5.0, 0.5);
    pump(hw, 3u);
    const std::uint32_t tx_after = hw.tick_group().acquire_snapshot()->buses[0].tx_frames;
    std::printf("  tx_frames %u -> %u (3 cycles)\n", tx_before, tx_after);
    JR_CHECK(tx_after > tx_before);

    /* ---- 安全落点：失能之后不得再发控制帧（这是"别让机器人动"的最后一道闸） ---- */
    JR_CHECK_MSG(hw.on_deactivate(rclcpp_lifecycle::State()) ==
                     JrSystemInterface::CallbackReturn::SUCCESS,
                 "on_deactivate failed");
    pump(hw, 2u);   /* 同上的时序：先让周期把设备的失能状态带回来，再开始计帧 */
    {
        const jr::StateSnapshot *s = hw.tick_group().acquire_snapshot();
        JR_CHECK(s != nullptr);
        if (s != nullptr) JR_CHECK_EQ(s->joints[0].enabled, false);
    }
    const std::uint32_t tx_after_disable = hw.tick_group().acquire_snapshot()->buses[0].tx_frames;
    hw.set_command_for_test(0u, 0.60, 0.0, 0.0, 5.0, 0.5);   /* 再来一条"会动"的命令 */
    pump(hw, 5u);
    const std::uint32_t tx_final = hw.tick_group().acquire_snapshot()->buses[0].tx_frames;
    std::printf("  tx after disable: %u -> %u (5 cycles)\n", tx_after_disable, tx_final);
    JR_CHECK_EQ(tx_final, tx_after_disable);

    /* 幂等：重复失能 / 关闭都不能崩，也不能"再动一下" */
    JR_CHECK(hw.on_deactivate(rclcpp_lifecycle::State()) ==
             JrSystemInterface::CallbackReturn::SUCCESS);
    JR_CHECK(hw.on_shutdown(rclcpp_lifecycle::State()) ==
             JrSystemInterface::CallbackReturn::SUCCESS);
    JR_CHECK(hw.on_shutdown(rclcpp_lifecycle::State()) ==
             JrSystemInterface::CallbackReturn::SUCCESS);
}

/** 按模式导出命令接口：这是"CSP/CSV/CST 客户"直接感受到的东西。
 *
 * 为什么必须这么测：客户会拿 URDF 里的接口名去 claim；导出多了 = "写了没用"，
 * 导出少了 = claim 失败。两种都是现场难查的问题，所以逐个模式钉死接口集合。 */
void test_per_mode_interfaces()
{
    JR_CASE("命令接口按 joints[].mode 导出：CSP→position / CSV→velocity / CST→effort（CURRENT 拒绝）");

    const auto find_iface = [](const std::vector<hardware_interface::CommandInterface> &v,
                               const char *joint, const char *iface) {
        for (const auto &c : v) {
            if (c.get_name().find(joint) != std::string::npos &&
                std::strcmp(c.get_interface_name().c_str(), iface) == 0) {
                return true;
            }
        }
        return false;
    };
    const auto count_iface = [](const std::vector<hardware_interface::CommandInterface> &v,
                                const char *joint) {
        unsigned n = 0u;
        for (const auto &c : v) {
            if (c.get_name().find(joint) != std::string::npos) ++n;
        }
        return n;
    };

    /* ---- ① CSP：j0 只导出 position；j1（MIT）仍是三件套 + 增益 ---- */
    {
        JR_CHECK(write_cfg_modes(kCfgPath, "csp", "mit"));
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, "controller_manager", nullptr);
        JR_CHECK_MSG(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::SUCCESS,
                     "CSP 关节必须能通过 on_init（它不是 CURRENT）");
        const auto cmds = hw.export_command_interfaces();
        JR_CHECK_EQ(cmds.size(), 6u);   /* j0: position 一个 + j1: 3 + 2 增益 */
        JR_CHECK_EQ(count_iface(cmds, "j0"), 1u);
        JR_CHECK_MSG(find_iface(cmds, "j0", "position"), "CSP 关节必须导出 position");
        JR_CHECK_MSG(!find_iface(cmds, "j0", "velocity"), "CSP 关节不得导出 velocity");
        JR_CHECK_MSG(!find_iface(cmds, "j0", "effort"), "CSP 关节不得导出 effort");
        JR_CHECK_MSG(!find_iface(cmds, "j0", "kp"), "CSP 帧里没有增益 ⇒ 不得导出 kp/kd");
        JR_CHECK_EQ(hw.exported_gain_interfaces(), 2u);   /* 只有 j1 那一对 */
        JR_CHECK_EQ(std::strcmp(hw.gain_interface_names(), "kp,kd"), 0);

        /* 线上行为：CSP 目标真的让关节动（外部模式每拍 step，确定性） */
        JR_CHECK(hw.on_configure(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        JR_CHECK(hw.on_activate(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        const double start = hw.state_position(0u);
        hw.set_command_for_test(0u, 0.05, 0.0, 0.0, 0.0, 0.0);   /* CSP：只有 position 有意义 */
        pump(hw, 400u);
        hw.read(rclcpp::Time(0), rclcpp::Duration(0, 0u));
        std::printf("  csp: position %.5f -> %.5f (target 0.05)\n", start, hw.state_position(0u));
        /* 阈值取目标的 90%：只断言"动了"太弱（发错帧型也可能动一点），
           而虚设备的 POS 模型是收敛型（v = 2·err），400 拍足够到 95% 以上。 */
        JR_CHECK_MSG(hw.state_position(0u) > 0.045,
                     "CSP：position 命令必须真的把关节控到目标附近（线上是 POS_CONTROL 帧）");
        JR_CHECK(hw.on_deactivate(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        JR_CHECK(hw.on_shutdown(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
    }

    /* ---- ② CSV：只导出 velocity ---- */
    {
        JR_CHECK(write_cfg_modes(kCfgPath, "csv", "mit"));
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, "controller_manager", nullptr);
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::SUCCESS);
        const auto cmds = hw.export_command_interfaces();
        JR_CHECK_EQ(count_iface(cmds, "j0"), 1u);
        JR_CHECK_MSG(find_iface(cmds, "j0", "velocity"), "CSV 关节必须导出 velocity");
        JR_CHECK_MSG(!find_iface(cmds, "j0", "position"), "CSV 帧里没有位置 ⇒ 不得导出 position");

        JR_CHECK(hw.on_configure(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        JR_CHECK(hw.on_activate(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        hw.set_command_for_test(0u, 0.0, 2.0, 0.0, 0.0, 0.0);
        pump(hw, 5u);
        hw.read(rclcpp::Time(0), rclcpp::Duration(0, 0u));
        std::printf("  csv: velocity=%.4f (target 2.0)\n", hw.state_velocity(0u));
        JR_CHECK_MSG(std::fabs(hw.state_velocity(0u) - 2.0) < 0.2,
                     "CSV：速度命令必须原样落到设备（模型直接给速度 ⇒ 这是量值断言）");
        JR_CHECK(hw.on_deactivate(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        JR_CHECK(hw.on_shutdown(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
    }

    /* ---- ③ CST：只导出 effort；力矩必须体现到反馈上（符号也要对） ---- */
    {
        JR_CHECK(write_cfg_modes(kCfgPath, "cst", "mit"));
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, "controller_manager", nullptr);
        JR_CHECK(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::SUCCESS);
        const auto cmds = hw.export_command_interfaces();
        JR_CHECK_EQ(count_iface(cmds, "j0"), 1u);
        JR_CHECK_MSG(find_iface(cmds, "j0", "effort"), "CST 关节必须导出 effort");
        JR_CHECK_MSG(!find_iface(cmds, "j0", "position"), "CST 帧里没有位置 ⇒ 不得导出 position");

        JR_CHECK(hw.on_configure(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        JR_CHECK(hw.on_activate(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        hw.set_command_for_test(0u, 0.0, 0.0, 5.0, 0.0, 0.0);
        pump(hw, 5u);
        hw.read(rclcpp::Time(0), rclcpp::Duration(0, 0u));
        const double e_pos = hw.state_effort(0u);
        hw.set_command_for_test(0u, 0.0, 0.0, -5.0, 0.0, 0.0);
        pump(hw, 5u);
        hw.read(rclcpp::Time(0), rclcpp::Duration(0, 0u));
        const double e_neg = hw.state_effort(0u);
        std::printf("  cst: effort(+5)=%.4f effort(-5)=%.4f\n", e_pos, e_neg);
        JR_CHECK_MSG(e_pos > 0.0 && e_neg < 0.0,
                     "CST：±5 N·m 的力矩必须体现在反馈里且方向不能反（虚设备把它呈现为反馈力矩/电流）");
        JR_CHECK(hw.on_deactivate(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
        JR_CHECK(hw.on_shutdown(rclcpp_lifecycle::State()) ==
                 JrSystemInterface::CallbackReturn::SUCCESS);
    }

    /* ---- ④ CURRENT：必须**拒绝**并说清为什么（effort 的语义会变） ---- */
    {
        JR_CHECK(write_cfg_modes(kCfgPath, "current", "mit"));
        JrSystemInterface hw;
        const auto info = make_info(kCfgPath, "controller_manager", nullptr);
        JR_CHECK_MSG(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::ERROR,
                     "CURRENT 关节必须被拒绝：ros2_control 的 effort 是 N·m，而 CURRENT 帧是电机端 A");
    }
}

void test_internal_tick_lifecycle()
{
    JR_CASE("tick_source=internal（默认）：内部 RT 线程 + 暂停/使能/恢复路径");

    JR_CHECK(write_cfg_modes(kCfgPath, "mit", "mit"));

    JrSystemInterface hw;
    const auto info = make_info(kCfgPath, "internal", nullptr);
    JR_CHECK_MSG(hw.on_init(make_params(info)) == JrSystemInterface::CallbackReturn::SUCCESS, "on_init failed");
    JR_CHECK_MSG(hw.on_configure(rclcpp_lifecycle::State()) ==
                     JrSystemInterface::CallbackReturn::SUCCESS,
                 "on_configure failed (internal tick thread did not produce a snapshot?)");
    JR_CHECK(hw.tick_group().external_tick() == false);

    /* 内部模式下 read/write 只是信箱交换；即使还没使能也不该报错 */
    for (unsigned i = 0u; i < 3u; ++i) {
        JR_CHECK(hw.read(rclcpp::Time(0), rclcpp::Duration(0, 0u)) ==
                 JrSystemInterface::return_type::OK);
        JR_CHECK(hw.write(rclcpp::Time(0), rclcpp::Duration(0, 0u)) ==
                 JrSystemInterface::return_type::OK);
    }

    JR_CHECK_MSG(hw.on_activate(rclcpp_lifecycle::State()) ==
                     JrSystemInterface::CallbackReturn::SUCCESS,
                 "on_activate failed (pause/activate/resume path)");
    JR_CHECK_MSG(hw.on_deactivate(rclcpp_lifecycle::State()) ==
                     JrSystemInterface::CallbackReturn::SUCCESS,
                 "on_deactivate failed");
    JR_CHECK(hw.on_shutdown(rclcpp_lifecycle::State()) ==
             JrSystemInterface::CallbackReturn::SUCCESS);
}

}  // namespace

int main()
{
    test_interface_contract();
    test_config_errors();
    test_external_tick_lifecycle();
    test_per_mode_interfaces();
    test_internal_tick_lifecycle();
    std::remove(kCfgPath);
    return jrtest::report();
}
