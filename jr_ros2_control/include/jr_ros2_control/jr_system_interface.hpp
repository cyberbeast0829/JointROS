/**
 * @file    jr_system_interface.hpp
 * @brief   ros2_control `SystemInterface`：CyberBeast 关节模组（MIT 控制）
 *
 * @par 职责边界（很重要）
 *  实时调度、CAN 收发、协议、描述符、安全失能序列全在 `jr_core` + JointSDK 里；本类只做
 *  **三件事**：① 把配置（URDF 里给的 YAML 路径）交给 `jr_core`；② 把 ros2_control 的
 *  状态/命令接口和 `jr_core` 的快照/信箱对接；③ 在生命周期切换点上做安全动作。
 *  任何"顺手也做点协议"的冲动都要拦回去（那会变成第二份实现，迟早和 jr_core 漂移）。
 *
 * @par 两种 tick 来源（ADR-6）
 *  - `internal`（默认）：`jr_core` 内部起 RT tick 线程。`read()/write()` 只做**信箱交换**
 *    （常量时间、不做 I/O），CAN 周期与 controller_manager 周期解耦（CM 100~500 Hz，
 *    CAN 可 1 kHz）。`write()` 里的命令最多晚 1 个 tick 生效。
 *  - `controller_manager`：由 CM 的 update 线程驱动 `jr_core` 的 `step()`。
 *    时序单一（好调试、好理解），代价是 CAN 周期 = CM 周期且抖动取决于控制器链。
 *
 * @par 接口契约（**按 `joints[].mode` 导出，接口集随模式变**）
 *  状态（所有模式相同）：`position` / `velocity` / `effort`（输出端 SI：rad、rad/s、N·m）。
 *  命令：
 *    - `mit`（默认）：`position` / `velocity` / `effort` + **两组增益**（二选一，见下）——
 *      MIT 帧里只有它带增益，控制律就在这两组增益上。
 *    - `csp`：只有 `position`（CSP 帧携带位置 + 速度/电流**限值**，没有增益）
 *    - `csv`：只有 `velocity`
 *    - `cst`：只有 `effort`
 *    - `current`：**不支持**（见 `map_joints()` 的拒绝理由：`effort` 的单位会从 N·m 变成
 *      电机端 A，等于在接口里撒谎；真要做得另起一个非标准接口名）
 *  ⚠ 非 MIT 模式**不导出增益接口**：帧里没有它们的位置 ⇒ 导出了就是“写了没用”。
 *  ⚠ 同一个控制器不能混模式：JTC 会同时 claim 它所有关节的 `position`+`velocity`，
 *    而 CSP 关节只导出 `position` ⇒ 混模式必须拆成多个控制器。
 *
 * @par 两组增益**只导出一组**（仅 MIT）
 *   `gain_mode=wire`（默认）导出 `kp`/`kd`（**线上值**，与 SDK 的 MIT 帧字段一致）；
 *   `gain_mode=si` 导出 `stiffness`/`damping`（N·m/rad、N·m·s/rad，由 SDK 换算）。
 *  ⚠ 两组**不会同时导出**：单位搞错是最难查的一类问题（ADR-5），宁可让控制器在
 *    claim 阶段就失败，也不要"看起来能动、其实刚度差了 gear_ratio² 倍"（见 UNITS 文档）。
 *
 * @par 安全
 *  - `on_activate` 才会使能关节（冷启动不自动使能，G6）；
 *  - `on_deactivate` 走 SDK 的安全失能序列（安全帧 → 等 2 周期 → STOP_MOTOR → 等 IDLE）；
 *  - `on_shutdown`/析构按 `safety.on_exit_action` 收尾（默认 disable，不会留下抱力关节）；
 *  - 未标定（`unit_scale.valid == 0`）时 `on_activate` **拒绝**（`require_calibrated`），
 *    并提示用 `calibrate`/`set_zero_here`（WP5 的服务）。
 */

#ifndef JR_ROS2_CONTROL_JR_SYSTEM_INTERFACE_HPP
#define JR_ROS2_CONTROL_JR_SYSTEM_INTERFACE_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "jr_ros2/jr_command.hpp"
#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_snapshot.hpp"
#include "jr_ros2/rt/jr_rt_sched.hpp"
#include "jr_ros2/rt/jr_tick_group.hpp"

#include "jr_ros2_control/compat/distro_compat.hpp"

namespace jr_ros2_control {

class JrSystemInterface : public hardware_interface::SystemInterface
{
public:
    using CallbackReturn = compat::CallbackReturn;
    using return_type = compat::return_type;

    JrSystemInterface() = default;
    ~JrSystemInterface() override;

    /** ⚠ 签名随发行版变化：Humble/Jazzy 是 `const HardwareInfo &`，Lyrical 起只剩
     *  `const HardwareComponentInterfaceParams &`。两边都由 `compat::OnInitParams`
     *  在编译期选中（见 `compat/distro_compat.hpp` 与 CMake 的编译探测），
     *  业务逻辑一行不动。 */
    CallbackReturn on_init(const compat::OnInitParams &params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &previous) override;

    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /* ---- 测试/诊断用只读查询（不改行为） ---- */
    const jr::Config &config() const noexcept { return cfg_; }
    /** ⚠ 非 const 重载只给**测试/诊断**用（要驱动 `step()`、`pause()`）；
        生产路径永远是 controller_manager 通过 read()/write() 驱动。 */
    jr::rt::TickGroup &tick_group() noexcept { return *tg_; }
    const jr::rt::TickGroup &tick_group() const noexcept { return *tg_; }
    bool tick_group_ready() const noexcept { return tg_ != nullptr; }
    unsigned exported_gain_interfaces() const noexcept { return gain_iface_count_; }
    const char *gain_interface_names() const noexcept { return gain_iface_names_; }
    double state_position(unsigned i) const noexcept { return state_position_[i]; }
    double state_velocity(unsigned i) const noexcept { return state_velocity_[i]; }
    double state_effort(unsigned i) const noexcept { return state_effort_[i]; }

    /** ⚠ **仅测试/诊断**：直接写命令数组。
     *
     *  正常路径是控制器写导出的命令接口（本类只读回它们）。存在这个口子的理由：
     *  组件的行为（URDF↔配置映射、两种 tick 来源、生命周期与安全落点）必须能在
     *  **不起 controller_manager** 的情况下被单测 —— 否则每个用例都要拖一整套
     *  CM + JTC + URDF，慢且难定位。 */
    void set_command_for_test(unsigned i, double position, double velocity, double effort,
                              double gain_a, double gain_b) noexcept;

private:
    enum class TickSource : std::uint8_t { kInternal = 0u, kControllerManager };

    /** 把 URDF 关节名映射到 `jr::Config` 里的**全局关节下标**；未知则报错。 */
    bool map_joints(std::string *err);

    /** 全局关节下标 → (总线下标, 总线内下标)。返回 false = 下标越界（正常不该发生）。 */
    bool locate_joint(unsigned global, unsigned *bus, unsigned *local) const noexcept;

    /** 给"不写 kp/kd 的控制器"准备**默认 MIT 增益**（使能后有效）。
     *
     *  为什么必须有：`JointTrajectoryController` 只写 `position`/`velocity`，
     *  **不会**碰 `kp`/`kd`。如果我们按 0 下发，MIT 帧里 kp=kd=0 ⇒ 关节是**自由状态**，
     *  表面现象是"轨迹命令下去了但关节不动/不跟"（很难查）。
     *
     *  取值来源：YAML 的 `limits.<joint>.stiffness/damping`（SI，N·m/rad）。
     *   `gain_mode=si` → 直接用；`gain_mode=wire` → 按 `kp = K·2π/gear_ratio` 换算
     *   （与 `BusRuntime::hold_all` 同一个公式，避免两套换算）。
     *  并按**设备量程** `mit_max_kp/kd` 夹紧（否则 SDK 会因越界拒绝目标），
     *  夹紧时日志里说清楚。 */
    void apply_default_gains();
    /** 配置阶段共用：读 YAML → 建 TickGroup → open → configure → 起 tick → 等首份快照。 */
    CallbackReturn configure_backend();
    /** 内部模式：暂停 → 执行 `fn` → 恢复（SDK 禁止在使能期间跨线程碰 context）。 */
    CallbackReturn with_paused(const char *what, bool require_calibrated);
    /** 等首份快照（内部模式靠 tick 线程产生；外部模式先手动跑一拍）。 */
    bool prime_snapshot(std::string *err);

    jr::Config  cfg_ = {};
    TickSource  tick_source_ = TickSource::kInternal;
    bool        gain_si_ = false;      /**< true = 导出 stiffness/damping（SI） */
    std::string config_file_;

    std::unique_ptr<jr::rt::TickGroup> tg_;

    /** URDF 关节顺序 → 配置里的全局关节下标。 */
    std::vector<unsigned> joint_index_;
    /** URDF 关节顺序 → 该关节使能时的模式（`jr::CmdMode` 取值）。
     *
     *  它决定**导出哪些命令接口**（CSP→只有 position，CSV→只有 velocity，CST→只有 effort，
     *  MIT→三件套 + 增益），也决定 `write()` 里填哪种目标。在 `map_joints()` 里填。 */
    std::vector<std::uint8_t> joint_mode_;
    /** 配置的全局关节下标 → 关节名（报错时能说清是哪个）。 */
    std::vector<std::string> joint_name_;

    /* ros2_control 直接读写的数组（生命周期内地址必须稳定）。 */
    std::vector<double> state_position_;
    std::vector<double> state_velocity_;
    std::vector<double> state_effort_;
    std::vector<double> cmd_position_;
    std::vector<double> cmd_velocity_;
    std::vector<double> cmd_effort_;
    std::vector<double> cmd_gain_a_;   /**< kp 或 stiffness */
    std::vector<double> cmd_gain_b_;   /**< kd 或 damping */

    std::uint64_t cmd_seq_ = 0u;
    bool          primed_ = false;
    bool          active_ = false;
    unsigned      gain_iface_count_ = 0u;
    char          gain_iface_names_[64] = {};

    rclcpp::Logger logger_ = rclcpp::get_logger("jr_ros2_control");
};

}  // namespace jr_ros2_control

#endif /* JR_ROS2_CONTROL_JR_SYSTEM_INTERFACE_HPP */
