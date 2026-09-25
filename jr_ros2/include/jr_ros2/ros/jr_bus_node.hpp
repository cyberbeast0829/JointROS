/**
 * @file    jr_bus_node.hpp
 * @brief   `jr_bus` —— 一条总线的 lifecycle 驱动节点（DESIGN §5 / §6 / §8）
 *
 * @par 职责
 *   - **拥有**一条总线的实时循环（`jr::rt::TickGroup`，内部 tick 线程）；
 *   - 把快照投影成 ROS 消息发布（`joint_feedback` / `joint_states` / `bus_status` /
 *     `rt_stats` / `faults`），把 ROS 命令写进无锁命令信箱；
 *   - 承接服务（使能/标定/参数/描述符…）：阻塞调用必须在**安全暂停窗口**内做（ADR-7）。
 *
 * @par 一个节点 = 一条总线
 *  ADR-1 约定话题形如 `/<robot>/jr/<bus>/…`，而 `BusStatus` 这类消息是**单总线**的
 *  —— 所以一个节点服务一条总线，多总线用 launch 起多个实例（各自的 `~/` 命名空间）。
 *  同一条总线的**可执行文件可以由一个进程起多个节点**，但每个节点仍然只碰自己那条。
 *
 * @par 线程模型（§5.1）
 *  - T1 = `TickGroup` 的 RT 线程（内部模式）：只碰 SDK 与无锁结构，**不碰任何 rclcpp**；
 *  - T2/T3/T4 = ROS 执行器：命令/状态/服务。本节点用 `MultiThreadedExecutor` +
 *    三个回调组，保证**阻塞服务不会把状态发布和命令接收一起卡住**。
 */

#ifndef JR_ROS2_ROS_JR_BUS_NODE_HPP
#define JR_ROS2_ROS_JR_BUS_NODE_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <jr_interfaces/msg/bus_status.hpp>
#include <jr_interfaces/msg/joint_command_array.hpp>
#include <jr_interfaces/msg/joint_fault.hpp>
#include <jr_interfaces/msg/joint_feedback_array.hpp>
#include <jr_interfaces/msg/mit_command_array.hpp>
#include <jr_interfaces/msg/rt_stats.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_config_yaml.hpp"
#include "jr_ros2/jr_status.hpp"
#include "jr_ros2/rt/jr_tick_group.hpp"

namespace jr {
namespace ros {

struct JrBusServices;   /**< 服务层（定义在 `jr_bus_services.cpp`：19 个服务 + ADR-7 安全暂停窗口）*/

class JrBusNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit JrBusNode(const rclcpp::NodeOptions &options);
    /** ⚠ 析构**必须**在能看到 `JrBusServices` 完整定义的地方定义（见 `jr_bus_services.cpp`）：
     *  `unique_ptr<不完整类型>` 的删除需要完整定义，否则是未定义行为。 */
    ~JrBusNode() override;

    JrBusNode(const JrBusNode &) = delete;
    JrBusNode &operator=(const JrBusNode &) = delete;

    /* ---------------- lifecycle（顺序与 DESIGN §8.1/§8.2 对应） ---------------- */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

    /* --- 给测试用的钩子（不属于稳定 API） --- */
    const Config &config_for_test() const noexcept { return cfg_; }
    rt::TickGroup *tick_group_for_test() const noexcept { return tg_.get(); }
    unsigned bus_index_for_test() const noexcept { return bus_index_; }

    /* --- 服务层需要的访问点（只给 JrBusServices 用）--- */
    friend struct JrBusServices;

private:
    /** 服务层用：关节名 → **总线内序号**（-1 = 不在本节点负责的总线上）。 */
    int local_index_of(const std::string &name) const noexcept;
    /** 校验请求里的 `bus` 字段（空 = 本节点；不匹配 → 帯人话的拒绝）。 */
    bool bus_matches(const std::string &req_bus, std::string *err) const;
    void create_services();
    void destroy_services();

    /* ---------------- 配置 ---------------- */
    bool load_and_validate(std::string *err);

    /* ---------------- ROS 实体（在 activate 建、deactivate 销） ---------------- */
    void create_entities();
    void destroy_entities();

    /* ---------------- 回调 ---------------- */
    void on_cmd_mit(const jr_interfaces::msg::MitCommandArray::ConstSharedPtr msg);
    void on_cmd_mode(const jr_interfaces::msg::JointCommandArray::ConstSharedPtr msg);
    void on_estop_local(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void on_estop_all(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void publish_feedback();
    void publish_joint_states();
    void publish_bus_status();
    void publish_rt_stats();
    void drain_and_publish_faults();

    /** 快照 → `BusStatus`（话题与服务**共用同一份投影**：两处各写一遍迟早漂）。 */
    void fill_bus_status(const StateSnapshot &s, jr_interfaces::msg::BusStatus *out) const;

    /* ---- 诊断（§6.5）：`jr:<bus>` 一个 updater，每个关节一个（各自的 hardware_id）---- */
    void create_diagnostics();
    void destroy_diagnostics();
    void diag_bus(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void diag_joint(unsigned local_index, diagnostic_updater::DiagnosticStatusWrapper &stat);

    /* ---------------- 内部工具 ---------------- */
    /** 单调时钟（与 RT 侧同源：Linux 上 steady_clock == CLOCK_MONOTONIC）。 */
    static std::uint64_t monotonic_ns() noexcept;

    /** 单调时刻 → ROS 时间（§6.6，启动时记一次偏置）。 */
    builtin_interfaces::msg::Time to_ros_time(std::uint64_t mono_ns) const noexcept;

    /** 关节名 → 全局关节索引（-1 = 不在本节点负责的总线上）。 */
    int global_index_of(const std::string &name) const noexcept;

    /** 把一条 MIT 命令填进命令集（返回 false = 这个关节不归本节点管）。 */
    bool fill_mit_target(const jr_interfaces::msg::MitCommand &in, CommandSet *out) const noexcept;

    /** 提交命令集（维护 seq/时间戳；覆盖次数由核心库记账）。 */
    void submit(CommandSet *cmd);

    /** 只报一次的"关节不归我管"提示（避免刷屏，但**必须**让人看见）。 */
    void warn_unknown_joint_once(const std::string &name);

    /* ---------------- 状态 ---------------- */
    std::string config_file_;
    std::string bus_name_;
    Config cfg_ = {};              /**< 共享 YAML → 核心库配置（§6.4） */
    NodeCfg node_cfg_ = {};        /**< 同一份 YAML 里的**节点层**键（发布率/插值） */
    ConfigNotes notes_ = {};
    bool cfg_loaded_ = false;

    /** 本节点负责的总线在 `cfg_.buses[]` 里的**配置级**索引（`cfg_.buses[...]` 用它）。 */
    unsigned bus_index_ = 0u;
    /** ⚠ 本总线在**它自己的 tick 组里**的位置（F12）。
     *
     *  快照 `StateSnapshot::buses[]` 是由 `TickGroup::fill_snapshot()` 按**组内**下标填的，
     *  快照里 `JointStatePOD::bus_index`、`tg_->bus(i)` 也都是**组内**下标 ——
     *  而 `bus_index_` 是**配置级**下标。两者只有在"进程里恰好跑着整份配置的第一条总线"时才相等。
     *
     *  真机症状（`humanoid_2bus` 示例）：`leg_right` 配置索引=1、组内只有 1 条总线(0)
     *  ⇒ `1 >= buses_count(1)` ⇒ `GetBusStats` 永远回 "no snapshot yet"；关节过滤
     *  `j.bus_index != bus_index_` 也全不匹配 ⇒ 该节点的 `/joint_feedback`、`/joint_states`、
     *  诊断、动关节服务**全废**（左侧恰好两个下标都是 0，所以一直看着是好的）。
     *  用法纪律：碰 `cfg_.buses[]` → 用 `bus_index_`；碰快照/`tg_->bus()`/比较 POD 的 `bus_index`
     *  → 用 `bus_in_group_`。 */
    unsigned bus_in_group_ = 0u;
    unsigned group_index_ = 0u;
    unsigned local_joint_count_ = 0u;
    std::vector<std::string> joint_names_;    /**< 本总线关节名（按总线内序号） */

    std::unique_ptr<rt::TickGroup> tg_;
    /** 服务层（activate 建 / deactivate 销）。
     *  ⚠ 用 `shared_ptr` 而不是 `unique_ptr`：构造函数里遇异常要销毁成员，`unique_ptr`
     *  会在**这一 TU** 实例化 `delete` → 需要完整类型（这里是前向声明）→ 编不过。
     *  `shared_ptr` 的删除器在构造处类型擦除，不完整类型也能安全析构。 */
    std::shared_ptr<JrBusServices> services_;

    std::int64_t steady_to_ros_ns_ = 0;       /**< ROS 时间 - 单调时间（configure 时记一次） */
    bool sim_time_seen_ = false;

    std::uint64_t cmd_seq_ = 0u;
    bool estop_latched_ = false;
    std::vector<std::string> warned_joints_;
    /** 已告警过的"模式不符/限制量不适用"组合（键如 `j0/8`）—— 避免每帧刷屏。 */
    std::vector<std::string> warned_modes_;
    /**
     * 节点侧拒绝转发的命令数（模式不符 / 非法模式值 / 非法量值 / 未知关节）。
     *
     * ⚠ 为什么要单独计数：核心库的 `cmd_rejected` 只能统计"面到了 RT 侧才被丢"的目标；
     *   而 `~/cmd_mit` 打到一个非 MIT 关节时，命令在 **ROS 域就被拒**了（根本不该进信箱）。
     *   两个计数合起来才是客户看到的"命令为什么没动"。
     */
    std::uint32_t cmd_rejected_self_ = 0u;

    /* 回调组（§5.1 的 T2/T3/T4 分离：阻塞服务不能卡住状态发布与命令接收） */
    rclcpp::CallbackGroup::SharedPtr ros_cbg_;   /**< 命令 + 状态（相互独立、可并发） */
    rclcpp::CallbackGroup::SharedPtr svc_cbg_;   /**< 阻塞型服务：与上面互斥隔离 */

    /* 发布者 */
    rclcpp::Publisher<jr_interfaces::msg::JointFeedbackArray>::SharedPtr pub_feedback_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::Publisher<jr_interfaces::msg::BusStatus>::SharedPtr pub_bus_status_;
    rclcpp::Publisher<jr_interfaces::msg::RtStats>::SharedPtr pub_rt_stats_;
    rclcpp::Publisher<jr_interfaces::msg::JointFault>::SharedPtr pub_faults_;

    /* 订阅 */
    rclcpp::Subscription<jr_interfaces::msg::MitCommandArray>::SharedPtr sub_cmd_mit_;
    rclcpp::Subscription<jr_interfaces::msg::JointCommandArray>::SharedPtr sub_cmd_mode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_all_;

    /* 计时器（T4） */
    rclcpp::TimerBase::SharedPtr tmr_feedback_;
    rclcpp::TimerBase::SharedPtr tmr_joint_states_;
    rclcpp::TimerBase::SharedPtr tmr_bus_status_;
    rclcpp::TimerBase::SharedPtr tmr_rt_stats_;
    rclcpp::TimerBase::SharedPtr tmr_faults_;

    /** 诊断：`[0]` = 总线级（hardware_id `jr:<bus>`），`[1+j]` = 关节级（`jr:<bus>/<joint>`）。 */
    std::vector<std::shared_ptr<diagnostic_updater::Updater>> diags_;
};

}  // namespace ros
}  // namespace jr

#endif /* JR_ROS2_ROS_JR_BUS_NODE_HPP */
