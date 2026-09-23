/**
 * @file    test_jr_bus_node.cpp
 * @brief   节点级测试（WP2）：lifecycle 顺序、冷启动不发帧、命令链路、超时可见、坏配置拒绝
 *
 * @par 与 `test_bus_runtime.cpp` 的分工
 *  核心库测试（无 ROS）证明"实时逻辑对"；这里证明**节点把它接对了**：
 *  生命周期迁移顺序、命令话题 → 信箱、快照 → 状态发布、退出时不留下"停发即看门狗"。
 *
 * @par 为什么要跑真 DDS
 *  命令链路（`~/cmd_mit` → 信箱）如果只在进程内直接调私有的转发函数，
 *  就测不到 QoS/命名空间/回调组这些**现场最容易错**的部分。因此这里起两个节点：
 *  被测节点 + 一个测试驱动节点（发布命令、订阅反馈），用多线程执行器跑。
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <jr_interfaces/msg/joint_command.hpp>
#include <jr_interfaces/msg/joint_command_array.hpp>
#include <jr_interfaces/msg/param_write.hpp>
#include <jr_interfaces/msg/rt_stats.hpp>
#include <jr_interfaces/srv/get_bus_stats.hpp>
#include <jr_interfaces/srv/jog.hpp>
#include <jr_interfaces/srv/read_params.hpp>
#include <jr_interfaces/srv/set_enabled.hpp>
#include <jr_interfaces/srv/write_params.hpp>

#include "jr_ros2/ros/jr_bus_node.hpp"
#include "jr_test.hpp"

using namespace std::chrono_literals;

namespace {

using jr::StateSnapshot;          /* 快照 POD 在 jr 命名空间（不是 jr::rt） */
using jr::rt::BusMode;
using jr::rt::TickGroup;

/* ==========================================================================
 * 枚举映射守卫（核心库 ↔ 消息常量）：改一处不改另一处 → **编译期**就红
 * ========================================================================== */
static_assert(static_cast<int>(jr::Status::kOk) ==
                  jr_interfaces::msg::JointResult::STATUS_OK,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kInvalidArgument) ==
                  jr_interfaces::msg::JointResult::STATUS_INVALID_ARGUMENT,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kInvalidState) ==
                  jr_interfaces::msg::JointResult::STATUS_INVALID_STATE,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kTimeout) == jr_interfaces::msg::JointResult::STATUS_TIMEOUT,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kTransport) ==
                  jr_interfaces::msg::JointResult::STATUS_TRANSPORT,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kProtocol) == jr_interfaces::msg::JointResult::STATUS_PROTOCOL,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kNotFound) == jr_interfaces::msg::JointResult::STATUS_NOT_FOUND,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kNotSupported) ==
                  jr_interfaces::msg::JointResult::STATUS_NOT_SUPPORTED,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kNoMemory) == jr_interfaces::msg::JointResult::STATUS_NO_MEMORY,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kNotCalibrated) ==
                  jr_interfaces::msg::JointResult::STATUS_NOT_CALIBRATED,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kLocked) == jr_interfaces::msg::JointResult::STATUS_LOCKED,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kIoError) == jr_interfaces::msg::JointResult::STATUS_IO_ERROR,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kUnverified) ==
                  jr_interfaces::msg::JointResult::STATUS_UNVERIFIED,
              "Status ↔ JointResult::STATUS_* 漂了");
static_assert(static_cast<int>(jr::Status::kInternal) == jr_interfaces::msg::JointResult::STATUS_INTERNAL,
              "Status ↔ JointResult::STATUS_* 漂了");

static_assert(static_cast<int>(jr::Advice::kNone) == jr_interfaces::msg::JointFault::ADVICE_NONE,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kRetryFaultReset) ==
                  jr_interfaces::msg::JointFault::ADVICE_RETRY_FAULT_RESET,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kNeedsDeviceReset) ==
                  jr_interfaces::msg::JointFault::ADVICE_NEEDS_DEVICE_RESET,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kCheckBusTermination) ==
                  jr_interfaces::msg::JointFault::ADVICE_CHECK_BUS_TERMINATION,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kCheckBusConfig) ==
                  jr_interfaces::msg::JointFault::ADVICE_CHECK_BUS_CONFIG,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kReduceRateOrRaiseWatchdog) ==
                  jr_interfaces::msg::JointFault::ADVICE_REDUCE_RATE_OR_RAISE_WATCHDOG,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kCheckTemperature) ==
                  jr_interfaces::msg::JointFault::ADVICE_CHECK_TEMPERATURE,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kCheckSupplyVoltage) ==
                  jr_interfaces::msg::JointFault::ADVICE_CHECK_SUPPLY_VOLTAGE,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kCalibrationRequired) ==
                  jr_interfaces::msg::JointFault::ADVICE_CALIBRATION_REQUIRED,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kEnsureSingleMaster) ==
                  jr_interfaces::msg::JointFault::ADVICE_ENSURE_SINGLE_MASTER,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kCheckRtPermissions) ==
                  jr_interfaces::msg::JointFault::ADVICE_CHECK_RT_PERMISSIONS,
              "Advice ↔ JointFault::ADVICE_* 漂了");
static_assert(static_cast<int>(jr::Advice::kFixBusPlanning) ==
                  jr_interfaces::msg::JointFault::ADVICE_FIX_BUS_PLANNING,
              "Advice ↔ JointFault::ADVICE_* 漂了");

static_assert(static_cast<int>(jr::ParamType::kUnsupported) ==
                  jr_interfaces::msg::ParamValue::TYPE_UNSUPPORTED,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kBool) == jr_interfaces::msg::ParamValue::TYPE_BOOL,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kU8) == jr_interfaces::msg::ParamValue::TYPE_U8,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kI8) == jr_interfaces::msg::ParamValue::TYPE_I8,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kU16) == jr_interfaces::msg::ParamValue::TYPE_U16,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kI16) == jr_interfaces::msg::ParamValue::TYPE_I16,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kU32) == jr_interfaces::msg::ParamValue::TYPE_U32,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kI32) == jr_interfaces::msg::ParamValue::TYPE_I32,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kU64) == jr_interfaces::msg::ParamValue::TYPE_U64,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kI64) == jr_interfaces::msg::ParamValue::TYPE_I64,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kF32) == jr_interfaces::msg::ParamValue::TYPE_F32,
              "ParamType ↔ ParamValue::TYPE_* 漂了");
static_assert(static_cast<int>(jr::ParamType::kF64) == jr_interfaces::msg::ParamValue::TYPE_F64,
              "ParamType ↔ ParamValue::TYPE_* 漂了");

/** 写一份虚拟总线配置（§6.4 schema；只暴露本测试关心的开关）。 */
std::string write_config(const char *path, bool auto_enable, unsigned master_id, unsigned timeout_ms,
                         unsigned publish_hz, unsigned node_id0 = 1u, bool allow_write = false,
                         bool allow_flash = false, const char *mode0 = "mit")
{
    std::FILE *f = std::fopen(path, "wb");
    if (f == nullptr) return std::string();
    std::fprintf(f,
                 "jr:\n"
                 "  rt: {enabled: false}\n"
                 "  tick_groups:\n"
                 "    - {name: g0, rate_hz: 1000, buses: [vbus]}\n"
                 "  buses:\n"
                 "    - name: vbus\n"
                 "      type: virtual\n"
                 "      spec: \"0:id=%u,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd;"
                 "1:id=2,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd\"\n"
                 "      is_fd: true\n"
                 "      master_id: %u\n"
                 "      joints: [j0, j1]\n"
                 "  joints:\n"
                 "    - {name: j0, bus: vbus, node_id: %u, mode: %s}\n"
                 "    - {name: j1, bus: vbus, node_id: 2}\n"
                 "  limits:\n"
                 "    j0: {position: [-1.0, 1.0], stiffness: 30.0, damping: 1.0}\n"
                 "    j1: {position: [-1.0, 1.0], stiffness: 30.0, damping: 1.0}\n"
                 "  feedback: {source: broadcast_plus_heartbeat, heartbeat_ms: 5, publish_hz: %u, "
                 "joint_state_hz: %u}\n"
                 "  command: {timeout_ms: %u, timeout_action: hold}\n"
                 "  safety: {auto_enable: %s, require_calibrated: true, on_exit_action: disable}\n"
                 "  params: {allow_write: %s, allow_flash_persist: %s}\n"
                 "  descriptor: {cache_enabled: false}\n"
                 "  bus_lock: {enabled: false}\n",
                 node_id0, master_id, node_id0, mode0, publish_hz, publish_hz, timeout_ms,
                 auto_enable ? "true" : "false", allow_write ? "true" : "false",
                 allow_flash ? "true" : "false");
    std::fclose(f);
    return std::string(path);
}

/**
 * 起一个被测节点（参数用 override 注入，等价于 `-p`）。
 *
 * ⚠ 每个用例必须给**唯一的节点名**：本测试是"一个进程里跑好几个节点"，
 *   DDS 图是按 `名字/话题` 索引的 —— 名字复用会让 `wait_for_service` 命中
 *   **上一个已经注销的服务**（图里还在），于是请求发出去没人应（超时），
 *   而现象看起来像"服务没实现"。实测踩过：`/jr_bus` 下数出 31 个服务，
 *   三个用例都卡在同一个名字上。
 */
std::shared_ptr<jr::ros::JrBusNode> make_node(const std::string &cfg_path, const char *bus = "vbus",
                                              const char *name = "jr_bus")
{
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("config_file", cfg_path),
                              rclcpp::Parameter("bus", std::string(bus)),
                              rclcpp::Parameter("use_sim_time", false)});
    /* `__node:=` 是标准的名字重映射（与 `ros2 run ... --ros-args -r __node:=x` 同一条路）。 */
    opts.arguments({"--ros-args", "-r", std::string("__node:=") + name});
    auto node = std::make_shared<jr::ros::JrBusNode>(opts);
    /* 名字重映射必须真的生效，否则上面那段注释就是假的（测试自己的前提也要断言）。
       ⚠ 必须 `strcmp`：`const char*` 用 `==` 比的是**指针**（刚踩过，永远是 false）。 */
    JR_CHECK_MSG(std::strcmp(node->get_name(), name) == 0,
                 "节点名重映射没生效：同进程重名会让 DDS 图撞车");
    return node;
}

bool configure(std::shared_ptr<jr::ros::JrBusNode> &n)
{
    return n->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id() ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool activate(std::shared_ptr<jr::ros::JrBusNode> &n)
{
    return n->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id() ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void deactivate(std::shared_ptr<jr::ros::JrBusNode> &n)
{
    n->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

/** 收尾：cleanup（若还没回到 UNCONFIGURED）→ shutdown（finalized）。
 *
 *  ⚠ 为什么必须走到 shutdown：`LifecycleNode` 析构时如果不是 finalized，
 *  rclcpp 会打印 "LifecycleNode is not shut down ... in destructor" ——
 *  这行 warning 会污染 colcon 的"零警告"验收，也会让真正的告警淹没在噪声里。 */
void cleanup(std::shared_ptr<jr::ros::JrBusNode> &n)
{
    if (n == nullptr) return;
    if (n->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        n->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
    n->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

/** 按关节名找快照里的下标（找不到返回 -1）。 */
int snap_index(const StateSnapshot *s, const char *name)
{
    if (s == nullptr) return -1;
    for (unsigned i = 0u; i < s->joint_count; ++i) {
        if (std::strcmp(s->joints[i].name, name) == 0) return static_cast<int>(i);
    }
    return -1;
}

/**
 * 打印一行快照摘要。
 *
 * ⚠ 为什么测试要自带这个：失败时只有一句 "max_pos > 0.10" 时，排查要从头猜
 * （是没使能？没发帧？名字对不上？）—— 把"节点实际看到什么"打出来，
 * 一眼就能定位到层。
 */
void dump_snapshot(const std::shared_ptr<jr::ros::JrBusNode> &node, const char *tag)
{
    TickGroup *tg = node->tick_group_for_test();
    if (tg == nullptr) {
        std::printf("  [%s] tick group is null (not configured)\n", tag);
        return;
    }
    const StateSnapshot *s = tg->acquire_snapshot();
    if (s == nullptr) {
        std::printf("  [%s] no snapshot yet\n", tag);
        return;
    }
    std::printf("  [%s] mode=%s tick=%llu tx=%u joints=%u note='%s'\n", tag,
                jr::rt::to_string(static_cast<BusMode>(s->mode)),
                static_cast<unsigned long long>(s->rt.tick_count), s->buses[0].tx_frames,
                s->joint_count, s->note);
    for (unsigned i = 0u; i < s->joint_count; ++i) {
        const jr::JointStatePOD &j = s->joints[i];
        std::printf("        joint[%u] name='%s' enabled=%d online=%d pos=%.4f tx_j=%u\n", i, j.name,
                    j.enabled ? 1 : 0, j.online ? 1 : 0, j.position, j.tx_frames);
    }
}

/** 有界等待：等到条件成立返回 true（**不用固定睡眠当断言**）。 */
template <typename Fn>
bool wait_for(Fn fn, unsigned timeout_ms)
{
    for (unsigned i = 0u; i < timeout_ms; ++i) {
        if (fn()) return true;
        std::this_thread::sleep_for(1ms);
    }
    return fn();
}

const char *kCfgPath = "jr_test_node_bus.yaml";

/* ==========================================================================
 * ① 冷启动：没有控制帧（§10.2）
 * ======================================================================== */

void test_cold_start_sends_no_control_frames()
{
    JR_CASE("冷启动不发任何控制帧；退出后设备不留在\"停发即看门狗\"状态（§10.2）");

    const std::string cfg = write_config(kCfgPath, /*auto_enable=*/false, 1u, 100u, 500u);
    JR_CHECK(!cfg.empty());

    auto node = make_node(cfg, "vbus", "jr_bus_cold");
    JR_CHECK_MSG(configure(node), "configure must succeed on a virtual bus");

    TickGroup *tg = node->tick_group_for_test();
    JR_CHECK(tg != nullptr);
    JR_CHECK(activate(node));
    dump_snapshot(node, "cold-start: just activated");

    /* ⚠ 口径：`buses[].tx_frames` 是**所有**发送帧（含 configure 阶段的描述符/参数帧、
       以及失能时的安全帧），所以不能用"绝对值 == 0"来断言"没发控制帧"——
       要用**增量**（这正是 WP3 测试的做法）。 */
    const std::uint32_t tx0 = tg->acquire_snapshot()->buses[0].tx_frames;

    /* 跑 200 ms（= 200 个 1 kHz 周期）后增量必须为 0。 */
    std::this_thread::sleep_for(200ms);
    const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
    dump_snapshot(node, "cold-start: after 200 ms");
    JR_CHECK(s != nullptr);
    if (s != nullptr) {
        JR_CHECK_MSG(s->buses[0].tx_frames == tx0,
                     "未使能时不得发控制帧（增量必须为 0）—— 这是『上电即驱动』的防线");
        JR_CHECK_MSG(s->rt.tick_count > 100u, "tick 线程应该在跑（tick_count 必须增长）");
        /* 反馈也要在（否则客户看不到"设备在线但未使能"） */
        const int j0 = snap_index(s, "j0");
        JR_CHECK_MSG(j0 >= 0, "快照必须带关节名（否则 ROS 侧的消息里名字是空的）");
        if (j0 >= 0) {
            JR_CHECK_EQ(s->joints[j0].enabled, false);
            JR_CHECK_EQ(s->joints[j0].online, true);
        }
    }

    /* 退出：安全失能 → 停 tick → 关总线（幂等）。
       失能序列本身会发几帧（安全帧），所以要**先取基线再等**。 */
    deactivate(node);
    const std::uint32_t tx_after_deact = node->tick_group_for_test()->acquire_snapshot()->buses[0].tx_frames;
    std::this_thread::sleep_for(50ms);
    const StateSnapshot *s2 = node->tick_group_for_test()->acquire_snapshot();
    dump_snapshot(node, "after deactivate");
    if (s2 != nullptr) {
        JR_CHECK_EQ(s2->buses[0].tx_frames, tx_after_deact);
        JR_CHECK_EQ(static_cast<int>(s2->mode), static_cast<int>(BusMode::kReady));
    }
    cleanup(node);
    JR_CHECK(node->tick_group_for_test() == nullptr);   /* 总线已关、锁已释放 */
}

/* ==========================================================================
 * ② auto_enable + 命令链路 + 命令超时可见
 * ======================================================================== */

void test_command_path_and_timeout_visibility()
{
    JR_CASE("命令走真话题到达设备；停发后超时必须**可见**（§10.2）");

    const std::string cfg = write_config(kCfgPath, /*auto_enable=*/true, 1u, 100u, 200u);
    auto node = make_node(cfg, "vbus", "jr_bus_cmd");
    JR_CHECK(configure(node));
    JR_CHECK(activate(node));

    /* 驱动节点：发命令、收反馈（用真 DDS，验 QoS/命名空间/回调组）。
       ⚠ 话题名必须拼到**被测节点的私有命名空间**（`~/xxx` = `/<node_name>/xxx`），
          否则发布到 `/jr_test_driver/cmd_mit` 上就什么都没测到。 */
    auto driver = std::make_shared<rclcpp::Node>("jr_test_driver");
    const std::string cmd_topic = std::string("/") + node->get_name() + "/cmd_mit";
    const std::string fb_topic = std::string("/") + node->get_name() + "/joint_feedback";
    auto pub = driver->create_publisher<jr_interfaces::msg::MitCommandArray>(cmd_topic,
                                                                            rclcpp::QoS(1).reliable());
    std::atomic<int> feedback_count{0};
    auto sub = driver->create_subscription<jr_interfaces::msg::JointFeedbackArray>(
        fb_topic, rclcpp::QoS(1).best_effort(),
        [&feedback_count](jr_interfaces::msg::JointFeedbackArray::ConstSharedPtr) {
            feedback_count.fetch_add(1);
        });

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2u, false);
    /* lifecycle 节点要从 node base 接口加进去（`add_node(shared_ptr<Node>)` 不适用）。 */
    exec.add_node(node->get_node_base_interface());
    exec.add_node(driver);
    std::thread spin_thread([&exec]() { exec.spin(); });

    /* 使能后应该开始发控制帧（auto_enable=true）。 */
    const bool frames = wait_for(
        [&]() {
            const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
            return s != nullptr && s->buses[0].tx_frames > 0u;
        },
        2000u);
    JR_CHECK_MSG(frames, "auto_enable=true 时应该发出控制帧");
    dump_snapshot(node, "after auto-enable");
    {
        const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
        const int i = snap_index(s, "j0");
        JR_CHECK_MSG(i >= 0, "快照必须带关节名");
        JR_CHECK_MSG(i >= 0 && s->joints[i].enabled,
                     "auto_enable=true 时关节必须真的使能（否则后面的『位置没上升』只是在验一条死链路）");
    }

    /* 发一条 MIT 命令 → 关节应该动起来。 */
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    double max_pos = 0.0;
    while (std::chrono::steady_clock::now() < deadline) {
        jr_interfaces::msg::MitCommandArray cmd;
        cmd.header.frame_id = "vbus";
        jr_interfaces::msg::MitCommand c;
        c.name = "j0";
        c.gain_mode = jr_interfaces::msg::MitCommand::SI;
        c.position = 0.2;
        c.stiffness = 30.0;
        c.damping = 1.0;
        cmd.commands.push_back(c);
        pub->publish(cmd);
        std::this_thread::sleep_for(20ms);
        const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
        if (s != nullptr) {
            const int i = snap_index(s, "j0");
            if (i >= 0) max_pos = std::max(max_pos, s->joints[i].position);
            if (max_pos > 0.10) break;
        }
    }
    JR_CHECK_MSG(max_pos > 0.10, "命令链路没打通：j0 位置没有上升（>0.10 rad 才说明真的在闭环）");
    dump_snapshot(node, "after commanding j0=0.2");
    JR_CHECK_MSG(feedback_count.load() > 5, "joint_feedback 应该被发布出来");

    /* 停发命令 → 100 ms 后必须触发"命令超时"（动作按配置 = hold），且备注可见。 */
    const bool timeout_visible = wait_for(
        [&]() {
            const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
            return s != nullptr && std::strstr(s->note, "timeout") != nullptr;
        },
        1500u);
    JR_CHECK_MSG(timeout_visible, "命令超时必须在快照备注里可见（否则现场只能看到\"关节不动\"）");

    exec.cancel();
    spin_thread.join();
    deactivate(node);
    cleanup(node);
}

/* ==========================================================================
 * ⑤ 服务：闸门必须拦住，逐关节使能必须真的逐关节
 * ======================================================================== */

/**
 * 调一次服务并返回响应（失败返回 nullptr）。
 *
 * ⚠ 两条纪律（都是踩出来的）：
 *   1. 失败**必须自己打印原因**，不能只靠断言消息 —— 重定向到文件时 stdio 是全缓冲的，
 *      进程如果继续跑到段错误，断言消息可能就没了（实测踩过：ctest 只报 "1 failure"）。
 *   2. 返回**响应指针**而不是 bool：调用点 `resp->xxx` 前必须判空，
 *      否则一次服务不可用就把测试变成段错误（信息量为零）。
 */
template <class SrvT>
typename SrvT::Response::SharedPtr call_service(const rclcpp::Node::SharedPtr &driver,
                                                const std::string &name,
                                                const typename SrvT::Request::SharedPtr &req)
{
    auto client = driver->create_client<SrvT>(name);
    if (!client->wait_for_service(3s)) {
        std::printf("  [srv] %s NOT AVAILABLE (3 s)\n", name.c_str());
        std::fflush(stdout);
        return nullptr;
    }
    auto fut = client->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) {
        std::printf("  [srv] %s TIMED OUT (5 s)\n", name.c_str());
        std::fflush(stdout);
        return nullptr;
    }
    auto resp = fut.get();
    if (resp == nullptr) {
        std::printf("  [srv] %s returned a NULL response\n", name.c_str());
        std::fflush(stdout);
    }
    return resp;
}

void test_service_gates_and_per_joint_enable()
{
    JR_CASE("服务闸门拦得住；逐关节使能真的只动选中的那个（§8.5 / ADR-7）");

    /* allow_write=false → 写参数必须被拒（闸门在**服务层**，不依赖客户自觉）。 */
    const std::string cfg = write_config(kCfgPath, /*auto_enable=*/false, 1u, 100u, 200u,
                                        /*node_id0=*/1u, /*allow_write=*/false);
    auto node = make_node(cfg, "vbus", "jr_bus_svc");
    JR_CHECK(configure(node));
    JR_CHECK(activate(node));

    auto driver = std::make_shared<rclcpp::Node>("jr_test_driver_svc");

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2u, false);
    exec.add_node(node->get_node_base_interface());
    exec.add_node(driver);
    std::thread spin_thread([&exec]() { exec.spin(); });

    const std::string ns = std::string("/") + node->get_name();

    /* ⚠ 本用例同时是**回调组生命周期**的回归用例：服务跑在节点持有的自定义互斥组里。
       如果哪天有人把那个组降级成函数的局部变量（rclcpp 的节点与实体只存 weak_ptr 引用），
       下面所有调用会**静默超时**（服务在图里还看得见、`wait_for_service` 也成功）——
       正是这条用例最先把它抓出来。 */

    /* ① 服务是不是真的都起来了（19 个）：先看服务清单，缺服务比"调用超时"好定位得多。 */
    {
        const auto names = driver->get_service_names_and_types();
        int jr_srv = 0;
        for (const auto &kv : names) {
            if (kv.first.compare(0u, ns.size(), ns) == 0) ++jr_srv;
        }
        /* 清单里还含生命周期/参数服务（12 个），所以只断言"我们的 19 个一个不少"。 */
        for (const char *n : {"set_enabled", "calibrate", "home", "set_zero", "save_config",
                              "reset_device", "set_node_id", "fault_reset", "jog", "read_params",
                              "write_params", "list_endpoints", "lookup_endpoint", "get_device_info",
                              "get_bus_stats", "get_descriptor_info", "export_descriptor",
                              "import_descriptor", "publish_heartbeat_hint"}) {
            const std::string full = ns + "/" + n;
            JR_CHECK_MSG(names.find(full) != names.end(), full.c_str());
        }
        std::printf("  [srv] %d service(s) under %s (12 lifecycle/param + 19 ours)\n", jr_srv, ns.c_str());
        std::fflush(stdout);
    }

    /* ② WriteParams：confirm=true 但 allow_write=false → 拒绝，且要说清缺哪个开关 */
    {
        auto req = std::make_shared<jr_interfaces::srv::WriteParams::Request>();
        req->confirm = true;
        jr_interfaces::msg::ParamWrite w;
        w.joint = "j0";
        w.path = "axis0.motor.config.pre_calibrated";
        w.text_value = "true";
        req->writes.push_back(w);
        auto resp = call_service<jr_interfaces::srv::WriteParams>(driver, ns + "/write_params", req);
        JR_CHECK_MSG(resp != nullptr, "write_params 必须可调用（服务在 /jr_bus 下、ServicesQoS）");
        if (resp != nullptr) {
            JR_CHECK_MSG(!resp->success, "allow_write=false 时写参数必须被拒");
            JR_CHECK_MSG(resp->message.find("allow_write") != std::string::npos,
                         "拒绝时必须点名缺哪个开关（客户才知道怎么开）");
            if (resp->message.find("allow_write") == std::string::npos) {
                std::printf("  [srv] 拒绝理由实际是: '%s'\n", resp->message.c_str());
                std::fflush(stdout);
            }
        }
    }
    /* ② WriteParams：confirm=false → 拒绝 */
    {
        auto req = std::make_shared<jr_interfaces::srv::WriteParams::Request>();
        req->confirm = false;
        jr_interfaces::msg::ParamWrite w;
        w.joint = "j0";
        w.path = "axis0.motor.config.pre_calibrated";
        w.text_value = "true";
        req->writes.push_back(w);
        auto resp = call_service<jr_interfaces::srv::WriteParams>(driver, ns + "/write_params", req);
        JR_CHECK(resp != nullptr);
        if (resp != nullptr) {
            JR_CHECK_MSG(!resp->success, "写参数缺少 confirm=true 必须被拒（§8.5）");
        }
    }
    /* ③ Jog：confirm=false → 拒绝；duration 超硬上限 → 拒绝（不做"帮你截断"） */
    {
        auto req = std::make_shared<jr_interfaces::srv::Jog::Request>();
        req->joint = "j0";
        req->duration_s = 1.0;
        req->confirm = false;
        auto resp = call_service<jr_interfaces::srv::Jog>(driver, ns + "/jog", req);
        JR_CHECK(resp != nullptr);
        if (resp != nullptr) JR_CHECK_MSG(!resp->success, "Jog 必须要求 confirm=true");

        auto req2 = std::make_shared<jr_interfaces::srv::Jog::Request>();
        req2->joint = "j0";
        req2->duration_s = 30.0;   /* 硬上限 10 s */
        req2->confirm = true;
        auto resp2 = call_service<jr_interfaces::srv::Jog>(driver, ns + "/jog", req2);
        JR_CHECK(resp2 != nullptr);
        if (resp2 != nullptr) JR_CHECK_MSG(!resp2->success, "duration > 10 s 必须直接拒绝");
    }
    /* ④ SetEnabled：只使能 j0 → 必须**只**使能 j0（逐关节路径 + 安全暂停窗口） */
    {
        auto req = std::make_shared<jr_interfaces::srv::SetEnabled::Request>();
        req->joints = {"j0"};
        req->enable = true;
        req->resume = false;
        auto resp = call_service<jr_interfaces::srv::SetEnabled>(driver, ns + "/set_enabled", req);
        JR_CHECK_MSG(resp != nullptr, "set_enabled 必须可调用");
        if (resp != nullptr) {
            JR_CHECK_MSG(resp->success, resp->message.c_str());
            const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
            const int i0 = snap_index(s, "j0");
            const int i1 = snap_index(s, "j1");
            JR_CHECK(i0 >= 0 && i1 >= 0);
            if (i0 >= 0 && i1 >= 0) {
                JR_CHECK_MSG(s->joints[i0].enabled, "被选中的 j0 必须已使能");
                JR_CHECK_MSG(!s->joints[i1].enabled, "未选中的 j1 不该被顺带使能");
            }
            JR_CHECK_EQ(resp->results.size(), 1u);
        }
    }
    /* ⑤ ReadParams / GetBusStats：基本可用（读参数需要描述符已就绪） */
    {
        auto req = std::make_shared<jr_interfaces::srv::ReadParams::Request>();
        req->joints = {"j0"};
        req->paths = {"axis0.motor.config.gear_ratio"};
        auto resp = call_service<jr_interfaces::srv::ReadParams>(driver, ns + "/read_params", req);
        JR_CHECK(resp != nullptr);
        if (resp != nullptr) {
            JR_CHECK_EQ(resp->items.size(), 1u);
            if (!resp->items.empty()) {
                JR_CHECK_MSG(resp->items[0].type != jr_interfaces::msg::ParamValue::TYPE_UNSUPPORTED,
                             "gear_ratio 应该是可读的标量端点");
            }
        }
        auto breq = std::make_shared<jr_interfaces::srv::GetBusStats::Request>();
        auto bresp = call_service<jr_interfaces::srv::GetBusStats>(driver, ns + "/get_bus_stats", breq);
        JR_CHECK(bresp != nullptr);
        if (bresp != nullptr) {
            JR_CHECK_MSG(bresp->success, bresp->message.c_str());
            /* ⚠ `JR_CHECK_EQ` 是按数值比较的（内部 cast double）→ 字符串要用 JR_CHECK。 */
            JR_CHECK(bresp->status.bus == "vbus");
        }
    }

    exec.cancel();
    spin_thread.join();
    deactivate(node);
    cleanup(node);
}

/* ==========================================================================
 * ⑥ §10.2 ④：广播降级为单播时，原因必须**可见**
 * ======================================================================== */

void test_broadcast_degradation_is_visible()
{
    JR_CASE("广播被降级为单播时，原因必须出现在备注里（§10.2 ④）");

    /* node_id=9 > 7 → SDK 的位图广播寻址不到它 → 必须降级并**说明原因**。 */
    const std::string cfg = write_config(kCfgPath, /*auto_enable=*/true, 1u, 100u, 200u, /*node_id0=*/9u);
    auto node = make_node(cfg, "vbus", "jr_bus_deg");
    JR_CHECK(configure(node));
    JR_CHECK(activate(node));

    auto driver = std::make_shared<rclcpp::Node>("jr_test_driver_bcast");
    const std::string cmd_topic = std::string("/") + node->get_name() + "/cmd_mit";
    auto pub = driver->create_publisher<jr_interfaces::msg::MitCommandArray>(cmd_topic,
                                                                            rclcpp::QoS(1).reliable());
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2u, false);
    exec.add_node(node->get_node_base_interface());
    exec.add_node(driver);
    std::thread spin_thread([&exec]() { exec.spin(); });

    /* 发一批命令（两个关节都有目标），让 RT 侧具备"考虑广播"的前提。 */
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
        jr_interfaces::msg::MitCommandArray cmd;
        for (const char *name : {"j0", "j1"}) {
            jr_interfaces::msg::MitCommand c;
            c.name = name;
            c.gain_mode = jr_interfaces::msg::MitCommand::SI;
            c.position = 0.0;
            c.stiffness = 5.0;
            c.damping = 0.5;
            cmd.commands.push_back(c);
        }
        pub->publish(cmd);
        std::this_thread::sleep_for(20ms);
    }

    const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
    JR_CHECK(s != nullptr);
    if (s != nullptr) {
        const std::string note(s->buses[0].last_note);
        std::printf("  [degradation] bus note: '%s'\n", note.c_str());
        JR_CHECK_MSG(note.find("broadcast") != std::string::npos || note.find("unicast") != std::string::npos,
                     "降级原因必须写进备注（否则客户只知道『关节不动』）");
    }

    exec.cancel();
    spin_thread.join();
    deactivate(node);
    cleanup(node);
}

/* ==========================================================================
 * ⑦ §6.5 诊断：必须真的**发出来**，且 hardware_id 精确到总线/关节
 * ======================================================================== */

void test_diagnostics_are_published()
{
    JR_CASE("§6.5 诊断真的发到 /diagnostics（hardware_id 精确到总线与关节）");

    const std::string cfg = write_config(kCfgPath, false, 1u, 100u, 200u);
    auto node = make_node(cfg, "vbus", "jr_bus_diag");
    JR_CHECK(configure(node));
    JR_CHECK(activate(node));

    auto driver = std::make_shared<rclcpp::Node>("jr_test_driver_diag");
    std::mutex m;
    /* 收集 hardware_id → 该条目里的 key 集合（用来断言"关节条目带的是关节量"）。 */
    std::map<std::string, std::set<std::string>> keys;
    auto sub = driver->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).best_effort(),
        [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lk(m);
            for (const auto &st : msg->status) {
                auto &ks = keys[st.hardware_id];
                for (const auto &kv : st.values) ks.insert(kv.key);
            }
        });

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2u, false);
    exec.add_node(node->get_node_base_interface());
    exec.add_node(driver);
    std::thread spin_thread([&exec]() { exec.spin(); });

    /* 诊断 1 Hz → 有界等 6 s（不靠固定 sleep 断言）。 */
    const bool got_all = wait_for(
        [&]() {
            std::lock_guard<std::mutex> lk(m);
            return keys.count("jr:vbus") > 0u && keys.count("jr:vbus/j0") > 0u &&
                   keys.count("jr:vbus/j1") > 0u;
        },
        6000u);
    {
        std::lock_guard<std::mutex> lk(m);
        std::printf("  [diag] hardware_id seen: %zu\n", keys.size());
        for (const auto &kv : keys) std::printf("         %s (%zu key(s))\n", kv.first.c_str(), kv.second.size());
        std::fflush(stdout);
    }
    JR_CHECK_MSG(got_all, "1 Hz 诊断必须在 6 s 内出现在 /diagnostics 上（只建 updater 不算落地）");

    {
        std::lock_guard<std::mutex> lk(m);
        const auto bus_it = keys.find("jr:vbus");
        const auto j0_it = keys.find("jr:vbus/j0");
        if (bus_it != keys.end()) {
            /* 总线条目必须带"链路/负载/锁"这类总线级量（不是把关节量糊上来）。 */
            for (const char *k : {"link_up", "tx_frames", "bus_load_estimate", "lock_held"}) {
                JR_CHECK_MSG(bus_it->second.count(k) > 0u, k);
            }
        }
        if (j0_it != keys.end()) {
            /* 关节条目必须带关节量（温度/使能/错误位…）。 */
            for (const char *k : {"online", "enabled", "position_rad", "error_code_raw", "tx_rejected"}) {
                JR_CHECK_MSG(j0_it->second.count(k) > 0u, k);
            }
        }
        if (bus_it != keys.end() && j0_it != keys.end()) {
            JR_CHECK_MSG(bus_it->second.count("position_rad") == 0u,
                         "总线条目里不该出现关节量（否则客户按 hardware_id 过滤会读到脏数据）");
        }
    }

    exec.cancel();
    spin_thread.join();
    deactivate(node);
    cleanup(node);
}

/* ==========================================================================
 * ⑧ `~/cmd`（ADR-4 二级接口）：模式化命令真的能控、模式不符**可见地**被拒
 * ======================================================================== */

void test_mode_command_topic()
{
    JR_CASE("~/cmd：CSP 命令真的控住关节；模式不符的命令被拒且**计数可见**（ADR-4 / §8.5）");

    /* j0 配成 CSP，j1 保持 MIT —— 一条总线上两种模式共存。 */
    const std::string cfg = write_config(kCfgPath, /*auto_enable=*/true, 1u, 100u, 200u,
                                        /*node_id0=*/1u, /*allow_write=*/false,
                                        /*allow_flash=*/false, /*mode0=*/"csp");
    auto node = make_node(cfg, "vbus", "jr_bus_modecmd");
    JR_CHECK(configure(node));
    JR_CHECK(activate(node));

    auto driver = std::make_shared<rclcpp::Node>("jr_test_driver_mode");
    const std::string ns = std::string("/") + node->get_name();
    auto pub_mode = driver->create_publisher<jr_interfaces::msg::JointCommandArray>(
        ns + "/cmd", rclcpp::QoS(1).reliable());
    auto pub_mit = driver->create_publisher<jr_interfaces::msg::MitCommandArray>(
        ns + "/cmd_mit", rclcpp::QoS(1).reliable());

    std::atomic<std::uint32_t> rejected_seen{0u};
    auto sub_stats = driver->create_subscription<jr_interfaces::msg::RtStats>(
        ns + "/rt_stats", rclcpp::QoS(10).best_effort(),
        [&rejected_seen](jr_interfaces::msg::RtStats::ConstSharedPtr m) {
            if (m->cmd_rejected > rejected_seen.load()) rejected_seen.store(m->cmd_rejected);
        });

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2u, false);
    exec.add_node(node->get_node_base_interface());
    exec.add_node(driver);
    std::thread spin_thread([&exec]() { exec.spin(); });

    /* 等使能真的落下来（auto_enable=true 也需要几个周期）。 */
    const bool en = wait_for(
        [&]() {
            const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
            return s != nullptr && s->joints[0].enabled && s->joints[0].cmd_mode == 8u;
        },
        3000u);
    JR_CHECK_MSG(en, "j0 必须以 **CSP** 模式使能（配置 joints[].mode: csp 要真的生效）");

    /* ① CSP 命令：位置必须真的走到目标 */
    double max_pos = 0.0;
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
        jr_interfaces::msg::JointCommandArray arr;
        jr_interfaces::msg::JointCommand c;
        c.name = "j0";
        c.mode = jr_interfaces::msg::JointCommand::CSP;
        c.target = 0.25;
        arr.commands.push_back(c);
        pub_mode->publish(arr);
        std::this_thread::sleep_for(20ms);
        const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
        if (s != nullptr) max_pos = std::max(max_pos, s->joints[0].position);
        if (max_pos > 0.20) break;
    }
    JR_CHECK_MSG(max_pos > 0.20, "~/cmd 的 CSP 命令必须真的把关节控到 0.25 rad（>0.20 才说明闭环）");
    dump_snapshot(node, "after csp command");

    /* ② 模式不符：给 CSP 的 j0 发 **MIT** 模式的 ~/cmd → 必须被拒并计数 */
    const std::uint32_t rejected_before = rejected_seen.load();
    {
        for (int i = 0; i < 5; ++i) {
            jr_interfaces::msg::JointCommandArray arr;
            jr_interfaces::msg::JointCommand c;
            c.name = "j0";
            c.mode = 4u;   /* MIT（该关节是 CSP → 必须被拒） */
            c.target = -0.9;
            arr.commands.push_back(c);
            pub_mode->publish(arr);
            std::this_thread::sleep_for(20ms);
        }
    }
    const bool counted = wait_for([&]() { return rejected_seen.load() > rejected_before; }, 2000u);
    JR_CHECK_MSG(counted,
                 "模式不符的 ~/cmd 必须让 `rt_stats.cmd_rejected` 增长 —— 否则客户只能看到"
                 "『关节不动』而拿不到任何线索");
    {
        const StateSnapshot *s = node->tick_group_for_test()->acquire_snapshot();
        JR_CHECK(s != nullptr);
        if (s != nullptr) {
            JR_CHECK_MSG(s->joints[0].position > 0.15,
                         "被拒的 MIT 目标不得把 CSP 关节拖走（最后有效目标仍然有效）");
        }
    }

    /* ③ 反方向也要守：给 CSP 关节发 ~/cmd_mit → 节点侧就拒（不静默按 MIT 发） */
    const std::uint32_t rejected_before2 = rejected_seen.load();
    {
        for (int i = 0; i < 5; ++i) {
            jr_interfaces::msg::MitCommandArray arr;
            jr_interfaces::msg::MitCommand c;
            c.name = "j0";
            c.gain_mode = jr_interfaces::msg::MitCommand::SI;
            c.position = -0.9;
            c.stiffness = 30.0;
            c.damping = 1.0;
            arr.commands.push_back(c);
            pub_mit->publish(arr);
            std::this_thread::sleep_for(20ms);
        }
    }
    const bool counted2 =
        wait_for([&]() { return rejected_seen.load() > rejected_before2; }, 2000u);
    JR_CHECK_MSG(counted2, "给非 MIT 关节发 ~/cmd_mit 也必须计数（两个方向都要守）");

    exec.cancel();
    spin_thread.join();
    deactivate(node);
    cleanup(node);
}

/* ==========================================================================
 * ⑦ 坏配置：master_id = 0 必须拒绝启动并说清原因
 * ======================================================================== */

void test_invalid_master_id_is_rejected()
{
    JR_CASE("非法 master_id（0）→ 拒绝启动，且节点不进入 configured（§10.2）");

    const std::string cfg = write_config(kCfgPath, false, /*master_id=*/0u, 100u, 500u);
    auto node = make_node(cfg, "vbus", "jr_bus_badid");
    const auto st = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    JR_CHECK_MSG(st.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                 "master_id=0 必须让 configure 失败（设备会完全不回复，不能带病启动）");
    JR_CHECK(node->tick_group_for_test() == nullptr);
    cleanup(node);   /* configure 失败也要走到 finalized（否则析构时报 not shut down） */
}

/* ==========================================================================
 * ④ 总线名写错 → 拒绝并列出已知总线（可操作性）
 * ======================================================================== */

void test_unknown_bus_is_actionable()
{
    JR_CASE("配置里没有这个总线名 → 拒绝启动（不是静默跑一条空总线）");

    const std::string cfg = write_config(kCfgPath, false, 1u, 100u, 500u);
    auto node = make_node(cfg, "nosuchbus", "jr_bus_nobus");
    const auto st = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    JR_CHECK_EQ(static_cast<int>(st.id()),
                static_cast<int>(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
    cleanup(node);
}

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    test_cold_start_sends_no_control_frames();
    test_command_path_and_timeout_visibility();
    test_service_gates_and_per_joint_enable();
    test_broadcast_degradation_is_visible();
    test_diagnostics_are_published();
    test_mode_command_topic();
    test_invalid_master_id_is_rejected();
    test_unknown_bus_is_actionable();
    rclcpp::shutdown();
    std::remove(kCfgPath);
    return jrtest::report();
}
