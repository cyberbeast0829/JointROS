/**
 * @file    jr_config.hpp
 * @brief   核心库配置模型（**不依赖 rclcpp**；由 ROS 节点从参数/YAML 填充）
 *
 * @par 为什么核心库不自带 YAML 解析
 *  - 参数解析（文件 → 结构体）是**启动期**的事，与实时性无关；
 *  - 把 YAML/CLI 解析拉进核心库会立刻引入第三方依赖，并让"同一份配置有多种来源"
 *    的校验逻辑分散在两处（ROS 参数、工具命令行）→ 现在统一在这里做**校验**。
 *    来源可以是 ROS 参数、YAML、命令行、单元测试直接构造，`validate_config()` 是唯一闸门。
 */

#ifndef JR_ROS2_JR_CONFIG_HPP
#define JR_ROS2_JR_CONFIG_HPP

#include <cstdint>
#include <cstring>

#include "jr_ros2/jr_status.hpp"

/**
 * 每总线关节数上限。**必须**与 SDK 侧的 `JSDK_MAX_JOINTS_STATIC` 一致
 * （CMake 会把两个宏一起注入，见 `cmake/jr_sdk.cmake`）。
 */
#ifndef JR_MAX_JOINTS_PER_BUS
#  define JR_MAX_JOINTS_PER_BUS 16u
#endif

namespace jr {

inline constexpr unsigned kMaxJointsPerBus = JR_MAX_JOINTS_PER_BUS;
inline constexpr unsigned kMaxBuses        = 8u;
inline constexpr unsigned kMaxTickGroups   = 4u;
inline constexpr unsigned kMaxJoints       = kMaxJointsPerBus * kMaxBuses;
inline constexpr unsigned kJointNameLen    = 64u;
inline constexpr unsigned kBusNameLen      = 32u;
inline constexpr unsigned kChannelLen      = 128u;
inline constexpr unsigned kPathLen         = 192u;
inline constexpr unsigned kNotesLen        = 640u;

/** 协议硬限制：广播位图只能寻址 node_id 1..7（见 DESIGN §2.2）。 */
inline constexpr std::uint8_t kMaxBroadcastNodeId = 7u;

enum class HalKind : std::uint8_t {
    kVirtual = 0,   /**< SDK 内置设备模型（CI/演示；无硬件） */
    kSocketCan,     /**< Linux SocketCAN（生产） */
    kSlcan,         /**< 串口 slcan（CANable 等；仅调试，帧率低） */
    kPcan           /**< PEAK PCAN-Basic（Windows/macOS） */
};

/** `HalKind` 的稳定文本（**与 YAML 的 `type:` 取值逐字一致**）。
 *  为什么放在这里：工具（`jr_gen_config` 的 YAML 输出、`jr_hw_verify` 的打印）与将来
 *  `jr_ctl` 都要它，各写一份必然漂移（改了枚举漏改一处 → 配置文件与代码说的不是同一件事）。 */
inline const char *to_string(HalKind h) noexcept
{
    switch (h) {
    case HalKind::kVirtual: return "virtual";
    case HalKind::kSocketCan: return "socketcan";
    case HalKind::kSlcan: return "slcan";
    case HalKind::kPcan: return "pcan";
    }
    return "?";
}

/**
 * `type:` 文本 → `HalKind`；**不认识就返回 false**（绝不猜一个后端）。
 *
 *  与 `load_config_yaml()` 同义（那边用 `need_enum` 拒绝未知值，取空值才走默认）——
 *  这里只是给命令行/TUI 复用的同一份映射，不另立规矩。 */
inline bool hal_kind_from_name(const char *name, HalKind *out) noexcept
{
    if (name == nullptr || out == nullptr) return false;
    if (std::strcmp(name, "virtual") == 0) { *out = HalKind::kVirtual; return true; }
    if (std::strcmp(name, "socketcan") == 0) { *out = HalKind::kSocketCan; return true; }
    if (std::strcmp(name, "slcan") == 0) { *out = HalKind::kSlcan; return true; }
    if (std::strcmp(name, "pcan") == 0) { *out = HalKind::kPcan; return true; }
    return false;
}

enum class FeedbackPolicy : std::uint8_t {
    kBroadcastHeartbeat = 0, /**< 广播下发 + 心跳反馈（可选轮询；默认） */
    kHeartbeatOnly,          /**< 广播下发 + 仅心跳（最省带宽） */
    kUnicastPoll,            /**< 广播下发 + 轮转单播轮询（每关节 rate/N 的 MIT 反馈） */
    kUnicastOnly             /**< 全单播（每关节每 tick 都有 MIT 反馈；最贵） */
};

enum class TimeoutAction : std::uint8_t {
    kHold = 0,   /**< PD 锁位（默认；用 once 的 kp/kd，或 joint 的 stiffness/damping） */
    kZeroTorque, /**< 泄力（MIT: kp=kd=tau=0，目标 = 当前位置） */
    kDisable,    /**< 走安全失能序列 */
    kEstop       /**< 广播全总线急停（会打到线上所有节点） */
};

enum class ExitAction : std::uint8_t { kDisable = 0, kHold, kZeroTorque, kEstop, kNone };
enum class FaultAction : std::uint8_t { kNone = 0, kDisableJoint, kEstopBus };
enum class DescRetain : std::uint8_t { kAll = 0, kFiltered };

/**
 * 参考轨迹插值（DESIGN §6.4 的 `command.interpolation`）。
 *
 * @par 为什么需要它
 *  控制器常常跑得比 CAN tick 慢（例：控制器 100 Hz、tick 1 kHz）。`none` 下目标
 *  每 10 ms **跳**一次；`linear` 让核心在 tick 上把目标从旧值**线性推进**到新值，
 *  相当于把控制器输出上采样到 tick 率（波形更平滑、jerk 更低）。
 *
 * @par 它插什么、不插什么（写下来以免各人理解不同）
 *  - **插**：该模式承载的**运动量** —— MIT/CSP 的 `position`、CSV 的 `velocity`、
 *    CST 的 `torque`、CURRENT 的 `current`；
 *  - **不插**：增益（`kp/kd` / `stiffness/damping`）与限制量 —— 它们按最新值**立即**生效。
 *    理由：增益不是“轨迹”，把刚度缓缓推上去等于没人要求地改变闭环特性。
 *  - 第一条命令**立即生效**（还没有历史区间可插，从 0 慢慢爬上去是安全问题）。
 */
enum class Interpolation : std::uint8_t {
    kNone = 0,  /**< 原样下发（默认） */
    kLinear     /**< 在 tick 上线性过渡到新目标 */
};

/** 单个关节的静态配置（量程/kp 上限等**从设备读**，不在这里填）。 */
/**
 * 命令模式（ADR-4 的二级命令接口）。
 *
 * ⚠ 取值必须与 `jsdk_mode_t`（JointSDK）以及 `jr_interfaces/JointCommand.msg` 里的
 *   `CSP/CSV/CST/CURRENT` **逐个一致** —— 两处各有一组静态断言守着（改一处不改另一处就编译期红）：
 *   `jr_bus_runtime.cpp`（对 SDK）与 `test_jr_bus_node.cpp`（对消息常量）。
 *
 * 各模式的**单位口径**（换算责任在 SDK 侧，本包逐字透传）：
 *   CSP     输出端 rad、CSV 输出端 rad/s、CST 输出端 N·m、CURRENT **电机端** A。
 */
enum class CmdMode : std::uint8_t {
    kMit     = 4,  /**< MIT 力位混合（一级接口，默认） */
    kCsp     = 8,  /**< → POS_CONTROL(0x01)：目标 = 输出端位置 rad */
    kCsv     = 9,  /**< → VEL_CONTROL(0x02)：目标 = 输出端速度 rad/s */
    kCst     = 10, /**< → TORQUE_CONTROL(0x03)：目标 = 输出端力矩 N·m */
    kCurrent = 11, /**< → CURRENT_CONTROL(0x04)：目标 = **电机端**电流 A（逃生通道） */
};

/** 配置/日志用的稳定文本（小写，与 YAML 键值一致）。 */
inline const char *to_string(CmdMode m) noexcept
{
    switch (m) {
    case CmdMode::kMit: return "mit";
    case CmdMode::kCsp: return "csp";
    case CmdMode::kCsv: return "csv";
    case CmdMode::kCst: return "cst";
    case CmdMode::kCurrent: return "current";
    }
    return "?";
}

/* 下面三个同样是“与 YAML 取值逐字一致”的文本。
   ⚠ 为什么必须有：写出配置的工具（`jr_gen_config`）不能手打这些字符串 ——
     写错一个字母，生成的配置就会在客户机器上“加载失败”，而现场看到的是一句
     与生成器毫无关联的报错。**生成器的输出与加载器的输入必须来自同一份字面量**
     （由 `jr_gen_config` 的“生成→回读”用例守着：两边漂了就红）。 */
inline const char *to_string(TimeoutAction a) noexcept
{
    switch (a) {
    case TimeoutAction::kHold: return "hold";
    case TimeoutAction::kZeroTorque: return "zero_torque";
    case TimeoutAction::kDisable: return "disable";
    case TimeoutAction::kEstop: return "estop";
    }
    return "?";
}

inline const char *to_string(ExitAction a) noexcept
{
    switch (a) {
    case ExitAction::kDisable: return "disable";
    case ExitAction::kHold: return "hold";
    case ExitAction::kZeroTorque: return "zero_torque";
    case ExitAction::kEstop: return "estop";
    case ExitAction::kNone: return "none";
    }
    return "?";
}

inline const char *to_string(FaultAction a) noexcept
{
    switch (a) {
    case FaultAction::kNone: return "none";
    case FaultAction::kDisableJoint: return "disable_joint";
    case FaultAction::kEstopBus: return "estop_bus";
    }
    return "?";
}

inline const char *to_string(Interpolation i) noexcept
{
    switch (i) {
    case Interpolation::kNone: return "none";
    case Interpolation::kLinear: return "linear";
    }
    return "?";
}

/** 该模式的目标是「位置/速度/力矩/电流」中的哪一种（用于校验与文档，不参与发帧）。 */
enum class CmdQuantity : std::uint8_t { kNone, kPosition, kVelocity, kTorque, kCurrent };

inline CmdQuantity quantity_of(CmdMode m) noexcept
{
    switch (m) {
    case CmdMode::kMit: return CmdQuantity::kNone;      /* MIT 是混合的：位/速/力矩都可给 */
    case CmdMode::kCsp: return CmdQuantity::kPosition;
    case CmdMode::kCsv: return CmdQuantity::kVelocity;
    case CmdMode::kCst: return CmdQuantity::kTorque;
    case CmdMode::kCurrent: return CmdQuantity::kCurrent;
    }
    return CmdQuantity::kNone;
}

struct JointCfg {
    char     name[kJointNameLen] = {};
    std::uint8_t node_id = 0u;
    /**
     * 该关节使能时使用的模式（默认 MIT）。
     *
     * ⚠ **模式切换是配置项，不是运行期动作**：SDK 的 `set_mode()` 只是改一个字段，
     *   下一次 `request_enable()` 才生效；而在使能状态下改模式会让编码器**当场**换帧型
     *   （等效于一次未受保护的模式切换）。所以模式在启动时定下，运行期只接受
     *   “与当前模式一致”的命令（见 `JointTarget.mode` 的校验）。
     */
    CmdMode  mode = CmdMode::kMit;
    bool     has_position_limit = false;
    double   position_min = 0.0;   /**< 仅当 has_position_limit 时有效；用于**更紧**的软限位 */
    double   position_max = 0.0;
    double   stiffness = 0.0;      /**< 命令超时 hold 用（N·m/rad）；0 = 用上一次的 kp */
    double   damping = 0.0;        /**< 同上（N·m·s/rad） */
};

/** 描述符获取与缓存。 */
struct DescCfg {
    bool     cache_enabled = true;
    char     cache_dir[kPathLen] = {};   /**< 空 = 调用方/节点填默认（~/.cache/jr） */
    std::uint32_t timeout_ms = 5000u;
    std::uint32_t retries = 3u;          /**< 真机（slcan）首帧易丢，必须重试 */
    std::uint32_t retry_backoff_ms = 100u;
    /**
     * 默认 `kAll`：宿主（Linux/ROS）上 ~25 KB 内存换"任意端点可读可写"，
     * 这是我们对客户的主要卖点之一（换固件不用等我们发版）。
     * RAM 敏感（MCU/低配 SoC）时改 `kFiltered` + filter_paths。
     */
    DescRetain   retain = DescRetain::kAll;
    const char *const *filter_paths = nullptr;
    unsigned     filter_count = 0u;
};

/** 一条总线（对应一个 SDK context）。 */
struct BusCfg {
    char         name[kBusNameLen] = {};
    HalKind      hal = HalKind::kSocketCan;
    /** SocketCAN: "can0"；PCAN: "PCAN_USBBUS1"；slcan: "COM3"/"/dev/ttyACM0"；
        virtual: 设备模型规格串（见 SDK `jsdk_hal_virtual_open`）。 */
    char         channel[kChannelLen] = {};
    std::uint32_t serial_baud = 0u;       /**< slcan 专用（串口速率，不是 CAN 速率） */

    std::uint8_t master_id = 1u;          /**< 1..254；**禁止 0**（设备将完全不回复） */
    /** 对端用的是 CAN FD 吗。⚠ 这是**猜测/起点**，不是断言：协议没有运行时协商。
        默认 **false（Classic 起步）** —— 与 SDK 的推荐组合一致（`is_fd_explicit = 0`）：
        FD 控制器**也收**经典帧、反之不成立，所以先按 Classic 发最安全，SDK 会在收到本关节
        第一帧时自动对齐到对端格式并如实报告（§13.3-46）。
        ⚠ 以前默认 `true` 且从不设 `is_fd_explicit` ⇒ 在 **Classic** 设备上所有路径都以 FD 发帧，
        现场症状只是“收得到心跳、我的请求没人应”（描述符下载 `0/0 bytes` 卡死）——
        真机实测，且我们的工具因此完全用不了（§13.3-46）。 */
    bool         is_fd = false;
    std::uint32_t nominal_bitrate = 1000000u;
    std::uint32_t data_bitrate = 5000000u; /**< is_fd=false 时忽略 */

    unsigned     joint_count = 0u;
    JointCfg     joints[kMaxJointsPerBus] = {};

    std::uint32_t state_timeout_ms = 0u;  /**< 0 = 用 SDK 默认（标定 120 s / 回零 5 s） */
    bool         auto_keepalive = true;   /**< ↔ SDK cfg.auto_keepalive */
    bool         clamp_target = false;    /**< ↔ SDK clamp_target_position（false = 拒绝+安全帧） */
    bool         arm_device_watchdog = false; /**< ↔ SDK enable_watchdog_hint（默认不动客户设备） */
    std::uint8_t rx_burst_limit = 0u;     /**< 0 = SDK 默认 32 */

    FeedbackPolicy feedback = FeedbackPolicy::kBroadcastHeartbeat;
    std::uint32_t  heartbeat_ms = 5u;     /**< 建议的设备心跳周期（0 = 不用心跳反馈） */
    std::uint32_t  poll_period_ms = 10u;  /**< 轮询反馈周期（0 = 不轮询） */

    double         max_bus_load = 0.60;   /**< 启动预算检查阈值（超 → 拒绝；0.6~0.8 → 警告） */

    DescCfg        desc = {};
};

/** 一个 tick 组 = 一个 RT 线程 + 一个时间基准（可含多条总线）。 */
struct TickGroupCfg {
    char          name[kBusNameLen] = {};
    std::uint32_t rate_hz = 1000u;
    unsigned      bus_index[kMaxBuses] = {};
    unsigned      bus_count = 0u;
    int           cpu = -1;        /**< -1 = 不绑核（覆盖 RtCfg.cpu） */
    int           priority = -1;   /**< -1 = 用 RtCfg.priority */
};

struct RtCfg {
    bool enabled = true;           /**< false = 纯 CFS（调试；抖动不保证） */
    int  priority = 80;            /**< SCHED_FIFO 优先级 */
    bool mlock = true;             /**< mlockall / VirtualLock */
    int  cpu = -1;                 /**< 默认绑核；-1 = 不绑 */
    bool warn_if_throttled = true; /**< 检查 sched_rt_runtime_us / 高精度定时器 */
    bool deadline_policy = false;  /**< P2：SCHED_DEADLINE（当前未实现，置位即报 kNotSupported） */
};

struct CommandCfg {
    std::uint32_t timeout_ms = 100u;   /**< 0 = 关闭命令超时闸（**不推荐**，会在诊断里持续告警） */
    TimeoutAction on_timeout = TimeoutAction::kHold;
    /** 插值开关（**核心库**在 tick 上执行；节点与 ros2_control 共用）。 */
    Interpolation interpolation = Interpolation::kNone;
    double        hold_stiffness = 0.0; /**< hold 的默认刚度（0 = 沿用该关节上次命令的 kp 语义） */
    double        hold_damping = 0.0;
};

struct FaultAutoResetCfg {
    bool          enabled = false;   /**< 默认关：不"自动修好"（ADR-13） */
    unsigned      max_attempts = 1u;
    std::uint32_t backoff_ms = 1000u;
};

struct SafetyCfg {
    bool          auto_enable = false;        /**< 冷启动不使能（G6） */
    bool          require_calibrated = true;  /**< 未标定 → 拒绝使能并提示 */
    bool          allow_param_write = false;  /**< 参数写服务总闸（ADR-8） */
    bool          allow_flash_persist = false;/**< Flash 持久化总闸 */
    ExitAction    on_exit = ExitAction::kDisable;
    FaultAction   on_fault = FaultAction::kNone;
    FaultAutoResetCfg fault_auto_reset = {};
};

struct LockCfg {
    bool enabled = true;
    char lock_dir[kPathLen] = {};  /**< 空 = 平台默认（Linux /var/lock、Windows %LOCALAPPDATA%） */
    bool allow_shared = false;     /**< 显式并行（诊断里持续告警） */
};

struct Config {
    RtCfg        rt = {};
    BusCfg       buses[kMaxBuses] = {};
    unsigned     bus_count = 0u;
    TickGroupCfg groups[kMaxTickGroups] = {};
    unsigned     group_count = 0u;
    CommandCfg   command = {};
    SafetyCfg    safety = {};
    LockCfg      lock = {};
};

/** `validate_config()` 的"非致命但必须让用户看到"的结论。 */
struct ConfigNotes {
    bool has_joint_above_broadcast_id = false; /**< 有 node_id > 7 → 该关节无法参与广播同步 */
    bool has_classic_bus = false;              /**< Classic：带宽/描述符代价高，生产应避免 */
    bool has_slcan_bus = false;                /**< slcan：只适合配置/调试 */
    bool command_timeout_disabled = false;      /**< command.timeout_ms == 0 */
    bool rt_disabled = false;                  /**< rt.enabled == false */
    bool has_non_mit_joint = false;            /**< 有关节配了非 MIT 模式（命令面要改用 ~/cmd） */
    bool has_current_mode_joint = false;        /**< 有 CURRENT 模式关节（F19：不喂设备看门狗） */
    char text[kNotesLen] = {};                 /**< 逐行可读提示（追加式） */
};

/** 平台相关的默认值（SocketCAN vs PCAN），其余字段走结构体默认。 */
Config default_config() noexcept;

/** 只填一条总线的默认值（供节点/工具逐个构造）。 */
BusCfg default_bus_cfg() noexcept;

/**
 * 校验配置。**任何**非法项都返回非 kOk（不猜测、不放行）。
 * 通过时把"需要让用户看到但不必拒绝"的结论写入 `notes`（可为 nullptr）。
 */
Result validate_config(const Config &cfg, ConfigNotes *notes) noexcept;

/** 按名字找总线（找不到返回 nullptr）。 */
const BusCfg *find_bus(const Config &cfg, const char *name) noexcept;

/** 按名字找关节：返回全局关节索引（-1 = 未命中），并可选输出总线索引与"总线内序号"。 */
int find_joint(const Config &cfg, const char *name,
               unsigned *bus_index = nullptr, unsigned *local_index = nullptr) noexcept;

/** 全局关节索引的第一个/最后一个（供遍历）。 */
unsigned total_joint_count(const Config &cfg) noexcept;

/** 某总线第一个关节的全局索引。 */
unsigned bus_joint_base(const Config &cfg, unsigned bus_index) noexcept;

/**
 * 单 master 锁的**键**（F7）：锁按**物理通道**取，不按总线名。
 *
 * 以前键 = 总线名 ⇒ 同一个 `/dev/ttyACM0` 上写两份配置（`name:` 不同）**不会互斥**，
 * 两个 master 会真的同时上同一条总线（而 `jr_bus_lock.hpp` 的注释一直宣称那是"通道的排他锁"）。
 *
 * 规则（⚠ 两个方向都要对，测试里两个方向都有断言）：
 *  - `socketcan`/`slcan`/`pcan` ⇒ `"<hal>:<channel>"`（如 `slcan:/dev/ttyACM0`）—— 要争的是物理资源；
 *  - `virtual` ⇒ `"virtual:<bus name>"`：每个进程各有**自己的**仿真设备（`humanoid_2bus` 示例里
 *    两条 virtual 总线的 `spec:` **完全相同**也不共享任何东西）⇒ 若把它们互斥，演示与 CI 全卡死；
 *  - `channel` 为空（配置校验本该拦住）⇒ 退回总线名，免得退化成"所有总线共用一个键"。
 *
 * ⚠ 键会被拼成 `/var/lock/jr-<键>.lock`（`BusLock` 内部做文件名清洗，`/` 会变 `_`）。
 *   改键会换文件名 ⇒ 旧的 `jr-<总线名>.lock` 成为**无害的历史残留**（锁是 OS 对文件的锁，
 *   空文件本身不持锁），不需要清理脚本，但文档要说清。
 */
void lock_key(const BusCfg &bus, char *out, std::size_t cap) noexcept;

}  // namespace jr

#endif /* JR_ROS2_JR_CONFIG_HPP */
