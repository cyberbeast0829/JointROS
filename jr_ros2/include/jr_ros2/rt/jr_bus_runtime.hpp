/**
 * @file    jr_bus_runtime.hpp
 * @brief   一条总线的运行时（= 一个 `jsdk_context` 的所有者）
 *
 * @par 职责边界
 *   - **拥有** SDK context 的生命周期（配置阶段）与 tick 期间的调用权；
 *   - 对外只暴露两类接口：**非 RT**（open/configure/activate/close，可阻塞）
 *     与 **RT**（tick_begin / apply_command / tick_end，无锁无分配）；
 *   - 不碰 ROS、不建线程（线程由 `TickGroup` 管）。
 *
 * @par 为什么 context 存储要用运行时尺寸
 *  `JSDK_CONTEXT_MAX_SIZE`/`JSDK_MAX_JOINTS_STATIC` 是**编译期**常量，会随编译选项变化。
 *  用 `jsdk_context_size()` 的返回值精确分配，可以彻底避免"库与头文件常量不一致 →
 *  静默写溢出调用者的静态存储"这类最危险的回归（DESIGN ADR-12）。
 *  另外：`jsdk_context_config_t` 必须**活到 context 销毁**（SDK 会把
 *  `&cfg.desc.arena_used` 存下来回写），所以它是本类的成员，本类因此不可拷贝/移动。
 */

#ifndef JR_ROS2_RT_JR_BUS_RUNTIME_HPP
#define JR_ROS2_RT_JR_BUS_RUNTIME_HPP

#include <cstdint>
#include <vector>

#include "jr_ros2/jr_command.hpp"
#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_snapshot.hpp"
#include "jr_ros2/jr_status.hpp"
#include "jr_ros2/rt/jr_bus_lock.hpp"
#include "jr_ros2/rt/jr_bus_plan.hpp"
#include "jr_ros2/rt/jr_desc_cache.hpp"
#include "jr_ros2/rt/jr_ops.hpp"
#include "jr_ros2/rt/jr_spsc_ring.hpp"

/* 只前向声明 SDK 类型，头文件不把 joint_sdk.h 拖给 ROS 层（保持编译隔离）。
   ⚠ `jsdk_can_hal_t` / `jsdk_hal_handle_t` 是无 tag 的 typedef，无法前向声明 ——
   它们只出现在本文件的**不透明持有者** `HalHolder` 里（定义在 .cpp）。 */
struct jsdk_context;
struct jsdk_joint;

namespace jr {
namespace rt {

/** 总线运行时状态（DESIGN §5.4 状态机的可执行版本）。 */
enum class BusMode : std::uint32_t {
    kIdle = 0,      /**< 已初始化上下文，还不能用（未 configure） */
    kReady,         /**< configure 完成；**未使能** */
    kActive,        /**< 闭环执行中（tick 线程在发控制帧） */
    kPaused,        /**< 已安全失能 + 所有权交给非 RT 线程 */
    kFault          /**< 故障 / 不可自动恢复（需 FaultReset 或设备复位） */
};

const char *to_string(BusMode m) noexcept;

/** 诊断用：`hal_bus_flags`（`JSDK_HAL_BUS_*`）→ 短文本。
 *
 *  ⚠ 为什么要放在核心库：这些位定义是 **SDK 知识**。节点/工具不该 include `joint_sdk.h`
 *  （那样 SDK 头一变，ROS 侧整个重编），所以解码放在这里，上层只拿文本。
 *  未识别的位按十六进制保留（**不能不响**）。 */
void bus_flags_text(std::uint32_t flags, char *out, std::size_t cap) noexcept;

/** 诊断用：`JSDK_JF_*` 粘滞标志 → 短文本（未识别的位保留十六进制）。 */
void joint_status_flags_text(std::uint16_t flags, char *out, std::size_t cap) noexcept;

struct DeviceInfoPOD {
    std::uint32_t hw_version = 0u;
    std::uint32_t fw_version = 0u;
    std::uint64_t serial = 0u;
    bool          classic = false;
    bool          valid = false;
};

/** configure() 从设备读回的标定/量程（**只从设备读，绝不硬编码**）。 */
struct JointInfoPOD {
    double        gear_ratio = 0.0;
    double        mit_max_pos = 0.0;
    double        mit_max_vel = 0.0;
    double        mit_max_torque = 0.0;
    double        mit_max_kp = 0.0;
    double        mit_max_kd = 0.0;
    double        torque_constant = 0.0;
    std::uint32_t heartbeat_rate_ms = 0u;
    std::uint32_t break_timeout_ms = 0u;   /**< 0 = 设备侧协议超时**已禁用** */
    std::uint32_t node_id = 0u;
    bool          config_valid = false;    /**< 标定量程可用 */
    bool          calibrated = false;      /**< 物理量 API 可用（unit_scale.valid） */
};

struct DescInfoPOD {
    std::uint32_t total_len = 0u;
    std::uint32_t json_bytes = 0u;   /**< 我们捕获到的原始 JSON 字节数（路线 B 缓存用） */
    std::uint16_t crc = 0u;
    std::uint32_t fw_version = 0u;
    unsigned      endpoint_count = 0u;
    unsigned      parsed_total = 0u;
    unsigned      frames_rx = 0u;
    bool          complete = false;
    bool          shared_hit = false;
    bool          from_cache = false;
};

/**
 * 导入描述符时**必需**的元信息（对应 SDK 的 `jsdk_desc_hint_t`）。
 *
 * ⚠ 为什么必需：SDK 的 `jsdk_context_desc_import_raw()` 把 hint 当成硬前提
 *   （`!hint` ⇒ `JSDK_ERR_INVALID_ARG`）：CRC / fw 是**设备侧属性**，从 JSON 正文里推不出来。
 *   值可以是 0（= 未知 / 来自缓存），但**结构本身不能省**。
 *   v0.14 我们就是传了 `nullptr` ⇒ “导出→导入”**永远失败**，而且我们的错误文案
 *   把原因猜成了“payload 不是原始 JSON”（§13.4 那条误判的根因，§13.3-38）。
 */
struct DescHintPOD {
    std::uint16_t crc = 0u;         /**< 设备给的描述符 VersionCRC（`ExportDescriptor.crc` 原样带回） */
    std::uint32_t fw_version = 0u;  /**< 设备固件版本（`ExportDescriptor.fw_version` 原样带回） */
};

/** 故障边沿事件（RT 侧只入环，非 RT 侧发布）。 */
struct FaultEvent {
    std::uint64_t t_ns = 0u;
    std::uint8_t  node_id = 0u;
    std::uint8_t  event = 0u;      /**< 1 = 出现，2 = 清除 */
    std::uint8_t  err_code = 0u;   /**< MIT 4-bit（**摘要**，别当结论） */
    std::uint8_t  hb_error = 0u;   /**< 心跳 5-bit（原始位） */
    std::uint32_t axis_error = 0u;
    std::uint32_t motor_error = 0u;
    std::uint32_t encoder_error = 0u;
    std::uint32_t sensorless_error = 0u;
    std::uint32_t controller_error = 0u;
    std::uint32_t system_error = 0u;
    char          text[96] = {};   /**< SDK 的一行描述（排障入口） */
};

/** 配置期的完整体检报告（节点会把它变成日志/诊断/服务响应）。 */
struct BusReport {
    DeviceInfoPOD device[kMaxJointsPerBus] = {};
    JointInfoPOD  joint[kMaxJointsPerBus] = {};
    DescInfoPOD   desc = {};
    BusPlanResult plan = {};
    bool          abi_checked = false;
    bool          all_calibrated = false;  /**< 全部关节是否已标定（供节点/诊断直接取用） */
    char          text[768] = {};   /**< 逐行结论（含告警与建议） */
};

struct OpenOptions {
    bool        check_abi = true;
    bool        enable_lock = true;
    bool        allow_shared_lock = false;
    const char *lock_dir = nullptr;

    /**
     * 允许“**发现用**的空总线”打开（`bus.joint_count == 0`）。
     *
     * 默认 false：正常路径（节点/ros2_control）里“0 个关节”一定是配置写错了，
     * 应该**拒绝启动**（而且预算模型也需要关节列表才能算）。
     * 只有“先问总线、后填配置”的工具（`jr_gen_config`）才置 true —— 那时关节数
     * 本来就是未知的，所以跳过预算检查并**在 note 里写明“没查”**（不假装查过）。
     */
    bool        allow_empty_scan_bus = false;
};

class BusRuntime {
public:
    BusRuntime() noexcept;
    ~BusRuntime();

    BusRuntime(const BusRuntime &) = delete;
    BusRuntime &operator=(const BusRuntime &) = delete;

    /* ---------------- 非 RT 路径（配置阶段） ---------------- */

    /**
     * 取锁 → 开 HAL → 初始化 context → 加关节。
     * @param period_ns tick 周期（用于 SDK 的 keepalive/超时判定与总线预算检查）
     */
    Status open(const BusCfg &bus, std::uint32_t period_ns, const OpenOptions &opt,
                Result *res) noexcept;

    /** 描述符（缓存优先，可配重试）+ `configure()` + 读回量程 + 多项一致性检查。 */
    Status configure(BusReport *report, Result *res) noexcept;

    /** 使能全部关节（阻塞：SDK 内部自己跑周期）。**必须**在 tick 线程未持有 context 时调用。 */
    Status activate(bool require_calibrated, Result *res) noexcept;

    /** 安全失能（幂等）。 */
    Status deactivate(Result *res) noexcept;

    /** 故障复位（不动电机的 CLEAR_ERRORS 路径）。 */
    Status fault_reset(Result *res) noexcept;

    /** 按退出策略收尾并释放全部资源（幂等，可在任何状态调用）。 */
    void close(ExitAction action, Result *res) noexcept;

    /** 设置参考轨迹插值（`cfg.command.interpolation`；由 `TickGroup::init()` 统一接进来）。
     *  切到 `kNone` 会**同时清掉插值状态**（下一条件立即生效，不残留半段轨迹）。 */
    void set_interpolation(Interpolation i) noexcept;

    /** 打印当前设备侧关键配置（心跳/超时）到 `report.text`（只读，不修改设备）。 */
    void refresh_device_notes(BusReport *report) noexcept;

    /* ---------------- 运维 / 参数（非 RT；仅 READY / PAUSED） ----------------
       ⚠ 全部会阻塞；且 SDK 禁止在关节使能时做描述符/参数/标定类操作。
       节点必须先走 `TickGroup::pause()`（先安全失能再交出所有权），完成后 `resume()`。 */

    /** 批量读参数（同一关节的多条在 FD 下走 SDK 的单帧批量读）。 */
    Status read_params(ParamReadItem *items, unsigned count, Result *res) noexcept;

    /**
     * 批量写参数：写后**读回校验**，逐项给出 `requested / value / verified`。
     * `confirm` 必须为 true（写闸门）；`persist=true` 时在全部写成功后追加 `CONFIG_SAVE`。
     */
    Status write_params(ParamWriteItem *items, unsigned count, bool confirm, bool persist,
                        Result *res) noexcept;

    Status lookup_endpoint(const char *path, EndpointInfo *out, Result *res) noexcept;

    /**
     * 枚举端点。`filter` 为空 = 全部；以 `*` 结尾 = 前缀匹配；以 `.` 结尾 = 段前缀；
     * 否则按**子串**匹配（与 CLI 的 `ep-list --filter` 一致）。
     */
    Status list_endpoints(const char *filter, EndpointInfo *out, unsigned cap, unsigned *count_out,
                          Result *res) noexcept;

    Status set_zero_here(Result *res) noexcept;

    /* ---- 逐关节变体（服务层用；DESIGN §6.2 的服务契约是 `string[] joints`）----
       `idx == nullptr || n == 0` = **全部关节**（与上面的整总线版本等价）。
       为什么要逐关节：客户只动一个关节时不该把整条线都失能/标定；整总线版本保留是为了
       不破坏已有调用方（ros2_control 组件、核心测试）。 */

    /** 使能/失能**选中**的关节（阻塞：内部泵周期等状态落地，并**读回**确认）。 */
    Status set_enabled(const unsigned *idx, unsigned n, bool enable, bool require_calibrated,
                       Result *res) noexcept;

    Status set_zero_here(const unsigned *idx, unsigned n, Result *res) noexcept;
    Status calibrate(const unsigned *idx, unsigned n, Result *res) noexcept;
    Status home(const unsigned *idx, unsigned n, Result *res) noexcept;
    Status save_config(const unsigned *idx, unsigned n, Result *res) noexcept;
    Status reset_device(const unsigned *idx, unsigned n, Result *res) noexcept;
    Status fault_reset(const unsigned *idx, unsigned n, Result *res) noexcept;

    /** 重新读回设备侧配置（量程/心跳/超时/标定标志）→ 刷新 `report()`。
     *  服务与诊断要用"现在的事实"，而不是 configure 那一刻的缓存。 */
    Status refresh_report(Result *res) noexcept;

    /** 描述符**原始 JSON**（路线 B：导出的就是缓存里那份字节；可能为空）。 */
    const std::vector<std::uint8_t> &descriptor_json() const noexcept { return desc_json_; }

    /** 导入描述符原始 JSON（产线预烧）。成功后描述符已换 → 需要重新 configure。 */
    /**
     * 导入**设备原始 JSON** 描述符（不经 CAN；按当前 `cfg.desc` 重新解析 ⇒ 改 filter 无需重下）。
     * ⚠ `hint` 是 SDK 的**硬前提**（不是可选项），见 `DescHintPOD`。
     * ⚠ `needs_reconfigure`：描述符被改动（成功导入 ✓）**或**被回滚（失败导入后恢复 ✓）时为 true
     *   —— 两种情况都要求调用方重新 `configure()` 重新解析（否则端点表是旧的/空的）。
     *
     * ⚠⚠ SDK 的 `import_raw()` 会**先清空 store 再解析** ⇒ 失败的导入会摧毁内存里的描述符，
     *   而且 `configure()` 不会重新下载（SDK 认为它已存在）。本函数内置**回滚**护栏（§13.3-39）。
     */
    Status import_descriptor(const void *json, std::size_t len, const DescHintPOD &hint,
                             Result *res, bool *needs_reconfigure = nullptr) noexcept;

    /** 单关节使能状态（服务结果/诊断用）。 */
    bool joint_enabled(unsigned local_index) const noexcept;

    /** 本 tick **实际交给 SDK 的目标**（= 插值之后的生效值；未收到过命令时是全 0）。
     *
     *  为什么要有它："我发的目标"与"实际下发的目标"在开启插值后**不是一回事**，
     *  而这个区别在排障时很关键（客户看到关节"没跟上"时，先要能回答"到底要求它去哪"）。
     *  它也是插值功能的**可观测面**：不必去反解线上帧格式（那是 SDK 的职责与测试领域）。
     */
    const JointTarget &applied_target(unsigned local_index) const noexcept;

    /** 单关节设备身份（诊断用；`valid == false` = 没查到）。 */
    DeviceInfoPOD device_info(unsigned local_index) const noexcept;

    /** 诊断用：该关节**当前**故障的一行描述。
     *  @return true = 有故障（`out` 已填）；false = 没故障或未实现（`out` 清零）。 */
    bool joint_fault_text(unsigned local_index, char *out, std::size_t cap) const noexcept;

    /** 诊断用：该关节的 `JSDK_JF_*` 粘滞标志原值。 */
    std::uint16_t joint_status_flags(unsigned local_index) const noexcept;

    /** 锁信息（诊断用：谁持有、锁文件在哪）。 */
    const BusLock &lock() const noexcept { return lock_; }

    /**
     * 标定（写 `axis0.requested_state = 3` 并等状态跑完，再**读回** pre_calibrated）。
     * 预算来自 `BusCfg.state_timeout_ms`（0 = SDK 默认 120 s）：SDK 的 `jsdk_joint_calibrate()`
     * 不接受超时参数，所以这里**不给**一个会说谎的入参。
     */
    Status calibrate(Result *res) noexcept;
    Status home(Result *res) noexcept;
    Status save_config(Result *res) noexcept;
    /** 软复位。成功后本总线回到 `kIdle`（必须重新 configure）。 */
    Status reset_device(Result *res) noexcept;
    Status set_node_id(unsigned local_index, std::uint8_t new_id, bool persist, Result *res) noexcept;

    /**
     * 限时点动（调试/产线翻转测试）。**必须** `confirm=true`，时长硬上限 10 s。
     * 结束时一定会走"hold → 等 2 周期 → 失能 → 等 IDLE"，**不会**留下抱力关节。
     */
    Status jog(unsigned local_index, const JointTarget &target, unsigned duration_ms, bool confirm,
               Result *res) noexcept;

    /* ---------------- RT 路径（tick 线程独占） ---------------- */

    void tick_begin(std::uint64_t now_ns) noexcept;
    /** 应用本 tick 目标（只写被 mask 的关节；未 mask 的沿用 SDK 里上一次目标）。 */
    void apply_command(const CommandSet &cmd, unsigned joint_base) noexcept;
    void tick_end(std::uint64_t now_ns) noexcept;

    /** 安全动作（RT 安全）。 */
    void estop_now() noexcept;
    /**
     * 全部关节 PD 锁位（**输出端物理量**刚度/阻尼；内部换算成 SDK 的线上 kp/kd）。
     * 传 0 则退化为 SDK 的 `hold_position()`（保持位置但不主动出力）。
     */
    void hold_all(double stiffness_nm_per_rad, double damping_nm_s_per_rad) noexcept;
    void zero_torque_all() noexcept;
    void request_disable_all() noexcept;

    /** 把本总线的状态写进快照（tick 线程序列化，读者只看完整快照）。 */
    void fill_snapshot(StateSnapshot &s, unsigned bus_index, unsigned joint_base) noexcept;

    /**
     * RT：在 `cycle_begin()`/`cycle_end()` **之间**尝试一次 MIT 广播下发。
     *
     * 返回 true = 本 tick 的命令已由 **1 条**帧发出（SDK 会给组内关节打上"已发送"标记，
     * `cycle_end()` 不会再补单播）；false = 不满足广播条件，让 SDK 走逐关节单播。
     *
     * 条件（不满足就**不假装**是广播）：node_id ≤ 7、全部已使能、全部在 MIT、
     * 且每个关节都收到过命令；Classic 链路下不做（只能"全员同一目标"，收益也小）。
     * 降级原因会写进 `note()`，由快照的 `last_note` 暴露出去（SDK 文档明确要求可见）。
     */
    bool try_broadcast_mit() noexcept;

    /** 记一条要暴露给用户/诊断的结论（写进快照的 last_note）。 */
    void set_note(const char *text) noexcept;
    const char *note() const noexcept { return snapshot_note_; }

    /* ---------------- 查询 / 诊断 ---------------- */

    BusMode mode() const noexcept { return mode_; }
    void set_mode(BusMode m) noexcept { mode_ = m; }

    const BusCfg &cfg() const noexcept { return bus_; }
    const BusReport &report() const noexcept { return report_; }
    const char *last_error() const noexcept { return last_error_; }
    bool opened() const noexcept { return ctx_ != nullptr; }
    bool active() const noexcept { return mode_ == BusMode::kActive; }

    /** 非 RT 侧消费故障事件（返回 false = 环空）。 */
    bool pop_fault(FaultEvent *out) noexcept { return faults_.pop(out); }
    std::uint64_t faults_dropped() const noexcept { return faults_.dropped(); }

    /** 最近一次 `cycle_begin/end` 的状态码（RT 侧记账用）。 */
    int last_cycle_status() const noexcept { return last_cycle_status_; }

    /** 设备侧是否已收到过 ESTOP（用于诊断/恢复建议）。 */
    bool estop_sent() const noexcept { return entered_estop_; }

    /* --- 以下三个是给 SDK 回调用的桥（回调不能访问私有成员） ---
       ⚠ 不属于稳定 API，勿在业务代码里调用。 */
    void push_fault_pub(const FaultEvent &ev) noexcept;
    void note_overflow_pub() noexcept;
    std::vector<std::uint8_t> &desc_json_mut() noexcept;

    /* 非 RT、**仅在 kReady/kPaused/kFault** 状态下使用（参数服务/诊断），
       返回裸句柄让 param 服务复用 SDK 的 SDO/参数 API。 */
    jsdk_context *raw_context() noexcept;
    jsdk_joint *raw_joint(unsigned local_index) noexcept;
    /** ⚠ 测试钩子（不属于稳定 API）：HAL 句柄，用于注入故障。
     *
     *  为什么要它：像"描述符下载被中断 → 缓存不得落盘"（§10.2 ⑥）这种断言，
     *  必须能**真的**制造一次失败的下载（虚拟后端提供了
     *  `jsdk_hal_virtual_set_tx_fail()`），否则那个守卫就只能靠"看代码"相信。 */
    void *hal_handle_for_test() const noexcept;
private:
    void close_hal() noexcept;
    void destroy_context() noexcept;
    void set_last_error(const char *fmt, ...) noexcept;
    /** 运维类操作的公共守卫：只允许在 READY / PAUSED 下执行。 */
    bool ops_window_ok(Result *res, const char *what) noexcept;

    /** 把"选中的关节"展开成下标数组（`idx==nullptr || n==0` = 全部）。
     *  返回 false = 入参非法（已填 `res`）。越界**不截断**、不猜。 */
    bool select_joints(const unsigned *idx, unsigned n, unsigned *out, unsigned *count_out,
                       Result *res) const noexcept;
    /** 把一条故障事件放进 SPSC 环（RT 安全）。 */
    bool push_fault(const FaultEvent &ev) noexcept;
    void note_append(const char *fmt, ...) noexcept;
    Status read_back_after_configure(BusReport *report, Result *res) noexcept;

    BusCfg bus_ = {};
    BusMode mode_ = BusMode::kIdle;
    std::uint32_t period_ns_ = 0u;

    /* SDK 侧对象：配置结构体必须活到 context 销毁（SDK 会回写 arena_used），
       而 SDK 的 `jsdk_context_config_t` 是无 tag 的 typedef（无法前向声明）——
       所以这里用一块**不透明内存**持有它，定义与类型转换都在 .cpp 里完成。 */
    void *sdk_cfg_mem_ = nullptr;
    jsdk_context *ctx_ = nullptr;
    void *ctx_mem_ = nullptr;
    std::size_t ctx_mem_bytes_ = 0u;
    std::vector<std::uint8_t> arena_;

    struct HalHolder;
    HalHolder *hal_ = nullptr;      /* 隐藏 jsdk_can_hal_t / jsdk_hal_handle_t 的细节 */

    jsdk_joint *joints_[kMaxJointsPerBus] = {};
    unsigned   joint_count_ = 0u;

    /**
     * 每个关节**使能时**用的模式（来自配置 `joints[].mode`，默认 MIT）。     *
     * 为什么要有这份记录：命令路径必须能回答“这条目标与当前模式一致吗”——
     * 不一致就丢掉并计数（切模式是阻塞动作，不可能在 RT 路径上做，见 JointTarget.mode）。
     * 它也是快照 `cmd_mode` 的来源（ROS 侧拿它给客户解释为什么命令没动）。
     */
    std::uint8_t joint_mode_[kMaxJointsPerBus] = {};
    /** 本 tick 实际下发的目标（插值后的生效值；`applied_target()` 的意义见头文件注释）。 */
    JointTarget   applied_[kMaxJointsPerBus] = {};
    /** 被我们丢弃的命令数（逐关节累计；快照 `cmd_rejected`，设备侧计数是另一个 `tx_rejected`）。 */
    std::uint32_t cmd_rejected_[kMaxJointsPerBus] = {};

    BusLock lock_;
    BusReport report_ = {};

    /* 描述符原始字节（路线 B 缓存用；非 RT 路径，允许分配） */
    std::vector<std::uint8_t> desc_json_;
    bool desc_json_overflow_ = false;

    /* 故障环（RT 侧入、非 RT 侧出） */
    SpscRing<FaultEvent, 64u> faults_ = {};

    int  last_cycle_status_ = 0;
    bool entered_estop_ = false;
    char last_error_[512] = {};

    /** 本 tick 的时间戳（`tick_begin()` 写入，`apply_command()` 用它算插值进度）。
     *  为什么不让插值用 `CommandSet::stamp_ns`：那个时间戳来自**ROS 域**的时钟，
     *  与 tick 的单调时钟不是一个域 —— 跨域相减会得到一个看着像"过去"的值，
     *  症状是插值永远停在 0% 或直接跳到 100%（而两边各自看起来都对）。 */
    std::uint64_t tick_now_ns_ = 0u;

    /* 线性插值（`command.interpolation=linear`）——全部 POD，RT 路径只做算术。 */
    Interpolation interp_mode_ = Interpolation::kNone;
    JointTarget   interp_from_[kMaxJointsPerBus] = {};   /**< 本段起点（上一段的终点） */
    JointTarget   interp_to_[kMaxJointsPerBus] = {};     /**< 本段终点（最新命令） */
    std::uint64_t interp_t0_ns_[kMaxJointsPerBus] = {};  /**< 本段起点时刻（tick 时钟） */
    std::uint64_t interp_dt_ns_[kMaxJointsPerBus] = {};  /**< 本段时长 = **实测**的命令间隔 */
    std::uint64_t interp_last_cmd_ns_[kMaxJointsPerBus] = {};
    std::uint64_t interp_seq_[kMaxJointsPerBus] = {};    /**< 已接收的命令代际 */
    bool          interp_active_[kMaxJointsPerBus] = {}; /**< 正在过渡 */
    bool          interp_have_[kMaxJointsPerBus] = {};   /**< 有过历史目标（首条命令不插值） */

    /* 广播下发需要"上一次目标"（SDK 的 group_set_mit 要显式的目标值，不会自己回读）。 */
    JointTarget last_target_[kMaxJointsPerBus] = {};
    bool        has_target_[kMaxJointsPerBus] = {};
    /** 最近一次拿到**非陈旧**反馈的时刻（tick 时钟；0 = 从未拿到过）。
     *  为什么自己记：见 `feedback_age_ms()` 的注释。 */
    std::uint64_t last_fresh_ns_[kMaxJointsPerBus] = {};
    char        snapshot_note_[160] = {};
};

/**
 * 反馈"年龄"（毫秒）—— **我们自己的口径**，不直接转发 SDK 的 `age_ms`。
 *
 * ⚠ 为什么不能直接转发（v0.17 真机）：SDK 在**同一个反馈里**一边报
 *   `JSDK_JF_FEEDBACK_STALE`（头文件原话：“反馈超时（**age_ms 超阈值**）”），
 *   一边报 `age_ms = 0` —— 自相矛盾。真机上实测就是这样（`status_flags=8` + `age_ms=0`），
 *   于是**一个冻结在早先时刻的值看起来“刚刚才更新”**。宁可我们自己算：
 *
 *  - 不陈旧 ⇒ 返回 SDK 的 `age_ms`（那是**设备侧采样**的年龄，比我们更有意义）；
 *  - 陈旧 且从未见过新鲜帧 ⇒ `kFeedbackAgeUnknown`（不知道，别编）；
 *  - 陈旧 且有历史 ⇒ `now - last_fresh_ns`（饱和到 `kFeedbackAgeStaleCap`）。
 */
constexpr std::uint32_t kFeedbackAgeUnknown = 0xFFFFFFFFu;  /**< "未知 / 从未新鲜过" */
constexpr std::uint32_t kFeedbackAgeStaleCap = 0xFFFFFFFEu; /**< 陈旧年龄的饱和上限 */

std::uint32_t feedback_age_ms(std::uint64_t now_ns, std::uint64_t last_fresh_ns, bool stale,
                              std::uint32_t sdk_age_ms) noexcept;

}  // namespace rt
}  // namespace jr

#endif /* JR_ROS2_RT_JR_BUS_RUNTIME_HPP */
