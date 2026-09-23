/**
 * @file    jr_bus_runtime.cpp
 * @brief   BusRuntime 实现（唯一直接调用 jsdk_* 的实时路径所在）
 *
 * @par RT 路径铁律（本文件的 tick_begin / apply_command / tick_end / estop / hold / fill_snapshot）
 *  无锁、无分配、无日志、无 HAL 之外的系统调用；失败只记状态位与 last_error_。
 *  所有 `std::thread`/`std::vector`/`printf` 都只允许出现在**配置阶段**函数里
 *  （open/configure/activate/deactivate/close）—— 这些函数由非 RT 线程调用。
 */

#include "jr_ros2/rt/jr_bus_runtime.hpp"

#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>
#include <chrono>
#include <thread>

#include "joint_sdk/joint_sdk.h"

#include "jr_ros2/jr_abi_check.hpp"
#include "jr_ros2/rt/jr_rt_sched.hpp"
#include "jr_sdk_map.hpp"

/* ==========================================================================
 * 命令模式：`CmdMode` ↔ `jsdk_mode_t` 必须逐个一致
 * ------------------------------------------------------------------------
 * 本包把配置里的模式**直传** `jsdk_joint_config_t.initial_mode` 与
 * `jsdk_joint_request_enable()`，所以取值漂了不会报错，只会“跑到别的模式上去”。
 * 这里用编译期断言把它钉死（对消息常量的那一半在 test_jr_bus_node.cpp）。
 * ======================================================================== */
static_assert(static_cast<int>(jr::CmdMode::kMit) == static_cast<int>(JSDK_MODE_MIT),
              "CmdMode::kMit 与 JSDK_MODE_MIT 不一致");
static_assert(static_cast<int>(jr::CmdMode::kCsp) == static_cast<int>(JSDK_MODE_CSP),
              "CmdMode::kCsp 与 JSDK_MODE_CSP 不一致");
static_assert(static_cast<int>(jr::CmdMode::kCsv) == static_cast<int>(JSDK_MODE_CSV),
              "CmdMode::kCsv 与 JSDK_MODE_CSV 不一致");
static_assert(static_cast<int>(jr::CmdMode::kCst) == static_cast<int>(JSDK_MODE_CST),
              "CmdMode::kCst 与 JSDK_MODE_CST 不一致");
static_assert(static_cast<int>(jr::CmdMode::kCurrent) == static_cast<int>(JSDK_MODE_CURRENT),
              "CmdMode::kCurrent 与 JSDK_MODE_CURRENT 不一致");

/* 后端可用性：默认与 SDK 的 CMake 默认值一致，可由构建系统覆盖
   （`-DJR_HAS_HAL_SOCKETCAN=0` 等），这样"客户只编了 SocketCAN"也能正常链接。 */
#ifndef JR_HAS_HAL_VIRTUAL
#  define JR_HAS_HAL_VIRTUAL 1
#endif
#ifndef JR_HAS_HAL_SLCAN
#  define JR_HAS_HAL_SLCAN 1
#endif
#ifndef JR_HAS_HAL_SOCKETCAN
#  if defined(__linux__)
#    define JR_HAS_HAL_SOCKETCAN 1
#  else
#    define JR_HAS_HAL_SOCKETCAN 0
#  endif
#endif
#ifndef JR_HAS_HAL_PCAN
#  if defined(_WIN32) || defined(__APPLE__)
#    define JR_HAS_HAL_PCAN 1
#  else
#    define JR_HAS_HAL_PCAN 0
#  endif
#endif

#if (JR_HAS_HAL_VIRTUAL + JR_HAS_HAL_SLCAN + JR_HAS_HAL_SOCKETCAN + JR_HAS_HAL_PCAN) > 0
#  include "joint_sdk/jsdk_hal_builtin.h"
#endif

namespace jr {
namespace rt {
namespace {

/** SDK 配置结构体的持有者（`jsdk_context_config_t` 是无 tag typedef，无法前向声明）。 */
struct SdkCfgHolder {
    jsdk_context_config_t cfg;
};

Status map_status(jsdk_status_t st) noexcept { return internal::map_sdk_status(st); }

/** 由 SDK 的错误文本推断"下一步该干什么"（给客户可编程的建议码）。 */
Advice advice_from(const char *sdk_text) noexcept
{
    return internal::advice_from_sdk_text(sdk_text);
}

void sleep_ms(unsigned ms) noexcept
{
    if (ms == 0u) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

const char *hal_kind_name(HalKind k) noexcept
{
    switch (k) {
    case HalKind::kVirtual:   return "virtual";
    case HalKind::kSocketCan: return "socketcan";
    case HalKind::kSlcan:     return "slcan";
    case HalKind::kPcan:      return "pcan";
    }
    return "?";
}

/* ---------------- SDK 回调（必须精确匹配 SDK 的函数指针类型） ---------------- */

void fault_cb(jsdk_joint_t *joint, const jsdk_fault_info_t *info, void *user) noexcept
{
    auto *self = static_cast<BusRuntime *>(user);
    if (self == nullptr) return;

    FaultEvent ev;
    ev.t_ns = now_ns();
    ev.node_id = 0u;
    /* jsdk_joint_t* → node_id（≤16 项线性查找，RT 里可接受） */
    for (unsigned i = 0u; i < kMaxJointsPerBus; ++i) {
        if (self->raw_joint(i) == joint) {
            ev.node_id = static_cast<std::uint8_t>(self->cfg().joints[i].node_id);
            break;
        }
    }
    if (info != nullptr) {
        const bool active = (info->mit_err != 0u) || (info->hb_flags != 0u) ||
                            (info->axis_error != 0u);
        ev.event = active ? 1u : 2u;
        ev.err_code = info->mit_err;
        ev.hb_error = info->hb_flags;
        ev.axis_error = info->axis_error;
        ev.motor_error = info->motor_error;
        ev.encoder_error = info->encoder_error;
        ev.sensorless_error = info->sensorless_error;
        ev.controller_error = info->controller_error;
        ev.system_error = info->system_error;
    }
    /* SDK 的一行描述（纯 snprintf，无系统调用；现场排障第一手信息）。 */
    (void)jsdk_joint_describe_fault(joint, ev.text, sizeof(ev.text));
    (void)self->push_fault_pub(ev);
}

int raw_sink(jsdk_context_t * /*ctx*/, const void *data, size_t len, std::uint32_t offset,
             void *user) noexcept
{
    auto *self = static_cast<BusRuntime *>(user);
    if (self == nullptr || data == nullptr || len == 0u) return 0;

    /* 非 RT 路径（描述符下载），允许分配；容量上限防御异常固件。 */
    constexpr std::size_t kMaxDescBytes = 512u * 1024u;
    const std::size_t end = static_cast<std::size_t>(offset) + len;
    if (end > kMaxDescBytes) {
        self->note_overflow_pub();
        return 1;   /* 放弃（SDK 会记 raw_sink_failed） */
    }
    try {
        if (self->desc_json_mut().size() < end) self->desc_json_mut().resize(end);
        std::memcpy(self->desc_json_mut().data() + offset, data, len);
    } catch (...) {
        self->note_overflow_pub();
        return 1;
    }
    return 0;
}

}  // namespace

struct BusRuntime::HalHolder {
    jsdk_can_hal_t     hal;
    jsdk_hal_handle_t *handle = nullptr;
};

const char *to_string(BusMode m) noexcept
{
    switch (m) {
    case BusMode::kIdle:   return "idle";
    case BusMode::kReady:  return "ready";
    case BusMode::kActive: return "active";
    case BusMode::kPaused: return "paused";
    case BusMode::kFault:  return "fault";
    }
    return "?";
}

BusRuntime::BusRuntime() noexcept
{
    last_error_[0] = '\0';
}

BusRuntime::~BusRuntime()
{
    /* 析构是最后一道保险：若调用方忘了 close，这里仍走**安全失能**（不是 kNone）。 */
    Result ignored;
    close(ExitAction::kDisable, &ignored);
}

/* 供回调使用的两个小桥（声明在 .cpp，避免把 std::vector 暴露给所有 TU）。 */
void BusRuntime::push_fault_pub(const FaultEvent &ev) noexcept { (void)push_fault(ev); }
void BusRuntime::note_overflow_pub() noexcept
{
    desc_json_overflow_ = true;
    note_append("[warn] descriptor raw JSON exceeded the capture limit; cache NOT written");
}
std::vector<std::uint8_t> &BusRuntime::desc_json_mut() noexcept { return desc_json_; }

jsdk_context *BusRuntime::raw_context() noexcept { return ctx_; }

jsdk_joint *BusRuntime::raw_joint(unsigned local_index) noexcept
{
    if (local_index >= kMaxJointsPerBus) return nullptr;
    return joints_[local_index];
}

bool BusRuntime::push_fault(const FaultEvent &ev) noexcept { return faults_.push(ev); }

void BusRuntime::note_append(const char *fmt, ...) noexcept
{
    const std::size_t used = std::strlen(report_.text);
    if (used + 2u >= sizeof(report_.text)) return;
    const std::size_t remain = sizeof(report_.text) - used;
    report_.text[used] = '\n';
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(report_.text + used + 1u, remain - 1u, fmt, ap);
    va_end(ap);
    report_.text[sizeof(report_.text) - 1u] = '\0';
}

void BusRuntime::set_last_error(const char *fmt, ...) noexcept
{
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(last_error_, sizeof(last_error_), fmt, ap);
    va_end(ap);
    last_error_[sizeof(last_error_) - 1u] = '\0';
}

/* ==========================================================================
 * 打开：锁 → HAL → context → 关节
 * ======================================================================== */

Status BusRuntime::open(const BusCfg &bus, std::uint32_t period_ns, const OpenOptions &opt,
                        Result *res) noexcept
{
    if (ctx_ != nullptr || hal_ != nullptr) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "bus '%s' already open", bus.name);
        return Status::kInvalidState;
    }

    bus_ = bus;                 /* 自持一份，调用方的 Config 可以随时销毁 */
    period_ns_ = period_ns;
    mode_ = BusMode::kIdle;
    report_ = BusReport{};
    desc_json_.clear();
    desc_json_overflow_ = false;
    entered_estop_ = false;

    const double rate_hz = (period_ns != 0u) ? 1e9 / static_cast<double>(period_ns) : 0.0;

    /* ---- ① 总线预算检查：不可行的组合**拒绝启动**（而不是跑起来再掉帧） -------
       ⚠ 例外：`allow_empty_scan_bus` 且当前 0 个关节 = **发现用**的空总线打开
       （`jr_gen_config` 还不知道有几个关节）。这时预算模型没有输入，跳过检查
       但**写明跳过**（否则日志看起来像检查过了）。 */
    if (bus_.joint_count == 0u && opt.allow_empty_scan_bus) {
        note_append("[info] bus_plan: skipped (discovery-only open with 0 joints; the budget is "
                    "checked again once the joint list is known)");
    } else {
        report_.plan = plan_bus(bus_, rate_hz);
        if (!report_.plan.feasible) {
            set_last_error("bus plan rejected: %s", report_.plan.text);
            if (res != nullptr) {
                res->set(Status::kInvalidArgument, Advice::kFixBusPlanning, "bus '%s': %s", bus_.name,
                         report_.plan.text);
            }
            return Status::kInvalidArgument;
        }
        note_append("[info] bus_plan: %s", report_.plan.text);
        if (report_.plan.warn) {
            note_append("[warn] bus load is %.1f%%: above 75%% of the configured limit — expect "
                        "little headroom for retries/errors",
                        report_.plan.load * 100.0);
        }
    }

    /* ---- ② SDK ABI 自检 ---- */
    if (opt.check_abi) {
        AbiReport abi;
        if (check_sdk_abi(&abi) != Status::kOk) {
            set_last_error("SDK ABI check failed: %s", abi.detail);
            if (res != nullptr) {
                res->set(Status::kInternal, Advice::kCheckBusConfig, "SDK ABI check failed: %s",
                         abi.detail);
            }
            return Status::kInternal;
        }
        report_.abi_checked = true;
        note_append("[info] sdk abi: %s", abi.detail);
    }

    /* ---- ③ 单 master 锁 ---- */
    if (opt.enable_lock) {
        char msg[512] = {};
        const Status lst =
            lock_.acquire(bus_.name, opt.lock_dir, opt.allow_shared_lock, msg, sizeof(msg));
        note_append("[info] %s", msg);
        if (lst != Status::kOk) {
            set_last_error("%s", msg);
            if (res != nullptr) res->set(lst, Advice::kEnsureSingleMaster, "%s", msg);
            bus_ = BusCfg{};
            return lst;
        }
    }

    /* ---- ④ HAL ---- */
    hal_ = new (std::nothrow) HalHolder{};
    if (hal_ == nullptr) {
        if (res != nullptr) res->set(Status::kNoMemory, Advice::kNone, "out of memory (HAL holder)");
        close(ExitAction::kNone, nullptr);
        return Status::kNoMemory;
    }
    std::memset(&hal_->hal, 0, sizeof(hal_->hal));

    jsdk_status_t hst = JSDK_ERR_UNSUPPORTED;
    switch (bus_.hal) {
    case HalKind::kVirtual:
#if JR_HAS_HAL_VIRTUAL
        hst = jsdk_hal_virtual_open(&hal_->hal, &hal_->handle,
                                   (bus_.channel[0] != '\0') ? bus_.channel : nullptr);
        if (hst == JSDK_OK) {
            /* 仿真后端默认冻结时钟；配置阶段的阻塞 API（configure/calibrate）需要时间前进。 */
            jsdk_hal_virtual_set_autotick(hal_->handle, 1);
        }
#else
        hst = JSDK_ERR_UNSUPPORTED;
#endif
        break;

    case HalKind::kSocketCan:
#if JR_HAS_HAL_SOCKETCAN
        hst = jsdk_hal_socketcan_open(&hal_->hal, &hal_->handle, bus_.channel,
                                      bus_.nominal_bitrate, bus_.is_fd ? bus_.data_bitrate : 0u);
#else
        hst = JSDK_ERR_UNSUPPORTED;
#endif
        break;

    case HalKind::kSlcan:
#if JR_HAS_HAL_SLCAN
        hst = jsdk_hal_slcan_open(&hal_->hal, &hal_->handle, bus_.channel, bus_.serial_baud,
                                  bus_.is_fd ? bus_.data_bitrate : 0u);
#else
        hst = JSDK_ERR_UNSUPPORTED;
#endif
        break;

    case HalKind::kPcan:
#if JR_HAS_HAL_PCAN
        hst = jsdk_hal_pcan_open(&hal_->hal, &hal_->handle, bus_.channel, bus_.nominal_bitrate,
                                 bus_.is_fd ? bus_.data_bitrate : 0u);
#else
        hst = JSDK_ERR_UNSUPPORTED;
#endif
        break;
    }

    if (hst != JSDK_OK) {
        if (hst == JSDK_ERR_UNSUPPORTED) {
            set_last_error("backend '%s' is not built into this library (JR_HAS_HAL_%s=0)",
                           hal_kind_name(bus_.hal), hal_kind_name(bus_.hal));
            if (res != nullptr) {
                res->set(Status::kNotSupported, Advice::kCheckBusConfig,
                         "backend '%s' not available in this build: open '%s' failed",
                         hal_kind_name(bus_.hal), bus_.channel);
            }
        } else {
            set_last_error("HAL open('%s') failed: %s", bus_.channel, jsdk_status_string(hst));
            if (res != nullptr) {
                res->set(Status::kTransport, Advice::kCheckBusConfig,
                         "cannot open '%s' (%s): %s — check the interface name, whether the link is "
                         "up (ip link), the baud/bittiming match, and that no other process owns it",
                         bus_.channel, hal_kind_name(bus_.hal), jsdk_status_string(hst));
            }
        }
        close(ExitAction::kNone, nullptr);
        return (hst == JSDK_ERR_UNSUPPORTED) ? Status::kNotSupported : Status::kTransport;
    }
    note_append("[info] HAL: %s '%s' (FD=%d, nominal=%u, data=%u)", hal_kind_name(bus_.hal),
                bus_.channel, bus_.is_fd ? 1 : 0, bus_.nominal_bitrate, bus_.data_bitrate);

    /* ---- ⑤ context 配置 ---- */
    SdkCfgHolder *holder = new (std::nothrow) SdkCfgHolder{};
    if (holder == nullptr) {
        if (res != nullptr) res->set(Status::kNoMemory, Advice::kNone, "out of memory (sdk cfg)");
        close(ExitAction::kNone, nullptr);
        return Status::kNoMemory;
    }
    sdk_cfg_mem_ = holder;

    jsdk_context_config_t &c = holder->cfg;
    jsdk_context_config_default(&c);
    c.hal = hal_->hal;
    c.master_id = bus_.master_id;
    c.is_fd = bus_.is_fd ? 1u : 0u;
    c.period_ns = period_ns;
    c.state_timeout_ms = bus_.state_timeout_ms;
    /* ⚠ `CURRENT` 关节与 SDK 的 auto_keepalive 的关系（已核对 SDK 实现与它自己的测试）：
       `CURRENT_CONTROL(0x04)` 不是 `is_ctrl` 帧（F19），SDK 的补喂机制会**跳过** CURRENT 关节
       （`tests/test_ops.c`："一帧都不补"）—— 因为补喂用的是 MIT 帧，那会把客户选定的
       输入模式顶掉（表现是力矩周期性掉零），比"设备侧保护不武装"更危险。
       所以这里**不需要**我们替 SDK 关 keepalive；代价（设备侧 break_timeout 不武装）
       由 `validate_config` 的 CURRENT 提示明说。 */
    c.auto_keepalive = bus_.auto_keepalive ? 1u : 0u;
    c.clamp_target_position = bus_.clamp_target ? 1u : 0u;
    c.enable_watchdog_hint = bus_.arm_device_watchdog ? 1u : 0u;
    c.rx_burst_limit = bus_.rx_burst_limit;
    c.max_joints = 0u;

    c.desc.mode = JSDK_DESC_DYNAMIC;
    c.desc.retain = (bus_.desc.retain == DescRetain::kAll) ? JSDK_DESC_RETAIN_ALL
                                                           : JSDK_DESC_RETAIN_FILTERED;
    c.desc.share_by_crc = 1u;
    /* 路线 B（缓存原始 JSON）要求**完整**下载；而且提前终止会让"任意端点可读"
       这个卖点失效（filter 非精确路径时 SDK 本来也不会提前终止，这里显式关闭）。 */
    c.desc.stop_when_satisfied = 0u;
    c.desc.timeout_ms = bus_.desc.timeout_ms;
    c.desc.filter_paths = bus_.desc.filter_paths;
    c.desc.filter_count = bus_.desc.filter_count;

    arena_.assign(jsdk_desc_arena_size(&c.desc), 0u);
    c.desc.arena = arena_.empty() ? nullptr : arena_.data();
    c.desc.arena_size = arena_.size();
    if (c.desc.arena == nullptr) {
        set_last_error("descriptor arena allocation failed (asked %u bytes)",
                       static_cast<unsigned>(c.desc.arena_size));
        if (res != nullptr) res->set(Status::kNoMemory, Advice::kNone, "descriptor arena allocation failed");
        close(ExitAction::kNone, nullptr);
        return Status::kNoMemory;
    }

    /* 用**运行时**尺寸精确分配：不依赖 JSDK_CONTEXT_MAX_SIZE 常量（ADR-12）。 */
    const std::size_t need = jsdk_context_size(&c);
    ctx_mem_bytes_ = (need > sizeof(jsdk_context_storage_t)) ? need : sizeof(jsdk_context_storage_t);
    ctx_mem_ = ::operator new(ctx_mem_bytes_, std::align_val_t(alignof(std::max_align_t)));
    /* ⚠⚠ 必须**清零**：`jsdk_context_init()` 用首字段 magic 做 ABI 守卫（
       `magic != 0 && magic != JSDK_CTX_MAGIC` → INVALID_ARG，含义是"调用方给了脏存储"），
       即它要求存储是"全新"的。而 `::operator new` 给的是**未初始化**内存 ——
       复用到非零脏堆块就中招，症状是**偶发**的 `context init failed: invalid-argument`
       并级联到整个启动流程全红（实测：全量重建后首跑时出现，看着像"负载抖动"，
       其实与负载无关）。SDK 示例用静态数组（天然零初始化），所以只有动态分配才会碰上。
       回归用例：test_bus_runtime.cpp 的 test_open_after_dirty_heap（把分配器换成返回
       脏内存 → 漏清零必然红）。 */
    std::memset(ctx_mem_, 0, ctx_mem_bytes_);
    ctx_ = reinterpret_cast<jsdk_context *>(ctx_mem_);

    const jsdk_status_t ist = jsdk_context_init(ctx_, &c);
    if (ist != JSDK_OK) {
        /* 自诊断：SDK 只给一个状态码（invalid-argument 可能来自 6 个不同校验），
           把**我们交给它的那两块内存长什么样**一并报出来，客户一眼能定位。 */
        const unsigned char *raw = static_cast<const unsigned char *>(ctx_mem_);
        const unsigned magic0 = (raw != nullptr)
                                    ? (static_cast<unsigned>(raw[0]) |
                                       (static_cast<unsigned>(raw[1]) << 8) |
                                       (static_cast<unsigned>(raw[2]) << 16) |
                                       (static_cast<unsigned>(raw[3]) << 24))
                                    : 0u;
        set_last_error("jsdk_context_init failed: %s (storage[0..3]=0x%08x arena=%u B "
                       "master_id=%u fd=%u) - the SDK requires zero-initialized context storage",
                       jsdk_status_string(ist), magic0,
                       static_cast<unsigned>(c.desc.arena_size), static_cast<unsigned>(c.master_id),
                       static_cast<unsigned>(c.is_fd));
        if (res != nullptr) {
            res->set(map_status(ist), Advice::kNone,
                     "context init failed: %s (storage[0..3]=0x%08x)", jsdk_status_string(ist),
                     magic0);
        }
        close(ExitAction::kNone, nullptr);
        return map_status(ist);
    }

    /* ---- ⑥ 关节 ---- */
    joint_count_ = 0u;
    for (unsigned j = 0u; j < bus_.joint_count && j < kMaxJointsPerBus; ++j) {
        jsdk_joint_config_t jc;
        std::memset(&jc, 0, sizeof jc);
        jc.magic = 0u;
        jc.node_id = bus_.joints[j].node_id;
        jc.axis = 0u;
        /* 量程全部**从设备读**（configure 阶段），这里保持 0。 */
        jc.initial_mode = static_cast<jsdk_mode_t>(bus_.joints[j].mode);
        joint_mode_[j] = static_cast<std::uint8_t>(bus_.joints[j].mode);
        cmd_rejected_[j] = 0u;

        jsdk_joint_t *jt = nullptr;
        const jsdk_status_t ast = jsdk_context_add_joint(ctx_, &jc, &jt);
        if (ast != JSDK_OK) {
            /* 最常见的两种：SDK 的 JSDK_MAX_JOINTS_STATIC 比我们的配置小，或重复 node_id。 */
            set_last_error("add_joint(node %u, '%s') failed: %s", bus_.joints[j].node_id,
                           bus_.joints[j].name, jsdk_status_string(ast));
            if (res != nullptr) {
                res->set(map_status(ast), Advice::kNone,
                         "add_joint '%s' (node %u) failed: %s. If this is a capacity error, rebuild "
                         "the SDK with JSDK_MAX_JOINTS_STATIC >= %u (our JR_MAX_JOINTS_PER_BUS=%u "
                         "must match it) — see docs/INTEGRATION.zh-CN.md",
                         bus_.joints[j].name, static_cast<unsigned>(bus_.joints[j].node_id),
                         jsdk_status_string(ast),
                         static_cast<unsigned>(bus_.joint_count),
                         static_cast<unsigned>(kMaxJointsPerBus));
            }
            close(ExitAction::kNone, nullptr);
            return map_status(ast);
        }
        joints_[j] = jt;
        joint_count_ = j + 1u;
    }

    jsdk_context_set_fault_callback(ctx_, &fault_cb, this);
    jsdk_context_set_desc_raw_sink(ctx_, &raw_sink, this);

    mode_ = BusMode::kIdle;
    note_append("[info] context ready: %u joint(s), master_id=%u, period=%u ns", joint_count_,
                static_cast<unsigned>(bus_.master_id), period_ns);
    if (res != nullptr) res->set(Status::kOk, Advice::kNone, "bus '%s' opened", bus_.name);
    return Status::kOk;
}

/* ==========================================================================
 * 配置：描述符（缓存优先）+ configure + 读回量程 + 一致性检查
 * ======================================================================== */

Status BusRuntime::configure(BusReport *rep, Result *res) noexcept
{
    if (ctx_ == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "bus '%s' is not open", bus_.name);
        return Status::kInvalidState;
    }
    if (mode_ == BusMode::kActive) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "bus '%s' is ACTIVE: configuring while joints are enabled is forbidden "
                     "(descriptor frames would starve the control frames and trip the device "
                     "watchdog). Deactivate first (the node does this automatically).",
                     bus_.name);
        }
        return Status::kInvalidState;
    }

    /* ---- ① 设备信息（便宜：一帧 QUERY_DEVICE_INFO，用来给缓存做失效判断） ---- */
    for (unsigned j = 0u; j < joint_count_; ++j) {
        jsdk_device_info_t di;
        std::memset(&di, 0, sizeof di);
        if (jsdk_joint_get_device_info(joints_[j], &di) == JSDK_OK) {
            report_.device[j].hw_version = di.hw_version;
            report_.device[j].fw_version = di.fw_version;
            report_.device[j].serial = di.serial;
            report_.device[j].classic = (di.classic != 0u);
            report_.device[j].valid = true;
        } else {
            report_.device[j].valid = false;
        }
    }

    /* ---- ② 描述符：缓存优先（路线 B） ---- */
    const std::uint32_t dev_fw = report_.device[0].fw_version;
    bool desc_ready = false;

    if (bus_.desc.cache_enabled) {
        DescCache cache;
        std::vector<std::uint8_t> json;
        DescCacheMeta meta;
        char msg[192] = {};
        const CacheOutcome co = cache.load(bus_.desc.cache_dir, bus_.name, dev_fw, &json, &meta, msg, sizeof(msg));
        note_append("[info] %s", msg);
        if (co == CacheOutcome::kHit) {
            jsdk_desc_hint_t hint;
            hint.crc = meta.crc;
            hint.fw_version = meta.fw_version;
            const jsdk_status_t ist = jsdk_context_desc_import_raw(ctx_, json.data(), json.size(), &hint);
            if (ist == JSDK_OK) {
                desc_ready = true;
                report_.desc.from_cache = true;   /* ⚠ 只此一处：以前还写了一个 `desc_from_cache`
                                                    平行字段，结果它才是被读取的那个，
                                                    而 `desc.from_cache` 永远 false（服务/诊断
                                                    就一直在说谎）—— 同一个事实不允许两处。 */
                report_.desc.json_bytes = static_cast<std::uint32_t>(json.size());
                jsdk_desc_info_t di;
                std::memset(&di, 0, sizeof di);
                if (jsdk_context_get_desc_info(ctx_, &di) == JSDK_OK) {
                    report_.desc.total_len = di.total_len;
                    report_.desc.crc = di.crc;
                    report_.desc.fw_version = di.fw_version;
                    report_.desc.endpoint_count = di.endpoint_count;
                    report_.desc.parsed_total = di.parsed_total;
                    report_.desc.frames_rx = 0u;
                    report_.desc.complete = (di.complete != 0u);
                }
                note_append("[info] descriptor imported from cache (%u bytes, fw=%u crc=%u)",
                            static_cast<unsigned>(json.size()),
                            static_cast<unsigned>(meta.fw_version),
                            static_cast<unsigned>(meta.crc));
            } else {
                /* 缓存坏了/与当前 SDK 不兼容 → 明确回退到下载（不静默用一半数据）。 */
                note_append("[warn] cached descriptor rejected by SDK (%s); falling back to a full "
                            "download",
                            jsdk_status_string(ist));
            }
        } else if (co == CacheOutcome::kError) {
            note_append("[warn] descriptor cache unusable; falling back to a full download");
        }
    }

    if (!desc_ready) {
        unsigned attempt = 0u;
        jsdk_status_t fst = JSDK_ERR_TIMEOUT;
        for (attempt = 0u; attempt < bus_.desc.retries; ++attempt) {
            desc_json_.clear();
            desc_json_overflow_ = false;
            fst = jsdk_context_desc_fetch(ctx_);
            if (fst == JSDK_OK) break;
            note_append("[warn] descriptor download attempt %u/%u failed: %s", attempt + 1u,
                        bus_.desc.retries, jsdk_status_string(fst));
            if (attempt + 1u < bus_.desc.retries) sleep_ms(bus_.desc.retry_backoff_ms);
        }
        if (fst != JSDK_OK) {
            set_last_error("descriptor download failed after %u attempt(s): %s (%s)", bus_.desc.retries,
                           jsdk_status_string(fst), jsdk_context_last_error(ctx_));
            if (res != nullptr) {
                res->set(map_status(fst), Advice::kCheckBusConfig,
                         "bus '%s': descriptor download failed after %u attempt(s): %s — %s. "
                         "On slcan the first frames after opening the port are often dropped; a "
                         "retry usually succeeds. Otherwise check channel/bitrate/FD match.",
                         bus_.name, bus_.desc.retries, jsdk_status_string(fst),
                         jsdk_context_last_error(ctx_));
            }
            return map_status(fst);
        }

        jsdk_desc_info_t di;
        std::memset(&di, 0, sizeof di);
        if (jsdk_context_get_desc_info(ctx_, &di) == JSDK_OK) {
            report_.desc.total_len = di.total_len;
            report_.desc.crc = di.crc;
            report_.desc.fw_version = di.fw_version;
            report_.desc.endpoint_count = di.endpoint_count;
            report_.desc.parsed_total = di.parsed_total;
            report_.desc.frames_rx = di.frames_rx;
            report_.desc.complete = (di.complete != 0u);
            report_.desc.shared_hit = (di.shared_hit != 0u);
            const bool complete = report_.desc.complete;
            if (bus_.desc.cache_enabled && complete && !desc_json_overflow_ &&
                desc_json_.size() == di.total_len) {
                DescCache cache;
                char msg[192] = {};
                (void)cache.save(bus_.desc.cache_dir, bus_.name, di.fw_version, di.crc,
                                 desc_json_.data(), desc_json_.size(), msg, sizeof(msg));
                note_append("[info] %s", msg);
            } else if (bus_.desc.cache_enabled && !complete) {
                /* DESIGN §10.2 的证伪用例：**不完整就绝不落盘**。 */
                note_append("[warn] descriptor download was not complete (complete=0): cache NOT "
                            "written (a partial cache would silently mis-decode parameters)");
            }
        }
    }

    /* ---- ③ configure()（握手 + 量程读回 + 必要的看门狗协商） ---- */
    jsdk_status_t cst = JSDK_ERR_TIMEOUT;
    unsigned attempt = 0u;
    for (attempt = 0u; attempt < bus_.desc.retries; ++attempt) {
        cst = jsdk_context_configure(ctx_);
        if (cst == JSDK_OK) break;
        /* configure 的阻塞部分（读量程）同样可能因首帧丢失而超时 → 重试是现场必需。 */
        note_append("[warn] configure() attempt %u/%u failed: %s", attempt + 1u, bus_.desc.retries,
                    jsdk_status_string(cst));
        if (attempt + 1u < bus_.desc.retries) sleep_ms(bus_.desc.retry_backoff_ms);
    }
    if (cst != JSDK_OK) {
        /* ---- 特殊情形：**设备未标定** ----
           `configure()` 会因为"标定量程不可用"而失败，但"先标定"正是客户接下来要做的事。
           如果这里直接把总线卡在 kIdle，就会出现鸡生蛋：标定服务（写 requested_state）
           需要端点表，而被卡住的状态又不允许调。
           因此只在**所有关节都读不到有效量程**时降级为 READY（未标定）；
           其它失败（超时/协议错）照旧失败，绝不掩盖。
           ⚠ 能安全降级的前提是**描述符已经拿到**（上面的②先于③），否则端点表是空的。 */
        bool all_uncalibrated = (joint_count_ > 0u);
        for (unsigned j = 0u; j < joint_count_; ++j) {
            jsdk_unit_scale_t sc;
            std::memset(&sc, 0, sizeof sc);
            /* ⚠ `jsdk_joint_get_scale()` 返回 void —— 有效性看 `sc.valid`，
               不要写成 `... == JSDK_OK`（编译期就会红）。 */
            jsdk_joint_get_scale(joints_[j], &sc);
            if (sc.valid != 0) {
                all_uncalibrated = false;
                break;
            }
        }
        if (all_uncalibrated) {
            note_append("[warn] configure() reported '%s' because no joint has a usable calibrated "
                        "range. The bus stays READY with the endpoint table available: run "
                        "SetZero/Calibrate, then configure again. Enabling remains blocked until "
                        "then.",
                        jsdk_status_string(cst));
            (void)read_back_after_configure(&report_, nullptr);
            report_.all_calibrated = false;
            mode_ = BusMode::kReady;
            if (rep != nullptr) *rep = report_;
            if (res != nullptr) {
                res->set(Status::kOk, Advice::kCalibrationRequired,
                         "bus '%s': joint is NOT CALIBRATED (configure() said '%s'). The bus is "
                         "READY and the endpoint table is loaded, so you can run SetZero/Calibrate "
                         "now; control/enable is blocked until the calibration is valid.",
                         bus_.name, jsdk_status_string(cst));
            }
            return Status::kOk;
        }

        set_last_error("configure() failed after %u attempt(s): %s (%s)", bus_.desc.retries,
                       jsdk_status_string(cst), jsdk_context_last_error(ctx_));
        if (res != nullptr) {
            res->set(map_status(cst), advice_from(jsdk_context_last_error(ctx_)),
                     "bus '%s': configure() failed: %s — %s", bus_.name, jsdk_status_string(cst),
                     jsdk_context_last_error(ctx_));
        }
        return map_status(cst);
    }

    const Status rbs = read_back_after_configure(&report_, res);
    if (rbs != Status::kOk) return rbs;

    mode_ = BusMode::kReady;
    if (rep != nullptr) *rep = report_;
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "bus '%s' configured (ready, NOT enabled)", bus_.name);
    }
    return Status::kOk;
}

Status BusRuntime::read_back_after_configure(BusReport *rep, Result *res) noexcept
{
    for (unsigned j = 0u; j < joint_count_; ++j) {
        jsdk_joint_config_snapshot_t snap;
        std::memset(&snap, 0, sizeof snap);
        const jsdk_status_t sst = jsdk_joint_read_config_snapshot(joints_[j], &snap);

        jsdk_unit_scale_t scale;
        std::memset(&scale, 0, sizeof scale);
        (void)jsdk_joint_get_scale(joints_[j], &scale);

        JointInfoPOD &info = report_.joint[j];
        if (sst == JSDK_OK) {
            info.gear_ratio = snap.gear_ratio;
            info.mit_max_pos = snap.mit_max_pos;
            info.mit_max_vel = snap.mit_max_vel;
            info.mit_max_torque = snap.mit_max_torque;
            info.mit_max_kp = snap.mit_max_kp;
            info.mit_max_kd = snap.mit_max_kd;
            info.torque_constant = snap.torque_constant;
            info.node_id = snap.node_id;
            info.heartbeat_rate_ms = snap.heartbeat_rate_ms;
            info.break_timeout_ms = snap.break_timeout_ms;
            info.config_valid = (snap.valid != 0);
        }
        info.calibrated = (scale.valid != 0);

        const JointCfg &jc = bus_.joints[j];

        /* ---- 量程一致性：客户的软限位一旦超过设备标定范围，必须**拒绝启动** ---- */
        if (jc.has_position_limit && info.mit_max_pos > 0.0) {
            const double worst = (std::fabs(jc.position_min) > std::fabs(jc.position_max))
                                     ? std::fabs(jc.position_min)
                                     : std::fabs(jc.position_max);
            if (worst > info.mit_max_pos + 1e-9) {
                set_last_error("joint '%s': requested limit ±%.4f rad exceeds the device's "
                               "calibrated range ±%.4f rad",
                               jc.name, worst, info.mit_max_pos);
                if (res != nullptr) {
                    res->set(Status::kInvalidArgument, Advice::kCalibrationRequired,
                             "joint '%s' (%s): limit ±%.4f rad > device range ±%.4f rad. Refusing "
                             "to start: the SDK would reject the out-of-range target every cycle "
                             "and send a safe frame instead, so the joint would never reach the "
                             "commanded pose (silent 'it just does not move').",
                             jc.name, bus_.name, worst, info.mit_max_pos);
                }
                return Status::kInvalidArgument;
            }
        }

        /* ---- 未标定：不是错误，但必须让客户看到"为什么动不了" ---- */
        if (!info.calibrated) {
            note_append("[warn] joint '%s': NOT CALIBRATED (unit_scale invalid) — the physical "
                        "quantity APIs are rejected until you run Calibrate/SetZero and re-configure",
                        jc.name);
        }

        /* ---- 反馈策略与设备侧心跳的一致性（**只报告，不擅自改设备**） ---- */
        const bool uses_hb = (bus_.feedback == FeedbackPolicy::kBroadcastHeartbeat) ||
                             (bus_.feedback == FeedbackPolicy::kHeartbeatOnly);
        if (uses_hb) {
            if (info.heartbeat_rate_ms == 0u) {
                note_append("[warn] joint '%s': device heartbeat is DISABLED (heartbeat_rate_ms=0) "
                            "but feedback policy needs it → no periodic feedback at all. "
                            "Enable it explicitly (service SetParams / jsdk-cli write).",
                            jc.name);
            } else if (info.heartbeat_rate_ms != bus_.heartbeat_ms) {
                note_append("[info] joint '%s': device heartbeat_rate_ms=%u, configured %u "
                            "(device config NOT changed; we only use what it sends)",
                            jc.name, info.heartbeat_rate_ms, bus_.heartbeat_ms);
            }
        }

        /* ---- 设备侧协议超时：0 = **禁用**（最新固件语义） ---- */
        if (info.break_timeout_ms == 0u) {
            note_append("[info] joint '%s': device-side protocol timeout = 0 (DISABLED — in current "
                        "firmware 0 means 'no timeout', not '100 ms')", jc.name);
        } else {
            const double period_ms = static_cast<double>(period_ns_) / 1e6;
            if (period_ms >= static_cast<double>(info.break_timeout_ms)) {
                note_append("[warn] joint '%s': control period %.3f ms >= device break_timeout %u ms "
                            "— the SDK rejects such a loop; shorten the period or raise/disable the "
                            "device timeout (we do NOT change it for you)",
                            jc.name, period_ms, info.break_timeout_ms);
            } else {
                note_append("[info] joint '%s': device-side protocol timeout = %u ms (armed)",
                            jc.name, info.break_timeout_ms);
            }
        }
    }

    note_append("[info] descriptor: %s, %u/%u endpoints retained, crc=%u fw=%u",
                report_.desc.from_cache ? "from cache" : "downloaded",
                report_.desc.endpoint_count, report_.desc.parsed_total,
                static_cast<unsigned>(report_.desc.crc), static_cast<unsigned>(report_.desc.fw_version));

    /* 汇总标定就绪度（节点/诊断直接取值，无需自己遍历）。 */
    report_.all_calibrated = (joint_count_ > 0u);
    for (unsigned j = 0u; j < joint_count_; ++j) {
        if (!report_.joint[j].calibrated) report_.all_calibrated = false;
    }
    if (!report_.all_calibrated) {
        note_append("[warn] not all joints are calibrated: physical-quantity commands are rejected "
                    "until calibration is valid (see per-joint notes above)");
    }

    if (rep != nullptr) *rep = report_;
    return Status::kOk;
}

void BusRuntime::refresh_device_notes(BusReport *report) noexcept
{
    if (!opened()) return;
    report_.text[0] = '\0';
    (void)read_back_after_configure(&report_, nullptr);
    if (report != nullptr) *report = report_;
}

/* ==========================================================================
 * 使能 / 失能 / 故障复位
 * ======================================================================== */

Status BusRuntime::activate(bool require_calibrated, Result *res) noexcept
{
    if (ctx_ == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "bus '%s' is not open", bus_.name);
        return Status::kInvalidState;
    }
    if (mode_ == BusMode::kActive) {
        if (res != nullptr) res->set(Status::kOk, Advice::kNone, "bus '%s' already active", bus_.name);
        return Status::kOk;
    }
    if (mode_ != BusMode::kReady && mode_ != BusMode::kFault && mode_ != BusMode::kPaused) {
        if (res != nullptr) {
            /* kPaused 是**允许**的：那正是"安全暂停窗口内做阻塞操作"的用法（ADR-7）。 */
            res->set(Status::kInvalidState, Advice::kNone,
                     "bus '%s' is %s: enabling requires READY (configured) or PAUSED (blocking-op "
                     "window). Call configure() first if the bus has never been configured.",
                     bus_.name, to_string(mode_));
        }
        return Status::kInvalidState;
    }

    if (require_calibrated) {
        for (unsigned j = 0u; j < joint_count_; ++j) {
            if (!report_.joint[j].calibrated) {
                if (res != nullptr) {
                    res->set(Status::kNotCalibrated, Advice::kCalibrationRequired,
                             "joint '%s' (%s) is not calibrated: the SDK refuses physical-quantity "
                             "commands (unit_scale invalid). Run Calibrate (or SetZero if the joint "
                             "is already zeroed) and retry.",
                             bus_.joints[j].name, bus_.name);
                }
                return Status::kNotCalibrated;
            }
        }
    }

    const jsdk_status_t st = jsdk_context_activate(ctx_);
    if (st != JSDK_OK) {
        set_last_error("activate failed: %s (%s)", jsdk_status_string(st),
                       jsdk_context_last_error(ctx_));
        if (res != nullptr) {
            /* SDK 明确：activate 超时时**部分关节可能已经带电**，不能当"什么都没发生"。 */
            res->set(map_status(st), Advice::kRetryFaultReset,
                     "bus '%s': enable failed: %s — %s. NOTE: on timeout some joints may already be "
                     "energized and driven; either call deactivate() (safe shutdown) or keep ticking.",
                     bus_.name, jsdk_status_string(st), jsdk_context_last_error(ctx_));
        }
        mode_ = BusMode::kFault;
        return map_status(st);
    }

    mode_ = BusMode::kActive;
    entered_estop_ = false;
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "bus '%s' active (%u joint(s) enabled)", bus_.name,
                 joint_count_);
    }
    return Status::kOk;
}

Status BusRuntime::deactivate(Result *res) noexcept
{
    if (ctx_ == nullptr) {
        if (res != nullptr) res->set(Status::kOk, Advice::kNone, "bus '%s' not open", bus_.name);
        return Status::kOk;
    }
    /* SDK 的 deactivate 是"安全帧 → 等 2 周期 → STOP_MOTOR → 等 IDLE"，幂等。 */
    jsdk_context_deactivate(ctx_);
    if (mode_ == BusMode::kActive || mode_ == BusMode::kFault) mode_ = BusMode::kReady;
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "bus '%s' joints disabled (safe sequence)", bus_.name);
    }
    return Status::kOk;
}

Status BusRuntime::fault_reset(Result *res) noexcept
{
    return fault_reset(nullptr, 0u, res);
}

Status BusRuntime::fault_reset(const unsigned *idx, unsigned n, Result *res) noexcept
{
    if (ctx_ == nullptr) return Status::kInvalidState;
    unsigned sel[kMaxJointsPerBus] = {};
    unsigned sel_count = 0u;
    if (!select_joints(idx, n, sel, &sel_count, res)) return Status::kInvalidArgument;

    bool any_fault = false;
    for (unsigned k = 0u; k < sel_count; ++k) {
        const unsigned j = sel[k];
        if (jsdk_joint_is_fault(joints_[j]) != 0) any_fault = true;
        jsdk_joint_request_fault_reset(joints_[j]);   /* 非阻塞：需要跑周期才落地 */
    }
    if (!any_fault) {
        if (res != nullptr) res->set(Status::kOk, Advice::kNone, "bus '%s': no fault latched", bus_.name);
        return Status::kOk;
    }

    /* 跑若干周期让 SDK 走完 CLEAR_ERRORS → START_MOTOR 的请求队列。 */
    const std::uint64_t t0 = now_ns();
    const std::uint64_t budget_ns = 300ull * 1000000ull;
    bool cleared = false;
    while (now_ns() - t0 < budget_ns) {
        (void)jsdk_context_cycle_begin(ctx_, now_ns());
        (void)jsdk_context_cycle_end(ctx_);
        cleared = true;
        for (unsigned k = 0u; k < sel_count; ++k) {
            if (jsdk_joint_is_fault(joints_[sel[k]]) != 0) {
                cleared = false;
                break;
            }
        }
        if (cleared) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (!cleared) {
        if (res != nullptr) {
            /* 实测（fw 1545）：estop 锁存后 CLEAR_ERRORS **清不掉**，只有 RESET_DEVICE 或断电。 */
            res->set(Status::kTimeout, Advice::kNeedsDeviceReset,
                     "bus '%s': fault did not clear via CLEAR_ERRORS. Measured behaviour (fw 1545): "
                     "an ESTOP-latched error cannot be cleared this way — use ResetDevice "
                     "(soft reset) or power-cycle the joint.",
                     bus_.name);
        }
        return Status::kTimeout;
    }
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "bus '%s': faults cleared", bus_.name);
    }
    return Status::kOk;
}

/* ==========================================================================
 * 逐关节使能/失能（服务层用）
 * ======================================================================== */

bool BusRuntime::joint_enabled(unsigned local_index) const noexcept
{
    if (ctx_ == nullptr || local_index >= joint_count_) return false;
    return jsdk_joint_is_enabled(joints_[local_index]) != 0;
}

const JointTarget &BusRuntime::applied_target(unsigned local_index) const noexcept
{
    /* 越界不崩（诊断/日志会拿没使能的关节来问）：返回一个全零静态对象，
       而不是"最后一个关节"那个错值。 */
    static const JointTarget kZero = {};
    if (local_index >= kMaxJointsPerBus) return kZero;
    return applied_[local_index];
}

DeviceInfoPOD BusRuntime::device_info(unsigned local_index) const noexcept
{
    if (local_index >= joint_count_) return DeviceInfoPOD{};
    return report_.device[local_index];
}

void *BusRuntime::hal_handle_for_test() const noexcept
{
    return (hal_ != nullptr) ? static_cast<void *>(hal_->handle) : nullptr;
}

std::uint16_t BusRuntime::joint_status_flags(unsigned local_index) const noexcept
{
    if (local_index >= joint_count_) return 0u;
    jsdk_joint_feedback_t fb;
    std::memset(&fb, 0, sizeof fb);
    if (jsdk_joint_get_feedback(joints_[local_index], &fb) != JSDK_OK) return 0u;
    return fb.status_flags;
}

bool BusRuntime::joint_fault_text(unsigned local_index, char *out, std::size_t cap) const noexcept
{
    if (out != nullptr && cap > 0u) out[0] = '\0';
    if (local_index >= joint_count_ || out == nullptr || cap == 0u) return false;
    if (jsdk_joint_is_fault(joints_[local_index]) == 0) return false;
    /* SDK 自己有一行描述（fault 位→人话），不要自己再写一份会漂的表。 */
    (void)jsdk_joint_describe_fault(joints_[local_index], out, cap);
    if (out[0] == '\0') {
        std::snprintf(out, cap, "fault latched (no description available)");
    }
    return true;
}

Status BusRuntime::set_enabled(const unsigned *idx, unsigned n, bool enable, bool require_calibrated,
                               Result *res) noexcept
{
    if (ctx_ == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "bus '%s' is not open", bus_.name);
        return Status::kInvalidState;
    }

    /* 全选 = 用 SDK 的**阻塞**整总线版本（它自己做完整序列，且已被端到端验证过）。 */
    if (idx == nullptr || n == 0u) {
        return enable ? activate(require_calibrated, res) : deactivate(res);
    }
    if (!ops_window_ok(res, enable ? "enable(subset)" : "disable(subset)")) return Status::kInvalidState;

    unsigned sel[kMaxJointsPerBus] = {};
    unsigned sel_count = 0u;
    if (!select_joints(idx, n, sel, &sel_count, res)) return Status::kInvalidArgument;

    if (enable && require_calibrated) {
        for (unsigned k = 0u; k < sel_count; ++k) {
            if (!report_.joint[sel[k]].calibrated) {
                if (res != nullptr) {
                    res->set(Status::kNotCalibrated, Advice::kCalibrationRequired,
                             "joint '%s' (%s) is not calibrated: the SDK refuses physical-quantity "
                             "commands (unit_scale invalid). Run Calibrate (or SetZero if the joint is "
                             "already zeroed) and retry.",
                             bus_.joints[sel[k]].name, bus_.name);
                }
                return Status::kNotCalibrated;
            }
        }
    }

    /* ⚠ SDK 明确：`request_enable/disable` **都是非阻塞请求**，状态变化发生在后续若干次
       `cycle_end()` 里。所以这里必须**泵周期**并**读回确认**，不能"请求完就当完成了"。
       （整总线版本用 `jsdk_context_activate/deactivate`，那两个是阻塞的 ✓。）
       模式取该关节**配置的模式**（`joints[].mode`，默认 MIT）：写死 MIT 会让一个 CSP
       配置的关节被子集使能"偷偷换成 MIT"（症状：位置命令不生效、力矩突然出现）。 */
    for (unsigned k = 0u; k < sel_count; ++k) {
        const unsigned j = sel[k];
        if (enable) {
            jsdk_joint_request_enable(joints_[j], static_cast<jsdk_mode_t>(joint_mode_[j]));
        } else {
            jsdk_joint_request_disable(joints_[j]);
        }
    }

    std::uint32_t budget_ms = bus_.state_timeout_ms;
    if (budget_ms == 0u) budget_ms = 2000u;   /* 使能/失能不需要标定那么久（那是几十秒） */
    const std::uint64_t t0 = now_ns();
    bool done = false;
    while (now_ns() - t0 < static_cast<std::uint64_t>(budget_ms) * 1000000ull) {
        (void)jsdk_context_cycle_begin(ctx_, now_ns());
        (void)jsdk_context_cycle_end(ctx_);
        done = true;
        for (unsigned k = 0u; k < sel_count; ++k) {
            if (jsdk_joint_is_enabled(joints_[sel[k]]) != (enable ? 1 : 0)) {
                done = false;
                break;
            }
        }
        if (done) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    /* 按**实际**状态定总线模式（不要按"我请求了什么"）。 */
    bool any_enabled = false;
    for (unsigned j = 0u; j < joint_count_; ++j) {
        if (jsdk_joint_is_enabled(joints_[j]) != 0) any_enabled = true;
    }
    if (any_enabled) {
        mode_ = BusMode::kActive;
    } else if (mode_ == BusMode::kActive) {
        mode_ = BusMode::kReady;
    }

    if (!done) {
        if (res != nullptr) {
            res->set(Status::kTimeout, Advice::kRetryFaultReset,
                     "bus '%s': %s of %u joint(s) did not complete within %u ms (some joints may be "
                     "in a partial state: read back `enabled` per joint before acting)",
                     bus_.name, enable ? "enable" : "disable", sel_count, budget_ms);
        }
        return Status::kTimeout;
    }
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "bus '%s': %s done on %u joint(s) (verified by read-back)",
                 bus_.name, enable ? "enable" : "disable", sel_count);
    }
    return Status::kOk;
}

/* ==========================================================================
 * 诊断文本（SDK 知识留在核心库）
 * ======================================================================== */

void bus_flags_text(std::uint32_t flags, char *out, std::size_t cap) noexcept
{
    if (out == nullptr || cap == 0u) return;
    if (flags == 0u) {
        std::snprintf(out, cap, "none (HAL reports nothing)");
        return;
    }
    char buf[192] = {};
    std::size_t used = 0u;
    const struct {
        std::uint32_t bit;
        const char *name;
    } kNames[] = {
        {JSDK_HAL_BUS_OK, "ok"},
        {JSDK_HAL_BUS_ERROR_WARN, "error-warning"},
        {JSDK_HAL_BUS_ERROR_PASS, "error-passive"},
        {JSDK_HAL_BUS_OFF, "bus-off"},
    };
    for (const auto &n : kNames) {
        if ((flags & n.bit) == 0u) continue;
        const int w = std::snprintf(buf + used, sizeof(buf) - used, "%s%s", (used != 0u) ? "," : "",
                                    n.name);
        if (w > 0) used += static_cast<std::size_t>(w);
        if (used >= sizeof(buf)) break;
    }
    const std::uint32_t known = JSDK_HAL_BUS_OK | JSDK_HAL_BUS_ERROR_WARN | JSDK_HAL_BUS_ERROR_PASS |
                                JSDK_HAL_BUS_OFF;
    if ((flags & ~known) != 0u && used < sizeof(buf)) {
        std::snprintf(buf + used, sizeof(buf) - used, "%s0x%X(unknown)", (used != 0u) ? "," : "",
                      flags & ~known);
    }
    std::snprintf(out, cap, "%s", buf);
}

void joint_status_flags_text(std::uint16_t flags, char *out, std::size_t cap) noexcept
{
    if (out == nullptr || cap == 0u) return;
    if (flags == 0u) {
        std::snprintf(out, cap, "none");
        return;
    }
    char buf[256] = {};
    std::size_t used = 0u;
    const struct {
        std::uint16_t bit;
        const char *name;
    } kNames[] = {
        {JSDK_JF_TARGET_REJECTED, "TARGET_REJECTED"},
        {JSDK_JF_SAFE_FRAME_SENT, "SAFE_FRAME_SENT"},
        {JSDK_JF_WATCHDOG_RISK, "WATCHDOG_RISK"},
        {JSDK_JF_FEEDBACK_STALE, "FEEDBACK_STALE"},
        {JSDK_JF_TX_FAILED, "TX_FAILED"},
        {JSDK_JF_SCALE_INVALID, "SCALE_INVALID"},
        {JSDK_JF_WATCHDOG_UNVERIFIED, "WATCHDOG_UNVERIFIED"},
    };
    for (const auto &n : kNames) {
        if ((flags & n.bit) == 0u) continue;
        const int w = std::snprintf(buf + used, sizeof(buf) - used, "%s%s", (used != 0u) ? "," : "",
                                    n.name);
        if (w > 0) used += static_cast<std::size_t>(w);
        if (used >= sizeof(buf)) break;
    }
    std::uint16_t known = 0u;
    for (const auto &n : kNames) known = static_cast<std::uint16_t>(known | n.bit);
    if ((flags & ~known) != 0u) {
        if (used < sizeof(buf)) {
            std::snprintf(buf + used, sizeof(buf) - used, "%s0x%X(unknown)", (used != 0u) ? "," : "",
                          static_cast<unsigned>(flags & ~known));
        }
    }
    std::snprintf(out, cap, "%s", buf);
}

Status BusRuntime::refresh_report(Result *res) noexcept
{
    if (ctx_ == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "bus '%s' is not open", bus_.name);
        return Status::kInvalidState;
    }
    /* 读回量程/心跳/超时/标定标志（服务与诊断要"现在的事实"）。 */
    const Status st = read_back_after_configure(&report_, res);
    refresh_device_notes(&report_);
    return st;
}

Status BusRuntime::import_descriptor(const void *json, std::size_t len, Result *res) noexcept
{
    if (ctx_ == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "bus '%s' is not open", bus_.name);
        return Status::kInvalidState;
    }
    if (json == nullptr || len == 0u) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kNone,
                     "descriptor payload is empty: pass the bytes produced by ExportDescriptor (raw JSON)");
        }
        return Status::kInvalidArgument;
    }
    /* hint = nullptr：让 SDK 按当前 `cfg.desc` 重新解析（这正是我们想要的路线 B 行为）。 */
    const jsdk_status_t st = jsdk_context_desc_import_raw(ctx_, json, len, nullptr);
    if (st != JSDK_OK) {
        if (res != nullptr) {
            res->set(map_status(st), Advice::kNone,
                     "bus '%s': importing the descriptor failed: %s (%s). The payload must be the "
                     "raw JSON captured by ExportDescriptor for a compatible firmware.",
                     bus_.name, jsdk_status_string(st), jsdk_context_last_error(ctx_));
        }
        return map_status(st);
    }
    desc_json_.assign(static_cast<const std::uint8_t *>(json),
                      static_cast<const std::uint8_t *>(json) + len);
    /* 描述符换了 → 之前解析出的端点表与量程都不再可信：回到 IDLE，要求重新 configure。 */
    mode_ = BusMode::kIdle;
    note_append("[info] descriptor imported: the bus returned to IDLE, run configure() again");
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone,
                 "bus '%s': descriptor imported (%zu bytes). The bus is now IDLE: call configure() "
                 "again to parse it and re-read the ranges.",
                 bus_.name, len);
    }
    return Status::kOk;
}


/* ==========================================================================
 * 关停
 * ======================================================================== */

void BusRuntime::close(ExitAction action, Result *res) noexcept
{
    if (ctx_ == nullptr && hal_ == nullptr && sdk_cfg_mem_ == nullptr && !lock_.held()) {
        if (res != nullptr) res->set(Status::kOk, Advice::kNone, "bus '%s' already closed", bus_.name);
        return;
    }

    if (ctx_ != nullptr && (mode_ == BusMode::kActive || mode_ == BusMode::kFault)) {
        switch (action) {
        case ExitAction::kHold:
            /* 明确告知代价：我们一退出，设备侧若武装了超时就会故障掉。 */
            hold_all(0.0, 0.0);
            note_append("[warn] on_exit=hold: joints were left holding position. This only works "
                        "while this process keeps sending frames; device-side break_timeout (if "
                        "armed) will fault them after we exit.");
            break;
        case ExitAction::kZeroTorque:
            zero_torque_all();
            break;
        case ExitAction::kEstop:
            estop_now();
            break;
        case ExitAction::kNone:
            note_append("[warn] on_exit=none: leaving the devices as they are (debug only)");
            break;
        case ExitAction::kDisable:
        default:
            break;   /* 下面的 deactivate 会走安全序列 */
        }

        if (action != ExitAction::kNone) {
            /* 发出去的帧需要几个周期才真正上总线并被设备执行。 */
            for (unsigned k = 0u; k < 3u; ++k) {
                (void)jsdk_context_cycle_begin(ctx_, now_ns());
                (void)jsdk_context_cycle_end(ctx_);
            }
        }
    }

    if (ctx_ != nullptr) {
        if (action == ExitAction::kDisable) {
            jsdk_context_deactivate(ctx_);   /* 安全帧 → 等 2 周期 → STOP_MOTOR → 等 IDLE */
        }
        jsdk_context_destroy(ctx_);
    }

    destroy_context();
    close_hal();
    lock_.release();
    mode_ = BusMode::kIdle;

    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "bus '%s' closed (exit action: %s)", bus_.name,
                 [action]() {
                     switch (action) {
                     case ExitAction::kDisable:    return "disable";
                     case ExitAction::kHold:       return "hold";
                     case ExitAction::kZeroTorque: return "zero-torque";
                     case ExitAction::kEstop:      return "estop";
                     case ExitAction::kNone:       return "none";
                     }
                     return "?";
                 }());
    }
}

void BusRuntime::destroy_context() noexcept
{
    if (ctx_mem_ != nullptr) {
        ::operator delete(ctx_mem_, std::align_val_t(alignof(std::max_align_t)));
        ctx_mem_ = nullptr;
        ctx_ = nullptr;
    }
    ctx_mem_bytes_ = 0u;
    if (sdk_cfg_mem_ != nullptr) {
        delete static_cast<SdkCfgHolder *>(sdk_cfg_mem_);
        sdk_cfg_mem_ = nullptr;
    }
    arena_.clear();
    arena_.shrink_to_fit();
    for (unsigned j = 0u; j < kMaxJointsPerBus; ++j) joints_[j] = nullptr;
    joint_count_ = 0u;
}

void BusRuntime::close_hal() noexcept
{
    if (hal_ != nullptr) {
        if (hal_->handle != nullptr) {
#if (JR_HAS_HAL_VIRTUAL + JR_HAS_HAL_SLCAN + JR_HAS_HAL_SOCKETCAN + JR_HAS_HAL_PCAN) > 0
            (void)jsdk_hal_close(hal_->handle);
#endif
            hal_->handle = nullptr;
        }
        delete hal_;
        hal_ = nullptr;
    }
}

/* ==========================================================================
 * RT 路径
 * ======================================================================== */

void BusRuntime::tick_begin(std::uint64_t now_ns_in) noexcept
{
    if (ctx_ == nullptr) return;
    tick_now_ns_ = now_ns_in;   /* 插值进度用这个时钟（不是 ROS 域的 stamp_ns） */
    last_cycle_status_ = static_cast<int>(jsdk_context_cycle_begin(ctx_, now_ns_in));
}

void BusRuntime::set_interpolation(Interpolation i) noexcept
{
    interp_mode_ = i;
    /* 关掉时把状态清干净：否则下次再打开会从一段**陈旧**的 from/to 开始插，
       表现为"刚下发的新目标不动，过一会儿才自己跑"。 */
    for (unsigned j = 0u; j < kMaxJointsPerBus; ++j) {
        interp_active_[j] = false;
        interp_have_[j] = false;
        interp_seq_[j] = 0u;
        interp_dt_ns_[j] = 0u;
        interp_t0_ns_[j] = 0u;
        interp_last_cmd_ns_[j] = 0u;
    }
}

/** 线性插值：只插"该模式承载的运动量"（增益/限制量按最新值**立即**生效）。
 *
 *  - MIT：position / velocity / torque（三者都是轨迹量；只插位置会让前馈阶跃，
 *    与前馈的意义相背）；
 *  - CSP：position；CSV：velocity；CST：torque；CURRENT：current。
 *  - **不插** kp/kd/stiffness/damping 与 velocity_limit/current_limit。
 */
static void lerp_motion(const JointTarget &from, const JointTarget &to, double f,
                        JointTarget *out) noexcept
{
    const auto mix = [f](double a, double b) noexcept { return a + (b - a) * f; };
    *out = to;   /* 先整体取最新值（增益/限制/模式/si_gain 都按最新） */
    switch (to.mode) {
    case CmdMode::kMit:
        out->position = mix(from.position, to.position);
        out->velocity = mix(from.velocity, to.velocity);
        out->torque = mix(from.torque, to.torque);
        break;
    case CmdMode::kCsp:
        out->position = mix(from.position, to.position);
        break;
    case CmdMode::kCsv:
        out->velocity = mix(from.velocity, to.velocity);
        break;
    case CmdMode::kCst:
        out->torque = mix(from.torque, to.torque);
        break;
    case CmdMode::kCurrent:
        out->current = mix(from.current, to.current);
        break;
    }
}

void BusRuntime::apply_command(const CommandSet &cmd, unsigned joint_base) noexcept
{
    if (ctx_ == nullptr) return;

    if (cmd.estop) {
        estop_now();
        return;
    }
    if (cmd.zero_torque_all) {
        zero_torque_all();
        return;
    }

    for (unsigned j = 0u; j < joint_count_; ++j) {
        const unsigned g = joint_base + j;
        if (g >= kMaxJoints) break;
        if (!cmd.has(g)) continue;   /* 未置位：SDK 会沿用上一次目标（保持） */
        const JointTarget &t = cmd.joint[g];
        /* ⚠ 模式必须与使能时选择的模式一致。不一致就丢掉并计数，**不静默按 MIT 发**：
           MIT 广播/单播帧会把 POS/VEL/TORQUE 模式的关节悄悄换回 MIT 输入模式
           （SDK 在 `try_broadcast_mit()` 里也把这一条写成了降级理由）。
           切模式是阻塞的（disable→enable），只能在 ADR-7 窗口里由服务做，
           不可能在 RT 命令路径上做——所以这里只能拒。 */
        const std::uint8_t want = static_cast<std::uint8_t>(t.mode);
        if (want != joint_mode_[j]) {
            ++cmd_rejected_[j];
            continue;   /* SDK 仍沿用上一次目标（相当于保持），不会因为这条而跳变 */
        }

        /* ---- 线性插值（`command.interpolation=linear`）----
           ⚠ 顺序：**先过模式校验**再插值。被拒的命令不该开始一段新轨迹（否则
              "被拒的"命令仍然会拖着关节走完那一段，与"丢弃并计数"自相矛盾）。 */
        JointTarget eff = t;
        if (interp_mode_ == Interpolation::kLinear) {
            if (interp_seq_[j] != cmd.seq) {          /* 本 tick 收到新命令 */
                interp_seq_[j] = cmd.seq;
                if (interp_have_[j]) {
                    /* 起点 = 上一段的**终点意图**（不是上次真正发出去的插值中间值）——
                       后者会在命令到达很密时使关节越走越慢。 */
                    interp_from_[j] = interp_to_[j];
                    interp_to_[j] = t;
                    const std::uint64_t dt = tick_now_ns_ - interp_last_cmd_ns_[j];
                    /* 段长 = **实测**的相邻命令间隔（就是控制器周期），夹在
                       [0.5 ms, 200 ms]：0.5 ms 防除零/噪声，200 ms 防"很久没来命令"
                       被当成一段超长斜坡（那种情况应当立即到位，风险由超时闸管）。 */
                    const std::uint64_t lo = 500000u;
                    const std::uint64_t hi = 200000000u;
                    interp_dt_ns_[j] = (dt < lo) ? lo : ((dt > hi) ? hi : dt);
                    interp_t0_ns_[j] = tick_now_ns_;
                    interp_active_[j] = true;
                } else {
                    /* 首条命令**立即生效**：没有历史区间可插，从 0 慢慢爬上去是安全问题。 */
                    interp_to_[j] = t;
                    interp_active_[j] = false;
                    interp_have_[j] = true;
                }
                interp_last_cmd_ns_[j] = tick_now_ns_;
            }
            if (interp_active_[j]) {
                const std::uint64_t elapsed = tick_now_ns_ - interp_t0_ns_[j];
                const double f = (interp_dt_ns_[j] == 0u)
                                     ? 1.0
                                     : static_cast<double>(elapsed) /
                                           static_cast<double>(interp_dt_ns_[j]);
                lerp_motion(interp_from_[j], interp_to_[j], (f > 1.0) ? 1.0 : f, &eff);
                if (f >= 1.0) interp_active_[j] = false;   /* 到位 → 后面直接跟随 */
            }
        }

        /* 记账：本 tick 实际下发的是什么（插值后的生效值）——诊断/测试的可观测面。 */
        applied_[j] = eff;

        switch (eff.mode) {
        case CmdMode::kCsp:            jsdk_joint_set_target_position_rad(joints_[j], eff.position);
            if (eff.velocity_limit > 0.0 || eff.current_limit > 0.0) {
                jsdk_joint_set_limits(joints_[j], eff.velocity_limit, eff.current_limit);
            }
            break;
        case CmdMode::kCsv:
            jsdk_joint_set_target_velocity_rad_s(joints_[j], eff.velocity);
            if (eff.velocity_limit > 0.0 || eff.current_limit > 0.0) {
                jsdk_joint_set_limits(joints_[j], eff.velocity_limit, eff.current_limit);
            }
            break;
        case CmdMode::kCst:
            /* CST 的力矩走**输出端** API（SDK 内部按 gear_ratio 换算后再编码）。 */
            jsdk_joint_set_target_torque_Nm(joints_[j], eff.torque);
            break;
        case CmdMode::kCurrent:
            jsdk_joint_set_current_A(joints_[j], eff.current);
            if (t.current_limit > 0.0) {
                jsdk_joint_set_limits(joints_[j], 0.0, t.current_limit);
            }
            break;
        case CmdMode::kMit:
        default:
            if (t.si_gain) {
                jsdk_joint_set_mit_stiffness(joints_[j], t.position, t.velocity, t.stiffness,
                                             t.damping, t.torque);
            } else {
                jsdk_joint_set_mit(joints_[j], t.position, t.velocity, t.kp, t.kd, t.torque);
            }
            break;
        }
        last_target_[j] = t;
        has_target_[j] = true;
    }
}

bool BusRuntime::try_broadcast_mit() noexcept
{
    if (ctx_ == nullptr || joint_count_ == 0u) return false;

    /* Classic：SDK 只支持"全员同一目标"（槽位 0），收益小且容易造成误解 ——
       直接走单播，并把原因写进备注。 */
    if (!bus_.is_fd) {
        set_note("broadcast disabled on a Classic link (SDK only supports slot 0 / same target for "
                 "all); using per-joint unicast frames");
        return false;
    }

    jsdk_group_target_t targets[kMaxJointsPerBus];
    std::memset(targets, 0, sizeof targets);

    for (unsigned j = 0u; j < joint_count_; ++j) {
        const std::uint8_t nid = bus_.joints[j].node_id;
        if (nid > kMaxBroadcastNodeId) {
            set_note("broadcast degraded to unicast: a joint has node_id > 7 (bitmap cannot "
                     "address it)");
            return false;
        }
        if (jsdk_joint_is_enabled(joints_[j]) == 0) {
            set_note("broadcast degraded to unicast: not all joints are enabled yet");
            return false;
        }
        if (jsdk_joint_get_mode_state(joints_[j]) != JSDK_MODESTATE_MIT) {
            set_note("broadcast degraded to unicast: a joint is not in MIT mode (a MIT broadcast "
                     "would silently switch it back to MIT input mode)");
            return false;
        }
        if (!has_target_[j]) {
            /* 还没收到过命令 → 让 SDK 走单播（它会发"安全首帧"）。 */
            set_note("broadcast degraded to unicast: a joint has not received a target yet");
            return false;
        }

        const JointTarget &t = last_target_[j];
        targets[j].node_id = nid;
        targets[j].pos_rad = t.position;
        targets[j].vel_rad_s = t.velocity;
        targets[j].tau_Nm = t.torque;
        if (t.si_gain) {
            /* 广播路径 SDK 只收线上 kp/kd：与 set_mit_stiffness 同一换算。 */
            const double gear = report_.joint[j].gear_ratio;
            if (gear <= 0.0) {
                set_note("broadcast degraded to unicast: joint is not calibrated, cannot convert "
                         "stiffness/damping to wire kp/kd");
                return false;
            }
            const double k = 2.0 * 3.14159265358979323846 / gear;
            targets[j].kp = t.stiffness * k;
            targets[j].kd = t.damping * k;
        } else {
            targets[j].kp = t.kp;
            targets[j].kd = t.kd;
        }
    }

    const jsdk_status_t st = jsdk_group_set_mit(ctx_, targets, joint_count_);
    if (st != JSDK_OK) {
        set_note("broadcast failed, using unicast: see SDK last_error");
        return false;
    }
    /* 成功：清掉上一次的降级原因（避免陈旧备注让人误以为一直在降级）。 */
    set_note("broadcast: 1 frame drives all joints this tick");
    return true;
}

void BusRuntime::set_note(const char *text) noexcept
{
    std::snprintf(snapshot_note_, sizeof(snapshot_note_), "%s", text != nullptr ? text : "");
}

void BusRuntime::tick_end(std::uint64_t /*now_ns_in*/) noexcept
{
    if (ctx_ == nullptr) return;
    const jsdk_status_t st = jsdk_context_cycle_end(ctx_);
    if (st != JSDK_OK && st != static_cast<jsdk_status_t>(last_cycle_status_)) {
        last_cycle_status_ = static_cast<int>(st);
    }
}

void BusRuntime::estop_now() noexcept
{
    if (ctx_ != nullptr) {
        jsdk_context_estop(ctx_);
        entered_estop_ = true;
    }
    /* 安全动作后**必须**清掉插值历史：否则复位后第一条命令会从一段旧轨迹
       慢慢爬（而不是立即到位）。
       做法：丢掉历史 ⇒ `interp_have_` 为假 ⇒ 下一条命令走"首条立即生效"分支。 */
    for (unsigned j = 0u; j < kMaxJointsPerBus; ++j) {
        interp_active_[j] = false;
        interp_have_[j] = false;
    }
}

void BusRuntime::hold_all(double stiffness_nm_per_rad, double damping_nm_s_per_rad) noexcept
{
    if (ctx_ == nullptr) return;
    /* 安全动作后不留插值历史（否则复位后第一条命令会从旧轨迹慢慢爬）。 */
    for (unsigned j = 0u; j < kMaxJointsPerBus; ++j) {
        interp_active_[j] = false;
        interp_have_[j] = false;
    }
    const bool si = (stiffness_nm_per_rad > 0.0) || (damping_nm_s_per_rad > 0.0);
    for (unsigned j = 0u; j < joint_count_; ++j) {
        /* ⚠ 非 MIT 模式的关节**只能用 `hold_position()`**：SDK 会根据该关节的模式发
           “最小能量”帧（CSP → 目标=实际位置；CSV/CST/CURRENT → 零）。
           给它们发 MIT 的 PD 帧会把关节**当场换回 MIT 输入模式**（与广播路径同一条风险），
           而且 CST/CURRENT 下“保持”本就是零力矩 —— 这条事实必须让客户在文档里看到
           （发起侧在 `jr_bus_node` 的 configure 里给出 WARN）。 */
        if (joint_mode_[j] != static_cast<std::uint8_t>(CmdMode::kMit)) {
            jsdk_joint_hold_position(joints_[j]);
            continue;
        }
        if (!si) {
            /* 保持位置、不主动出力（SDK 语义：MIT 下 kp=kd=tau=0，目标 = 当前位置）。 */
            jsdk_joint_hold_position(joints_[j]);
            continue;
        }
        /* ⚠ SDK 的 `hold_position_pd()` 接的是**线上 kp/kd**（作用在电机端 turns 误差上），
           而客户给的是输出端物理量。换算与 `set_mit_stiffness()` 同一公式：
               线上值 = 物理量 × 2π / gear_ratio
           取不到 gear_ratio（未标定）时**不下发 PD**，退化为 hold_position ——
           宁可不抱持，也不拿一个猜出来的刚度去驱动电机。 */
        const double gear = report_.joint[j].gear_ratio;
        if (gear <= 0.0) {
            jsdk_joint_hold_position(joints_[j]);
            continue;
        }
        const double k = 2.0 * 3.14159265358979323846 / gear;
        jsdk_joint_hold_position_pd(joints_[j], stiffness_nm_per_rad * k,
                                    damping_nm_s_per_rad * k);
    }
}

void BusRuntime::zero_torque_all() noexcept
{
    if (ctx_ == nullptr) return;
    for (unsigned j = 0u; j < joint_count_; ++j) {
        jsdk_joint_hold_position(joints_[j]);   /* MIT: pos=实际, kp=kd=tau=0 → 泄力 */
    }
    /* 同上：安全动作后不留插值历史。 */
    for (unsigned j = 0u; j < kMaxJointsPerBus; ++j) {
        interp_active_[j] = false;
        interp_have_[j] = false;
    }
}

void BusRuntime::request_disable_all() noexcept
{
    if (ctx_ == nullptr) return;
    for (unsigned j = 0u; j < joint_count_; ++j) {
        jsdk_joint_request_disable(joints_[j]);
    }
}

void BusRuntime::fill_snapshot(StateSnapshot &s, unsigned bus_index, unsigned joint_base) noexcept
{
    if (ctx_ == nullptr || bus_index >= kMaxBuses) return;

    jsdk_bus_state_t bs;
    std::memset(&bs, 0, sizeof bs);
    if (jsdk_context_get_bus_state(ctx_, &bs) == JSDK_OK) {
        BusStatsPOD &b = s.buses[bus_index];
        b.link_up = (bs.link_up != 0u);
        b.tx_frames = bs.tx_frames;
        b.rx_frames = bs.rx_frames;
        b.tx_failed = bs.tx_failed;
        b.rx_dropped = bs.rx_dropped;
        b.keepalive_sent = bs.keepalive_sent;
        b.link_errors = bs.link_errors;
        b.last_rx_age_ms = bs.last_rx_age_ms;
        b.hal_bus_flags = bs.hal_bus_flags;
        b.nodes_online = bs.nodes_online;
    }
    BusStatsPOD &b = s.buses[bus_index];
    std::snprintf(b.name, sizeof(b.name), "%s", bus_.name);
    b.bus_load_estimate = report_.plan.load;
    b.degraded = (last_cycle_status_ != 0) || entered_estop_;
    std::snprintf(b.last_note, sizeof(b.last_note), "%s", snapshot_note_);

    for (unsigned j = 0u; j < joint_count_; ++j) {
        const unsigned g = joint_base + j;
        if (g >= kMaxJoints) break;

        jsdk_joint_feedback_t fb;
        std::memset(&fb, 0, sizeof fb);
        const bool have = (jsdk_joint_get_feedback(joints_[j], &fb) == JSDK_OK);

        JointStatePOD &o = s.joints[g];
        /* ⚠ 名字必须填：ROS 侧按名字投影消息（`JointFeedback.name` / `JointState.name`）。
           漏填不会编译报错，但**发给客户的话题里全是空字符串** —— 由
           `test_jr_bus_node` 的"按名字找关节"用例抓出来的（核心测试当时没覆盖到）。 */
        std::snprintf(o.name, sizeof(o.name), "%s", bus_.joints[j].name);
        o.bus_index = static_cast<std::uint8_t>(bus_index);
        o.node_id = static_cast<std::uint8_t>(bus_.joints[j].node_id);
        if (have) {
            o.position = fb.pos;
            o.velocity = fb.vel;
            o.current = fb.current_A;
            o.effort = fb.torque_Nm;
            o.motor_temperature = fb.t_motor_C;
            o.fet_temperature = fb.t_fet_C;
            o.bus_voltage = fb.vbus_V;
            o.bus_current = fb.ibus_A;
            o.age_ms = fb.age_ms;
            o.err_code = fb.err_code;
            o.hb_error = fb.hb_error;
            o.axis_error = fb.axis_error;
            o.mode_state = static_cast<std::uint16_t>(fb.mode_state);
            o.status_flags = fb.status_flags;
            o.tx_rejected = fb.tx_rejected;
            o.tx_frames = fb.tx_frames;
            o.online = (fb.online != 0);
            o.valid_fresh = (fb.valid != 0);
            o.target_rejected = (fb.status_flags & 0x0001u) != 0u;      /* JSDK_JF_TARGET_REJECTED */
            o.feedback_stale = (fb.status_flags & 0x0008u) != 0u;       /* JSDK_JF_FEEDBACK_STALE */
            o.watchdog_unverified = (fb.status_flags & 0x0040u) != 0u;  /* JSDK_JF_WATCHDOG_UNVERIFIED */
        }
        o.enabled = (jsdk_joint_is_enabled(joints_[j]) != 0);
        o.calibrated = report_.joint[j].calibrated;
        o.cmd_mode = joint_mode_[j];
        o.cmd_rejected = cmd_rejected_[j];
    }
}

}  // namespace rt
}  // namespace jr
