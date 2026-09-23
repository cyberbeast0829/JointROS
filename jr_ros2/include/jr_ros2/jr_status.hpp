/**
 * @file    jr_status.hpp
 * @brief   JointROS 核心库状态码与"给客户可编程的建议"枚举（无 ROS 依赖）
 *
 * @details
 *  设计原则（沿用 JointSDK 的一贯要求）：**不欺骗**。
 *   - 每个错误都保留可读文本（调用方自己拼），并且**区分** "我请求了" 与 "我验证到了"；
 *   - 需要恢复动作时，返回 `Advice`（机器可判别的建议码），而不是把提示塞进字符串里
 *     让上层去 substring —— 客户代码据此自动决策（例如"要断电重启"vs"重试一次"）。
 */

#ifndef JR_ROS2_JR_STATUS_HPP
#define JR_ROS2_JR_STATUS_HPP

#include <cstdint>

namespace jr {

/** 核心库状态码。数值不对外承诺（与 SDK 的 jsdk_status_t 有意区分，避免混淆）。 */
enum class Status : std::int32_t {
    kOk = 0,
    kInvalidArgument,   /**< 参数非法（含越界、缺失必需项） */
    kInvalidState,      /**< 当前状态不允许该操作（如 ACTIVE 时下载描述符） */
    kTimeout,           /**< 等待超时（含设备未响应） */
    kTransport,         /**< 链路/HAL 层错误 */
    kProtocol,          /**< 设备侧拒绝或返回非法值 */
    kNotFound,          /**< 路径/端点不存在（**不做模糊匹配**，不猜） */
    kNotSupported,      /**< 本构建未启用该后端/功能 */
    kNoMemory,          /**< 分配失败（含 SDK 上下文容量不足） */
    kNotCalibrated,     /**< 设备未标定，物理量 API 不可用 */
    kLocked,            /**< 总线已被其它进程/组件占用（单 master 纪律） */
    kIoError,           /**< 文件/缓存/锁文件读写失败 */
    kUnverified,        /**< 写入被接受但**读回不一致**（不是失败，也不是成功） */
    kInternal           /**< 内部不变量被破坏（应视为 bug） */
};

/** 恢复建议。**给代码用**，不是给人看的文本。 */
enum class Advice : std::uint8_t {
    kNone = 0,
    kRetryFaultReset,            /**< 先试 CLEAR_ERRORS（不动电机） */
    kNeedsDeviceReset,           /**< 需要 RESET_DEVICE 或断电重启（estop 锁存实测如此） */
    kCheckBusTermination,        /**< 查终端电阻/线序（bus-off/error-passive） */
    kCheckBusConfig,             /**< 查位定时/FD 开关/通道名与设备侧是否一致 */
    kReduceRateOrRaiseWatchdog,  /**< 控制周期与设备 break_timeout 不匹配 */
    kCheckTemperature,           /**< 过温 */
    kCheckSupplyVoltage,         /**< 母线电压异常（注意固件把过压报成 UNDER_VOLTAGE） */
    kCalibrationRequired,        /**< 需先标定 */
    kEnsureSingleMaster,         /**< 同总线可能有第二个主站 */
    kCheckRtPermissions,         /**< SCHED_FIFO/mlockall 失败 → 实时性无保证 */
    kFixBusPlanning              /**< 关节数/反馈策略/波特率组合不可行（见 bus_plan） */
};

const char *to_string(Status s) noexcept;
const char *to_string(Advice a) noexcept;

/**
 * 统一的结果载体：状态 + 建议 + 可读文本（固定缓冲，非 RT 路径使用）。
 *
 * ⚠ 文本截断是**显式**的（`truncated` 置位），不静默。
 */
struct Result {
    Status status = Status::kOk;
    Advice advice = Advice::kNone;
    bool   truncated = false;
    /* ⚠ 这个尺寸是“错误文本的容量”：它必须装得下**可操作的那句话**。
       踩过：总线锁的报错（含持有者 PID + 逃生舱）比 192 字节长，结果用户看到的是
       被砍掉的半句话 —— `truncated` 标记确实置了 true，但现场读到的信息已经废了。 */
    char   message[512] = {};

    bool ok() const noexcept { return status == Status::kOk; }

    /** 拼一条可读文本（printf 风格）；非 RT 路径使用。 */
    void set(Status s, Advice a, const char *fmt, ...) noexcept;

    void set_from(const Result &other) noexcept;
};

}  // namespace jr

#endif /* JR_ROS2_JR_STATUS_HPP */
