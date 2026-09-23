# 故障边沿事件（ADR-13：**只上报，不自动"修好"**）。
#
# 凡是摘要/结论，都必须能追到原始位：因此本消息同时给
# `error_code`（4-bit 摘要）、`heartbeat_error`（5-bit 原始位）、
# `axis_error`（32-bit 原始位）与各子系统位。

uint8 APPEARED=1
uint8 CLEARED=2

# 恢复建议（给**代码**判别用，不是给人看的文本）。数值与核心库 jr::Advice 一一对应，
# 由 jr_ros2 的单元测试静态断言守住（改一处不改另一处就会红）。
uint8 ADVICE_NONE=0
uint8 ADVICE_RETRY_FAULT_RESET=1             # 先试 CLEAR_ERRORS（不动电机）
uint8 ADVICE_NEEDS_DEVICE_RESET=2            # 需要软复位或断电重启（estop 锁存实测如此）
uint8 ADVICE_CHECK_BUS_TERMINATION=3         # 查终端电阻/线序
uint8 ADVICE_CHECK_BUS_CONFIG=4              # 查位定时/FD 开关/通道名
uint8 ADVICE_REDUCE_RATE_OR_RAISE_WATCHDOG=5 # 控制周期与设备 break_timeout 不匹配
uint8 ADVICE_CHECK_TEMPERATURE=6
uint8 ADVICE_CHECK_SUPPLY_VOLTAGE=7          # 注意固件把过压也报成 UNDER_VOLTAGE
uint8 ADVICE_CALIBRATION_REQUIRED=8
uint8 ADVICE_ENSURE_SINGLE_MASTER=9          # 同总线可能有第二个主站
uint8 ADVICE_CHECK_RT_PERMISSIONS=10         # SCHED_FIFO/mlockall 失败
uint8 ADVICE_FIX_BUS_PLANNING=11             # 关节数/反馈策略/波特率组合不可行

string name                     # 关节名
uint8 event                     # APPEARED | CLEARED

uint8 error_code                # MIT 4-bit 摘要
uint8 heartbeat_error           # 心跳 5-bit（原始位）
uint32 axis_error               # 32-bit 子系统错误位

uint32 motor_error
uint32 encoder_error
uint32 sensorless_error
uint32 controller_error
uint32 system_error

string text                     # SDK 的一行描述（排障入口）
uint8 advice                    # ADVICE_*（机器可判别）
