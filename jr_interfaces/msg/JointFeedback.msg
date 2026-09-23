# 单个关节的状态快照（DESIGN §6.1）。物理量一律**输出端** SI 单位。
#
# ⚠ `error_code` 是 MIT 4-bit **摘要**，会把不同故障映射成同一个值
#   （实测：estop 与看门狗都报 CAN_TIMEOUT）→ 排障请看 `heartbeat_error`（原始位）
#   与 `axis_error`（32-bit 原始位）。摘要只用于快速浏览。

std_msgs/Header header          # stamp = 本 tick 起始时刻换算到 ROS 时间基（§6.6）

string name                     # 关节名（与 YAML / URDF 对齐）

float64 position                # rad（输出端）
float64 velocity                # rad/s（输出端）
float64 effort                  # N·m（SDK 估算：current × torque_constant × gear）
float64 current                 # **电机端** A（不是输出端，注意与 effort 的区别）

float64 motor_temperature       # °C
float64 fet_temperature         # °C
float64 bus_voltage             # V（来自心跳）
float64 bus_current             # A（来自心跳）

uint32 age_ms                   # 距上次**有效**反馈的时间

uint32 status_flags             # SDK JSDK_JF_* 位（原始位；含 FEEDBACK_STALE / WATCHDOG_* / TARGET_REJECTED）
uint16 mode_state               # 固件原始 nibble（0..7），排障用
uint8 error_code                # MIT 4-bit 摘要（**别当结论**）
uint8 heartbeat_error           # 心跳 5-bit 子系统位图（原始位）
uint32 axis_error               # 32-bit 子系统错误位（QUERY_ERROR；未查询时为 0）
uint32 tx_rejected              # 累计被设备拒绝的越界指令数（>0 说明客户在发越界目标）

bool online                     # 设备在总线上（有新鲜响应/心跳）
bool enabled
bool calibrated                 # 物理量 API 可用（unit_scale.valid）
bool valid                      # **本周期有新数据**（false = 这条是上一周期的陈旧值）
