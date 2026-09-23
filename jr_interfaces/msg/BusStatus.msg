# 一条总线的链路/运行统计（DESIGN §6.1）。计数字段来自 SDK `jsdk_bus_state_t`，原样映射。

string bus

bool link_up                    # HAL 自报链路可用
uint8 nodes_online              # 最近统计窗口内在线节点数

# ── 链路计数（原样映射 SDK，不要在这里做"解释"）────
uint32 tx_frames
uint32 rx_frames
uint32 tx_failed
uint32 rx_dropped
uint32 keepalive_sent
uint32 link_errors

uint32 last_rx_age_ms           # 距上次收到任何帧
uint32 hal_bus_flags            # HAL 状态位（listen-only / error-passive / bus-off 等）

float32 bus_load_estimate       # 0..1+：按 §7.1 模型**估计**（是估计值，不是测量值）
uint64 tick_overruns            # 丢 tick 次数（实时性健康度）
uint64 command_overwrites       # 命令在 RT 取走前被覆盖的次数

# ── 需要用户知道的状态 ─────────────────────────────
bool degraded                   # 有降级（例如广播被降级为单播、命令超时策略生效）
string note                     # 最近一次要让人看到的文本（**降级原因必须出现在这里**）
