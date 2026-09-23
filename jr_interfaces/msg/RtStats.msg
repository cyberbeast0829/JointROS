# 实时性度量（DESIGN §6.1/§6.6）。所有 *_ns 都是 CLOCK_MONOTONIC 差值。
#
# 设计原则（ADR-14）：度量是一等公民 —— 不达标要**看得见**，
# 而不是靠"我感觉还行"。因此抖动/周期都给出 max 与 p99。

float64 rate_hz                 # 本 tick 组的期望频率
uint32 period_ns                # 期望周期

uint32 jitter_min_ns            # 0 = 还没有样本（`stat_samples` 为 0 时不要当成"抖动为 0"）
uint32 jitter_mean_ns
uint32 jitter_last_ns           # 最近一个周期（"现在怎么样"最直接的信号）
uint32 jitter_max_ns
uint32 jitter_p99_ns            # 固定分桶直方图统计（无分配）

uint32 cycle_min_ns             # 一次 tick 的耗时（cycle_begin..cycle_end）
uint32 cycle_mean_ns
uint32 cycle_last_ns
uint32 cycle_max_ns

uint32 cmd_to_tx_mean_ns        # 命令被 RT 取走到发出控制帧的时延
uint32 cmd_to_tx_last_ns
uint32 cmd_to_tx_max_ns         # 注：未给 p99（核心库只对抖动做了直方图；这里不伪造）

uint32 stat_samples             # 统计样本数（min/mean 据此判断"有没有样本"）
uint32 snapshot_to_pub_ns       # 快照生成到发布出去的时延（ROS 域，仅供参考）

uint64 missed_ticks             # 丢 tick 次数
uint64 tick_count               # 已完成周期数
uint32 config_pauses            # 因配置类操作让出所有权的次数

# 被**我们**丢掉的命令数（节点侧拒绝 + 核心侧丢弃，累计）。两类原因：
#   ① 节点侧：模式与该关节使能时的模式不符 / 非法模式值 / 非有限值 / 未知关节；
#   ② 核心侧：目标面到了 RT 侧才发现模式不符（两道防线中的第二道）。
# ⚠ 与 `JointFeedback.tx_rejected` 不是一回事：那个是**设备**说"这帧我不收"。
# "关节不动"时先看这里：非零说明命令压根没到设备。
uint32 cmd_rejected

# ── 实时权限/环境（成立与否必须显式可见）─────────────
bool rt_sched_ok                # SCHED_FIFO 生效
bool mlock_ok                   # 内存锁定生效
bool affinity_ok                # 绑核生效
bool rt_throttled               # 内核正在节流实时任务（抖动会超预期）
