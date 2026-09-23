# MIT 目标（一级命令接口，ADR-4）。
#
# ⚠ `gain_mode` 必须显式给（ADR-5）：`kp` 在固件里作用在**电机端 turns 误差**上，
#   等效输出端刚度 = kp × gear_ratio / 2π。搞错 gain_mode 会得到"看起来能动但刚度量级不对"
#   的现场表现 —— 所以本接口**不允许**含糊。

uint8 WIRE=0                    # kp/kd 原样透传（线上值，作用在电机端）
uint8 SI=1                      # stiffness/damping 按**输出端**物理量

string name                     # 关节名；未列出的关节保持上一次目标
uint8 gain_mode                 # WIRE | SI

float64 position                # rad
float64 velocity                # rad/s
float64 kp                      # 仅 WIRE 模式使用
float64 kd                      # 仅 WIRE 模式使用
float64 stiffness               # 仅 SI 模式使用（N·m/rad，输出端）
float64 damping                 # 仅 SI 模式使用（N·m·s/rad，输出端）
float64 torque                  # N·m 前馈

bool enable                     # true = 本条命令同时请求使能（可选；默认不改变使能状态）
