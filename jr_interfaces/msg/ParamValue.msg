# 一次参数读的结果项。
#
# ⚠ `type` 是**端点声明的类型**，不是"我们猜的类型"。SDK 侧真实踩过：
#   按普通整数猜成 u32 → u16 端点被设备以 `descriptor=uint16 given=uint32` 拒绝。
#   本接口因此把类型显式带出来，客户代码按 `type` 取值。
#
# 取值规则（deterministic，避免"两个字段都可能对"的含糊）：
#   TYPE_BOOL         → bool_value
#   TYPE_U8..U32, I8..I64 → int64_value
#   TYPE_U64          → uint64_value（一律用它，即使值装得进 int64）
#   TYPE_F32/F64      → double_value
#   text_value        → 人类可读，**不要**拿它做数值判断

uint8 TYPE_UNSUPPORTED=0           # object / json / endpoint_ref / function：不可读写
uint8 TYPE_BOOL=1
uint8 TYPE_U8=2
uint8 TYPE_I8=3
uint8 TYPE_U16=4
uint8 TYPE_I16=5
uint8 TYPE_U32=6
uint8 TYPE_I32=7
uint8 TYPE_U64=8
uint8 TYPE_I64=9
uint8 TYPE_F32=10
uint8 TYPE_F64=11

string joint                       # 关节名（同一路径可能在多个关节上都读）
string path                        # 完整端点路径，如 axis0.motor.config.gear_ratio
uint8 type                         # TYPE_*

bool bool_value
int64 int64_value
uint64 uint64_value
float64 double_value
string text_value

uint8 status                       # 见 JointResult 的 STATUS_* 常量
string message
