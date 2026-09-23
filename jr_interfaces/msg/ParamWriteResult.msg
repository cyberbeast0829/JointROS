# 一次参数写的结果（与请求逐项一一对应）。
#
# ⚠ 本消息刻意把 `requested_*` 与 `value_*` **分开**（ADR-8）：
#   把"我请求的值"当成"结果"打印，就是"说得比知道的多"。
#   读不回来时 `value` 全为 0 且 `type = TYPE_UNSUPPORTED`。

string joint
string path
uint8 type                         # 端点声明类型（TYPE_*，见 ParamValue）

bool requested_bool                # 我请求写入的值
int64 requested_int64
uint64 requested_uint64
float64 requested_double

bool value_bool                    # **读回**的值
int64 value_int64
uint64 value_uint64
float64 value_double

bool verified                      # 读回与请求一致
bool consumed_by_firmware          # 写进去就被固件立即消费（如 axis0.requested_state）：
                                  # 这类端点 verified 必然为 false，**不算失败**，请看结果状态端点
bool persisted                     # 是否已落 Flash（仅在 persist=true 时可能为 true）

uint8 status                       # 见 JointResult 的 STATUS_* 常量
string message
