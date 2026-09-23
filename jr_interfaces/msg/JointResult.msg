# 逐项（逐关节）操作结果。**批量优先**：一次请求多个关节就逐项给结果，
# 不允许用一句"部分成功"含糊过去。
#
# `status` 是核心库 `jr::Status` 的**对外稳定编号**（客户代码可以据此分支，
# 不必去 substring 那句中英文混排的 message）。映射由 jr_ros2 的单元测试静态断言守住。

uint8 STATUS_OK=0
uint8 STATUS_INVALID_ARGUMENT=1    # 参数非法（越界/缺失必需项）
uint8 STATUS_INVALID_STATE=2       # 当前状态不允许（例如 ACTIVE 时下载描述符）
uint8 STATUS_TIMEOUT=3             # 等设备超时
uint8 STATUS_TRANSPORT=4           # 链路/HAL 层错误
uint8 STATUS_PROTOCOL=5            # 设备侧拒绝或返回非法值
uint8 STATUS_NOT_FOUND=6           # 路径/端点不存在（**不猜**、不做模糊匹配）
uint8 STATUS_NOT_SUPPORTED=7       # 本构建未启用该后端/功能
uint8 STATUS_NO_MEMORY=8
uint8 STATUS_NOT_CALIBRATED=9      # 设备未标定 → 物理量 API 不可用
uint8 STATUS_LOCKED=10             # 总线已被其它进程/组件占用（单 master 纪律）
uint8 STATUS_IO_ERROR=11
uint8 STATUS_UNVERIFIED=12         # 写入被接受但**读回不一致**：既不是成功，也不是失败
uint8 STATUS_INTERNAL=13           # 内部不变量被破坏（是 bug，请上报）

string name                        # 关节名（或对象名）
bool success
uint8 status                       # STATUS_*
string message                      # 可读原因（含 SDK last_error 原文）
