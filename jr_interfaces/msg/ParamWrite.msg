# 一次参数写请求项（WriteParams 的输入）。
#
# 两种给值方式（**优先文本**）：
#   ① `text_value` 非空 → 按**端点声明的类型**解析文本；超值域/格式错 → 直接拒绝（绝不截断）；
#   ② `text_value` 为空 → 用下面的类型化字段，此时 `type` 必须给出（TYPE_*）。
# 推荐用 ①：这样客户不必知道端点是 u16 还是 u32，由节点查描述符后按声明类型装箱。

string joint                       # 关节名；**空 = 本总线唯一关节**（总线上有多个关节时必须点名，不猜）
string path                        # 完整端点路径
uint8 type                         # TYPE_*（0 = 由节点查描述符决定；用 ① 时可留 0）

bool bool_value
int64 int64_value
uint64 uint64_value
float64 double_value
string text_value                  # 见上面 ①（推荐）
