# 模式化目标（二级命令接口，ADR-4；机械臂/ros2_control 之外的客户常用）。
#
# 各模式的 `target` 线上单位与换算责任都在 SDK 侧（本包按 SDK 语义逐字透传）：
#   CSP     : 输出端 rad（位置）
#   CSV     : 输出端 rad/s
#   CST     : 输出端 N·m（SDK 内部按 gear_ratio 换算到电机端再编码）
#   CURRENT : **电机端** A（逃生通道；输出端力矩等价量需要 torque_constant 换算）
#
# ⚠ `mode` 必须与该关节**使能时**选择的模式（配置 `joints[].mode`）一致：
#   不一致的命令会被**拒绝并计入 `RtStats.cmd_rejected`**（不是静默按 MIT 发 ——
#   那会改掉设备的输入模式，叠加 kp=kd=tau=0 ⇒ 表现为关节泄力）。
#   切模式是阻塞动作（disable→enable，ADR-7 窗口），不能在命令路径上做：改配置后
#   clean/configure/activate（或重启节点）。
#
# ⚠ CURRENT 的两条特例（SDK F19，已在 `validate_config` 里告警）：
#   ① `CURRENT_CONTROL` 不是协议里的 `is_ctrl` 帧 ⇒ **设备不会回任何响应**
#      （所以设备侧反馈电流看不到你发的值，只能靠抓帧验证）；
#   ② 它也不喂设备侧 `break_timeout` ⇒ 保护不武装；SDK 的自动补喂会**跳过** CURRENT 关节
#      （否则补喂的 MIT 帧会把输入模式顶掉）。

uint8 CSP=8
uint8 CSV=9
uint8 CST=10
uint8 CURRENT=11

string name
uint8 mode

float64 target                  # 单位随 mode（见上）
float64 velocity_limit          # 0 = 不限制（输出端 rad/s；CSP/CSV 有效；CST 无效）
float64 current_limit           # 0 = 不限制。⚠ 名字是历史遗留：它实际下发的是固件的
                                #   **力矩上限** `torque_lim`（正常工作上限，越界静默钳位），
                                #   不是过流告警门限 `current_lim`（越界会报错失能）。
                                #   单位按 **电机端 N·m** 解释（SDK `set_torque_limit_Nm()`）；
                                #   CSP/CSV/CURRENT 有效，CST 无效。
                                #   ⚠ 该值会**每帧**重写设备参数，0 = 沿用 SDK 默认（不写 0）。
