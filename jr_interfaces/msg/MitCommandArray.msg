# 批量 MIT 目标。
#
# 语义（重要）：**只更新列出的关节**；未列出的关节保持上一次目标，
# **不**自动泄力 —— 泄力由"命令超时"策略负责（`command.timeout_action`，§8.3）。
std_msgs/Header header

MitCommand[] commands
