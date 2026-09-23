#!/usr/bin/env bash
# ============================================================================
#  JTC 端到端 demo（WP3 验收）：起 controller_manager + JTC，走一条真实轨迹
# ----------------------------------------------------------------------------
#  为什么用脚本而不是 launch：容器里没有图形/交互，脚本能把
#  "起 CM → 加载控制器 → 发目标 → 断言 → 收尾"整条链的**退出码**带出来，
#  直接进 CI。客户要看"标准写法"的话，这里的每一步都是标准 CLI 调用。
#
#  用法（容器内，colcon 已 install）：
#      JR_DEMO_INSTALL=/tmp/jr-colcon/install bash jr_ros2_control/test/jtc_demo/run.sh
#      JR_DEMO_INSTALL=... bash jr_ros2_control/test/jtc_demo/run.sh --mode csp
#
#  `--mode mit|csp`（也可用 env `JR_DEMO_MODE`，默认 mit）走的是**两份配置 + 两种接口集**：
#    mit：三件套 + 增益（控制律在增益上）；csp：只导出 position（帧里没增益位置）。
#    ⚠ 接口集**同时**决定 URDF 里声明什么、控制器 claim 什么（三者必须一致，见下文生成处）。
#    两者跑同一条轨迹、同一套断言 —— “换模式不该改变轨迹精度”。
# ============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL="${JR_DEMO_INSTALL:?需要 JR_DEMO_INSTALL=<colcon install 目录>}"
DISTRO="${ROS_DISTRO:?需要 source 过 ROS}"

MODE="${JR_DEMO_MODE:-mit}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --mode=*) MODE="${1#*=}"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

JR_CFG="${HERE}/jr_vbus.yaml"
# controllers.yaml 里默认写的是 `command_interfaces: [position, velocity]`（MIT 下两者都有）。
# ⚠ CSP 关节**只导出 position** ⇒ 控制器必须只 claim position；
#   这就是“同一个控制器不能混模式”的具体表现（JTC 默认会 claim position+velocity）。
CTRL_IFACES='[position, velocity]'
case "${MODE}" in
  mit) ;;
  csp)
    JR_CFG="${HERE}/jr_vbus_csp.yaml"
    CTRL_IFACES='[position]'
    ;;
  *) echo "unknown mode '${MODE}' (expected mit|csp)" >&2; exit 2 ;;
esac
echo "[demo] mode=${MODE}  config=$(basename "${JR_CFG}")  JTC claims ${CTRL_IFACES}"

WORK=/tmp/jr-jtc-demo
rm -rf "$WORK"
mkdir -p "$WORK"

# ⚠ ROS 的 setup.bash 与 `set -u` 不兼容（引用未定义变量）→ source 前后临时关掉。
set +u
# shellcheck disable=SC1091
source "/opt/ros/${DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${INSTALL}/setup.bash"
set -u

# ---- 生成 robot_description（替换配置路径 + **按模式**填命令接口块） ----
# ⚠ 命令接口必须与组件**实际导出**的集合逐一对上（CSP 关节只导出 position）。
#   对不上时 controller_manager 直接拒初始化硬件：
#     "Discrepancy between robot description file (urdf) and actually exported HW interfaces"
#   （实测还伴随 CM 在 pal_statistics 里段错误——所以这里不能靠"多写几个接口不碍事"。）
python3 - "${HERE}/jr_vbus.urdf.in" "${WORK}/jr_vbus.urdf" "${JR_CFG}" "${MODE}" <<'PY'
import sys
tpl_path, out_path, cfg, mode = sys.argv[1:5]
if mode == "csp":
    lines = [
        "<!-- CSP：**只导出 position**。帧里没有 velocity/effort 通道，也没有增益位置，",
        "     所以控制器只能 claim position（JTC 的 command_interfaces 同步改成 [position]）。 -->",
        '<command_interface name="position"/>',
    ]
else:
    lines = [
        '<command_interface name="position"/>',
        '<command_interface name="velocity"/>',
        '<command_interface name="effort"/>',
        "<!-- kp/kd 声明出来但**没有控制器 claim**：其值保持组件给的默认值",
        "     （来自 YAML 的 limits.<joint>.stiffness/damping）——这正是 JTC 场景需要的。 -->",
        '<command_interface name="kp"/>',
        '<command_interface name="kd"/>',
    ]
block = "\n".join("      " + ln for ln in lines)
with open(tpl_path, encoding="utf-8") as f:
    text = f.read()
text = text.replace("@JR_CONFIG@", cfg).replace("@CMD_BLOCK@", block)
with open(out_path, "w", encoding="utf-8", newline="") as f:
    f.write(text)
n = text.count("<command_interface")
print(f"[demo] 生成 URDF：{out_path}（mode={mode}，command_interface x{n}）")
PY
# 自检：① 占位符必须都消失（模板注释里若写了 token 会被一起替换 —— 实测踩过一次，
#          症状是 URDF 里凭空多出 5 条 command_interface，CM 才对账就炸）
#       ② 接口数必须是该模式的期望值（mit 5×2、csp 1×2）
if grep -qF '@CMD_BLOCK@' "${WORK}/jr_vbus.urdf" || grep -qF '@JR_CONFIG@' "${WORK}/jr_vbus.urdf"; then
  echo "[demo] ERROR: URDF 里还有没替换掉的占位符（模板注释里写了 token？）" >&2
  exit 2
fi
EXPECT_CMD=$([[ "${MODE}" == "csp" ]] && echo 2 || echo 10)
GOT_CMD="$(grep -c '<command_interface' "${WORK}/jr_vbus.urdf")"
if [[ "${GOT_CMD}" != "${EXPECT_CMD}" ]]; then
  echo "[demo] ERROR: URDF 里 command_interface 数 ${GOT_CMD} ≠ ${EXPECT_CMD}（模板改过？）" >&2
  exit 2
fi

# 参数文件 = 生成的 controller_manager 段（含长字符串 robot_description）+ 静态控制器段。
# 为什么要生成：URDF 是多行长字符串，手写进 YAML 不现实；ROS 侧标准做法就是
# 把 URDF 作为 `robot_description` 参数传给 ros2_control_node。
{
  echo "controller_manager:"
  echo "  ros__parameters:"
  echo "    update_rate: 100"
  # ⚠ 控制器的 `type` 必须声明在 **controller_manager 自己的参数段**下：
  #   CM 是先看这里知道"要加载哪个插件类"，再去下面各自的段落取参数。
  #   写成控制器自己段落里的 `type` 会报：
  #   "The 'type' param was not defined for 'joint_state_broadcaster'."
  echo "    joint_state_broadcaster:"
  echo "      type: joint_state_broadcaster/JointStateBroadcaster"
  echo "    joint_trajectory_controller:"
  echo "      type: joint_trajectory_controller/JointTrajectoryController"
  echo "    robot_description: |"
  sed 's/^/      /' "${WORK}/jr_vbus.urdf"
} > "${WORK}/params.yaml"

# ⚠ 控制器**自己的**参数段要单独成一个文件，用 `spawner --param-file` 显式传给控制器：
#   Jazzy/Humble 上，把顶层控制器段（`<ctrl>: ros__parameters:`）混在 CM 的
#   `--params-file` 里也能生效（CM 建控制器子节点时会继承全局参数）；
#   **Lyrical 上不行**——实测 JTC init 直接失败：
#     "Invalid value set during initialization for parameter 'joints':
#      Length of parameter 'joints' is '0' but must be greater than '0'"
#   显式 `--param-file` 是 ros2_control 文档里的标准做法，三个发行版都稳。
CTRL_PARAMS="${WORK}/controllers.yaml"
grep -v '^#' "${HERE}/controllers.yaml" \
  | sed "s#command_interfaces: \[position, velocity\]#command_interfaces: ${CTRL_IFACES}#" \
  > "${CTRL_PARAMS}"
# 自检：接口集真的被改到了（否则 CSP 变体会以一句难懂的 claim 失败收场）
if ! grep -qF "command_interfaces: ${CTRL_IFACES}" "${CTRL_PARAMS}"; then
  echo "[demo] ERROR: 没能把 command_interfaces 改成 ${CTRL_IFACES}（controllers.yaml 改过？）" >&2
  exit 2
fi

CM_LOG="${WORK}/cm.log"
RSP_LOG="${WORK}/rsp.log"

# ⚠ 为什么必须有 robot_state_publisher：
#   **Jazzy 的 controller_manager 不从参数读 URDF，而是订阅 `/robot_description` 话题**
#   （日志原话："Subscribing to '/robot_description' topic for robot description" +
#     "Waiting for data on 'robot_description' topic to finish initialization"）。
#   Humble 读的是同名**参数**。两边都兼顾的做法：参数照样传（params.yaml 里有），
#   同时用标准的 robot_state_publisher 把同一个 URDF 发到话题上。
echo "[demo] 启动 robot_state_publisher（发布 /robot_description；日志：${RSP_LOG}）"
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat "${WORK}/jr_vbus.urdf")" > "${RSP_LOG}" 2>&1 &
RSP_PID=$!

sleep 1   # 让话题先就绪（CM 起来后如果没人发就会一直等）

echo "[demo] 启动 ros2_control_node（日志：${CM_LOG}）"
# shellcheck disable=SC2086
ros2 run controller_manager ros2_control_node --ros-args --params-file "${WORK}/params.yaml" \
  > "${CM_LOG}" 2>&1 &
CM_PID=$!
# ⚠ `ros2 run` 只是一层 python 包装，**真正的节点是它的子进程**：
#   只 kill $CM_PID 会留下孤儿的 ros2_control_node 继续跑，而它还占着
#   `/var/lock/jr-vbus.lock`（flock，进程活着就不放）→ 紧接着跑第二个 mode 时组件起不来：
#     "[FATAL] open failed: bus_lock: '.../jr-vbus.lock' is held by pid NNNN"
#   实测：Lyrical 侥幸过了（退出得快），Humble/Jazzy 直接红 —— 这种"看运气"的收尾必须修。
kill_tree() {
  local pid="$1" kid
  for kid in $(pgrep -P "${pid}" 2>/dev/null || true); do
    kill_tree "${kid}"
  done
  kill "${pid}" 2>/dev/null || true
}

# ⚠ 失败时**必须**把 CM 日志吐出来：否则只剩一句"spawner 连不上服务"，
#    而真正的原因（插件加载失败/URDF 解析失败/我们 on_init 的 FATAL）全在日志里。
cleanup() {
  local rc=$?
  if [[ ${rc} -ne 0 ]]; then
    echo "===================== controller_manager 日志（尾部 80 行） ====================="
    tail -80 "${CM_LOG}" 2>/dev/null || true
    echo "==============================================================================="
  fi
  kill_tree "${CM_PID}" 2>/dev/null || true
  kill_tree "${RSP_PID}" 2>/dev/null || true
  # 等节点**真的消失**再退出：进程还在 = 锁还在 = 下一个 mode 立刻撞锁。
  local i
  for i in $(seq 1 60); do
    pgrep -f "ros2_control_node" >/dev/null 2>&1 || break
    sleep 0.1
  done
  pkill -9 -f "ros2_control_node" 2>/dev/null || true   # 兜底（容器内只有我们这一个 CM）
  wait "${CM_PID}" 2>/dev/null || true
  wait "${RSP_PID}" 2>/dev/null || true
  return 0
}
trap cleanup EXIT

echo "[demo] 加载 joint_state_broadcaster"
ros2 run controller_manager spawner joint_state_broadcaster \
  --param-file "${CTRL_PARAMS}" \
  --controller-manager /controller_manager --controller-manager-timeout 30

echo "[demo] 加载 joint_trajectory_controller"
ros2 run controller_manager spawner joint_trajectory_controller \
  --param-file "${CTRL_PARAMS}" \
  --controller-manager /controller_manager --controller-manager-timeout 30

echo "[demo] 发轨迹并断言"
set +e
python3 "${HERE}/jtc_goal.py"
RC=$?
set -e

if [[ ${RC} -ne 0 ]]; then
  echo "===================== controller_manager 日志（尾部） ====================="
  tail -60 "${CM_LOG}" || true
  echo "========================================================================"
fi
exit "${RC}"
