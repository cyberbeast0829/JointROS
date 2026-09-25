#!/usr/bin/env bash
# ============================================================================
#  jr_bringup 示例冒烟：示例必须**真能跑起来**
# ----------------------------------------------------------------------------
#  §11 的 WP7 验收口径就两句：「`ros2 launch` 一条命令可复现」+「示例有 CI 冒烟」。
#  这个脚本就是第二句 —— 否则“示例”只是文档里的承诺。
#
#  环境（ctest 传入）：
#    JR_WS_SETUP = 工作区 setup.bash（由 CMake 从 CMAKE_INSTALL_PREFIX 算得）
#  ⚠ 这里一律走**安装空间**的 `ros2 run` / `ros2 launch`，不去猜 build 树里的路径。
#  退出码：0 = 全通过；1 = 有失败
# ============================================================================
set -u

: "${JR_WS_SETUP:?需要 JR_WS_SETUP=<工作区 setup.bash>}"
[ -f "$JR_WS_SETUP" ] || { echo "FAIL: 找不到 $JR_WS_SETUP"; exit 1; }

set +u
# shellcheck disable=SC1090
source "$JR_WS_SETUP"
set -u

PASS=0
FAIL=0
ok()  { PASS=$((PASS + 1)); echo "      [ok]   $*"; }
bad() { FAIL=$((FAIL + 1)); echo "      [FAIL] $*"; }

WORK="$(mktemp -d)"
LAUNCH_PID=""
kill_tree() {
  local pid="$1" kid
  for kid in $(pgrep -P "$pid" 2>/dev/null || true); do kill_tree "$kid"; done
  kill "$pid" 2>/dev/null || true
}
cleanup() {
  pkill -f "vbus_demo.launch.py" 2>/dev/null || true
  pkill -f "humanoid_2bus.launch.py" 2>/dev/null || true
  pkill -f "jr_bus" 2>/dev/null || true
  if [ -n "$LAUNCH_PID" ]; then
    kill_tree "$LAUNCH_PID"
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  rm -rf "$WORK"
}
trap cleanup EXIT

# 等某条总线上的节点**对外可用**（返回 0 = 好了）。
#
# ⚠ 判据为何是“服务可调”而不是 `ros2 lifecycle get`（v0.17 真机教训）：
#   ① 服务是 `on_activate` 里才建的（§13.3-36）⇒ **服务可调 ≡ 已经是 ACTIVE**，语义等价；
#   ② 这才是客户真正关心的“能不能用”；
#   ③ `ros2 lifecycle get` 是**冷启动 CLI**（没 daemon 时要现做发现），在真机 RT 宿主上
#      单次可能 >5 s，被外层 `timeout` 在发现完成前杀掉 ⇒ **无论预算多大都永远报“没到 active”**，
#      而 launch 自己的日志明明已打出“已 active”（实测：同一份日志里 `已 active` 出现 **7** 次、
#      5 条断言全 FAIL）—— 症状极具误导性，看起来像 launch 或预算坏了。
#   预算是**墙上时钟**（每轮探测自身要几百 ms~4 s，不能用“轮数 × 间隔”算）。
JR_READY_BUDGET_S="${JR_READY_BUDGET_S:-180}"
JR_READY_PROBE_S="${JR_READY_PROBE_S:-25}"
wait_ready() {
  local bus="$1" seconds="${2:-$JR_READY_BUDGET_S}" deadline
  deadline=$((SECONDS + seconds))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if timeout "$JR_READY_PROBE_S" ros2 run jr_ros2 jr_ctl --node "$bus" \
         --timeout-ms 4000 status > /dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

stop_launch() {
  pkill -f "$1" 2>/dev/null || true
  pkill -f "jr_bus" 2>/dev/null || true
  if [ -n "$LAUNCH_PID" ]; then kill_tree "$LAUNCH_PID"; wait "$LAUNCH_PID" 2>/dev/null || true; fi
  LAUNCH_PID=""
  sleep 2
}

echo "===== 0) 负向对照：等待器必须**能失败**（否则后头的“已就绪”只是空转）====="
if wait_ready vbusrp_nonexistent 12; then
  bad "对不存在的节点名等待器也返回成功 ⇒ 用例没牙齿"
else
  ok "对不存在的节点名等待器会超时失败（等待器有牙齿）"
fi

echo "===== ① 虚拟总线 demo：一条 ros2 launch 起 2 关节并走到 active ====="
ros2 launch jr_bringup vbus_demo.launch.py > "$WORK/vbus.log" 2>&1 &
LAUNCH_PID=$!
if wait_ready vbusrp; then
  ok "「ros2 launch jr_bringup vbus_demo.launch.py」→ /vbusrp 服务可调（= 已 active）"
else
  bad "${JR_READY_BUDGET_S} s 内服务没建起来（= 没到 active）"; tail -30 "$WORK/vbus.log"
fi
# ⚠ 服务是否真的建起来，才是生命周期节点的坑（§13.3-36）
if timeout 30 ros2 run jr_ros2 jr_ctl --node vbusrp --timeout-ms 5000 status > "$WORK/st.log" 2>&1; then
  ok "jr_ctl --node vbusrp status rc=0（服务可调）"
  # ⚠ 断言要落在**真的存在**的证据上：`status` 打印的是总线级快照，不含关节名表，
  #   所以这里查 `nodes_online`（SDK 的心跳在线数），关节寻址由下面那条 read 证。
  if grep -q "nodes_online=2" "$WORK/st.log"; then
    ok "总线上 2 个节点在线（nodes_online=2 ⇒ 配置里的 2 个关节都在）"
  else
    bad "nodes_online 不是 2"; tail -5 "$WORK/st.log"
  fi
else
  bad "jr_ctl status 失败（服务没建起来？）"; tail -10 "$WORK/st.log"
fi
# 逐关节寻址：两个关节名都得能寻到（这是“配置里真有 2 个关节”的直接证据）
if timeout 30 ros2 run jr_ros2 jr_ctl --node vbusrp --timeout-ms 5000 read --joints j1,j2 \
       --paths axis0.config.can.heartbeat_rate_ms > "$WORK/rd2.log" 2>&1; then
  if grep -q "^  j1 " "$WORK/rd2.log" && grep -q "^  j2 " "$WORK/rd2.log"; then
    ok "read --joints j1,j2 两个关节都读到（逐关节寻址正常）"
  else
    bad "read 输出里缺 j1/j2 行"; tail -6 "$WORK/rd2.log"
  fi
else
  bad "read --joints j1,j2 失败"; tail -6 "$WORK/rd2.log"
fi
if timeout 20 ros2 topic list 2>/dev/null | grep -q "/vbusrp/joint_states"; then
  ok "话题 /vbusrp/joint_states 存在（activate 之后才建）"
else
  bad "没有 /vbusrp/joint_states 话题"
fi
stop_launch "vbus_demo.launch.py"

echo "===== ①b jog:=true —— README 里承诺的“起来后自动点动”那条路 ====="
# ⚠ 文档里写了的开关就必须真跑一遍（否则“示例”只是承诺）。jog 会让关节真的动，
#   虚拟总线上安全；真机上这条路径别随手跑。
ros2 launch jr_bringup vbus_demo.launch.py jog:=true > "$WORK/jog.log" 2>&1 &
LAUNCH_PID=$!
if wait_ready vbusrp; then
  ok "「jog:=true」→ /vbusrp 服务可调（= 已 active）"
else
  bad "jog 例子里 ${JR_READY_BUDGET_S} s 内服务没建起来"; tail -20 "$WORK/jog.log"
fi
# 编排是「active 后 1 s enable → 再 3 s jog(0.5 s)」⇒ 给 40 s 等它把结果打出来。
for _ in $(seq 1 40); do grep -q "exit_reason=" "$WORK/jog.log" && break; sleep 1; done
if grep -q "exit_reason=" "$WORK/jog.log"; then
  if grep -q "exit_reason=failed" "$WORK/jog.log"; then
    bad "jog 报 failed"; grep -n "exit_reason=" "$WORK/jog.log" | tail -3
  else
    ok "使能 + 点动都成功（jog 打出了 exit_reason，且不是 failed）"
    grep -h "actual_duration=" "$WORK/jog.log" | tail -1 | sed 's/^/        /'
  fi
else
  bad "40 s 内没看到 jog 的结果"; tail -20 "$WORK/jog.log"
fi
# 失败时 launch 会把子进程的死讯打出来 —— 这是“静默失败”的兜底断言
if grep -q "process has died" "$WORK/jog.log"; then
  bad "jog 例子里有子进程非正常退出"; grep -n "process has died" "$WORK/jog.log" | head -3
else
  ok "jog 例子里没有子进程意外退出"
fi
stop_launch "vbus_demo.launch.py"

echo "===== ② 单关节 demo（joints:=1）====="
ros2 launch jr_bringup vbus_demo.launch.py joints:=1 > "$WORK/vb1.log" 2>&1 &
LAUNCH_PID=$!
if wait_ready vbusrp; then
  ok "「joints:=1」→ /vbusrp 服务可调（= 已 active，用 config/vbus_1joint.yaml）"
else
  bad "单关节 demo ${JR_READY_BUDGET_S} s 内服务没建起来"; tail -20 "$WORK/vb1.log"
fi
timeout 30 ros2 run jr_ros2 jr_ctl --node vbusrp --timeout-ms 5000 status > "$WORK/st1.log" 2>&1 \
  && grep -q "nodes_online=1" "$WORK/st1.log" \
  && ok "单关节配置生效（nodes_online=1）" \
  || { bad "单关节 demo 的 nodes_online 不是 1"; tail -5 "$WORK/st1.log"; }
timeout 30 ros2 run jr_ros2 jr_ctl --node vbusrp --timeout-ms 5000 read --paths axis0.config.can.heartbeat_rate_ms > "$WORK/rd.log" 2>&1 \
  && ok "单关节 demo 里 jr_ctl read 可调" || { bad "单关节 demo 里 read 失败"; tail -5 "$WORK/rd.log"; }
stop_launch "vbus_demo.launch.py"

echo "===== ③ 人形双总线示例：两个节点都要 active ====="
ros2 launch jr_bringup humanoid_2bus.launch.py > "$WORK/hum.log" 2>&1 &
LAUNCH_PID=$!
wait_ready leg_left  && ok "/leg_left 服务可调（= 已 active）" \
  || { bad "/leg_left ${JR_READY_BUDGET_S} s 内服务没建起来"; tail -25 "$WORK/hum.log"; }
wait_ready leg_right && ok "/leg_right 服务可调（= 已 active）" \
  || { bad "/leg_right ${JR_READY_BUDGET_S} s 内服务没建起来"; tail -25 "$WORK/hum.log"; }
timeout 30 ros2 run jr_ros2 jr_ctl --node leg_left --timeout-ms 5000 status > /dev/null 2>&1 \
  && ok "左侧总线的服务可调（jr_ctl status rc=0）" || bad "左侧 jr_ctl 失败"
# ⚠ F12 回归：**第二条总线**必须能读到**自己的**数据面证据。
#   真机上这条曾经永远失败：节点的 `bus_index_` 是**配置级**下标（leg_right = 1），
#   而快照 `buses[]` / `tg_->bus()` 是**tick 组内**下标（该进程组内只有 1 条 ⇒ 0）
#   ⇒ `1 >= bus_count(1)` ⇒ 永远 “no snapshot yet”；关节过滤 `j.bus_index != bus_index_`
#   也全不匹配 ⇒ 该节点的 `/joint_feedback`、`/joint_states`、诊断、动关节服务**全废**。
#   左侧恰好两个下标都是 0，所以旧冒烟（只查左侧服务）一直是绿的 —— 这两条断言就是补这个洞。
if timeout 30 ros2 run jr_ros2 jr_ctl --node leg_right --timeout-ms 5000 status > "$WORK/str.log" 2>&1 \
     && grep -q "nodes_online=2" "$WORK/str.log"; then
  ok "右侧总线读到**自己的**统计（nodes_online=2 ⇒ F12 不回归）"
else
  bad "右侧总线拿不到统计（F12 回归？）"; tail -5 "$WORK/str.log"
fi
if timeout 30 ros2 run jr_ros2 jr_ctl --node leg_right --timeout-ms 5000 \
     read --joints FR_hip,FR_knee --paths axis0.config.can.heartbeat_rate_ms > "$WORK/rdr.log" 2>&1 \
     && grep -q "^  FR_hip " "$WORK/rdr.log" && grep -q "^  FR_knee " "$WORK/rdr.log"; then
  ok "右侧关节能被寻到（FR_hip/FR_knee ⇒ 关节过滤也用对下标）"
else
  bad "右侧关节寻不到（F12 回归？）"; tail -6 "$WORK/rdr.log"
fi
if timeout 20 ros2 topic list 2>/dev/null | grep -q "robot_description"; then
  ok "robot_state_publisher 已发布 robot_description（示例 URDF 可用）"
else
  bad "没有 robot_description（URDF 示例没起作用）"
fi

echo "结果：$PASS 通过 / $FAIL 失败"
[ "$FAIL" = "0" ] || exit 1
exit 0
