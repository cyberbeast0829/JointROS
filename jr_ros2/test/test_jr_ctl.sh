#!/usr/bin/env bash
# ============================================================================
#  WP4 工具测试：`jr_ctl`（**走服务**的运维入口）
# ----------------------------------------------------------------------------
#  为什么必须真起一个节点：`jr_ctl` 的价值全在"**节点跑着的时候**能用"。
#  只测参数解析等于没测 —— 而且"节点没起来时要给出明确报错（rc=4）而不是挂住"
#  这条恰恰是现场最要紧的行为。
#
#  环境（ctest 传入）：
#    JR_BIN_DIR    = jr_bus / jr_ctl 所在目录（构建树或安装树均可）
#    JR_WS_SETUP   = 工作区 setup.bash（由 CMake 从 CMAKE_INSTALL_PREFIX 推得）
#  退出码：0 = 全部通过；1 = 有失败
# ============================================================================
set -u

: "${JR_BIN_DIR:?需要 JR_BIN_DIR=<jr_bus 与 jr_ctl 所在目录>}"
: "${JR_WS_SETUP:?需要 JR_WS_SETUP=<工作区 setup.bash>}"
BUS_BIN="${JR_BIN_DIR}/jr_bus"
CTL="${JR_BIN_DIR}/jr_ctl"
[ -x "$BUS_BIN" ] || { echo "FAIL: 找不到 $BUS_BIN"; exit 1; }
[ -x "$CTL" ] || { echo "FAIL: 找不到 $CTL"; exit 1; }
[ -f "$JR_WS_SETUP" ] || { echo "FAIL: 找不到 $JR_WS_SETUP"; exit 1; }

set +u
# shellcheck disable=SC1090
source "$JR_WS_SETUP"
set -u

WORK="$(mktemp -d)"
NODE_LOG="${WORK}/node.log"
BUS_NAME="vbusrp"
NODE_PID=""
# 收**整棵进程树**：`ros2 run` 只是 python 包装，真正的节点是它的子进程
# （不这么干会留下孤儿节点继续抱着总线锁 —— JTC demo 里真实踩过）。
kill_tree() {
  local pid="$1" kid
  for kid in $(pgrep -P "${pid}" 2>/dev/null || true); do kill_tree "${kid}"; done
  kill "${pid}" 2>/dev/null || true
}
cleanup() {
  pkill -f "jr_bus" 2>/dev/null || true
  if [ -n "${NODE_PID}" ]; then
    kill_tree "${NODE_PID}"
    wait "${NODE_PID}" 2>/dev/null || true
  fi
  rm -rf "$WORK"
}
trap cleanup EXIT

PASS=0
FAIL=0
ok()  { echo "  [ok]   $1"; PASS=$((PASS + 1)); }
bad() { echo "  [FAIL] $1"; FAIL=$((FAIL + 1)); }

# run <期望退出码> <描述> <子命令参数...>
# ⚠ 超时可覆盖（`CTL_TMO`）：动关节/存 Flash/标定这类命令在节点内部要**阻塞**（先安全失能
#   再复用），2 s 不够 —— 实测 `zero` 就因为 2 s 超时被报成"服务不可达"（把人引向错方向）。
run() {
  local want="$1" desc="$2"; shift 2
  local log="${WORK}/last.log"
  timeout 120 "$CTL" --node "$BUS_NAME" --timeout-ms "${CTL_TMO:-2000}" "$@" > "$log" 2>&1
  local rc=$?
  if [ "$rc" = "$want" ]; then
    ok "$desc（rc=$rc）"
  else
    bad "$desc（期望 rc=$want，实得 $rc）"
    echo "-------- 输出 --------"; cat "$log"; echo "----------------------"
  fi
  cat "$log"
}
# expect <描述> <子串> —— 在**上一次 run 的输出**里找
expect() {
  if grep -qF "$2" "${WORK}/last.log"; then ok "$1"; else
    bad "$1（输出里没有 '$2'）"; cat "${WORK}/last.log"
  fi
}

echo "===== ① 节点没起来时：必须明确报错且**不挂住**（rc=4）====="
timeout 30 "$CTL" --node "${BUS_NAME}_nope" --timeout-ms 1500 status > "${WORK}/last.log" 2>&1
rc=$?
if [ "$rc" = "4" ]; then ok "服务不可达时 rc=4"; else bad "服务不可达时应为 rc=4，实得 $rc"; fi
expect "报错里点名了服务与节点" "不可达"

echo "===== ② --help 不该依赖 DDS ====="
timeout 20 "$CTL" --help > "${WORK}/last.log" 2>&1
rc=$?
if [ "$rc" = "0" ]; then ok "--help rc=0"; else bad "--help 应 rc=0，实得 $rc"; fi
expect "用法里写清了退出码约定" "4 服务不可达"

echo "===== ③ 起一个真节点（虚拟总线）====="
cat > "${WORK}/bus.yaml" <<YAML
jr:
  # 参数写 / 存 Flash 的**总闸门**（ADR-8）。默认 false，这里显式打开，
  # 否则 `write` / `save` 会被正确拒绝（rc=1）—— 那是预期行为，不是缺陷。
  params:
    allow_write: true
    allow_flash_persist: true
  tick_groups:
    - {name: g0, rate_hz: 1000, buses: [${BUS_NAME}], cpu: -1, priority: -1}
  buses:
    - name: ${BUS_NAME}
      type: virtual
      spec: "0:id=1,gear=7.75,pmax=12.5,vmax=65,tmax=50,hb=5,timeout=100"
  joints:
    - {name: j1, bus: ${BUS_NAME}, node_id: 1}
YAML
"$BUS_BIN" --ros-args -r "__node:=${BUS_NAME}" \
           -p "config_file:=${WORK}/bus.yaml" -p "bus:=${BUS_NAME}" > "$NODE_LOG" 2>&1 &
NODE_PID=$!

# ⚠⚠ **必须自己走 lifecycle**：`jr_bus` 是 lifecycle 节点，而**服务与诊断是在 on_activate 建的**
#    —— 只 `ros2 run` 不配不动，客户看到的就是“进程活着、什么都没有”（实测就这么困惑了一轮）。
#    这也是文档（WP6）必须写的两条命令。
for _ in $(seq 1 50); do
  timeout 10 ros2 lifecycle set "/${BUS_NAME}" configure > "${WORK}/lifecycle.log" 2>&1 && break
  sleep 0.2
done
# ⚠ activate 的失败**不能吞**：服务是在 on_activate 建的 ⇒ activate 失败 = 后面全 rc=4，
#   而那时输出只会说"服务不可达"（把人引到错的方向）。这里先把它钉住。
ACTIVATE_LOG="${WORK}/activate.log"
if timeout 20 ros2 lifecycle set "/${BUS_NAME}" activate > "$ACTIVATE_LOG" 2>&1; then
  ok "activate 成功"
else
  bad "activate 失败（服务不会存在）"
  cat "$ACTIVATE_LOG"
  tail -20 "$NODE_LOG"
fi

# 等节点把服务挂上（**等**而不是睡固定时间）
READY=0
for _ in $(seq 1 100); do
  if timeout 5 "$CTL" --node "$BUS_NAME" --timeout-ms 400 status > /dev/null 2>&1; then
    READY=1; break
  fi
  sleep 0.2
done
if [ "$READY" = "1" ]; then
  ok "节点已 ready（configure+activate 后服务可调）"
else
  # ⚠ 显式报红：老写法只写 `ok`（不 ok 就什么都不打印）⇒ 跑满次数时**静默放过**。
  bad "节点 20 s 内没 ready（服务一直调不通）"
  tail -20 "$NODE_LOG"
fi
grep -q "configured" "$NODE_LOG" && ok "节点日志里有 configured 行" || {
  bad "节点没到 configured"; tail -20 "$NODE_LOG"; }

echo "===== ④ 读取类命令 ====="
CTL_TMO=15000   # 读写类命令：标定/回零/存 Flash 会阻塞（见上面的注释）
run 0 "status" status
expect "status 打印了链路与计数" "tx="
run 0 "info（设备身份）" info
expect "info 打印了 fw" "fw=0x"
run 0 "desc-info" desc-info
expect "desc-info 打印了端点数" "endpoints="
run 0 "ep-list（限量）" ep-list --max 5
expect "ep-list 说了匹配到多少条与返回多少条" "matched "
expect "ep-list 的每行都带端点 id" "id="
run 0 "ep-lookup（存在的端点）" ep-lookup axis0.motor.config.gear_ratio
expect "ep-lookup 说 found 信息" "id="
run 1 "ep-lookup（不存在的端点）应 rc=1" ep-lookup axis0.no.such.path
expect "且明确说不做模糊匹配" "no approximate matching"
expect "并给出可判别字段 found=false" "found=false"
run 0 "read（单个参数）" read --paths axis0.motor.config.gear_ratio
expect "read 打出了值" "axis0.motor.config.gear_ratio"
run 0 "hb-hint（只给建议）" hb-hint --rate-ms 5
expect "hb-hint 明确不改设备" "device_changed=0"
run 0 "desc-export 到文件" desc-export "${WORK}/desc.json"
[ -s "${WORK}/desc.json" ] && ok "描述符文件非空（$(wc -c < "${WORK}/desc.json") 字节）" || bad "描述符文件为空"
# 导出时打印的 hint（crc/fw）是**设备侧属性**，从 JSON 正文里推不出来 ⇒ 必须原样带回给导入。
HINT_CRC="$(grep -aoE 'crc=0x[0-9a-fA-F]+' "${WORK}/last.log" | head -1 | cut -d= -f2)"
HINT_FW="$(grep -aoE 'fw=0x[0-9a-fA-F]+' "${WORK}/last.log" | head -1 | cut -d= -f2)"
if [ -n "$HINT_CRC" ] && [ -n "$HINT_FW" ]; then
  ok "desc-export 打印了 hint（crc=$HINT_CRC fw=$HINT_FW）"
else
  bad "desc-export 没打印 crc/fw hint（导入必须带上它们）"; cat "${WORK}/last.log"
fi

# ⭐ 真正的**往返**（v0.15 修好）：导出 → 导入（带 hint）→ 服务重新 configure 成功。
#   ⚠ v0.14 这里曾经是**断言会失败**的，而且把根因记成了"导出格式 ≠ 导入要求"（误判）：
#   真正的原因是核心库把 SDK 的 hint 传成了 `nullptr`，而 SDK 的第一句就是
#   `!hint ⇒ JSDK_ERR_INVALID_ARG`（CRC/fw 从 JSON 正文推不出来）—— 见 §13.3-38。
run 0 "desc-import（导出→导入往返，带 hint）" desc-import "${WORK}/desc.json" --crc "$HINT_CRC" --fw "$HINT_FW" --confirm
expect "导入后已重新 configure 生效" "device_reconfigured=1"
expect "且回显了版本 CRC 与数据 CRC32" "data_crc32=0x"

# 负例（证伪）：同一份文件**截断**成非法 JSON 必须失败 —— 不猜、不静默成功。
head -c 200 "${WORK}/desc.json" > "${WORK}/broken.json"
run 1 "desc-import（截断的 JSON 必须失败）" desc-import "${WORK}/broken.json" --crc "$HINT_CRC" --fw "$HINT_FW" --confirm
# ⭐⭐ 证伪：**失败的导入不允许把已有描述符废掉**（v0.15 护栏）。
#   SDK 的 `import_raw()` 先清空 store 再解析 ⇒ 失败会**摧毁**内存里的描述符，
#   而且 `configure()` 不会再下载（SDK 认为它已存在）⇒ 实测“一次手误传错文件，之后
#   read/write 的文本路径全废”。护栏：失败时用我们手里的那份 JSON + crc/fw **回滚**。
run 0 "失败导入之后：read 仍可用（描述符已回滚）" read --paths axis0.config.can.heartbeat_rate_ms
run 0 "失败导入之后：write 的文本路径仍可用" write --path axis0.config.can.heartbeat_rate_ms --value 5 --confirm
expect "且 write 报告经读回校验" "verified="
# 负例（证伪）：`--persist`（写设备 Flash）本 SDK 版本没有这个能力 ⇒ 必须**明确拒绝**，
#   不能静默忽略（否则客户以为"已经预烧进设备了"）。
run 1 "desc-import --persist 必须被明确拒绝" desc-import "${WORK}/desc.json" --crc "$HINT_CRC" --fw "$HINT_FW" --persist --confirm
expect "且说明了原因" "not supported"

echo "===== ⑤ 闸门与安全动作 ====="
run 2 "write 不带 --confirm 必须被拒（rc=2）" write --path axis0.motor.config.gear_ratio --value 7.75
expect "拒绝理由指名 §8.5 / --confirm" "confirm"
run 2 "jog 不带 --confirm 必须被拒" jog --joint j1 --pos 0.01 --duration-s 0.2
run 2 "jog 时长 > 10 s 必须被拒" jog --joint j1 --pos 0.01 --duration-s 20 --confirm
run 2 "save 不带 --confirm 必须被拒" save
run 2 "reset 不带 --confirm 必须被拒" reset
run 2 "node-id 的 new_id=0 必须被拒" node-id j1 0 --confirm

echo "===== ⑥ 动关节/设备的命令（虚拟总线，可放心跑）====="
run 0 "enable（使能全部）" enable
expect "enable 有逐关节结果" "j1"
run 0 "zero（置零，不落 Flash）" zero
run 0 "fault-reset（此刻无故障，必须 rc=0）" fault-reset --confirm
run 0 "save（存 Flash，虚拟设备）" save --confirm
# ⚠ `--joint` 缺省：总线只有一个关节时由服务解析（多关节必须点名，服务会拒绝）。
run 0 "write（写回原值，幂等）" write --path axis0.config.can.heartbeat_rate_ms --value 5 --confirm
expect "write 打印了 requested/value/verified" "verified="

# ⚠⚠ 顺序不是随便定的（前两轮才理清）：**会留下锁存故障的命令放最后**。
#   `jog`（点动）在这台设备上结束时会锁存 estop，而按 fw 1545 的实测行为，estop 锁存
#   `CLEAR_ERRORS`(fault-reset) **清不掉**，只能 ResetDevice（`reset` 软复位）或断电。
#   所以：zero/save/write 这些要"设备无故障且使能"的命令都排在 jog 之前。
#
# ⚠ F10（真机发现）：**不给增益的点动 = 零力矩 MIT 目标** —— 电机根本不会动，可是
#   "使能 → 保持 → 失能"整套流程照样跑完并返回成功。这条用例原先写的正是
#   `jog ... --confirm`（不带 `--kp/--kd`）并断言 rc=0 ⇒ **它把这个缺陷写成了"期望行为"**
#   （红灯变绿灯的假象：命令成功了，但关节没动）。现在两个方向都要断言：
#   全零必须**在碰设备之前**被拒；带增益必须真的成功。
run 1 "jog 全零增益必须被拒（F10：零力矩目标不会动，但流程照样'成功'）" \
    jog --joint j1 --pos 0.02 --duration-s 0.2 --confirm
expect "拒绝理由说清是零力矩" "zero-torque"
run 0 "jog（0.2 s 点动，显式给增益）" \
    jog --joint j1 --pos 0.02 --kp 2 --kd 0.2 --duration-s 0.2 --confirm
expect "jog 给出了退出原因" "exit_reason="

# jog 结束后走的是"hold → 等 2 周期 → 失能"，**不留**锁存故障 ⇒ fault-reset 应为 rc=0。
# （⚠ 我一开始猜"jog 会锁存 estop"，实测 rc=0 —— 断言按**观察到的事实**写，
#   真正的锁存源是下面的 calib：虚拟设备在状态 3 里报 CAN_BUS_FAILED，之后就清不掉了。）
run 0 "jog 后 fault-reset（jog 不留锁存故障）" fault-reset --confirm
expect "并确认当前无故障" "no fault"

# ⚠ calib / home 在**虚拟设备**上无法成功（设备在状态 3/11 里报 CAN_BUS_FAILED，
#   即仿真不实现标定/回零的状态机）。这里如实断言"会失败并且说清原因"，
#   而不是把断言放宽到看不出异常。真机上的成功路径见 §13.4 的待办。
run 1 "calib（虚拟设备不支持标定，应如实失败）" calib
expect "且给出失败原因" "failed"
run 1 "home（同上）" home
expect "且给出失败原因" "failed"

run 0 "disable（安全失能）" disable
run 0 "reset（软复位，之后节点需重新 configure）" reset --confirm

# node-id 放在**最后**：改完寻址就变了，后面的命令不一定还能用（这也是它该单独跑的原因）。
run 0 "node-id（改成 2，不落 Flash）" node-id j1 2 --confirm

echo "===== ⑦ WP4 端到端：jr_gen_config 生成的配置能被**节点**直接加载 ====="
# §11 的 WP4 验收口径里就有这一句（“生成的 YAML 能被节点直接加载”）。
# 之前只证到“同一个加载器自检 + 回喂 jr_hw_verify” —— 那是**加载器**级别，不是**节点**级别。
# 这里补最后一段：生成 → 起 jr_bus → configure + activate → 关节集合与生成文件一致。
GEN="${JR_BIN_DIR}/jr_gen_config"
if [ ! -x "$GEN" ]; then
  bad "找不到 jr_gen_config（$GEN）：WP4 这条验收口径没被验证"
else
  # 两台设备（node 3 的量程故意非默认）；**换个总线名**避免与上面的 vbusrp 争单 master 锁。
  GENBUS="vbgen"
  SPEC_GEN="0:id=1,gear=7.75,pmax=12.5,vmax=65,tmax=50,hb=5,timeout=100;1:id=3,gear=16.5,pmax=6.25,vmax=32,tmax=25,hb=5"
  timeout 60 "$GEN" --if virtual --channel "$SPEC_GEN" --name "$GENBUS" --probe 8 \
           --lock-dir "$WORK" --out "${WORK}/gen.yaml" > "${WORK}/gen.log" 2>&1
  rc=$?
  if [ "$rc" = "0" ]; then ok "生成配置（扫描两台设备）rc=0"; else bad "生成配置应 rc=0，实得 $rc"; cat "${WORK}/gen.log"; fi
  # 设备不上报名字 ⇒ 生成器按发现顺序编 j<node_id>；node 3 只存在于生成文件里。
  if grep -q "name: j3, bus: $GENBUS, node_id: 3" "${WORK}/gen.yaml"; then
    ok "生成文件按发现顺序编出关节 j1/j3"
  else
    bad "生成文件里没有 node 3 的关节"; cat "${WORK}/gen.yaml"
  fi

  GEN_NODE_LOG="${WORK}/gen_node.log"
  "$BUS_BIN" --ros-args -r "__node:=${GENBUS}" \
             -p "config_file:=${WORK}/gen.yaml" -p "bus:=${GENBUS}" > "$GEN_NODE_LOG" 2>&1 &
  GEN_PID=$!
  for _ in $(seq 1 50); do
    timeout 10 ros2 lifecycle set "/${GENBUS}" configure > "${WORK}/gen_lifecycle.log" 2>&1 && break
    sleep 0.2
  done
  if timeout 20 ros2 lifecycle set "/${GENBUS}" activate > "${WORK}/gen_activate.log" 2>&1; then
    ok "★ 生成的配置能被节点 configure + activate（§11 的验收口径）"
  else
    bad "★ 生成的配置 activate 失败"
    cat "${WORK}/gen_activate.log"; tail -20 "$GEN_NODE_LOG"
  fi
  for _ in $(seq 1 100); do
    if timeout 5 "$CTL" --node "$GENBUS" --timeout-ms 400 info --joint j3 > "${WORK}/last.log" 2>&1; then
      INFO_OK=1; break
    fi
    sleep 0.2
  done
  # ⚠ 不能写成"循环里 `&& break`、循环后看 `$?`"：跑满次数时 `$?` 取到的是最后一次
  #   `sleep` 的 0 ⇒ 断言**永远不会红**（这种"没牙齿的等待"比没有用例更危险）。
  if [ "${INFO_OK:-0}" = "1" ]; then
    ok "生成的关节 j3 被节点识别（info --joint j3 rc=0）"
  else
    bad "info --joint j3 一直失败（生成的关节没被节点认出来）"; tail -5 "${WORK}/last.log"
  fi
  expect "info 打出了设备信息" "fw="
  kill_tree "$GEN_PID" 2>/dev/null || true
  wait "$GEN_PID" 2>/dev/null || true
fi

echo "===== ⑧ 关掉节点后：必须回到 rc=4（不留半死状态）====="
pkill -f "jr_bus" 2>/dev/null || true
if [ -n "${NODE_PID}" ]; then kill_tree "${NODE_PID}"; wait "${NODE_PID}" 2>/dev/null || true; fi
NODE_PID=""
for _ in $(seq 1 50); do
  timeout 5 "$CTL" --node "$BUS_NAME" --timeout-ms 300 status > /dev/null 2>&1
  [ $? = "4" ] && break
  sleep 0.2
done
timeout 20 "$CTL" --node "$BUS_NAME" --timeout-ms 800 status > "${WORK}/last.log" 2>&1
rc=$?
if [ "$rc" = "4" ]; then ok "节点停掉后 rc=4"; else bad "节点停掉后应 rc=4，实得 $rc"; fi

echo "结果：$PASS 通过 / $FAIL 失败"
[ "$FAIL" = "0" ] || exit 1
exit 0
