#!/usr/bin/env bash
#
# @file    test_double_start.sh
# @brief   §10.2 ⑦ 同一条总线被两个进程抢：第二个必须**明确失败**并指出持有者。
#
# @par 为什么必须是一条独立的（跨进程）用例
#   两个 master 挂在同一条 CAN 上时，设备会记住**最后**一个 master id。现场症状是
#   "心跳交错、参数写偶尔失败、有时关节不动" —— 从日志里根本看不出是**双开**导致的。
#   所以必须：① 第二个进程**起不来**（不是"起来了但行为怪"）；② 说清谁占着锁；
#   ③ 进程退出后锁**会被释放**（否则一次崩溃就等于总线需要人工清锁）。
#
# @par 时序
#   全程不靠 sleep 猜时长：用"锁文件 / owner 文件 / 日志出现某行"做有界轮询同步点。
#
# 用法：test_double_start.sh <jr_lock_probe 可执行文件>
# 退出码：0 = 全部通过；1 = 有条目失败（明细打在 stdout）

set -u

PROBE="${1:?usage: test_double_start.sh <jr_lock_probe>}"
WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT

CFG="${WORK}/jr.yaml"
CFG_SHARED="${WORK}/jr_shared.yaml"
LOCKDIR="${WORK}/locks"
mkdir -p "${LOCKDIR}"

checks=0
failures=0
check() {  # check <0|1> <message>
    checks=$((checks + 1))
    if [ "$1" = 0 ]; then return; fi
    failures=$((failures + 1))
    printf 'FAIL: %s\n' "$2"
}

write_cfg() {  # $1 = 文件，$2 = allow_shared
    cat > "$1" <<YAML
jr:
  rt: {enabled: false}
  tick_groups:
    - {name: g0, rate_hz: 1000, buses: [vbus]}
  buses:
    - name: vbus
      type: virtual
      spec: "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd;1:id=2,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd"
      is_fd: true
      master_id: 1
      joints: [j0, j1]
  joints:
    - {name: j0, bus: vbus, node_id: 1}
    - {name: j1, bus: vbus, node_id: 2}
  limits:
    j0: {position: [-1.0, 1.0], stiffness: 30.0, damping: 1.0}
    j1: {position: [-1.0, 1.0], stiffness: 30.0, damping: 1.0}
  feedback: {source: broadcast_plus_heartbeat, heartbeat_ms: 5, publish_hz: 100, joint_state_hz: 100}
  command: {timeout_ms: 100, timeout_action: hold}
  safety: {auto_enable: false, require_calibrated: true, on_exit_action: disable}
  params: {allow_write: false, allow_flash_persist: false}
  descriptor: {cache_enabled: false}
  bus_lock: {enabled: true, lock_dir: "${LOCKDIR}", allow_shared: $2}
YAML
}

wait_for() {  # wait_for <bounded_ms> <command...>：轮询到命令成功
    local left="$1"
    shift
    while [ "${left}" -gt 0 ]; do
        if "$@" >/dev/null 2>&1; then return 0; fi
        sleep 0.05
        left=$((left - 50))
    done
    "$@" >/dev/null 2>&1
}

write_cfg "${CFG}" false
write_cfg "${CFG_SHARED}" true

# 锁文件名 = `jr-<锁键>.lock`，锁键由 `lock_key()` 决定（F7）：真后端按**物理通道**
# （`slcan:/dev/ttyACM0`），`virtual` 按**总线名**（每条 virtual 总线各有自己的仿真设备，
# 同 spec 也互不干扰 —— 所以不能按通道互斥，否则人形示例/CI 全卡死）。
# 这里是 `type: virtual` + `name: vbus` ⇒ 键是 `virtual:vbus`，`BusLock` 把 `:` 清洗成 `_`。
# ⚠ 故意把名字**写死**：这条用例就是锁键契约的守卫，键规则一改这里必须跟着改（可见、可审）。
LOCK_FILE="${LOCKDIR}/jr-virtual_vbus.lock"
LOCK_OWNER="${LOCKDIR}/jr-virtual_vbus.owner"
RELEASE="${WORK}/release"

# ---- ① 第一个进程：configure 成功并**一直持锁** --------------------------------
"${PROBE}" "${CFG}" jr_lock_a "${RELEASE}" > "${WORK}/a.log" 2>&1 &
A_PID=$!

wait_for 10000 test -f "${LOCK_FILE}"
wait_for 1000 test -f "${LOCK_OWNER}"
check "$([ -f "${LOCK_FILE}" ] && [ -f "${LOCK_OWNER}" ] && echo 0 || echo 1)" \
    "第一个进程没有拿到锁（${LOCK_FILE} / ${LOCK_OWNER} 一直没出现）"
if [ ! -f "${LOCK_FILE}" ]; then
    printf -- '--- a.log ---\n'
    cat "${WORK}/a.log"
    printf 'double_start: %d check(s), %d failure(s)\n' "${checks}" "${failures}"
    exit 1
fi
printf '  [lock] owner file says: %s\n' "$(cat "${LOCK_OWNER}" 2>/dev/null)"

# ---- ② 第二个进程：必须失败（rc=3），且日志点名持有者 ---------------------------
"${PROBE}" "${CFG}" jr_lock_b > "${WORK}/b.log" 2>&1
B_RC=$?
check "$([ "${B_RC}" = 3 ] && echo 0 || echo 1)" \
    "第二个进程必须 configure 失败（rc=3），实际 rc=${B_RC}"
grep -q "is held by" "${WORK}/b.log"
check $? "第二个进程的日志必须说清锁被谁占着（'is held by'）"
grep -q "pid ${A_PID}" "${WORK}/b.log"
check $? "日志里必须给出**持有者 PID**（${A_PID}）—— 否则现场只能靠猜"
grep -q "allow_shared=true" "${WORK}/b.log"
check $? "必须给出逃生舱（lock.allow_shared=true）并说清风险"

# ---- ③ 持有者退出 → 锁必须自动释放（否则崩溃一次就要人工清锁）----------------
touch "${RELEASE}"
wait "${A_PID}"
A_RC=$?
check "$([ "${A_RC}" = 0 ] && echo 0 || echo 1)" "第一个进程应正常退出（rc=0），实际 rc=${A_RC}"

"${PROBE}" "${CFG}" jr_lock_c > "${WORK}/c.log" 2>&1
C_RC=$?
check "$([ "${C_RC}" = 0 ] && echo 0 || echo 1)" \
    "持有者退出后新进程必须能拿到锁（rc=0），实际 rc=${C_RC}"

# ---- ④ 逃生舱不是谎话：allow_shared=true 时两个进程可共存 ----------------------
SHARED_RELEASE="${WORK}/release_shared"
"${PROBE}" "${CFG_SHARED}" jr_shared_a "${SHARED_RELEASE}" > "${WORK}/sa.log" 2>&1 &
SA_PID=$!
wait_for 10000 grep -q "holding the bus lock" "${WORK}/sa.log"
"${PROBE}" "${CFG_SHARED}" jr_shared_b > "${WORK}/sb.log" 2>&1
SB_RC=$?
check "$([ "${SB_RC}" = 0 ] && echo 0 || echo 1)" \
    "allow_shared=true 时第二个进程应该能起来（rc=0），实际 rc=${SB_RC}"
if [ "${SB_RC}" != 0 ]; then
    printf -- '--- shared b.log ---\n'
    cat "${WORK}/sb.log"
fi
touch "${SHARED_RELEASE}"
wait "${SA_PID}" 2>/dev/null

# ---- 汇总 ---------------------------------------------------------------------
printf 'double_start: %d check(s), %d failure(s)\n' "${checks}" "${failures}"
[ "${failures}" = 0 ] || exit 1
printf 'PASS: the second master was refused (holder named) and the lock was released on exit\n'
