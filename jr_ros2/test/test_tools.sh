#!/usr/bin/env bash
# ============================================================================
#  WP4 工具测试：`jr_hw_verify` / `jr_gen_config`
# ----------------------------------------------------------------------------
#  为什么是"真跑 CLI"而不是单元测试：
#   这两个工具的价值全在**端到端的那些字**上（"量程来自设备"、"缺节点要点名"、
#   "生成的配置真能被加载"）。单元测试测的是函数，测不到"客户照抄的那条命令"。
#
#  为什么能进 CI：工具是**非 ROS** 的，虚拟总线又不需要硬件 —— 整条链在容器里
#  （和 Windows 开发机上）都能跑。
#
#  环境：由 ctest 传入
#    JR_TOOLS_DIR  = 装工具可执行文件的目录（add_test 里用 $<TARGET_FILE_DIR:...>）
#  退出码：0 = 全部通过；1 = 有失败（逐条打印）
#
#  ⚠ 这个用例**跑得飞快**（0.3 s 量级）是正常的，别当成"没真跑"：
#    虚拟后端开了 autotick，所以"被动听 200 ms 心跳"与"描述符 5 s 超时"都在
#    **虚拟时间**里走完。判断它有没有真跑，看断言的**具体字符串**
#    （例如 `gear=7.7500` 只可能来自设备读回，空跑命中不了）。
# ============================================================================
set -u

: "${JR_TOOLS_DIR:?需要 JR_TOOLS_DIR=<工具所在目录>}"
VERIFY="${JR_TOOLS_DIR}/jr_hw_verify"
GEN="${JR_TOOLS_DIR}/jr_gen_config"
[ -x "$VERIFY" ] || { echo "FAIL: 找不到 $VERIFY"; exit 1; }
[ -x "$GEN" ] || { echo "FAIL: 找不到 $GEN"; exit 1; }

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
PASS=0
FAIL=0

ok()   { echo "  [ok]   $1"; PASS=$((PASS + 1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL + 1)); }

# check_rc <期望码> <实际码> <描述>
check_rc() { if [ "$2" = "$1" ]; then ok "$3（rc=$2）"; else bad "$3（期望 rc=$1，实得 $2）"; fi; }
# check_has <文件> <子串> <描述>
check_has() {
  if grep -qF "$2" "$1"; then ok "$3"; else
    bad "$3（没在输出里找到 '$2'）"; echo "-------- 实际输出 --------"; cat "$1"; echo "--------------------------"
  fi
}

# 虚拟总线：两台设备，其中 node 3 的量程**故意选非默认值**（6.25/32/25），
# 这样"量程来自设备"这句话才有牙齿：硬编码默认值(12.5/65/50)会被抓出来。
SPEC_GOOD="0:id=1,gear=7.75,pmax=12.5,vmax=65,tmax=50,hb=5,timeout=100;1:id=3,gear=16.5,pmax=6.25,vmax=32,tmax=25,hb=5"

cat > "$WORK/one.yaml" <<YAML
jr:
  tick_groups:
    - {name: g0, rate_hz: 1000, buses: [virt], cpu: -1, priority: -1}
  buses:
    - name: virt
      type: virtual
      spec: "0:id=1,gear=7.75,pmax=12.5,vmax=65,tmax=50,hb=5,timeout=100"
  joints:
    - {name: j1, bus: virt, node_id: 1}
YAML

# ---- ① 体检一条正常总线：必须全绿，且量程必须是**设备读回**的值 ----------
echo "[1] jr_hw_verify：正常配置应当 rc=0"
"$VERIFY" --config "$WORK/one.yaml" --lock-dir "$WORK" > "$WORK/v1.log" 2>&1
check_rc 0 $? "正常配置 rc=0"
check_has "$WORK/v1.log" "gear=7.7500" "量程来自设备（gear=7.75，不是默认 16.0）"
check_has "$WORK/v1.log" "全部 1 个关节已标定" "标定状态有结论"

# ---- ② 配置里声明了一台**不在线上**的设备：必须 rc=1 且点名 ----------------
echo "[2] jr_hw_verify：配置声明的节点不在线上 → 必须失败并点名"
sed 's/node_id: 1/node_id: 5/' "$WORK/one.yaml" > "$WORK/missing.yaml"
"$VERIFY" --config "$WORK/missing.yaml" --lock-dir "$WORK" > "$WORK/v2.log" 2>&1
check_rc 1 $? "缺节点时 rc=1"
check_has "$WORK/v2.log" "没有应答" "点名了缺哪个节点"

# ---- ③ 生成配置：rc=0，且量程是设备读回值 ------------------------------echo "[3] jr_gen_config：扫描两台设备 → 生成配置"
"$GEN" --if virtual --channel "$SPEC_GOOD" --name virt --probe 8 \
       --lock-dir "$WORK" --out "$WORK/gen.yaml" > "$WORK/g1.log" 2>&1
check_rc 0 $? "生成 rc=0"
[ -s "$WORK/gen.yaml" ] && ok "配置文件已写出" || bad "配置文件没写出"
check_has "$WORK/gen.yaml" "[-6.2500, 6.2500]" "node 3 的量程来自设备（非默认值）"
check_has "$WORK/gen.yaml" "node_id: 3" "node 3 出现在关节表里"

# ---- ④ 已存在时不覆盖（除非 --force）-----------------------------------
echo "[4] jr_gen_config：不覆盖已存在的文件"
"$GEN" --if virtual --channel "$SPEC_GOOD" --name virt --probe 8 \
       --lock-dir "$WORK" --out "$WORK/gen.yaml" > "$WORK/g2.log" 2>&1
check_rc 1 $? "已存在时 rc=1（需要 --force）"

# ---- ⑤ 生成的配置能被加载并体检通过 ------------------------------------
echo "[5] 生成的配置能直接喂给 jr_hw_verify"
"$VERIFY" --config "$WORK/gen.yaml" --lock-dir "$WORK" --allow-uncalibrated > "$WORK/v3.log" 2>&1
check_rc 0 $? "生成的配置可加载且体检通过"

# ---- ⑥ 变异：把生成的 node_id 改成 0 → 加载器必须拒绝 -------------------
echo "[6] 变异：node_id=0 必须被拒绝"
sed 's/node_id: 1/node_id: 0/' "$WORK/gen.yaml" > "$WORK/mut.yaml"
"$VERIFY" --config "$WORK/mut.yaml" --lock-dir "$WORK" > "$WORK/v4.log" 2>&1
check_rc 1 $? "node_id=0 被拒"
check_has "$WORK/v4.log" "must be 1..254" "报的是加载器的原话"

# ---- ⑦ 变异：删掉 tick_groups → 加载器必须拒绝（这条真踩过）------------
echo "[7] 变异：缺 tick_groups 必须被拒绝"
grep -v "tick_groups\|name: g0" "$WORK/gen.yaml" > "$WORK/mut2.yaml"
"$VERIFY" --config "$WORK/mut2.yaml" --lock-dir "$WORK" > "$WORK/v5.log" 2>&1
check_rc 1 $? "缺 tick_groups 被拒"

# ---- ⑧ jr_bus_plan：可行配置 rc=0；不可行配置 rc=1 且给出可操作建议 -----
echo "[8] jr_bus_plan：先算再上机"
PLAN="${JR_TOOLS_DIR}/jr_bus_plan"
if [ -x "$PLAN" ]; then
  "$PLAN" --config "$WORK/one.yaml" > "$WORK/p1.log" 2>&1
  check_rc 0 $? "可行配置 rc=0"
  check_has "$WORK/p1.log" "0 条不可行" "汇总里明确说了几条不可行"

  # 8 个关节：node_id 8 > 7 会被模型判为不可行（广播位图只到 7 + 帧数翻倍）
  python3 - "$WORK/one.yaml" "$WORK/many.yaml" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
text = open(src).read()
joints = "".join("    - {name: j%d, bus: virt, node_id: %d}\n" % (i, i) for i in range(1, 9))
text = text.replace("    - {name: j1, bus: virt, node_id: 1}\n", joints)
open(dst, "w").write(text)
PY
  "$PLAN" --config "$WORK/many.yaml" > "$WORK/p2.log" 2>&1
  check_rc 1 $? "8 关节（node_id>7）时 rc=1"
  check_has "$WORK/p2.log" "不可行" "明确说了不可行"
  check_has "$WORK/p2.log" "node_id=8" "点名了是哪个 node_id 越界"
else
  bad "找不到 jr_bus_plan（本用例需要它）"
fi

echo "结果：$PASS 通过 / $FAIL 失败"
[ "$FAIL" = "0" ] || exit 1
exit 0
