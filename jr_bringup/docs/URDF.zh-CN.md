# URDF 片段：怎么写、怎么写错（WP7）

`jr_bus` 节点**不需要 URDF**（配置就够了）。URDF 只在你要用
`ros2_control` / rviz2 / MoveIt 时才出现，而它最容易出错的地方是**与配置的一致性**。

本页给的是"照抄就能用"的片段与三条硬约束。可运行的例子见
`jr_bringup/urdf/humanoid_2bus.urdf`（配合 `humanoid_2bus.launch.py`）。

---

## 1. 谁对谁负责

| 东西 | 权威来源 | 说明 |
|---|---|---|
| **关节名** | 配置 `joints[].name`（或 `jr_gen_config` 生成的配置） | URDF 里的 `<joint name>` 必须**逐字相同**；`ros2_control` 靠它把接口对到硬件 |
| **关节位置/速度/力矩限位** | 设备读回值（`jr_gen_config` / `jr_hw_verify` 的量程） | **不要手抄**。真机量程是设备里边的值，抄错就是"RViz 里能动、真机上被拒" |
| **命令接口集** | `joints[].mode`（MIT / CSP / CSV / CST） | 见 §3，**这条最容易出错** |
| **父子关系 / 几何** | 你的机器人（出厂 CAD） | 这是唯一"本仓库不知道"的部分 |

## 2. 最小片段（MIT 关节，走 `ros2_control`）

`jr_bus` 节点用户**不需要**这段；用 `ros2_control`（`jr_ros2_control` 组件）时才需要：

```xml
<ros2_control name="jr" type="system">
  <hardware>
    <plugin>jr_ros2_control/JrSystemInterface</plugin>
    <!-- 与节点/工具**同一份** YAML（§6.4）——不要各写一份 -->
    <param name="config_file">/etc/jr/robot.yaml</param>
    <param name="tick_source">internal</param>   <!-- internal | controller_manager -->
    <param name="gain_mode">wire</param>         <!-- wire(kp/kd) | si(stiffness/damping) -->
  </hardware>

  <!-- ⚠ name 必须与 YAML 里的 joints[].name 逐字一致（不一致会在 on_init 直接失败并列出可用名字） -->
  <joint name="FL_hip">
    <!-- MIT：三件套 + 一组增益（gain_mode=wire 导出 kp/kd；si 导出 stiffness/damping） -->
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

## 3. ⚠ 三条硬约束（都是实测踩出来的）

1. **`<command_interface>` 必须与组件实际导出的集合逐一对上**。
   组件是**按 `joints[].mode` 导出接口**的：

   | 模式 | 导出的命令接口 |
   |---|---|
   | `mit` | `position` + `velocity` + `effort` + **一组**增益（`kp`,`kd` 或 `stiffness`,`damping`） |
   | `csp` | `position` |
   | `csv` | `velocity` |
   | `cst` | `effort` |
   | `current` | **拒绝**（`effort` 的单位会从 N·m 变成电机端 A） |

   多写/少写一个，controller_manager 会直接拒绝初始化硬件：

   > `Discrepancy between robot description file (urdf) and actually exported HW interfaces`

   而且后面还可能连带 `pal_statistics` 段错误 —— 别指望"先跑起来再说"。

2. **同一个控制器不能混模式**。JTC 默认 claim `position + velocity`，纯 CSP 配置下会失败 ⇒
   `controllers.yaml` 的 `command_interfaces` 也要跟着模式改。

3. **Jazzy 起，`controller_manager` 从 `/robot_description` 话题拿 URDF**，不是从参数里读。
   所以要么用 `robot_state_publisher` 发一份（本例就是这样），要么用 `xacro`/`robot_description` 参数
   的标准组合 —— 缺了它 CM 会一直等：`Waiting for data on 'robot_description' topic`。

## 4. 生成思路（别手抄限位）

```bash
# ① 扫总线，生成配置（量程来自设备读回，不落盘前会自检）
jr_gen_config --if socketcan --channel can0 --name leg_left --probe 16 --out leg_left.yaml

# ② 上机前体检（含量程/标定/心跳的逐层结论）
jr_hw_verify --config leg_left.yaml

# ③ 把 ① 里的 limits 抄进 URDF 的 <limit>（或用脚本从 YAML 生成 URDF —— 见下）
```

从配置生成 URDF 片段（把 YAML 里的 `limits:` 灌进模板）：

```bash
python3 - <<'PY'
import yaml, sys
cfg = yaml.safe_load(open('leg_left.yaml'))
for j in cfg['jr']['joints']:
    lim = cfg['jr'].get('limits', {}).get(j['name'], {})
    pos = lim.get('position') or [0.0, 0.0]
    print(f'  <joint name="{j["name"]}_joint" type="revolute">')
    print(f'    <axis xyz="0 1 0"/>')
    print(f'    <limit lower="{pos[0]}" upper="{pos[1]}" '
          f'effort="{lim.get("effort", 0.0)}" velocity="{lim.get("velocity", 0.0)}"/>')
    print('  </joint>')
PY
```

⚠ `limits:` 里的 `stiffness` / `damping` **故意不写**：那是控制器增益（设备里没有），
由你的控制器配置决定，不由设备读回。

## 5. 常见症状对照表

| 症状 | 多半是 |
|---|---|
| `Discrepancy between robot description file (urdf) and actually exported HW interfaces` | URDF 的命令接口集与 `mode` 不符（§3.1） |
| `on_init` 阶段报找不到关节 + 列出可用名字 | URDF 的 `<joint name>` 与配置的 `joints[].name` 不一致 |
| CM 一直等 `robot_description` | 没发 URDF 话题（§3.3） |
| RViz 能动、真机上目标被拒 | URDF 的 `<limit>` 与设备量程不一致（手抄过时了） |
