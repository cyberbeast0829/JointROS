# JointROS —— 把 CyberBeast 关节模组接进 ROS 2

**一句话**：关节模组（电机 + 驱动器 + 编码器一体）通过 CAN 接到电脑后，JointROS 把它变成一个
**标准的 ROS 2 机器人部件** —— 有 `/joint_states`、能被 `joint_trajectory_controller`（以及 MoveIt 之类）
驱动、有诊断，还有一套"上机前先体检"的工具。

|  |  |
|---|---|
| ✅ **这一层负责** | 实时控制循环与调度（可选 PREEMPT_RT）、ROS 2 接口（话题 / 服务 / 诊断）、安全策略（冷启动不使能、退出走安全失能、急停）、单主站纪律与总线预算检查、设备描述符缓存、上机工具链 |
| ❌ **这一层不负责** | CAN 帧格式、协议编解码、参数（端点）系统、设备状态机 —— 这些由 **JointSDK** 负责。本仓库是它之上的 ROS 2 集成层 |

**我该看哪一节？**

| 你的情况 | 去哪 |
|---|---|
| 还没硬件，想先看看它能干什么 | [§2 五分钟上手](#2-五分钟上手不需要硬件) |
| 有模组和 USB-CAN 盒，想让它转起来 | [§3 第一次接真机](#3-第一次接真机照这个顺序做) |
| 想改配置（限位 / 模式 / 发布率 / 实时） | [§4 配置文件怎么读](#4-配置文件怎么读) |
| 想知道"它现在正常吗 / 为什么不正常" | [§5 生命周期与运行状态](#5-生命周期与运行状态)、[§9 排障](#9-现场纪律与排障) |
| 要把关节接进自己的机器人（JTC / MoveIt / 自研控制器） | [§8 集成到你的系统](#8-集成到你的系统三条路线) |
| 想查某个命令怎么用 | [§6 常用任务速查](#6-常用任务速查照抄这一节就行) |
| 想了解架构与验收证据 | [`docs/DESIGN.zh-CN.md`](docs/DESIGN.zh-CN.md)（**唯一权威**） |

<details>
<summary><b>先认几个名词（点开，30 秒）</b></summary>

| 名词 | 一句话 |
|---|---|
| **模组 / 关节** | 一个"电机 + 驱动器 + 编码器"的整体，总线上有自己的 `node_id`（1..254，**不能是 0**） |
| **适配器 / 主机** | 电脑侧那块 USB-CAN 盒子。它和你自己写的程序合起来叫"主站（master）" |
| **SocketCAN / `can0`** | Linux 原生 CAN 接口（生产推荐）。适配器配成这个模式后，系统里会出现 `can0` |
| **slcan / `/dev/ttyACM0`** | 适配器的"串口模拟 CAN"模式（调试方便，但带宽低，**别用来跑高频控制**） |
| **Classic / CAN FD** | 两种 CAN 帧格式。FD 载荷更大、更省带宽 —— **生产建议 FD**；配错格式的典型症状是"心跳收得到、我的请求没人应" |
| **端点（endpoint）** | 设备里的一个"参数 / 状态量"，有路径（如 `axis0.encoder.pos_estimate`）、类型、读写权限 |
| **描述符（descriptor）** | 设备把自己的端点表打包给你的一份清单；节点会缓存它，避免每次上电都重下 |
| **生命周期（lifecycle）** | ROS 2 的节点状态机。本驱动节点**必须**先 `configure` 再 `activate` 才能用（§5） |
| **tick / 控制周期** | 节点内部以固定频率跑一圈（默认 1 kHz），在这一圈里下发命令、收反馈 |
| **主站锁** | 防止两个程序同时上同一条总线。锁文件在 `/var/lock/jr-<通道>.lock` |

</details>

---

## 1. 准备清单

### 1.1 硬件

| 部件 | 说明 |
|---|---|
| 关节模组 | CyberBeast 系列（电机 + 驱动器一体，带编码器） |
| CAN 适配器 | ① **MCS CyberBeast USB2CAN**（开发/调试：插上即 `/dev/ttyACM0`，slcan 模式）② SocketCAN 兼容（显示为 `can0`，**生产推荐**）③ PEAK PCAN-USB |
| 电源 | 按模组铭牌。板上电压可以读回来核对（端点 `vbus_voltage`，我们实测的一台是 23 V 左右） |
| 线缆与终端电阻 | 总线**两端**各 120 Ω；走线尽量短、双绞；适配器与模组共地 |
| （可以没有） | 仓库自带**虚拟总线**后端：CI 与演示全程用它，**零硬件** |

接线示意（一条总线，两个模组）：

```
   PC ──USB──┐
             │
        [USB-CAN 适配器]                  120Ω 终端
             │                        ┌──────┐
             ├──────── CAN_H ────────┤  模组1 │
             │                        └──────┘
             ├──────── CAN_L ────────┬──────┐
             │                       │ 模组2 │
             └── GND（可选，长线建议）└──────┘
                                    120Ω 终端（另一端）
```

> 每次只建议先接 **1 个模组**、小幅动作，确认链路与配置都对了再扩到多关节/多总线。

### 1.2 软件：按你的操作系统

#### Ubuntu（推荐；生产就是这个）

```bash
# ① 装 ROS 2（下面以 Humble 为例。Jazzy / Lyrical 同理，把 humble 换成发行版名）
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions

# ② 装本项目的编译依赖
sudo apt install -y build-essential cmake git libyaml-cpp-dev \
                    ros-humble-diagnostic-updater ros-humble-ros2-control \
                    ros-humble-ros2-controllers ros-humble-robot-state-publisher
```

支持的发行版：**Humble (22.04) / Jazzy (24.04) / Lyrical (26.04，主目标)**，三个都实测过（§7）。

#### 实时内核（可选；要跑 1 kHz 控制建议装）

```bash
# ① 装 PREEMPT_RT 内核：按 Ubuntu 的实时内核文档来（不同版本包名不一样，常见的有
#    linux-realtime / linux-image-*-realtime）
# ② 给**当前用户**放开 RT 权限（下面两行就是我们在测试机上实际用的内容；
#    写错成不存在的组名会静默无效 —— 配完用 §5 的办法验证）
echo -e "$USER - rtprio 99\n$USER - memlock unlimited" | sudo tee /etc/security/limits.d/99-jointros.conf
# ③ 关掉 RT 限流（永久生效请写进 /etc/sysctl.d/）
sudo sysctl -w kernel.sched_rt_runtime_us=-1
```

> 这两步的价值我们**实测**过：配之前节点只能落到普通调度（`SCHED_OTHER`）且 `throttled=1`；
> 配之后是 `SCHED_FIFO(prio=80)`、`throttled=0` —— 状态在 `jr_ctl status` 里就能看到。

#### Windows / macOS（只能做离线自检）

可以编译并测试**实时核心库**与 3 个**不依赖 ROS** 的工具（`jr_hw_verify` / `jr_gen_config` / `jr_bus_plan`），
但**不能**跑 `jr_bus` 驱动节点、`ros2_control` 组件、`jr_ctl`（它们都依赖 ROS 2）。
接真机做控制请用 Ubuntu；WSL2 能跑通功能链路，但**不保证实时**。

### 1.3 拿到两个仓库

```bash
git clone <JointSDK 仓库地址>  ~/JointSDK     # 协议 / 驱动层（客户拿到的地址）
git clone <JointROS 仓库地址>  ~/JointROS     # 本仓库（ROS 2 集成层）
```

下面所有命令都假设它们是**兄弟目录**（本仓库默认去 `../JointSDK` 找 SDK；也可以用
`JRSDK_SOURCE_DIR`，或先装好 SDK 让 `find_package(jsdk_can)` 找到，见 [`cmake/jr_sdk.cmake`](cmake/jr_sdk.cmake)）。

---

## 2. 五分钟上手（不需要硬件）

### 路线甲：容器里一条命令（第一次最省事，需要有 WSL2 + Docker）

```bash
cd ~/JointROS
bash docker/run.sh jazzy        # 也可 humble / lyrical
```

**应该看到**：容器内 `ctest` 全通过、`colcon test` **0 failures**、`joint_trajectory_controller`
端到端 **PASS**（`mit` 与 `csp` 两种模式）。
一次性准备与镜像源问题见 [`docker/README.md`](docker/README.md)；各发行版的实测记录见 DESIGN §13.2。

### 路线乙：在你自己的机器上（已装 ROS 2）

```bash
cd ~/JointROS
source /opt/ros/humble/setup.bash
colcon build --base-paths jr_interfaces jr_ros2 jr_ros2_control jr_bringup \
             --cmake-args -DJRSDK_SOURCE_DIR=$HOME/JointSDK
source install/setup.bash

ros2 launch jr_bringup vbus_demo.launch.py      # 虚拟关节（不会碰任何真硬件）
```

另开一个终端（**建议把下面几条都试一遍**，这是最直接的"它真的在工作"的证据）：

```bash
source ~/JointROS/install/setup.bash
J="ros2 run jr_ros2 jr_ctl --node vbusrp"
$J status                                   # 总线状态快照
$J ep-list --filter axis0. --max 3          # 设备端点（id / 类型 / 读写）
ros2 topic echo /vbusrp/joint_states --once # 标准关节状态话题
```

**应该看到**（真实输出长这样）：

```
snapshot at tick 28120
  bus=vbusrp link_up=1 nodes_online=2 degraded=1
  tx=30 rx=22678 tx_failed=0 rx_dropped=0 keepalive=0 link_errors=0
  ...
```

`nodes_online=2` = 两个虚拟关节都在应答。

**让它真的动一下**（虚拟关节，安全）：

```bash
ros2 launch jr_bringup vbus_demo.launch.py jog:=true    # 起来后自动使能 + 点动一次
ros2 launch jr_bringup vbus_demo.launch.py joints:=1    # 只 1 个关节
ros2 launch jr_bringup humanoid_2bus.launch.py          # 人形：两条总线 + 示例 URDF
```

> **两条最容易踩的**
> 1. **不要在仓库根目录直接 `colcon build`。** 根目录的 `CMakeLists.txt` 是"纯 CMake 开发入口"，
>    colcon 会把它当成一个 `cmake` 包**并不再深入子目录** ⇒ 真正的 `jr_*` 包不会被编译，
>    但测试却可能照样跑（看着像成功）。请用 `--base-paths` 指定包目录，
>    或把 `jr_*` 包拷进你自己的 ROS 工作区 `src/` 再 `colcon build`。
> 2. **`jr_bus` 是生命周期节点，没有 autostart。** 只 `ros2 run` 的话进程"活着"，
>    但话题/服务/诊断**一个都不建**（都在 `activate` 那一步创建），客户端只会看到"服务不可达"。
>    `ros2 launch jr_bringup ...` 已经替你走了这两步；手写 launch 时别忘了（§5）。

### 路线丙：完全不用 ROS，先验证核心库

```bash
cd ~/JointROS && tools/build_dev.sh          # 默认找 ../JointSDK；JRSDK_SOURCE_DIR=... 可覆盖
```

**应该看到**：`100% tests passed out of 11`（Windows/MinGW 下同样 11/11）。

---

## 3. 第一次接真机（照这个顺序做）

> 顺序不是仪式：**先扫总线生成配置 → 再只读体检 → 最后才让节点上总线**。
> 每一步都能单独失败并告诉你原因，出问题时你不会同时面对三个未知数。

### 3.1 让适配器出现成一个设备

| 后端 | 做法 | 适用 |
|---|---|---|
| **slcan**（USB2CAN 出厂模式） | 插上后即 `/dev/ttyACM0`（`ls -l /dev/ttyACM0`）。⚠ slcan 是 ASCII 串口协议，**只适合配置 / 监控 / 低速**，不要用它跑高频控制 | 开发、调试 |
| **SocketCAN** | 把适配器配置成 SocketCAN 模式后，`ip link show can0` 应显示 `UP`（置位方式见适配器手册；本项目只要求 `can0` 已经 `UP`） | **生产** |

串口权限不够时，把用户加进 `dialout` 组（**要重新登录**才生效）：

```bash
sudo usermod -aG dialout $USER
```

### 3.2 扫总线 → 生成配置（`jr_gen_config`）

```bash
source ~/JointROS/install/setup.bash
mkdir -p ~/jr_hw
ros2 run jr_ros2 jr_gen_config --if slcan --channel /dev/ttyACM0 --serial-baud 115200 \
                               --name axis1 --out ~/jr_hw/axis1.yaml
```

它会：听心跳把设备发现出来 → 把**从设备读回**的量程 / 减速比 / 心跳周期写进 YAML →
**当场回读自检**（能被加载器读、能过校验、能过总线预算）→ 自检不过**就不落盘**。

**应该看到**（我们在真机上跑这条命令的真实输出）：

```
[info] probe_max=16 -> discovered 1 node(s)
[info] node 1: fw=0x00000609 hw=0x00040237 serial=0x0 classic=yes
[info]   gear=7.7500 pos=12.5000 vel=65.0000 trq=50.0000 kp_max=500.0 kd_max=5.0 hb=100 ms break_timeout=0 ms calibrated=yes
jr_gen_config: 已写出 /home/you/jr_hw/axis1.yaml（1463 字节，自检通过：load+validate+plan_bus）
  ⚠ 生成后请复核：master_id / is_fd / 关节名（见文件头注释）
```

生成的文件长这样（**每一节的含义见 §4**），其中 `limits` 是设备读回的真实量程：

```yaml
jr:
  tick_groups:
    - {name: g0, rate_hz: 1000, buses: [axis1], cpu: -1, priority: -1}
  buses:
    - name: axis1
      type: slcan
      interface: /dev/ttyACM0
      serial_baud: 115200         # 串口速率（不是 CAN 速率）
      is_fd: false                # 由设备的 classic 位推断
      master_id: 1
      bitrate: {nominal: 1000000, data: 5000000}
      joints: [j1]
  joints:
    - {name: j1, bus: axis1, node_id: 1, mode: mit}
  limits:
    j1: {position: [-12.5, 12.5], velocity: 65.0, effort: 50.0}
  command:
    timeout_ms: 100
```

### 3.3 只读体检（`jr_hw_verify`）——**上电前最后一道闸**

```bash
ros2 run jr_ros2 jr_hw_verify --config ~/jr_hw/axis1.yaml
```

**只读**：不使能、不发控制帧。它逐层给结论：

| 层 | 检查什么 |
|---|---|
| **L1** | 配置能否被加载与校验（YAML 语法、取值域、引用一致性） |
| **L2** | 打开总线：锁（谁持有）、SDK ABI 自检、HAL 打开 |
| **L3** | 节点发现：配置里声明的 `node_id` 是否都在应答；有没有"配置外"的节点 |
| **L4** | 设备身份 / 量程 / 标定标志（fw/hw/serial、gear、限值——**都来自设备读回**） |
| **L5** | 心跳与看门狗（`heartbeat_rate_ms`，`break_timeout`：**0 = 禁用**） |
| **L6** | 一致性与总线预算（帧数 × 帧时间 ≤ tick 预算的 60%） |

**应该看到**（我们在一台真机上的典型结果）：**9 通过 / 2 告警 / 0 失败**。
告警通常是"设备侧看门狗未武装（`break_timeout=0`）"这类**只告诉你、不会替你改设备**的提示 —— 这是故意的。

### 3.4 起节点（生命周期四步）

```bash
ros2 run jr_ros2 jr_bus --ros-args -r __node:=axis1 \
    -p config_file:=$HOME/jr_hw/axis1.yaml -p bus:=axis1 &
ros2 lifecycle set /axis1 configure      # 读配置 + 打开总线（锁、ABI、设备发现、描述符）
ros2 lifecycle set /axis1 activate       # ← 话题 / 服务 / 诊断都在这一步才创建
ros2 run jr_ros2 jr_ctl --node axis1 status
```

`activate` 之后节点会自己打印它建了什么（真实输出）：

```
[axis1]: bus 'axis1' configured:
[axis1]: configured (joints NOT enabled). Enable with the SetEnabled service, or set safety.auto_enable=true.
[axis1]: publishers/subscribers up: joint_feedback=500 Hz, joint_states=100 Hz
[axis1]: services up (19): motion/lifecycle + params/descriptor/diagnostics
[axis1]: active: tick running, joints NOT enabled (cold start never drives - G6)
```

> ⚠ 从这一刻起，**这条总线上不要再开第二个主站**（`jsdk-cli`、Python 绑定、第二个 `jr_bus` 都算）。
> 节点自己会拿文件锁拦住第二个进程（锁文件 `/var/lock/jr-<通道>.lock`，例如
> `jr-slcan__dev_ttyACM0.lock`），失败时会告诉你**是哪个进程**占着。

### 3.5 读参数核对

```bash
J="ros2 run jr_ros2 jr_ctl --node axis1"
$J ep-list --filter axis0. --max 20        # 先看设备有哪些端点（路径前缀来自设备描述符）
$J ep-lookup axis0.encoder.pos_estimate    # 查单个端点：id / 类型 / 读写权限
$J read --paths axis0.config.can.node_id,axis0.encoder.pos_estimate
$J info                                    # 设备身份：fw / hw / serial / classic 位
```

**应该看到**：`read` 每行的形状是 `关节名  路径  声明类型  值`：

```
  j1           axis0.config.can.heartbeat_rate_ms           u32         5
  j1           axis0.encoder.pos_estimate                   f32         0.0521071
```

> 💡 **反馈怎么看才准**：`~/joint_feedback` 里的位置/速度来自设备**主动上报的帧**，
> 某些固件上这帧可能是陈旧的（我们会如实打 `feedback_stale` 标志）。
> **要真值就直接读端点**：`pos_estimate` / `vel_estimate`（上表那条命令），这条路每次都对。

### 3.6 小心地让它动一下（**第一次请小幅、短时**）

```bash
J="ros2 run jr_ros2 jr_ctl --node axis1"
$J enable  --joints j1 --confirm
$J jog --joint j1 --pos 0.02 --kp 2 --kd 0.2 --duration-s 0.3 --confirm
$J disable --joints j1 --confirm
```

参考结果：`jog finished on 'j1': 300 ms / 272 ticks. Joint is disabled again (safe state).`

> ⚠ **`jog` 不给增益会被直接拒绝**（`kp=kd=torque=0` 是"零力矩"目标：电机不会动，
> 但整套流程会"成功" —— 我们宁可拒掉也不报一个没发生过的动作）。请显式给 `--kp/--kd`，
> 或者给 `--tau` 前馈。增益量级可以先用 `$J read --paths axis0.controller.config.pos_gain` 看个大概
> （真正的上限在 `$J info` 的 `kp_max/kd_max` 那一行：我们这台是 `kp_max=500 kd_max=5`）。

### 3.7 收工（每次都这样收）

```bash
J="ros2 run jr_ros2 jr_ctl --node axis1"
$J disable --joints j1 --confirm          # 确认关节已失能（安全态）
$J status | head -3                       # 看一眼还算不算健康
ros2 lifecycle set /axis1 deactivate      # 拆服务/话题
ros2 lifecycle set /axis1 cleanup
# 或直接 Ctrl-C 那个 jr_bus 进程：它走的是同一条"安全失能"路径
```

> 💡 关进程时**别用** `pkill -f jr_bus`：那个模式会匹配到**你自己的 shell**（我们真踩过）。
> 用 `pkill -x jr_bus`（精确匹配进程名）或走 lifecycle / Ctrl-C。

---

## 4. 配置文件怎么读

**一份 YAML 同时给三样东西用**：驱动节点（`jr_bus`）、`ros2_control` 组件、以及所有工具。
这样"节点看到的"和"控制器看到的"永远是同一份，不会漂移。

```yaml
jr:
  # ── 实时（可选）──────────────────────────────────────────────
  rt:
    enabled: true          # 试图用 PREEMPT_RT（SCHED_FIFO + mlock）
    policy: fifo           # fifo | other（other = 不做实时设置）
                           # ⚠ 还有个 deadline（SCHED_DEADLINE）**尚未实现**，写了会直接报错
    priority: 80
    mlock: true            # 锁内存，避免页错误打断控制循环
    cpu_affinity: []       # 如 [2, 3]：每个 tick 组依次取一个（要不要绑核见 §9）
    warn_if_throttled: true

  # ── 控制周期与"哪些总线一起跑"────────────────────────────────
  tick_groups:
    - {name: g0, rate_hz: 1000, buses: [axis1], cpu: -1, priority: -1}
    #  ↑ 一个 tick 组 = 一个 jr_bus 进程；组内的总线一起跑、跨总线同步

  # ── 总线（一条 CAN 一条）─────────────────────────────────────
  buses:
    - name: axis1
      type: slcan                # socketcan | slcan | pcan | virtual
      interface: /dev/ttyACM0    # socketcan/pcan/slcan = 接口名；virtual 用下面的 spec
      # spec: "0:id=1,gear=7.75,pmax=12.5,..."   # 仅 virtual：设备模型串
      serial_baud: 115200        # 仅 slcan：串口速率（不是 CAN 速率）
      is_fd: false               # Classic 起步；FD 请显式写 true
      master_id: 1               # ⚠ 设备侧读不回来，别让两台主站用同一个号
      bitrate: {nominal: 1000000, data: 5000000}
      joints: [j1]               # 这条总线上的关节名（引用下面的 joints）
      # 可覆盖全局的：feedback / heartbeat_ms / poll_period_ms / max_bus_load /
      #               auto_keepalive / clamp_target / arm_device_watchdog /
      #               state_timeout_ms / rx_burst_limit

  # ── 关节（名字必须与 URDF / 控制器参数逐字一致）──────────────
  joints:
    - {name: j1, bus: axis1, node_id: 1, mode: mit}
    #  mode: mit | csp | csv | cst   ← 使能时定下，运行期不改（换模式=改配置+重启）

  # ── 限位（**从设备读回**，别手抄）────────────────────────────
  limits:
    j1:
      position: [-12.5, 12.5]    # rad（输出端）
      velocity: 65.0             # rad/s
      effort: 50.0               # N·m
      # stiffness/damping 是**控制器增益**，设备里没有，故意不写（要用请自己填）

  # ── 反馈与发布率 ─────────────────────────────────────────────
  feedback:
    endpoint_poll_ms: 0    # F11：端点轮询（0 = 关，默认）。>0 时在**本条总线所有关节都失能**时
                          #   按周期读 pos_estimate/vel_estimate，用它覆盖话题里的位置/速度，
                          #   并置 status_flags 的 0x80000000 表示来源。关节一动它自动停
                          #   （读参数要暂停 tick，而暂停会先安全失能）。真机建议 5000（真机上 1000 ms 实测约 4100 丢拍/s）
    state_request_ms: 0    # F31：非阻塞状态请求（0 = 关，默认）。>0 时在 tick 里按周期发 QUERY_POS_VEL(0x41)，
                          #   发出即返回、**不需要安全暂停窗口 ⇒ 驱动中也能用**；SDK 建议 ≤10 Hz/关节
                          #   （100..5000）。⚠ 该帧不喂设备看门狗，只是额外查询；判新鲜度看 age_ms
    source: broadcast_heartbeat  # broadcast_heartbeat | heartbeat_only | unicast_poll | unicast_only
    heartbeat_ms: 5              # 期望的设备心跳周期
    poll_period_ms: 10           # SDK 轮询周期（只在上面选 unicast_* 时有用）
    publish_hz: 500              # ~/joint_feedback 发布率
    joint_state_hz: 100          # ~/joint_states 发布率

  # ── 命令超时（安全网）────────────────────────────────────────
  command:
    timeout_ms: 100              # 多久没收到新命令就执行 timeout_action
    timeout_action: hold         # hold | zero_torque | disable
    interpolation: none          # none | linear（把运动量插值到新目标）

  # ── 安全 ─────────────────────────────────────────────────────
  safety:
    auto_enable: false           # 冷启动**不**自动使能（默认）
    require_calibrated: true     # 未标定就拒绝物理量命令
    arm_device_watchdog: false   # **不**替你改设备（要看门狗请显式打开）
    clamp_target: false          # false = 越界目标被拒并发安全帧
    on_exit_action: disable      # disable | hold | zero_torque | estop | none
    fault_action: none           # none | disable_joint | estop_bus（默认不动手）
    fault_auto_reset: false

  # ── 写保护（服务的闸门）──────────────────────────────────────
  params:
    allow_write: false           # ⚠ 默认**关**：要用 ROS 服务写设备参数得显式打开
    allow_flash_persist: false   # 同理：默认禁止“存 Flash”这类危险操作

  # ── 描述符下载与缓存 ─────────────────────────────────────────
  descriptor:
    cache_enabled: true
    # cache_dir: 省略 = 平台默认（~/.cache/jr）
    timeout_ms: 5000
    retries: 3                   # 真机（slcan）首帧易丢，必须重试
    retry_backoff_ms: 100
    retain: all                  # all | filtered（filtered 要配 filter_paths）
    filter_paths: []

  # ── 单主站锁 ─────────────────────────────────────────────────
  bus_lock:
    enabled: true
    # lock_dir: 省略 = 平台默认（Linux /var/lock）
    allow_shared: false          # true 只用于调试（会持续告警）
```

**三个最常改的地方**

| 想干什么 | 改哪里 | 注意 |
|---|---|---|
| 换命令模式（MIT ↔ CSP/CSV/CST） | `joints[].mode` + URDF 里的 `<command_interface>` | 两者必须一致，改完要 `cleanup`+`configure`+`activate` |
| 调限位 / 发布率 | `limits.*` / `feedback.publish_hz`、`joint_state_hz` | 限位应从设备读回（`jr_gen_config` 已经帮你写好） |
| **用服务写参数 / 存 Flash** | `params.allow_write`、`params.allow_flash_persist` | **默认都是 `false`**（写不进去先查这里，不是设备坏了） |
| 压榨实时性或排查抖动 | `rt.*` + `tick_groups[].cpu/priority` | 先用 `jr_ctl status` 里的 `tick_overruns` / `rt_throttled` 看现状再调 |

改完配置**必须重走 lifecycle**（`deactivate → cleanup → configure → activate`），节点运行期不会重读 YAML。

---

## 5. 生命周期与运行状态

`jr_bus` 是 ROS 2 **生命周期节点**，四个状态各自做什么：

```
         ┌───────────────┐  configure   ┌──────────┐  activate  ┌────────┐
  unconfigured ──────────┼─────────────▶│ inactive │───────────▶│ active │
         ▲                │              └──────────┘            └────────┘
         │  cleanup       │                    │  deactivate          │
         └────────────────┴────────────────────┘                      │
                                                            （SIGINT / Ctrl-C）
                                                       走同一条"安全失能"路径
```

| 状态 | 建了什么 | 能干什么 |
|---|---|---|
| `unconfigured` | 什么都没建（`ros2 run` 之后就是这个状态） | 只能 `configure` |
| `inactive` | 读配置、打开总线（锁 / ABI / 设备发现 / 描述符）、校验预算 | 可以 `activate`；出问题看 `ros2 lifecycle get` 与节点日志 |
| `active` | 起 tick 线程、建话题 / 19 个服务 / 诊断 | 正常使用：发命令、读参数、点动、标定 |
| 错误 | — | 先 `deactivate` + `cleanup`，修完再 `configure` |

**一条命令判断"它活着且正常"**：

```bash
J="ros2 run jr_ros2 jr_ctl --node axis1"
$J status
```

输出里这几个字段最该看（**真实输出**）：

```
snapshot at tick 28120                     ← tick 在跑（数字会一直涨）
  bus=vbusrp link_up=1 nodes_online=2      ← 链路在 + 有几个节点应答
  degraded=1                               ← 有需要注意的地方（看下面 note）
  tx=30 rx=22678 tx_failed=0 rx_dropped=0  ← 发/收帧数；失败与丢弃应为 0
  last_rx_age=2 ms hal_bus_flags=0x00000001
  load≈16.9% tick_overruns=1               ← 总线负载与"没跟上拍"的次数
  note: ...                                ← 人话解释（含建议）
```

| 字段 | 含义 | 什么算不正常 |
|---|---|---|
| `tick_overruns` | tick 没能按时跑完的次数 | 持续增长 ⇒ 看 RT 配置 / 降 `rate_hz` / 降总线负载 |
| `load≈N%` | 估算的总线占用 | 接近 60% 就该警惕（预算检查超 60% 会**拒绝启动**） |
| `rx_dropped` / `tx_failed` | 链路质量 | 长期不为 0 ⇒ 检查线缆/终端电阻/适配器 |
| `degraded` + `note` | 汇总 + 人话原因 | 看 `note` 里点名的那一项（未标定 / 心跳关掉 / …） |

**退出行为**：`Ctrl-C`（SIGINT）与 `lifecycle set deactivate` 走**同一条**路径 ——
先发安全目标、等 2 个周期、失能、回到安全态，**不会留下抱力关节**。

---

## 6. 常用任务速查（照抄这一节就行）

先设好这两个变量（节点名换成你的）：

```bash
source ~/JointROS/install/setup.bash
J="ros2 run jr_ros2 jr_ctl --node axis1"
```

| 想做什么 | 命令 | 说明 |
|---|---|---|
| 看总线状态 | `$J status` | 帧数 / 负载 / 丢帧 / note |
| 看设备身份 | `$J info` | fw / hw / serial / 是不是 Classic |
| 列端点 | `$J ep-list --filter axis0. --max 20` | 路径前缀来自设备描述符；`*` 结尾=前缀，`.` 结尾=段前缀 |
| 查单个端点 | `$J ep-lookup <路径>` | id / 类型 / 读写权限（**不做模糊匹配**） |
| 读参数 | `$J read --paths a,b [--joints x,y]` | 支持一次读多个 |
| 写参数（会读回校验） | `$J write --path <路径> --value <值> --confirm` | 打印 `requested/value/verified` 三段：请求值、读回值、是否一致。⚠ **默认会被拒**（见下） |
| 改 node_id | `$J node-id <关节> <新ID> [--persist] --confirm` | ⚠ 会改变寻址，之后要重新 `configure` |
| 存 Flash | `$J save [--joints a,b] --confirm` | 掉电保留；受 `params.allow_flash_persist` 闸门约束 |
| 使能 / 失能 | `$J enable|disable [--joints a,b] --confirm` | 失能走 SDK 安全序列 |
| 点动 | `$J jog --joint <名> --pos <rad> --kp K --kd D --duration-s S --confirm` | 限时 + 自动失能收尾；**不给增益会被拒** |
| 标定 / 回零 | `$J calib` / `$J home` | 真机上会动关节（实测 `calib` 通过并读回 `pre_calibrated=true`）；⚠ `home` 需要**限位开关** —— 没有限位的模组会被设备以 `HOMING_WITHOUT_ENDSTOP` 拒绝；虚拟后端两个都不实现 |
| 置零 | `$J zero [--joints a,b]` | 不落 Flash |
| 清故障 | `$J fault-reset --confirm` | `STOP_MOTOR → CLEAR_ERRORS`，**不动电机** |
| 软复位 | `$J reset --confirm` | 复位设备；之后必须重新 `configure` |
| 急停 | `ros2 topic pub --once /jr/estop_all std_msgs/msg/Bool '{data: true}'`（或发 `~/estop`） | ⚠ **整条总线广播**，且**会锁存**（§9.3）。`jr_ctl` 里**没有** `estop` 子命令 —— 急停走的是话题 |
| 描述符导出 / 导入 | `$J desc-export <file>` / `$J desc-import <file> --confirm` | 导入是产线预烧场景；`--persist` 本 SDK 不支持（会被明确拒绝） |
| 心跳建议 | `$J hb-hint --rate-ms 5` | **只给建议，不改设备** |
| 算总线预算（不接硬件） | `ros2 run jr_ros2 jr_bus_plan --config ~/jr_hw/axis1.yaml` | 帧数 × 帧时间 ≤ tick 预算 60% 才算可行 |

> `jr_ctl` 的退出码是**分了类的**，方便脚本判断：
> `0` 成功 / `1` 操作失败（服务调通了但 `success=false`）/ `2` 用法错 / **`4` 服务不可达**（节点没起、没 activate、或名字错）。

> ⚠ **写不进去先查闸门，别先怀疑设备**：`write` / `save` 默认是**关**的（`params.allow_write` /
> `params.allow_flash_persist`，两者默认 `false`）。默认打开时会看到这样的**明确拒绝**（不是超时、不是静默失败）：
>
> ```
> $ ros2 run jr_ros2 jr_ctl --node vbusrp write --path axis0.config.can.heartbeat_rate_ms --value 5 --confirm
> parameter writes are disabled (safety.allow_param_write=false, i.e. params.allow_write)
>   all_verified=0
> [ros2run]: Process exited with failure 1
> ```
>
> 打开方法：在 YAML 里写 `params: {allow_write: true}`（存 Flash 同理，用 `allow_flash_persist`）。

---

## 7. 怎么自测（三层，从快到慢）

| 层 | 命令 | 应该看到 | 需要什么 |
|---|---|---|---|
| ① 离线核心（最快） | `tools/build_dev.sh`（或 `ctest --test-dir build`） | `100% tests passed out of 11` | 只有编译器 + CMake（**Windows 也行**） |
| ② 容器三发行版 | `bash docker/run.sh jazzy\|humble\|lyrical` | 三条全绿（`ctest` / `colcon test` / JTC 端到端）—— **2026-09-27 实测**：jazzy / humble / lyrical 全部 `RC=0`、`ctest` 13/13、JTC **PASS** | WSL2 + Docker |
| ③ 真机 | `jr_hw_verify --config ...` → 起节点 → `colcon test --base-paths jr_ros2` | 体检 0 失败；`colcon test` 全过 | 一台 Ubuntu + 真实模组 |

> 具体数字会随版本变（当前版本与逐项证据在 [`docs/DESIGN.zh-CN.md`](docs/DESIGN.zh-CN.md) 的 §13.2 与 §14），
> 建议记住**判据**而不是数字：`tests passed` / `0 failures` / 体检 0 失败。

---

## 8. 集成到你的系统（三条路线）

### 路线 A：用现成控制器（最省事，推荐先走这条）

让 `ros2_control` 的标准控制器（`joint_state_broadcaster` + `joint_trajectory_controller`，
向上还能接 MoveIt）来驱动关节。你只需要两样东西：**一份配置 YAML**（§3.2 生成的那份）和
**URDF 里的一段 `ros2_control` 声明**。

```xml
<ros2_control name="jr" type="system">
  <hardware>
    <plugin>jr_ros2_control/JrSystemInterface</plugin>
    <param name="config_file">/home/you/jr_hw/axis1.yaml</param>
    <!-- 两个可选参数，省略时就是下面这两个默认值 -->
    <param name="tick_source">internal</param>   <!-- internal | controller_manager -->
    <param name="gain_mode">wire</param>         <!-- wire(kp/kd，线上值) | si(stiffness/damping，物理量) -->
  </hardware>
  <!-- ⚠ 关节名必须与 YAML 里的 joints[].name **逐字一致**；接口集必须与 joints[].mode 一致 -->
  <joint name="j1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <command_interface name="kp"/>       <!-- 只有 mode: mit 才导出这两个 -->
    <command_interface name="kd"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

**三条硬约束**（写错的症状都很像"硬件起不来"）：

1. **`<command_interface>` 必须与实际导出的接口集一致**。导出集由 `joints[].mode` 决定：
   `mit` → `position/velocity/effort + kp/kd`；`csp` → 只有 `position`；`csv` → 只有 `velocity`；`cst` → 只有 `effort`。
2. **关节名逐字一致**（YAML ↔ URDF ↔ 控制器参数）。不一致会在 `on_init` 就失败并列出可用名字。
3. **`gain_mode` 二选一**：`wire` 导出 `kp/kd`（与 MIT 帧字段一致的线上值），`si` 导出
   `stiffness/damping`（N·m/rad、N·m·s/rad）。两组**不会同时导出** —— 单位搞错比 claim 失败危险得多。

启动顺序就是标准 `ros2_control` 的那一套（把 `your_robot.urdf`、控制器参数换成你自己的）：

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat your_robot.urdf)"
ros2 run controller_manager ros2_control_node
ros2 run controller_manager spawner joint_state_broadcaster --param-file your_controllers.yaml
ros2 run controller_manager spawner joint_trajectory_controller --param-file your_controllers.yaml
```

仓库自带一个**可跑的端到端示例**（起 `controller_manager` + JTC + 发一条真实轨迹 + 断言终点误差）：

```bash
JR_DEMO_INSTALL=<colcon install 目录> bash jr_ros2_control/test/jtc_demo/run.sh --mode mit
JR_DEMO_INSTALL=<colcon install 目录> bash jr_ros2_control/test/jtc_demo/run.sh --mode csp
```

URDF 怎么写、怎么写错（**症状对照表**）见 [`jr_bringup/docs/URDF.zh-CN.md`](jr_bringup/docs/URDF.zh-CN.md)。

### 路线 B：自己写一个 ROS 2 节点（只需要 `jr_interfaces`）

节点（`jr_bus`）已经把"这条总线"整成一个 ROS 2 部件。你订阅它的话题、给它发命令、或者调它的服务即可 ——
**你的代码可以只依赖 `jr_interfaces`**（消息 / 服务定义），不需要链接任何 C++ 库。

命名空间：节点名叫什么，话题就在它下面（`~/joint_states` ⇒ `/vbusrp/joint_states`）。

**它发布（你订阅）**

| 话题 | 类型 | 默认频率 | 说明 |
|---|---|---|---|
| `~/joint_states` | `sensor_msgs/JointState` | 100 Hz | 标准关节状态（位置/速度/力矩） |
| `~/joint_feedback` | `jr_interfaces/JointFeedbackArray` | 500 Hz | 更全的原始反馈（温度/电压/错误位/是否陈旧…） |
| `~/bus_status` | `jr_interfaces/BusStatus` | — | 总线级统计（帧数/丢帧/负载/链路） |
| `~/rt_stats` | `jr_interfaces/RtStats` | — | 实时性能（抖动、丢拍、命令延迟） |
| `~/faults` | `jr_interfaces/JointFault` | 事件 | 故障出现/清除 |

**它订阅（你发布）**

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/cmd` | `jr_interfaces/JointCommandArray` | **按配置里的模式**下发的命令（`mode`：CSP=8 / CSV=9 / CST=10 / CURRENT=11，单位随模式） |
| `~/cmd_mit` | `jr_interfaces/MitCommandArray` | MIT 语义命令：`position/velocity/kp/kd/torque`（`gain_mode` 选 `WIRE=0` 或 `SI=1`） |
| `~/estop` | `std_msgs/Bool` | 本条总线急停（`true`） |
| `/jr/estop_all` | `std_msgs/Bool` | **所有**总线节点都订阅（全局急停） |

消息字段一句话看全：

```bash
ros2 interface show jr_interfaces/msg/MitCommand
ros2 interface show jr_interfaces/msg/JointCommand
```

**发一条 MIT 命令"到底长什么样"**（下面这条在虚拟总线上实测有效：`j1` 从 `0.008` 走到 `0.072`）：

```bash
ros2 topic pub --once /vbusrp/cmd_mit jr_interfaces/msg/MitCommandArray \
  '{commands: [{name: j1, gain_mode: 1, position: 0.05, velocity: 0.0, stiffness: 20.0, damping: 1.0, torque: 0.0}]}'
```

> ⚠ **两个坑（都实测踩过）**：
> 1. **用 `gain_mode: 1`（SI，N·m/rad）+ `stiffness`/`damping` 字段**。`gain_mode: 0`（WIRE）时
>    `kp`/`kd` 要按设备 `kp_max`/`kd_max` **归一化**（`axis0.controller.config.*`、`$J info` 里能看到），
>    随手写 `kp: 2` 可能小到看不出任何动作 —— 而且**不报错**（命令被正常接受）。
> 2. **`MitCommandArray` 只更新列出的关节**，未列出的关节保持上一次目标（不会自动泄力）；
>    超时后的行为由 `command.timeout_action` 决定，所以要**持续发**。

**它提供的服务**：19 个，名字是 srv 类型的 snake_case（`GetBusStats` → `~/get_bus_stats`）。
日常运维**不必记服务名** —— 用 `jr_ctl`（§6）或 `ros2 service list | grep <节点名>` 看全。

> ⚠ 命令有**超时保护**：`command.timeout_ms`（默认 100 ms）内没收到新命令，节点会按
> `command.timeout_action` 走安全动作。所以在自己的控制器里请**持续发**，不要"发一次就不管了"。

### 路线 C：不用 ROS

| 你想要 | 用什么 | 说明 |
|---|---|---|
| C++、无 ROS、自己做调度 | 直接链本仓库的 **`jr_core`**（`jr_ros2` 里的静态库，**不依赖 rclcpp**） | 享受同一套实时核心、安全策略与配置校验；接口就是 `jr_ros2/include/jr_ros2/` 下的头文件 |
| C / C++ / Python，只要协议与设备 | 直接用 **JointSDK** | 那是最底层；本仓库的价值（ROS 接口、诊断、工具、纪律）就没有了 |

> 这条路线我们不提供现成示例（客户形态差异太大）。需要时请对着 `jr_core` 头文件与
> DESIGN §5/§6 设计；也可以只借 `jr_core` 的**配置校验 + 总线预算**两件小事。

---

## 9. 现场纪律与排障

### 9.1 三条硬纪律

1. **同一条 CAN 总线只允许一个主站进程。** 节点运行期间不要再跑 `jsdk-cli` / Python 绑定。
   设备只记得"最后见到的那个主站"，症状是心跳乱跳、参数偶发写不进。运维请用 `jr_ctl`（走服务）。
   我们的文件锁能拦住第二个 `jr_bus`，但**拦不住 `jsdk-cli`**（SDK 侧不加锁）—— 那半边靠纪律。
2. **生产用 CAN FD，且每条总线 ≤ 7 个关节**（广播同步寻址上限）。Classic / slcan 只用于上电检查与调试；
   启动时的总线预算检查会直接拒绝不可行的组合。
3. **冷启动不自动使能**（`safety.auto_enable` 默认 `false`），退出默认走**安全失能**序列，
   不会留下"抱力"关节。`estop` 是**整条总线的广播**，而且**锁存**（§9.3）。

### 9.2 怎么读故障与错误码

```bash
J="ros2 run jr_ros2 jr_ctl --node axis1"
$J status                                   # 总线级：负载/丢帧/note
$J read --paths axis0.error,axis0.motor.error
ros2 topic list | grep faults               # 故障事件话题（平时安静，有出现/清除时才发消息）
```

⚠ **别只看摘要**：`JointFeedback.error_code` 是 MIT 协议的 **4-bit 摘要**，会把不同故障映射成同一个值
（我们实测过：急停与看门狗都报 `CAN_TIMEOUT`）。**排障请看原始位**：
`heartbeat_error`（心跳子系统位图）、`axis_error`（32-bit），以及 `$J status` 的 `note`。

### 9.3 排障表（症状 → 最可能原因 → 怎么办）

| 症状 | 最可能的原因 | 怎么办 |
|---|---|---|
| `jr_ctl` 一直报"服务不可达"（rc=4） | 节点没起来 / 没 `configure`+`activate` / 节点名写错 | `ros2 node list`；`ros2 lifecycle get /<节点名>` 必须是 `active` |
| **收得到心跳，但我的请求没人应** | **帧格式不对**（设备是 Classic 而你按 CAN FD 发） | 先看 `$J info` 的 `classic` 位；配置里 `is_fd` 默认按 Classic 起步。⚠ **别拿 `is_fd` 做实验**：用错格式打开 slcan 是**破坏性**的，可能要拔插适配器才能恢复 |
| 偶发读不到（约 1/10 次） | slcan 的**首帧丢失**（固有特性） | 重试一次即可；高频/生产请换 SocketCAN 或 CAN FD |
| 第二个进程起不来 | 单主站锁 | 报错里会写**谁**占着（PID）与锁文件路径；`lock.allow_shared=true` 只用于调试 |
| `estop` 之后关节怎么都不动 | 急停**锁存**了 | `$J fault-reset --confirm`；不行就 `$J reset --confirm`（软复位，之后要重新 configure）或**断电重启**（我们实测：锁存状态下只有 reset/断电一定能清） |
| 命令发出去了，关节不动 | ① `kp/kd/tau` 全 0（会被拒）② 没使能 ③ 命令模式与 `joints[].mode` 不符（会被丢弃并计数） | `$J status` 看 `cmd_rejected`；`$J enable --joints ... --confirm`；模式见配置 |
| 反馈位置/速度看着不对 | 反馈帧是设备**主动上报**的，某些固件上它会陈旧 | 看 `~/joint_feedback` 的 `feedback_stale`；**要真值直接读端点**：`$J read --paths axis0.encoder.pos_estimate,axis0.encoder.vel_estimate` |
| 实时性不达标（抖动大 / 丢拍多） | 没有 RT 权限，或被内核限流 | 按 §1.2 配 RT；`$J status` 的 `tick_overruns`、`rt_throttled` 是判据 |
| `calib` / `home` 失败 | ① 虚拟后端不实现这两个状态机；② **`home` 需要限位开关** —— 本模组没有，设备直接报 `detail_err=0x00020000`(`HOMING_WITHOUT_ENDSTOP`) 并超时；③ 有故障未清时使能也会超时（实测） | 先 `$J fault-reset`（不动电机）清错误，再 `jr_hw_verify` 看标定标志；`home` 在本模组上属**设备侧不支持**，别当 bug 查 |
| 节点起不来，日志说 `bus_lock` 被占 | 上一次没退干净（尤其被 `kill -9`） | `pkill -x jr_bus`（**别用 `-f`**）；仍然占着就按报错里的 PID 处理 |
| 起节点报 `cannot open '/dev/ttyACM0' (slcan): invalid-argument` | 串口被别的进程占着 —— **最常见**是你自己起了 `slcand -o -c -s8 /dev/ttyACM0 slcan0` | 二选一：① 停掉 `slcand`（走裸串口路径）；② 改成 SocketCAN：`type: socketcan` + `interface: slcan0`（同一根物理总线，**不需要 sudo**） |
| 直接用 SDK / 工具时报 `configure: transport ... could not be sent` | 它们**默认按 CAN FD 起步**，而 Classic 链路（接口 `mtu 16`）上第一次发送必然失败（顺带：`master_id == node_id` 时请求与应答**共用一个 CAN ID**，抓包分不清收发） | 显式声明 Classic（Python `is_fd=False`、`jsdk-cli --classic`；本仓库生成的配置本来就有 `is_fd: false`）；`master_id` 取与 node_id 不同的值（生成器默认已改成 126） |

---

## 10. 已知限制与边界（如实登记）

- **命令模式**：`mit` / `csp` / `csv` / `cst` 可用；**`CURRENT` 明确拒绝**（`effort` 的单位会从 N·m 变成电机端 A，差 gear×kt，容易出事故）。
- **每条总线 ≤ 7 关节**（广播同步寻址上限）；超过会被预算检查拒绝。
- **反馈帧**（`~/joint_feedback` 的位置/速度）在某些固件上可能**陈旧**：我们会如实打
  `FEEDBACK_STALE`，`age_ms` 也不会假报"刚刚更新"；**把反馈源改成自动端点轮询还没做**（见 DESIGN §13.4）。
- **`calib` / `home`**：`calib` 已在真机跑通（读回 `pre_calibrated=true`，关节真的转了）；`home` 的失败**归因在设备侧** —— 本模组没有限位开关，设备报 `HOMING_WITHOUT_ENDSTOP`(0x20000) 并超时，`HOME` 这条路径在当前硬件上不可用（虚拟后端两个状态机都不实现）。
- `jr_latency_bench`（实时性能基准工具）**未实现**（需要真机才有意义）。
- `desc-import --persist`（把描述符写进设备 Flash）**不支持**（SDK 没有这个能力，会被明确拒绝，不会静默忽略）。
- **Windows / macOS** 不支持 ROS 侧（节点、`ros2_control`、`jr_ctl`）。
- 急停可以**锁存**；生产上请把"清锁存"写进你的操作规程（`fault-reset` → 必要时 `reset` / 断电）。

---
- **⚠ 单位别混**：`~/joint_feedback` 的 `position/velocity` 是**输出端 rad**，而 `jr_ctl read --paths axis0.encoder.pos_estimate` 读回的是**电机端 turns**（设备内存原样，SDK 不换算）⇒ 换算 `rad = turns × 2π / gear`（本机 gear=7.75 ⇒ ×0.8108）。直接相减会得到"差 0.1〜0.4"这种看着像故障的假象（我们自己也踩过）。另：Classic 链路上心跳里的位置只有 **0.0081 rad/LSB**，别拿它当高精度源。
- **驱动中的反馈**：开 `jr.feedback.state_request_ms`（如 100）后，节点在 tick 里以该周期发 `QUERY_POS_VEL`(0x41，**发出即返回、不需要安全暂停窗口 ⇒ 驱动中也能用**)，SDK 建议 **≤10 Hz/关节**、该帧**不喂设备看门狗**（只能额外发）。真机实测：设备**逐帧在回**（待机 3 s 内 30 请求/30 应答/30 心跳），代价 ≈ 0；`age_ms` 待机时也是真实的（13〜83 ms）。⚠ 判新鲜度**看 `age_ms`**（它一直可靠），**不要**用 `valid`（那是每周期旗标）或只看 `status_flags` 的 `FEEDBACK_STALE`（那位在本固件上粘滞）。
- **`estop` 锁存**：`fault-reset` 清不掉，只有软复位/断电重启能清（§9.3）；清了之后 `enable`/`jog` 一次通过（实测 `500 ms / 284 ticks`）。

## 11. 目录、文档、版本、许可

```
docs/DESIGN.zh-CN.md          设计与验收的**唯一权威**（架构 / 工作包 / 证据 / 变更记录）
docker/README.md              容器环境（三发行版矩阵）的一次性准备与用法
jr_bringup/README.md          示例与 launch（零硬件跑通整条链路）
jr_bringup/docs/URDF.zh-CN.md URDF 片段怎么写、怎么写错（症状对照表）
cmake/jr_sdk.cmake            JointSDK 的三种定位方式
jr_interfaces/                消息 / 服务定义（自研控制器可以只依赖它）
jr_ros2/                      实时核心库 jr_core + 驱动节点 jr_bus + 工具（jr_ctl / jr_hw_verify / jr_gen_config / jr_bus_plan）
jr_ros2_control/              ros2_control 硬件组件（JTC / MoveIt 走这条）
jr_bringup/                   launch / 配置模板 / URDF 示例 / 演示
tools/build_dev.sh            无 ROS 的"配置 + 构建 + 测试"一条命令
```

- **当前版本**：v0.19（逐版本变更见 DESIGN §14；已验证环境与逐项证据见 §13.2，含真机）。
- **许可**：**MIT**，见 [`LICENSE`](LICENSE)。
