# JointROS —— 把 CyberBeast 关节模组接进 ROS 2

**一句话**：关节模组（电机 + 驱动器 + 编码器一体）通过 CAN 接到电脑后，JointROS 把它变成一个
**标准的 ROS 2 机器人部件** —— 有 `/joint_states`、能被 `joint_trajectory_controller`（以及 MoveIt 之类）
驱动、有诊断、还有一套"上机前先体检"的工具。

|  |  |
|---|---|
| ✅ **这一层负责** | 实时控制循环与调度（可选 PREEMPT_RT）、ROS 2 接口（话题 / 服务 / 诊断）、安全策略（冷启动不使能、退出走安全失能、急停）、单主站纪律与总线预算检查、设备描述符缓存、上机工具链 |
| ❌ **这一层不负责** | CAN 帧格式、协议编解码、参数（端点）系统、设备状态机 —— 这些由 **JointSDK** 负责。本仓库是它之上的 ROS 2 集成层 |

**我该看哪一节？**

| 你的情况 | 去哪 |
|---|---|
| 还没硬件，想先看看它能干什么 | [§2 五分钟上手](#2-五分钟上手不需要硬件) |
| 有模组和 USB-CAN 盒，想让它转起来 | [§3 第一次接真机](#3-第一次接真机照这个顺序做) |
| 要把关节接进自己的机器人（JTC / MoveIt / 自研控制器） | [§5 集成到你的系统](#5-集成到你的系统三条路线) |
| 报错了 / 不动 / 读不到数 | [§6 现场纪律与排障表](#6-现场纪律与排障表) |
| 想了解架构与验收证据 | [`docs/DESIGN.zh-CN.md`](docs/DESIGN.zh-CN.md)（**唯一权威**） |

---

## 1. 准备清单

### 1.1 硬件

| 部件 | 说明 |
|---|---|
| 关节模组 | CyberBeast 系列（电机 + 驱动器一体，带编码器） |
| CAN 适配器 | ① **MCS CyberBeast USB2CAN**（开发/调试：插上即 `/dev/ttyACM0`，slcan 模式）② SocketCAN 兼容（显示为 `can0`，**生产推荐**）③ PEAK PCAN-USB |
| 电源 | 按模组铭牌。板上电压可以读回核对（端点 `vbus_voltage`，我们实测的一台是 23 V 左右） |
| 线缆与终端电阻 | 总线**两端**各 120 Ω；走线尽量短、双绞；适配器与模组共地 |
| （可以没有） | 仓库自带**虚拟总线**后端：CI 与演示全程用它，**零硬件** |

### 1.2 软件，按你的操作系统

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

支持的发行版：**Humble (22.04) / Jazzy (24.04) / Lyrical (26.04，主目标)**，三个都实测过（§4）。

#### 实时内核（可选；要跑 1 kHz 控制建议装）

```bash
# ① 装 PREEMPT_RT 内核：按 Ubuntu 的实时内核文档来（不同版本包名不一样，
#    常见的有 linux-realtime / linux-image-*-realtime）
# ② 给**当前用户**放开 RT 权限（下面两行就是我们在测试机上实际用的内容；
#    写错成不存在的组名会静默无效 —— 配完用 §4/§6.2 的办法验证）
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
`JRSDK_SOURCE_DIR` 或 `find_package(jsdk_can)`，见 [`cmake/jr_sdk.cmake`](cmake/jr_sdk.cmake)）。

---

## 2. 五分钟上手（不需要硬件）

### 路线甲：容器里一条命令（第一次最省事，需要有 WSL2 + Docker）

```bash
cd ~/JointROS
bash docker/run.sh jazzy        # 也可 humble / lyrical
```

**应该看到**：容器内 `ctest` 全通过、`colcon test` **0 failures**、`joint_trajectory_controller` 端到端 **PASS**（`mit` 与 `csp` 两种模式）。
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

另开一个终端（**建议把下面几条都试一遍**，这是最直接的“它真的在工作”的证据）：

```bash
source ~/JointROS/install/setup.bash
ros2 run jr_ros2 jr_ctl --node vbusrp status
ros2 run jr_ros2 jr_ctl --node vbusrp ep-list --filter axis0. --max 3
ros2 topic echo /vbusrp/joint_states --once
```

**应该看到**（真实输出长这样）：

```
snapshot at tick 28120
  bus=vbusrp link_up=1 nodes_online=2 degraded=1
  tx=30 rx=22678 tx_failed=0 rx_dropped=0 keepalive=0 link_errors=0
  ...
```

`nodes_online=2` = 两个虚拟关节都在应答。`ep-list` 会列出设备端点（`id / 类型 / 读写权限`），
`topic echo` 会打出关节名与位置。想让它真的“动一下”（虚拟关节，安全）：

```bash
ros2 launch jr_bringup vbus_demo.launch.py jog:=true     # 起来后自动使能 + 点动一次
ros2 launch jr_bringup vbus_demo.launch.py joints:=1     # 只 1 个关节
```

> ⚠ **两个最常见的新手坑**
> 1. **不要在仓库根目录直接 `colcon build`。** 根目录的 `CMakeLists.txt` 是"纯 CMake 开发入口"，
>    colcon 会把它当成一个 `cmake` 包**并不再深入子目录** ⇒ 真正的 `jr_*` 包不会被编译，
>    但测试却可能照样跑（看着像成功）。请用 `--base-paths` 指定包目录，
>    或把 `jr_*` 包拷进你自己的 ROS 工作区 `src/` 再 `colcon build`。
> 2. **`jr_bus` 是生命周期节点，没有 autostart。** 只 `ros2 run` 的话进程"活着"，
>    但话题/服务/诊断**一个都不建**（都在 `activate` 那一步创建），客户端只会看到"服务不可达"。
>    `ros2 launch jr_bringup ...` 已经替你走了这两步；手写 launch 时别忘了（§3.4）。

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

串口权限不够时，把用户加进 `dialout` 组：

```bash
sudo usermod -aG dialout $USER      # 之后重新登录
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

**应该看到**（我们在真机上跑这条命令的**真实输出**）：

```
[info] probe_max=16 -> discovered 1 node(s)
[info] node 1: fw=0x00000609 hw=0x00040237 serial=0x0 classic=yes
[info]   gear=7.7500 pos=12.5000 vel=65.0000 trq=50.0000 kp_max=500.0 kd_max=5.0 hb=100 ms break_timeout=0 ms calibrated=yes
jr_gen_config: 已写出 /home/you/jr_hw/axis1.yaml（1463 字节，自检通过：load+validate+plan_bus）
  ⚠ 生成后请复核：master_id / is_fd / 关节名（见文件头注释）
```

文件头部还会把“设备读不回来、要你人工核对”的三项点名（`master_id`、`is_fd`、以及“增益故意不写”）。

### 3.3 只读体检（`jr_hw_verify`）——**上电前最后一道闸**

```bash
ros2 run jr_ros2 jr_hw_verify --config ~/jr_hw/axis1.yaml
```

**只读**：不使能、不发控制帧。它逐层给结论（配置 → 打开总线（含锁与 ABI 自检）→ 设备发现 →
设备身份 / 量程 / 标定 → 心跳与看门狗 → 一致性 + 总线预算）。

**应该看到**（我们在一台真机上的典型结果）：**9 通过 / 2 告警 / 0 失败**。
告警通常是"设备侧看门狗未武装（`break_timeout=0`）"这类**只告诉你、不会替你改设备**的提示 —— 这是故意的。

### 3.4 起节点（生命周期四步）

```bash
ros2 run jr_ros2 jr_bus --ros-args -r __node:=axis1 \
    -p config_file:=$HOME/jr_hw/axis1.yaml -p bus:=axis1 &
ros2 lifecycle set /axis1 configure      # 读配置 + 打开总线（锁、ABI、设备发现）
ros2 lifecycle set /axis1 activate       # ← 话题 / 服务 / 诊断都在这一步才创建
ros2 run jr_ros2 jr_ctl --node axis1 status
```

> ⚠ 从这一刻起，**这条总线上不要再开第二个主站**（`jsdk-cli`、Python 绑定、第二个 `jr_bus` 都算）。
> 节点自己会拿文件锁拦住第二个进程（锁文件在 `/var/lock/jr-<通道>.lock`，例如
> `jr-slcan__dev_ttyACM0.lock`），失败时会告诉你**是哪个进程**占着。

### 3.5 读参数核对（都用 `jr_ctl`，它**只走服务、不发 CAN 帧**，所以可以边跑边用）

```bash
J="ros2 run jr_ros2 jr_ctl --node axis1"
$J ep-list --filter axis0. --max 20                 # 先看设备有哪些端点（路径前缀来自设备描述符）
$J read --paths axis0.config.can.node_id,axis0.motor.config.gear_ratio
$J info                                             # 设备身份：fw / hw / classic 位
```

**应该看到**：`read` 每行是 `关节名 路径 声明类型 值`（值就是你写进去/设备里真实的那个）。

### 3.6 小心地让它动一下（**第一次请小幅、短时**）

```bash
$J enable --joints j1 --confirm
$J jog --joint j1 --pos 0.02 --kp 2 --kd 0.2 --duration-s 0.3 --confirm   # 小幅点到 0.02 rad 后自己停并失能
$J disable --joints j1 --confirm
```

> ⚠ **`jog` 不给增益会被直接拒绝**（`kp=kd=torque=0` 是"零力矩"目标：电机不会动，
> 但整套流程会"成功"—— 我们宁可拒掉也不报一个没发生过的动作）。请显式给 `--kp/--kd`，
> 或者给 `--tau` 前馈。设备侧能接受的增益上限可以用 `$J read --paths axis0.controller.config.pos_gain`
> 之类端点看个大概（先用 `$J ep-list --filter axis0.` 找出你设备上真正的路径）。

参考：我们在真机上跑这条路径的结果是 `jog finished on 'j1': 300 ms / 272 ticks. Joint is disabled again (safe state).`

---

## 4. 怎么自测（三层，从快到慢）

| 层 | 命令 | 应该看到 | 需要什么 |
|---|---|---|---|
| ① 离线核心（最快） | `tools/build_dev.sh`（或 `ctest --test-dir build`） | `100% tests passed out of 11` | 只有编译器 + CMake（**Windows 也行**） |
| ② 容器三发行版 | `bash docker/run.sh jazzy\|humble\|lyrical` | 三条全绿（`ctest` / `colcon test` / JTC 端到端） | WSL2 + Docker |
| ③ 真机 | `jr_hw_verify --config ...` → 起节点 → `colcon test --base-paths jr_ros2` | 体检 0 失败；`colcon test` 全过 | 一台 Ubuntu + 真实模组 |

> 具体数字会随版本变（当前版本与逐项证据在 [`docs/DESIGN.zh-CN.md`](docs/DESIGN.zh-CN.md) 的 §13.2 与 §14），
> 建议记住判据而不是数字：**`tests passed` / `0 failures` / 体检 0 失败**。

---

## 5. 集成到你的系统（三条路线）

### 路线 A：用现成控制器（最省事，推荐先走这条）

让 `ros2_control` 的**标准控制器**（`joint_state_broadcaster` + `joint_trajectory_controller`，
向上还能接 MoveIt）来驱动关节。你只需要两样东西：**一份配置 YAML**（与节点、工具共用同一份，
由 §3.2 的 `jr_gen_config` 生成）和 **URDF 里的一段 `ros2_control` 声明**。

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

三条硬约束（写错的症状都很像"硬件起不来"）：

1. **`<command_interface>` 必须与实际导出的接口集一致**。导出集由配置里 `joints[].mode` 决定：
   `mit` → `position/velocity/effort + kp/kd`；`csp` → 只有 `position`；`csv` → 只有 `velocity`；`cst` → 只有 `effort`。
2. **关节名逐字一致**（YAML ↔ URDF ↔ 控制器参数），不一致会在 `on_init` 就失败并列出可用名字。
3. **`gain_mode` 二选一**：`wire` 导出 `kp/kd`（与 MIT 帧字段一致的线上值），`si` 导出
   `stiffness/damping`（N·m/rad、N·m·s/rad）。两组**不会同时导出** —— 单位搞错比 claim 失败危险得多。

仓库自带一个可跑的端到端示例（起 `controller_manager` + JTC + 发一条真实轨迹 + 断言）：

```bash
JR_DEMO_INSTALL=<colcon install 目录> bash jr_ros2_control/test/jtc_demo/run.sh --mode mit
JR_DEMO_INSTALL=<colcon install 目录> bash jr_ros2_control/test/jtc_demo/run.sh --mode csp
```

URDF 怎么写、怎么写错（症状对照表）见 [`jr_bringup/docs/URDF.zh-CN.md`](jr_bringup/docs/URDF.zh-CN.md)。

### 路线 B：自己写一个 ROS 2 节点（只需要 `jr_interfaces`）

节点（`jr_bus`）已经把"这条总线"整成一个 ROS 2 部件。你订阅它的话题、给它发命令、或者调它的服务即可 ——
**你的代码可以只依赖 `jr_interfaces`**（消息 / 服务定义），不需要链接任何 C++ 库。

命名空间：节点名叫什么，话题就在它下面（`~/joint_states` ⇒ `/vbusrp/joint_states`）。

**它发布（你订阅）**

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/joint_states` | `sensor_msgs/JointState` | 标准关节状态（位置/速度/力矩），默认 100 Hz |
| `~/joint_feedback` | `jr_interfaces/JointFeedbackArray` | 更全的原始反馈（温度/电压/错误位/是否陈旧…），默认 500 Hz |
| `~/bus_status` | `jr_interfaces/BusStatus` | 总线级统计（帧数/丢帧/负载/链路） |
| `~/rt_stats` | `jr_interfaces/RtStats` | 实时性能（抖动、丢拍、命令延迟） |
| `~/faults` | `jr_interfaces/JointFault` | 故障事件（出现/清除） |

**它订阅（你发布）**

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/cmd` | `jr_interfaces/JointCommandArray` | **按配置里的模式**下发的命令（`mode` 字段：CSP=8 / CSV=9 / CST=10 / CURRENT=11，单位随模式） |
| `~/cmd_mit` | `jr_interfaces/MitCommandArray` | MIT 语义命令：`position/velocity/kp/kd/torque`（`gain_mode` 选 `WIRE` 或 `SI`） |
| `~/estop` | `std_msgs/Bool` | 本条总线急停（`true`） |
| `/jr/estop_all` | `std_msgs/Bool` | **所有**总线节点都订阅（全局急停） |

消息字段可以用一句话看全：

```bash
ros2 interface show jr_interfaces/msg/MitCommand
ros2 interface show jr_interfaces/msg/JointCommand
```

**它提供的服务**（19 个，名称是 srv 类型的 snake_case，例如 `GetBusStats` → `~/get_bus_stats`）。
日常运维**不必记服务名** —— 用 `jr_ctl`（§3.5），它对常用服务都有子命令、退出码也分了类
（`0` 成功 / `1` 操作失败 / `2` 用法错 / `4` 服务不可达）。

> ⚠ 命令有**超时保护**：`command.timeout_ms`（默认 100 ms）内没收到新命令，节点会自动走安全动作
> （`command.on_timeout`）。所以在自己的控制器里请**持续发**，不要"发一次就不管了"。

### 路线 C：不用 ROS

| 你想要 | 用什么 | 说明 |
|---|---|---|
| C++，无 ROS，自己做调度 | 直接链本仓库的 **`jr_core`**（`jr_ros2` 里的静态库，**不依赖 rclcpp**） | 享受同一套实时核心、安全策略与配置校验；`jr_ros2/include/jr_ros2/` 下的头文件就是接口 |
| C / C++ / Python，只想要协议与设备 | 直接用 **JointSDK** | 那是最底层；本仓库的价值（ROS 接口、诊断、工具、纪律）就没有了 |

> 这条路线我们不提供现成示例（客户形态差异太大）。需要时请对着 `jr_core` 的头文件与
> `docs/DESIGN.zh-CN.md` 的 §5/§6 来设计；也可以只借 `jr_core` 的**配置校验** + **总线预算**两件小事。

---

## 6. 现场纪律与排障表

### 6.1 三条硬纪律

1. **同一条 CAN 总线只允许一个主站进程。** 节点运行期间不要再跑 `jsdk-cli` / Python 绑定。
   设备只记得"最后见到的那个主站"，症状是心跳乱跳、参数偶发写不进。运维请用 `jr_ctl`（走服务）。
   我们的文件锁能拦住第二个 `jr_bus`，但**拦不住 `jsdk-cli`**（SDK 侧不加锁）—— 那半边靠纪律。
2. **生产用 CAN FD，且每条总线 ≤ 7 个关节**（广播同步寻址的上限）。Classic / slcan 只用于上电检查与调试；
   启动时的总线预算检查会直接拒绝不可行的组合。
3. **冷启动不自动使能**（`safety.auto_enable` 默认 `false`），节点退出默认走**安全失能**序列，
   不会留下"抱力"的关节。`estop` 是**整条总线的广播**，而且**锁存**后需要 `reset`（软复位）或断电才能清。

### 6.2 排障表（症状 → 先查什么 → 怎么看）

| 症状 | 最可能的原因 | 怎么办 |
|---|---|---|
| `jr_ctl` 一直报"服务不可达"（rc=4） | 节点没起来 / 没 `configure`+`activate` / 节点名写错 | `ros2 node list`；`ros2 lifecycle get /<节点名>` 必须是 `active` |
| **收得到心跳，但我的请求没人应** | **帧格式不对**（设备是 Classic 而你按 CAN FD 发） | 先看 `jr_ctl info` 的 `classic` 位；配置里 `is_fd` 默认按 Classic 起步。⚠ **别拿 `is_fd` 做实验**：用错格式打开 slcan 是**破坏性**的，可能要拔插适配器才能恢复 |
| 偶发读不到（约 1/10 次） | slcan 的**首帧丢失**（固有特性） | 重试一次即可；高频/生产请换 SocketCAN 或 CAN FD |
| 第二个进程起不来 | 单主站锁 | 报错里会写**谁**占着（PID）与锁文件路径；`lock.allow_shared=true` 只用于调试 |
| `estop` 之后关节怎么都不动 | 急停**锁存**了 | `jr_ctl fault-reset --confirm`，不行就 `jr_ctl reset --confirm`（软复位，之后要重新 configure）或断电重启 |
| 命令发出去了，关节不动 | ① `kp/kd/tau` 全 0（会被拒）② 没使能 ③ 命令模式与配置的 `joints[].mode` 不符（会被丢弃并计数） | `jr_ctl status` 看 `cmd_rejected`；用 `jr_ctl enable --joints ... --confirm`；模式见配置 |
| 反馈值看着不对 / 不动 | 反馈是**设备主动上报的帧**，在某些固件上是陈旧值 | 看 `~/joint_feedback` 里的 `feedback_stale` 标志；**要真值请直接读端点**：`jr_ctl read --paths axis0.encoder.pos_estimate,axis0.encoder.vel_estimate` |
| 实时性不达标（抖动大 / 丢拍多） | 没有 RT 权限，或被内核限流 | 按 §1.2 配 RT；`jr_ctl status` 里的 `tick_overruns`、`rt_throttled` 是判据 |
| `calib` / `home` 失败 | 虚拟后端不实现这两个状态机；真机上未标定/有故障时也会被设备拒 | 先 `jr_hw_verify` 看标定标志；真机路径见 DESIGN §13.4 的待办 |

---

## 7. 已知限制与边界（如实登记）

- **命令模式**：`mit` / `csp` / `csv` / `cst` 可用；**`CURRENT` 明确拒绝**（`effort` 的单位会从 N·m 变成电机端 A，差 gear×kt，容易出事故）。
- **每条总线 ≤ 7 关节**（广播同步寻址上限）；超过会被预算检查拒绝。
- **反馈帧**（`~/joint_feedback` 的 pos/vel）在某些固件上可能**陈旧**：我们会如实打
  `FEEDBACK_STALE`，`age_ms` 也不会假报"刚刚更新"；但**把反馈源改成端点轮询还没做**（见 DESIGN §13.4）。
- **`calib` / `home` 的真机成功路径尚未验证**（虚拟设备不实现这两个状态机；真机上被急停锁存挡住过）。
- `jr_latency_bench`（实时性能基准工具）**未实现**（P1，需要真机才有意义）。
- `desc-import --persist`（把描述符写进设备 Flash）**不支持**（SDK 没有这个能力，会被明确拒绝，不会静默忽略）。
- **Windows / macOS** 不支持 ROS 侧（节点、`ros2_control`、`jr_ctl`）。
- 急停可以**锁存**；生产上请把"清锁存"写进你的操作规程（`fault-reset` → 必要时 `reset` / 断电）。

---

## 8. 目录、文档、版本、许可

```
docs/DESIGN.zh-CN.md        设计与验收的**唯一权威**（架构 / 工作包 / 证据 / 变更记录）
docker/README.md            容器环境（三发行版矩阵）的一次性准备与用法
jr_bringup/README.md        示例与 launch（零硬件跑通整条链路）
jr_bringup/docs/URDF.zh-CN.md  URDF 片段怎么写、怎么写错（症状对照表）
cmake/jr_sdk.cmake          JointSDK 的三种定位方式
jr_interfaces/              消息 / 服务定义（自研控制器可以只依赖它）
jr_ros2/                    实时核心库 jr_core + 驱动节点 jr_bus + 工具（jr_ctl / jr_hw_verify / jr_gen_config / jr_bus_plan）
jr_ros2_control/            ros2_control 硬件组件（JTC / MoveIt 走这条）
jr_bringup/                 launch / 配置模板 / URDF 示例 / 演示
tools/build_dev.sh          无 ROS 的"配置 + 构建 + 测试"一条命令
```

- **当前版本**：v0.18（逐版本变更见 DESIGN §14；已验证环境与逐项证据见 §13.2，含真机）。
- **许可**：**MIT**，见 [`LICENSE`](LICENSE)。
