# JointROS —— CyberBeast 关节模组的 ROS 2 包

> 状态：**WP0/WP1 已落地并验证**（`jr_core` 实时核心库 + 运维面 + 虚拟总线端到端测试），
> **WP3（`ros2_control`）已落地**：`jr_ros2_control` 在 **Lyrical（主目标）/ Jazzy / Humble** 三个发行版上都
> 编译通过并跑通组件测试与 **JTC 端到端（`mit` + `csp` 两种模式）**；命令接口按
> `joints[].mode` 导出（CSP/CSV/CST 可用；`CURRENT` 显式拒绝并说明原因）。
> **WP2 已完整落地**：`jr_interfaces`（16 msg + 19 srv）与 `jr_bus` 驱动节点
> （生命周期/命令/状态发布/单 master 锁/描述符缓存/**19 个服务**/**§6.5 诊断**/`~/cmd` 模式化命令）。
> **WP4 工具已落地 4/5 并收尾**：`jr_hw_verify`（上机前自检，**只读**，逐层给结论）、
> `jr_gen_config`（扫总线 → 生成配置 → **当场回读自检**，不过就不落盘）、
> `jr_bus_plan`（离线算总线负载/帧率/可行性）、
> `jr_ctl`（对标 `jsdk-cli` 的子命令，但**只走服务** ⇒ 可以在节点运行时用）。
> §11 WP4 的三条验收口径逐条有证据（工具测试 / 虚拟总线可进 CI / **生成配置→节点 configure+activate**）；
> 只剩 `jr_latency_bench`（**P1**：实时性能要有真机才有意义）。
> 容器内统一口径：`ctest` **12/12**、`colcon test` **21 tests / 0 failures / 0 告警**
> （Windows 无 ROS：10 测试/529 断言）。设计与工作包见
> [`docs/DESIGN.zh-CN.md`](docs/DESIGN.zh-CN.md)。

本仓库提供基于 [JointSDK](../JointSDK)（CYBERBEAST 协议 / CAN & CAN-FD 后端）的 ROS 2 集成层。
协议、帧格式、端点解析、设备状态机**全部**由 JointSDK 负责；本仓库只负责：
**实时调度、ROS 接口、安全策略、诊断与运维**。

## 目标客户

人形机器人 / 机器狗 / 外骨骼 / 工业机械臂的整机厂家。

## 命名约定

所有包/话题/诊断 id 统一用 **`jr_`** 前缀（`j` = joint，`r` = ros2，与 JointSDK 的 `jsdk_` 同风格）：

| 包 | 作用 | 状态 |
|---|---|---|
| `jr_interfaces` | 消息与服务定义（自研控制器可只依赖它） | **已实现**（WP2）：16 msg + 19 srv |
| `jr_ros2` | **实时核心库**（目标 `jr_core`，无 rclcpp 依赖）+ 生命周期驱动节点（`jr_bus`）+ 工具（`jr_ctl` / `jr_hw_verify` / `jr_gen_config` / `jr_bus_plan` / `jr_latency_bench`） | **`jr_core` 已实现并测试通过**；**`jr_bus` 节点已完整落地**（话题/19 服务/诊断/`~/cmd`）；**工具已落地 4/5**（`jr_hw_verify` / `jr_gen_config` / `jr_bus_plan` 非 ROS，`jr_ctl` 走服务）；`jr_latency_bench` 待实现（P1，要有真机） |
| `jr_ros2_control` | `ros2_control`（`hardware_interface`）硬件组件 | **已实现并测试通过**（WP3）：三发行版 `colcon test` **22 tests / 0 failures**（工作区总数，含 `jr_bringup` 的 `launch_smoke`） + JTC 端到端 PASS（mit + csp） ；**命令接口按 `joints[].mode` 导出**（CSP/CSV/CST 可用，`CURRENT` 拒绝） |
| `jr_bringup` | launch / 参数模板 / URDF 示例 / 演示应用 | **已实现（WP7）**：`vbus_demo.launch.py`（虚拟总线 + 自动 configure/activate，可选点动）、`humanoid_2bus.launch.py`（双总线 + 示例 URDF）、`docs/URDF.zh-CN.md`；ctest `launch_smoke` 真跑示例 |

## 目标环境

| 项 | 值 |
|---|---|
| ROS 2 | **Lyrical（主目标）**，兼容 **Jazzy** 与 **Humble** |
| 操作系统 | Ubuntu + **PREEMPT_RT**（生产）；WSL2（开发，不保证实时）；Windows 仅用于开发 |
| 硬件后端 | Linux SocketCAN（生产）、PEAK PCAN、slcan（仅调试）、虚拟总线（CI / 演示，无需硬件） |

## 构建与测试

### 不需要 ROS：只验证实时核心（本机可跑）

```bash
tools/build_dev.sh                       # = configure + build + ctest
# 或手动：
cmake -S . -B build -G "MinGW Makefiles" -DJRSDK_SOURCE_DIR=/path/to/JointSDK   # Linux 用默认生成器
cmake --build build -j
ctest --test-dir build --output-on-failure
```

当前覆盖（**10 个测试 / 529 项断言**，包含 DESIGN §10.2 的"证伪"用例）：
无锁三缓冲与 SPSC 环（含并发压测与**撕裂读检测**）、参数编解码（超值域必拒）、
总线预算模型、配置校验、SDK ABI 自检（含**检查器自身的变异测试**）、
**虚拟总线端到端**：未使能时零控制帧 / 安全暂停协议 / 广播 1 帧驱动多关节 /
8 关节 @1 kHz 必须拒绝启动 / 单 master 锁 / 运维面（`confirm` 闸门、写后读回、
路径不存在即 `kNotFound`、类型不符拒绝、未标定仍可标定）、
**模式化命令**（CSP 收敛到目标 / CSV 速度量值 / CST 力矩体现在反馈电流且符号正确 /
CURRENT 验**发出去的帧**，因为设备对该帧不回响应（SDK F19）；模式不符的目标必须被丢弃并计数），
以及**脏堆回归**（上下文存储必须由我们清零）。

> 测试本身也做了防"假红"处理：不用"睡固定毫秒再断言进度"（主机被抢占时会误报），
> 一律**等到够为止**；确实只能特定前提下成立的检查会打 `[ENV]` 跳过（单独计数）；
> 每次摘要还会打一行 `[HOST]`（调度响应性实测），让人一眼看出时间敏感结论有没有资格当证据。

### Linux + ROS（容器，一条命令）

本机 Windows 编不了也测不了 ROS 侧与仅 Linux 的代码路径（`flock` / `SCHED_FIFO` /
`mlockall` / `clock_nanosleep` / SocketCAN）。用容器一次覆盖（需要 WSL2 + Docker）：

```bash
# 在 WSL 里：
cd /mnt/d/projects/cheetah/JointROS
bash docker/run.sh lyrical    # 主目标：Ubuntu 26.04 / gcc 15.2 / CMake 4.2；ctest 12/12 + colcon 20/0 + JTC PASS
bash docker/run.sh jazzy      # Ubuntu 24.04 / gcc 13.3；同上三项全绿
bash docker/run.sh humble     # Ubuntu 22.04 / gcc 11.4；同上三项全绿
```

细节（一次性准备、镜像源、网络受限时的退路）见 [`docker/README.md`](docker/README.md)。

### 装了 ROS：colcon

**把 `jr_*` 包放进你自己的 ROS 工作区 `src/`**（推荐，也是客户的常规做法）：

```bash
cd ~/my_ws && colcon build --packages-select jr_ros2 --cmake-args -DJRSDK_SOURCE_DIR=/path/to/JointSDK
```

⚠ **不要在 JointROS 仓库根目录直接 `colcon build`**：那里也有 `CMakeLists.txt`（纯 CMake 开发入口），
colcon 会把它识别成一个 `cmake` 包**并且不再深入子目录** —— 真正的 ament 包 `jr_ros2` 就不会被构建，
但测试却可能照样跑（看着像成功）。要用本仓库树直接构建，请显式指定包目录：

```bash
colcon build --base-paths jr_ros2 --cmake-args -DJRSDK_SOURCE_DIR=/path/to/JointSDK
```

`JointSDK` 的三种获取方式（`find_package(jsdk_can)` → `JRSDK_SOURCE_DIR` → `JRSDK_GIT_REPOSITORY`）
见 [`cmake/jr_sdk.cmake`](cmake/jr_sdk.cmake) 里的注释与选型说明。

### ros2_control（`jr_ros2_control`，WP3）

URDF 里给一个**配置文件路径**（与节点/工具**同一份 YAML**，见 DESIGN §6.4），其余全在配置里：

```xml
<ros2_control name="jr" type="system">
  <hardware>
    <plugin>jr_ros2_control/JrSystemInterface</plugin>
    <param name="config_file">/etc/jr/robot.yaml</param>
    <!-- 建议默认（下面两项省略时就是这两个值）： -->
    <param name="tick_source">internal</param>            <!-- internal | controller_manager -->
    <param name="gain_mode">wire</param>                  <!-- wire(kp/kd) | si(stiffness/damping) -->
  </hardware>
  <!-- 关节名必须与 YAML 里的 name 一致（不一致会在 on_init 阶段就失败并列出可用名字） -->
  <joint name="FL_hip">
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

两个 `tick_source` 的差别（ADR-6）：`internal`（默认）= 组件内部跑 RT tick，CAN 周期与
controller_manager 周期**解耦**，`read()/write()` 只做信箱交换（常量时间）；`controller_manager` =
由 CM 的 update 线程驱动（时序单一、延迟更小，但抖动取决于控制器链）。

⚠ **`gain_mode` 二选一**：`wire` 导出 `kp/kd`（与 SDK 的 MIT 帧字段一致的线上值），`si` 导出
`stiffness/damping`（N·m/rad、N·m·s/rad）。**两组不会同时导出** —— 单位搞错比 claim 失败危险得多
（详见 JointSDK 的 UNITS 文档）。

## 跑起来：`jr_bus` 是**生命周期节点**（没有 autostart）

**最快的路**（虚拟总线，不需要任何硬件 —— 示例会替你走下面的 configure/activate 两步）：

```bash
ros2 launch jr_bringup vbus_demo.launch.py      # 2 关节；joints:=1 单关节；jog:=true 还会点动一次
```

**手动方式**（真机/自己写 launch 时就是这四步）：

```bash
ros2 run jr_ros2 jr_bus --ros-args -r __node:=vbusrp -p config_file:=/etc/jr/robot.yaml &
ros2 lifecycle set /vbusrp configure     # 读配置 + 打开总线（锁、ABI、设备发现）
ros2 lifecycle set /vbusrp activate      # ← 话题/服务/诊断都在**这一步**才创建
jr_ctl --node vbusrp status              # 之后所有运维都用 jr_ctl（走服务，不动 CAN）
```

⚠ **只 `ros2 run` 而不 configure/activate** ⇒ 进程“活着”但**什么都不提供**：
`ros2 node list` 看得到它，服务却一个也没有，客户端只会看到“服务不可达”。
这是 v0.14 花时间最多的一个“看起来像工具坏了”的坑（DESIGN §13.3-36）。

## 三条必须先知道的现场纪律

1. **同一条 CAN 总线只允许一个主站进程。** 运行本节点时**不要**再跑 `jsdk-cli` 或 Python 绑定
   （设备只会记住最后见到的 `master_id`，症状是心跳乱跳、参数偶发写不进）。
   节点运行期间的诊断请用 `jr_ctl`（走 ROS 服务）。节点会用文件锁主动拦住第二个进程。
2. **生产请用 CAN FD**，且**每条总线 ≤ 7 个关节**（广播同步寻址上限）。
   Classic / slcan 只用于上电检查与调试——启动时的总线预算检查会直接拒绝不可行的组合。
3. **冷启动不自动使能**（`safety.auto_enable` 默认 `false`）；节点退出默认走安全失能序列，
   不会留下抱力关节。`estop` 是**整条总线的全局广播**，且锁存后需要 `reset` / 断电才能清。

## 目录

```
docs/                        设计与交付文档（DESIGN 是唯一权威）
cmake/jr_sdk.cmake           JointSDK 定位 + 容量宏注入（三选一）
jr_interfaces/               消息 / 服务
jr_ros2/                     RT 核心库（jr_core）+ 驱动节点（jr_bus）+ 工具
jr_ros2_control/             ros2_control 硬件组件
jr_bringup/                  launch / 配置 / 示例
tools/                       构建与自检脚本
```

## 许可

**专有许可**，与 JointSDK 一致（仅授权客户随产品分发）。详见后续 `LICENSE`。
