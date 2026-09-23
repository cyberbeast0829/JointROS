# jr_bringup —— 示例与启动文件（WP7）

**不需要任何硬件**就能跑通整条链路（虚拟总线后端）。

## 一条命令跑起来

```bash
ros2 launch jr_bringup vbus_demo.launch.py              # 虚拟总线，2 关节
ros2 launch jr_bringup vbus_demo.launch.py joints:=1    # 单关节
ros2 launch jr_bringup vbus_demo.launch.py jog:=true    # 起来后自动使能 + 点动一次
ros2 launch jr_bringup humanoid_2bus.launch.py          # 人形：两条总线（两个 jr_bus）+ 示例 URDF
```

它会替你走完 `jr_bus` 的 lifecycle 两步 —— **这一点很重要**：

> `jr_bus` 是**生命周期节点且没有 autostart**。只 `ros2 run jr_ros2 jr_bus` 的话，
> 进程"活着"，但话题/服务/诊断一个都不建（都在 `on_activate` 里创建），
> 客户端只会看到"服务不可达"（DESIGN §13.3-36）。

跑起来之后（另开一个终端）：

```bash
ros2 node list                                  # /vbusrp
ros2 lifecycle get /vbusrp                      # active
ros2 run jr_ros2 jr_ctl --node vbusrp status    # 运维入口（走服务，不动 CAN）
ros2 topic echo /vbusrp/joint_states --once
```

## 目录

| 路径 | 说明 |
|---|---|
| `launch/vbus_demo.launch.py` | 单条虚拟总线 + lifecycle 编排（可选点动） |
| `launch/humanoid_2bus.launch.py` | 双总线示例（**一个节点 = 一条总线**）+ `robot_state_publisher` |
| `config/vbus_1joint.yaml` / `vbus_2joint.yaml` | 虚拟总线配置（§6.4 schema） |
| `config/humanoid_2bus.yaml` | 两条总线的机器人级配置（一份，不漂移） |
| `urdf/humanoid_2bus.urdf` | 示例 URDF（rviz2 可直接看） |
| `docs/URDF.zh-CN.md` | **URDF 片段怎么写、怎么写错**（三条硬约束 + 症状对照表） |
| `test/test_launch_smoke.sh` | CI 冒烟：示例必须**真能起来**（ctest `launch_smoke`） |

## 约定（与配置有关的两件事）

1. **节点名/总线名从配置里读**（`jr.buses[0].name`），launch 不提供 `bus:=` 覆盖：
   `jr_bus` 会核对请求的总线名与配置是否一致（防多总线串号），手工改名只会撞拒绝。
   要换名字就改配置 —— **名字属于配置，不属于命令行**。
2. **一条 CAN 总线只允许一个主站进程** ⇒ 多总线 = 多进程（`humanoid_2bus` 起两个 `jr_bus`）。
   广播型命令（`estop` 等）是**总线内**语义，两条总线要各发一次（§8.4）。
3. ⚠⚠ **一条总线一个 tick 组**。`jr_bus` 打开的单位是 **tick 组**，不是 `bus` 参数选中的那一条：
   把两条总线放进同一个组，两个进程都会去开这两条 → 第二个直接死在总线锁上
   （`bus_lock: '/var/lock/jr-leg_left.lock' is held by pid …`），而第一个进程的日志里会写着
   `tick_group 'g0': 2 bus(es)` —— 那就是信号。`humanoid_2bus.yaml` 的注释里留了这个反例。
