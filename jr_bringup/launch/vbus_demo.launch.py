"""虚拟总线 demo：一条命令把 `jr_bus` 起起来并走完 lifecycle（configure → activate）。

用法：
    ros2 launch jr_bringup vbus_demo.launch.py                 # 2 关节（默认）
    ros2 launch jr_bringup vbus_demo.launch.py joints:=1       # 单关节
    ros2 launch jr_bringup vbus_demo.launch.py jog:=true       # 起来后自动使能 + 点动一次

⚠⚠ `jr_bus` 是**生命周期节点且没有 autostart**：只 `ros2 run` 的话进程“活着”，
   但话题/服务/诊断一个都不建（它们都在 `on_activate` 里创建）—— 客户端只会看到
   “服务不可达”（DESIGN §13.3-36）。这个 launch 就是替你走那两步。

⚠ **节点名/总线名从配置里读**（`jr.buses[0].name`），不提供 `bus:=` 覆盖：
   `jr_bus` 会核对请求的总线名与配置是否一致（防止多总线串号），手工改名只会撞拒绝。
   要换名字就改配置 —— 名字属于配置，不属于命令行。

跑起来之后（另开一个终端）：
    ros2 node list
    ros2 lifecycle get /vbusrp
    ros2 run jr_ros2 jr_ctl --node vbusrp status
    ros2 topic echo /vbusrp/joint_states --once

⚠ 本文件只用 launch / launch_ros 的**公开且三发行版都有**的 API（Humble/Jazzy/Lyrical）。
"""
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.events import matches_action
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

TRANSITION_CONFIGURE = Transition.TRANSITION_CONFIGURE
TRANSITION_ACTIVATE = Transition.TRANSITION_ACTIVATE

# ⚠⚠ 匹配器只认 `launch.events.matches_action`（比较 `action == 目标对象` 的同一性）。
#   别手写“按名字比”的匹配器：`Node.node_name` 在执行前会**抛 RuntimeError**
#   （"cannot access 'node_name' before executing action"），而 `getattr` 只吞
#   AttributeError —— 异常会从事件处理链里冒出去，**CONFIGURE 就这么静默地没发出去**
#   （现象：`ros2 lifecycle get /vbusrp` 永远是 unconfigured，日志里毫无线索）。
#   注：日志串里不放 `✅` 之类的符号 —— 项目规矩是**运行期输出只用 ASCII/中文**
#   （cp936 控制台显示不了就成了 `?`，排障时反而更难读）。


def _buses_in(config_path):
    with open(config_path, encoding='utf-8') as fh:
        doc = yaml.safe_load(fh)
    return [b['name'] for b in doc['jr']['buses']]


def _jog_actions(bus, node):
    """（可选）起来之后使能 + 点动一次：把“能真的动”这件事也演示到。"""
    ctl = ['ros2', 'run', 'jr_ros2', 'jr_ctl', '--node', bus]
    return [
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state='activating',
                goal_state='active',
                entities=[
                    LogInfo(msg='[demo] 使能并点动 j1 一次（jog 会让关节真的动；虚拟设备上安全）'),
                    TimerAction(period=1.0, actions=[
                        ExecuteProcess(cmd=ctl + ['enable'], output='screen'),
                        TimerAction(period=3.0, actions=[
                            ExecuteProcess(cmd=ctl + ['jog', '--joint', 'j1', '--pos', '0.05',
                                                     '--duration-s', '0.5', '--confirm'],
                                           output='screen'),
                        ]),
                    ]),
                ],
            )
        )
    ]


def _setup(context, *args, **kwargs):
    joints = LaunchConfiguration('joints').perform(context).strip()
    jog = LaunchConfiguration('jog').perform(context).strip().lower() in ('1', 'true', 'yes', 'on')
    cfg = '%s/config/vbus_%sjoint.yaml' % (get_package_share_directory('jr_bringup'), joints)
    buses = _buses_in(cfg)
    assert len(buses) == 1, 'vbus_%sjoint.yaml 应当只定义一条总线，实得 %s' % (joints, buses)
    bus = buses[0]

    node = LifecycleNode(
        package='jr_ros2',
        executable='jr_bus',
        name=bus,
        namespace='',
        parameters=[{'config_file': cfg, 'bus': bus}],
        output='screen',
    )

    actions = [
        LogInfo(msg='[demo] 虚拟总线 /%s（%s 个关节）：起节点 → configure → activate' % (bus, joints)),
        node,
        # ⚠ 用 TimerAction 而不是立刻 EmitEvent：节点还没起来时发的事件会被丢掉。
        TimerAction(period=2.0, actions=[
            EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                        transition_id=TRANSITION_CONFIGURE)),
        ]),
        # configure 成功（inactive）→ 立刻 activate；服务/话题/诊断在 activate 之后才存在。
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state='configuring',
                goal_state='inactive',
                entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                                      transition_id=TRANSITION_ACTIVATE))],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state='activating',
                goal_state='active',
                entities=[
                    LogInfo(msg='[demo] /%s 已 active。下一步：'
                                'ros2 run jr_ros2 jr_ctl --node %s status' % (bus, bus)),
                ],
            )
        ),
    ]
    if jog:
        actions += _jog_actions(bus, node)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joints', default_value='2', choices=['1', '2'],
                              description='虚拟关节数（决定用 config/vbus_Njoint.yaml）'),
        DeclareLaunchArgument('jog', default_value='false',
                              description='true = 起来后自动使能并点动一次（会让关节真的动）'),
        OpaqueFunction(function=_setup),
    ])
