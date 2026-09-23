"""人形示例：**两条总线**（每条一个 `jr_bus` 进程），并发布一份示例 URDF。

    ros2 launch jr_bringup humanoid_2bus.launch.py

起什么：
  1. 两个 `jr_bus`（默认 `/leg_left`、`/leg_right`）—— **一个节点 = 一条总线**（设计约束）；
     两条总线的定义在**同一份** `config/humanoid_2bus.yaml` 里（机器人级配置只有一份，不会漂移）。
  2. `robot_state_publisher`（`urdf/humanoid_2bus.urdf`）—— 让 rviz2 直接能看：
        ros2 launch jr_bringup humanoid_2bus.launch.py
        rviz2            # Fixed Frame = base_link，加 RobotModel + TF
     ⚠ URDF 只是**示例**；真机上的关节名/限位要从 `jr_gen_config` 生成的配置与
       `jr_hw_verify` 的读回值来（见 `docs/URDF.zh-CN.md`）。

⚠ 单 master 纪律：一条 CAN 总线只允许一个主站进程 ⇒ 多总线 = 多进程。
⚠ 广播（`estop` 等）是**总线内**语义：两条总线要各发一次，别指望一个节点管全机（§8.4）。
⚠ 节点名/总线名从配置里读（`jr.buses[].name`），不提供 `bus:=` 覆盖（避免与配置不一致被服务拒绝）。
"""
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

TRANSITION_CONFIGURE = Transition.TRANSITION_CONFIGURE
TRANSITION_ACTIVATE = Transition.TRANSITION_ACTIVATE

# ⚠⚠ 匹配器只能用 `matches_action`（同一性比较）。手写“按名字比”会踩
#   `Node.node_name` 在执行前抛 RuntimeError 的坑，且 `getattr(..., None)` 只吞
#   AttributeError —— 异常让 ChangeState 根本发不出去，节点永远停在 unconfigured。


def _buses_in(config_path):
    with open(config_path, encoding='utf-8') as fh:
        doc = yaml.safe_load(fh)
    return [b['name'] for b in doc['jr']['buses']]


def _one_bus(bus, cfg):
    """一条总线的 lifecycle 编排：configure → activate。"""
    node = LifecycleNode(
        package='jr_ros2',
        executable='jr_bus',
        name=bus,
        namespace='',
        parameters=[{'config_file': cfg, 'bus': bus}],
        output='screen',
    )
    return [
        node,
        TimerAction(period=2.0, actions=[
            EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                        transition_id=TRANSITION_CONFIGURE)),
        ]),
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
                entities=[LogInfo(msg='[demo] /%s 已 active' % bus)],
            )
        ),
    ]


def _setup(context, *args, **kwargs):
    share = get_package_share_directory('jr_bringup')
    cfg = '%s/config/humanoid_2bus.yaml' % share
    urdf = '%s/urdf/humanoid_2bus.urdf' % share

    defined = _buses_in(cfg)
    want = [b.strip() for b in LaunchConfiguration('buses').perform(context).split(',') if b.strip()]
    unknown = [b for b in want if b not in defined]
    assert not unknown, '配置里没有这些总线：%s（已定义：%s）' % (unknown, defined)

    actions = [
        LogInfo(msg='[demo] 人形双总线示例：%s（每条总线一个 jr_bus 进程）' % want),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher',
             parameters=[{'robot_description': open(urdf, encoding='utf-8').read()}],
             output='screen'),
    ]
    for bus in want:
        actions += _one_bus(bus, cfg)
    actions.append(LogInfo(
        msg='[demo] 试试： ros2 run jr_ros2 jr_ctl --node %s status（另一条同理）' % want[0]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('buses', default_value='leg_left,leg_right',
                              description='要起的总线名（逗号分隔，必须都在 config 里定义）'),
        OpaqueFunction(function=_setup),
    ])
