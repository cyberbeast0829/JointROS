#!/usr/bin/env python3
"""JTC 端到端断言（WP3 验收）。

为什么要有这个：组件单测能证明"接口、生命周期、安全落点"对，但**证明不了**
"一条 FollowJointTrajectory 真的能让关节动到目标位"。这条链路跨了
controller_manager → JTC → 我们的 SystemInterface → jr_core → JointSDK → 设备模型 → 反馈回读，
只有真跑一遍才算数。

断言的取向：**宁可粗但要真**
  - 目标被接受且结果为 SUCCESSFUL（说明整条链路没断）；
  - 关节**真的动了**（位移 > 0.1 rad）—— 防"命令下去了但关节没跟"这种最容易漏的故障；
  - 终点误差在容差内（虚拟设备是简化的 PD 模型，不追高精度，追"确实闭上了环"）；
  - 期间 `/joint_states` 一直在发（说明反馈链也活着）。
失败时把测到的数值全部打出来 —— 不要只给一句 "FAILED"。
"""

import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["j0", "j1"]
TARGET = {"j0": 0.30, "j1": -0.20}
MOVE_TIME = 1.5          # s（轨迹时长）
GOAL_TIMEOUT = 20.0      # s（等 action 结果）
CTRL_TIMEOUT = 40.0      # s（等控制器 active）
TOL = 0.20               # rad（终点容差）


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("jr_jtc_probe")
        self.samples = 0
        self.last: dict[str, float] = {}
        self.max_seen: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.list_clients = {
            name: self.create_client(ListControllers, name)
            for name in ("/controller_manager/list_controllers",)
        }

    def _on_js(self, msg: JointState) -> None:
        self.samples += 1
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                self.last[name] = pos
                self.max_seen[name] = max(self.max_seen.get(name, 0.0), abs(pos))

    def spin(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def wait_for_controllers(node: Probe, names: list[str]) -> tuple[bool, str]:
    """等这两个控制器都 active。返回 (是否成功, 说明)。"""
    cli = node.list_clients["/controller_manager/list_controllers"]
    if not cli.wait_for_service(timeout_sec=CTRL_TIMEOUT):
        return False, "/controller_manager/list_controllers 服务一直没出现"

    deadline = time.time() + CTRL_TIMEOUT
    last_state = ""
    while time.time() < deadline:
        if not cli.service_is_ready():
            node.spin(0.1)
            continue
        fut = cli.call_async(ListControllers.Request())
        while not fut.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not fut.done():
            continue
        res = fut.result()
        states = {c.name: c.state for c in res.controller}
        last_state = ", ".join(f"{k}={v}" for k, v in states.items()) or "<none>"
        if all(states.get(n) == "active" for n in names):
            return True, last_state
        node.spin(0.2)
    return False, f"控制器没能在 {CTRL_TIMEOUT:.0f}s 内全部 active（当前：{last_state}）"


def send_trajectory(node: Probe) -> tuple[bool, str]:
    """发一条轨迹并等结果。"""
    action = ActionClient(node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
    if not action.wait_for_server(timeout_sec=CTRL_TIMEOUT):
        return False, "follow_joint_trajectory action server 没出现"

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = [TARGET[j] for j in JOINTS]
    pt.velocities = [0.0] * len(JOINTS)
    pt.time_from_start.sec = int(MOVE_TIME)
    pt.time_from_start.nanosec = int((MOVE_TIME % 1.0) * 1e9)
    goal.trajectory.points = [pt]

    send_future = action.send_goal_async(goal)
    deadline = time.time() + GOAL_TIMEOUT
    while not send_future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not send_future.done():
        return False, "发送目标超时"
    handle = send_future.result()
    if handle is None or not handle.accepted:
        return False, "目标被拒绝（accepted=false）"

    result_future = handle.get_result_async()
    while not result_future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not result_future.done():
        return False, "等 action 结果超时"
    status = result_future.result().status
    # action_msgs/GoalStatus: 4 = SUCCEEDED
    code = getattr(result_future.result().result, "error_code", 0)
    if status != 4:
        return False, f"目标未成功：goal_status={status} error_code={code}"
    return True, f"goal_status={status} error_code={code}"


def main() -> int:
    rclpy.init()
    node = Probe()
    ok = True
    try:
        started = time.time()
        good, why = wait_for_controllers(node, ["joint_state_broadcaster", "joint_trajectory_controller"])
        print(f"[e2e] 等控制器 active: {'OK' if good else 'FAIL'}  ({why})")
        if not good:
            return 1

        # ⚠ 先等"轨迹前基线"到齐再用：Humble 上实测过"控制器刚 active 但 /joint_states
        #    还没来"的窗口，那时基线是空字典，后面算位移会默默退化成"相对 0"。
        deadline = time.time() + 5.0
        while time.time() < deadline and not all(j in node.last for j in JOINTS):
            node.spin(0.1)
        if not all(j in node.last for j in JOINTS):
            print(f"[e2e] FAIL: 等不到 /joint_states 里的 {JOINTS}（只收到 {sorted(node.last)}）")
            return 1
        before = dict(node.last)
        print(f"[e2e] 轨迹前的反馈: {before}")

        good, why = send_trajectory(node)
        print(f"[e2e] 轨迹执行: {'OK' if good else 'FAIL'}  ({why})")
        ok = ok and good

        # 让反馈再追几拍（轨迹结束后设备还在收敛）
        node.spin(1.0)
        after = dict(node.last)
        moved = {j: abs(after.get(j, 0.0) - before.get(j, 0.0)) for j in JOINTS}
        err = {j: abs(after.get(j, 0.0) - TARGET[j]) for j in JOINTS}
        print(f"[e2e] 轨迹后的反馈: {after}")
        print(f"[e2e] 位移={moved}  终点误差={err}  容差={TOL}")
        print(f"[e2e] /joint_states 样本数={node.samples}，最大|位置|={node.max_seen}")
        print(f"[e2e] 总耗时 {time.time() - started:.1f}s")

        if node.samples < 10:
            print("[e2e] FAIL: /joint_states 几乎没有样本（反馈链没起来？）")
            ok = False
        for j in JOINTS:
            if moved.get(j, 0.0) < 0.10:
                print(f"[e2e] FAIL: 关节 {j} 几乎没动（位移 {moved.get(j, 0.0):.3f} rad < 0.10）"
                      f" —— 命令没到设备，或 MIT 增益为 0（关节自由）")
                ok = False
            if err.get(j, 99.0) > TOL:
                print(f"[e2e] FAIL: 关节 {j} 终点误差 {err.get(j, 99.0):.3f} rad > 容差 {TOL}")
                ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("[e2e] " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
