import math
import os
import yaml
import rclpy
from rclpy.node import Node as RosNode

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

from modules.base_bt_nodes import (
    Node,
    Status,
    BTNodeList as BaseBTNodeList
)
from modules.base_bt_nodes_ros import (
    ActionWithROSAction,
)


# ------------------------------------------------------------
# 공통 좌표 (pickup/charge 는 고정)
# ------------------------------------------------------------
def deg(d):
    return math.radians(d)

CHARGE_X, CHARGE_Y, CHARGE_YAW = -4.198, 0.2, deg(89.274)
PICKUP_X, PICKUP_Y, PICKUP_YAW = -6.326, 3.209, deg(-78.415)


# ------------------------------------------------------------
def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _build_nav_goal(x, y, yaw, agent):
    goal = NavigateToPose.Goal()

    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = agent.ros_bridge.node.get_clock().now().to_msg()

    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation = yaw_to_quaternion(yaw)

    goal.pose = ps
    return goal


# ============================================================
# 1) SetGoal : pickup / charge / qr (3종류를 모두 수행)
# ============================================================
class SetGoal(Node):
    def __init__(self, name, mode):
        super().__init__(name)
        self.mode = mode

        # YAML 파일 위치 (현재 파일 기준)
        base = os.path.dirname(__file__)
        self.yaml_path = os.path.join(base, "qr_locations.yaml")

        # YAML 로딩
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, "r", encoding="utf-8") as f:
                self.qr_map = yaml.safe_load(f).get("qr_targets", {})
            print(f"[BT] Loaded QR YAML: {self.yaml_path}")
        else:
            self.qr_map = {}
            print(f"[BT] WARNING: qr_locations.yaml not found at {self.yaml_path}")

    async def run(self, agent, blackboard):

        # -------------------------
        # pickup
        # -------------------------
        if self.mode == "pickup":
            blackboard["goal_x"] = PICKUP_X
            blackboard["goal_y"] = PICKUP_Y
            blackboard["goal_yaw"] = PICKUP_YAW
            print(f"[BT] SetGoal: pickup → ({PICKUP_X}, {PICKUP_Y})")
            return Status.SUCCESS

        # -------------------------
        # charge
        # -------------------------
        if self.mode == "charge":
            blackboard["goal_x"] = CHARGE_X
            blackboard["goal_y"] = CHARGE_Y
            blackboard["goal_yaw"] = CHARGE_YAW
            print(f"[BT] SetGoal: charge → ({CHARGE_X}, {CHARGE_Y})")
            return Status.SUCCESS

        # -------------------------
        # qr : YAML 기반 목적지
        # -------------------------
        if self.mode == "qr":
            qr_key = blackboard.get("qr_code")

            if qr_key is None:
                print("[BT] SetGoal(qr): qr_code 없음")
                return Status.FAILURE

            if qr_key not in self.qr_map:
                print(f"[BT] SetGoal(qr): '{qr_key}' YAML에 없음")
                return Status.FAILURE

            pose = self.qr_map[qr_key]

            blackboard["goal_x"] = pose["x"]
            blackboard["goal_y"] = pose["y"]
            blackboard["goal_yaw"] = pose["yaw"]

            print(f"[BT] SetGoal(qr): {qr_key} → ({pose['x']}, {pose['y']})")
            return Status.SUCCESS

        return Status.FAILURE


# ============================================================
# 2) MoveToGoal : Nav2 NavigateToPose
# ============================================================
class MoveToGoal(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):

        x = blackboard.get("goal_x")
        y = blackboard.get("goal_y")
        yaw = blackboard.get("goal_yaw")

        if x is None:
            print("[BT] MoveToGoal: goal_x 없음")
            return None

        return _build_nav_goal(x, y, yaw, agent)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS if status_code == 0 else Status.FAILURE


# ============================================================
# 3) ReceiveParcel : QR + 택배 수령
# ============================================================
class ReceiveParcel(Node):
    def __init__(self, name, wait_sec=3.0):
        super().__init__(name)
        self.wait_sec = wait_sec
        self.start = None

    async def run(self, agent, blackboard):
        import time

        if self.start is None:
            self.start = time.time()
            return Status.RUNNING

        if time.time() - self.start >= self.wait_sec:
            self.start = None
            print("[BT] ReceiveParcel: 완료")
            return Status.SUCCESS

        return Status.RUNNING


# ============================================================
# 4) DropoffParcel
# ============================================================
class DropoffParcel(Node):
    def __init__(self, name, wait_sec=3.0):
        super().__init__(name)
        self.wait_sec = wait_sec
        self.start = None

    async def run(self, agent, blackboard):
        import time

        if self.start is None:
            self.start = time.time()
            return Status.RUNNING

        if time.time() - self.start >= self.wait_sec:
            self.start = None
            print("[BT] DropoffParcel: 완료")
            return Status.SUCCESS

        return Status.RUNNING


# ============================================================
# 5) HasParcel : qr_code 유무로 판단
# ============================================================
class HasParcel(Node):
    async def run(self, agent, blackboard):
        qr = blackboard.get("qr_code")

        if qr is None:
            print("[BT] HasParcel: 택배 없음 → FAILURE")
            return Status.FAILURE

        print("[BT] HasParcel: 택배 있음")
        return Status.SUCCESS


# ============================================================
# 노드 등록
# ============================================================
class BTNodeList:
    CONTROL_NODES = BaseBTNodeList.CONTROL_NODES
    ACTION_NODES = [
        "SetGoal",
        "MoveToGoal",
        "ReceiveParcel",
        "DropoffParcel",
        "HasParcel"
    ]
    CONDITION_NODES = []
    DECORATOR_NODES = []
