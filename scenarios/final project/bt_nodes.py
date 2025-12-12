# scenarios/final_project/bt_nodes.py

import math
import time
import rclpy

# ROS 2 Messages & Action
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus

# Base Modules
from modules.base_bt_nodes import (
    BTNodeList,
    Status,
    Node,
    Sequence,
    Fallback,
)
from modules.base_bt_nodes_ros import (
    ConditionWithROSTopics,
    ActionWithROSAction,
)

# =========================================================
# Helper Functions (원본 유지)
# =========================================================
def deg(d: float) -> float:
    return math.radians(d)

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def _create_nav_goal(node, x, y, yaw=None, pose_stamped=None):
    goal = NavigateToPose.Goal()
    if pose_stamped:
        goal.pose = pose_stamped
    else:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        if yaw is not None:
            ps.pose.orientation = yaw_to_quaternion(yaw)
        else:
            ps.pose.orientation.w = 1.0
        goal.pose = ps
    return goal


# =========================================================
# Coordinates (원본 유지)
# =========================================================
CHARGE_X, CHARGE_Y, CHARGE_YAW = -4.198, 0.200, deg(0.0)
PICKUP_X, PICKUP_Y, PICKUP_YAW = -6.326, 3.209, deg(90.0)
WAIT_X,   WAIT_Y,   WAIT_YAW   = -3.000, 1.500, deg(0.0)

NAV_ACTION_NAME = "/limo/navigate_to_pose"

RECEIVE_BUSY_TOPIC = "/receive_busy"
DROPOFF_BUSY_TOPIC = "/dropoff_busy"


# =========================================================
# 1. Decorators (실제 구현)
# =========================================================
class RetryUntilSuccessful(Node):
    def __init__(self, name, child, num_attempts=1):
        super().__init__(name)
        self.child = child
        self.max_attempts = int(num_attempts)
        self.count = 0
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        result = await self.child.run(agent, blackboard)

        if result == Status.SUCCESS:
            self.count = 0
            return Status.SUCCESS

        if result == Status.FAILURE:
            self.count += 1
            if self.count >= self.max_attempts:
                self.count = 0
                return Status.FAILURE
            return Status.RUNNING

        return Status.RUNNING


class Timeout(Node):
    def __init__(self, name, child, duration=10.0):
        super().__init__(name)
        self.child = child
        self.duration = float(duration)
        self.start_time = None
        self.type = "Decorator"

    async def run(self, agent, blackboard):
        if self.start_time is None:
            self.start_time = time.time()

        if time.time() - self.start_time > self.duration:
            self.start_time = None
            return Status.FAILURE

        result = await self.child.run(agent, blackboard)

        if result != Status.RUNNING:
            self.start_time = None

        return result


# =========================================================
# 2. Button-based Conditions (원본 + 유지)
# =========================================================
class ReceiveParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [
            (String, "/limo/button_status", "button_state")
        ])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False

        state = self._cache["button_state"].data.strip().lower()
        if state == "pressed":
            del self._cache["button_state"]
            return True
        return False


class DropoffParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [
            (String, "/limo/button_status", "button_state")
        ])

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False

        state = self._cache["button_state"].data.strip().lower()
        if state in ["released", "release"]:
            del self._cache["button_state"]
            return True
        return False


# =========================================================
# 3. Busy Conditions (추가)
# =========================================================
class OtherRobotReceiving(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [
            (Bool, RECEIVE_BUSY_TOPIC, "recv_busy")
        ])
        self.type = "Condition"

    async def run(self, agent, blackboard):
        msg = self._cache.get("recv_busy", None)
        if msg is None:
            return Status.FAILURE
        return Status.SUCCESS if msg.data else Status.FAILURE


class OtherRobotDropping(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, [
            (Bool, DROPOFF_BUSY_TOPIC, "drop_busy")
        ])
        self.type = "Condition"

    async def run(self, agent, blackboard):
        msg = self._cache.get("drop_busy", None)
        if msg is None:
            return Status.FAILURE
        return Status.SUCCESS if msg.data else Status.FAILURE


# =========================================================
# 4. Arrival Conditions (추가)
# =========================================================
class AtPickupLocation(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Condition"

    async def run(self, agent, blackboard):
        return Status.SUCCESS if blackboard.get("at_pickup", False) else Status.FAILURE


class AtDeliveryLocation(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Condition"

    async def run(self, agent, blackboard):
        return Status.SUCCESS if blackboard.get("at_delivery", False) else Status.FAILURE


# =========================================================
# 5. QR Detection (원본 유지 + 안정화)
# =========================================================
class WaitForQRPose(Node):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name)
        self.ros = agent.ros_bridge
        self.qr_pose = None

        self.sub = self.ros.node.create_subscription(
            PoseStamped,
            "/qr_warehouse_pose",
            self.listener_callback,
            10
        )
        self.type = "Action"

    def listener_callback(self, msg: PoseStamped):
        self.qr_pose = msg

    async def run(self, agent, blackboard):
        if self.qr_pose:
            blackboard["qr_target_pose"] = self.qr_pose
            return Status.SUCCESS
        return Status.RUNNING

    def halt(self):
        self.qr_pose = None
        super().halt()


# =========================================================
# 6. Action Nodes
# =========================================================
class MoveToCharge(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(name or node_name, agent,
                         (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node,
                                CHARGE_X, CHARGE_Y, CHARGE_YAW)


class MoveToPickup(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(name or node_name, agent,
                         (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node,
                                PICKUP_X, PICKUP_Y, PICKUP_YAW)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            blackboard["at_pickup"] = True
            return Status.SUCCESS
        return Status.FAILURE


class MoveToWaiting(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(name or node_name, agent,
                         (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node,
                                WAIT_X, WAIT_Y, WAIT_YAW)


class MoveToDelivery(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(name or node_name, agent,
                         (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        pose = blackboard.get("qr_target_pose")
        if pose is None:
            return None
        return _create_nav_goal(self.ros.node, 0, 0, pose_stamped=pose)

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            blackboard["at_delivery"] = True
            blackboard.pop("qr_target_pose", None)
            return Status.SUCCESS
        return Status.FAILURE


# =========================================================
# 7. Node Registration (완전 보존)
# =========================================================
BTNodeList.ACTION_NODES.extend([
    "MoveToCharge",
    "MoveToPickup",
    "MoveToWaiting",
    "MoveToDelivery",
    "WaitForQRPose",
])

BTNodeList.CONDITION_NODES.extend([
    "ReceiveParcel",
    "DropoffParcel",
    "OtherRobotReceiving",
    "OtherRobotDropping",
    "AtPickupLocation",
    "AtDeliveryLocation",
])

BTNodeList.DECORATOR_NODES.extend([
    "RetryUntilSuccessful",
    "Timeout",
])
