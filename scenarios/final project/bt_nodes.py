# scenarios/final_project/bt_nodes.py

import math
import time

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus

from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveFallback
from modules.base_bt_nodes_ros import (
    ConditionWithROSTopics,
    ActionWithROSAction,
)

# =========================================================
# Helper
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

    if pose_stamped is not None:
        goal.pose = pose_stamped
        return goal

    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0

    if yaw is not None:
        ps.pose.orientation = yaw_to_quaternion(float(yaw))
    else:
        ps.pose.orientation.w = 1.0

    goal.pose = ps
    return goal


# =========================================================
# Coordinates
# =========================================================
CHARGE_X, CHARGE_Y, CHARGE_YAW = -4.198, 0.200, deg(0.0)
PICKUP_X, PICKUP_Y, PICKUP_YAW = -6.326, 3.209, deg(90.0)
WAIT_X, WAIT_Y, WAIT_YAW = -3.000, 1.500, deg(0.0)

NAV_ACTION_NAME = "/limo/navigate_to_pose"

RECEIVE_BUSY_TOPIC = "/receive_busy"
DROPOFF_BUSY_TOPIC = "/dropoff_busy"


# =========================================================
# Decorators
# =========================================================
class RetryUntilSuccessful(Node):
    def __init__(self, name, child, num_attempts=1):
        super().__init__(name)
        self.child = child
        self.max_attempts = int(num_attempts)
        self.attempts = 0

    async def run(self, agent, blackboard):
        result = await self.child.run(agent, blackboard)

        if result == Status.SUCCESS:
            self.attempts = 0
            return Status.SUCCESS

        self.attempts += 1
        if self.attempts < self.max_attempts:
            return Status.RUNNING

        self.attempts = 0
        return Status.FAILURE

class Timeout(Node):
    def __init__(self, name, child, duration=10.0):
        super().__init__(name)
        self.child = child
        self.duration = float(duration)
        self.start_time = None
        self.is_running = False 
        self.type = "Decorator"

        # 🔴 추가: busy 퍼블리셔 & 중복 방지 플래그
        self.busy_pub = None
        self.busy_cleared = False

    async def run(self, agent, blackboard):
        # 🔴 추가: 퍼블리셔는 최초 1회만 생성
        if self.busy_pub is None:
            self.busy_pub = agent.ros_bridge.node.create_publisher(
                Bool, "/receive_busy", 10
            )

        # [핵심] 처음 시작할 때만 타이머 시작
        if not self.is_running or self.start_time is None:
            self.start_time = time.time()
            self.is_running = True
            self.busy_cleared = False   # 🔴 추가
            print(f"[{self.name}] ⏳ Timer Started. Limit: {self.duration}s")

        elapsed = time.time() - self.start_time
        
        # 1초마다 남은 시간 출력 (기존 그대로)
        if int(elapsed * 10) % 10 == 0:
             print(f"[{self.name}] ... {elapsed:.1f}s / {self.duration}s")

        # ⛔ 시간 초과
        if elapsed > self.duration:
            print(f"[{self.name}] 🚨 TIMEOUT! ({elapsed:.1f}s). Force FAILURE.")

            # 🔴 추가: busy false 한 번만 발행
            if not self.busy_cleared:
                self.busy_pub.publish(Bool(data=False))
                self.busy_cleared = True
                print(f"[{self.name}] 🔓 /receive_busy = false (timeout)")

            if hasattr(self.child, 'halt'):
                self.child.halt()

            self.is_running = False 
            self.status = Status.FAILURE
            return Status.FAILURE

        result = await self.child.run(agent, blackboard)
        
        if result == Status.SUCCESS:
            print(f"[{self.name}] Child Succeeded!")
            self.is_running = False
            self.status = Status.SUCCESS
            return Status.SUCCESS
        
        if result == Status.FAILURE:
            self.is_running = False
            self.status = Status.FAILURE
            return Status.FAILURE

        self.status = Status.RUNNING
        return Status.RUNNING

    def reset(self):
        # [의도 유지] 시간 유지
        super().reset()
        if hasattr(self.child, 'reset'):
            self.child.reset()

    def halt(self):
        # 🔴 halt 시에는 타이머 리셋 (기존 그대로)
        self.is_running = False
        self.start_time = None
        if hasattr(self.child, 'halt'):
            self.child.halt()


# =========================================================
# Condition Nodes
# =========================================================
class ReceiveParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(
            final_name,
            agent,
            [(String, "/limo/button_status", "button_state")]
        )

        self.busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, "/receive_busy", 10
        )
        self.busy_cleared = False

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False

        raw = self._cache["button_state"].data.strip().lower()

        if raw == "pressed":
            if not self.busy_cleared:
                self.busy_pub.publish(Bool(data=False))
                self.busy_cleared = True
                print("[ReceiveParcel] 🔓 /receive_busy = false")

            del self._cache["button_state"]
            return True

        return False


class DropoffParcel(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(
            final_name,
            agent,
            [(String, "/limo/button_status", "button_state")]
        )

        self.busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, DROPOFF_BUSY_TOPIC, 10
        )
        self.busy_cleared = False

    def _predicate(self, agent, blackboard):
        if "button_state" not in self._cache:
            return False

        state = self._cache["button_state"].data.strip().lower()

        if state in ["released", "release"]:
            if not self.busy_cleared:
                self.busy_pub.publish(Bool(data=False))
                self.busy_cleared = True
                print("[DropoffParcel] 🔓 /dropoff_busy = false")

            del self._cache["button_state"]
            return True

        return False


class ParcelAvailable(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(Bool, "/parcel_available", "parcel")]
        )

    async def run(self, agent, blackboard):
        msg = self._cache.get("parcel")
        if msg is None:
            return Status.FAILURE
        return Status.SUCCESS if msg.data else Status.FAILURE


# =========================================================
# 🔴 핵심: Busy 조건 (IF 노드)
# =========================================================
class OtherRobotReceiving(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(Bool, RECEIVE_BUSY_TOPIC, "recv_busy")]
        )

    async def run(self, agent, blackboard):
        msg = self._cache.get("recv_busy")
        if msg is None:
            return Status.FAILURE
        return Status.SUCCESS if msg.data else Status.FAILURE


class OtherRobotDropping(ConditionWithROSTopics):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            [(Bool, DROPOFF_BUSY_TOPIC, "drop_busy")]
        )

    async def run(self, agent, blackboard):
        msg = self._cache.get("drop_busy")
        if msg is None:
            return Status.FAILURE
        return Status.SUCCESS if msg.data else Status.FAILURE


# =========================================================
# QR
# =========================================================
class WaitForQRPose(Node):
    def __init__(self, node_name, agent, name=None):
        super().__init__(name if name else node_name)
        self.qr_pose = None
        agent.ros_bridge.node.create_subscription(
            PoseStamped,
            "/qr_warehouse_pose",
            self._cb,
            10
        )

    def _cb(self, msg):
        self.qr_pose = msg

    async def run(self, agent, blackboard):
        if self.qr_pose is None:
            return Status.RUNNING
        blackboard["qr_target_pose"] = self.qr_pose
        self.qr_pose = None
        return Status.SUCCESS


# =========================================================
# Actions
# =========================================================
class MoveToCharge(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(name if name else node_name, agent,
                         (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node, CHARGE_X, CHARGE_Y, CHARGE_YAW)


class MoveToPickup(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(
            name if name else node_name,
            agent,
            (NavigateToPose, NAV_ACTION_NAME)
        )       

        self.busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, RECEIVE_BUSY_TOPIC, 10
        )
        self.busy_sent = False  # 🔑 중복 발행 방지

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(
            self.ros.node,
            PICKUP_X, PICKUP_Y, PICKUP_YAW
        )

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[MoveToPickup] ✅ ARRIVED at pickup (nav success)")

            if not self.busy_sent:
                self.busy_pub.publish(Bool(data=True))
                self.busy_sent = True
                print("[MoveToPickup] 🔴 /receive_busy = true (published once)")

            return Status.SUCCESS

        return Status.FAILURE

class MoveToWaiting(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        super().__init__(name if name else node_name, agent,
                         (NavigateToPose, NAV_ACTION_NAME))

    def _build_goal(self, agent, blackboard):
        return _create_nav_goal(self.ros.node, WAIT_X, WAIT_Y, WAIT_YAW)


class MoveToDelivery(ActionWithROSAction):
    def __init__(self, node_name, agent, name=None):
        final_name = name if name else node_name
        super().__init__(final_name, agent, (NavigateToPose, NAV_ACTION_NAME))

        # 🔴 추가: dropoff busy 퍼블리셔
        self.busy_pub = agent.ros_bridge.node.create_publisher(
            Bool, DROPOFF_BUSY_TOPIC, 10
        )
        self.busy_sent = False  # 🔑 중복 발행 방지

    def _build_goal(self, agent, blackboard):
        qr_pose = blackboard.get("qr_target_pose")
        if qr_pose is None: 
            print(f"[{self.name}] ❌ ERROR: No QR Pose in blackboard!")
            return None

        print(f"[{self.name}] 🚚 Moving to Delivery Point (from QR)...")
        return _create_nav_goal(self.ros.node, 0, 0, pose_stamped=qr_pose)
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print(f"[{self.name}] ✅ ARRIVED at delivery")

            # 🔴 dropoff busy = true (한 번만)
            if not self.busy_sent:
                self.busy_pub.publish(Bool(data=True))
                self.busy_sent = True
                print(f"[{self.name}] 🔴 /dropoff_busy = true (published once)")

            # QR pose는 이제 소모됨
            if "qr_target_pose" in blackboard:
                del blackboard["qr_target_pose"]

            return Status.SUCCESS

        return Status.FAILURE


# =========================================================
# Registration
# =========================================================
BTNodeList.ACTION_NODES += [
    "MoveToCharge", "MoveToPickup", "MoveToWaiting",
    "MoveToDelivery", "WaitForQRPose"
]

BTNodeList.CONDITION_NODES += [
    "ReceiveParcel", "DropoffParcel",
    "OtherRobotReceiving", "OtherRobotDropping","ParcelAvailable"
]

BTNodeList.DECORATOR_NODES += [
    "RetryUntilSuccessful", "Timeout"
]
