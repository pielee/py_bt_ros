import math
import rclpy
from rclpy.node import Node as RosNode

# --- [새로 추가된 모듈] ---
from modules.utils import config  
from modules.qr_processor import AsyncQRProcessor
from sensor_msgs.msg import Image
# ------------------------

from modules.base_bt_nodes import (
    Node,
    Status,
    Sequence,
    BTNodeList as BaseBTNodeList
)
from modules.base_bt_nodes_ros import (
    ActionWithROSAction,
)

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


def deg(d: float) -> float:
    return math.radians(d)

# ================================
# 고정 위치 좌표 (충전소, 수령장소 등)
# ================================
CHARGE_X,  CHARGE_Y,  CHARGE_YAW  = -4.198, 0.2, deg(89.274)
PICKUP_X,  PICKUP_Y,  PICKUP_YAW  = -6.326, 3.209, deg(-78.415)
DELIV_X,   DELIV_Y,   DELIV_YAW   = -4.19, 2.063, deg(-6.810)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _build_nav_goal(x: float, y: float, yaw: float, agent) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = agent.ros_bridge.node.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation = yaw_to_quaternion(yaw)
    goal.pose = ps
    return goal


# ============================================
# 1) MoveToCharge
# ============================================
class MoveToCharge(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        return _build_nav_goal(CHARGE_X, CHARGE_Y, CHARGE_YAW, agent)


# ============================================
# 2) MoveToPickup
# ============================================
class MoveToPickup(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        return _build_nav_goal(PICKUP_X, PICKUP_Y, PICKUP_YAW, agent)


# ============================================
# 3) MoveToDelivery
# ============================================
class MoveToDelivery(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        pose: PoseStamped = blackboard.get("qr_target_pose", None)
        if pose is None:
            agent.ros_bridge.node.get_logger().warn("[MoveToDelivery] qr_target_pose is None")
            return None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal


# ============================================
# 4) ReceiveParcel
# ============================================
class ReceiveParcel(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent
        self._button_state = "release"
        self._node: RosNode = agent.ros_bridge.node
        self._sub = self._node.create_subscription(String, "/limo/button", self._callback, 10)

    def _callback(self, msg: String):
        self._button_state = msg.data.strip()

    async def run(self, agent, blackboard):
        if self._button_state.lower() == "pressed":
            self.status = Status.SUCCESS
            self._node.get_logger().info("[ReceiveParcel] Button Pressed")
        else:
            self.status = Status.RUNNING
        return self.status


# ============================================
# 5) DropoffParcel
# ============================================
class DropoffParcel(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent
        self._button_state = "release"
        self._node: RosNode = agent.ros_bridge.node
        self._sub = self._node.create_subscription(String, "/limo/button", self._callback, 10)

    def _callback(self, msg: String):
        self._button_state = msg.data.strip()

    async def run(self, agent, blackboard):
        if self._button_state.lower() == "release":
            self.status = Status.SUCCESS
            self._node.get_logger().info("[DropoffParcel] Button Released")
        else:
            self.status = Status.RUNNING
        return self.status


# =========================================================
# 6) WaitForQRPose (New Async Version)
# =========================================================
class WaitForQRPose(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.agent = agent
        
        # 비동기 프로세서 생성 (Config 전달)
        self.processor = AsyncQRProcessor(config)
        
        self._node: RosNode = agent.ros_bridge.node
        topic_name = config['qr_system']['camera_topic']

        self._sub = self._node.create_subscription(
            Image,
            topic_name, 
            self._img_callback,
            10
        )
        self._node.get_logger().info(f"[WaitForQRPose] Async Scan Started on {topic_name}")

    def _img_callback(self, msg: Image):
        # 콜백은 최대한 가볍게! 이미지만 던져주고 끝냄
        self.processor.update_image(msg)

    async def run(self, agent, blackboard):
        # 1. 프로세서에게 "결과 나왔니?" 하고 물어봄 (즉시 리턴됨)
        result = self.processor.get_result()

        if result is None:
            return Status.RUNNING
        
        # 2. 결과가 있으면 처리
        x, y, yaw_deg = result
        
        ps = PoseStamped()
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        
        yaw_rad = math.radians(yaw_deg)
        ps.pose.orientation.z = math.sin(yaw_rad / 2.0)
        ps.pose.orientation.w = math.cos(yaw_rad / 2.0)

        blackboard["qr_target_pose"] = ps
        self._node.get_logger().info(f"[WaitForQRPose] Target Found: ({x}, {y})")
        
        return Status.SUCCESS


# ============================================
# BT Node Registration
# ============================================
class BTNodeList:
    CONTROL_NODES = BaseBTNodeList.CONTROL_NODES
    ACTION_NODES = [
        "MoveToCharge",
        "MoveToPickup",
        "MoveToDelivery",
        "ReceiveParcel",
        "DropoffParcel",
        "WaitForQRPose",
    ]
    CONDITION_NODES = []
    DECORATOR_NODES = []