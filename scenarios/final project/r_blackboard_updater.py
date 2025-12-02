import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class QrBlackboardUpdater(Node):
    def __init__(self, bt_blackboard):
        super().__init__("qr_blackboard_updater")

        self.blackboard = bt_blackboard

        # QR Router에서 보내는 PoseStamped
        self.create_subscription(
            PoseStamped,
            "qr_warehouse_pose",
            self.pose_callback,
            10
        )

        # QR code 원본 문자열
        self.create_subscription(
            String,
            "qr_warehouse",
            self.code_callback,
            10
        )

        print("[QR-BB] Initialized.")

    def pose_callback(self, msg: PoseStamped):
        self.blackboard["goal_x"] = msg.pose.position.x
        self.blackboard["goal_y"] = msg.pose.position.y
        self.blackboard["goal_yaw"] = 0.0  # 필요시 yaw 추가

        print(f"[QR-BB] Pose → Blackboard: "
              f"({msg.pose.position.x}, {msg.pose.position.y})")

    def code_callback(self, msg: String):
        self.blackboard["qr_code"] = msg.data
        print(f"[QR-BB] QR Code → Blackboard: {msg.data}")

