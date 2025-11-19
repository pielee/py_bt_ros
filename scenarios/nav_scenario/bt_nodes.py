import math
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback
# BT Node List
CUSTOM_ACTION_NODES = [
    'MoveToGoal',
    'CaptureImage',
    'ReturnHome',
]

CUSTOM_CONDITION_NODES = [
    'InitNavGoal',
]

# BT Node List
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction, ActionWithROSService


class InitNavGoal(ConditionWithROSTopics):
    """
    /bt/goal_pose 와 /amcl_pose 를 받아서
    - blackboard['goal_pose']
    - blackboard['initial_pose']
    를 세팅하는 조건 노드.
    둘 다 한 번 이상 들어오면 SUCCESS.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""

        # /bt/goal_pose: PoseStamped
        goal_topic = "/bt/goal_pose"

        # amcl_pose 토픽 네임스페이스는 상황에 따라 다를 수 있음
        # Nav2 기본이면 보통 "/amcl_pose"
        amcl_topic = f"{ns}/amcl_pose" if ns else "/amcl_pose"

        super().__init__(name, agent, [
            (PoseStamped, goal_topic, 'goal'),
            (PoseWithCovarianceStamped, amcl_topic, 'amcl'),
        ])


    def _predicate(self, agent, blackboard):
        cache = self._cache

        if 'goal' not in cache or 'amcl' not in cache:
            # 아직 둘 중 하나라도 안 들어왔으면 조건 불만족
            return False

        goal_msg: PoseStamped = cache['goal']
        amcl_msg: PoseWithCovarianceStamped = cache['amcl']


        # goal_pose 그대로 저장
        blackboard['goal_pose'] = goal_msg

        # amcl_pose 를 PoseStamped 로 변환해서 initial_pose 로 저장
        init = PoseStamped()
        init.header.frame_id = amcl_msg.header.frame_id
        init.header.stamp = amcl_msg.header.stamp
        init.pose = amcl_msg.pose.pose
        blackboard['initial_pose'] = init

        # 필요하면 debug 로그
        self.ros.node.get_logger().info(
            f"[InitNavGoal] goal=({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}), "
            f"initial=({init.pose.position.x:.2f}, {init.pose.position.y:.2f})"
        )

        return True

# bt_nodes.py (발췌)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class MoveToGoal(ActionWithROSAction):
    """
    블랙보드의 'goal_pose'(PoseStamped)를 가져와
    Nav2 /navigate_to_pose 액션에 전달하는 노드.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, bb):
        if 'goal_pose' not in bb:
            self.ros.node.get_logger().error("[MoveToGoal] goal_pose not in blackboard")
            return None

        goal_pose: PoseStamped = bb['goal_pose']
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['move_result'] = 'succeeded'
            self.ros.node.get_logger().info("[MoveToGoal] Nav2 succeeded")
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['move_result'] = 'canceled'
            self.ros.node.get_logger().warn("[MoveToGoal] Nav2 canceled")
            return Status.FAILURE
        else:
            bb['move_result'] = f'aborted({status_code})'
            self.ros.node.get_logger().error(f"[MoveToGoal] Nav2 aborted with status {status_code}")
            return Status.FAILURE

class CaptureImage(ActionWithROSService):
    """
    /capture_image (std_srvs/Trigger) 서비스를 호출해서
    사진을 저장하는 노드.
    """

    def __init__(self, name, agent):
        super().__init__(name, agent, (Trigger, "/capture_image"))

    def _build_request(self, agent, blackboard):
        return Trigger.Request()

    def _interpret_response(self, response, agent, blackboard):
        if response.success:
            blackboard['capture_result'] = response.message
            self.ros.node.get_logger().info(f"[CaptureImage] {response.message}")
            return Status.SUCCESS
        else:
            self.ros.node.get_logger().error(f"[CaptureImage] failed: {response.message}")
            return Status.FAILURE



# ============================================
# 4) ReturnHome : Nav2 Action 복귀
# ============================================
class ReturnHome(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, blackboard):
        pose: PoseStamped = blackboard.get("initial_pose")
        if pose is None:
            return None

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        return Status.SUCCESS