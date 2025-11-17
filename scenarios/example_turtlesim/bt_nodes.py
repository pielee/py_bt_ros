
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy.time

import modules.base_bt_nodes as base_bt_nodes
from modules.base_bt_nodes import (
    Status, 
    Sequence, 
    Fallback, 
    ReactiveSequence, 
    ReactiveFallback,
    AlwaysSuccess
)
from action_msgs.msg import GoalStatus


class CheckForNewGoal(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, 
                         msg_types_topics=[
                             (PoseStamped, '/bt/goal_pose', 'goal_pose')
                         ])
        self.last_goal_timestamp = None 

    def _predicate(self, agent, blackboard):
        if 'goal_pose' not in self._cache or self._cache['goal_pose'] is None:
            return False 

        current_goal_msg = self._cache['goal_pose']
        current_timestamp = current_goal_msg.header.stamp
        
        if self.last_goal_timestamp is None or current_timestamp != self.last_goal_timestamp:
            blackboard['goal_pose'] = current_goal_msg
            self.last_goal_timestamp = current_timestamp
            return True 
        else:
            return False 

class SaveInitialPose(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent,
                         msg_types_topics=[
                             (PoseWithCovarianceStamped, '/amcl_pose', 'amcl_pose')
                         ])
                         
    def _predicate(self, agent, blackboard):
        if 'amcl_pose' in self._cache and self._cache['amcl_pose'] is not None:
            current_pose_with_cov = self._cache['amcl_pose']
            initial_pose = current_pose_with_cov.pose.pose
            
            blackboard['home_pose'] = initial_pose 
            return True 
        else:
            return False 

class MoveToGoal(ActionWithROSAction):
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent,
            (NavigateToPose, f"{ns}/navigate_to_pose")
        )
        self.goal_pub = self.ros.node.create_publisher(PoseStamped, "debug/current_goal", 10)

    def _get_goal_pose(self, bb):
        goal = bb.get('goal_pose') 
        if isinstance(goal, PoseStamped):
            return goal
        return None

    def _build_goal(self, agent, bb):
        ps = self._get_goal_pose(bb)
        if ps is None:
            return None
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = ps
        return goal
    
    def _on_running(self, agent, bb):
        ps = self._get_goal_pose(bb)
        if ps is None: return
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        self.goal_pub.publish(ps)

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['move_to_goal_result'] = 'succeeded' 
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['move_to_goal_result'] = 'canceled'
            return Status.FAILURE
        else:
            bb['move_to_goal_result'] = 'aborted'
            return Status.FAILURE

class Return(ActionWithROSAction):
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent,
            (NavigateToPose, f"{ns}/navigate_to_pose")
        )
        goal_topic = f"{ns}/goal_pose" if ns else "/goal_pose"
        self.goal_pub = self.ros.node.create_publisher(PoseStamped, goal_topic, 10)

    def _get_home_xy(self, bb):
        home = bb.get('home_pose')
        if isinstance(home, Pose): 
            return home.position.x, home.position.y
        return None

    def _build_goal(self, agent, bb):
        xy = self._get_home_xy(bb)
        if xy is None:
            return None
        x, y = xy
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = ps
        return goal
    
    def _on_running(self, agent, bb):
        xy = self._get_home_xy(bb)
        if xy is None: return
        x, y = xy
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['return_result'] = 'succeeded' 
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['return_result'] = 'canceled'
            return Status.FAILURE
        else:
            bb['return_result'] = 'aborted'
            return Status.FAILURE

CUSTOM_ACTION_NODES = [
    'MoveToGoal',
    'Return', 
]
CUSTOM_CONDITION_NODES = [
    'CheckForNewGoal',
    'SaveInitialPose',
]

class BTNodeList:
    """ List of all available BT nodes """
    ACTION_NODES = base_bt_nodes.BTNodeList.ACTION_NODES.copy()
    CONDITION_NODES = base_bt_nodes.BTNodeList.CONDITION_NODES.copy()
    CONTROL_NODES = base_bt_nodes.BTNodeList.CONTROL_NODES.copy()
    DECORATOR_NODES = base_bt_nodes.BTNodeList.DECORATOR_NODES.copy()

    ACTION_NODES.extend(CUSTOM_ACTION_NODES)
    CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)