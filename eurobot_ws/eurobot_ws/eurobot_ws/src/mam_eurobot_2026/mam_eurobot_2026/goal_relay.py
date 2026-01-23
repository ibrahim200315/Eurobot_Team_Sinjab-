#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class GoalRelay(Node):
    def __init__(self):
        super().__init__('goal_relay')
        self.sub = self.create_subscription(PoseStamped, '/task_manager/nav_goal', self.cb, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Goal relay ready: /task_manager/nav_goal -> navigate_to_pose')

    def cb(self, pose: PoseStamped):
        if not self.ac.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 navigate_to_pose action not available yet')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.ac.send_goal_async(goal)
        def on_goal(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected by Nav2')
                return
            self.get_logger().info('Goal accepted; waiting for result')
            _ = goal_handle.get_result_async()
        send_future.add_done_callback(on_goal)

def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

