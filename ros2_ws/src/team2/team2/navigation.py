#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# Import the professor's custom status message
from project_status.msg import Status

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Setup Status Publisher
        self._status_pub = self.create_publisher(Status, '/status', 10)
        self.current_status = Status()
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.has_goal = False
        self.turn_direction = 0.5
        self.get_logger().info('Node ready. Randomly roaming the house...')

    def publish_status(self):
        self._status_pub.publish(self.current_status)

    def scan_callback(self, msg: LaserScan):
        if self.has_goal:
            return  # Let Nav2 handle driving

        ranges = msg.ranges
        front_ranges = ranges[0:20] + ranges[-20:]
        valid_ranges = [r for r in front_ranges if 0.1 < r < 10.0]
        min_dist = min(valid_ranges) if valid_ranges else 10.0

        twist = Twist()
        if min_dist < 0.30:
            twist.linear.x = 0.0
            twist.angular.z = self.turn_direction
        else:
            twist.linear.x = 0.2
            twist.angular.z = random.uniform(-0.15, 0.15)
            self.turn_direction = random.choice([-0.6, 0.6])
            
        self._cmd_vel_pub.publish(twist)

    def goal_callback(self, msg: PoseStamped):
        if not self.has_goal:
            self.has_goal = True
            self._cmd_vel_pub.publish(Twist()) # Stop roaming
            self.get_logger().info('Goal received! Nav2 taking over.')
            
        # Update professor's status tracker
        self.current_status.goal1_received = True
        self.current_status.goal1_achieved = False

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('SUCCESS: Destination reached!')
            self.current_status.goal1_achieved = True
            self.has_goal = False 
            
def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
