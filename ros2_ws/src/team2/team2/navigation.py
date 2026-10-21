import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from project_status.msg import Status
from std_msgs.msg import Bool
import math


class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self._status_pub = self.create_publisher(Status, '/status', 10)
        self._pause_pub = self.create_publisher(Bool, '/pause_exploration', 10)

        self.create_subscription(PointStamped, '/goal1', self.goal1_callback, 10)
        self.create_subscription(PointStamped, '/goal2', self.goal2_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Point, '/human_detected', self.human_detected_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.current_status = Status()
        self.current_pose = None

        self.goal1_target = None
        self.goal2_target = None
        self.pending_target = None
        self.target_dispatched = False
        self.explorer_paused = False

        self.create_timer(0.1, self.publish_status)
        self.create_timer(1.0, self.nav_dispatch_loop)
        self.get_logger().info('Master Navigation Controller Active.')

    def set_explorer_pause(self, state):
        if self.explorer_paused != state:
            self.explorer_paused = state
            msg = Bool()
            msg.data = state
            self._pause_pub.publish(msg)
            if state:
                self.get_logger().info('>>> COMMANDED EXPLORER TO PAUSE <<<')
            else:
                self.get_logger().info('>>> COMMANDED EXPLORER TO RESUME <<<')

    def nav_dispatch_loop(self):
        if self.pending_target is not None and not self.target_dispatched:
            if self.nav_client.wait_for_server(timeout_sec=0.5):
                x, y = self.pending_target
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                # Let Nav2 use the latest transform instead of a rigid clock stamp that causes TF errors
                goal_msg.pose.header.stamp.sec = 0
                goal_msg.pose.header.stamp.nanosec = 0
                goal_msg.pose.pose.position.x = float(x)
                goal_msg.pose.pose.position.y = float(y)
                goal_msg.pose.pose.orientation.w = 1.0

                self.nav_client.send_goal_async(goal_msg)
                self.target_dispatched = True
                self.get_logger().info(f'>>> NAV2 DISPATCHED MANUAL GOAL: ({x:.2f}, {y:.2f}) <<<')

    def goal1_callback(self, msg):
        self.set_explorer_pause(True)
        self.current_status.goal1_received = True
        self.current_status.goal1_achieved = False
        self.goal1_target = (msg.point.x, msg.point.y)
        self.pending_target = self.goal1_target
        self.target_dispatched = False
        self.get_logger().info(f'Received /goal1: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')

    def goal2_callback(self, msg):
        self.set_explorer_pause(True)
        self.current_status.goal2_received = True
        self.current_status.goal2_achieved = False
        self.goal2_target = (msg.point.x, msg.point.y)
        self.get_logger().info(f'Received /goal2: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')

        if self.current_status.goal1_achieved:
            self.pending_target = self.goal2_target
            self.target_dispatched = False

    def human_detected_callback(self, msg):
        self.current_status.human_found = True
        self.current_status.human_location = msg
        self.get_logger().info(f'Human coordinates secured at: X={msg.x:.2f}, Y={msg.y:.2f}')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position

        if self.current_status.goal1_received and not self.current_status.goal1_achieved and self.goal1_target:
            dist1 = math.sqrt(
                (self.current_pose.x - self.goal1_target[0]) ** 2 + (self.current_pose.y - self.goal1_target[1]) ** 2)
            if dist1 < 0.8:
                self.current_status.goal1_achieved = True
                self.get_logger().info('>>> GOAL 1 ACHIEVED! <<<')

                if self.current_status.goal2_received and self.goal2_target:
                    self.pending_target = self.goal2_target
                    self.target_dispatched = False
                else:
                    self.set_explorer_pause(False)

        if self.current_status.goal2_received and not self.current_status.goal2_achieved and self.goal2_target:
            dist2 = math.sqrt(
                (self.current_pose.x - self.goal2_target[0]) ** 2 + (self.current_pose.y - self.goal2_target[1]) ** 2)
            if dist2 < 0.8:
                self.current_status.goal2_achieved = True
                self.get_logger().info('>>> GOAL 2 ACHIEVED! <<<')
                self.set_explorer_pause(False)

    def publish_status(self):
        self._status_pub.publish(self.current_status)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()