#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from queue import Queue

from project_status.msg import Status

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self._status_pub = self.create_publisher(Status, '/status', 10)
        self._resume_pub = self.create_publisher(Bool, '/explore_resume', 10)
        
        self.current_status = Status()
        self.create_timer(1.0, self.publish_status)
        
        self.create_subscription(PointStamped, '/goal1', self.goal1_callback, 10)
        self.create_subscription(PointStamped, '/goal2', self.goal2_callback, 10)
        self.create_subscription(PointStamped, '/goal3', self.goal3_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.has_goal = False
        self.active_msg = PointStamped()
        self.active_goal_id = 0
        self.turn_direction = 0.5
        self.get_logger().info('Midterm Node ready. Awaiting goals while Frontier Explorer maps...')
        self.queue = Queue()

    def publish_status(self):
        self._status_pub.publish(self.current_status)

    def scan_callback(self, msg: LaserScan):
        pass  # Disabled random LiDAR roaming so Frontier Exploration can map freely

    def dispatch_nav2_goal(self, pt: PointStamped, goal_id: int):
        self.has_goal = True
        self.active_goal_id = goal_id
        self.active_msg = pt
        self._cmd_vel_pub.publish(Twist()) # Force immediate stop
        
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 action server not available!')
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header = pt.header
        goal_msg.pose.pose.position.x = pt.point.x
        goal_msg.pose.pose.position.y = pt.point.y
        goal_msg.pose.pose.orientation.w = 1.0
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal1_callback(self, msg: PointStamped):
        if self.current_status.emergency:
            self.get_logger().warn('Ignoring /goal1: Robot is in EMERGENCY mode!')
            return
            
        self.get_logger().info(f'Received /goal1: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')
        self.current_status.goal1_received = True
        self.current_status.goal1_achieved = False
        self.dispatch_nav2_goal(msg, 1)

    def goal2_callback(self, msg: PointStamped):
        if self.current_status.emergency:
            self.get_logger().warn('Ignoring /goal2: Robot is in EMERGENCY mode!')
            return
            
        self.get_logger().info(f'Received /goal2: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')
        self.current_status.goal2_received = True
        self.current_status.goal2_achieved = False
        self.dispatch_nav2_goal(msg, 2)

    def goal3_callback(self, msg: PointStamped):
        self.get_logger().warn(f'EMERGENCY /goal3 RECEIVED: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')
        self.current_status.emergency = True

        if (self.has_goal == True):
            self.queue.put((self.active_msg, self.active_goal_id))

        self.dispatch_nav2_goal(msg, 3) 

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'SUCCESS: Reached Goal {self.active_goal_id}!')
            
            if self.active_goal_id == 1:
                self.current_status.goal1_achieved = True
            elif self.active_goal_id == 2:
                self.current_status.goal2_achieved = True
            elif self.active_goal_id == 3:
                self.get_logger().info('Emergency goal secured.')
                
                if (self.queue.qsize() != 0):
                    mssg, id = self.queue.get()
                    self.get_logger().info(f'Returning to Goal {id} at X={mssg.point.x:.2f}, Y={mssg.point.y:.2f}')
                    self.active_goal_id = id
                    self.dispatch_nav2_goal(mssg, id)
                    return  # Do not publish resume signal if returning to queue
            
            self.has_goal = False 
            
            # Publish resume signal so Frontier Explorer can start mapping again
            msg = Bool()
            msg.data = True
            self._resume_pub.publish(msg)
            
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
