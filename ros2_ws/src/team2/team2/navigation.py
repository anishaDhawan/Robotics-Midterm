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
        # Control topic for Frontier Exploration (True = explore, False = pause)
        self._explore_control_pub = self.create_publisher(Bool, '/explore_control', 10)
        
        self.current_status = Status()
        self.create_timer(1.0, self.publish_status)
        
        self.create_subscription(PointStamped, '/goal1', self.goal1_callback, 10)
        self.create_subscription(PointStamped, '/goal2', self.goal2_callback, 10)
        self.create_subscription(PointStamped, '/goal3', self.goal3_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.has_goal = False
        self.pending_goal1 = None
        self.pending_goal2 = None
        self.active_msg = PointStamped()
        self.active_goal_id = 0
        self.turn_direction = 0.5
        self.queue = Queue()
        
        self.get_logger().info('Midterm Master Controller ready. Awaiting goals while Frontier Explorer maps...')

    def publish_status(self):
        self._status_pub.publish(self.current_status)

    def scan_callback(self, msg: LaserScan):
        pass  # Muted to allow Frontier Exploration to navigate

    def set_exploration_state(self, enable_exploration: bool):
        msg = Bool()
        msg.data = enable_exploration
        self._explore_control_pub.publish(msg)

    def dispatch_nav2_goal(self, pt: PointStamped, goal_id: int):
        # Explicitly pause exploration ONLY when an active goal is dispatched
        self.set_exploration_state(False)
        
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
        self.get_logger().info(f'Received /goal1: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')
        self.current_status.goal1_received = True
        self.current_status.goal1_achieved = False

        if self.current_status.emergency:
            self.get_logger().warn('Emergency active! Holding Goal 1 in queue...')
            self.pending_goal1 = msg
            return
            
        self.pending_goal1 = None
        self.dispatch_nav2_goal(msg, 1)

    def goal2_callback(self, msg: PointStamped):
        self.get_logger().info(f'Received /goal2: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')
        self.current_status.goal2_received = True
        self.current_status.goal2_achieved = False
        
        if self.current_status.emergency or not self.current_status.goal1_achieved:
            self.get_logger().info('Goal 1 not yet achieved or Emergency active. Holding Goal 2 in queue (Exploration continues)...')
            self.pending_goal2 = msg
            return
            
        self.pending_goal2 = None
        self.dispatch_nav2_goal(msg, 2)

    def goal3_callback(self, msg: PointStamped):
        self.get_logger().warn(f'EMERGENCY /goal3 RECEIVED: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')
        self.current_status.emergency = True

        if self.has_goal:
            self.get_logger().info(f'Interrupting active Goal {self.active_goal_id}. Saving to queue...')
            self.queue.put((self.active_msg, self.active_goal_id))

        self.dispatch_nav2_goal(msg, 3) 

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 server.')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'SUCCESS: Reached Goal {self.active_goal_id}!')
            
            if self.active_goal_id == 1:
                self.current_status.goal1_achieved = True
                self.pending_goal1 = None
                
                # Automatically execute Goal 2 if it arrived earlier
                if self.pending_goal2 is not None:
                    self.get_logger().info('Goal 1 complete! Executing pending Goal 2...')
                    p_msg = self.pending_goal2
                    self.pending_goal2 = None
                    self.dispatch_nav2_goal(p_msg, 2)
                    return

            elif self.active_goal_id == 2:
                self.current_status.goal2_achieved = True
                self.pending_goal2 = None

            elif self.active_goal_id == 3:
                self.get_logger().info('Emergency goal secured.')
                self.current_status.emergency = False  # Reset emergency flag
                
                # Priority 1: Resume goal interrupted by emergency
                if self.queue.qsize() != 0:
                    mssg, id = self.queue.get()
                    self.get_logger().info(f'Resuming interrupted Goal {id} at X={mssg.point.x:.2f}, Y={mssg.point.y:.2f}')
                    self.dispatch_nav2_goal(mssg, id)
                    return  
                
                # Priority 2: Execute Goal 1 if received during emergency
                if self.pending_goal1 is not None:
                    self.get_logger().info('Emergency cleared! Executing pending Goal 1...')
                    p_msg = self.pending_goal1
                    self.pending_goal1 = None
                    self.dispatch_nav2_goal(p_msg, 1)
                    return

                # Priority 3: Execute Goal 2 if Goal 1 is done and Goal 2 was waiting
                if self.current_status.goal1_achieved and self.pending_goal2 is not None:
                    self.get_logger().info('Emergency cleared! Executing pending Goal 2...')
                    p_msg = self.pending_goal2
                    self.pending_goal2 = None
                    self.dispatch_nav2_goal(p_msg, 2)
                    return

            self.has_goal = False 
            # No active or pending goals remain -> Resume exploration
            self.get_logger().info('All active goals completed. Resuming Frontier Exploration...')
            self.set_exploration_state(True)

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
