#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from queue import Queue

# Import the professor's custom status message
from project_status.msg import Status

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation', namespace='team2')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Setup Status Publisher
        self._status_pub = self.create_publisher(Status, '/status', 10)
        self._failed_frontier_pub = self.create_publisher(PointStamped, 'failed_frontier', 10)
        self.current_status = Status()
        # Retain the exploration state so a later-starting explorer receives it.
        control_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._explore_control_pub = self.create_publisher(Bool, 'explore_control', control_qos)
        self.exploration_enabled = True
        self.create_subscription(PointStamped, 'exploration_goal', self.exploration_callback, 10)
        self.create_timer(1.0, self.publish_status)
        
        # Listen to the specific topics graded by the professor
        self.create_subscription(PointStamped, '/goal1', self.goal1_callback, 10)
        self.create_subscription(PointStamped, '/goal2', self.goal2_callback, 10)
        self.create_subscription(PointStamped, '/goal3', self.goal3_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        self.create_subscription(PointStamped, 'human_location', self.human_callback, 10)
        self.create_subscription(Twist, 'nav2_cmd_vel', self.velocity_callback, 10)
        self.goal_handles = {}
        self.request_number = 0
        self.pending_goal1 = None
        self.pending_goal2 = None
        self.last_velocity = None
        self.create_timer(0.1, self.stop_if_needed)
        self.has_goal = False
        self.active_msg = PointStamped()
        self.active_goal_id = 0
        self.get_logger().info('Waiting for goals; human detection ends this mission.')
        self.queue = Queue()
        self.set_exploration_state(True)

    def publish_status(self):
        """Send the current goal progress and human-found result."""
        self._status_pub.publish(self.current_status)

    def scan_callback(self, msg: LaserScan):
        # Nav2 consumes the LiDAR; exploration also travels through Nav2.
        return

    def set_exploration_state(self, enabled):
        self.exploration_enabled = enabled and not self.current_status.human_found
        message = Bool()
        message.data = self.exploration_enabled
        self._explore_control_pub.publish(message)

    def exploration_callback(self, msg):
        # The explorer chooses a frontier, but this controller owns all Nav2 goals.
        # That prevents an exploration request from interrupting an assignment goal.
        if (self.current_status.human_found or self.current_status.emergency
                or self.has_goal or not self.exploration_enabled):
            return
        if msg.header.frame_id != 'map' or not all(
                math.isfinite(v) for v in (msg.point.x, msg.point.y)):
            return
        self.dispatch_nav2_goal(msg, 0)

    def human_callback(self, msg: PointStamped):
        # The vision node repeats its result. Handle only the first valid result.
        if self.current_status.human_found:
            return
        if msg.header.frame_id != 'map' or not all(
                math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z)):
            self.get_logger().warning('Ignoring human location without finite map coordinates.')
            return
        self.current_status.human_found = True
        self.set_exploration_state(False)
        self.current_status.human_location = msg.point
        self.current_status.emergency = False
        self.has_goal = False
        self.last_velocity = None
        self.queue = Queue()
        self.pending_goal1 = None
        self.pending_goal2 = None
        # Stop immediately; also cancel every accepted goal owned by this node.
        self._cmd_vel_pub.publish(Twist())
        for handle in list(self.goal_handles.values()):
            self.cancel_handle(handle)
        self.publish_status()
        self.get_logger().info('Human found: mission stopped. Restart navigation for a new trial.')

    def cancel_handle(self, handle):
        future = handle.cancel_goal_async()
        future.add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self.get_logger().warning('Nav2 did not accept cancellation; movement remains blocked.')
        except Exception as exc:
            self.get_logger().error(f'Cancellation failed: {exc}; movement remains blocked.')

    def movement_allowed(self):
        """Allow driving only while a goal is active and no human has been found."""
        # Depth updates do not gate movement; Nav2 still uses available obstacles.
        return not self.current_status.human_found and self.has_goal

    def velocity_callback(self, msg):
        # Nav2's movement commands pass through here first, so a late
        # command cannot restart the robot after the human has been found.
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y,
                msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)):
            self.last_velocity = None
            self._cmd_vel_pub.publish(Twist())
            return
        self.last_velocity = time.monotonic()
        self._cmd_vel_pub.publish(msg if self.movement_allowed() else Twist())

    def stop_if_needed(self):
        # Stop if Nav2 stops sending commands, or the mission no longer allows movement.
        if (not self.movement_allowed() or self.last_velocity is None
                or time.monotonic() - self.last_velocity > 0.5):
            self._cmd_vel_pub.publish(Twist())

    def dispatch_nav2_goal(self, pt: PointStamped, goal_id: int):
        """Give Nav2 the next destination and remember which request it belongs to."""
        if self.current_status.human_found:
            return
        self.set_exploration_state(False)
        self.request_number += 1
        request = self.request_number
        self.has_goal = True
        self.active_goal_id = goal_id
        self.active_msg = pt
        self._cmd_vel_pub.publish(Twist()) # Force immediate stop
        
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 action server not available!')
            self.has_goal = False
            if goal_id == 0:
                self.set_exploration_state(True)
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header = pt.header
        # goal_msg.pose.header.frame_id = 'map'
        # goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pt.point.x
        goal_msg.pose.pose.position.y = pt.point.y
        goal_msg.pose.pose.orientation.w = 1.0
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, request, goal_id))

    def goal1_callback(self, msg: PointStamped):
        """Start goal1, or save it until the emergency is finished."""
        if self.current_status.human_found:
            return
        self.current_status.goal1_received = True
        self.current_status.goal1_achieved = False
        self.publish_status()
        # Preserve the team's new behavior: keep goal1 if it arrives in an emergency.
        if self.current_status.emergency:
            self.pending_goal1 = msg
            return
        self.pending_goal1 = None
        self.dispatch_nav2_goal(msg, 1)

    def goal2_callback(self, msg: PointStamped):
        """Save goal2 until goal1 has been reached."""
        if self.current_status.human_found:
            return
        # Receiving goal2 does not mean we may drive there yet.
        # Keep the latest point, including if it arrives during an emergency.
        self.pending_goal2 = msg
        self.current_status.goal2_received = True
        self.current_status.goal2_achieved = False
        self.publish_status()
        self.start_pending_goal2()

    def start_pending_goal2(self):
        # Wait for successful goal1 completion and for any emergency to finish.
        if (self.current_status.human_found or self.current_status.emergency
                or (self.has_goal and self.active_goal_id != 0)
                or not self.current_status.goal1_achieved
                or self.pending_goal2 is None):
            return
        point = self.pending_goal2
        self.pending_goal2 = None
        self.dispatch_nav2_goal(point, 2)

    def goal3_callback(self, msg: PointStamped):
        """Go to the emergency first and remember the interrupted assignment goal."""
        if self.current_status.human_found:
            return
        self.get_logger().warn(f'EMERGENCY /goal3 RECEIVED: X={msg.point.x:.2f}, Y={msg.point.y:.2f}')
        self.current_status.emergency = True

        if self.has_goal and self.active_goal_id in (1, 2):
            self.queue.put((self.active_msg, self.active_goal_id))

        self.publish_status()
        self.dispatch_nav2_goal(msg, 3) 

    def goal_response_callback(self, future, request, goal_id):
        """Check whether Nav2 accepted this goal; cancel it if it is no longer needed."""
        try:
            goal_handle = future.result()
        except Exception as exc:
            if request == self.request_number:
                self.has_goal = False
                if goal_id == 0:
                    self.set_exploration_state(True)
            self.get_logger().error(f'Goal request failed: {exc}')
            return
        if not goal_handle.accepted:
            if request == self.request_number:
                self.has_goal = False
                if goal_id == 0:
                    self.set_exploration_state(True)
            return
        self.goal_handles[request] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.get_result_callback(result, request, goal_id))
        # Finding the human can happen while Nav2 is still accepting a goal.
        if self.current_status.human_found or request != self.request_number:
            self.cancel_handle(goal_handle)

    def get_result_callback(self, future, request, goal_id):
        """Update progress when a goal finishes, then choose what happens next."""
        self.goal_handles.pop(request, None)
        # Old goal results must not resume movement or overwrite the new mission state.
        if self.current_status.human_found or request != self.request_number:
            return
        try:
            result = future.result()
        except Exception as exc:
            self.has_goal = False
            self.get_logger().error(f'Goal result failed: {exc}')
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.has_goal = False
            if goal_id == 0:
                # Only a failed exploration goal is remembered; emergencies and human
                # stops must not label an interrupted route as unreachable.
                if result.status == GoalStatus.STATUS_ABORTED:
                    self._failed_frontier_pub.publish(self.active_msg)
                self.set_exploration_state(True)
            return
        self.has_goal = False
        if goal_id == 1:
            self.current_status.goal1_achieved = True
        elif goal_id == 2:
            self.current_status.goal2_achieved = True
        elif goal_id == 3:
            self.current_status.emergency = False
            self.publish_status()
            # Resume an interrupted assignment goal before selecting new frontiers.
            if not self.queue.empty():
                point, interrupted_id = self.queue.get()
                self.dispatch_nav2_goal(point, interrupted_id)
                return
        self.publish_status()
        # Preserve goals that arrived while the emergency was being handled.
        if self.pending_goal1 is not None:
            point = self.pending_goal1
            self.pending_goal1 = None
            self.current_status.goal1_achieved = False
            self.publish_status()
            self.dispatch_nav2_goal(point, 1)
            return
        self.start_pending_goal2()
        if not self.has_goal:
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
