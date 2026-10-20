import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.min_frontier_dist = 2.3
        self.paused = False
        self.current_goal_handle = None
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Listen strictly to the Master Controller's explore state commands
        self.create_subscription(Bool, '/explore_control', self.control_callback, 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.current_pose = None
        self.latest_map = None
        self.goal_in_progress = False

        self.get_logger().info('Frontier Explorer Node active with Master Controller control.')

    def control_callback(self, msg: Bool):
        if not msg.data:
            # False = Pause exploration
            if not self.paused:
                self.get_logger().info('Master Controller requested pause. Stopping exploration.')
                self.paused = True
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
        else:
            # True = Resume exploration
            if self.paused:
                self.get_logger().info('Master Controller requested resume. Resuming exploration!')
                self.paused = False
                self.goal_in_progress = False
                self.find_and_send_frontier()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def map_callback(self, msg):
        if self.paused:
            return
        self.latest_map = msg
        if not self.goal_in_progress and self.current_pose:
            self.find_and_send_frontier()

    def find_and_send_frontier(self):
        if self.latest_map is None or self.current_pose is None or self.paused:
            return

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        frontiers = self.get_frontiers(self.latest_map)
        if not frontiers:
            self.get_logger().info('No remaining frontiers detected.')
            return

        valid_frontiers = []
        for fx, fy in frontiers:
            dist = math.sqrt((fx - cx)**2 + (fy - cy)**2)
            if dist >= self.min_frontier_dist:
                valid_frontiers.append(((fx, fy), dist))

        if valid_frontiers:
            target_pt, dist = min(valid_frontiers, key=lambda item: item[1])
        else:
            target_pt = frontiers[0]
            dist = math.sqrt((target_pt[0] - cx)**2 + (target_pt[1] - cy)**2)

        self.send_goal(target_pt[0], target_pt[1], dist)

    def get_frontiers(self, grid_map):
        frontiers = []
        width = grid_map.info.width
        height = grid_map.info.height
        res = grid_map.info.resolution
        origin_x = grid_map.info.origin.position.x
        origin_y = grid_map.info.origin.position.y
        data = grid_map.data

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x
                if data[idx] == 0:
                    neighbors = [data[idx + 1], data[idx - 1], data[idx + width], data[idx - width]]
                    if -1 in neighbors:
                        world_x = origin_x + (x + 0.5) * res
                        world_y = origin_y + (y + 0.5) * res
                        frontiers.append((world_x, world_y))
        return frontiers

    def send_goal(self, x, y, dist):
        if self.paused:
            return
        self.goal_in_progress = True
        self.get_logger().info(f'Exploring new area at X: {x:.2f}, Y: {y:.2f} (Target Distance: {dist:.2f}m)')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by navigation server.')
            self.goal_in_progress = False
            return
        
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.current_goal_handle = None
        if self.paused:
            return
        
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached target area.')
            self.goal_in_progress = False
        else:
            self.get_logger().warn('Goal preempted externally.')
            self.goal_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
