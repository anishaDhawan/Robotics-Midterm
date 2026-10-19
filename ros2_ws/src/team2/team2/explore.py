import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer', namespace='team2')
        self.min_frontier_dist = 2.3
        self.paused = True
        self.current_goal_handle = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PointStamped, 'exploration_goal', 10)
        
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Listen strictly to the Master Controller's explore state commands
        self.create_subscription(Bool, 'explore_control', self.control_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        
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

        # Use the robot position in the same map frame as the frontiers.
        try:
            pose = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        cx = pose.transform.translation.x
        cy = pose.transform.translation.y

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

        # Navigation owns Nav2 goals, so exploration cannot interrupt goal1/2/3.
        point = PointStamped()
        point.header.frame_id = 'map'
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        self.goal_pub.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
