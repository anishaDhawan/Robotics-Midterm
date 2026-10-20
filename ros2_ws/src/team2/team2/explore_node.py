#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Twist
import numpy as np

class LidarSafeExplorer(Node):
    def __init__(self):
        super().__init__('explore_node')
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.goal_pub = self.create_publisher(PointStamped, '/goal_point', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.min_obstacle_distance = 0.4  # meters
        self.is_danger = False
        self.latest_map = None
        
        self.get_logger().info('LiDAR-Backed Safe Explorer Active.')

    def scan_callback(self, msg):
        # Check 180 degrees in front of the robot for immediate obstacles
        front_ranges = msg.ranges[len(msg.ranges)*3//8 : len(msg.ranges)*5//8]
        valid_ranges = [r for r in front_ranges if not np.isinf(r) and not np.isnan(r)]
        
        if valid_ranges and min(valid_ranges) < self.min_obstacle_distance:
            self.is_danger = True
            # Emergency stop & backup command
            twist = Twist()
            twist.linear.x = -0.05
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)
            self.get_logger().warn(f'LiDAR Emergency! Wall detected at {min(valid_ranges):.2f}m. Backing away.')
        else:
            self.is_danger = False

    def map_callback(self, msg):
        if self.is_danger:
            return  # Skip frontier selection if escaping an obstacle
            
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Find true unmapped frontiers: Free cells (0) bordering unknown space (-1)
        frontiers = []
        for y in range(8, height - 8, 2):
            for x in range(8, width - 8, 2):
                if data[y, x] == 0:
                    neighbors = data[y-1:y+2, x-1:x+2]
                    if -1 in neighbors:
                        # Ensure no mapped walls (100) are within a strict 8-cell safety zone (~0.4m)
                        local_area = data[y-8:y+9, x-8:x+9]
                        if 100 not in local_area:
                            frontiers.append((x, y))

        if not frontiers:
            self.get_logger().info('No safe frontiers found. Exploration complete.')
            return

        # Pick the first verified frontier in unknown space
        target_x, target_y = frontiers[0]
        world_x = target_x * resolution + origin_x
        world_y = target_y * resolution + origin_y

        goal_msg = PointStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.point.x = float(world_x)
        goal_msg.point.y = float(world_y)
        goal_msg.point.z = 0.0

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Navigating to Unmapped Frontier: X={world_x:.2f}, Y={world_y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarSafeExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
