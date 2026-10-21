import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
from std_msgs.msg import Bool
import numpy as np
import math

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pause_sub = self.create_subscription(Bool, '/pause_exploration', self.pause_callback, 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.clear_global_client = self.create_client(Empty, '/global_costmap/clear_entire_global_costmap')
        self.clear_local_client = self.create_client(Empty, '/local_costmap/clear_entire_local_costmap')
        
        self.latest_map = None
        self.is_navigating = False
        self.is_paused = False
        
        self.min_cluster_size = 6
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.visited_goals = []
        self.last_target = None

        self.timer = self.create_timer(3.0, self.control_loop)
        self.get_logger().info('Wall-Safe Frontier Explorer Initialized!')

    def pause_callback(self, msg):
        self.is_paused = msg.data

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        self.latest_map = msg

    def control_loop(self):
        if not self.is_paused and not self.is_navigating and self.latest_map is not None:
            self.explore_next_frontier()

    def explore_next_frontier(self):
        try:
            grid = np.array(self.latest_map.data).reshape((self.latest_map.info.height, self.latest_map.info.width))
            frontiers = self.find_frontier_points(grid)
            
            if len(frontiers) == 0:
                self.get_logger().info('Exploration complete!')
                return
                
            clusters = self.cluster_points(frontiers)
            valid_clusters = [c for c in clusters if len(c) >= self.min_cluster_size]
            
            if not valid_clusters:
                self.clear_costmaps()
                return

            scored_targets = []

            for cluster in valid_clusters:
                centroid_float = np.mean(cluster, axis=0)
                best_p = min(cluster, key=lambda p: math.hypot(p[0]-centroid_float[0], p[1]-centroid_float[1]))
                r, c = int(best_p[0]), int(best_p[1])

                # WALL SAFETY CHECK: Ensure target cell and immediate surroundings are fully free (not near walls/obstacles)
                safety_radius = 4 # pixels
                r_min, r_max = max(0, r - safety_radius), min(grid.shape[0], r + safety_radius)
                c_min, c_max = max(0, c - safety_radius), min(grid.shape[1], c + safety_radius)
                
                if np.any(grid[r_min:r_max, c_min:c_max] > 50): # Occupied obstacle check
                    continue

                target_x = self.latest_map.info.origin.position.x + (c * self.latest_map.info.resolution)
                target_y = self.latest_map.info.origin.position.y + (r * self.latest_map.info.resolution)

                is_too_close = False
                for vx, vy in self.visited_goals:
                    if math.hypot(target_x - vx, target_y - vy) < 1.5:
                        is_too_close = True
                        break

                if is_too_close:
                    continue
                    
                unknown_count = np.sum(grid[r_min:r_max, c_min:c_max] == -1)
                dist = math.hypot(target_x - self.robot_x, target_y - self.robot_y)
                score = unknown_count / (dist + 0.5)

                scored_targets.append({
                    'x': target_x, 'y': target_y, 'score': score, 'dist': dist
                })

            if not scored_targets:
                self.visited_goals.clear()
                self.clear_costmaps()
                return

            scored_targets.sort(key=lambda item: -item['score'])
            best_target = scored_targets[0]
            
            self.visited_goals.append((best_target['x'], best_target['y']))
            if len(self.visited_goals) > 20:
                self.visited_goals.pop(0)

            self.send_goal(best_target['x'], best_target['y'], best_target['dist'])

        except Exception as e:
            self.get_logger().error(f'Exploration error: {str(e)}')
            self.is_navigating = False

    def find_frontier_points(self, grid):
        free_mask = (grid == 0)
        unknown_mask = (grid == -1)
        frontiers = []
        rows, cols = grid.shape
        for r in range(1, rows - 1, 2):
            for c in range(1, cols - 1, 2):
                if free_mask[r, c]:
                    if np.any(unknown_mask[r - 1:r + 2, c - 1:c + 2]):
                        frontiers.append((r, c))
        return frontiers

    def cluster_points(self, points):
        clusters = []
        visited = set()
        for p in points:
            if p in visited:
                continue
            cluster = []
            queue = [p]
            visited.add(p)
            while queue:
                curr = queue.pop(0)
                cluster.append(curr)
                for other in points:
                    if other not in visited and (abs(curr[0] - other[0]) <= 2 and abs(curr[1] - other[1]) <= 2):
                        visited.add(other)
                        queue.append(other)
            clusters.append(cluster)
        return clusters

    def clear_costmaps(self):
        if self.clear_global_client.wait_for_service(timeout_sec=0.2):
            self.clear_global_client.call_async(Empty.Request())
        if self.clear_local_client.wait_for_service(timeout_sec=0.2):
            self.clear_local_client.call_async(Empty.Request())

    def send_goal(self, x, y, dist):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return
        self.is_navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0
        self.get_logger().info(f'Safe Frontier Target: ({x:.2f}, {y:.2f}) | Dist: {dist:.1f}m')
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_navigating = False
            self.clear_costmaps()
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.clear_costmaps()
        self.is_navigating = False

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
