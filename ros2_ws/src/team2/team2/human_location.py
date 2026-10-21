import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import math
from ultralytics import YOLO

class AIDetectorNode(Node):
    def __init__(self):
        super().__init__('human_location_node')
        self.bridge = CvBridge()
        
        self.get_logger().info('Loading Proximity-Filtered YOLOv8 AI Model...')
        self.model = YOLO('yolov8n.pt')
        
        self.fx = self.fy = self.cx = self.cy = None
        self.human_found = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.human_pub = self.create_publisher(Point, '/human_detected', 10)
        self.get_logger().info('Proximity Vision Active. Scanning for close humans...')

    def info_callback(self, msg):
        if self.fx is None:
            self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def rgb_callback(self, msg):
        if self.human_found or self.fx is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            results = self.model(cv_image, classes=[0], verbose=False)
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    conf = box.conf[0].item()
                    
                    # Require higher confidence (50%) to prevent startup ghost triggers
                    if conf > 0.50:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        box_width = x2 - x1
                        box_height = y2 - y1
                        
                        if (box_height / (box_width + 1e-5)) < 1.1:
                            continue

                        raw_distance = (self.fy * 1.8) / box_height
                        distance_z = min(max(raw_distance * 0.35, 0.4), 3.0)
                        
                        # PROXIMITY GATE: Ignore detections calculated further than 3.5 meters away
                        if distance_z > 3.5:
                            continue

                        human_map_x = self.robot_x + distance_z * math.cos(self.robot_yaw)
                        human_map_y = self.robot_y + distance_z * math.sin(self.robot_yaw)
                        
                        self.human_found = True
                        self.human_pub.publish(Point(x=float(human_map_x), y=float(human_map_y), z=0.0))
                        self.get_logger().info(f'>>> REAL PROXIMITY HUMAN AT: X={human_map_x:.2f}, Y={human_map_y:.2f} <<<')
                        return
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AIDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
