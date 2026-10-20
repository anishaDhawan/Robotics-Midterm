#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class AIDetectorNode(Node):
    def __init__(self):
        super().__init__('human_location_node')
        self.bridge = CvBridge()
        
        # Load the YOLOv8 Nano model (fastest, runs easily on CPU/VMs)
        self.get_logger().info('Loading YOLOv8 AI Model...')
        self.model = YOLO('yolov8n.pt')
        
        self.fx = self.fy = self.cx = self.cy = None
        self.human_found = False
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, qos_profile_sensor_data)
        self.human_pub = self.create_publisher(Point, '/human_detected', 10)
        
        self.get_logger().info('YOLOv8 Vision Active. Scanning for humans...')

    def info_callback(self, msg):
        if self.fx is None:
            self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def rgb_callback(self, msg):
        if self.human_found or self.fx is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run the AI specifically looking for Class 0 (Person)
            results = self.model(cv_image, classes=[0], verbose=False)
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = box.conf[0].item()
                    
                    if conf > 0.50:  # 50% confidence threshold
                        self.get_logger().info(f'AI Detection Confirmed! Confidence: {conf*100:.1f}%')
                        
                        # Calculate distance using Pinhole Camera Math (Assumed human height: 1.8m)
                        box_height_pixels = y2 - y1
                        distance_z = (self.fy * 0.8) / box_height_pixels
                        
                        # Find the center pixel of the bounding box
                        center_x_pixel = (x1 + x2) / 2.0
                        center_y_pixel = (y1 + y2) / 2.0
                        
                        # Convert to camera lens coordinates
                        x_cam = (center_x_pixel - self.cx) * distance_z / self.fx
                        y_cam = (center_y_pixel - self.cy) * distance_z / self.fy
                        
                        # Broadcast into TF2 Tree
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = 'camera_rgb_optical_frame'
                        t.child_frame_id = 'detected_human'
                        t.transform.translation.x = float(x_cam)
                        t.transform.translation.y = float(y_cam)
                        t.transform.translation.z = float(distance_z)
                        t.transform.rotation.w = 1.0
                        self.tf_broadcaster.sendTransform(t)
                        
                        try:
                            rclpy.spin_once(self, timeout_sec=0.1)
                            trans = self.tf_buffer.lookup_transform('map', 'detected_human', rclpy.time.Time())
                            dynamic_x = trans.transform.translation.x
                            dynamic_y = trans.transform.translation.y
                            
                            self.human_found = True
                            self.human_pub.publish(Point(x=float(dynamic_x), y=float(dynamic_y), z=0.0))
                            self.get_logger().info(f'>>> AI CALCULATED COORDINATES: X={dynamic_x:.2f}, Y={dynamic_y:.2f} <<<')
                        except Exception:
                            pass
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AIDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
