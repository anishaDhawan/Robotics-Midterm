"""Use matching colour and depth images to find the human's location on the map.
Use TF to account for where the camera was when the images were taken.
Publish the confirmed location.
"""
import time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
import message_filters
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from team2.fused_depth import FusedDepth, Confirmation

class HumanLocationNode(Node):

    def __init__(self):
        super().__init__('human_location_node')
        self.declare_parameter('model_dir', str(Path.home() / 'human_model'))
        self.detector = FusedDepth(self.get_parameter('model_dir').value)
        self.confirm = Confirmation()
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointStamped, 'human_location', 10)
        # Receive colour images, depth images matched to them, and the camera settings.
        self.subscribers = [message_filters.Subscriber(self, t, topic, qos_profile=qos_profile_sensor_data) for (t, topic) in [(Image, 'camera/image_raw'), (Image, 'camera/depth/image_raw'), (CameraInfo, 'camera/camera_info')]]
        # Only use colour and depth images taken at the same time.
        self.sync = message_filters.TimeSynchronizer(self.subscribers, 10)
        self.sync.registerCallback(self.receive)
        # Let ROS keep receiving messages while YOLO processes the image in the background.
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.accepted = None
        self.create_timer(0.1, self.finish)
        self.create_timer(1.0, self.repeat_accepted)
        self.get_logger().info('Waiting for aligned RGB-D and image-time map TF; only accepted locations are published.')

    def receive(self, rgb, dep, info):
        """Check the camera images and save where the camera was on the map before running YOLO."""
        if self.accepted is not None or self.future is not None:
            return
        if not rgb.header.frame_id or not rgb.header.frame_id == dep.header.frame_id == info.header.frame_id:
            return
        if not self.buffer.can_transform('map', rgb.header.frame_id, Time.from_msg(rgb.header.stamp)):
            return
        depth = self.bridge.imgmsg_to_cv2(dep, 'passthrough').astype(np.float32)
        # Change depth from millimetres to metres when needed.
        if dep.encoding == '16UC1':
            depth *= 0.001
        elif dep.encoding != '32FC1':
            return
        image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        # Use where the camera was when this image was taken.
        self.transform = self.buffer.lookup_transform('map', rgb.header.frame_id, Time.from_msg(rgb.header.stamp))
        self.header = rgb.header
        self.started = time.monotonic()
        self.future = self.pool.submit(self.detector.estimate, image, depth, info)

    def finish(self):
        """Find the selected point's position on the map and send it after several observations give similar locations."""
        if self.future is None or not self.future.done():
            return
        future = self.future
        self.future = None
        try:
            (choice, _) = future.result()
            if choice is None:
                self.confirm.reset()
                return
            p = PointStamped()
            p.header = self.header
            (p.point.x, p.point.y, p.point.z) = map(float, choice['selected']['xyz'])
            # Use TF to find where this camera point is on the map.
            mapped = tf2_geometry_msgs.do_transform_point(p, self.transform)
            mapped.header.frame_id = 'map'
            mapped.header.stamp = self.header.stamp
            stamp = self.header.stamp.sec + self.header.stamp.nanosec * 1e-09
            now = self.get_clock().now().nanoseconds / 1000000000.0
            # Skip results if the image is too old or simulation time has moved backwards.
            if now - stamp > 5 or now < stamp:
                self.confirm.reset()
                return
            accepted = self.confirm.add([mapped.point.x, mapped.point.y, mapped.point.z], stamp, choice['source'])
            if accepted:
                self.accepted = mapped
                self.publisher.publish(mapped)
                self.get_logger().info(f'Accepted map location: {mapped.point.x:.3f}, {mapped.point.y:.3f}')
        except (ValueError, RuntimeError, TransformException) as exc:
            self.confirm.reset()
            self.get_logger().error(str(exc))

    def repeat_accepted(self):
        """Send the confirmed human location every second, keeping the time of the original camera image."""
        if self.accepted is not None:
            self.publisher.publish(self.accepted)

    def destroy_node(self):
        self.pool.shutdown(wait=True)
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HumanLocationNode()
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
