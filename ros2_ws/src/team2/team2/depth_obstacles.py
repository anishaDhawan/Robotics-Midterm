"""Turn the simulated RGB-D camera's depth image into obstacle observations.

This node does not drive the robot. Nav2 must be configured separately to use
camera/obstacles for marking and camera/clearing_points for clearing.
Assumes an undistorted depth image, matching CameraInfo, and a flat floor at
z=0 in base_footprint. It does not detect holes, stairs, or hidden obstacles.

References:
https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg
https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/observation_buffer.cpp
"""

import numpy as np
import message_filters
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


def depth_to_points(depth, projection, step, near, far):
    """Use depth and camera calibration to find 3D points relative to the camera."""
    fx, fy, cx, cy = projection[0], projection[5], projection[2], projection[6]
    if not np.all(np.isfinite(projection)) or fx <= 0 or fy <= 0:
        raise ValueError('Camera calibration is missing or invalid.')
    # This camera has no stereo offset. Other projection layouts need adapting.
    expected = np.array([fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0])
    if not np.allclose(projection, expected):
        raise ValueError('Expected a rectified camera with zero stereo offset.')
    v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    z = depth[::step, ::step]
    # Missing readings are unknown space, not evidence that a path is clear.
    valid = np.isfinite(z) & (z >= near) & (z <= far)
    z, u, v = z[valid], u[valid], v[valid]
    points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
    return points[np.linalg.norm(points, axis=1) <= far]


def rotation_matrix(quaternion):
    """Turn TF's camera orientation into a matrix that rotates our points."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or norm < 1e-8:
        raise ValueError('Invalid camera orientation.')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def separate_points(camera_points, rotation, translation, minimum_height,
                    maximum_height, half_length, half_width):
    """Remove the robot itself, then separate obstacles from floor readings."""
    body_points = camera_points @ rotation.T + translation
    # Ignore returns inside the Waffle's body so it does not block its own path.
    # These rectangular bounds are configurable and need checking on the robot.
    outside_robot = ((np.abs(body_points[:, 0]) > half_length)
                     | (np.abs(body_points[:, 1]) > half_width))
    height = body_points[:, 2]
    obstacles = (outside_robot & (height >= minimum_height)
                 & (height <= maximum_height))
    # Keep camera coordinates in both outputs. Nav2 then knows that each
    # measurement started at the camera, not at the centre of the robot.
    return camera_points[obstacles], camera_points[outside_robot]


def make_cloud(header, points):
    """Pack XYZ coordinates into the standard point-cloud message Nav2 accepts."""
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [PointField(name=name, offset=offset,
                               datatype=PointField.FLOAT32, count=1)
                    for name, offset in [('x', 0), ('y', 4), ('z', 8)]]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud


class DepthObstacles(Node):
    """Read each matching depth frame and publish current obstacle observations."""

    def __init__(self):
        super().__init__('depth_obstacles')
        defaults = {
            'ground_frame': 'base_footprint',
            'pixel_step': 2,
            'minimum_range': 0.10,
            'maximum_range': 3.5,
            'minimum_height': 0.02,
            'maximum_height': 1.8,
            'robot_half_length': 0.25,
            'robot_half_width': 0.20,
            'maximum_image_age': 0.5,
        }
        # Keep these limits as ROS parameters so they can be changed at launch.
        self.settings = {key: self.declare_parameter(key, value).value
                         for key, value in defaults.items()}
        s = self.settings
        if (not s['ground_frame'] or s['pixel_step'] < 1
                or not all(np.isfinite(s[k]) for k in defaults
                           if k != 'ground_frame')
                or not 0 < s['minimum_range'] < s['maximum_range']
                or not 0 < s['minimum_height'] < s['maximum_height']
                or s['robot_half_length'] <= 0 or s['robot_half_width'] <= 0
                or s['maximum_image_age'] <= 0):
            raise ValueError('Invalid obstacle detection settings.')
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # Send obstacle points and clearing points separately so Nav2 can use both.
        self.obstacles = self.create_publisher(PointCloud2, 'camera/obstacles', 10)
        self.clearing = self.create_publisher(PointCloud2, 'camera/clearing_points', 10)
        self.subscribers = [
            message_filters.Subscriber(self, Image, 'camera/depth/image_raw',
                                       qos_profile=qos_profile_sensor_data),
            message_filters.Subscriber(self, CameraInfo, 'camera/camera_info',
                                       qos_profile=qos_profile_sensor_data),
        ]
        # Match calibration to the depth frame so pixel positions mean the same thing.
        self.sync = message_filters.TimeSynchronizer(self.subscribers, 5)
        self.sync.registerCallback(self.receive)
        self.get_logger().info('Depth obstacle observations only; Nav2 connection is still required.')

    def receive(self, depth_message, info):
        """Check the image, calculate points, and send both clouds with its timestamp."""
        s = self.settings
        try:
            header = depth_message.header
            if not header.frame_id or header.frame_id != info.header.frame_id:
                raise ValueError('Depth and calibration must use the same optical frame.')
            stamp = Time.from_msg(header.stamp)
            age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            # Old observations can put a moving obstacle where it used to be.
            if age < 0 or age > s['maximum_image_age']:
                raise ValueError('Depth frame is stale or from a different clock.')
            if (depth_message.width != info.width or depth_message.height != info.height
                    or info.binning_x > 1 or info.binning_y > 1
                    or info.roi.x_offset or info.roi.y_offset
                    or info.roi.width not in (0, info.width)
                    or info.roi.height not in (0, info.height)):
                raise ValueError('Depth image must match the full calibration image.')
            if not np.all(np.isfinite(info.d)) or np.any(np.abs(info.d) > 1e-8):
                raise ValueError('This version expects the undistorted simulation camera.')
            depth = self.bridge.imgmsg_to_cv2(depth_message, 'passthrough').astype(np.float32)
            if depth_message.encoding == '16UC1':
                depth *= 0.001  # Convert millimetres to metres.
            elif depth_message.encoding != '32FC1':
                raise ValueError('Expected 16UC1 or 32FC1 depth.')
            if depth.shape != (info.height, info.width):
                raise ValueError('Unexpected depth image dimensions.')
            points = depth_to_points(depth, info.p, s['pixel_step'],
                                     s['minimum_range'], s['maximum_range'])
            if not len(points):
                raise ValueError('No usable depth readings; do not assume the path is clear.')
            # Measure height above the floor using the camera's pose at image time.
            transform = self.buffer.lookup_transform(s['ground_frame'], header.frame_id, stamp)
            t, q = transform.transform.translation, transform.transform.rotation
            translation = np.array([t.x, t.y, t.z])
            if not np.all(np.isfinite(translation)):
                raise ValueError('Invalid camera position.')
            rotation = rotation_matrix([q.x, q.y, q.z, q.w])
            obstacles, clearing = separate_points(
                points, rotation, translation, s['minimum_height'], s['maximum_height'],
                s['robot_half_length'], s['robot_half_width'])
            if not len(clearing):
                raise ValueError('Only robot-body returns; no usable view of the surroundings.')
            # Publish an empty obstacle cloud when we see only clear floor.
            # Clearing still needs real depth readings; an empty cloud alone clears nothing.
            self.clearing.publish(make_cloud(header, clearing))
            self.obstacles.publish(make_cloud(header, obstacles))
        except (ValueError, CvBridgeError, TransformException) as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacles()
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
