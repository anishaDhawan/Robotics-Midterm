# Robotics-Midterm

### Build workspace
cd ~/Robotics-Midterm/ros2_ws

colcon build --packages-select team2

source install/setup.bash

### Terminal 1 Launch Gazebo
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-4.5 y_pose:=3.5

### Terminal 2 SLAM Launch
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 launch team2 navigation_launch.py

### Terminal 3 Nav
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 run team2 navigation


### Terminal 4 professor grade checker
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 topic echo /status

### Terminal 5 Send out Goals
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 topic pub --once /goal1 geometry_msgs/msg/Point "{x: 2.5, y: -1.0, z: 0.0}"

ros2 topic pub --once /goal2 geometry_msgs/msg/Point "{x: 1.5, y: -1.5, z: 0.0}"

ros2 topic pub --once /goal3 geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 0.0}"





# Human detection and location

Uses YOLO and an RGB-D camera to find a point on the human and publish its location on the map.

### Setup (once)

Requires ROS2 Humble and Gazebo Fortress. The vision files are included in the existing `team2` package.

```bash
source /opt/ros/humble/setup.bash
cd ~/Robotics-Midterm/ros2_ws
rosdep install --from-paths src/team2 --ignore-src -r -y
python3 -m venv --system-site-packages ~/venvs/team2_vision
~/venvs/team2_vision/bin/pip install 'numpy<2' 'onnxruntime==1.23.2'
colcon build --symlink-install --packages-select team2
source install/setup.bash
```

Put these models in `~/human_model` (model files are not included in Git):
- `yolov8n.onnx`
- `yolov8n-pose.onnx`
- `yolov8n-seg.onnx`

Use the official YOLOv8 weights and [export to ONNX](https://docs.ultralytics.com/integrations/onnx/) with `imgsz=640`, `opset=17`, `simplify=False`, `dynamic=False`, `half=False`, and `nms=False`.

### Terminal 1 — RGB-D House world

Use this launcher if your existing House simulation does not already provide the RGB-D camera topics needed by this package. It replaces your current House launch command—do not run both together.

```bash
source /opt/ros/humble/setup.bash
source ~/Robotics-Midterm/ros2_ws/install/setup.bash
ros2 launch team2 rgbd_house_test.launch.py
```

This starts the simulation server. Open the Gazebo window separately with `ign gazebo -g` if needed.

### Before starting detection

SLAM/localization must be running and provide the camera-to-map TF connection. This vision setup uses `/team2/tf`, `/team2/tf_static`, and `/team2/clock`; these must connect to the running navigation setup.
### Terminal 2 — Human detection

```bash
source /opt/ros/humble/setup.bash
source ~/Robotics-Midterm/ros2_ws/install/setup.bash
~/venvs/team2_vision/bin/python -m team2.human_location_node --ros-args \
  -r __ns:=/team2 -r /tf:=/team2/tf -r /tf_static:=/team2/tf_static \
  -r /clock:=/team2/clock -p use_sim_time:=true
```

The camera must publish matching colour/depth images and calibration on `/team2/camera/image_raw`, `/team2/camera/depth/image_raw`, and `/team2/camera/camera_info`.

### Terminal 3 — Check the location

```bash
source /opt/ros/humble/setup.bash
source ~/Robotics-Midterm/ros2_ws/install/setup.bash
ros2 topic echo /team2/human_location geometry_msgs/msg/PointStamped
```

The output is in the `map` frame. Three observations must agree within 30 cm before publishing. The confirmed location is then repeated every second. Restart detection for a new trial.


### Sources

- [Ultralytics YOLOv8](https://docs.ultralytics.com/models/yolov8/), [pose](https://docs.ultralytics.com/tasks/pose/), and [segmentation](https://docs.ultralytics.com/tasks/segment/).
- [Ultralytics mask processing](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/utils/ops.py).
- [ROS camera calibration](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg) and [TF point transformation](https://github.com/ros2/geometry2/blob/humble/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py).
- [ROBOTIS TurtleBot3 simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) — camera model adaptations, Apache-2.0. Ultralytics weights have their own license.
