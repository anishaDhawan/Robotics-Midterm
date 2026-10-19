# Robotics-Midterm

### Build workspace
source /opt/ros/humble/setup.bash

cd ~/Robotics-Midterm/ros2_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select project_status team2

source install/setup.bash

### Terminal 1 Launch Gazebo
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

export TURTLEBOT3_MODEL=waffle

ros2 launch team2 rgbd_house.launch.py

This provides the RGB-D camera and `/team2` robot topics. Open the Gazebo window with `ign gazebo -g` in another terminal if needed. If your existing world already provides the camera and topics listed below, use that instead; do not launch a second world.

### Terminal 2 Navigation & Goal state
#### It has the explore.py, navigation_launch.py, navigation.py

It also starts human detection and depth obstacles. Complete the human detection setup below before starting this terminal.

cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

export TURTLEBOT3_MODEL=waffle

ros2 launch team2 midterm.launch.py

### Terminal 3 Test Runs
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 topic pub -1 /goal1 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: 2.5, y: -1.0, z: 0.0}}"

ros2 topic pub -1 /goal2 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: -4.0, y: 1.0, z: 0.0}}"

ros2 topic pub -1 /goal3 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: 0.0, y: 0.0, z: 0.0}}"

### Human detection setup

Run this once in Ubuntu. Internet access is needed to download the packages and models. No training or GPU is needed.

First, create the Python environment used when running the robot:

```bash
sudo apt update
sudo apt install -y python3-venv
python3 -m venv --system-site-packages ~/venvs/team2_vision
~/venvs/team2_vision/bin/python -m pip install 'numpy<2' 'onnxruntime==1.23.2'
```

Create a separate environment for exporting the models:

```bash
python3 -m venv ~/venvs/team2_export
~/venvs/team2_export/bin/python -m pip install --upgrade pip
~/venvs/team2_export/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
~/venvs/team2_export/bin/python -m pip install 'ultralytics==8.3.221' 'onnx==1.17.0' 'numpy<2' 'opencv-python==4.11.0.86'
```

Download the three official YOLOv8 models and export them into `~/human_model`. Copy this whole block:

```bash
mkdir -p ~/human_model
(
  cd ~/human_model || exit 1
  for model in yolov8n yolov8n-pose yolov8n-seg; do
    ~/venvs/team2_export/bin/yolo export model="${model}.pt" format=onnx imgsz=640 batch=1 opset=17 simplify=False dynamic=False half=False nms=False device=cpu || exit 1
  done
)
```

Ultralytics downloads each `.pt` file automatically if it is missing. Exporting converts the pretrained model into the ONNX format our node uses; it does not train a new model. The export version is pinned to keep the expected output format. See [Ultralytics ONNX export](https://docs.ultralytics.com/integrations/onnx/).

Check that all three exported files exist:

```bash
ls -lh ~/human_model/yolov8n.onnx ~/human_model/yolov8n-pose.onnx ~/human_model/yolov8n-seg.onnx
```

If any export fails, fix that error before launching the mission. The model files stay outside Git. After setup, use the Terminal 1 and Terminal 2 commands above; you do not need to export again each time.

### Camera and launch settings

For an existing camera setup, provide aligned colour/depth images and CameraInfo on `/team2/camera/image_raw`, `/team2/camera/depth/image_raw` and `/team2/camera/camera_info`, with matching image timestamps and optical frames. Robot topics include `/team2/scan`, `/team2/odom`, `/team2/cmd_vel`, `/team2/tf`, `/team2/tf_static` and `/team2/clock`.

The RGB-D launcher keeps the team's original starting position: `(-4.5, 3.5, yaw 0.0)`. The mission defaults match, so no extra origin arguments are needed.

If using a different starting position, set `odom_origin_x`, `odom_origin_y` and `odom_origin_yaw` on the mission launch to match where odometry originally started, not the robot's current position. Use `vision_python` and `model_dir` if your Python environment or model folder differs.

### Human location and status

Human detection sends the human_location topic. Navigation updates /status and stops the robot.

```bash
ros2 topic echo /status
```

### Sources

- [Ultralytics YOLOv8](https://docs.ultralytics.com/models/yolov8/): person, pose and segmentation models.
- [OpenCV](https://docs.opencv.org/4.x/): image processing.
- [ROS CameraInfo](https://docs.ros.org/en/humble/p/sensor_msgs/msg/CameraInfo.html), [tf2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html): depth projection and map coordinates.
- [ROBOTIS TurtleBot3 simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations): Waffle model and House world.

### This branch

This is the smaller vision integration. It keeps the team’s original DWB controller and frontier selection. Vision, depth obstacles, human-found stopping and team2 namespacing are included. This combined branch has not been run in Ubuntu yet.
