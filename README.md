# Robotics-Midterm

Our navigation + goal state (main) combined with OpenCV/YOLO human detection.
Runs in the regular TurtleBot3 house world. No RGB-D launcher is used.

### One-time human detection setup
sudo apt update && sudo apt install -y python3-venv

python3 -m venv --system-site-packages ~/venvs/team2_vision

~/venvs/team2_vision/bin/python -m pip install 'numpy<2' 'onnxruntime==1.23.2'

python3 -m venv ~/venvs/team2_export

~/venvs/team2_export/bin/python -m pip install --upgrade pip

~/venvs/team2_export/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

~/venvs/team2_export/bin/python -m pip install 'ultralytics==8.3.221' 'onnx==1.17.0' 'numpy<2' 'opencv-python==4.11.0.86'

mkdir -p ~/human_model && cd ~/human_model

for m in yolov8n yolov8n-pose yolov8n-seg; do ~/venvs/team2_export/bin/yolo export model=${m}.pt format=onnx imgsz=640 batch=1 opset=17 simplify=False dynamic=False half=False nms=False device=cpu; done

ls -lh ~/human_model/*.onnx   # all three must exist (yolov8n-pose and yolov8n-seg are required)

### Install the professor's world (with the human at x=3.5, y=1.8)
sudo cp -n /opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world ~/turtlebot3_house.world.original

sudo cp turtlebot3_house_with_human.world /opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world

mkdir -p ~/team2_test_human && curl -fL https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11/media/models/walk.dae -o ~/team2_test_human/walk.dae

sudo cp -n ~/team2_test_human/walk.dae /opt/ros/humble/share/turtlebot3_gazebo/worlds/walk.dae

(The human's ground truth is (3.5, 1.8). /human_location should land within 1 m of it.)

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

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-4.5 y_pose:=3.5

### Check the camera topics (needed by human detection)
ros2 topic list | grep -i camera

Human detection needs a colour image, a DEPTH image and camera_info with the same
frame. Defaults are /camera/image_raw, /camera/depth/image_raw, /camera/camera_info.
If your robot uses other names, pass them to midterm.launch.py (see below).

### Terminal 2 Navigation, Goal state & Human detection
#### It has the explore.py, navigation_launch.py, navigation.py and human_location_node.py

cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

export TURTLEBOT3_MODEL=waffle

ros2 launch team2 midterm.launch.py

Optional arguments:

ros2 launch team2 midterm.launch.py vision:=false   # navigation only

ros2 launch team2 midterm.launch.py rgb_topic:=/my/rgb depth_topic:=/my/depth info_topic:=/my/info

### Terminal 3 Test Runs
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 topic pub -1 /goal1 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: 2.5, y: -1.0, z: 0.0}}"

ros2 topic pub -1 /goal2 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: -4.0, y: 1.0, z: 0.0}}"

ros2 topic pub -1 /goal3 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: 0.0, y: 0.0, z: 0.0}}"

### Human location and status
Human detection publishes /human_location (map frame). Navigation sets human_found and
human_location in /status, cancels all goals, stops the robot and stops exploring.
Restart midterm.launch.py for a new trial.

ros2 topic echo /status

ros2 topic echo /human_location
