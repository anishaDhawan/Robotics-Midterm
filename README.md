
Olive Obianagha- Human Detection

Dhaval Pucha - Exploration

Anisha Dhawan - SLAM and goal state

Jianing Leng - Navigation

sudo cp ~/Robotics-Midterm/turtlebot3_house_with_human.world /opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world

### Install

sudo apt install ros-humble-turtlebot3* ros-humble-nav2* ros-humble-cartographer* -y

pip install ultralytics "setuptools==58.2.0"

### Build workspace

cd ~/Robotics-Midterm/ros2_ws

colcon build --packages-select team2

source install/setup.bash

### Terminal 1 Launch Gazebo
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

export TURTLEBOT3_MODEL=waffle

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-4.5 y_pose:=3.5

### Terminal 2 Navigation & Goal state

cd ~/Robotics-Midterm/ros2_ws

colcon build --packages-select team2

source install/setup.bash

export TURTLEBOT3_MODEL=waffle

ros2 launch team2 midterm.launch.py

### Terminal 3 Test Runs
cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 topic pub -1 /goal1 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: 4.0, y: 2.0, z: 0.0}}"

ros2 topic pub -1 /goal2 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: -4.0, y: 1.8, z: 0.0}}"

ros2 topic pub -1 /goal3 geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: 0.0, y: 0.0, z: 0.0}}"

### Terminal 4 Check Status

cd ~/Robotics-Midterm/ros2_ws

source install/setup.bash

ros2 topic echo /status
