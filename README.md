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




