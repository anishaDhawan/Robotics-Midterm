# Robotics-Midterm

## Build workspace
cd ~/Robotics-Midterm/ros2_ws
colcon build --packages-select team2
source install/setup.bash

## Launch Gazebo
cd ~/Robotics-Midterm/ros2_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-4.5 y_pose:=3.5

## SLAM Launch
cd ~/Robotics-Midterm/ros2_ws
source install/setup.bash
ros2 launch team2 navigation_launch.py

## Nav
cd ~/Robotics-Midterm/ros2_ws
source install/setup.bash
ros2 run team2 navigation


## TEST the bot
cd ~/Robotics-Midterm/ros2_ws
source install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.5, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}"


