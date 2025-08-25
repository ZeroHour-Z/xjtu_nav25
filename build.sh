#!/usr/bin/zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --parallel-workers 4 --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release
source ./install/setup.zsh
colcon build --symlink-install --parallel-workers 4 --packages-skip livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release