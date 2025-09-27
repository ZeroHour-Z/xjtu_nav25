#!/usr/bin/zsh
source /opt/ros/humble/setup.zsh

# 由于faster_lio的编译需要消耗极多内存,不得已出此下策
# 同时把faster_lio的-O3和调试全关了，不然我自己电脑要爆swap了
P_WORKERS=$(( $(grep MemTotal /proc/meminfo | awk '{print $2}') / 1024 / 1024 / 7 ))
P_WORKERS=$(( P_WORKERS > 0 ? P_WORKERS : 1 ))
P_WORKERS_2=$((P_WORKERS * 2))
echo "Using --parallel-workers=$P_WORKERS"\

# 编译前置驱动
colcon build --symlink-install --parallel-workers 4 --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release

# 这几个激光雷达里程计框架的编译非常吃内存
source ./install/setup.zsh
colcon build --symlink-install --parallel-workers $P_WORKERS --packages-select faster_lio_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release 
colcon build --symlink-install --parallel-workers $P_WORKERS_2 --packages-select point_lio fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release 

# 这几个包基本不消耗什么内存,--parallel-workers随便开
colcon build --symlink-install --parallel-workers 4 --packages-skip livox_ros_driver2 faster_lio_ros2 point_lio fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release