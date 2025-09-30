#!/usr/bin/zsh
source /opt/ros/humble/setup.zsh

# 检查总内存 (单位: KB)
MEM_KB=$(grep MemTotal /proc/meminfo | awk '{print $2}')
# 8GB的KB数
MEM_THRESHOLD_KB=$((8 * 1024 * 1024))

# 编译前置驱动
colcon build --symlink-install --parallel-workers 4 --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release

source ./install/setup.zsh

# 根据内存大小决定编译策略
if (( MEM_KB < MEM_THRESHOLD_KB )); then
    echo "系统内存小于8GB，为节省内存将分开编译各个雷达里程计..."
    # 由于faster_lio的编译需要消耗极多内存,不得已出此下策
    # 同时把faster_lio的-O3和调试全关了，不然电脑要爆swap了
    colcon build --symlink-install --parallel-workers 1 --packages-select faster_lio_ros2 point_lio_ros2 fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release 
else
    echo "系统内存大于等于8GB，将共同编译雷达里程计以提高效率..."
    # 内存充足，一起编译
    colcon build --symlink-install --parallel-workers 3 --packages-select faster_lio_ros2 point_lio_ros2 fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

# 这几个包基本不消耗什么内存,--parallel-workers可以开高一些
echo "编译其余软件包..."
colcon build --symlink-install --parallel-workers 4 --packages-skip livox_ros_driver2 faster_lio_ros2 point_lio_ros2 fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
