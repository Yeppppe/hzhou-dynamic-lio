#!/bin/bash

# 检查是否已经 source 了 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "请先 source ROS 环境！"
    exit 1
fi

# 设置 GTSAM 库路径（只需要设置一次，子进程会继承）
export LD_LIBRARY_PATH=/path/to/your/gtsam/install/lib:$LD_LIBRARY_PATH

# 打开第一个终端并运行 sr_lio
gnome-terminal -- bash -c "roslaunch sr_lio lio_ulhk_CA.launch; exec bash"

# 等待几秒确保第一个节点启动
sleep 3

# 打开第二个终端并运行 aloam_velodyne
gnome-terminal -- bash -c "roslaunch aloam_velodyne loop_closure.launch; exec bash"

# 等待几秒确保第二个节点启动
sleep 3

# 打开第三个终端并运行 aloam_velodyne
gnome-terminal -- bash -c "rosbag play /home/yep/datasets/ca/CA-20190828184706_blur_align-002.bag; exec bash"

echo "所有节点已启动！" 

