#!/bin/bash
echo "=== 部署雷达避障系统 ==="

# 1. 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. 克隆雷达驱动 (HesaiLidar_ROS_2.0)
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git

# 3. 克隆FAST-LIVO2
git clone https://github.com/hku-mars/FAST-LIVO2.git
cd FAST-LIVO2
git submodule update --init
cd ..

# 4. 复制本项目代码
cp -r /path/to/radar_obstacle_avoidance .

# 5. 安装依赖
sudo apt-get update
sudo apt-get install -y libboost-all-dev libyaml-cpp-dev libpcl-dev ros-noetic-pcl-ros

# 6. 编译
cd ~/catkin_ws
catkin_make -j$(nproc)

# 7. 配置环境
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "=== 部署完成 ==="
echo "请修改以下配置文件："
echo "1. $(find radar_obstacle_avoidance -name hesai_config.yaml) - 雷达IP和参数"
echo "2. $(find FAST-LIVO2 -name avia.yaml) - 确认lidar_topic为/lidar_points"
