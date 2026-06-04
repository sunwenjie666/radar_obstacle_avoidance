#!/bin/bash
echo "=== 安装视觉系统依赖 ==="

# 1. 安装Python依赖
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv python3-numpy

# 2. 安装ROS视觉相关包
sudo apt-get install -y ros-noetic-vision-msgs \
                       ros-noetic-cv-bridge \
                       ros-noetic-image-transport \
                       ros-noetic-camera-info-manager \
                       ros-noetic-image-geometry

# 3. 安装RealSense驱动（如果需要）
# sudo apt-get install -y ros-noetic-realsense2-camera
# 或者
# git clone https://github.com/IntelRealSense/realsense-ros.git

# 4. 安装ONNX Runtime GPU版本（Jetson专用）
# 注意：根据您的Jetson版本选择正确的wheel
# 访问：https://elinux.org/Jetson_Zoo#ONNX_Runtime

# 对于Jetson Orin NX（JetPack 5.1.1）
wget https://nvidia.box.com/shared/static/0zzlnc6e2dbm1h9h3n8vw7w0x6lxyvuj.whl -O onnxruntime_gpu-1.15.1-cp38-cp38-linux_aarch64.whl
pip3 install onnxruntime_gpu-1.15.1-cp38-cp38-linux_aarch64.whl

# 5. 安装消息过滤和TF2
sudo apt-get install -y ros-noetic-message-filters \
                       ros-noetic-tf2-sensor-msgs

# 6. 创建模型目录
mkdir -p ~/catkin_ws/src/radar_obstacle_avoidance/models
echo "请将YOLOv8 ONNX模型复制到: ~/catkin_ws/src/radar_obstacle_avoidance/models/"

echo "=== 视觉依赖安装完成 ==="
