# 无人机视觉-LiDAR 融合感知系统

基于 **YOLOv8 + BEV变换 + 软关联 + 交叉注意力** 的视觉-LiDAR 融合感知系统，部署于 **Jetson Orin NX 8GB**，目标运行频率 10-15Hz。

---

## 系统架构

```
硬件输入
  Hesai XT32 LiDAR ──→ FAST-LIVO2 (里程计) ──→ /fast_livo/cloud_registered
  USB Camera         ──→ /camera/image_raw
                        /camera/camera_info

感知流水线
  2D图像 ──→ YOLOv8 ──→ norfair 跟踪 ──→ BEV 融合节点
                                              ├ LiDAR点→图像投影
LiDAR点云 ──────────────────────────────┘     ├ 局部BEV网格
                                              ├ 软关联权重
                                              ├ 交叉注意力
                                              └ 3D障碍物输出

ESDF建图
  3D障碍物 ──→ fiesta_bridge_node ──→ 注入虚拟点云 ──→ FIESTA ESDF
```

**关键话题流:**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/yolo/detections` | `Detection2DArray` | YOLO 2D检测结果 |
| `/tracked_objects` | `Detection2DArray` | norfair 跟踪结果 (含ID/速度) |
| `/fusion/obstacles_3d` | `Obstacle3DArray` | **最终输出**: 3D障碍物列表 |
| `/fusion/markers` | `MarkerArray` | RViz可视化 |
| `/fusion/bev_image` | `Image` | BEV热力图 (调试) |
| `/fusion/injected_cloud` | `PointCloud2` | 注入虚拟点的点云 → FIESTA |

---

## 文件结构

```
radar_obstacle_avoidance/
├── config/               # 配置文件 (YAML)
│   ├── bev_fusion.yaml   # BEV融合参数
│   └── camera_config.yaml# 相机内参+外参
├── launch/               # ROS launch 文件
│   ├── full_system.launch# 全系统启动
│   └── vision_system.launch# 视觉子系统
├── msg/                  # 自定义消息
│   ├── Obstacle3D.msg
│   └── Obstacle3DArray.msg
├── scripts/              # Python 节点
│   ├── yolo_detector_node.py    # YOLOv8 检测
│   ├── object_tracker_node.py   # norfair 跟踪
│   ├── bev_fusion_node.py       # BEV 融合 (核心)
│   ├── fiesta_bridge_node.py    # ESDF 桥接
│   ├── calibrate_camera_lidar.py# 外参标定
│   └── test_calibration.py      # 标定验证
├── src/                  # C++ 节点
└── models/               # YOLOv8 ONNX 模型
```

---

## 启动方式

### 完整系统 (硬件就绪)

```bash
roslaunch radar_obstacle_avoidance full_system.launch
```

启动全部模块: LiDAR驱动 + FAST-LIVO2 + YOLOv8 + norfair跟踪 + BEV融合 + FIESTA建图

### 视觉子系统 (单独调试)

```bash
roslaunch radar_obstacle_avoidance vision_system.launch
```

### 可选参数

```bash
# 关闭ESDF桥接 (纯LiDAR建图)
roslaunch radar_obstacle_avoidance full_system.launch enable_esdf_bridge:=false

# 不使用RTK-GPS
roslaunch radar_obstacle_avoidance full_system.launch enable_rtk:=false
```

---

## 标定

> ⚠️ 需要相机硬件安装到无人机上并与LiDAR刚性固定

### 相机内参标定

```bash
roslaunch radar_obstacle_avoidance camera_intrinsic_calibration.launch
rosrun camera_calibration cameracalibrator.py \
    --size 9x6 --square 0.108 \
    image:=/camera/image_raw camera:=/camera
```

### 相机-LiDAR 外参标定

```bash
roslaunch radar_obstacle_avoidance lidar_camera_calibration.launch
```

在窗口中移动棋盘格，按 `c` 计算外参，结果自动写入 `camera_config.yaml` 和 `bev_fusion.yaml`。

### 验证

```bash
rosrun radar_obstacle_avoidance test_calibration.py
```

---

## 核心算法

### BEV 变换

每个检测框独立生成 20m×20m 局部 BEV 网格 (0.2m分辨率, 100×100网格)。网格存储最大高度、点密度、计数。

### 软关联

```python
weight = α · G_img + β · G_3d + γ · score
```
- `G_img`: 图像空间高斯 (sigma=50px)
- `G_3d`: 3D空间高斯 (sigma=2m)
- `score`: 检测置信度

### 交叉注意力

单头 dot-product attention:
- Query: [class_onehot(10) | box_geo(3) | score(1)] → 14维
- Key: [rel_xyz(3) | intensity(1) | dist(1) | assoc_weight(1)] → 6维
- 局部注意力掩码: 半径 5m

---

## 依赖

- ROS Noetic
- OpenCV
- ONNX Runtime (CUDA)
- norfair (`pip3 install norfair`)
- FIESTA (ESDF建图)
- FAST-LIVO2 (里程计)
- Hesai XT32 LiDAR 驱动

---

## 许可证

本项目仅供个人学习和研究使用。
