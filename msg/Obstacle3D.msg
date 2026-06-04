# 3D障碍物消息
# 由BEV融合节点发布，融合YOLO检测和LiDAR点云

Header header

# 唯一ID
uint32 id

# 类别信息
string class_name    # 类别名称
uint8 class_id       # 类别ID
float32 confidence   # 置信度

# 3D位置 (LiDAR坐标系，单位: 米)
float32 x
float32 y
float32 z

# 3D尺寸 (单位: 米)
float32 width    # 宽度 (X轴方向)
float32 length   # 长度 (Y轴方向)
float32 height   # 高度 (Z轴方向)

# 朝向 (绕Z轴，单位: 弧度)
float32 yaw

# 速度 (单位: m/s，正=远离传感器)
float32 velocity_x
float32 velocity_y

# 软关联信息
int32 num_lidar_points    # 关联到的LiDAR点数
float32 association_score # 软关联评分 [0, 1]
