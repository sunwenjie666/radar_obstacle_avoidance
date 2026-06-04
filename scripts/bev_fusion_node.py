#!/usr/bin/env python3
"""
BEV Fusion Node
===============
核心融合节点: YOLOv8 2D检测 + FAST-LIVO2 LiDAR点云 → BEV变换 → 软关联 → 交叉注意力 → 3D障碍物

Topics:
  Sub:  /yolo/detections        (vision_msgs/Detection2DArray)
  Sub:  /fast_livo/cloud_registered  (sensor_msgs/PointCloud2)
  Sub:  /camera/camera_info     (sensor_msgs/CameraInfo)
  Pub:  /fusion/obstacles_3d    (radar_obstacle_avoidance/Obstacle3DArray)
  Pub:  /fusion/markers         (visualization_msgs/MarkerArray)
  Pub:  /fusion/bev_image       (sensor_msgs/Image)  [debug]

针对 Jetson Orin NX 8GB 优化, 目标 10-15Hz
"""

import rospy
import numpy as np
from threading import Lock

# ROS messages
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from sensor_msgs.point_cloud2 import read_points
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA, Header
from cv_bridge import CvBridge
import message_filters

# Custom messages
from radar_obstacle_avoidance.msg import Obstacle3D, Obstacle3DArray

import struct
import time
import math
import cv2


# ============================================================
# 工具函数
# ============================================================

def pointcloud2_to_numpy(msg, fields=('x', 'y', 'z', 'intensity')):
    """
    将 ROS PointCloud2 转为 numpy 数组 [N, 4]。
    不依赖 pcl_ros, 直接解析二进制buffer。
    """
    points_list = []
    for p in read_points(msg, field_names=fields, skip_nans=True):
        points_list.append([p[0], p[1], p[2], p[3] if len(p) > 3 else 0.0])
    return np.array(points_list, dtype=np.float32)


def build_projection_matrix(cam_info):
    """从 CameraInfo 构建 3x4 投影矩阵 P = K * [I|0]"""
    fx = cam_info.K[0]
    fy = cam_info.K[4]
    cx = cam_info.K[2]
    cy = cam_info.K[5]
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float32)
    return K


def quaternion_to_rotation_matrix(q):
    """四元数 [x, y, z, w] -> 3x3 旋转矩阵"""
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
    ], dtype=np.float32)


def create_cube_marker(obs, marker_id, frame_id, stamp):
    """创建单个3D边界框 Marker"""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "obstacles_3d"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = obs.x
    marker.pose.position.y = obs.y
    marker.pose.position.z = obs.z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = math.sin(obs.yaw / 2.0)
    marker.pose.orientation.w = math.cos(obs.yaw / 2.0)
    marker.scale.x = obs.width
    marker.scale.y = obs.length
    marker.scale.z = obs.height
    marker.color.r = 1.0
    marker.color.g = 0.2
    marker.color.b = 0.2
    marker.color.a = 0.6
    marker.lifetime = rospy.Duration(0.5)
    return marker


# ============================================================
# BEVFusionNode
# ============================================================

class BEVFusionNode:
    def __init__(self):
        rospy.init_node('bev_fusion_node', anonymous=False)

        # ---- 加载参数 ----
        self.load_parameters()

        # ---- 状态 ----
        self.K = None               # 相机内参矩阵 (3x3)
        self.camera_frame = 'camera'
        self.lidar_frame = 'livox_frame'
        self.img_width = 640
        self.img_height = 480
        self.lock = Lock()

        # ---- BEV网格缓存 (避免反复分配) ----
        self.grid_size = int(self.local_bev_size / self.bev_resolution)
        self._bev_grid = np.zeros((self.grid_size, self.grid_size, 3), dtype=np.float32)

        # ---- 性能统计 ----
        self.frame_count = 0
        self.total_time = 0.0
        self.last_log_time = time.time()

        # ---- 可视化 ----
        self.bridge = CvBridge()

        # ---- 订阅 ----
        self.cloud_sub = message_filters.Subscriber(
            '/fast_livo/cloud_registered', PointCloud2)
        self.detection_sub = message_filters.Subscriber(
            '/yolo/detections', Detection2DArray)
        self.cam_info_sub = rospy.Subscriber(
            '/camera/camera_info', CameraInfo, self.cam_info_callback)

        # 时间同步 (近似同步)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.cloud_sub, self.detection_sub],
            queue_size=10,
            slop=0.1  # 允许100ms偏差
        )
        self.ts.registerCallback(self.sync_callback)

        # ---- 发布 ----
        self.obstacle_pub = rospy.Publisher(
            '/fusion/obstacles_3d', Obstacle3DArray, queue_size=10)
        self.marker_pub = rospy.Publisher(
            '/fusion/markers', MarkerArray, queue_size=10)
        self.bev_image_pub = rospy.Publisher(
            '/fusion/bev_image', Image, queue_size=5)

        # ---- 外参 (标定后生效) ----
        self.T_lidar2cam = None  # 4x4 变换矩阵
        self.setup_extrinsics()

        rospy.loginfo("[BEVFusion] Node initialized. "
                      f"Local BEV: {self.local_bev_size}m @ {self.bev_resolution}m, "
                      f"Attention: {self.attention_enable}, "
                      f"Association: α={self.soft_alpha}, β={self.soft_beta}, γ={self.soft_gamma}")

    def load_parameters(self):
        """加载所有配置参数"""
        # BEV
        self.local_bev_size = rospy.get_param('~local_bev_size', 20.0)
        self.bev_resolution = rospy.get_param('~bev_resolution', 0.2)
        self.max_detections = rospy.get_param('~max_detections', 20)

        # 点云
        self.downsample_voxel = rospy.get_param('~downsample_voxel', 0.1)
        self.max_range = rospy.get_param('~max_range', 80.0)
        self.min_range = rospy.get_param('~min_range', 0.5)
        self.roi_margin = rospy.get_param('~roi_margin', 50)

        # 软关联
        self.soft_alpha = rospy.get_param('~soft_alpha', 0.5)
        self.soft_beta = rospy.get_param('~soft_beta', 0.3)
        self.soft_gamma = rospy.get_param('~soft_gamma', 0.2)
        self.sigma_img = rospy.get_param('~sigma_img', 50.0)
        self.sigma_3d = rospy.get_param('~sigma_3d', 2.0)
        self.min_assoc_points = rospy.get_param('~min_assoc_points', 3)

        # 注意力
        self.attention_enable = rospy.get_param('~attention_enable', True)
        self.attention_d_model = rospy.get_param('~attention_d_model', 32)
        self.attention_radius = rospy.get_param('~attention_radius', 5.0)
        self.attention_min_points = rospy.get_param('~attention_min_points', 5)

        # 深度
        self.depth_method = rospy.get_param('~depth_method', 'weighted_mean')

        # 过滤
        self.min_confidence = rospy.get_param('~min_confidence', 0.35)
        self.min_box_area = rospy.get_param('~min_box_area', 200)

        # 可视化
        self.publish_bev = rospy.get_param('~publish_bev_image', True)
        self.publish_markers = rospy.get_param('~publish_markers', True)

    def setup_extrinsics(self):
        """从参数加载外参"""
        try:
            tx = rospy.get_param('~calibration/translation_x', 0.0)
            ty = rospy.get_param('~calibration/translation_y', 0.0)
            tz = rospy.get_param('~calibration/translation_z', 0.0)
            qx = rospy.get_param('~calibration/quaternion_x', 0.0)
            qy = rospy.get_param('~calibration/quaternion_y', 0.0)
            qz = rospy.get_param('~calibration/quaternion_z', 0.0)
            qw = rospy.get_param('~calibration/quaternion_w', 1.0)

            R = quaternion_to_rotation_matrix([qx, qy, qz, qw])
            t = np.array([tx, ty, tz], dtype=np.float32).reshape(3, 1)
            self.T_lidar2cam = np.vstack([np.hstack([R, t]), [0, 0, 0, 1]])
            rospy.loginfo(f"[BEVFusion] Extrinsics loaded: t=({tx:.3f},{ty:.3f},{tz:.3f})")
        except Exception as e:
            rospy.logwarn(f"[BEVFusion] No extrinsics from params ({e}), will use TF")
            self.T_lidar2cam = None

    def cam_info_callback(self, msg):
        """获取相机内参"""
        if self.K is None:
            self.K = build_projection_matrix(msg)
            self.img_width = msg.width
            self.img_height = msg.height
            rospy.loginfo(f"[BEVFusion] Camera info received: "
                          f"{self.img_width}x{self.img_height}, fx={self.K[0,0]:.1f}")

    def sync_callback(self, cloud_msg, detection_msg):
        """同步回调: 融合主流程"""
        if self.K is None:
            rospy.logwarn_throttle(5.0, "[BEVFusion] Waiting for camera info...")
            return

        start_time = time.time()

        try:
            with self.lock:
                # ---- 1. 解析点云 ----
                points = self.parse_pointcloud(cloud_msg)
                if len(points) == 0:
                    return

                # ---- 2. 解析检测框 ----
                detections = self.parse_detections(detection_msg)
                if len(detections) == 0:
                    return

                # ---- 3-5. BEV + 软关联 + 注意力 ----
                obstacles = self.fuse_detections(points, detections, cloud_msg.header)

                # ---- 6. 发布 ----
                self.publish_results(obstacles, cloud_msg.header, detection_msg.header)

        except Exception as e:
            rospy.logerr(f"[BEVFusion] Error: {e}")

        # 性能统计
        elapsed = time.time() - start_time
        self.frame_count += 1
        self.total_time += elapsed
        if time.time() - self.last_log_time > 5.0:
            avg_ms = (self.total_time / self.frame_count) * 1000
            rospy.loginfo(f"[BEVFusion] FPS: {self.frame_count/5.0:.1f}, "
                          f"Avg latency: {avg_ms:.1f}ms")
            self.frame_count = 0
            self.total_time = 0.0
            self.last_log_time = time.time()

    # ----------------------------------------------------------
    # 1. 点云预处理
    # ----------------------------------------------------------
    def parse_pointcloud(self, cloud_msg):
        """解析并下采样点云"""
        points = pointcloud2_to_numpy(cloud_msg)
        if len(points) == 0:
            return points

        # 距离过滤
        dist = np.linalg.norm(points[:, :3], axis=1)
        mask = (dist >= self.min_range) & (dist <= self.max_range)
        points = points[mask]
        if len(points) == 0:
            return points

        # 体素下采样 (简单网格滤波)
        if self.downsample_voxel > 0:
            points = self.voxel_downsample(points, self.downsample_voxel)

        return points

    def voxel_downsample(self, points, voxel_size):
        """体素网格下采样"""
        if len(points) == 0:
            return points

        # 计算体素坐标
        voxel_coords = np.floor(points[:, :3] / voxel_size).astype(np.int32)

        # 使用 unique 获取每个体素中第一个点 (效率高于取均值)
        _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
        return points[unique_indices]

    # ----------------------------------------------------------
    # 2. 检测框解析
    # ----------------------------------------------------------
    def parse_detections(self, msg):
        """解析YOLO检测结果"""
        dets = []
        for det in msg.detections:
            if not det.results:
                continue
            score = det.results[0].score
            if score < self.min_confidence:
                continue

            cx = det.bbox.center.x
            cy = det.bbox.center.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            area = w * h
            if area < self.min_box_area:
                continue

            dets.append({
                'cx': cx, 'cy': cy, 'w': w, 'h': h,
                'class_id': det.results[0].id,
                'class_name': self.get_class_name(det.results[0].id),
                'score': score,
                'bbox': [cx - w/2, cy - h/2, cx + w/2, cy + h/2]  # x1,y1,x2,y2
            })

        # 按置信度排序，取前 max_detections 个
        dets.sort(key=lambda d: d['score'], reverse=True)
        return dets[:self.max_detections]

    def get_class_name(self, class_id):
        """获取类别名称 (与YOLO训练一致)"""
        names = {0: 'pedestrian', 1: 'people', 2: 'bicycle', 3: 'car',
                 4: 'van', 5: 'truck', 6: 'tricycle', 7: 'awning-tricycle',
                 8: 'bus', 9: 'motor'}
        return names.get(class_id, f'class_{class_id}')

    # ----------------------------------------------------------
    # 3-5. 核心融合: BEV + 软关联 + 交叉注意力
    # ----------------------------------------------------------
    def fuse_detections(self, points, detections, header):
        """
        对每个检测框执行融合流程:
          a) 投影点云到图像 → 筛选框内点
          b) 生成局部BEV网格
          c) 软关联距离加权
          d) 交叉注意力融合
          e) 输出3D障碍物
        """
        obstacles = []

        # 获取LiDAR→相机变换
        T = self.get_lidar_to_camera_transform()
        if T is None:
            T = np.eye(4, dtype=np.float32)

        # 将所有LiDAR点投影到图像平面 (一次完成, 避免重复投影)
        uvs, depths, valid_mask = self.project_points_to_image(points, T)

        if valid_mask is None or not np.any(valid_mask):
            return obstacles

        proj_points = points[valid_mask]
        proj_uvs = uvs[valid_mask]
        proj_depths = depths[valid_mask]

        for det_idx, det in enumerate(detections):
            try:
                obs = self.process_single_detection(
                    proj_points, proj_uvs, proj_depths, det, det_idx, T, header)
                if obs is not None:
                    obstacles.append(obs)
            except Exception as e:
                rospy.logwarn_throttle(10.0, f"[BEVFusion] Detection {det_idx} error: {e}")
                continue

        return obstacles

    def project_points_to_image(self, points, T_lidar2cam):
        """
        批量投影LiDAR点到图像平面
        返回: (uvs, depths, valid_mask)
        """
        if self.K is None:
            return None, None, None

        N = points.shape[0]

        # LiDAR点 → 齐次坐标 [N, 4]
        ones = np.ones((N, 1), dtype=np.float32)
        pts_h = np.hstack([points[:, :3], ones])  # [N, 4]

        # LiDAR → 相机坐标系
        pts_cam = (T_lidar2cam @ pts_h.T).T  # [N, 4]

        # 有效深度过滤
        z = pts_cam[:, 2]
        valid_z = (z > 0.1) & (z < self.max_range)

        if not np.any(valid_z):
            return None, None, valid_z

        # 投影到图像平面
        pts_cam_valid = pts_cam[valid_z]
        pts_uv = (self.K @ pts_cam_valid[:, :3].T).T  # [M, 3]
        u = pts_uv[:, 0] / pts_uv[:, 2]
        v = pts_uv[:, 1] / pts_uv[:, 2]

        # 有效像素范围
        valid_uv = (u >= 0) & (u < self.img_width) & (v >= 0) & (v < self.img_height)

        uvs = np.column_stack([u, v])
        depths = pts_cam_valid[:, 2]

        # 合并valid_z中的有效投影
        final_valid = np.zeros(N, dtype=bool)
        final_valid[valid_z] = valid_uv

        full_uvs = np.zeros((N, 2), dtype=np.float32)
        full_depths = np.zeros(N, dtype=np.float32)

        valid_indices = np.where(valid_z)[0][valid_uv]
        if len(valid_indices) > 0:
            full_uvs[valid_indices] = uvs[valid_uv]
            full_depths[valid_indices] = depths[valid_uv]

        return full_uvs, full_depths, final_valid

    def get_lidar_to_camera_transform(self):
        """获取 LiDAR→相机 4x4 变换矩阵"""
        if self.T_lidar2cam is not None:
            return self.T_lidar2cam
        # 若未标定, 返回单位阵 (用于调试)
        if not hasattr(self, '_warned_tf'):
            rospy.logwarn("[BEVFusion] No LiDAR-Camera extrinsics! Using identity. "
                          "Run calibration first.")
            self._warned_tf = True
        return np.eye(4, dtype=np.float32)

    def process_single_detection(self, points, uvs, depths, det, det_idx, T, header):
        """处理单个检测框的融合"""
        x1, y1, x2, y2 = det['bbox']

        # ---- a) 筛选框内LiDAR点 (带ROI margin) ----
        margin = self.roi_margin
        in_box = ((uvs[:, 0] >= x1 - margin) & (uvs[:, 0] <= x2 + margin) &
                  (uvs[:, 1] >= y1 - margin) & (uvs[:, 1] <= y2 + margin))

        box_points = points[in_box]
        box_uvs = uvs[in_box]
        box_depths = depths[in_box]

        if len(box_points) < self.min_assoc_points:
            return None

        # ---- b) 生成局部BEV网格 ----
        center_3d = np.median(box_points[:, :3], axis=0)
        bev_grid = self.build_local_bev(box_points[:, :3], center_3d)
        if bev_grid is None:
            return None

        # ---- c) 软关联: 计算每个点的关联权重 ----
        assoc_weights = self.soft_association(
            box_points[:, :3], box_uvs, box_depths, det, center_3d)

        # ---- d) 交叉注意力融合 ----
        if self.attention_enable and len(box_points) >= self.attention_min_points:
            fused_pos = self.cross_attention_fusion(
                box_points[:, :3], box_points[:, 3] if box_points.shape[1] > 3 else None,
                assoc_weights, det, center_3d)
        else:
            # 降级: 加权平均
            weights_sum = np.sum(assoc_weights)
            if weights_sum > 0:
                fused_pos = np.sum(
                    box_points[:, :3] * assoc_weights.reshape(-1, 1), axis=0) / weights_sum
            else:
                fused_pos = center_3d

        # ---- e) 构建3D障碍物 ----
        obs = self.build_obstacle(
            fused_pos, box_points[:, :3], assoc_weights, det, det_idx, header)

        return obs

    # ----------------------------------------------------------
    # 3b. 局部BEV网格
    # ----------------------------------------------------------
    def build_local_bev(self, points_3d, center):
        """
        以 center 为中心, 生成 local_bev_size 范围的BEV网格。
        网格: [grid_size, grid_size, 3] = (max_z, intensity_density, point_count)
        """
        self._bev_grid.fill(0)

        # 计算相对坐标
        rel = points_3d - center.reshape(1, 3)  # [N, 3]
        half_size = self.local_bev_size / 2.0

        # 范围过滤
        in_range = (np.abs(rel[:, 0]) < half_size) & (np.abs(rel[:, 1]) < half_size)
        rel = rel[in_range]

        if len(rel) == 0:
            return self._bev_grid

        # 网格坐标
        gx = ((rel[:, 0] + half_size) / self.bev_resolution).astype(np.int32)
        gy = ((rel[:, 1] + half_size) / self.bev_resolution).astype(np.int32)

        # 边界裁剪
        valid = (gx >= 0) & (gx < self.grid_size) & (gy >= 0) & (gy < self.grid_size)
        gx, gy = gx[valid], gy[valid]
        rel_valid = rel[valid]

        # 填充网格: channel 0 = max_z, channel 1 = density, channel 2 = count
        for i in range(len(gx)):
            cell = self._bev_grid[gx[i], gy[i]]
            z = rel_valid[i, 2]
            if z > cell[0]:
                cell[0] = z
            cell[1] += 1.0
            cell[2] += 1.0

        # density 归一化
        max_count = np.max(self._bev_grid[:, :, 1])
        if max_count > 0:
            self._bev_grid[:, :, 1] /= max_count

        return self._bev_grid

    # ----------------------------------------------------------
    # 3c. 软关联 (Soft Association)
    # ----------------------------------------------------------
    def soft_association(self, points_3d, uvs, depths, det, center_3d):
        """
        计算每个LiDAR点与检测框的软关联权重。

        weight = α · G_image + β · G_3d + γ · score

        其中 G_image = exp(-||uv - box_center||² / σ_img²)
             G_3d    = exp(-||p_3d - det_center_3d||² / σ_3d²)
        """
        # 图像空间距离
        box_cx, box_cy = det['cx'], det['cy']
        d_img = np.linalg.norm(uvs - np.array([box_cx, box_cy]), axis=1)
        w_img = np.exp(-0.5 * (d_img / self.sigma_img) ** 2)

        # 3D空间距离 (相对于3D质心)
        d_3d = np.linalg.norm(points_3d - center_3d.reshape(1, 3), axis=1)
        w_3d = np.exp(-0.5 * (d_3d / self.sigma_3d) ** 2)

        # 置信度 (对所有点都相同)
        w_conf = np.full(len(points_3d), det['score'])

        # 加权组合
        weights = (self.soft_alpha * w_img +
                   self.soft_beta * w_3d +
                   self.soft_gamma * w_conf)

        # 归一化到 [0, 1]
        if np.max(weights) > 0:
            weights = weights / np.max(weights)

        return weights

    # ----------------------------------------------------------
    # 3d. 交叉注意力 (Cross-Attention, numpy实现)
    # ----------------------------------------------------------
    def cross_attention_fusion(self, points_3d, intensities, assoc_weights, det, center_3d):
        """
        单头交叉注意力融合。

        Query:   检测框特征 [d_model]
                  = [class_onehot|box_geo|confidence]

        Keys:    LiDAR点特征 [N, d_model]
                  = [rel_xyz|intensity|distance_to_center|assoc_weight]

        Values:  LiDAR点3D位置 [N, 3]

        Output:  attention_weighted_3d_position [3]
        """
        N = len(points_3d)
        if N == 0:
            return center_3d
        if N < self.attention_min_points:
            # 点太少, 降级到加权平均
            w_sum = np.sum(assoc_weights)
            if w_sum > 0:
                return np.sum(points_3d * assoc_weights.reshape(-1, 1), axis=0) / w_sum
            return center_3d

        d = self.attention_d_model

        # ---- Build Query (检测级特征) ----
        # 类别 one-hot (10类)
        n_classes = 10
        class_onehot = np.zeros(n_classes, dtype=np.float32)
        if det['class_id'] < n_classes:
            class_onehot[det['class_id']] = 1.0

        # 检测框几何: [log(w), log(h), aspect_ratio]
        box_geo = np.array([
            math.log(max(det['w'], 1)),
            math.log(max(det['h'], 1)),
            det['w'] / max(det['h'], 1)
        ], dtype=np.float32)

        # 拼接 query
        query = np.concatenate([
            class_onehot,           # 10
            box_geo,                # 3
            np.array([det['score']], dtype=np.float32)  # 1
        ])  # total: 14

        # 投影到 d_model (使用固定种子保证可复现)
        if not hasattr(self, '_Wq'):
            rng = np.random.RandomState(42)
            self._Wq = rng.randn(d, len(query)).astype(np.float32) * 0.02
            self._Wk = rng.randn(d, 6).astype(np.float32) * 0.02
            self._scale = 1.0 / math.sqrt(d)

        q = self._Wq @ query  # [d_model]

        # ---- Build Keys (LiDAR点特征) ----
        # 每个点的特征: [rel_x, rel_y, rel_z, intensity(0), dist_to_center, assoc_weight]
        rel_pos = points_3d - center_3d.reshape(1, 3)
        dist_to_center = np.linalg.norm(rel_pos, axis=1)
        intensity = intensities if intensities is not None else np.zeros(N, dtype=np.float32)

        keys_feat = np.column_stack([
            rel_pos,
            intensity,
            dist_to_center,
            assoc_weights
        ])  # [N, 6]

        K = (self._Wk @ keys_feat.T).T  # [N, d_model]

        # ---- Compute Attention Scores ----
        # scores = q @ K^T * scale
        scores = np.dot(q, K.T) * self._scale  # [N]

        # 局部注意力掩码: 只关注半径内的点
        mask = dist_to_center < self.attention_radius
        scores[~mask] = -1e9

        # Softmax
        scores_exp = np.exp(scores - np.max(scores))
        attn_weights = scores_exp / (np.sum(scores_exp) + 1e-8)

        # ---- Weighted Sum of Values ----
        # Values = 3D positions [N, 3]
        values = points_3d  # [N, 3]
        fused_pos = np.sum(values * attn_weights.reshape(-1, 1), axis=0)

        return fused_pos

    # ----------------------------------------------------------
    # 3e. 构建3D障碍物
    # ----------------------------------------------------------
    def build_obstacle(self, fused_pos, points_3d, weights, det, det_idx, header):
        """输出3D障碍物消息"""
        obs = Obstacle3D()
        obs.header = header
        obs.id = det_idx
        obs.class_name = det['class_name']
        obs.class_id = det['class_id']
        obs.confidence = det['score']

        # 3D位置
        obs.x = fused_pos[0]
        obs.y = fused_pos[1]
        obs.z = fused_pos[2]

        # 3D尺寸 (从关联点云估算)
        w_sum = np.sum(weights)
        if w_sum > 0 and len(points_3d) > 1:
            # 加权协方差 → 主轴
            mean = np.average(points_3d, axis=0, weights=weights)
            centered = points_3d - mean.reshape(1, 3)
            cov = np.dot((centered * weights.reshape(-1, 1)).T, centered) / w_sum

            # 用特征值估算尺寸
            try:
                eigenvalues = np.linalg.eigvalsh(cov)
                eigenvalues = np.sqrt(np.maximum(eigenvalues, 0)) * 3.0  # 3σ
                eigenvalues = np.sort(eigenvalues)[::-1]
                obs.width = max(eigenvalues[0], 0.3)
                obs.length = max(eigenvalues[1] if len(eigenvalues) > 1 else obs.width, 0.3)
                obs.height = max(eigenvalues[2] if len(eigenvalues) > 2 else 0.5, 0.3)
            except np.linalg.LinAlgError:
                obs.width = 1.0
                obs.length = 1.0
                obs.height = 1.0
        else:
            obs.width = 1.0
            obs.length = 1.0
            obs.height = 1.0

        obs.yaw = 0.0  # 可后续用PCA计算

        # 关联信息
        obs.num_lidar_points = len(points_3d)
        obs.association_score = float(np.mean(weights))

        return obs

    # ----------------------------------------------------------
    # 6. 发布结果
    # ----------------------------------------------------------
    def publish_results(self, obstacles, cloud_header, detection_header):
        """发布3D障碍物列表和可视化"""
        # ---- Obstacle3DArray ----
        if self.obstacle_pub.get_num_connections() > 0:
            msg = Obstacle3DArray()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = cloud_header.frame_id
            msg.obstacles = obstacles
            self.obstacle_pub.publish(msg)

        # ---- MarkerArray ----
        if self.publish_markers and self.marker_pub.get_num_connections() > 0:
            markers = MarkerArray()
            for i, obs in enumerate(obstacles):
                markers.markers.append(
                    create_cube_marker(obs, i, cloud_header.frame_id,
                                       rospy.Time.now()))
            # 清理
            if len(markers.markers) > 0:
                markers.markers[-1].lifetime = rospy.Duration(0.5)
            self.marker_pub.publish(markers)

        # ---- BEV 可视化图像 ----
        if self.publish_bev and self.bev_image_pub.get_num_connections() > 0:
            self.publish_bev_image(obstacles, cloud_header.frame_id)

    def publish_bev_image(self, obstacles, frame_id):
        """将BEV网格渲染为图像 (调试用)"""
        grid_size = self.grid_size
        bev_img = np.zeros((grid_size, grid_size, 3), dtype=np.uint8)

        # 热力图渲染: 点密度
        if hasattr(self, '_bev_grid'):
            density = self._bev_grid[:, :, 1]
            # 映射到 [0, 255]
            norm = np.clip(density * 255, 0, 255).astype(np.uint8)
            bev_img[:, :, 0] = norm
            bev_img[:, :, 1] = norm
            bev_img[:, :, 2] = 255 - norm

        # 标注障碍物位置
        for obs in obstacles:
            half = grid_size / 2
            cx = int(half + obs.x / self.bev_resolution)
            cy = int(half + obs.y / self.bev_resolution)
            if 0 <= cx < grid_size and 0 <= cy < grid_size:
                r = max(3, int(obs.width / self.bev_resolution / 2))
                cv2.circle(bev_img, (cx, cy), r, (0, 0, 255), -1)
                cv2.putText(bev_img, obs.class_name[:3],
                            (cx - 10, cy - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.3, (255, 255, 255), 1)

        # 发布
        try:
            # 缩放到大图以便查看
            scale = 3
            bev_big = cv2.resize(bev_img, (grid_size * scale, grid_size * scale),
                                 interpolation=cv2.INTER_NEAREST)
            ros_img = self.bridge.cv2_to_imgmsg(bev_big, "bgr8")
            ros_img.header.stamp = rospy.Time.now()
            ros_img.header.frame_id = frame_id
            self.bev_image_pub.publish(ros_img)
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[BEVFusion] BEV image error: {e}")

    def run(self):
        """主循环"""
        rospy.spin()


# ============================================================
if __name__ == '__main__':
    node = BEVFusionNode()
    node.run()
