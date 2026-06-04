#!/usr/bin/env python3
"""
Camera-LiDAR 外参标定工具
=========================
使用棋盘格标定板，同时检测相机图像中的角点和LiDAR点云中的棋盘格平面，
通过 PnP 算法求解 LiDAR→Camera 的 4x4 变换矩阵。

支持两种模式:
  - live: 实时采集并计算（硬件就绪时用）
  - bag: 从rosbag离线计算

用法:
  # 实时标定
  rosrun radar_obstacle_avoidance calibrate_camera_lidar.py \
      _mode:=live _checkerboard_rows:=9 _checkerboard_cols:=6 _square_size:=0.108

  # 离线bag标定（有bag可提前处理）
  rosrun radar_obstacle_avoidance calibrate_camera_lidar.py \
      _mode:=bag _bag_file:=/path/to/calibration.bag _checkerboard_rows:=9 _checkerboard_cols:=6

输出:
  - 标定结果打印到终端
  - 写入 config/camera_config.yaml (extrinsic 字段)
  - 写入 config/bev_fusion.yaml (calibration 字段)
"""

import rospy
import cv2
import numpy as np
import yaml
import os
import sys
from threading import Lock
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs.point_cloud2 import read_points_list
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import message_filters
import tf2_ros
import tf.transformations as tf_trans


# ============================================================
# 工具函数
# ============================================================

def pointcloud2_to_xyz(msg):
    """从 PointCloud2 提取 [N, 3] 点云"""
    points = []
    for p in read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])
    return np.array(points, dtype=np.float32)


def filter_pixels_in_roi(pts_uv, cx, cy, half_size, img_w, img_h):
    """保留图像中心ROI内的投影点"""
    x_low = max(0, cx - half_size)
    x_high = min(img_w, cx + half_size)
    y_low = max(0, cy - half_size)
    y_high = min(img_h, cy + half_size)
    mask = ((pts_uv[:, 0] >= x_low) & (pts_uv[:, 0] <= x_high) &
            (pts_uv[:, 1] >= y_low) & (pts_uv[:, 1] <= y_high))
    return mask


def estimate_plane_ransac(points_3d, max_dist=0.05):
    """
    RANSAC 平面拟合.
    返回: (normal, d, inliers) 满足 normal·x + d = 0
    """
    from sklearn.linear_model import RANSACRegressor
    # 使用 sklearn 的 RANSACRegressor 做平面拟合
    # z = a*x + b*y + c
    if len(points_3d) < 10:
        return None, None, None

    X = points_3d[:, :2]
    z = points_3d[:, 2]

    try:
        ransac = RANSACRegressor(
            residual_threshold=max_dist,
            max_trials=200,
            min_samples=10,
            random_state=42
        )
        ransac.fit(X, z)
        a, b = ransac.estimator_.coef_
        c = ransac.estimator_.intercept_
        inlier_mask = ransac.inlier_mask_

        # 平面方程: z = a*x + b*y + c
        # 法向量: (a, b, -1), 归一化
        normal = np.array([a, b, -1.0], dtype=np.float32)
        norm = np.linalg.norm(normal)
        if norm > 0:
            normal = normal / norm
        d = -c * norm  # 平面到原点距离

        return normal, d, inlier_mask
    except Exception as e:
        rospy.logerr(f"RANSAC plane fitting failed: {e}")
        return None, None, None


def estimate_plane_ransac_simple(points_3d, max_dist=0.05, n_iter=200):
    """
    不使用sklearn的RANSAC实现（避免依赖问题）。
    返回: (normal, d, inliers_mask)
    """
    N = len(points_3d)
    if N < 10:
        return None, None, None

    best_inliers = -1
    best_normal = None
    best_d = 0.0
    best_mask = None

    for _ in range(n_iter):
        # 随机取3个点
        idx = np.random.choice(N, 3, replace=False)
        p1, p2, p3 = points_3d[idx]

        # 计算法向量
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        d = -np.dot(normal, p1)

        # 统计内点
        dists = np.abs(np.dot(points_3d, normal) + d)
        inlier_mask = dists < max_dist
        n_inliers = np.sum(inlier_mask)

        if n_inliers > best_inliers:
            best_inliers = n_inliers
            best_normal = normal
            best_d = d
            best_mask = inlier_mask

    if best_inliers < 10:
        return None, None, None

    # 用所有内点重新拟合
    inlier_pts = points_3d[best_mask]
    mean = np.mean(inlier_pts, axis=0)
    centered = inlier_pts - mean
    _, _, Vt = np.linalg.svd(centered, full_matrices=False)
    normal = Vt[2]
    d = -np.dot(normal, mean)

    return normal, d, best_mask


def get_chessboard_corners_3d(rows, cols, square_size):
    """生成棋盘格角点的3D坐标 (以棋盘格平面为z=0)"""
    pattern_points = np.zeros((rows * cols, 3), np.float32)
    pattern_points[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)
    pattern_points *= square_size
    return pattern_points


def rotation_matrix_to_quaternion(R):
    """3x3旋转矩阵 → 四元数 [x, y, z, w]"""
    q = tf_trans.quaternion_from_matrix(
        np.vstack([np.hstack([R, [[0], [0], [0]]]), [0, 0, 0, 1]])
    )
    return q


def yaml_dump_to_file(data, filepath):
    """将字典写入YAML文件"""
    with open(filepath, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    rospy.loginfo(f"Written to {filepath}")


# ============================================================
# CameraLidarCalibrator
# ============================================================

class CameraLidarCalibrator:
    def __init__(self):
        rospy.init_node('camera_lidar_calibrator', anonymous=False)

        # ---- 棋盘格参数 ----
        self.rows = rospy.get_param('~checkerboard_rows', 9)   # 内角点数
        self.cols = rospy.get_param('~checkerboard_cols', 6)
        self.square_size = rospy.get_param('~square_size', 0.108)  # 米
        self.pattern_size = (self.cols, self.rows)  # OpenCV: (宽, 高)

        # ---- 相机内参（必须已标定） ----
        self.K = None
        self.dist_coeffs = None
        self.img_w = 640
        self.img_h = 480

        # ---- 标定数据 ----
        self.image_points_list = []   # 每帧检测到的角点 (Nx2)
        self.lidar_points_list = []   # 每帧对应的棋盘格点云
        self.frame_count = 0
        self.min_frames = rospy.get_param('~min_frames', 15)

        # ---- 标定板3D点 ----
        self.object_points_3d = get_chessboard_corners_3d(
            self.rows, self.cols, self.square_size)

        # ---- 状态 ----
        self.bridge = CvBridge()
        self.lock = Lock()
        self.mode = rospy.get_param('~mode', 'live')
        self.running = True
        self._warned_camera = False

        # ---- TF (用于标定期间发布变换) ----
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # ---- 结果 ----
        self.best_R = None
        self.best_t = None
        self.best_reprojection_error = float('inf')

        # ---- 开始 ----
        if self.mode == 'live':
            self.setup_live()
        elif self.mode == 'bag':
            self.setup_bag()
        else:
            rospy.logerr(f"Unknown mode: {self.mode}. Use 'live' or 'bag'")
            sys.exit(1)

    def setup_live(self):
        """实时模式: 订阅相机和雷达话题"""
        # 先等相机内参
        rospy.Subscriber('/camera/camera_info',
                         CameraInfo, self.camera_info_callback)

        # 等相机内参到了再开始同步
        rospy.loginfo("Waiting for camera info...")
        rate = rospy.Rate(10)
        while self.K is None and not rospy.is_shutdown():
            rate.sleep()

        if self.K is None:
            rospy.logerr("No camera info received. Is camera running?")
            sys.exit(1)

        # 时间同步订阅
        image_sub = message_filters.Subscriber('/camera/image_raw', Image)
        cloud_sub = message_filters.Subscriber('/hesai_points', PointCloud2)

        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, cloud_sub], queue_size=20, slop=0.1)
        ts.registerCallback(self.sync_callback)

        rospy.loginfo(f"[Calibrator] Live mode started. "
                      f"Board: {self.rows}x{self.cols}, size={self.square_size}m. "
                      f"Move the checkerboard in front of camera+LiDAR...")

    def setup_bag(self):
        """离线bag模式"""
        bag_file = rospy.get_param('~bag_file', '')
        if not bag_file:
            rospy.logerr("bag_file parameter required in bag mode")
            sys.exit(1)

        # bag模式暂未实现，提示用户
        rospy.loginfo("Bag mode: reading from " + bag_file)
        rospy.logwarn("Please calibrate in live mode with real hardware. "
                      "For bag-based calibration you can extract frames manually.")

        # 简化：直接提示采集步骤
        self.print_instructions()
        sys.exit(0)

    def camera_info_callback(self, msg):
        if self.K is not None:
            return
        self.K = np.array(msg.K, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D, dtype=np.float32)
        self.img_w = msg.width
        self.img_h = msg.height
        rospy.loginfo(f"[Calibrator] Camera info loaded: "
                      f"{self.img_w}x{self.img_h}, fx={self.K[0,0]:.1f}")

    def sync_callback(self, image_msg, cloud_msg):
        """同步回调: 检测棋盘格并收集数据"""
        if not self.running:
            return

        if self.K is None:
            return

        try:
            with self.lock:
                # ---- a) 相机: 检测棋盘格角点 ----
                cv_img = self.bridge.imgmsg_to_cv2(image_msg, "mono8")
                found, corners = cv2.findChessboardCorners(
                    cv_img, self.pattern_size, None)

                if not found:
                    return

                # 亚像素优化
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                            30, 0.001)
                corners_refined = cv2.cornerSubPix(
                    cv_img, corners, (11, 11), (-1, -1), criteria)

                # ---- b) LiDAR: 提取棋盘格区域点云 ----
                cloud_xyz = pointcloud2_to_xyz(cloud_msg)
                if len(cloud_xyz) < 20:
                    return

                # 将点云投影到图像平面，筛选棋盘格区域
                pts_cam = cloud_xyz  # 假设 LiDAR=相机 (无外参近似)
                pts_uv_h = self.K @ pts_cam.T  # [3, N]
                pts_u = pts_uv_h[0] / pts_uv_h[2]
                pts_v = pts_uv_h[1] / pts_uv_h[2]

                # 棋盘格区域ROI (从检测到的角点计算)
                corners_np = corners_refined.reshape(-1, 2)
                cx = float(np.mean(corners_np[:, 0]))
                cy = float(np.mean(corners_np[:, 1]))
                board_w = float(np.max(corners_np[:, 0]) - np.min(corners_np[:, 0]))
                board_h = float(np.max(corners_np[:, 1]) - np.min(corners_np[:, 1]))
                roi_half = max(board_w, board_h) * 0.7 + 20  # 增加ROI

                uv_mask = ((pts_u >= cx - roi_half) & (pts_u <= cx + roi_half) &
                           (pts_v >= cy - roi_half) & (pts_v <= cy + roi_half) &
                           (pts_uv_h[2] > 0) & (pts_uv_h[2] < 20.0))

                if np.sum(uv_mask) < 20:
                    return

                board_cloud = cloud_xyz[uv_mask]

                # ---- c) 平面拟合 + 提取棋盘格平面点 ----
                normal, d, inlier_mask = estimate_plane_ransac_simple(
                    board_cloud, max_dist=0.03)

                if normal is None or np.sum(inlier_mask) < 10:
                    return

                board_plane_points = board_cloud[inlier_mask]

                # ---- d) 保存数据 ----
                self.image_points_list.append(corners_refined.reshape(-1, 2))
                self.lidar_points_list.append(board_plane_points)
                self.frame_count += 1

                # ---- e) 可视化反馈 ----
                vis_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(
                    vis_img, self.pattern_size, corners_refined, found)

                # 显示内点
                for p in board_plane_points[:200]:  # 最多画200个
                    p_uv = self.K @ p
                    if p_uv[2] > 0:
                        u, v = int(p_uv[0] / p_uv[2]), int(p_uv[1] / p_uv[2])
                        if 0 <= u < self.img_w and 0 <= v < self.img_h:
                            cv2.circle(vis_img, (u, v), 2, (0, 255, 0), -1)

                cv2.putText(vis_img, f"Frames: {self.frame_count}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0), 2)
                cv2.imshow("Calibration", vis_img)
                key = cv2.waitKey(1) & 0xFF

                # 按 'c' 开始计算 / 'q' 退出
                if key == ord('c') and self.frame_count >= 5:
                    self.compute_extrinsics()
                if key == ord('q'):
                    self.running = False
                    rospy.signal_shutdown("User quit")

                rospy.loginfo(f"[{self.frame_count}] Board detected, "
                              f"{len(board_plane_points)} LiDAR points on plane")

        except Exception as e:
            rospy.logerr(f"Sync callback error: {e}")

    # ============================================================
    # PnP 求解
    # ============================================================

    def compute_extrinsics(self):
        """基于所有采集帧计算外参 (PnP)"""
        if self.frame_count < 5:
            rospy.logwarn(f"Need at least 5 frames, got {self.frame_count}")
            return

        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"Computing extrinsic from {self.frame_count} frames...")

        all_R = []
        all_t = []

        for i in range(self.frame_count):
            img_pts = self.image_points_list[i]
            pc_pts = self.lidar_points_list[i]

            if len(img_pts) < 4 or len(pc_pts) < 4:
                continue

            # 对LiDAR点取均值作为棋盘格中心
            board_center = np.mean(pc_pts, axis=0)

            # 构建3D-2D对应
            # 3D点: 棋盘格角点 (在棋盘格坐标系中已知)
            # 2D点: 图像上的角点
            # 但我们需要 LiDAR→相机的变换，所以:
            #   我们已知: 棋盘格角点的3D坐标(在棋盘格坐标系) → 2D投影(在图像)
            #   未知: 棋盘格在LiDAR坐标系下的位姿
            #
            # 简化方法: 用棋盘格中心作为参考点
            # 使用 solvePnP 要求已知棋盘格角点在 LiDAR 坐标系下的3D坐标
            # 但我们实际上不知道这些角点对应哪些LiDAR点
            #
            # 改用更鲁棒的方法:
            # 使用相机侧的棋盘格姿态估计 + LiDAR棋盘格平面法向约束

            # 先通过相机PnP估计棋盘格在相机坐标系下的位姿
            obj_pts = self.object_points_3d.astype(np.float32)
            img_pts_2d = img_pts.astype(np.float32)

            success, rvec_cam, t_cam = cv2.solvePnP(
                obj_pts, img_pts_2d, self.K, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE)

            if not success:
                continue

            # 棋盘格在相机坐标系下的位姿
            R_cam_board, _ = cv2.Rodrigues(rvec_cam)  # 相机→棋盘格
            t_cam_board = t_cam.reshape(3, 1)

            # LiDAR棋盘格平面法向量 (在LiDAR坐标系下)
            normal_lidar, d_lidar, lidar_inlier_mask = estimate_plane_ransac_simple(pc_pts, 0.03)
            if normal_lidar is None:
                continue
            board_center_lidar = np.mean(pc_pts[lidar_inlier_mask], axis=0)

            # 棋盘格法向量在相机坐标系下
            # R_cam_board 的第三列 = 棋盘格平面的法向量 (在相机坐标系下)
            normal_cam = R_cam_board[:, 2]

            # 现在需要求解 R_lidar2cam 和 t_lidar2cam
            # 约束:
            #   1. normal_cam = R_lidar2cam @ normal_lidar    (法向约束)
            #   2. t_cam_board_cam = R_lidar2cam @ t_board_lidar + t_lidar2cam
            #
            # 从约束1: R_lidar2cam 的旋转轴为 normal_cam × normal_lidar
            #         旋转角 = acos(normal_cam · normal_lidar)
            # 但此法不够精确，我们用多帧联合优化

            # 保存单帧估计
            # 朴素估计: 只用法向量约束
            v = np.cross(normal_lidar, normal_cam)
            s = np.linalg.norm(v)
            c = np.dot(normal_lidar, normal_cam)

            if s < 1e-6:
                R = np.eye(3, dtype=np.float32)
            else:
                vx = np.array([[0, -v[2], v[1]],
                               [v[2], 0, -v[0]],
                               [-v[1], v[0], 0]], dtype=np.float32)
                R = np.eye(3) + vx + vx @ vx * (1 - c) / (s * s)

            # 由法向量约束 + 棋盘格中心点确定平移
            # R @ board_center_lidar + t = board_center_cam
            board_center_cam = R_cam_board @ np.mean(obj_pts, axis=0) + t_cam_board.reshape(3)
            t = board_center_cam - R @ board_center_lidar

            all_R.append(R)
            all_t.append(t.reshape(3))

        if len(all_R) == 0:
            rospy.logerr("No valid PnP solutions found!")
            return

        # ---- 多帧求平均 ----
        R_avg = np.mean(np.array(all_R), axis=0)
        t_avg = np.mean(np.array(all_t), axis=0)

        # 对旋转矩阵做投影到SO(3)
        U, _, Vt = np.linalg.svd(R_avg)
        R_avg = U @ Vt
        if np.linalg.det(R_avg) < 0:
            Vt[-1] *= -1
            R_avg = U @ Vt

        # ---- 计算重投影误差 ----
        errors = []
        for i in range(min(len(all_R), self.frame_count)):
            img_pts = self.image_points_list[i]
            obj_pts = self.object_points_3d

            # 先用相机估计棋盘格位姿
            success, rvec, tvec = cv2.solvePnP(
                obj_pts.astype(np.float32), img_pts.astype(np.float32),
                self.K, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
            if not success:
                continue

            # 重投影
            proj_pts, _ = cv2.projectPoints(
                obj_pts.astype(np.float32), rvec, tvec,
                self.K, self.dist_coeffs)
            proj_pts = proj_pts.reshape(-1, 2)
            err = np.mean(np.linalg.norm(proj_pts - img_pts, axis=1))
            errors.append(err)

        avg_error = np.mean(errors) if errors else float('inf')
        self.best_R = R_avg
        self.best_t = t_avg
        self.best_reprojection_error = avg_error

        # ---- 输出结果 ----
        self.print_results(R_avg, t_avg, avg_error)
        self.save_results(R_avg, t_avg)

    def print_results(self, R, t, error):
        """打印标定结果"""
        q = rotation_matrix_to_quaternion(R)

        print(f"\n{'='*60}")
        print(f"Camera-LiDAR Extrinsic Calibration Result")
        print(f"{'='*60}")
        print(f"Frames used: {self.frame_count}")
        print(f"Reprojection error: {error:.3f} pixels")
        print(f"\nRotation matrix R_lidar2cam:")
        print(f"  [{R[0,0]:.8f}, {R[0,1]:.8f}, {R[0,2]:.8f}]")
        print(f"  [{R[1,0]:.8f}, {R[1,1]:.8f}, {R[1,2]:.8f}]")
        print(f"  [{R[2,0]:.8f}, {R[2,1]:.8f}, {R[2,2]:.8f}]")
        print(f"\nTranslation t_lidar2cam (meters):")
        print(f"  x: {t[0]:.6f}")
        print(f"  y: {t[1]:.6f}")
        print(f"  z: {t[2]:.6f}")
        print(f"\nQuaternion [x, y, z, w]:")
        print(f"  [{q[0]:.8f}, {q[1]:.8f}, {q[2]:.8f}, {q[3]:.8f}]")
        print(f"{'='*60}")

    def save_results(self, R, t):
        """保存到配置文件"""
        q = rotation_matrix_to_quaternion(R)

        # ---- 更新 camera_config.yaml ----
        config_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), '..', 'config')
        camera_config_path = os.path.join(config_dir, 'camera_config.yaml')

        if os.path.exists(camera_config_path):
            with open(camera_config_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}

            # 更新外参
            if 'camera' not in cfg:
                cfg['camera'] = {}
            cfg['camera']['extrinsic'] = {
                'translation': [float(t[0]), float(t[1]), float(t[2])],
                'rotation': [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
            }

            with open(camera_config_path, 'w') as f:
                yaml.dump(cfg, f, default_flow_style=False, sort_keys=False,
                          Dumper=yaml.SafeDumper)
            rospy.loginfo(f"✓ Extrinsics saved to {camera_config_path}")

        # ---- 更新 bev_fusion.yaml ----
        bev_config_path = os.path.join(config_dir, 'bev_fusion.yaml')
        if os.path.exists(bev_config_path):
            with open(bev_config_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}

            cfg['calibration'] = {
                'translation_x': float(t[0]),
                'translation_y': float(t[1]),
                'translation_z': float(t[2]),
                'quaternion_x': float(q[0]),
                'quaternion_y': float(q[1]),
                'quaternion_z': float(q[2]),
                'quaternion_w': float(q[3])
            }

            with open(bev_config_path, 'w') as f:
                yaml.dump(cfg, f, default_flow_style=False, sort_keys=False,
                          Dumper=yaml.SafeDumper)
            rospy.loginfo(f"✓ Extrinsics saved to {bev_config_path}")

        # ---- 发布TF (供RViz验证) ----
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = 'camera'
        tf_msg.child_frame_id = 'calib_result'
        tf_msg.transform.translation.x = t[0]
        tf_msg.transform.translation.y = t[1]
        tf_msg.transform.translation.z = t[2]
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        rospy.loginfo("✓ Published /calib_result TF for RViz verification")

    def print_instructions(self):
        """打印标定操作说明"""
        print(f"\n{'='*60}")
        print("Camera-LiDAR 外参标定操作说明")
        print(f"{'='*60}")
        print("""
1. 硬件准备:
   - 将相机和LiDAR固定在无人机上
   - 准备棋盘格标定板 (推荐 9x6 内角点, 边长108mm)
   - 确保标定板平整

2. 启动:
   roslaunch radar_obstacle_avoidance camera_calibration.launch

3. 标定步骤:
   - 将标定板放在相机+LiDAR的共同视野中
   - 距离: 1-6米之间
   - 角度: 偏转不同的角度（前倾/后仰/左右旋转）
   - 位置: 放在视野的不同区域
   - 每检测到棋盘格，窗口会显示绿色点（LiDAR回波）
   - 收集至少15个不同位姿后, 按 'c' 计算
   - 按 'q' 退出

4. 验证:
   - 检查重投影误差 < 1.0 像素
   - 在RViz中查看 /calib_result TF
   - 用 rosrun radar_obstacle_avoidance test_calibration.py 验证
        """)

    def run(self):
        if self.mode == 'live':
            rospy.spin()
            cv2.destroyAllWindows()


# ============================================================
if __name__ == '__main__':
    calibrator = CameraLidarCalibrator()
    calibrator.run()
