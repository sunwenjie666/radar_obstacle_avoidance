#!/usr/bin/env python3
"""
标定结果验证工具
================
使用已标定的外参，将LiDAR点云投影到相机图像上，检查对齐效果。
如果标定正确，LiDAR点云应该精确投影到对应的物体边缘。

用法:
  rosrun radar_obstacle_avoidance test_calibration.py

需要:
  - camera 和 LiDAR 都在运行
  - camera_config.yaml 中已有外参
  - bev_fusion.yaml 中已有外参
"""

import rospy
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs.point_cloud2 import read_points_list
from cv_bridge import CvBridge
from threading import Lock
import message_filters


def load_extrinsics_from_yaml():
    """从配置文件加载外参"""
    config_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config')

    # 先从 bev_fusion.yaml 读取
    bev_path = os.path.join(config_dir, 'bev_fusion.yaml')
    if os.path.exists(bev_path):
        with open(bev_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        calib = cfg.get('calibration', {})
        if calib.get('quaternion_w', 0) != 0:
            return calib

    # 再从 camera_config.yaml 读取
    cam_path = os.path.join(config_dir, 'camera_config.yaml')
    if os.path.exists(cam_path):
        with open(cam_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        extrinsic = cfg.get('camera', {}).get('extrinsic', {})
        trans = extrinsic.get('translation', [])
        rot = extrinsic.get('rotation', [])
        if len(trans) == 3 and len(rot) == 4:
            return {
                'translation_x': trans[0],
                'translation_y': trans[1],
                'translation_z': trans[2],
                'quaternion_x': rot[0],
                'quaternion_y': rot[1],
                'quaternion_z': rot[2],
                'quaternion_w': rot[3],
            }

    return None


def quaternion_to_rotation_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
    ], dtype=np.float32)


class CalibrationTester:
    def __init__(self):
        rospy.init_node('calibration_tester', anonymous=True)

        # 加载外参
        calib = load_extrinsics_from_yaml()
        if calib is None:
            rospy.logerr("No calibration data found! "
                         "Run calibrate_camera_lidar.py first.")
            rospy.logerr("Expected in config/bev_fusion.yaml or "
                         "config/camera_config.yaml")
            sys.exit(1)

        tx = calib['translation_x']
        ty = calib['translation_y']
        tz = calib['translation_z']
        qx = calib['quaternion_x']
        qy = calib['quaternion_y']
        qz = calib['quaternion_z']
        qw = calib['quaternion_w']

        R = quaternion_to_rotation_matrix([qx, qy, qz, qw])
        t = np.array([tx, ty, tz], dtype=np.float32).reshape(3, 1)
        self.T_lidar2cam = np.vstack([np.hstack([R, t]), [0, 0, 0, 1]])

        rospy.loginfo(f"[Test] Extrinsics loaded: "
                      f"t=({tx:.4f},{ty:.4f},{tz:.4f})")

        # 状态
        self.K = None
        self.bridge = CvBridge()
        self.lock = Lock()

        # 订阅相机内参
        rospy.Subscriber('/camera/camera_info', CameraInfo,
                         self.cam_info_callback)

        # 同步订阅
        image_sub = message_filters.Subscriber('/camera/image_raw', Image)
        cloud_sub = message_filters.Subscriber('/hesai_points', PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, cloud_sub], queue_size=5, slop=0.1)
        ts.registerCallback(self.sync_callback)

        rospy.loginfo("[Test] Waiting for sync data...")

    def cam_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.K, dtype=np.float32).reshape(3, 3)

    def sync_callback(self, image_msg, cloud_msg):
        if self.K is None:
            return

        try:
            with self.lock:
                # 解析点云
                points = []
                for p in read_points_list(cloud_msg, field_names=("x", "y", "z"),
                                          skip_nans=True):
                    points.append([p[0], p[1], p[2]])
                points = np.array(points, dtype=np.float32)

                if len(points) == 0:
                    return

                # 投影到图像
                N = points.shape[0]
                ones = np.ones((N, 1), dtype=np.float32)
                pts_h = np.hstack([points[:, :3], ones])
                pts_cam = (self.T_lidar2cam @ pts_h.T).T

                z = pts_cam[:, 2]
                valid = (z > 0.1) & (z < 30.0)
                if not np.any(valid):
                    return

                pts_cam_valid = pts_cam[valid]
                pts_uv = (self.K @ pts_cam_valid[:, :3].T).T
                u = pts_uv[:, 0] / pts_uv[:, 2]
                v = pts_uv[:, 1] / pts_uv[:, 2]

                # 绘制
                cv_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
                h_img, w_img = cv_img.shape[:2]

                in_frame = ((u >= 0) & (u < w_img) & (v >= 0) & (v < h_img))
                u, v, depth = u[in_frame], v[in_frame], z[valid][in_frame]

                # 根据距离着色 (近=红, 远=蓝)
                depth_norm = np.clip((depth - 1.0) / 20.0, 0, 1)
                for i in range(0, len(u), 3):  # 间隔采样避免过密
                    color = (
                        int(255 * (1 - depth_norm[i])),
                        0,
                        int(255 * depth_norm[i])
                    )
                    cv2.circle(cv_img, (int(u[i]), int(v[i])), 2, color, -1)

                cv2.putText(cv_img, f"LiDAR points: {len(u)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2)
                cv2.putText(cv_img, "Red=near, Blue=far",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1)

                cv2.imshow("Calibration Validation", cv_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.signal_shutdown("User quit")

        except Exception as e:
            rospy.logerr(f"Error: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys
    tester = CalibrationTester()
    tester.run()
