#!/usr/bin/env python3
"""
FIESTA ESDF Bridge
==================
将 BEV 融合检测到的 3D 障碍物以虚拟点云形式注入 FIESTA 的点云流，
使 FIESTA 的 ESDF 地图包含动态障碍物信息，供路径规划器使用。

原理:
  每个 Obstacle3D 障碍物在其包围盒内生成 N 个虚拟 LiDAR 点，
  合并到原始点云后发布。FIESTA 通过 Raycasting 将这些点标记为占据，
  从而在 ESDF 中形成障碍物区域。

Topics:
  Sub: /fast_livo/cloud_registered   (sensor_msgs/PointCloud2)  — 原始LiDAR点云
  Sub: /fusion/obstacles_3d          (Obstacle3DArray)          — BEV融合障碍物
  Pub: /fusion/injected_cloud        (sensor_msgs/PointCloud2)  — 注入后的点云
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from radar_obstacle_avoidance.msg import Obstacle3DArray
import struct
import time


class FiestaESDFBridge:
    def __init__(self):
        rospy.init_node('fiesta_esdf_bridge', anonymous=False)

        # ---- 参数 ----
        self.points_per_m3 = rospy.get_param('~points_per_m3', 20.0)
        self.max_inject_points = rospy.get_param('~max_inject_points', 800)
        self.min_confidence = rospy.get_param('~min_confidence', 0.35)
        self.min_num_pts = rospy.get_param('~min_lidar_points', 3)
        self.use_original_cloud = rospy.get_param('~use_original_cloud', True)

        # ---- 最近一次障碍物 ----
        self.latest_obstacles = []
        self.obstacle_timestamp = rospy.Time(0)
        self.lock = rospy.Lock()

        # ---- 随机生成器 (固定种子, 保证可复现) ----
        self.rng = np.random.RandomState(42)

        # ---- 订阅障碍物 ----
        rospy.Subscriber('/fusion/obstacles_3d', Obstacle3DArray,
                         self.obstacle_callback, queue_size=5)

        # ---- 订阅原始点云 (用于透传) ----
        self.cloud_sub = rospy.Subscriber(
            '/fast_livo/cloud_registered', PointCloud2,
            self.cloud_callback, queue_size=5)

        # ---- 发布注入后的点云 ----
        self.injected_pub = rospy.Publisher(
            '/fusion/injected_cloud', PointCloud2, queue_size=5)

        # ---- 性能统计 ----
        self.frame_count = 0
        self.total_inject = 0
        self.last_log_time = time.time()

        rospy.loginfo(f"[FiestaBridge] Started. "
                      f"pts/m³={self.points_per_m3}, "
                      f"max_inject={self.max_inject_points}")

    def obstacle_callback(self, msg):
        """存储最新的障碍物列表"""
        with self.lock:
            self.latest_obstacles = [
                obs for obs in msg.obstacles
                if (obs.confidence >= self.min_confidence and
                    obs.num_lidar_points >= self.min_num_pts)
            ]
            self.obstacle_timestamp = msg.header.stamp

    def generate_obstacle_points(self, obstacles):
        """
        为每个障碍物在包围盒内生成虚拟点。
        返回 [N, 3] 的点云数组。
        """
        if not obstacles:
            return np.empty((0, 3), dtype=np.float32)

        all_points = []

        for obs in obstacles:
            # 在障碍物包围盒内均匀采样
            dx = max(obs.width, 0.3)
            dy = max(obs.length, 0.3)
            dz = max(obs.height, 0.3)

            volume = dx * dy * dz
            n_points = max(5, int(volume * self.points_per_m3))

            # 限制单障碍物点数
            n_points = min(n_points, 100)

            # 均匀采样
            pts = self.rng.uniform(
                low=[-dx/2, -dy/2, -dz/2],
                high=[dx/2, dy/2, dz/2],
                size=(n_points, 3)
            )

            # 平移到障碍物中心
            pts[:, 0] += obs.x
            pts[:, 1] += obs.y
            pts[:, 2] += obs.z

            all_points.append(pts)

        if not all_points:
            return np.empty((0, 3), dtype=np.float32)

        result = np.vstack(all_points).astype(np.float32)

        # 限制总点数
        if len(result) > self.max_inject_points:
            # 均匀采样
            idx = np.linspace(0, len(result)-1, self.max_inject_points).astype(np.int32)
            result = result[idx]

        return result

    def cloud_callback(self, cloud_msg):
        """点云回调: 合并原始点云 + 虚拟障碍物点"""
        try:
            with self.lock:
                obstacles = list(self.latest_obstacles)

            if not obstacles:
                # 无障碍物，直接透传原始点云
                if self.injected_pub.get_num_connections() > 0:
                    self.injected_pub.publish(cloud_msg)
                return

            # ---- 生成虚拟障碍物点 ----
            obstacle_pts = self.generate_obstacle_points(obstacles)
            n_obstacle = len(obstacle_pts)

            if n_obstacle == 0:
                if self.injected_pub.get_num_connections() > 0:
                    self.injected_pub.publish(cloud_msg)
                return

            # ---- 解析原始点云为 numpy ----
            if self.use_original_cloud:
                orig_points = self.pointcloud2_to_xyz(cloud_msg)
                if len(orig_points) == 0:
                    return
            else:
                orig_points = np.empty((0, 3), dtype=np.float32)

            # ---- 合并 ----
            combined = np.vstack([orig_points, obstacle_pts]).astype(np.float32)

            # ---- 发布合并点云 ----
            header = Header()
            header.stamp = cloud_msg.header.stamp
            header.frame_id = cloud_msg.header.frame_id

            # 构建 PointCloud2
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]

            cloud_out = point_cloud2.create_cloud(header, fields, combined)
            self.injected_pub.publish(cloud_out)

            # ---- 统计 ----
            self.frame_count += 1
            self.total_inject += n_obstacle
            if time.time() - self.last_log_time > 5.0:
                rospy.loginfo(
                    f"[FiestaBridge] Injected {n_obstacle} pts from "
                    f"{len(obstacles)} obstacles into {len(orig_points)} LiDAR pts"
                )
                self.frame_count = 0
                self.total_inject = 0
                self.last_log_time = time.time()

        except Exception as e:
            rospy.logerr(f"[FiestaBridge] Error: {e}")

    def pointcloud2_to_xyz(self, msg):
        """
        从 PointCloud2 提取 xyz 坐标。
        轻量实现，只读3个float x 3 = 12 bytes / point。
        """
        fmt = '<fff'
        point_step = msg.point_step
        data = msg.data
        n_pts = msg.width * msg.height if msg.height > 0 else len(data) // point_step

        pts = []
        for i in range(n_pts):
            offset = i * point_step
            try:
                x, y, z = struct.unpack_from(fmt, data, offset)
                pts.append([x, y, z])
            except struct.error:
                continue

        return np.array(pts, dtype=np.float32) if pts else np.empty((0, 3), dtype=np.float32)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = FiestaESDFBridge()
    node.run()
