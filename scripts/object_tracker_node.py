#!/usr/bin/env python3
"""
norfair 2D 目标跟踪节点
=====================
替代原有的 IoU 匹配跟踪器，使用 norfair 的 Kalman Filter 实现：
  - 运动模型预测 (位置 + 速度)
  - 遮挡时继续预测位置
  - 跟踪 ID 自动管理
  - 速度估计 (可用于后续运动预测)

用法:
  rosrun radar_obstacle_avoidance object_tracker_node.py

Topics:
  Sub: /yolo/detections    (vision_msgs/Detection2DArray)
  Pub: /tracked_objects    (vision_msgs/Detection2DArray)

vs 旧版 IoU 跟踪器:
  指标          旧版 IoU     norfair
  运动模型       ❌ 无       ✅ Kalman Filter
  速度估计       ❌ 无       ✅
  遮挡恢复       ❌ 直接丢    ✅ 保留推测位置
  CPU 耗时      ~0.3ms      ~0.5ms
"""

import rospy
import numpy as np
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import time

# norfair
try:
    from norfair import Detection, Tracker
except ImportError:
    rospy.logerr("norfair not installed! Run: pip3 install norfair")
    raise


def get_optimal_distance_threshold(img_width, img_height):
    """根据图像尺寸自适应距离阈值 (像素)"""
    return max(img_width, img_height) * 0.06  # 例如 640x480 → 38px


class NorfairObjectTracker:
    def __init__(self):
        rospy.init_node('object_tracker', anonymous=False)

        # ---- 参数 ----
        self.img_width = rospy.get_param('~img_width', 640)
        self.img_height = rospy.get_param('~img_height', 480)
        distance_threshold = rospy.get_param(
            '~distance_threshold',
            get_optimal_distance_threshold(self.img_width, self.img_height)
        )
        self.hit_counter_max = rospy.get_param('~hit_counter_max', 20)
        self.initialization_delay = rospy.get_param('~initialization_delay', 3)

        # ---- norfair 跟踪器 ----
        # 使用内置 euclidean 距离函数 (向量化实现, 更快)
        # + KalmanFilter (基于 filterpy) 实现运动预测
        self.tracker = Tracker(
            distance_function='euclidean',
            distance_threshold=distance_threshold,
            hit_counter_max=self.hit_counter_max,
            initialization_delay=self.initialization_delay,
        )

        # ---- 性能统计 ----
        self.frame_count = 0
        self.det_count = 0
        self.track_count = 0
        self.processing_time = 0.0
        self.last_log_time = time.time()

        # ---- ROS 接口 ----
        self.detection_sub = rospy.Subscriber(
            '/yolo/detections',
            Detection2DArray,
            self.detection_callback,
            queue_size=10,
            buff_size=2**20
        )

        self.track_pub = rospy.Publisher(
            '/tracked_objects',
            Detection2DArray,
            queue_size=10
        )

        rospy.loginfo(f"[NorfairTracker] Init: "
                      f"threshold={distance_threshold:.1f}px, "
                      f"hit_max={self.hit_counter_max}, "
                      f"init_delay={self.initialization_delay}")

    def detection_callback(self, msg):
        """YOLO 检测回调: 构建 norfair Detections 并更新"""
        start_time = time.time()

        norfair_dets = []

        for det in msg.detections:
            if not det.results:
                continue

            score = det.results[0].score
            cx = det.bbox.center.x
            cy = det.bbox.center.y
            w = det.bbox.size_x
            h = det.bbox.size_y

            # 构建 norfair Detection
            # points: [cx, cy] → shape (1, 2)
            points = np.array([[cx, cy]], dtype=np.float32)
            scores = np.array([score], dtype=np.float32)

            norfair_det = Detection(
                points=points,
                scores=scores,
                label=int(det.results[0].id),
                data={
                    'bbox': [cx, cy, w, h],
                    'class_id': det.results[0].id,
                    'score': score,
                }
            )
            norfair_dets.append(norfair_det)

        # ---- norfair 更新 ----
        tracks = self.tracker.update(detections=norfair_dets)

        # ---- 发布跟踪结果 ----
        self.publish_tracks(tracks, msg.header)

        # ---- 性能统计 ----
        elapsed = time.time() - start_time
        self.frame_count += 1
        self.det_count += len(norfair_dets)
        self.track_count = len(tracks)
        self.processing_time += elapsed

        if time.time() - self.last_log_time > 5.0:
            avg_ms = (self.processing_time / max(self.frame_count, 1)) * 1000
            rospy.loginfo(
                f"[NorfairTracker] Dets:{self.det_count // max(self.frame_count,1):.1f}/fr "
                f"→ Tracks:{self.track_count} "
                f"({avg_ms:.2f}ms)"
            )
            self.frame_count = 0
            self.det_count = 0
            self.processing_time = 0.0
            self.last_log_time = time.time()

    def publish_tracks(self, tracks, header):
        """将 norfair tracks 转为 ROS Detection2DArray"""
        if self.track_pub.get_num_connections() == 0:
            return

        msg = Detection2DArray()
        msg.header = header

        for track in tracks:
            if track.label is None:
                continue

            detection = Detection2D()
            detection.header = header

            # ---- 获取跟踪器估计的 bbox ----
            estimate = track.estimate  # [N, 2] = [cx, cy]

            # 从 Detation.data 获取原始 bbox 尺寸 (via track.last_detection)
            if (track.last_detection is not None and
                    hasattr(track.last_detection, 'data') and
                    track.last_detection.data):
                orig_w = track.last_detection.data.get('bbox', [0, 0, 50, 50])[2]
                orig_h = track.last_detection.data.get('bbox', [0, 0, 50, 50])[3]
            else:
                orig_w, orig_h = 50.0, 50.0

            # 使用 Kalman 预测的 center
            if estimate.shape[0] > 0:
                cx, cy = float(estimate[0, 0]), float(estimate[0, 1])
            else:
                cx, cy = 0.0, 0.0

            detection.bbox.center.x = cx
            detection.bbox.center.y = cy
            detection.bbox.size_x = orig_w
            detection.bbox.size_y = orig_h

            # ---- 类别 + track_id ----
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = track.label
            # score: 从 last_detection.scores 获取 (last_detection_score 属性不存在)
            if (track.last_detection is not None and
                    hasattr(track.last_detection, 'scores') and
                    len(track.last_detection.scores) > 0):
                hypothesis.score = float(track.last_detection.scores[0])
            else:
                hypothesis.score = 0.5

            # pose.position 存 track_id, pose.orientation 存 velocity
            hypothesis.pose.pose.position.x = float(track.id)
            hypothesis.pose.pose.position.y = float(track.age)
            hypothesis.pose.pose.position.z = float(track.hit_counter)

            # velocity (from Kalman filter internal state)
            if hasattr(track, 'estimate_velocity') and track.estimate_velocity is not None:
                vel_arr = np.asarray(track.estimate_velocity)
                if vel_arr.size >= 2:
                    vx, vy = float(vel_arr[0, 0]), float(vel_arr[0, 1])
                else:
                    vx, vy = 0.0, 0.0
            else:
                vx, vy = 0.0, 0.0

            hypothesis.pose.pose.orientation.x = vx
            hypothesis.pose.pose.orientation.y = vy
            hypothesis.pose.pose.orientation.z = 0.0
            hypothesis.pose.pose.orientation.w = 1.0

            detection.results.append(hypothesis)
            msg.detections.append(detection)

        self.track_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        tracker = NorfairObjectTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
