#!/usr/bin/env python3
"""
BEV Visualizer
==============
订阅 BEV 融合结果并在独立窗口/RViz中可视化。
用于调试和验证 BEV 融合效果。
"""

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from radar_obstacle_avoidance.msg import Obstacle3DArray


class BEVVisualizer:
    def __init__(self):
        rospy.init_node('bev_visualizer', anonymous=True)

        self.bridge = CvBridge()

        # 订阅
        self.obstacle_sub = rospy.Subscriber(
            '/fusion/obstacles_3d', Obstacle3DArray, self.obstacle_callback)
        self.bev_image_sub = rospy.Subscriber(
            '/fusion/bev_image', Image, self.bev_image_callback)
        self.marker_sub = rospy.Subscriber(
            '/fusion/markers', MarkerArray, self.marker_callback)

        # 状态
        self.last_obstacles = None
        self.last_bev = None

        # 统计
        self.frame_count = 0
        self.last_log = time.time()

        rospy.loginfo("[BEVVisualizer] Ready")

        # OpenCV 窗口 (若没有显示器则跳过)
        try:
            cv2.namedWindow("BEV Fusion", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("BEV Fusion", 800, 600)
            self.use_gui = True
        except:
            self.use_gui = False
            rospy.loginfo("[BEVVisualizer] No display available, running headless")

    def obstacle_callback(self, msg):
        """3D障碍物列表回调"""
        self.last_obstacles = msg
        rospy.loginfo(f"[BEVVisualizer] Obstacles: {len(msg.obstacles)} objects")
        for obs in msg.obstacles:
            rospy.loginfo(f"  [{obs.class_name}] "
                          f"pos=({obs.x:.2f},{obs.y:.2f},{obs.z:.2f}) "
                          f"size=({obs.width:.2f}x{obs.length:.2f}x{obs.height:.2f}) "
                          f"score={obs.association_score:.2f}")

    def bev_image_callback(self, msg):
        """BEV图像回调"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_bev = cv_img
            if self.use_gui:
                cv2.imshow("BEV Fusion", cv_img)
                cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn(f"[BEVVisualizer] BEV image error: {e}")

    def marker_callback(self, msg):
        """MarkerArray回调"""
        pass  # RViz 负责渲染

    def run(self):
        rospy.spin()
        if self.use_gui:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    viz = BEVVisualizer()
    viz.run()
