#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionTester:
    def __init__(self):
        rospy.init_node('vision_tester')
        self.bridge = CvBridge()
        
        # 订阅话题
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.detection_sub = rospy.Subscriber("/yolo/detections", Detection2DArray, self.detection_callback)
        self.fusion_sub = rospy.Subscriber("/fusion/obstacles", Detection2DArray, self.fusion_callback)
        
        # 统计信息
        self.frame_count = 0
        self.detection_count = 0
        self.fusion_count = 0
        
        rospy.loginfo("Vision Tester ready")
    
    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            rospy.loginfo(f"Received {self.frame_count} frames")
    
    def detection_callback(self, msg):
        self.detection_count += len(msg.detections)
        rospy.loginfo(f"YOLO detections: {len(msg.detections)} objects")
        
        for det in msg.detections:
            class_id = det.results[0].id
            score = det.results[0].score
            rospy.logdebug(f"  - Class {class_id}, Score: {score:.2f}")
    
    def fusion_callback(self, msg):
        self.fusion_count += len(msg.detections)
        rospy.loginfo(f"Fusion results: {len(msg.detections)} objects")
        
        for det in msg.detections:
            if det.results:
                pos = det.results[0].pose.pose.position
                rospy.loginfo(f"  - 3D Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
    
    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            rospy.loginfo(f"=== Summary ===")
            rospy.loginfo(f"Frames: {self.frame_count}")
            rospy.loginfo(f"Detections: {self.detection_count}")
            rospy.loginfo(f"Fusion results: {self.fusion_count}")
            rospy.loginfo("===============")
            rate.sleep()

if __name__ == '__main__':
    tester = VisionTester()
    tester.run()
