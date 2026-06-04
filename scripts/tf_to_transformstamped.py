#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from tf2_ros import TransformException
import time

class TFExtractor:
    def __init__(self):
        rospy.init_node('tf_extractor')
        
        self.target_frame = rospy.get_param('~target_frame', 'aft_mapped')
        self.source_frame = rospy.get_param('~source_frame', 'camera_init')
        
        self.pub = rospy.Publisher('/fiesta/transform', 
                                   geometry_msgs.msg.TransformStamped, 
                                   queue_size=10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 等待变换可用，最多等10秒
        rospy.loginfo(f"等待 {self.source_frame} -> {self.target_frame} 变换...")
        start = time.time()
        while not rospy.is_shutdown() and (time.time() - start) < 10.0:
            try:
                self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rospy.Time(0))
                rospy.loginfo("变换已就绪")
                break
            except TransformException:
                rospy.sleep(0.1)
        else:
            rospy.logwarn("等待变换超时，将继续尝试但可能失败")
        
        rospy.loginfo(f"TF Extractor 已启动，发布 {self.source_frame} -> {self.target_frame}")
    
    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rospy.Time(0))
                self.pub.publish(transform)
            except TransformException as e:
                rospy.logwarn_throttle(5, f"无法获取变换: {e}")
            rate.sleep()

if __name__ == '__main__':
    node = TFExtractor()
    node.run()
