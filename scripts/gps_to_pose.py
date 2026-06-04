#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class GPSToPose:
    def __init__(self):
        rospy.init_node('gps_to_pose', anonymous=True)

        self.transform_pub = rospy.Publisher('/fiesta/transform', TransformStamped, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)

        rospy.loginfo('gps_to_pose 已启动')
        rospy.loginfo('  订阅: /mavros/local_position/odom')
        rospy.loginfo('  发布: /fiesta/transform + /tf (camera_init ← aft_mapped)')

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'camera_init'
        t.child_frame_id = 'aft_mapped'
        t.transform.translation = msg.pose.pose.position
        t.transform.rotation = msg.pose.pose.orientation
        self.transform_pub.publish(t)
        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    try:
        node = GPSToPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass