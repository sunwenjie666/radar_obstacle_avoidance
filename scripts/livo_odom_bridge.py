#!/usr/bin/env python3
"""
FAST-LIVO2 里程计 → FIESTA 位姿桥接节点
订阅 /aft_mapped_to_init (Odometry), 发布 /fiesta/transform (TransformStamped)
用于无 GPS/RTK 时，用 FAST-LIVO2 自己的里程计给 FIESTA 提供动态位姿
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class LIVOOdomBridge:
    def __init__(self):
        rospy.init_node('livo_odom_bridge', anonymous=True)

        self.odom_sub = rospy.Subscriber(
            '/aft_mapped_to_init', Odometry, self.odom_callback, queue_size=10)

        self.transform_pub = rospy.Publisher(
            '/fiesta/transform', TransformStamped, queue_size=10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo('livo_odom_bridge 已启动')
        rospy.loginfo('  订阅: /aft_mapped_to_init')
        rospy.loginfo('  发布: /fiesta/transform + /tf (camera_init <- aft_mapped)')

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id          # "camera_init"
        t.child_frame_id = msg.child_frame_id             # "aft_mapped"
        t.transform.translation = msg.pose.pose.position
        t.transform.rotation = msg.pose.pose.orientation

        self.transform_pub.publish(t)
        self.tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    try:
        node = LIVOOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
