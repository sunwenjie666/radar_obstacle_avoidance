#!/usr/bin/env python3
"""
无 PX4/MAVROS 时的静态位姿兜底节点
发布单位置/零姿态到 /fiesta/transform 和 /tf（camera_init ← aft_mapped）
"""
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros

rospy.init_node('static_tf_fallback', anonymous=True)

pub = rospy.Publisher('/fiesta/transform', TransformStamped, queue_size=10)
br = tf2_ros.TransformBroadcaster()

t = TransformStamped()
t.header.frame_id = 'camera_init'
t.child_frame_id = 'aft_mapped'
t.transform.translation.x = 0.0
t.transform.translation.y = 0.0
t.transform.translation.z = 0.0
t.transform.rotation.x = 0.0
t.transform.rotation.y = 0.0
t.transform.rotation.z = 0.0
t.transform.rotation.w = 1.0

rospy.loginfo('静态位姿兜底节点已启动（origin 处）')

rate = rospy.Rate(10)  # 10Hz
while not rospy.is_shutdown():
    t.header.stamp = rospy.Time.now()
    pub.publish(t)
    br.sendTransform(t)
    rate.sleep()
