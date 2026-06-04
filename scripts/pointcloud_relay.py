#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudRelay:
    def __init__(self):
        rospy.init_node('pointcloud_relay', anonymous=True)

        mount_roll = rospy.get_param('~mount_roll', 0.0)
        mount_pitch = rospy.get_param('~mount_pitch', 0.0)
        mount_yaw = rospy.get_param('~mount_yaw', 0.0)
        self.R_mount = self._euler_to_rotation_matrix(
            math.radians(mount_roll), math.radians(mount_pitch), math.radians(mount_yaw))
        if abs(mount_roll) > 0.01 or abs(mount_pitch) > 0.01 or abs(mount_yaw) > 0.01:
            rospy.loginfo('LiDAR 安装角补偿: roll=%.1f°, pitch=%.1f°, yaw=%.1f°',
                          mount_roll, mount_pitch, mount_yaw)

        self.cloud_pub = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=10)
        self.cloud_sub = rospy.Subscriber('/hesai_points', PointCloud2, self.cloud_callback)

        rospy.loginfo('pointcloud_relay 已启动')
        rospy.loginfo('  订阅: /hesai_points')
        rospy.loginfo('  发布: /cloud_registered (aft_mapped 帧 → FIESTA 自动变换到 camera_init)')

    def _euler_to_rotation_matrix(self, roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])

    def cloud_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity', 'ring'), skip_nans=True))
        if not points:
            return
        points_np = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float64)
        body_points = points_np @ self.R_mount.T
        cloud_out = pc2.create_cloud_xyz32(msg.header, body_points.tolist())
        cloud_out.header.stamp = rospy.Time.now()
        cloud_out.header.frame_id = 'aft_mapped'
        self.cloud_pub.publish(cloud_out)

if __name__ == '__main__':
    try:
        node = PointCloudRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass