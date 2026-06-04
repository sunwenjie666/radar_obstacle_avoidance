import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

class YawCalibrator:
    def __init__(self):
        rospy.init_node('yaw_calibrator', anonymous=True)

        # 存储飞控初始航向 (yaw)
        self.initial_yaw = 0.0
        self.yaw_received = False

        # 订阅飞控的IMU数据，获取真实的初始航向角
        self.mavros_imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)

        rospy.loginfo("等待飞控航向数据...")
        # 等待数据到来，最多等待5秒
        rospy.sleep(1.0)

        if self.yaw_received:
            rospy.loginfo(f"收到飞控数据，雷达坐标系将校正 {math.degrees(self.initial_yaw):.2f} 度")
            # 发布静态变换，对齐坐标系
            br = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "base_link"  # 机体坐标系
            static_transformStamped.child_frame_id = "map"        # 雷达地图坐标系
            static_transformStamped.transform.translation.x = 0.0
            static_transformStamped.transform.translation.y = 0.0
            static_transformStamped.transform.translation.z = 0.0

            # 根据飞控提供的初始航向，创建一个旋转四元数
            q = quaternion_from_euler(0, 0, self.initial_yaw)
            static_transformStamped.transform.rotation.x = q[0]
            static_transformStamped.transform.rotation.y = q[1]
            static_transformStamped.transform.rotation.z = q[2]
            static_transformStamped.transform.rotation.w = q[3]

            br.sendTransform(static_transformStamped)
            rospy.loginfo("静态变换 [base_link -> map] 发布成功！")
        else:
            rospy.logerr("无法获取飞控初始航向，请检查MAVROS连接！")

    def imu_callback(self, msg):
        if not self.yaw_received:
            # 从IMU消息的四元数中提取出欧拉角
            orientation_q = msg.orientation
            quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            _, _, self.initial_yaw = euler_from_quaternion(quat)
            self.yaw_received = True
            rospy.loginfo(f"接收到飞控数据，初始航向角 (yaw) = {math.degrees(self.initial_yaw):.2f} 度")

if __name__ == '__main__':
    try:
        YawCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
