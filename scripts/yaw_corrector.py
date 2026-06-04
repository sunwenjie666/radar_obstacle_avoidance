#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class YawCorrector:
    def __init__(self):
        rospy.init_node('yaw_corrector', anonymous=True)
        
        # 标记是否已校准
        self.calibrated = False
        self.initial_yaw = 0.0
        
        # 订阅飞控的 IMU 数据
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        
        # 创建一个静态 TF 广播器（后续使用）
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        rospy.loginfo("等待飞控 IMU 数据...")
    
    def imu_callback(self, msg):
        if self.calibrated:
            return  # 已经校正过，不再处理
        
        # 提取四元数
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # 转换为欧拉角（roll, pitch, yaw）
        roll, pitch, yaw = euler_from_quaternion(q)
        
        # 记录初始航向角（弧度）
        self.initial_yaw = yaw
        self.calibrated = True
        
        rospy.loginfo("获取到飞控初始航向角: %.2f 度 (%.4f 弧度)", math.degrees(yaw), yaw)
        
        # 发布静态变换，将 FAST-LIVO2 的 map 坐标系对齐到飞控的 base_link 坐标系
        self.publish_static_transform()
    
    def publish_static_transform(self):
        # 创建一个静态变换：从 base_link（飞控机体） 到 map（FAST-LIVO2 原始地图）
        # 注意：这里假设 FAST-LIVO2 启动时 map 坐标系的原点在雷达位置，但 yaw=0 为雷达初始朝向。
        # 我们通过旋转 map 使它的 x 轴指向飞控的航向。
        
        static_transform = geometry_msgs.msg.TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "base_link"   # 父坐标系（飞控机体）
        static_transform.child_frame_id = "map"          # 子坐标系（雷达地图）
        
        # 平移部分置零（假设雷达与飞控中心重合，或后续再调整）
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        
        # 旋转部分：将 map 的初始朝向（yaw=0）旋转到飞控的当前 yaw 角度
        # 即：map 坐标系绕 Z 轴旋转 initial_yaw 弧度
        q_corr = quaternion_from_euler(0, 0, self.initial_yaw)
        static_transform.transform.rotation.x = q_corr[0]
        static_transform.transform.rotation.y = q_corr[1]
        static_transform.transform.rotation.z = q_corr[2]
        static_transform.transform.rotation.w = q_corr[3]
        
        # 广播静态变换
        self.tf_broadcaster.sendTransform(static_transform)
        rospy.loginfo("已发布静态 TF：base_link → map，Yaw 偏移已修正")
        
        # 注意：此节点可继续运行，但不再重复修正。也可以 rospy.signal_shutdown 退出

if __name__ == '__main__':
    try:
        node = YawCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
