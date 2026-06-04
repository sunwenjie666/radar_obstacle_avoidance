#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import PointStamped
from geometry_msgs.msg import PointStamped
from geographic_msgs.msg import GeoPointStamped
import pyproj
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode

class PointWorldConverter:
    def __init__(self):
        rospy.init_node('point_world_converter')
        
        # 存储起飞点的RTK和航向
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.origin_yaw = None
        
        # 是否已记录
        self.origin_recorded = False
        
        # 创建投影对象，用于经纬度到ENU转换
        # 初始占位，待原点确定后重新创建
        self.proj = None
        
        # 订阅RTK位置
        self.global_pos_sub = rospy.Subscriber('/mavros/global_position/global', 
                                               GlobalPositionTarget, self.global_callback)
        # 订阅IMU获取航向
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        
        # 订阅用户点击的点（在RViz中用 Publish Point 按钮选取）
        self.click_sub = rospy.Subscriber('/clicked_point', PointStamped, self.click_callback)
        
        # 等待记录原点
        rospy.loginfo("等待起飞点RTK和航向数据...")
        while not self.origin_recorded and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo("原点已记录，等待在RViz中点击点云上的点...")
        
    def global_callback(self, msg):
        if not self.origin_recorded:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            # 创建投影器：以起飞点为原点的ENU坐标系
            self.proj = pyproj.Proj(proj='aeqd', lat_0=self.origin_lat, lon_0=self.origin_lon, 
                                    ellps='WGS84', datum='WGS84')
            rospy.loginfo(f"RTK原点: lat={self.origin_lat}, lon={self.origin_lon}, alt={self.origin_alt}")
    
    def imu_callback(self, msg):
        if not self.origin_recorded:
            quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            _, _, self.origin_yaw = euler_from_quaternion(quat)
            self.origin_recorded = True
            rospy.loginfo(f"初始航向角: {math.degrees(self.origin_yaw):.1f} 度")
    
    def click_callback(self, msg):
        if not self.origin_recorded:
            rospy.logwarn("原点尚未记录，请等待")
            return
        
        # 获取点在map坐标系下的坐标
        x_map = msg.point.x
        y_map = msg.point.y
        z_map = msg.point.z
        
        # 应用航向旋转（将map坐标系下的点旋转到航向对齐的世界坐标系）
        # 注意：map的x轴初始朝向雷达，我们需要旋转到飞控的航向。
        # 旋转矩阵：绕Z轴旋转 origin_yaw 角度
        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)
        x_rot = x_map * cos_yaw - y_map * sin_yaw
        y_rot = x_map * sin_yaw + y_map * cos_yaw
        z_rot = z_map
        
        # 此时 (x_rot, y_rot, z_rot) 是以起飞点为原点的ENU坐标（假设map原点即起飞点）
        # 但实际上map原点可能不是起飞点（因为无人机移动了）。我们需要一个平移补偿。
        # 最简单的办法：在起飞悬停时，记录此时雷达map坐标（应该为0），但实际可能有小偏移。
        # 为简化，我们假设起飞后立即悬停，map原点就是起飞点。
        # 如果你有更精确的起点偏移，可以在此添加平移。
        
        # 将ENU坐标转回WGS84经纬高
        # 使用pyproj进行正算：给定ENU坐标，计算经纬度
        # 注意pyproj的aeqd投影输入是(东向, 北向)，输出为(经度, 纬度)
        lon, lat = self.proj(x_rot, y_rot, inverse=True)
        alt = self.origin_alt + z_rot
        
        # 输出结果
        rospy.loginfo("=== 坐标转换结果 ===")
        rospy.loginfo(f"雷达检测点 map坐标: ({x_map:.3f}, {y_map:.3f}, {z_map:.3f})")
        rospy.loginfo(f"转换后世界坐标: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.3f}")
        rospy.loginfo("请与RTK实测该点坐标进行对比")
        
        # 可选：发布一个Marker在RViz中显示该点

if __name__ == '__main__':
    try:
        converter = PointWorldConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
