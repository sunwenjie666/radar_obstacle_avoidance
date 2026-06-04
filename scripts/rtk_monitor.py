#!/usr/bin/env python3
import rospy
import math
import utm
import pyproj
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

class RTKMonitor:
    def __init__(self):
        rospy.init_node('rtk_monitor', anonymous=True)

        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.origin_easting = None
        self.origin_northing = None
        self.origin_yaw = None
        self.origin_recorded = False
        self.proj = None

        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.heading_sub = rospy.Subscriber('/handsfree/rtk/heading', Float64, self.heading_callback)

        rospy.loginfo('等待 RTK 数据（位置 + 航向）记录原点...')

    def heading_callback(self, msg):
        if self.origin_recorded:
            self.heading_sub.unregister()
            return

        self.origin_yaw = msg.data

        if self.origin_lat is not None:
            self._record_origin()

    def _record_origin(self):
        self.origin_recorded = True
        self.proj = pyproj.Proj(
            proj='aeqd',
            lat_0=self.origin_lat,
            lon_0=self.origin_lon,
            ellps='WGS84',
            datum='WGS84'
        )
        self.heading_sub.unregister()
        rospy.loginfo('=' * 70)
        rospy.loginfo('原点记录完成！')
        rospy.loginfo('航向: %.2f°', self.origin_yaw)
        rospy.loginfo('原点: lat=%.8f, lon=%.8f, alt=%.3f',
                      self.origin_lat, self.origin_lon, self.origin_alt)
        rospy.loginfo('--- 实时 ENU 偏移 + WGS84 坐标 ---')
        rospy.loginfo('=' * 70)

    def gps_callback(self, msg):
        if msg.status.status < 0:
            rospy.logwarn_throttle(3, 'GPS 无定位...')
            return

        if not self.origin_recorded:
            if self.origin_lat is None:
                self.origin_lat = msg.latitude
                self.origin_lon = msg.longitude
                self.origin_alt = msg.altitude
                try:
                    easting, northing, _, _ = utm.from_latlon(
                        self.origin_lat, self.origin_lon)
                    self.origin_easting = easting
                    self.origin_northing = northing
                except Exception as e:
                    rospy.logerr('UTM 转换失败: %s', e)
                if self.origin_yaw is not None:
                    self._record_origin()
            return

        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        e = easting - self.origin_easting
        n = northing - self.origin_northing
        u = msg.altitude - self.origin_alt

        rospy.loginfo('ENU: E=%7.3f  N=%7.3f  U=%6.3f  |  WGS84: lat=%.8f  lon=%.8f  alt=%.3f',
                      e, n, u, msg.latitude, msg.longitude, msg.altitude)

if __name__ == '__main__':
    try:
        node = RTKMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass