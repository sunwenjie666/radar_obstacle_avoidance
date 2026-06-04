#!/usr/bin/env python3
import rospy
import math
import utm
import pyproj
import csv
import os
from datetime import datetime
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion

class RTKChecker:
    def __init__(self):
        rospy.init_node('rtk_checker', anonymous=True)

        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.origin_easting = None
        self.origin_northing = None
        self.origin_yaw = None
        self.origin_recorded = False
        self.utm_zone = None
        self.proj = None

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None

        self.log_file = self._open_log()

        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
        self.click_sub = rospy.Subscriber('/clicked_point', PointStamped, self.click_callback)

        self.min_fix = rospy.get_param('~min_fix', 0)
        rospy.loginfo('rtk_checker 已启动')
        rospy.loginfo('  GPS: /mavros/global_position/global')
        rospy.loginfo('  位姿: /mavros/local_position/odom')
        rospy.loginfo('日志文件: %s', self.log_file.name)

    def _open_log(self):
        log_dir = os.path.expanduser('~/bag_analysis')
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, 'rtk_click_log.csv')
        file_exists = os.path.isfile(log_path)
        f = open(log_path, 'a', newline='')
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow([
                'timestamp', 'click_x', 'click_y', 'click_z',
                'enu_e', 'enu_n', 'enu_u',
                'abs_e', 'abs_n', 'abs_u',
                'wgs84_lat', 'wgs84_lon', 'wgs84_alt',
                'rtk_lat', 'rtk_lon', 'rtk_alt',
                'origin_lat', 'origin_lon', 'origin_alt', 'origin_yaw'
            ])
            f.flush()
        return f

    def _log_csv(self, data):
        writer = csv.writer(self.log_file)
        writer.writerow([
            data['timestamp'], data['click_x'], data['click_y'], data['click_z'],
            data['enu_e'], data['enu_n'], data['enu_u'],
            data['abs_e'], data['abs_n'], data['abs_u'],
            data['wgs84_lat'], data['wgs84_lon'], data['wgs84_alt'],
            data['rtk_lat'], data['rtk_lon'], data['rtk_alt'],
            data['origin_lat'], data['origin_lon'], data['origin_alt'], data['origin_yaw'],
        ])
        self.log_file.flush()

    def gps_callback(self, msg):
        status_names = {-1: '无定位', 0: '单点GPS', 1: 'SBAS', 2: 'RTK'}
        fix_status = msg.status.status
        fix_name = status_names.get(fix_status, f'未知({fix_status})')

        if fix_status < 0:
            rospy.logwarn_throttle(3, 'GPS 无定位...')
            return
        if fix_status < self.min_fix:
            rospy.logwarn_throttle(5, '定位精度不足 (当前: %s, 要求 fix>%d)...', fix_name, self.min_fix)
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            try:
                easting, northing, zone_num, zone_letter = utm.from_latlon(self.origin_lat, self.origin_lon)
                self.origin_easting = easting
                self.origin_northing = northing
                self.utm_zone = (zone_num, zone_letter)
                rospy.loginfo('GPS 位置已记录: lat=%.8f, lon=%.8f, alt=%.3f',
                              self.origin_lat, self.origin_lon, self.origin_alt)
            except Exception as e:
                rospy.logerr('UTM 转换失败: %s', e)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        current_yaw_deg = math.degrees(yaw)

        if not self.origin_recorded and self.origin_easting is not None:
            self.origin_yaw = current_yaw_deg
            self.origin_recorded = True
            self.proj = pyproj.Proj(proj='aeqd', lat_0=self.origin_lat, lon_0=self.origin_lon,
                                    ellps='WGS84', datum='WGS84')
            rospy.loginfo('=' * 60)
            rospy.loginfo('原点记录完成！可以在 RViz 中点云上点击目标点了')
            rospy.loginfo('初始航向: %.2f°', self.origin_yaw)
            rospy.loginfo('原点 GPS: lat=%.8f, lon=%.8f, alt=%.3f',
                          self.origin_lat, self.origin_lon, self.origin_alt)
            rospy.loginfo('=' * 60)
            self._log_csv({
                'timestamp': 'ORIGIN',
                'click_x': '0', 'click_y': '0', 'click_z': '0',
                'enu_e': '0', 'enu_n': '0', 'enu_u': '0',
                'abs_e': f'{self.origin_easting:.3f}',
                'abs_n': f'{self.origin_northing:.3f}',
                'abs_u': f'{self.origin_alt:.3f}',
                'wgs84_lat': f'{self.origin_lat:.8f}',
                'wgs84_lon': f'{self.origin_lon:.8f}',
                'wgs84_alt': f'{self.origin_alt:.3f}',
                'rtk_lat': f'{self.origin_lat:.8f}',
                'rtk_lon': f'{self.origin_lon:.8f}',
                'rtk_alt': f'{self.origin_alt:.3f}',
                'origin_lat': f'{self.origin_lat:.8f}',
                'origin_lon': f'{self.origin_lon:.8f}',
                'origin_alt': f'{self.origin_alt:.3f}',
                'origin_yaw': f'{self.origin_yaw:.2f}',
            })

    def click_callback(self, msg):
        if not self.origin_recorded:
            rospy.logwarn('原点尚未记录，等待 GPS+ODOM 数据中...')
            return

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        frame = msg.header.frame_id

        rospy.loginfo('收到点击点: (%.3f, %.3f, %.3f) 在 %s 坐标系', x, y, z, frame)

        e = x
        n = y
        u = z

        abs_e = self.origin_easting + e
        abs_n = self.origin_northing + n
        abs_u = self.origin_alt + u

        try:
            lon, lat = self.proj(e, n, inverse=True)
        except Exception:
            lat, lon = utm.to_latlon(abs_e, abs_n, self.utm_zone[0], self.utm_zone[1])

        rospy.loginfo('=' * 60)
        rospy.loginfo('=== LiDAR 点 → WGS84 坐标转换结果 ===')
        rospy.loginfo('LiDAR 检测点 (camera_init=ENU): (%.3f, %.3f, %.3f)', x, y, z)
        rospy.loginfo('ENU 坐标 (相对原点): E=%.3f, N=%.3f, U=%.3f', e, n, u)
        rospy.loginfo('>> WGS84 坐标: lat=%.8f, lon=%.8f, alt=%.3f', lat, lon, abs_u)
        rospy.loginfo('>> 当前飞机 RTK 位置: lat=%.8f, lon=%.8f, alt=%.3f',
                      self.current_lat, self.current_lon, self.current_alt)
        rospy.loginfo('=' * 60)

        self._log_csv({
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'click_x': f'{x:.3f}', 'click_y': f'{y:.3f}', 'click_z': f'{z:.3f}',
            'enu_e': f'{e:.3f}', 'enu_n': f'{n:.3f}', 'enu_u': f'{u:.3f}',
            'abs_e': f'{abs_e:.3f}', 'abs_n': f'{abs_n:.3f}', 'abs_u': f'{abs_u:.3f}',
            'wgs84_lat': f'{lat:.8f}', 'wgs84_lon': f'{lon:.8f}', 'wgs84_alt': f'{abs_u:.3f}',
            'rtk_lat': f'{self.current_lat:.8f}', 'rtk_lon': f'{self.current_lon:.8f}',
            'rtk_alt': f'{self.current_alt:.3f}',
            'origin_lat': f'{self.origin_lat:.8f}',
            'origin_lon': f'{self.origin_lon:.8f}',
            'origin_alt': f'{self.origin_alt:.3f}',
            'origin_yaw': f'{self.origin_yaw:.2f}',
        })
        rospy.loginfo('数据已记录到日志文件')

    def __del__(self):
        if hasattr(self, 'log_file'):
            self.log_file.close()

if __name__ == '__main__':
    try:
        node = RTKChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass