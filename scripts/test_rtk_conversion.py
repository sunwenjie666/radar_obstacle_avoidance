#!/usr/bin/env python3
"""
LiDAR → WGS84 坐标转换 离线验证工具
用法:
  # 模拟测试（默认）
  python3 test_rtk_conversion.py

  # 读取现场记录的 CSV 日志进行离线验证
  python3 test_rtk_conversion.py ~/bag_analysis/rtk_click_log.csv
"""

import math
import utm
import pyproj
import csv
import sys
import os


def camera_init_to_wgs84(click_x, click_y, click_z,
                         origin_lat, origin_lon, origin_alt, origin_yaw):
    """LiDAR 点击点 (camera_init) → WGS84 (lat, lon, alt) + ENU"""
    easting, northing, zone_num, zone_letter = utm.from_latlon(
        origin_lat, origin_lon)

    proj = pyproj.Proj(proj='aeqd', lat_0=origin_lat, lon_0=origin_lon,
                       ellps='WGS84', datum='WGS84')

    yaw_rad = math.radians(origin_yaw)
    e =  click_x * math.sin(yaw_rad) - click_y * math.cos(yaw_rad)
    n =  click_x * math.cos(yaw_rad) + click_y * math.sin(yaw_rad)
    u =  click_z

    abs_e = easting + e
    abs_n = northing + n
    abs_u = origin_alt + u

    try:
        lon, lat = proj(e, n, inverse=True)
    except Exception:
        lat, lon = utm.to_latlon(abs_e, abs_n, zone_num, zone_letter)

    return {
        'enu_e': e, 'enu_n': n, 'enu_u': u,
        'abs_e': abs_e, 'abs_n': abs_n, 'abs_u': abs_u,
        'wgs84_lat': lat, 'wgs84_lon': lon, 'wgs84_alt': abs_u,
    }


def haversine(lat1, lon1, lat2, lon2):
    """计算两点间 GPS 距离（米）"""
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2)**2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon/2)**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


def run_simulation():
    """模式1：模拟测试，直接验算"""
    origin_lat = 39.880000
    origin_lon = 116.410000
    origin_alt = 50.0
    origin_yaw = 90.0
    click_x, click_y, click_z = 10.0, 0.0, 2.0
    current_lat, current_lon, current_alt = 39.880500, 116.411200, 55.0

    result = camera_init_to_wgs84(click_x, click_y, click_z,
                                   origin_lat, origin_lon, origin_alt, origin_yaw)

    print('=' * 70)
    print('  模式1：模拟测试 — 坐标转换验算')
    print('=' * 70)
    print(f'\n⭐ 原点 RTK（真值）: lat={origin_lat:.8f}, lon={origin_lon:.8f}, alt={origin_alt:.3f}')
    print(f'   初始航向: {origin_yaw:.2f}°')
    print(f'\nLiDAR 点击点:     ({click_x:.3f}, {click_y:.3f}, {click_z:.3f})')
    print(f'ENU 偏移:          E={result["enu_e"]:.3f}, N={result["enu_n"]:.3f}, U={result["enu_u"]:.3f}')
    print(f'\n{"=" * 70}')
    print(f'  >> 转换 WGS84:   lat={result["wgs84_lat"]:.8f}, lon={result["wgs84_lon"]:.8f}, alt={result["wgs84_alt"]:.3f}')
    print(f'{"=" * 70}')
    diff = haversine(origin_lat, origin_lon, result["wgs84_lat"], result["wgs84_lon"])
    print(f'\n精度验证：转换结果与原点相差 {diff:.3f} 米')
    print(f'（目标点应该在飞机前方 10m 处，所以差值应为 10m）')


def verify_log(log_path):
    """模式2：读取现场 CSV 日志，离线验证"""
    if not os.path.isfile(log_path):
        print(f'错误：找不到日志文件 {log_path}')
        return

    with open(log_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print('错误：日志文件为空')
        return

    print('=' * 70)
    print('  离线验证 — 读取现场日志 对比 LiDAR 转换 vs RTK 真值')
    print('=' * 70)
    print(f'\n日志文件: {log_path}')
    print(f'总记录数: {len(rows)}')
    print()

    origin_printed = False
    click_count = 0

    for i, row in enumerate(rows):
        timestamp = row['timestamp']

        if timestamp == 'ORIGIN':
            print(f'--- 原点（RTK 真值） ---')
            print(f'  lat={row["wgs84_lat"]}, lon={row["wgs84_lon"]}, alt={row["wgs84_alt"]}')
            print(f'  航向: {row["origin_yaw"]}°')
            print(f'  （飞机停在此处时记录，此为板子的真实位置）')
            print()
            origin_printed = True
            continue

        click_count += 1
        click_x = float(row['click_x'])
        click_y = float(row['click_y'])
        click_z = float(row['click_z'])
        log_lat = float(row['wgs84_lat'])
        log_lon = float(row['wgs84_lon'])
        log_alt = float(row['wgs84_alt'])
        enu_e = float(row['enu_e'])
        enu_n = float(row['enu_n'])
        enu_u = float(row['enu_u'])
        origin_lat = float(row['origin_lat'])
        origin_lon = float(row['origin_lon'])
        origin_alt = float(row['origin_alt'])

        print(f'--- 第 {click_count} 次点击 ({timestamp}) ---')
        print(f'LiDAR 点击:       ({click_x:.3f}, {click_y:.3f}, {click_z:.3f})')
        print(f'ENU 偏移:          E={enu_e:.3f}, N={enu_n:.3f}, U={enu_u:.3f}')
        print(f'转换 WGS84:        lat={log_lat:.8f}, lon={log_lon:.8f}, alt={log_alt:.3f}')

        if origin_printed:
            diff = haversine(origin_lat, origin_lon, log_lat, log_lon)
            alt_diff = log_alt - origin_alt
            print(f'⭐ 与真值差距:      水平 {diff:.3f} 米, 高度 {alt_diff:+.3f} 米')
            if diff < 0.5:
                print(f'   ✅ 精度很好（厘米级）')
            elif diff < 2.0:
                print(f'   ⚠️ 精度尚可（米级）')
            else:
                print(f'   ❌ 偏差较大，需排查参数')
        print()


def main():
    if len(sys.argv) >= 2:
        verify_log(sys.argv[1])
    else:
        run_simulation()


if __name__ == '__main__':
    main()