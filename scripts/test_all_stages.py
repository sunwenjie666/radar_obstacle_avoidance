#!/usr/bin/env python3
import rospy
import subprocess
import time
import sys

def test_stage(stage_name, launch_file):
    print(f"\n=== 测试 {stage_name} ===")
    
    # 启动launch文件
    launch_cmd = f"roslaunch radar_obstacle_avoidance {launch_file}"
    process = subprocess.Popen(launch_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # 等待系统启动
    time.sleep(10)
    
    # 检查关键话题
    topics_to_check = {
        "第一阶段": ["/lidar_points", "/fast_livo/odometry"],
        "第二阶段": ["/feature_entropy", "/feature_rich_event"],
        "第三阶段": ["/rtk/data", "/synced/cloud"],
        "第四阶段": ["/global_map", "/path/planning"]
    }
    
    success = True
    if stage_name in topics_to_check:
        for topic in topics_to_check[stage_name]:
            try:
                result = subprocess.check_output(f"rostopic list | grep {topic}", shell=True)
                if topic in result.decode():
                    print(f"  ✓ {topic} 话题正常")
                else:
                    print(f"  ✗ {topic} 话题缺失")
                    success = False
            except subprocess.CalledProcessError:
                print(f"  ✗ {topic} 话题缺失")
                success = False
    
    # 停止进程
    process.terminate()
    process.wait()
    
    return success

def main():
    rospy.init_node('system_tester', anonymous=True)
    
    stages = [
        ("第一阶段: 环境与驱动", "radar_processor.launch"),
        ("第二阶段: 特征检测", "feature_detection.launch"),
        ("第三阶段: RTK集成", "rtk_gps.launch"),
        ("第四阶段: 完整系统", "full_system.launch")
    ]
    
    print("开始系统测试...")
    
    all_passed = True
    for stage_name, launch_file in stages:
        if test_stage(stage_name, launch_file):
            print(f"{stage_name}: 通过")
        else:
            print(f"{stage_name}: 失败")
            all_passed = False
        
        time.sleep(5)  # 阶段间间隔
    
    print("\n=== 测试总结 ===")
    if all_passed:
        print("✓ 所有测试通过！系统可以正常运行。")
        sys.exit(0)
    else:
        print("⚠ 部分测试失败，请检查相关配置。")
        sys.exit(1)

if __name__ == '__main__':
    main()
