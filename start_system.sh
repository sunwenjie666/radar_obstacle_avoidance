#!/bin/bash

# ============================================
# 雷达建图系统一键启动脚本
# 路线：Hesai雷达 → FAST-LIVO2 → FIESTA 建图
# 有GPS: MAVROS/RTK 提供绝对位姿 (gps_to_pose)
# 无GPS: FAST-LIVO2 里程计提供动态位姿 (livo_odom_bridge)
#
# 外场实验用法:
#   1. RTK 双天线连接到 PX4，PX4 通过串口连接到 Jetson
#   2. 飞机停在目标点上方（记录原点）
#   3. 运行本脚本
#   4. 等待 "原点记录完成" 日志出现
#   5. 在 RViz 中点云上点击目标点验证坐标
# ============================================

# 不使用 set -e，单个步骤失败不中断整体流程

echo "=== 1. 检查网络接口 ==="
ip link show || true

echo "=== 2. 停掉 NetworkManager 对 eth0 的管理 ==="
sudo nmcli device set eth0 managed no 2>/dev/null || true
echo "已停止 NetworkManager 接管 eth0"

echo "=== 3. 设置有线网卡IP (与雷达同网段) ==="
sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0 up || true

echo "=== 4. 查看IP配置 ==="
ip addr show eth0

echo "=== 5. 测试雷达连通性 (ping 5次) ==="
ping -c 5 192.168.1.201 || true

echo "=== 6. 确认禾赛驱动包存在 ==="
ls ~/catkin_ws/src/ | grep -i hesai || true

echo "=== 7. 编译工作空间（确保最新） ==="
cd ~/catkin_ws
catkin_make
source devel/setup.bash

echo "=== 8. 启动禾赛雷达驱动 ==="
roslaunch hesai_ros_driver start.launch &
RADAR_PID=$!
sleep 3

echo "=== 8.5 启动 FAST-LIVO2 里程计 ==="
roslaunch fast_livo mapping_avia.launch &
LIVO_PID=$!
sleep 3

echo "=== 9. 检测 PX4/MAVROS 是否连接 ==="
# 尝试启动 MAVROS，然后检查是否有 odom 数据
roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS0:115200" &
MAVROS_PID=$!
sleep 4

# 检查 MAVROS 是否连接成功（看 odom 话题是否有消息）
MAVROS_CONNECTED=false
if rostopic list 2>/dev/null | grep -q "/mavros/local_position/odom"; then
    if timeout 2 rostopic echo /mavros/local_position/odom --noarr -n 1 2>/dev/null | grep -q "pose"; then
        MAVROS_CONNECTED=true
    fi
fi

if [ "$MAVROS_CONNECTED" = true ]; then
    echo "  ✅ MAVROS 已连接，启动 GPS 相关节点"

    echo "=== 9.1 启动 GPS→位姿 转换节点 ==="
    rosrun radar_obstacle_avoidance gps_to_pose.py &
    GPS_POSE_PID=$!
    sleep 1

    echo "=== 9.2 启动 RTK 坐标验证节点 ==="
    rosrun radar_obstacle_avoidance rtk_checker.py &
    RTK_PID=$!
    sleep 1
else
    echo "  ⚠️ MAVROS 未连接（无 PX4/RTK），使用 FAST-LIVO2 里程计"
    echo "  livo_odom_bridge: /aft_mapped_to_init → /fiesta/transform"
    kill $MAVROS_PID 2>/dev/null || true
    MAVROS_PID=""
    rosrun radar_obstacle_avoidance livo_odom_bridge.py &
    ODOM_BRIDGE_PID=$!
    sleep 1
fi

echo "=== 10. 启动 FIESTA 建图节点 ==="
roslaunch fiesta cow_and_lady.launch &
FIESTA_PID=$!

echo "=== 11. 录制标定数据包 (10秒) ==="
# 等待各节点稳定后，录制一段包用于离线校正 FIESTA 地面倾斜
# 可用来标定 pointcloud_relay.py 的 mount_roll/pitch/yaw 安装角参数
sleep 3
BAG_DIR=~/bagfiles
mkdir -p $BAG_DIR
BAG_NAME="fiesta_calib_$(date +%Y%m%d_%H%M%S).bag"
rosbag record -O $BAG_DIR/$BAG_NAME \
    /cloud_registered \
    /fiesta/transform \
    /mavros/local_position/odom \
    /hesai_points &
RECORD_PID=$!
sleep 10
kill $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true
echo "标定包已保存: $BAG_DIR/$BAG_NAME"

echo "========================================"
echo "所有节点已启动"
echo "  [8] 雷达驱动 PID:        $RADAR_PID"
echo "  [8.5] FAST-LIVO2 PID:    $LIVO_PID"
if [ "$MAVROS_CONNECTED" = true ]; then
echo "  [9] MAVROS PID:          $MAVROS_PID"
echo "  [9.1] GPS→位姿 PID:     $GPS_POSE_PID"
echo "  [9.2] RTK Checker PID:   $RTK_PID"
else
echo "  [9] MAVROS 未连接（里程计桥接 PID: $ODOM_BRIDGE_PID）"
fi
echo "  [10] FIESTA PID:         $FIESTA_PID"
echo "  [11] 标定包已保存: $BAG_NAME"
echo ""
if [ "$MAVROS_CONNECTED" = true ]; then
echo "等待 rtk_checker / gps_to_pose 输出 '原点记录完成'"
else
echo "⚠️ 无 GPS 模式 (FAST-LIVO2 里程计)，漂移随时间累积"
echo "定位精度: 局部 ~厘米级，全局 ~1%/100m"
fi
echo "在 RViz 中将 Fixed Frame 设为 camera_init"
echo "按 Ctrl+C 可退出所有进程"
echo "========================================"

wait