#!/bin/bash

# ============================================
# FIESTA 地面倾斜离线标定工具
#
# 用法:
#   1. 外场用 start_system.sh 录好标定包 (~/bagfiles/fiesta_calib_*.bag)
#   2. 回到室内离线运行本脚本，调整安装角参数直到地面变平
#
# 示例:
#   # 先用默认参数（0,0,0）看倾斜情况
#   ./calibrate_tilt.sh ~/bagfiles/fiesta_calib_20250525_*.bag
#
#   # 调整 pitch 补偿前后倾斜
#   ./calibrate_tilt.sh ~/bagfiles/fiesta_calib_20250525_*.bag --pitch 2.0
#
#   # 同时调整三个轴
#   ./calibrate_tilt.sh ~/bagfiles/fiesta_calib_20250525_*.bag \
#       --roll 0.5 --pitch 1.5 --yaw 0.0
#
#   当发现地面变平后，记下 roll/pitch/yaw 的值，
#   然后更新 start_system.sh 中 pointcloud_relay.py 的默认参数
# ============================================

set -e

BAG_FILE=""
MOUNT_ROLL=0.0
MOUNT_PITCH=0.0
MOUNT_YAW=0.0
WITH_FIESTA=false

usage() {
    echo "用法: $0 <bag文件> [选项]"
    echo ""
    echo "参数:"
    echo "  <bag文件>          标定包路径（必填）"
    echo ""
    echo "选项:"
    echo "  --roll <度>         LiDAR 安装角 roll 补偿 (默认: 0.0)"
    echo "  --pitch <度>        LiDAR 安装角 pitch 补偿 (默认: 0.0)"
    echo "  --yaw <度>          LiDAR 安装角 yaw 补偿 (默认: 0.0)"
    echo "  --with-fiesta       同时启动 FIESTA 看建图效果 (默认: 仅点云)"
    echo "  -h, --help          显示帮助"
    exit 1
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --roll)
            MOUNT_ROLL="$2"
            shift 2
            ;;
        --pitch)
            MOUNT_PITCH="$2"
            shift 2
            ;;
        --yaw)
            MOUNT_YAW="$2"
            shift 2
            ;;
        --with-fiesta)
            WITH_FIESTA=true
            shift
            ;;
        -h|--help)
            usage
            ;;
        -*)
            echo "未知选项: $1"
            usage
            ;;
        *)
            if [[ -z "$BAG_FILE" ]]; then
                BAG_FILE="$1"
                shift
            else
                echo "多余参数: $1"
                usage
            fi
            ;;
    esac
done

if [[ -z "$BAG_FILE" ]]; then
    echo "错误: 请指定 bag 文件路径"
    usage
fi

if [[ ! -f "$BAG_FILE" ]]; then
    echo "错误: 文件不存在 -> $BAG_FILE"
    exit 1
fi

source ~/catkin_ws/devel/setup.bash

# 自动启动 roscore（如果还没运行）
if ! rostopic list > /dev/null 2>&1; then
    echo "启动 roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 2
    # 等 roscore 真正就绪
    until rostopic list > /dev/null 2>&1; do sleep 1; done
    echo "roscore 已就绪"
fi

echo "============================================"
echo "FIESTA 地面倾斜标定工具"
echo "  包文件:    $BAG_FILE"
echo "  mount_roll:  $MOUNT_ROLL°"
echo "  mount_pitch: $MOUNT_PITCH°"
echo "  mount_yaw:   $MOUNT_YAW°"
echo "  FIESTA:     $([ "$WITH_FIESTA" = true ] && echo '开启' || echo '关闭')"
echo "============================================"
echo ""
echo "启动节点中..."

# 清理函数
cleanup() {
    echo ""
    echo "正在停止所有节点..."
    kill $BAG_PID $GPS_POSE_PID $CLOUD_PID $FIESTA_PID $ROSCORE_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "已停止，当前使用的参数:"
    echo "  roll=$MOUNT_ROLL  pitch=$MOUNT_PITCH  yaw=$MOUNT_YAW"
    echo "如果地面已变平，请将这些值记下，更新到 start_system.sh 中"
    exit 0
}
trap cleanup SIGINT SIGTERM

# 启动点云变换节点（带安装角参数）
rosrun radar_obstacle_avoidance pointcloud_relay.py \
    _mount_roll:=$MOUNT_ROLL \
    _mount_pitch:=$MOUNT_PITCH \
    _mount_yaw:=$MOUNT_YAW &
CLOUD_PID=$!
sleep 1

# 启动 GPS→位姿 转换节点（回放时也用到 odom → transform）
rosrun radar_obstacle_avoidance gps_to_pose.py &
GPS_POSE_PID=$!
sleep 1

# 可选启动 FIESTA
if [ "$WITH_FIESTA" = true ]; then
    roslaunch fiesta cow_and_lady.launch &
    FIESTA_PID=$!
    sleep 2
fi

# 回放 bag（循环播放）
echo "开始回放: $BAG_FILE"
rosbag play "$BAG_FILE" --loop --rate 1.0 &
BAG_PID=$!

echo ""
echo "============================================"
echo "一切就绪！现在可以打开 RViz 查看效果:"
echo ""
echo "  rosrun rviz rviz -f camera_init"
echo ""
echo "添加 PointCloud2 话题 /cloud_registered"
echo "观察地面是否水平"
echo ""
echo "如果地面仍有倾斜:"
echo "  按 Ctrl+C 停止"
echo "  调整参数重新运行，例如:"
echo "  ./calibrate_tilt.sh $BAG_FILE --pitch 2.5 --roll 0.8"
echo "============================================"

wait