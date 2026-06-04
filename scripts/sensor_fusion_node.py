#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import pcl
import pcl_helper

class SensorFusion:
    def __init__(self):
        rospy.init_node('sensor_fusion')
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 相机内参
        self.camera_info = None
        self.K = None  # 相机内参矩阵
        self.bridge = CvBridge()
        
        # 订阅相机内参
        rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback)
        
        # 同步订阅（时间同步）
        cloud_sub = message_filters.Subscriber("/fast_livo/cloud_registered", PointCloud2)
        detection_sub = message_filters.Subscriber("/tracked_objects", Detection2DArray)
        
        # 近似时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [cloud_sub, detection_sub],
            queue_size=10,
            slop=0.1  # 允许的时间差异（秒）
        )
        self.ts.registerCallback(self.sync_callback)
        
        # 发布融合结果
        self.fusion_pub = rospy.Publisher("/fusion/obstacles", Detection2DArray, queue_size=10)
        
        rospy.loginfo("Sensor Fusion node initialized")
    
    def camera_info_callback(self, msg):
        """获取相机内参"""
        if self.K is None:
            self.K = np.array(msg.K).reshape(3, 3)
            self.camera_info = msg
            rospy.loginfo("Camera intrinsics received")
    
    def project_point_to_image(self, point_3d, transform):
        """将3D点投影到图像平面"""
        # 转换到相机坐标系
        point_camera = np.dot(transform, np.array([point_3d[0], point_3d[1], point_3d[2], 1.0]))
        
        # 投影到图像平面
        if point_camera[2] > 0:  # 确保点在相机前方
            point_2d = np.dot(self.K, point_camera[:3])
            point_2d = point_2d / point_2d[2]
            return point_2d[:2]
        
        return None
    
    def sync_callback(self, cloud_msg, detection_msg):
        """同步回调：融合雷达和视觉数据"""
        if self.K is None:
            rospy.logwarn("Waiting for camera intrinsics")
            return
        
        try:
            # 获取坐标变换：雷达 -> 相机
            transform = self.tf_buffer.lookup_transform(
                detection_msg.header.frame_id,  # 通常是相机坐标系
                cloud_msg.header.frame_id,      # 雷达坐标系
                rospy.Time(0)
            )
            
            # 转换变换为4x4矩阵
            from scipy.spatial.transform import Rotation
            import numpy as np
            
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            rotation = Rotation.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            
            R = rotation.as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = translation
            
            # 处理点云
            cloud = pcl_helper.ros_to_pcl(cloud_msg)
            
            # 提取地面（可选）
            # cloud_filtered = self.remove_ground(cloud)
            
            # 为每个视觉检测寻找对应的雷达点
            fused_detections = Detection2DArray()
            fused_detections.header = detection_msg.header
            
            for detection in detection_msg.detections:
                # 获取检测框
                bbox_center = np.array([
                    detection.bbox.center.x,
                    detection.bbox.center.y
                ])
                
                bbox_size = np.array([
                    detection.bbox.size_x,
                    detection.bbox.size_y
                ])
                
                # 搜索检测框内的雷达点
                points_in_bbox = []
                for point in cloud:
                    point_3d = [point[0], point[1], point[2]]
                    point_2d = self.project_point_to_image(point_3d, T)
                    
                    if point_2d is not None:
                        # 检查点是否在检测框内
                        if (bbox_center[0] - bbox_size[0]/2 <= point_2d[0] <= bbox_center[0] + bbox_size[0]/2 and
                            bbox_center[1] - bbox_size[1]/2 <= point_2d[1] <= bbox_center[1] + bbox_size[1]/2):
                            points_in_bbox.append(point_3d)
                
                if points_in_bbox:
                    # 计算3D边界框
                    points_array = np.array(points_in_bbox)
                    
                    # 最小外接矩形（简化版）
                    min_bound = np.min(points_array, axis=0)
                    max_bound = np.max(points_array, axis=0)
                    
                    # 创建融合检测结果
                    fused_detection = Detection2D()
                    fused_detection.header = detection.header
                    fused_detection.bbox = detection.bbox
                    fused_detection.results = detection.results
                    
                    # 添加3D信息到结果中
                    from geometry_msgs.msg import Pose2D
                    for result in fused_detection.results:
                        result.pose.pose.position.x = min_bound[0]
                        result.pose.pose.position.y = min_bound[1]
                        result.pose.pose.position.z = min_bound[2]
                        result.pose.pose.orientation.x = max_bound[0] - min_bound[0]  # 宽度
                        result.pose.pose.orientation.y = max_bound[1] - min_bound[1]  # 深度
                        result.pose.pose.orientation.z = max_bound[2] - min_bound[2]  # 高度
                    
                    fused_detections.detections.append(fused_detection)
                    
                    rospy.logdebug(f"Fused detection with {len(points_in_bbox)} radar points")
            
            # 发布融合结果
            self.fusion_pub.publish(fused_detections)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF error: {e}")
        except Exception as e:
            rospy.logerr(f"Fusion error: {e}")
    
    def remove_ground(self, cloud):
        """移除地面点（简单版本）"""
        # 使用RANSAC平面分割
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.2)
        
        inliers, coefficients = seg.segment()
        
        # 提取非地面点
        cloud_filtered = cloud.extract(inliers, negative=True)
        
        return cloud_filtered

if __name__ == '__main__':
    fusion = SensorFusion()
    rospy.spin()
