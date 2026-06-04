#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import onnxruntime as ort
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import time

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        
        # ROS 参数
        self.model_path = rospy.get_param('~model_path', 'best.onnx')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        self.input_size = rospy.get_param('~input_size', 640)
        
        # 类别（根据您的VisDrone10类）
        self.classes = ['pedestrian', 'people', 'bicycle', 'car', 'van', 
                       'truck', 'tricycle', 'awning-tricycle', 'bus', 'motor']
        
        # 颜色映射
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        
        # 初始化ONNX Runtime
        rospy.loginfo(f"Loading YOLOv8 model from {self.model_path}")
        self.session = ort.InferenceSession(
            self.model_path, 
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        self.input_name = self.session.get_inputs()[0].name
        
        # ROS 发布/订阅
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", 
            Image, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        self.detection_pub = rospy.Publisher(
            "/yolo/detections", 
            Detection2DArray,
            queue_size=10
        )
        
        self.visualization_pub = rospy.Publisher(
            "/yolo/image_with_boxes", 
            Image,
            queue_size=10
        )
        
        # 性能监控
        self.frame_count = 0
        self.processing_time = 0
        self.last_log_time = time.time()
        
        rospy.loginfo("YOLO Detector initialized successfully")
    
    def preprocess(self, image):
        """预处理图像"""
        # 调整大小并保持比例
        h, w = image.shape[:2]
        scale = min(self.input_size / w, self.input_size / h)
        new_w, new_h = int(w * scale), int(h * scale)
        
        img_resized = cv2.resize(image, (new_w, new_h))
        
        # 填充黑边
        pad_w = (self.input_size - new_w) // 2
        pad_h = (self.input_size - new_h) // 2
        
        img_padded = cv2.copyMakeBorder(
            img_resized, 
            pad_h, pad_h, pad_w, pad_w, 
            cv2.BORDER_CONSTANT, 
            value=(114, 114, 114)
        )
        
        # 转换格式
        img_rgb = cv2.cvtColor(img_padded, cv2.COLOR_BGR2RGB)
        img_tensor = img_rgb / 255.0
        img_tensor = np.transpose(img_tensor, (2, 0, 1))  # HWC -> CHW
        img_tensor = np.expand_dims(img_tensor, axis=0)   # CHW -> BCHW
        
        return img_tensor.astype(np.float32), scale, pad_w, pad_h
    
    def postprocess(self, outputs, scale, pad_w, pad_h, original_shape):
        """后处理检测结果"""
        h, w = original_shape[:2]
        outputs = outputs[0].transpose(1, 0)
        
        # 筛选置信度
        scores = np.max(outputs[:, 4:4+len(self.classes)], axis=1)
        keep = scores > self.conf_threshold
        
        if not np.any(keep):
            return []
        
        boxes = outputs[keep]
        scores = scores[keep]
        
        # 解析边界框 (x_center, y_center, width, height)
        xc = boxes[:, 0]
        yc = boxes[:, 1]
        bw = boxes[:, 2]
        bh = boxes[:, 3]
        
        # 转换到原始图像坐标
        x1 = (xc - bw/2 - pad_w) / scale
        y1 = (yc - bh/2 - pad_h) / scale
        x2 = (xc + bw/2 - pad_w) / scale
        y2 = (yc + bh/2 - pad_h) / scale
        
        # 限制在图像范围内
        x1 = np.clip(x1, 0, w)
        y1 = np.clip(y1, 0, h)
        x2 = np.clip(x2, 0, w)
        y2 = np.clip(y2, 0, h)
        
        # 获取类别
        class_ids = np.argmax(boxes[:, 4:4+len(self.classes)], axis=1)
        
        # 应用NMS
        indices = self.non_max_suppression(
            np.column_stack([x1, y1, x2, y2]), 
            scores, 
            self.iou_threshold
        )
        
        results = []
        for idx in indices:
            results.append({
                'bbox': [x1[idx], y1[idx], x2[idx], y2[idx]],
                'score': scores[idx],
                'class_id': class_ids[idx],
                'class_name': self.classes[class_ids[idx]]
            })
        
        return results
    
    def non_max_suppression(self, boxes, scores, iou_threshold):
        """NMS实现"""
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            
            intersection = w * h
            iou = intersection / (areas[i] + areas[order[1:]] - intersection)
            
            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]
        
        return keep
    
    def image_callback(self, msg):
        """图像回调函数"""
        start_time = time.time()
        
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            original_shape = cv_image.shape
            
            # 预处理
            img_tensor, scale, pad_w, pad_h = self.preprocess(cv_image)
            
            # 推理
            outputs = self.session.run(None, {self.input_name: img_tensor})
            
            # 后处理
            detections = self.postprocess(outputs, scale, pad_w, pad_h, original_shape)
            
            # 发布检测结果
            self.publish_detections(detections, msg.header)
            
            # 发布可视化图像
            viz_image = self.draw_detections(cv_image.copy(), detections)
            self.publish_visualization(viz_image, msg.header)
            
            # 性能监控
            self.frame_count += 1
            self.processing_time += time.time() - start_time
            
            if time.time() - self.last_log_time > 5.0:
                fps = self.frame_count / 5.0
                avg_time = self.processing_time / self.frame_count
                rospy.loginfo(f"YOLO FPS: {fps:.1f}, Avg time: {avg_time*1000:.1f}ms")
                self.frame_count = 0
                self.processing_time = 0
                self.last_log_time = time.time()
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def publish_detections(self, detections, header):
        """发布检测结果到ROS话题"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        for det in detections:
            detection = Detection2D()
            detection.header = header
            
            # 边界框
            x1, y1, x2, y2 = det['bbox']
            detection.bbox.center.x = (x1 + x2) / 2
            detection.bbox.center.y = (y1 + y2) / 2
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            
            # 类别和置信度
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = det['class_id']
            hypothesis.score = det['score']
            detection.results.append(hypothesis)
            
            detection_array.detections.append(detection)
        
        self.detection_pub.publish(detection_array)
    
    def draw_detections(self, image, detections):
        """在图像上绘制检测框"""
        for det in detections:
            x1, y1, x2, y2 = map(int, det['bbox'])
            class_id = det['class_id']
            score = det['score']
            
            # 颜色
            color = self.colors[class_id].tolist()
            
            # 绘制边界框
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制标签
            label = f"{det['class_name']}: {score:.2f}"
            
            # 标签背景
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            
            cv2.rectangle(
                image, 
                (x1, y1 - text_height - baseline - 10), 
                (x1 + text_width, y1), 
                color, 
                -1
            )
            
            # 标签文字
            cv2.putText(
                image, 
                label, 
                (x1, y1 - baseline - 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (255, 255, 255), 
                1
            )
        
        # 添加FPS信息
        cv2.putText(
            image,
            f"FPS: {self.frame_count / max(time.time() - self.last_log_time, 0.1):.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        return image
    
    def publish_visualization(self, image, header):
        """发布可视化图像"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            ros_image.header = header
            self.visualization_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"Error publishing visualization: {e}")
    
    def run(self):
        """运行节点"""
        rospy.spin()

if __name__ == '__main__':
    detector = YOLODetector()
    detector.run()
