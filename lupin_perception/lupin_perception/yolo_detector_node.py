#!/usr/bin/env python3
"""
ROS2 node that subscribes to the gripper camera `image_raw` topic and runs YOLO detections.

Subscribes to:
 - /gripper_camera/image_raw (Image)

Publishes:
 - /yolo/image_detections (Image) -- annotated frame with bounding boxes
 - /yolo/detections (String) -- JSON detections

Requires `ultralytics`, `opencv-python`, and `cv_bridge` installed in the environment.
"""

from pathlib import Path
import threading
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        default_model_path = Path(get_package_share_directory('lupin_perception')) / 'best.pt'
        self.declare_parameter('model_path', str(default_model_path))
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)

        model_path = Path(self.get_parameter('model_path').get_parameter_value().string_value)
        self.conf = float(self.get_parameter('conf').get_parameter_value().double_value)
        self.imgsz = int(self.get_parameter('imgsz').get_parameter_value().integer_value)

        if not model_path.exists():
            self.get_logger().error(f"Model not found: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")

        # Load model (Ultralytics YOLO)
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(str(model_path))

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Publisher for annotated detection images
        self.pub_image = self.create_publisher(Image, 'yolo/image_detections', 10)
        # Publisher for detection metadata (JSON list)
        self.pub_detections = self.create_publisher(String, 'yolo/detections', 10)

        # Subscriptions (only raw image topic)
        self.create_subscription(Image, '/gripper_camera/image_raw', self.image_raw_cb, 10)

        self.get_logger().info('YoloDetectorNode ready and listening for images.')

    def image_raw_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return
        self._run_inference(cv_image)

    def _run_inference(self, cv_image: np.ndarray):
        # Run inference in a thread-safe way
        with self.lock:
            try:
                results = self.model.predict(
                    source=cv_image,
                    conf=self.conf,
                    imgsz=self.imgsz,
                    verbose=False,
                )
            except Exception as e:
                self.get_logger().error(f'YOLO inference error: {e}')
                return

        if not results:
            self.get_logger().debug('No results returned from YOLO')
            return

        # Annotated image from the first result
        try:
            annotated = results[0].plot()  # returns an OpenCV BGR image
        except Exception:
            annotated = cv_image

        # Publish annotated image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

        # Log and publish detections as JSON
        detections = []
        for r in results:
            boxes = getattr(r, 'boxes', None)
            if boxes is None:
                continue
            for b in boxes:
                try:
                    cls = int(b.cls[0].item())
                    conf = float(b.conf[0].item())
                    xyxy = [float(x) for x in b.xyxy[0].tolist()]
                    detections.append({
                        'class': cls,
                        'confidence': conf,
                        'bbox_xyxy': xyxy,
                    })
                except Exception:
                    continue

        if detections:
            self.get_logger().info(f'Detections: {detections}')
            try:
                msg = String()
                msg.data = json.dumps(detections)
                self.pub_detections.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish detections JSON: {e}')
        else:
            self.get_logger().debug('No detections')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
