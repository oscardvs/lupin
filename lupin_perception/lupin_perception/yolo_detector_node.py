#!/usr/bin/env python3
"""
ROS2 node that subscribes to the gripper camera compressed image topic and runs YOLO detections/tracking.

Subscribes to:
 - /gripper_camera/image_raw/compressed (CompressedImage)

Publishes:
 - /yolo/image_detections (Image) -- annotated frame with bounding boxes and track IDs
 - /yolo/detections (String) -- JSON detections/tracks

Requires `ultralytics`, `opencv-python`, and `cv_bridge` installed in the environment.
"""

from pathlib import Path
import threading
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        default_model_path = (
            Path(get_package_share_directory('lupin_perception')) / 'models' / 'best.pt'
        )

        self.declare_parameter('model_path', str(default_model_path))
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)

        # New tracking parameters
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('tracker', 'bytetrack.yaml')

        model_path = Path(
            self.get_parameter('model_path').get_parameter_value().string_value
        )

        self.conf = float(
            self.get_parameter('conf').get_parameter_value().double_value
        )

        self.imgsz = int(
            self.get_parameter('imgsz').get_parameter_value().integer_value
        )

        self.enable_tracking = bool(
            self.get_parameter('enable_tracking').get_parameter_value().bool_value
        )

        self.tracker = (
            self.get_parameter('tracker').get_parameter_value().string_value
        )

        if not model_path.exists():
            self.get_logger().error(f"Model not found: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(str(model_path))

        self.get_logger().info(f'YOLO tracking enabled: {self.enable_tracking}')
        if self.enable_tracking:
            self.get_logger().info(f'Using tracker: {self.tracker}')

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.pub_image = self.create_publisher(Image, 'yolo/image_detections', 10)
        self.pub_detections = self.create_publisher(String, 'yolo/detections', 10)

        self.create_subscription(
            CompressedImage,
            '/gripper_camera/image_raw/compressed',
            self.image_compressed_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'YoloDetectorNode ready and listening to /gripper_camera/image_raw/compressed'
        )

    def image_compressed_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error('Failed to decode compressed image')
                return

        except Exception as e:
            self.get_logger().error(f'Compressed image decode failed: {e}')
            return

        self._run_inference(cv_image, msg.header)

    def _run_inference(self, cv_image: np.ndarray, header):
        with self.lock:
            try:
                if self.enable_tracking:
                    results = self.model.track(
                        source=cv_image,
                        conf=self.conf,
                        imgsz=self.imgsz,
                        tracker=self.tracker,
                        persist=True,
                        verbose=False,
                    )
                else:
                    results = self.model.predict(
                        source=cv_image,
                        conf=self.conf,
                        imgsz=self.imgsz,
                        verbose=False,
                    )

            except Exception as e:
                self.get_logger().error(f'YOLO inference/tracking error: {e}')
                return

        if not results:
            self.get_logger().debug('No results returned from YOLO')
            return

        try:
            annotated = results[0].plot()
        except Exception:
            annotated = cv_image

        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header = header
            self.pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

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

                    track_id = None
                    if getattr(b, 'id', None) is not None:
                        track_id = int(b.id[0].item())

                    class_name = None
                    if hasattr(self.model, 'names') and cls in self.model.names:
                        class_name = self.model.names[cls]

                    detections.append({
                        'track_id': track_id,
                        'class': cls,
                        'class_name': class_name,
                        'confidence': conf,
                        'bbox_xyxy': xyxy,
                    })

                except Exception as e:
                    self.get_logger().debug(f'Failed to parse detection: {e}')
                    continue

        if detections:
            self.get_logger().info(f'Detections/tracks: {detections}')

            try:
                det_msg = String()
                det_msg.data = json.dumps(detections)
                self.pub_detections.publish(det_msg)

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
