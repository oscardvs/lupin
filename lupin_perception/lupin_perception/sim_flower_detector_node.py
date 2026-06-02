"""sim_flower_detector_node — HSV colour flower detector for Gazebo.

Sim-only stand-in for ``yolo_detector_node``. The real ``best.pt`` YOLO is
trained on photographs of physical dahlias and fires on nothing in Gazebo, so
in sim we segment the bright emissive blossom colours that
``generate_greenhouse_world.py`` paints into the world and publish the **exact
same** ``/yolo/detections`` JSON contract the hardware YOLO does. That keeps
``perception_aggregator`` → twin → HMI byte-identical between sim and hardware:
the detector is the ONLY swap, which is the whole point — minimal sim→hardware
gap. Tomorrow's robot run just launches ``perception_stack.launch.py`` (real
YOLO on the real gripper cam) in place of this node.

Classes (match FlowerObservation.species + yolo_detector_node + the aggregator's
``flower_class_names`` default):
    0 = tulip_red    magenta / hot-pink blossom   (pink hue, HIGH saturation)
    1 = tulip_white  white blossom                (~zero saturation, high value)
    2 = tulip_pink   pale / light pink blossom    (pink hue, LOW saturation)
    3 = bug          dark anomaly marker          (very low value)

Input:  image_topic (sensor_msgs/Image, bgr8) — the gripper camera.
Output: detections_topic (std_msgs/String, JSON):
    [{"class": int, "confidence": float, "bbox_xyxy": [x1,y1,x2,y2]}, ...]
one entry per class whose largest blob is >= min_area_px; confidence scales with
blob area. The aggregator takes the dominant non-bug class as the species while
the robot is SCANNING a tag, and raises ``anomaly`` if the bug class appears.

The HSV bands below are an INITIAL estimate from the world's blossom RGB — the
Gazebo render + lighting will shift them, so they are node parameters: capture a
frame (``ros2 run lupin_perception sim_flower_detector`` publishes an annotated
debug image on ``overlay_topic``) and re-tune. Magenta vs pale-pink separate by
SATURATION, white by ~zero saturation, bug by low value.
"""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


# class id -> (name, list of (h_lo,h_hi) hue bands, (s_lo,s_hi), (v_lo,v_hi))
# OpenCV HSV ranges: H 0-179, S 0-255, V 0-255.
_FLOWER_CLASSES = (
    # 0 tulip_red = magenta/hot-pink: pink-magenta hue, VERY HIGH saturation
    (0, 'tulip_red',   [(150, 179), (0, 8)],   (150, 255), (70, 255)),
    # 1 tulip_white: any hue, ~zero saturation, bright
    (1, 'tulip_white', [(0, 179)],             (0, 38),    (140, 255)),
    # 2 tulip_pink = pale pink: pink hue, MID saturation (gap below red), bright
    (2, 'tulip_pink',  [(140, 179), (0, 14)],  (40, 135),  (140, 255)),
)
_BUG_CLASS = 3            # dark marker
_BUG_V_MAX = 50          # value below this (and not tiny) => bug
_BUG_S_MIN = 0


class SimFlowerDetector(Node):
    """HSV colour-segmentation flower detector, /yolo/detections-compatible."""

    def __init__(self) -> None:
        super().__init__('sim_flower_detector')

        self.declare_parameter('image_topic', '/gripper_camera/image_raw')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('overlay_topic', '/sim_flower/overlay')
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('image_qos', 'reliable')   # Gazebo cam is RELIABLE
        self.declare_parameter('min_area_px', 150)
        self.declare_parameter('conf_area_px', 2000.0)    # area -> confidence 1.0
        self.declare_parameter('morph_kernel', 5)

        image_topic = str(self.get_parameter('image_topic').value)
        det_topic = str(self.get_parameter('detections_topic').value)
        self._overlay_topic = str(self.get_parameter('overlay_topic').value)
        self._publish_overlay = bool(self.get_parameter('publish_overlay').value)
        self._min_area = int(self.get_parameter('min_area_px').value)
        self._conf_area = float(self.get_parameter('conf_area_px').value)
        k = int(self.get_parameter('morph_kernel').value)
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

        self.bridge = CvBridge()

        if str(self.get_parameter('image_qos').value).lower() in ('reliable', 'rel'):
            img_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
        else:
            img_qos = qos_profile_sensor_data

        self.create_subscription(Image, image_topic, self._on_image, img_qos)
        self._det_pub = self.create_publisher(String, det_topic, 5)
        self._overlay_pub = (
            self.create_publisher(Image, self._overlay_topic, 1)
            if self._publish_overlay else None
        )

        self.get_logger().info(
            'sim_flower_detector online — image="%s" -> detections="%s" '
            '(HSV colour stand-in for YOLO; min_area=%d px)'
            % (image_topic, det_topic, self._min_area)
        )

    # ------------------------------------------------------------------
    def _largest_blob(self, mask: np.ndarray):
        """Return (area, bbox_xyxy) of the largest contour in mask, or None."""
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(c))
        if area < self._min_area:
            return None
        x, y, w, h = cv2.boundingRect(c)
        return area, [int(x), int(y), int(x + w), int(y + h)]

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(
                'cv_bridge failed: %s' % e, throttle_duration_sec=5.0,
            )
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        detections: list[dict] = []
        overlay = bgr.copy() if self._overlay_pub is not None else None

        # Flower colour classes
        for cls_id, name, hue_bands, (s_lo, s_hi), (v_lo, v_hi) in _FLOWER_CLASSES:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for h_lo, h_hi in hue_bands:
                lo = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
                hi = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
            found = self._largest_blob(mask)
            if found is None:
                continue
            area, bbox = found
            conf = float(min(1.0, area / max(self._conf_area, 1.0)))
            detections.append({'class': cls_id, 'confidence': conf, 'bbox_xyxy': bbox})
            if overlay is not None:
                cv2.rectangle(overlay, (bbox[0], bbox[1]), (bbox[2], bbox[3]),
                              (0, 255, 0), 2)
                cv2.putText(overlay, f'{name} {conf:.2f}', (bbox[0], max(bbox[1] - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Bug / anomaly: dark compact blob (very low value)
        bug_mask = cv2.inRange(
            hsv,
            np.array([0, _BUG_S_MIN, 0], dtype=np.uint8),
            np.array([179, 255, _BUG_V_MAX], dtype=np.uint8),
        )
        bug = self._largest_blob(bug_mask)
        if bug is not None:
            area, bbox = bug
            conf = float(min(1.0, area / max(self._conf_area, 1.0)))
            detections.append({'class': _BUG_CLASS, 'confidence': conf, 'bbox_xyxy': bbox})
            if overlay is not None:
                cv2.rectangle(overlay, (bbox[0], bbox[1]), (bbox[2], bbox[3]),
                              (0, 0, 255), 2)
                cv2.putText(overlay, f'bug {conf:.2f}', (bbox[0], max(bbox[1] - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        out = String()
        out.data = json.dumps(detections)
        self._det_pub.publish(out)

        if overlay is not None:
            ov = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
            ov.header = msg.header
            self._overlay_pub.publish(ov)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimFlowerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
