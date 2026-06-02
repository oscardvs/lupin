"""
AprilTag Detection and Annotation Node.

OpenCV-ArUco detector (36h11 family) → TF broadcasts + JSON detection
metadata for the HMI overlay. Camera intrinsics are read from a live
`sensor_msgs/CameraInfo` topic so the same node works against any
calibrated camera; no per-bot hard-coded K.

For simulation (where the Gazebo camera plugin may publish CameraInfo
late, or a stripped sensor may omit it) an optional ``fallback_intrinsics``
parameter supplies a pinhole K to fall back on so PnP — and therefore the
tag→map TF the perception aggregator needs — still happens. The fallback is
**opt-in** (default all-zeros = disabled), so on hardware behaviour is
unchanged: an uncalibrated camera still publishes corners-only until real
CameraInfo arrives.

Topics
------
Subscriptions:
    image_topic         (sensor_msgs/Image)        — camera frames
    camera_info_topic   (sensor_msgs/CameraInfo)   — intrinsics + distortion
Publications:
    /camera/tag_detections_json (std_msgs/String)  — JSON for HMI overlay
    /tf (tf2_msgs/TFMessage)                       — tag_<id> in camera frame

Parameters (all overridable via launch / `--ros-args`)
-------------------------------------------------------
image_topic        : str   — default /camera/color/image_raw
camera_info_topic  : str   — default /camera/color/camera_info
detections_topic   : str   — default /camera/tag_detections_json
tag_size_m         : float — physical edge length of the tag, default 0.04
tf_frame_prefix    : str   — broadcast child_frame_id = f"{prefix}{id}", default "tag_"
image_qos          : str   — "sensor_data" | "reliable" (default sensor_data)
fallback_intrinsics: float[]— [fx, fy, cx, cy] used if no CameraInfo latched
                              and fx > 0; default [0,0,0,0] (disabled).

The image QoS defaults to BEST_EFFORT/KEEP_LAST(5) so the node connects
to both the vendor Orbbec driver (RELIABLE) and any BEST_EFFORT
republisher (e.g. `topic_tools throttle`) without manual reconfiguration.
"""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class TagAnnotator(Node):
    """Detect AprilTags, broadcast TF, publish overlay JSON for the HMI."""

    def __init__(self) -> None:
        super().__init__('tag_annotator')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('detections_topic', '/camera/tag_detections_json')
        self.declare_parameter('tag_size_m', 0.04)
        self.declare_parameter('tf_frame_prefix', 'tag_')
        self.declare_parameter('image_qos', 'sensor_data')
        # [fx, fy, cx, cy] — used only when no CameraInfo has been latched
        # and fx > 0. Default disabled so hardware behaviour is unchanged.
        self.declare_parameter('fallback_intrinsics', [0.0, 0.0, 0.0, 0.0])

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        det_topic = self.get_parameter('detections_topic').value
        self.tag_size = float(self.get_parameter('tag_size_m').value)
        self.tf_prefix = str(self.get_parameter('tf_frame_prefix').value)
        image_qos_name = str(self.get_parameter('image_qos').value).lower()

        fb = list(self.get_parameter('fallback_intrinsics').value or [])
        self._fallback_K: np.ndarray | None = None
        if len(fb) == 4 and float(fb[0]) > 0.0 and float(fb[1]) > 0.0:
            fx, fy, cx, cy = (float(v) for v in fb)
            self._fallback_K = np.array(
                [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                dtype=np.float32,
            )
        self._used_fallback = False

        # ── Detector setup ──────────────────────────────────────────────
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11,
        )
        # OpenCV ≥ 4.7 introduced `ArucoDetector` + `DetectorParameters()`
        # constructor; 4.6 (still common on Ubuntu 22.04 / ROS Humble) keeps
        # the old `DetectorParameters_create()` factory + module-level
        # `detectMarkers`. Pick whichever this host ships.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params = cv2.aruco.DetectorParameters()
            _detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self._detect_markers = _detector.detectMarkers
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            _adict, _aparams = self.aruco_dict, self.aruco_params
            self._detect_markers = lambda img: cv2.aruco.detectMarkers(
                img, _adict, parameters=_aparams,
            )

        # Intrinsics — populated on the first CameraInfo message and then
        # cached. Until then, image_callback either falls back to the
        # configured pinhole K (sim) or short-circuits PnP with a throttled
        # warning (hardware, uncalibrated).
        self.K: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None

        # ── ROS plumbing ────────────────────────────────────────────────
        # CameraInfo: RELIABLE + VOLATILE. The DDS rule that bit us once:
        # a TRANSIENT_LOCAL subscriber will REJECT a VOLATILE publisher
        # (incompatible durability — sub demand > pub offer). Orbbec
        # publishes VOLATILE camera_info every frame, so VOLATILE sub gets
        # it within ~1 frame. usb_cam (gripper) latches camera_info and
        # then republishes per-frame while the camera is running; VOLATILE
        # sub also matches there. Stick to VOLATILE so we accept both.
        info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(CameraInfo, info_topic, self._info_cb, info_qos)

        # Image QoS:
        #   - `sensor_data` (default): BEST_EFFORT, KEEP_LAST(5). Connects
        #     to both RELIABLE publishers (downgrade) and BEST_EFFORT
        #     republishers (e.g. topic_tools throttle).
        #   - `reliable`: legacy sim-style profile (Gazebo camera plugin).
        if image_qos_name in ('reliable', 'rel'):
            image_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
        else:
            image_qos = qos_profile_sensor_data

        self.create_subscription(Image, image_topic, self.image_callback, image_qos)
        self.metadata_pub = self.create_publisher(String, det_topic, 2)

        self.get_logger().info(
            f'tag_annotator online — image="{image_topic}" '
            f'camera_info="{info_topic}" tag_size={self.tag_size:.3f} m'
            + (' (fallback intrinsics armed)' if self._fallback_K is not None else ''),
        )

    # ────────────────────────────────────────────────────────────────────
    def _info_cb(self, msg: CameraInfo) -> None:
        """Latch intrinsics on the first valid CameraInfo message we see."""
        if self.K is not None and not self._used_fallback:
            return
        K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        if float(K[0, 0]) <= 0.0 or float(K[1, 1]) <= 0.0:
            return  # zero/garbage K — keep waiting (or stay on fallback)
        # CameraInfo.d is variable-length (Plumb-Bob = 5, Rational = 8…).
        # OpenCV's solvePnP accepts any length, so just pass it through.
        d = np.array(msg.d, dtype=np.float32).reshape(-1, 1) if msg.d else np.zeros((4, 1), dtype=np.float32)
        self.K = K
        self.dist_coeffs = d
        self._used_fallback = False
        self.get_logger().info(
            f'Latched intrinsics from camera_info: fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} '
            f'cx={K[0, 2]:.1f} cy={K[1, 2]:.1f} dist_model="{msg.distortion_model}" '
            f'k_terms={len(msg.d)}',
        )

    # ────────────────────────────────────────────────────────────────────
    def image_callback(self, image_msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except Exception as e:  # cv_bridge raises on encoding mismatches
            self.get_logger().error(f'cv_bridge failed: {e}', throttle_duration_sec=5.0)
            return

        # No real CameraInfo yet? Fall back to the configured pinhole K so
        # sim still computes poses (and the tag→map TF the aggregator needs).
        if self.K is None and self._fallback_K is not None:
            self.K = self._fallback_K
            self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
            self._used_fallback = True
            self.get_logger().warn(
                'No CameraInfo yet — using fallback_intrinsics for PnP. '
                'Will switch to live CameraInfo if/when it arrives.',
                throttle_duration_sec=30.0,
            )

        corners, ids, _ = self._detect_markers(cv_image)

        # solvePnP requires a non-degenerate intrinsics matrix. The gripper
        # cam ships uncalibrated (K all zeros) so we skip pose estimation
        # in that case and still emit corners + id for the HMI overlay —
        # the HMI draws the green box from `tag.corners` alone; `dist` is
        # cosmetic. Once a real calibration lands, pose computation kicks
        # back in automatically.
        has_intrinsics = (
            self.K is not None
            and float(self.K[0, 0]) > 0.0
            and float(self.K[1, 1]) > 0.0
        )
        if not has_intrinsics:
            self.get_logger().warn(
                'CameraInfo missing or uncalibrated (K is zero); publishing '
                'corner overlays without pose. Calibrate the camera or set '
                'fallback_intrinsics if you need accurate tag distances.',
                throttle_duration_sec=30.0,
            )

        detections: list[dict] = []
        if ids is not None:
            half = self.tag_size / 2.0
            obj_pts = np.array(
                [
                    [-half,  half, 0.0],
                    [ half,  half, 0.0],
                    [ half, -half, 0.0],
                    [-half, -half, 0.0],
                ],
                dtype=np.float32,
            )

            for i, raw_id in enumerate(ids):
                tag_id = int(raw_id[0])
                c = corners[i].reshape((4, 2)).astype(int)

                dist = 0.0
                if has_intrinsics:
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_pts, corners[i], self.K, self.dist_coeffs,
                    )
                    if not ok:
                        continue
                    self._broadcast_tf(tag_id, rvec, tvec, image_msg.header)
                    dist = float(tvec[2][0])

                detections.append({
                    'id': tag_id,
                    'corners': c.tolist(),
                    'dist': dist,
                })

        msg = String()
        msg.data = json.dumps(detections)
        self.metadata_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────────
    def _broadcast_tf(self, tag_id: int, rvec, tvec, header) -> None:
        t = TransformStamped()
        t.header.stamp = header.stamp
        # Use the image's frame_id verbatim — the camera driver owns the
        # optical-frame name (e.g. "camera_color_optical_frame" on Orbbec,
        # or the Gazebo camera link's optical frame in sim).
        t.header.frame_id = header.frame_id
        t.child_frame_id = f'{self.tf_prefix}{tag_id}'

        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[1][0])
        t.transform.translation.z = float(tvec[2][0])

        rot_matrix, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rot_matrix).as_quat()  # (x, y, z, w)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagAnnotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
