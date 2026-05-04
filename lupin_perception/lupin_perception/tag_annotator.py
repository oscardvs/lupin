"""
AprilTag Detection and Annotation Node

This module implements a custom AprilTag detection system using OpenCV's native
ArUco detector. It replaces the external apriltag_ros package, providing:

- Real-time AprilTag detection (36h11 family)
- 3D pose estimation using perspective-n-point (PnP) solver
- TF2 transform broadcasting for detected tags
- Annotated video stream visualization

The system uses camera calibration parameters (camera matrix and distortion coefficients)
to compute accurate 3D positions of tags in the camera frame. All poses are broadcast
via TF2 for integration with navigation and manipulation tasks.

Architecture:
    1. Subscribe to camera stream (/camera/image_raw)
    2. Detect AprilTags using OpenCV ArUco detector
    3. For each detected tag:
       - Solve PnP to get rotation and translation vectors
       - Broadcast TF transform from camera frame to tag frame
    4. Publish annotated image with bounding boxes and distance labels

Topics:
    Subscriptions:
        - /camera/image_raw (sensor_msgs/Image): Input camera stream
    Publications:
        - /camera/image_raw_boxed (sensor_msgs/Image): Annotated video with tag overlays
        - /tf (tf2_msgs/TFMessage): TF transforms for each detected tag

Transforms:
    Broadcasts: camera_optical_frame → tag_<id>
    - Position: 3D location of tag in camera frame
    - Orientation: Tag's rotation relative to camera

Author: Team Lupin
Replaces: apriltag_ros (now using native OpenCV ArUco detector)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R

class FinalTagSystem(Node):
    """
    AprilTag detection and annotation node using OpenCV.
    
    This node:
    - Detects AprilTags (36h11 family) in camera images
    - Computes 3D poses using camera calibration
    - Broadcasts TF transforms for detected tags
    - Publishes annotated video for visualization
    """

    def __init__(self):
        super().__init__('tag_annotator')
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera Calibration Parameters
        # These are intrinsic camera parameters obtained from calibration.
        # Format: K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        # where: fx, fy = focal length (pixels)
        #        cx, cy = principal point (image center in pixels)
        # CUSTOMIZE FOR YOUR CAMERA: Run camera calibration to obtain these values
        self.K = np.array([
            [554.254691191187, 0.0, 320.5],      # Focal length X, Principal point X
            [0.0, 554.254691191187, 240.5],      # Focal length Y, Principal point Y
            [0.0, 0.0, 1.0]                       # Homography scale
        ], dtype=np.float32)
        
        # Distortion coefficients (k1, k2, p1, p2) for lens distortion correction
        # Set to zeros for cameras without significant distortion (e.g., Gazebo simulation)
        self.dist_coeffs = np.zeros((4, 1))
        
        # Physical size of AprilTags in meters (all tags are 10cm x 10cm)
        self.tag_size = 0.1  # 10 cm tags
        
        # ============= ArUco/AprilTag Detector Setup =============
        # OpenCV's ArUco module can detect AprilTags using predefined marker families.
        # DICT_APRILTAG_36h11 is the standard AprilTag family (36 bits, Hamming distance 11).
        # This provides robust detection and error correction.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # ============= ROS2 Publishers and Subscribers =============
        # QoS configuration for Gazebo camera compatibility
        # RELIABLE delivery ensures no frames are dropped
        # KEEP_LAST with depth=1 maintains real-time behavior
        image_qos = QoSProfile(
            depth=1, 
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Subscribe to camera input
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            image_qos
        )
        
        # Publish annotated frames (with bounding boxes and labels)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw_boxed', 2)
        
        self.get_logger().info("Tag annotator online: AprilTag detection active")

    def image_callback(self, image_msg):
        """
        Process incoming camera frames to detect and annotate AprilTags.
        
        For each frame:
        1. Convert ROS message to OpenCV format
        2. Detect AprilTags using ArUco detector
        3. For each detected tag:
           - Solve PnP to get 3D pose
           - Broadcast TF transform
           - Draw visualization
        4. Publish annotated frame
        
        Args:
            image_msg: ROS Image message from camera
        """
        try:
            # Convert ROS Image message to OpenCV Mat
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Detect AprilTags in the image
            # Returns: corners (array of detected corners), ids (array of tag IDs)
            corners, ids, _ = self.detector.detectMarkers(cv_image)
            
            # Visualization parameters
            box_color = (0, 255, 0)          # Green bounding box
            box_thickness = 2                # Box line thickness (pixels)
            text_color = (0, 0, 255)         # Red text (BGR format)
            text_font = cv2.FONT_HERSHEY_DUPLEX 
            text_scale = 0.4                 # Font size
            label_offset = 15                # Pixels above tag for text label
            
            # Process each detected tag
            if ids is not None:
                for i in range(len(ids)):
                    # Extract corner points and convert to integer coordinates
                    c = corners[i].reshape((4, 2)).astype(int)
                    tag_id = ids[i][0]

                    # --- Draw Bounding Box ---
                    # Use polylines for custom styling (color and thickness control)
                    cv2.polylines(cv_image, [c.reshape((-1, 1, 2))], isClosed=True, 
                                  color=box_color, thickness=box_thickness)

                    # --- Solve Perspective-n-Point Problem ---
                    # PnP finds the 3D pose (rotation + translation) of the tag
                    # by matching 4 known 3D points (corners of the tag square)
                    # with their 2D projections in the image.
                    
                    # Define 3D object points: tag corners in a square with given size
                    # Center the coordinate system at the tag's center
                    obj_pts = np.array([
                        [-self.tag_size/2,  self.tag_size/2, 0],  # Top-left
                        [ self.tag_size/2,  self.tag_size/2, 0],  # Top-right
                        [ self.tag_size/2, -self.tag_size/2, 0],  # Bottom-right
                        [-self.tag_size/2, -self.tag_size/2, 0]   # Bottom-left
                    ], dtype=np.float32)

                    # Solve PnP using camera calibration and detected corners
                    # Returns: success flag, rotation vector, translation vector
                    success, rvec, tvec = cv2.solvePnP(
                        obj_pts,           # 3D tag corners
                        corners[i],        # 2D detected corners in image
                        self.K,            # Camera intrinsic matrix
                        self.dist_coeffs   # Distortion coefficients
                    )
                    
                    if success:
                        # Broadcast TF transform based on PnP solution
                        self.broadcast_tf(ids[i][0], rvec, tvec, image_msg.header)
                        
                        # Extract distance (Z-component of translation)
                        distance = tvec[2][0]

                        # Create label with tag ID and distance
                        label_text = f"Tag: {tag_id} | Dist: {distance:.2f}m"
                        
                        # Calculate text position (above the top-left corner of tag)
                        top_left = tuple(c[0])
                        text_pos = (top_left[0], top_left[1] - label_offset)
                        
                        # Draw semi-opaque background for text readability
                        # Black drop shadow (offset)
                        cv2.putText(cv_image, label_text, (text_pos[0]+2, text_pos[1]+2), 
                                    text_font, text_scale, (0, 0, 0), 2)
                        # Main text
                        cv2.putText(cv_image, label_text, text_pos, 
                                    text_font, text_scale, text_color, 1)
                

            # Publish annotated frame
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = image_msg.header
            self.image_pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    def broadcast_tf(self, tag_id, rvec, tvec, header):
        """
        Broadcast TF transform for detected tag using PnP pose estimation.
        
        This method publishes a transform from the camera optical frame to the tag frame,
        allowing other nodes to use the tag's 3D position and orientation for navigation,
        manipulation, and other tasks.
        
        Args:
            tag_id: The numeric ID of the detected tag
            rvec: Rotation vector (Rodrigues representation) from PnP solver
            tvec: Translation vector from PnP solver (X, Y, Z in camera frame)
            header: ROS header with timestamp and frame_id from the camera frame
            
        Publishes:
            Transform: camera_optical_frame → tag_{tag_id}
            - Translation: 3D position of tag center in camera frame
            - Rotation: Tag orientation (converted from Rodrigues to quaternion)
        """
        # Create TransformStamped message
        t = TransformStamped()
        
        # Timestamp from the image (keeps everything time-synchronized)
        t.header.stamp = header.stamp
        
        # Frame IDs: from camera optical frame to tag frame
        # Typically: "camera_optical_frame" → "tag_0", "tag_1", etc.
        t.header.frame_id = header.frame_id 
        t.child_frame_id = f'tag_{tag_id}'

        # Set translation (position of tag center relative to camera)
        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[1][0])
        t.transform.translation.z = float(tvec[2][0])

        # Convert rotation vector to quaternion
        # Step 1: Rodrigues formula converts rotation vector to rotation matrix
        rot_matrix, _ = cv2.Rodrigues(rvec)
        
        # Step 2: Convert rotation matrix to quaternion (scipy)
        # Quaternion format: [x, y, z, w] where w is the scalar part
        quat = R.from_matrix(rot_matrix).as_quat()

        # Step 3: Set quaternion in transform message
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Broadcast the transform so other nodes can access it via TF2
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    """
    Entry point for the tag_annotator node.
    
    Initializes the ROS 2 node, spins the event loop, and handles shutdown gracefully.
    """
    rclpy.init(args=args)
    node = FinalTagSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()