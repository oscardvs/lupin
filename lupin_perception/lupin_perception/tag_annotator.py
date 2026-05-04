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
    def __init__(self):
        super().__init__('tag_annotator')
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Updated with YOUR specific Camera Matrix (K)
        self.K = np.array([
            [554.254691191187, 0.0, 320.5],
            [0.0, 554.254691191187, 240.5],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((4,1)) 
        self.tag_size = 0.1  # 10cm tags

        # Setup ArUco/AprilTag Detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Subscriber for Gazebo (RELIABLE to match camera_controller)
        image_qos = QoSProfile(
            depth=1, 
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, image_qos)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw_boxed', 2)
        self.get_logger().info("System Online: 3D TF Broadcaster + 2D Video Stream Active")

    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            corners, ids, _ = self.detector.detectMarkers(cv_image)
            
            box_color = (0, 255, 0)    
            box_thickness = 2            
            text_color = (0, 0, 255)   
            text_font = cv2.FONT_HERSHEY_DUPLEX 
            text_scale = 0.4             
            label_offset = 15            # Pixels above the tag to draw text
            if ids is not None:
                for i in range(len(ids)):
                    # 3D Math: Perspective-n-Point
                    c = corners[i].reshape((4, 2)).astype(int)
                    tag_id = ids[i][0]

                    # --- 1. Custom Drawing of the Box Outline ---
                    # Using polylines lets you set color and thickness
                    cv2.polylines(cv_image, [c.reshape((-1, 1, 2))], isClosed=True, 
                                  color=box_color, thickness=box_thickness)

                    # --- 2. Calculate 3D math (Required for distance label) ---
                    obj_pts = np.array([
                        [-self.tag_size/2,  self.tag_size/2, 0],
                        [ self.tag_size/2,  self.tag_size/2, 0],
                        [ self.tag_size/2, -self.tag_size/2, 0],
                        [-self.tag_size/2, -self.tag_size/2, 0]
                    ], dtype=np.float32)

                    success, rvec, tvec = cv2.solvePnP(obj_pts, corners[i], self.K, self.dist_coeffs)
                    
                    if success:
                        self.broadcast_tf(ids[i][0], rvec, tvec, image_msg.header)
                        
                        distance = tvec[2][0]

                        label_text = f"Tag: {tag_id} | Dist: {distance:.2f}m"
                        
                        # Calculate position above top-left corner
                        top_left = tuple(c[0])
                        text_pos = (top_left[0], top_left[1] - label_offset)
                        
                        # Use two putText calls to create a "drop shadow" effect for high contrast
                        # Drop Shadow (Black background)
                        cv2.putText(cv_image, label_text, (text_pos[0]+2, text_pos[1]+2), 
                                    text_font, text_scale, (0, 0, 0), 2)
                        # Main Text
                        cv2.putText(cv_image, label_text, text_pos, 
                                    text_font, text_scale, text_color, 1)
                

            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = image_msg.header
            self.image_pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    def broadcast_tf(self, tag_id, rvec, tvec, header):
        t = TransformStamped()
        t.header.stamp = header.stamp # Keep time synced to the image frame
        t.header.frame_id = header.frame_id 
        t.child_frame_id = f'tag_{tag_id}'

        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[1][0])
        t.transform.translation.z = float(tvec[2][0])

        rot_matrix, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rot_matrix).as_quat()

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
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