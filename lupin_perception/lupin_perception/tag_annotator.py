import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import message_filters
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class TagAnnotator(Node):
    def __init__(self):
        super().__init__('tag_annotator')
        self.bridge = CvBridge()
        
        # Subscribe to both the camera and the mathematical detections
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        self.tag_sub = message_filters.Subscriber(self, AprilTagDetectionArray, '/detections')
        
        # Synchronize them so we only draw on the exact frame the tag was seen in
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.tag_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # The topic your HMI will subscribe to
        self.image_pub = self.create_publisher(Image, '/camera/image_raw_boxed', 2)
        # self.frame_count = 0
        self.get_logger().info("Faster Tag Annotator Node Started! Drawing boxes...")

    def sync_callback(self, image_msg, detections_msg):
        # self.frame_count += 1
        # if self.frame_count % 2 != 0:
        #     return
        try:
            # Convert ROS image to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Loop through every tag the robot currently sees
            for detection in detections_msg.detections:
                tag_id = detection.id
                corners = detection.corners
                
                # Extract the 4 pixel coordinates of the tag's corners
                # ptA = (int(corners[0].x), int(corners[0].y)
                pts = np.array([
                    [corners[0].x, corners[0].y],
                    [corners[1].x, corners[1].y],
                    [corners[2].x, corners[2].y],
                    [corners[3].x, corners[3].y]
                ], np.int32).reshape((-1, 1, 2))
                
                # Draw the bounding box (Green, 3px thick)
                # cv2.line(cv_image, ptA, ptB, (0, 255, 0), 3)
                # cv2.line(cv_image, ptB, ptC, (0, 255, 0), 3)
                # cv2.line(cv_image, ptC, ptD, (0, 255, 0), 3)
                # cv2.line(cv_image, ptD, ptA, (0, 255, 0), 3)
                cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
                
                # Draw the ID label above the box (Red)
                # text = f"ID: {tag_id}"
                # cv2.putText(cv_image, text, (ptA[0], ptA[1] - 15),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(cv_image, f"ID: {detection.id}", (int(corners[0].x), int(corners[0].y) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Convert back to a ROS message and publish
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = image_msg.header
            self.image_pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to draw: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TagAnnotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()