import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Set the QoS profile to match the camera node's QoS settings
        qos_profile = QoSProfile(
            depth=10,  # Adjust the depth as needed
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Set the reliability policy
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',  # Update the topic to match your camera topic
            self.image_callback,
            qos_profile  # Use the QoS profile
        )

        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Display the image using OpenCV
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)  # Adjust the delay time as needed
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    camera_viewer = CameraViewer()

    rclpy.spin(camera_viewer)

    camera_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()