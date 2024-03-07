import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np


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
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # for i in cv_image:
            #     for j in i:
            #         for k in j:
            #             if k!=255:
            #                 print(j)

            # Define range of red color
            l_red = np.array([50, 0, 0])
            u_red = np.array([255, 100, 25])

            # Green Color
            l_green = np.array([0, 135, 0])
            u_green = np.array([60, 179, 113])

            # Blue Color
            l_blue = np.array([0, 0, 100])
            u_blue = np.array([51, 153, 255])

            # Threshold the RGB image 
            mask_r = cv2.inRange(cv_image, l_red, u_red)
            mask_g = cv2.inRange(cv_image, l_green, u_green)
            mask_b = cv2.inRange(cv_image, l_blue, u_blue)

            if cv2.countNonZero(mask_r)>0:
                print("Red")
                contours, _ = cv2.findContours(mask_r,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                #     print("Area: ", cv2.contourArea(cnt))
                    cv2.drawContours(cv_image, [cnt],-1,(0, 250, 10),3)


            if cv2.countNonZero(mask_g)>0:
                print("Green")
                contours, _ = cv2.findContours(mask_g,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                #     print("Area: ", cv2.contourArea(cnt))
                    cv2.drawContours(cv_image, [cnt],-1,(10, 255, 0),3)

            if cv2.countNonZero(mask_b)>0:
                print("Blue")
                contours, _ = cv2.findContours(mask_b,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                #     print("Area: ", cv2.contourArea(cnt))
                    cv2.drawContours(cv_image, [cnt],-1,(10, 255, 0),3)

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