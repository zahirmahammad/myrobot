import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
import time


class Autonomous(Node):
    def __init__(self):
        super().__init__('Autonomous')
        qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=5)

        self.camera_subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',  # Update the topic to match your camera topic
            self.image_callback,
            qos_profile  # Use the QoS profile
        )

        self.sub_node1 = rclpy.create_node('imu_data_subscriber_node')

        self.imu_subscription = self.sub_node1.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            qos_profile
        )

        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.cv_bridge = CvBridge()
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/joint_velocity_controller/commands', 10)
        self.linear_vel = 10.0
        self.steer = 10.0
        self.wheel_velocities = Float64MultiArray()
        self.yaw=0.0
        self.front=0.0
        self.left=0.0
        self.right=0.0
        self.cv_image=None
        self.goal = False


    def image_callback(self, msg):
        try:
            cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Camera Feed', 900, 600)  # Set the desired width and height

            # Convert the ROS Image message to a CV2 image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Define range of red color
            l_red = np.array([50, 0, 0])
            u_red = np.array([255, 100, 25])

            # Green Color
            l_green = np.array([0, 100, 0])
            u_green = np.array([60, 179, 113])

            # Blue Color
            l_blue = np.array([0, 0, 100])
            u_blue = np.array([51, 153, 255])

            # Threshold the RGB image 
            mask_r = cv2.inRange(cv_image, l_red, u_red)
            mask_g = cv2.inRange(cv_image, l_green, u_green)
            mask_b = cv2.inRange(cv_image, l_blue, u_blue)

            if cv2.countNonZero(mask_r)>0:
                self.moveforward()
                cv2.putText(self.cv_image, "Right Ahead", (890, 90), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(self.cv_image, "Moving Forward..", (880, 590), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(self.cv_image, "Red Square Detected", (10, 590), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                self.color='red'
                contours, _ = cv2.findContours(mask_r,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    # print("Area: ", cv2.contourArea(cnt))
                    if cv2.contourArea(cnt)>6000 or self.front<0.1:
                        self.stop()
                        self.turnright()
                    cv2.drawContours(self.cv_image, [cnt],-1,(0, 250, 10),3)


            elif cv2.countNonZero(mask_g)>0:
                self.moveforward()
                cv2.putText(self.cv_image, "Green Square Detected", (10, 590), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                self.color='green'
                contours, _ = cv2.findContours(mask_g,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    # print("Area: ", cv2.contourArea(cnt))
                    if cv2.contourArea(cnt)>6000 or self.front<0.1:
                        self.stop()
                        self.turnright()
                        self.moveforward()
                        self.goal=True
                    cv2.drawContours(self.cv_image, [cnt],-1,(10, 255, 0),3)

            elif cv2.countNonZero(mask_b)>0:
                self.moveforward()
                cv2.putText(self.cv_image, "Left Ahead", (890, 90), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(self.cv_image, "Blue Square Detected", (10, 590), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                self.color='blue'
                contours, _ = cv2.findContours(mask_b,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    # print("Area: ", cv2.contourArea(cnt))
                    if cv2.contourArea(cnt)>6000 or self.front<0.1:
                        self.stop()
                        self.turnleft()
                    cv2.drawContours(self.cv_image, [cnt],-1,(10, 255, 0),3)
            else:
                if self.goal:
                    cv2.putText(self.cv_image, "End goal Reached!", (300, 250), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(self.cv_image, "Stopped!", (880, 590), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(self.cv_image, "No Squares Detected", (10, 590), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                self.stop()

            cv2.imshow('Camera Feed', self.cv_image)
            cv2.waitKey(1)  # Adjust the delay time as needed
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


    def imu_callback(self, msg):
        # IMU data callback
        _,_, self.yaw = self.quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)



    def scan_callback(self, msg):
        # Get the range measurements
        ranges = msg.ranges
        # Get distance measurements in each direction
        self.left = ranges[11:]  # 90 degrees left
        self.front = ranges[10]  # Front
        self.right = ranges[:10]  # 90 degrees right
        # print(f"left Distance: {self.left}")
        # print(f"right Distance: {self.right}")
        # print(f"Front Distance: {self.front}")

        if any(value<0.4 for value in self.left):
            self.turnright_angle(2)
        if any(value<0.4 for value in self.right):
            self.turnleft_angle(2)




    def quaternion_to_euler(self, qw, qx, qy, qz):
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
        pitch = math.asin(2 * (qw * qy - qz * qx))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
        return round(roll,3), round(pitch,3), round(yaw,3)



    def yaw_correction(self, yaw):
        correction=[]
        # 0 degree
        correction.append(yaw)
        # 1.57
        correction.append(-1.57+yaw)
        # -1.57
        correction.append(1.57+yaw)
        # 3.14
        correction.append(-3.14+yaw)
        # -3.14
        correction.append(3.14+yaw)

        # find the index of absolute minimum number
        min_index = min(range(len(correction)), key=lambda i: abs(correction[i]))

        return correction[min_index]    
    


    def moveforward(self):
        print("Moving front")
        cv2.putText(self.cv_image, "Moving Forward..", (880, 590), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        self.wheel_velocities.data = [self.linear_vel, self.linear_vel, self.linear_vel, self.linear_vel]
        self.wheel_velocities_pub.publish(self.wheel_velocities)



    def movebackward(self):
        self.wheel_velocities.data = [-self.linear_vel, -self.linear_vel, -self.linear_vel, -self.linear_vel]
        self.wheel_velocities_pub.publish(self.wheel_velocities)



    def turnleft(self):
        rclpy.spin_once(self.sub_node1)
        current_yaw = self.yaw
        c_value = self.yaw_correction(current_yaw)
        print("turning left")
        while abs(self.yaw - current_yaw)<math.radians(90)-c_value:
            # print("Correction: ", c_value)
            # print("target Turn: ", math.radians(90)-c_value )
            rclpy.spin_once(self.sub_node1)
            self.wheel_velocities.data = [self.steer, -self.steer, self.steer, -self.steer]
            self.wheel_velocities_pub.publish(self.wheel_velocities)
        self.stop()



    def turnleft_angle(self, angle):
        rclpy.spin_once(self.sub_node1)
        current_yaw = self.yaw
        while abs(self.yaw - current_yaw)<math.radians(angle):
            rclpy.spin_once(self.sub_node1)
            self.wheel_velocities.data = [self.steer, -self.steer, self.steer, -self.steer]
            self.wheel_velocities_pub.publish(self.wheel_velocities)
        self.stop()


    def turnright(self):
        rclpy.spin_once(self.sub_node1)
        current_yaw = self.yaw
        c_value = self.yaw_correction(current_yaw)
        print("turning right")
        while abs(self.yaw - current_yaw)<math.radians(90)+c_value:
            # print("Correction: ", c_value)
            # print("target Turn: ", math.radians(90)+c_value )
            rclpy.spin_once(self.sub_node1)
            self.wheel_velocities.data = [-self.steer, self.steer, -self.steer, self.steer]
            self.wheel_velocities_pub.publish(self.wheel_velocities)
        self.stop()


    def turnright_angle(self, angle):
        rclpy.spin_once(self.sub_node1)
        current_yaw = self.yaw
        while abs(self.yaw - current_yaw)<math.radians(angle):
            rclpy.spin_once(self.sub_node1)
            self.wheel_velocities.data = [-self.steer, self.steer, -self.steer, self.steer]
            self.wheel_velocities_pub.publish(self.wheel_velocities)
        self.stop()



    def stop(self):
        self.wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
        self.wheel_velocities_pub.publish(self.wheel_velocities)




def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = Autonomous()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
