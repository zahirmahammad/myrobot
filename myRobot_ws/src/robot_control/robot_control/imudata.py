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


class IMUData(Node):
    def __init__(self):
        super().__init__('IMUData')
        qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=5)




        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            qos_profile
        )




    def imu_callback(self, msg):
        # IMU data callback
        _,_, self.yaw = self.quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

        print("Yaw: ", round(self.yaw,2))




    def quaternion_to_euler(self, qw, qx, qy, qz):
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
        pitch = math.asin(2 * (qw * qy - qz * qx))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
        return round(roll,3), round(pitch,3), round(yaw,3)










def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUData()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
