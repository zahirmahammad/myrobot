import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=5)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )


    def scan_callback(self, msg):
        # Get the range measurements
        ranges = msg.ranges
        # Assuming the lidar only has 3 directions: left, front, and right
        # Get distance measurements in each direction
        left_distance = ranges[2]  # 90 degrees left
        front_distance = ranges[1]  # Front
        right_distance = ranges[0]  # 90 degrees right

        # Print the distances
        print(f"Left Distance: {left_distance}")
        print(f"Front Distance: {front_distance}")
        print(f"Right Distance: {right_distance}")

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
