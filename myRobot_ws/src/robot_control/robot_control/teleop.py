#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Define key codes
LIN_VEL_STEP_SIZE = 2
ANG_VEL_STEP_SIZE = 10

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        # self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/joint_velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
             w     
        a    s    d

        q : force stop
       
        Esc to quit
        """

        self.get_logger().info(self.msg)
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_l=0.0
        steer_r=0.0
        temp=0.0    

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_l=0.0
                    steer_r=0.0
                    temp=0.0
                    print("\"q\" key pressed: Stopping.. - Stopped ")
                elif key == 'w':  # Forward
                    steer_l=0
                    steer_r=0
                    linear_vel = temp if temp!=0.0 else linear_vel + LIN_VEL_STEP_SIZE
                    temp=0.0
                    print("\"w\" key pressed: Moving Forward - Velocity - ",linear_vel )
                elif key == 's':  # Reverse
                    steer_l=0
                    steer_r=0
                    linear_vel = temp if temp!=0.0 else linear_vel - LIN_VEL_STEP_SIZE
                    temp=0.0
                    print("\"s\" key pressed: Moving Backward - Velocity - ",linear_vel )
                elif key == 'd':  # Right
                    temp = linear_vel
                    linear_vel=0.0
                    steer_l += ANG_VEL_STEP_SIZE
                    steer_r -= ANG_VEL_STEP_SIZE
                    print("\"d\" key pressed: Turning Right" )
                elif key == 'a':  # Left
                    temp = linear_vel
                    linear_vel=0.0
                    steer_r += ANG_VEL_STEP_SIZE
                    steer_l -= ANG_VEL_STEP_SIZE
                    print("\"a\" key pressed: Turning Left" )



                # if steer_l>1.0:
                #         steer_l=1.0
                # if steer_l<-1.0:
                #     steer_l=-1.0

                # print("Linear Velocity",linear_vel)
                # Publish the twist message
                wheel_velocities.data = [linear_vel+steer_r,linear_vel+steer_l,linear_vel+steer_r, linear_vel+steer_l]
                # joint_positions.data = [0.0,0.0,0.0,0.0,steer_vel,steer_vel]

                # self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()