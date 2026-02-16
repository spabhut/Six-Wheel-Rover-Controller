#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class SixWheelRoverController(Node):
    def __init__(self):
        super().__init__('six_wheel_rover_controller')

        self.pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # === 1. Define Robot Geometry ===
        self.r = 0.1     # Wheel radius
        self.b = 0.275   # Half track width (distance from center to wheel)
        self.d = 0.4    # Distance from Center of Mass to Front/Rear Axle (Adjust this to match your URDF!)

        self.last_msg_time = 0.0
        self.current_cmd = np.array([0.0, 0.0, 0.0]) # [vx, vy, omega]
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Six Wheel Kinematic Controller Started.')

    def cmd_callback(self, msg):
        # === 2. Fix Input Mapping ===
        self.current_cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self.last_msg_time = time.time()

    def control_loop(self):
        if time.time() - self.last_msg_time > 0.5:
            q_dot = np.array([0.0, 0.0, 0.0])
        else:
            q_dot = self.current_cmd

        # === 3. Implement Kinematics ===
        # Rows: [vx contribution, vy contribution, omega contribution]
        S_mid = np.array([
            [self.r/2,           self.r/2],          # vx
            [0,                  0],                 # vy
            [self.r/(2*self.b), -self.r/(2*self.b)]  # omega
        ])

        # S_front (distance = d)
        # From image: Row 2 is (r/2b)*(b*sin + d*cos) -> (r/2b)*d when phi=0
        S_front = np.array([
            [self.r/2,              self.r/2],             # vx
            [(self.r*self.d)/(2*self.b), -(self.r*self.d)/(2*self.b)], # vy (Lateral slip term)
            [self.r/(2*self.b),    -self.r/(2*self.b)]     # omega
        ])

        # S_rear (distance = -d)
        # Same as front but d becomes -d
        S_rear = np.array([
            [self.r/2,              self.r/2],             # vx
            [-(self.r*self.d)/(2*self.b), (self.r*self.d)/(2*self.b)],  # vy
            [self.r/(2*self.b),    -self.r/(2*self.b)]     # omega
        ])

        # INVERT: wheel_vel = pinv(S_total) * q_dot
        # We use Pseudo-Inverse because the matrix is not square (3x2)
        S_inv_mid = np.linalg.pinv(S_mid)
        S_inv_front = np.linalg.pinv(S_front)
        S_inv_rear = np.linalg.pinv(S_rear)

        wheel_speeds_mid = S_inv_mid @ q_dot
        wheel_speeds_front = S_inv_front @ q_dot
        wheel_speeds_rear = S_inv_rear @ q_dot
        
        wLm = wheel_speeds_mid[0]
        wRm = wheel_speeds_mid[1]

        wLf = wheel_speeds_front[0]
        wRf = wheel_speeds_front[1]

        wLr = wheel_speeds_rear[0]
        wRr = wheel_speeds_rear[1]

        # Publish to all 6 wheels
        cmd = Float64MultiArray()
        cmd.data = [wLf, wLm, wLr, wRf, wRm, wRr]
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = SixWheelRoverController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()