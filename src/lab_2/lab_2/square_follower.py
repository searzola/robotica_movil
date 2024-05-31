#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np


class Blue_Follower(Node):
    def __init__(self, kp):
        super().__init__('square_follower')
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.reference = 0
        self.kp = kp
        self.linear_velocity = 0.0
        self.blue_sub = self.create_subscription(Float64, '/blue_square_position', self.p_controller, 10)

    def p_controller(self, data):
        self.get_logger().info('Centro del cuadrado recibido')
        self.state = float(data.data)
        error = self.reference - self.state
        velocidad = Twist() 
        if abs(error) < 0.1: # Avanza linealmente
            velocidad.linear.x = self.linear_velocity
            velocidad.angular.z = 0
        else:
            velocidad.linear.x = self.linear_velocity
            velocidad.angular.z = self.kp * error
        
        self.get_logger().info('Error: "%.2f"' % error)

        self.vel_publisher.publish(velocidad)


def main(args=None):
    rclpy.init()
    blue_stalker = Blue_Follower(kp=0.05)
    rclpy.spin(blue_stalker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    blue_stalker.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()