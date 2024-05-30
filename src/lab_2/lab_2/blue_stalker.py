#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np


class Blue_Stalker(Node):
    def __init__(self):
        super().__init__('blue_stalker')
        self.publisher_ref = self.create_publisher(Float64, 'angular_setpoint', 10)
        msg = Float64()
        msg.data = float(0.0)
        self.publisher_ref.publish(msg)
        self.publisher_state = self.create_publisher(Float64, 'angular_state', 10)
        self.blue_sub = self.create_subscription(Float64, '/blue_square_position', self.read_position, 10)
        self.control_p_sub = self.create_subscription(Float64, 'angular_control_effort', self.velocida_angular, 10)
        self.publisher_velocity = self.create_publisher(Twist, "/commands/velocity", 10)

    def read_position(self, data):
        self.blue_position = float(data.data)
        msg = Float64()
        msg.data = float(self.blue_position)
        self.publisher_state.publish(msg)

    def velocida_angular(self, dato):
        dato = float(dato.data)
        if dato > 0:
            v_angular = min(0.4, dato)
        else:
            v_angular = max(-0.4, dato)
        self.get_logger().info('V: %f' % v_angular)  
        velocidad = Twist()
        velocidad.linear.x = 0.0
        velocidad.angular.z = v_angular
        self.publisher_velocity.publish(velocidad)




def main(args=None):
    rclpy.init()
    blue_stalker = Blue_Stalker()
    rclpy.spin(blue_stalker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    blue_stalker.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()