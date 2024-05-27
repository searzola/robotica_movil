#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

import threading
import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Float64
from std_msgs.msg import Empty

class Wall_Actuation(Node):

    def __init__(self):
        super().__init__('wall_actuation')

        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)

        #self.real_pose_sub = self.create_subscription(Pose, '/real_pose', self.read_real_pose, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.read_odom_pose, 10)
        self.list_odom_pose = []
        self.list_real_pose = []
        self.linear_vel = 0.2 #m/s
        self.setpoint = 0

        self.setpoint_publisher =  self.create_publisher(Float64, 'wall_setpoint', 1)
        self.state_publisher = self.create_publisher(Float64, 'wall_state', 1 )
        self.actuation_sub = self.create_subscription(Float64, 'wall_control_effort', self.angular_actuation_ctrl, 1)

        self.actual_state = 0.0

        self.last_pose = [0.0]
        self.actual_pose_index = 0

    def aplicar_velocidad(self):
            self.angular_publish = True
            angular_setpoint_msg = Float64()
            angular_state_msg = Float64()
            angular_setpoint_msg.data = self.setpoint
            while self.setpoint_publisher.get_subscription_count() == 0:
                if self.setpoint_publisher.get_subscription_count() > 0:
                    break
            self.actual_state = self.angular_state - self.last_pose[self.actual_pose_index]
            angular_state_msg.data = self.actual_state
            self.setpoint_publisher.publish(angular_setpoint_msg)
            self.state_publisher.publish(angular_state_msg)
            while True: 
                #self.get_logger().info( 'Actual state: %f , angular_state: %f, Setpoint: %f' % (self.actual_ang_state, self.angular_state, speed_comand[1]))
                if float(self.setpoint) - float(round(self.actual_state, 2)) < 0.0001:
                    self.get_logger().info( 'Angular setpoint reached')
                    self.last_pose.append(self.angular_state)
                    self.actual_pose_index += 1
                    self.get_logger().info( 'actual_pose x: %f, y: %f , thetha: %f '% (self.last_pose[self.actual_pose_index][0], self.last_pose[self.actual_pose_index][1], self.last_pose[self.actual_pose_index][2]))
                    break

    def angular_actuation_ctrl(self, ang_control_effort):
        actuation = float(ang_control_effort.data)
        state_msg = Float64()
        velocidad = Twist()   
        comando_vel = min(actuation, self.max_angular_vel)
        velocidad.angular.z = comando_vel
        velocidad.linear.x = self.linear_vel
        self.vel_publisher.publish(velocidad)
        self.actual_state = self.angular_state - self.last_pose[self.actual_pose_index]
        state_msg.data = self.actual_ang_state
        self.state_publisher.publish(state_msg)


    def accion_mover_cb(self, goal_list):
        thread = threading.Thread(target=self.threat_mover, args=(goal_list,))
        thread.start()
        return

    def threat_mover(self, goal_list):
        self.aplicar_velocidad()
        self.guardar_datos(self.list_odom_pose, self.list_real_pose)


    def read_odom_pose(self, odom_pose):
        x = odom_pose.pose.pose.position.x
        y = odom_pose.pose.pose.position.y
        z = odom_pose.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( odom_pose.pose.pose.orientation.x,
                                                    odom_pose.pose.pose.orientation.y,
                                                    odom_pose.pose.pose.orientation.z,
                                                    odom_pose.pose.pose.orientation.w ) )

        self.list_odom_pose.append(x+1)
        self.list_odom_pose.append(y+1)

        self.linear_state_x = x
        self.linear_state_y = y
        if yaw >= 0:
            self.angular_state = yaw
        else:
            self.angular_state = 6.28 + yaw

        #self.get_logger().info( 'Current odom pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )


    def guardar_datos(self, real_pose, odom_pose):
        yf = real_pose[-1]
        xf = real_pose[-2]
        xi = real_pose[0]
        yi = real_pose[1]
        d = np.sqrt((xf - xi)**2 + (yf - yi)**2)
        self.get_logger().info('diferencia: %s"' % d)
        real_pose_x = real_pose[::2]
        real_pose_y = real_pose[1::2]
        odom_pose_x = odom_pose[::2]
        odom_pose_y = odom_pose[1::2]
        plt.plot(real_pose_x, real_pose_y, label="real pose")
        plt.plot(odom_pose_x, odom_pose_y, label="odom pose")
        plt.legend(loc='upper center')
        plt.xlabel('Eje x')
        plt.ylabel('Eje y')
        plt.title('real v/s odom + factor')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    dead_reckoning_nav_node = Wall_Actuation()
    rclpy.spin(dead_reckoning_nav_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dead_reckoning_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
