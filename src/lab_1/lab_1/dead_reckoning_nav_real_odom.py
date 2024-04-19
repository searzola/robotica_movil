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


class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')
    
        self.real_pose_sub = self.create_subscription(Pose, '/real_pose', self.read_real_pose, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.read_odom_pose, 10)

        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.cb_sub = self.create_subscription(PoseArray, 'goal_list', self.accion_mover_cb, 10)

        # self.real_pose_sub = self.create_subscription(Pose, '/real_pose', self.read_real_pose, 10)
        # self.odom_sub = self.create_subscription(Odometry, '/odom', self.read_odom_pose, 10)
        self.list_odom_pose = []
        self.list_real_pose = []
        self.linear_vel = 0.2 #m/s
        self.angular_vel = 1.0 #rad/s

    def aplicar_velocidad(self, speed_command_list):
        for speed_comand in speed_command_list:
            velocidad = Twist()
            velocidad.linear.x = speed_comand[0]
            velocidad.angular.z = speed_comand[1]
            t = time.time() + speed_comand[2]
            while time.time() < t:
                # self.get_logger().info("publicando velocidad (%f, %f) por %f s" % (velocidad.linear.x, velocidad.angular.z, speed_comand[2]))
                self.vel_publisher.publish(velocidad)

    def mover_robot_a_destino(self, goal_pose):

        distancia_x = goal_pose.position.x
        distancia_y = goal_pose.position.y
        roll, pitch, yaw =  euler_from_quaternion( ( goal_pose.orientation.x,
                                                     goal_pose.orientation.y,
                                                     goal_pose.orientation.z,
                                                     goal_pose.orientation.w ) )
        distancia_theta = yaw
        lista_comandos_vel = []

        if distancia_x != 0.0:
            if distancia_x < 0.0:
                comando = (0.0, self.angular_vel, 1.0)
            t = abs(distancia_x/self.linear_vel)
            comando = (self.linear_vel, 0.0, t)
            lista_comandos_vel.append(comando)

        if distancia_y != 0.0:
            if distancia_y < 0.0:
                comando = (0.0, self.angular_vel, 1.0)
            t = abs(distancia_y/self.linear_vel)
            comando = (self.linear_vel, 0.0, t)
            lista_comandos_vel.append(comando)

        if distancia_theta != 0.0:
            t = abs(distancia_theta/self.angular_vel)
            if distancia_theta < 0.0:
                comando = (0.0, -self.angular_vel, t)
            else:
                comando = (0.0, self.angular_vel, t)
            lista_comandos_vel.append(comando)

        self.aplicar_velocidad(lista_comandos_vel)

    def accion_mover_cb(self, goal_list):
        threat = threading.Thread(target=self.threat_mover, args=(goal_list,), daemon=True)
        threat.start()
        return

    def threat_mover(self, goal_list):
        for pose in goal_list.poses:
            self.mover_robot_a_destino(pose)
        guardar_datos(self.list_odom_pose, self.list_real_pose)

    def read_real_pose(self, real_pose):
        x = real_pose.position.x
        y = real_pose.position.y
        z = real_pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( real_pose.orientation.x,
                                                    real_pose.orientation.y,
                                                    real_pose.orientation.z,
                                                    real_pose.orientation.w ) )

        self.list_real_pose.append(x)
        self.list_real_pose.append(y)

        self.get_logger().info( 'Current real pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )

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

        self.get_logger().info( 'Current odom pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )


def guardar_datos(real_pose, odom_pose):
    real_pose_x = real_pose[::2]
    real_pose_y = real_pose[1::2]
    odom_pose_x = odom_pose[::2]
    odom_pose_y = odom_pose[1::2]
    plt.plot(real_pose_x, real_pose_y, label="real pose")
    plt.plot(odom_pose_x, odom_pose_y, label="odom pose")
    plt.legend(loc='upper center')
    plt.xlabel('Eje x')
    plt.ylabel('Eje y')
    plt.title('real v/s odom')
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    dead_reckoning_nav_node = DeadReckoningNav()
    rclpy.spin(dead_reckoning_nav_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dead_reckoning_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
