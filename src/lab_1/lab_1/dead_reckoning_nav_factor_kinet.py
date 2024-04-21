#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

import threading


class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')

        self.kinet_vision = self.create_subscription(Vector3, '/occupancy_state', self.read_kinet, 10)
        self.obs_detectado = False

        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.cb_sub = self.create_subscription(PoseArray, 'goal_list', self.accion_mover_cb, 10)

        self.list_odom_pose = []
        self.list_real_pose = []
        self.linear_vel = 0.2  # m/s
        self.angular_vel = 1.0  # rad/s

    def aplicar_velocidad(self, speed_command_list):
        for speed_comand in speed_command_list:
            velocidad = Twist()
            velocidad.linear.x = speed_comand[0]
            velocidad.angular.z = speed_comand[1]
            t = time.time() + speed_comand[2]
            tiempo_detenido = 0

            while time.time() < t:

                if self.obs_detectado:
                    td = time.time()
                    if tiempo_detenido == 0:
                        tiempo_detenido = t - td
                    t = td + tiempo_detenido
                    velocidad.linear.x = 0.0
                    velocidad.angular.z = 0.0
                    self.vel_publisher.publish(velocidad)
                else:
                    tiempo_detenido = 0
                    velocidad.linear.x = speed_comand[0]
                    velocidad.angular.z = speed_comand[1]
                    #self.get_logger().info("publicando velocidad (%f, %f) por %f s" % (velocidad.linear.x, velocidad.angular.z, speed_comand[2]))
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
            t = abs(distancia_theta/self.angular_vel) * 1.105  # factor de correccción
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

    def read_kinet(self, kinet_vision):
        izquierda = kinet_vision.x
        centro = kinet_vision.y
        derecha = kinet_vision.z

        if izquierda == 1.0:
            self.get_logger().info("obstacle left")
            self.obs_detectado = True
        elif centro == 1.0:
            self.get_logger().info("obstacle center")
            self.obs_detectado = True
        elif derecha == 1.0:
            self.get_logger().info("obstacle right")
            self.obs_detectado = True
        else:
            self.obs_detectado = False


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
