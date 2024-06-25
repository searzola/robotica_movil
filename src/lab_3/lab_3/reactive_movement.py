#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2

import threading

class ReactiveMovement(Node):

    def __init__(self):
        super().__init__('reactive_movement')
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        #self.cb_sub = self.create_subscription(PoseArray, 'goal_list', self.accion_mover_cb, 10)
        self.cb_obstacle_sub = self.create_subscription(LaserScan, '/scan', self.deteccion_obstaculo, 10)
        self.linear_vel = 0.2 #m/s
        self.angular_vel = 1.0 #rad/s
        self.factor_correccion = 1.0922
        self.obs_detectado = False
    
        self.resolucion = 0.01
        file_path = 'robotica_movil/src/lab_3/lab_3/mapa.pgm'
        img = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
        self.img = img
        self.img_copy = img.copy()
        self.len_img = len(img)
        self.last_full_turn_left = False
        self.turn_direction = 1
        self.last_change = 0.0

        self.ver_imagen = False
        threat = threading.Thread(target=self.ver, daemon=True)
        threat.start()

    def aplicar_velocidad(self, speed_command):
        #for speed_comand in speed_command_list:
        velocidad = Twist()
        velocidad.linear.x = speed_command[0]
        velocidad.angular.z = speed_command[1]
        t = time.time() + speed_command[2]
        while time.time() < t:
            self.vel_publisher.publish(velocidad)
            time.sleep(0.05)

                

    def deteccion_obstaculo(self, scan):
        #se toman las mediciones del lidar y se agrupan las 3 más a la izquierda, 3 al centro y 3 a la derecha para sacar un promedio de su medición
        #self.get_logger().info("min angle: %f" % scan.angle_min)
        #self.get_logger().info("max angle: %f" % scan.angle_max)
        #self.get_logger().info("angle increment: %f" % scan.angle_increment)

        laser_distance_array = np.array(scan.ranges)[62:-62] #[::-1]
        #self.get_logger().info("laser_distance_array length: %f" % len(laser_distance_array))
        derecha = (laser_distance_array[0] + laser_distance_array[1] + laser_distance_array[2])/3
        #self.get_logger().info("distancia derecha 1: %f" % laser_distance_array[0])
        #self.get_logger().info("distancia derecha 2: %f" % laser_distance_array[1])
        #self.get_logger().info("distancia derecha 3: %f" % laser_distance_array[2])
        #self.get_logger().info("distancia derecha: %f" % derecha)
        centro = (laser_distance_array[27] + laser_distance_array[28] + laser_distance_array[29])/3
        #self.get_logger().info("distancia centro 1: %f" % laser_distance_array[27])
        #self.get_logger().info("distancia centro 2: %f" % laser_distance_array[28])
        #self.get_logger().info("distancia centro 3: %f" % laser_distance_array[29])
        #self.get_logger().info("distancia centro: %f" % centro)
        izquierda = (laser_distance_array[54] + laser_distance_array[55] + laser_distance_array[56])/3
        #self.get_logger().info("distancia izquierda 1: %f" % laser_distance_array[54])
        #self.get_logger().info("distancia izquierda 2: %f" % laser_distance_array[55])
        #self.get_logger().info("distancia izquierda 3: %f" % laser_distance_array[56])
        #self.get_logger().info("distancia izquierda: %f" % izquierda)

        """laser_index_list = []

        for n in laser_index_list:
            test_z = laser_distance_array[n]
            z_theta = np.deg2rad(n - 28.5)
            self.look_map_especial(0.5, 0.5, 0.0,test_z, z_theta)"""

        speed_command = [0.2, 0.0, 0.1] #m/s, rad/s, s
        if centro < 0.5:
            #self.get_logger().info("obstacle center")
            self.obs_detectado = True
            if self.last_full_turn_left:
                if time.time() - self.last_change > 5:
                    self.last_full_turn_left = False
                    self.turn_direction = -1
                    self.last_change = time.time()
                    self.get_logger().info("next turn left")

            else:
                if time.time() - self.last_change > 5:
                    self.last_full_turn_left = True
                    self.turn_direction = 1
                    self.last_change = time.time()
                    self.get_logger().info("next turn right")
            speed_command = [0.0, self.turn_direction*1.0, np.random.uniform(1,3)]
        elif derecha < 0.5:
            #self.get_logger().info("obstacle right")
            self.obs_detectado = True
            speed_command = [0.2, 1.0, 0.06]
        elif izquierda < 0.5:
            #self.get_logger().info("obstacle left")
            self.obs_detectado = True
            speed_command = [0.2, -1.0, 0.06]
        else:
            self.obs_detectado = False
            speed_command = [0.2, 0.0, 0.06]
        self.aplicar_velocidad(speed_command)


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
                comando = (0.0, self.angular_vel, 1*self.factor_correccion)
            t = abs(distancia_x/self.linear_vel)
            comando = (self.linear_vel, 0.0, t)
            lista_comandos_vel.append(comando)

        if distancia_y != 0.0:
            if distancia_y < 0.0:
                comando = (0.0, self.angular_vel, 1*self.factor_correccion)
            t = abs(distancia_y/self.linear_vel)
            comando = (self.linear_vel, 0.0, t)
            lista_comandos_vel.append(comando)

        if distancia_theta != 0.0:
            t = abs(distancia_theta/self.angular_vel) * self.factor_correccion #factor de correccción
            if distancia_theta < 0.0:
                comando = (0.0, -self.angular_vel, t)
            else:
                comando = (0.0, self.angular_vel, t)
            lista_comandos_vel.append(comando)
            
        self.aplicar_velocidad(lista_comandos_vel)
            
    def accion_mover_cb(self, goal_list):
        thread = threading.Thread(target=self.thread_movimiento, args=(goal_list,))
        thread.start()
        return
    
    def thread_movimiento(self, goal_list):
        for pose in goal_list.poses:
            self.mover_robot_a_destino(pose)
    
    def look_map_especial(self, x, y, theta, z, z_theta):
        #x, y, theta, z, z_theta = data
        coordenadas = self.calcular_pose_map(x, y, theta, z, z_theta)
        #x, y = coordenadas
        #
        #x, y, theta, z, z_theta = data
        x = int(round(x/self.resolucion, 0))
        y = self.len_img - int(round(y/self.resolucion, 0))
        origen = (y, x)
        #self.get_logger().info("origen: %f , %f" % (origen[0], origen[1]))
        self.img_copy[origen] = 0
        self.img_copy[(min(origen[0]+1, 269), origen[1])] = 0
        self.img_copy[(min(origen[0]+2, 269), origen[1])] = 0
        self.img_copy[(max(origen[0]-1, 0), origen[1])] = 0
        self.img_copy[(max(origen[0]-2, 0), origen[1])] = 0
        self.img_copy[(origen[0], min(origen[1]+1, 269))] = 0
        self.img_copy[(origen[0], min(origen[1]+2, 269))] = 0
        self.img_copy[(origen[0], max(origen[1]-1, 0))] = 0
        self.img_copy[(origen[0], max(origen[1]-2, 0))] = 0
        #####################################
        x, y = coordenadas
        if 0 <= x < 270 and 0 <= y < 270:
            self.img_copy[coordenadas] = 0
            self.img_copy[(min(coordenadas[0]+1, 269), coordenadas[1])] = 0
            self.img_copy[(min(coordenadas[0]+2,269), coordenadas[1])] = 0
            self.img_copy[(max(coordenadas[0]-1, 0), coordenadas[1])] = 0
            self.img_copy[(max(coordenadas[0]-2, 0), coordenadas[1])] = 0
            self.img_copy[(coordenadas[0], min(coordenadas[1]+1, 269))] = 0
            self.img_copy[(coordenadas[0], min(coordenadas[1]+2, 269))] = 0
            self.img_copy[(coordenadas[0], max(coordenadas[1]-1, 0))] = 0
            self.img_copy[(coordenadas[0], max(coordenadas[1]-2, 0))] = 0


    def calcular_pose_map(self, x, y, theta, z, theta_z, x_y=False):
            cos_theta = np.cos(theta)
            sen_theta = np.sin(theta)
            cos_p_z_theta = np.cos(theta + theta_z)
            sen_p_z_theta = np.sin(theta + theta_z)
            pose_x = x
            pose_y = y
            pose = np.array([[pose_x], [pose_y]])
            pose_senor = np.array([[-0.0], [0.0]])
            vector_rotacion = np.array([[cos_theta, -sen_theta], [sen_theta, cos_theta]])
            z_position = np.array([[cos_p_z_theta], [sen_p_z_theta]])
            dot = np.dot(vector_rotacion, pose_senor)
            coordenadas = pose + dot + z*z_position
            #self.get_logger().info("coordenada laser hit: %f , %f" % (coordenadas[0], coordenadas[1]))
            coordenadas /= self.resolucion
            coordenadas = coordenadas.round(0).astype(np.uint16)
            coordenadas = tuple(coordenadas.T.tolist().pop())
            coordenadas = (self.len_img - coordenadas[1], coordenadas[0])
            if x_y:
                pose_x = int(round(pose_x/self.resolucion, 0))
                pose_y = self.len_img - int(round(pose_y/self.resolucion, 0))
                return [coordenadas, (pose_y, pose_x)]
            return coordenadas

    def ver(self):
            while True:
                cv2.imshow('raw RGB', self.img_copy)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

def main(args=None):
    rclpy.init(args=args)
    reactive_movement_node = ReactiveMovement()
    rclpy.spin(reactive_movement_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reactive_movement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
