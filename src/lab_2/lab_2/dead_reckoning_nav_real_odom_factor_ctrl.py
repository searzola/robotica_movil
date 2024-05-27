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

class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')

        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.cb_sub = self.create_subscription(PoseArray, 'goal_list', self.accion_mover_cb, 10)

        #self.real_pose_sub = self.create_subscription(Pose, '/real_pose', self.read_real_pose, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.read_odom_pose, 10)
        self.list_odom_pose = []
        self.list_real_pose = []
        self.max_linear_vel = 0.2 #m/s
        self.max_angular_vel = 1.0 #rad/s
        self.factor_correccion = 1.0922 #1.0922

        self.linear_setpoint_publisher =  self.create_publisher(Float64, 'setpoint', 1)
        self.linear_state_publisher = self.create_publisher(Float64, 'state', 1 )
        self.linear_actuation_sub = self.create_subscription(Float64, 'control_effort', self.linear_actuation_ctrl, 1)

        self.angular_setpoint_publisher =  self.create_publisher(Float64, 'angular_setpoint', 1)
        self.angular_state_publisher = self.create_publisher(Float64, 'angular_state', 1 )
        self.angular_actuation_sub = self.create_subscription(Float64, 'angular_control_effort', self.angular_actuation_ctrl, 1)

        self.linear_state_x = 0.0
        self.linear_state_y = 0.0
        self.angular_state = 0.0
        self.actual_state = 0.0
        self.linear_publish = True
        self.angular_publish = True

        self.last_pose = [[0.0, 0.0, 0.0]]
        self.actual_pose_index = 0

        self.init_time = 0.0
        self.linear_setpoint = 0.0
        self.list_linear_time = []
        self.list_linear_actual_state = []
        self.list_linear_setpoint = []
        self.list_linear_control_effort = []
        self.angular_setpoint = 0.0
        self.list_angular_time = []
        self.list_angular_actual_state = []
        self.list_angular_setpoint = []
        self.list_angular_control_effort = []


    def aplicar_velocidad(self, speed_command_list):
        for speed_comand in speed_command_list:
            self.get_logger().info( "Comandos x: %f z: %f" % (speed_comand[0], speed_comand[1]))
            if speed_comand[0] != 0.0:
                self.linear_publish = True
                linear_setpoint_msg = Float64()
                linear_state_msg = Float64()
                linear_setpoint_msg.data = float(speed_comand[0])
                self.linear_setpoint = float(speed_comand[0])
                while self.linear_setpoint_publisher.get_subscription_count() == 0:
                    if self.linear_setpoint_publisher.get_subscription_count() > 0:
                        break
                self.actual_state = np.sqrt((self.linear_state_x - self.last_pose[self.actual_pose_index][0])**2+(self.linear_state_y - self.last_pose[self.actual_pose_index][1])**2)
                linear_state_msg.data = self.actual_state
                self.linear_setpoint_publisher.publish(linear_setpoint_msg)
                self.linear_state_publisher.publish(linear_state_msg)
                #self.init_time = time.time()
                while True: 
                    #self.get_logger().info( 'Actual state: %f , linear_state_x: %f, linear_state_y: %f,  Setpoint: %f' % (self.actual_state, self.linear_state_x, self.linear_state_y, speed_comand[0]))
                    if float(speed_comand[0]) - float(round(self.actual_state, 2)) < 0.0001:
                        self.get_logger().info( 'Linear setpoint reached')
                        self.linear_publish = False
                        break

            elif speed_comand[1] != 0.0:
                self.angular_publish = True
                angular_setpoint_msg = Float64()
                angular_state_msg = Float64()
                angular_setpoint_msg.data = float(speed_comand[1])
                self.angular_setpoint = float(speed_comand[1])
                while self.angular_setpoint_publisher.get_subscription_count() == 0:
                    if self.angular_setpoint_publisher.get_subscription_count() > 0:
                        break
                self.actual_ang_state = self.angular_state - self.last_pose[self.actual_pose_index][2]
                angular_state_msg.data = self.actual_ang_state
                self.angular_setpoint_publisher.publish(angular_setpoint_msg)
                self.angular_state_publisher.publish(angular_state_msg)
                self.init_time = time.time()
                while True: 
                    #self.get_logger().info( 'Actual state: %f , angular_state: %f, Setpoint: %f' % (self.actual_ang_state, self.angular_state, speed_comand[1]))
                    if float(speed_comand[1]) - float(round(self.actual_ang_state, 2)) < 0.0001:
                        self.get_logger().info( 'Angular setpoint reached')
                        self.angular_publish = False
                        self.last_pose.append([self.linear_state_x, self.linear_state_y, self.angular_state])
                        self.actual_pose_index += 1
                        self.get_logger().info( 'actual_pose x: %f, y: %f , thetha: %f '% (self.last_pose[self.actual_pose_index][0], self.last_pose[self.actual_pose_index][1], self.last_pose[self.actual_pose_index][2]))
                        break

    def linear_actuation_ctrl(self, control_effort):
        actuation = float(control_effort.data)
        state_msg = Float64()
        velocidad = Twist()   
        comando_vel = min(actuation, self.max_linear_vel)
        velocidad.linear.x = comando_vel
        self.vel_publisher.publish(velocidad)
        self.actual_state = np.sqrt((self.linear_state_x - self.last_pose[self.actual_pose_index][0])**2+(self.linear_state_y - self.last_pose[self.actual_pose_index][1])**2)
        state_msg.data = self.actual_state

        if self.actual_pose_index == 0:
            actual_time = time.time() - self.init_time
            self.list_linear_time.append(actual_time)
            self.list_linear_actual_state.append(self.actual_state)
            self.list_linear_control_effort.append(comando_vel)
            self.list_linear_setpoint.append(self.linear_setpoint)
            #self.get_logger().info( 't: %f, setpoint: %f' % (actual_time, float(speed_comand[0])) )

        if self.linear_publish:
            self.linear_state_publisher.publish(state_msg)

    def angular_actuation_ctrl(self, ang_control_effort):
        actuation = float(ang_control_effort.data)
        state_msg = Float64()
        velocidad = Twist()   
        comando_vel = min(actuation, self.max_angular_vel)
        velocidad.angular.z = comando_vel
        self.vel_publisher.publish(velocidad)
        self.actual_ang_state = self.angular_state - self.last_pose[self.actual_pose_index][2]
        state_msg.data = self.actual_ang_state

        if self.actual_pose_index == 0:
            actual_time = time.time() - self.init_time
            self.list_angular_time.append(actual_time)
            self.list_angular_actual_state.append(self.actual_ang_state)
            self.list_angular_control_effort.append(comando_vel)
            self.list_angular_setpoint.append(self.angular_setpoint)
            #self.get_logger().info( 't: %f, setpoint: %f' % (actual_time, float(speed_comand[0])) )

        if self.angular_publish:
            self.angular_state_publisher.publish(state_msg)

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
                comando = (0.0, 3.14)
                lista_comandos_vel.append(comando)
            comando = (distancia_x, 0.0)
            lista_comandos_vel.append(comando)

        if distancia_theta != 0.0:
            if distancia_theta < 0.0:
                comando = (0.0, -3.14)
            else:
                comando = (0.0, yaw)
            lista_comandos_vel.append(comando)

        if distancia_y != 0.0:
            if distancia_y < 0.0:
                comando = (0.0, 3.14)
                lista_comandos_vel.append(comando)
            comando = (distancia_y, 0.0)
            lista_comandos_vel.append(comando)

        self.aplicar_velocidad(lista_comandos_vel)

    def accion_mover_cb(self, goal_list):
        thread = threading.Thread(target=self.threat_mover, args=(goal_list,))
        thread.start()
        return

    def threat_mover(self, goal_list):
        for pose in goal_list.poses:
            self.mover_robot_a_destino(pose)
        self.guardar_datos(self.list_odom_pose, self.list_real_pose)

    """def read_real_pose(self, real_pose):
        x = real_pose.position.x
        y = real_pose.position.y
        z = real_pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( real_pose.orientation.x,
                                                    real_pose.orientation.y,
                                                    real_pose.orientation.z,
                                                    real_pose.orientation.w ) )

        self.list_real_pose.append(x)
        self.list_real_pose.append(y)

        self.get_logger().info( 'Current real pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )"""

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
        plt.title('trayectoria')
        plt.show()

        #plt.plot(self.list_linear_time, self.list_linear_setpoint, label="setpoint")
        #plt.plot(self.list_linear_time, self.list_linear_actual_state, label="actual state")
        #plt.plot(self.list_linear_time, self.list_linear_control_effort, label="control effort")
        #plt.legend(loc='upper center')
        #plt.xlabel('t (s)')
        #plt.ylabel('Magnitud')
        #plt.title('reference v/s actual state + control effort')
        #plt.show()

        #plt.plot(self.list_angular_time, self.list_angular_setpoint, label="setpoint")
        #plt.plot(self.list_angular_time, self.list_angular_actual_state, label="actual state")
        #plt.plot(self.list_angular_time, self.list_angular_control_effort, label="control effort")
        #plt.legend(loc='upper center')
        #plt.xlabel('t (s)')
        #plt.ylabel('Magnitud')
        #plt.title('reference v/s actual state + control effort (Angular)')
        #plt.show()

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
