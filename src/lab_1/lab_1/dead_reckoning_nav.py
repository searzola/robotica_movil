#!/usr/bin/env python3
import rclpy
import Time
from rclpy.node import Node
from std_msgs.msg import String

class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.odom_sub = self.create_subscription( Odometry, '/odom', self.mover_robot_a_destino, 10)
        self.linear_vel = 0.2 #m/s
        self.angular_vel = 1 #rad/s
    
    def aplicar_velocidad(self, speed_command_list):
        for speed_comand in speed_command_list:
            velocidad = Twist()
            velocidad.linear.x = speed_comand[0]
            velocidad.angular.z = speed_comand[1]
            timer = Time.time()
            t = Time.time() + speed_comand[2]
            while timer < t:
                self.get_logger().info("publicando velocidad (%f, %f)" % (lin_speed, ang_speed))
                self.vel_publisher.publish(velocidad)

    def mover_robot_a_destino(self, goal_pose, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion((odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w))

        pose_actual = (x,y,yaw)
        lista_comandos_vel = []

        distancia_x = goal_pose[0] - pose_actual[0]
        distancia_y = goal_pose[1] - pose_actual[1]
        distancia_theta = goal_pose[2] - pose_actual[2]

        if distancia_x != 0:
            t = distancia_x/self.linear_vel
            comando = (self.linear_vel, 0, t)
            comando.append(comando)
        if distancia_y != 0:
            t = distancia_y/self.linear_vel
            comando = (self.linear_vel, 0, t)
            comando.append(comando)
        if distancia_theta != 0:
            t = distancia_theta/self.angular_vel
            comando = (0, self.angular_vel, t)
            comando.append(comando)
            
        self.aplicar_velocidad(lista_comandos_vel)
            
    def accion_mover_cb(self, goal_list):
        for pose in goal_list:
            self.mover_robot_a_destino(pose)


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
