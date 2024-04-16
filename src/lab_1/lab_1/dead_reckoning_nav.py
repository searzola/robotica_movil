#!/usr/bin/env python3
import rclpy
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
            self.get_logger().info("publicando velocidad (%f, %f)" % (lin_speed, ang_speed))
            self.timer = self.create_timer(speed_comand[2], self.vel_publisher.publish(velocidad))

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

        if pose_actual[0] < goal_pose[0]:
            t = (goal_pose[0] - pose_actual[0])/self.linear_vel
            comando = (self.linear_vel, 0, t)
            lista_comandos_vel.append(comando)
        else if pose_actual[0] > goal_pose[0]:
            comando = (0, self.angular_vel, 1)
            lista_comandos_vel.append(comando)
            t = (pose_actual[0] - goal_pose[0])/self.linear_vel
            comando = (self.linear_vel, 0, t)

       """ if pose_actual[1] < goal_pose[1]:
            t = (goal_pose[0] - pose_actual[0])/self.linear_vel
            comando = (self.linear_vel, 0, t)
            lista_comandos_vel.append(comando)
        else if pose_actual[0] > goal_pose[0]:
            comando = (0, self.angular_vel, 1)
            lista_comandos_vel.append(comando)
            t = (pose_actual[0] - goal_pose[0])/self.linear_vel
            comando = (self.linear_vel, 0, t) """
            



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
