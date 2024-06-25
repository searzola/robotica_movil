#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray, Vector3
from random import uniform
import numpy as np

class PoseChanger(Node):

    def __init__(self):
        super().__init__('pose_changer')
        self.pose_publisher = self.create_publisher(Vector3, "/move_particles", 10)
        self.create_timer( 1.0, self.publish_poses)
    
    def publish_poses(self):
        delta_vector = Vector3()
        delta_vector.x = uniform(-0.2, 0.2)
        delta_vector.y = uniform(-0.2, 0.2)
        delta_vector.z = uniform(-np.pi/2, np.pi/2)
       # self.get_logger().info("Waiting for particle_manager host...")
        while self.pose_publisher.get_subscription_count() == 0:
            if self.pose_publisher.get_subscription_count() > 0:
                break
       # self.get_logger().info("Host reached. Publishing poses")
        self.pose_publisher.publish(delta_vector)

def main(args=None):
    rclpy.init(args=args)
    pose_changer_node = PoseChanger()
    rclpy.spin(pose_changer_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_changer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
