#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray

class PoseLoader(Node):

    def __init__(self):
        super().__init__('pose_loader')
        self.pose_publisher = self.create_publisher(PoseArray, "goal_list", 10)
        self.publish_poses()
    
    def publish_poses(self):
        poses_text = open ("robotica_movil/src/lab_2/lab_2/poses.txt", "r") #formato de .txt 0.0,0.0,0.0\n
        poses_string = poses_text.read()
        poses_list = poses_string.split("\n")
        for pose in poses_list:
            self.get_logger().info("Pose adquired: (%s)" % pose)
        pose_array = PoseArray()
        for poses in poses_list:
            coord_array = poses.split(",")
            q = quaternion_from_euler( 0.0, 0.0, float(coord_array[2]))
            pose = Pose( position = Point( x = float(coord_array[0]), y = float(coord_array[1]), z = 0.0 ),
                         orientation = Quaternion( x = q[0], y = q[1], z = q[2], w = q[3] ) )
            pose_array.poses.append(pose)

        self.get_logger().info("Waiting for dead_reckon_node host...")
        while self.pose_publisher.get_subscription_count() == 0:
            if self.pose_publisher.get_subscription_count() > 0:
                break
        self.get_logger().info("Host reached. Publishing poses")
        self.pose_publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    pose_loader_node = PoseLoader()
    rclpy.spin(pose_loader_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_loader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
