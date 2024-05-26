#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np


class Wall_Distance(Node):
    def __init__(self):
        super().__init__('wall_distance')
        self.distance_publisher = self.create_publisher(Float64, "wall_state", 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.bridge = CvBridge()
        self.current_cv_depth_image = None

    def depth_cb(self, data):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(data)
        msg = (divide_vision(self.current_cv_depth_image))
        self.distance_publisher.publish(msg)



def divide_vision(current_cv_depth_image):
    n_column = current_cv_depth_image.shape[1]
    left = current_cv_depth_image[:, :n_column // 2]
    right = current_cv_depth_image[:, n_column // 2:]
    mean_left_distance = np.mean(left) # Probando si sirve entregando el promedio
    mean_right_distance = np.mean(right)
    output = mean_left_distance - mean_right_distance
    return output

def main(args=None):
    rclpy.init()
    obstacle_detector = Wall_Distance()
    rclpy.spin(obstacle_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()