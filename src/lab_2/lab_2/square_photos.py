#!/usr/bin/env python3

import cv2
import numpy as np
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge

import threading


class Blue_Watcher(Node):
    def __init__(self):
        super().__init__('square_photos')
        self.publisher_ = self.create_publisher(Image, 'blue_square_photos', 10)
        self.bridge = CvBridge()
        self.path_1 = os.path.join("blue_square", "frame000220.png")
        self.path_2 = os.path.join("blue_square", "frame000310.png")
        self.path_3 = os.path.join("blue_square", "frame000470.png")
        self.path_4 = os.path.join("blue_square", "frame000550.png")
        self.path_5 = os.path.join("blue_square", "frame000640.png")

        self.send_pic(self.path_1) # Ir cambiando el path para probar cada posición


    def send_pic(self, path):
        # self.get_logger().info('Publishing: "%s"' % data)
        cv_image = cv2.imread(path)
        self.current_img = self.bridge.cv2_to_imgmsg(cv_image)
        self.publisher_.publish(self.current_img)
        


def main(args=None):
    rclpy.init()
    blue_watcher = Blue_Watcher()
    rclpy.spin(blue_watcher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    blue_watcher.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()