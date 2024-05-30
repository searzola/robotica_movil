#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge

import threading


class Blue_Watcher(Node):
    def __init__(self):
        super().__init__('square_watcher')
        self.publisher_ = self.create_publisher(Float64, '/blue_square_position', 10)
        #self.rgb_sub = self.create_subscription(Image, '/camera/rgb/image_color', self.detector_de_objeto, 10)
        self.second_rgb_sub = self.create_subscription(Image, 'blue_square_photos', self.detector_de_objeto, 10)
        
        self.bridge = CvBridge()
        self.current_cv_rgb_image = None                             # 100
        self.lower_color_blue = np.array([100, 120, 200])     # 100, 120, 200
        self.upper_color_blue = np.array([115, 190, 250])    # 115, 190, 250
        self.ver_imagen = True
        self.current_cv_depth_image = None

        # frame = cv2.imread("frame000100.png")
        # self.watcher(frame)
        #frame000100.png

    def depth_cb(self, data):
        # self.get_logger().info('Publishing: "%s"' % data)
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(data)
        self.frame = self.current_cv_depth_image
        if self.ver_imagen:
            self.ver_imagen = False
            threat = threading.Thread(target=self.ver, daemon=True)
            threat.start()

    def detector_de_objeto(self, data):
        self.get_logger().info('Foto recibida ')
        self.current_cv_rgb_image = self.bridge.imgmsg_to_cv2(data)
        self.watcher(self.current_cv_rgb_image)

    def watcher(self, frame):
        self.frame = frame
        
        frame_front = frame
        frame_low_front = cv2.GaussianBlur(frame_front, (5, 5), 0)
        hsv_frame_front = cv2.cvtColor(frame_low_front, cv2.COLOR_BGR2HSV)
        mask_color_front = cv2.inRange(hsv_frame_front, self.lower_color_blue, self.upper_color_blue)

        edges_front = cv2.Canny(mask_color_front, 250, 255)
        contours_front, _ = cv2.findContours(edges_front, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_front:
            if cv2.contourArea(contour) > 5:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX_front = int(M["m10"] / M["m00"])
                    cY_front = int(M["m01"] / M["m00"])
                else:
                    cX_front, cY_front = 0, 0

                cv2.circle(frame_front, (cX_front, cY_front), 1, (0, 255, 0), -1)
                cv2.drawContours(frame_front, [contour], -1, (0, 255, 0), 2)
        
        pose = [cX_front - 320, - cY_front + 240] # Relativo al centro de la imagen
        self.get_logger().info('Enviando centro del cuadrado: "%.2f"' % pose[0])
        msg = Float64()
        msg.data = float(pose[0])
        self.publisher_.publish(msg)


        hsv_frame_mask_front = cv2.bitwise_and(frame_low_front, frame_low_front, mask=mask_color_front)
        enemy_txt = cv2.putText(img = self.frame, text = str(pose[0])+","+ str(pose[1]) + "," , org = (pose[0]+320, -(pose[1]-240)), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.3, color = (0, 0, 0), thickness = 1)


        # cv2.imshow('raw RGB', frame)
        # cv2.waitKey()
        if self.ver_imagen:
            self.ver_imagen = False
            threat = threading.Thread(target=self.ver, daemon=True)
            threat.start()

    def ver(self):
        while True:
            cv2.imshow('raw RGB', self.frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            # cv2.waitKey()
            # if cv2.waitKey(1) & 0xFF == 27:
            #     break



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