import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import numpy as np


class Obstacle_Detector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.publisher_ = self.create_publisher(Vector3, '/occupancy_state', 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.bridge = CvBridge()
        self.current_cv_depth_image = None

    def depth_cb(self, data):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(data)
        v3 = obstacles(divide_vision(self.current_cv_depth_image))
        self.obstacles_detected(v3)

    def obstacles_detected(self, obstacles):
        v3 = Vector3()
        v3.x, v3.y, v3.z = obstacles
        self.publisher_.publish(v3)


def obstacles(min_distancias):
    vector_obstacles = []
    for ele in min_distancias:
        if ele <= 0.8 or str(ele) == "nan":
            vector_obstacles.append(1.0)
        else:
            vector_obstacles.append(0.0)
    return vector_obstacles


def divide_vision(current_cv_depth_image):
    n_column = current_cv_depth_image.shape[1]
    left = current_cv_depth_image[:, :n_column // 3]
    center = current_cv_depth_image[:, n_column // 3: 2 * n_column // 3]
    right = current_cv_depth_image[:, 2 * n_column // 3:]
    min_left = np.min(left)
    min_central = np.min(center)
    min_right = np.min(right)
    return [min_left, min_central, min_right]


def main(args=None):
    rclpy.init()
    obstacle_detector = Obstacle_Detector()
    rclpy.spin(obstacle_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
