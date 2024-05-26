import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np
from scipy.ndimage import convolve, gaussian_filter, median_filter
import threading


class Obstacle_Detector2(Node):
    def __init__(self):
        super().__init__('obstacle_detector2')
        self.publisher_ref = self.create_publisher(Float64, 'setpoint', 10)
        self.ref = 0.0
        msg = Float64()
        msg.data = float(self.ref)
        self.publisher_ref.publish(msg)
        self.publisher_state = self.create_publisher(Float64, '/state', 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.publisher_velocity = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.control_p_sub = self.create_subscription(Float64, 'control_effort', self.velocida_angular, 10)
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        self.v_angular = 0.0
        self.velo_max = 0.2
        self.start = True
        self.get_logger().info('READING')

    def depth_cb(self, data):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(data)
        self.current_cv_depth_image = np.nan_to_num(self.current_cv_depth_image, nan=0.3)
        L, R = divide_vision(self.current_cv_depth_image)

        coordenadas_numpyR = encontrar_objeto_mas_cercano_optimizado(R)
        coordenadas_numpyL = encontrar_objeto_mas_cercano_optimizado(L)

        valor_minimoR = R[coordenadas_numpyR]
        valor_minimoL = L[coordenadas_numpyL]
        xr = coordenadas_numpyR[1]
        xl = coordenadas_numpyL[1] -320
        thetar = (xr)*(28.5/320)
        xr = valor_minimoR*np.sin(np.deg2rad(thetar))
        thetal = (xl)*(28.5/320)
        xl = valor_minimoL*np.sin(np.deg2rad(thetal))

        if self.ref == 0.3:
            ref = xr
            if ref > 0.3:
                self.ref = 0.0
                msg = Float64()
                msg.data = float(self.ref)
                self.publisher_ref.publish(msg)
                ref = -(xr + xl)
        elif self.ref == -0.3:
            ref = xl
            if ref < -0.3:
                self.ref = 0.0
                msg = Float64()
                msg.data = float(self.ref)
                self.publisher_ref.publish(msg)
                ref = -(xr + xl)
        else:
            ref = -(xr + xl)
            if xl == 0.0 and xr == 0.0:
                self.v_angular = 0.0
            elif xr == 0.0:
                ref = xr
                if self.ref != 0.3:
                    self.ref = 0.3
                    msg = Float64()
                    msg.data = float(self.ref)
                    self.publisher_ref.publish(msg)
            elif xl == 0.0:

                ref = xl
                if self.ref != -0.3:
                    self.ref = -0.3
                    msg = Float64()
                    msg.data = float(self.ref)
                    self.publisher_ref.publish(msg)
            else:
                if self.ref != 0.0 and xr > 0.3 and xl < -0.3 and abs(self.state) > self.ref:
                    self.ref = 0.0
                    msg = Float64()
                    msg.data = float(self.ref)
                    self.publisher_ref.publish(msg)
        self.state = ref
        msg = Float64()
        msg.data = float(ref)
        self.publisher_state.publish(msg)
        self.get_logger().info('Publishing3: xr:"%.2f", xl:"%.2f", ref:"%.2f",state:"%.2f", w:"%.2f"' % (xr, xl,self.ref, ref, self.v_angular))
        if self.start:
            self.start = False
            threat = threading.Thread(target=self.velocidad, daemon=True)
            threat.start()
        

    def velocida_angular(self, dato):
        dato = float(dato.data)
        if dato > 0:
            v_angular = min(0.18, dato)
        else:
            v_angular = max(-0.18, dato)
        # v_angular = dato
        self.v_angular = v_angular

    def velocidad(self):
        while True:
            self.velo = self.velo_max - self.v_angular
            velocidad = Twist()
            velocidad.linear.x = self.velo
            velocidad.angular.z = self.v_angular
            self.publisher_velocity.publish(velocidad)


def suavizar_matriz(matriz, sigma=1):
    return gaussian_filter(matriz, sigma=sigma)


def eliminar_ruido(matriz, umbral):
    matriz[matriz > umbral] = umbral
    return matriz


def filtro_mediana(matriz, tamano_kernel=3):
    return median_filter(matriz, size=tamano_kernel)


def encontrar_concentracion_baja(matriz, tamano_kernel=3):
    kernel = np.ones((tamano_kernel, tamano_kernel))
    matriz_sumas = convolve(matriz, kernel, mode='constant', cval=np.inf)
    indice = np.unravel_index(np.argmin(matriz_sumas), matriz_sumas.shape)
    return indice


# Enfoque con bucles anidados
def encontrar_objeto_mas_cercano_bucles(matriz):
    distancia_minima = np.inf
    coordenadas_mas_cercanas = (0, 0)
    
    for i in range(matriz.shape[0]):
        for j in range(matriz.shape[1]):
            if matriz[i, j] < distancia_minima:
                distancia_minima = matriz[i, j]
                coordenadas_mas_cercanas = (i, j)
    
    return coordenadas_mas_cercanas


# Enfoque con NumPy
def encontrar_objeto_mas_cercano_optimizado(matriz):
    indice = np.unravel_index(np.argmin(matriz), matriz.shape)
    return indice


def divide_vision(current_cv_depth_image):
    n_column = current_cv_depth_image.shape[1]
    left = current_cv_depth_image[:, :n_column // 2]
    right = current_cv_depth_image[:, n_column // 2:]
    return [left, right]


def main(args=None):
    rclpy.init()
    obstacle_detector2 = Obstacle_Detector2()
    rclpy.spin(obstacle_detector2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detector2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
