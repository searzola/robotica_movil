import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy as np
imagen = 'mapa.pgm'
try:
    from likelihood_map import Likelihood_Map
except Exception as e:
    # Cuando se usa launcho o se hace run con ros2
    import importlib.util
    import sys
            # cambiar ruta al archivo
    ruta = 'ros2_ws/src/my_first_packege/my_first_packege/'
    archivo = ruta+'likelihood_map.py'
    nombre_modulo = 'likelihood_map'

    spec = importlib.util.spec_from_file_location(nombre_modulo, archivo)
    mi_modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mi_modulo)
    sys.modules[nombre_modulo] = mi_modulo
    from likelihood_map import Likelihood_Map
    imagen = ruta+imagen    # Si la foto esta en otra parte cambiar la ruta

class Likelihood_test(Node):
    def __init__(self):
        super().__init__('likelihood_test')
        self.likelihood_map = Likelihood_Map(imagen)
        self.likelihood_map.calcular_mapa()
        # self.sub_likelihood_map = self.create_subscription(Float64, 'pose_proba', self.sensor_model, 10)
        # self.publisher_poses = self.create_publisher(PoseArray, 'likelihood_model', 10)

        self.sub_action_run = self.create_subscription(Float64, 'action_run', self.run, 10)

        self.depth_sub = self.create_subscription(LaserScan, '/scan', self.depth_cb2, 10)
        self.real_pose_sub = self.create_subscription(Pose, '/real_pose', self.read_real_pose, 10)
        self.real_pose = []
        self.real_pose_d = []
        self.visto = True
        self.poses = []
        self.n = 100
        self.contador = 0
        self.resultados = {}
        self.datos_recolectados_b = []
        self.datos_recolectados_m = []
        self.fin = False
        self.pose_lidar = {}
        self.get_logger().info( 'MAPA CALCULADO')

    def sensor_model(self, data):
        proba = data
        self.get_logger().info( '%s:proba: %s' % (self.contador, proba))
        valor = self.resultados[str(self.contador)]
        valor[1] = proba
        self.resultados[str(self.contador)] = valor
        self.visto = True
        self.contador += 1
        # if self.poses == []:
        if self.contador == len(self.poses):
            self.poses = []
            mayor = max(self.resultados, key=lambda k: self.resultados[k][1])
            print(self.resultados[mayor])
            print(self.resultados[str(self.contador-1)])
            if self.resultados[mayor][1] != self.resultados[str(self.contador-1)][1]:
                self.likelihood_map.look_especial(self.pose_lidar[str(self.resultados[str(self.contador-1)][0])], self.pose_lidar[str(self.resultados[mayor][0])])

            # print("FIN")
            # input("ENTER: ")
            self.real_pose = []
            self.contador = 0
            self.resultados = {}

    def depth_cb(self, data):
        ranges = np.array(data.ranges)[62:-62] #[::-1]
        if self.real_pose != [] and self.visto and self.poses != []:
            pose_real = []
            for pose_test in self.poses:
                pose_array = []
                for n in range(len(ranges)):
                    test_z = ranges[n]
                    z_theta = np.deg2rad(n - 28.5)
                    x, y, theta = pose_test
                    pose = [x, y, theta, test_z, z_theta]
                    pose_array.append(pose)
                self.pose_lidar[str(pose_test)] = pose_array
                self.sensor_model(self.likelihood_map.pose_probabiliti(pose_array))

    def depth_cb2(self, data):
        ranges = np.array(data.ranges)[62:-62] #[::-1]
        if self.real_pose != [] and self.visto and self.poses != []:
            poses_array = []
            for pose_test in self.poses:
                pose_array = []
                for n in range(len(ranges)):
                    test_z = ranges[n]
                    z_theta = np.deg2rad(n - 28.5)

                    x, y, theta = pose_test
                    pose = [x, y, theta, test_z, z_theta]
                    pose_array.append(pose)
                poses_array.append(pose_array)
            arrays_probabilida = self.likelihood_map.poses_arrays_probabiliti(poses_array)
            # con "arrays_probabilida" hacer un mensaje hacia otro nodo

    def read_real_pose(self, real_pose):
        x = real_pose.position.x
        y = real_pose.position.y
        z = real_pose.position.z
        roll, pitch, yaw = euler_from_quaternion((real_pose.orientation.x,
                                                    real_pose.orientation.y,
                                                    real_pose.orientation.z,
                                                    real_pose.orientation.w))
        if self.real_pose == []:
            self.list_poses([x, y, yaw])
            self.real_pose_d = [x, y, yaw]
        self.real_pose = [x, y, yaw]

    def list_poses(self, real_pose):
        x = np.random.uniform(0, 2.7, self.n)
        y = np.random.uniform(0, 2.7, self.n)
        theta = np.random.uniform(0, 2*np.pi, self.n)

        poses = np.column_stack((x, y, theta))
        self.poses = poses.tolist()
        self.poses.append(real_pose)
        for i in range(len(self.poses)):
            self.resultados[str(i)] = [self.poses[i], 0]
            self.pose_lidar[str(self.poses[i])] = [0]

    def run(self, data):
        pass



def main(args=None):
    rclpy.init()
    likelihood_test = Likelihood_test()
    rclpy.spin(likelihood_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    likelihood_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()