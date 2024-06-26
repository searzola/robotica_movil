import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float64, Bool
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from random import gauss, uniform
# imagen = 'mapa.pgm'
# try:
#     from likelihood_map import Likelihood_Map
# except Exception as e:
#     import importlib.util
#     import sys

#     ruta = 'ros2_ws/src/my_first_packege/my_first_packege/'
#     archivo = ruta+'likelihood_map.py'
#     nombre_modulo = 'likelihood_map'

#     spec = importlib.util.spec_from_file_location(nombre_modulo, archivo)
#     mi_modulo = importlib.util.module_from_spec(spec)
#     spec.loader.exec_module(mi_modulo)
#     sys.modules[nombre_modulo] = mi_modulo
#     from likelihood_map import Likelihood_Map
#     imagen = ruta+imagen


class Particle( object ):
    particle_id = 0

    def __init__( self, x, y, ang, sigma = 0.1, weight=1.0 ):
        self.x, self.y, self.ang = x, y, ang
        self.last_x, self.last_y, self.last_ang = x, y, ang
        self.sigma = sigma
        self.id = Particle.particle_id
        Particle.particle_id += 1
        self.weight = weight

    def move( self, delta_x, delta_y, delta_ang ):
        self.x += delta_x + gauss (0, self.sigma )
        self.y += delta_y + gauss( 0, self.sigma )
        ang = self.ang + delta_ang + gauss( 0, self.sigma )
        self.ang = ((ang + np.pi) % (2 * np.pi)) - np.pi 
        # print(self)

    def pos( self ):
        return [self.x, self.y, self.ang]

    def last_pos( self ):
        return [self.last_x, self.last_y, self.last_ang]

    def __str__(self) -> str:
        return f"id: {str(self.id)}: [x: {str(self.x)}, y: {str(self.y)}, ang: {str(self.ang)}], [weight: {str(self.weight)}]. "


# cambiar nombre a Localizacion o filtro de particulas
class ParticlesManager( Node ):

    def __init__( self, num_particles, range_x, range_y):
        super().__init__( 'particles_manager' )
        self.declare_parameter( 'ruta', 'ros2_ws/src/my_first_packege/my_first_packege/')
        self.ruta = self.get_parameter( 'ruta' ).value
        #
        import importlib.util
        import sys

        # ruta = 'ros2_ws/src/my_first_packege/my_first_packege/'
        archivo = self.ruta+'likelihood_map_C.py'
        nombre_modulo = 'likelihood_map'
        imagen = 'mapa.pgm'

        spec = importlib.util.spec_from_file_location(nombre_modulo, archivo)
        mi_modulo = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mi_modulo)
        sys.modules[nombre_modulo] = mi_modulo
        from likelihood_map import Likelihood_Map
        imagen = self.ruta+imagen
        #
        self.likelihood_map = Likelihood_Map(imagen)
        self.get_logger().info( 'CALCULANDO MAPA ...')
        self.likelihood_map.calcular_mapa()
        self.get_logger().info( 'MAPA CALCULADO')
        self.num_particles = num_particles
        self.range_x = range_x
        self.range_y = range_y
        self.sigma = 0.01
        self.particles = []
        self.pub_particles = self.create_publisher(PoseArray, 'particles', 10 )
        self.real_pose_sub = self.create_subscription(Pose, '/real_pose', self.read_real_pose, 10)
        self.pose_real = []
        self.pose_real_actualizada = False
        self.dic_particles = {}
        self.depth_sub = self.create_subscription(LaserScan, '/scan', self.depth_cb2, 10)
        self.particulas_movidas = True
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.read_odom_pose, 10)
        self.last_pose = []
        self.new_pose = []
        self.read_first_pose = False
        self.interaction = True
        self.mcl_pub = self.create_publisher(Float64, 'MCL', 10)
        self.react_sub = self.create_subscription(Bool, "reactive", self.señal_run, 10)
        self.time = 1.0
        self.max_weight = 0

    def create_particles( self, range_x, range_y, numero, add=True ):
        for i in range( 0, numero ):
            x = uniform( range_x[0], range_x[1] )
            y = uniform( range_y[0], range_y[1] )
            ang = uniform( -np.pi, np.pi )
            weight = 1.0
            new_particle = Particle( x, y, ang, sigma = self.sigma, weight=weight)
            self.dic_particles[str(new_particle.id)] = new_particle
            if add:
                self.particles.append(str(new_particle.id))
        self.publish_particles()

    def update_particles( self, move_vector):
        delta_x = move_vector[0] - self.last_pose[0]
        delta_y = move_vector[1] - self.last_pose[1]
        delta_ang = move_vector[2] - self.last_pose[2]
        particle_to_move = list(set(self.particles))
        for particle in particle_to_move:
            delta_xr, delta_yr, delta_angr = mover_particula(self.dic_particles[particle].pos(), [delta_x, delta_y, delta_ang], move_vector)
            self.dic_particles[particle].move(delta_xr, delta_yr, delta_angr)
        self.particulas_movidas = True
        self.publish_particles()

    def publish_particles(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header = Header()
        pose_array_msg.header.frame_id = "map"

        for part in self.particles:
            part = self.dic_particles[part]
            part_pose = Pose()
            part_pose.position.x, part_pose.position.y = part.x, part.y
            quat = quaternion_from_euler(0,0, part.ang)

            part_pose.orientation.x = quat[0]
            part_pose.orientation.y = quat[1]
            part_pose.orientation.z = quat[2]
            part_pose.orientation.w = quat[3]

            pose_array_msg.poses.append(part_pose)

        self.pub_particles.publish(pose_array_msg)

    def depth_cb2(self, data):
        ranges = np.array(data.ranges)[62:-62]
        if self.particulas_movidas and self.particles != [] and self.read_first_pose and self.pose_real_actualizada:
            self.likelihood_map.img_copy = self.likelihood_map.img.copy()
            self.pose_real_actualizada = False
            self.particulas_movidas = False
            particulas_unique = list(set(self.particles))
            for particle in particulas_unique:
                particle = self.dic_particles[particle]
                pose_array = []
                for n in range(len(ranges)):
                    test_z = ranges[n]
                    z_theta = np.deg2rad(n - 28.5)
                    x, y, theta = particle.x, particle.y, particle.ang
                    pose = [x, y, theta, test_z, z_theta]
                    pose_array.append(pose)
                particle.weight = self.likelihood_map.pose_probabiliti(pose_array)
                # self.likelihood_map.show_particles(pose)
            self.normalizar_particulas()
            self.new_sample()
            self.last_pose = self.new_pose
            msg = Float64()
            msg.data = self.time
            self.mcl_pub.publish(msg)

    def normalizar_particulas(self):
        values = list(self.dic_particles.values())
        valu = list(filter(lambda x: x.weight != 0.0, values))
        weights = np.array(list(map(lambda x: x.weight, valu)))
        weights_array = weights/np.sum(weights)
        self.max_weight = np.max(weights_array)
        weights = softmax(weights_array).tolist()
        for ele in valu:
            ele.weight = weights.pop(0)

    def new_sample(self):
        values = np.array(list(self.dic_particles.values()))
        weights = np.array(list(map(lambda x: x.weight, values)))
        keys = np.array(list(self.dic_particles.keys()))
        sample = np.random.choice(keys, p=weights, size=self.num_particles, replace=True) # Puedo escoger más de una vez una partìcula
        # Tengo que ver una forma de poder copiar partículas con un buen peso

        # self.sample_list = sample.tolist()
        # self.particles = []
        # print(len(set(self.sample_list)))


        # # Crear copias de cada partícula y agregarlas a una nueva lista
        # for i in list(self.sample_list):#range(len(list(set(self.sample_list)))):
        #     particula_actual = self.dic_particles[i]#self.sample_list[i]
        #     if i not in self.particles:
        #         self.particles.append(i)
        #         continue
        #     # print("actual ",particula_actual)
        #     new_particle = Particle(x=particula_actual.x, y=particula_actual.y, ang=particula_actual.ang)
        #     # print("new ",new_particle)
        #     self.dic_particles[str(new_particle.id)] = new_particle
        #     self.particles.append(str(new_particle.id))
        #sample = np.random.choice(keys, p=weights, size=self.num_particles)
        self.particles = sample.tolist()
        var_x, var_y = self.calcular_incertidumbre(list(set(self.particles)))
        cx, cy, ctheta = self.calcular_pose_convergencia(list(set(self.particles)))
        rx, ry, rtheta = self.pose_real
        print(self.pose_real)
        print("dx: ", rx - cx)
        print("dy: ", ry - cy)
        print("dtheta: ", rtheta - ctheta)
        print("CANTIDAD DE PARTICULAS VIVAS ", len(list(set(self.particles))), " de ", len(self.particles))
        if len(list(set(self.particles))) < int(self.num_particles*0.1):
            print("POCAS PARTICULAS ", len(list(set(self.particles))), "INYECTAR PARTICULAS")
            # self.sample_list = sample.tolist()
            # self.particles = []
            # print(len(set(self.sample_list)))


            # # Crear copias de cada partícula y agregarlas a una nueva lista
            # for i in list(self.sample_list):#range(len(list(set(self.sample_list)))):
            #     particula_actual = self.dic_particles[i]#self.sample_list[i]
            #     if i not in self.particles:
            #         self.particles.append(i)
            #         continue
            #     # print("actual ",particula_actual)
            #     new_particle = Particle(x=particula_actual.x, y=particula_actual.y, ang=particula_actual.ang)
            #     # print("new ",new_particle)
            #     self.dic_particles[str(new_particle.id)] = new_particle
            #     self.particles.append(str(new_particle.id))
            #     # if var_x < 0.01 and var_y < 0.01:
            rango_x = [max(cx-0.5,0), min(cx+0.5, 2.7)]
            rango_y = [max(cy-0.5,0), min(cy+0.5, 2.7)]
            self.create_particles(rango_x, rango_y, int(self.num_particles*0.3), add=True)
            # else:
            self.create_particles(self.range_x, self.range_y, int(self.num_particles*0.7), add=True)

    def read_odom_pose(self, odom_pose):
        x = odom_pose.pose.pose.position.x
        y = odom_pose.pose.pose.position.y
        z = odom_pose.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( odom_pose.pose.pose.orientation.x,
                                                    odom_pose.pose.pose.orientation.y,
                                                    odom_pose.pose.pose.orientation.z,
                                                    odom_pose.pose.pose.orientation.w ) )
        if self.new_pose == []:
            self.new_pose = [x, y, yaw]
            self.read_first_pose = True
        if not self.interaction:
            self.new_pose = [x, y, yaw]
            self.interaction = True
            self.update_particles(self.new_pose)

    def señal_run(self, data):
        self.interaction = False
        pass

    def calcular_incertidumbre(self, list_particulas):
        part = list(map(lambda x: self.dic_particles[x].pos(), list_particulas))
        particulas = np.array(part)
        # Calcular la media de las partículas
        media_x = np.mean(particulas[:, 0])
        media_y = np.mean(particulas[:, 1])
        media_theta = np.mean(particulas[:, 2])

        # Calcular la varianza de las partículas
        var_x = np.var(particulas[:, 0])
        var_y = np.var(particulas[:, 1])
        var_theta = np.var(particulas[:, 2])

        print('media_x: ', media_x)
        print('media_y: ', media_y)
        print('media_theta: ', media_theta)
        print('var_x: ', var_x)
        print('var_y: ', var_y)
        print('var_theta: ', var_theta)

        incertidumbre = {
            'media_x': media_x,
            'media_y': media_y,
            'media_theta': media_theta,
            'var_x': var_x,
            'var_y': var_y,
            'var_theta': var_theta
        }
        # return incertidumbre
        return [var_x, var_y]

    def calcular_pose_convergencia(self, list_particulas):
        part = list(map(lambda x: self.dic_particles[x].pos(), list_particulas))
        particulas = np.array(part)
        weights = list(map(lambda x: self.dic_particles[x].weight, list_particulas))
        probabilidades = np.array(weights)

        # Calcular la media ponderada
        media_x = (np.sum(particulas[:, 0] * probabilidades)) / np.sum(probabilidades)
        media_y = (np.sum(particulas[:, 1] * probabilidades)) / np.sum(probabilidades)
        media_theta = (np.sum(particulas[:, 2] * probabilidades)) / np.sum(probabilidades)

        print('x: ', media_x)
        print('y: ', media_y)
        print('theta: ', media_theta)

        pose_convergencia = {
            'x': media_x,
            'y': media_y,
            'theta': media_theta
        }
        # return pose_convergencia
        return [media_x, media_y, media_theta]
    
    def read_real_pose(self, real_pose):
        x = real_pose.position.x
        y = real_pose.position.y
        z = real_pose.position.z
        roll, pitch, yaw = euler_from_quaternion((real_pose.orientation.x,
                                                    real_pose.orientation.y,
                                                    real_pose.orientation.z,
                                                    real_pose.orientation.w))
        self.pose_real = [x, y, yaw]
        self.pose_real_actualizada = True


def softmax(x, temperature=1.0, axis=None):
    # Ajustar los logits con la temperatura
    x_adjusted = x / temperature
    # Subtract the max value along the specified axis for numerical stability
    shifted_logits = x_adjusted - np.max(x_adjusted, axis=axis, keepdims=True)
    exps = np.exp(shifted_logits)
    sum_exps = np.sum(exps, axis=axis, keepdims=True)
    return exps / sum_exps


def mover_particula(pose_inicial, odometria, real_odo):
    x, y, theta = pose_inicial
    dx, dy, dtheta = odometria
    odx, ody, odtheta = real_odo
    theta_continuo = (odtheta) % (2*np.pi)
    theta_r = theta - theta_continuo
    c = np.cos(theta_r)
    s = np.sin(theta_r)
    vector_rotacion = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    odome = np.array([[dx], [dy], [dtheta]])
    dot = np.dot(vector_rotacion, odome)
    return dot.T.tolist().pop(0)


def main():
    rclpy.init()

    map_width_pix = 270 # [pix]
    map_height_pix = 270 # [pix]
    map_resolution = 0.01 # [m/pix]

    map_width_m = map_width_pix * map_resolution
    map_height_m = map_height_pix * map_resolution
    num_particles = 1000
    particle_manager = ParticlesManager( num_particles, [0, map_width_m] , [0, map_height_m])
    particle_manager.create_particles( [0, map_width_m], [0, map_height_m] , num_particles)

    rclpy.spin( particle_manager )
    particle_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
