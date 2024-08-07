import numpy as np
from scipy.integrate import quad
from scipy.optimize import minimize
import cv2
import threading
import itertools
import time



class Likelihood_Map():
    def __init__(self, file_path):
        img = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
        self.img = img
        self.resolucion = 0.01
        self.len_img = len(img)
        self.sensor = 0.0
        self.z_max = 4.0/self.resolucion
<<<<<<< HEAD
        self.zhit = 1.09
        self.sigma2 = 1.1
        self.zrand = 0.09
        self.zmaz = 1e-07
=======
        self.zhit = 5.327786164740783
        self.sigma2 = 1.479256698286517
        self.zrand = 1e-06
        self.zmaz = 1e-06
>>>>>>> f0a220e14e8366261e894bfa1ecbd07a75521e1d
        self.img_copy = img.copy()

        self.mapa_T = np.zeros_like(img, dtype=np.float64)

        # Filtrar puntos que no son cero en la imagen
        mask = (img == 0)

        # Coordenadas de puntos fijados donde img == 0
        self.puntos_fijos = np.column_stack(np.where(mask))

    def calcular_mapa(self):
        self.area, _ = quad(proba, -self.z_max, self.z_max, args=(self.sigma2, self.z_max, self.zhit, self.zrand, self.zmaz))
        # Crear arrays de coordenadas
        Y, X = np.indices(self.img.shape)

        print("Calculado Mapa")
        # # Calcular el mapa de probabilidades
        contador = 0
        for yf, xf in self.puntos_fijos:
            #  # Distancia de todos los puntos al punto fijo
            distancias = np.sqrt((Y - yf)**2 + (X - xf)**2)

            # # Calcular la probabilidad gaussiana
            prob_gaussian = gaussian_2Vd(distancias, self.sigma2, self.z_max, self.zhit)

            prob_rand = p_randV(distancias, self.z_max, self.zrand)

            prob_z_max = p_z_maxV(distancias, self.z_max, self.zmaz)

            # # Sumar las probabilidades al mapa
            proba_total = (prob_gaussian + prob_rand + prob_z_max)/self.area
            self.mapa_T += proba_total
            contador += 1
            #print(f'\r{contador/len(self.puntos_fijos):.2f}%', end="")



        self.t = normalizar_advanced(self.mapa_T)
        self.min_proba = np.min(self.t)*1e-5

        # print('\nMapa listo')
        self.ver_imagen = False
        threat = threading.Thread(target=self.ver, daemon=True)
        #threat.start()

    def look_especial(self, array_b, array_m):
        self.img_copy = self.img.copy()
        qb = 1
        for pose in array_b:
            probabilidad = self.look_map_especial(pose)
            qb *= probabilidad[0]
            print(probabilidad, qb)
        print("MALO")
        qm = 1
        for pose in array_m:
            probabilidad = self.look_map_especial(pose)
            qm *= probabilidad[0]
            print(probabilidad, qm)


    def look_map_especial(self, data):
        x, y, theta, z, z_theta = data
        coordenadas = self.calcular_pose_map(x, y, theta, z, z_theta)
        probabilidad = self.probablidad_mapa(coordenadas)
        x, y = coordenadas
        xf, yf = find_nearest_coordinate(self.puntos_fijos, x, y)
        dis = distancia((x, y), (xf, yf))
        #
        x, y, theta, z, z_theta = data
        x = int(round(x/self.resolucion, 0))
        y = self.len_img - int(round(y/self.resolucion, 0))
        if 0 <= x < 270 and 0 <= y < 270:
            origen = (y, x)
            self.img_copy[origen] = 0
            self.img_copy[(min(origen[0]+1, 269), origen[1])] = 0
            self.img_copy[(min(origen[0]+2, 269), origen[1])] = 0
            self.img_copy[(max(origen[0]-1, 0), origen[1])] = 0
            self.img_copy[(max(origen[0]-2, 0), origen[1])] = 0
            self.img_copy[(origen[0], min(origen[1]+1, 269))] = 0
            self.img_copy[(origen[0], min(origen[1]+2, 269))] = 0
            self.img_copy[(origen[0], max(origen[1]-1, 0))] = 0
            self.img_copy[(origen[0], max(origen[1]-2, 0))] = 0
        #####################################
        x, y = coordenadas
        if 0 <= x < 270 and 0 <= y < 270:
            self.img_copy[coordenadas] = 0
            self.img_copy[(min(coordenadas[0]+1, 269), coordenadas[1])] = 0
            self.img_copy[(min(coordenadas[0]+2,269), coordenadas[1])] = 0
            self.img_copy[(max(coordenadas[0]-1, 0), coordenadas[1])] = 0
            self.img_copy[(max(coordenadas[0]-2, 0), coordenadas[1])] = 0
            self.img_copy[(coordenadas[0], min(coordenadas[1]+1, 269))] = 0
            self.img_copy[(coordenadas[0], min(coordenadas[1]+2, 269))] = 0
            self.img_copy[(coordenadas[0], max(coordenadas[1]-1, 0))] = 0
            self.img_copy[(coordenadas[0], max(coordenadas[1]-2, 0))] = 0
        return [probabilidad, dis]
    
    def show_particles(self, data):
        x, y, theta, z, z_theta = data
        x = int(round(x/self.resolucion, 0))
        y = self.len_img - int(round(y/self.resolucion, 0))
        if 0 <= x < 270 and 0 <= y < 270:
            origen = (y, x)
            self.img_copy[origen] = 0
            self.img_copy[(min(origen[0]+1, 269), origen[1])] = 0
            self.img_copy[(min(origen[0]+2, 269), origen[1])] = 0
            self.img_copy[(max(origen[0]-1, 0), origen[1])] = 0
            self.img_copy[(max(origen[0]-2, 0), origen[1])] = 0
            self.img_copy[(origen[0], min(origen[1]+1, 269))] = 0
            self.img_copy[(origen[0], min(origen[1]+2, 269))] = 0
            self.img_copy[(origen[0], max(origen[1]-1, 0))] = 0
            self.img_copy[(origen[0], max(origen[1]-2, 0))] = 0


    def pose_probabiliti(self, array):
        q = 1
        for pose in array:
            probabilidad = self.look_map(pose)
            q *= probabilidad
        return q
    
    def pose_probabiliti2(self, array):
        q = 0.0
        for pose in array:
            probabilidad = self.look_map(pose)
            q += np.log(probabilidad)
        return np.exp(q)

    def look_map(self, data):
        x, y, theta, z, z_theta = data
        coordenadas, pose = self.calcular_pose_map(x, y, theta, z, z_theta, x_y=True)
        probabilidad = self.probablidad_mapa(coordenadas, pose=pose, desconcido=True)
        return probabilidad


    def params_stimator(self, array_b, array_m):
        # datos_buenos = np.array(open_csv(self.datos_buenos))
        # datos_malos = np.array(open_csv("datos_intermedios.csv"))
        # datos = np.concatenate((datos_buenos, datos_malos))
        datos = [] # funciona con lo de arriba si le entregas un array de valores
        print("CALCULANDO ...")
        zhit, sigma2, zrand, zmaz = self.Negativa_Log_Verosimilitud(datos)
        print("zhit ", zhit)
        print("sigma2 ", sigma2)
        print("zrand ", zrand)
        print("zmaz ", zmaz)
        # safe_csv(self.datos_buenos, datos_buenos.tolist())
        # safe_csv(self.datos_malos, datos_malos.tolist())
        #####
        self.zhit = zhit
        self.sigma2 = sigma2
        self.zrand = zrand
        self.zmaz = zmaz
        return

    def get_distance(self, data):
        x, y, theta, z, z_theta = data
        coordenadas = self.calcular_pose_map(x, y, theta, z, z_theta)
        x, y = coordenadas
        xf, yf = find_nearest_coordinate(self.puntos_fijos, x, y)
        dis = distancia((x, y), (xf, yf))
        return dis

    def calcular_pose_map(self, x, y, theta, z, theta_z, x_y=False):
        cos_theta = np.cos(theta)
        sen_theta = np.sin(theta)
        cos_p_z_theta = np.cos(theta + theta_z)
        sen_p_z_theta = np.sin(theta + theta_z)
        pose_x = x
        pose_y = y
        pose = np.array([[pose_x], [pose_y]])
        pose_senor = np.array([[-self.sensor], [0.0]])
        vector_rotacion = np.array([[cos_theta, -sen_theta], [sen_theta, cos_theta]])
        z_position = np.array([[cos_p_z_theta], [sen_p_z_theta]])
        dot = np.dot(vector_rotacion, pose_senor)
        coordenadas = pose + dot + z*z_position
        coordenadas /= self.resolucion
        coordenadas = coordenadas.round(0).astype(np.uint16)
        coordenadas = tuple(coordenadas.T.tolist().pop())
        coordenadas = (self.len_img - coordenadas[1], coordenadas[0])
        if x_y:
            pose_x = int(round(pose_x/self.resolucion, 0))
            pose_y = self.len_img - int(round(pose_y/self.resolucion, 0))
            return [coordenadas, (pose_y, pose_x)]
        return coordenadas

    def probablidad_mapa(self, coordenadas, pose=(0, 0), desconcido=False):
        if desconcido and (0 <= pose[0] < 270 and 0 <= pose[1] < 270) and self.img[pose] != 255:
            return self.min_proba*0.0
        x, y = coordenadas
        if 0 <= x < 270 and 0 <= y < 270:
            probabilidad = self.t[coordenadas]
            return probabilidad
        return self.min_proba


    
    def Negativa_Log_Verosimilitud(self, data):
        # Parámetros iniciales
        initial_params = [self.zhit, self.sigma2, self.zrand, self.zmaz]

        # Limites para los parametros: media puede ser cualquier cosa, sigma debe ser positivo
        bounds = [(1e-6, None), (1, 15), (1e-4, None), (1e-6, None)]

        # Minimizar la función de negativa log-verosimilitud
        result = minimize(negative_log_likelihood, initial_params, args=(data, self.z_max, ), method='L-BFGS-B', bounds=bounds)

        # Obtener los parámetros estimados
        # mu_est, sigma_est = result.x
        # print(result)
        return result.x
    
    def ver(self):
        while True:
            cv2.imshow('raw RGB', self.img_copy)
            if cv2.waitKey(1) & 0xFF == 27:
                break





def negative_log_likelihood(parametros, z, z_max):
    zhit, sigma2, zrand, zmax = parametros
    area, _ = quad(proba, -z_max, z_max, args=(sigma2, z_max, zhit, zrand, zmax))
    return -np.sum(np.log(proba2(z, sigma2, z_max, zhit, zrand, zmax, area)))


def normalizar_advanced(mapa_g):
    # std_val = np.std(mapa_g)
    # # print(std_val)
    # min_val = np.min(mapa_g) - std_val
    # max_val = np.max(mapa_g) + std_val
    # mapa_g_normalizado = (mapa_g - min_val) / (max_val - min_val)
    # print(f'Normalizacion ESPECISL: min: {np.min(mapa_g_normalizado)}, max: {np.max(mapa_g_normalizado)}')
    mapa_N_sum = mapa_g / (np.sum(mapa_g))*1000
    # print(f'Normalizacion SUMA: min: {np.min(mapa_N_sum)}, max: {np.max(mapa_N_sum)}')
    # mapa_N_softmax = softmax(mapa_g, 1.0)*100   # 0.5 bueno     #0.1 mucho mas extremo
    # print(f'Normalizacion SOFTMAX: min: {np.min(mapa_N_softmax)}, max: {np.max(mapa_N_softmax)}')
    return mapa_N_sum


def softmax(x, temperature=1.0, axis=None):
    # Ajustar los logits con la temperatura
    x_adjusted = x / temperature
    # Subtract the max value along the specified axis for numerical stability
    shifted_logits = x_adjusted - np.max(x_adjusted, axis=axis, keepdims=True)
    exps = np.exp(shifted_logits)
    sum_exps = np.sum(exps, axis=axis, keepdims=True)
    return exps / sum_exps


def gaussian_2Vd(z, sigma2, z_max, zhit):
    coef = 1 / np.sqrt(2 * np.pi * sigma2)
    exponent = -(z**2) / (2 * sigma2)
    gaussian = zhit*(coef * np.exp(exponent))
    gaussian[z > z_max] = 0.0  # Aplicar la condición
    return gaussian


def p_randV(z, z_max, zrand):
    p = np.zeros_like(z, dtype=np.float64)
    p[z <= z_max] = zrand*(1 / z_max)
    return p


def p_z_maxV(z, z_max, zmax):
    p = np.zeros_like(z, dtype=np.float64)
    p[z == z_max] = zmax*1.0
    return p


def proba(z, sigma2, z_max, zhit, zrand, zmaz):
    p_gausian = gaussian_2d(z, sigma2, z_max, zhit)
    p_prand = p_rand(z, z_max, zrand)
    p_max = p_z_max(z, z_max, zmaz)
    proba_final = p_gausian + p_prand + p_max
    return proba_final

def proba2(z, sigma2, z_max, zhit, zrand, zmaz, area):
    p_gausian = gaussian_2Vd(z, sigma2, z_max, zhit)
    p_prand = p_randV(z, z_max, zrand)
    p_max = p_z_maxV(z, z_max, zmaz)
    proba_final = p_gausian + p_prand + p_max
    return proba_final/area

def gaussian_2d(z, sigma2, z_max, zhit):
    if z <= z_max:
        coef = 1 / np.sqrt(2 * np.pi * sigma2)
        exponent = -(z**2) / (2 * sigma2)
        return (coef * np.exp(exponent))*zhit
    else:
        return 0.0


def p_rand(z, z_max, zrand):
    if z <= z_max:
        return (1/z_max)*zrand
    else:
        return 0.0


def p_z_max(z, z_max, zmaz):
    if z == z_max:
        return 1.0*zmaz
    else:
        return 0.0


def distancia(pz, pzk):
    zk_x, zk_y = pzk
    x, y = pz
    distancia = np.sqrt((x - zk_x)**2 + (y - zk_y)**2)
    return distancia

def find_nearest_coordinate(puntos_fijos, x_ref, y_ref):

    # Calcular las distancias euclidianas entre (x_ref, y_ref) y todas las coordenadas en puntos_fijos
    distancias = np.sqrt((puntos_fijos[:, 0] - x_ref)**2 + (puntos_fijos[:, 1] - y_ref)**2)

    # Encontrar el índice de la distancia mínima
    indice_min = np.argmin(distancias)

    # Devolver la coordenada correspondiente a la distancia mínima
    return tuple(puntos_fijos[indice_min])








def main(args=None):
    mapa = Likelihood_Map('mapa.pgm')
    pass


if __name__ == '__main__':
    main()