# from asyncore import read
# from turtle import color
from numpy import *
import json
import os
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True

class State:
    '''
    Esta clase es de utilidad para transformaciones entre el espacio de la tarea y el de las articulaciones.
    '''
    def __init__(self, r, theta, o, flow, vibrato_freq=0, vibrato_amp=0, lips=31, tongue=0):
        self._r = r
        self._theta = theta
        self._o = o
        self.not_posible = False
        try:
            x, z, alpha = get_x_z_alpha(r, theta, o)
        except (ValueError, ZeroDivisionError, ArithmeticError):
            x, z, alpha = 0, 0, 0
            self.not_posible = True
        self._x = x
        self._z = z
        self._alpha = alpha
        self.flow = flow
        self.vibrato_freq = vibrato_freq
        self.vibrato_amp = vibrato_amp
        self.lips = lips
        self.tongue = tongue
    
    @property
    def r(self):
        return round(self._r,2)
    
    @r.setter
    def r(self, other):
        self._r = other
        try:
            self._x, self._z, self._alpha = get_x_z_alpha(self._r, self._theta, self._o)
            self.not_posible = False
        except (ValueError, ZeroDivisionError, ArithmeticError):
            self.not_posible = True
    
    @property
    def theta(self):
        return round(self._theta,2)
    
    @theta.setter
    def theta(self, other):
        self._theta = other
        try:
            self._x, self._z, self._alpha = get_x_z_alpha(self._r, self._theta, self._o)
            self.not_posible = False
        except (ValueError, ZeroDivisionError, ArithmeticError):
            self.not_posible = True
    
    @property
    def o(self):
        return round(self._o,2)
    
    @o.setter
    def o(self, other):
        self._o = other
        try:
            self._x, self._z, self._alpha = get_x_z_alpha(self._r, self._theta, self._o)
            self.not_posible = False
        except (ValueError, ZeroDivisionError, ArithmeticError):
            self.not_posible = True
    
    @property
    def x(self):
        return round(self._x,2)
    
    @x.setter
    def x(self, other):
        self._x = other
        self._r, self._theta, self._o = get_l_theta_of(self._x, self._z, self._alpha)
    
    @property
    def z(self):
        return round(self._z, 2)
    
    @z.setter
    def z(self, other):
        self._z = other
        self._r, self._theta, self._o = get_l_theta_of(self._x, self._z, self._alpha)
    
    @property
    def alpha(self):
        return round(self._alpha,2)
    
    @alpha.setter
    def alpha(self, other):
        self._alpha = other
        self._r, self._theta, self._o = get_l_theta_of(self._x, self._z, self._alpha)
    
    def __str__(self):
        return f'r: {self.r}, theta: {self.theta}, offset: {self.o}, flow: {self.flow}'
    
    def cart_coords(self):
        return self.x, self.z, self.alpha
    
    def flute_coords(self):
        return self.r, self.theta, self.o

    # def homed(self):
    #     self.x = 0
    #     self.z = 0
    #     self.alpha = 0
    
    def change_state(self, other):
        self.r = other.r
        self.theta = other.theta
        self.o = other.o
        self.flow = other.flow
        self.vibrato_amp = other.vibrato_amp
        self.vibrato_freq = other.vibrato_freq

    # def is_too_close(self, other, thr_x=0.3, thr_z=0.3, thr_alpha=0.5):
    #     if abs(other.x - self.x) < thr_x and abs(other.z - self.z) < thr_z and abs(other.x - self.x) < thr_alpha:
    #         return True
    #     return False


def read_variables():
    ## lee un archivo que contiene datos de la configuración utilizada.
    dir = os.path.dirname(os.path.realpath(__file__)) + '/settings.json'
    with open(dir) as json_file:
        DATA = json.load(json_file)
    return DATA

DATA = read_variables()
DATA_dir = dir = os.path.dirname(os.path.realpath(__file__)) + '\settings.json'

def save_variables():
    global DATA, DATA_dir
    with open(DATA_dir, 'w') as json_file:
        json.dump(DATA, json_file, indent=4, sort_keys=True)


def get_pos_punta(x,z,alpha, offset_x=0, offset_z=0):
    ## calcula la posición de la punta de la boca a partir de la posición en cada articulacion
    global DATA
    dh = DATA["physical_constants"]["dh"]
    dv = DATA["physical_constants"]["dv"]
    x2 = x + dh * cos(alpha) - dv * sin(alpha)
    z2 = z + dh * sin(alpha) + dv * cos(alpha)
    return x2, z2

def get_x_z_from_punta(x2, z2, alpha):
    ## funcion inversa de get_pos_punta
    global DATA
    x = x2 - DATA["physical_constants"]["dh"]*cos(alpha) + DATA["physical_constants"]["dv"]*sin(alpha)
    z = z2 - DATA["physical_constants"]["dh"]*sin(alpha) - DATA["physical_constants"]["dv"]*cos(alpha)
    return x, z

def get_l_theta_of(x,z,alpha):
    ## funcion para pasar del espacio de las articulaciones al de la tarea
    global DATA
    alpha_f = DATA["flute_position"]["alpha_flauta"]
    xf = DATA["flute_position"]["X_F"]
    zf = DATA["flute_position"]["Z_F"]
    alpha = alpha * pi / 180
    theta = alpha + alpha_f
    x2, z2 = get_pos_punta(x,z,alpha)
    l = (xf - x2) * cos(alpha) + (zf - z2) * sin(alpha)
    of = - (xf - x2) * sin(alpha) + (zf - z2) * cos(alpha)
    return l, theta*180/pi, of

def get_x_z_alpha(l, theta, of):
    ## funcion para pasar del espacio de las tarea al de las articulaciones
    global DATA
    alpha_f = DATA["flute_position"]["alpha_flauta"]
    xf = DATA["flute_position"]["X_F"]
    zf = DATA["flute_position"]["Z_F"]
    theta = theta * pi / 180
    alpha = theta - alpha_f
    x2 = - l * cos(alpha) + of * sin(alpha) + xf
    z2 = - l * sin(alpha) - of * cos(alpha) + zf
    x, z = get_x_z_from_punta(x2, z2, alpha)
    return x, z, alpha*180/pi


change_to_joint_space = vectorize(get_x_z_alpha) ## funcion get_x_z_alpha en forma vectorial
change_to_task_space = vectorize(get_l_theta_of) ## funcion get_l_theta_of en forma vectorial

def x_mm_to_units(mm):
    ## para transformar de mm a pasos de los motores en ejes x y z
    return int(round(mm * 4000 / 8 , 0))

def alpha_angle_to_units(angle):
    ## para transformar de ° en el eje alpha a pasos del mismo motor
    return int(round(angle * 4000 / 360, 0))
    
mm2units = vectorize(x_mm_to_units) ## funcion x_mm_to_units en forma vectorial
angle2units = vectorize(alpha_angle_to_units) ## funcion alpha_angle_to_units en forma vectorial

if __name__ == "__main__":
    pass