import sys
sys.path.insert(0, 'C:/Users/ferna/Dropbox/UC/Magister/robot-flautista')
import matplotlib.pyplot as plt
from src.cinematica import *
import json
from scipy import signal
import csv

def get_route_positions(xi, zi, alphai, xf, zf, alphaf, divisions=20, plot=False, aprox=True): 
    """
    Entrega una ruta en linea recta en el espacio de la tarea desde una posicion de inicio xi, zi, alphai hasta una posicion final xf, zf, alphaf, avanzando pasos homogeneos en cada uno de los ejes de la tarea (en cada paso se avanza el mismo diferencial de l, theta y offset).
    El numero de pasos viene dado por la cantidad de divisiones.
    La funcion retorna:
    - x_points: los puntos de x en la ruta
    - z_points: los puntos de z en la ruta
    - alpha_points: los puntos de alpha en la ruta
    - d: la distancia recorrida hasta cada punto de la ruta
    """
    # obtenemos las posiciones de inicio y fin en el espacio de la tarea
    ri, thetai, oi = get_l_theta_of(xi, zi, alphai)
    rf, thetaf, of = get_l_theta_of(xf, zf, alphaf)

    # calculamos cuanto queremos avanzar en cada eje
    deltaR = rf - ri
    deltaTheta = thetaf - thetai
    deltaO = of - oi

    x_a, z_a, alpha_a = xi, zi, alphai # partimos con los valores iniciales

    dist = 0
    d = []
    x_points = []
    z_points = []
    alpha_points = []
    for n in range(divisions+1):
        xn, zn, alphan = get_x_z_alpha(ri + n*deltaR/divisions, thetai + n*deltaTheta/divisions, oi + n*deltaO/divisions) # en cada iteracion calculamos la posicion en x, z y alpha luego de dar pasos homogeneos en el espacio de la tarea.
        x_points.append(round(xn,3))
        z_points.append(round(zn,3))
        alpha_points.append(round(alphan,3))
        dist += sqrt((xn - x_a)**2 + (zn - z_a)**2 + (alphan - alpha_a)**2) # calculamos la distancia euclidiana entre este punto y el anterior y la sumamos a la acumulada
        d.append(dist)
        x_a, z_a, alpha_a = xn, zn, alphan # actualizamos los valores anteriores

    return x_points, z_points, alpha_points, d

def max_dist_rec(acc, dec, T):
    """
    retorna la máxima distancia que se puede recorrer en un tiempo T con aceleracion acc y desaceleracion dec, partiendo y terminando en reposo y asumiendo que la velocidad nunca satura.
    """
    # se hace un perfil de velocidad triangular y se integra
    d_acc = (acc/2) * ((dec*T)/(acc+dec))**2
    d_dec = acc*(dec*T)/(acc+dec) * (T-(dec*T)/(acc+dec)) / 2
    dist_max = d_acc + d_dec
    return dist_max

def plan_speed_curve(d, acceleration, deceleration, T):
    """
    A partir de una distancia que se quiere recorrer, una aceleracion, una desaceleracion y un tiempo total T, esta funcion retorna la velocidad que se debe alcanzar en un perfil de velocidad trapezoidal, asi como el tiempo en el que se alcanza y el tiempo en el cual comienza a desacelerar.
    """
    speed = (acceleration*deceleration*T - sqrt(acceleration*deceleration*(acceleration*deceleration*T**2 - 2*acceleration*d - 2*deceleration*d))) / (acceleration+deceleration) # obtenemos esta solucion luego de calcular el area del trapezoide como la multiplicacion de su semibase (T + T-v/acc-v/dec) con la altura (v) y despejando para v. Nos da una ecuacion cuadrática 
    t_acc = speed / acceleration
    t_dec = T - speed / deceleration
    return speed, t_acc, t_dec

def plan_temps_according_to_speed(distances, vel, t_acc, t_dec, acc, dec):
    """
    A partir de un vector de distancias avanzadas, y un perfil de velocidad trapezoidal bien definido, esta funcion entrega un vector de tiempos en los cuales el robot se encuentra en cada una de las distancias del vector entregado. 
    """
    ## Como dato recibimos un vector con las distancias recorridas hasta el tiempo t, y queremos calcular los tiempos en los cuales estaremos ahi.
    ## Sabemos además que el perfil de velocidades es trapezoidal, y conocemos las caracteristicas de este perfil: aceleracion, tiempo de aceleracion, velocidad maxima, tiempo de desaceleración y desaceleración. 
    ## Con estos datos es trivial encontrar una funcion que al entregarle el tiempo nos diga que distancia habremos recorrido (integrando la curva).
    ## Es decir, podemos calcular d(t) facilmente. Luego solo nos falta invertirla para encontrar t(d).
    ## Al tener esta función, le podemos dar como entrada la distancia recorrida y nos entregará el tiempo en el cual lo recorrió.

    # por el perfil de velocidades sabemos que se trata de una funcion por partes, asique calculamos las distancias donde la curva cambia
    d_t_acc = acc * t_acc**2 / 2  # distancia cuando termina de acelerar
    d_t_dec = d_t_acc + vel * (t_dec - t_acc) # distancia cuando empieza a desacelerar
    temps = [] 
    for d_sum in distances:
        if d_sum < d_t_acc: # en la primera parte de la curva. La distancia es el area de un triangulo hasta t, luego lo invertimos
            temps.append(sqrt(2*d_sum/acc))
        elif d_sum < d_t_dec: # en la segunda parte de la curva. La distancia es la suma de lo que avanzo acelerando con lo que avanza a velocidad constante. Luego se invierte
            temps.append((d_sum - d_t_acc)/vel + t_acc)
        else: # en la tercera parte de la curva. La distancia es la suma de las dos primeras más la del ultimo triángulo, luego invertimos. Como esta ultima área parte desacelerando desde una velocidad no nula, nos queda una funcion cuadrática con soluciones reales distintas. Para esto usamos la solucion de ecuaciones cuadráticas clasica
            a = dec / 2
            b = -(vel + (2*t_dec*dec)/2)
            c = d_sum - d_t_dec + vel*t_dec + (dec*t_dec**2)/2
            t = (-b - sqrt(round(b**2 - 4*a*c,3)))/(2*a)
            temps.append(t)
    return temps

def x_mm_to_units(mm, aprox=True):
    # los dispositivos se programaron con 4000 pasos por vuelta y el eje avanza 8mm por vuelta
    if aprox:
        return int(mm * 4000 / 8 )
    else:
        return mm * 4000 / 8

def x_units_to_mm(units):
    # los dispositivos se programaron con 4000 pasos por vuelta y el eje avanza 8mm por vuelta
    return units * 8 / 4000

def encoder_units_to_mm(units):
    # los encoders se programaron con 4000 pasos por vuelta y el eje avanza 8mm por vuelta
    return units * 8 / 4000

def encoder_units_to_angle(units):
    # los encoders se programaron con 4000 pasos por vuelta
    return units * 360 / 4000
    
def z_mm_to_units(mm, aprox=True):
    # los dispositivos se programaron con 4000 pasos por vuelta y el eje avanza 8mm por vuelta
    if aprox:
        return int(mm * 4000 / 8 )
    else:
        return mm * 4000 / 8

def z_units_to_mm(units):
    # los dispositivos se programaron con 4000 pasos por vuelta y el eje avanza 8mm por vuelta
    return units * 8 / 4000
    
def alpha_angle_to_units(angle, aprox=True):
    # los dispositivos se programaron con 4000 pasos por vuelta
    if aprox:
        return int(angle * 4000 / 360)
    else:
        return angle * 4000 / 360

def alpha_units_to_angle(units):
    # los dispositivos se programaron con 4000 pasos por vuelta
    return units * 360 / 4000
    
def plan_route(x_points, z_points, alpha_points, temps, aprox=True):
    """
    A partir de vectores para la trayectoria de x, z, alpha y los tiempos, devuelve un diccionario con las posiciones transformadas a pasos de los motores.
    """
    points = {'x': [], 'z': [], 'alpha': [], 't': []}

    for i in range(len(x_points) - 1):
        x = x_mm_to_units(x_points[i], aprox=aprox)            
        z = z_mm_to_units(z_points[i], aprox=aprox)
        alpha = alpha_angle_to_units(alpha_points[i], aprox=aprox)
        t = temps[i]

        points['x'].append(x)
        points['z'].append(z)
        points['alpha'].append(alpha)
        points['t'].append(t)

    return points

def get_min_T(d, acc, dec, v_max=4000):
    """
    calcula el tiempo mínimo en el que se puede desplazar una distancia d comenzando y terminando en reposo con aceleracion acc, desaceleracion dec y velocidad de saturacion v_max
    """
    dist_1 = v_max**2 / (2*acc) + v_max**2 / (2*dec) # calculamos la distancia que podemos avanzar si aceleramos hasta v_max y al alcanzarla inmediatamente desaceleramos hasta 0.
    if dist_1 < d: # si esta distancia es menor a la distancia d que queremos recorrer, significa que para recorrerla de la forma más rapida vamos a llegar hasta v_max, permanecer un tiempo a esa velocidad y luego desacelerar. 
        dif = d - dist_1 # la distancia que nos faltó recorrer es la que se avanza a v_max
        return v_max / acc + v_max / dec + dif / v_max # luego el tiempo minimo que toma recorrer esa distancia es la suma del tiempo de aceleracion, del tiempo de desaceleracion y del tiempo que se permanece a velocidad máxima para recorrer la diferencia de distancia
    else: # si la distancia es mayor que la que se quiere recorrer, significa que no es necesario llegar hasta v_max y luego desacelerar, sino que podemos llegar hasta una velocidad menor. El perfil de velocidades que lo recorre en menos tiempo es triangular, aumenta hasta alcanzar una velocidad v_2 y al alcanzarla inmediatamente desacelera hasta 0
        v_2 = sqrt(2*acc*dec*d/(acc+dec)) # calculamos esta velocidad v_2 < v_max igualando el area del triángulo con la distancia d y despejando v_2
        return v_2 / acc + v_2 / dec # luego el tiempo seria la suma del que pasa acelerando con el que pasa desacelerando

def get_1D_route(initial_p, final_p, speed, acc=20, dec=20):
    """
    Retorna la trayectoria en linea recta para un vector de 1 dimension siguiendo un perfil de velocidades trapezoidal. 
    Sus argumentos son:
    - initial_p (int): posicion de inicio
    - final_p (int): posicion final
    - speed (int): parametro de 1 a 100 que indica que tan rapido se quiere realizar el movimiento
    - acc (int): aceleracion del movimiento (en pasos/s^2)
    - dec (int): desaceleracion del movimiento (en pasos/s^2)
    """
    d = abs(final_p - initial_p) # cantidad de pasos a recorrer
    if d == 0:
        return [] # si no hay diferencia, ya estamos en el punto final y no hay trayectoria que planificar
    min_T = get_min_T(d, acc, dec) # calculamos el tiempo mínimo en el que se puede hacer el movimiento
    T = min_T * (100/speed) # calculamos el tiempo que nos queremos demorar, que sea inversamente proporcional al parametro de velocidad y que en el caso más rapido podamos realizarlo (cuando speed=100)
    vel, t_acc, t_dec = plan_speed_curve(d, acc, dec, T) # calculamos los parametros del perfil de la velocidad
    x_points = linspace(initial_p, final_p, int(T*200)) # armamos un arreglo, con 200 puntos por segundo
    temps = plan_temps_according_to_speed(abs(x_points - initial_p), vel, t_acc, t_dec, acc, dec) # calculamos el vector de tiempos en los que debería pasar por cada punto

    # ahora calculamos las velocidades
    speeds = []
    for i in range(len(temps) - 1):
        dT = (temps[i + 1] - temps[i]) # diferencial de tiempo entre dos puntos
        if dT == 0:
            speeds.append(0)
        else:
            speeds.append(int((x_points[i + 1] - x_points[i]) / dT)) # vel = dist / t
    speeds.append(0) # agregamos la velocidad final
    
    return temps, x_points, speeds

def get_route(initial_state, final_state, acc=20, dec=20, T=None, divisions=100, aprox=False, speed=1):
    """
    A partir de un estado inicial y un estado final planifica una trayectoria en line recta en el espacio de la tarea y con un perfil de velocidad trapezoidal con aceleracion y desaceleracion regulables. T es el tiempo de duracion del movimiento, en caso de ser imposible realizar el movimiento con las aceleraciones y desaceleraciones en un tiempo T la funcion retorna None.
    En caso de que no se especifique un T, se busca un T en el que es posible realizar el movimiento, inversamente proporcional a la velocidad (que va de 1 a 100). El número de divisiones (que son por segundo) determina N. 
    
    La trayectoria que entrega es de la forma:
        {
            't': [t_0, t_1, ..., t_N],
            'x': [x_0, x_1, ..., x_N],
            'z': [z_0, z_1, ..., z_N],
            'alpha': [alpha_0, alpha_1, ..., alpha_N],
            'flow': [flow_0, flow_1, ..., flow_N],
            'x_vel': [x_vel_0, x_vel_1, ..., x_vel_N],
            'z_vel': [z_vel_0, z_vel_1, ..., z_vel_N],
            'alpha_vel': [alpha_vel_0, alpha_vel_1, ..., alpha_vel_N],
        }
    """
    x_points, z_points, alpha_points, d = get_route_positions(*initial_state.cart_coords(), *final_state.cart_coords(), divisions=divisions, plot=False) # primero generamos la lista de puntos (igualmente separados) desde el estado de inicio hasta el final en linea recta en el espacio de la tarea
    if not T: # si no se especifica T, partimos con un T = 0.1 s y vamos sumandole 0.1 hasta que sea posible realizar el movimiento
        T = 0.1
        while True:
            if not max_dist_rec(acc, dec, T) < d[-1]: # en d se tiene una lista de las distancias hasta esa parte del recorrido. El ultimo elemento por lo tanto tiene la distancia total
                break # cuando es posible realizar el movimiento en ese tiempo se rompe el loop
            T += 0.1
        T = T*2/speed # por seguridad multiplicamos T por 2 y lo dividimos en la velocidad
    else: # si en cambio se da un T de forma explicita, debemos comprobar que sea posible realizar el movimiento en ese tiempo
        if max_dist_rec(acc, dec, T) < d[-1]:
            print(f'Impossible to achieve such position with given acceleration and deceleration. {d[-1]} > {max_dist_rec(acc, dec, T)}')
            return None
    x_points, z_points, alpha_points, d = get_route_positions(*initial_state.cart_coords(), *final_state.cart_coords(), divisions=int(divisions*T), plot=False) # volvemos a generar los puntos de la trayectoria, pero esta vez con $divisions*T$ divisiones
    vel, t_acc, t_dec = plan_speed_curve(d[-1], acc, dec, T) # generamos el perfil trapezoidal, con velocidad que alcanza, tiempo en el que deja de acelerar y tiempo en el que empieza a desacelerar
    temps = plan_temps_according_to_speed(d, vel, t_acc, t_dec, acc, dec) # generamos un arreglo con el tiempo escalado en el cual tendría que pasar por cada punto
    route = plan_route(x_points, z_points, alpha_points, temps, aprox=aprox) # con todo esto, formamos la trayectoria
    route['x'].append(x_mm_to_units(final_state.x, aprox=aprox)) # agregamos el estado final
    route['z'].append(z_mm_to_units(final_state.z, aprox=aprox)) # agregamos el estado final
    route['alpha'].append(alpha_angle_to_units(final_state.alpha, aprox=aprox)) # agregamos el estado final
    route['t'].append(T) # agregamos el estado final

    # ahora calculamos las velocidades
    route['x_vel'] = []
    route['z_vel'] = []
    route['alpha_vel'] = []
    
    for i in range(len(route['t']) - 1):
        dT = (route['t'][i + 1] - route['t'][i])
        if dT == 0: # si el diferencial de tiempo es 0 (que no debiese pasar, pero por aproximacion podria) decimos que la velocidad es 0
            route['x_vel'].append(0)
            route['z_vel'].append(0)
            route['alpha_vel'].append(0)
        else: # calculamos la velocidad como la pendiente
            route['x_vel'].append(int((route['x'][i + 1] - route['x'][i]) / dT))
            route['z_vel'].append(int((route['z'][i + 1] - route['z'][i]) / dT))
            route['alpha_vel'].append(int((route['alpha'][i + 1] - route['alpha'][i]) / dT))
    route['x_vel'].append(0) # agregamos la velocidad final = 0
    route['z_vel'].append(0) # agregamos la velocidad final = 0
    route['alpha_vel'].append(0) # agregamos la velocidad final = 0
    
    # finalmente aproximamos todo (porque los dispositivos reciben int)
    route['x'] = list(map(lambda x: round(x), route['x']))
    route['z'] = list(map(lambda x: round(x), route['z']))
    route['alpha'] = list(map(lambda x: round(x), route['alpha']))

    route['x_vel'] = list(map(lambda x: round(x), route['x_vel']))
    route['z_vel'] = list(map(lambda x: round(x), route['z_vel']))
    route['alpha_vel'] = list(map(lambda x: round(x), route['alpha_vel']))

    # ahora nos ocupamos de la trayectoria para el flujo
    Fi = initial_state.flow
    Ff = final_state.flow
    deformation = 1 # el factor de deformacion indica la concavidad con que cambia el flujo. Un factor <1 cambia muy rapido al principio y despues se estanca, en cambio un factor > 1 cambia muy rapido al final. Con un factor = 1, la curva es recta
    route['flow'] = []

    for i in range(len(route['t'])):
        t = route['t'][i]
        ramp = Fi + (Ff-Fi) * (t / T) ** deformation # calculamos el flujo
        flow_sat = max(0,min(50, ramp)) # saturamos entre 0 y 50
        route['flow'].append(flow_sat)
        
    return route

def get_value_from_func(t, func, approx=True):
    """
    A partir de una función f: t -> x y un tiempo especifico t* retorna el valor de la funcion en ese punto interpolando linealmente
    """
    t_val = min(int((len(func) - 1) * t / max(func[-1][0], 0.000001)), len(func) - 1) # devuelve el indice del elemento de la funcion con el tiempo más cercano a t. (un approach para no recorrer toda la lista)
    if t < func[t_val][0]: # si t en verdad es menor al tiempo que encontramos
        while t < func[t_val][0]: # empezamos a recorrer hacia atras hasta que encontramos que t deja de ser menor que el tiempo de un elemento
            t_val -= 1
            if t_val < 0: # si llegamos al primer elemento de la lista, nos quedamos con ese
                if approx:
                    return round(func[0][1])
                else:
                    return func[0][1]
        # sale del while porque encontro un t_val tal que func[t_val][0] < t < func[t_val+1][0]
        # hacemos una interpolacion lineal entre los dos elementos consecutivos
        r = func[t_val][1] + ((t - func[t_val][0]) / (func[t_val + 1][0] - func[t_val][0])) * (func[t_val + 1][1] - func[t_val][1])
        if approx:
            return round(r)
        else:
            return r
    else: # si t en verdad es mayor o igual al tiempo que encontramos
        while t > func[t_val][0]:
            t_val += 1
            if t_val >= len(func): # si llegamos al último elemento de la lista, nos quedamos con ese
                if approx:
                    return round(func[-1][1])
                else:
                    return func[-1][1]
        # sale del while porque encontro un t_val tal que func[t_val-1][0] < t < func[t_val][0]
        # hacemos una interpolacion lineal entre los dos elementos consecutivos
        r = func[t_val - 1][1] + ((t - func[t_val - 1][0]) / (func[t_val][0] - func[t_val - 1][0])) * (func[t_val][1] - func[t_val - 1][1])
        if approx:
            return round(r)
        else:
            return r

def get_value_from_func_2d(t, func): 
    """
    A partir de una función f: t -> x, x' y un tiempo especifico t* retorna el valor de la funcion en ese punto interpolando linealmente
    """
    t_val = min(int((len(func) - 1) * t / max(0.000001, func[-1][0])), len(func) - 1) # devuelve el indice del elemento de la funcion con el tiempo más cercano a t. (un approach para no recorrer toda la lista)
    if t < func[t_val][0]: # si t en verdad es menor al tiempo que encontramos
        while t < func[t_val][0]: # empezamos a recorrer hacia atras hasta que encontramos que t deja de ser menor que el tiempo de un elemento
            t_val -= 1
            if t_val < 0: # si llegamos al primer elemento de la lista, nos quedamos con ese
                return func[0][1], func[0][2]
        # sale del while porque encontro un t_val tal que func[t_val][0] < t < func[t_val+1][0]
        # hacemos una interpolacion lineal entre los dos elementos consecutivos
        return round(func[t_val][1] + ((t - func[t_val][0]) / (func[t_val + 1][0] - func[t_val][0])) * (func[t_val + 1][1] - func[t_val][1])), round(func[t_val][2] + ((t - func[t_val][0]) / (func[t_val + 1][0] - func[t_val][0])) * (func[t_val + 1][2] - func[t_val][2]))
    else: # si t en verdad es mayor o igual al tiempo que encontramos
        while t > func[t_val][0]:
            t_val += 1
            if t_val >= len(func): # si llegamos al último elemento de la lista, nos quedamos con ese
                return func[-1][1], func[-1][2]
        # sale del while porque encontro un t_val tal que func[t_val-1][0] < t < func[t_val][0]
        # hacemos una interpolacion lineal entre los dos elementos consecutivos
        return round(func[t_val - 1][1] + ((t - func[t_val - 1][0]) / (func[t_val][0] - func[t_val - 1][0])) * (func[t_val][1] - func[t_val - 1][1])), round(func[t_val - 1][2] + ((t - func[t_val - 1][0]) / (func[t_val][0] - func[t_val - 1][0])) * (func[t_val][2] - func[t_val - 1][2]))


def third_order_time_scale(t, T):
    """
    entrega el tiempo escalado por un polinomio de orden 3, para suavizar los cambios. Ver libro Modern Robotics cap. 9
    s(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3
    """
    a0 = 0
    a1 = 0
    a2 = 3 / (T**2)
    a3 = -2 / (T**3)
    return a0 + a1*t + a2*t**2 + a3*t**3

def third_order_time_scale_speed(t, T):
    """
    entrega la derivada del tiempo t escalado por un polinomio de orden 3, para suavizar los cambios. Ver libro Modern Robotics cap. 9
    s'(t) = a_1 + 2 a_2 t + 3 a_3 t^2
    """
    a1 = 0
    a2 = 3 / (T**2)
    a3 = -2 / (T**3)
    return a1 + 2*a2*t + 3*a3*t**2

def time_scaled_straight_line(route, T):
    """
    A partir de una ruta de la forma [(x_0, z_0, alpha_0), (x_1, z_1, alpha_1), ..., (x_N, z_N, alpha_N)] y un tiempo total T entrega una trayectoria de la forma:
        {
            'x': [[t_0, x_0, x'_0], [t_1, x_1, x'_1], ..., [t_N, x_N, x'_N]],
            'z': [[t_0, z_0, z'_0], [t_1, z_1, z'_1], ..., [t_N, z_N, z'_N]],
            'alpha': [[t_0, alpha_0, alpha'_0], [t_1, alpha_1, alpha'_1], ..., [t_N, alpha_N, alpha'_N]]
        }
    Donde el tiempo es escalado por un polinomio de tercer orden, para suavizar el movimiento.
    Ver libro Modern Robotics cap. 9
    """
    r = {'x': [],
         'z': [],
         'alpha': []}
    vel_route = gradient(route)
    for i in range(len(route) + 1):
        s = third_order_time_scale(i * T / len(route), T) # calculamos el valor del tiempo escalado
        sp = third_order_time_scale_speed(i * T / len(route), T)
        # el tiempo simplemente se consigue multiplicando s * T, recordemos que s -> [0, 1] 
        # la posición la obtenemos a partir de la posicion en la ruta que esté en el s%
        # la velocidad la calculamos con la regla de la cadena: x' = dx/ds * ds/dt, donde ds/dt lo obtenemos de la funcion third_order_time_scale_speed y dx/ds es el gradiente de la ruta (en la posicion del s%) multiplicada por la frecuencia de sampleo len(route)/T
        r['x'].append([s*T, route[int(s*(len(route)-1))][0], int(round(sp*vel_route[0][int(s*(len(route)-1))][0]*len(route)/T,0))])
        r['z'].append([s*T, route[int(s*(len(route)-1))][1], int(round(sp*vel_route[0][int(s*(len(route)-1))][1]*len(route)/T,0))])
        r['alpha'].append([s*T, route[int(s*(len(route)-1))][2], int(round(sp*vel_route[0][int(s*(len(route)-1))][2]*len(route)/T,0))])
    return r


if __name__ == '__main__':
    x_i = x_mm_to_units(19.976)
    z_i = z_mm_to_units(20.034)
    alpha_i = alpha_angle_to_units(0.09)
    x_f = x_mm_to_units(20.82116192497326)
    z_f = z_mm_to_units(10.545442961167977)
    alpha_f = alpha_angle_to_units(10.09)
    T = 2.0
    r = time_scaled_straight_line(x_i, z_i, alpha_i, x_f, z_f, alpha_f, T)
    print(r['x'])