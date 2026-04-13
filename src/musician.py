from multiprocessing import Process, Event, Value, Pipe
import time
import logging
import pandas as pd
from src.drivers import DATA, AMCIDriver, INPUT_FUNCTION_BITS, FlowControllerDriver, FingersDriver, PressureSensor, Microphone, SAFETY_WATCHDOG_TIMEOUT
from src.communication import CommunicationCenter
from src.motor_route import *
from src.route import *
from src.cinematica import *

logger = logging.getLogger('pierre.musician')

class Musician(Process):
    """
    Esta clase es el cerebro del robot, recibe todas las solicitudes de la interfaz grafica, las procesa, llama las funciones para crear las trayectorias y le envia instrucciones específicas a cada uno de los drivers para cada dispositivo.
    """
    def __init__(self, host, connections, running, end_pipe, data, interval=0.01, home=True, x_connect=True, z_connect=True, alpha_connect=True, flow_connect=True, fingers_connect=True, pressure_sensor_connect=True, mic_connect=True):
        Process.__init__(self) # Initialize the threading superclass
        global DATA
        self.t0 = time.perf_counter()
        self.host = host # 192.168.2.10
        self.running = running # evento que conecta los procesos. Si se le hace clear() terminan todos los procesos y se cierra el programa
        self.end_pipe = end_pipe # pipe que conecta con la interfaz gráfica
        self.interval = interval # tiempo de espera entre iteracion
        self.x_connect = x_connect # condicion de conectividad con driver del eje x
        self.z_connect = z_connect # condicion de conectividad con driver del eje z
        self.alpha_connect = alpha_connect # condicion de conectividad con driver del eje alpha
        self.flow_connect = flow_connect # condicion de conectividad con controlador de flujo
        self.fingers_connect = fingers_connect # condicion de conectividad con driver de los dedos
        self.pressure_sensor_connect = pressure_sensor_connect # condicion de conectividad con sensor de presion
        self.mic_connect = mic_connect # condicion de conectividad con el microfono
        self.connections = connections # lista con todas las direcciones IP de los dispositivos en el orden: [ip_driver_x, ip_driver_z, ip_driver_alpha, ip_controlador_flujo, ip_sensor_presion, ip_driver_dedos]
        self.home = home # condicion de hacer la rutina de homing al inicio de la operacion
        self.instrument = 'flute' # instrumento que se usa, puede ser flute, quena o test
        self.data = data # data compartida entre procesos
        
        self.loaded_route_x = [] # arreglo donde se almacenara una ruta pre cargada para el eje x
        self.loaded_route_z = [] # arreglo donde se almacenara una ruta pre cargada para el eje z 
        self.loaded_route_alpha = [] # arreglo donde se almacenara una ruta pre cargada para el eje alpha
        self.loaded_route_flow = [] # arreglo donde se almacenara una ruta pre cargada para el flujo
        self.loaded_route_notes = [] # arreglo donde se almacenara una ruta pre cargada para los dedos
        self.loaded_route_aperture = []
        self.loaded_route_tongue = []
    
    def run(self): # funcion principal, que se llama cuando se inicia el proceso
        global DATA
        print("Running musician...") # agregamos algunos prints para ver como va el proceso de inicio de todas las etapas

        if self.x_connect or self.z_connect or self.alpha_connect or self.flow_connect or self.pressure_sensor_connect or self.fingers_connect:
            communication_connect = True  # si hay algun dispositivo conectado (que use protocolo de comunicacion ethernet ip), aunque sea solo uno, habra que abrir una conexion de ethernet ip
        else:
            communication_connect = False # si no hay dispositivos conectados, se evita abrir tal conexion

        print("Connecting communications...")
        self.comm_event = Event() # para la central de comunicaciones creamos un segundo evento, que se despeja un poco despues del principal (se usa un time.sleep) para dar tiempo a los dispositivos a cerrar las conexiones antes de que se cierre el proceso a cargo de las comunicaciones
        self.comm_event.set()
        self.comm_pipe, comm_pipe2 = Pipe() # creamos un pipe para enviarle instrucciones a la central de comunicaciones
        self.communications = CommunicationCenter(self.host, self.comm_event, comm_pipe2, self.data, connect=communication_connect, verbose=False)
        self.communications.daemon = True
        self.communications.start()

        print("Communication started...\nConnecting Drivers...")

        ## ahora se crean todos los drivers
        ## para cada driver creamos un pipe distinto
        ## ademas, creamos un segundo pipe para cada driver que conecta directamente con los ejes virtuales que cada uno crea. 
        ## De esta forma el musico queda con un pipe que conecta directamente a cada driver, y un pipe que lo conecta a los ejes virtuales de cada driver
        self.x_driver_conn, x_driver_end_conn = Pipe()
        self.x_virtual_axis_conn, x_virtual_axis_end_conn = Pipe()

        self.x_driver = AMCIDriver(self.connections[0], self.running, x_driver_end_conn, self.comm_pipe, self.data, x_virtual_axis_end_conn, self.t0, connected=self.x_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'], virtual_axis_follow_acceleration=DATA['x_control']['acceleration'], virtual_axis_follow_deceleration=DATA['x_control']['deceleration'], home=self.home, use_encoder_bit=1, motor_current=30, virtual_axis_proportional_coef=DATA['x_control']['proportional_coef'], encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=1, Kp=DATA['x_control']['kp'], Ki=DATA['x_control']['ki'], Kd=DATA['x_control']['kd'], Kp_vel=DATA['x_control']['kp_vel'], Ki_vel=DATA['x_control']['ki_vel'], Kd_vel=DATA['x_control']['kd_vel'])
        ## starting_speed: velocidad con que inicia un movimiento. Es importante dejarla en 1
        ## verbose: condicion para imprimir informacion del funcionamiento, util para debugear
        ## input_2_function_bits: funcion que cumple el input 2 del driver. Ver manual
        ## virtual_axis_follow_acceleration: aceleracion con el que se sigue el eje virtual. Ver manual movimiento 'synchrostep'
        ## virtual_axis_follow_deceleration: deceleracion con el que se sigue el eje virtual. Ver manual movimiento 'synchrostep'
        ## home: si se hace la rutina de homing
        ## use_encoder_bit: si se usan encoders. Ver el manual
        ## motor_current: corriente que se envia a los motores. Cada unidad representa 0.1A. Ver manual
        ## virtual_axis_proportional_coef: coeficiente que determina que tan violento se sigue el eje virtual. Ver manual movimiento 'synchrostep'
        ## encoder_pulses_turn: pulsos por vuelta del encoder (dejar en 4000)
        ## motors_step_turn: pulsos por vuelta del motor (dejar en 4000)
        ## hybrid_control_gain: ver el manual, sirve para el movimiento hybrido (aunque no se esta usando)
        ## enable_stall_detection_bit: dejar en 0
        ## current_loop_gain: dejar en 1
        ## Kp: Kp del controlador de posicion
        ## Ki: Ki del controlador de posicion
        ## Kd: Kd del controlador de posicion
        ## Kp_vel: Kp del controlador de velocidad
        ## Ki_vel: Ki del controlador de velocidad
        ## Kd_vel: Kd del controlador de velocidad
        

        ## hacemos lo mismo para el driver del eje z
        self.z_driver_conn, z_driver_end_conn = Pipe()
        self.z_virtual_axis_conn, z_virtual_axis_end_conn = Pipe()

        self.z_driver = AMCIDriver(self.connections[1], self.running, z_driver_end_conn, self.comm_pipe, self.data, z_virtual_axis_end_conn, self.t0, connected=self.z_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'], virtual_axis_follow_acceleration=DATA['z_control']['acceleration'], virtual_axis_follow_deceleration=DATA['z_control']['deceleration'], home=self.home, use_encoder_bit=1, motor_current=30, virtual_axis_proportional_coef=DATA['z_control']['proportional_coef'], encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=1, Kp=DATA['z_control']['kp'], Ki=DATA['z_control']['ki'], Kd=DATA['z_control']['kd'], Kp_vel=DATA['z_control']['kp_vel'], Ki_vel=DATA['z_control']['ki_vel'], Kd_vel=DATA['z_control']['kd_vel'])
        
        ## y lo mismo para el driver del eje alpha
        self.alpha_driver_conn, alpha_driver_end_conn = Pipe()
        self.alpha_virtual_axis_conn, alpha_virtual_axis_end_conn = Pipe()
        
        self.alpha_driver = AMCIDriver(self.connections[2], self.running, alpha_driver_end_conn, self.comm_pipe, self.data, alpha_virtual_axis_end_conn, self.t0, connected=self.alpha_connect, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=DATA['alpha_control']['acceleration'], virtual_axis_follow_deceleration=DATA['alpha_control']['deceleration'], home=self.home, use_encoder_bit=1, motor_current=30, virtual_axis_proportional_coef=DATA['alpha_control']['proportional_coef'], encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=1, Kp=DATA['alpha_control']['kp'], Ki=DATA['alpha_control']['ki'], Kd=DATA['alpha_control']['kd'], Kp_vel=DATA['alpha_control']['kp_vel'], Ki_vel=DATA['alpha_control']['ki_vel'], Kd_vel=DATA['alpha_control']['kd_vel'])

        ## creamos el driver del controlador de flujo
        self.flow_driver_conn, flow_driver_end_conn = Pipe()
        self.virtual_flow_conn, virtual_flow_end_conn = Pipe()
        self.flow_driver = FlowControllerDriver(self.connections[3], self.running, self.t0, flow_driver_end_conn, self.comm_pipe, self.data, virtual_flow_end_conn, connected=self.flow_connect, verbose=False)

        ## del driver de los dedos
        self.fingers_driver_conn, fingers_driver_end_conn = Pipe()
        self.virtual_fingers_conn, virtual_fingers_end_conn = Pipe()
        
        self.fingers_driver = FingersDriver(self.connections[5], self.running, fingers_driver_end_conn, virtual_fingers_end_conn, self.t0, self.comm_pipe, connected=self.fingers_connect, verbose=False)
        
        ## del sensor de presion. Este como es solo un sensor no corre un eje virtual, asique creamos solo un pipe que lo conecta
        self.preasure_sensor_conn, preasure_sensor_end_conn = Pipe()
        self.preasure_sensor = PressureSensor(self.connections[4], self.running, preasure_sensor_end_conn, self.comm_pipe, self.data, connected=self.pressure_sensor_connect, verbose=False)
        
        ## creamos un objeto que se ocupa del microfono. Nuevamente es un sensor asique tampoco corre un eje virtual.
        self.mic_conn, mic_end_conn = Pipe()
        self.mic_running = Event() # tambien creamos un evento especial para esta clase, para poder cambiar el microfono durante la ejecucion del programa (sin terminar el proceso)
        # los parametros de la deteccion de pitch, asi como el dispositivo que se usa se toman de settings.json. Si el dispositivo indicado no existe puede lanzar error. En ese caso cambiar a un dispositivo que si exista. Ver sd.query_devices() en tal caso
        self.microphone = Microphone(self.running, mic_end_conn, DATA['frequency_detection']['device'], DATA['frequency_detection']['method'], DATA['frequency_detection']['YIN'], DATA['frequency_detection']['pYIN'], self.mic_running, connected=self.mic_connect, verbose=False)

        print("Drivers created...\nCreating memory...")
        ## se crearon todos los drivers que interactuan con dispositivos, ahora se crea un objeto 'memoria' que se ocupa de leer todos los sensores de forma sincrona y llevar un registro de sus lecturas
        self.memory_conn, memory_end_conn = Pipe() # tambien se crea un pipe para conectarse a este objeto y enviarle instrucciones como que empiece o termine de grabar
        self.memory = Memory(self.running, self.x_driver, self.z_driver, self.alpha_driver, self.flow_driver, self.preasure_sensor, self.microphone, self.fingers_driver, memory_end_conn, self.data, windowWidth=200, interval=0.01)

        print("Memory created...\nStarting...")
        ## Marcar todos los subprocesos como daemon para que mueran con el proceso padre
        for child in [self.memory, self.x_driver, self.z_driver,
                      self.alpha_driver, self.flow_driver, self.fingers_driver,
                      self.preasure_sensor, self.microphone]:
            child.daemon = True

        ## habiendo creado todos los objetos, se empiezan a correr.
        self.memory.start()

        self.x_driver.start()
        self.z_driver.start()
        self.alpha_driver.start()
        self.flow_driver.start()
        self.fingers_driver.start()
        self.preasure_sensor.start()

        self.microphone.start()

        self.end_pipe.send(['instances created']) # se avisa a la interfaz grafica que ya se crearon y corrieron todas las instancias para avanzar en la carga de la ventana de start-up. Aun queda que cada driver termine sus rutinas de inicio
        print("Pierre started listening...")

        devices_connected = 0 # contador de dispositivos que ya terminaron sus rutinas de inicio y estan operacionales
        # Mapa de pipes a mensajes esperados y notificaciones a la GUI
        _startup_pipes = [
            (self.x_driver_conn, "driver_started", "x_driver_started"),
            (self.z_driver_conn, "driver_started", "z_driver_started"),
            (self.alpha_driver_conn, "driver_started", "alpha_driver_started"),
            (self.memory_conn, "memory_started", "memory_started"),
            (self.mic_conn, "microphone_started", "microphone_started"),
            (self.flow_driver_conn, "flow_driver_started", "flow_driver_started"),
            (self.preasure_sensor_conn, "pressure_sensor_started", "pressure_sensor_started"),
            (self.fingers_driver_conn, "finger_driver_started", "finger_driver_started"),
        ]
        _startup_t0 = time.perf_counter()
        _startup_heartbeat = time.perf_counter()
        while True: # esperar a que todos los dispositivos terminen sus rutinas de inicio
            if devices_connected == 8:
                break
            if time.perf_counter() - _startup_t0 > 120:  # timeout global de 2 minutos para el startup
                logger.error(f"TIMEOUT: Solo {devices_connected}/8 dispositivos conectados despues de 120s")
                break
            # Enviar heartbeats durante startup para evitar watchdog timeouts en drivers ya arrancados
            if time.perf_counter() - _startup_heartbeat > SAFETY_WATCHDOG_TIMEOUT / 3:
                for pipe_conn, expected_msg, gui_msg in _startup_pipes[:3]:  # solo los 3 AMCI drivers
                    try:
                        pipe_conn.send(["heartbeat"])
                    except Exception:
                        pass
                _startup_heartbeat = time.perf_counter()
            # Revisar TODOS los pipes en cada iteracion (no elif) para no perder mensajes simultaneos
            for pipe_conn, expected_msg, gui_msg in _startup_pipes:
                if pipe_conn.poll(0.01):  # poll con timeout breve
                    message = pipe_conn.recv()
                    if message[0] == expected_msg:
                        devices_connected += 1
                        self.end_pipe.send([gui_msg])
                        logger.info(f"Dispositivo conectado: {gui_msg} ({devices_connected}/8)")

        _last_heartbeat = time.perf_counter()
        while self.running.is_set(): # habiendo partido todos los dispositivos, el musico entra en el loop principal donde queda escuchando cualquier instruccion desde la interfaz de usuario para ejecutarla
            # Enviar heartbeat periodico a los drivers (watchdog)
            if time.perf_counter() - _last_heartbeat > SAFETY_WATCHDOG_TIMEOUT / 3:
                try:
                    if self.x_connect:
                        self.x_driver_conn.send(["heartbeat"])
                    if self.z_connect:
                        self.z_driver_conn.send(["heartbeat"])
                    if self.alpha_connect:
                        self.alpha_driver_conn.send(["heartbeat"])
                except Exception:
                    pass  # si falla el heartbeat, el watchdog del driver se encargara
                _last_heartbeat = time.perf_counter()
            if self.end_pipe.poll(self.interval): # esperamos por self.interval ms un mensaje de la interfaz
                message = self.end_pipe.recv() # si hay mensaje lo recibimos y vemos que dice. El primer elemento del mensaje será un label de la operacion a realizar
                if message[0] == "execute_fingers_action": # si se pide un cambio de digitacion de dedos
                    self.virtual_fingers_conn.send(["merge_ref", [(0, message[1])]]) # se crea una ruta para el eje virtual de los dedos: [(0, message[1])] y se envia una solicitud de merge_ref
                elif message[0] == "execute_lips_surface_action": # si se pide un cambio en la apertura de los labios
                    self.virtual_fingers_conn.send(["merge_lips_surface_ref", [(0, message[1])]])
                elif message[0] == "execute_tongue_action": # si se pide un cambio en la posicion de la lengua
                    self.virtual_fingers_conn.send(["merge_tongue_ref", [(0, message[1])]])
                elif message[0] == "move_to": # se pide un cambio de estado
                    # en este caso message[1] es un objeto State (del estado al que se quiere mover), message[2] dice el tiempo en el que se quiere realizar el movimiento (si es None se calcula en base a la velocidad), message[3] condicion que indica que el movimiento es exclusivamente en el eje X, message[4] condicion que indica que el movimiento es exclusivamente en el eje Z, message[5] condicion que indica que el movimiento es exclusivamente en el eje alpha, message[6] condicion que indica que el movimiento es exclusivamente de flujo y message[7] la velocidad a la que se quiere mover (de 1 a 100)
                    self.move_to(message[1], T=message[2], only_x=message[3], only_z=message[4], only_alpha=message[5], only_flow=message[6], speed=message[7]) # ejecutamos la funcion move_to con todos los parametros entregados desde la interfaz de usuario
                elif message[0] == "move_to_final": # tambien se pide un cambio de estado, pero sin calcular una trayectoria. Simplemente se le pide a los dispositivos que se muevan a la direccion indicada en message[1] (de forma inmediata)
                    # esta instruccion se usa cuando el robot esta siendo controlado por el ps move, donde se envia posiciones de forma periodica, construyendo asi la trayectoria suave
                    desired = message[1] 
                    self.x_virtual_axis_conn.send(["merge_ref", [(0, mm2units(desired.x), 0)]])
                    self.z_virtual_axis_conn.send(["merge_ref", [(0, mm2units(desired.z), 0)]])
                    self.alpha_virtual_axis_conn.send(["merge_ref", [(0, angle2units(desired.alpha), 0)]])
                    self.virtual_flow_conn.send(["merge_ref", [(0, desired.flow)], 0, 0])
                elif message[0] == "reset_x_controller": # resetea el driver del motor del eje x por si tuvo alguna falla
                    pass # TODO
                elif message[0] == "reset_z_controller": # resetea el driver del motor del eje Z por si tuvo alguna falla
                    pass # TODO
                elif message[0] == "reset_alpha_controller": # resetea el driver del motor del eje alpha por si tuvo alguna falla
                    pass # TODO
                elif message[0] == "load_routes": # precarga las rutas que se entregan en los arreglos propios de esta clase, para esperar listo la instruccion de start y ejecutar las trayectorias
                    self.loaded_route_x = message[1]
                    self.loaded_route_z = message[2]
                    self.loaded_route_alpha = message[3]
                    self.loaded_route_flow = message[4]
                    self.loaded_route_notes = message[5]
                    self.loaded_route_aperture = message[6]
                    self.loaded_route_tongue = message[7]
                elif  message[0] == "start_loaded_script": # ejecuta las rutas que se habian cargado anteriormente
                    if message[1]: # el message[1] es un bool que dice si se graba durante la ejecucion
                        self.memory_conn.send(["start_saving"]) # la memoria va guardando los valores de cada parametro
                        self.mic_conn.send(["start_saving"]) # el microfono graba un buffer con el audio
                    self.start_loaded_script() # funcion que ejecuta las rutas pre-cargadas
                elif  message[0] == "start_free_recording": # 
                    self.memory_conn.send(["start_saving"]) # la memoria va guardando los valores de cada parametro
                    self.mic_conn.send(["start_saving"]) # el microfono graba un buffer con el audio
                elif  message[0] == "stop_free_recording": # 
                    self.memory_conn.send(["stop_recording"]) # la memoria va guardando los valores de cada parametro
                    self.mic_conn.send(["stop_recording"]) # el microfono graba un buffer con el audio
                    self.memory_conn.send(["save_recorded_data", message[1]])
                    self.mic_conn.send(["save_recorded_data", message[2]])
                elif message[0] == "stop_playing": # si se pide que se deje de ejecutar una trayectoria
                    self.stop() # para los motores y lleva el flujo a 0
                    if message[1]: # si se estaba grabando, se termina de grabar
                        self.memory_conn.send(["stop_recording"])
                        self.mic_conn.send(["stop_recording"]) #
                elif message[0] == "stop": # hace lo mismo que la anterior
                    self.stop()
                    self.memory_conn.send(["stop_recording"])
                    self.mic_conn.send(["stop_recording"])
                elif message[0] == "emergency_stop": # parada de emergencia inmediata desde GUI
                    self.emergency_stop()
                    self.memory_conn.send(["stop_recording"])
                    self.mic_conn.send(["stop_recording"])
                elif message[0] == "memory.save_recorded_data": # instruccion para guardar en un archivo los datos guardados durante una ejecucion previa. En message[1] tiene la direccion del archivo .csv y en message[2] la direccion del archivo .wav
                    self.memory_conn.send(["save_recorded_data", message[1]])
                    self.mic_conn.send(["save_recorded_data", message[2]])
                elif message[0] == "flow_driver.change_controlled_var": # instruccion de cambiar la variable controlada en el controlador de flujo
                    self.flow_driver_conn.send(["change_controlled_var", message[1]])
                elif message[0] == "flow_driver.change_control_loop": # instruccion de cambiar la forma del loop de control en el controlador de flujo
                    self.flow_driver_conn.send(["change_control_loop", message[1]])
                elif message[0] == "flow_driver.change_kp": # instruccion de cambiar el kp en el controlador de flujo
                    self.flow_driver_conn.send(["change_kp", message[1]])
                elif message[0] == "flow_driver.change_ki": # instruccion de cambiar el ki en el controlador de flujo
                    self.flow_driver_conn.send(["change_ki", message[1]])
                elif message[0] == "flow_driver.change_kd": # instruccion de cambiar el kd en el controlador de flujo
                    self.flow_driver_conn.send(["change_kd", message[1]])
                elif message[0] == "set_instrument": # instruccion de setear el instrumento. Actualmente no se usa, pero podría crearse una ventana de dialogo al principio del programa preguntando qué instrumento se quiere usar
                    self.set_instrument(message[1])
                elif message[0] == "x_driver.ask_control": # instruccion que pide reenviar el estado actual del controlador del driver del eje x
                    self.x_driver_conn.send(["ask_control"]) # se pregunta por el control
                    data = self.x_driver_conn.recv()[0] # se espera la respuesta
                    self.end_pipe.send([data]) # se reenvia la respuesta a la interfaz
                elif message[0] == "x_driver.change_control": # instruccion de actualizar el controlador del driver x
                    self.x_driver_conn.send(["change_control", message[1]]) # se reenvia el mensaje al driver x
                elif message[0] == "z_driver.ask_control": # instruccion que pide reenviar el estado actual del controlador del driver del eje z
                    self.z_driver_conn.send(["ask_control"]) # se pregunta por el control
                    data = self.z_driver_conn.recv()[0] # se espera la respuesta
                    self.end_pipe.send([data]) # se reenvia la respuesta a la interfaz
                elif message[0] == "z_driver.change_control": # instruccion de actualizar el controlador del driver z
                    self.z_driver_conn.send(["change_control", message[1]]) # se reenvia el mensaje al driver z
                elif message[0] == "alpha_driver.ask_control": # instruccion que pide reenviar el estado actual del controlador del driver del eje alpha
                    self.alpha_driver_conn.send(["ask_control"]) # se pregunta por el control
                    data = self.alpha_driver_conn.recv()[0] # se espera la respuesta
                    self.end_pipe.send([data]) # se reenvia la respuesta a la interfaz
                elif message[0] == "alpha_driver.change_control": # instruccion de actualizar el controlador del driver alpha
                    self.alpha_driver_conn.send(["change_control", message[1]]) # se reenvia el mensaje al driver alpha
                elif message[0] == "change_flute_pos": # instruccion de actualizar la posicion en la que se encuentra el bisel de la flauta
                    DATA['flute_position']['X_F'] = message[1]['X_F'] # se actualiza su ubicacion
                    DATA['flute_position']['Z_F'] = message[1]['Z_F']
                elif message[0] == "microphone.change_frequency_detection": # instruccion de cambiar los parametros de la deteccion de pitch
                    self.mic_conn.send(["change_frequency_detection", message[1]]) # se reenvia el mensaje al objeto que se encarga del microfono
                elif message[0] == "pivot": # instruccion de realizar un movimiento tipo pivote
                    # esta funcion en verdad funciona muy parecido a una isntruccion load_routes seguida de una start_loaded_script, solo que no hace cambios en los dedos ni en el flujo.
                    # recibe tres rutas, para el eje x, el eje z y el eje alpha
                    route_x = message[1]
                    route_z = message[2]
                    route_a = message[3]
                    move_t0 = time.perf_counter() - self.t0 + 0.2 # asignamos un tiempo de inicio dentro de 0.2 segundos
                    for i in range(len(route_x)):
                        route_x[i][0] += move_t0 # desfasamos el tiempo en las rutas para que calce con el tiempo actual
                        route_z[i][0] += move_t0
                        route_a[i][0] += move_t0
                    # finalmente se envian las instrucciones a cada uno de los ejes virtuales
                    if self.x_connect:
                        self.x_virtual_axis_conn.send(["merge_ref", route_x])
                    if self.z_connect:
                        self.z_virtual_axis_conn.send(["merge_ref", route_z])
                    if self.alpha_connect:
                        self.alpha_virtual_axis_conn.send(["merge_ref", route_a])
        
        # antes de salir del programa, se esperan 0.5 segundos para despejar los eventos del microfono y de la central de comunicaciones
        time.sleep(0.5)
        self.mic_running.clear()
        self.comm_event.clear()


    def set_instrument(self, instrument): # setea el instrumento que se esta usando
        self.instrument = instrument

    def move_to(self, desired_state, T=None, only_x=False, only_z=False, only_alpha=False, only_flow=False, speed=50):
        """
        Esta funcion recibe un estado final y se encarga de calcular una trayectoria en linea recta
        Normalmente la trayectoria es en linea recta en el espacio de la tarea y sigue un perfil de velocidad trapezoidal definido por el tiempo T que demora el movimiento o el parametro speed si no se especifica un tiempo T.
        En caso de que se encuentren activados only_x, only_z o only_alpha, el movimiento será en linea recta en el espacio de las junturas. En este caso el perfil de velocidad tambien sera trapezoidal.
        Si el movimiento es solo de flujo, logicamente el robot no se mueve y el cambio de flujo es con forma de escalon
        """
        if only_x: # si queremos hacer un movimiento exclusivamente en el eje x
            x_now = self.x_driver.encoder_position.value # posicion actual del motor leida en el encoder
            x_ref = x_mm_to_units(desired_state.x) # posicion a la que se quiere llegar
            temps, x_points, accel = get_1D_route(x_now, x_ref, speed, acc=4000, dec=4000) # creamos una trayectoria. Como es un solo eje la hacemos de una dimension
            move_t0 = time.perf_counter() - self.t0
            r = []
            for i in range(len(temps)):
                r.append((temps[i] + move_t0, x_points[i], accel[i])) # desplazamos el eje del tiempo para que calce con el tiempo actual
            self.x_virtual_axis_conn.send(["merge_ref", r]) # y la mandamos al eje virtual de x
            return 0
        if only_z: # si queremos hacer un movimiento exclusivamente en el eje z
            z_now = self.z_driver.encoder_position.value # posicion actual del motor leida en el encoder
            z_ref = z_mm_to_units(desired_state.z) # posicion a la que se quiere llegar
            temps, z_points, accel = get_1D_route(z_now, z_ref, speed, acc=4000, dec=4000) # creamos una trayectoria. Como es un solo eje la hacemos de una dimension
            move_t0 = time.perf_counter() - self.t0
            r = []
            for i in range(len(temps)):
                r.append((temps[i] + move_t0, z_points[i], accel[i])) # desplazamos el eje del tiempo para que calce con el tiempo actual
            self.z_virtual_axis_conn.send(["merge_ref", r]) # y la mandamos al eje virtual de z
            return 0
        if only_alpha: # si queremos hacer un movimiento exclusivamente en el eje alpha
            alpha_now = self.alpha_driver.encoder_position.value # posicion actual del motor leida en el encoder
            alpha_ref = alpha_angle_to_units(desired_state.alpha) # posicion a la que se quiere llegar
            temps, alpha_points, accel = get_1D_route(alpha_now, alpha_ref, speed, acc=4000, dec=4000) # creamos una trayectoria. Como es un solo eje la hacemos de una dimension
            move_t0 = time.perf_counter() - self.t0
            r = []
            for i in range(len(temps)):
                r.append((temps[i] + move_t0, alpha_points[i], accel[i])) # desplazamos el eje del tiempo para que calce con el tiempo actual
            self.alpha_virtual_axis_conn.send(["merge_ref", r]) # y la mandamos al eje virtual de alpha
            return 0

        ## si el movimiento no es exclusivamente en x, z o alpha calculamos una trayectoria en linea recta en el espacio de la tarea
        my_state = State(0, 0, 0, 0) # cramos un estado donde pondremos el estado actual del robot (de acuerdo a los encoders)
        my_state.x = encoder_units_to_mm(self.x_driver.encoder_position.value)
        my_state.z = encoder_units_to_mm(self.z_driver.encoder_position.value)
        my_state.alpha = encoder_units_to_angle(self.alpha_driver.encoder_position.value) 
        my_state.flow = self.flow_driver.mass_flow_reading.value
        route = get_route(my_state, desired_state, T=T, acc=50, dec=50, speed=speed/50) # y calculamos la trayectoria entre ambos estados

        ## ahora formateamos la trayectoria y desplazamos el eje del tiempo para que calce con el tiempo actual
        move_t0 = time.perf_counter() - self.t0
        route_x = []
        route_z = []
        route_alpha = []
        route_flow = []
        for i in range(len(route['t'])):
            route_x.append((route['t'][i] + move_t0, route['x'][i], route['x_vel'][i]))
            route_z.append((route['t'][i] + move_t0, route['z'][i], route['z_vel'][i]))
            route_alpha.append((route['t'][i] + move_t0, route['alpha'][i], route['alpha_vel'][i]))
            route_flow.append((route['t'][i] + move_t0, round(route['flow'][i], 2)))
        ## finalmente le enviamos las trayectorias a cada uno de los ejes virtuales
        if self.x_connect and not only_z and not only_alpha and not only_flow:
            self.x_virtual_axis_conn.send(["merge_ref", route_x])
        if self.z_connect and not only_x and not only_alpha and not only_flow:
            self.z_virtual_axis_conn.send(["merge_ref", route_z])
        if self.alpha_connect and not only_x and not only_z and not only_flow:
            self.alpha_virtual_axis_conn.send(["merge_ref", route_alpha])
        if self.flow_connect:
            self.virtual_flow_conn.send(["merge_ref", route_flow, desired_state.vibrato_amp, desired_state.vibrato_freq])
        if self.fingers_connect:
            self.virtual_fingers_conn.send(["merge_lips_surface_ref", [(0, desired_state.lips)]])
            self.virtual_fingers_conn.send(["merge_tongue_ref", [(0, desired_state.tongue)]])
            
        return route['t'][-1] # retornamos el tiempo que dura el movimiento

    def start_loaded_script(self):
        """
        esta funcion se encarga de comenzar la ejecucion de una trayectoria previamente cargada.
        Si el robot no se encuentra dentro de un rango del punto de inicio de la trayectoria, no se ejecuta y la funcion retorna
        """
        ## primero revisamos si estamos dentro de un rango del punto de inicio
        # Tolerancia: 250 steps ≈ 0.5mm para X/Z (500 steps/mm), consistente con GUI
        POSITION_TOLERANCE = 250
        start_already = True
        if self.x_connect:
            x = self.x_driver.encoder_position.value
            x_target = self.loaded_route_x[0][1]
            x_err = abs(x - x_target)
            if x_err > POSITION_TOLERANCE:
                logger.warning(f'start_loaded_script: X no en posición (actual={x}, target={x_target}, error={x_err} steps / {x_err/500:.2f}mm)')
                start_already = False
        if self.z_connect:
            z = self.z_driver.encoder_position.value
            z_target = self.loaded_route_z[0][1]
            z_err = abs(z - z_target)
            if z_err > POSITION_TOLERANCE:
                logger.warning(f'start_loaded_script: Z no en posición (actual={z}, target={z_target}, error={z_err} steps / {z_err/500:.2f}mm)')
                start_already = False
        if self.alpha_connect:
            alpha = self.alpha_driver.encoder_position.value
            alpha_target = self.loaded_route_alpha[0][1]
            alpha_err = abs(alpha - alpha_target)
            if alpha_err > POSITION_TOLERANCE:
                logger.warning(f'start_loaded_script: Alpha no en posición (actual={alpha}, target={alpha_target}, error={alpha_err} steps)')
                start_already = False

        if not start_already:
            logger.error('start_loaded_script: Robot no está en posición de inicio. Abortando.')
            return
        
        # en caso contrario, se ejecuta
        t_start = time.perf_counter() - self.t0 + 0.2 # +200ms en el futuro para que todos los ejes reciban su ruta antes de que comience la ejecución

        for i in range(len(self.loaded_route_flow)): # y desplazamos el eje del tiempo de las trayectorias pre cargadas
            self.loaded_route_x[i][0] += t_start
            self.loaded_route_z[i][0] += t_start
            self.loaded_route_alpha[i][0] += t_start
            self.loaded_route_flow[i][0] += t_start
            self.loaded_route_notes[i][0] += t_start
            self.loaded_route_aperture[i][0] += t_start
            self.loaded_route_tongue[i][0] += t_start
        
        # finalmente enviamos las trayectorias a cada uno de los dispositivos
        if self.x_connect:
            self.x_virtual_axis_conn.send(["merge_ref", self.loaded_route_x])
        if self.z_connect:
            self.z_virtual_axis_conn.send(["merge_ref", self.loaded_route_z])
        if self.alpha_connect:
            self.alpha_virtual_axis_conn.send(["merge_ref", self.loaded_route_alpha])
        if self.flow_connect:
            self.virtual_flow_conn.send(["merge_ref", self.loaded_route_flow, 0, 0])
        if self.fingers_connect:
            self.virtual_fingers_conn.send(["merge_ref", self.loaded_route_notes])
            self.virtual_fingers_conn.send(["merge_lips_surface_ref", self.loaded_route_aperture])
            self.virtual_fingers_conn.send(["merge_tongue_ref", self.loaded_route_tongue])

    def stop(self): # detiene cada uno de los dispositivos (soft stop via ejes virtuales)
        self.x_virtual_axis_conn.send(["stop"])
        self.z_virtual_axis_conn.send(["stop"])
        self.alpha_virtual_axis_conn.send(["stop"])
        self.virtual_flow_conn.send(["stop"])
        self.virtual_fingers_conn.send(["stop"])

    def emergency_stop(self):
        """
        Parada de emergencia: detiene inmediatamente todos los motores y lleva el flujo a cero.
        Usa immediate_stop en los drivers AMCI (mas rapido que el soft stop) y envia flujo 0 al Alicat.
        """
        logger.critical("EMERGENCY STOP activado — deteniendo todos los dispositivos")
        # Soft stop de ejes virtuales para detener la generacion de trayectoria
        try:
            self.x_virtual_axis_conn.send(["stop"])
            self.z_virtual_axis_conn.send(["stop"])
            self.alpha_virtual_axis_conn.send(["stop"])
            self.virtual_flow_conn.send(["stop"])
            self.virtual_fingers_conn.send(["stop"])
        except Exception as e:
            logger.error(f"Error al detener ejes virtuales en emergency_stop: {e}")
        # Enviar immediate_stop directamente a los drivers AMCI via sus pipes
        try:
            self.x_driver_conn.send(["emergency_stop"])
            self.z_driver_conn.send(["emergency_stop"])
            self.alpha_driver_conn.send(["emergency_stop"])
        except Exception as e:
            logger.error(f"Error al enviar emergency_stop a drivers: {e}")

    def get_instrument(self): # retorna el instrumento seleccionado
        return self.instrument


class Memory(Process):
    """
    Esta clase se encarga de almacenar la historia de las variables medidas. windowWidth dice la cantidad de datos a almacenar e interval el tiempo (en milisegundos) para obtener una muestra.
    """
    def __init__(self, running, x_driver, z_driver, alpha_driver, flow_controller, pressure_sensor, microphone, fingers_driver, pipe_end, data, windowWidth=200, interval=0.05):
        Process.__init__(self) # Initialize the threading superclass
        self.saving = False
        self.x_driver = x_driver
        self.z_driver = z_driver
        self.alpha_driver = alpha_driver
        self.flow_controller = flow_controller
        self.pressure_sensor = pressure_sensor
        self.microphone = microphone
        self.fingers_driver = fingers_driver
        self.pipe_end = pipe_end # pipe que conecta con el musico
        self.windowWidth = windowWidth

        # creamos dos estados, el real y el de referencia
        self.ref_state = State(0,0,0,0)
        self.real_state = State(0,0,0,0)

        # el estado referencia sigue los ejes virtuales
        self.ref_state.x = x_units_to_mm(self.x_driver.pos_ref.value)
        self.ref_state.z = z_units_to_mm(self.z_driver.pos_ref.value)
        self.ref_state.alpha = alpha_units_to_angle(self.alpha_driver.pos_ref.value)
        self.ref_state.flow = self.flow_controller.mass_flow_set_point_reading.value
        # el estado real refleja lo medido en los encoders
        self.real_state.x = x_units_to_mm(self.x_driver.encoder_position.value)
        self.real_state.z = z_units_to_mm(self.z_driver.encoder_position.value)
        self.real_state.alpha = alpha_units_to_angle(self.alpha_driver.encoder_position.value)
        self.real_state.flow = self.flow_controller.mass_flow_reading.value

        self.data = data # data compartida entre los procesos. 
        # Esta clase se encarga de actualizar las siguientes listas en data:
        self.data['flow_ref'] = linspace(0,0,200) # donde almacenamos los ultimos 200 valores (midiendo cada 0.01s)
        self.data['x_ref'] = linspace(0,0,200) # un arreglo de 200 ceros
        self.data['z_ref'] = linspace(0,0,200)
        self.data['alpha_ref'] = linspace(0,0,200)
        self.data['x'] = linspace(0,0,200)
        self.data['z'] = linspace(0,0,200)
        self.data['alpha'] = linspace(0,0,200)
        self.data['radius'] = linspace(0,0,200)
        self.data['theta'] = linspace(0,0,200)
        self.data['offset'] = linspace(0,0,200)
        self.data['radius_ref'] = linspace(0,0,200)
        self.data['theta_ref'] = linspace(0,0,200)
        self.data['offset_ref'] = linspace(0,0,200)
        self.data['mouth_pressure'] = linspace(0,0,200)
        self.data['volume_flow'] = linspace(0,0,200)
        self.data['mass_flow'] = linspace(0,0,200)
        self.data['temperature'] = linspace(0,0,200)
        self.data['frequency'] = linspace(0,0,200)
        self.data['tongue'] = linspace(0,0,200)
        self.data['lips'] = linspace(0,0,200)
        self.data['notes'] = ['D3' for i in range(200)]
        self.data['times'] = linspace(0,0,200)
        
        self.t0 = time.time()
        self.first_entry = False
        self.t1 = 0

        # tambien creamos un data frame para cuando queramos grabar. Este tiene las columnas de todas las variables medidas
        self.data_frame = pd.DataFrame(columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref', 'tongue', 'lips'])

        self.interval = interval
        self.running = running

    def run(self):
        self.pipe_end.send(["memory_started"]) # se avisa al musico que la memoria ya esta operativa
        while self.running.is_set():
            # en cada iteracion actualizamos los valores del estado referencia y el estado real leyendo los encoders y los ejes virtuales
            #print(self.fingers_driver.note_played.value.decode('utf-8').strip(), end="")
            self.ref_state.x = x_units_to_mm(self.x_driver.pos_ref.value)
            self.ref_state.z = z_units_to_mm(self.z_driver.pos_ref.value)
            self.ref_state.alpha = alpha_units_to_angle(self.alpha_driver.pos_ref.value)
            self.ref_state.flow = self.flow_controller.mass_flow_set_point_reading.value
            self.ref_state.tongue = self.fingers_driver.tongue_played.value
            self.ref_state.lips = self.fingers_driver.lips_played.value
            self.real_state.x = encoder_units_to_mm(self.x_driver.encoder_position.value)
            self.real_state.z = z_units_to_mm(self.z_driver.encoder_position.value)
            self.real_state.alpha = alpha_units_to_angle(self.alpha_driver.encoder_position.value)
            self.real_state.flow = self.flow_controller.mass_flow_reading.value

            # luego actualizamos los arreglos, haciendo un shift a la izquierda y agregando el valor recien leido
            self.data['x_ref'] = np.hstack([self.data['x_ref'][1:], self.ref_state.x])
            self.data['z_ref'] = np.hstack([self.data['z_ref'][1:], self.ref_state.z])
            self.data['alpha_ref'] = np.hstack([self.data['alpha_ref'][1:], self.ref_state.alpha])
            self.data['flow_ref'] = np.hstack([self.data['flow_ref'][1:], self.flow_controller.mass_flow_set_point_reading.value])
            self.data['x'] = np.hstack([self.data['x'][1:], self.real_state.x])
            self.data['z'] = np.hstack([self.data['z'][1:], self.real_state.z])
            self.data['alpha'] = np.hstack([self.data['alpha'][1:], self.real_state.alpha])
            self.data['volume_flow'] = np.hstack([self.data['volume_flow'][1:], self.flow_controller.vol_flow_reading.value])

            self.data['radius'] = np.hstack([self.data['radius'][1:], self.real_state.r])
            self.data['theta'] = np.hstack([self.data['theta'][1:], self.real_state.theta])
            self.data['offset'] = np.hstack([self.data['offset'][1:], self.real_state.o])
            self.data['radius_ref'] = np.hstack([self.data['radius_ref'][1:], self.ref_state.r])
            self.data['theta_ref'] = np.hstack([self.data['theta_ref'][1:], self.ref_state.theta])
            self.data['offset_ref'] = np.hstack([self.data['offset_ref'][1:], self.ref_state.o])
            self.data['mouth_pressure'] = np.hstack([self.data['mouth_pressure'][1:], self.pressure_sensor.pressure.value])
            self.data['mass_flow'] = np.hstack([self.data['mass_flow'][1:], self.flow_controller.mass_flow_reading.value])
            self.data['temperature'] = np.hstack([self.data['temperature'][1:], self.flow_controller.temperature_reading.value])
            self.data['frequency'] = np.hstack([self.data['frequency'][1:], self.microphone.pitch.value])
            self.data['tongue'] = np.hstack([self.data['tongue'][1:], self.ref_state.tongue])
            self.data['lips'] = np.hstack([self.data['lips'][1:], self.ref_state.lips])
            self.data['notes'] = self.data['notes'][1:] + [self.fingers_driver.note_played.value.decode('utf-8').strip()]
            self.data['times'] = np.hstack([self.data['times'][1:], time.time() - self.t0])  
            if self.saving: # si esta condicion esta activa agregaremos el estado actual al dataframe como una entrada
                if self.first_entry: # si es la primera entrada del data frame, tomamos el tiempo actual y lo usamos como tiempo inicial para desfasar todas las entradas, y asi partir en cero
                    self.t1 = self.data['times'][-1]
                    self.first_entry = False
                new_data = pd.DataFrame([[self.data['times'][-1] - self.t1, self.data['frequency'][-1], self.data["temperature"][-1], self.data["mass_flow"][-1], self.data["volume_flow"][-1], self.data["mouth_pressure"][-1], self.data["offset"][-1], self.data["theta"][-1], self.data["radius"][-1], self.data["offset_ref"][-1], self.data["theta_ref"][-1], self.data["radius_ref"][-1], self.data["alpha"][-1], self.data["z"][-1], self.data["x"][-1], self.data["alpha_ref"][-1], self.data["z_ref"][-1], self.data["x_ref"][-1], self.data["flow_ref"][-1], self.data['notes'][-1], self.data["tongue"][-1], self.data['lips'][-1]]], columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref', 'note', 'tongue', 'lips']) # creamos la nueva entrada
                self.data_frame = pd.concat([self.data_frame, new_data], ignore_index=True) # y la concatenamos con lo que teniamos
            
            if self.pipe_end.poll(self.interval): # luego escuchamos si hay instrucciones del musico. Acá esperamos 0.01s
                message = self.pipe_end.recv()
                print("Message received in memory:", message[0])
                if message[0] == "start_saving": # si se quiere empezar a grabar
                    self.start_saving()
                if message[0] == "stop_recording": # si se quiere dejar de grabar
                    self.stop_recording()
                if message[0] == "save_recorded_data": # si se quiere guardar en un archivo el dataframe grabado. message[1] tiene la direccion donde se quiere guardar
                    self.save_recorded_data(message[1])
                    print("Data saved to file", message[1])
    
    def start_saving(self): # funcion para empezar a grabar datos leidos. Si habían datos grabados de antes, se formatean para empezar denuevo
        self.first_entry = True # se activa esta condicion para tomar el proximo tiempo como el primero
        self.data_frame = pd.DataFrame(columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref', 'note']) # creamos un nuevo data_frame vacio
        self.saving = True # activamos esta condicion
    
    def pause_saving(self): # para pausar la grabacion de datos (mismo efecto que stop)
        self.saving = False

    def resume_saving(self): # para reanudar una grabacion sin borrar los datos que se tenian antes
        self.saving = True
    
    def finish_saving(self, file_name): # para dejar de grabar, y al mismo tiempo guardar la informacion grabada en un archivo
        self.saving = False
        self.data_frame.to_csv(file_name)

    def save_recorded_data(self, filename1): # redundante. solo llama a finish_saving
        self.finish_saving(filename1)

    def stop_recording(self): # para pausar la grabacion de datos (mismo efecto que pause)
        self.saving = False