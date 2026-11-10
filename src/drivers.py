import sounddevice as sd
import soundfile as sf
from scipy import signal
from librosa import yin, pyin, note_to_hz
from scipy.io.wavfile import write

from multiprocessing import Process, Event, Value, Pipe, Array, Manager
import threading
from collections import deque
import src.lib.ethernet_ip.ethernetip as ethernetip
from src.motor_route import *
from src.route import *
from src.cinematica import *

import builtins
import struct
import time
import numpy as np
import io
import logging

# Configuracion del sistema de logging persistente
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
    handlers=[
        logging.FileHandler('pierre_robot.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('pierre')

# Constantes de seguridad cargadas desde settings.json
SAFETY = DATA.get('safety', {})
SAFETY_POS_LIMITS = {
    'x_min': SAFETY.get('x_min_steps', -2000),
    'x_max': SAFETY.get('x_max_steps', 100000),
    'z_min': SAFETY.get('z_min_steps', -2000),
    'z_max': SAFETY.get('z_max_steps', 100000),
    'alpha_min': SAFETY.get('alpha_min_steps', -4000),
    'alpha_max': SAFETY.get('alpha_max_steps', 4000),
}
SAFETY_MAX_VELOCITY = SAFETY.get('max_velocity_steps_s', 200000)
SAFETY_CONNECTION_TIMEOUT = SAFETY.get('connection_timeout_s', 30)
SAFETY_HOMING_TIMEOUT = SAFETY.get('homing_timeout_s', 60)
SAFETY_MOVE_TIMEOUT = SAFETY.get('move_complete_timeout_s', 30)
SAFETY_WATCHDOG_TIMEOUT = SAFETY.get('watchdog_timeout_s', 5.0)
SAFETY_MAX_PRESSURE = SAFETY.get('max_pressure_kpa', 200.0)
SAFETY_FLOW_RATE_LIMIT = SAFETY.get('flow_rate_limit_per_step', 5.0)
NETWORK_DELAY = DATA.get('network_delay', 20)


def clamp_int(value, min_value, max_value, default=0):
    try:
        value = int(round(float(value)))
    except (TypeError, ValueError, OverflowError):
        value = default
    return builtins.max(min_value, builtins.min(max_value, value))

# Parametros de homing cargados desde settings.json
HOMING = DATA.get('homing', {})
HOMING_CCW_JOG_SPEED = HOMING.get('ccw_jog_speed', 100)
HOMING_CCW_JOG_ACC = HOMING.get('ccw_jog_acceleration', 5)
HOMING_CW_JOG_SPEED = HOMING.get('cw_jog_speed', 4000)
HOMING_ALPHA_OFFSET = HOMING.get('alpha_home_offset_steps', -569)
HOMING_ALPHA_MOVE_SPEED = HOMING.get('alpha_home_move_speed', 1000)
HOMING_XZ_SAFETY_DIST = HOMING.get('xz_safety_distance_steps', -800)
HOMING_XZ_SAFETY_SPEED = HOMING.get('xz_safety_move_speed', 4000)

# INPUT_FUNCTION_BITS definen el tipo de entrada que tienen los drivers AMCI
INPUT_FUNCTION_BITS = {'General Purpose Input': 0, 'CW Limit': 1, 'CCW Limit': 2, 'Start Index Move': 3, 'Capture Encoder Value': 3, 'Stop Jog': 4, 'Stop Registration Move': 4, 'Emergency Stop': 5, 'Home': 6}

class Command:
    """
    Esta clase se usa para preparar mensajes de tipo comando para los drivers AMCI.
    Normalmente se llama este objeto y se activan ciertos atributos. 
    Luego se transforma al formato que los drivers lo entienden usando los metodos get_bytes_to_send en caso de un mensaje explicito y get_list_to_send en caso de un mensaje implicito
    """
    def __init__(self, preset_encoder=0, run_assembled_move=0, program_assembled=0, read_assembled_data=0, reset_errors=0, preset_motor_position=0, jog_ccw=0, jog_cw=0, find_home_ccw=0, find_home_cw=0, immediate_stop=0, resume_move=0, hold_move=0, relative_move=0, absolute_move=0, enable_driver=1, virtual_encoder_follower=0, general_purpose_output_state=0, virtual_position_follower=0, backplane_proximity_bit=0, clear_driver_fault=0, assembled_move_type=0, indexed_command=0, registration_move=0, enable_electronic_gearing_mode=0, save_assembled_move=0, reverse_blend_direction=0, hybrid_control_enable=0, encoder_registration_move=0, current_key=0, desired_command_word_2=0, desired_command_word_3=0, desired_command_word_4=0, desired_command_word_5=0, desired_command_word_6=0, desired_command_word_7=0, desired_command_word_8=0, desired_command_word_9=0, name=''):
        self.desired_mode_select_bit = 0
        self.desired_preset_encoder = preset_encoder
        self.desired_run_assembled_move = run_assembled_move
        self.desired_program_assembled = program_assembled
        self.desired_read_assembled_data = read_assembled_data
        self.desired_reset_errors = reset_errors
        self.desired_preset_motor_position = preset_motor_position
        self.desired_jog_ccw = jog_ccw
        self.desired_jog_cw = jog_cw
        self.desired_find_home_ccw = find_home_ccw
        self.desired_find_home_cw = find_home_cw
        self.desired_immediate_stop = immediate_stop
        self.desired_resume_move = resume_move
        self.desired_hold_move = hold_move
        self.desired_relative_move = relative_move
        self.desired_absolute_move = absolute_move

        self.desired_enable_driver = enable_driver
        self.desired_virtual_encoder_follower = virtual_encoder_follower
        self.desired_general_purpose_output_state = general_purpose_output_state
        self.desired_virtual_position_follower = virtual_position_follower
        self.desired_backplane_proximity_bit = backplane_proximity_bit
        self.desired_clear_driver_fault = clear_driver_fault
        self.desired_assembled_move_type = assembled_move_type
        self.desired_indexed_command = indexed_command
        self.desired_registration_move = registration_move
        self.desired_enable_electronic_gearing_mode = enable_electronic_gearing_mode
        self.desired_save_assembled_move = save_assembled_move
        self.desired_reverse_blend_direction = reverse_blend_direction
        self.desired_hybrid_control_enable = hybrid_control_enable
        self.desired_encoder_registration_move = encoder_registration_move
        self.desired_current_key = current_key

        self.desired_command_word_2 = desired_command_word_2        
        self.desired_command_word_3 = desired_command_word_3        
        self.desired_command_word_4 = desired_command_word_4
        self.desired_command_word_5 = desired_command_word_5
        self.desired_command_word_6 = desired_command_word_6
        self.desired_command_word_7 = desired_command_word_7
        self.desired_command_word_8 = desired_command_word_8
        self.desired_command_word_9 = desired_command_word_9

        self.name = name
    
    def get_ints_to_send(self):
        word0 = f'{self.desired_mode_select_bit}{self.desired_preset_encoder}{self.desired_run_assembled_move}{self.desired_read_assembled_data}{self.desired_program_assembled}{self.desired_reset_errors}{self.desired_preset_motor_position}{self.desired_jog_ccw}{self.desired_jog_cw}{self.desired_find_home_ccw}{self.desired_find_home_cw}{self.desired_immediate_stop}{self.desired_resume_move}{self.desired_hold_move}{self.desired_relative_move}{self.desired_absolute_move}'
        word0 = int(word0, 2)
        if word0 >= 2**15:
            word0 -= 2**16

        word1 = f'{self.desired_enable_driver}{self.desired_virtual_encoder_follower}{self.desired_general_purpose_output_state}{self.desired_virtual_position_follower}{self.desired_backplane_proximity_bit}{self.desired_clear_driver_fault}{self.desired_assembled_move_type}{self.desired_indexed_command}{self.desired_registration_move}{self.desired_enable_electronic_gearing_mode}{self.desired_save_assembled_move}{self.desired_reverse_blend_direction}{self.desired_hybrid_control_enable}{self.desired_encoder_registration_move}' + format(self.desired_current_key, 'b').zfill(2)
        word1 = int(word1, 2)
        if word1 >= 2**15:
            word1 -= 2**16

        return [word0, word1, self.desired_command_word_2, self.desired_command_word_3, self.desired_command_word_4, self.desired_command_word_5, self.desired_command_word_6, self.desired_command_word_7, self.desired_command_word_8, self.desired_command_word_9]

    def get_bytes_to_send(self):
        ints_to_send = self.get_ints_to_send()
        bytes_to_send  = b''
        for i in ints_to_send:
            bytes_to_send += struct.pack("h", i)
        return bytes_to_send

    def get_list_to_send(self):
        ## entrega el comando en el formato en que se puede copiar a la salida de la librería que se usa para la comunicación.
        bytes_to_send = self.get_bytes_to_send()
        bits_to_send = ''.join(format(byte, '08b')[::-1] for byte in bytes_to_send)
        as_list = [i == '1' for i in bits_to_send]
        return as_list

class Setting:
    """
    Esta clase se usa para preparar mensajes de tipo configuración para los drivers AMCI.
    Normalmente se llama este objeto y se activan ciertos atributos. 
    Luego se transforma al formato que los drivers lo entienden usando los metodos get_bytes_to_send en caso de un mensaje explicito y get_list_to_send en caso de un mensaje implicito
    """
    def __init__(self, disable_anti_resonance_bit=0, enable_stall_detection_bit=1, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=1, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=50, motors_step_turn=1000, hybrid_control_gain=0, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=30, current_loop_gain=5):
        
        self.desired_mode_select_bit = 1
        self.desired_disable_anti_resonance_bit = disable_anti_resonance_bit
        self.desired_enable_stall_detection_bit = enable_stall_detection_bit
        self.desired_use_backplane_proximity_bit = use_backplane_proximity_bit
        self.desired_use_encoder_bit = use_encoder_bit
        self.desired_home_to_encoder_z_pulse = home_to_encoder_z_pulse
        self.desired_input_3_function_bits = input_3_function_bits
        self.desired_input_2_function_bits = input_2_function_bits
        self.desired_input_1_function_bits = input_1_function_bits

        self.desired_output_functionality_bit = output_functionality_bit
        self.desired_output_state_control_on_network_lost = output_state_control_on_network_lost
        self.desired_output_state_on_network_lost = output_state_on_network_lost
        self.desired_read_present_configuration = read_present_configuration
        self.desired_save_configuration = save_configuration
        self.desired_binary_input_format = binary_input_format
        self.desired_binary_output_format = binary_output_format
        self.desired_binary_endian = binary_endian
        self.desired_input_3_active_level = input_3_active_level
        self.desired_input_2_active_level = input_2_active_level
        self.desired_input_1_active_level = input_1_active_level        

        self.desired_starting_speed = starting_speed
        self.desired_motors_step_turn = motors_step_turn
        self.desired_hybrid_control_gain = hybrid_control_gain
        self.desired_encoder_pulses_turn = encoder_pulses_turn
        self.desired_idle_current_percentage = idle_current_percentage
        self.desired_motor_current = motor_current
        self.desired_current_loop_gain = current_loop_gain

        self.name = 'Settings configuration'

    def get_ints_to_send(self):
        word0 = f'{self.desired_mode_select_bit}{self.desired_disable_anti_resonance_bit}{self.desired_enable_stall_detection_bit}0{self.desired_use_backplane_proximity_bit}{self.desired_use_encoder_bit}{self.desired_home_to_encoder_z_pulse}' + format(self.desired_input_3_function_bits, 'b').zfill(3) + format(self.desired_input_2_function_bits, 'b').zfill(3) + format(self.desired_input_1_function_bits, 'b').zfill(3)
        word0 = int(word0, 2)
        if word0 >= 2**15:
            word0 -= 2**16

        word1 = f'0{self.desired_output_functionality_bit}{self.desired_output_state_control_on_network_lost}{self.desired_output_state_on_network_lost}{self.desired_read_present_configuration}{self.desired_save_configuration}{self.desired_binary_input_format}{self.desired_binary_output_format}{self.desired_binary_endian}0000{self.desired_input_3_active_level}{self.desired_input_2_active_level}{self.desired_input_1_active_level}'
        word1 = int(word1, 2)
        if word1 >= 2**15:
            word1 -= 2**16

        return [word0, word1, self.desired_starting_speed//1000, self.desired_starting_speed%1000, self.desired_motors_step_turn, self.desired_hybrid_control_gain, self.desired_encoder_pulses_turn, self.desired_idle_current_percentage, self.desired_motor_current, self.desired_current_loop_gain]

    def get_bytes_to_send(self):
        ints_to_send = self.get_ints_to_send()
        byte_to_send = b''
        for i in ints_to_send:
            byte_to_send += struct.pack("h", i)
        return byte_to_send

    def get_list_to_send(self):
        ## entrega la configuración en el formato en que se puede copiar a la salida de la librería que se usa para la comunicación.
        bytes_to_send = self.get_bytes_to_send()
        bits_to_send = ''.join(format(byte, '08b')[::-1] for byte in bytes_to_send)
        as_list = [i == '1' for i in bits_to_send]
        return as_list

class VirtualAxis(threading.Thread):
    """
    Esta clase se usa para simular un eje virtual.
    Es instanciada dentro de uno de los drivers de un motor, y queda corriendo como un thread, el que actualiza e informa la posición y la velocidad deseada para cada eje en tiempo real.
    """
    def __init__(self, running, interval, t0, pipe_conn, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.running = running
        self.ref = deque([(0,0,0)]) # la ruta es un deque donde cada elemento es de la forma (tiempo, posicion, velocidad)
        self.last_pos = 0
        self.interval = interval
        self.t0 = t0
        self.pipe_conn = pipe_conn
        self.verbose = verbose
        self.pos = 0
        self.vel = 0
        
    def run(self):
        while self.running.is_set(): # corre durante toda la ejecución principal
            t = time.perf_counter() - self.t0
            self.pos, self.vel = self.get_ref(t) # de acuerdo al tiempo actual actualiza la posicion y velocidad a partir de la ruta
            if self.verbose:
                print(t, self.pos, self.vel)
            self.update_ref(t)
            try:
                if self.pipe_conn.poll(self.interval): # escucha si hay alguna instruccion
                    message = self.pipe_conn.recv()
                    if message[0] == "get_ref":
                        pos, vel = self.get_ref(message[1])
                    elif message[0] == "update_ref":
                        self.update_ref(message[1])
                    elif message[0] == "merge_ref":
                        self.merge_ref(message[1])
                    elif message[0] == "stop":
                        self.stop()
            except (EOFError, BrokenPipeError, ConnectionResetError, OSError):
                self.stop()  # pipe roto → frenar eje
                break

    def get_ref(self, t): # actualiza la posición y velocidad para el tiempo actual de acuerdo a la ruta referencia
        if self.ref[-1][0] > t: # si la ruta esta definida hasta un t_final > t_actual
            pos, vel = get_value_from_func_2d(t, self.ref) # busca el elemento de la ruta cuyo tiempo se acerque mas al actual
        else:
            pos = self.ref[-1][1] # si t_actual > t_final de la ruta referencia la posicion se mantiene y la velocidad se hace 0
            vel = 0
        return int(pos), int(vel)

    def update_ref(self, t): # actualiza la ruta borrando todos los elementos con t < t_actual (dejando al menos un elemento en el deque)
        while self.ref[0][0] < t and len(self.ref) > 1:
            self.ref.popleft()

    def merge_ref(self, new_ref): # agrega o reemplaza elementos a la ruta
        t_change = new_ref[0][0]
        if self.ref[-1][0] < t_change: # si el t del ultimo elemento de la ruta anterior es menor al primero de la ruta que se quiere agregar, simplemente se agrega
            self.ref.extend(new_ref)
        else: # si hay solapado entre las rutas, se reemplaza la anterior por la nueva.
            i = 0
            while self.ref[i][0] < t_change:
                i += 1
            for _ in range(i, len(self.ref)):
                self.ref.pop()
            self.ref.extend(new_ref)

    def stop(self): # para frenar rapidamente el motor (con un soft stop) se detiene el eje virtual borrando toda la ruta cargada, dejando la posicion actual y la velocidad en 0
        self.ref = deque([(0,self.pos,0)])

class AMCIDriver(Process):
    """
    Esta clase interactua con los drivers para los motores AMCI. Define los mensajes a enviar e interpreta la informacion leida desde los mismos.
    Al instanciar esta clase se pueden definir parámetros de su configuración. Para tener información de que significa cada parámetro referirse al manual de los drivers.
    """
    def __init__(self, hostname, running, musician_pipe, comm_pipe, comm_data, virtual_axis_pipe, t0, connected=True, disable_anti_resonance_bit=0, enable_stall_detection_bit=0, use_backplane_proximity_bit=0, use_encoder_bit=0, home_to_encoder_z_pulse=0, input_3_function_bits=0, input_2_function_bits=0, input_1_function_bits=0, output_functionality_bit=0, output_state_control_on_network_lost=0, output_state_on_network_lost=0, read_present_configuration=0, save_configuration=0, binary_input_format=0, binary_output_format=0, binary_endian=0, input_3_active_level=0, input_2_active_level=0, input_1_active_level=0, starting_speed=1, motors_step_turn=1000, hybrid_control_gain=1, encoder_pulses_turn=1000, idle_current_percentage=30, motor_current=40, current_loop_gain=5, homing_slow_speed=200, verbose=False, virtual_axis_follow_acceleration=50, virtual_axis_follow_deceleration=50, home=True, virtual_axis_proportional_coef=1, Kp=0, Ki=5, Kd=0.01, Kp_vel=0, Ki_vel=0, Kd_vel=0):
        Process.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.virtual_axis = None
        self.musician_pipe = musician_pipe
        self.comm_pipe = comm_pipe
        self.comm_data = comm_data
        self.virtual_axis_pipe = virtual_axis_pipe
        self.t0 = t0
        self.connected = connected
        self.acc = virtual_axis_follow_acceleration
        self.dec = virtual_axis_follow_deceleration
        self.virtual_axis_proportional_coef = virtual_axis_proportional_coef
        self.home = home
        self.forced_break = False
        self.motor_current = motor_current
        self.verbose = verbose
        self.init_params()
        self.fast_ccw_limit_homing = False
        self.slow_ccw_limit_homing = False
        self.fast_cw_limit_homing = False
        self.slow_cw_limit_homing = False

        # configuracion inicial a mandar al dispositivo
        self.initial_settings = Setting(disable_anti_resonance_bit, enable_stall_detection_bit, use_backplane_proximity_bit, use_encoder_bit, home_to_encoder_z_pulse, input_3_function_bits, input_2_function_bits, input_1_function_bits, output_functionality_bit, output_state_control_on_network_lost, output_state_on_network_lost, read_present_configuration, save_configuration, binary_input_format, binary_output_format, binary_endian, input_3_active_level, input_2_active_level, input_1_active_level, starting_speed, motors_step_turn, hybrid_control_gain, encoder_pulses_turn, idle_current_percentage, motor_current, current_loop_gain)

        # el intervalo para cada iteración
        self.Ts = 0.01

        # parametros del control. En cada iteración compara la posición y velocidad leidas por los encoders con las de referencia que entrega el eje virtual
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.e0 = 0
        self.MV_I0 = 0
        self.MV = 0
        self.Ka = 1.0  # ganancia anti-windup (>0 para activar)
        self._pos_clamp_logged = False  # throttle para log de clamp

        # Limites de posicion para anti-windup y proteccion mecanica
        # Se determinan segun el eje usando el hostname para distinguir
        self.pos_min = SAFETY_POS_LIMITS.get('x_min', -2000)  # se actualizan en run() segun eje
        self.pos_max = SAFETY_POS_LIMITS.get('x_max', 100000)
        self.MV_low = self.pos_min
        self.MV_high = self.pos_max

        self.Kp_vel = Kp_vel
        self.Ki_vel = Ki_vel
        self.Kd_vel = Kd_vel #0.00005
        self.e0_vel = 0
        self.MV_I0_vel = 0
        self.MV_vel = 0
        self.Ka_vel = 1.0  # ganancia anti-windup velocidad
        self.MV_low_vel = -SAFETY_MAX_VELOCITY
        self.MV_high_vel = SAFETY_MAX_VELOCITY
        self.last_pos = 0
        self.last_watchdog_time = time.perf_counter()  # watchdog: ultimo heartbeat recibido del musician
        
        if self.connected:
            # si el eje esta conectado, se envia el mensaje a la central de comunicacion para que establezca una conexión con el dispositivo
            self.comm_pipe.send(["explicit_conn", self.hostname, 20*8, 20*8, 20, 20, ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, 100, ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, 150])
    
    def init_params(self): # inicializa algunos parametros del estado del dispositivo
        self.mode_select_bit = 0
        
        self.disable_anti_resonance_bit = 0
        self.enable_stall_detection_bit = 0
        self.use_backplane_proximity_bit = 0
        self.use_encoder_bit = 0
        self.home_to_encoder_z_pulse = 0
        self.input_3_function_bits = 0
        self.input_2_function_bits = 0
        self.input_1_function_bits = 0

        self.output_functionality_bit = 0
        self.output_state_control_on_network_lost = 0
        self.output_state_on_network_lost = 0
        self.read_present_configuration = 0
        self.save_configuration = 0
        self.binary_input_format = 0
        self.binary_output_format = 0
        self.binary_endian = 0
        self.input_3_active_level = 0
        self.input_2_active_level = 0
        self.input_1_active_level = 0

        self.starting_speed = 0
        self.motors_step_turn = 0
        self.hybrid_control_gain = 0
        self.encoder_pulses_turn = 0
        self.idle_current_percentage = 0
        #self.motor_current = 0
        self.current_loop_gain = 0


        self.module_ok = 0
        self.configuration_error = 0
        self.command_error = 0
        self.input_error = 0
        self.position_invalid = 0
        self.waiting_for_assembled_segment = 0
        self.in_assembled_mode = 0
        self.move_complete = 0
        self.decelerating = 0
        self.accelerating = 0
        self.at_home = 0
        self.stopped = 0
        self.in_hold_state = 0
        self.moving_ccw = 0
        self.moving_cw = 0

        self.driver_is_enabled = 0
        self.stall_detected = 0
        self.output_state = 0
        self.heartbeat_bit = 0
        self.limit_condition = 0
        self.invalid_jog_change = 0
        self.motion_lag = 0
        self.driver_fault = 0
        self.connection_was_lost = 0
        self.plc_in_prog_mode = 0
        self.temperature_above_90 = 0
        self.in_3_active = 0
        self.in_2_active = 0
        self.in_1_active = 0

        self.motor_position = Value("i", 0)
        self.encoder_position = Value("i", 0)
        self.pos_ref = Value("i", 0)
        self.captured_encoder_position = 0
        self.programed_motor_current = 0
        self.acceleration_jerk = 0


        self.desired_mode_select_bit = 0
        self.desired_disable_anti_resonance_bit = 0
        self.desired_enable_stall_detection_bit = 0
        self.desired_use_backplane_proximity_bit = 0
        self.desired_use_encoder_bit = 0
        self.desired_home_to_encoder_z_pulse = 0
        self.desired_input_3_function_bits = 0
        self.desired_input_2_function_bits = 0
        self.desired_input_1_function_bits = 0

        self.desired_output_functionality_bit = 0
        self.desired_output_state_control_on_network_lost = 0
        self.desired_output_state_on_network_lost = 0
        self.desired_read_present_configuration = 0
        self.desired_save_configuration = 0
        self.desired_binary_input_format = 0
        self.desired_binary_output_format = 0
        self.desired_binary_endian = 0
        self.desired_input_3_active_level = 0
        self.desired_input_2_active_level = 0
        self.desired_input_1_active_level = 0

        self.desired_starting_speed = 0
        self.desired_motors_step_turn = 0
        self.desired_hybrid_control_gain = 0
        self.desired_encoder_pulses_turn = 0
        self.desired_idle_current_percentage = 0
        self.desired_motor_current = 0
        self.desired_current_loop_gain = 0


        self.desired_preset_encoder = 0
        self.desired_run_assembled_move = 0
        self.desired_program_assembled = 0
        self.desired_read_assembled_data = 0
        self.desired_reset_errors = 0
        self.desired_preset_motor_position = 0
        self.desired_jog_ccw = 0
        self.desired_jog_cw = 0
        self.desired_find_home_ccw = 0
        self.desired_find_home_cw = 0
        self.desired_immediate_stop = 0
        self.desired_resume_move = 0
        self.desired_hold_move = 0
        self.desired_relative_move = 0
        self.desired_absolute_move = 0

        self.desired_enable_driver = 0
        self.desired_virtual_encoder_follower = 0
        self.desired_general_purpose_output_state = 0
        self.desired_virtual_position_follower = 0
        self.desired_backplane_proximity_bit = 0
        self.desired_clear_driver_fault = 0
        self.desired_assembled_move_type = 0
        self.desired_indexed_command = 0
        self.desired_registration_move = 0
        self.desired_enable_electronic_gearing_mode = 0
        self.desired_save_assembled_move = 0
        self.desired_reverse_blend_direction = 0
        self.desired_hybrid_control_enable = 0
        self.desired_encoder_registration_move = 0
        self.desired_current_key = 0

        self.desired_command_word_2 = 0
        self.desired_command_word_3 = 0
        self.desired_command_word_4 = 0
        self.desired_command_word_5 = 0
        self.desired_command_word_6 = 0
        self.desired_command_word_7 = 0
        self.desired_command_word_8 = 0
        self.desired_command_word_9 = 0

    def run(self): # ejecución
        logger.info(f"Running AMCI driver {self.hostname}...")
        # Determinar limites de posicion segun eje (basado en la configuracion de input_2)
        if self.initial_settings.desired_input_2_function_bits == INPUT_FUNCTION_BITS['CCW Limit']:
            # Eje Alpha
            self.pos_min = SAFETY_POS_LIMITS['alpha_min']
            self.pos_max = SAFETY_POS_LIMITS['alpha_max']
            self.axis_name = 'alpha'
        else:
            # Ejes X o Z — usamos limites genericos (X y Z tienen el mismo rango)
            self.pos_min = SAFETY_POS_LIMITS['x_min']
            self.pos_max = SAFETY_POS_LIMITS['x_max']
            self.axis_name = 'x_or_z'
        self.MV_low = self.pos_min
        self.MV_high = self.pos_max
        logger.info(f"  {self.hostname} position limits: [{self.pos_min}, {self.pos_max}]")

        ## Instanciamos e iniciamos el eje virtual
        self.virtual_axis = VirtualAxis(self.running, 0.01, self.t0, self.virtual_axis_pipe, verbose=False)
        self.virtual_axis.start()
        logger.info(f"Virtual axis started for {self.hostname}...")
        if not self.connected:
            self.musician_pipe.send(["driver_started"]) # se da aviso de que el driver esta funcionando
        else:
            # Se envia el mensaje a la central de comunicacion para que registre la sesión para iniciar la comunicación
            self.comm_pipe.send(["registerSession", self.hostname])
            time.sleep(0.1)
            
            # se envía la configuración inicial al dispositivo
            self.send_data(self.initial_settings.get_bytes_to_send())
            time.sleep(0.1)
            data = self.read_input(explicit=True) # se lee el estatus del dispositivo
            self.process_incoming_data(data) # se procesa la información obtenida
            

            return_to_command_mode = self.get_return_to_command_mode_command()
            self.send_data(return_to_command_mode.get_bytes_to_send()) # se envia el dispositivo a estado de comando
            time.sleep(0.1)

            # se obtiene la instrucción de movimiento 'synchrostep' en la que estará trabajando el driver
            synchrostep_command = self.get_synchrostep_move_command(0, 0, speed=0, acceleration=self.acc, deceleration=self.dec, proportional_coefficient=self.virtual_axis_proportional_coef, network_delay=NETWORK_DELAY, encoder=False)

            # Se envia el mensaje a la central de comunicacion para que empiece el intercambio de mensajería implicito
            self.comm_pipe.send(["sendFwdOpenReq", self.hostname, 100, 150, 110, 5, 5, ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_HIGH])

            _conn_t0 = time.perf_counter()
            while self.hostname not in " ".join(self.comm_data.keys()): # mientras no se establezca la conexión espera
                if time.perf_counter() - _conn_t0 > SAFETY_CONNECTION_TIMEOUT:
                    logger.error(f"TIMEOUT: No se pudo establecer conexion implicita con {self.hostname} en {SAFETY_CONNECTION_TIMEOUT}s")
                    self.musician_pipe.send(["driver_started"])
                    return
                time.sleep(0.01)
            
            # en caso de que el motor haya estado moviendose se frena. 
            # este comando sirve de ejemplo de como se envian comandos mientras esté abierta la linea de comunicación implicita
            stop = self.get_immediate_stop_command() # primero se obtiene el comando (para lo que hay varias funciones implementadas en esta clase) que es de la clase Comando
            self.comm_data[self.hostname + '_out'] = stop.get_list_to_send() # luego se asigna en self.comm_data[self.hostname + '_out'] (que depende del hostname) el comando en forma de lista. Esta lista es una lista de bools con los bits del mensaje a enviar
            time.sleep(0.1)

            if self.home: # si se quiere hacer una rutina de homing, que es necesaria para saber la posición real del motor
                if self.initial_settings.desired_input_2_function_bits == INPUT_FUNCTION_BITS['CCW Limit']: # el eje alpha tiene su rutina de homing hacia el sentido opuesto del reloj, mientras que los otros dos ejes la tienen en sentido contrario. Esto resulta util para algunas diferencias en sus procesos de homing. Cuando los ejes X y Z alcanzan el limit switch simplemente toman una distancia de seguridad y asignan ese punto como su origen. El eje alpha en cambio debe quedar horizontal, por lo que al encontrar el limit switch despues se mueve una cierta cantidad de pasos (calculado como el offset del limit switch) y define ese punto como su origen.
                    logger.info(f"{self.hostname}: Buscando CCW (speed={HOMING_CCW_JOG_SPEED}, acc={HOMING_CCW_JOG_ACC})")
                    self.ccw_find_home_to_limit() # inicia la busqueda del origen. Esta rutina tiene dos etapas. Primero busca el origen a una velocidad mayor, cuando lo encuentra toma un poco de distancia y lo vuelve a buscar pero más lento para tener mayor precisión.
                    self.slow_ccw_limit_homing = True # Inicia rápido hasta que se activa el limit switch
                    _homing_t0 = time.perf_counter()
                    _homing_failed = False
                    _homing_log_count = 0
                    while self.fast_ccw_limit_homing or self.slow_ccw_limit_homing: # mientras no termine las dos etapas se mantiene en loop
                        if time.perf_counter() - _homing_t0 > SAFETY_HOMING_TIMEOUT:
                            logger.error(f"TIMEOUT: Homing CCW de {self.hostname} excedio {SAFETY_HOMING_TIMEOUT}s. Deteniendo motor. Driver continua en modo degradado.")
                            self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
                            _homing_failed = True
                            break
                        time.sleep(0.5)
                        data = self.read_input() # lee el estado del motor.
                        self.process_incoming_data(data) # Si se activo el limit switch, el motor se frenó y estará activado el limit_condition
                        _homing_log_count += 1
                        if _homing_log_count % 10 == 0:  # log cada 5s (10 * 0.5s)
                            logger.info(f"{self.hostname} homing CCW: t={time.perf_counter()-_homing_t0:.0f}s, moving_ccw={self.moving_ccw}, moving_cw={self.moving_cw}, stopped={self.stopped}, limit={self.limit_condition}, pos={self.encoder_position.value}, mode={self.mode_select_bit}")
                        if self.limit_condition:
                            if self.verbose:
                                print('Limit condition')
                            if self.fast_ccw_limit_homing: # si estaba moviendose rapido cuando se activo la condición
                                if self.verbose:
                                    print('was moving fast ccw')
                                self.slow_ccw_limit_homing = True # ahora intentará buscar el origen mas lento
                                self.fast_ccw_limit_homing = False
                                # Para esto usamos un assembly move, donde primero se aleja una distancia y luego vuelve mas lento
                                steps = [{'pos': 300, 'speed': 400, 'acc': 400, 'dec': 400, 'jerk': 0}, 
                                            {'pos': -350, 'speed': 100, 'acc': 400, 'dec': 400, 'jerk': 0}]
                                self.program_run_assembled_move(steps, dwell_move=1, dwell_time=100, motor_current=self.motor_current)
                                time.sleep(3)

                            elif self.slow_ccw_limit_homing: # si estaba moviendose lento cuando se activo la condición se sale del loop
                                if self.verbose:
                                    print('was moving slow ccw')
                                self.slow_ccw_limit_homing = False

                    if not _homing_failed:
                        time.sleep(0.5)

                        # decimos que nos encontramos en la posición HOMING_ALPHA_OFFSET (en pasos del motor)
                        c = self.get_preset_encoder_position_command(HOMING_ALPHA_OFFSET)
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                        time.sleep(0.1)

                        c = self.get_preset_position_command(HOMING_ALPHA_OFFSET)
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                        time.sleep(0.1)

                        # luego nos movemos al origen (donde la boca esta horizontal)
                        c = self.get_relative_move_command(-HOMING_ALPHA_OFFSET, programmed_speed=HOMING_ALPHA_MOVE_SPEED, acceleration=self.acc, deceleration=self.dec, motor_current=self.motor_current)
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                        time.sleep(0.1)
                        _move_t0 = time.perf_counter()
                        while True: # esperamos que el movimiento al origen termine antes de seguir
                            data = self.read_input()
                            self.process_incoming_data(data)
                            if self.move_complete:
                                break
                            if time.perf_counter() - _move_t0 > SAFETY_MOVE_TIMEOUT:
                                logger.error(f"TIMEOUT: Movimiento a origen de {self.hostname} excedio {SAFETY_MOVE_TIMEOUT}s")
                                self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
                                break
                            time.sleep(0.1)

                        c = self.get_reset_errors_command()
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send() # se eliminan los errores en caso de haber uno
                        time.sleep(0.1)

                        c = self.get_return_to_command_mode_command()
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                        time.sleep(0.1)

                        if self.verbose:
                            print('Homed')

                elif self.initial_settings.desired_input_2_function_bits == INPUT_FUNCTION_BITS['CW Limit']:
                    # la rutina de homing de X y Z es parecida a la de alpha, solo que en sentido de giro opuesto
                    self.cw_find_home_to_limit()
                    self.fast_cw_limit_homing = True
                    _homing_t0 = time.perf_counter()
                    _homing_failed = False
                    while self.fast_cw_limit_homing or self.slow_cw_limit_homing:
                        if time.perf_counter() - _homing_t0 > SAFETY_HOMING_TIMEOUT:
                            logger.error(f"TIMEOUT: Homing CW de {self.hostname} excedio {SAFETY_HOMING_TIMEOUT}s. Deteniendo motor. Driver continua en modo degradado.")
                            self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
                            _homing_failed = True
                            break
                        time.sleep(0.5)
                        data = self.read_input()
                        self.process_incoming_data(data)
                        if self.limit_condition:
                            if self.verbose:
                                print('Limit condition')
                            if self.fast_cw_limit_homing:
                                if self.verbose:
                                    print('was moving fast cw')
                                # self.request_write_reset_errors()
                                self.slow_cw_limit_homing = True
                                self.fast_cw_limit_homing = False
                                steps = [{'pos': -4000, 'speed': 2000, 'acc': 400, 'dec': 400, 'jerk': 0}, 
                                            {'pos': 4500, 'speed': 800, 'acc': 400, 'dec': 400, 'jerk': 0}]
                                self.program_run_assembled_move(steps, dwell_move=1, dwell_time=100, motor_current=self.motor_current)
                                time.sleep(2.5)
                                
                            elif self.slow_cw_limit_homing:
                                if self.verbose:
                                    print('was moving slow cw')
                                self.slow_cw_limit_homing = False
                    if not _homing_failed:
                        time.sleep(0.5)

                        # cuando ya esta en el origen, se toma una distancia de seguridad (para desactivar la condicion de limite)
                        c = self.get_relative_move_command(HOMING_XZ_SAFETY_DIST, programmed_speed=HOMING_XZ_SAFETY_SPEED, acceleration=self.acc, deceleration=self.dec, motor_current=self.motor_current)
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                        time.sleep(0.1)
                        _move_t0 = time.perf_counter()
                        while True:  # esperamos verificando move_complete en vez de sleep fijo
                            data = self.read_input()
                            self.process_incoming_data(data)
                            if self.move_complete:
                                break
                            if time.perf_counter() - _move_t0 > SAFETY_MOVE_TIMEOUT:
                                logger.error(f"TIMEOUT: Movimiento de seguridad de {self.hostname} excedio {SAFETY_MOVE_TIMEOUT}s")
                                self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
                                break
                            time.sleep(0.1)

                        c = self.get_reset_errors_command()
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send() # se eliminan los errores
                        time.sleep(0.1)

                        # y luego se define este punto como el origen
                        c = self.get_preset_position_command(0)
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                        time.sleep(0.1)

                        c = self.get_preset_encoder_position_command(0)
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                        time.sleep(0.1)

                        c = self.get_return_to_command_mode_command()
                        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                        time.sleep(0.1)

                        if self.verbose:
                            print('Homed')
            else:
                # si no se quiere hacer la rutina de homing se asigna la posición de inicio como el origen
                c = self.get_preset_position_command(0)
                self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
                time.sleep(0.1)

            self.musician_pipe.send(["driver_started"]) # se da aviso de que el driver esta funcionando
            self.synchrostep_out_list = synchrostep_command.get_list_to_send()
            self.comm_data[self.hostname + '_out'] = self.synchrostep_out_list # se comienza el movimiento 'synchrostep' donde el motor intenta seguir el eje virtual cuya posición y velocidad se informan de forma periódica al dispositivo

            # Resetear watchdog timer post-homing: el homing puede tardar segundos,
            # y el timer se inició en __init__, así que sin reset el watchdog salta inmediatamente.
            self.last_watchdog_time = time.perf_counter()
            # Drenar heartbeats que se acumularon en el pipe durante homing
            while self.musician_pipe.poll(0):
                try:
                    msg = self.musician_pipe.recv()
                    if msg[0] == "heartbeat":
                        self.last_watchdog_time = time.perf_counter()
                except Exception:
                    break

            _t_loop_prev = time.perf_counter()
            while self.running.is_set(): # en este loop se mantiene el resto del programa
                _t_loop_start = time.perf_counter()
                if self.forced_break: # si se fuerza una salida
                    break
                # Watchdog: si el musician no envia heartbeat en SAFETY_WATCHDOG_TIMEOUT, detener motor
                if SAFETY_WATCHDOG_TIMEOUT > 0 and (_t_loop_start - self.last_watchdog_time) > SAFETY_WATCHDOG_TIMEOUT:
                    logger.critical(f'{self.hostname}: WATCHDOG TIMEOUT — musician no responde hace {SAFETY_WATCHDOG_TIMEOUT}s. Deteniendo motor.')
                    self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
                    self.virtual_axis.stop()
                    self.last_watchdog_time = time.perf_counter()  # reset para no spamear el log
                # Medir dt real para el PID (con clamp de seguridad)
                dt = max(0.001, min(0.05, _t_loop_start - _t_loop_prev))
                _t_loop_prev = _t_loop_start
                data = self.read_input(read_output=False)
                self.process_incoming_data(data)
                self.pos_ref.value = self.virtual_axis.pos
                corrected_pos, corrected_vel = self.pid_control(self.virtual_axis.pos, self.virtual_axis.vel, dt) # se calculan la posicion y velocidad corregida por el control pid
                if type(corrected_pos) == int and type(corrected_vel) == int:
                    try:
                        self.set_output(-corrected_pos, -corrected_vel) # se envian las posiciones al dispositivo (dentro de esta función se actualiza la lista de salida)
                    except (struct.error, KeyError, IndexError, ValueError) as e:
                        logger.error(f'{self.hostname} Error en set_output: {e} | ref: {self.virtual_axis.pos}, {self.virtual_axis.vel}, corr: {corrected_pos}, {corrected_vel}')
                    except Exception as e:
                        logger.critical(f'{self.hostname} Error inesperado en set_output: {e}', exc_info=True)
                
                try:
                    if self.musician_pipe.poll(): # se revisa si hay alguna instrucción desde el proceso del musico
                        message = self.musician_pipe.recv()
                        if self.verbose:
                            print("Message received in", self.hostname, message)
                        if message[0] == "emergency_stop": # parada de emergencia inmediata
                            logger.critical(f'{self.hostname}: EMERGENCY STOP recibido — ejecutando immediate_stop')
                            self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
                            self.virtual_axis.stop()
                        elif message[0] == "heartbeat": # watchdog heartbeat del musician
                            self.last_watchdog_time = time.perf_counter()
                        elif message[0] == "ask_control": # si quiere saber los parametros del control
                            l = [self.Kp, self.Ki, self.Kd, self.acc, self.dec, self.virtual_axis_proportional_coef, self.Kp_vel, self.Ki_vel, self.Kd_vel]
                            self.musician_pipe.send([l])
                        elif message[0] == "change_control": # si se ordena cambiar los parametros del control
                            if self.Kp != message[1]['kp'] or self.Ki != message[1]['ki'] or self.Kd != message[1]['kd'] or self.acc != message[1]['acceleration'] or self.dec != message[1]['deceleration'] or self.virtual_axis_proportional_coef != message[1]['proportional_coef'] or self.Kp_vel != message[1]['kp_vel'] or self.Ki_vel != message[1]['ki_vel'] or self.Kd_vel != message[1]['kd_vel']:
                                print("Control loop changed")
                                self.comm_data[self.hostname + '_out'] = self.get_return_to_command_mode_command().get_list_to_send()
                                time.sleep(0.1)

                            self.Kp = message[1]['kp']
                            self.Ki = message[1]['ki']
                            self.Kd = message[1]['kd']
                            self.acc = message[1]['acceleration']
                            self.dec = message[1]['deceleration']
                            self.virtual_axis_proportional_coef = message[1]['proportional_coef']
                            self.Kp_vel = message[1]['kp_vel']
                            self.Ki_vel = message[1]['ki_vel']
                            self.Kd_vel = message[1]['kd_vel']
                            self.synchrostep_out_list = self.get_synchrostep_move_command(self.virtual_axis.pos, 0, speed=self.virtual_axis.vel, acceleration=self.acc, deceleration=self.dec, proportional_coefficient=self.virtual_axis_proportional_coef, network_delay=NETWORK_DELAY, encoder=False).get_list_to_send()
                except (EOFError, BrokenPipeError, ConnectionResetError):
                    logger.critical(f'{self.hostname}: PIPE ROTO — musician desconectado. Deteniendo motor.')
                    self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
                    self.virtual_axis.stop()
                    break  # salir del loop — ya no hay comunicación

                # Spin-wait híbrido: sleep grueso + spin fino para el último ms
                _elapsed = time.perf_counter() - _t_loop_start
                _remaining = self.Ts - _elapsed
                if _remaining > 0.002:
                    time.sleep(_remaining - 0.001)  # sleep grueso, dejando 1ms de margen
                while time.perf_counter() - _t_loop_start < self.Ts:
                    pass  # spin-wait fino para precision sub-milisegundo

            # cuando se termina el programa se cierra la linea de comunicacion
            self.comm_pipe.send(["stopProduce", self.hostname, 100, 150, 110])

    def pid_control(self, ref_pos, ref_vel, dt):
        """
        Control PID para corregir la posición y velocidad.
        Incluye anti-windup y clampeo de posicion/velocidad por seguridad.
        Usa dt medido (tiempo real del loop) en vez de Ts hardcodeado.
        """
        SP = ref_pos # set point
        CV = self.encoder_position.value # controlled var
        e = SP-CV # error
        MV_P = self.Kp*e

        SP_vel = ref_vel
        CV_vel = (CV - self.last_pos) / dt
        e_vel = SP_vel-CV_vel
        MV_P_vel = self.Kp_vel*e_vel

        MV_I_vel = self.MV_I0_vel + self.Ki_vel*dt*e_vel
        MV_D_vel = self.Kd_vel*(e_vel-self.e0_vel)/dt

        MV_I = self.MV_I0 + self.Ki*dt*e
        MV_D = self.Kd*(e-self.e0)/dt

        # Calcular salida y clampear a limites de posicion
        raw_pos = int(round(SP + MV_P + MV_I + MV_D, 0))
        self.MV = max(self.pos_min, min(self.pos_max, raw_pos))
        # Anti-windup: si la salida satura, corregir el integral para evitar acumulación
        if raw_pos != self.MV:
            MV_I -= self.Ka * (raw_pos - self.MV)
            if not self._pos_clamp_logged:
                logger.warning(f"{self.hostname}: Posicion PID clampeada de {raw_pos} a {self.MV} (limites: [{self.pos_min}, {self.pos_max}])")
                self._pos_clamp_logged = True
        else:
            self._pos_clamp_logged = False
        self.e0 = e
        self.MV_I0 = MV_I

        # Calcular velocidad y clampear
        raw_vel = int(round(SP_vel + MV_P_vel + MV_I_vel + MV_D_vel, 0))
        self.MV_vel = max(-SAFETY_MAX_VELOCITY, min(SAFETY_MAX_VELOCITY, raw_vel))
        # Anti-windup para velocidad
        if raw_vel != self.MV_vel:
            MV_I_vel -= self.Ka_vel * (raw_vel - self.MV_vel)
        self.e0_vel = e_vel
        self.MV_I0_vel = MV_I_vel

        self.last_pos = CV

        return self.MV, self.MV_vel
    
    def sat(self, x, low, high): # satura x en low y high
        if x < low:
            return low
        elif x > high:
            return high
        return x
    
    def break_loop(self): # fuerza la salida del loop principal
        self.forced_break = True

    def get_absolute_move_command(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=5): # retorna el comando para ejecutar un movimiento absoluto
        #print(target_position, programmed_speed)
        command = Command(absolute_move=1, name='Absolute Move')
        command.desired_command_word_2 = abs(target_position) // 1000 * sign(target_position)
        command.desired_command_word_3 = abs(target_position)  % 1000 * sign(target_position)
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_relative_move_command(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=0): # retorna el comando para ejecutar un movimiento relativo
        command = Command(relative_move=1, name='Relative Move')
        command.desired_command_word_2 = abs(target_position) // 1000 * sign(target_position)
        command.desired_command_word_3 = abs(target_position)  % 1000 * sign(target_position)
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def ccw_find_home_to_limit(self): # función para buscar el home contra el sentido del reloj
        #self.fast_ccw_limit_homing = True
        ccw_jog = self.get_ccw_jog_command(programmed_speed=HOMING_CCW_JOG_SPEED, acceleration=HOMING_CCW_JOG_ACC, motor_current=self.motor_current)
        self.comm_data[self.hostname + '_out'] = ccw_jog.get_list_to_send()
    
    def cw_find_home_to_limit(self): # función para buscar el home al sentido del reloj
        self.fast_cw_limit_homing = True
        cw_jog = self.get_cw_jog_command(programmed_speed=HOMING_CW_JOG_SPEED, motor_current=self.motor_current)
        self.comm_data[self.hostname + '_out'] = cw_jog.get_list_to_send()

    def get_reset_errors_command(self): # retorna el comando para resetear errores
        command = Command(reset_errors=1, enable_driver=1, clear_driver_fault=1, name='Reset Errors')
        command.desired_command_word_8 = self.motor_current
        return command

    def get_immediate_stop_command(self, motor_current=30): # retorna el comando para ejecutar stop inmediato
        command = Command(immediate_stop=1, name='Immediate Stop')
        command.desired_command_word_8 = motor_current
        return command

    def get_ccw_jog_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1): # retorna el comando para ejecutar un movimiento jog ccw (moverse indefinidamente ccw)
        command = Command(jog_ccw=1, name='CCW Jog')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_cw_jog_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1): # retorna el comando para ejecutar un movimiento jog cw (moverse indefinidamente cw)
        command = Command(jog_cw=1, name='CW Jog')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_return_to_command_mode_command(self): # retorna el comando para volver al modo de comando (si se encuentra en modo de configuración)
        command = Command(name='Return to Command Mode')
        return command

    def get_preset_position_command(self, position): # retorna el comando para setear la posicion actual del motor en algun valor
        command = Command(preset_motor_position=1, name='Preset Position')
        command.desired_command_word_2 = abs(position) // 1000 * sign(position)
        command.desired_command_word_3 = abs(position)  % 1000 * sign(position)
        command.desired_command_word_8 = self.motor_current
        return command
    
    def get_preset_encoder_position_command(self, position): # retorna el comando para setear la posicion actual del encoder en algun valor
        command = Command(preset_encoder=1, name='Preset Encoder Position')
        command.desired_command_word_2 = abs(position) // 1000 * sign(position)
        command.desired_command_word_3 = abs(position)  % 1000 * sign(position)
        return command

    def get_ccw_find_home_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1): # retorna el comando para buscar el home ccw. En nuestro caso no sirve porque al usar el movimiento synchrostep se fija la velocidad de inicio en 1 (lo mas bajo que se puede), lo que hace que este comando sea demasiado lento. Para modificar este valor se debe cambiar a modo de configuración, pero entonces la posicion se invalida porque se puede haber movido, lo que se evita hacer una vez que se encuentra el origen
        command = Command(find_home_ccw=1, name='CCW FFind Home')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_cw_find_home_command(self, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=1): # retorna el comando para buscar el home cw. Mismo caso que el anterior
        command = Command(find_home_cw=1, name='CW FFind Home')
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def read_input(self, read_output=False, explicit=False): # lee el estado del dispositivo
        if not explicit: # si la lectura de los datos se realiza de forma implicita
            words = []
            for w in range(20):
                words.append(int("".join(["1" if self.comm_data[self.hostname + '_in'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b = bytearray(20)
            struct.pack_into('20B', b, 0, *words) # se debe empaquetar el mensaje para interpretarlo bien
        else: # si la lectura de los datos se realiza de forma explicita
            self.comm_pipe.send(["getAttrSingle", self.hostname, 0x04, 100, 0x03])
            while True:
                response = self.comm_pipe.recv()
                if response[0] == self.hostname:
                    status = response[1]
                    break
            b = status[1]
        
        [i0, i1, i2, i3, i4, i5, i6, i7, i8, i9] = struct.unpack('<10H', b) # se desempaquetan los bytes en 10 ints
        
        if read_output: # en caso de que se quiera leer el output tambien
            words2 = []
            for w in range(20):
                words2.append(int("".join(["1" if self.comm_data[self.hostname + '_out'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b2 = bytearray(20)
            struct.pack_into('20B', b2, 0, *words2)
            [o0, o1, o2, o3, o4, o5, o6, o7, o8, o9] = struct.unpack('<10H', b2)
        
        return [i0, i1, i2, i3, i4, i5, i6, i7, i8, i9]

    def get_program_assembled_command(self): # retorna un comando para programar un movimiento assembly (que es uno de los pasos)
        command = Command(program_assembled=1, name='Program Assembles')
        return command

    def get_assembled_segment_command(self, target_position, programmed_speed=200, acceleration=100, deceleration=100, motor_current=30, acceleration_jerk=0): # retorna un comando para programar un segmento de un movimiento assembly (otro paso)
        command = Command(read_assembled_data=1, program_assembled=1, name='Write Assembled Segment')
        command.desired_command_word_2 = abs(target_position) // 1000 * sign(target_position)
        command.desired_command_word_3 = abs(target_position)  % 1000 * sign(target_position)
        command.desired_command_word_4 = programmed_speed // 1000
        command.desired_command_word_5 = programmed_speed  % 1000
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = acceleration_jerk
        return command

    def get_run_assembled_move_command(self, motor_current=30, blend_direction=0, dwell_move=0, dwell_time=0): # retorna un comando para ejecutar un movimiento assembly (ultimo paso)
        command = Command(run_assembled_move=1, reverse_blend_direction=blend_direction, assembled_move_type=dwell_move, name='Run Assembled Move')
        command.desired_command_word_8 = motor_current
        command.desired_command_word_9 = dwell_time
        return command

    def program_run_assembled_move(self, steps, motor_current=30, blend_direction=0, dwell_move=0, dwell_time=0): # esta función programa y ejecuta un movimiento assembly especificado en los pasos de la lista steps, donde cada paso tiene pos, speed, acc, dec y jerk. El máximo de pasos es de ... (referirse al manual)
        c = self.get_program_assembled_command()
        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
        time.sleep(0.1)
        for step in steps:
            c = self.get_assembled_segment_command(target_position=step['pos'], programmed_speed=step['speed'], acceleration=step['acc'], deceleration=step['dec'], acceleration_jerk=step['jerk'], motor_current=motor_current)
            self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
            time.sleep(0.1)
            c = self.get_program_assembled_command()
            self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
            time.sleep(0.1)
        c = self.get_return_to_command_mode_command()
        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
        time.sleep(0.1)
        c = self.get_run_assembled_move_command(motor_current=motor_current, blend_direction=blend_direction, dwell_move=dwell_move, dwell_time=dwell_time)
        self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
        time.sleep(0.1)

    def process_incoming_data(self, data): # procesa la data que se lee del dispositivo para actualizar los atributos de la clase.
        word0 = format(data[0], 'b').zfill(16) # se escriben las dos primeras palabras como bits para interpretarlos uno a uno
        word1 = format(data[1], 'b').zfill(16)
        mode = int(word0[0], 2) # el modo es el primer bit (configuracion o comando). Este bit define la interpretacion de todos los demas.

        if mode != self.mode_select_bit:
            if mode:
                print('Changed to configuration mode')
                #print(data)
                command = self.get_return_to_command_mode_command()
                self.send_data(command.get_bytes_to_send())
                time.sleep(0.01)

        self.mode_select_bit = int(word0[0], 2)
        
        if self.mode_select_bit: ## configuration mode
            self.disable_anti_resonance_bit = int(word0[1], 2)
            self.enable_stall_detection_bit = int(word0[2], 2)
            self.use_backplane_proximity_bit = int(word0[4], 2)
            self.use_encoder_bit = int(word0[5], 2)
            self.home_to_encoder_z_pulse = int(word0[6], 2)
            self.input_3_function_bits = int(word0[7:10], 2)
            self.input_2_function_bits = int(word0[10:13], 2)
            self.input_1_function_bits = int(word0[13:16], 2)

            self.output_functionality_bit = int(word1[1], 2)
            self.output_state_control_on_network_lost = int(word1[2], 2)
            self.output_state_on_network_lost = int(word1[3], 2)
            self.read_present_configuration = int(word1[4], 2)
            self.save_configuration = int(word1[5], 2)
            self.binary_input_format = int(word1[6], 2)
            self.binary_output_format = int(word1[7], 2)
            self.binary_endian = int(word1[8], 2)
            self.input_3_active_level = int(word1[13], 2)
            self.input_2_active_level = int(word1[14], 2)
            self.input_1_active_level = int(word1[15], 2)

            self.starting_speed = data[2]*1000 + data[3]
            self.motors_step_turn = data[4]
            self.hybrid_control_gain = data[5]
            self.encoder_pulses_turn = data[6]
            self.idle_current_percentage = data[7]
            #self.motor_current = data[8]
            self.current_loop_gain = data[9]
        else: ## command mode
            module_ok = int(word0[1], 2)
            self.module_ok = module_ok
            configuration_error = int(word0[2], 2)
            if configuration_error and configuration_error != self.configuration_error:
                logger.error(f'{self.hostname}: Configuration Error')
            self.configuration_error = configuration_error
            command_error = int(word0[3], 2)
            if command_error and command_error != self.command_error:
                logger.error(f'{self.hostname}: Command Error')
            self.command_error = command_error
            input_error = int(word0[4], 2)
            if input_error and input_error != self.input_error:
                logger.error(f'{self.hostname}: Input Error')
            self.input_error = input_error
            position_invalid = int(word0[5], 2)
            if position_invalid and position_invalid != self.position_invalid:
                logger.warning(f'{self.hostname}: Position Invalid')
            self.position_invalid = position_invalid
            waiting_for_assembled_segment = int(word0[6], 2)
            if waiting_for_assembled_segment != self.waiting_for_assembled_segment:
                if self.verbose:
                    print('Waiting for assembled segment changed')
            self.waiting_for_assembled_segment = waiting_for_assembled_segment
            in_assembled_mode = int(word0[7], 2)
            if in_assembled_mode != self.in_assembled_mode:
                if self.verbose:
                    print('In assembled mode changed')
            self.in_assembled_mode = in_assembled_mode
            move_complete = int(word0[8], 2)
            self.move_complete = move_complete
            decelerating = int(word0[9], 2)
            self.decelerating = decelerating
            accelerating = int(word0[10], 2)
            self.accelerating = accelerating
            at_home = int(word0[11], 2)
            self.at_home = at_home
            stopped = int(word0[12], 2)
            self.stopped = stopped
            in_hold_state = int(word0[13], 2)
            self.in_hold_state = in_hold_state
            moving_ccw = int(word0[14], 2)
            self.moving_ccw = moving_ccw
            moving_cw = int(word0[15], 2)
            self.moving_cw = moving_cw

            driver_is_enabled = int(word1[0], 2)
            self.driver_is_enabled = driver_is_enabled
            stall_detected = int(word1[1], 2)
            if stall_detected and stall_detected != self.stall_detected:
                logger.warning(f'{self.hostname}: STALL DETECTADO — el motor puede estar bloqueado. Deteniendo por seguridad.')
                # Detener el motor para evitar dano mecanico
                self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
            self.stall_detected = stall_detected
            output_state = int(word1[2], 2)
            self.output_state = output_state
            heartbeat_bit = int(word1[4], 2)
            self.heartbeat_bit = heartbeat_bit
            limit_condition = int(word1[5], 2)
            self.limit_condition = limit_condition
            invalid_jog_change = int(word1[6], 2)
            if invalid_jog_change != self.invalid_jog_change:
                if self.verbose:
                    print('Invalid jog changed')
            self.invalid_jog_change = invalid_jog_change
            motion_lag = int(word1[7], 2)
            self.motion_lag = motion_lag
            driver_fault = int(word1[8], 2)
            if driver_fault and driver_fault != self.driver_fault:
                logger.error(f'{self.hostname}: DRIVER FAULT detectado. Intentando reset de errores.')
                c = self.get_reset_errors_command()
                self.comm_data[self.hostname + '_out'] = c.get_list_to_send()
            self.driver_fault = driver_fault
            connection_was_lost = int(word1[9], 2)
            if connection_was_lost and connection_was_lost != self.connection_was_lost:
                logger.critical(f'{self.hostname}: CONEXION PERDIDA con el driver AMCI.')
            self.connection_was_lost = connection_was_lost
            plc_in_prog_mode = int(word1[10], 2)
            self.plc_in_prog_mode = plc_in_prog_mode
            temperature_above_90 = int(word1[11], 2)
            if temperature_above_90 and temperature_above_90 != self.temperature_above_90:
                logger.critical(f'{self.hostname}: TEMPERATURA SOBRE 90°C — deteniendo motor para evitar dano termico.')
                self.comm_data[self.hostname + '_out'] = self.get_immediate_stop_command(motor_current=self.motor_current).get_list_to_send()
            elif not temperature_above_90 and self.temperature_above_90:
                logger.info(f'{self.hostname}: Temperatura volvio a rango normal.')
            self.temperature_above_90 = temperature_above_90
            in_3_active = int(word1[13], 2)
            self.in_3_active = in_3_active
            in_2_active = int(word1[14], 2)
            self.in_2_active = in_2_active
            in_1_active = int(word1[15], 2)
            self.in_1_active = in_1_active

            pos1 = data[2]
            pos2 = data[3]
            if pos1 > 2**15:
                pos1 = pos1 - 2**16
            if pos2 > 2**15:
                pos2 = pos2 - 2**16
            motor_position = pos1*1000 + pos2
            self.motor_position.value = -motor_position
            pos1 = data[4]
            pos2 = data[5]
            if pos1 > 2**15:
                pos1 = pos1 - 2**16
            if pos2 > 2**15:
                pos2 = pos2 - 2**16
            encoder_position = pos1*1000 + pos2
            self.encoder_position.value = -encoder_position
            captured_encoder_position = data[6]*1000 + data[7]
            self.captured_encoder_position = captured_encoder_position
            programed_motor_current = data[8]
            self.programed_motor_current = programed_motor_current
            acceleration_jerk = data[9]
            self.acceleration_jerk = acceleration_jerk

    def set_output(self, pos_value, vel_value): # función para actualizar la lista que se envia al dispositivo con las nuevas posiciones para la posicion y la velocidad
        if pos_value < 0: # si la posición es negativa, se le suma 2*(2**31) para la conversion a bits
            pos_value += 2*(2**31)
        pos_in_bits = "{0:b}".format(pos_value).zfill(32) # se escribe la posicion como bits
        
        if vel_value < 0: # si la velocidad es negativa, se le suma 2*(2**31) para la conversion a bits
            vel_value += 2*(2**31)
        speed_in_bits = "{0:b}".format(vel_value).zfill(32) # se escribe la velocidad como bits

        # ahora se separan la posicion y velocidad en 4 palabras de 16 bits cada uno (antes eran de 32)
        # cada palabra se transforma a int y si resulta negativo se le suma 2**16
        words = [0, 0, 0, 0]
        words[0] = int(pos_in_bits[16:], 2)
        if words[0] >= 2**15:
            words[0] -= 2**16
        words[1] = int(pos_in_bits[:16], 2)
        if words[1] >= 2**15:
            words[1] -= 2**16
        words[2] = int(speed_in_bits[16:], 2)
        if words[2] >= 2**15:
            words[2] -= 2**16
        words[3] = int(speed_in_bits[:16], 2)
        if words[3] >= 2**15:
            words[3] -= 2**16
        
        # luego se empaquetan como bytes (8)
        b = bytearray(8)
        struct.pack_into('hhhh', b, 0, *words)

        # Estos bytes se escriben en forma binaria y se invierten (cada uno, pero no entre ellos)
        words = []
        for byte in b:
            words.append(''.join(format(byte, '08b'))[::-1])
        
        # luego se actualiza la parte de la lista que se envia que contiene la informacion de la posicion y velocidad
        for i in range(8):
            for j in range(8):
                self.synchrostep_out_list[16*2+i*8+j] = int(words[i][j]) == 1
        self.comm_data[self.hostname + '_out'] = self.synchrostep_out_list

    def send_data(self, data): # envía un mensaje al dispositivo de forma explicita
        self.comm_pipe.send(["setAttrSingle", self.hostname, 0x04, 150, 0x03, data])

    def get_synchrostep_move_command(self, position, direction, speed=200, acceleration=50, deceleration=50, proportional_coefficient=1, network_delay=0, encoder=False): # retorna el comando para ejecutar el movimiento 'synchrostep'
        command = Command(name='Synchrostep Move')
        if encoder:
            command.desired_virtual_encoder_follower = 1
            command.desired_virtual_position_follower = 0
            #command.desired_encoder_registration_move = 1
            command.desired_hybrid_control_enable = 1
        else:
            command.desired_virtual_encoder_follower = 0
            command.desired_virtual_position_follower = 1
            command.desired_hybrid_control_enable = 0
            command.desired_encoder_registration_move = 0
        if direction:
            command.desired_jog_cw = 1
            command.desired_jog_ccw = 0
        else:
            command.desired_jog_cw = 0
            command.desired_jog_ccw = 1

        if position < 0:
            position += 2*(2**31)
            pos_in_bits = "{0:b}".format(position).zfill(32)
        else:
            pos_in_bits = "{0:b}".format(position).zfill(32)
        
        if speed < 0:
            speed += 2*(2**31)
            speed_in_bits = "{0:b}".format(speed).zfill(32)
        else:
            speed_in_bits = "{0:b}".format(speed).zfill(32)

        command.desired_command_word_2 = int(pos_in_bits[16:], 2)
        if command.desired_command_word_2 >= 2**15:
            command.desired_command_word_2 -= 2**16
        command.desired_command_word_3 = int(pos_in_bits[:16], 2)
        if command.desired_command_word_3 >= 2**15:
            command.desired_command_word_3 -= 2**16
        command.desired_command_word_4 = int(speed_in_bits[16:], 2)
        if command.desired_command_word_4 >= 2**15:
            command.desired_command_word_4 -= 2**16
        command.desired_command_word_5 = int(speed_in_bits[:16], 2)
        if command.desired_command_word_5 >= 2**15:
            command.desired_command_word_5 -= 2**16
        command.desired_command_word_6 = acceleration
        command.desired_command_word_7 = deceleration
        command.desired_command_word_8 = proportional_coefficient
        command.desired_command_word_9 = network_delay

        return command

class FlowControllerDriver(Process):
    """
    Esta clase interactua con el controlador de flujo. Define los mensajes a enviar (principalmente el setpoint) e interpreta la informacion leida desde este.
    """
    def __init__(self, hostname, running, t0, mus_pipe, comm_pipe, comm_data, axis_pipe, connected=True, verbose=False): # 
        Process.__init__(self) # Initialize the threading superclass
        self.hostname = hostname
        self.running = running
        self.t0 = t0
        self.mus_pipe = mus_pipe # pipe que conecta con el musico
        self.axis_pipe = axis_pipe # pipe que va al eje (flujo) virtual
        self.comm_pipe = comm_pipe # pipe que conecta con la central de comunicaciones
        self.comm_data = comm_data # data compartida entre procesos
        self.connected = connected
        self.virtual_flow = None

        self.verbose = verbose
        self.mass_flow_reading = Value("d", 0.0)
        self.vol_flow_reading = Value("d", 0.0)
        self.temperature_reading = Value("d", 0.0)
        self.absolute_preasure_reading = Value("d", 0.0)
        self.mass_flow_set_point_reading = Value("d", 0.0)
        self.last_setpoint = 0.0  # para rate-limiting del flujo
        self.alicat_status_error = False  # flag para errores de status del Alicat

        if self.connected:
            self.comm_pipe.send(["explicit_conn", self.hostname, 26*8, 4*8, 26, 4, ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, 101, ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, 100])

    def run(self):
        ## Instanciamos e iniciamos el flujo virtual
        self.virtual_flow = VirtualFlow(self.running, 0.01, self.t0, self.axis_pipe, verbose=False)
        self.virtual_flow.start()
        ## avisamos que el driver esta funcionando (para saltar el start-up menu)
        self.mus_pipe.send(["flow_driver_started"])
        if self.connected:
            self.comm_pipe.send(["registerSession", self.hostname])
            self.comm_pipe.send(["sendFwdOpenReq", self.hostname, 101, 100, 0x6e, 50, 50, ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_HIGH])

            ## esperamos que se haya registrado la linea de comunicación en la data compartida
            _conn_t0 = time.perf_counter()
            while self.hostname not in " ".join(self.comm_data.keys()):
                if time.perf_counter() - _conn_t0 > SAFETY_CONNECTION_TIMEOUT:
                    logger.error(f"TIMEOUT: No se pudo conectar al controlador de flujo {self.hostname}")
                    return
                time.sleep(0.01)

            ## Entramos en el loop donde quedará por el resto del programa
            _flow_Ts = 0.01  # periodo nominal del loop de flujo
            while self.running.is_set():
                _t_flow_start = time.perf_counter()
                self.read_input(read_output=False) # leemos la información entregada desde el dispositivo
                self.set_output(self.virtual_flow.flow) # se envía el set-point al dispositivo
                # Spin-wait híbrido
                _elapsed = time.perf_counter() - _t_flow_start
                _remaining = _flow_Ts - _elapsed
                if _remaining > 0.002:
                    time.sleep(_remaining - 0.001)
                while time.perf_counter() - _t_flow_start < _flow_Ts:
                    pass

            self.comm_pipe.send(["stopProduce", self.hostname, 101, 100, 0x6e])
    
    def set_output(self, value): # modifica la lista que se envia al controlador de flujo para cambiar el setpoint al especificado en value
        value = min(50, max(0, value)) # primero saturamos entre 0 y 50 (limites del controlador)
        # Rate-limiting: limitar el cambio maximo por paso para evitar transitorios bruscos
        delta = value - self.last_setpoint
        if abs(delta) > SAFETY_FLOW_RATE_LIMIT:
            value = self.last_setpoint + SAFETY_FLOW_RATE_LIMIT * (1 if delta > 0 else -1)
            value = min(50, max(0, value))
        self.last_setpoint = value

        # empaquetamos el valor en 4 bytes con forma de float
        b = bytearray(4)
        struct.pack_into('f', b, 0, value)
        
        # lo escribimos en su forma binaria
        words = []
        for byte in b:
            words.append(''.join(format(byte, '08b'))[::-1])
        
        # creamos una lista de su representacion binaria con bools
        l = [0 for i in range(4*8)]
        for i in range(4):
            for j in range(8):
                l[i*8+j] = int(words[i][j]) == 1

        self.comm_data[self.hostname + '_out'] = l # reemplazamos la lista que se envia al controlador de flujo

    def read_input(self, read_output=False): # interpreta la entrada del controlador de flujo
        # primero transformamos la lista de bools que recibimos (en este formato lo entrega la librería de ethernet ip) en bytes
        words = []
        for w in range(26):
            words.append(int("".join(["1" if self.comm_data[self.hostname + '_in'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
        b = bytearray(26)
        struct.pack_into('26B', b, 0, *words)

        # luego interpretamos estos bytes de acuerdo al formato y cantidad de bytes para cada dato
        [g, s, ap, ft, vf, mf, mfsp] = struct.unpack('<HIfffff', b)

        # Verificar status bytes del Alicat para detectar errores del dispositivo
        # Los bits de status del Alicat indican condiciones de alarma/error
        if s != 0 and not self.alicat_status_error:
            logger.warning(f'{self.hostname}: Alicat status no-cero detectado: gauge={g}, status={s} — posible error del dispositivo')
            self.alicat_status_error = True
        elif s == 0 and self.alicat_status_error:
            logger.info(f'{self.hostname}: Alicat status volvio a normal')
            self.alicat_status_error = False

        self.mass_flow_reading.value = mf
        self.vol_flow_reading.value = vf
        self.temperature_reading.value = ft
        self.absolute_preasure_reading.value = ap
        self.mass_flow_set_point_reading.value = mfsp

        if read_output: # si tambien se quiere leer el output
            words2 = []
            for w in range(4):
                words2.append(int("".join(["1" if self.comm_data[self.hostname + '_out'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            
            b2 = bytearray(4)
            struct.pack_into('4B', b2, 0, *words2)
            [ref] = struct.unpack('<f', b2)
            print(g, s, ap, ft, vf, mf, mfsp, ref)

    def change_controlled_var(self, value):
        logger.warning(f'{self.hostname}: change_controlled_var({value}) solicitado pero no implementado en protocolo EtherNet/IP. El cambio no se aplico al dispositivo.')

    def change_control_loop(self, value):
        logger.warning(f'{self.hostname}: change_control_loop({value}) solicitado pero no implementado en protocolo EtherNet/IP. El cambio no se aplico al dispositivo.')

    def change_kp(self, value):
        logger.warning(f'{self.hostname}: change_kp({value}) solicitado pero no implementado en protocolo EtherNet/IP. El cambio no se aplico al dispositivo.')

    def change_ki(self, value):
        logger.warning(f'{self.hostname}: change_ki({value}) solicitado pero no implementado en protocolo EtherNet/IP. El cambio no se aplico al dispositivo.')

    def change_kd(self, value):
        logger.warning(f'{self.hostname}: change_kd({value}) solicitado pero no implementado en protocolo EtherNet/IP. El cambio no se aplico al dispositivo.')

class VirtualFlow(threading.Thread):
    """
    Esta clase se usa para simular el flujo virtual.
    Es instanciada dentro del drivers del controlador de flujo, y queda corriendo como un thread, el que actualiza e informa el flujo deseado en tiempo real.
    """
    def __init__(self, running, interval, t0, pipe_conn, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.running = running
        self.ref = deque([(0,0)]) # la ruta es un deque donde cada elemento es de la forma (tiempo, flujo)
        self.last_pos = 0
        self.interval = interval
        self.t0 = t0
        self.pipe_conn = pipe_conn
        self.verbose = verbose
        self.flow = 0.0
        self.vibrato_amp = 0.0
        self.vibrato_freq = 0.0
        
    def run(self):
        while self.running.is_set(): # corre durante toda la ejecución principal
            t = time.perf_counter() - self.t0
            self.flow = self.get_ref(t) # de acuerdo al tiempo actual actualiza el flujo a partir de la ruta
            if not (type(self.flow) == float or type(self.flow) == int or type(self.flow) == np.float64):
                print(type(self.flow), self.flow)
            if self.verbose:
                print(t, self.flow)
            self.update_ref(t)
            try:
                if self.pipe_conn.poll(self.interval):
                    message = self.pipe_conn.recv()
                    if message[0] == "get_ref":
                        self.get_ref(message[1])
                    elif message[0] == "update_ref":
                        self.update_ref(message[1])
                    elif message[0] == "merge_ref":
                        self.vibrato_amp = message[2]
                        self.vibrato_freq = message[3]
                        self.merge_ref(message[1])
                    elif message[0] == "stop":
                        self.stop()
            except (EOFError, BrokenPipeError, ConnectionResetError, OSError):
                self.stop()
                break

    def get_ref(self, t): # actualiza el flujo para el tiempo actual de acuerdo a la ruta referencia
        if self.ref[-1][0] > t: # si la ruta esta definida hasta un t_final > t_actual
            ramp = get_value_from_func(t, self.ref, approx=False) # busca el elemento de la ruta cuyo tiempo se acerque mas al actual
            vibr = ramp * self.vibrato_amp * sin(t * 2*pi * self.vibrato_freq) # calcula el vibrato de acuerdo a la amplitud especificada y la frecuencia
            flow = max(0,min(50, ramp+vibr)) # sumamos y saturamos
        else:
            ramp = self.ref[-1][1] # si t_actual > t_final de la ruta referencia el flujo se mantiene y se suma el vibrato
            vibr = ramp * self.vibrato_amp * sin(t * 2*pi * self.vibrato_freq)
            flow = max(0,min(50, ramp+vibr))
        return flow

    def update_ref(self, t): # actualiza la ruta borrando todos los elementos con t < t_actual (dejando al menos un elemento en el deque)
        while self.ref[0][0] < t and len(self.ref) > 1:
            self.ref.popleft()

    def merge_ref(self, new_ref): # agrega o reemplaza elementos a la ruta
        t_change = new_ref[0][0]
        if self.ref[-1][0] < t_change: # si el t del ultimo elemento de la ruta anterior es menor al primero de la ruta que se quiere agregar, simplemente se agrega
            self.ref.extend(new_ref)
        else: # si hay solapado entre las rutas, se reemplaza la anterior por la nueva.
            i = 0
            while self.ref[i][0] < t_change:
                i += 1
            for _ in range(i, len(self.ref)):
                self.ref.pop()
            self.ref.extend(new_ref)

    def stop(self): # para frenar rapidamente el flujo (con un soft stop) se detiene el flujo virtual borrando toda la ruta cargada, dejando el flujo en 0
        self.ref = deque([(0,0)])

class VirtualFingers(threading.Thread):
    """
    Esta clase se usa para simular el dedaje en forma virtual.
    Es instanciada dentro del drivers de los dedos, y queda corriendo como un thread, el que actualiza e informa la nota a digitar en tiempo real.
    """
    def __init__(self, running, interval, t0, pipe_end, verbose=False):
        threading.Thread.__init__(self) # Initialize the threading superclass self.running, 0.05, self.t0, self.ref_pipe
        self.running = running
        self.ref = deque([(0,0)]) # la ruta es un deque donde cada elemento es de la forma (tiempo, nota)
        self.lips_surface_ref = deque([(0,31)]) # la ruta es un deque donde cada elemento es de la forma (tiempo, apertura)
        self.tongue_ref = deque([(0,0)]) # la ruta es un deque donde cada elemento es de la forma (tiempo, apertura)
        self.t0 = t0
        self.verbose = verbose
        self.interval = interval
        self.pipe_end = pipe_end
        self.note = 0
        self.aperture = 31
        self.tongue = 0
        
    def run(self):
        while self.running.is_set(): # corre durante toda la ejecución principal
            t = time.perf_counter() - self.t0
            self.note, self.aperture, self.tongue = self.get_ref(t)
            if self.verbose:
                print(t, self.note, self.aperture, self.tongue)
            self.update_ref(t)
            try:
                if self.pipe_end.poll(self.interval):
                    message = self.pipe_end.recv()
                    if message[0] == "get_ref":
                        pos = self.get_ref(message[1])
                    elif message[0] == "update_ref":
                        self.update_ref(message[1])
                    elif message[0] == "merge_ref":
                        self.merge_ref(message[1])
                    elif message[0] == "stop":
                        self.stop()
                    elif message[0] == "merge_lips_surface_ref":
                        self.merge_lips_surface_ref(message[1])
                    elif message[0] == "merge_tongue_ref":
                        self.merge_tongue_ref(message[1])
            except (EOFError, BrokenPipeError, ConnectionResetError, OSError):
                self.stop()
                break

    def get_ref(self, t): # actualiza la nota a digitar para el tiempo actual de acuerdo a la ruta referencia
        if self.ref[-1][0] > t: # si la ruta esta definida hasta un t_final > t_actual
            for i in self.ref:
                if i[0] > t:
                    pos = i[1] # busca el primer elemento de la ruta cuyo tiempo sea mayor al actual. A diferencia de los otros ejes virtuales, las notas son discretas por lo que no es necesario hacer una interpolacion
                    break
        else:
            pos = self.ref[-1][1] # si t_actual > t_final de la ruta referencia la nota se mantiene
        if self.lips_surface_ref[-1][0] > t: # si la ruta esta definida hasta un t_final > t_actual
            for i in self.lips_surface_ref:
                if i[0] > t:
                    apertura = i[1] # busca el primer elemento de la ruta cuyo tiempo sea mayor al actual. A diferencia de los otros ejes virtuales, las notas son discretas por lo que no es necesario hacer una interpolacion
                    break
        else:
            apertura = self.lips_surface_ref[-1][1] # si t_actual > t_final de la ruta referencia la nota se mantiene
        if self.tongue_ref[-1][0] > t: # si la ruta esta definida hasta un t_final > t_actual
            for i in self.tongue_ref:
                if i[0] > t:
                    tongue = i[1] # busca el primer elemento de la ruta cuyo tiempo sea mayor al actual. A diferencia de los otros ejes virtuales, las notas son discretas por lo que no es necesario hacer una interpolacion
                    break
        else:
            tongue = self.tongue_ref[-1][1] # si t_actual > t_final de la ruta referencia la nota se mantiene
        return pos, clamp_int(apertura, 0, 31, 31), clamp_int(tongue, 0, 3, 0)

    def update_ref(self, t): # actualiza la ruta borrando todos los elementos con t < t_actual (dejando al menos un elemento en el deque)
        while self.ref[0][0] < t and len(self.ref) > 1:
            self.ref.popleft()
        while self.lips_surface_ref[0][0] < t and len(self.lips_surface_ref) > 1:
            self.lips_surface_ref.popleft()
        while self.tongue_ref[0][0] < t and len(self.tongue_ref) > 1:
            self.tongue_ref.popleft()

    def merge_ref(self, new_ref): # agrega o reemplaza elementos a la ruta
        t_change = new_ref[0][0]
        if self.ref[-1][0] < t_change:  # si el t del ultimo elemento de la ruta anterior es menor al primero de la ruta que se quiere agregar, simplemente se agrega
            self.ref.extend(new_ref)
        else: # si hay solapado entre las rutas, se reemplaza la anterior por la nueva.
            i = 0
            while self.ref[i][0] < t_change:
                i += 1
            for _ in range(i, len(self.ref)):
                self.ref.pop()
            self.ref.extend(new_ref)

    def merge_lips_surface_ref(self, new_ref): # agrega o reemplaza elementos a la ruta
        new_ref = [(item[0], clamp_int(item[1], 0, 31, 31)) for item in new_ref]
        t_change = new_ref[0][0]
        if self.lips_surface_ref[-1][0] < t_change:  # si el t del ultimo elemento de la ruta anterior es menor al primero de la ruta que se quiere agregar, simplemente se agrega
            self.lips_surface_ref.extend(new_ref)
        else: # si hay solapado entre las rutas, se reemplaza la anterior por la nueva.
            i = 0
            while self.lips_surface_ref[i][0] < t_change:
                i += 1
            for _ in range(i, len(self.lips_surface_ref)):
                self.lips_surface_ref.pop()
            self.lips_surface_ref.extend(new_ref)

    def merge_tongue_ref(self, new_ref):
        new_ref = [(item[0], clamp_int(item[1], 0, 3, 0)) for item in new_ref]
        t_change = new_ref[0][0]
        if self.tongue_ref[-1][0] < t_change:  # si el t del ultimo elemento de la ruta anterior es menor al primero de la ruta que se quiere agregar, simplemente se agrega
            self.tongue_ref.extend(new_ref)
        else: # si hay solapado entre las rutas, se reemplaza la anterior por la nueva.
            i = 0
            while self.tongue_ref[i][0] < t_change:
                i += 1
            for _ in range(i, len(self.tongue_ref)):
                self.tongue_ref.pop()
            self.tongue_ref.extend(new_ref)

    def stop(self): # para eliminar los siguientes cambios de nota, elimina toda la ruta y deja el dedaje en la última nota tocada
        self.ref = deque([(0,self.note)])
        self.lips_surface_ref = deque([(0,31)])
        self.tongue_ref = deque([(0,0)])

class PressureSensor(Process):
    """
    Esta clase interactua con el sensor de presion. Interpreta la informacion leida desde este.
    """
    def __init__(self, hostname, running, musician_pipe, comm_pipe, comm_data, connected=False, verbose=False):
        Process.__init__(self)
        self.running = running
        self.musician_pipe = musician_pipe # pipe que conecta con el musico
        self.comm_pipe = comm_pipe # pipe que conecta con la central de comunicaciones
        self.comm_data = comm_data # data compartida entre procesos
        self.connected = connected
        self.verbose = verbose
        self.pressure = Value("d", 0.0)
        self.hostname = hostname
        self.overpressure_alarm = False  # flag para alarma de sobrepresion

        if self.connected:
            self.comm_pipe.send(["explicit_conn", self.hostname, 0, 4*8, 10, 4, ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, 101, ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, 100])

    def run(self):
        self.musician_pipe.send(["pressure_sensor_started"]) ## avisamos que el driver esta funcionando (para saltar el start-up menu)
        if self.connected:
            self.comm_pipe.send(["registerSession", self.hostname])
            self.comm_pipe.send(["sendFwdOpenReq", self.hostname, 101, 100, 0x6e, 50, 50, ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_LOW])
            
            ## esperamos que se haya registrado la linea de comunicación en la data compartida
            _conn_t0 = time.perf_counter()
            while self.hostname not in " ".join(self.comm_data.keys()):
                if time.perf_counter() - _conn_t0 > SAFETY_CONNECTION_TIMEOUT:
                    logger.error(f"TIMEOUT: No se pudo conectar al sensor de presion {self.hostname}")
                    return
                time.sleep(0.01)

            ## loop en el que se queda el resto del programa
            while self.running.is_set():
                time.sleep(0.01) # tasa de adquisicion de datos
                self.read_input() # leemos el input

            self.comm_pipe.send(["stopProduce", self.hostname, 101, 100, 0x6e]) # cerramos la linea de comunicacion

    def read_input(self): # interpreta el input que entrega el dispositivo
        try:
            # primero transformamos la lista de bools que recibimos (en este formato lo entrega la librería de ethernet ip) en bytes
            words = []
            for w in range(10):
                words.append(int("".join(["1" if self.comm_data[self.hostname + '_in'][i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
            b = bytearray(10)
            struct.pack_into('10B', b, 0, *words)

            # luego interpretamos estos bytes de acuerdo al formato y cantidad de bytes para cada dato
            [g, s, ap] = struct.unpack('<HIf', b)

            self.pressure.value = ap

            # Monitoreo de sobrepresion
            if ap > SAFETY_MAX_PRESSURE and not self.overpressure_alarm:
                logger.critical(f'{self.hostname}: SOBREPRESION DETECTADA: {ap:.1f} kPa > {SAFETY_MAX_PRESSURE} kPa')
                self.overpressure_alarm = True
            elif ap <= SAFETY_MAX_PRESSURE and self.overpressure_alarm:
                logger.info(f'{self.hostname}: Presion volvio a rango normal: {ap:.1f} kPa')
                self.overpressure_alarm = False
        except (KeyError, IndexError) as e:
            logger.warning(f"Error en lectura de sensor de presion (datos no disponibles): {e}")
        except (struct.error, ValueError) as e:
            logger.error(f"Error en decodificacion de datos del sensor de presion: {e}")

# diccionario con la digitación de notas para la flauta traversa. Las notas que no estan es porque no es posible digitarlas con los actuadores disponibles
flute_dict = {#'C3':  '00000 0000',
              #'C#3': '00000 0000',
              'D3':  '11110 1110',
              'D#3': '11110 1111',
              'E3':  '11110 1101',
              'F3':  '11110 1001',
              'F#3': '11110 0011',
              'G3':  '11110 0001',
              'G#3': '11111 0001',
              'A3':  '11100 0001',
              'A#3': '11000 1001',
              'B3':  '11000 0001',
              'C4':  '10000 0001',
              'C#4': '00000 0001',
              'D4':  '01110 1110',
              'D#4': '01110 1111',
              'E4':  '11110 1101',
              'F4':  '11110 1001',
              'F#4': '11110 0011',
              'G4':  '11110 0001',
              'G#4': '11111 0001',
              'A4':  '11100 0001',
              'A#4': '11000 1001',
              'B4':  '11000 0001',
              'C5':  '10000 0001',
              'C#5': '00000 0001',
              'D5':  '01110 0001',
              'D#5': '11111 1111',
              'E5':  '11100 1101',
              'F5':  '11010 1001',
              'F#5': '11010 0011',
              'G5':  '10110 0001',
              'G#5': '00111 0001',
              'A5':  '01100 1001',
              #'A#5': '00000 0000',
              #'B5':  '00000 0000',
              'C6':  '10111 1000'}

# diccionario con la digitación de notas para la quena
quena_dict = {'G3':  '00 1111111',
              'G#3': '00 0000000',
              'A3':  '00 1111110',
              'A#3': '00 0000000',
              'B3':  '00 1111100',
              'C4':  '00 1111000',
              'C#4': '00 0000000',
              'D4':  '00 1110000',
              'D#4': '00 0000000',
              'E4':  '00 1100000',
              'F4':  '00 0010000',
              'F#4': '00 1000000',
              'G4':  '00 0000110',
              'G#4': '00 0000000',
              'A4':  '00 1111110',
              'A#4': '00 0000000',
              'B4':  '00 1111100',
              'C5':  '00 1111000',
              'C#5': '00 0000000',
              'D5':  '00 1110000',
              'D#5': '00 0000000',
              'E5':  '00 1100000',
              'F5':  '00 1000000',
              'F#5': '00 0000000',
              'G5':  '00 0000110',
              'G#5': '00 0111111',
              'A5':  '00 0111110',
              'A#5': '00 0000000',
              'B5':  '00 1100010',
              'C6':  '00 1101111',
              'C#6': '00 1101001',
              'D6':  '00 1010000',
              'D#6': '00 0011110',
              'E6':  '00 0000000',
              'F6':  '00 0100000',
              'F#6': '00 1111100'}

# diccionario de testeo donde solo se activa un actuador a la vez
test_dict = {'1': '000000001',
             '2': '000000010',
             '3': '000000100',
             '4': '000001000',
             '5': '000010000',
             '6': '000100000',
             '7': '001000000',
             '8': '010000000',
             '9': '100000000',
             '0': '000000000'}

# diccionario de diccionarios de digitación, de acuerdo al intrumento seleccionado
instrument_dicts = {'flute': flute_dict,
                    'quena': quena_dict,
                    'test':  test_dict}

class FingersDriver(Process):
    """
    Esta clase interactua con el controlador de los dedos. Define los mensajes a enviar (que actuadores activar).
    """
    def __init__(self, hostname, running, pipe_end, ref_pipe, t0, comm_pipe, connected=True, instrument='flute', verbose=False):
        Process.__init__(self)
        self.running = running
        self.pipe_end = pipe_end # pipe que conecta al musico
        self.ref_pipe = ref_pipe # pipe que conecta a los dedos virtuales
        self.t0 = t0
        self.connected = connected
        self.verbose = verbose
        self.hostname = hostname
        self.comm_pipe = comm_pipe # pipe que conecta a la central de comunicacion
        self.note_played = Array('c', b'D3 ')
        self.tongue_played = Value("i", 0)
        self.lips_played = Value("i", 31)
        # Variables de músico
        self.instrument = instrument #instrumento seleccionado. Define el diccionario a usar y las llaves disponibles
        self.note_dict = instrument_dicts[instrument]
        self.state = '0011111000000000' 

        if self.connected:
            self.comm_pipe.send(["explicit_conn", self.hostname, 2*8, 1*8, 1, 2, ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, 101, ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, 100])
        

    def run(self):

        self.virtual_fingers = VirtualFingers(self.running, 0.05, self.t0, self.ref_pipe, verbose=False)
        self.virtual_fingers.start()
        last_note = -1
        last_aperture = -1
        last_tongue = -1
        self.pipe_end.send(["finger_driver_started"]) ## avisamos que el driver esta funcionando (para saltar el start-up menu)
        
        if self.connected:
            self.comm_pipe.send(["registerSession", self.hostname])
            time.sleep(0.1)
            ## loop en el que queda por el resto del programa
            _fingers_Ts = 0.02  # periodo nominal del loop de dedos
            while self.running.is_set():
                _t_fingers_start = time.perf_counter()
                ref_note = self.virtual_fingers.note # actualiza la nota que debería tocar
                ref_aperture = self.virtual_fingers.aperture
                ref_tongue = self.virtual_fingers.tongue
                ref_aperture = clamp_int(ref_aperture, 0, 31, 31)
                ref_tongue = clamp_int(ref_tongue, 0, 3, 0)
                if ref_note != last_note or ref_aperture != last_aperture or ref_tongue != last_tongue: # si es distinta a la anterior (que ya estaba digitada), envia un mensaje al controlador de los dedos con la nueva digitación para que la actualice
                    # Modifica el estado de servos interno según un diccionario
                    if dict_notes[ref_note] in instrument_dicts[self.instrument].keys():
                        dedos = instrument_dicts[self.instrument][dict_notes[ref_note]] #self.translate_fingers_to_servo(instrument_dicts[self.instrument][req_note])
                        to_send = format(ref_tongue, '02b')[::-1] + format(ref_aperture, '05b')[::-1] + dedos[::-1].replace(' ', '')
                        print(to_send)
                        self.state = int(to_send, 2).to_bytes(2, byteorder='little')
                    else:
                        print(f'Key error: {dict_notes[ref_note]} not in dict')
                    #print(self.state)

                    self.note_played.value = bytes(dict_notes[ref_note].ljust(3, ' '), 'utf-8')
                    self.tongue_played.value = ref_tongue
                    self.lips_played.value = ref_aperture
                    self.comm_pipe.send(["setAttrSingle", self.hostname, 0x04, 100, 0x03, self.state]) # a diferencia de los otros drivers, como los cambios de nota son menos frecuentes, se usa la mensajería explicita
                    last_note = ref_note
                    last_aperture = ref_aperture
                    last_tongue = ref_tongue

                # Spin-wait híbrido
                _elapsed = time.perf_counter() - _t_fingers_start
                _remaining = _fingers_Ts - _elapsed
                if _remaining > 0.002:
                    time.sleep(_remaining - 0.001)
                while time.perf_counter() - _t_fingers_start < _fingers_Ts:
                    pass

            # antes de cerrar el programa se abren todas las llaves para acceder más facil a la flauta en caso de que quiera ser removida. Tambien funciona por seguridad para que no se gasten las gomas de las llaves si se dejan mucho tiempo cerradas
        
            self.state = int('0011111000000000', 2).to_bytes(2, byteorder='little')
            self.comm_pipe.send(["setAttrSingle", self.hostname, 0x04, 100, 0x03, self.state])

            time.sleep(0.1)
            print('Fingers Driver thread killed')

    def request_finger_action(self, req_note: str):
        """
        Función para llamar desde FingersController
        :param req_note: string indicando la nota que se desea.
        """

        # Modifica el estado de servos interno según un diccionario
        if req_note in instrument_dicts[self.instrument].keys():
            servo = instrument_dicts[self.instrument][req_note] #self.translate_fingers_to_servo(instrument_dicts[self.instrument][req_note])
            self.state = int(servo[::-1].replace(' ', ''), 2).to_bytes(2, byteorder='little')
        else:
            print(f'Key error: {req_note} not in dict')

    def translate_fingers_to_servo(self, note_bits):
        """
        Intercambia las llaves 4 y 5 por disposición geométrica.
        - llave nueva 4 <-- llave antigua 5.
        - llave nueva 5 <-- llave antigua 4.
        :param note_bits:
        :return:
        """
        servo_bits = list(note_bits)
        servo_bits[3] = note_bits[4]
        servo_bits[4] = note_bits[3]

        return ''.join(servo_bits)

class Microphone(Process):
    """
    Lee, filtra, interpreta y graba la informacion recibida desde el microfono
    """
    pad_modes = ["constant", "edge", "empty", "linear_ramp", "maximum", "mean", "median", "minimum", "reflect", "symmetric", "wrap"]
    # opciones de pad para la deteccion de pitch con algoritmo YIN o pYIN
    def __init__(self, running, end_pipe, device, method, yin_settings, pyin_settings, mic_running, connected=False, verbose=False):
        Process.__init__(self)
        self.running = running
        self.connected = connected
        self.verbose = verbose
        self.device = device # microfono a usar. Puede ser cambiado por el usuario
        self.method = method # metodo de deteccion de pitch a usar: YIN o pYIN
        self.yin_settings = yin_settings
        self.pyin_settings = pyin_settings
        self.end_pipe = end_pipe # pipe que conecta con el musico
        self.pitch = Value('d', 0.0)
        self.sr = 44100 # sample rate
        self.max_num_points = int(self.sr*0.1) # puntos que usa para la detección de pitch. 
        self.last_mic_data = np.array([]) # array con la info escuchada mas recientemente, donde se mantienen self.max_num_points valores
        self.saving = False # bool para grabar 
        self.next_saving = False
        # self.mic_data = np.array([])
        self.mic_running = mic_running
        self.mic_running.set()
        self.buffer = io.BytesIO() # buffer para guardar el audio grabado

    def micCallback(self, indata, frames, time, status): # se llama esta funcion en cada actualización de la informacion escuchada por el microfono
        if status:
            print('Status:', status)
        # actualizamos la ultima informacion escuchada, truncuando los ultimos self.max_num_points puntos
        self.last_mic_data = np.hstack((self.last_mic_data, np.transpose(indata)[0]))
        self.last_mic_data = self.last_mic_data[-self.max_num_points:]
        if self.saving: # si estamos grabando escribimos la info nueva en el buffer
            if not self.next_saving:
                self.saving = False
            self.buffer.write(np.transpose(indata)[0].copy())

    def start_saving(self): # para empezar a grabar
        # self.mic_data = np.array([])
        self.saving = True
        self.next_saving = True
    
    def pause_saving(self): # para pausar la grabacion
        self.next_saving = False

    def resume_saving(self): # para resumir la grabacion
        self.saving = True
        self.next_saving = True
    
    def finish_saving(self, file_name): # para finalizar la grabacion y guardar el archivo
        self.next_saving = False
        while self.saving:
            pass
        self.buffer.seek(0)
        deserialized_bytes = np.frombuffer(self.buffer.read(), dtype=np.float32)
        write(file_name, self.sr, deserialized_bytes)
        self.buffer.seek(0)
        self.buffer.truncate(0) # reseteamos el buffer
        

    def handle_messages(self):
        if self.end_pipe.poll(0.05): # revisa si hay alguna instruccion de otro proceso. Si no recibe nada en 0.05 ms avanza
            message = self.end_pipe.recv()
            # print("Message received in microphone", message)
            if message[0] == 'start_saving': # para empezar a grabar
                self.start_saving()
            elif message[0] == 'pause_saving':
                self.pause_saving()
            elif message[0] == 'stop_recording':
                self.pause_saving()
            elif message[0] == 'resume_saving':
                self.resume_saving()
            elif message[0] == 'save_recorded_data':
                self.finish_saving(message[1])
                print("Audio saved to file", message[1])
            elif message[0] == 'change_frequency_detection': # para hacer cambios en la configuracion. Desde un cambio de microfono a un cambio en la configuracion de la deteccion de pitch
                if self.device != message[1]['device']:
                    self.mic_running.clear()
                self.device = message[1]['device']
                self.method = message[1]['method']
                self.yin_settings = message[1]['YIN']
                self.pyin_settings = message[1]['pYIN']

    def detect_pitch(self):
        pitch = 0
        if self.method == 0: # si se usa el metodo YIN
            try:
                pitch = yin(self.last_mic_data, sr=self.sr, fmin=self.yin_settings['fmin'], frame_length=self.yin_settings['frame_length'], fmax=self.yin_settings['fmax'], trough_threshold=self.yin_settings['trough_threshold'], center=self.yin_settings['center'], hop_length=self.yin_settings['hop_length'], win_length=self.yin_settings['win_length'], pad_mode=Microphone.pad_modes[self.yin_settings['pad_mode']])[-1] 
            except Exception as e:
                logger.debug(f"Pitch detection YIN fallo: {e}")
                pitch = 0 # si falla la deteccion se dice que el pitch es 0
        else:  # si se usa el metodo pYIN
            try:
                if self.pyin_settings['fill_na'] == 0:
                    pitch = pyin(self.last_mic_data, sr=self.sr, fmin=self.pyin_settings['fmin'], fmax=self.pyin_settings['fmax'], frame_length=self.pyin_settings['frame_length'], win_length=self.pyin_settings['win_length'], hop_length=self.pyin_settings['hop_length'], n_thresholds=self.pyin_settings['n_threshold'], beta_parameters=(self.pyin_settings['beta_parameter_a'], self.pyin_settings['beta_parameter_b']), boltzmann_parameter=self.pyin_settings['boltzmann_parameter'], resolution=self.pyin_settings['resolution'], max_transition_rate=self.pyin_settings['max_transition_rate'], switch_prob=self.pyin_settings['switch_prob'], no_trough_prob=self.pyin_settings['no_trough_prob'], fill_na=None, center=self.pyin_settings['center'], pad_mode=Microphone.pad_modes[self.pyin_settings['pad_mode']])[0][-1]
                elif self.pyin_settings['fill_na'] == 1:
                    pitch = pyin(self.last_mic_data, sr=self.sr, fmin=self.pyin_settings['fmin'], fmax=self.pyin_settings['fmax'], frame_length=self.pyin_settings['frame_length'], win_length=self.pyin_settings['win_length'], hop_length=self.pyin_settings['hop_length'], n_thresholds=self.pyin_settings['n_threshold'], beta_parameters=(self.pyin_settings['beta_parameter_a'], self.pyin_settings['beta_parameter_b']), boltzmann_parameter=self.pyin_settings['boltzmann_parameter'], resolution=self.pyin_settings['resolution'], max_transition_rate=self.pyin_settings['max_transition_rate'], switch_prob=self.pyin_settings['switch_prob'], no_trough_prob=self.pyin_settings['no_trough_prob'], fill_na=self.pyin_settings['fill_na_float'], center=self.pyin_settings['center'], pad_mode=Microphone.pad_modes[self.pyin_settings['pad_mode']])[0][-1]
                elif self.pyin_settings['fill_na'] == 2:
                    pitch = pyin(self.last_mic_data, sr=self.sr, fmin=self.pyin_settings['fmin'], fmax=self.pyin_settings['fmax'], frame_length=self.pyin_settings['frame_length'], win_length=self.pyin_settings['win_length'], hop_length=self.pyin_settings['hop_length'], n_thresholds=self.pyin_settings['n_threshold'], beta_parameters=(self.pyin_settings['beta_parameter_a'], self.pyin_settings['beta_parameter_b']), boltzmann_parameter=self.pyin_settings['boltzmann_parameter'], resolution=self.pyin_settings['resolution'], max_transition_rate=self.pyin_settings['max_transition_rate'], switch_prob=self.pyin_settings['switch_prob'], no_trough_prob=self.pyin_settings['no_trough_prob'], fill_na=np.nan, center=self.pyin_settings['center'], pad_mode=Microphone.pad_modes[self.pyin_settings['pad_mode']])[0][-1]
            except Exception as e:
                logger.debug(f"Pitch detection pYIN fallo: {e}")
                pitch = 0

        self.pitch.value = pitch

    def run(self):
        self.end_pipe.send(["microphone_started"]) ## avisamos que el driver esta funcionando (para saltar el start-up menu)
        if self.connected:
            #print(sd.query_devices())
            while self.running.is_set(): # se crean dos loops, con dos eventos. self.running es el de mayor jerarquia, que se mantiene durante toda la duracion del programa. self.mic_running en cambio se mantiene mientras no haya cambios de microfono. Asi si el usuario decide usar un microfono distinto es posible cambiarlo sin tener que reiniciar el programa.
                self.mic_running.set()
                with sd.InputStream(samplerate=self.sr, channels=1, callback=self.micCallback, device=self.device, latency='high'):#,  blocksize=300000): #, latency='high'
                    while self.mic_running.is_set(): # se mantiene en este loop mientras no cambie el microfono o se cierre la aplicacion
                        self.handle_messages() # revisa si hay alguna instruccion de otro proceso
                        self.detect_pitch() # hace la deteccion de pitch

                
            print("Mic thread killed")

if __name__ == "__main__":
    pass
