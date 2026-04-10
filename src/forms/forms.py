from PyQt5.QtWidgets import QDialog, QLabel, QCheckBox, QHBoxLayout, QWidget, QGridLayout, QTableWidget, QTableWidgetItem, QPushButton, QVBoxLayout
from PyQt5.QtCore import QEventLoop, Qt
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets

from PyQt5.QtWidgets import QMessageBox
from PyQt5 import QtWidgets
from PyQt5.QtGui import QPixmap

from src.forms.point import Ui_Dialog as PointFormDialog
from src.forms.vibrato import Ui_Dialog as VibratoFormDialog
from src.forms.filter import Ui_Dialog as FilterFormDialog
from src.forms.func_table import Ui_Dialog as FuncTableDialog
from src.forms.notes import Ui_Dialog as NotesFormDialog
from src.forms.duration import Ui_Dialog as DurationFormDialog
from src.forms.correction import Ui_Dialog as CorrectionFormDialog
from src.forms.scale_time import Ui_Dialog as ScaleTimeFormDialog
from src.forms.settings import Ui_Dialog as SettingsFormDialog
from src.forms.trill import Ui_Dialog as TrillFormDialog
from src.forms.states_from_notes import Ui_Dialog as StatesFromNotesDialog
from src.forms.surfacePointForm import Ui_Dialog as SurfacePointDialog
from src.forms.tonguePointForm import Ui_Dialog as TonguePointDialog
from src.forms.DynamicPintForm import Ui_Dialog as DynamicPointDialog
from src.route import dict_notes, dict_notes_rev
from functools import partial
from src.cinematica import *
from src.route import dict_notes
from sounddevice import query_hostapis, query_devices

class SettingsForm(QDialog, SettingsFormDialog):
    """
    Formulario para configurar los ajustes del robot. Se accede a este formulario desde la ventana principal > File > Settings
    """
    def __init__(self, parent=None, data=[0 for i in range(34)]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data
        self.setAllValues()
        self.connectAllSignals()

    def get_available_mics(self): # revisa los microfonos disponibles por sistema y retorna una lista de strings que los representa
        devices = query_devices()
        hostapis = query_hostapis()
        l = []
        for d in devices:
            i = d['index']
            n = d['name']
            h = hostapis[d['hostapi']]['name']
            inp = d['max_input_channels']
            out = d['max_output_channels']
            l.append(f'{i} {n}, {h} ({inp} in, {out} out)')
        return l

    def setAllValues(self): # lee los ajustes guardados en DATA y pre-rellena el formulario con sus valores actuales
        global DATA
        self.xFlutePos.setValue(DATA["flute_position"]["X_F"])
        self.zFlutePos.setValue(DATA["flute_position"]["Z_F"])
        pixmap = QPixmap('src/forms/flute_pos2.png')
        self.image_flute_pos.setPixmap(pixmap)

        self.micDevices.addItems(self.get_available_mics())
        self.micDevices.setCurrentIndex(DATA["frequency_detection"]["device"])
        self.frequencyDetectionMethod.setCurrentIndex(DATA["frequency_detection"]["method"])
        self.YINfmin.setValue(DATA["frequency_detection"]["YIN"]["fmin"])
        self.YINfmax.setValue(DATA["frequency_detection"]["YIN"]["fmax"])
        self.YINframe_length.setValue(DATA["frequency_detection"]["YIN"]["frame_length"])
        self.YINwin_length.setValue(DATA["frequency_detection"]["YIN"]["win_length"])
        self.YINhop_length.setValue(DATA["frequency_detection"]["YIN"]["hop_length"])
        self.YINtrough_threshold.setValue(DATA["frequency_detection"]["YIN"]["trough_threshold"])
        self.YINcenter.setChecked(DATA["frequency_detection"]["YIN"]["center"])
        self.YINpad_mode.setCurrentIndex(DATA["frequency_detection"]["YIN"]["pad_mode"])
        self.pYINfmin.setValue(DATA["frequency_detection"]["pYIN"]["fmin"])
        self.pYINfmax.setValue(DATA["frequency_detection"]["pYIN"]["fmax"])
        self.pYINframe_length.setValue(DATA["frequency_detection"]["pYIN"]["frame_length"])
        self.pYINwin_length.setValue(DATA["frequency_detection"]["pYIN"]["win_length"])
        self.pYINhop_length.setValue(DATA["frequency_detection"]["pYIN"]["hop_length"])
        self.pYINn_thresholds.setValue(DATA["frequency_detection"]["pYIN"]["n_threshold"])
        self.pYINbeta_parameter_a.setValue(DATA["frequency_detection"]["pYIN"]["beta_parameter_a"])
        self.pYINbeta_parameter_b.setValue(DATA["frequency_detection"]["pYIN"]["beta_parameter_b"])
        self.pYINcenter.setChecked(DATA["frequency_detection"]["pYIN"]["center"])
        self.pYINmax_transition_rate.setValue(DATA["frequency_detection"]["pYIN"]["max_transition_rate"])
        self.pYINresolution.setValue(DATA["frequency_detection"]["pYIN"]["resolution"])
        self.pYINboltzmann_parameter.setValue(DATA["frequency_detection"]["pYIN"]["boltzmann_parameter"])
        self.pYINswitch_prob.setValue(DATA["frequency_detection"]["pYIN"]["switch_prob"])
        self.pYINno_trough_prob.setValue(DATA["frequency_detection"]["pYIN"]["no_trough_prob"])
        self.pYINfill_na.setCurrentIndex(DATA["frequency_detection"]["pYIN"]["fill_na"])
        self.pYINfill_na_float.setValue(DATA["frequency_detection"]["pYIN"]["fill_na_float"])
        self.pYINpad_mode.setCurrentIndex(DATA["frequency_detection"]["pYIN"]["pad_mode"])
        if DATA["frequency_detection"]["method"] == 0: # solo se muestran las configuraciones para el metodo de pitch seleccionado
            self.pYINGroupBox.hide()
        else:
            self.YINGroupBox.hide()

        self.flowVarToControl.setCurrentIndex(DATA["flow_control"]["var_to_control"])
        self.flowControlLoop.setCurrentIndex(DATA["flow_control"]["control_loop"])
        self.flowKp.setValue(DATA["flow_control"]["kp"])
        self.flowKi.setValue(DATA["flow_control"]["ki"])
        self.flowKd.setValue(DATA["flow_control"]["kd"])

        self.X_kp_value.setValue(DATA["x_control"]["kp"])
        self.X_ki_value.setValue(DATA["x_control"]["ki"])
        self.X_kd_value.setValue(DATA["x_control"]["kd"])
        self.X_acc_value.setValue(DATA["x_control"]["acceleration"])
        self.X_dec_value.setValue(DATA["x_control"]["deceleration"])
        self.X_prop_value.setValue(DATA["x_control"]["proportional_coef"])
        self.X_kp_vel_value.setValue(DATA["x_control"]["kp_vel"])
        self.X_ki_vel_value.setValue(DATA["x_control"]["ki_vel"])
        self.X_kd_vel_value.setValue(DATA["x_control"]["kd_vel"])
        pixmap = QPixmap('src/forms/control_motores.drawio.png')
        self.control_image.setPixmap(pixmap)
        self.control_image2.setPixmap(pixmap)
        self.control_image3.setPixmap(pixmap)

        self.Z_kp_value.setValue(DATA["z_control"]["kp"])
        self.Z_ki_value.setValue(DATA["z_control"]["ki"])
        self.Z_kd_value.setValue(DATA["z_control"]["kd"])
        self.Z_acc_value.setValue(DATA["z_control"]["acceleration"])
        self.Z_dec_value.setValue(DATA["z_control"]["deceleration"])
        self.Z_prop_value.setValue(DATA["z_control"]["proportional_coef"])
        self.Z_kp_vel_value.setValue(DATA["z_control"]["kp_vel"])
        self.Z_ki_vel_value.setValue(DATA["z_control"]["ki_vel"])
        self.Z_kd_vel_value.setValue(DATA["z_control"]["kd_vel"])

        self.A_kp_value.setValue(DATA["alpha_control"]["kp"])
        self.A_ki_value.setValue(DATA["alpha_control"]["ki"])
        self.A_kd_value.setValue(DATA["alpha_control"]["kd"])
        self.A_acc_value.setValue(DATA["alpha_control"]["acceleration"])
        self.A_dec_value.setValue(DATA["alpha_control"]["deceleration"])
        self.A_prop_value.setValue(DATA["alpha_control"]["proportional_coef"])
        self.A_kp_vel_value.setValue(DATA["alpha_control"]["kp_vel"])
        self.A_ki_vel_value.setValue(DATA["alpha_control"]["ki_vel"])
        self.A_kd_vel_value.setValue(DATA["alpha_control"]["kd_vel"])
        
    def connectAllSignals(self): # conecta los cambios de cualquier campo con una funcion que actualiza DATA
        self.xFlutePos.valueChanged.connect(partial(self.update_data, ["flute_position", "X_F"]))
        self.zFlutePos.valueChanged.connect(partial(self.update_data, ["flute_position", "Z_F"]))

        self.frequencyDetectionMethod.currentIndexChanged.connect(partial(self.update_data, ["frequency_detection", "method"]))
        self.micDevices.currentIndexChanged.connect(partial(self.update_data, ["frequency_detection", "device"]))
        self.YINfmin.valueChanged.connect(partial(self.update_data, ["frequency_detection", "YIN", "fmin"]))
        self.YINfmax.valueChanged.connect(partial(self.update_data, ["frequency_detection", "YIN", "fmax"]))
        self.YINframe_length.valueChanged.connect(partial(self.update_data, ["frequency_detection", "YIN", "frame_length"]))
        self.YINwin_length.valueChanged.connect(partial(self.update_data, ["frequency_detection", "YIN", "win_length"]))
        self.YINhop_length.valueChanged.connect(partial(self.update_data, ["frequency_detection", "YIN", "hop_length"]))
        self.YINtrough_threshold.valueChanged.connect(partial(self.update_data, ["frequency_detection", "YIN", "trough_threshold"]))
        self.YINcenter.stateChanged.connect(partial(self.update_data, ["frequency_detection", "YIN", "center"]))
        self.YINpad_mode.currentIndexChanged.connect(partial(self.update_data, ["frequency_detection", "YIN", "pad_mode"]))
        self.pYINfmin.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "fmin"]))
        self.pYINfmax.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "fmax"]))
        self.pYINframe_length.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "frame_length"]))
        self.pYINwin_length.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "win_length"]))
        self.pYINhop_length.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "hop_length"]))
        self.pYINn_thresholds.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "n_threshold"]))
        self.pYINbeta_parameter_a.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "beta_parameter_a"]))
        self.pYINbeta_parameter_b.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "beta_parameter_b"]))
        self.pYINcenter.stateChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "center"]))
        self.pYINmax_transition_rate.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "max_transition_rate"]))
        self.pYINresolution.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "resolution"]))
        self.pYINboltzmann_parameter.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "boltzmann_parameter"]))
        self.pYINswitch_prob.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "switch_prob"]))
        self.pYINno_trough_prob.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "no_trough_prob"]))
        self.pYINfill_na.currentIndexChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "fill_na"]))
        self.pYINfill_na_float.valueChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "fill_na_float"]))
        self.pYINpad_mode.currentIndexChanged.connect(partial(self.update_data, ["frequency_detection", "pYIN", "pad_mode"]))

        self.flowVarToControl.currentIndexChanged.connect(partial(self.update_data, ["flow_control", "var_to_control"]))
        self.flowControlLoop.currentIndexChanged.connect(partial(self.update_data, ["flow_control", "control_loop"]))
        self.flowKp.valueChanged.connect(partial(self.update_data, ["flow_control", "kp"]))
        self.flowKi.valueChanged.connect(partial(self.update_data, ["flow_control", "ki"]))
        self.flowKd.valueChanged.connect(partial(self.update_data, ["flow_control", "kd"]))

        self.X_kp_value.valueChanged.connect(partial(self.update_data, ["x_control", "kp"]))
        self.X_ki_value.valueChanged.connect(partial(self.update_data, ["x_control", "ki"]))
        self.X_kd_value.valueChanged.connect(partial(self.update_data, ["x_control", "kd"]))
        self.X_acc_value.valueChanged.connect(partial(self.update_data, ["x_control", "acceleration"]))
        self.X_dec_value.valueChanged.connect(partial(self.update_data, ["x_control", "deceleration"]))
        self.X_prop_value.valueChanged.connect(partial(self.update_data, ["x_control", "proportional_coef"]))
        self.X_kp_vel_value.valueChanged.connect(partial(self.update_data, ["x_control", "kp_vel"]))
        self.X_ki_vel_value.valueChanged.connect(partial(self.update_data, ["x_control", "ki_vel"]))
        self.X_kd_vel_value.valueChanged.connect(partial(self.update_data, ["x_control", "kd_vel"]))

        self.Z_kp_value.valueChanged.connect(partial(self.update_data, ["z_control", "kp"]))
        self.Z_ki_value.valueChanged.connect(partial(self.update_data, ["z_control", "ki"]))
        self.Z_kd_value.valueChanged.connect(partial(self.update_data, ["z_control", "kd"]))
        self.Z_acc_value.valueChanged.connect(partial(self.update_data, ["z_control", "acceleration"]))
        self.Z_dec_value.valueChanged.connect(partial(self.update_data, ["z_control", "deceleration"]))
        self.Z_prop_value.valueChanged.connect(partial(self.update_data, ["z_control", "proportional_coef"]))
        self.Z_kp_vel_value.valueChanged.connect(partial(self.update_data, ["z_control", "kp_vel"]))
        self.Z_ki_vel_value.valueChanged.connect(partial(self.update_data, ["z_control", "ki_vel"]))
        self.Z_kd_vel_value.valueChanged.connect(partial(self.update_data, ["z_control", "kd_vel"]))

        self.A_kp_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "kp"]))
        self.A_ki_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "ki"]))
        self.A_kd_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "kd"]))
        self.A_acc_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "acceleration"]))
        self.A_dec_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "deceleration"]))
        self.A_prop_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "proportional_coef"]))
        self.A_kp_vel_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "kp_vel"]))
        self.A_ki_vel_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "ki_vel"]))
        self.A_kd_vel_value.valueChanged.connect(partial(self.update_data, ["alpha_control", "kd_vel"]))

        # tambien conectamos los botones del grupo de botones y el de store settings
        self.buttonBox.clicked.connect(self.button_clicked) 
        self.storeSettings.clicked.connect(self.store_settings)

    def update_data(self, index, value): # actualiza el cambios en algun campo, que se indica con la llave index. El valor nuevo esta en value
        global DATA
        if index == ["frequency_detection", "method"]: # si se cambia el metodo de deteccion de pitch hay que actualizar el formulario
            if value: # se elige pYIN
                self.YINGroupBox.hide()
                self.pYINGroupBox.show()
            else: # se elige YIN
                self.pYINGroupBox.hide()
                self.YINGroupBox.show()

        # index es una lista que puede tener largos 1, 2 o 3
        if len(index) == 1:
            DATA[index[0]] = value
            print(DATA[index[0]])
        elif len(index) == 2:
            DATA[index[0]][index[1]] = value
            print(DATA[index[0]][index[1]])
        elif len(index) == 3:
            DATA[index[0]][index[1]][index[2]] = value
            print(DATA[index[0]][index[1]][index[2]])
    
    def button_clicked(self, button): # aplica los cambios al apretar botones del grupo de botones
        # el grupo de botones tiene 4 botones:
        # OK: aplica los cambios y cierra el formulario
        # Cancel: cierra el formulario sin aplicar los cambios
        # Apply: aplica los cambios pero no cierra el formulario
        # Restore default: vuelve a las configuraciones guardadas
        global DATA
        if button.text() == 'Apply':
            self.parent.refresh_settings()
        elif button.text() == 'Restore Defaults':
            DATA = read_variables() # volvemos a leer el archivo settings.json
            self.setAllValues()

    def store_settings(self): # guarda las configuraciones actuales en settings.json
        global DATA, DATA_dir
        with open(DATA_dir, 'w') as json_file:
            json.dump(DATA, json_file, indent=4, sort_keys=True)

class PointForm(QDialog, PointFormDialog):
    """
    Formulario para crear o editar un punto que se añadirá a una de las curvas para la trayectoria.
    Debe ajustarse tiempo y valor
    """
    def __init__(self, parent=None, data=[0,0], max_t=100, min_v=0, max_v=100):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # ponemos valores iniciales y fijamos limites
        self.time.setValue(data[0])
        self.time.setMaximum(max_t)
        
        self.value.setMaximum(max_v)
        self.value.setMinimum(min_v)
        self.value.setValue(data[1])

        # conectamos los cambios en los campos con update_data
        self.time.valueChanged.connect(partial(self.update_data, 'time'))
        self.value.valueChanged.connect(partial(self.update_data, 'value'))

    def update_data(self, tag, value):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'time':
            self.data[0] = value
        elif tag == 'value':
            self.data[1] = value

class DynamicForm(QDialog, DynamicPointDialog):
    def __init__(self, parent=None, dynamic_list=[], data=[0,0], max_t=100, min_v=0, max_v=100):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # ponemos valores iniciales y fijamos limites
        self.timeInput.setValue(data[0])
        self.timeInput.setMaximum(max_t)
        
        self.dynamicInput.addItems(dynamic_list)
        self.dynamicInput.setCurrentIndex(data[1])

        # conectamos los cambios en los campos con update_data
        self.timeInput.valueChanged.connect(partial(self.update_data, 'time'))
        self.dynamicInput.currentIndexChanged.connect(partial(self.update_data, 'value'))

    def update_data(self, tag, value):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'time':
            self.data[0] = value
        elif tag == 'value':
            self.data[1] = value

class SurfacePointForm(QDialog, SurfacePointDialog):
    """
    Formulario para crear o editar un punto que se añadirá a una de las curvas para la trayectoria de la superficie de los labios.
    Debe ajustarse tiempo y valor
    """
    def __init__(self, parent=None, data=[0,0], max_t=100):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # ponemos valores iniciales y fijamos limites
        self.time.setValue(data[0])
        self.time.setMaximum(max_t)
        
        self.value.setValue(data[1])

        # conectamos los cambios en los campos con update_data
        self.time.valueChanged.connect(partial(self.update_data, 'time'))
        self.value.valueChanged.connect(partial(self.update_data, 'value'))

    def update_data(self, tag, value):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'time':
            self.data[0] = value
        elif tag == 'value':
            self.data[1] = value
    
class TonguePointForm(QDialog, TonguePointDialog):
    """
    Formulario para crear o editar un punto que se añadirá a una de las curvas para la trayectoria de la superficie de la lengua.
    Debe ajustarse tiempo y valor
    """
    def __init__(self, parent=None, data=[0,0], max_t=100):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # ponemos valores iniciales y fijamos limites
        self.time.setValue(data[0])
        self.time.setMaximum(max_t)
        
        self.value.setValue(data[1])

        # conectamos los cambios en los campos con update_data
        self.time.valueChanged.connect(partial(self.update_data, 'time'))
        self.value.valueChanged.connect(partial(self.update_data, 'value'))

    def update_data(self, tag, value):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'time':
            self.data[0] = value
        elif tag == 'value':
            self.data[1] = value

class TrillForm(QDialog, TrillFormDialog):
    """
    Formulario para crear o editar un elemento trill que se añadirá a la curvas de las notas.
    Debe ajustarse tiempo de inicio, duracion, distancia y frecuencia
    """
    def __init__(self, parent=None, data=[0,0,0,0], max_t=100):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # ponemos valores iniciales y fijamos limites
        self.time.setValue(data[0])
        self.time.setMaximum(max_t)

        self.distance.setValue(int(data[1]))
        self.duration.setMaximum(max_t-data[0])
        
        self.frequency.setValue(data[2])
        self.duration.setValue(data[3])
        
        # conectamos los cambios en los campos con update_data
        self.time.valueChanged.connect(partial(self.update_data, 'time'))
        self.distance.valueChanged.connect(partial(self.update_data, 'distance'))
        self.frequency.valueChanged.connect(partial(self.update_data, 'frequency'))
        self.duration.valueChanged.connect(partial(self.update_data, 'duration'))

    def update_data(self, tag, value):
        if tag == 'time':
            self.data[0] = value
        elif tag == 'distance':
            self.data[1] = value
        elif tag == 'frequency':
            self.data[2] = value
        elif tag == 'duration':
            self.data[3] = value

class ScaleTimeForm(QDialog, ScaleTimeFormDialog):
    """
    Formulario para escalar el tiempo de una trayectoria.
    Debe ajustarse el factor por el que se quiere escalar
    """
    def __init__(self, parent=None, data=[0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # fijamos un valor inicial
        self.scaleFactor.setValue(data[0])
        # y conectamos el cambio
        self.scaleFactor.valueChanged.connect(self.update_data)

    def update_data(self, value):
        # actualiza self.data con los valores ingresados por el usuario
        self.data[0] = value

class CorrectionForm(QDialog, CorrectionFormDialog):
    """
    Formulario para añadir correcciones en los ejes del tiempo y valor para cualquiera de las curvas de la trayectoria
    Debe ajustarse los delta t y los delta y en cada una de las 5 curvas
    """
    def __init__(self, parent=None, data=[0,0,0,0,0,0,0,0,0,0], space=0):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data
        self.space = space # espacio de la tarea o de las junturas. Cambian los labels
        
        # ponemos valores iniciales
        self.r_dis.setValue(data[0])
        self.theta_dis.setValue(data[1])
        self.offset_dis.setValue(data[2])
        self.flow_dis.setValue(data[3])
        self.notes_dis.setValue(data[4])

        self.leadDelayR.setValue(data[5])
        self.leadDelayTheta.setValue(data[6])
        self.leadDelayOffset.setValue(data[7])
        self.leadDelayFlow.setValue(data[8])
        self.leadDelayNotes.setValue(data[9])

        if self.space == 1: # espacio de las junturas
            self.label.setText("X (mm)")
            self.label_2.setText("Z (mm)")
            self.label_3.setText("Alpha (°)")
            self.label_6.setText("Lead or delay X (s)")
            self.label_7.setText("Lead or delay Z (s)")
            self.label_8.setText("Lead or delay Alpha (s)")

        # conectamos los cambios en los campos con update_data
        self.r_dis.valueChanged.connect(partial(self.update_data, 'r'))
        self.theta_dis.valueChanged.connect(partial(self.update_data, 'theta'))
        self.offset_dis.valueChanged.connect(partial(self.update_data, 'offset'))
        self.flow_dis.valueChanged.connect(partial(self.update_data, 'flow'))
        self.notes_dis.valueChanged.connect(partial(self.update_data, 'notes'))

        self.leadDelayR.valueChanged.connect(partial(self.update_data, 'leadDelayR'))
        self.leadDelayTheta.valueChanged.connect(partial(self.update_data, 'leadDelayTheta'))
        self.leadDelayOffset.valueChanged.connect(partial(self.update_data, 'leadDelayOffset'))
        self.leadDelayFlow.valueChanged.connect(partial(self.update_data, 'leadDelayFlow'))
        self.leadDelayNotes.valueChanged.connect(partial(self.update_data, 'leadDelayNotes'))

    def update_data(self, tag, value):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'r':
            self.data[0] = value
        elif tag == 'theta':
            self.data[1] = value
        elif tag == 'offset':
            self.data[2] = value
        elif tag == 'flow':
            self.data[3] = value
        elif tag == 'notes':
            self.data[4] = value
        elif tag == 'leadDelayR':
            self.data[5] = value
        elif tag == 'leadDelayTheta':
            self.data[6] = value
        elif tag == 'leadDelayOffset':
            self.data[7] = value
        elif tag == 'leadDelayFlow':
            self.data[8] = value
        elif tag == 'leadDelayNotes':
            self.data[9] = value

class DurationForm(QDialog, DurationFormDialog):
    """
    Formulario para cambiar la duracion total de la trayectoria
    Debe ajustarse el nuevo tiempo total
    """
    def __init__(self, parent=None, data=[0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # ponemos valor inicial y conectamos con la funcion que actualiza self.data
        self.time.setValue(data[0])
        self.time.valueChanged.connect(self.change_time)

    def change_time(self, value):
        # actualiza self.data con los valores ingresados por el usuario
        self.data[0] = value

class NoteForm(QDialog, NotesFormDialog):
    """
    Formulario para crear o editar una nota que se añadirá a la curvas de las notas para la trayectoria.
    Debe ajustarse tiempo y valor
    """
    def __init__(self, parent=None, data=[0,0], max_t=100):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # ponemos valores iniciales y fijamos limites
        self.time.setValue(data[0])
        self.time.setMaximum(max_t)
        self.note_choice.addItems(list(dict_notes.values()))
        self.note_choice.setCurrentIndex(data[1])

        # conectamos los cambios en los campos con update_data
        self.time.valueChanged.connect(partial(self.update_data, 'time'))
        self.note_choice.currentIndexChanged.connect(partial(self.update_data, 'value'))

    def update_data(self, tag, value):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'time':
            self.data[0] = value
        elif tag == 'value':
            self.data[1] = value

windows_vibrato = ['rect', 'triangular', 'blackman', 'hamming', 'hanning', 'kaiser1', 'kaiser2', 'kaiser3', 'kaiser4', 'ramp', 'reversed_ramp']
class VibratoForm(QDialog, VibratoFormDialog):
    """
    Formulario para crear o editar un elemento de vibrato que se añadirá a una de las curvas para la trayectoria.
    Debe ajustarse tiempo de inicio, duracion, aplitud, frecuencia y ventana por la que se multiplicará
    """
    def __init__(self, parent=None, data=[0,0,0,0,0], max_t=100):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data
        self.max_t = max_t

        # ponemos valores iniciales y fijamos limites
        self.time_i.setValue(data[0])
        self.duration.setValue(data[1])
        self.duration.setMaximum(max_t - data[0])
        self.amp.setValue(data[2])
        self.freq.setValue(data[3])
        self.window_v.setCurrentIndex(data[4])

        # conectamos los cambios en los campos con update_data
        self.time_i.valueChanged.connect(partial(self.update_data, 'time_i'))
        self.duration.valueChanged.connect(partial(self.update_data, 'duration'))
        self.amp.valueChanged.connect(partial(self.update_data, 'amp'))
        self.freq.valueChanged.connect(partial(self.update_data, 'freq'))
        self.window_v.currentIndexChanged.connect(partial(self.update_data, 'window_v'))

    def update_data(self, tag, *args):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'time_i':
            self.data[0] = args[0]
            self.duration.setMaximum(self.max_t - self.data[0]) # al cambiar el tiempo de inicio cambia la duracion posible
        elif tag == 'duration':
            self.data[1] = args[0]
        elif tag == 'amp':
            self.data[2] = args[0]
        elif tag == 'freq':
            self.data[3] = args[0]
        elif tag == 'window_v':
            self.data[4] = args[0]

# estas listas son las mismas que tiene el formulario, son necesarias para traducir los valores seleccionados
filter_choices = ['firwin', 'remez', 'butter', 'chebyshev', 'elliptic']
filter_windows = ['hamming', 'hann', 'blackman', 'bartlett', 'rect']
class FilterForm(QDialog, FilterFormDialog):
    """
    Formulario para crear o editar un elemento de filtro que se añadirá a una de las curvas para la trayectoria.
    Debe ajustarse tiempo de inicio, tiempo de fin, tipo de filtro y sus parámetros (que varian de acuerdo al filtro)
    """
    def __init__(self, parent=None, data=[0 for i in range(16)]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # fijamos los valores iniciales
        self.time_i.setValue(data[0])
        self.time_f.setValue(data[1])
        self.filter_choice.setCurrentIndex(data[2])

        self.window_choice.setCurrentIndex(data[3])
        self.window_n.setValue(data[4])
        self.cutoff.setValue(data[5])

        self.Ap.setValue(data[6])
        self.As.setValue(data[7])
        self.fp.setValue(data[8])
        self.fs.setValue(data[9])

        self.chebN.setCurrentIndex(data[10])
        self.chebAp.setValue(data[11])
        self.chebAs.setValue(data[12])
        self.chebfp.setValue(data[13])
        self.chebfs.setValue(data[14])
        self.chebrp.setValue(data[15])

        # conectamos los cambios en los campos con update_data
        self.time_i.valueChanged.connect(partial(self.update_data, 'time_i'))
        self.time_f.valueChanged.connect(partial(self.update_data, 'time_f'))
        self.filter_choice.currentIndexChanged.connect(self.update_form) # en el caso de filter_choice lo conectamos a update_form

        self.window_choice.currentIndexChanged.connect(partial(self.update_data, 'window_choice'))
        self.window_n.valueChanged.connect(partial(self.update_data, 'window_n'))
        self.cutoff.valueChanged.connect(partial(self.update_data, 'cutoff'))

        self.Ap.valueChanged.connect(partial(self.update_data, 'Ap'))
        self.As.valueChanged.connect(partial(self.update_data, 'As'))
        self.fp.valueChanged.connect(partial(self.update_data, 'fp'))
        self.fs.valueChanged.connect(partial(self.update_data, 'fs'))

        self.chebN.currentIndexChanged.connect(partial(self.update_data, 'chebN'))
        self.chebAp.valueChanged.connect(partial(self.update_data, 'chebAp'))
        self.chebAs.valueChanged.connect(partial(self.update_data, 'chebAs'))
        self.chebfp.valueChanged.connect(partial(self.update_data, 'chebfp'))
        self.chebfs.valueChanged.connect(partial(self.update_data, 'chebfs'))
        self.chebrp.valueChanged.connect(partial(self.update_data, 'chebrp'))

        if self.data[2] == 0: # firwin tiene un set de parametros distinto al resto
            #self.windowGroup.hide()
            self.OtherGroup.hide()
            self.ChebGroup.hide()
        elif self.data[2] == 3: # chebyshev tambien tiene un set de parametros distinto
            self.windowGroup.hide()
            self.OtherGroup.hide()
            #self.ChebGroup.hide()
        else: # remez, butter o elliptic tienen los mismos parametros
            self.windowGroup.hide()
            #self.OtherGroup.hide()
            self.ChebGroup.hide()
        self.resize(400,100) # para ajustarse por los distintos tamaños
        self.min_size = self.size()

    def update_form(self, new_index):
        # toma los cambios en el tipo de filtro seleccionado y actualiza el display del formulario de acuerdo a los parametros necesarios
        if new_index == 0: # firwin
            self.windowGroup.show()
            self.OtherGroup.hide()
            self.ChebGroup.hide()
            self.adjustSize() # por los diferentes tamaños de los grupos es necesario ajustar el tamaño
            self.resize(self.min_size)
        elif new_index == 3: # chebyshev
            self.windowGroup.hide()
            self.OtherGroup.hide()
            self.ChebGroup.show()
            self.adjustSize() # por los diferentes tamaños de los grupos es necesario ajustar el tamaño
            self.resize(self.min_size)
        else: # remez, butter o elliptic
            self.windowGroup.hide()
            self.OtherGroup.show()
            self.ChebGroup.hide()
            self.adjustSize() # por los diferentes tamaños de los grupos es necesario ajustar el tamaño
            self.resize(self.min_size)
        self.data[2] = new_index # finalmente tambien actualizamos self.data

    def update_data(self, tag, *args):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'time_i':
            self.data[0] = args[0]
        elif tag == 'time_f':
            self.data[1] = args[0]
        elif tag == 'window_choice':
            self.data[3] = args[0]
        elif tag == 'window_n':
            self.data[4] = args[0]
        elif tag == 'cutoff':
            self.data[5] = args[0]
        elif tag == 'Ap':
            self.data[6] = args[0]
        elif tag == 'As':
            self.data[7] = args[0]
        elif tag == 'fp':
            self.data[8] = args[0]
        elif tag == 'fs':
            self.data[9] = args[0]
        elif tag == 'chebN':
            self.data[10] = args[0]
        elif tag == 'chebAp':
            self.data[11] = args[0]
        elif tag == 'chebAs':
            self.data[12] = args[0]
        elif tag == 'chebfp':
            self.data[13] = args[0]
        elif tag == 'chebfs':
            self.data[14] = args[0]
        elif tag == 'chebrp':
            self.data[15] = args[0]

class StatesFromNotesForm(QDialog, StatesFromNotesDialog):
    """
    Formulario para llevar a cabo la operacion de crear los estados de una partitura en base a las notas ingresadas.
    Debe ajustarse tiempo de duración de la transicion y el desfase que tiene el final de la transicion con la nota tocada
    """
    def __init__(self, parent=None, data=[0,0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        # fijamos los valores iniciales
        self.transition_time.setValue(data[0])
        self.transition_offset.setValue(data[1])

        # conectamos los cambios en los campos con update_data
        self.transition_time.valueChanged.connect(partial(self.update_data, 'transition_time'))
        self.transition_offset.valueChanged.connect(partial(self.update_data, 'transition_offset'))

    def update_data(self, tag, *args):
        # actualiza self.data con los valores ingresados por el usuario
        if tag == 'transition_time':
            self.data[0] = args[0]
        elif tag == 'transition_offset':
            self.data[1] = args[0]

class FuncTableForm(QDialog, FuncTableDialog):
    """
    Este formulario es diferente al resto. Quizas más que un formulario es una ventana. 
    Muestra todas las caracteristicas de la trayectoria en tablas.
    Es posible cambiar de funciones (o curvas) con un menu, para ver las propiedades de la curva de l, theta, offset, flow o notas.
    Tambien es posible ver todos los puntos que definen cada una de estas curvas en una tabla, asi como los vibratos que tiene y los elementos de filtro que se le agregaron. Se cambia de elemento a mostrar en la tabla con otro menu.
    Desde esta ventana es posible agregar, editar o eliminar cualquier elemento a cualquiera de las curvas.
    """
    def __init__(self, parent=None, data=[]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data
        self.item_selected = None
        self.last_note_t = 0
        self.last_note = 0

        self.function_choice.setCurrentIndex(data[0])
        if data[0] == 4: # si se elige la quinta curva, la de las notas
            self.property_choice.clear() # en la curva de las notas, la unica propiedad que tienen son las notas (falta implementar agregar los trills TODO)
            self.property_choice.addItems(['notes'])
        if data[0] == 5: # si se elige la quinta curva, la de las aperturas de los labios
            self.property_choice.clear() 
            self.property_choice.addItems(['points'])
        if data[0] == 6: # si se elige la quinta curva, la de la lengua
            self.property_choice.clear() 
            self.property_choice.addItems(['points'])
        self.property_choice.setCurrentIndex(data[1]) 

        # conectamos los cambios en los menus
        self.function_choice.currentIndexChanged.connect(self.function_change)
        self.property_choice.currentIndexChanged.connect(self.property_change)

        # conectamos los botones de añadir, editar y eliminar
        self.addButton.clicked.connect(self.add_action)
        self.editButton.clicked.connect(self.edit_action)
        self.deleteButton.clicked.connect(self.delete_action)

        # ahora llenamos las tablas con los elementos que corresponde
        self.poblate()

    def add_action(self):
        """
        Permite añadir un elemento (punto, vibrato, filtro o nota) a alguna de las curvas elegidas.
        Abre un formulario para llenar con los parametros necesarios y si la informacion es correcta agrega el elemento a la curva seleccionada
        """
        if self.data[0] in [0,1,2,3]: # mientras no se haya elegido la quinta curva, la de las notas, se puede agregar puntos, vibratos o filtros
            if self.data[1] == 0: # punto
                data = [0, 0] # partimos con un punto en 0, 0
                # fijamos los limites de acuerdo a la curva
                if self.data[0] == 0:
                    min_v, max_v = 0, 100
                elif self.data[0] == 1:
                    min_v, max_v = 20, 70
                elif self.data[0] == 2:
                    min_v, max_v = -99, 99
                elif self.data[0] == 3:
                    min_v, max_v = 0, 50
                dlg = PointForm(parent=self, data=data, max_t=self.data[2]['total_t'], min_v=min_v, max_v=max_v) # creamos el formulario 
                dlg.setWindowTitle("Add Point")
                if dlg.exec(): # si resulta exitoso agregamos el punto
                    new_x = data[0]
                    new_y = data[1]
                    self.parent.add_item(self.data[0], self.data[1], [new_x, new_y]) # le pedimos al parent (que es Window) que agregue el elemento
                    self.poblate() # volvemos a poblar las tablas
            elif self.data[1] == 1: # vibrato
                data=[0, 0, 0, 0, 0]
                dlg = VibratoForm(parent=self, data=data, max_t=self.data[2]['total_t']) # creamos el formulario
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec(): # si resulta exitoso lo agregamos
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.parent.add_item(self.data[0], self.data[1], [time_i, duration, amp, freq, window_v]) # le pedimos al parent (que es Window) que agregue el elemento
                    self.poblate() # volvemos a poblar las tablas
            elif self.data[1] == 2: # filtro
                data=[0, 0] + [0 for i in range(14)]
                while True:
                    dlg = FilterForm(parent=self, data=data) # creamos el formulario
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec(): # si resulta exitoso lo agregamos
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0:
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3:
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else:
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice] # traducimos el filtro a un string
                        if self.parent.check_filter(time_i, time_f, filter_choice, params): # revisamos que los parametros sean validos
                            self.parent.add_item(self.data[0], self.data[1], [time_i, time_f, filter_choice, params]) # le pedimos al parent (que es Window) que agregue el elemento
                            self.poblate() # volvemos a poblar las tablas
                            break
                    else:
                        break
        elif self.data[0] in [5,6]:
            if self.data[0] == 5:
                data = [0, 31] # partimos con un punto en 0, 0
                dlg = SurfacePointForm(parent=self, data=data, max_t=self.data[2]['total_t']) # creamos el formulario 
            else:
                data = [0, 0] # partimos con un punto en 0, 0
                dlg = TonguePointForm(parent=self, data=data, max_t=self.data[2]['total_t']) # creamos el formulario 
            dlg.setWindowTitle("Add Point")
            if dlg.exec(): # si resulta exitoso agregamos el punto
                new_x = data[0]
                new_y = data[1]
                self.parent.add_item(self.data[0], self.data[1], [new_x, new_y]) # le pedimos al parent (que es Window) que agregue el elemento
                self.poblate() # volvemos a poblar las tablas
        else: # si se selecciono la curva de las notas, solo se pueden agregar notas. Todavia no se implementa los trills TODO
            data = [self.last_note_t+0.1, self.last_note]
            dlg = NoteForm(parent=self, data=data, max_t=self.data[2]['total_t'])  # creamos el formulario
            dlg.setWindowTitle("Add Note")
            if dlg.exec(): # si resulta exitoso lo agregamos
                new_x = data[0]
                new_y = dict_notes[data[1]/2] # traducimos el valor a una nota (string)
                self.last_note_t = data[0]
                self.last_note = data[1]
                self.parent.add_item(4, 0, [new_x, new_y]) # le pedimos al parent (que es Window) que agregue el elemento
                self.poblate() # volvemos a poblar las tablas

    def edit_action(self):
        """
        Permite editar un elemento (punto, vibrato, filtro o nota) de alguna de las curvas.
        Abre un formulario para llenar con los parametros necesarios y si la informacion es correcta modifica el elemento
        """
        if self.item_selected == None: # este parametro cambia cuando se hace click sobre un elemento. Si no hay ningun elemento seleccionado, el boton edit no hace nada
            pass
        else:
            if self.data[0] in [0,1,2,3]: # mientras no se haya seleccionado la quinta curva (la de las notas)
                route=self.data[self.data[0] + 2] # en self.data se tienen las rutas de las 5 curvas, en orden y partiendo en 2
                if self.data[1] == 0: # si se quiere editar un punto
                    data = route['points'][self.item_selected] # usamos la data de ese punto para prellenar el formulario
                    dlg = PointForm(parent=self, data=data, max_t=self.data[2]['total_t'])
                    dlg.setWindowTitle("Add Point")
                    if dlg.exec():
                        new_x = data[0]
                        new_y = data[1]
                        self.parent.edit_item(self.data[0], self.data[1], self.item_selected, [new_x, new_y]) # le pedimos a la ventana principal que edite el elemento y actualice los graficos
                        self.poblate() # actualizamos la tabla
                elif self.data[1] == 1: # si se quiere editar un vibrato
                    data = route['vibrato'][self.item_selected] # usamos la data de ese vibrato para prellenar el formulario
                    data[4] = windows_vibrato.index(data[4]) # traducimos la ventana a un indice
                    dlg = VibratoForm(parent=self, data=data, max_t=self.data[2]['total_t']) 
                    dlg.setWindowTitle("Add Vibrato")
                    if dlg.exec():
                        time_i = data[0]
                        duration = data[1]
                        amp = data[2]
                        freq = data[3]
                        window_v = windows_vibrato[data[4]]
                        self.parent.edit_item(self.data[0], self.data[1], self.item_selected, [time_i, duration, amp, freq, window_v]) # le pedimos a la ventana principal que edite el elemento y actualice los graficos
                        self.poblate() # actualizamos la tabla
                elif self.data[1] == 2:
                    data = [0, 0] + [0 for i in range(14)]
                    new_data = route['filters'][self.item_selected] # i_init, i_end, filter, params. usamos la data de ese filtro para prellenar el formulario.
                    # en el caso de los filtros estos arreglos no coinciden, asique hay que formatearlo de acuerdo al tipo de filtro que sea
                    data[0] = new_data[0] # time_i
                    data[1] = new_data[1] # time_f
                    data[2] = filter_choices.index(new_data[2]) # filter_choice
                    if data[2] == 0: # de ventana
                        data[3] = filter_windows.index(new_data[3][0]) # window_choice
                        data[4] = new_data[3][1] # window_n
                        data[5] = new_data[3][2] # cutoff
                    elif data[2] == 3: # chebyshev
                        data[10] = new_data[3][0] # chebN
                        data[11] = new_data[3][1] # chebAp
                        data[12] = new_data[3][2] # chebAs
                        data[13] = new_data[3][3] # chebfp
                        data[14] = new_data[3][4] # chebfs
                        data[15] = new_data[3][5] # chebrp
                    else: # remez, butter o elliptic
                        data[6] = new_data[3][0] # Ap
                        data[7] = new_data[3][1] # As
                        data[8] = new_data[3][2] # fp
                        data[9] = new_data[3][3] # fs
                    while True: # lo hacemos dentro de un loop para iterar si los parametros del filtro no son validos
                        dlg = FilterForm(parent=self, data=data)
                        dlg.setWindowTitle("Add Filter")
                        if dlg.exec():
                            time_i = data[0]
                            time_f = data[1]
                            choice = data[2]
                            if choice == 0: # de ventana
                                params = [filter_windows[data[3]], data[4], data[5]]
                            elif choice == 3: # chebyshev
                                params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                            else: # remez, butter o elliptic
                                params = [data[6], data[7], data[8], data[9]]
                            filter_choice = filter_choices[choice] # traducimos el indice a un label
                            if self.parent.check_filter(time_i, time_f, filter_choice, params): # comprobamos que el filtro sea estable
                                self.parent.edit_item(self.data[0], self.data[1], self.item_selected, [time_i, time_f, filter_choice, params]) # le pedimos a la ventana principal que edite el elemento y actualice los graficos
                                self.poblate() # actualizamos la tabla
                                break
                        else:
                            break
            elif self.data[0] in [5,6]:
                route=self.data[self.data[0] + 2]
                data = route['points'][self.item_selected]
                if self.data[0] == 5:
                    dlg = SurfacePointForm(parent=self, data=data, max_t=self.data[2]['total_t']) # creamos el formulario 
                else:
                    dlg = TonguePointForm(parent=self, data=data, max_t=self.data[2]['total_t']) # creamos el formulario 
                dlg.setWindowTitle("Edit Point")
                if dlg.exec(): # si resulta exitoso agregamos el punto
                    new_x = data[0]
                    new_y = data[1]
                    self.parent.edit_item(self.data[0], self.data[1], self.item_selected, [new_x, new_y]) # le pedimos a la ventana principal que edite el elemento y actualice los graficos
                    self.poblate() # actualizamos la tabla
            else: # si nos encontramos en la quinta curva (la de las notas), solo podemos editar las notas
                data = self.data[6]['notes'][self.item_selected] # usamos la data de esa nota para prellenar el formulario.
                data[1] = int(round(dict_notes_rev[data[1]]*2, 0)) # traducimos de nota a indice
                dlg = NoteForm(parent=self, data=data, max_t=self.data[6]['total_t'])
                dlg.setWindowTitle("Edit Note")
                if dlg.exec():
                    data[1] = dict_notes[data[1]/2] # traducimos de indice a nota
                    self.parent.edit_item(4, 0, self.item_selected, data) # le pedimos a la ventana principal que edite el elemento y actualice los graficos
                    self.poblate() # actualizamos la tabla

    def delete_action(self):
        """
        Permite eliminar un elemento (punto, vibrato, filtro o nota) de alguna de las curvas.
        """
        if self.item_selected == None: # este parametro cambia cuando se hace click sobre un elemento. Si no hay ningun elemento seleccionado, el boton edit no hace nada
            pass
        else:
            self.parent.delete_item(self.data[0], self.data[1], self.item_selected) # le pedimos a la ventana principal que edite el elemento y actualice los graficos
            self.listWidget.takeItem(self.item_selected) # eliminamos el elemento de la tabla
        self.item_selected = None # soltamos este parametro (por si se apreta editar o eliminar nuevamente)

    def function_change(self, new_val):
        """
        Esta funcion escucha cambios en la seleccion de la curva. Actualiza los datos de la tabla conforme a la curva que se seleccione
        """
        self.item_selected = None # se suelta cualquier seleccion que se tenga
        self.data[0] = new_val
        if new_val == 4: # la curva de las notas es la unica de las curvas que tiene distintas propiedades. En esta solo se tienen las notas
            self.data[1] = 0
            self.property_choice.clear()
            self.property_choice.addItems(['notes'])
        elif new_val == 5 or new_val == 6:
            self.data[1] = 0
            self.property_choice.clear()
            self.property_choice.addItems(['points'])
        else: # las otras cuatro curvas tienen puntos, vibratos y filtros
            self.property_choice.clear()
            self.property_choice.addItems(['points', 'vibratos', 'filters'])
        self.poblate()

    def property_change(self, new_val):
        """
        Esta funcion escucha cambios en la seleccion del parametro de la curva. Actualiza los datos de la tabla conforme al parametro que se seleccione
        """
        self.item_selected = None
        self.data[1] = new_val
        self.poblate()

    def poblate(self):
        """
        Actualiza la lista que se muestra en pantalla
        """
        self.listWidget.clear() # borra la lista 
        self.route = self.data[self.data[0]+2] # obtiene la ruta de la que sacar los datos
        if self.data[0] in [0,1,2,3]: # si no es la curva de las notas
            if self.data[1] == 0: # queremos mostrar lo puntos de la curva seleccionada
                for dot in self.route['points']:
                    self.listWidget.addItem(f"t: {dot[0]}, y: {dot[1]}") # mostramos el valor del punto y su tiempo
            if self.data[1] == 1: # queremos mostrar lo vibratos de la curva seleccionada
                for vib in self.route['vibrato']:
                    self.listWidget.addItem(f"ti: {vib[0]}, d: {vib[1]}, amp: {vib[2]}, freq: {vib[3]}, win: {vib[4]}") # mostramos el tiempo de inicio del vibrato, su duracion, su amplitud, su frecuencia y la ventana por la que se multiplica el vibrato
            if self.data[1] == 2: # queremos mostrar lo filtros de la curva seleccionada
                for fil in self.route['filters']:
                    self.listWidget.addItem(f"ti: {fil[0]}, tf: {fil[1]}, fil: {fil[2]}, params: {fil[3]}") # mostramos el tiempo de inicio, el de fin, el tipo de filtro y sus parametros
        elif self.data[0] in [5,6]: # si no es la curva de las notas
            for dot in self.route['points']:
                self.listWidget.addItem(f"t: {dot[0]}, y: {dot[1]}") # mostramos el valor del punto y su tiempo
        else: # en este caso mostramos las notas (porque no tenemos otra propiedad)
            for n in self.route['notes']:
                self.listWidget.addItem(f"{n[0]} s -> {n[1]}") # agregamos nota por nota, mostrando que nota es y el tiempo en el que inicia
        self.listWidget.itemClicked.connect(self.item_clicked) # conectamos la lista a item_clicked
    
    def item_clicked(self, item):
        """
        Escucha cuando se hace click sobre algun elemento de la lista y obtiene su indice
        """
        self.item_selected = self.listWidget.row(item)