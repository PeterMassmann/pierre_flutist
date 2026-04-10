from PyQt5 import QtGui, QtCore, QtWidgets
from src.views.manual_move import Ui_MainWindow as ManualWindow
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
import sys
import json
from src.route import dict_notes, dict_notes_rev
from src.cinematica import *
from src.motor_route import *
from time import time
import numpy as np

with open('src/look_up_table.json', 'r') as f: # abrimos el diccionario con las posiciones para cada nota
    LOOK_UP_TABLE = json.load(f)

class ManualWindow(QMainWindow, ManualWindow):
    # stop_playing = QtCore.pyqtSignal() #
    def __init__(self, app, musician_pipe, data, parent=None, connected=False):
        super().__init__(parent)
        self.setupUi(self)
        self.app = app
        self.parent = parent
        self.musician_pipe = musician_pipe # pipe que conecta con el musico para darle instrucciones
        self.data = data # data compartida entre procesos
        self.noteComboBox.addItems(list(dict_notes.values())) # agregamos las notas al combo box (depende del instrumento elegido)

        # preparamos la tabla para las posiciones de las notas
        self.tableWidget.setColumnCount(5) # 5 columnas: l, theta, offset, flow y lips
        self.tableWidget.setRowCount(len(LOOK_UP_TABLE)) # una fila por nota

        labels = list(LOOK_UP_TABLE.keys()) # los nombres de las notas (como labels)
        self.tableWidget.setVerticalHeaderLabels(labels) # los fijamos en el eje vertical

        labels = ["l [mm]", "theta [°]", "offset [mm]", "flow [SLPM]", "lips [pos]"] # labels para el eje horizontal
        self.tableWidget.setHorizontalHeaderLabels(labels)
        self.tableWidget.setAlternatingRowColors(True) # tabla con lineas gris y blanco

        # ahora llenamos la tabla
        row = 0
        for value in LOOK_UP_TABLE.values():
            item = QtWidgets.QTableWidgetItem(str(value['l']))
            self.tableWidget.setItem(row, 0, item)
            item = QtWidgets.QTableWidgetItem(str(value['theta']))
            self.tableWidget.setItem(row, 1, item)
            item = QtWidgets.QTableWidgetItem(str(value['offset']))
            self.tableWidget.setItem(row, 2, item)
            item = QtWidgets.QTableWidgetItem(str(value['flow']))
            self.tableWidget.setItem(row, 3, item)
            item = QtWidgets.QTableWidgetItem(str(value['lips']))
            self.tableWidget.setItem(row, 4, item)
            row += 1

        self.tableWidget.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch) # si la ventana cambia de tamaño la tabla tambien
        self.tableWidget.itemChanged.connect(self.item_changed) # cada vez que se cambia un elemento de la tabla se llama esta funcion

        self.speed = 50 # velocidad de movimiento inicial = 50
        self.moving_with_notes = False # bool que condiciona si el cambio de nota va acompañado de movimiento de posicion al lugar definido en la look_up_table
        self.changing_other = False # bool que bloquea los cambios de variables afectadas al modificar otra variable. Por ejemplo: al cambiar X tambien cambiamos l, pero no queremos que se mueva dos veces, por esto bloqueamos el efecto de la segunda.
        self.desired_state = State(0,0,0,0) # estado en el que se quiere estar, indicado por los spin boxes
        self.desired_state.x = self.data['x'][-1] # parte en el lugar donde realmente esta el robot
        self.desired_state.z = self.data['z'][-1] # parte en el lugar donde realmente esta el robot
        self.desired_state.alpha = self.data['alpha'][-1] # parte en el lugar donde realmente esta el robot
        self.moveToNoteCheckBox.stateChanged.connect(self.move_to_notes_change)

        ## en los spinboxes se llena con la posicion que tiene el robot cuando se abre la ventana
        self.xSpinBox.setValue(self.data['x'][-1])
        self.zSpinBox.setValue(self.data['z'][-1])
        self.alphaSpinBox.setValue(self.data['alpha'][-1])
        self.rSpinBox.setValue(self.data['radius'][-1])
        self.thetaSpinBox.setValue(self.data['theta'][-1])
        self.offsetSpinBox.setValue(self.data['offset'][-1])
        self.flowSpinBox.setValue(self.data['mass_flow'][-1])

        ## Y conectamos los cambios de spinbox con funciones
        self.xSpinBox.valueChanged.connect(self.change_x)
        self.zSpinBox.valueChanged.connect(self.change_z)
        self.alphaSpinBox.valueChanged.connect(self.change_angle)
        self.rSpinBox.valueChanged.connect(self.change_r)
        self.thetaSpinBox.valueChanged.connect(self.change_theta)
        self.offsetSpinBox.valueChanged.connect(self.change_offset)
        self.flowSpinBox.valueChanged.connect(self.change_flow)
        self.freqSpinBox.valueChanged.connect(self.change_flow_vibrato)
        self.ampSpinBox.valueChanged.connect(self.change_flow_vibrato_amp)
        self.noteComboBox.currentIndexChanged.connect(self.change_note)
        self.speedSlider1.valueChanged.connect(self.change_speed)
        self.pivotGoButton.clicked.connect(self.pivot)
        self.tongue_slider.valueChanged.connect(self.tongue_move_pressed)
        # self.tongue_slider.sliderReleased.connect(self.tongue_move_released)
        self.lips_surface_slider.valueChanged.connect(self.change_lips_surface)
        self.setNoteDictionary.clicked.connect(self.set_note_in_dictionary)
        self.actionSaveDictionayAs.triggered.connect(self.save_dict_as)
    
    def save_dict_as(self):
        fname, _ = QFileDialog.getSaveFileName(self, 'Open file', self.parent.base_path,"JSON files (*.json)")
        if fname != '':
            if fname[-5:] != '.json':
                fname += '.json'
            with open(fname, 'w') as file: # inmediatamente se actualiza la lista en el archivo, asi cuando se vuelve a correr el programa quedan guardados los valores
                json.dump(LOOK_UP_TABLE, file, indent=4, sort_keys=False)

    def set_note_in_dictionary(self):
        note = self.data['notes'][-1]
        l = self.data['radius'][-1]
        theta = self.data['theta'][-1]
        offset = self.data['offset'][-1]
        flow = self.data['mass_flow'][-1]
        lips = self.data['lips'][-1]
        LOOK_UP_TABLE[note]['l'] = l
        LOOK_UP_TABLE[note]['theta'] = theta
        LOOK_UP_TABLE[note]['offset'] = offset
        LOOK_UP_TABLE[note]['lips'] = lips
        row = list(LOOK_UP_TABLE.keys()).index(note)
        item = QtWidgets.QTableWidgetItem(str(l))
        self.tableWidget.setItem(row, 0, item)
        item = QtWidgets.QTableWidgetItem(str(theta))
        self.tableWidget.setItem(row, 1, item)
        item = QtWidgets.QTableWidgetItem(str(offset))
        self.tableWidget.setItem(row, 2, item)
        item = QtWidgets.QTableWidgetItem(str(flow))
        self.tableWidget.setItem(row, 3, item)
        item = QtWidgets.QTableWidgetItem(str(lips))
        self.tableWidget.setItem(row, 4, item)

    def tongue_move_pressed(self, value):
        self.musician_pipe.send(['execute_tongue_action', value])
        self.tongue_label.setText(str(value))

    # def tongue_move_released(self):
    #     self.tongue_slider.setValue(0)

    def change_lips_surface(self, value):
        self.musician_pipe.send(['execute_lips_surface_action', value]) # se envia primero la instruccion de cambiar la digitacion
        self.surface_label.setText(str(value))

    def rotate(self, x1, z1, xr, zr, alpha): # funcion para encontrar la posicion final de una rotacion en alpha grados de un punto, con centro de rotacion en xr, zr
        xf = (x1 - xr) * np.cos(alpha) - (z1 - zr) * np.sin(alpha) + xr
        zf = (x1 - xr) * np.sin(alpha) + (z1 - zr) * np.cos(alpha) + zr
        return xf, zf
    
    def get_pivot_route(self, reference, delta_x, delta_z, angle, speed):
        """
        al llamar esta funcion se le manda al musico la instruccion de hacerun movimiento tipo 'pivote', donde la boca gira en torno a un punto una cantidad definida de grados. La trayectoria es de radio constante con respecto al centro de rotacion y la inclinacion de la boca varia la misma cantidad de grados que el movimiento. El perfil de velocidad del movimiento es polinomial de tercer orden.
        Los argumentos de la funcion son:
         - reference (int) -> un numero de 0 a 2 que dice en que punto se encuentra el origen de coordenadas con el que se realizará la rotacion. 0 significa que el origen coordenado esta en la punta de la boca, 1 pone el origen en el bisel de la flauta y 2 en el origen absoluto.
         - delta_x (float) -> dice cuanto se desplaza el centro de rotacion del origen en el eje x
         - delta_y (float) -> dice cuanto se desplaza el centro de rotacion del origen en el eje y
         - angle (float) -> de cuantos grados es la rotacion
         - speed (int) -> valor de 1 a 100 que es inversamente proporcional al tiempo que toma realizar el movimiento
        """
        global DATA
        # el punto de partida es el estado actual del robot (medido por los encoders) en coordenadas de las junturas
        x_i = self.data['x'][-1] 
        z_i = self.data['z'][-1]
        alpha_i = self.data['alpha'][-1]
        if reference == 0:
            ## Mouth end
            x_mouth, z_mouth = get_pos_punta(x_i,z_i,alpha_i*np.pi/180)  # obtenemos la posicion actual de la punta de la boca
            x_pivot = x_mouth + delta_x # sumamos los delta
            z_pivot = z_mouth + delta_z
        elif reference == 1:
            ## flute labium
            x_pivot = DATA['flute_position']['X_F'] + delta_x # la posicion del bisel de la flauta más los delta
            z_pivot = DATA['flute_position']['Z_F'] + delta_z
        elif reference == 2:
            ## origin
            x_pivot = delta_x
            z_pivot = delta_z
        
        route = []
        for i in range(101): # armamos una ruta con 101 puntos (actual + 99 intermedios + final)
            x_f, z_f = self.rotate(x_i, z_i, x_pivot, z_pivot, i*angle*np.pi/180/100) # calculamos la posicion luego de cada diferencial del angulo a rotar
            alpha_f = alpha_i + i*angle/100 # el ángula que debiera inclinar la cabeza para cada punto de la rotacion
            x_f = x_mm_to_units(x_f) # obtenemos las rutas en pasos de los motores
            z_f = z_mm_to_units(z_f) 
            alpha_f = alpha_angle_to_units(alpha_f)
            route.append([x_f, z_f, alpha_f]) # y vamos armando la ruta
        
        T = abs(angle) * 15/speed # definimos un tiempo total para realizar el movimiento, que es proporcional al angulo que se quiere rotar e inversamente proporcional a la velocidad
        route = time_scaled_straight_line(route, T) # formateamos la ruta y le agregamos el tiempo, con el escalamiento de tercer orden

        self.musician_pipe.send(["pivot", route['x'], route['z'], route['alpha']]) # le mandamos las trayectorias al musico y le pedimos que realice el movimiento
        self.changing_other = True # activamos esta condicion para evitar otros cambios al actualizar los otros spinboxes
        self.desired_state.x = x_units_to_mm(route['x'][-1][1])
        self.desired_state.z = z_units_to_mm(route['z'][-1][1])
        self.desired_state.alpha = alpha_units_to_angle(route['alpha'][-1][1]) 
        self.update_values() # actualizamos los spinboxes
        self.changing_other = False # desactivamos la condicion
        

    def pivot(self): # se llama esta función al apretar el boton de pivotear, lee los valores ingresados en los spinboxes de los parametros para el giro y llama a la funcion que se encarga de pivotear
        delta_x = self.pivotDeltaX.value()
        delta_z = self.pivotDeltaZ.value()
        reference = self.pivotReference.currentIndex()
        angle = self.pivotAngle.value()
        self.get_pivot_route(reference, delta_x, delta_z, angle, self.speed)

    def change_speed(self, value): # actualiza la velocidad de acuerdo al spindle
        self.speed = value

    def change_note(self, value): # se llama cuando el usuario cambia la nota en el combo box. Se encarga de mandar la instruccion al musico para cambiar la nota
        self.musician_pipe.send(['execute_fingers_action', dict_notes_rev[self.noteComboBox.itemText(value)], False]) # se envia primero la instruccion de cambiar la digitacion
        if self.moving_with_notes: # si además esta activada la opcion de moverse junto con las notas, se encarga de mover el robot a la posicion que se tiene en la LOOK_UP_TABLE
            self.changing_other = True  # activamos esta condicion para evitar otros cambios al actualizar los otros spinboxes
            pos = LOOK_UP_TABLE[self.noteComboBox.itemText(value)] # leemos la posicion asociada a la nota que se quiere tocar
            self.desired_state.r = pos['l']
            self.desired_state.theta = pos['theta']
            self.desired_state.o = pos['offset']
            self.desired_state.flow = pos['flow']
            self.desired_state.lips = pos['lips']
            self.desired_state.vibrato_amp = 0
            self.desired_state.vibrato_freq = 0
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, False, self.speed]) # le pedimos al musico que se mueva a la posición asociada a la nota
            self.update_values() # actualizamos los spinBoxes
            self.changing_other = False # desactivamos la condicion

    def move_to_notes_change(self, value): # se encarga de escuchar cambios en el checkbox del movimiento con las notas
        self.moving_with_notes = value

    def update_values(self): # actualiza los valores de los spinboxes con los del estado al que se quiere llegar. 
        # se ocupa por ejemplo cuando se hace un cambio en un espacio de coordenadas, para que el cambio se vea reflejado en el otro espacio
        self.flowSpinBox.setValue(self.desired_state.flow)
        self.xSpinBox.setValue(self.desired_state.x)
        self.zSpinBox.setValue(self.desired_state.z)
        self.alphaSpinBox.setValue(self.desired_state.alpha)
        self.rSpinBox.setValue(self.desired_state.r)
        self.thetaSpinBox.setValue(self.desired_state.theta)
        self.offsetSpinBox.setValue(self.desired_state.o)
        self.freqSpinBox.setValue(self.desired_state.vibrato_freq)
        self.ampSpinBox.setValue(self.desired_state.vibrato_amp)
        self.lips_surface_slider.setValue(self.desired_state.lips)
        self.tongue_slider.setValue(self.desired_state.tongue)

    def change_x(self, value): # cuando el usuario escribe un cambio en el spinbox de X, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.x = value
            self.musician_pipe.send(["move_to", self.desired_state, None, True, False, False, False, self.speed]) # el True significa que el movimiento será solo en el eje X
            self.update_values()
            self.changing_other = False # desactivamos la condicion
    
    def change_z(self, value): # cuando el usuario escribe un cambio en el spinbox de Z, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.z = value
            self.musician_pipe.send(["move_to", self.desired_state, None, False, True, False, False, self.speed]) # el True significa que el movimiento será solo en el eje Z
            self.update_values()
            self.changing_other = False # desactivamos la condicion

    def change_angle(self, value): # cuando el usuario escribe un cambio en el spinbox de Alpha, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.alpha = value
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, True, False, self.speed])# el True significa que el movimiento será solo en el eje Alpha
            self.update_values()
            self.changing_other = False # desactivamos la condicion

    def change_r(self, value): # cuando el usuario escribe un cambio en el spinbox de L, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.r = value
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False # desactivamos la condicion

    def change_theta(self, value): # cuando el usuario escribe un cambio en el spinbox de Theta, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.theta = value
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False # desactivamos la condicion

    def change_offset(self, value): # cuando el usuario escribe un cambio en el spinbox de Offset, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.o = value
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False # desactivamos la condicion
    
    def change_flow(self, value): # cuando el usuario escribe un cambio en el spinbox de Flow, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.flow = value
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, True, self.speed]) # el True significa que el cambio es solo del flujo
            self.changing_other = False # desactivamos la condicion

    def change_flow_vibrato(self, value): # cuando el usuario escribe un cambio en el spinbox de la frecuencia del vibrato, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.vibrato_freq = value
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, True, self.speed]) # el True significa que el cambio es solo del flujo
            self.changing_other = False # desactivamos la condicion

    def change_flow_vibrato_amp(self, value): # cuando el usuario escribe un cambio en el spinbox de la amplitud del vibrato, esta funcion se encarga de la instruccion al musico
        if not self.changing_other:
            self.changing_other = True # activamos esta condicion para evitar multiples instrucciones al musico cuando se cambien los otros spinboxes
            self.desired_state.vibrato_amp = value
            self.musician_pipe.send(["move_to", self.desired_state, None, False, False, False, True, self.speed]) # el True significa que el cambio es solo del flujo
            self.changing_other = False # desactivamos la condicion

    def item_changed(self, item): # se llama cuando hay un cambio en un elemento de la tabla que se usa para las posiciones de cada nota
        # Get the new value of the item
        new_value = item.text()
        # Get the row and column of the item
        row = item.row()
        col = item.column()
        # Get the key corresponding to the row and col
        key = self.tableWidget.verticalHeaderItem(row).text()
        key2 = self.tableWidget.horizontalHeaderItem(col).text().split(" ")[0]

        # Check the input is a number
        try:
            new_value = float(new_value)
            if key2 == "lips":
                new_value = int(min(max(new_value, 0), 31))
                item.setText(str(new_value))
        except:
            # si no es un numero, se vuelve a poner en la casilla el valor que se tenía antes y se retorna
            item.setText(str(LOOK_UP_TABLE[key][key2]))
            return
        
        # Update the value in the dictionary
        LOOK_UP_TABLE[key][key2] = new_value

        with open('src/look_up_table.json', 'w') as file: # inmediatamente se actualiza la lista en el archivo, asi cuando se vuelve a correr el programa quedan guardados los valores
            json.dump(LOOK_UP_TABLE, file, indent=4, sort_keys=False)
        


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ManualWindow(app, connected=False)
    win.show()

    sys.exit(app.exec())