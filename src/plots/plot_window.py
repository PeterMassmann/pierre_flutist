from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QFileDialog, QLineEdit, QPushButton, QVBoxLayout
    )
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

import sys
from time import time, sleep
from numpy import linspace
from random import random, randint
from functools import partial

from src.plots.plot_window_view import Ui_MainWindow as PlotWindow
from src.plots.pasive_plot import Ui_MainWindow as ReferencePlot

from src.route import calculate_route
from src.cinematica import change_to_joint_space
from numpy import gradient

## creamos una lista con todas las señales que queremos plotear
signals = ['Radius', 'Incidence Angle', 'Jet Offset', 'Position', 'Mouth Pressure', 'Mass Flow Rate', 'Volume Flow Rate', 'Air Temperature', 'Sound Frequency', 'X Position', 'Z Position', 'Alpha Position']

class LivePlotWindow(QMainWindow, PlotWindow, QtCore.QThread):
    """
    Esta ventana permite graficar señales en tiempo real. Se va actualizando permanentemente (25 veces por segundo) reemplazando las curvas ploteadas por sus versiones mas recientemente medidas.
    Sus parametros para instanciarla son:
     - app: aplicacion donde vive la ventana
     - measure: un int entre 0 y len(signals) que indica la señal a graficar
     - data: diccionario que contiene la informacion en tiempo real de las señales. Este diccionario es actualizado por otro proceso
     - interval: intervalo de refrezco del grafico
     - parent: objeto que crea la instancia. En el codigo es de la clase Window
    """
    def __init__(self, app, measure, data, interval=40, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.interval = interval
        self.parent = parent
        self.app = app
        self.measures = [measure] # en esta lista agregaremos las señales que se quiere plotear
        self.traces = [] # en esta lista agregaremos acciones (botones de un menu) que permitiran agregar señales a la ventana
        if measure != 3:
            for i in range(len(signals)):
                if measure != i and i != 3: # agregamos al menu aquellas que son distinta de la que actualmente esta siendo ploteada y de Position que es un caso especial (esta no plotea tiempo vs valor a diferencia del resto, sino que x vs z)
                    self.traces.append(self.menuAdd_Trace.addAction(signals[i])) # agregamos acciones al menu
                    self.traces[-1].triggered.connect(partial(self.add_trace, i)) # las conectamos a una funcion que nos permite empezar a plotear las curvas
        self.data = data # data compartida entre procesos
        self.curves = [] # en esta lista agregaremos los objetos graficos donde plotearemos las señales
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w', (randint(0,255),randint(0,255),randint(0,255)), (randint(0,255),randint(0,255),randint(0,255)), (randint(0,255),randint(0,255),randint(0,255)), (randint(0,255),randint(0,255),randint(0,255))] # una lista con distintos colores para plotear cada señal. Las primeras señales tienen colores definidos, las siguientes se eligen de forma aleatoria

        for i in range(len(signals)): # añadimos a la lista self.curves un elemento grafico para cada señal. Cada uno con su color particular
            self.curves.append(self.graphicsView.plot(pen=pg.mkPen(colors[i], width=1)))
        self.ref_curve = self.graphicsView.plot(pen=pg.mkPen('w', width=1, style=QtCore.Qt.DashLine)) # creamos otro elemento grafico, para plotear una señal de referencia. El trazo de esta curva será dashed en lugar de continuo
        if measure == 3: # si queremos graficar la posicion, establecemos limites para el grafico especiales
            self.graphicsView.setXRange(-1, 110, padding=0)
            self.graphicsView.setYRange(-1, 110, padding=0)
        
        # creamos un timer que se encarga de correr la funcion self.update() cada intervalos regulares de tiempo
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval) # lo partimos al tiro

        self.t0 = time() # un tiempo inicial para plotear el eje de tiempo 
        self.t = 0 # nuestra variable de tiempo
    
    def add_trace(self, index): # agrega elementos a self.measures para plotear nuevas curvas.
        # TODO arreglar esta funcion
        self.measures.append(index)

    def update(self): # actualiza el grafico, cambiando la data de las curvas por la que fue medida de forma mas reciente
        for index in range(len(self.measures)): # revisamos qué señales graficar en base a los elementos de self.measures
            if self.measures[index] == 0: # Radius
                self.curves[index].setData(self.data['times'], self.data['radius'])
            elif self.measures[index] == 1: # Theta
                self.curves[index].setData(self.data['times'], self.data['theta'])
            elif self.measures[index] == 2: # Offset
                self.curves[index].setData(self.data['times'], self.data['offset'])
            elif self.measures[index] == 3: # Position
                self.curves[index].setData(self.data['x'], self.data['z'])
            elif self.measures[index] == 4: # Mouth Pressure
                self.curves[index].setData(self.data['times'], self.data['mouth_pressure'])
            elif self.measures[index] == 5: # Mass Flow. En este caso agregamos una curva para flow_ref
                self.curves[index].setData(self.data['times'], self.data['mass_flow'])
                self.ref_curve.setData(self.data['times'], self.data['flow_ref'])
            elif self.measures[index] == 6: # Volume Flow
                self.curves[index].setData(self.data['times'], self.data['volume_flow'])
            elif self.measures[index] == 7: # Temperature
                self.curves[index].setData(self.data['times'], self.data['temperature'])
            elif self.measures[index] == 8: # Frequency
                self.curves[index].setData(self.data['times'], self.data['frequency'])
            elif self.measures[index] == 9: # X. En este caso agregamos una curva para x_ref
                self.curves[index].setData(self.data['times'], self.data['x'])
                self.ref_curve.setData(self.data['times'], self.data['x_ref'])
            elif self.measures[index] == 10: # Z. En este caso agregamos una curva para z_ref
                self.curves[index].setData(self.data['times'], self.data['z'])
                self.ref_curve.setData(self.data['times'], self.data['z_ref'])
            elif self.measures[index] == 11: # Alpha. En este caso agregamos una curva para alpha_ref
                self.curves[index].setData(self.data['times'], self.data['alpha'])
                self.ref_curve.setData(self.data['times'], self.data['alpha_ref'])
            
        self.app.processEvents()


class PassivePlotWindow(QMainWindow, ReferencePlot):
    """
    Esta ventana grafica la trayectoria que se le enviará a los motores como referencia a partir de una trayectoria.
    Grafica las posiciones y velocidades para los tres motores en cada instante de tiempo
    """
    def __init__(self, app, route1, route2, route3, parent=None, space=0):
        super().__init__(parent)
        self.setupUi(self)
        self.parent = parent 
        self.app = app
        self.route1 = route1 # ruta para l (o x si se entrega en el espacio de las junturas)
        self.route2 = route2 # ruta para theta (o z si se entrega en el espacio de las junturas)
        self.route3 = route3 # ruta para offset (o alpha si se entrega en el espacio de las junturas)
        self.space = space # indica en que espacio se entregan las rutas. 0=espacio de la tarea, 1=espacio de las junturas
        self.Fs = self.route1['Fs']

        if self.space == 0: # si las rutas estan en el espacio de la tarea, calculamos las funciones de l, theta y offset a partir de sus rutas
            t, f_r, p, vib, fil = calculate_route(self.route1)
            t, f_theta, p, vib, fil = calculate_route(self.route2)
            t, f_offset, p, vib, fil = calculate_route(self.route3)
            f_x, f_z, f_alpha = change_to_joint_space(f_r, f_theta, f_offset) # en este caso deberemos transformar al espacio de las junturas
        elif self.space == 1: # si las rutas estan en el espacio de las junturas, calculamos las funciones de x, z y alpha a partir de sus rutas
            t, f_x, p, vib, fil = calculate_route(self.route1)
            t, f_z, p, vib, fil = calculate_route(self.route2)
            t, f_alpha, p, vib, fil = calculate_route(self.route3)
            # en este caso ya estamos en el espacio de las junturas, no es necesaria la conversion
        self.time = t
        self.x_ref = f_x
        self.x_vel_ref = gradient(self.x_ref)*self.Fs # calculamos los gradientes para las velocidades
        self.z_ref = f_z
        self.z_vel_ref = gradient(self.z_ref)*self.Fs
        self.alpha_ref = f_alpha
        self.alpha_vel_ref = gradient(self.alpha_ref)*self.Fs

        colors = ['b', 'g', 'r', 'c', 'm', 'y'] # cada curva tiene un color propio
        ## ahora creamos los 6 elementos graficos con los que plotearemos las 6 señales.
        self.x_curve = pg.PlotCurveItem(pen=pg.mkPen(colors[0], width=1))
        self.x_vel_curve = pg.PlotCurveItem(pen=pg.mkPen(colors[1], width=1))
        self.z_curve = pg.PlotCurveItem(pen=pg.mkPen(colors[2], width=1))
        self.z_vel_curve = pg.PlotCurveItem(pen=pg.mkPen(colors[3], width=1))
        self.alpha_curve = pg.PlotCurveItem(pen=pg.mkPen(colors[4], width=1))
        self.alpha_vel_curve = pg.PlotCurveItem(pen=pg.mkPen(colors[5], width=1))

        # les asignamos la data
        self.x_curve.setData(self.time, self.x_ref)
        self.x_vel_curve.setData(self.time, self.x_vel_ref)
        self.z_curve.setData(self.time, self.z_ref)
        self.z_vel_curve.setData(self.time, self.z_vel_ref)
        self.alpha_curve.setData(self.time, self.alpha_ref)
        self.alpha_vel_curve.setData(self.time, self.alpha_vel_ref)

        # e iniciamos la ventana mostrando solamente las curvas para la posicion y velocidad de X. Es posible agregar el resto con checkboxes
        self.graphicsView.addItem(self.x_curve)
        self.graphicsView.addItem(self.x_vel_curve)

        # conectamos todos los checkboxes
        self.xCheck.stateChanged.connect(self.x_checked)
        self.xVelCheck.stateChanged.connect(self.x_vel_checked)
        self.zCheck.stateChanged.connect(self.z_checked)
        self.zVelCheck.stateChanged.connect(self.z_vel_checked)
        self.alphaCheck.stateChanged.connect(self.alpha_checked)
        self.alphaVelCheck.stateChanged.connect(self.alpha_vel_checked)

        self.refreshButton.clicked.connect(self.refresh) # conectamos el boton para actualizar el grafico
        

    def refresh(self):
        # esta funcion permite actualizar el grafico si se hace una modificacion a la ruta sin la necesidad de cerrar la ventana y volver a abrirla.
        # sigue el mismo proceso de cuando se instancia el objeto
        Fs = self.route1['Fs']
        if self.space == 0:
            t, f_r, p, vib, fil = calculate_route(self.route1)
            t, f_theta, p, vib, fil = calculate_route(self.route2)
            t, f_offset, p, vib, fil = calculate_route(self.route3)
            f_x, f_z, f_alpha = change_to_joint_space(f_r, f_theta, f_offset)
        elif self.space == 1:
            t, f_x, p, vib, fil = calculate_route(self.route1)
            t, f_z, p, vib, fil = calculate_route(self.route2)
            t, f_alpha, p, vib, fil = calculate_route(self.route3)
        time = t
        x_ref = f_x
        x_vel_ref = gradient(x_ref)*Fs
        z_ref = f_z
        z_vel_ref = gradient(z_ref)*Fs
        alpha_ref = f_alpha
        alpha_vel_ref = gradient(alpha_ref)*Fs

        self.x_curve.setData(time, x_ref)
        self.x_vel_curve.setData(time, x_vel_ref)
        self.z_curve.setData(time, z_ref)
        self.z_vel_curve.setData(time, z_vel_ref)
        self.alpha_curve.setData(time, alpha_ref)
        self.alpha_vel_curve.setData(time, alpha_vel_ref)

    def x_checked(self, value): # cambia el estado del checkbox de la posicion de x. Muestra o esconde la curva
        if value:
            self.graphicsView.addItem(self.x_curve)
        else:
            self.graphicsView.removeItem(self.x_curve)

    def x_vel_checked(self, value): # cambia el estado del checkbox de la velocidad de x. Muestra o esconde la curva
        if value:
            self.graphicsView.addItem(self.x_vel_curve)
        else:
            self.graphicsView.removeItem(self.x_vel_curve)

    def z_checked(self, value): # cambia el estado del checkbox de la posicion de z. Muestra o esconde la curva
        if value:
            self.graphicsView.addItem(self.z_curve)
        else:
            self.graphicsView.removeItem(self.z_curve)

    def z_vel_checked(self, value): # cambia el estado del checkbox de la velocidad de z. Muestra o esconde la curva
        if value:
            self.graphicsView.addItem(self.z_vel_curve)
        else:
            self.graphicsView.removeItem(self.z_vel_curve)

    def alpha_checked(self, value): # cambia el estado del checkbox de la posicion de alpha. Muestra o esconde la curva
        if value:
            self.graphicsView.addItem(self.alpha_curve)
        else:
            self.graphicsView.removeItem(self.alpha_curve)

    def alpha_vel_checked(self, value): # cambia el estado del checkbox de la velocidad de alpha. Muestra o esconde la curva
        if value:
            self.graphicsView.addItem(self.alpha_vel_curve)
        else:
            self.graphicsView.removeItem(self.alpha_vel_curve)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window(app)
    win.show()

    sys.exit(app.exec())