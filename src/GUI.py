import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QMenu, QMessageBox,
    QFileDialog, QSplashScreen, QDesktopWidget, QLabel, QFrame, QVBoxLayout,
    QHBoxLayout, QWidget, QSizePolicy, QGroupBox)
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

import sys
from time import time, sleep
from numpy import linspace
from random import random, randint
from functools import partial
import json
import os
from datetime import date, datetime
from multiprocessing import Process, Event, Value, Pipe, Manager
import subprocess

from src.views.mainwindow import Ui_MainWindow as PlotWindow
from src.views.start_up_screen import Ui_Form as StartWindow
from src.route import *
from src.cinematica import *
from src.manual_move_win import *
from src.interactive_plot import InteractivePlotWidget

from src.plots.plot_window import LivePlotWindow, PassivePlotWindow

from src.forms.forms import PointForm, VibratoForm, windows_vibrato, FilterForm, filter_windows, filter_choices, FuncTableForm, NoteForm, DurationForm, CorrectionForm, ScaleTimeForm, SettingsForm, TrillForm, StatesFromNotesForm, SurfacePointForm, TonguePointForm

# --- Constantes visuales (coherentes con theme.qss) ---
PLOT_BG = '#FFFFFF'
COLOR_REF = '#2962FF'       # azul primario - curvas de referencia
COLOR_MEAS = '#00C853'      # verde acento - mediciones reales
COLOR_POINT = (0, 200, 83, 255)      # verde - puntos de trayectoria
COLOR_VIBRATO = (211, 47, 47, 255)   # rojo - marcadores de vibrato
COLOR_FILTER = (41, 98, 255, 255)    # azul primario - marcadores de filtro
COLOR_CURSOR = '#AB47BC'    # púrpura - cursor temporal
POINT_SIZE = 10
CURVE_WIDTH = 2


class StartUpWindow(QSplashScreen, StartWindow):
    """
    Esta ventana se abre al inicio del programa, mientras se cargan todos los modulos.
    Una vez que esta todo corriendo se cierra.
    """
    def __init__(self, app, pipe, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.app = app
        self.pipe = pipe # en este pipe va recibiendo los mensajes de los drivers que estan listos
    
    def set_progress(self, value):
        self.progressBar.setValue(int(value))

    def location_on_the_screen(self):
        ag = QDesktopWidget().availableGeometry()
        sg = QDesktopWidget().screenGeometry()

        widget = self.geometry()
        x = int(ag.width()/2 - widget.width()/2)
        y = int(ag.height()/2 - widget.height()/2)
        self.move(x, y)

    def wait_loading(self): # se ejecuta esta funcion para esperar que se inicien y esten listos todos los procesos de inicio del robot
        progress = 10
        self.set_progress(10) # definimos 9 procesos que hay que esperar, cada uno suma 10%, asi que se empieza con 10 para sumar 100
        while True: # antes que se inicien todos los procesos se mantiene en este loop. Este se ejecuta en el thread principal, por lo que debe salir antes de seguir ejecutando codigo (lo que sigue al llamado de esta funcion es el show de la ventana principal)
            if self.pipe.poll(0.2):
                m = self.pipe.recv()
                if m[0] == "instances created": # se crearon las instancias de los drivers y otros objetos, pero quizas aun no estan operacionales
                    progress += 10
                    self.instancesCheck.setChecked(True)
                elif m[0] == "x_driver_started": # eje x operacional y home listo
                    progress += 10
                    self.XCheck.setChecked(True)
                elif m[0] == "z_driver_started": # eje z operacional y home listo
                    progress += 10
                    self.ZCheck.setChecked(True)
                elif m[0] == "alpha_driver_started": # eje alpha operacional y home listo
                    progress += 10
                    self.AlphaCheck.setChecked(True)
                elif m[0] == "memory_started": # memoria lista
                    progress += 10
                    self.memoryCheck.setChecked(True)
                elif m[0] == "microphone_started": # microfono listo
                    progress += 10
                    self.microphoneCheck.setChecked(True)
                elif m[0] == "flow_driver_started": # controlador de flujo listo
                    progress += 10
                    self.flowCheck.setChecked(True)
                elif m[0] == "pressure_sensor_started": # sensor de presion listo
                    progress += 10
                    self.pressureCheck.setChecked(True)
                elif m[0] == "finger_driver_started": # controlador de dedos listo
                    progress += 10
                    self.fingersCheck.setChecked(True)
            self.set_progress(progress) # aumentamos el progreso de la barra
            if progress >= 100: # si todos los procesos se terminaron, salimos
                self.close()
                break

class Window(QMainWindow, PlotWindow):
    """
    Ventana principal de la interaccion con el robot.
    """
    stop_playing = QtCore.pyqtSignal() # sirve para emitir la señal de que se terminó de tocar una partitura. Puede ser porque llego al final de la partitura o porque fue detenida por el usuario
    refresh_plots_signal = QtCore.pyqtSignal(list) # sirve para emitir la señal de que se tienen que actualizar uno o varios gráficos desde un thread distinto al principal
    def __init__(self, app, running, musician_pipe, data, parent=None, connected=False):
        super().__init__(parent)
        self.setupUi(self)
        self._rebuild_side_panel()
        self.app = app
        self.route = [] # datos para el grafico 1: l
        self.route2 = [] # datos para el grafico 2: theta
        self.route3 = [] # datos para el grafico 3: offset
        self.route4 = [] # datos para el grafico 4: flow
        self.route5 = [] # datos para el grafico 5: notas
        self.route6 = [] # datos para el grafico 6: lips surface
        self.route7 = [] # datos para el grafico 7: tongue

        self.running = running # evento que conecta todos los procesos. Se cierra al cerrar la aplicacion
        self.musician_pipe = musician_pipe # pipe que conecta con el musico. Se usa para enviarle todas las instrucciones que se comandan a traves de esta interfaz gráfica
        self.data = data # data compartida entre procesos. Tiene por ejemplo las mediciones de los distintos sensores, que este proceso usa para graficarlas
        self.connected = connected # bool que dice si los actuadores estan realmente conectados o si solo es una simulacion
        self.playing = threading.Event() # evento para ejecutar una partitura (sin bloquear el thread principal). Se usa para comunicar al thread si el usuario decide parar la ejecución apretando el botón de stop

        ## se tienen 5 graphicsView para insertar los distintos graficos de l, theta, offset, flow y notas
        self.graphicsView.setBackground(PLOT_BG) # fondo blanco
        self.graphicsView_2.setBackground(PLOT_BG)
        self.graphicsView_3.setBackground(PLOT_BG)
        self.graphicsView_4.setBackground(PLOT_BG)
        self.graphicsView_5.setBackground(PLOT_BG)
        self.graphicsView_6.setBackground(PLOT_BG)
        self.graphicsView_7.setBackground(PLOT_BG)
        self.graphicsView_8 = InteractivePlotWidget(self.app, self)
        self.verticalLayout.addWidget(self.graphicsView_8)

        ## se crean lineas verticales para representar el tiempo actual. Lo llamo cursor en adelante, al moverlo se indica la parte de la partitura en donde se quiere empezar a tocar y cuando se ejecuta una cancion el cursor va avanzando en la partitura. Puede moverse con un slider que se encuentra abajo de los 5 graphicsView
        self.rule = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(COLOR_CURSOR, width=2), label='t')
        self.rule2 = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(COLOR_CURSOR, width=2), label='t')
        self.rule3 = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(COLOR_CURSOR, width=2), label='t')
        self.rule4 = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(COLOR_CURSOR, width=2), label='t')
        self.rule5 = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(COLOR_CURSOR, width=2), label='t')
        self.rule6 = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(COLOR_CURSOR, width=2), label='t')
        self.rule7 = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(COLOR_CURSOR, width=2), label='t')
        self.graphicsView.addItem(self.rule)
        self.graphicsView_2.addItem(self.rule2)
        self.graphicsView_3.addItem(self.rule3)
        self.graphicsView_4.addItem(self.rule4)
        self.graphicsView_5.addItem(self.rule5)
        self.graphicsView_6.addItem(self.rule6)
        self.graphicsView_7.addItem(self.rule7)
        
        ## 1: jet-lenght
        self.func1 = pg.PlotCurveItem(pen=pg.mkPen(COLOR_REF, width=CURVE_WIDTH)) # creamos la curva que representa la referencia para l
        self.func1.setClickable(10) # la hacemos clickeable para interactuar más facil con ella (para agregar puntos)
        self.func1.sigClicked.connect(self.onCurveClicked) # ejecutamos la funcion onCurveClicked cuando se hace click sobre la curva.  
        self.graphicsView.addItem(self.func1) # la agregamos al graphicsView
        self.graphicsView.setLabel('left', 'l', units='mm') # le ponemos nombre y unidad al eje y

        self.r_real = pg.PlotCurveItem(pen=pg.mkPen(COLOR_MEAS, width=CURVE_WIDTH)) # curva que representará la medición de l a partir de los encoders de los motores. Esta curva solo será ploteada cuando se este ejecutando una partitura
        self.graphicsView.addItem(self.r_real)

        self.scatter1 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_POINT)) # este primer scatter tiene los puntos que definen las posiciones por donde pasa la curva de referencia para l.
        self.scatter1.sigClicked.connect(self.onPointsClicked) # hacemos que los puntos sean clickeables para interactuar mas facil con ellos. Al hacerles click se despliega un menu que permite mover los puntos (de forma dinamica), editarlos o borrarlos 
        self.graphicsView.addItem(self.scatter1)

        self.vibscatter1 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_VIBRATO)) # el segundo scatter tiene los puntos donde se comienza un vibrato. Estos son rojos.
        self.vibscatter1.sigClicked.connect(self.onVibratoClicked) # los hacemos clickeables, funcionan parecido a los puntos de la trayectoria
        self.graphicsView.addItem(self.vibscatter1)

        self.filscatter1 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_FILTER)) # el tercer scatter tiene los puntos donde se agregan filtros. Estos son azules.
        self.filscatter1.sigClicked.connect(self.onFilterClicked) # nuevamente los hacemos clickeables y funcionan parecido a los otros dos
        self.graphicsView.addItem(self.filscatter1)

        ## Se repiten estas mismas curvas y scatters para cada uno de los graficos
        ## 2: theta
        self.func2 = pg.PlotCurveItem(pen=pg.mkPen(COLOR_REF, width=CURVE_WIDTH))
        self.func2.setClickable(10)
        self.func2.sigClicked.connect(self.onCurveClicked2)
        self.graphicsView_2.addItem(self.func2)
        self.graphicsView_2.setLabel('left', u"\u03b8", units='°')

        self.theta_real = pg.PlotCurveItem(pen=pg.mkPen(COLOR_MEAS, width=CURVE_WIDTH))
        self.graphicsView_2.addItem(self.theta_real)
        
        self.scatter2 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_POINT))
        self.scatter2.sigClicked.connect(self.onPointsClicked2)
        self.graphicsView_2.addItem(self.scatter2)

        self.vibscatter2 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_VIBRATO))
        self.vibscatter2.sigClicked.connect(self.onVibratoClicked2)
        self.graphicsView_2.addItem(self.vibscatter2)

        self.filscatter2 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_FILTER))
        self.filscatter2.sigClicked.connect(self.onFilterClicked2)
        self.graphicsView_2.addItem(self.filscatter2)

        ## 3: offset
        self.func3 = pg.PlotCurveItem(pen=pg.mkPen(COLOR_REF, width=CURVE_WIDTH))
        self.func3.setClickable(10)
        self.func3.sigClicked.connect(self.onCurveClicked3)
        self.graphicsView_3.addItem(self.func3)
        self.graphicsView_3.setLabel('left', "Offset", units='mm')

        self.offset_real = pg.PlotCurveItem(pen=pg.mkPen(COLOR_MEAS, width=CURVE_WIDTH))
        self.graphicsView_3.addItem(self.offset_real)
        
        self.scatter3 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_POINT))
        self.scatter3.sigClicked.connect(self.onPointsClicked3)
        self.graphicsView_3.addItem(self.scatter3)

        self.vibscatter3 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_VIBRATO))
        self.vibscatter3.sigClicked.connect(self.onVibratoClicked3)
        self.graphicsView_3.addItem(self.vibscatter3)

        self.filscatter3 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_FILTER))
        self.filscatter3.sigClicked.connect(self.onFilterClicked3)
        self.graphicsView_3.addItem(self.filscatter3)

        ## 4: flow
        self.func4 = pg.PlotCurveItem(pen=pg.mkPen(COLOR_REF, width=CURVE_WIDTH))
        self.func4.setClickable(10)
        self.func4.sigClicked.connect(self.onCurveClicked4)
        self.graphicsView_4.addItem(self.func4)
        self.graphicsView_4.setLabel('left', "Flow", units='SLPM')

        self.flow_real = pg.PlotCurveItem(pen=pg.mkPen(COLOR_MEAS, width=CURVE_WIDTH))
        self.graphicsView_4.addItem(self.flow_real)

        self.scatter4 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_POINT))
        self.scatter4.sigClicked.connect(self.onPointsClicked4)
        self.graphicsView_4.addItem(self.scatter4)

        self.vibscatter4 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_VIBRATO))
        self.vibscatter4.sigClicked.connect(self.onVibratoClicked4)
        self.graphicsView_4.addItem(self.vibscatter4)

        self.filscatter4 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_FILTER))
        self.filscatter4.sigClicked.connect(self.onFilterClicked4)
        self.graphicsView_4.addItem(self.filscatter4)

        ## 5: notas
        ## el grafico de las notas, a diferencia de los demas, no tiene el scatter de los filtros, y el del vibrato funciona distinto (se reemplaza por trill)
        self.func5 = pg.PlotCurveItem(pen=pg.mkPen(COLOR_REF, width=CURVE_WIDTH))
        self.func5.setClickable(10)
        self.func5.sigClicked.connect(self.onCurveClicked5)
        self.graphicsView_5.addItem(self.func5)
        self.graphicsView_5.setLabel('left', "Notes", units='n')
        self.graphicsView_5.getAxis('left').setTicks([dict_notes.items()])

        self.freq_real = pg.PlotCurveItem(pen=pg.mkPen(COLOR_MEAS, width=CURVE_WIDTH))
        self.graphicsView_5.addItem(self.freq_real)
        
        self.scatter5 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_POINT))
        self.scatter5.sigClicked.connect(self.onPointsClicked5)
        self.graphicsView_5.addItem(self.scatter5)

        self.horizontalSlider.valueChanged.connect(self.move_coursor)
        self.vibscatter5 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_VIBRATO))
        self.vibscatter5.sigClicked.connect(self.onVibratoClicked5)
        self.graphicsView_5.addItem(self.vibscatter5)

        ## 6: lips surface
        ## el grafico de la apertura de los labios no tiene vibrato ni filtros
        self.func6 = pg.PlotCurveItem(pen=pg.mkPen(COLOR_REF, width=CURVE_WIDTH))
        self.func6.setClickable(10)
        self.func6.sigClicked.connect(self.onCurveClicked6)
        self.graphicsView_6.addItem(self.func6)
        self.graphicsView_6.setLabel('left', "Lips Aperture", units='n')
        #self.graphicsView_6.getAxis('left').setTicks([i for i in range(32)])
        
        self.scatter6 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_POINT))
        self.scatter6.sigClicked.connect(self.onPointsClicked6)
        self.graphicsView_6.addItem(self.scatter6)

        ## 7: tongue
        ## el grafico de la lengua no tiene vibrato ni filtros
        self.func7 = pg.PlotCurveItem(pen=pg.mkPen(COLOR_REF, width=CURVE_WIDTH))
        self.func7.setClickable(10)
        self.func7.sigClicked.connect(self.onCurveClicked7)
        self.graphicsView_7.addItem(self.func7)
        self.graphicsView_7.setLabel('left', "Tongue", units='n')
        #self.graphicsView_7.getAxis('left').setTicks([i for i in range(4)])
        
        self.scatter7 = pg.ScatterPlotItem(size=POINT_SIZE, brush=pg.mkBrush(*COLOR_POINT))
        self.scatter7.sigClicked.connect(self.onPointsClicked7)
        self.graphicsView_7.addItem(self.scatter7)


        ## creamos los arreglos donde se almacenará la informacion de los graficos de las mediciones reales de cada eje
        self.r_plot = np.array([])
        self.theta_plot = np.array([])
        self.offset_plot = np.array([])
        self.flow_plot = np.array([])
        self.freq_plot = np.array([])
        self.t_plot = np.array([])

        ## Se linkean los ejes X para que se muevan en conjunto. Esto se hace de forma escalonada (el 2 con el 1, el 3 con el 2, el 4 con el 3...)
        self.graphicsView.setXLink(self.graphicsView_2)
        self.graphicsView_2.setXLink(self.graphicsView_3)
        self.graphicsView_3.setXLink(self.graphicsView_4)
        self.graphicsView_4.setXLink(self.graphicsView_8.graphicsView)
        self.graphicsView_8.graphicsView.setXLink(self.graphicsView_6)
        self.graphicsView_6.setXLink(self.graphicsView_7)
        self.graphicsView_7.setXLink(self.graphicsView_5)
        

        ## Conectamos los checkboxes a sus funciones. Si se aprieta alguno se esconde o muestra uno de los gráficos
        self.checkBox.stateChanged.connect(self.checkBox_clicked)
        self.checkBox_2.stateChanged.connect(self.checkBox2_clicked)
        self.checkBox_3.stateChanged.connect(self.checkBox3_clicked)
        self.checkBox_4.stateChanged.connect(self.checkBox4_clicked)
        self.checkBox_5.stateChanged.connect(self.checkBox5_clicked)
        self.checkBox_6.stateChanged.connect(self.checkBox6_clicked)
        self.checkBox_7.stateChanged.connect(self.checkBox7_clicked)
        self.checkBox_8.stateChanged.connect(self.checkBox8_clicked)
        self.graphicsView_6.hide()
        self.graphicsView_7.hide()
        self.graphicsView_8.hide()
        
        ## Creamos todos los menus desplegables (que se muestran al hacer click en algun lugar)
        self.pointMenu = QMenu(self) # este se mostrara al hacer click en algun punto de las trayectorias (los puntos verdes)
        self.movePoint = self.pointMenu.addAction("Move")
        self.editPoint = self.pointMenu.addAction("Edit")
        self.deletePoint = self.pointMenu.addAction("Delete")
        self.vibratoMenu = QMenu(self) # este en algun punto que represente un vibrato (los puntos rojos)
        self.moveVibrato = self.vibratoMenu.addAction("Move")
        self.editVibrato = self.vibratoMenu.addAction("Edit")
        self.deleteVibrato = self.vibratoMenu.addAction("Delete")
        self.filterMenu = QMenu(self) # este en algun punto que represente un filtro (los puntos azules)
        self.moveFilter = self.filterMenu.addAction("Move")
        self.editFilter = self.filterMenu.addAction("Edit")
        self.deleteFilter = self.filterMenu.addAction("Delete")
        self.graphMenu = QMenu(self) # este en algun lugar de la curva (que no sea sobre un punto verde, azul o rojo)
        self.addPoint = self.graphMenu.addAction("Add point")
        self.addVibrato = self.graphMenu.addAction("Add vibrato")
        self.addFilter = self.graphMenu.addAction("Add filter")
        self.openTable = self.graphMenu.addAction("Open as table")
        self.noteMenu = QMenu(self) # este es parecido al self.graphMenu, pero para el quinto grafico (el de las notas)
        self.addNote = self.noteMenu.addAction("Add note")
        self.addTrill = self.noteMenu.addAction("Add trill")
        self.openNotesTable = self.noteMenu.addAction("Open as table")
        self.tongueSurfaceMenu = QMenu(self) # este es parecido al self.graphMenu, pero para el quinto grafico (el de las notas)
        self.tongueSurfaceMenuPoint = self.tongueSurfaceMenu.addAction("Add point")
        self.tongueSurfaceMenuTable = self.tongueSurfaceMenu.addAction("Open as table")

        # estas listas se usan para indicar si se esta moviendo actualmente un punto en cada uno de los gráficos. Se usan como condicion para decidir hacer algo cuando se esta moviendo el mouse por encima de un gráfico
        self.moving_point = [False for i in range(7)]
        self.moving_vibrato = [False for i in range(7)]
        self.moving_filter = [False for i in range(7)]
        self.segundo_click = [False for i in range(7)] # se usa para soltar los puntos

        # en caso de que se esté moviendo un punto (listas anteriores) estas listas indican qué punto es
        self.moving_point_index = [0 for i in range(7)]
        self.moving_vibrato_index = [0 for i in range(7)]
        self.moving_filter_index = [0 for i in range(7)]

        self.graphicsView.scene().sigMouseClicked.connect(self.mouse_clicked) # cuando se hace click en alguna parte del graphicsView que no sea sobre un punto o una curva
        self.graphicsView.scene().sigMouseMoved.connect(self.mouse_moved) # cuando se mueve por encima del graphicsView

        self.graphicsView_2.scene().sigMouseClicked.connect(self.mouse_clicked2) # cuando se hace click en alguna parte del graphicsView_2 que no sea sobre un punto o una curva
        self.graphicsView_2.scene().sigMouseMoved.connect(self.mouse_moved2) # cuando se mueve por encima del graphicsView_2

        self.graphicsView_3.scene().sigMouseClicked.connect(self.mouse_clicked3)
        self.graphicsView_3.scene().sigMouseMoved.connect(self.mouse_moved3)

        self.graphicsView_4.scene().sigMouseClicked.connect(self.mouse_clicked4)
        self.graphicsView_4.scene().sigMouseMoved.connect(self.mouse_moved4)

        self.graphicsView_5.scene().sigMouseClicked.connect(self.mouse_clicked5)
        self.graphicsView_5.scene().sigMouseMoved.connect(self.mouse_moved5)

        self.graphicsView_6.scene().sigMouseClicked.connect(self.mouse_clicked6)
        self.graphicsView_6.scene().sigMouseMoved.connect(self.mouse_moved6)

        self.graphicsView_7.scene().sigMouseClicked.connect(self.mouse_clicked7)
        self.graphicsView_7.scene().sigMouseMoved.connect(self.mouse_moved7)

        ## conectamos los botones a sus funciones y agregamos tooltips
        self.statesFromNotesButton.clicked.connect(self.get_states_from_notes)
        self.statesFromNotesButton.setToolTip("Generar trayectorias (l, θ, offset, flow) a partir de las notas definidas")
        self.changeDurationButton.clicked.connect(self.change_score_duration)
        self.changeDurationButton.setToolTip("Cambiar la duración total de la partitura")
        self.addCorrectionButton.clicked.connect(self.add_correction)
        self.addCorrectionButton.setToolTip("Aplicar corrección (offset/escala) a un eje completo")
        self.scaleTimeButton.clicked.connect(self.scale_time)
        self.scaleTimeButton.setToolTip("Comprimir o expandir el tiempo de la partitura")
        self.seeMotorRefsButton.clicked.connect(self.see_motor_refs)
        self.seeMotorRefsButton.setToolTip("Ver referencias de posición y velocidad para cada motor (X, Z, Alpha)")
        self.manualControlButton.clicked.connect(self.open_manual_control)
        self.manualControlButton.setToolTip("Abrir control manual del robot (mover ejes individualmente)")
        self.psControlButton.clicked.connect(self.ps_control)
        self.psControlButton.setToolTip("Controlar el robot con PS Move")
        self.softStopButton.clicked.connect(self.soft_stop)
        self.softStopButton.setToolTip("EMERGENCIA: Detener todos los motores inmediatamente") 
        self.process_running = Event() # se usa este evento mientras se corre el script de ps move
        ## se crean algunas variables para leer la informacion obtenida del script de ps move
        self.ps_serial1 = ""
        self.ps_tracked1 = False
        self.ps_serial2 = ""
        self.ps_tracked2 = False

        ## conectamos otras funciones
        self.actionNew.triggered.connect(self.new_file) # archivo nuevo
        self.actionSave.triggered.connect(self.save) # guardar archivo
        self.actionSave_as.triggered.connect(self.save_as) # guardar como
        self.actionOpen.triggered.connect(self.open) # abrir
        self.goToCoursorButton.clicked.connect(self.go_to_coursor)
        self.goToCoursorButton.setToolTip("Mover el robot a la posición del cursor")
        self.playButton.clicked.connect(self.play)
        self.playButton.setToolTip("Ejecutar partitura desde el cursor (primero usar 'Go to cursor')")
        self.stopButton.clicked.connect(self.stop)
        self.stopButton.setToolTip("Detener ejecución de la partitura")
        self.stop_playing.connect(self.stop) # conectamos la señal stop_playing que se emite cuando la partitura llegó hasta el final (la emite un thread diferente al thread principal)
        self.refresh_plots_signal.connect(self.refresh_plots) # conectamos la señal refresh_plots_signal que se emite cuando se tienen que actualizar uno o varios gráficos desde un thread distinto al principal
        self.clearPlotButton.clicked.connect(self.clear_plot) # para limpiar un grafico (borrar las mediciones obtenidas durante una ejecucion)
        self.clearPlotButton.setEnabled(False)
        self.free_recording = False
        self.startFreeRocordingButton.clicked.connect(self.start_free_recording)

        self.base_path = os.path.dirname(os.path.realpath(__file__)) # ubicacion del archivo dentro del sistema de archivos del computador
        self.filename = None # nombre con el que se guarda una partitura
        self.find_recent_files() # lee el archivo de recent_saves.txt para poblar el menu de archivos abiertos recientemente

        ## conectamos las acciones que se encuentran en el menu Plot. Cada una de estas abre una ventana que se actualiza en tiempo real con cada una de las mediciones que corresponda
        self.actionLip_to_edge_distance.triggered.connect(self.measure_radius)
        self.actionIncident_angle.triggered.connect(self.measure_theta)
        self.actionOffset.triggered.connect(self.measure_offset)
        self.actionPosition.triggered.connect(self.measure_position)
        self.actionMouth_Pressure.triggered.connect(self.measure_mouth_presure)
        self.actionMass_Flow_Rate.triggered.connect(self.measure_mass_flow_rate)
        self.actionVolume_Flow_Rate.triggered.connect(self.measure_volume_flow_rate)
        self.actionAir_Temperature.triggered.connect(self.measure_temperature)
        self.actionSound_Frequency.triggered.connect(self.measure_sound_frequency)
        self.actionX.triggered.connect(self.measure_x_position)
        self.actionZ.triggered.connect(self.measure_z_position)
        self.actionAlpha.triggered.connect(self.measure_alpha_position)

        ## algunas variables para el manejo de versiones (poder hacer undo y redo)
        self.undo_list = []
        self.redo_list = []
        self.actionUndo.triggered.connect(self.undo) # Estas tambien estan conectadas con los atajos cntrl+z y cntrl+y
        self.actionRedo.triggered.connect(self.redo) #

        ## Algunas variables para definir el espacio de instruccion. Normalemente los gráficos representan l, theta y offset; pero se pueden cambiar a x, z y alpha.
        self.space_of_instruction = 0 # 0 = espacio de la tarea, 1 = espacio de las junturas
        self.actionChange_to_joint_space.triggered.connect(self.change_to_joint_space)
        self.actionChange_to_task_space.triggered.connect(self.change_to_task_space)

        ## conectamos la accion de abrir ajustes
        self.actionSettings.triggered.connect(self.change_settings) # esto abre un formulario con todos los ajustes posibles

        self.populate_graph() # creamos una version inicial de las trayectorias
        r = self.get_copy_of_routes(self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7, self.graphicsView_8.route) # creamos una copia del estado actual de las rutas para cada eje y la almacenamos en la lista de los estados para hacer undo.
        self.undo_list.append(r)
        
        # Actualizamos los gráficos los graficos
        self.refresh_plots_signal.emit([1,2,3,4,5,6,7])

        self.setWindowTitle("Pierre - Flutist Robot")

        # Actualizar LED de conexión según el estado inicial
        self._set_led(self._led_connected, self.connected)

        # Barra de estado inferior con información contextual
        self._status_space_label = QLabel("Espacio: Tarea")
        self._status_file_label = QLabel("Sin guardar")
        self._status_time_label = QLabel("t: 0.0s")
        for lbl in (self._status_space_label, self._status_file_label, self._status_time_label):
            lbl.setStyleSheet("padding: 0 12px; color: #455A64;")
        self.statusbar.addWidget(self._status_file_label, 1)
        self.statusbar.addWidget(self._status_space_label)
        self.statusbar.addWidget(self._status_time_label)
        self.file_saved = True # esta variable se hace falsa cuando el archivo es modificado. Entonces se agrega un asterisco al titulo de la ventana y antes de cerrar o abrir otra partitura se pregunta al usuario si quiere guardar los cambios

    def _rebuild_side_panel(self):
        """Reorganiza el panel lateral derecho con grupos lógicos y SOFT STOP prominente."""
        # Remover todos los widgets e items del layout original
        while self.verticalLayout_2.count():
            item = self.verticalLayout_2.takeAt(0)
            # No destruir los widgets, solo quitarlos del layout
            if item.widget():
                item.widget().setParent(None)
            elif item.layout():
                # Quitar widgets de sub-layouts (horizontalLayout_4, horizontalLayout_2)
                sub = item.layout()
                while sub.count():
                    sub_item = sub.takeAt(0)
                    if sub_item.widget():
                        sub_item.widget().setParent(None)

        def _make_separator():
            sep = QFrame()
            sep.setFrameShape(QFrame.HLine)
            sep.setFixedHeight(1)
            return sep

        def _make_group_label(text):
            lbl = QLabel(text)
            lbl.setStyleSheet("color: #455A64; font-size: 8pt; font-weight: 700; "
                              "text-transform: uppercase; padding: 4px 0px 2px 0px;")
            return lbl

        # --- SOFT STOP arriba de todo ---
        self.verticalLayout_2.addWidget(self.softStopButton)

        self.verticalLayout_2.addWidget(_make_separator())

        # --- Panel de estado (LEDs) ---
        self.verticalLayout_2.addWidget(_make_group_label("ESTADO"))
        self._led_connected = self._make_led("Conectado", active=False)
        self._led_in_position = self._make_led("En posición", active=False)
        self._led_playing = self._make_led("Ejecutando", active=False)
        self.verticalLayout_2.addWidget(self._led_connected)
        self.verticalLayout_2.addWidget(self._led_in_position)
        self.verticalLayout_2.addWidget(self._led_playing)

        self.verticalLayout_2.addWidget(_make_separator())

        # --- Grupo: Transporte ---
        self.verticalLayout_2.addWidget(_make_group_label("TRANSPORTE"))
        self.verticalLayout_2.addWidget(self.goToCoursorButton)
        transport_row = QHBoxLayout()
        transport_row.addWidget(self.playButton)
        transport_row.addWidget(self.stopButton)
        self.verticalLayout_2.addLayout(transport_row)
        self.verticalLayout_2.addWidget(self.recordCheckBox)

        self.verticalLayout_2.addWidget(_make_separator())

        # --- Grupo: Grabación ---
        self.verticalLayout_2.addWidget(_make_group_label("GRABACIÓN"))
        self.verticalLayout_2.addWidget(self.clearPlotButton)
        self.verticalLayout_2.addWidget(self.startFreeRocordingButton)

        self.verticalLayout_2.addWidget(_make_separator())

        # --- Grupo: Edición ---
        self.verticalLayout_2.addWidget(_make_group_label("EDICIÓN"))
        self.verticalLayout_2.addWidget(self.statesFromNotesButton)
        edit_row = QHBoxLayout()
        edit_row.addWidget(self.addCorrectionButton)
        edit_row.addWidget(self.scaleTimeButton)
        self.verticalLayout_2.addLayout(edit_row)
        self.verticalLayout_2.addWidget(self.changeDurationButton)

        self.verticalLayout_2.addWidget(_make_separator())

        # --- Grupo: Herramientas ---
        self.verticalLayout_2.addWidget(_make_group_label("HERRAMIENTAS"))
        self.verticalLayout_2.addWidget(self.seeMotorRefsButton)
        self.verticalLayout_2.addWidget(self.manualControlButton)
        self.verticalLayout_2.addWidget(self.psControlButton)

        # Spacer al fondo para empujar todo arriba
        self.verticalLayout_2.addStretch(1)

    def _make_led(self, text, active=False):
        """Crea un QLabel con indicador LED (círculo de color + texto)."""
        lbl = QLabel(f"  {text}")
        color = "#00C853" if active else "#BDBDBD"
        lbl.setStyleSheet(
            f"padding: 2px 4px; font-size: 9pt; color: #212121; "
            f"border: none; background: transparent;"
        )
        lbl.setProperty("led_color", color)
        # Usamos un prefijo Unicode como indicador visual
        lbl.setText(f"\u25CF  {text}")
        lbl.setStyleSheet(
            f"padding: 2px 4px; font-size: 9pt; color: {color}; "
            f"border: none; background: transparent; font-weight: 600;"
        )
        return lbl

    def _set_led(self, led, active):
        """Actualiza el color de un LED indicator."""
        text = led.text()[3:]  # quitar el prefijo "●  "
        color = "#00C853" if active else "#BDBDBD"
        led.setText(f"\u25CF  {text}")
        led.setStyleSheet(
            f"padding: 2px 4px; font-size: 9pt; color: {color}; "
            f"border: none; background: transparent; font-weight: 600;"
        )

    def start_free_recording(self):
        self.free_recording = not self.free_recording
        if self.free_recording:
            self.goToCoursorButton.setEnabled(False)
            self.playButton.setEnabled(False)
            self.startFreeRocordingButton.setText("Stop Recording...")
            self.startFreeRocordingButton.setStyleSheet("background-color : red")
            self.musician_pipe.send(["start_free_recording"])
        else:
            self.goToCoursorButton.setEnabled(True)
            self.startFreeRocordingButton.setText("Start Free Record")
            self.startFreeRocordingButton.setStyleSheet("")
            directory = "recordings/free_recording/" + datetime.now().strftime("%Y-%m-%d at %Hh%Mm%Ss")
            os.mkdir(directory)
            self.musician_pipe.send(["stop_free_recording", directory+'/data.csv', directory+'/audio.wav'])

    def refresh_plots(self, plot_list): # se ejecuta cuando se emite la señal refresh_plots_signal, simplemente llama las 6 funciones que plotean en los graphicViews.
        if 2 in plot_list:
            self.reprint_plot_2()
        if 3 in plot_list:
            self.reprint_plot_3()
        if 4 in plot_list:
            self.reprint_plot_4()
        if 5 in plot_list:
            self.reprint_plot_5()
        if 6 in plot_list:
            self.reprint_plot_6()
        if 7 in plot_list:
            self.reprint_plot_7()
        if 1 in plot_list:
            self.reprint_plot_1()
        if 'r' in plot_list:
            self.reprint_real_func() # esta plotea las curvas de las mediciones reales

    def get_copy_of_routes(self, r1, r2, r3, r4, r5, r6, r7, r8): # entrega copias de los diccionarios para almacenar versiones de los estados. Se hacen copias para que no se alteren cuando se realizan cambios y se pueda volver atras
        r1_copy = {'total_t': r1['total_t'], 'Fs': r1['Fs'], 'points': r1['points'].copy(), 'filters': r1['filters'].copy(), 'vibrato': r1['vibrato'].copy(), 'history': r1['history'].copy()}
        r2_copy = {'total_t': r2['total_t'], 'Fs': r2['Fs'], 'points': r2['points'].copy(), 'filters': r2['filters'].copy(), 'vibrato': r2['vibrato'].copy(), 'history': r2['history'].copy()}
        r3_copy = {'total_t': r3['total_t'], 'Fs': r3['Fs'], 'points': r3['points'].copy(), 'filters': r3['filters'].copy(), 'vibrato': r3['vibrato'].copy(), 'history': r3['history'].copy()}
        r4_copy = {'total_t': r4['total_t'], 'Fs': r4['Fs'], 'points': r4['points'].copy(), 'filters': r4['filters'].copy(), 'vibrato': r4['vibrato'].copy(), 'history': r4['history'].copy()}
        r5_copy = {'total_t': r5['total_t'], 'Fs': r5['Fs'], 'notes': r5['notes'].copy(), 'trill': r5['trill'].copy(), 'history': r5['history'].copy()}
        r6_copy = {'total_t': r6['total_t'], 'Fs': r6['Fs'], 'points': r6['points'].copy(), 'history': r6['history'].copy()}
        r7_copy = {'total_t': r7['total_t'], 'Fs': r7['Fs'], 'points': r7['points'].copy(), 'history': r7['history'].copy()}
        r8_copy = {'total_t': r8['total_t'], 'Fs': r8['Fs'], 'points': r8['points'].copy(), 'filters': r8['filters'].copy(), 'vibrato': r8['vibrato'].copy(), 'history': r8['history'].copy()}
        return [r1_copy, r2_copy, r3_copy, r4_copy, r5_copy, r6_copy, r7_copy, r8_copy]

    def ps_control(self): # se ejecuta al presionar el boton de PS Control, e inicia un thread.
        self.process_running.set()
        thread = threading.Thread(target=self.psmove)
        thread.start()

    def get_l_theta_of_ps(self, x,z,alpha,xf,zf): # obtiene l, theta y offset a partir de x,z,alpha,xf,zf. Copia de funcion que se tiene en cinematica.py
        alpha_f = pi/4
        alpha = alpha * pi / 180
        theta = alpha + alpha_f
        l = (xf - x) * cos(alpha) + (zf - z) * sin(alpha)
        of = - (xf - x) * sin(alpha) + (zf - z) * cos(alpha)
        return l, theta*180/pi, of
    
    def ps_play_tune(self): # se ejecuta como un thread para cambiar notas de forma automatica (se usan las notas entregadas en la partitura)
        t0 = time()
        for note in self.route5["notes"]:
            while self.ps_playing_tune.is_set(): # si el usuario decide dejar de tocar la cancion puede apretar cuadrado y se limpia este evento
                if time() - t0 >= note[0]: # si el tiempo actual es mayor al tiempo de la nota que toca, se envia la instruccion de digitar tal nota
                    self.musician_pipe.send(['execute_fingers_action', dict_notes_rev[note[1]], False])
                    break # se sale del loop while y avanzamos a la siguiente nota.
                sleep(0.01)

    def psmove(self):
        # Ejecutar el programa en C precompilado.
        # Ver documentación de psmoveapi para ver como compilar estos ejecutables. Los codigos fueron clonados de su github y modificados localmente. En /src/psmovapi se encuentran los scripts con los cambios
        process = subprocess.Popen("C:/Users/ferna/Documents/psmoveapi/build-x64/psmove.exe test-tracker", stdout=subprocess.PIPE) # ejecutamos el programa precompilado
        
        # Creamos unos arreglos con los ultimos valores de l, theta y offset. Esto lo hacemos para usar filtros y evitar movimientos muy temblorosos por el ruido de las señales.
        self.ps_l_hist = self.data['radius'][-1]*np.ones([1000])
        self.ps_theta_hist = self.data['theta'][-1]*np.ones([1000])
        self.ps_of_hist = self.data['offset'][-1]*np.ones([1000])
        
        # parametros del filtro
        Fs = 100
        fp = 0.05
        fs = 0.1
        fc = (fp+fs)/2
        n  = 100
        self.ps_flt = signal.firwin(numtaps=n, cutoff=fc, window="hamming", pass_zero="lowpass", fs=Fs)
        self.ps_A = [1] +  [0 for i in range(n-1)]

        # Evento que señala cuando se esta tocando una partitura mientras se controla el robot con el comando de ps move (para hacer el cambio automatico de nota)
        self.ps_playing_tune = threading.Event()

        self.ps_playing = False # este bool dice si el robot esta siendo controlado por el usuario mediante el movimiento del control. El usuario puede activarlo y desactivarlo apretando el boton Move (boton del centro del control)
        self.ps_last_Btn_MOVE = False # se usa para detectar cambios de estado del Btn_MOVE
        self.ps_last_Btn_CROSS = False # se usa para detectar cambios de estado del Btn_CROSS
        self.ps_last_Btn_CIRCLE = False # se usa para detectar cambios de estado del Btn_CIRCLE

        self.ps_last_Btn_CROSS2 = False
        self.ps_last_Btn_CIRCLE2 = False
        self.ps_last_Btn_SQUARE2 = False
        self.ps_last_Btn_TRIANGLE2 = False
        self.ps_last_Btn_MOVE2 = False
        self.ps_last_trigger_v2 = 0
        self.ps_melodia = [] # lista donde se agregan todas las notas que estan escritas en la partitura al momento de ejecutar el script del control del robot mediante ps move. El usuario despues puede avanzar nota por nota apretando el boton Btn_CROSS o activar un thread que las empieza a pasar de forma automatica apretando el boton Btn_CIRCLE
        for note in self.route5["notes"]:
            self.ps_melodia.append(note[1])
        self.ps_melodia_index = 0
        if len(self.ps_melodia):
            self.musician_pipe.send(['execute_fingers_action', dict_notes_rev[self.ps_melodia[self.ps_melodia_index]], False])
        while self.process_running.is_set():
            # Leer la salida del programa C
            output = process.stdout.readline().decode("utf-8").strip()
            
            # Comprobar si se ha alcanzado el estado final
            if output == "estado_final": # el programa imprime estado_final cuando se cierra. Esto permite salir del loop cuando se cierra la pestaña del ps move
                self.process_running.clear()
                break
            
            # Hacer algo con el estado
            palabras = output.split()
            # el script va informando dos cosas: state y state2. El primero tiene informacion de los sensores del control (acelerometros, giroscopios) asi como sus botones. El segundo tiene información del tracking de los controles. Los valores se entregan separados por espacios. 
            if palabras[0] == "state":
                self.ps_serial1 = palabras[1] ## el primer valor informa cual de los dos controles esta siendo medido. El programa solo informa de uno de los dos, porque el otro solo se usa en el trackeo. El que se mide es el que representa la boca del robot y es el que el usuario maneja con la mano.
                self.ps_trigger_v = int(palabras[2]) / 255 # cuanto se presiona el trigger del control (el boton de atras) normalizado
                ## acelerometro
                self.ps_accel_x = int(palabras[3]) / 4500
                self.ps_accel_y = int(palabras[4]) / 4500
                self.ps_accel_z = int(palabras[5]) / 4500
                ## Giroscopio
                self.ps_gyro_x = int(palabras[6]) / 4500
                self.ps_gyro_y = int(palabras[7]) / 4500
                self.ps_gyro_z = int(palabras[8]) / 4500
                ## Magnetometro
                self.ps_magneto_x = int(palabras[9]) / 255
                self.ps_magneto_y = int(palabras[10]) / 255
                self.ps_magneto_z = int(palabras[11]) / 255
                ## Botones
                botones = int(palabras[12], 16)
                self.ps_Btn_TRIANGLE = botones & 1 << 4 != 0
                self.ps_Btn_CIRCLE = botones & 1 << 5 != 0
                self.ps_Btn_CROSS = botones & 1 << 6 != 0
                self.ps_Btn_SQUARE = botones & 1 << 7 != 0

                self.ps_Btn_SELECT = botones & 1 << 8 != 0
                self.ps_Btn_START = botones & 1 << 11 != 0

                self.ps_Btn_PS = botones & 1 << 16 != 0
                self.ps_Btn_MOVE = botones & 1 << 19 != 0
                self.ps_Btn_T = botones & 1 << 20 != 0

                botones2 = int(palabras[13], 16)
                self.ps_Btn_TRIANGLE2 = botones2 & 1 << 4 != 0
                self.ps_Btn_CIRCLE2 = botones2 & 1 << 5 != 0
                self.ps_Btn_CROSS2 = botones2 & 1 << 6 != 0
                self.ps_Btn_SQUARE2 = botones2 & 1 << 7 != 0

                self.ps_Btn_SELECT2 = botones2 & 1 << 8 != 0
                self.ps_Btn_START2 = botones2 & 1 << 11 != 0

                self.ps_Btn_PS2 = botones2 & 1 << 16 != 0
                self.ps_Btn_MOVE2 = botones2 & 1 << 19 != 0
                self.ps_Btn_T2 = botones2 & 1 << 20 != 0

                self.ps_trigger_v2 = int(round(int(palabras[14]) / 255 * 31, 0))

                if self.ps_Btn_MOVE and not self.ps_last_Btn_MOVE: # si se apreta el boton move cambia el estado del control del robot (empieza a seguir lo que se le pide de acuerdo al movimiento del control o deja de seguirlo)
                    self.ps_playing = not self.ps_playing
                if self.ps_Btn_CIRCLE and not self.ps_last_Btn_CIRCLE: # si se apreta el boton circulo se comienza o se deja de tocar en forma automatica las notas de la melodia
                    if self.ps_playing_tune.is_set():
                        self.ps_playing_tune.clear()
                    else:
                        self.ps_playing_tune.set()
                        thread2 = threading.Thread(target=self.ps_play_tune)
                        thread2.start()
                if self.ps_Btn_CROSS and not self.ps_last_Btn_CROSS: # si se apreta cross se pueden cambiar las notas de la melodia de forma manual (una a una) hacia adelante
                    self.ps_melodia_index = (self.ps_melodia_index + 1) % len(self.ps_melodia)
                    if len(self.ps_melodia):
                        self.musician_pipe.send(['execute_fingers_action', dict_notes_rev[self.ps_melodia[self.ps_melodia_index]], False])
                if self.ps_Btn_SQUARE and not self.ps_last_Btn_SQUARE: # si se apreta cross se pueden cambiar las notas de la melodia de forma manual (una a una) hacia atras
                    self.ps_melodia_index = (self.ps_melodia_index - 1) % len(self.ps_melodia)
                    if len(self.ps_melodia):
                        self.musician_pipe.send(['execute_fingers_action', dict_notes_rev[self.ps_melodia[self.ps_melodia_index]], False])
                
                

                self.ps_last_Btn_MOVE = self.ps_Btn_MOVE
                self.ps_last_Btn_CROSS = self.ps_Btn_CROSS
                self.ps_last_Btn_CIRCLE = self.ps_Btn_CIRCLE
                self.ps_last_Btn_SQUARE = self.ps_Btn_SQUARE

                

            elif palabras[0] == "state2":
                if palabras[1] == self.ps_serial1: # si la informacion que entrega es del control que representa la boca
                    self.ps_tracked1 = palabras[2] == "1" # si el control se encuentra en la imagen, este valor es 1 y se informa su posicion
                    if self.ps_tracked1:
                        self.ps_x1 = float(palabras[3])
                        self.ps_y1 = float(palabras[4])
                        self.ps_r1 = float(palabras[5])
                else: # si la informacion que entrega es del control que representa el bisel de la flauta
                    self.ps_serial2 = palabras[1]
                    self.ps_tracked2 = palabras[2] == "1" # si el control se encuentra en la imagen, este valor es 1 y se informa su posicion
                    if self.ps_tracked2:
                        self.ps_x2 = float(palabras[3])
                        self.ps_y2 = float(palabras[4])
                        self.ps_r2 = float(palabras[5])
            elif palabras[0] == "state3":
                pass
            if self.ps_tracked1 and self.ps_tracked2 and self.ps_playing: # si ambos controles estan siendo trackeados y se tiene activado ps_playing, entonces se mueve el robot conforme a lo que se pide
                if self.ps_last_trigger_v2 != self.ps_trigger_v2:
                    self.musician_pipe.send(['execute_lips_surface_action', 31 - self.ps_trigger_v2])
                if self.ps_Btn_CROSS2 and not self.ps_last_Btn_CROSS2:
                    self.musician_pipe.send(['execute_tongue_action', 1])
                if self.ps_Btn_SQUARE2 and not self.ps_last_Btn_SQUARE2:
                    self.musician_pipe.send(['execute_tongue_action', 2])
                if self.ps_Btn_MOVE2 and not self.ps_last_Btn_MOVE2:
                    self.musician_pipe.send(['execute_tongue_action', 3])
                if not self.ps_Btn_CROSS2 and self.ps_last_Btn_CROSS2 or not self.ps_Btn_SQUARE2 and self.ps_last_Btn_SQUARE2 or not self.ps_Btn_MOVE2 and self.ps_last_Btn_MOVE2:
                    self.musician_pipe.send(['execute_tongue_action', 0])
                self.ps_last_Btn_CROSS2 = self.ps_Btn_CROSS2
                self.ps_last_Btn_CIRCLE2 = self.ps_Btn_CIRCLE2
                self.ps_last_Btn_SQUARE2 = self.ps_Btn_SQUARE2
                self.ps_last_Btn_TRIANGLE2 = self.ps_Btn_TRIANGLE2
                self.ps_last_Btn_MOVE2 = self.ps_Btn_MOVE2

                self.ps_last_trigger_v2 = self.ps_trigger_v2
                    
                flow = self.ps_trigger_v*50 # el flujo se indica con el trigger (más presionado es más flujo)
                if self.ps_Btn_TRIANGLE: # si el usuario apreta cuadrado se agrega un vibrato. Sin embargo no es posible modificar su amplitud ni su frecuencia
                    flow += flow*0.1*np.sin(2*np.pi*5*time())
                ## a partir de  x, z, self.ps_accel_y, xf y zf, las posiciones de los comandos de ps move que representan la boca del robot y el bisel de la flauta (entregadas como pixeles en la imagen), se escalan los valores para dirigir el movimiento del robot
                alpha = -self.ps_accel_y*45
                l, theta, of = self.get_l_theta_of_ps(self.ps_x1,self.ps_y1,alpha,self.ps_x2,self.ps_y2)
                of = of/25 # cada px de la imagen se escala en 1/25 para el offset
                l  = max(0,l-20) / 25 # cada px de la imagen se escala en 1/25 para el largo del jet

                self.ps_theta_hist = np.hstack([self.ps_theta_hist[1:], theta]) # actualizamos la lista de los ultimos valores para theta
                theta_filtrado = signal.lfilter(self.ps_flt, self.ps_A, self.ps_theta_hist) # aplicamos un filtro pasa bajos para suavizar los cambios. El desfase de grupo no debe ser muy grande para que no se demore tanto en reaccionar el robot
                self.ps_l_hist = np.hstack([self.ps_l_hist[1:], l]) # mismo procedimiento para l y offset
                l_filtrado = signal.lfilter(self.ps_flt, self.ps_A, self.ps_l_hist)
                self.ps_of_hist = np.hstack([self.ps_of_hist[1:], of])
                of_filtrado = signal.lfilter(self.ps_flt, self.ps_A, self.ps_of_hist)

                desired_state = State(l_filtrado[-1], theta_filtrado[-1], of_filtrado[-1], flow) # se crea un estado referencia, con los valores filtrados
                self.musician_pipe.send(["move_to_final", desired_state])
            elif self.ps_tracked1 and self.ps_tracked2: # si no se esta persiguiendo el control (ps_playing = False), pero se tiene un trackeo de los controles, igual se acumula su historia. Así cuando se quiere empezar a seguir no hay problemas con discontinuidades por el filtro
                self.ps_theta_hist = np.hstack([self.ps_theta_hist[1:], self.ps_theta_hist[-1]])
                self.ps_l_hist = np.hstack([self.ps_l_hist[1:], self.ps_l_hist[-1]])
                self.ps_of_hist = np.hstack([self.ps_of_hist[1:], self.ps_of_hist[-1]])
            

        # Esperar a que el proceso C termine
        process.kill() # si se sale del loop (porque se cierra la aplicacion por ejemplo) se termina el proceso
        self.ps_tracked1 = False
        self.ps_tracked2 = False
        self.ps_serial1 = ""
        self.ps_serial2 = ""

        # Obtener el código de salida del programa C
        return_code = process.returncode
        print("Programa C terminado con código de salida:", return_code)

    def undo(self): # vuelve al estado anterior de la partitura (antes del ultimo cambio)
        if len(self.undo_list) >= 2: # la lista self.undo_list tiene como su ultimo elemento el estado actual, por lo tanto para volver atras tiene que haber al menos 2 elementos en la lista
            self.redo_list.append(self.undo_list.pop()) # sacamos de la lista de undo el estado actual y lo agregamos a la lista de redo
            r = self.get_copy_of_routes(self.undo_list[-1][0], self.undo_list[-1][1], self.undo_list[-1][2], self.undo_list[-1][3], self.undo_list[-1][4], self.undo_list[-1][5], self.undo_list[-1][6], self.undo_list[-1][7]) # hacemos una copia del estado anterior
            # y asignamos este estado como el estado actual
            self.route = r[0]
            self.route2 = r[1]
            self.route3 = r[2]
            self.route4 = r[3]
            self.route5 = r[4]
            self.route6 = r[5]
            self.route7 = r[6]
            self.graphicsView_8.route = r[7]
            self.refresh_plots_signal.emit([1,2,3,4,5,6,7]) # se actualizan los graficos
            self.graphicsView_8.refresh()
            self.changes_made(from_hist=True) # y se informa que hubo cambios

    def redo(self): # vuelve al estado antes del undo
        if len(self.redo_list) >= 1: # a diferencia de la lista del undo, redo_list no contiene el estado actual
            to = self.redo_list.pop() # sacamos el ultimo elemento de la lista
            self.undo_list.append(to) # y lo agregamos a la lista del undo. Este será el estado nuevo
            r = self.get_copy_of_routes(to[0], to[1], to[2], to[3], to[4], to[5], to[6], to[7]) # creamos una copia
            # y lo asignamos como el estado actual
            self.route = r[0] 
            self.route2 = r[1]
            self.route3 = r[2]
            self.route4 = r[3]
            self.route5 = r[4]
            self.route6 = r[5]
            self.route7 = r[6]
            self.graphicsView_8.route = r[7]
            self.refresh_plots_signal.emit([1,2,3,4,5,6,7]) # se actualizan los graficos
            self.graphicsView_8.refresh()
            self.changes_made(from_hist=True) # y se informa que hubo cambios

    def change_to_joint_space(self):
        self.space_of_instruction = 1
        self.graphicsView.setLabel('left', "X", units='mm')
        self.graphicsView_2.setLabel('left', "Z", units='mm')
        self.graphicsView_3.setLabel('left', "Alpha", units='°')
        self.checkBox.setText("X")
        self.checkBox_2.setText("Z")
        self.checkBox_3.setText("Alpha")
        self._status_space_label.setText("Espacio: Articular")

    def change_to_task_space(self):
        self.space_of_instruction = 0
        self.graphicsView.setLabel('left', 'r', units='mm')
        self.graphicsView_2.setLabel('left', u"\u03b8", units='°')
        self.graphicsView_3.setLabel('left', "Offset", units='mm')
        self.checkBox.setText("Lip-to-edge distance")
        self.checkBox_2.setText("Angle of incidence")
        self.checkBox_3.setText("Offset")
        self._status_space_label.setText("Espacio: Tarea")

    def soft_stop(self): # detiene los movimientos que se esten realizando y lleva el flujo a cero
        self.musician_pipe.send(["stop"])
        if self.playing.is_set(): # si se estaba tocando una melodía, tambien se detiene
            self.stop()

    def open_manual_control(self): # abre la ventana con los controles para manejar de forma manual el robot
        manual_control = ManualWindow(self.app, self.musician_pipe, self.data, parent=self)
        manual_control.setWindowTitle("Manual Control")
        manual_control.show()

    def clear_plot(self): # borra los gráficos de las mediciones de las distintas variables que fueron ploteadas durante una ejecución de la partitura
        # primero vaciamos las listas que contienen la informacion de estas curvas
        self.r_plot = np.array([])
        self.theta_plot = np.array([])
        self.offset_plot = np.array([])
        self.flow_plot = np.array([])
        self.freq_plot = np.array([])
        self.t_plot = np.array([])
        self.clearPlotButton.setEnabled(False) # opcional
        # y luego actualizamos el gráfico
        self.refresh_plots_signal.emit(['r'])
        
    def go_to_coursor(self): # pre-carga las trayectorias para cada variable de la partitura (iniciando donde se encuentra el cursor) y lleva al robot al estado de la posicion del cursor en la partitura. Usa lineas rectas en el espacio que se esta usando (el de la tarea o el de las junturas)
        # en estas listas se escribirá la trayectoria
        flow_route = []
        x_route = []
        z_route = []
        alpha_route = []
        notes = []
        aperture = []
        tongue = []

        t, f_flow, p, vib, fil = calculate_route(self.route4) # calculamos la ruta para el flujo, que es independiente al espacio que se esta usando
        t, f_notes, xp, yp, tx, ty = calculate_notes_route(self.route5) # lo mismo para las notas
        t, f_aperture, px, py = calculate_route_tongue(self.route6)
        t, f_tongue, px, py = calculate_route_tongue(self.route7)
        ti_index = int(len(t) * self.horizontalSlider.value() / 100) # calculamos el tiempo de inicio de acuerdo a la posicion del cursor

        if self.space_of_instruction == 0: # si usamos el espacio de la tarea
            t, f_r, p, vib, fil = calculate_route(self.route) # con el primer grafico calculamos la ruta para l
            t, f_theta, p, vib, fil = calculate_route(self.route2) # con el segundp grafico calculamos la ruta para theta
            t, f_offset, p, vib, fil = calculate_route(self.route3) # con el tercer grafico calculamos la ruta para offset
            x_pos_ref, z_pos_ref, alpha_pos_ref = change_to_joint_space(f_r, f_theta, f_offset) # calculamos la trayectoria en terminos de x, z y alpha
            desired_state = State(f_r[ti_index], f_theta[ti_index], f_offset[ti_index], 0) # donde comienza la partitura. Estado al que queremos movernos antes de empezar a tocar

        elif self.space_of_instruction == 1: # si usamos el espacio de las junturas
            t, x_pos_ref, p, vib, fil = calculate_route(self.route) # con el primer grafico calculamos la ruta para x
            t, z_pos_ref, p, vib, fil = calculate_route(self.route2) # con el segundp grafico calculamos la ruta para z
            t, alpha_pos_ref, p, vib, fil = calculate_route(self.route3) # con el tercer grafico calculamos la ruta para alpha
            desired_state = State(0, 0, 0, 0) # donde comienza la partitura. Estado al que queremos movernos antes de empezar a tocar
            desired_state.x = x_pos_ref[ti_index]
            desired_state.z = z_pos_ref[ti_index]
            desired_state.alpha = alpha_pos_ref[ti_index]

        # ahora lo transformamos a pasos de los motores y calculamos sus gradientes para las velocidades. El gradiente se multiplica por el tiempo total por la normalizacion
        x_pos_ref = mm2units(x_pos_ref)
        x_vel_ref = gradient(x_pos_ref)*self.route['total_t']
        z_pos_ref = mm2units(z_pos_ref)
        z_vel_ref = gradient(z_pos_ref)*self.route['total_t']
        alpha_pos_ref = angle2units(alpha_pos_ref)
        alpha_vel_ref = gradient(alpha_pos_ref)*self.route['total_t']

        # con estas trayectorias podemos rellenar las listas que definimos al principio con el formato que usamos. Aca se rellena desde ti_index
        for i in range(len(t) - ti_index):
            flow_route.append([t[i], f_flow[i + ti_index]]) # se desfasa el tiempo en -ti_index
            x_route.append([t[i], x_pos_ref[i + ti_index], x_vel_ref[i + ti_index]])
            z_route.append([t[i], z_pos_ref[i + ti_index], z_vel_ref[i + ti_index]])
            alpha_route.append([t[i], alpha_pos_ref[i + ti_index], alpha_vel_ref[i + ti_index]])
            notes.append([t[i], f_notes[i + ti_index]])
            aperture.append([t[i], int(f_aperture[i + ti_index])])
            tongue.append([t[i], int(f_tongue[i + ti_index])])

        self.musician_pipe.send(["load_routes", x_route, z_route, alpha_route, flow_route, notes, aperture, tongue]) # pre-cargamos las rutas calculadas
        self.musician_pipe.send(["move_to", desired_state, None, False, False, False, False, 50]) # nos movemos al estado inicial
        x = threading.Thread(target=self.wait_musician_is_in_place, args=(desired_state,)) # creamos un thread que revisa cuando el robot llega al estado inicial. Una vez que llega a posición es posible pedirle que empiece a tocar la partitura
        x.start()
    
    def wait_musician_is_in_place(self, desired_state):
        if self.connected:
            while True:
                if abs(self.data['x'][-1] - desired_state.x) < 0.5 and abs(self.data['z'][-1] - desired_state.z) < 0.5  and abs(self.data['alpha'][-1] - desired_state.alpha) < 1:
                    self.playButton.setEnabled(True)
                    self._set_led(self._led_in_position, True)
                    break
                sleep(0.1)
        else:
            self.playButton.setEnabled(True)
            self._set_led(self._led_in_position, True)

    def play(self):
        self.playing.set()
        self.clearPlotButton.setEnabled(False)
        self.startFreeRocordingButton.setEnabled(False)
        rec = self.recordCheckBox.isChecked()
        self.musician_pipe.send(["start_loaded_script", rec])
        coursor = threading.Thread(target=self.move_coursor_and_plot, args=())
        coursor.start()
        self.stopButton.setEnabled(True)
        self.playButton.setEnabled(False)
        self.recordCheckBox.setEnabled(False)
        self.horizontalSlider.setEnabled(False)
        self._set_led(self._led_playing, True)
        self._set_led(self._led_in_position, False)
    
    def move_coursor_and_plot(self): # funcion que actualiza la posicion del cursor y plotea los valores medidos de cada variable
        t, f_r, p, vib, fil = calculate_route(self.route)
        slider_initial_value = self.horizontalSlider.value()
        ti = int(len(t) * slider_initial_value / 99) # indice inicial para el tiempo
        t_0 = time() # tiempo de inicio
        self.clear_plot() # limpiamos las listas donde se plotean los valores medidos
        last_t = self.data["times"][-1] # se actualiza en cada iteración
        first_t = self.data["times"][-1] # se mantiene constante, para restarlo y comenzar en 0
        while True:
            if self.playing.is_set():
                t_act = time() - t_0 # tiempo actual
                slider_move = slider_initial_value + (99 - slider_initial_value) * min(100, max(0, t_act / (t[-1] - t[ti]))) # nueva posicion del slider
                self.horizontalSlider.setValue(int(slider_move))
                last_index = self.data["times"].searchsorted(last_t) # self.data["times"] se va actualizando a una tasa distinta de la del ploteo, asique en cada iteración agregamos la información nueva que no estaba ploteada. Para esto buscamos el índice en el que quedo el último tiempo que se agregó al plot e incorporamos toda la información desde el índice siguiente 
                try:
                    if self.space_of_instruction == 0: # se plotea l, theta y offset
                        self.r_plot = np.hstack([self.r_plot, self.data["radius"][last_index+1:]])
                        self.theta_plot = np.hstack([self.theta_plot, self.data["theta"][last_index+1:]])
                        self.offset_plot = np.hstack([self.offset_plot, self.data["offset"][last_index+1:]])
                    elif self.space_of_instruction == 1: # se plotea x, z y alpha
                        self.r_plot = np.hstack([self.r_plot, self.data["x"][last_index+1:]])
                        self.theta_plot = np.hstack([self.theta_plot, self.data["z"][last_index+1:]])
                        self.offset_plot = np.hstack([self.offset_plot, self.data["alpha"][last_index+1:]])
                    self.flow_plot = np.hstack([self.flow_plot, self.data["mass_flow"][last_index+1:]]) # ploteamos el flujo (mass_flow)
                    self.freq_plot = np.hstack([self.freq_plot, (12*np.log2(self.data["frequency"][last_index+1:]) - 12*np.log2(440*2**(-7/12))) / 2]) # en el caso de las notas, se plotea el pitch detectado por el metodo YIN o pYIN
                    self.t_plot = np.hstack([self.t_plot, self.data["times"][last_index+1:] - first_t + t[ti]])
                    self.refresh_plots_signal.emit(['r']) # se actualizan los graficos
                except:
                    print("Hubo un error")
                last_t = self.data["times"][-1] # actualizamos el últumo tiempo que se ploteo
                if t_act + t[ti] > t[-1]: # una vez que llega al final de la partitura emite la señal de parar de tocar
                    self.stop_playing.emit()
                    break
                sleep(0.05) # tasa de actualizacion = 0.05s
            else: # si se sale por alguna otra razon (como que el usuario aprete el boton de stop)
                break
            
    def stop(self): # finaliza la ejecución de una partitura, ya sea porque llego al final de esta o porque fue interrumpida por el usuario
        self.playing.clear() # despeja el evento, lo que permite salir del loop en el que se encuentra el loop que plotea las mediciones de las variables
        rec = self.recordCheckBox.isChecked() # casilla de grabar 
        self.musician_pipe.send(["stop_playing", rec]) # le avisa al musico que deje de tocar (no es necesario si llego al final). Tambien deja de grabar informacion, aunque todavia no la guarda (espera que le llegue la instruccion con los nombres de los archivos)
        if rec: # si la casilla de grabar estaba apretada se pregunta si quiere guardar los datos
            msg = QMessageBox()
            #msg.setIcon(QMessageBox.Critical)
            msg.setText("Save the data recorded during execution?")
            msg.setInformativeText("Data will be lost if you don't save them.")
            msg.setWindowTitle("Save data?")
            dont_save_button = msg.addButton("Don't save", QMessageBox.NoRole)
            save_button = msg.addButton("Save", QMessageBox.YesRole)

            retval = msg.exec_()
            if retval == 0: # don't save
                pass # no hace nada
            elif retval == 1: # save
                self.save_recorded_data() # guarda los datos

        self.horizontalSlider.setEnabled(True) # nuevamente se puede mover el cursor con libertad
        self.recordCheckBox.setEnabled(True) # lo mismo la casilla de record
        self.stopButton.setEnabled(False) # se desactiva el stop (ya se ejecutó)
        self.clearPlotButton.setEnabled(True) # se activa el clear plot, porque hay graficos nuevos
        self.startFreeRocordingButton.setEnabled(True)
        self._set_led(self._led_playing, False)

    def save_recorded_data(self): # abre un dialogo para ponerle nombre a los archivos donde se guardará la informacion y se los envia al musico.
        fname, _ = QFileDialog.getSaveFileName(self, 'Open file', self.base_path,"CSV files (*.csv)") # nombre del archivo csv con la tabla de las mediciones tomadas
        if fname[-4:] != '.csv': # agrega el formato
            fname += '.csv'
        self.last_path = os.path.split(fname)[0] # para abrir esta direccion en la proxima ventana de archivos
        fname2, _ = QFileDialog.getSaveFileName(self, 'Open file', self.last_path,"WAV files (*.wav)") # nombre del archivo de audio wav
        if fname2[-4:] != '.wav':
            fname2 += '.wav'
        self.last_path = os.path.split(fname2)[0]
        self.musician_pipe.send(["memory.save_recorded_data", fname, fname2]) # envía estos nombres al musico, quien a su vez le pedira a la memoria que los guarde

    def closeEvent(self, a0: QtGui.QCloseEvent):
        '''
        Esta función se ejecuta al cerrar el programa, para terminar todos los procesos que están corriendo
        '''
        self.process_running.clear() # despeja el evento que controla la ejecucion del thread del comando por ps move
        self.playing.clear() # despeja el evento que controla la ejecucion de una partitura (se interrumpe su ejecucion)
        self.running.clear() # despeja el evento que comunica con los demas procesos. Mata al musico asi como todos los drivers, memorias, centro de comunicación, etc.
        sleep(0.5) # esperamos un tiempo para asegurar que todo haya cerrado correctamente antes del loop/proceso principal
        return super().closeEvent(a0)
    
    def change_settings(self): # abre un formulario con todos los ajustes posibles
        dlg = SettingsForm(parent=self)
        ## el formulario tiene 5 botones de accion: OK (cierra el formulario y efectua los cambios), cancelar (cierra el formulario y olvida los cambios en las configuraciones), apply (efectua los cambios en las configuraciones sin cerrar el formulario), save (guarda las configuraciones en el archivo settings.json para un proximo inicio) y return to default (vuelve a los ajustes originales que se encuentran en el archivo settings.json)
        dlg.setWindowTitle("Settings")
        if dlg.exec(): # entra al if si el formulario se cierra con OK
            self.refresh_settings() # se actualizan los cambios en la configuracion

    def refresh_settings(self): # actualiza los cambios en la configuracion en todos los procesos afectados
        self.musician_pipe.send(["x_driver.change_control", DATA["x_control"]])
        self.musician_pipe.send(["z_driver.change_control", DATA["z_control"]])
        self.musician_pipe.send(["alpha_driver.change_control", DATA["alpha_control"]])
        self.musician_pipe.send(["change_flute_pos", DATA["flute_position"]])
        self.musician_pipe.send(["microphone.change_frequency_detection", DATA["frequency_detection"]])

    def find_recent_files(self):
        '''
        Se usa esta función para encontrar los archivos guardados recientemente y añadirlos al menu de 'Recent Files'
        '''
        if 'recent_saves.txt' in os.listdir(self.base_path): # primero revisa si existe el archivo recent_saves.txt
            recents = [] # creamos una lista donde solo vamos a almacenar los 5 archivos mas recientes (que estan al principio de la lista)
            with open(self.base_path + '/recent_saves.txt', 'r') as file:
                for line in file.readlines():
                    if len(recents) >= 5: # si ya tenemos 5 archivos, dejamos de recorrer el archivo
                        return
                    line = line.replace("\n", "")
                    if line in recents: # si ya tenemos este archivo, no lo guardamos dos veces
                        pass
                    else: 
                        if len(line) > 0 and os.path.exists(line): # si el archivo existe lo guardamos
                            recents.append(line)
                            head, tail = os.path.split(line)
                            editAct = self.menuOpen_recent.addAction(tail) # creamos una nueva opcion en el menu de recent files
                            editAct.triggered.connect(partial(self.open, line)) # la conectamos a la funcion abrir con line (la direccion del archivo que direcciona) como argumento
        else:
            pass
    
    def new_file(self):
        '''
        Se usa esta función para comenzar una partitura nueva, si la actual no está guardada se ofrece la posibilidad de guardar los cambios antes de cerrarlos.
        '''
        if (len(self.route['history']) != 0 or len(self.route2['history']) != 0 or len(self.route3['history']) != 0 or len(self.route4['history']) != 0 or len(self.route5['history']) != 0) and not self.file_saved: # si se hizo algo al archivo se pregunta si quiere guardarlo antes de cerrarlo y abrir uno nuevo
            msg = QMessageBox()
            msg.setText("Save changes to this score before creating a new file?")
            msg.setInformativeText("Your changes will be lost if you don't save them.")
            msg.setWindowTitle("Save Score?")
            dont_save_button = msg.addButton("Don't save", QMessageBox.NoRole)
            cancel_button = msg.addButton("Cancel", QMessageBox.YesRole)
            save_button = msg.addButton("Save", QMessageBox.YesRole)

            retval = msg.exec_()
            if retval == 0: # don't save
                self.clean_score()
                self.filename = None
            elif retval == 1: # cancel vuelve sin hacer nada
                pass
            elif retval == 2: # guardar
                self.save()
                self.clean_score()
                self.filename = None
        else:
            self.clean_score()
            self.filename = None

    def save(self):
        '''
        Se usa esta función para guardar una partitura o los cambios realizados
        '''
        if len(self.route['history']) == 0 and len(self.route2['history']) == 0 and len(self.route3['history']) == 0 and len(self.route4['history']) == 0 and len(self.route5['history']) == 0 and len(self.graphicsView_8.route['history']) == 0: # si la partitura esta en blanco, no hace nada 
            return False
        if self.filename: # si tiene un filename significa que ya ha sido guardado antes, asique se sigue guardando con el mismo nombre. Si no lo tiene se ejecuta primero save_as para obtener un nombre de archivo
            data = {'route_r': self.route, 'route_theta': self.route2, 'route_offset': self.route3, 'route_flow': self.route4, 'route_fingers': self.route5, 'route_lips': self.route6, 'route_tongue': self.route7, 'dynamics': self.graphicsView_8.route, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")} # formateamos la informacion de la partitura como un diccionario (para poder guardarla como json)
            with open(self.filename, 'w') as file:
                json.dump(data, file, indent=4, sort_keys=True)
                self.changes_saved()
            if 'recent_saves.txt' in os.listdir(self.base_path):
                with open(self.base_path + '/recent_saves.txt', 'r+') as file: # guardamos la direccion del archivo en recent_saves.txt
                    content = file.read() # copiamos todo el contenido del documento
                    file.seek(0, 0) # nos ponemos al principio de la primera linea
                    file.write(self.filename + '\n' + content) # insertamos la direccion del archivo al principio del documento y volvemos a copiar su contenido
            else:
                with open(self.base_path + '/recent_saves.txt', 'w') as file: # si recent_saves.txt no existe lo creamos
                    file.write(self.filename)
        else: # es decir, no tiene nombre de archivo porque es primera vez que se guarda
            self.save_as()
        return True
        
    def save_as(self):
        '''
        Se usa esta función para guardar la partitura actual como un archivo nuevo.
        '''
        fname, _ = QFileDialog.getSaveFileName(self, 'Open file', self.base_path,"JSON files (*.json)")
        if fname != '':
            if fname[-5:] != '.json':
                fname += '.json'
            self.filename = fname
            self.save()

    def clean_score(self):
        '''
        Se usa esta función para borrar todas las acciones ingresadas en una partitura
        '''
        self.populate_graph() # reseteamos la informacion de cada trayectoria
        self.horizontalSlider.setValue(0) # movemos el cursor al inicio
        self.clear_plot() # limpiamos las curvas de las variables medidas
        self.refresh_plots_signal.emit([1,2,3,4,5,6,7]) # actualizamos los graficos
        self.changes_saved()

    def open(self, fname=None):
        '''
        Se usa esta función para abrir una partitura que había sido guardada con anterioridad. Además, si hay trabajo sin guardar se ofrece la posibilidad de guardarlo antes de abrir el nuevo.
        '''
        if (len(self.route['history']) != 0 or len(self.route2['history']) != 0 or len(self.route3['history']) != 0 or len(self.route4['history']) != 0 or len(self.route5['history']) != 0 or len(self.graphicsView_8.route['history']) != 0) and not self.file_saved: # si se tiene un archivo con cambios sin guardar, antes de abrir uno nuevo se pregunta si se quiere guardar los cambios
            msg = QMessageBox()
            #msg.setIcon(QMessageBox.Critical)
            msg.setText("Save changes to this score before opening other file?")
            msg.setInformativeText("Your changes will be lost if you don't save them.")
            msg.setWindowTitle("Save Score?")
            dont_save_button = msg.addButton("Don't save", QMessageBox.NoRole)
            cancel_button = msg.addButton("Cancel", QMessageBox.YesRole)
            save_button = msg.addButton("Save", QMessageBox.YesRole)

            retval = msg.exec_()
            if retval == 0: # don't save. No guarda y abre uno nuevo
                self.clean_score()
                self.open(fname=fname)
            elif retval == 1: # cancel no hace nada
                pass
            elif retval == 2: # save. guarda y abre uno nuevo
                self.save()
                self.clean_score()
                self.open(fname=fname)
        else:
            if not fname: # esta funcion puede llamarse al apretar el boton Open, en cuyo caso fname es None y se pregunta por un nombre de archivo por una ventana de dialogo, o de forma interna entregando una direccion de archivo en fname, en cuyo caso no se abre la ventana de dialogo
                fname, _ = QFileDialog.getOpenFileName(self, 'Open file', self.base_path,"JSON files (*.json)")
            try: # el primer try except es por si el archivo no existe.
                self.clean_score() 
                with open(fname) as json_file: # aca fallaría si no se encuentra el archivo
                    data = json.load(json_file)
                    try: # el segundo try except es por si el contenido del archivo no es el adecuado
                        self.route = data['route_r']
                        self.route2 = data['route_theta']
                        self.route3 = data['route_offset']
                        self.route4 = data['route_flow']
                        self.route5 = data['route_fingers']
                        self.route6 = data['route_lips']
                        self.route7 = data['route_tongue']
                        self.graphicsView_8.route = data['dynamics']
                        self.refresh_plots_signal.emit([1,2,3,4,5,6,7]) 
                        self.graphicsView_8.refresh()
                        self.filename = fname
                        self.changes_saved()
                    except:
                        self.clean_score()
                        msg = QMessageBox()
                        msg.setIcon(QMessageBox.Critical)
                        msg.setText("Couldn't open file.")
                        msg.setInformativeText("File format does not coincide. Try with other file.")
                        msg.setWindowTitle("File Error")
                        #msg.setDetailedText("The details are as follows:")
                        retval = msg.exec_()
            except:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Couldn't find file.")
                msg.setInformativeText("Maybe the file was moved or removed.")
                msg.setWindowTitle("File not found")
                retval = msg.exec_()

    def changes_saved(self):
        '''
        Se llama esta función cuando la partitura no tiene cambios respecto a la versión guardada.
        '''
        self.setWindowTitle("Pierre - Flutist Robot")
        self.file_saved = True
        if self.filename:
            self._status_file_label.setText(os.path.basename(self.filename))
        else:
            self._status_file_label.setText("Sin guardar")

    def changes_made(self, from_hist=False):
        '''
        Se llama esta función cuando la partitura sufre algun cambio.
        '''
        self.setWindowTitle("Pierre - Flutist Robot*")
        fname = os.path.basename(self.filename) if self.filename else "Sin guardar"
        self._status_file_label.setText(f"{fname} (modificado)")
        #self.seeMotorRefsButton.setEnabled(False)
        self.file_saved = False
        if not from_hist: # si el cambio se hace mediante undo o redo no se entra a este if.
            r = self.get_copy_of_routes(self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7, self.graphicsView_8.route) # se crea una copia del estado actual (que es distinto al anterior)
            self.undo_list.append(r) # se agrega a la lista de undo
            self.redo_list = [] # se limpia la lista de redo si hay algun cambio

    def change_score_duration(self): # abre un formulario para cambiar la duracion total de la partitura
        data = [self.route['total_t']] # abrimos la ventana con el valor actual de la duracion de la partitura
        dlg = DurationForm(parent=self, data=data)
        dlg.setWindowTitle("Change score duration")
        if dlg.exec(): # si la ventana se cierra con un OK
            new_dur = data[0]
            # actualizamos la duracion de todas las curvas
            self.route['total_t'] = new_dur
            self.route2['total_t'] = new_dur
            self.route3['total_t'] = new_dur
            self.route4['total_t'] = new_dur
            self.route5['total_t'] = new_dur
            self.route6['total_t'] = new_dur
            self.route7['total_t'] = new_dur
            self.refresh_plots_signal.emit([1,2,3,4,5,6,7]) # y actualizamos los graficos

    def add_correction(self): # abre un formulario para agregarle correcciones a toda una curva (o curvas) en el eje del tiempo o el de su valor (un offset)
        data = [0,0,0,0,0,0,0,0,0,0] # el formulario inicial no tiene desplazamiento en ningun eje
        dlg = CorrectionForm(parent=self, data=data, space=self.space_of_instruction)
        dlg.setWindowTitle("Add correction to states")
        if dlg.exec():
            r_dis = data[0] # desplazamiento en l
            theta_dis = data[1] # desplazamiento en theta
            offset_dis = data[2] # desplazamiento en offset
            flow_dis = data[3] # desplazamiento en flow
            notes_dis = data[4] # desplazamiento en notas (semitonos)
            r_t_dis = data[5] # desplazamiento en el tiempo de l
            theta_t_dis = data[6] # desplazamiento en el tiempo de theta
            offset_t_dis = data[7] # desplazamiento en el tiempo del offset
            flow_t_dis = data[8] # desplazamiento en el tiempo del flow
            notes_t_dis = data[9] # desplazamiento en el tiempo de las notas
            for p in self.route['points']: # para desplazar las curvas, desplazamos cada uno de sus puntos (en t y en y)
                p[0] += r_t_dis
                p[0] = min(self.route['total_t'], max(p[0], 0)) # saturamos en 0 y total_t
                p[1] += r_dis
            for p in self.route2['points']:
                p[0] += theta_t_dis
                p[0] = min(self.route['total_t'], max(p[0], 0))
                p[1] += theta_dis
            for p in self.route3['points']:
                p[0] += offset_t_dis
                p[0] = min(self.route['total_t'], max(p[0], 0))
                p[1] += offset_dis
            for p in self.route4['points']:
                p[0] += flow_t_dis
                p[0] = min(self.route['total_t'], max(p[0], 0))
                p[1] += flow_dis
            for p in self.route5['notes']:
                n = dict_notes_rev[p[1]]
                p[0] += notes_t_dis
                p[0] = min(self.route['total_t'], max(p[0], 0))
                p[1] = dict_notes[round((n + notes_dis/2)*2) / 2] # traducimos las notas
            self.refresh_plots_signal.emit([1,2,3,4,5]) # actualizamos los graficos

    def scale_time(self): # escala la partitura en el eje del tiempo
        data = [1]
        dlg = ScaleTimeForm(parent=self, data=data) # primero se abre un formulario para preguntar por el factor de escalamiento
        dlg.setWindowTitle("Scale time of score")
        if dlg.exec(): # si se cierra el formulario con OK
            scale = data[0]
            for route in [self.route, self.route2, self.route3, self.route4]: # hacemos lo mismo en cada una de las 4 rutas (menos la quinta que es la de las notas)
                route['total_t'] *= scale # primero escalamos el tiempo total
                for p in route['points']: # escalamos el tiempo de cada uno de sus puntos
                    p[0] *= scale
                for v in route['vibrato']: # luego cada uno de los vibratos, donde escalamos el tiempo de inicio y su duracion
                    v[0] *= scale
                    v[1] *= scale
                for f in route['filters']: # y al final los filtros, donde escalamos el tiempo de inicio y el tiempo de fin
                    f[0] *= scale
                    f[1] *= scale
            
            self.route5['total_t'] *= scale # para la ruta5 (de las notas) solo escalamos el tiempo total y las notas
            for p in self.route5['notes']: 
                p[0] *= scale
            self.route6['total_t'] *= scale # para la ruta5 (de las notas) solo escalamos el tiempo total y las notas
            for p in self.route6['points']: 
                p[0] *= scale
            self.route7['total_t'] *= scale # para la ruta5 (de las notas) solo escalamos el tiempo total y las notas
            for p in self.route7['points']: 
                p[0] *= scale
            
            self.refresh_plots_signal.emit([1,2,3,4,5,6,7]) # actualizamos los graficos

    def see_motor_refs(self): # abre una ventana donde se plotean las trayectorias de cada motor junto con sus velocidades.
        plotwin = PassivePlotWindow(self.app, self.route, self.route2, self.route3, parent=self, space=self.space_of_instruction) # Esta ventana muestra las trayectorias de cada motor de acuerdo a la partitura que se escribio y el espacio en el que se trabaja
        plotwin.setWindowTitle("Motors references")
        plotwin.show()

    def get_states_from_notes(self): # a partir de las notas escritas obtiene los estados para el resto de los parámetros
        global LOOK_UP_TABLE # usa el diccionario con las posiciones predefinidas para cada nota (que puede ser ajustado en la ventana de movimiento manual)

        data = [100, 0] # partimos haciendo las transiciones tan largas como sea posible (por defecto)
        dlg = StatesFromNotesForm(parent=self, data=data) # primero se abre un formulario para preguntar por los parametros para la creacion de la partitura a partir de las notas
        dlg.setWindowTitle("States from notes parameters")
        if dlg.exec(): # si se cierra el formulario con OK
            ## Borramos todos los puntos que se tenian en los otros cuatro parámetros (porque van a ser reescritos)
            self.route['points'] = []
            self.route2['points'] = []
            self.route3['points'] = []
            self.route4['points'] = []
            self.route6['points'] = []

            # declaramos parametros para hacer las transiciones
            transition = data[0]
            offset = data[1]
            t_past = 0
            l_past = 0
            theta_past = 0
            of_past = 0
            flow_past = 0
            for i in range(len(self.route5['notes'])): # entonces vamos nota por nota agregando los estados definidos por el diccionario
                t = self.route5['notes'][i][0] # tiempo de la nota
                n = self.route5['notes'][i][1] # nota a tocar
                try:
                    dict_to_use = self.graphicsView_8.get_dict_for_t(t)
                except:
                    print("hubo error")
                    dict_to_use = LOOK_UP_TABLE
                if i == 0: # si es la primera nota sabemos que vamos a partir en su estado, no es necesaria una transicion
                    tf = max(0, min(t+offset, self.route['total_t'])) # tiempo final de la transicion
                    self.add_item(0, 0, [tf, dict_to_use[n]['l']], from_func=True)
                    self.add_item(1, 0, [tf, dict_to_use[n]['theta']], from_func=True)
                    self.add_item(2, 0, [tf, dict_to_use[n]['offset']], from_func=True)
                    self.add_item(3, 0, [tf, dict_to_use[n]['flow']], from_func=True)
                    self.add_item(5, 0, [tf, dict_to_use[n]['lips']], from_func=True)
                else:
                    if t - transition < t_past: # si la nota anterior esta más proxima al tiempo que se pidio hacer la transicion, solo se agregan los puntos finales de la transicion (los de partida estaran dados por la nota anterior)
                        tf = max(0, min(t+offset, self.route['total_t'])) # tiempo final de la transicion
                        self.add_item(0, 0, [tf, dict_to_use[n]['l']], from_func=True)
                        self.add_item(1, 0, [tf, dict_to_use[n]['theta']], from_func=True)
                        self.add_item(2, 0, [tf, dict_to_use[n]['offset']], from_func=True)
                        self.add_item(3, 0, [tf, dict_to_use[n]['flow']], from_func=True)
                        self.add_item(5, 0, [tf, dict_to_use[n]['lips']], from_func=True)
                    else: # si la nota anterior está mas lejos temporalmente, se hace la transicion como se solicitó. Iniciando transition-offset segundos antes
                        ti = max(0, min(t+offset-transition, self.route['total_t'])) # tiempo inicial de la transicion
                        tf = max(0, min(t+offset, self.route['total_t'])) # tiempo final de la transicion
                        self.add_item(0, 0, [ti, l_past], from_func=True)
                        self.add_item(1, 0, [ti, theta_past], from_func=True)
                        self.add_item(2, 0, [ti, of_past], from_func=True)
                        self.add_item(3, 0, [ti, flow_past], from_func=True)
                        self.add_item(0, 0, [tf, dict_to_use[n]['l']], from_func=True)
                        self.add_item(1, 0, [tf, dict_to_use[n]['theta']], from_func=True)
                        self.add_item(2, 0, [tf, dict_to_use[n]['offset']], from_func=True)
                        self.add_item(3, 0, [tf, dict_to_use[n]['flow']], from_func=True)
                        self.add_item(5, 0, [tf, dict_to_use[n]['lips']], from_func=True)
                t_past = t
                l_past = dict_to_use[n]['l']
                theta_past = dict_to_use[n]['theta']
                of_past = dict_to_use[n]['offset']
                flow_past = dict_to_use[n]['flow']
            self.changes_made() # informamos que hay cambios

    def move_coursor(self, value): # mueve el cursor. Esta funcion se llama cuando se mueve el slider en la ventana principal
        t = self.route['total_t'] * value / 99
        # movemos todas las reglas juntas
        self.rule.setPos(t)
        self.rule2.setPos(t)
        self.rule3.setPos(t)
        self.rule4.setPos(t)
        self.rule5.setPos(t)
        self.rule6.setPos(t)
        self.rule7.setPos(t)
        self.graphicsView_8.rule.setPos(t)
        self.playButton.setEnabled(False) # al cambiar el cursor hay que volver a apretar Move to Cursor para poder ejecutar una partitura
        self._status_time_label.setText(f"t: {t:.1f}s")

    ####
    ####
    #### Aqui vienen varias funciones muy parecidas, que se repiten para cada eje. 
    #### Solo se explicarán como funcionan en el primer eje
    ####
    #### Estas son:
    ####      - onFilterClicked[n]: 
    ####            se llama cuando se hace click sobre uno de los puntos que indican un filtro (los puntos azules) en el grafico n. Abre un menu de opciones para editar, mover o borrar un filtro
    ####      - onVibratoClicked[n]: 
    ####            se llama cuando se hace click sobre uno de los puntos que indican un vibrato (los puntos rojos) en el grafico n. Abre un menu de opciones para editar, mover o borrar un vibrato
    ####      - onCurveClicked[n]: 
    ####            se llama cuando se hace click sobre la trayectoria en el grafico n. Abre un menu de opciones para agregar un punto, un vibrato, un filtro o abrir una ventana de dialogo que muestra las propiedades de la curva en una tabla
    ####      - onPointsClicked[n]: 
    ####            se llama cuando se hace click sobre uno de los puntos que indican un punto de la trayectoria (los puntos verdes) en el grafico n. Abre un menu de opciones para editar, mover o borrar un punto de la trayectoria
    ####      - mouse_moved[n]: 
    ####            se llama cuando el mouse pasa por encima del graphicView n (sin necesariamente estar por encima de un punto o en la curva). Se usa cuando se esta moviendo un elemento de la curva para mostrar como cambia dinamicamente.
    ####      - mouse_clicked[n]: 
    ####            se llama cuando se hace click en alguna parte del graphicView n donde no hay ningun punto ni curva. Si se hace un doble click se agrega un punto de la trayectoria en el lugar en el que se clickeo. Si se esta moviendo un elemento de la curva por la vista, se usa esta funcion para fijarlo en el lugar donde se hace click
    ####
    #### En el caso del grafico 5 hay unas pocas diferencias
    ####

    def onFilterClicked(self, obj, point, event):
        """
        se llama cuando se hace click sobre uno de los puntos que indican un filtro (los puntos azules) en el grafico n. Abre un menu de opciones para editar, mover o borrar un filtro
        """
        if not self.moving_point[0] and not self.moving_vibrato[0] and not self.moving_filter[0]: # solo se entra si no se esta moviendo ni un punto ni un vibrato ni un filtro de este grafico
            ## leemos el valor de x e y donde se hizo click (para identificar que punto se clickeo)
            x = event.pos()[0]
            y = event.pos()[1]
            # usamos estas ubicaciones de la pantalla en cambio para desplegar el menu donde esta el mouse
            x_screen = int(event.screenPos()[0]) 
            y_screen = int(event.screenPos()[1])
            action = self.filterMenu.exec_(QtCore.QPoint(x_screen,y_screen)) # este tiene 3 acciones: moveFilter, editFilter y deleteFilter
            if action == self.moveFilter: # si se quiere mover un filtro, es posible moverlo a lo largo de la curva, es decir, cambiamos el tiempo de inicio (manteniendo su duracion) al tiempo que se desplaza el mouse por el grafico
                self.changes_made()
                self.moving_filter[0] = True # activamos este bool para indicar que queremos que se mueva un filtro
                self.moving_filter_index[0] = self.find_closest_filter(self.route['filters'], x) # y en esta lista indicamos cual es el indice del filtro que queremos mover. Este indice lo encontramos con la funcion find_closest_filter, usando el tiempo de inicio como argumento de busqueda
            elif action == self.editFilter: # si se quiere editar un filtro, esta opcion abre un formulario como el que se usa cuando se quiere crear un filtro, prellenado con las propiedades del filtro que se clickeo.
                index = self.find_closest_filter(self.route['filters'], x) # primero buscamos el índice del filtro
                data = [0, 0] + [0 for i in range(14)] # la data es: time_i, time_f, filter_choice, window_choice, window_n, cutoff, Ap, As, fp, fs, chebN, chebAp, chebAs, chebfp, chebfs, chebrp
                new_data = self.route['filters'][index] # i_init, i_end, filter, params
                data[0] = new_data[0]
                data[1] = new_data[1]
                data[2] = filter_choices.index(new_data[2]) # nos da el indice del filtro que se seleccionó
                if data[2] == 0: # filtro firwin
                    data[3] = filter_windows.index(new_data[3][0])
                    data[4] = new_data[3][1]
                    data[5] = new_data[3][2]
                elif data[2] == 3: # filtro chebyshev
                    data[10] = new_data[3][0]
                    data[11] = new_data[3][1]
                    data[12] = new_data[3][2]
                    data[13] = new_data[3][3]
                    data[14] = new_data[3][4]
                    data[15] = new_data[3][5]
                else: # filtros remez, butter o elliptic
                    data[6] = new_data[3][0]
                    data[7] = new_data[3][1]
                    data[8] = new_data[3][2]
                    data[9] = new_data[3][3]
                ## en este punto ya se llenó la lista data con la informacion del filtro sobre el que se hizo click
                while True:
                    # entonces abrimos el formulario
                    dlg = FilterForm(parent=self, data=data)
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec(): # si se sale del formulario con OK se actualiza el filtro
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0: # si se seleccionó un filtro firwin
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3: # si se seleccionó un filtro chebyshev
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else: # si se seleccionó un filtro remez, butter o elliptic
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice] # traducimos del indice al string
                        if self.check_filter(time_i, time_f, filter_choice, params):
                            self.edit_item(0, 2, index, [time_i, time_f, filter_choice, params]) # actualizamos el filtro
                            break
                        # si hay algun error (por ejemplo porque el filtro queda inestable) se vuelve a abrir el formulario y se pide ingresar nuevos valores
                    else: # si se cierra el formulario con cancel se sale y no se hacen cambios
                        break
            elif action == self.deleteFilter: # si se quiere eliminar un filtro
                index = self.find_closest_filter(self.route['filters'], x) # primero buscamos el indice del filtro a partir del valor de x sobre el punto donde se hizo click
                p = self.route['filters'][index] # encontramos el filtro en la ruta
                self.route['filters'].remove(p) # lo eliminamos de la ruta
                self.route['history'].append(['delete_filter', p]) # agregamos al historial que se eliminó este punto
                self.refresh_plots_signal.emit([1]) # se actualiza el gráfico
                self.changes_made() # se avisa que hubo cambios
        else: # si se estaba moviendo un punto, un vibrato o un filtro, y se hace click, entonces se deja de mover el elemento que se estaba moviendo y queda en su ultima posicion
            self.moving_point[0] = False
            self.moving_vibrato[0] = False
            self.moving_filter[0] = False
            self.segundo_click[0] = False

    def onVibratoClicked(self, obj, point, event):
        """
        se llama cuando se hace click sobre uno de los puntos que indican un vibrato (los puntos rojos) en el grafico n. Abre un menu de opciones para editar, mover o borrar un vibrato
        """
        if not self.moving_point[0] and not self.moving_vibrato[0] and not self.moving_filter[0]: # solo se entra si no se esta moviendo ni un punto ni un vibrato ni un filtro de este grafico
            ## leemos el valor de x e y donde se hizo click (para identificar que punto se clickeo)
            x = event.pos()[0]
            y = event.pos()[1]
            # usamos estas ubicaciones de la pantalla en cambio para desplegar el menu donde esta el mouse
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.vibratoMenu.exec_(QtCore.QPoint(x_screen,y_screen)) # este tiene 3 acciones: moveVibrato, editVibrato y deleteVibrato

            if action == self.moveVibrato: # si se quiere mover un vibrato, es posible moverlo a lo largo de la curva, es decir, cambiamos el tiempo de inicio (manteniendo su duracion) al tiempo que se desplaza el mouse por el grafico
                self.changes_made()
                self.moving_vibrato[0] = True # activamos este bool para indicar que queremos que se mueva un vibrato
                self.moving_vibrato_index[0] = self.find_closest_vibrato(self.route['vibrato'], x) # y en esta lista indicamos cual es el indice del vibrato que queremos mover. Este indice lo encontramos con la funcion find_closest_vibrato, usando el tiempo de inicio como argumento de busqueda
            elif action == self.editVibrato: # si se quiere editar un vibrato, esta opcion abre un formulario como el que se usa cuando se quiere crear un vibrato, prellenado con las propiedades del vibrato que se clickeo.
                index = self.find_closest_vibrato(self.route['vibrato'], x) # encontramos el indice del vibrato usando como argumento de busqueda el tiempo de inicio (que es igual a x en el grafico)
                data = self.route['vibrato'][index] # usamos como data para el formulario la informacion del vibrato
                data[4] = windows_vibrato.index(data[4]) # la ventana que se guarda como string se reemplaza por su indice en esta lista
                dlg = VibratoForm(parent=self, data=data, max_t=self.route['total_t']) # y ejecutamos el formulario
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec(): # si se cierra con un OK
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.edit_item(0, 1, index, [time_i, duration, amp, freq, window_v]) # se efectuan los cambios
            elif action == self.deleteVibrato: # si se quiere eliminar un vibrato
                index = self.find_closest_vibrato(self.route['vibrato'], x) # buscamos el indice del vibrato
                p = self.route['vibrato'][index] # tomamos el objeto
                self.route['vibrato'].remove(p) # lo eliminamos de la lista de vibratos
                self.route['history'].append(['delete_vibrato', p]) # agregamos al historial que se eliminto
                self.refresh_plots_signal.emit([1])
                self.changes_made()
        else: # si se estaba moviendo un punto, un vibrato o un filtro, y se hace click, entonces se deja de mover el elemento que se estaba moviendo y queda en su ultima posicion
            self.moving_point[0] = False
            self.moving_vibrato[0] = False
            self.moving_filter[0] = False
            self.segundo_click[0] = False
        
    def onCurveClicked(self, obj, event):
        if not self.moving_point[0] and not self.moving_vibrato[0] and not self.moving_filter[0]: # solo se entra si no se esta moviendo ni un punto ni un vibrato ni un filtro de este grafico
            ## leemos el valor de x e y donde se hizo click
            x = event.pos()[0]
            y = event.pos()[1]
            # usamos estas ubicaciones de la pantalla en cambio para desplegar el menu donde esta el mouse
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.graphMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.addPoint: # si queremos agregar un punto
                # en este caso no se despliega un formulario, simplemente se agrega un punto y se empieza a mover
                self.route['points'].append([x, y]) # agregamos el punto donde se hizo click
                self.route['points'].sort(key=lambda x: x[0])
                self.route['history'].append(['add_point', [x, y]])
                self.refresh_plots_signal.emit([1])
                self.changes_made()
                self.moving_point[0] = True # activamos este bool para indicar que queremos que se mueva el punto
                self.moving_point_index[0] = self.find_closest_point(self.route['points'], x, y) # y en esta lista indicamos cual es el indice del punto que queremos mover. Este indice lo encontramos con la funcion find_closest_point, usando el tiempo de inicio como argumento de busqueda
            elif action == self.addVibrato: # si queremos agregar un vibrato en el lugar donde se hizo click
                data=[x, 0, 0, 0, 0]
                dlg = VibratoForm(parent=self, data=data, max_t=self.route['total_t']) # creamos un formulario para ingresar los parametros del vibrato
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec():
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.route['vibrato'].append([time_i, duration, amp, freq, window_v]) # agregamos el vibrato
                    self.route['history'].append(['vibrato', [time_i, duration, amp, freq, window_v]])
                    self.refresh_plots_signal.emit([1]) # actualizamos el grafico
                    self.changes_made()
            elif action == self.addFilter: # si queremos agregar un filtro en el lugar donde se hizo click
                data=[x, x] + [0 for i in range(14)]
                while True: # en el caso del filtro lo hacemos dentro de un while por si los parametros ingresados generan error (filtro inestable), para volver a desplegar el formulario
                    dlg = FilterForm(parent=self, data=data) # creamos el formulario
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec():
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0:
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3:
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else:
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice]
                        if self.check_filter(time_i, time_f, filter_choice, params): # revisamos que los parametros ingresados sean validos
                            self.route['filters'].append([time_i, time_f, filter_choice, params])
                            self.route['history'].append(['filter', [time_i, time_f, filter_choice, params]])
                            self.refresh_plots_signal.emit([1])
                            self.changes_made()
                            break
                        # si el filtro ingresado no es valido se vuelve a desplegar el formulario
                    else: # si el usuario apreta Cancelar, se sale del loop y no se crea el filtro
                        break
                    
            elif action == self.openTable: # la ultima opcion permite abrir la trayectoria como una tabla con todos los elementos que la componen. En esta tabla se pueden agregar puntos, vibratos y filtros asi como tambien es posible editar o eliminar los que ya existen
                data = [0, 0, self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7]
                dlg = FuncTableForm(parent=self, data=data) # creamos el formulario con todas las tablas
                dlg.setWindowTitle("Function as table")
                if dlg.exec():
                    pass
        else: # si se estaba moviendo un punto, un vibrato o un filtro, y se hace click, entonces se deja de mover el elemento que se estaba moviendo y queda en su ultima posicion
            self.moving_point[0] = False
            self.moving_vibrato[0] = False
            self.moving_filter[0] = False
            self.segundo_click[0] = False

    def onPointsClicked(self, obj, point, event):
        """
        se llama cuando se hace click sobre uno de los puntos que indican un punto de la trayectoria (los puntos verdes) en el grafico n. Abre un menu de opciones para editar, mover o borrar un punto de la trayectoria
        """
        if not self.moving_point[0] and not self.moving_vibrato[0] and not self.moving_filter[0]:  # solo se entra si no se esta moviendo ni un punto ni un vibrato ni un filtro de este grafico
            ## leemos el valor de x e y donde se hizo click (para identificar que punto se clickeo)
            x = event.pos()[0]
            y = event.pos()[1]
            # usamos estas ubicaciones de la pantalla en cambio para desplegar el menu donde esta el mouse
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.pointMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.movePoint: # si se quiere mover un punto, es posible moverlo a lo largo del plano (tanto en el eje del tiempo como en y) al tiempo que se desplaza el mouse por el grafico
                self.changes_made()
                self.moving_point[0] = True # activamos este bool para indicar que queremos que se mueva un punto
                self.moving_point_index[0] = self.find_closest_point(self.route['points'], x, y) # y en esta lista indicamos cual es el indice del punto que queremos mover. Este indice lo encontramos con la funcion find_closest_point, usando el tiempo de inicio como argumento de busqueda
            elif action == self.editPoint: # si se quiere editar un punto, esta opcion abre un formulario prellenado con las propiedades del punto que se clickeo.
                index = self.find_closest_point(self.route['points'], x, y) # buscamos el indice del punto a editar
                data = self.route['points'][index]
                dlg = PointForm(parent=self, data=data, max_t=self.route['total_t']) # creamos un formulario con su informacion
                dlg.setWindowTitle("Add Point")
                if dlg.exec(): # lo ejecutamos, y si el usuario cierra el formulario con OK, editamos la informacion
                    self.edit_item(0, 0, index, data)
            elif action == self.deletePoint: # si queremos eliminar un punto
                p = self.route['points'][self.find_closest_point(self.route['points'], x, y)] # encontramos el punto a eliminar
                self.route['points'].remove(p) # lo eliminamos
                self.route['history'].append(['delete_point', p])
                self.changes_made()
                try:
                    self.refresh_plots_signal.emit([1]) # actualizamos el grafico
                except:
                    pass
        else: # si se estaba moviendo un punto, un vibrato o un filtro, y se hace click, entonces se deja de mover el elemento que se estaba moviendo y queda en su ultima posicion
            self.moving_point[0] = False
            self.moving_vibrato[0] = False
            self.moving_filter[0] = False
            self.segundo_click[0] = False

    def mouse_moved(self, event):
        if self.moving_point[0]: # si se esta moviendo un punto de este grafico
            self.segundo_click[0] = True # se activa este bool porque al hacer el click sobre la opcion 'mover' se llama la funcion mouse_clicked, la que puede pensar que se quizo dejar de mover el elemento. Por esto se necesita una especie de contador, que lo suelte al segundo click realmente
            vb = self.graphicsView.plotItem.vb # lo necesitamos para interpretar la posicion en la que se encuentra el mouse
            mouse_point = vb.mapSceneToView(event) # posicion del mouse
            x = round(mouse_point.x(), 2)
            y = max(0,round(mouse_point.y(), 2)) # limitamos en 0 por abajo
            min_x, max_x = self.find_moving_point_limits(self.route['points'], self.moving_point_index[0], self.route['total_t']) # limitamos el movimiento del punto, no es posible moverlo antes o despues de los puntos que le siguen
            if x > min_x and x < max_x: # si esta dentro del rango posible actualizamos la posicion del punto
                self.route['points'][self.moving_point_index[0]] = [x, y]
                self.refresh_plots_signal.emit([1])
        if self.moving_vibrato[0]: # si se esta moviendo un vibrato de este grafico
            self.segundo_click[0] = True # se activa este bool porque al hacer el click sobre la opcion 'mover' se llama la funcion mouse_clicked, la que puede pensar que se quizo dejar de mover el elemento. Por esto se necesita una especie de contador, que lo suelte al segundo click realmente
            vb = self.graphicsView.plotItem.vb # lo necesitamos para interpretar la posicion en la que se encuentra el mouse
            mouse_point = vb.mapSceneToView(event) # posicion del mouse
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_vibrato_limits(self.route['vibrato'], self.moving_vibrato_index[0], self.route['total_t']) # limitamos el movimiento del vibrato, no es posible moverlo fuera del grafico
            if x >= min_x and x <= max_x: # si esta dentro del rango posible actualizamos la posicion del vibrato
                self.route['vibrato'][self.moving_vibrato_index[0]][0] = x
                self.refresh_plots_signal.emit([1])
        if self.moving_filter[0]: # si se esta moviendo un filtro de este grafico
            self.segundo_click[0] = True # se activa este bool porque al hacer el click sobre la opcion 'mover' se llama la funcion mouse_clicked, la que puede pensar que se quizo dejar de mover el elemento. Por esto se necesita una especie de contador, que lo suelte al segundo click realmente
            vb = self.graphicsView.plotItem.vb # lo necesitamos para interpretar la posicion en la que se encuentra el mouse
            mouse_point = vb.mapSceneToView(event) # posicion del mouse
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_filter_limits(self.route['filters'], self.moving_filter_index[0], self.route['total_t']) # limitamos el movimiento del filtro, no es posible moverlo fuera del grafico
            if x >= min_x and x <= max_x: # si esta dentro del rango posible actualizamos la posicion del filtro
                diff = self.route['filters'][self.moving_filter_index[0]][1] - self.route['filters'][self.moving_filter_index[0]][0] # para mantener la duracion del filtro
                self.route['filters'][self.moving_filter_index[0]][0] = x
                self.route['filters'][self.moving_filter_index[0]][1] = x + diff
                self.refresh_plots_signal.emit([1])
        # si no se esta moviendo ninguno de los tres, esta funcion no hace nada

    def mouse_clicked(self, event):
        """
        se llama cuando se hace click en alguna parte del graphicView n donde no hay ningun punto ni curva. Si se hace un doble click se agrega un punto de la trayectoria en el lugar en el que se clickeo. Si se esta moviendo un elemento de la curva por la vista, se usa esta funcion para fijarlo en el lugar donde se hace click
        """
        if (self.moving_point[0] or self.moving_vibrato[0] or self.moving_filter[0]) and self.segundo_click[0]: # si estabamos moviendo algun elemento del grafico y ya se cumplio la condicion segundo_click (que dice que el click que se recibe es para dejar el elemento), se deja de mover y el elemento queda en la ultima posicion valida a la que se movio
            self.moving_point[0] = False
            self.moving_vibrato[0] = False
            self.moving_filter[0] = False
            self.segundo_click[0] = False
        else:
            if event.double(): # si el usuario hace un doble click en una parte cualquiera del plano donde no haya otro elemento, se crea en tal lugar un punto de la trayectoria
                vb = self.graphicsView.plotItem.vb # lo necesitamos para conocer la posicion del mouse
                mouse_point = vb.mapSceneToView(event.scenePos()) # posicion del mouse
                x = round(mouse_point.x(), 2)
                y = max(0,round(mouse_point.y(), 2)) # limitamos y en 0 por abajo
                self.route['points'].append([x, y]) # agregamos el punto
                self.route['points'].sort(key=lambda x: x[0])
                self.route['history'].append(['add_point', [x, y]])
                self.refresh_plots_signal.emit([1]) # actualizamos el grafico
                self.changes_made()

    def onFilterClicked2(self, obj, point, event): # ver comentarios en onFilterClicked
        if not self.moving_point[1] and not self.moving_vibrato[1] and not self.moving_filter[1]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.filterMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.moveFilter:
                self.changes_made()
                self.moving_filter[1] = True
                self.moving_filter_index[1] = self.find_closest_filter(self.route2['filters'], x)
            elif action == self.editFilter:
                index = self.find_closest_filter(self.route2['filters'], x)
                data = [0, 0] + [0 for i in range(14)] # time_i, time_f, filter_choice, window_choice, window_n, cutoff, Ap, As, fp, fs, chebN, chebAp, chebAs, chebfp, chebfs, chebrp
                new_data = self.route2['filters'][index] # i_init, i_end, filter, params
                data[0] = new_data[0]
                data[1] = new_data[1]
                data[2] = filter_choices.index(new_data[2])
                if data[2] == 0:
                    data[3] = filter_windows.index(new_data[3][0])
                    data[4] = new_data[3][1]
                    data[5] = new_data[3][2]
                elif data[2] == 3:
                    data[10] = new_data[3][0]
                    data[11] = new_data[3][1]
                    data[12] = new_data[3][2]
                    data[13] = new_data[3][3]
                    data[14] = new_data[3][4]
                    data[15] = new_data[3][5]
                else:
                    data[6] = new_data[3][0]
                    data[7] = new_data[3][1]
                    data[8] = new_data[3][2]
                    data[9] = new_data[3][3]
                while True:
                    dlg = FilterForm(parent=self, data=data)
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec():
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0:
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3:
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else:
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice]
                        if self.check_filter(time_i, time_f, filter_choice, params):
                            self.edit_item(1, 2, index, [time_i, time_f, filter_choice, params])
                            break
                    else:
                        break
            elif action == self.deleteFilter:
                index = self.find_closest_filter(self.route2['filters'], x)
                p = self.route2['filters'][index]
                self.route2['filters'].remove(p)
                self.route2['history'].append(['delete_filter', p])
                self.refresh_plots_signal.emit([2])
                self.changes_made()
        else:
            self.moving_point[1] = False
            self.moving_vibrato[1] = False
            self.moving_filter[1] = False
            self.segundo_click[1] = False

    def onVibratoClicked2(self, obj, point, event): # ver comentarios en onVibratoClicked
        if not self.moving_point[1] and not self.moving_vibrato[1] and not self.moving_filter[1]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.vibratoMenu.exec_(QtCore.QPoint(x_screen,y_screen))

            if action == self.moveVibrato:
                self.changes_made()
                self.moving_vibrato[1] = True
                self.moving_vibrato_index[1] = self.find_closest_vibrato(self.route2['vibrato'], x)
            elif action == self.editVibrato:
                index = self.find_closest_vibrato(self.route2['vibrato'], x)
                data = self.route2['vibrato'][index]
                data[4] = windows_vibrato.index(data[4])
                dlg = VibratoForm(parent=self, data=data, max_t=self.route2['total_t'])
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec():
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.edit_item(1, 1, index, [time_i, duration, amp, freq, window_v])
            elif action == self.deleteVibrato:
                index = self.find_closest_vibrato(self.route2['vibrato'], x)
                p = self.route2['vibrato'][index]
                self.route2['vibrato'].remove(p)
                self.route2['history'].append(['delete_vibrato', p])
                self.refresh_plots_signal.emit([2])
                self.changes_made()
        else:
            self.moving_point[1] = False
            self.moving_vibrato[1] = False
            self.moving_filter[1] = False
            self.segundo_click[1] = False
        
    def onCurveClicked2(self, obj, event): # ver comentarios en onCurveClicked
        if not self.moving_point[1] and not self.moving_vibrato[1] and not self.moving_filter[1]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.graphMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.addPoint:
                self.route2['points'].append([x, y])
                self.route2['points'].sort(key=lambda x: x[0])
                self.route2['history'].append(['add_point', [x, y]])
                self.refresh_plots_signal.emit([2])
                self.changes_made()
                self.moving_point[1] = True
                self.moving_point_index[1] = self.find_closest_point(self.route2['points'], x, y)
            elif action == self.addVibrato:
                data=[x, 0, 0, 0, 0]
                dlg = VibratoForm(parent=self, data=data, max_t=self.route2['total_t'])
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec():
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.route2['vibrato'].append([time_i, duration, amp, freq, window_v])
                    self.route2['history'].append(['vibrato', [time_i, duration, amp, freq, window_v]])
                    self.refresh_plots_signal.emit([2])
                    self.changes_made()
            elif action == self.addFilter:
                data=[x, x] + [0 for i in range(14)]
                while True:
                    dlg = FilterForm(parent=self, data=data)
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec():
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0:
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3:
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else:
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice]
                        if self.check_filter(time_i, time_f, filter_choice, params):
                            self.route2['filters'].append([time_i, time_f, filter_choice, params])
                            self.route2['history'].append(['filter', [time_i, time_f, filter_choice, params]])
                            self.refresh_plots_signal.emit([2])
                            self.changes_made()
                            break
                    else:
                        break
                    
            elif action == self.openTable:
                data = [1, 0, self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7]
                dlg = FuncTableForm(parent=self, data=data)
                dlg.setWindowTitle("Function as table")
                if dlg.exec():
                    pass
        else:
            self.moving_point[1] = False
            self.moving_vibrato[1] = False
            self.moving_filter[1] = False
            self.segundo_click[1] = False

    def onPointsClicked2(self, obj, point, event): # ver comentarios en onPointsClicked
        if not self.moving_point[1] and not self.moving_vibrato[1] and not self.moving_filter[1]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.pointMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.movePoint:
                self.changes_made()
                self.moving_point[1] = True
                self.moving_point_index[1] = self.find_closest_point(self.route2['points'], x, y)
            elif action == self.editPoint:
                index = self.find_closest_point(self.route2['points'], x, y)
                data = self.route2['points'][index]
                dlg = PointForm(parent=self, data=data, max_t=self.route2['total_t'], min_v=20, max_v=70)
                dlg.setWindowTitle("Add Point")
                if dlg.exec():
                    self.edit_item(1, 0, index, data)
            elif action == self.deletePoint:
                p = self.route2['points'][self.find_closest_point(self.route2['points'], x, y)]
                self.route2['points'].remove(p)
                self.route2['history'].append(['delete_point', p])
                try:
                    self.refresh_plots_signal.emit([2])
                    self.changes_made()
                except:
                    pass
        else:
            self.moving_point[1] = False
            self.moving_vibrato[1] = False
            self.moving_filter[1] = False
            self.segundo_click[1] = False

    def mouse_moved2(self, event): # ver comentarios en mouse_moved
        if self.moving_point[1]:
            self.segundo_click[1] = True
            vb = self.graphicsView_2.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = max(20,min(70,round(mouse_point.y(), 2)))
            min_x, max_x = self.find_moving_point_limits(self.route2['points'], self.moving_point_index[1], self.route2['total_t'])
            if x > min_x and x < max_x:
                self.route2['points'][self.moving_point_index[1]] = [x, y]
                self.refresh_plots_signal.emit([2])
        if self.moving_vibrato[1]:
            self.segundo_click[1] = True
            vb = self.graphicsView_2.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = max(20,min(70,round(mouse_point.y(), 2)))
            min_x, max_x = self.find_moving_vibrato_limits(self.route2['vibrato'], self.moving_vibrato_index[1], self.route2['total_t'])
            if x >= min_x and x <= max_x:
                self.route2['vibrato'][self.moving_vibrato_index[1]][0] = x
                self.refresh_plots_signal.emit([2])
        if self.moving_filter[1]:
            self.segundo_click[1] = True
            vb = self.graphicsView_2.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_filter_limits(self.route2['filters'], self.moving_filter_index[1], self.route2['total_t'])
            if x >= min_x and x <= max_x:
                diff = self.route2['filters'][self.moving_filter_index[1]][1] - self.route2['filters'][self.moving_filter_index[1]][0]
                self.route2['filters'][self.moving_filter_index[1]][0] = x
                self.route2['filters'][self.moving_filter_index[1]][1] = x + diff
                self.refresh_plots_signal.emit([2])

    def mouse_clicked2(self, event): # ver comentarios en mouse_clicked
        if (self.moving_point[1] or self.moving_vibrato[1] or self.moving_filter[1]) and self.segundo_click[1]:
            self.moving_point[1] = False
            self.moving_vibrato[1] = False
            self.moving_filter[1] = False
            self.segundo_click[1] = False
        else:
            if event.double():
                vb = self.graphicsView_2.plotItem.vb
                mouse_point = vb.mapSceneToView(event.scenePos())
                x = round(mouse_point.x(), 2)
                y = round(mouse_point.y(), 2)
                self.route2['points'].append([x, y])
                self.route2['points'].sort(key=lambda x: x[0])
                self.route2['history'].append(['add_point', [x, y]])
                self.refresh_plots_signal.emit([2])
                self.changes_made()

    def onFilterClicked3(self, obj, point, event): # ver comentarios en onFilterClicked
        if not self.moving_point[2] and not self.moving_vibrato[2] and not self.moving_filter[2]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.filterMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.moveFilter:
                self.changes_made()
                self.moving_filter[2] = True
                self.moving_filter_index[2] = self.find_closest_filter(self.route3['filters'], x)
            elif action == self.editFilter:
                index = self.find_closest_filter(self.route3['filters'], x)
                data = [0, 0] + [0 for i in range(14)] # time_i, time_f, filter_choice, window_choice, window_n, cutoff, Ap, As, fp, fs, chebN, chebAp, chebAs, chebfp, chebfs, chebrp
                new_data = self.route3['filters'][index] # i_init, i_end, filter, params
                data[0] = new_data[0]
                data[1] = new_data[1]
                data[2] = filter_choices.index(new_data[2])
                if data[2] == 0:
                    data[3] = filter_windows.index(new_data[3][0])
                    data[4] = new_data[3][1]
                    data[5] = new_data[3][2]
                elif data[2] == 3:
                    data[10] = new_data[3][0]
                    data[11] = new_data[3][1]
                    data[12] = new_data[3][2]
                    data[13] = new_data[3][3]
                    data[14] = new_data[3][4]
                    data[15] = new_data[3][5]
                else:
                    data[6] = new_data[3][0]
                    data[7] = new_data[3][1]
                    data[8] = new_data[3][2]
                    data[9] = new_data[3][3]
                while True:
                    dlg = FilterForm(parent=self, data=data)
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec():
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0:
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3:
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else:
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice]
                        if self.check_filter(time_i, time_f, filter_choice, params):
                            self.edit_item(2, 2, index, [time_i, time_f, filter_choice, params])
                            break
                    else:
                        break
            elif action == self.deleteFilter:
                index = self.find_closest_filter(self.route3['filters'], x)
                p = self.route3['filters'][index]
                self.route3['filters'].remove(p)
                self.route3['history'].append(['delete_filter', p])
                self.changes_made()
                self.refresh_plots_signal.emit([3])
        else:
            self.moving_point[2] = False
            self.moving_vibrato[2] = False
            self.moving_filter[2] = False
            self.segundo_click[2] = False

    def onVibratoClicked3(self, obj, point, event): # ver comentarios en onVibratoClicked
        if not self.moving_point[2] and not self.moving_vibrato[2] and not self.moving_filter[2]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.vibratoMenu.exec_(QtCore.QPoint(x_screen,y_screen))

            if action == self.moveVibrato:
                self.changes_made()
                self.moving_vibrato[2] = True
                self.moving_vibrato_index[2] = self.find_closest_vibrato(self.route3['vibrato'], x)
            elif action == self.editVibrato:
                index = self.find_closest_vibrato(self.route3['vibrato'], x)
                data = self.route3['vibrato'][index]
                data[4] = windows_vibrato.index(data[4])
                dlg = VibratoForm(parent=self, data=data, max_t=self.route3['total_t'])
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec():
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.edit_item(2, 1, index, [time_i, duration, amp, freq, window_v])
            elif action == self.deleteVibrato:
                index = self.find_closest_vibrato(self.route3['vibrato'], x)
                p = self.route3['vibrato'][index]
                self.route3['vibrato'].remove(p)
                self.route3['history'].append(['delete_vibrato', p])
                self.changes_made()
                self.refresh_plots_signal.emit([3])
        else:
            self.moving_point[2] = False
            self.moving_vibrato[2] = False
            self.moving_filter[2] = False
            self.segundo_click[2] = False
        
    def onCurveClicked3(self, obj, event): # ver comentarios en onCurveClicked
        if not self.moving_point[2] and not self.moving_vibrato[2] and not self.moving_filter[2]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.graphMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.addPoint:
                self.route3['points'].append([x, y])
                self.route3['points'].sort(key=lambda x: x[0])
                self.route3['history'].append(['add_point', [x, y]])
                self.refresh_plots_signal.emit([3])
                self.changes_made()
                self.moving_point[2] = True
                self.moving_point_index[2] = self.find_closest_point(self.route3['points'], x, y)
            elif action == self.addVibrato:
                data=[x, 0, 0, 0, 0]
                dlg = VibratoForm(parent=self, data=data, max_t=self.route3['total_t'])
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec():
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.route3['vibrato'].append([time_i, duration, amp, freq, window_v])
                    self.route3['history'].append(['vibrato', [time_i, duration, amp, freq, window_v]])
                    self.changes_made()
                    self.refresh_plots_signal.emit([3])
            elif action == self.addFilter:
                data=[x, x] + [0 for i in range(14)]
                while True:
                    dlg = FilterForm(parent=self, data=data)
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec():
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0:
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3:
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else:
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice]
                        if self.check_filter(time_i, time_f, filter_choice, params):
                            self.route3['filters'].append([time_i, time_f, filter_choice, params])
                            self.route3['history'].append(['filter', [time_i, time_f, filter_choice, params]])
                            self.refresh_plots_signal.emit([3])
                            self.changes_made()
                            break
                    else:
                        break
                    
            elif action == self.openTable:
                data = [2, 0, self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7]
                dlg = FuncTableForm(parent=self, data=data)
                dlg.setWindowTitle("Function as table")
                if dlg.exec():
                    pass
        else:
            self.moving_point[2] = False
            self.moving_vibrato[2] = False
            self.moving_filter[2] = False
            self.segundo_click[2] = False

    def onPointsClicked3(self, obj, point, event): # ver comentarios en onPointsClicked
        if not self.moving_point[2] and not self.moving_vibrato[2] and not self.moving_filter[2]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.pointMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.movePoint:
                self.changes_made()
                self.moving_point[2] = True
                self.moving_point_index[2] = self.find_closest_point(self.route3['points'], x, y)
            elif action == self.editPoint:
                index = self.find_closest_point(self.route3['points'], x, y)
                data = self.route3['points'][index]
                dlg = PointForm(parent=self, data=data, max_t=self.route3['total_t'], min_v=-99, max_v=99)
                dlg.setWindowTitle("Add Point")
                if dlg.exec():
                    self.edit_item(2, 0, index, data)
            elif action == self.deletePoint:
                p = self.route3['points'][self.find_closest_point(self.route3['points'], x, y)]
                self.route3['points'].remove(p)
                self.route3['history'].append(['delete_point', p])
                self.changes_made()
                try:
                    self.refresh_plots_signal.emit([3])
                except:
                    pass
        else:
            self.moving_point[2] = False
            self.moving_vibrato[2] = False
            self.moving_filter[2] = False
            self.segundo_click[2] = False

    def mouse_moved3(self, event): # ver comentarios en mouse_moved
        if self.moving_point[2]:
            self.segundo_click[2] = True
            vb = self.graphicsView_3.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_point_limits(self.route3['points'], self.moving_point_index[2], self.route3['total_t'])
            if x > min_x and x < max_x:
                self.route3['points'][self.moving_point_index[2]] = [x, y]
                self.refresh_plots_signal.emit([3])
        if self.moving_vibrato[2]:
            self.segundo_click[2] = True
            vb = self.graphicsView_3.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_vibrato_limits(self.route3['vibrato'], self.moving_vibrato_index[2], self.route3['total_t'])
            if x >= min_x and x <= max_x:
                self.route3['vibrato'][self.moving_vibrato_index[2]][0] = x
                self.refresh_plots_signal.emit([3])
        if self.moving_filter[2]:
            self.segundo_click[2] = True
            vb = self.graphicsView_3.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_filter_limits(self.route3['filters'], self.moving_filter_index[2], self.route3['total_t'])
            if x >= min_x and x <= max_x:
                diff = self.route3['filters'][self.moving_filter_index[2]][1] - self.route3['filters'][self.moving_filter_index[2]][0]
                self.route3['filters'][self.moving_filter_index[2]][0] = x
                self.route3['filters'][self.moving_filter_index[2]][1] = x + diff
                self.refresh_plots_signal.emit([3])

    def mouse_clicked3(self, event): # ver comentarios en mouse_clicked
        if (self.moving_point[2] or self.moving_vibrato[2] or self.moving_filter[2]) and self.segundo_click[2]:
            self.moving_point[2] = False
            self.moving_vibrato[2] = False
            self.moving_filter[2] = False
            self.segundo_click[2] = False
        else:
            if event.double():
                vb = self.graphicsView_3.plotItem.vb
                mouse_point = vb.mapSceneToView(event.scenePos())
                x = round(mouse_point.x(), 2)
                y = round(mouse_point.y(), 2)
                self.route3['points'].append([x, y])
                self.route3['points'].sort(key=lambda x: x[0])
                self.route3['history'].append(['add_point', [x, y]])
                self.refresh_plots_signal.emit([3])
                self.changes_made()

    def onFilterClicked4(self, obj, point, event): # ver comentarios en onFilterClicked
        if not self.moving_point[3] and not self.moving_vibrato[3] and not self.moving_filter[3]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.filterMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.moveFilter:
                self.changes_made()
                self.moving_filter[3] = True
                self.moving_filter_index[3] = self.find_closest_filter(self.route4['filters'], x)
            elif action == self.editFilter:
                index = self.find_closest_filter(self.route4['filters'], x)
                data = [0, 0] + [0 for i in range(14)] # time_i, time_f, filter_choice, window_choice, window_n, cutoff, Ap, As, fp, fs, chebN, chebAp, chebAs, chebfp, chebfs, chebrp
                new_data = self.route4['filters'][index] # i_init, i_end, filter, params
                data[0] = new_data[0]
                data[1] = new_data[1]
                data[2] = filter_choices.index(new_data[2])
                if data[2] == 0:
                    data[3] = filter_windows.index(new_data[3][0])
                    data[4] = new_data[3][1]
                    data[5] = new_data[3][2]
                elif data[2] == 3:
                    data[10] = new_data[3][0]
                    data[11] = new_data[3][1]
                    data[12] = new_data[3][2]
                    data[13] = new_data[3][3]
                    data[14] = new_data[3][4]
                    data[15] = new_data[3][5]
                else:
                    data[6] = new_data[3][0]
                    data[7] = new_data[3][1]
                    data[8] = new_data[3][2]
                    data[9] = new_data[3][3]
                while True:
                    dlg = FilterForm(parent=self, data=data)
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec():
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0:
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3:
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else:
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice]
                        if self.check_filter(time_i, time_f, filter_choice, params):
                            self.edit_item(3, 2, index, [time_i, time_f, filter_choice, params])
                            break
                    else:
                        break
            elif action == self.deleteFilter:
                index = self.find_closest_filter(self.route4['filters'], x)
                p = self.route4['filters'][index]
                self.route4['filters'].remove(p)
                self.route4['history'].append(['delete_filter', p])
                self.changes_made()
                self.refresh_plots_signal.emit([4])
        else:
            self.moving_point[3] = False
            self.moving_vibrato[3] = False
            self.moving_filter[3] = False
            self.segundo_click[3] = False

    def onVibratoClicked4(self, obj, point, event): # ver comentarios en onVibratoClicked
        if not self.moving_point[3] and not self.moving_vibrato[3] and not self.moving_filter[3]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.vibratoMenu.exec_(QtCore.QPoint(x_screen,y_screen))

            if action == self.moveVibrato:
                self.changes_made()
                self.moving_vibrato[3] = True
                self.moving_vibrato_index[3] = self.find_closest_vibrato(self.route4['vibrato'], x)
            elif action == self.editVibrato:
                index = self.find_closest_vibrato(self.route4['vibrato'], x)
                data = self.route4['vibrato'][index]
                data[4] = windows_vibrato.index(data[4])
                dlg = VibratoForm(parent=self, data=data, max_t=self.route4['total_t'])
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec():
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.edit_item(3, 1, index, [time_i, duration, amp, freq, window_v])
            elif action == self.deleteVibrato:
                index = self.find_closest_vibrato(self.route4['vibrato'], x)
                p = self.route4['vibrato'][index]
                self.route4['vibrato'].remove(p)
                self.route4['history'].append(['delete_vibrato', p])
                self.changes_made()
                self.refresh_plots_signal.emit([4])
        else:
            self.moving_point[3] = False
            self.moving_vibrato[3] = False
            self.moving_filter[3] = False
            self.segundo_click[3] = False
        
    def onCurveClicked4(self, obj, event): # ver comentarios en onCurveClicked
        if not self.moving_point[3] and not self.moving_vibrato[3] and not self.moving_filter[3]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.graphMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.addPoint:
                self.route4['points'].append([x, y])
                self.route4['points'].sort(key=lambda x: x[0])
                self.route4['history'].append(['add_point', [x, y]])
                self.refresh_plots_signal.emit([4])
                self.changes_made()
                self.moving_point[3] = True
                self.moving_point_index[3] = self.find_closest_point(self.route4['points'], x, y)
            elif action == self.addVibrato:
                data=[x, 0, 0, 0, 0]
                dlg = VibratoForm(parent=self, data=data, max_t=self.route4['total_t'])
                dlg.setWindowTitle("Add Vibrato")
                if dlg.exec():
                    time_i = data[0]
                    duration = data[1]
                    amp = data[2]
                    freq = data[3]
                    window_v = windows_vibrato[data[4]]
                    self.route4['vibrato'].append([time_i, duration, amp, freq, window_v])
                    self.route4['history'].append(['vibrato', [time_i, duration, amp, freq, window_v]])
                    self.changes_made()
                    self.refresh_plots_signal.emit([4])
            elif action == self.addFilter:
                data=[x, x] + [0 for i in range(14)]
                while True:
                    dlg = FilterForm(parent=self, data=data)
                    dlg.setWindowTitle("Add Filter")
                    if dlg.exec():
                        time_i = data[0]
                        time_f = data[1]
                        choice = data[2]
                        if choice == 0:
                            params = [filter_windows[data[3]], data[4], data[5]]
                        elif choice == 3:
                            params = [data[10], data[11], data[12], data[13], data[14], data[15]]
                        else:
                            params = [data[6], data[7], data[8], data[9]]
                        filter_choice = filter_choices[choice]
                        if self.check_filter(time_i, time_f, filter_choice, params):
                            self.route4['filters'].append([time_i, time_f, filter_choice, params])
                            self.route4['history'].append(['filter', [time_i, time_f, filter_choice, params]])
                            self.refresh_plots_signal.emit([4])
                            self.changes_made()
                            break
                    else:
                        break
                    
            elif action == self.openTable:
                data = [3, 0, self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7]
                dlg = FuncTableForm(parent=self, data=data)
                dlg.setWindowTitle("Function as table")
                if dlg.exec():
                    pass
        else:
            self.moving_point[3] = False
            self.moving_vibrato[3] = False
            self.moving_filter[3] = False
            self.segundo_click[3] = False

    def onPointsClicked4(self, obj, point, event): # ver comentarios en onPointsClicked
        if not self.moving_point[3] and not self.moving_vibrato[3] and not self.moving_filter[3]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.pointMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.movePoint:
                self.changes_made()
                self.moving_point[3] = True
                self.moving_point_index[3] = self.find_closest_point(self.route4['points'], x, y)
            elif action == self.editPoint:
                index = self.find_closest_point(self.route4['points'], x, y)
                data = self.route4['points'][index]
                dlg = PointForm(parent=self, data=data, max_t=self.route4['total_t'], min_v=0, max_v=50)
                dlg.setWindowTitle("Add Point")
                if dlg.exec():
                    self.edit_item(3, 0, index, data)
            elif action == self.deletePoint:
                p = self.route4['points'][self.find_closest_point(self.route4['points'], x, y)]
                self.route4['points'].remove(p)
                self.route4['history'].append(['delete_point', p])
                self.changes_made()
                try:
                    self.refresh_plots_signal.emit([4])
                except:
                    pass
        else:
            self.moving_point[3] = False
            self.moving_vibrato[3] = False
            self.moving_filter[3] = False
            self.segundo_click[3] = False

    def mouse_moved4(self, event): # ver comentarios en mouse_moved
        if self.moving_point[3]:
            self.segundo_click[3] = True
            vb = self.graphicsView_4.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = min(50,max(0, round(mouse_point.y(), 2)))
            min_x, max_x = self.find_moving_point_limits(self.route4['points'], self.moving_point_index[3], self.route4['total_t'])
            if x > min_x and x < max_x:
                self.route4['points'][self.moving_point_index[3]] = [x, y]
                self.refresh_plots_signal.emit([4])
        if self.moving_vibrato[3]:
            self.segundo_click[3] = True
            vb = self.graphicsView_4.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_vibrato_limits(self.route4['vibrato'], self.moving_vibrato_index[3], self.route4['total_t'])
            if x >= min_x and x <= max_x:
                self.route4['vibrato'][self.moving_vibrato_index[3]][0] = x
                self.refresh_plots_signal.emit([4])
        if self.moving_filter[3]:
            self.segundo_click[3] = True
            vb = self.graphicsView_4.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_filter_limits(self.route4['filters'], self.moving_filter_index[3], self.route4['total_t'])
            if x >= min_x and x <= max_x:
                diff = self.route4['filters'][self.moving_filter_index[3]][1] - self.route4['filters'][self.moving_filter_index[3]][0]
                self.route4['filters'][self.moving_filter_index[3]][0] = x
                self.route4['filters'][self.moving_filter_index[3]][1] = x + diff
                self.refresh_plots_signal.emit([4])

    def mouse_clicked4(self, event): # ver comentarios en mouse_clicked
        if (self.moving_point[3] or self.moving_vibrato[3] or self.moving_filter[3]) and self.segundo_click[3]:
            self.moving_point[3] = False
            self.moving_vibrato[3] = False
            self.moving_filter[3] = False
            self.segundo_click[3] = False
        else:
            if event.double():
                vb = self.graphicsView_4.plotItem.vb
                mouse_point = vb.mapSceneToView(event.scenePos())
                x = round(mouse_point.x(), 2)
                y = min(50,max(0, round(mouse_point.y(), 2)))
                self.route4['points'].append([x, y])
                self.route4['points'].sort(key=lambda x: x[0])
                self.route4['history'].append(['add_point', [x, y]])
                self.refresh_plots_signal.emit([4])
                self.changes_made()
    
    def onCurveClicked5(self, obj, event): # ver comentarios en onCurveClicked
        if not self.moving_point[4] and not self.moving_vibrato[4]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.noteMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.addNote:
                data = [x, int(round(2*y,0))]
                dlg = NoteForm(parent=self, data=data, max_t=self.route5['total_t'])
                dlg.setWindowTitle("Add Note")
                if dlg.exec():
                    new_x = data[0]
                    new_y = dict_notes[data[1]/2]
                    self.route5['notes'].append([new_x, new_y])
                    self.route5['notes'].sort(key=lambda x: x[0])
                    self.route5['history'].append(['add_notes', [new_x, new_y]])
                    self.changes_made()
                    self.refresh_plots_signal.emit([5])
            elif action == self.addTrill:
                data = [x, 0, 0, 0]
                dlg = TrillForm(parent=self, data=data, max_t=self.route5['total_t'])
                dlg.setWindowTitle("Add Note")
                if dlg.exec():
                    time = data[0]
                    dist = data[1]/2
                    freq = data[2]
                    duration = data[3]
                    self.route5['trill'].append([time, dist, freq, duration])
                    self.route5['trill'].sort(key=lambda x: x[0])
                    self.route5['history'].append(['add_trill', [time, dist, freq, duration]])
                    self.changes_made()
                    self.refresh_plots_signal.emit([5])
            elif action == self.openNotesTable:
                data = [4, 0, self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7]
                dlg = FuncTableForm(parent=self, data=data)
                dlg.setWindowTitle("Function as table")
                if dlg.exec():
                    pass
        else:
            self.moving_point[4] = False
            self.segundo_click[4] = False
            self.moving_vibrato[4] = False

    def onPointsClicked5(self, obj, point, event): # ver comentarios en onPointsClicked
        if not self.moving_point[4] and not self.moving_vibrato[4]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.pointMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.movePoint:
                self.changes_made()
                self.moving_point[4] = True
                self.moving_point_index[4] = self.find_closest_note(self.route5['notes'], x, y)
            elif action == self.editPoint:
                index = self.find_closest_note(self.route5['notes'], x, y)
                data = self.route5['notes'][index]
                data[1] = int(round(dict_notes_rev[data[1]]*2, 0))
                dlg = NoteForm(parent=self, data=data, max_t=self.route5['total_t'])
                dlg.setWindowTitle("Edit Note")
                if dlg.exec():
                    data[1] = dict_notes[data[1]/2]
                    self.edit_item(4, 0, index, data)
            elif action == self.deletePoint:
                p = self.route5['notes'][self.find_closest_note(self.route5['notes'], x, y)]
                self.route5['notes'].remove(p)
                self.route5['history'].append(['delete_notes', p])
                self.changes_made()
                try:
                    self.refresh_plots_signal.emit([5])
                except:
                    pass
        else:
            self.moving_point[4] = False
            self.segundo_click[4] = False
            self.moving_vibrato[4] = False

    def onVibratoClicked5(self, obj, point, event): # ver comentarios en onVibratoClicked
        ## En lugar de vibrato, en las notas usamos trill
        if not self.moving_point[4] and not self.moving_vibrato[4]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.vibratoMenu.exec_(QtCore.QPoint(x_screen,y_screen))

            if action == self.moveVibrato:
                self.changes_made()
                self.moving_vibrato[4] = True
                self.moving_vibrato_index[4] = self.find_closest_trill(self.route5['trill'], x)
            elif action == self.editVibrato:
                index = self.find_closest_trill(self.route5['trill'], x)
                data = self.route5['trill'][index]
                #data[4] = windows_vibrato.index(data[4])
                data[1] *= 2
                dlg = TrillForm(parent=self, data=data, max_t=self.route4['total_t'])
                dlg.setWindowTitle("Edit Trill")
                if dlg.exec():
                    time = data[0]
                    dist = data[1]/2
                    freq = data[2]
                    duration = data[3]
                    self.edit_item(4, 1, index, [time, dist, freq, duration])
            elif action == self.deleteVibrato:
                index = self.find_closest_trill(self.route5['trill'], x)
                p = self.route5['trill'][index]
                self.route5['trill'].remove(p)
                self.route5['history'].append(['delete_trill', p])
                self.changes_made()
                self.refresh_plots_signal.emit([5])
        else:
            self.moving_point[3] = False
            self.moving_vibrato[4] = False

    def mouse_moved5(self, event): # ver comentarios en mouse_moved
        if self.moving_point[4]:
            self.segundo_click[4] = True
            vb = self.graphicsView_5.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_note_limits(self.route5['notes'], self.moving_point_index[4], self.route5['total_t'])
            if x > min_x and x < max_x:
                if y >= 0 and y < 15.75:
                    note = dict_notes[round(y * 2) / 2]
                    self.route5['notes'][self.moving_point_index[4]] = [x, note]
                    self.refresh_plots_signal.emit([5])
                elif y >= 15.75:
                    note = dict_notes[17]
                    self.route5['notes'][self.moving_point_index[4]] = [x, note]
                    self.refresh_plots_signal.emit([5])
        if self.moving_vibrato[4]:
            self.segundo_click[4] = True
            vb = self.graphicsView_5.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_trill_limits(self.route5['trill'], self.moving_vibrato_index[4], self.route5['total_t'])
            if x >= min_x and x <= max_x:
                self.route5['trill'][self.moving_vibrato_index[4]][0] = x
                self.refresh_plots_signal.emit([5])

    def mouse_clicked5(self, event): # ver comentarios en mouse_clicked
        if (self.moving_point[4] or self.moving_vibrato[4]) and self.segundo_click[4]:
            self.moving_point[4] = False
            self.segundo_click[4] = False
            self.moving_vibrato[4] = False
        else:
            if event.double():
                vb = self.graphicsView_5.plotItem.vb
                mouse_point = vb.mapSceneToView(event.scenePos())
                x = round(mouse_point.x(), 2)
                y = round(mouse_point.y(), 2)
                if y < 0:
                    note = dict_notes[0]
                elif y >= 0 and y < 15.75:
                    note = dict_notes[round(y * 2) / 2]
                elif y >= 15.75:
                    note = dict_notes[17]
                self.route5['notes'].append([x, note])
                self.route5['notes'].sort(key=lambda x: x[0])
                self.route5['history'].append(['add_note', [x, note]])
                self.refresh_plots_signal.emit([5])
                self.changes_made()

    def onCurveClicked6(self, obj, event): # ver comentarios en onCurveClicked
        if not self.moving_point[5] and not self.moving_vibrato[5]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.tongueSurfaceMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.tongueSurfaceMenuPoint:
                data = [x, int(y)]
                dlg = SurfacePointForm(parent=self, data=data, max_t=self.route6['total_t'])
                dlg.setWindowTitle("Add Point")
                if dlg.exec():
                    new_x = data[0]
                    new_y = data[1]
                    self.route6['points'].append([new_x, new_y])
                    self.route6['points'].sort(key=lambda x: x[0])
                    self.route6['history'].append(['add_point', [new_x, new_y]])
                    self.changes_made()
                    self.refresh_plots_signal.emit([6])
            elif action == self.tongueSurfaceMenuTable:
                data = [5, 0, self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7]
                dlg = FuncTableForm(parent=self, data=data)
                dlg.setWindowTitle("Function as table")
                if dlg.exec():
                    pass
        else:
            self.moving_point[5] = False
            self.segundo_click[5] = False

    def onPointsClicked6(self, obj, point, event): # ver comentarios en onPointsClicked
        if not self.moving_point[5] and not self.moving_vibrato[5]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.pointMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.movePoint:
                self.changes_made()
                self.moving_point[5] = True
                self.moving_point_index[5] = self.find_closest_point(self.route6['points'], x, y)
            elif action == self.editPoint:
                index = self.find_closest_point(self.route6['points'], x, y)
                data = self.route6['points'][index]
                dlg = SurfacePointForm(parent=self, data=data, max_t=self.route6['total_t'])
                dlg.setWindowTitle("Edit Point")
                if dlg.exec():
                    self.edit_item(5, 0, index, data)
            elif action == self.deletePoint:
                p = self.route6['points'][self.find_closest_point(self.route6['points'], x, y)]
                self.route6['points'].remove(p)
                self.route6['history'].append(['delete_point', p])
                self.changes_made()
                try:
                    self.refresh_plots_signal.emit([6])
                except:
                    pass
        else:
            self.moving_point[5] = False
            self.segundo_click[5] = False

    def mouse_moved6(self, event): # ver comentarios en mouse_moved
        if self.moving_point[5]:
            self.segundo_click[5] = True
            vb = self.graphicsView_6.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_point_limits(self.route6['points'], self.moving_point_index[5], self.route6['total_t'])
            if x > min_x and x < max_x:
                if y >= 0 and y < 32:
                    self.route6['points'][self.moving_point_index[5]] = [x, int(y)]
                    self.refresh_plots_signal.emit([6])

    def mouse_clicked6(self, event): # ver comentarios en mouse_clicked
        if self.moving_point[5] and self.segundo_click[5]:
            self.moving_point[5] = False
            self.segundo_click[5] = False
        else:
            if event.double():
                vb = self.graphicsView_6.plotItem.vb
                mouse_point = vb.mapSceneToView(event.scenePos())
                x = round(mouse_point.x(), 2)
                y = round(mouse_point.y(), 0)
                if y < 0:
                    note = 0
                elif y >= 0 and y < 32:
                    note = int(y)
                elif y >= 32:
                    note = 31
                self.route6['points'].append([x, note])
                self.route6['points'].sort(key=lambda x: x[0])
                self.route6['history'].append(['add_point', [x, note]])
                self.refresh_plots_signal.emit([6])
                self.changes_made()

    def onCurveClicked7(self, obj, event): # ver comentarios en onCurveClicked
        if not self.moving_point[6] and not self.moving_vibrato[6]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.tongueSurfaceMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.tongueSurfaceMenuPoint:
                data = [x, int(y)]
                dlg = TonguePointForm(parent=self, data=data, max_t=self.route7['total_t'])
                dlg.setWindowTitle("Add Point")
                if dlg.exec():
                    new_x = data[0]
                    new_y = data[1]
                    self.route7['points'].append([new_x, new_y])
                    self.route7['points'].sort(key=lambda x: x[0])
                    self.route7['history'].append(['add_point', [new_x, new_y]])
                    self.changes_made()
                    self.refresh_plots_signal.emit([7])
            elif action == self.tongueSurfaceMenuTable:
                data = [6, 0, self.route, self.route2, self.route3, self.route4, self.route5, self.route6, self.route7]
                dlg = FuncTableForm(parent=self, data=data)
                dlg.setWindowTitle("Function as table")
                if dlg.exec():
                    pass
        else:
            self.moving_point[6] = False
            self.segundo_click[6] = False

    def onPointsClicked7(self, obj, point, event): # ver comentarios en onPointsClicked
        if not self.moving_point[6] and not self.moving_vibrato[6]:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.pointMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.movePoint:
                self.changes_made()
                self.moving_point[6] = True
                self.moving_point_index[6] = self.find_closest_point(self.route7['points'], x, y)
            elif action == self.editPoint:
                index = self.find_closest_point(self.route7['points'], x, y)
                data = self.route7['points'][index]
                dlg = TonguePointForm(parent=self, data=data, max_t=self.route7['total_t'])
                dlg.setWindowTitle("Edit Point")
                if dlg.exec():
                    self.edit_item(6, 0, index, data)
            elif action == self.deletePoint:
                p = self.route7['points'][self.find_closest_point(self.route7['points'], x, y)]
                self.route7['points'].remove(p)
                self.route7['history'].append(['delete_point', p])
                self.changes_made()
                try:
                    self.refresh_plots_signal.emit([7])
                except:
                    pass
        else:
            self.moving_point[6] = False
            self.segundo_click[6] = False

    def mouse_moved7(self, event): # ver comentarios en mouse_moved
        if self.moving_point[6]:
            self.segundo_click[6] = True
            vb = self.graphicsView_7.plotItem.vb
            mouse_point = vb.mapSceneToView(event)
            x = round(mouse_point.x(), 2)
            y = round(mouse_point.y(), 2)
            min_x, max_x = self.find_moving_point_limits(self.route7['points'], self.moving_point_index[6], self.route7['total_t'])
            if x > min_x and x < max_x:
                if y >= 0 and y < 4:
                    self.route7['points'][self.moving_point_index[6]] = [x, int(y)]
                    self.refresh_plots_signal.emit([7])

    def mouse_clicked7(self, event): # ver comentarios en mouse_clicked
        if self.moving_point[6] and self.segundo_click[6]:
            self.moving_point[6] = False
            self.segundo_click[6] = False
        else:
            if event.double():
                vb = self.graphicsView_7.plotItem.vb
                mouse_point = vb.mapSceneToView(event.scenePos())
                x = round(mouse_point.x(), 2)
                y = round(mouse_point.y(), 0)
                if y < 0:
                    note = 0
                elif y >= 0 and y < 4:
                    note = int(y)
                elif y >= 4:
                    note = 3
                self.route7['points'].append([x, note])
                self.route7['points'].sort(key=lambda x: x[0])
                self.route7['history'].append(['add_point', [x, note]])
                self.refresh_plots_signal.emit([7])
                self.changes_made()

#### Final de funciones de click y movimiento de mouse
####

    def check_filter(self, time_i, time_f, filter_choice, params): # evalua si los parametros elegidos para un filtro son validos
        ## TODO: implementar funcion que revise si el filtro es valido
        if time_f < time_i:
            return False
        if filter_choice == 0: # ventana -> params = [window_choice, window_n, cutoff]
            if params[1] == 0 or params[2] == 0:
                return False
        elif filter_choice == 1: # remez -> params = [Ap, As, fp, fs]
            pass
        elif filter_choice == 2: # butter -> params = [Ap, As, fp, fs]
            pass
        elif filter_choice == 3: # cheb -> params = [N, Ap, As, fp, fs, rp]
            pass
        elif filter_choice == 4: # elliptic -> params = [Ap, As, fp, fs]
            pass
        return True 

    def find_moving_note_limits(self, notes, index, total_t): # busca los limites de movimiento para una nota en particular
        lim_inf = 0
        lim_sup = total_t
        if index > 0:
            lim_inf = notes[index - 1][0] # no es posible moverlo antes de la nota anterior
        if index < len(notes) - 1:
            lim_sup = notes[index + 1][0] # no es posible moverlo despues de la nota que le sigue
        return lim_inf, lim_sup
    
    def find_moving_filter_limits(self, filters, index, total_t): # busca los limites de movimiento para un filtro en particular
        return 0, total_t - filters[index][1] # entre 0 y el largo total menos su duracion
    
    def find_moving_vibrato_limits(self, vibratos, index, total_t): # busca los limites de movimiento para un vibrato en particular
        return 0, total_t - vibratos[index][1] # entre 0 y el largo total menos su duracion
    
    def find_moving_trill_limits(self, trills, index, total_t): # busca los limites de movimiento para un trill en particular
        return 0, total_t - trills[index][3] # entre 0 y el largo total menos su duracion
    
    def find_moving_point_limits(self, points, index, total_t): # busca los limites de movimiento para un punto en particular
        lim_inf = 0
        lim_sup = total_t
        if index > 0:
            lim_inf = points[index - 1][0] # no es posible moverlo antes del punto anterior
        if index < len(points) - 1:
            lim_sup = points[index + 1][0] # no es posible moverlo despues del punto que le sigue
        return lim_inf, lim_sup
    
    def find_closest_note(self, notes, x, y): # busca la nota mas cercana a las coordenadas x, y en donde se hizo click (que puede no ser necesariamente justo arriba del punto que se quiera mover... se necesita una holgura)
        if len(notes) == 0:
            return -1 # si no hay ninguna nota se devuelve un indice de -1 para indicar error
        else:
            closest = 0 # partimos con el indice 0
            dist = abs(notes[0][0] - x) + abs(dict_notes_rev[notes[0][1]] - y) # calculamos la distancia simple del primer punto al lugar donde se hizo click
            for i in range(len(notes)): # iteramos en la lista de notas
                new_dist = abs(notes[i][0] - x) + abs(dict_notes_rev[notes[0][1]] - y) # calculamos la distancia de cada punto al lugar en donde se hizo click
                if new_dist < dist: # si esta nueva distancia es menor a la que teniamos antes como la mas cercana, asignamos un nuevo punto mas cercano
                    closest = i # cambiamos el indice del mas cercano
                    dist = new_dist # y actualizamos la menor distancia
            return closest # despues de iterar en todos los elementos de la lista, encontramos el mas cercano de todos

    def find_closest_point(self, points, x, y): # busca el punto mas cercano a las coordenadas x, y en donde se hizo click (que puede no ser necesariamente justo arriba del punto que se quiera mover... se necesita una holgura). Ver comentarios de find_closest_note
        if len(points) == 0:
            return -1
        else:
            closest = 0
            dist = abs(points[0][0] - x) + abs(points[0][1] - y)
            for i in range(len(points)):
                new_dist = abs(points[i][0] - x) + abs(points[i][1] - y)
                if new_dist < dist:
                    closest = i
                    dist = new_dist
            return closest
    
    def find_closest_filter(self, filters, x): # busca el filtro mas cercano a las coordenadas x, y en donde se hizo click (que puede no ser necesariamente justo arriba del punto que se quiera mover... se necesita una holgura). Ver comentarios de find_closest_note
        if len(filters) == 0:
            return -1
        else:
            closest = 0
            dist = abs(filters[0][0] - x)
            for i in range(len(filters)):
                new_dist = abs(filters[i][0] - x)
                if new_dist < dist:
                    closest = i
                    dist = new_dist
            return closest
        
    def find_closest_vibrato(self, vibratos, x): # busca el vibrato mas cercano a las coordenadas x, y en donde se hizo click (que puede no ser necesariamente justo arriba del punto que se quiera mover... se necesita una holgura). Ver comentarios de find_closest_note
        if len(vibratos) == 0:
            return -1
        else:
            closest = 0
            dist = abs(vibratos[0][0] - x)
            for i in range(len(vibratos)):
                new_dist = abs(vibratos[i][0] - x)
                if new_dist < dist:
                    closest = i
                    dist = new_dist
            return closest
    
    def find_closest_trill(self, trills, x): # busca el trill mas cercano a las coordenadas x, y en donde se hizo click (que puede no ser necesariamente justo arriba del punto que se quiera mover... se necesita una holgura). Ver comentarios de find_closest_note
        if len(trills) == 0:
            return -1
        else:
            closest = 0
            dist = abs(trills[0][0] - x)
            for i in range(len(trills)):
                new_dist = abs(trills[i][0] - x)
                if new_dist < dist:
                    closest = i
                    dist = new_dist
            return closest
        
    def edit_item(self, func, prop, index, params): 
        """
        Permite editar cualquier elemento de uno de los graficos.
        sus parametros son:
        - func (int) -> un numero de 0 a 4 que indica a cual de los graficos pertenece el elemento que se quiere editar
        - prop (int) -> un numero de 0 a 2 que indica que tipo de elemento se quiere editar, 0: punto/nota, 1: vibrato/trill o 2: filtro
        - index (int) -> indica el indice del elemento que se quiere editar
        - params (list) -> entrega la lista de parametros para el nuevo elemento. Varia de acuerdo al elemento que se quiere editar 
        """
        self.changes_made() # avisamos que se hacen cambios
        if func == 0: # para editar un elemento del primer grafico
            if prop == 0: # punto
                self.route['points'][index] = params
                self.route['points'].sort(key=lambda x: x[0])
            elif prop == 1: # vibrato
                self.route['vibrato'][index] = params
                self.route['vibrato'].sort(key=lambda x: x[0])
            elif prop == 2: # filtro
                self.route['filters'][index] = params
                self.route['filters'].sort(key=lambda x: x[0])
            self.refresh_plots_signal.emit([1]) # actualizamos grafico
        elif func == 1: # para editar un elemento del segundo grafico
            if prop == 0: # punto
                self.route2['points'][index] = params
                self.route2['points'].sort(key=lambda x: x[0])
            elif prop == 1: # vibrato
                self.route2['vibrato'][index] = params
                self.route2['vibrato'].sort(key=lambda x: x[0])
            elif prop == 2: # filtro
                self.route2['filters'][index] = params
                self.route2['filters'].sort(key=lambda x: x[0])
            self.refresh_plots_signal.emit([2]) # actualizamos grafico
        elif func == 2: # para editar un elemento del tercer grafico
            if prop == 0: # punto
                self.route3['points'][index] = params
                self.route3['points'].sort(key=lambda x: x[0])
            elif prop == 1: # vibrato
                self.route3['vibrato'][index] = params
                self.route3['vibrato'].sort(key=lambda x: x[0])
            elif prop == 2: # filtro
                self.route3['filters'][index] = params
                self.route3['filters'].sort(key=lambda x: x[0])
            self.refresh_plots_signal.emit([3]) # actualizamos grafico
        elif func == 3: # para editar un elemento del cuarto grafico
            if prop == 0: # punto
                self.route4['points'][index] = params
                self.route4['points'].sort(key=lambda x: x[0])
            elif prop == 1: # vibrato
                self.route4['vibrato'][index] = params
                self.route4['vibrato'].sort(key=lambda x: x[0])
            elif prop == 2: # filtro
                self.route4['filters'][index] = params
                self.route4['filters'].sort(key=lambda x: x[0])
            self.refresh_plots_signal.emit([4]) # actualizamos grafico
        elif func == 4: # para editar un elemento del quinto grafico
            if prop == 0: # nota
                self.route5['notes'][index] = params
                self.route5['notes'].sort(key=lambda x: x[0])
            elif prop == 1: # trill
                self.route5['trill'][index] = params
                self.route5['trill'].sort(key=lambda x: x[0])
            self.refresh_plots_signal.emit([5]) # actualizamos grafico
        elif func == 5: # para editar un elemento del sexto grafico
            if prop == 0: # punto
                self.route6['points'][index] = params
                self.route6['points'].sort(key=lambda x: x[0])
            self.refresh_plots_signal.emit([6]) # actualizamos grafico
        elif func == 6: # para editar un elemento del septimo grafico
            if prop == 0: # punto
                self.route7['points'][index] = params
                self.route7['points'].sort(key=lambda x: x[0])
            self.refresh_plots_signal.emit([7]) # actualizamos grafico

    def delete_item(self, func, prop, index):
        """
        Permite eliminar cualquier elemento de uno de los graficos.
        sus parametros son:
        - func (int) -> un numero de 0 a 4 que indica a cual de los graficos pertenece el elemento que se quiere eliminar
        - prop (int) -> un numero de 0 a 2 que indica que tipo de elemento se quiere eliminar, 0: punto/nota, 1: vibrato/trill o 2: filtro
        - index (int) -> indica el indice del elemento que se quiere eliminar
        """
        self.changes_made() # avisamos que se hacen cambios
        if func == 0: # para eliminar un elemento del primer grafico
            if prop == 0: # punto
                p = self.route['points'].pop(index)
                self.route['history'].append(['delete_point', p])
            elif prop == 1: # vibrato
                p = self.route['vibrato'].pop(index)
                self.route['history'].append(['delete_vibrato', p])
            elif prop == 2: # filtro
                p = self.route['filters'].pop(index)
                self.route['history'].append(['delete_filter', p])
            self.refresh_plots_signal.emit([1])
        elif func == 1: # para eliminar un elemento del segundo grafico
            if prop == 0: # punto
                p = self.route2['points'].pop(index)
                self.route2['history'].append(['delete_point', p])
            elif prop == 1: # vibrato
                p = self.route2['vibrato'].pop(index)
                self.route2['history'].append(['delete_vibrato', p])
            elif prop == 2: # filtro
                p = self.route2['filters'].pop(index)
                self.route2['history'].append(['delete_filter', p])
            self.refresh_plots_signal.emit([2])
        elif func == 2: # para eliminar un elemento del tercer grafico
            if prop == 0: # punto
                p = self.route3['points'].pop(index)
                self.route3['history'].append(['delete_point', p])
            elif prop == 1: # vibrato
                p = self.route3['vibrato'].pop(index)
                self.route3['history'].append(['delete_vibrato', p])
            elif prop == 2: # filtro
                p = self.route3['filters'].pop(index)
                self.route3['history'].append(['delete_filter', p])
            self.refresh_plots_signal.emit([3])
        elif func == 3: # para eliminar un elemento del cuarto grafico
            if prop == 0: # punto
                p = self.route4['points'].pop(index)
                self.route4['history'].append(['delete_point', p])
            elif prop == 1: # vibrato
                p = self.route4['vibrato'].pop(index)
                self.route4['history'].append(['delete_vibrato', p])
            elif prop == 2: # filtro
                p = self.route4['filters'].pop(index)
                self.route4['history'].append(['delete_filter', p])
            self.refresh_plots_signal.emit([4])
        elif func == 4: # para eliminar un elemento del quinto grafico
            if prop == 0: # nota
                p = self.route5['notes'].pop(index)
                self.route5['history'].append(['delete_note', p])
            if prop == 1: # trill
                p = self.route5['trill'].pop(index)
                self.route5['history'].append(['delete_trill', p])
            self.refresh_plots_signal.emit([5])
        elif func == 5: # para eliminar un elemento del sexto grafico
            if prop == 0: # punto
                p = self.route6['points'].pop(index)
                self.route6['history'].append(['delete_point', p])
            self.refresh_plots_signal.emit([6])
        elif func == 6: # para eliminar un elemento del septimo grafico
            if prop == 0: # punto
                p = self.route7['points'].pop(index)
                self.route7['history'].append(['delete_point', p])
            self.refresh_plots_signal.emit([7])

    def add_item(self, func, prop, params, from_func=False):
        """
        Permite editar cualquier elemento de uno de los graficos.
        sus parametros son:
        - func (int) -> un numero de 0 a 4 que indica a cual de los graficos pertenece el elemento que se quiere editar
        - prop (int) -> un numero de 0 a 2 que indica que tipo de elemento se quiere editar, 0: punto/nota, 1: vibrato/trill o 2: filtro
        - index (int) -> indica el indice del elemento que se quiere editar
        - params (list) -> entrega la lista de parametros para el nuevo elemento. Varia de acuerdo al elemento que se quiere editar 
        - from_func (bool) -> si un elemento se agregó al hacer un get_states_from_notes. Si fue asi, no se informan que hubo cambios (para solo hacerlo una vez al final)
        """
        if not from_func: # si no se agrega el elemento desde get_states_from_notes informamos que hay cambios
            self.changes_made()
        if func == 0: # para agregar un elemento al primer grafico
            if prop == 0: # punto
                self.route['points'].append(params)
                self.route['points'].sort(key=lambda x: x[0])
                self.route['history'].append(['add_point', params])
            elif prop == 1: # vibrato
                self.route['vibrato'].append(params)
                self.route['vibrato'].sort(key=lambda x: x[0])
                self.route['history'].append(['vibrato', params])
            elif prop == 2: # filtro
                self.route['filters'].append(params)
                self.route['filters'].sort(key=lambda x: x[0])
                self.route['history'].append(['filter', params])
            self.refresh_plots_signal.emit([1])
        elif func == 1: # para agregar un elemento al segundo grafico
            if prop == 0: # punto
                self.route2['points'].append(params)
                self.route2['points'].sort(key=lambda x: x[0])
                self.route2['history'].append(['add_point', params])
            elif prop == 1: # vibrato
                self.route2['vibrato'].append(params)
                self.route2['vibrato'].sort(key=lambda x: x[0])
                self.route2['history'].append(['vibrato', params])
            elif prop == 2: # filtro
                self.route2['filters'].append(params)
                self.route2['filters'].sort(key=lambda x: x[0])
                self.route2['history'].append(['filter', params])
            self.refresh_plots_signal.emit([2])
        elif func == 2: # para agregar un elemento al tercer grafico
            if prop == 0: # punto
                self.route3['points'].append(params)
                self.route3['points'].sort(key=lambda x: x[0])
                self.route3['history'].append(['add_point', params])
            elif prop == 1: # vibrato
                self.route3['vibrato'].append(params)
                self.route3['vibrato'].sort(key=lambda x: x[0])
                self.route3['history'].append(['vibrato', params])
            elif prop == 2: # filtro
                self.route3['filters'].append(params)
                self.route3['filters'].sort(key=lambda x: x[0])
                self.route3['history'].append(['filter', params])
            self.refresh_plots_signal.emit([3])
        elif func == 3: # para agregar un elemento al cuarto grafico
            if prop == 0: # punto
                self.route4['points'].append(params)
                self.route4['points'].sort(key=lambda x: x[0])
                self.route4['history'].append(['add_point', params])
            elif prop == 1: # vibrato
                self.route4['vibrato'].append(params)
                self.route4['vibrato'].sort(key=lambda x: x[0])
                self.route4['history'].append(['vibrato', params])
            elif prop == 2: # filtro
                self.route4['filters'].append(params)
                self.route4['filters'].sort(key=lambda x: x[0])
                self.route4['history'].append(['filter', params])
            self.refresh_plots_signal.emit([4])
        elif func == 4: # para agregar un elemento al quinto grafico
            if prop == 0: # nota
                self.route5['notes'].append(params)
                self.route5['notes'].sort(key=lambda x: x[0])
                self.route5['history'].append(['add_note', params])
            if prop == 1: # trill
                self.route5['trill'].append(params)
                self.route5['trill'].sort(key=lambda x: x[0])
                self.route5['history'].append(['add_trill', params])
            self.refresh_plots_signal.emit([5])
        elif func == 5: # para agregar un elemento al sexto grafico
            if prop == 0: # punto
                self.route6['points'].append(params)
                self.route6['points'].sort(key=lambda x: x[0])
                self.route6['history'].append(['add_point', params])
            self.refresh_plots_signal.emit([6])
        elif func == 6: # para agregar un elemento al septimo grafico
            if prop == 0: # punto
                self.route7['points'].append(params)
                self.route7['points'].sort(key=lambda x: x[0])
                self.route7['history'].append(['add_point', params])
            self.refresh_plots_signal.emit([7])

    def populate_graph(self): # rellena las rutas de las trayectorias con una ruta estandar, de 20s de duracion con un punto en cada trayectoria
        self.route = {  # la ruta que define el primer grafico
                        'total_t': 20, # tiempo total
                        'Fs': 100, # frecuencia de sampleo (no cambia)
                        'points': [[0, 10]], # lista de puntos por los que pasa
                        'filters': [], # lista de filtros. Parte vacia
                        'vibrato': [], # lista de vibratos tambien parte vacia
                        'history': [] # lista de historial. Obsoleto
                     }
        self.route2 = {   # la ruta que define el segundo grafico
                        'total_t': 20,
                        'Fs': 100,
                        'points': [[0, 45]],
                        'filters': [],
                        'vibrato': [],
                        'history': []
                     }
        self.route3 = {   # la ruta que define el tercer grafico
                        'total_t': 20,
                        'Fs': 100,
                        'points': [[0, 0]],
                        'filters': [],
                        'vibrato': [],
                        'history': []
                     }
        self.route4 = {   # la ruta que define el cuarto grafico
                        'total_t': 20,
                        'Fs': 100,
                        'points': [[0, 0]],
                        'filters': [],
                        'vibrato': [],
                        'history': []
                     }
        self.route5 = {   # la ruta que define el quinto grafico
                        'total_t': 20,
                        'Fs': 100,
                        'notes': [],
                        'trill': [],
                        'history': []
                     }
        self.route6 = {   # la ruta que define el segundo grafico
                        'total_t': 20,
                        'Fs': 100,
                        'points': [[0, 31]],
                        'history': []
                     }
        self.route7 = {   # la ruta que define el segundo grafico
                        'total_t': 20,
                        'Fs': 100,
                        'points': [[0, 0]],
                        'history': []
                     }

    def reprint_real_func(self): # actualiza las curvas de las mediciones reales de los parametros
        self.r_real.setData(self.t_plot, self.r_plot) # actualiza la curva de l
        self.theta_real.setData(self.t_plot, self.theta_plot) # actualiza la curva de theta
        self.offset_real.setData(self.t_plot, self.offset_plot) # actualiza la curva del offset
        self.flow_real.setData(self.t_plot, self.flow_plot) # actualiza la curva del flow
        self.freq_real.setData(self.t_plot, self.freq_plot) # actualiza la curva del pitch

    def reprint_plot_1(self): # actualiza la curva para la trayectoria del primer grafico
        t, f, p, vib, fil = calculate_route(self.route) # esta funcion entrega a partir de la trayectoria que se le entrega, un vector con el tiempo, otro para la curva (f) con todos los puntos vibratos y filtros incorporados. Tambien entrega la lista con las posiciones de los puntos para el scatter plot (p), lo mismo para los puntos que representan los vibratos (vib) y los que representan los filtros (fil)
        # listas para el scatter de los puntos verdes
        x = []
        y = []
        for point in p:
            x.append(point[0])
            y.append(point[1])
        # listas para el scatter de los puntos rojos
        vib_x = []
        vib_y = []
        for v in vib:
            vib_x.append(v)
            vib_y.append(f[int(round(v*100, 0))] + 1) # los puntos rojos los graficamos 1 unidad mas arriba de la curva, para evitar que calcen con otros puntos verdes o azules
        # listas para el scatter de los puntos azules
        fil_x = []
        fil_y = []
        for fi in fil:
            fil_x.append(fi)
            fil_y.append(f[int(round(fi*100, 0))] - 1) # los puntos azulos los graficamos 1 unidad mas abajo de la curva, para evitar que calcen con otros puntos verdes o rojos
        # ahora actualizamos las curvas y scatters
        self.func1.setData(t, f)
        self.scatter1.setData(x, y)
        self.vibscatter1.setData(vib_x, vib_y)
        self.filscatter1.setData(fil_x, fil_y)
    
    def reprint_plot_2(self): # actualiza la curva para la trayectoria del segundo grafico. Ver comentarios de reprint_plot_1
        t, f, p, vib, fil = calculate_route(self.route2)
        x = []
        y = []
        for point in p:
            x.append(point[0])
            y.append(point[1])
        vib_x = []
        vib_y = []
        for v in vib:
            vib_x.append(v)
            vib_y.append(f[int(round(v*100, 0))] + 1)
        fil_x = []
        fil_y = []
        for fi in fil:
            fil_x.append(fi)
            fil_y.append(f[int(round(fi*100, 0))] - 1)
        self.func2.setData(t, f)
        self.scatter2.setData(x, y)
        self.vibscatter2.setData(vib_x, vib_y)
        self.filscatter2.setData(fil_x, fil_y)
    
    def reprint_plot_3(self): # actualiza la curva para la trayectoria del tercer grafico. Ver comentarios de reprint_plot_1
        t, f, p, vib, fil = calculate_route(self.route3)
        x = []
        y = []
        for point in p:
            x.append(point[0])
            y.append(point[1])
        vib_x = []
        vib_y = []
        for v in vib:
            vib_x.append(v)
            vib_y.append(f[int(round(v*100, 0))] + 1)
        fil_x = []
        fil_y = []
        for fi in fil:
            fil_x.append(fi)
            fil_y.append(f[int(round(fi*100, 0))] - 1)
        self.func3.setData(t, f)
        self.scatter3.setData(x, y)
        self.vibscatter3.setData(vib_x, vib_y)
        self.filscatter3.setData(fil_x, fil_y)
    
    def reprint_plot_4(self): # actualiza la curva para la trayectoria del cuarto grafico. Ver comentarios de reprint_plot_1
        t, f, p, vib, fil = calculate_route(self.route4)
        x = []
        y = []
        for point in p:
            x.append(point[0])
            y.append(point[1])
        vib_x = []
        vib_y = []
        for v in vib:
            vib_x.append(v)
            vib_y.append(f[int(round(v*100, 0))] + 1)
        fil_x = []
        fil_y = []
        for fi in fil:
            fil_x.append(fi)
            fil_y.append(f[int(round(fi*100, 0))] - 1)
        self.func4.setData(t, f)
        self.scatter4.setData(x, y)
        self.vibscatter4.setData(vib_x, vib_y)
        self.filscatter4.setData(fil_x, fil_y)
    
    def reprint_plot_5(self): # actualiza la curva para la trayectoria del quinto grafico. Ver comentarios de reprint_plot_1
        t, f, px, py, tr_x, tr_y = calculate_notes_route(self.route5)
        self.func5.setData(t, f)
        self.scatter5.setData(px, py)
        self.vibscatter5.setData(tr_x, tr_y)

    def reprint_plot_6(self): # actualiza la curva para la trayectoria del segundo grafico. Ver comentarios de reprint_plot_1
        t, f, px, py = calculate_route_tongue(self.route6)
        self.func6.setData(t, f)
        self.scatter6.setData(px, py)
    
    def reprint_plot_7(self): # actualiza la curva para la trayectoria del segundo grafico. Ver comentarios de reprint_plot_1
        t, f, px, py = calculate_route_tongue(self.route7)
        self.func7.setData(t, f)
        self.scatter7.setData(px, py)

    def checkBox_clicked(self, value): # revisa cambios en el estado del checkBox
        if value:
            self.graphicsView.show() # muestra el primer grafico
        else:
            self.graphicsView.hide() # esconde el primer grafico
    
    def checkBox2_clicked(self, value): # revisa cambios en el estado del checkBox2
        if value:
            self.graphicsView_2.show() # muestra el segundo grafico
        else:
            self.graphicsView_2.hide() # esconde el segundo grafico

    def checkBox3_clicked(self, value): # revisa cambios en el estado del checkBox3
        if value:
            self.graphicsView_3.show() # muestra el tercer grafico
        else:
            self.graphicsView_3.hide() # esconde el tercer grafico

    def checkBox4_clicked(self, value): # revisa cambios en el estado del checkBox4
        if value:
            self.graphicsView_4.show() # muestra el cuarto grafico
        else:
            self.graphicsView_4.hide() # esconde el cuarto grafico

    def checkBox5_clicked(self, value): # revisa cambios en el estado del checkBox5
        if value:
            self.graphicsView_5.show() # muestra el quinto grafico
        else:
            self.graphicsView_5.hide() # esconde el quinto grafico

    def checkBox6_clicked(self, value): # revisa cambios en el estado del checkBox5
        if value:
            self.graphicsView_6.show() # muestra el quinto grafico
        else:
            self.graphicsView_6.hide() # esconde el quinto grafico

    def checkBox7_clicked(self, value): # revisa cambios en el estado del checkBox5
        if value:
            self.graphicsView_7.show() # muestra el quinto grafico
        else:
            self.graphicsView_7.hide() # esconde el quinto grafico
    
    def checkBox8_clicked(self, value): # revisa cambios en el estado del checkBox5
        if value:
            self.graphicsView_8.show() # muestra el quinto grafico
        else:
            self.graphicsView_8.hide() # esconde el quinto grafico

    def plot_measure(self, measure, title):
        '''
        Se usa esta función para desplegar una ventana con el gráfico de alguna variable de interés
        '''
        plotwin = LivePlotWindow(self.app, measure, self.data, parent=self)
        plotwin.setWindowTitle(title)
        plotwin.show()

    def measure_radius(self):
        '''
        Para graficar la evolución en el tiempo del radio
        '''
        self.plot_measure(0, "Radius Plot")

    def measure_theta(self):
        '''
        Para graficar la evolución en el tiempo del ángulo de incidencia
        '''
        self.plot_measure(1, "Theta Plot")

    def measure_offset(self):
        '''
        Para graficar la evolución en el tiempo del offset
        '''
        self.plot_measure(2, "Offset Plot")

    def measure_position(self):
        '''
        Para graficar la evolución en el tiempo de la posición (plano XZ)
        '''
        self.plot_measure(3, "Position Plot")

    def measure_mouth_presure(self):
        '''
        Para graficar la evolución en el tiempo de la presión en la boca
        '''
        self.plot_measure(4, "Mouth Preasure Plot")

    def measure_mass_flow_rate(self):
        '''
        Para graficar la evolución en el tiempo del flujo másico
        '''
        self.plot_measure(5, "Mass Flow Rate Plot")

    def measure_volume_flow_rate(self):
        '''
        Para graficar la evolución en el tiempo del flujo volumétrico
        '''
        self.plot_measure(6, "Volume Flow Rate Plot")

    def measure_temperature(self):
        '''
        Para graficar la evolución en el tiempo de la temperatura del flujo
        '''
        self.plot_measure(7, "Flow Temperature Plot")

    def measure_sound_frequency(self):
        '''
        Para graficar la evolución en el tiempo de la frecuencia del sonido
        '''
        self.plot_measure(8, "Sound Frequency Plot")

    def measure_x_position(self):
        '''
        Para graficar la evolución en el tiempo de x
        '''
        self.plot_measure(9, "X Position Plot")

    def measure_z_position(self):
        '''
        Para graficar la evolución en el tiempo de z
        '''
        self.plot_measure(10, "Z Position Plot")
    
    def measure_alpha_position(self):
        '''
        Para graficar la evolución en el tiempo de alpha
        '''
        self.plot_measure(11, "Alpha Position Plot")

if __name__ == "__main__":
    pass