import sys
sys.path.insert(0, 'C:/Users/ferna/Dropbox/UC/Magister/pierre_flutist')
from PyQt5 import QtGui
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QFileDialog, QLineEdit, QPushButton, QVBoxLayout, QWidget, QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsRectItem, QGraphicsEllipseItem, QMenu
    )
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import Qt
from PyQt5 import QtCore
from src.views.interactive_plot_signal_creator import Ui_Form as InteractivePlotUi
from src.forms.forms import DynamicForm
from src.route import get_route_ramped
from src.motor_route import get_value_from_func_2d
import os
import json
import pyqtgraph as pg
import numpy as np



class InteractivePlotWidget(QWidget, InteractivePlotUi):
    refresh_plot_signal = QtCore.pyqtSignal()
    reset_plot_signal = QtCore.pyqtSignal()
    def __init__(self, app, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.app = app
        self.parent = parent
        self.dict_number = {}
        self.dict_data = {}
        self.dict_to_number = {}
        i = 0
        for filename in os.listdir("src/dynamics_dicts"):
            self.dict_number[i] = filename[:-5]
            self.dict_to_number[filename[:-5]] = i
            with open("src/dynamics_dicts/" + filename) as json_file: # aca fallaría si no se encuentra el archivo
                data = json.load(json_file)
                self.dict_data[i] = data
            i += 1
            
        self.route = {  # la ruta que define el primer grafico
                        'total_t': 20, # tiempo total
                        'Fs': 100, # frecuencia de sampleo (no cambia)
                        'points': [], # lista de puntos por los que pasa
                        'filters': [], # lista de filtros. Parte vacia
                        'vibrato': [], # lista de vibratos tambien parte vacia
                        'history': [] # lista de historial. Obsoleto
                     }
        self.graphicsView.setBackground('w')
        self.rule = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('m', width=2), label='t')
        self.graphicsView.addItem(self.rule)

        self.func = pg.PlotCurveItem(pen=pg.mkPen('b', width=2)) # creamos la curva
        self.func.setClickable(10) # la hacemos clickeable para interactuar más facil con ella (para agregar puntos)
        self.func.sigClicked.connect(self.onCurveClicked) # ejecutamos la funcion onCurveClicked cuando se hace click sobre la curva.  
        self.graphicsView.addItem(self.func) # la agregamos al graphicsView
        self.graphicsView.setLabel('left', 'Dynamic', units='mm') # le ponemos nombre y unidad al eje y
        self.graphicsView.getAxis('left').setTicks([self.dict_number.items()])

        self.scatter = pg.ScatterPlotItem(size=8, brush=pg.mkBrush(30, 255, 35, 255)) # este primer scatter tiene los puntos que definen las posiciones por donde pasa la curva de referencia para l.
        self.scatter.sigClicked.connect(self.onPointsClicked) # hacemos que los puntos sean clickeables para interactuar mas facil con ellos. Al hacerles click se despliega un menu que permite mover los puntos (de forma dinamica), editarlos o borrarlos 
        self.graphicsView.addItem(self.scatter)

        self.plot_data = np.array([])
        self.pointMenu = QMenu(self)
        self.movePoint = self.pointMenu.addAction("Move")
        self.editPoint = self.pointMenu.addAction("Edit")
        self.deletePoint = self.pointMenu.addAction("Delete")
        self.graphMenu = QMenu(self)
        self.addPoint = self.graphMenu.addAction("Add point")
        self.refreshDicts = self.graphMenu.addAction("Refresh dictionaries")

        self.moving_point = False
        self.segundo_click = False
        self.moving_point_index = 0

        self.graphicsView.scene().sigMouseClicked.connect(self.mouse_clicked)
        self.graphicsView.scene().sigMouseMoved.connect(self.mouse_moved)
        self.refresh_plot_signal.connect(self.refresh_plots)
        self.reset_plot_signal.connect(self.reset_plot)
        t, f, px, py = self.calculate_dynamics_route()
        self.func.setData(t, f)
        self.scatter.setData(px, py)
    
    def get_dict_for_t(self, t):
        dict_anterior = None
        dict_siguiente = None
        for i in self.route['points']:
            if i[0] < t:
                dict_anterior = self.dict_to_number[i[1]]
            if i[0] > t:
                dict_siguiente = self.dict_to_number[i[1]]
                break
        if dict_anterior is None:
            if dict_siguiente is None:
                raise Exception("No dictionary")
            else:
                return self.dict_data[dict_siguiente]
        elif dict_siguiente is None:
            return self.dict_data[dict_anterior]
        elif dict_anterior == dict_siguiente:
            return self.dict_data[dict_anterior]
        else:
            dict_interpolado = {}
            t_line, f, px, py = self.calculate_dynamics_route()
            val = np.interp(t, t_line, f)
            ponderador = 1 - (val - dict_anterior) / (dict_siguiente - dict_anterior)
            for note in self.dict_data[dict_anterior].keys():
                dict_interpolado[note] = {
                    "l": self.dict_data[dict_anterior][note]["l"]*ponderador + self.dict_data[dict_siguiente][note]["l"]*(1-ponderador),
                    "theta": self.dict_data[dict_anterior][note]["theta"]*ponderador + self.dict_data[dict_siguiente][note]["theta"]*(1-ponderador),
                    "offset": self.dict_data[dict_anterior][note]["offset"]*ponderador + self.dict_data[dict_siguiente][note]["offset"]*(1-ponderador),
                    "flow": self.dict_data[dict_anterior][note]["flow"]*ponderador + self.dict_data[dict_siguiente][note]["flow"]*(1-ponderador),
                    "lips": int(round(self.dict_data[dict_anterior][note]["lips"]*ponderador + self.dict_data[dict_siguiente][note]["lips"]*(1-ponderador), 0))
                }
            return dict_interpolado

    def reset_plot(self):
        self.dict_number = {}
        self.dict_data = {}
        self.dict_to_number = {}
        i = 0
        for filename in os.listdir("src/dynamics_dicts"):
            self.dict_number[i] = filename[:-5]
            self.dict_to_number[filename[:-5]] = i
            with open("src/dynamics_dicts/" + filename) as json_file: # aca fallaría si no se encuentra el archivo
                data = json.load(json_file)
                self.dict_data[i] = data
            i += 1
        self.graphicsView.getAxis('left').setTicks([self.dict_number.items()])
        self.reprint_plot()

    def refresh(self):
        self.refresh_plot_signal.emit()

    def refresh_plots(self):
        self.reprint_plot()

    def reprint_plot(self):
        t, f, px, py = self.calculate_dynamics_route()
        self.func.setData(t, f)
        self.scatter.setData(px, py)

    def calculate_dynamics_route(self):
        t_total = self.route['total_t']
        Fs = self.route['Fs']
        points = self.route['points']
        t = np.linspace(0, t_total, int(round(t_total*Fs, 0)))
        point_number = []
        x_points = []
        y_points = []
        for i in points:
            x_points.append(i[0])
            y_points.append(self.dict_to_number[i[1]])
            p = [i[0], self.dict_to_number[i[1]]]
            point_number.append(p)
        f = get_route_ramped(point_number, t_max=self.route["total_t"], Fs=self.route["Fs"])
        # f = np.zeros([int(round(t_total*Fs, 0))])
        
        # x_points = []
        # y_points = []
        # if len(points): # si hay alguna nota, partimos de esta nota (la sumamos como offset)
        #     alt_i = self.dict_to_number[points[0][1]]
        #     f += alt_i
        # for n in points: # ahora nos encargamos de pasar por los puntos haciendo el ZO
        #     x_points.append(n[0])
        #     y_points.append(self.dict_to_number[n[1]])
        #     f += np.heaviside(t - n[0], 1) * (self.dict_to_number[n[1]] - alt_i) # sumamos los escalones
        #     alt_i = self.dict_to_number[n[1]]
        
        return t, f, x_points, y_points

    def onCurveClicked(self, obj, event):
        if not self.moving_point:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.graphMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.addPoint:
                y = int(min(max(0,round(y, 0)), len(list(self.dict_number.items()))-1))
                self.route['points'].append([x, self.dict_number[y]])
                self.route['points'].sort(key=lambda x: x[0])
                self.route['history'].append(['add_point', [x, self.dict_number[y]]])
                self.refresh_plot_signal.emit()
                self.moving_point = True # activamos este bool para indicar que queremos que se mueva el punto
                self.moving_point_index = self.find_closest_point(x, y) 
            elif action == self.refreshDicts:
                self.reset_plot_signal.emit()
        else:
            self.moving_point = False
            self.segundo_click = False

    def onPointsClicked(self, obj, point, event):
        if not self.moving_point:
            x = event.pos()[0]
            y = event.pos()[1]
            x_screen = int(event.screenPos()[0])
            y_screen = int(event.screenPos()[1])
            action = self.pointMenu.exec_(QtCore.QPoint(x_screen,y_screen))
            if action == self.movePoint:
                self.moving_point = True
                self.moving_point_index = self.find_closest_point(x, y)
            elif action == self.editPoint:
                index = self.find_closest_point(x, y)
                data = self.route['points'][index]
                data[1] = self.dict_to_number[data[1]]
                dlg = DynamicForm(parent=self, dynamic_list=list(self.dict_number.values()), data=data, max_t=self.route['total_t'])
                dlg.setWindowTitle("Edit Point")
                if dlg.exec():
                    data[1] = self.dict_number[data[1]]
                    self.route["points"][index] = data
                    self.refresh_plot_signal.emit()
                    if self.parent:
                        self.parent.changes_made()
            elif action == self.deletePoint:
                p = self.route['points'][self.find_closest_point(x, y)]
                self.route['points'].remove(p)
                self.route['history'].append(['delete_point', p])
                self.refresh_plot_signal.emit()
                if self.parent:
                    self.parent.changes_made()
        else:
            self.moving_point = False
            self.segundo_click = False

    def mouse_clicked(self, event):
        if self.moving_point and self.segundo_click:
            self.moving_point = False
            self.segundo_click = False
            if self.parent:
                self.parent.changes_made()
        else:
            if event.double():
                vb = self.graphicsView.plotItem.vb # lo necesitamos para conocer la posicion del mouse
                mouse_point = vb.mapSceneToView(event.scenePos()) # posicion del mouse
                x = max(0, min(round(mouse_point.x(), 2), self.route["total_t"]))
                y = int(min(max(0,round(mouse_point.y(), 0)), len(list(self.dict_number.items()))-1))
                self.route['points'].append([x, self.dict_number[y]]) # agregamos el punto
                self.route['points'].sort(key=lambda x: x[0])
                self.route['history'].append(['add_point', [x, y]])
                self.refresh_plot_signal.emit() # actualizamos el grafico
                if self.parent:
                    self.parent.changes_made()

    def mouse_moved(self, event):
        if self.moving_point:
            self.segundo_click = True 
            vb = self.graphicsView.plotItem.vb # lo necesitamos para interpretar la posicion en la que se encuentra el mouse
            mouse_point = vb.mapSceneToView(event) # posicion del mouse
            x = round(mouse_point.x(), 2)
            y = int(min(max(0,round(mouse_point.y(), 0)), len(list(self.dict_number.items()))-1))
            min_x, max_x = self.find_moving_point_limits()
            if x > min_x and x < max_x: # si esta dentro del rango posible actualizamos la posicion del punto
                self.route['points'][self.moving_point_index] = [x, self.dict_number[y]]
                self.refresh_plot_signal.emit()

    def find_moving_point_limits(self):
        lim_inf = 0
        lim_sup = self.route["total_t"]
        if self.moving_point_index > 0:
            lim_inf = self.route["points"][self.moving_point_index - 1][0] # no es posible moverlo antes del punto anterior
        if self.moving_point_index < len(self.route["points"]) - 1:
            lim_sup = self.route["points"][self.moving_point_index + 1][0] # no es posible moverlo despues del punto que le sigue
        return lim_inf, lim_sup

    def find_closest_point(self, x, y):
        if len(self.route["points"]) == 0:
            return -1
        else:
            closest = 0
            dist = abs(self.route["points"][0][0] - x) + abs(self.dict_to_number[self.route["points"][0][1]] - y)
            for i in range(len(self.route["points"])):
                new_dist = abs(self.route["points"][i][0] - x) + abs(self.dict_to_number[self.route["points"][i][1]] - y)
                if new_dist < dist:
                    closest = i
                    dist = new_dist
            return closest

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = InteractivePlotWidget(app)
    win.show()

    sys.exit(app.exec())