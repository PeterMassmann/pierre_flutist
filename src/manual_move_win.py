from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import (QApplication, QMainWindow, QFileDialog, QWidget,
                             QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
                             QLabel, QPushButton, QDoubleSpinBox, QSlider,
                             QTabWidget, QComboBox, QCheckBox, QTableWidget,
                             QTableWidgetItem, QHeaderView, QAbstractSpinBox,
                             QSizePolicy, QFrame, QMenu, QMenuBar, QAction)
from PyQt5.QtCore import Qt, QTimer
import sys
import json
from src.route import dict_notes, dict_notes_rev
from src.cinematica import *
from src.motor_route import *
from time import time
import numpy as np

with open('src/look_up_table.json', 'r') as f:
    LOOK_UP_TABLE = json.load(f)


class AxisControl(QWidget):
    """Widget compuesto para controlar un eje: label + jog buttons + slider + spinbox."""
    commandReady = QtCore.pyqtSignal(float)  # emite el valor final deseado

    def __init__(self, name, unit, min_val, max_val, step_fine, step_coarse,
                 decimals=1, parent=None):
        super().__init__(parent)
        self.name = name
        self.step_fine = step_fine
        self.step_coarse = step_coarse
        self._block = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # Fila 1: label
        header = QLabel(f"{name} [{unit}]")
        header.setStyleSheet("font-weight: bold; font-size: 11px;")
        layout.addWidget(header)

        # Fila 2: jog buttons + slider
        row = QHBoxLayout()
        row.setSpacing(3)

        self.btn_minus_coarse = QPushButton(f"-{step_coarse}")
        self.btn_minus_fine = QPushButton(f"-{step_fine}")
        self.btn_plus_fine = QPushButton(f"+{step_fine}")
        self.btn_plus_coarse = QPushButton(f"+{step_coarse}")

        for btn in [self.btn_minus_coarse, self.btn_minus_fine,
                    self.btn_plus_fine, self.btn_plus_coarse]:
            btn.setFixedWidth(42)
            btn.setFixedHeight(26)
            btn.setStyleSheet("font-size: 10px; padding: 2px;")

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(min_val * 10 ** decimals))
        self.slider.setMaximum(int(max_val * 10 ** decimals))
        self._decimals = decimals

        row.addWidget(self.btn_minus_coarse)
        row.addWidget(self.btn_minus_fine)
        row.addWidget(self.slider, 1)
        row.addWidget(self.btn_plus_fine)
        row.addWidget(self.btn_plus_coarse)
        layout.addLayout(row)

        # Fila 3: spinbox centrado
        spin_row = QHBoxLayout()
        spin_row.addStretch()
        self.spinbox = QDoubleSpinBox()
        self.spinbox.setMinimum(min_val)
        self.spinbox.setMaximum(max_val)
        self.spinbox.setDecimals(decimals)
        self.spinbox.setSingleStep(step_fine)
        self.spinbox.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spinbox.setFixedWidth(100)
        self.spinbox.setAlignment(Qt.AlignCenter)
        spin_row.addWidget(self.spinbox)
        spin_row.addStretch()
        layout.addLayout(spin_row)

        # Conexiones — solo editingFinished y sliderReleased
        self.spinbox.editingFinished.connect(self._on_spinbox_done)
        self.slider.sliderReleased.connect(self._on_slider_released)

        self.btn_minus_coarse.clicked.connect(lambda: self._jog(-step_coarse))
        self.btn_minus_fine.clicked.connect(lambda: self._jog(-step_fine))
        self.btn_plus_fine.clicked.connect(lambda: self._jog(step_fine))
        self.btn_plus_coarse.clicked.connect(lambda: self._jog(step_coarse))

    def setValue(self, val):
        """Actualiza el valor mostrado sin disparar señales."""
        self._block = True
        self.spinbox.setValue(val)
        self.slider.blockSignals(True)
        self.slider.setValue(int(val * 10 ** self._decimals))
        self.slider.blockSignals(False)
        self._block = False

    def value(self):
        return self.spinbox.value()

    def _on_spinbox_done(self):
        if not self._block:
            val = self.spinbox.value()
            self.slider.blockSignals(True)
            self.slider.setValue(int(val * 10 ** self._decimals))
            self.slider.blockSignals(False)
            self.commandReady.emit(val)

    def _on_slider_released(self):
        if not self._block:
            val = self.slider.value() / (10 ** self._decimals)
            self.spinbox.blockSignals(True)
            self.spinbox.setValue(val)
            self.spinbox.blockSignals(False)
            self.commandReady.emit(val)

    def _jog(self, delta):
        new_val = self.spinbox.value() + delta
        new_val = max(self.spinbox.minimum(), min(self.spinbox.maximum(), new_val))
        self.setValue(new_val)
        self.commandReady.emit(new_val)


class CollapsibleSection(QWidget):
    """Sección colapsable con flecha clickeable."""
    def __init__(self, title, widget, parent=None, collapsed=True):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.toggle_btn = QPushButton(f"{'▶' if collapsed else '▼'} {title}")
        self.toggle_btn.setStyleSheet(
            "text-align: left; font-weight: bold; border: none; padding: 6px; font-size: 11px;")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setChecked(not collapsed)
        self.toggle_btn.clicked.connect(self.toggle)
        self._title = title
        self.content = widget
        layout.addWidget(self.toggle_btn)
        layout.addWidget(self.content)
        self.content.setVisible(not collapsed)

    def toggle(self):
        visible = self.toggle_btn.isChecked()
        self.content.setVisible(visible)
        arrow = "▼" if visible else "▶"
        self.toggle_btn.setText(f"{arrow} {self._title}")


class ManualWindow(QMainWindow):
    def __init__(self, app, musician_pipe, data, parent=None, connected=False):
        super().__init__(parent)
        self.setWindowTitle("Control Manual - Pierre")
        self.resize(820, 700)
        self.app = app
        self.parent_win = parent
        self.musician_pipe = musician_pipe
        self.data = data
        self.speed = 50
        self.moving_with_notes = False
        self.changing_other = False
        self.desired_state = State(0, 0, 0, 0)
        self.desired_state.x = self.data['x'][-1]
        self.desired_state.z = self.data['z'][-1]
        self.desired_state.alpha = self.data['alpha'][-1]

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(6)

        # ============ READBACK DE POSICIÓN ACTUAL ============
        readback_group = QGroupBox("Posición Actual")
        readback_group.setStyleSheet(
            "QGroupBox { background: #E8E8E8; font-weight: bold; }"
            "QLabel { font-family: Consolas, monospace; font-size: 12px; }")
        rg = QGridLayout(readback_group)
        self.rb_x = QLabel("X: --- mm")
        self.rb_z = QLabel("Z: --- mm")
        self.rb_alpha = QLabel("Alpha: --- °")
        self.rb_l = QLabel("L: --- mm")
        self.rb_theta = QLabel("Theta: --- °")
        self.rb_offset = QLabel("Offset: --- mm")
        rg.addWidget(self.rb_x, 0, 0)
        rg.addWidget(self.rb_z, 0, 1)
        rg.addWidget(self.rb_alpha, 0, 2)
        rg.addWidget(self.rb_l, 1, 0)
        rg.addWidget(self.rb_theta, 1, 1)
        rg.addWidget(self.rb_offset, 1, 2)
        main_layout.addWidget(readback_group)

        # ============ DOS COLUMNAS PRINCIPALES ============
        columns = QHBoxLayout()
        columns.setSpacing(8)

        # --- COLUMNA IZQUIERDA: ejes ---
        left_col = QVBoxLayout()
        left_col.setSpacing(4)

        self.tabs = QTabWidget()

        # Tab Joint Space
        joint_tab = QWidget()
        jl = QVBoxLayout(joint_tab)
        jl.setSpacing(8)
        self.ax_x = AxisControl("X", "mm", 0, 200, 0.1, 1.0)
        self.ax_z = AxisControl("Z", "mm", 0, 200, 0.1, 1.0)
        self.ax_alpha = AxisControl("Alpha", "°", -45, 45, 0.5, 5.0)
        jl.addWidget(self.ax_x)
        jl.addWidget(self.ax_z)
        jl.addWidget(self.ax_alpha)
        jl.addStretch()
        self.tabs.addTab(joint_tab, "Joint Space")

        # Tab Task Space
        task_tab = QWidget()
        tl = QVBoxLayout(task_tab)
        tl.setSpacing(8)
        self.ax_l = AxisControl("L", "mm", 0, 300, 0.1, 1.0)
        self.ax_theta = AxisControl("Theta", "°", 0, 90, 0.5, 5.0)
        self.ax_offset = AxisControl("Offset", "mm", -100, 100, 0.1, 1.0)
        tl.addWidget(self.ax_l)
        tl.addWidget(self.ax_theta)
        tl.addWidget(self.ax_offset)
        tl.addStretch()
        self.tabs.addTab(task_tab, "Task Space")

        # Tab Pivot
        pivot_tab = QWidget()
        pl = QGridLayout(pivot_tab)
        pl.addWidget(QLabel("Delta X [mm]"), 0, 0)
        self.pivotDeltaX = QDoubleSpinBox()
        self.pivotDeltaX.setMinimum(-100)
        self.pivotDeltaX.setMaximum(100)
        pl.addWidget(self.pivotDeltaX, 0, 1)
        pl.addWidget(QLabel("Delta Z [mm]"), 0, 2)
        self.pivotDeltaZ = QDoubleSpinBox()
        self.pivotDeltaZ.setMinimum(-100)
        self.pivotDeltaZ.setMaximum(100)
        pl.addWidget(self.pivotDeltaZ, 0, 3)
        pl.addWidget(QLabel("Referencia"), 1, 0)
        self.pivotReference = QComboBox()
        self.pivotReference.addItems(["Mouth end", "Flute labium", "Absolute zero"])
        pl.addWidget(self.pivotReference, 1, 1, 1, 3)
        pl.addWidget(QLabel("Ángulo [°]"), 2, 0)
        self.pivotAngle = QDoubleSpinBox()
        self.pivotAngle.setMinimum(-45)
        self.pivotAngle.setMaximum(45)
        self.pivotAngle.setSingleStep(0.5)
        pl.addWidget(self.pivotAngle, 2, 1)
        self.pivotGoButton = QPushButton("EJECUTAR PIVOT")
        self.pivotGoButton.setStyleSheet("font-weight: bold;")
        pl.addWidget(self.pivotGoButton, 2, 2, 1, 2)
        pl.setRowStretch(3, 1)
        self.tabs.addTab(pivot_tab, "Pivot")

        left_col.addWidget(self.tabs, 1)

        # Speed
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed:"))
        self.speedSlider1 = QSlider(Qt.Horizontal)
        self.speedSlider1.setMinimum(1)
        self.speedSlider1.setMaximum(100)
        self.speedSlider1.setValue(50)
        speed_row.addWidget(self.speedSlider1, 1)
        self.speed_label = QLabel("50%")
        self.speed_label.setFixedWidth(35)
        speed_row.addWidget(self.speed_label)
        left_col.addLayout(speed_row)

        columns.addLayout(left_col, 3)

        # --- COLUMNA DERECHA: aire, boca, dedos ---
        right_col = QVBoxLayout()
        right_col.setSpacing(6)

        # Flow
        flow_group = QGroupBox("Aire")
        fg = QGridLayout(flow_group)
        fg.addWidget(QLabel("Flow [SLPM]"), 0, 0)
        self.flowSpinBox = QDoubleSpinBox()
        self.flowSpinBox.setMaximum(50.0)
        self.flowSpinBox.setSingleStep(0.5)
        self.flowSpinBox.setDecimals(1)
        fg.addWidget(self.flowSpinBox, 0, 1)
        fg.addWidget(QLabel("Vib Amp [SLPM]"), 1, 0)
        self.ampSpinBox = QDoubleSpinBox()
        self.ampSpinBox.setMaximum(20.0)
        self.ampSpinBox.setSingleStep(0.5)
        self.ampSpinBox.setDecimals(1)
        fg.addWidget(self.ampSpinBox, 1, 1)
        fg.addWidget(QLabel("Vib Freq [Hz]"), 2, 0)
        self.freqSpinBox = QDoubleSpinBox()
        self.freqSpinBox.setMaximum(50.0)
        self.freqSpinBox.setSingleStep(0.5)
        self.freqSpinBox.setDecimals(1)
        fg.addWidget(self.freqSpinBox, 2, 1)
        right_col.addWidget(flow_group)

        # Mouth
        mouth_group = QGroupBox("Boca")
        mg = QGridLayout(mouth_group)
        mg.addWidget(QLabel("Lips"), 0, 0)
        self.lips_surface_slider = QSlider(Qt.Horizontal)
        self.lips_surface_slider.setMaximum(31)
        self.lips_surface_slider.setValue(31)
        mg.addWidget(self.lips_surface_slider, 0, 1)
        self.surface_label = QLabel("31")
        self.surface_label.setFixedWidth(25)
        self.surface_label.setAlignment(Qt.AlignCenter)
        mg.addWidget(self.surface_label, 0, 2)
        mg.addWidget(QLabel("Tongue"), 1, 0)
        self.tongue_slider = QSlider(Qt.Horizontal)
        self.tongue_slider.setMaximum(3)
        mg.addWidget(self.tongue_slider, 1, 1)
        self.tongue_label = QLabel("0")
        self.tongue_label.setFixedWidth(25)
        self.tongue_label.setAlignment(Qt.AlignCenter)
        mg.addWidget(self.tongue_label, 1, 2)
        right_col.addWidget(mouth_group)

        # Fingers
        fingers_group = QGroupBox("Dedos")
        ffg = QGridLayout(fingers_group)
        ffg.addWidget(QLabel("Nota"), 0, 0)
        self.noteComboBox = QComboBox()
        self.noteComboBox.addItems(list(dict_notes.values()))
        ffg.addWidget(self.noteComboBox, 0, 1)
        self.moveToNoteCheckBox = QCheckBox("Mover al cambiar nota")
        ffg.addWidget(self.moveToNoteCheckBox, 1, 0, 1, 2)
        self.setNoteDictionary = QPushButton("Set nota en diccionario")
        ffg.addWidget(self.setNoteDictionary, 2, 0, 1, 2)
        right_col.addWidget(fingers_group)

        # Status
        status_group = QGroupBox("Estado")
        sg = QHBoxLayout(status_group)
        self.status_label = QLabel("LISTO")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "background: #E8F5E9; color: #2E7D32; font-weight: bold; "
            "padding: 6px; border-radius: 4px;")
        sg.addWidget(self.status_label)
        right_col.addWidget(status_group)

        right_col.addStretch()
        columns.addLayout(right_col, 2)

        main_layout.addLayout(columns, 1)

        # ============ BOTÓN STOP ============
        self.stopButton = QPushButton("DETENER MOVIMIENTO")
        self.stopButton.setObjectName("manualStopButton")
        self.stopButton.setStyleSheet(
            "QPushButton { background: #D32F2F; color: white; font-weight: bold; "
            "font-size: 14px; padding: 10px; border-radius: 4px; }"
            "QPushButton:hover { background: #B71C1C; }"
            "QPushButton:pressed { background: #7F0000; }")
        self.stopButton.setFixedHeight(44)
        main_layout.addWidget(self.stopButton)

        # ============ TABLA COLAPSABLE ============
        self.tableWidget = QTableWidget()
        self.tableWidget.setColumnCount(5)
        self.tableWidget.setRowCount(len(LOOK_UP_TABLE))
        labels = list(LOOK_UP_TABLE.keys())
        self.tableWidget.setVerticalHeaderLabels(labels)
        self.tableWidget.setHorizontalHeaderLabels(
            ["L [mm]", "Theta [°]", "Offset [mm]", "Flow [SLPM]", "Lips"])
        self.tableWidget.setAlternatingRowColors(True)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        row = 0
        for value in LOOK_UP_TABLE.values():
            self.tableWidget.setItem(row, 0, QTableWidgetItem(str(value['l'])))
            self.tableWidget.setItem(row, 1, QTableWidgetItem(str(value['theta'])))
            self.tableWidget.setItem(row, 2, QTableWidgetItem(str(value['offset'])))
            self.tableWidget.setItem(row, 3, QTableWidgetItem(str(value['flow'])))
            self.tableWidget.setItem(row, 4, QTableWidgetItem(str(value['lips'])))
            row += 1

        self.table_section = CollapsibleSection("Diccionario de Notas",
                                                 self.tableWidget, collapsed=True)
        main_layout.addWidget(self.table_section)

        # ============ MENU ============
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        self.actionSaveDictionayAs = QAction("Save Dictionary As...", self)
        file_menu.addAction(self.actionSaveDictionayAs)

        # ============ INICIALIZAR VALORES ============
        self.ax_x.setValue(self.data['x'][-1])
        self.ax_z.setValue(self.data['z'][-1])
        self.ax_alpha.setValue(self.data['alpha'][-1])
        self.ax_l.setValue(self.data['radius'][-1])
        self.ax_theta.setValue(self.data['theta'][-1])
        self.ax_offset.setValue(self.data['offset'][-1])
        self.flowSpinBox.setValue(self.data['mass_flow'][-1])

        # ============ CONECTAR SEÑALES ============
        # Ejes Joint Space
        self.ax_x.commandReady.connect(self.change_x)
        self.ax_z.commandReady.connect(self.change_z)
        self.ax_alpha.commandReady.connect(self.change_angle)

        # Ejes Task Space
        self.ax_l.commandReady.connect(self.change_r)
        self.ax_theta.commandReady.connect(self.change_theta)
        self.ax_offset.commandReady.connect(self.change_offset)

        # Flow — editingFinished en vez de valueChanged
        self.flowSpinBox.editingFinished.connect(self.change_flow)
        self.freqSpinBox.editingFinished.connect(self.change_flow_vibrato)
        self.ampSpinBox.editingFinished.connect(self.change_flow_vibrato_amp)

        # Mouth — inmediato (actuadores discretos)
        self.tongue_slider.valueChanged.connect(self.tongue_move_pressed)
        self.lips_surface_slider.valueChanged.connect(self.change_lips_surface)

        # Fingers
        self.noteComboBox.currentIndexChanged.connect(self.change_note)
        self.moveToNoteCheckBox.stateChanged.connect(self.move_to_notes_change)
        self.setNoteDictionary.clicked.connect(self.set_note_in_dictionary)

        # Speed
        self.speedSlider1.valueChanged.connect(self.change_speed)

        # Pivot
        self.pivotGoButton.clicked.connect(self.pivot)

        # Stop
        self.stopButton.clicked.connect(self.stop_movement)

        # Table edit
        self.tableWidget.itemChanged.connect(self.item_changed)

        # Menu
        self.actionSaveDictionayAs.triggered.connect(self.save_dict_as)

        # ============ TIMER DE READBACK ============
        self._readback_timer = QTimer()
        self._readback_timer.setInterval(100)
        self._readback_timer.timeout.connect(self._update_readback)
        self._readback_timer.start()

    # ===================== READBACK =====================

    def _update_readback(self):
        """Actualiza posición actual y estado cada 100ms."""
        try:
            x = self.data['x'][-1]
            z = self.data['z'][-1]
            alpha = self.data['alpha'][-1]
        except (IndexError, KeyError):
            return

        self.rb_x.setText(f"X: {x:.1f} mm")
        self.rb_z.setText(f"Z: {z:.1f} mm")
        self.rb_alpha.setText(f"Alpha: {alpha:.1f}°")

        try:
            r, theta, offset = get_l_theta_of(x, z, alpha)
            self.rb_l.setText(f"L: {r:.1f} mm")
            self.rb_theta.setText(f"Theta: {theta:.1f}°")
            self.rb_offset.setText(f"Offset: {offset:.1f} mm")
        except (ValueError, ZeroDivisionError, ArithmeticError):
            self.rb_l.setText("L: ---")
            self.rb_theta.setText("Theta: ---")
            self.rb_offset.setText("Offset: ---")

        # Estado de movimiento
        try:
            moving = (abs(x - self.desired_state.x) > 0.3 or
                      abs(z - self.desired_state.z) > 0.3 or
                      abs(alpha - self.desired_state.alpha) > 0.3)
        except Exception:
            moving = False

        if moving:
            self.status_label.setText("MOVIENDO...")
            self.status_label.setStyleSheet(
                "background: #FFF3E0; color: #E65100; font-weight: bold; "
                "padding: 6px; border-radius: 4px;")
        else:
            self.status_label.setText("LISTO")
            self.status_label.setStyleSheet(
                "background: #E8F5E9; color: #2E7D32; font-weight: bold; "
                "padding: 6px; border-radius: 4px;")

    # ===================== STOP =====================

    def stop_movement(self):
        """Para todos los ejes inmediatamente."""
        self.musician_pipe.send(["stop"])
        # Actualizar desired_state a posición actual para evitar re-movimiento
        try:
            self.desired_state.x = self.data['x'][-1]
            self.desired_state.z = self.data['z'][-1]
            self.desired_state.alpha = self.data['alpha'][-1]
            self.update_values()
        except (IndexError, KeyError):
            pass

    # ===================== MOVEMENT =====================

    def update_values(self):
        """Actualiza todos los widgets con los valores del desired_state."""
        self.ax_x.setValue(self.desired_state.x)
        self.ax_z.setValue(self.desired_state.z)
        self.ax_alpha.setValue(self.desired_state.alpha)
        self.ax_l.setValue(self.desired_state.r)
        self.ax_theta.setValue(self.desired_state.theta)
        self.ax_offset.setValue(self.desired_state.o)
        self.flowSpinBox.setValue(self.desired_state.flow)
        self.freqSpinBox.setValue(self.desired_state.vibrato_freq)
        self.ampSpinBox.setValue(self.desired_state.vibrato_amp)
        self.lips_surface_slider.blockSignals(True)
        self.lips_surface_slider.setValue(self.desired_state.lips)
        self.lips_surface_slider.blockSignals(False)
        self.tongue_slider.blockSignals(True)
        self.tongue_slider.setValue(self.desired_state.tongue)
        self.tongue_slider.blockSignals(False)

    def change_x(self, value):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.x = value
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     True, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_z(self, value):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.z = value
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, True, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_angle(self, value):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.alpha = value
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, False, True, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_r(self, value):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.r = value
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_theta(self, value):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.theta = value
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_offset(self, value):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.o = value
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def change_flow(self):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.flow = self.flowSpinBox.value()
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, False, False, True, self.speed])
            self.changing_other = False

    def change_flow_vibrato(self):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.vibrato_freq = self.freqSpinBox.value()
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, False, False, True, self.speed])
            self.changing_other = False

    def change_flow_vibrato_amp(self):
        if not self.changing_other:
            self.changing_other = True
            self.desired_state.vibrato_amp = self.ampSpinBox.value()
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, False, False, True, self.speed])
            self.changing_other = False

    # ===================== MOUTH =====================

    def tongue_move_pressed(self, value):
        self.musician_pipe.send(['execute_tongue_action', value])
        self.tongue_label.setText(str(value))

    def change_lips_surface(self, value):
        self.musician_pipe.send(['execute_lips_surface_action', value])
        self.surface_label.setText(str(value))

    # ===================== SPEED =====================

    def change_speed(self, value):
        self.speed = value
        self.speed_label.setText(f"{value}%")

    # ===================== FINGERS =====================

    def change_note(self, value):
        self.musician_pipe.send(['execute_fingers_action',
                                 dict_notes_rev[self.noteComboBox.itemText(value)], False])
        if self.moving_with_notes:
            self.changing_other = True
            pos = LOOK_UP_TABLE[self.noteComboBox.itemText(value)]
            self.desired_state.r = pos['l']
            self.desired_state.theta = pos['theta']
            self.desired_state.o = pos['offset']
            self.desired_state.flow = pos['flow']
            self.desired_state.lips = pos['lips']
            self.desired_state.vibrato_amp = 0
            self.desired_state.vibrato_freq = 0
            self.musician_pipe.send(["move_to", self.desired_state, None,
                                     False, False, False, False, self.speed])
            self.update_values()
            self.changing_other = False

    def move_to_notes_change(self, value):
        self.moving_with_notes = value

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
        self.tableWidget.setItem(row, 0, QTableWidgetItem(str(l)))
        self.tableWidget.setItem(row, 1, QTableWidgetItem(str(theta)))
        self.tableWidget.setItem(row, 2, QTableWidgetItem(str(offset)))
        self.tableWidget.setItem(row, 3, QTableWidgetItem(str(flow)))
        self.tableWidget.setItem(row, 4, QTableWidgetItem(str(lips)))

    # ===================== PIVOT =====================

    def rotate(self, x1, z1, xr, zr, alpha):
        xf = (x1 - xr) * np.cos(alpha) - (z1 - zr) * np.sin(alpha) + xr
        zf = (x1 - xr) * np.sin(alpha) + (z1 - zr) * np.cos(alpha) + zr
        return xf, zf

    def get_pivot_route(self, reference, delta_x, delta_z, angle, speed):
        global DATA
        x_i = self.data['x'][-1]
        z_i = self.data['z'][-1]
        alpha_i = self.data['alpha'][-1]
        if reference == 0:
            x_mouth, z_mouth = get_pos_punta(x_i, z_i, alpha_i * np.pi / 180)
            x_pivot = x_mouth + delta_x
            z_pivot = z_mouth + delta_z
        elif reference == 1:
            x_pivot = DATA['flute_position']['X_F'] + delta_x
            z_pivot = DATA['flute_position']['Z_F'] + delta_z
        elif reference == 2:
            x_pivot = delta_x
            z_pivot = delta_z

        route = []
        for i in range(101):
            x_f, z_f = self.rotate(x_i, z_i, x_pivot, z_pivot,
                                   i * angle * np.pi / 180 / 100)
            alpha_f = alpha_i + i * angle / 100
            x_f = x_mm_to_units(x_f)
            z_f = z_mm_to_units(z_f)
            alpha_f = alpha_angle_to_units(alpha_f)
            route.append([x_f, z_f, alpha_f])

        T = abs(angle) * 15 / speed
        route = time_scaled_straight_line(route, T)

        self.musician_pipe.send(["pivot", route['x'], route['z'], route['alpha']])
        self.changing_other = True
        self.desired_state.x = x_units_to_mm(route['x'][-1][1])
        self.desired_state.z = z_units_to_mm(route['z'][-1][1])
        self.desired_state.alpha = alpha_units_to_angle(route['alpha'][-1][1])
        self.update_values()
        self.changing_other = False

    def pivot(self):
        delta_x = self.pivotDeltaX.value()
        delta_z = self.pivotDeltaZ.value()
        reference = self.pivotReference.currentIndex()
        angle = self.pivotAngle.value()
        self.get_pivot_route(reference, delta_x, delta_z, angle, self.speed)

    # ===================== TABLE =====================

    def item_changed(self, item):
        new_value = item.text()
        row = item.row()
        col = item.column()
        key = self.tableWidget.verticalHeaderItem(row).text()
        key2 = self.tableWidget.horizontalHeaderItem(col).text().split(" ")[0]

        try:
            new_value = float(new_value)
            if key2 == "Lips":
                key2 = "lips"
                new_value = int(min(max(new_value, 0), 31))
                item.setText(str(new_value))
            elif key2 == "L":
                key2 = "l"
            elif key2 == "Theta":
                key2 = "theta"
            elif key2 == "Offset":
                key2 = "offset"
            elif key2 == "Flow":
                key2 = "flow"
        except ValueError:
            item.setText(str(LOOK_UP_TABLE[key].get(key2.lower(), "")))
            return

        LOOK_UP_TABLE[key][key2] = new_value
        with open('src/look_up_table.json', 'w') as file:
            json.dump(LOOK_UP_TABLE, file, indent=4, sort_keys=False)

    # ===================== SAVE =====================

    def save_dict_as(self):
        base = self.parent_win.base_path if self.parent_win else ""
        fname, _ = QFileDialog.getSaveFileName(self, 'Save file', base,
                                               "JSON files (*.json)")
        if fname:
            if not fname.endswith('.json'):
                fname += '.json'
            with open(fname, 'w') as file:
                json.dump(LOOK_UP_TABLE, file, indent=4, sort_keys=False)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    # Solo para preview, sin conexión real
    print("ManualWindow requires musician_pipe and data to run.")
    sys.exit(1)
