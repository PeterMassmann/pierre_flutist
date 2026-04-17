"""Visualizador 3D + readouts del mapeo magnificado.

Dos vistas:
- Vista 3D superior/perspectiva: muestra la cabeza virtual, el bisel,
  el ray del chorro, y el punto de impacto.
- Panel de readouts: valores numéricos de (l, theta, offset) y estado
  de conexión.

Usa pyqtgraph para 3D (ya es dependencia de Pierre).
"""

from __future__ import annotations

import math
import sys
from typing import Callable

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtCore, QtGui, QtWidgets

from mapping import (
    HeadPose,
    MappingConfig,
    MappingResult,
    magnified_flute_mapping,
)
from sources import MouseSource, OSCSource, PoseSource


class HelmetVisualizer(QtWidgets.QMainWindow):
    def __init__(
        self,
        source: PoseSource,
        config: MappingConfig,
        zero_pose: HeadPose,
        tick_ms: int = 33,
        on_zero_request: Callable[[], None] | None = None,
    ):
        super().__init__()
        self.source = source
        self.config = config
        self.zero_pose = zero_pose
        self._on_zero_request = on_zero_request

        self.setWindowTitle("Casco Magnificado — H.0 prueba conceptual")
        self.resize(1300, 800)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        self.setCentralWidget(central)

        self._build_3d_view()
        layout.addWidget(self.view3d, 3)

        side = self._build_side_panel()
        layout.addLayout(side, 2)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(tick_ms)

    def _build_3d_view(self) -> None:
        self.view3d = gl.GLViewWidget()
        self.view3d.setBackgroundColor(pg.mkColor(30, 30, 35))
        self.view3d.setCameraPosition(distance=0.7, elevation=25, azimuth=-60)

        grid = gl.GLGridItem()
        grid.setSize(1.0, 1.0)
        grid.setSpacing(0.05, 0.05)
        grid.translate(0, 0, -0.01)
        self.view3d.addItem(grid)

        self._add_axes()

        bisel_pos = self.config.P_bisel
        self.bisel_marker = gl.GLScatterPlotItem(
            pos=np.array([bisel_pos]),
            color=(1.0, 0.4, 0.2, 1.0),
            size=18.0,
        )
        self.view3d.addItem(self.bisel_marker)

        self.bisel_plane = self._make_bisel_plane()
        self.view3d.addItem(self.bisel_plane)

        self.head_marker = gl.GLScatterPlotItem(
            pos=np.array([[0, 0, 0]]),
            color=(0.3, 0.8, 1.0, 1.0),
            size=14.0,
        )
        self.view3d.addItem(self.head_marker)

        self.mouth_marker = gl.GLScatterPlotItem(
            pos=np.array([[0, -0.05, -0.08]]),
            color=(1.0, 0.9, 0.2, 1.0),
            size=10.0,
        )
        self.view3d.addItem(self.mouth_marker)

        self.jet_ray = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, -0.4]]),
            color=(0.3, 1.0, 0.5, 0.9),
            width=3.0,
            antialias=True,
        )
        self.view3d.addItem(self.jet_ray)

        self.closest_line = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, 0]]),
            color=(1.0, 0.3, 0.3, 0.8),
            width=2.0,
            antialias=True,
        )
        self.view3d.addItem(self.closest_line)

        self.impact_marker = gl.GLScatterPlotItem(
            pos=np.array([[0, 0, -0.2]]),
            color=(1.0, 1.0, 1.0, 1.0),
            size=8.0,
        )
        self.view3d.addItem(self.impact_marker)

    def _add_axes(self) -> None:
        for vec, color in [
            (np.array([0.1, 0, 0]), (1.0, 0.3, 0.3, 0.7)),
            (np.array([0, 0.1, 0]), (0.3, 1.0, 0.3, 0.7)),
            (np.array([0, 0, -0.1]), (0.3, 0.5, 1.0, 0.7)),
        ]:
            line = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], vec]),
                color=color,
                width=2.0,
            )
            self.view3d.addItem(line)

    def _make_bisel_plane(self) -> gl.GLLinePlotItem:
        c = self.config.P_bisel
        w = 0.06
        h = 0.04
        pts = np.array([
            [c[0] - w, c[1] - h, c[2]],
            [c[0] + w, c[1] - h, c[2]],
            [c[0] + w, c[1] + h, c[2]],
            [c[0] - w, c[1] + h, c[2]],
            [c[0] - w, c[1] - h, c[2]],
        ])
        return gl.GLLinePlotItem(
            pos=pts, color=(1.0, 0.5, 0.3, 0.8), width=2.0)

    def _build_side_panel(self) -> QtWidgets.QVBoxLayout:
        side = QtWidgets.QVBoxLayout()

        title = QtWidgets.QLabel("<h2>Casco Magnificado — H.0</h2>")
        side.addWidget(title)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setStyleSheet("font-family: Menlo; font-size: 11pt;")
        side.addWidget(self.status_label)

        readout_group = QtWidgets.QGroupBox("Mapeo al robot (magnificado)")
        rl = QtWidgets.QFormLayout(readout_group)
        self.l_label = self._mk_readout("l (m)")
        self.theta_label = self._mk_readout("θ (°)")
        self.offset_label = self._mk_readout("offset (m)")
        self.miss_label = self._mk_readout("error de apuntamiento (°)")
        rl.addRow("l", self.l_label)
        rl.addRow("θ", self.theta_label)
        rl.addRow("offset", self.offset_label)
        rl.addRow("miss", self.miss_label)
        side.addWidget(readout_group)

        head_group = QtWidgets.QGroupBox("Pose de la cabeza (relativa a zero)")
        hl = QtWidgets.QFormLayout(head_group)
        self.yaw_label = self._mk_readout("yaw (°)")
        self.pitch_label = self._mk_readout("pitch (°)")
        self.roll_label = self._mk_readout("roll (°)")
        hl.addRow("yaw", self.yaw_label)
        hl.addRow("pitch", self.pitch_label)
        hl.addRow("roll", self.roll_label)
        side.addWidget(head_group)

        cfg_group = QtWidgets.QGroupBox("Configuración")
        cl = QtWidgets.QFormLayout(cfg_group)

        self.k_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.k_slider.setMinimum(30)
        self.k_slider.setMaximum(200)
        self.k_slider.setValue(int(self.config.magnification_k * 10))
        self.k_slider.valueChanged.connect(self._on_k_changed)
        self.k_value = QtWidgets.QLabel(f"{self.config.magnification_k:.1f}")
        k_row = QtWidgets.QHBoxLayout()
        k_row.addWidget(self.k_slider)
        k_row.addWidget(self.k_value)
        cl.addRow("Magnificación k", k_row)

        self.bisel_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.bisel_slider.setMinimum(10)
        self.bisel_slider.setMaximum(50)
        self.bisel_slider.setValue(int(self.config.bisel_offset_m * 100))
        self.bisel_slider.valueChanged.connect(self._on_bisel_changed)
        self.bisel_value = QtWidgets.QLabel(
            f"{self.config.bisel_offset_m * 100:.0f} cm")
        b_row = QtWidgets.QHBoxLayout()
        b_row.addWidget(self.bisel_slider)
        b_row.addWidget(self.bisel_value)
        cl.addRow("Distancia bisel", b_row)

        side.addWidget(cfg_group)

        btn_row = QtWidgets.QHBoxLayout()
        self.zero_btn = QtWidgets.QPushButton("Zero pose (sostener cabeza al frente)")
        self.zero_btn.clicked.connect(self._on_zero_clicked)
        btn_row.addWidget(self.zero_btn)
        side.addLayout(btn_row)

        tip = QtWidgets.QLabel(
            "<i>Tip:</i> Ponte el teléfono con vincha en la frente, "
            "mira al frente, pulsa <b>Zero</b>, y mueve la cabeza como "
            "si apuntaras el chorro de aire al bisel naranja."
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("color: #888;")
        side.addWidget(tip)

        side.addStretch(1)
        return side

    def _mk_readout(self, name: str) -> QtWidgets.QLabel:
        lbl = QtWidgets.QLabel("—")
        lbl.setStyleSheet("font-family: Menlo; font-size: 14pt;")
        return lbl

    def _on_k_changed(self, val: int) -> None:
        self.config.magnification_k = val / 10.0
        self.k_value.setText(f"{self.config.magnification_k:.1f}")

    def _on_bisel_changed(self, val: int) -> None:
        self.config.bisel_offset_m = val / 100.0
        self.bisel_value.setText(f"{val} cm")
        self.bisel_marker.setData(pos=np.array([self.config.P_bisel]))
        self.view3d.removeItem(self.bisel_plane)
        self.bisel_plane = self._make_bisel_plane()
        self.view3d.addItem(self.bisel_plane)

    def _on_zero_clicked(self) -> None:
        self.zero_pose = self.source.get_pose()
        if self._on_zero_request is not None:
            self._on_zero_request()
        self.status_label.setText("<span style='color:#6c6;'>Zero-pose recalibrado.</span>")

    def _on_tick(self) -> None:
        pose = self.source.get_pose()

        if hasattr(self.source, "is_fresh") and not self.source.is_fresh():
            self.status_label.setText(
                "<span style='color:#f66;'>● Sin datos OSC (revisa la app del teléfono).</span>"
            )
        else:
            self.status_label.setText(
                "<span style='color:#6c6;'>● Recibiendo pose.</span>"
            )

        result = magnified_flute_mapping(pose, self.zero_pose, self.config)

        self.head_marker.setData(pos=np.array([pose.position]))
        self.mouth_marker.setData(pos=np.array([result.P_mouth]))
        ray_end = result.P_mouth + result.v_jet * 0.5
        self.jet_ray.setData(pos=np.array([result.P_mouth, ray_end]))

        self.closest_line.setData(
            pos=np.array([result.P_closest, self.config.P_bisel]))
        self.impact_marker.setData(pos=np.array([result.P_closest]))

        self.l_label.setText(f"{result.l * 1000:.1f} mm")
        self.theta_label.setText(f"{math.degrees(result.theta):+6.2f}°")
        self.offset_label.setText(f"{result.offset * 1000:+6.1f} mm")
        self.miss_label.setText(f"{result.aiming_error_deg():5.2f}°")

        rel = pose.rotation() * self.zero_pose.rotation().inv()
        yaw, pitch, roll = rel.as_euler("YXZ", degrees=True)
        self.yaw_label.setText(f"{yaw:+6.2f}°")
        self.pitch_label.setText(f"{pitch:+6.2f}°")
        self.roll_label.setText(f"{roll:+6.2f}°")


def run_with_source(source: PoseSource) -> int:
    app = QtWidgets.QApplication(sys.argv)
    source.start()
    config = MappingConfig()
    zero = source.get_pose()
    window = HelmetVisualizer(source, config, zero_pose=zero)
    window.show()
    try:
        return app.exec_()
    finally:
        source.stop()
