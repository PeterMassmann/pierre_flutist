"""Fuentes de datos de pose de la cabeza para el prototipo.

Tres fuentes intercambiables:
- OSCSource: recibe de una app en el celular (GyrOSC, SensorLogger, etc.)
- MouseSource: mueve pose con el mouse (test sin hardware)
- ScriptedSource: reproduce una secuencia pre-definida (regresión)

Todas implementan el mismo protocolo: get_pose() -> HeadPose.
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from mapping import HeadPose, euler_to_quat_zyx


class PoseSource(ABC):
    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...

    @abstractmethod
    def get_pose(self) -> HeadPose:
        ...

    def is_fresh(self, max_age_s: float = 0.2) -> bool:
        return True


@dataclass
class _PoseState:
    pose: HeadPose
    last_update_s: float


class OSCSource(PoseSource):
    """Recibe orientación por OSC desde una app de celular.

    Patrones reconocidos:
    - GyrOSC-style:     /gyrosc/quat <qw> <qx> <qy> <qz>
    - SensorOSC-style:  /sensors/orientation <roll> <pitch> <yaw>
    - Custom:           /helmet/quat <qw> <qx> <qy> <qz>
    - Custom:           /helmet/euler <yaw> <pitch> <roll>

    Posición: para H.0 asumimos posición fija (sensor de posición
    no viene del celular). Solo probamos mapeo de orientación.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8000,
                 verbose: bool = False):
        self.host = host
        self.port = port
        self.verbose = verbose
        self._state = _PoseState(HeadPose.neutral(), time.perf_counter())
        self._lock = threading.Lock()
        self._server = None
        self._thread = None

    def start(self) -> None:
        try:
            from pythonosc.dispatcher import Dispatcher
            from pythonosc.osc_server import ThreadingOSCUDPServer
        except ImportError as e:
            raise RuntimeError(
                "python-osc not installed. Run: pip install python-osc"
            ) from e

        dispatcher = Dispatcher()
        dispatcher.map("/gyrosc/quat", self._on_quat)
        dispatcher.map("/helmet/quat", self._on_quat)
        dispatcher.map("/sensors/orientation", self._on_euler_rpy)
        dispatcher.map("/helmet/euler", self._on_euler_ypr)
        dispatcher.set_default_handler(self._on_default)

        self._server = ThreadingOSCUDPServer(
            (self.host, self.port), dispatcher)
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True)
        self._thread.start()
        print(f"[OSC] Escuchando en {self.host}:{self.port}")
        print("      Envía /helmet/quat qw qx qy qz  — o /helmet/euler yaw pitch roll")

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()

    def get_pose(self) -> HeadPose:
        with self._lock:
            return HeadPose(
                position=self._state.pose.position.copy(),
                quaternion=self._state.pose.quaternion.copy(),
            )

    def is_fresh(self, max_age_s: float = 0.2) -> bool:
        with self._lock:
            return (time.perf_counter() - self._state.last_update_s) < max_age_s

    def _on_quat(self, address, *args):
        if len(args) < 4:
            return
        q = np.array([float(a) for a in args[:4]])
        q = q / np.linalg.norm(q)
        self._update(HeadPose(position=np.zeros(3), quaternion=q))
        if self.verbose:
            print(f"[OSC] quat <- {address} {q}")

    def _on_euler_ypr(self, address, *args):
        if len(args) < 3:
            return
        yaw, pitch, roll = [float(a) for a in args[:3]]
        q = euler_to_quat_zyx(yaw, pitch, roll)
        self._update(HeadPose(position=np.zeros(3), quaternion=q))

    def _on_euler_rpy(self, address, *args):
        if len(args) < 3:
            return
        roll, pitch, yaw = [float(a) for a in args[:3]]
        q = euler_to_quat_zyx(yaw, pitch, roll)
        self._update(HeadPose(position=np.zeros(3), quaternion=q))

    def _on_default(self, address, *args):
        if self.verbose:
            print(f"[OSC] (no match) {address} {args}")

    def _update(self, pose: HeadPose) -> None:
        with self._lock:
            self._state = _PoseState(pose, time.perf_counter())


class MouseSource(PoseSource):
    """Fuente de pose controlada por el mouse, para testing sin hardware.

    - Desplazamiento horizontal del mouse -> yaw
    - Desplazamiento vertical del mouse -> pitch
    - Rueda del mouse -> traslación adelante/atrás
    """

    def __init__(self):
        self._yaw = 0.0
        self._pitch = 0.0
        self._z = 0.0

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def set_from_mouse(self, norm_x: float, norm_y: float) -> None:
        self._yaw = -norm_x * np.pi / 4
        self._pitch = -norm_y * np.pi / 6

    def translate_z(self, delta: float) -> None:
        self._z = float(np.clip(self._z + delta, -0.15, 0.15))

    def get_pose(self) -> HeadPose:
        q = euler_to_quat_zyx(self._yaw, self._pitch, 0.0)
        return HeadPose(
            position=np.array([0.0, 0.0, self._z]),
            quaternion=q,
        )


class ScriptedSource(PoseSource):
    """Reproduce una secuencia pre-definida de poses en el tiempo."""

    def __init__(self, script: Callable[[float], HeadPose]):
        self._script = script
        self._t0 = None

    def start(self) -> None:
        self._t0 = time.perf_counter()

    def stop(self) -> None:
        pass

    def get_pose(self) -> HeadPose:
        if self._t0 is None:
            return HeadPose.neutral()
        return self._script(time.perf_counter() - self._t0)
