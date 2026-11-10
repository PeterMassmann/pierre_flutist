import json
import logging

try:
    from PyQt5 import QtCore
except ImportError:
    QtCore = None

logger = logging.getLogger("pierre.helmet_button")

try:
    import serial
except ImportError:
    serial = None


DEFAULT_HELMET_BUTTON_CONFIG = {
    "enabled": True,
    "auto_start": True,
    "port": "COM6",
    "baudrate": 115200,
    "timeout_s": 0.2,
}


def merged_helmet_button_config(config=None):
    merged = DEFAULT_HELMET_BUTTON_CONFIG.copy()
    if config:
        merged.update(config)
    return merged


def parse_helmet_button_line(line):
    if isinstance(line, bytes):
        line = line.decode("utf-8", errors="ignore")

    line = str(line).strip()
    if not line:
        return None

    payload = json.loads(line)
    if "button" not in payload:
        return None

    button = payload["button"]
    if not isinstance(button, bool):
        raise ValueError("Campo 'button' debe ser booleano")

    return button


if QtCore is None:
    class HelmetButtonReader:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("PyQt5 no esta instalado")
else:
    class HelmetButtonReader(QtCore.QThread):
        buttonChanged = QtCore.pyqtSignal(bool)
        buttonPressed = QtCore.pyqtSignal()
        statusChanged = QtCore.pyqtSignal(str)
        errorOccurred = QtCore.pyqtSignal(str)

        def __init__(self, port, baudrate=115200, timeout_s=0.2, parent=None):
            super().__init__(parent)
            self.port = port
            self.baudrate = int(baudrate)
            self.timeout_s = float(timeout_s)
            self._running = True

        def stop(self):
            self._running = False
            self.requestInterruption()

        def run(self):
            if serial is None:
                self.errorOccurred.emit("pyserial no esta instalado")
                return

            last_state = None

            try:
                with serial.Serial(self.port, self.baudrate, timeout=self.timeout_s) as bt:
                    self.statusChanged.emit(f"Casco BT conectado en {self.port}")

                    while self._running and not self.isInterruptionRequested():
                        line = bt.readline()
                        if not line:
                            continue

                        try:
                            button_state = parse_helmet_button_line(line)
                        except (json.JSONDecodeError, ValueError) as exc:
                            logger.warning("Mensaje invalido desde casco BT: %r (%s)", line, exc)
                            continue

                        if button_state is None or button_state == last_state:
                            continue

                        self.buttonChanged.emit(button_state)
                        if button_state and last_state is not True:
                            self.buttonPressed.emit()
                        last_state = button_state

            except Exception as exc:
                self.errorOccurred.emit(f"Casco BT desconectado: {exc}")
