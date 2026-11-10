import sys
import os
import atexit
from time import time, perf_counter, sleep

if __name__ == '__main__':
    import json
    from PyQt5.QtWidgets import QApplication
    from multiprocessing import Process, Event, Value, Pipe, Manager
    from src.GUI import StartUpWindow, Window
    from src.musician import Musician

    app = QApplication(sys.argv)

    ## Cargar tema visual
    theme_path = os.path.join(os.path.dirname(__file__), "src", "theme.qss")
    if os.path.exists(theme_path):
        with open(theme_path, "r") as f:
            app.setStyleSheet(f.read())
    ## Leer configuración de red desde settings.json
    settings_path = os.path.join(os.path.dirname(__file__), "src", "settings.json")
    with open(settings_path, "r") as f:
        settings = json.load(f)
    net = settings.get("network", {})
    host = net.get("host", "192.168.2.10")
    connections = [
        net.get("x_driver", "192.168.2.102"),
        net.get("z_driver", "192.168.2.104"),
        net.get("alpha_driver", "192.168.2.103"),
        net.get("flow_controller", "192.168.2.101"),
        net.get("pressure_sensor", "192.168.2.100"),
        net.get("fingers_driver", "192.168.2.105"),
    ]

    ## Primero abrimos una ventana de carga que se va a ir actualizando a medida que se inician los distintos procesos
    pipe2pierre, pierre_pipe = Pipe()
    s = StartUpWindow(app, pipe2pierre)
    s.location_on_the_screen()
    s.show()
    event = Event()
    event.set()

    mgr = Manager() 
    data = mgr.dict() # para compartir memoria entre los procesos

    t0 = perf_counter()
    connect = True
    ## Creamos el objeto Musico, que conecta a todos los dispositivos
    pierre = Musician(host, connections, event, pierre_pipe, data, fingers_connect=connect, x_connect=connect, z_connect=connect, alpha_connect=connect, flow_connect=connect, pressure_sensor_connect=connect, mic_connect=connect, home=True)
    pierre.start()

    def cleanup():
        """Asegurar que todos los procesos hijos terminen al cerrar."""
        event.clear()
        pierre.join(timeout=3)
        if pierre.is_alive():
            pierre.terminate()
            pierre.join(timeout=2)

    atexit.register(cleanup)

    import logging
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s')
    logger = logging.getLogger('pierre.main')

    ## Antes de abrir la ventana principal esperamos que se hayan inicializado todos los procesos asi como objetos compartidos entre ellos
    logger.info("Esperando que todos los dispositivos inicien (wait_loading)...")
    s.wait_loading()
    logger.info("wait_loading completado. Creando ventana principal...")

    ## Entonces se abre la ventana principal
    try:
        win = Window(app, event, pipe2pierre, data, connected=connect)
        logger.info("Window creada exitosamente. Mostrando...")
        win.show()
        logger.info("Window visible. Iniciando event loop (app.exec)...")
    except Exception as e:
        logging.critical(f"Error creando ventana principal: {e}", exc_info=True)
        event.clear()
        sys.exit(1)

    ret = app.exec()
    logger.info(f"app.exec() retorno con codigo {ret}")
    sys.exit(ret)