import sys
import os
from time import time, perf_counter, sleep

if __name__ == '__main__':
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

    ## Primero abrimos una ventana de carga que se va a ir actualizando a medida que se inician los distintos procesos
    pipe2pierre, pierre_pipe = Pipe()
    s = StartUpWindow(app, pipe2pierre)
    s.location_on_the_screen()
    s.show()

    ## 
    host = "192.168.2.10"   # dirección del pc
    connections = ["192.168.2.102", "192.168.2.104", "192.168.2.103", "192.168.2.101", "192.168.2.100", "192.168.2.105"] # direcciones de los dispositivos
    event = Event()
    event.set()

    mgr = Manager() 
    data = mgr.dict() # para compartir memoria entre los procesos

    t0 = perf_counter()
    connect = True
    ## Creamos el objeto Musico, que conecta a todos los dispositivos
    pierre = Musician(host, connections, event, pierre_pipe, data, fingers_connect=connect, x_connect=connect, z_connect=connect, alpha_connect=connect, flow_connect=connect, pressure_sensor_connect=connect, mic_connect=connect, home=True)
    pierre.start()

    ## Antes de abrir la ventana principal esperamos que se hayan inicializado todos los procesos asi como objetos compartidos entre ellos
    s.wait_loading()

    ## Entonces se abre la ventana principal
    win = Window(app, event, pipe2pierre, data, connected=connect)
    win.show()

    sys.exit(app.exec())