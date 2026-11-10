import json
import os

import serial

from src.helmet_button import merged_helmet_button_config, parse_helmet_button_line


SETTINGS_PATH = os.path.join(os.path.dirname(__file__), "src", "settings.json")


def load_config():
    try:
        with open(SETTINGS_PATH, "r") as file:
            settings = json.load(file)
    except (OSError, json.JSONDecodeError):
        settings = {}

    return merged_helmet_button_config(settings.get("helmet_button", {}))


def main():
    config = load_config()
    port = config["port"]
    baudrate = int(config["baudrate"])
    timeout_s = float(config["timeout_s"])

    print(f"Conectando a {port}...")

    with serial.Serial(port, baudrate, timeout=timeout_s) as bt:
        print("Conectado. Esperando mensajes...")

        last_state = None

        while True:
            line = bt.readline()
            if not line:
                continue

            try:
                button_pressed = parse_helmet_button_line(line)
            except (json.JSONDecodeError, ValueError):
                print(f"Mensaje invalido recibido: {line!r}")
                continue

            if button_pressed is None:
                print(f"JSON sin campo 'button': {line!r}")
                continue

            if button_pressed != last_state:
                last_state = button_pressed

                if button_pressed:
                    print("Boton presionado")
                else:
                    print("Boton soltado")


if __name__ == "__main__":
    main()
