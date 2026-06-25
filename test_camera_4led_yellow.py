import json
from pathlib import Path

from src.led_tracker import LEDPoseTracker, merged_led_tracker_config


SETTINGS_PATH = Path(__file__).resolve().parent / "src" / "settings.json"

# Ajusta valores aqui para probar la deteccion sin tocar settings.json.
# Estos overrides se aplican solo a este script y nunca mandan comandos al robot.
TEST_CONFIG_OVERRIDES = {
    "show_windows": True,
}


def load_test_config():
    config = {}
    if SETTINGS_PATH.exists():
        with SETTINGS_PATH.open("r", encoding="utf-8") as settings_file:
            settings = json.load(settings_file)
        config.update(settings.get("led_control", {}))

    config.update(TEST_CONFIG_OVERRIDES)
    return merged_led_tracker_config(config)


def main():
    config = load_test_config()
    alpha_sign = float(config.get("tilt_to_alpha_sign", 1.0))
    tracker = LEDPoseTracker(config)
    tracker.open()
    try:
        while True:
            measurement, display, plot_image, _now, error = tracker.read()
            if error:
                break
            if measurement is not None:
                alpha_delta = measurement["tilt_deg"] * alpha_sign
                print(
                    f"camera_x={measurement['camera_x_depth']:.3f}, "
                    f"camera_z={measurement['camera_z_vertical']:.3f}, "
                    f"tilt={measurement['tilt_deg']:.3f}, "
                    f"alpha_delta={alpha_delta:.3f}"
                )
            key = tracker.show(display, plot_image)
            if key == 27 or key == ord("q"):
                break
    finally:
        tracker.close()


if __name__ == "__main__":
    main()
