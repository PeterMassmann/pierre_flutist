import logging
import math
import time
from multiprocessing import Process

from src.led_tracker import LEDPoseTracker, merged_led_tracker_config

logger = logging.getLogger("pierre.led_control")


DEFAULT_LED_CONTROL_CONFIG = {
    "enabled": True,
    "distance_zero": 30.0,
    "distance_amplification": 0.5,
    "update_interval_s": 0.08,
    "stale_timeout_s": 0.8,
    "max_camera_x_delta_mm": 15.0,
    "max_camera_z_delta_mm": 10.0,
    "max_alpha_delta_deg": 5.0,
    "max_robot_n_rate_mm_s": 10.0,
    "max_robot_perp_rate_mm_s": 10.0,
    "max_alpha_rate_deg_s": 10.0,
    "translation_deadband_mm": 0.1,
    "alpha_deadband_deg": 0.1,
    "tilt_to_alpha_sign": -1.0,
}


def merged_led_control_config(config=None):
    merged = DEFAULT_LED_CONTROL_CONFIG.copy()
    if config:
        merged.update(config)
    return merged


def _clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class LEDControlDriver(Process):
    """
    Proceso dedicado al control por LEDs.

    Recibe start/stop desde Musician, lee la camara con LEDPoseTracker y emite
    targets en coordenadas de tarea relativos a la pose base recibida.
    """

    def __init__(self, running, pipe_end, config=None, verbose=False):
        Process.__init__(self)
        self.running = running
        self.pipe_end = pipe_end
        self.config = merged_led_control_config(config)
        self.tracker_config = merged_led_tracker_config(config)
        self.verbose = verbose

        self.active = False
        self.shutdown_requested = False
        self.tracker = None
        self.base_state = None
        self.last_valid_measurement_time = None
        self.last_command_time = None
        self.last_commanded_n_delta = None
        self.last_commanded_perp_delta = None
        self.last_commanded_alpha_delta = None
        self.stale_reported = False
        self.active_reported = False

    def run(self):
        self.pipe_end.send(["led_control_driver_started"])
        while self.running.is_set() and not self.shutdown_requested:
            loop_t0 = time.perf_counter()
            self._handle_pending_commands()

            if self.active:
                self._run_active_iteration()
                elapsed = time.perf_counter() - loop_t0
                remaining = float(self.config["update_interval_s"]) - elapsed
                if remaining > 0:
                    time.sleep(remaining)
            else:
                time.sleep(0.05)

        self._stop("shutdown")

    def _handle_pending_commands(self):
        while self.pipe_end.poll():
            message = self.pipe_end.recv()
            if not isinstance(message, (list, tuple)) or not message:
                continue

            cmd = message[0]
            if cmd == "start":
                self._start(message[1])
            elif cmd == "stop":
                self._stop("stopped")
            elif cmd == "shutdown":
                self.shutdown_requested = True
            elif cmd == "configure":
                self.config = merged_led_control_config(message[1])
                self.tracker_config = merged_led_tracker_config(message[1])

    def _start(self, base_state):
        if not self.config.get("enabled", True):
            self.pipe_end.send(["led_control_status", "disabled", "Control por LEDs deshabilitado"])
            return

        self._stop("restart")
        self.base_state = dict(base_state)
        self.last_valid_measurement_time = None
        self.last_command_time = time.perf_counter()
        self.last_commanded_n_delta = 0.0
        self.last_commanded_perp_delta = 0.0
        self.last_commanded_alpha_delta = 0.0
        self.stale_reported = False
        self.active_reported = False

        try:
            self.tracker = LEDPoseTracker(self.tracker_config)
            self.tracker.open()
        except Exception as exc:
            logger.error(f"No se pudo iniciar control por LEDs: {exc}", exc_info=True)
            self.tracker = None
            self.active = False
            self.pipe_end.send(["led_control_status", "error", str(exc)])
            return

        self.active = True
        self.pipe_end.send(["led_control_status", "starting", "Buscando LEDs"])

    def _stop(self, reason):
        was_active = self.active
        self.active = False
        if self.tracker is not None:
            try:
                self.tracker.close()
            except Exception:
                logger.debug("Error cerrando tracker LED", exc_info=True)
            self.tracker = None

        self.base_state = None
        if was_active and reason != "restart":
            self.pipe_end.send(["led_control_stopped", reason])

    def _run_active_iteration(self):
        try:
            measurement, display, plot_image, now, error = self.tracker.read()
        except Exception as exc:
            logger.error(f"Error leyendo LEDs: {exc}", exc_info=True)
            self.pipe_end.send(["led_control_status", "error", str(exc)])
            self._stop("error")
            return

        if error:
            self.pipe_end.send(["led_control_status", "error", error])
            self._stop(error)
            return

        key = self.tracker.show(display, plot_image)
        if key == 27 or key == ord("q"):
            self._stop("user_closed")
            return

        if measurement is None or not measurement.get("fresh", False):
            self._handle_missing_measurement(now)
            return

        self.last_valid_measurement_time = time.perf_counter()
        self.stale_reported = False
        if not self.active_reported:
            self.active_reported = True
            self.pipe_end.send(["led_control_status", "active", "Control por LEDs activo"])

        target = self._measurement_to_target(measurement)
        if target is None:
            return

        self.pipe_end.send(["led_control_target", target, measurement])

    def _handle_missing_measurement(self, _now):
        if self.last_valid_measurement_time is None:
            return

        stale_for = time.perf_counter() - self.last_valid_measurement_time
        if stale_for >= float(self.config["stale_timeout_s"]) and not self.stale_reported:
            self.stale_reported = True
            self.pipe_end.send([
                "led_control_status",
                "holding",
                "LEDs perdidos: manteniendo ultima referencia",
            ])

    def _measurement_to_target(self, measurement):
        camera_x = float(measurement["camera_x_depth"])
        camera_z = float(measurement["camera_z_vertical"])
        camera_distance = float(measurement["camera_distance_xz"])
        tilt = float(measurement["tilt_deg"])

        if not all(math.isfinite(value) for value in (camera_x, camera_z, camera_distance, tilt)):
            return None

        distance_zero = float(self.config["distance_zero"])
        if camera_distance <= distance_zero:
            return None

        amplification = float(self.config["distance_amplification"])
        camera_x_delta = camera_x - distance_zero
        camera_z_delta = camera_z

        n_delta = _clamp(
            -camera_x_delta * amplification,
            -self._get_config_float("max_camera_x_delta_mm", "max_radius_delta_mm"),
            self._get_config_float("max_camera_x_delta_mm", "max_radius_delta_mm"),
        )
        perp_delta = _clamp(
            -camera_z_delta * amplification,
            -self._get_config_float("max_camera_z_delta_mm", "max_radius_delta_mm"),
            self._get_config_float("max_camera_z_delta_mm", "max_radius_delta_mm"),
        )
        alpha_delta = _clamp(
            tilt * float(self.config["tilt_to_alpha_sign"]),
            -self._get_config_float("max_alpha_delta_deg", "max_theta_delta_deg"),
            self._get_config_float("max_alpha_delta_deg", "max_theta_delta_deg"),
        )

        n_delta, perp_delta, alpha_delta = self._rate_limit_target(
            n_delta,
            perp_delta,
            alpha_delta,
        )

        if (
            abs(n_delta - self.last_commanded_n_delta) < self._get_config_float("translation_deadband_mm", "radius_deadband_mm")
            and abs(perp_delta - self.last_commanded_perp_delta) < self._get_config_float("translation_deadband_mm", "radius_deadband_mm")
            and abs(alpha_delta - self.last_commanded_alpha_delta) < self._get_config_float("alpha_deadband_deg", "theta_deadband_deg")
        ):
            return None

        self.last_commanded_n_delta = n_delta
        self.last_commanded_perp_delta = perp_delta
        self.last_commanded_alpha_delta = alpha_delta
        self.last_command_time = time.perf_counter()

        alpha0_rad = math.radians(float(self.base_state["alpha"]))
        n_x = math.cos(alpha0_rad)
        n_z = math.sin(alpha0_rad)
        perp_x = -math.sin(alpha0_rad)
        perp_z = math.cos(alpha0_rad)

        target_x = float(self.base_state["x"]) + n_delta * n_x + perp_delta * perp_x
        target_z = float(self.base_state["z"]) + n_delta * n_z + perp_delta * perp_z
        target_alpha = float(self.base_state["alpha"]) + alpha_delta

        return {
            "x": target_x,
            "z": target_z,
            "alpha": target_alpha,
            "flow": float(self.base_state.get("flow", 0.0)),
            "lips": int(self.base_state.get("lips", 31)),
            "tongue": int(self.base_state.get("tongue", 0)),
            "camera_x": camera_x,
            "camera_z": camera_z,
            "camera_distance": camera_distance,
            "tilt": tilt,
            "camera_x_delta": camera_x_delta,
            "camera_z_delta": camera_z_delta,
            "n_delta": n_delta,
            "perp_delta": perp_delta,
            "alpha_delta": alpha_delta,
        }

    def _rate_limit_target(self, n_delta, perp_delta, alpha_delta):
        now = time.perf_counter()
        dt = max(now - self.last_command_time, float(self.config["update_interval_s"]))

        max_n_step = self._get_config_float("max_robot_n_rate_mm_s", "max_radius_rate_mm_s") * dt
        max_perp_step = self._get_config_float("max_robot_perp_rate_mm_s", "max_radius_rate_mm_s") * dt
        max_alpha_step = self._get_config_float("max_alpha_rate_deg_s", "max_theta_rate_deg_s") * dt

        limited_n = self.last_commanded_n_delta + _clamp(
            n_delta - self.last_commanded_n_delta,
            -max_n_step,
            max_n_step,
        )
        limited_perp = self.last_commanded_perp_delta + _clamp(
            perp_delta - self.last_commanded_perp_delta,
            -max_perp_step,
            max_perp_step,
        )
        limited_alpha = self.last_commanded_alpha_delta + _clamp(
            alpha_delta - self.last_commanded_alpha_delta,
            -max_alpha_step,
            max_alpha_step,
        )
        return limited_n, limited_perp, limited_alpha

    def _get_config_float(self, key, fallback_key):
        if key in self.config:
            return float(self.config[key])
        return float(self.config[fallback_key])
