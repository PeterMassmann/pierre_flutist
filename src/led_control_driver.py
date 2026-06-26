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
    "mouth_tip_offset_cm": 4.5,
    "flute_center_offset_cm": 6.0,
    "flute_safety_radius_cm": 6.0,
    "update_interval_s": 0.08,
    "stale_timeout_s": 0.8,
    "max_robot_delta_cm": 10.0,
    "max_camera_x_delta_mm": 15.0,
    "max_camera_z_delta_mm": 10.0,
    "max_alpha_abs_deg": 5.0,
    "max_alpha_delta_deg": 5.0,
    "max_robot_n_rate_mm_s": 10.0,
    "max_robot_perp_rate_mm_s": 10.0,
    "max_alpha_rate_deg_s": 10.0,
    "translation_deadband_mm": 0.1,
    "alpha_deadband_deg": 0.1,
    "tilt_to_alpha_sign": -1.0,
    "angle_to_alpha_sign": 1.0,
    "angle_to_alpha_offset_deg": 0.0,
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
        self.flute_center_x_mm = None
        self.flute_center_z_mm = None
        self.neutral_tip_offset_x_mm = None
        self.neutral_tip_offset_z_mm = None
        self.last_valid_measurement_time = None
        self.last_command_time = None
        self.last_commanded_x_delta_mm = None
        self.last_commanded_z_delta_mm = None
        self.last_commanded_alpha = None
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
        self._set_base_geometry()
        self.last_valid_measurement_time = None
        self.last_command_time = time.perf_counter()
        self.last_commanded_x_delta_mm = 0.0
        self.last_commanded_z_delta_mm = 0.0
        self.last_commanded_alpha = float(self.base_state["alpha"])
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
        self.flute_center_x_mm = None
        self.flute_center_z_mm = None
        self.neutral_tip_offset_x_mm = None
        self.neutral_tip_offset_z_mm = None
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
        camera_x_cm = float(measurement["camera_x_depth"])
        camera_z_cm = float(measurement["camera_z_vertical"])
        camera_distance_cm = float(measurement["camera_distance_xz"])
        led_angle_deg = float(measurement["tilt_deg"])

        if not all(math.isfinite(value) for value in (camera_x_cm, camera_z_cm, camera_distance_cm, led_angle_deg)):
            return None

        amplification = float(self.config["distance_amplification"])
        measured_tip_offset_x_mm = camera_x_cm * 10.0
        measured_tip_offset_z_mm = camera_z_cm * 10.0
        tip_offset_x_mm = (
            self.neutral_tip_offset_x_mm
            + (measured_tip_offset_x_mm - self.neutral_tip_offset_x_mm) * amplification
        )
        tip_offset_z_mm = (
            self.neutral_tip_offset_z_mm
            + (measured_tip_offset_z_mm - self.neutral_tip_offset_z_mm) * amplification
        )
        mouth_tip_x_mm = self.flute_center_x_mm + tip_offset_x_mm
        mouth_tip_z_mm = self.flute_center_z_mm + tip_offset_z_mm

        raw_alpha_target = _clamp(
            led_angle_deg * self._get_angle_to_alpha_sign() + self._get_angle_to_alpha_offset_deg(),
            -self._get_config_float("max_alpha_abs_deg", "max_alpha_delta_deg"),
            self._get_config_float("max_alpha_abs_deg", "max_alpha_delta_deg"),
        )

        dt = self._get_rate_limit_dt()
        alpha_target = self._rate_limit_alpha(raw_alpha_target, dt)
        target_x, target_z = self._mouth_tip_to_robot_point(
            mouth_tip_x_mm,
            mouth_tip_z_mm,
            alpha_target,
        )
        target_x, target_z, safety_limited = self._enforce_flute_safety(
            target_x,
            target_z,
            alpha_target,
        )

        x_delta_mm = target_x - float(self.base_state["x"])
        z_delta_mm = target_z - float(self.base_state["z"])
        x_delta_mm, z_delta_mm = self._limit_robot_delta_radius(x_delta_mm, z_delta_mm)
        target_x = float(self.base_state["x"]) + x_delta_mm
        target_z = float(self.base_state["z"]) + z_delta_mm
        target_x, target_z, safety_limited_after_radius = self._enforce_flute_safety(
            target_x,
            target_z,
            alpha_target,
        )
        safety_limited = safety_limited or safety_limited_after_radius
        x_delta_mm = target_x - float(self.base_state["x"])
        z_delta_mm = target_z - float(self.base_state["z"])

        x_delta_mm, z_delta_mm = self._rate_limit_translation_deltas(
            x_delta_mm,
            z_delta_mm,
            dt,
        )
        target_x = float(self.base_state["x"]) + x_delta_mm
        target_z = float(self.base_state["z"]) + z_delta_mm
        target_x, target_z, safety_limited_after_rate = self._enforce_flute_safety(
            target_x,
            target_z,
            alpha_target,
        )
        safety_limited = safety_limited or safety_limited_after_rate
        x_delta_mm = target_x - float(self.base_state["x"])
        z_delta_mm = target_z - float(self.base_state["z"])

        alpha_delta_from_start = alpha_target - float(self.base_state["alpha"])

        if (
            abs(x_delta_mm - self.last_commanded_x_delta_mm) < self._get_config_float("translation_deadband_mm", "radius_deadband_mm")
            and abs(z_delta_mm - self.last_commanded_z_delta_mm) < self._get_config_float("translation_deadband_mm", "radius_deadband_mm")
            and abs(alpha_target - self.last_commanded_alpha) < self._get_config_float("alpha_deadband_deg", "theta_deadband_deg")
        ):
            return None

        self.last_commanded_x_delta_mm = x_delta_mm
        self.last_commanded_z_delta_mm = z_delta_mm
        self.last_commanded_alpha = alpha_target
        self.last_command_time = time.perf_counter()

        return {
            "x": target_x,
            "z": target_z,
            "alpha": alpha_target,
            "flow": float(self.base_state.get("flow", 0.0)),
            "lips": int(self.base_state.get("lips", 31)),
            "tongue": int(self.base_state.get("tongue", 0)),
            "camera_x": camera_x_cm,
            "camera_z": camera_z_cm,
            "camera_distance": camera_distance_cm,
            "tilt": led_angle_deg,
            "camera_x_delta": camera_x_cm,
            "camera_z_delta": camera_z_cm,
            "x_delta_mm": x_delta_mm,
            "z_delta_mm": z_delta_mm,
            "mouth_tip_x": mouth_tip_x_mm,
            "mouth_tip_z": mouth_tip_z_mm,
            "flute_center_x": self.flute_center_x_mm,
            "flute_center_z": self.flute_center_z_mm,
            "safety_limited": bool(safety_limited),
            "n_delta": x_delta_mm,
            "perp_delta": z_delta_mm,
            "alpha_delta": alpha_delta_from_start,
            "alpha_target": alpha_target,
        }

    def _set_base_geometry(self):
        base_x = float(self.base_state["x"])
        base_z = float(self.base_state["z"])
        base_alpha = float(self.base_state["alpha"])
        direction_x, direction_z = self._alpha_unit(base_alpha)
        center_offset_mm = float(self.config["flute_center_offset_cm"]) * 10.0
        mouth_offset_mm = float(self.config["mouth_tip_offset_cm"]) * 10.0

        self.flute_center_x_mm = base_x + center_offset_mm * direction_x
        self.flute_center_z_mm = base_z + center_offset_mm * direction_z
        neutral_offset_mm = mouth_offset_mm - center_offset_mm
        self.neutral_tip_offset_x_mm = neutral_offset_mm * direction_x
        self.neutral_tip_offset_z_mm = neutral_offset_mm * direction_z

    def _alpha_unit(self, alpha_deg):
        alpha_rad = math.radians(float(alpha_deg))
        return math.cos(alpha_rad), math.sin(alpha_rad)

    def _mouth_tip_to_robot_point(self, mouth_tip_x_mm, mouth_tip_z_mm, alpha_deg):
        direction_x, direction_z = self._alpha_unit(alpha_deg)
        mouth_offset_mm = float(self.config["mouth_tip_offset_cm"]) * 10.0
        return (
            mouth_tip_x_mm - mouth_offset_mm * direction_x,
            mouth_tip_z_mm - mouth_offset_mm * direction_z,
        )

    def _enforce_flute_safety(self, target_x_mm, target_z_mm, alpha_deg):
        safety_radius_mm = float(self.config["flute_safety_radius_cm"]) * 10.0
        dx = target_x_mm - self.flute_center_x_mm
        dz = target_z_mm - self.flute_center_z_mm
        distance_mm = math.hypot(dx, dz)

        if distance_mm >= safety_radius_mm or safety_radius_mm <= 0.0:
            return target_x_mm, target_z_mm, False

        if distance_mm <= 1e-9:
            direction_x, direction_z = self._alpha_unit(alpha_deg)
            dx = -direction_x
            dz = -direction_z
            distance_mm = 1.0

        scale = safety_radius_mm / distance_mm
        return (
            self.flute_center_x_mm + dx * scale,
            self.flute_center_z_mm + dz * scale,
            True,
        )

    def _limit_robot_delta_radius(self, x_delta_mm, z_delta_mm):
        max_radius_mm = self._get_config_float("max_robot_delta_cm", "max_radius_delta_cm") * 10.0
        radius_mm = math.hypot(x_delta_mm, z_delta_mm)

        if radius_mm <= max_radius_mm or radius_mm <= 1e-9:
            return x_delta_mm, z_delta_mm

        scale = max_radius_mm / radius_mm
        return x_delta_mm * scale, z_delta_mm * scale

    def _get_rate_limit_dt(self):
        return max(
            time.perf_counter() - self.last_command_time,
            float(self.config["update_interval_s"]),
        )

    def _rate_limit_translation_deltas(self, x_delta_mm, z_delta_mm, dt):
        max_x_step = self._get_config_float("max_robot_n_rate_mm_s", "max_radius_rate_mm_s") * dt
        max_z_step = self._get_config_float("max_robot_perp_rate_mm_s", "max_radius_rate_mm_s") * dt

        limited_x = self.last_commanded_x_delta_mm + _clamp(
            x_delta_mm - self.last_commanded_x_delta_mm,
            -max_x_step,
            max_x_step,
        )
        limited_z = self.last_commanded_z_delta_mm + _clamp(
            z_delta_mm - self.last_commanded_z_delta_mm,
            -max_z_step,
            max_z_step,
        )
        return limited_x, limited_z

    def _rate_limit_alpha(self, alpha_target, dt):
        max_alpha_step = self._get_config_float("max_alpha_rate_deg_s", "max_theta_rate_deg_s") * dt
        return self.last_commanded_alpha + _clamp(
            alpha_target - self.last_commanded_alpha,
            -max_alpha_step,
            max_alpha_step,
        )

    def _get_angle_to_alpha_sign(self):
        if "angle_to_alpha_sign" in self.config:
            return float(self.config["angle_to_alpha_sign"])
        return float(self.config["tilt_to_alpha_sign"])

    def _get_angle_to_alpha_offset_deg(self):
        return float(self.config.get("angle_to_alpha_offset_deg", 0.0))

    def _get_config_float(self, key, fallback_key):
        if key in self.config:
            return float(self.config[key])
        return float(self.config[fallback_key])
