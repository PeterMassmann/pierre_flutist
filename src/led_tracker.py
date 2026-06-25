import itertools
import math
import time

import numpy as np

try:
    import cv2
except ImportError:  # Permite iniciar Pierre aunque OpenCV no este instalado.
    cv2 = None


DEFAULT_LED_TRACKER_CONFIG = {
    "camera_index": 1,
    "camera_frame_width": None,
    "camera_frame_height": None,
    "camera_fps": None,
    "camera_auto_exposure": None,
    "camera_exposure": None,
    "camera_gain": None,
    "camera_auto_wb": None,
    "camera_wb_temperature": None,
    "red_green_distance": 9.0,
    "red_blue_distance": 12.2,
    "green_blue_distance": 9.0,
    "yellow_red_distance": 9.0,
    "yellow_blue_distance": 9.0,
    "yellow_plane_offset_z": -2.7,
    "camera_matrix": [
        [913.94, 0.0, 640.0],
        [0.0, 913.94, 360.0],
        [0.0, 0.0, 1.0],
    ],
    "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    "approx_focal_length_factor": 1.0,
    "use_previous_pose_guess": True,
    "reject_tilt_jumps": True,
    "max_tilt_jump_deg": 6.0,
    "reject_position_jumps": True,
    "max_camera_x_jump": 8.0,
    "max_camera_z_jump": 8.0,
    "max_camera_distance_jump": 10.0,
    "max_consecutive_rejects": 8,
    "print_rejected_frames": True,
    "reject_reprojection_errors": True,
    "max_mean_reprojection_error_px": 8.0,
    "max_reprojection_error_px": 15.0,
    "reject_bad_image_geometry": True,
    "min_led_pair_distance_px": 3.0,
    "min_rgb_triangle_area_px2": 8.0,
    "min_yellow_triangle_area_px2": 6.0,
    "min_image_geometry_scale": 0.35,
    "max_image_geometry_scale": 2.8,
    "light_low_hsv": [80, 0, 200],
    "light_high_hsv": [100, 10, 255],
    "red_low_hsv": [0, 120, 120],
    "red_high_hsv": [10, 255, 255],
    "red_low_hsv_2": [170, 120, 120],
    "red_high_hsv_2": [179, 255, 255],
    "green_low_hsv": [35, 80, 80],
    "green_high_hsv": [85, 255, 255],
    "blue_low_hsv": [90, 80, 80],
    "blue_high_hsv": [130, 255, 255],
    "yellow_low_hsv": [18, 80, 80],
    "yellow_high_hsv": [38, 255, 255],
    "min_light_area": 20,
    "max_light_area": None,
    "min_light_circularity": 0.2,
    "max_light_aspect_ratio": 3.0,
    "max_light_candidates": 6,
    "min_color_area": 10,
    "morph_kernel_size": 5,
    "match_radius": 25,
    "max_match_distance": 80,
    "distance_weight": 0.5,
    "overlap_weight": 0.5,
    "color_score_weight": 1.0,
    "tracking_score_weight": 0.35,
    "tracking_enabled": True,
    "tracking_max_distance_px": 80.0,
    "min_color_score": 0.08,
    "min_total_assignment_score": 0.8,
    "min_assignment_margin": 0.02,
    "moving_average_n": 10,
    "camera_window_name": "Camara original con deteccion",
    "plot_window_name": "Graficos coordenadas camara y tilt",
    "show_windows": True,
    "plot_width": 900,
    "plot_history_seconds": 10.0,
    "plot_margin_left": 65,
    "plot_margin_right": 25,
    "plot_margin_top": 35,
    "plot_margin_bottom": 40,
    "plot_height": 650,
    "camera_x_plot_min": 0.0,
    "camera_x_plot_max": 60.0,
    "camera_z_plot_min": -30.0,
    "camera_z_plot_max": 30.0,
    "tilt_plot_min": -90.0,
    "tilt_plot_max": 90.0,
}


def merged_led_tracker_config(config=None):
    merged = DEFAULT_LED_TRACKER_CONFIG.copy()
    if config:
        merged.update(config)
    return merged


def _as_hsv_array(value):
    return np.array(value, dtype=np.uint8)


def _as_float_matrix(value):
    if value is None:
        return None
    return np.array(value, dtype=np.float32)


def _optional_float(value):
    if value is None:
        return None
    return float(value)


def _angle_difference_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def _triangle_area_2d(a, b, c):
    return abs(
        (float(b[0]) - float(a[0])) * (float(c[1]) - float(a[1]))
        - (float(c[0]) - float(a[0])) * (float(b[1]) - float(a[1]))
    ) * 0.5


def _append_capped_sample(samples, value, max_count):
    samples.append(float(value))
    while len(samples) > max_count:
        samples.pop(0)


def _average_values(values):
    if not values:
        return None
    return float(sum(values) / len(values))


def _average_angles_deg(angles_deg):
    if not angles_deg:
        return None

    sin_sum = 0.0
    cos_sum = 0.0
    for angle_deg in angles_deg:
        angle_rad = math.radians(angle_deg)
        sin_sum += math.sin(angle_rad)
        cos_sum += math.cos(angle_rad)

    if abs(sin_sum) < 1e-12 and abs(cos_sum) < 1e-12:
        return 0.0

    return float(math.degrees(math.atan2(sin_sum, cos_sum)))


def _trim_history(history, current_time, history_seconds):
    min_time = current_time - history_seconds
    while history and history[0][0] < min_time:
        history.pop(0)


class LEDPoseTracker:
    """
    Detecta los cuatro LEDs, estima distancia/tilt con solvePnP y genera las
    dos vistas usadas por el control: camara con vectores y grafico temporal.
    """

    def __init__(self, config=None):
        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) no esta instalado.")

        self.config = merged_led_tracker_config(config)
        self._read_config()
        self._validate_config()

        self.cap = None
        self.last_accepted_pose = None
        self.last_rvec = None
        self.last_tvec = None
        self.last_centers = None
        self.last_assignment_info = {}
        self.last_pose_rejection_reason = None
        self.consecutive_rejects = 0
        self.camera_x_history = []
        self.camera_z_history = []
        self.tilt_history = []
        self.camera_x_average_samples = []
        self.camera_z_average_samples = []
        self.tilt_average_samples = []

    def _read_config(self):
        cfg = self.config

        self.camera_index = int(cfg["camera_index"])
        self.camera_frame_width = _optional_float(cfg.get("camera_frame_width"))
        self.camera_frame_height = _optional_float(cfg.get("camera_frame_height"))
        self.camera_fps = _optional_float(cfg.get("camera_fps"))
        self.camera_auto_exposure = _optional_float(cfg.get("camera_auto_exposure"))
        self.camera_exposure = _optional_float(cfg.get("camera_exposure"))
        self.camera_gain = _optional_float(cfg.get("camera_gain"))
        self.camera_auto_wb = _optional_float(cfg.get("camera_auto_wb"))
        self.camera_wb_temperature = _optional_float(cfg.get("camera_wb_temperature"))
        self.red_green_distance = float(cfg["red_green_distance"])
        self.red_blue_distance = float(cfg["red_blue_distance"])
        self.green_blue_distance = float(cfg["green_blue_distance"])
        self.yellow_red_distance = float(cfg["yellow_red_distance"])
        self.yellow_blue_distance = float(cfg["yellow_blue_distance"])
        self.yellow_plane_offset_z = float(cfg["yellow_plane_offset_z"])

        self.camera_matrix = _as_float_matrix(cfg.get("camera_matrix"))
        dist_coeffs = cfg.get("dist_coeffs")
        self.dist_coeffs = (
            np.array(dist_coeffs, dtype=np.float32).reshape(-1, 1)
            if dist_coeffs is not None
            else np.zeros((5, 1), dtype=np.float32)
        )
        self.approx_focal_length_factor = float(cfg["approx_focal_length_factor"])

        self.use_previous_pose_guess = bool(cfg["use_previous_pose_guess"])
        self.reject_tilt_jumps = bool(cfg["reject_tilt_jumps"])
        self.max_tilt_jump_deg = float(cfg["max_tilt_jump_deg"])
        self.reject_position_jumps = bool(cfg["reject_position_jumps"])
        self.max_camera_x_jump = float(cfg["max_camera_x_jump"])
        self.max_camera_z_jump = float(cfg["max_camera_z_jump"])
        self.max_camera_distance_jump = float(cfg["max_camera_distance_jump"])
        self.max_consecutive_rejects = int(cfg["max_consecutive_rejects"])
        self.print_rejected_frames = bool(cfg["print_rejected_frames"])
        self.reject_reprojection_errors = bool(cfg["reject_reprojection_errors"])
        self.max_mean_reprojection_error_px = float(cfg["max_mean_reprojection_error_px"])
        self.max_reprojection_error_px = float(cfg["max_reprojection_error_px"])
        self.reject_bad_image_geometry = bool(cfg["reject_bad_image_geometry"])
        self.min_led_pair_distance_px = float(cfg["min_led_pair_distance_px"])
        self.min_rgb_triangle_area_px2 = float(cfg["min_rgb_triangle_area_px2"])
        self.min_yellow_triangle_area_px2 = float(cfg["min_yellow_triangle_area_px2"])
        self.min_image_geometry_scale = float(cfg["min_image_geometry_scale"])
        self.max_image_geometry_scale = float(cfg["max_image_geometry_scale"])

        self.light_low_hsv = _as_hsv_array(cfg["light_low_hsv"])
        self.light_high_hsv = _as_hsv_array(cfg["light_high_hsv"])
        self.red_low_hsv = _as_hsv_array(cfg["red_low_hsv"])
        self.red_high_hsv = _as_hsv_array(cfg["red_high_hsv"])
        self.red_low_hsv_2 = _as_hsv_array(cfg["red_low_hsv_2"])
        self.red_high_hsv_2 = _as_hsv_array(cfg["red_high_hsv_2"])
        self.green_low_hsv = _as_hsv_array(cfg["green_low_hsv"])
        self.green_high_hsv = _as_hsv_array(cfg["green_high_hsv"])
        self.blue_low_hsv = _as_hsv_array(cfg["blue_low_hsv"])
        self.blue_high_hsv = _as_hsv_array(cfg["blue_high_hsv"])
        self.yellow_low_hsv = _as_hsv_array(cfg["yellow_low_hsv"])
        self.yellow_high_hsv = _as_hsv_array(cfg["yellow_high_hsv"])

        self.min_light_area = float(cfg["min_light_area"])
        self.max_light_area = _optional_float(cfg.get("max_light_area"))
        self.min_light_circularity = float(cfg["min_light_circularity"])
        self.max_light_aspect_ratio = float(cfg["max_light_aspect_ratio"])
        self.max_light_candidates = int(cfg["max_light_candidates"])
        self.min_color_area = int(cfg["min_color_area"])
        self.morph_kernel_size = int(cfg["morph_kernel_size"])
        self.match_radius = int(cfg["match_radius"])
        self.max_match_distance = float(cfg["max_match_distance"])
        self.distance_weight = float(cfg["distance_weight"])
        self.overlap_weight = float(cfg["overlap_weight"])
        self.color_score_weight = float(cfg["color_score_weight"])
        self.tracking_score_weight = float(cfg["tracking_score_weight"])
        self.tracking_enabled = bool(cfg["tracking_enabled"])
        self.tracking_max_distance_px = float(cfg["tracking_max_distance_px"])
        self.min_color_score = float(cfg["min_color_score"])
        self.min_total_assignment_score = float(cfg["min_total_assignment_score"])
        self.min_assignment_margin = float(cfg["min_assignment_margin"])
        self.moving_average_n = int(cfg["moving_average_n"])

        self.camera_window_name = str(cfg["camera_window_name"])
        self.plot_window_name = str(cfg["plot_window_name"])
        self.show_windows = bool(cfg["show_windows"])
        self.plot_width = int(cfg["plot_width"])
        self.plot_height = int(cfg["plot_height"])
        self.plot_history_seconds = float(cfg["plot_history_seconds"])
        self.plot_margin_left = int(cfg["plot_margin_left"])
        self.plot_margin_right = int(cfg["plot_margin_right"])
        self.plot_margin_top = int(cfg["plot_margin_top"])
        self.plot_margin_bottom = int(cfg["plot_margin_bottom"])
        self.camera_x_plot_min = cfg.get("camera_x_plot_min", cfg.get("distance_plot_min", 0.0))
        self.camera_x_plot_max = cfg.get("camera_x_plot_max", cfg.get("distance_plot_max", 60.0))
        self.camera_z_plot_min = cfg.get("camera_z_plot_min", -30.0)
        self.camera_z_plot_max = cfg.get("camera_z_plot_max", 30.0)
        self.tilt_plot_min = cfg["tilt_plot_min"]
        self.tilt_plot_max = cfg["tilt_plot_max"]

    def _validate_config(self):
        distances = {
            "red_green_distance": self.red_green_distance,
            "red_blue_distance": self.red_blue_distance,
            "green_blue_distance": self.green_blue_distance,
            "yellow_red_distance": self.yellow_red_distance,
            "yellow_blue_distance": self.yellow_blue_distance,
        }
        for name, value in distances.items():
            if value <= 0:
                raise ValueError(f"{name} debe ser mayor que cero.")

        d_rg = self.red_green_distance
        d_rb = self.red_blue_distance
        d_gb = self.green_blue_distance
        if d_rg + d_rb <= d_gb or d_rg + d_gb <= d_rb or d_rb + d_gb <= d_rg:
            raise ValueError("Las distancias RGB no forman un triangulo valido.")

        yellow_x = (
            self.yellow_red_distance ** 2
            - self.yellow_blue_distance ** 2
            + self.red_blue_distance ** 2
        ) / (2.0 * self.red_blue_distance)
        yellow_y_squared = (
            self.yellow_red_distance ** 2
            - yellow_x ** 2
            - self.yellow_plane_offset_z ** 2
        )
        if yellow_y_squared < -1e-6:
            raise ValueError("La geometria del LED amarillo no es posible.")

        if self.moving_average_n < 1:
            raise ValueError("moving_average_n debe ser mayor o igual a 1.")

        if self.max_light_candidates < 4:
            raise ValueError("max_light_candidates debe ser mayor o igual a 4.")

        if self.min_light_area <= 0:
            raise ValueError("min_light_area debe ser mayor que cero.")

        if self.max_light_area is not None and self.max_light_area <= self.min_light_area:
            raise ValueError("max_light_area debe ser mayor que min_light_area.")

        if self.min_light_circularity < 0.0 or self.min_light_circularity > 1.0:
            raise ValueError("min_light_circularity debe estar entre 0 y 1.")

        if self.max_light_aspect_ratio < 1.0:
            raise ValueError("max_light_aspect_ratio debe ser mayor o igual a 1.")

        if self.min_color_score < 0.0 or self.min_color_score > 1.0:
            raise ValueError("min_color_score debe estar entre 0 y 1.")

        if self.max_match_distance <= 0:
            raise ValueError("max_match_distance debe ser mayor que cero.")

        if self.tracking_max_distance_px <= 0:
            raise ValueError("tracking_max_distance_px debe ser mayor que cero.")

        if self.min_image_geometry_scale <= 0 or self.max_image_geometry_scale <= self.min_image_geometry_scale:
            raise ValueError("Los limites de escala de geometria de imagen son invalidos.")

        if self.max_mean_reprojection_error_px <= 0 or self.max_reprojection_error_px <= 0:
            raise ValueError("Los limites de error de reproyeccion deben ser mayores que cero.")

        if self.max_camera_x_jump <= 0 or self.max_camera_z_jump <= 0 or self.max_camera_distance_jump <= 0:
            raise ValueError("Los limites de salto de posicion deben ser mayores que cero.")

        if (
            self.camera_x_plot_min is not None
            and self.camera_x_plot_max is not None
            and self.camera_x_plot_max <= self.camera_x_plot_min
        ):
            raise ValueError("camera_x_plot_max debe ser mayor que camera_x_plot_min.")

        if (
            self.camera_z_plot_min is not None
            and self.camera_z_plot_max is not None
            and self.camera_z_plot_max <= self.camera_z_plot_min
        ):
            raise ValueError("camera_z_plot_max debe ser mayor que camera_z_plot_min.")

        if (
            self.tilt_plot_min is not None
            and self.tilt_plot_max is not None
            and self.tilt_plot_max <= self.tilt_plot_min
        ):
            raise ValueError("tilt_plot_max debe ser mayor que tilt_plot_min.")

    def open(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la camara de LEDs.")
        self._apply_camera_settings()

    def _set_camera_property(self, property_name, value):
        if value is None:
            return

        property_id = getattr(cv2, property_name, None)
        if property_id is None:
            return

        self.cap.set(property_id, float(value))

    def _apply_camera_settings(self):
        self._set_camera_property("CAP_PROP_FRAME_WIDTH", self.camera_frame_width)
        self._set_camera_property("CAP_PROP_FRAME_HEIGHT", self.camera_frame_height)
        self._set_camera_property("CAP_PROP_FPS", self.camera_fps)
        self._set_camera_property("CAP_PROP_AUTO_EXPOSURE", self.camera_auto_exposure)
        self._set_camera_property("CAP_PROP_EXPOSURE", self.camera_exposure)
        self._set_camera_property("CAP_PROP_GAIN", self.camera_gain)
        self._set_camera_property("CAP_PROP_AUTO_WB", self.camera_auto_wb)
        self._set_camera_property("CAP_PROP_WB_TEMPERATURE", self.camera_wb_temperature)

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        if self.show_windows:
            try:
                cv2.destroyWindow(self.camera_window_name)
            except cv2.error:
                pass
            try:
                cv2.destroyWindow(self.plot_window_name)
            except cv2.error:
                pass

    def _create_mask(self, hsv_frame, low_hsv, high_hsv, low_hsv_2=None, high_hsv_2=None):
        mask = cv2.inRange(hsv_frame, low_hsv, high_hsv)

        if low_hsv_2 is not None and high_hsv_2 is not None:
            mask_2 = cv2.inRange(hsv_frame, low_hsv_2, high_hsv_2)
            mask = cv2.bitwise_or(mask, mask_2)

        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _find_light_candidates(self, light_mask):
        contours, _ = cv2.findContours(
            light_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_light_area:
                continue
            if self.max_light_area is not None and area > self.max_light_area:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue

            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.min_light_circularity:
                continue

            _x, _y, width, height = cv2.boundingRect(contour)
            aspect_ratio = max(width, height) / max(1, min(width, height))
            if aspect_ratio > self.max_light_aspect_ratio:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            candidates.append({
                "center": np.array([cx, cy], dtype=np.float32),
                "area": area,
                "circularity": circularity,
                "aspect_ratio": aspect_ratio,
                "contour": contour,
            })

        candidates.sort(key=lambda c: c["area"], reverse=True)
        return candidates[:self.max_light_candidates]

    def _prepare_color_mask_info(self, color_mask):
        binary = (color_mask > 0).astype(np.uint8)
        pixel_count = int(np.count_nonzero(binary))
        distance_map = None

        if pixel_count >= self.min_color_area:
            inverted = 1 - binary
            distance_map = cv2.distanceTransform(
                inverted,
                cv2.DIST_L2,
                cv2.DIST_MASK_PRECISE,
            )

        return {
            "mask": color_mask,
            "binary": binary,
            "pixel_count": pixel_count,
            "distance_map": distance_map,
        }

    def _get_distance_to_mask_score(self, center, mask_info):
        x, y = center.astype(int)
        height, width = mask_info["mask"].shape

        if x < 0 or x >= width or y < 0 or y >= height:
            return 0.0, float("inf")

        distance_map = mask_info["distance_map"]
        if distance_map is None:
            return 0.0, float("inf")

        distance_px = float(distance_map[y, x])
        score = 1.0 - min(distance_px / self.max_match_distance, 1.0)
        return score, distance_px

    def _get_overlap_score(self, center, mask_info):
        color_mask = mask_info["mask"]
        height, width = color_mask.shape
        x, y = center.astype(int)
        circle_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(circle_mask, (x, y), self.match_radius, 255, -1)

        circle_area = np.count_nonzero(circle_mask)
        if circle_area == 0:
            return 0.0

        overlap = cv2.bitwise_and(color_mask, circle_mask)
        overlap_count = np.count_nonzero(overlap)
        return float(overlap_count / circle_area)

    def _score_candidate_for_color(self, candidate, mask_info):
        center = candidate["center"]
        distance_score, distance_px = self._get_distance_to_mask_score(center, mask_info)
        overlap_score = self._get_overlap_score(center, mask_info)
        weight_sum = self.distance_weight + self.overlap_weight

        if weight_sum <= 0:
            combined_score = 0.0
        else:
            combined_score = (
                self.distance_weight * distance_score
                + self.overlap_weight * overlap_score
            ) / weight_sum

        return {
            "combined_score": combined_score,
            "distance_score": distance_score,
            "overlap_score": overlap_score,
            "distance_px": distance_px,
        }

    def _empty_centers(self, reason):
        self.last_assignment_info = {
            "accepted": False,
            "reason": reason,
        }
        return {"red": None, "green": None, "blue": None, "yellow": None}

    def _get_tracking_score(self, color, candidate):
        if not self.tracking_enabled or not self.last_centers:
            return None

        previous_center = self.last_centers.get(color)
        if previous_center is None:
            return None

        distance_px = float(np.linalg.norm(candidate["center"] - previous_center))
        score = 1.0 - min(distance_px / self.tracking_max_distance_px, 1.0)
        return {
            "score": score,
            "distance_px": distance_px,
        }

    def _combine_color_and_tracking_score(self, color_score, tracking_info):
        if tracking_info is None:
            return color_score

        weight_sum = self.color_score_weight + self.tracking_score_weight
        if weight_sum <= 0:
            return color_score

        return (
            self.color_score_weight * color_score
            + self.tracking_score_weight * tracking_info["score"]
        ) / weight_sum

    def _assign_colors_to_candidates(self, candidates, red_mask, green_mask, blue_mask, yellow_mask):
        if len(candidates) < 4:
            return self._empty_centers("not_enough_candidates")

        color_mask_info = {
            "red": self._prepare_color_mask_info(red_mask),
            "green": self._prepare_color_mask_info(green_mask),
            "blue": self._prepare_color_mask_info(blue_mask),
            "yellow": self._prepare_color_mask_info(yellow_mask),
        }
        colors = ["red", "green", "blue", "yellow"]
        score_table = {}

        for color in colors:
            score_table[color] = []
            for candidate in candidates:
                score_table[color].append(
                    self._score_candidate_for_color(candidate, color_mask_info[color])
                )

        best_total_score = -1.0
        second_best_total_score = -1.0
        best_assignment = None
        best_assignment_details = None
        for permutation in itertools.permutations(range(len(candidates)), 4):
            total_score = 0.0
            assignment_details = {}
            for color, candidate_index in zip(colors, permutation):
                color_score = score_table[color][candidate_index]["combined_score"]
                tracking_info = self._get_tracking_score(color, candidates[candidate_index])
                combined_score = self._combine_color_and_tracking_score(color_score, tracking_info)
                total_score += combined_score
                assignment_details[color] = {
                    "candidate_index": candidate_index,
                    "color_score": color_score,
                    "tracking_info": tracking_info,
                    "combined_score": combined_score,
                }

            if total_score > best_total_score:
                second_best_total_score = best_total_score
                best_total_score = total_score
                best_assignment = dict(zip(colors, permutation))
                best_assignment_details = assignment_details
            elif total_score > second_best_total_score:
                second_best_total_score = total_score

        if best_assignment is None:
            return self._empty_centers("no_assignment")

        color_scores = [
            best_assignment_details[color]["color_score"]
            for color in colors
        ]
        min_color_score = min(color_scores)
        assignment_margin = (
            best_total_score - second_best_total_score
            if second_best_total_score >= 0.0
            else float("inf")
        )

        if min_color_score < self.min_color_score:
            return self._empty_centers("low_color_score")

        if best_total_score < self.min_total_assignment_score:
            return self._empty_centers("low_total_assignment_score")

        if assignment_margin < self.min_assignment_margin:
            return self._empty_centers("ambiguous_assignment")

        self.last_assignment_info = {
            "accepted": True,
            "reason": "ok",
            "total_score": best_total_score,
            "margin": assignment_margin,
            "min_color_score": min_color_score,
            "details": best_assignment_details,
        }

        return {
            "red": candidates[best_assignment["red"]]["center"],
            "green": candidates[best_assignment["green"]]["center"],
            "blue": candidates[best_assignment["blue"]]["center"],
            "yellow": candidates[best_assignment["yellow"]]["center"],
        }

    def _validate_ratio_scale(self, observed_ratio, expected_ratio):
        if observed_ratio <= 0.0 or expected_ratio <= 0.0:
            return False

        scale = observed_ratio / expected_ratio
        return self.min_image_geometry_scale <= scale <= self.max_image_geometry_scale

    def _validate_image_geometry(self, centers):
        if not self.reject_bad_image_geometry:
            return True, None

        required_colors = ("red", "green", "blue", "yellow")
        if any(centers[color] is None for color in required_colors):
            return False, "missing_center"

        pair_distances = {}
        for color_a, color_b in itertools.combinations(required_colors, 2):
            distance_px = float(np.linalg.norm(centers[color_a] - centers[color_b]))
            pair_distances[(color_a, color_b)] = distance_px
            if distance_px < self.min_led_pair_distance_px:
                return False, f"pair_too_close_{color_a}_{color_b}"

        red = centers["red"]
        green = centers["green"]
        blue = centers["blue"]
        yellow = centers["yellow"]
        rgb_area = _triangle_area_2d(red, green, blue)
        if rgb_area < self.min_rgb_triangle_area_px2:
            return False, "rgb_triangle_too_small"

        yellow_red_blue_area = _triangle_area_2d(yellow, red, blue)
        yellow_red_green_area = _triangle_area_2d(yellow, red, green)
        yellow_blue_green_area = _triangle_area_2d(yellow, blue, green)
        if max(yellow_red_blue_area, yellow_red_green_area, yellow_blue_green_area) < self.min_yellow_triangle_area_px2:
            return False, "yellow_geometry_too_flat"

        image_rb = pair_distances[("red", "blue")]
        if image_rb <= 0.0:
            return False, "red_blue_distance_zero"

        if not self._validate_ratio_scale(
            pair_distances[("red", "green")] / image_rb,
            self.red_green_distance / self.red_blue_distance,
        ):
            return False, "red_green_ratio_out_of_range"

        if not self._validate_ratio_scale(
            pair_distances[("green", "blue")] / image_rb,
            self.green_blue_distance / self.red_blue_distance,
        ):
            return False, "green_blue_ratio_out_of_range"

        image_yb = pair_distances[("blue", "yellow")]
        if image_yb <= 0.0:
            return False, "yellow_blue_distance_zero"

        if not self._validate_ratio_scale(
            pair_distances[("red", "yellow")] / image_yb,
            self.yellow_red_distance / self.yellow_blue_distance,
        ):
            return False, "yellow_ratio_out_of_range"

        return True, None

    def _get_camera_matrix(self, frame):
        if self.camera_matrix is not None:
            return self.camera_matrix

        height, width = frame.shape[:2]
        fx = self.approx_focal_length_factor * width
        fy = self.approx_focal_length_factor * width
        cx = width / 2.0
        cy = height / 2.0
        return np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)

    def _get_object_points(self):
        d_rg = self.red_green_distance
        d_rb = self.red_blue_distance
        d_gb = self.green_blue_distance
        d_yr = self.yellow_red_distance
        d_yb = self.yellow_blue_distance
        z_y = self.yellow_plane_offset_z

        red = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        blue = np.array([d_rb, 0.0, 0.0], dtype=np.float32)

        green_x = (d_rg ** 2 - d_gb ** 2 + d_rb ** 2) / (2.0 * d_rb)
        green_y_squared = d_rg ** 2 - green_x ** 2
        green_y = math.sqrt(max(green_y_squared, 0.0))
        green = np.array([green_x, green_y, 0.0], dtype=np.float32)

        yellow_x = (d_yr ** 2 - d_yb ** 2 + d_rb ** 2) / (2.0 * d_rb)
        yellow_y_squared = d_yr ** 2 - yellow_x ** 2 - z_y ** 2
        yellow_y = -math.sqrt(max(yellow_y_squared, 0.0))
        yellow = np.array([yellow_x, yellow_y, z_y], dtype=np.float32)

        rgb_centroid = (red + green + blue) / 3.0
        return np.array([
            red - rgb_centroid,
            green - rgb_centroid,
            blue - rgb_centroid,
            yellow - rgb_centroid,
        ], dtype=np.float32)

    def _solve_pose_with_optional_guess(self, object_points, image_points, camera_matrix):
        if self.use_previous_pose_guess and self.last_rvec is not None and self.last_tvec is not None:
            try:
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    camera_matrix,
                    self.dist_coeffs,
                    rvec=self.last_rvec.copy(),
                    tvec=self.last_tvec.copy(),
                    useExtrinsicGuess=True,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )

                if success:
                    return {
                        "success": True,
                        "rvec": rvec,
                        "tvec": tvec,
                        "method": "ITERATIVE_WITH_PREVIOUS_GUESS",
                    }
            except cv2.error:
                pass

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not success:
            return {"success": False, "rvec": None, "tvec": None, "method": "EPNP_FAILED"}

        try:
            refined_success, refined_rvec, refined_tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                self.dist_coeffs,
                rvec=rvec.copy(),
                tvec=tvec.copy(),
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if refined_success:
                return {
                    "success": True,
                    "rvec": refined_rvec,
                    "tvec": refined_tvec,
                    "method": "EPNP_THEN_ITERATIVE",
                }
        except cv2.error:
            pass

        return {"success": True, "rvec": rvec, "tvec": tvec, "method": "EPNP_ONLY"}

    def _calculate_reprojection_error(self, object_points, image_points, rvec, tvec, camera_matrix):
        projected_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            camera_matrix,
            self.dist_coeffs,
        )
        projected_points = projected_points.reshape(-1, 2)
        errors = np.linalg.norm(projected_points - image_points.reshape(-1, 2), axis=1)
        return {
            "mean_px": float(np.mean(errors)),
            "max_px": float(np.max(errors)),
            "errors_px": errors,
            "projected_points": projected_points,
        }

    def _is_reprojection_accepted(self, reprojection):
        if not self.reject_reprojection_errors:
            return True

        return (
            reprojection["mean_px"] <= self.max_mean_reprojection_error_px
            and reprojection["max_px"] <= self.max_reprojection_error_px
        )

    def _pose_to_camera_coordinates_and_tilt(self, rvec, tvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        normal_local = np.array([[0.0], [0.0], [1.0]], dtype=np.float32)
        normal_camera = (rotation_matrix @ normal_local).flatten()

        if normal_camera[2] < 0:
            normal_camera = -normal_camera

        camera_x_depth = float(tvec[2][0])
        camera_z_vertical = -float(tvec[1][0])
        camera_distance_xz = math.hypot(camera_x_depth, camera_z_vertical)
        tilt_deg = math.degrees(math.atan2(normal_camera[1], normal_camera[2]))
        return camera_x_depth, camera_z_vertical, camera_distance_xz, tilt_deg, normal_camera

    def _estimate_distance_and_tilt(self, centers, frame):
        if any(centers[color] is None for color in ("red", "green", "blue", "yellow")):
            return None

        geometry_ok, geometry_reason = self._validate_image_geometry(centers)
        if not geometry_ok:
            self.last_pose_rejection_reason = geometry_reason
            if self.print_rejected_frames:
                print(f"rejected_image_geometry={geometry_reason}")
            return None

        object_points = self._get_object_points()
        image_points = np.array([
            centers["red"],
            centers["green"],
            centers["blue"],
            centers["yellow"],
        ], dtype=np.float32)
        camera_matrix = self._get_camera_matrix(frame)
        pose_solution = self._solve_pose_with_optional_guess(
            object_points,
            image_points,
            camera_matrix,
        )

        if not pose_solution["success"]:
            return None

        reprojection = self._calculate_reprojection_error(
            object_points,
            image_points,
            pose_solution["rvec"],
            pose_solution["tvec"],
            camera_matrix,
        )
        if not self._is_reprojection_accepted(reprojection):
            self.last_pose_rejection_reason = (
                f"reprojection mean={reprojection['mean_px']:.3f}px "
                f"max={reprojection['max_px']:.3f}px"
            )
            if self.print_rejected_frames:
                print(f"rejected_{self.last_pose_rejection_reason}")
            return None

        camera_x_depth, camera_z_vertical, camera_distance_xz, tilt_deg, normal_camera = self._pose_to_camera_coordinates_and_tilt(
            pose_solution["rvec"],
            pose_solution["tvec"],
        )
        self.last_pose_rejection_reason = None
        return {
            "camera_x_depth": camera_x_depth,
            "camera_z_vertical": camera_z_vertical,
            "camera_distance_xz": camera_distance_xz,
            "distance_z": camera_x_depth,
            "tilt_deg": tilt_deg,
            "normal_camera": normal_camera,
            "rvec": pose_solution["rvec"],
            "tvec": pose_solution["tvec"],
            "method": pose_solution["method"],
            "reprojection_mean_px": reprojection["mean_px"],
            "reprojection_max_px": reprojection["max_px"],
        }

    def _apply_pose_jump_rejection(self, new_pose):
        if new_pose is None:
            return self.last_accepted_pose, False

        if self.last_accepted_pose is None:
            self.consecutive_rejects = 0
            return new_pose, True

        jump_reasons = []

        new_tilt = new_pose["tilt_deg"]
        old_tilt = self.last_accepted_pose["tilt_deg"]
        tilt_jump = abs(_angle_difference_deg(new_tilt, old_tilt))
        if self.reject_tilt_jumps and tilt_jump > self.max_tilt_jump_deg:
            jump_reasons.append(f"tilt={tilt_jump:.3f}deg")

        camera_x_jump = abs(new_pose["camera_x_depth"] - self.last_accepted_pose["camera_x_depth"])
        camera_z_jump = abs(new_pose["camera_z_vertical"] - self.last_accepted_pose["camera_z_vertical"])
        camera_distance_jump = abs(new_pose["camera_distance_xz"] - self.last_accepted_pose["camera_distance_xz"])
        if self.reject_position_jumps:
            if camera_x_jump > self.max_camera_x_jump:
                jump_reasons.append(f"camera_x={camera_x_jump:.3f}")
            if camera_z_jump > self.max_camera_z_jump:
                jump_reasons.append(f"camera_z={camera_z_jump:.3f}")
            if camera_distance_jump > self.max_camera_distance_jump:
                jump_reasons.append(f"camera_distance={camera_distance_jump:.3f}")

        if not jump_reasons:
            self.consecutive_rejects = 0
            return new_pose, True

        self.consecutive_rejects += 1
        if self.print_rejected_frames:
            print(
                f"rejected_pose_jump={','.join(jump_reasons)}, "
                f"new_tilt={new_tilt:.3f}, old_tilt={old_tilt:.3f}, "
                f"rejects={self.consecutive_rejects}"
            )

        if self.consecutive_rejects >= self.max_consecutive_rejects:
            self.consecutive_rejects = 0
            return new_pose, True

        return self.last_accepted_pose, False

    def _draw_marker(self, frame, center, color_bgr, label):
        if center is None:
            return

        x, y = center.astype(int)
        cv2.circle(frame, (x, y), 9, color_bgr, 2)
        cv2.circle(frame, (x, y), 3, color_bgr, -1)
        cv2.putText(
            frame,
            label,
            (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color_bgr,
            2,
            cv2.LINE_AA,
        )

    def _draw_line(self, frame, p1, p2, color_bgr=(255, 255, 255)):
        if p1 is None or p2 is None:
            return

        cv2.line(frame, tuple(p1.astype(int)), tuple(p2.astype(int)), color_bgr, 2)

    def _get_averaged_measurement(self, pose_data, now, fresh):
        if pose_data is None and not self.camera_x_average_samples:
            return None

        if fresh and pose_data is not None:
            _append_capped_sample(
                self.camera_x_average_samples,
                pose_data["camera_x_depth"],
                self.moving_average_n,
            )
            _append_capped_sample(
                self.camera_z_average_samples,
                pose_data["camera_z_vertical"],
                self.moving_average_n,
            )
            _append_capped_sample(
                self.tilt_average_samples,
                pose_data["tilt_deg"],
                self.moving_average_n,
            )

        averaged_camera_x_depth = _average_values(self.camera_x_average_samples)
        averaged_camera_z_vertical = _average_values(self.camera_z_average_samples)
        averaged_tilt_deg = _average_angles_deg(self.tilt_average_samples)

        if averaged_camera_x_depth is None or averaged_camera_z_vertical is None or averaged_tilt_deg is None:
            return None

        averaged_camera_distance_xz = math.hypot(
            averaged_camera_x_depth,
            averaged_camera_z_vertical,
        )
        raw_pose = pose_data if pose_data is not None else {
            "camera_x_depth": averaged_camera_x_depth,
            "camera_z_vertical": averaged_camera_z_vertical,
            "camera_distance_xz": averaged_camera_distance_xz,
            "tilt_deg": averaged_tilt_deg,
            "reprojection_mean_px": None,
            "reprojection_max_px": None,
        }
        measurement = {
            "camera_x_depth": averaged_camera_x_depth,
            "camera_z_vertical": averaged_camera_z_vertical,
            "camera_distance_xz": averaged_camera_distance_xz,
            "distance_z": averaged_camera_x_depth,
            "tilt_deg": averaged_tilt_deg,
            "raw_camera_x_depth": float(raw_pose["camera_x_depth"]),
            "raw_camera_z_vertical": float(raw_pose["camera_z_vertical"]),
            "raw_camera_distance_xz": float(raw_pose["camera_distance_xz"]),
            "raw_distance_z": float(raw_pose["camera_x_depth"]),
            "raw_tilt_deg": float(raw_pose["tilt_deg"]),
            "reprojection_mean_px": raw_pose.get("reprojection_mean_px"),
            "reprojection_max_px": raw_pose.get("reprojection_max_px"),
            "timestamp": now,
            "fresh": bool(fresh),
        }
        if fresh:
            self.camera_x_history.append((now, measurement["camera_x_depth"]))
            self.camera_z_history.append((now, measurement["camera_z_vertical"]))
            self.tilt_history.append((now, measurement["tilt_deg"]))
        return measurement

    def _get_plot_limits(self, history, configured_min, configured_max):
        values = [value for _, value in history]

        if not values:
            y_min = 0.0 if configured_min is None else float(configured_min)
            y_max = 1.0 if configured_max is None else float(configured_max)
            return y_min, y_max

        y_min = min(values) if configured_min is None else float(configured_min)
        y_max = max(values) if configured_max is None else float(configured_max)

        if abs(y_max - y_min) < 1e-9:
            y_min -= 1.0
            y_max += 1.0
        else:
            padding = 0.15 * (y_max - y_min)
            if configured_min is None:
                y_min -= padding
            if configured_max is None:
                y_max += padding

        return y_min, y_max

    def _value_to_plot_y(self, value, y_min, y_max, top, bottom):
        alpha = (value - y_min) / (y_max - y_min)
        alpha = max(0.0, min(1.0, alpha))
        return int(bottom - alpha * (bottom - top))

    def _time_to_plot_x(self, sample_time, current_time, left, right):
        start_time = current_time - self.plot_history_seconds
        alpha = (sample_time - start_time) / self.plot_history_seconds
        alpha = max(0.0, min(1.0, alpha))
        return int(left + alpha * (right - left))

    def _draw_time_series_panel(
        self,
        image,
        history,
        current_time,
        title,
        unit,
        configured_min,
        configured_max,
        top,
        bottom,
        line_color,
    ):
        left = self.plot_margin_left
        right = self.plot_width - self.plot_margin_right
        y_min, y_max = self._get_plot_limits(history, configured_min, configured_max)

        cv2.rectangle(image, (left, top), (right, bottom), (28, 28, 28), -1)
        cv2.rectangle(image, (left, top), (right, bottom), (180, 180, 180), 1)

        vertical_grid_count = 5
        for i in range(vertical_grid_count + 1):
            x = int(left + i * (right - left) / vertical_grid_count)
            cv2.line(image, (x, top), (x, bottom), (55, 55, 55), 1)
            seconds_ago = self.plot_history_seconds * (1.0 - i / vertical_grid_count)
            label = f"-{seconds_ago:.0f}s" if seconds_ago > 0.01 else "0s"
            cv2.putText(
                image,
                label,
                (x - 15, bottom + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (190, 190, 190),
                1,
                cv2.LINE_AA,
            )

        horizontal_grid_count = 4
        for i in range(horizontal_grid_count + 1):
            y = int(top + i * (bottom - top) / horizontal_grid_count)
            cv2.line(image, (left, y), (right, y), (55, 55, 55), 1)
            value = y_max - i * (y_max - y_min) / horizontal_grid_count
            cv2.putText(
                image,
                f"{value:.1f}",
                (8, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (190, 190, 190),
                1,
                cv2.LINE_AA,
            )

        cv2.putText(
            image,
            f"{title} [{unit}]",
            (left, top - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (230, 230, 230),
            2,
            cv2.LINE_AA,
        )

        if len(history) >= 2:
            points = []
            for sample_time, value in history:
                x = self._time_to_plot_x(sample_time, current_time, left, right)
                y = self._value_to_plot_y(value, y_min, y_max, top, bottom)
                points.append([x, y])

            cv2.polylines(image, [np.array(points, dtype=np.int32)], False, line_color, 2, cv2.LINE_AA)

        latest_label = "actual: --"
        if history:
            latest_label = f"actual: {history[-1][1]:.3f} {unit}"

        cv2.putText(
            image,
            latest_label,
            (right - 220, top + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )

    def _create_realtime_plot_image(self, current_time):
        plot_image = np.zeros((self.plot_height, self.plot_width, 3), dtype=np.uint8)
        plot_image[:] = (18, 18, 18)

        cv2.putText(
            plot_image,
            f"Coordenadas camara y tilt vs tiempo | promedio N={self.moving_average_n}",
            (20, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (240, 240, 240),
            2,
            cv2.LINE_AA,
        )

        panel_gap = 45
        available_height = (
            self.plot_height
            - self.plot_margin_top
            - self.plot_margin_bottom
            - 2 * panel_gap
        )
        panel_height = available_height // 3
        camera_x_top = self.plot_margin_top + 25
        camera_x_bottom = camera_x_top + panel_height
        camera_z_top = camera_x_bottom + panel_gap
        camera_z_bottom = camera_z_top + panel_height
        tilt_top = camera_z_bottom + panel_gap
        tilt_bottom = tilt_top + panel_height

        self._draw_time_series_panel(
            plot_image,
            self.camera_x_history,
            current_time,
            "Camara X profundidad",
            "unidad",
            self.camera_x_plot_min,
            self.camera_x_plot_max,
            camera_x_top,
            camera_x_bottom,
            (0, 220, 255),
        )
        self._draw_time_series_panel(
            plot_image,
            self.camera_z_history,
            current_time,
            "Camara Z vertical",
            "unidad",
            self.camera_z_plot_min,
            self.camera_z_plot_max,
            camera_z_top,
            camera_z_bottom,
            (120, 220, 120),
        )
        self._draw_time_series_panel(
            plot_image,
            self.tilt_history,
            current_time,
            "Tilt",
            "deg",
            self.tilt_plot_min,
            self.tilt_plot_max,
            tilt_top,
            tilt_bottom,
            (255, 120, 255),
        )
        return plot_image

    def read(self):
        if self.cap is None:
            self.open()

        ret, frame = self.cap.read()
        now = time.time()
        if not ret:
            return None, None, None, now, "camera_read_failed"

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        light_mask = self._create_mask(hsv, self.light_low_hsv, self.light_high_hsv)
        candidates = self._find_light_candidates(light_mask)
        red_mask = self._create_mask(
            hsv,
            self.red_low_hsv,
            self.red_high_hsv,
            self.red_low_hsv_2,
            self.red_high_hsv_2,
        )
        green_mask = self._create_mask(hsv, self.green_low_hsv, self.green_high_hsv)
        blue_mask = self._create_mask(hsv, self.blue_low_hsv, self.blue_high_hsv)
        yellow_mask = self._create_mask(hsv, self.yellow_low_hsv, self.yellow_high_hsv)

        centers = self._assign_colors_to_candidates(
            candidates,
            red_mask,
            green_mask,
            blue_mask,
            yellow_mask,
        )
        new_pose = self._estimate_distance_and_tilt(centers, frame)
        accepted_pose, was_accepted = self._apply_pose_jump_rejection(new_pose)

        if was_accepted and accepted_pose is not None:
            self.last_accepted_pose = accepted_pose
            self.last_rvec = accepted_pose["rvec"]
            self.last_tvec = accepted_pose["tvec"]
            self.last_centers = {
                color: centers[color].copy()
                for color in ("red", "green", "blue", "yellow")
                if centers[color] is not None
            }

        fresh_measurement = was_accepted and new_pose is not None
        measurement = self._get_averaged_measurement(
            accepted_pose,
            now,
            fresh_measurement,
        )
        _trim_history(self.camera_x_history, now, self.plot_history_seconds)
        _trim_history(self.camera_z_history, now, self.plot_history_seconds)
        _trim_history(self.tilt_history, now, self.plot_history_seconds)

        display = self._create_detection_image(frame, centers, measurement)
        plot_image = self._create_realtime_plot_image(now)
        return measurement, display, plot_image, now, None

    def _create_detection_image(self, frame, centers, measurement):
        display = frame.copy()
        red_center = centers["red"]
        green_center = centers["green"]
        blue_center = centers["blue"]
        yellow_center = centers["yellow"]

        self._draw_marker(display, red_center, (0, 0, 255), "Rojo")
        self._draw_marker(display, green_center, (0, 255, 0), "Verde")
        self._draw_marker(display, blue_center, (255, 0, 0), "Azul")
        self._draw_marker(display, yellow_center, (0, 255, 255), "Amarillo")

        self._draw_line(display, red_center, green_center)
        self._draw_line(display, green_center, blue_center)
        self._draw_line(display, blue_center, red_center)
        self._draw_line(display, red_center, yellow_center, (0, 255, 255))
        self._draw_line(display, blue_center, yellow_center, (0, 255, 255))
        self._draw_line(display, green_center, yellow_center, (160, 160, 160))

        if measurement is not None:
            cv2.putText(
                display,
                f"Cam X depth avg: {measurement['camera_x_depth']:.2f}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                display,
                f"Cam Z vertical avg: {measurement['camera_z_vertical']:.2f}",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                display,
                f"Tilt avg: {measurement['tilt_deg']:.2f} deg",
                (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

        return display

    def show(self, display, plot_image):
        if not self.show_windows or display is None or plot_image is None:
            return None

        cv2.imshow(self.camera_window_name, display)
        cv2.imshow(self.plot_window_name, plot_image)
        return cv2.waitKey(1) & 0xFF

    def run_demo(self):
        self.open()
        try:
            while True:
                measurement, display, plot_image, _now, error = self.read()
                if error:
                    break
                if measurement is not None:
                    print(
                        f"camera_x={measurement['camera_x_depth']:.3f}, "
                        f"camera_z={measurement['camera_z_vertical']:.3f}, "
                        f"tilt={measurement['tilt_deg']:.3f}"
                    )
                key = self.show(display, plot_image)
                if key == 27 or key == ord("q"):
                    break
        finally:
            self.close()
