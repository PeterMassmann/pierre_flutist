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
    "flip_camera_x": True,
    "tip_color": "red",
    "red_blue_distance": 12.2,
    "flute_safety_radius_cm": 6.0,
    "light_low_hsv": [0, 0, 200],
    "light_high_hsv": [179, 10, 255],
    "red_low_hsv": [0, 120, 120],
    "red_high_hsv": [10, 255, 255],
    "red_low_hsv_2": [170, 120, 120],
    "red_high_hsv_2": [179, 255, 255],
    "blue_low_hsv": [90, 80, 80],
    "blue_high_hsv": [130, 255, 255],
    "candidate_mask_mode": "light",
    "blur_kernel_size": 3,
    "min_light_area": 8,
    "max_light_area": None,
    "min_light_circularity": 0.2,
    "max_light_aspect_ratio": 3.0,
    "max_light_candidates": 6,
    "min_color_area": 10,
    "morph_kernel_size": 5,
    "match_radius": 25,
    "max_match_distance": 80.0,
    "distance_weight": 0.5,
    "overlap_weight": 0.5,
    "color_score_weight": 1.0,
    "tracking_score_weight": 0.35,
    "tracking_enabled": True,
    "tracking_max_distance_px": 80.0,
    "min_color_score": 0.08,
    "min_total_assignment_score": 0.35,
    "min_assignment_margin": 0.02,
    "moving_average_n": 10,
    "reject_tilt_jumps": False,
    "max_tilt_jump_deg": 6.0,
    "reject_position_jumps": True,
    "max_camera_x_jump": 8.0,
    "max_camera_z_jump": 8.0,
    "max_camera_distance_jump": 10.0,
    "max_consecutive_rejects": 8,
    "print_rejected_frames": True,
    "min_led_pair_distance_px": 3.0,
    "camera_window_name": "Camara original con deteccion",
    "plot_window_name": "Graficos coordenadas camara y angulo",
    "show_windows": True,
    "plot_width": 900,
    "plot_height": 650,
    "plot_history_seconds": 10.0,
    "plot_margin_left": 65,
    "plot_margin_right": 25,
    "plot_margin_top": 35,
    "plot_margin_bottom": 40,
    "camera_x_plot_min": -10.0,
    "camera_x_plot_max": 0.0,
    "camera_z_plot_min": -10.0,
    "camera_z_plot_max": 0.0,
    "tilt_plot_min": -90.0,
    "tilt_plot_max": 90.0,
}


COLORS = ("red", "blue")


def merged_led_tracker_config(config=None):
    merged = DEFAULT_LED_TRACKER_CONFIG.copy()
    if config:
        merged.update(config)
    return merged


def _as_hsv_array(value):
    return np.array(value, dtype=np.uint8)


def _optional_float(value):
    if value is None:
        return None
    return float(value)


def _angle_difference_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


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
    Detecta dos LEDs (rojo y azul) y estima posicion lateral 2D en cm.

    La referencia de camara es la esquina inferior derecha. X positivo apunta a
    la derecha de la imagen y Z positivo hacia abajo; por lo tanto, en el
    cuadrante visible desde esa esquina, X y Z normalmente son negativos.
    """

    def __init__(self, config=None):
        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) no esta instalado.")

        self.config = merged_led_tracker_config(config)
        self._read_config()
        self._validate_config()

        self.cap = None
        self.last_accepted_pose = None
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
        self.flip_camera_x = bool(cfg["flip_camera_x"])
        self.tip_color = str(cfg["tip_color"])

        self.red_blue_distance = float(cfg["red_blue_distance"])
        self.flute_safety_radius_cm = float(cfg["flute_safety_radius_cm"])

        self.light_low_hsv = _as_hsv_array(cfg["light_low_hsv"])
        self.light_high_hsv = _as_hsv_array(cfg["light_high_hsv"])
        self.red_low_hsv = _as_hsv_array(cfg["red_low_hsv"])
        self.red_high_hsv = _as_hsv_array(cfg["red_high_hsv"])
        self.red_low_hsv_2 = _as_hsv_array(cfg["red_low_hsv_2"])
        self.red_high_hsv_2 = _as_hsv_array(cfg["red_high_hsv_2"])
        self.blue_low_hsv = _as_hsv_array(cfg["blue_low_hsv"])
        self.blue_high_hsv = _as_hsv_array(cfg["blue_high_hsv"])

        self.candidate_mask_mode = str(cfg["candidate_mask_mode"])
        self.blur_kernel_size = int(cfg["blur_kernel_size"])
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

        self.reject_tilt_jumps = bool(cfg["reject_tilt_jumps"])
        self.max_tilt_jump_deg = float(cfg["max_tilt_jump_deg"])
        self.reject_position_jumps = bool(cfg["reject_position_jumps"])
        self.max_camera_x_jump = float(cfg["max_camera_x_jump"])
        self.max_camera_z_jump = float(cfg["max_camera_z_jump"])
        self.max_camera_distance_jump = float(cfg["max_camera_distance_jump"])
        self.max_consecutive_rejects = int(cfg["max_consecutive_rejects"])
        self.print_rejected_frames = bool(cfg["print_rejected_frames"])
        self.min_led_pair_distance_px = float(cfg["min_led_pair_distance_px"])

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
        self.camera_x_plot_min = cfg.get("camera_x_plot_min")
        self.camera_x_plot_max = cfg.get("camera_x_plot_max")
        self.camera_z_plot_min = cfg.get("camera_z_plot_min")
        self.camera_z_plot_max = cfg.get("camera_z_plot_max")
        self.tilt_plot_min = cfg.get("tilt_plot_min")
        self.tilt_plot_max = cfg.get("tilt_plot_max")

    def _validate_config(self):
        if self.red_blue_distance <= 0:
            raise ValueError("red_blue_distance debe ser mayor que cero.")
        if self.flute_safety_radius_cm < 0:
            raise ValueError("flute_safety_radius_cm debe ser mayor o igual a cero.")
        if self.tip_color not in COLORS:
            raise ValueError("tip_color debe ser red o blue.")
        if self.blur_kernel_size < 1 or self.blur_kernel_size % 2 == 0:
            raise ValueError("blur_kernel_size debe ser impar y mayor o igual a 1.")
        if self.candidate_mask_mode not in {"light", "color", "light_or_color"}:
            raise ValueError("candidate_mask_mode debe ser light, color o light_or_color.")
        if self.morph_kernel_size < 1:
            raise ValueError("morph_kernel_size debe ser mayor o igual a 1.")
        if self.max_light_candidates < 2:
            raise ValueError("max_light_candidates debe ser mayor o igual a 2.")
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
        if self.moving_average_n < 1:
            raise ValueError("moving_average_n debe ser mayor o igual a 1.")
        if self.min_led_pair_distance_px <= 0:
            raise ValueError("min_led_pair_distance_px debe ser mayor que cero.")

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

    def _build_candidate_mask(self, light_mask, red_mask, blue_mask):
        color_mask = cv2.bitwise_or(red_mask, blue_mask)

        if self.candidate_mask_mode == "light":
            return light_mask
        if self.candidate_mask_mode == "color":
            return color_mask

        return cv2.bitwise_or(light_mask, color_mask)

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
        return {"red": None, "blue": None}

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

    def _assign_colors_to_candidates(self, candidates, red_mask, blue_mask):
        if len(candidates) < 2:
            return self._empty_centers("not_enough_candidates")

        color_mask_info = {
            "red": self._prepare_color_mask_info(red_mask),
            "blue": self._prepare_color_mask_info(blue_mask),
        }
        score_table = {}

        for color in COLORS:
            score_table[color] = []
            for candidate in candidates:
                score_table[color].append(
                    self._score_candidate_for_color(candidate, color_mask_info[color])
                )

        best_total_score = -1.0
        second_best_total_score = -1.0
        best_assignment = None
        best_assignment_details = None

        for permutation in itertools.permutations(range(len(candidates)), len(COLORS)):
            total_score = 0.0
            assignment_details = {}
            for color, candidate_index in zip(COLORS, permutation):
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
                best_assignment = dict(zip(COLORS, permutation))
                best_assignment_details = assignment_details
            elif total_score > second_best_total_score:
                second_best_total_score = total_score

        if best_assignment is None:
            return self._empty_centers("no_assignment")

        color_scores = [
            best_assignment_details[color]["color_score"]
            for color in COLORS
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
            color: candidates[best_assignment[color]]["center"]
            for color in COLORS
        }

    def _estimate_2d_pose(self, centers, frame):
        red_center = centers["red"]
        blue_center = centers["blue"]
        if red_center is None or blue_center is None:
            return None

        frame_height, frame_width = frame.shape[:2]
        reference_corner = np.array([frame_width - 1, frame_height - 1], dtype=np.float32)
        tip_center = centers[self.tip_color]
        tail_color = "blue" if self.tip_color == "red" else "red"
        tail_center = centers[tail_color]
        led_vector_px = tail_center - tip_center
        led_distance_px = float(np.linalg.norm(led_vector_px))
        if led_distance_px < self.min_led_pair_distance_px:
            self.last_pose_rejection_reason = "led_pair_too_close"
            if self.print_rejected_frames:
                print(f"rejected_image_geometry={self.last_pose_rejection_reason}")
            return None

        cm_per_px = self.red_blue_distance / led_distance_px
        tip_from_corner_px = tip_center - reference_corner
        camera_x_cm = float(tip_from_corner_px[0] * cm_per_px)
        camera_z_cm = float(tip_from_corner_px[1] * cm_per_px)
        camera_distance_cm = math.hypot(camera_x_cm, camera_z_cm)
        angle_deg = math.degrees(math.atan2(float(led_vector_px[1]), float(led_vector_px[0])))
        angle_deg = _angle_difference_deg(angle_deg, 0.0)
        if angle_deg > 90.0:
            angle_deg -= 180.0
        elif angle_deg <= -90.0:
            angle_deg += 180.0

        self.last_pose_rejection_reason = None
        return {
            "camera_x_depth": camera_x_cm,
            "camera_z_vertical": camera_z_cm,
            "camera_distance_xz": camera_distance_cm,
            "distance_z": camera_x_cm,
            "tilt_deg": angle_deg,
            "raw_camera_x_depth": camera_x_cm,
            "raw_camera_z_vertical": camera_z_cm,
            "raw_camera_distance_xz": camera_distance_cm,
            "raw_distance_z": camera_x_cm,
            "raw_tilt_deg": angle_deg,
            "cm_per_px": cm_per_px,
            "led_distance_px": led_distance_px,
            "reference_corner_px": reference_corner,
            "red_center_px": red_center.copy(),
            "blue_center_px": blue_center.copy(),
            "tip_color": self.tip_color,
            "tip_center_px": tip_center.copy(),
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
            jump_reasons.append(f"angle={tilt_jump:.3f}deg")

        camera_x_jump = abs(new_pose["camera_x_depth"] - self.last_accepted_pose["camera_x_depth"])
        camera_z_jump = abs(new_pose["camera_z_vertical"] - self.last_accepted_pose["camera_z_vertical"])
        camera_distance_jump = abs(new_pose["camera_distance_xz"] - self.last_accepted_pose["camera_distance_xz"])
        if self.reject_position_jumps:
            if camera_x_jump > self.max_camera_x_jump:
                jump_reasons.append(f"camera_x={camera_x_jump:.3f}cm")
            if camera_z_jump > self.max_camera_z_jump:
                jump_reasons.append(f"camera_z={camera_z_jump:.3f}cm")
            if camera_distance_jump > self.max_camera_distance_jump:
                jump_reasons.append(f"camera_distance={camera_distance_jump:.3f}cm")

        if not jump_reasons:
            self.consecutive_rejects = 0
            return new_pose, True

        self.consecutive_rejects += 1
        if self.print_rejected_frames:
            print(
                f"rejected_pose_jump={','.join(jump_reasons)}, "
                f"new_angle={new_tilt:.3f}, old_angle={old_tilt:.3f}, "
                f"rejects={self.consecutive_rejects}"
            )

        if self.consecutive_rejects >= self.max_consecutive_rejects:
            self.consecutive_rejects = 0
            return new_pose, True

        return self.last_accepted_pose, False

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
            "cm_per_px": None,
            "led_distance_px": None,
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
            "cm_per_px": raw_pose.get("cm_per_px"),
            "led_distance_px": raw_pose.get("led_distance_px"),
            "reference_corner_px": raw_pose.get("reference_corner_px"),
            "red_center_px": raw_pose.get("red_center_px"),
            "blue_center_px": raw_pose.get("blue_center_px"),
            "tip_color": raw_pose.get("tip_color", self.tip_color),
            "tip_center_px": raw_pose.get("tip_center_px"),
            "reprojection_mean_px": None,
            "reprojection_max_px": None,
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
            f"Coordenadas camara 2D y angulo vs tiempo | promedio N={self.moving_average_n}",
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
            "Camara X desde esquina",
            "cm",
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
            "Camara Z desde esquina",
            "cm",
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
            "Angulo LEDs",
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

        if self.flip_camera_x:
            frame = cv2.flip(frame, 1)

        blurred = cv2.GaussianBlur(
            frame,
            (self.blur_kernel_size, self.blur_kernel_size),
            0,
        )
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        light_mask = self._create_mask(hsv, self.light_low_hsv, self.light_high_hsv)
        red_mask = self._create_mask(
            hsv,
            self.red_low_hsv,
            self.red_high_hsv,
            self.red_low_hsv_2,
            self.red_high_hsv_2,
        )
        blue_mask = self._create_mask(hsv, self.blue_low_hsv, self.blue_high_hsv)
        candidate_mask = self._build_candidate_mask(light_mask, red_mask, blue_mask)

        candidates = self._find_light_candidates(candidate_mask)
        centers = self._assign_colors_to_candidates(candidates, red_mask, blue_mask)
        new_pose = self._estimate_2d_pose(centers, frame)
        accepted_pose, was_accepted = self._apply_pose_jump_rejection(new_pose)

        if was_accepted and accepted_pose is not None:
            self.last_accepted_pose = accepted_pose
            self.last_centers = {
                color: centers[color].copy()
                for color in COLORS
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

        display = self._create_detection_image(frame, candidates, centers, measurement)
        plot_image = self._create_realtime_plot_image(now)
        return measurement, display, plot_image, now, None

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

    def _draw_candidate_contours(self, frame, candidates):
        for index, candidate in enumerate(candidates, start=1):
            cv2.drawContours(frame, [candidate["contour"]], -1, (120, 120, 120), 1)
            x, y = candidate["center"].astype(int)
            cv2.putText(
                frame,
                str(index),
                (x + 6, y + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )

    def _draw_reference_corner(self, frame, measurement):
        height, width = frame.shape[:2]
        origin = (width - 1, height - 1)
        safety_color = (0, 220, 255)
        cm_per_px = measurement.get("cm_per_px") if measurement is not None else None

        if (
            cm_per_px is not None
            and math.isfinite(float(cm_per_px))
            and float(cm_per_px) > 0.0
            and self.flute_safety_radius_cm > 0.0
        ):
            safety_radius_px = int(round(self.flute_safety_radius_cm / float(cm_per_px)))
            cv2.circle(frame, origin, safety_radius_px, safety_color, 2)
            cv2.putText(
                frame,
                f"Radio seguridad {self.flute_safety_radius_cm:.1f} cm",
                (max(0, origin[0] - 245), max(18, origin[1] - safety_radius_px - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                safety_color,
                2,
                cv2.LINE_AA,
            )

        cv2.circle(frame, origin, 8, (255, 255, 255), 2)
        cv2.line(frame, (origin[0] - 35, origin[1]), origin, (255, 255, 255), 1)
        cv2.line(frame, (origin[0], origin[1] - 35), origin, (255, 255, 255), 1)
        cv2.putText(
            frame,
            "0,0",
            (max(0, origin[0] - 48), max(18, origin[1] - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def _format_center(self, center):
        if center is None:
            return "None"

        x, y = center.astype(int)
        return f"({x}, {y})"

    def _create_detection_image(self, frame, candidates, centers, measurement):
        display = frame.copy()
        red_center = centers["red"]
        blue_center = centers["blue"]

        self._draw_candidate_contours(display, candidates)
        self._draw_reference_corner(display, measurement)
        self._draw_line(display, blue_center, red_center)
        red_label = "Rojo/punta" if self.tip_color == "red" else "Rojo"
        blue_label = "Azul/punta" if self.tip_color == "blue" else "Azul"
        self._draw_marker(display, red_center, (0, 0, 255), red_label)
        self._draw_marker(display, blue_center, (255, 0, 0), blue_label)

        self._draw_status(display, centers, measurement, len(candidates))
        return display

    def _draw_status(self, frame, centers, measurement, candidate_count):
        assignment_reason = self.last_assignment_info.get("reason", "unknown")
        pose_reason = self.last_pose_rejection_reason
        status = pose_reason or assignment_reason
        status_ok = measurement is not None and measurement.get("fresh", False)
        status_color = (60, 220, 60) if status_ok else (80, 80, 255)

        if measurement is None:
            x_label = "X cm: --"
            z_label = "Z cm: --"
            angle_label = "Alpha cam: --"
            scale_label = "Escala: --"
            safety_label = "Radio seguridad: --"
        else:
            x_label = f"X cm: {measurement['camera_x_depth']:.3f}"
            z_label = f"Z cm: {measurement['camera_z_vertical']:.3f}"
            angle_label = f"Alpha cam: {measurement['tilt_deg']:.3f} deg"
            cm_per_px = measurement.get("cm_per_px")
            scale_label = "Escala: --" if cm_per_px is None else f"Escala: {cm_per_px:.5f} cm/px"
            safety_label = f"Radio seguridad: {self.flute_safety_radius_cm:.1f} cm"

        rows = [
            x_label,
            z_label,
            angle_label,
            scale_label,
            safety_label,
            f"Rojo px: {self._format_center(centers['red'])}",
            f"Azul px: {self._format_center(centers['blue'])}",
            f"Punta: {self.tip_color}",
            f"Candidatos: {candidate_count}",
            f"Estado: {status}",
        ]

        x0, y0 = 18, 28
        line_height = 24
        box_width = 390
        box_height = line_height * len(rows) + 18
        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (x0 - 10, y0 - 22),
            (x0 + box_width, y0 - 22 + box_height),
            (0, 0, 0),
            -1,
        )
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

        for index, row in enumerate(rows):
            color = status_color if row.startswith("Estado:") else (255, 255, 255)
            cv2.putText(
                frame,
                row,
                (x0, y0 + index * line_height),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

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
                        f"camera_x_cm={measurement['camera_x_depth']:.3f}, "
                        f"camera_z_cm={measurement['camera_z_vertical']:.3f}, "
                        f"angle={measurement['tilt_deg']:.3f}"
                    )
                key = self.show(display, plot_image)
                if key == 27 or key == ord("q"):
                    break
        finally:
            self.close()
