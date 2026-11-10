import itertools
import math
import time

import cv2
import numpy as np


CONFIG = {
    "camera_index": 1,
    "camera_frame_width": None,
    "camera_frame_height": None,
    "camera_fps": None,
    "camera_auto_exposure": None,
    "camera_exposure": None,
    "camera_gain": None,
    "camera_auto_wb": None,
    "camera_wb_temperature": None,
    "window_name": "Deteccion 2D LEDs rojo/azul",
    "show_debug_masks": False,
    "flip_camera_x": True,
    "reference_point_px": None,
    "invalid_zone_overlay_alpha": 0.55,
    "red_blue_distance": 12.2,
    "mouth_tip_offset_cm": 4.5,
    "flute_safety_radius_cm": 6.0,
    "blur_kernel_size": 5,
    "candidate_mask_mode": "light",
    "light_low_hsv": [80, 0, 200],
    "light_high_hsv": [100, 10, 255],
    "red_low_hsv": [0, 120, 120],
    "red_high_hsv": [10, 255, 255],
    "red_low_hsv_2": [170, 180, 120],
    "red_high_hsv_2": [179, 255, 255],
    "blue_low_hsv": [90, 120, 80],
    "blue_high_hsv": [130, 255, 255],
    "morph_kernel_size": 3,
    "min_light_area": 5,
    "max_light_area": None,
    "min_light_circularity": 0.2,
    "max_light_aspect_ratio": 3.0,
    "max_light_candidates": 6,
    "min_color_area": 10,
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
    "print_every_seconds": 0.1,
}


COLORS = ("red", "blue")
DISPLAY_NAMES = {
    "red": "Rojo",
    "blue": "Azul",
}
DISPLAY_BGR = {
    "red": (0, 0, 255),
    "blue": (255, 0, 0),
}


def as_hsv_array(value):
    return np.array(value, dtype=np.uint8)


def optional_float(value):
    if value is None:
        return None
    return float(value)


class TwoLEDTracker:
    def __init__(self, config):
        self.config = config.copy()
        self.cap = None
        self.last_centers = None
        self.last_assignment_info = {"accepted": False, "reason": "not_started"}
        self.last_print_time = 0.0
        self.reference_point_px = self._parse_reference_point(self.config.get("reference_point_px"))
        self.mouse_callback_ready = False

        self.camera_index = int(self.config["camera_index"])
        self.camera_frame_width = optional_float(self.config.get("camera_frame_width"))
        self.camera_frame_height = optional_float(self.config.get("camera_frame_height"))
        self.camera_fps = optional_float(self.config.get("camera_fps"))
        self.camera_auto_exposure = optional_float(self.config.get("camera_auto_exposure"))
        self.camera_exposure = optional_float(self.config.get("camera_exposure"))
        self.camera_gain = optional_float(self.config.get("camera_gain"))
        self.camera_auto_wb = optional_float(self.config.get("camera_auto_wb"))
        self.camera_wb_temperature = optional_float(self.config.get("camera_wb_temperature"))
        self.window_name = str(self.config["window_name"])
        self.show_debug_masks = bool(self.config["show_debug_masks"])
        self.flip_camera_x = bool(self.config["flip_camera_x"])
        self.invalid_zone_overlay_alpha = float(self.config["invalid_zone_overlay_alpha"])
        self.red_blue_distance = float(self.config["red_blue_distance"])
        self.mouth_tip_offset_cm = float(self.config["mouth_tip_offset_cm"])
        self.flute_safety_radius_cm = float(self.config["flute_safety_radius_cm"])
        self.blur_kernel_size = int(self.config["blur_kernel_size"])
        self.candidate_mask_mode = str(self.config["candidate_mask_mode"])

        self.light_low_hsv = as_hsv_array(self.config["light_low_hsv"])
        self.light_high_hsv = as_hsv_array(self.config["light_high_hsv"])
        self.red_low_hsv = as_hsv_array(self.config["red_low_hsv"])
        self.red_high_hsv = as_hsv_array(self.config["red_high_hsv"])
        self.red_low_hsv_2 = as_hsv_array(self.config["red_low_hsv_2"])
        self.red_high_hsv_2 = as_hsv_array(self.config["red_high_hsv_2"])
        self.blue_low_hsv = as_hsv_array(self.config["blue_low_hsv"])
        self.blue_high_hsv = as_hsv_array(self.config["blue_high_hsv"])

        self.morph_kernel_size = int(self.config["morph_kernel_size"])
        self.min_light_area = float(self.config["min_light_area"])
        self.max_light_area = optional_float(self.config.get("max_light_area"))
        self.min_light_circularity = float(self.config["min_light_circularity"])
        self.max_light_aspect_ratio = float(self.config["max_light_aspect_ratio"])
        self.max_light_candidates = int(self.config["max_light_candidates"])
        self.min_color_area = int(self.config["min_color_area"])
        self.match_radius = int(self.config["match_radius"])
        self.max_match_distance = float(self.config["max_match_distance"])
        self.distance_weight = float(self.config["distance_weight"])
        self.overlap_weight = float(self.config["overlap_weight"])
        self.color_score_weight = float(self.config["color_score_weight"])
        self.tracking_score_weight = float(self.config["tracking_score_weight"])
        self.tracking_enabled = bool(self.config["tracking_enabled"])
        self.tracking_max_distance_px = float(self.config["tracking_max_distance_px"])
        self.min_color_score = float(self.config["min_color_score"])
        self.min_total_assignment_score = float(self.config["min_total_assignment_score"])
        self.min_assignment_margin = float(self.config["min_assignment_margin"])
        self.print_every_seconds = float(self.config["print_every_seconds"])

        self._validate_config()

    def _validate_config(self):
        if self.blur_kernel_size < 1:
            raise ValueError("blur_kernel_size debe ser mayor o igual a 1.")
        if self.blur_kernel_size % 2 == 0:
            raise ValueError("blur_kernel_size debe ser impar.")
        if self.candidate_mask_mode not in {"light", "color", "light_or_color"}:
            raise ValueError("candidate_mask_mode debe ser light, color o light_or_color.")
        if self.invalid_zone_overlay_alpha < 0.0 or self.invalid_zone_overlay_alpha > 1.0:
            raise ValueError("invalid_zone_overlay_alpha debe estar entre 0 y 1.")
        if self.morph_kernel_size < 1:
            raise ValueError("morph_kernel_size debe ser mayor o igual a 1.")
        if self.max_light_candidates < 2:
            raise ValueError("max_light_candidates debe ser mayor o igual a 2.")
        if self.min_light_area <= 0:
            raise ValueError("min_light_area debe ser mayor que cero.")
        if self.max_light_area is not None and self.max_light_area <= self.min_light_area:
            raise ValueError("max_light_area debe ser mayor que min_light_area.")
        if not 0.0 <= self.min_light_circularity <= 1.0:
            raise ValueError("min_light_circularity debe estar entre 0 y 1.")
        if self.max_light_aspect_ratio < 1.0:
            raise ValueError("max_light_aspect_ratio debe ser mayor o igual a 1.")
        if self.max_match_distance <= 0:
            raise ValueError("max_match_distance debe ser mayor que cero.")
        if self.tracking_max_distance_px <= 0:
            raise ValueError("tracking_max_distance_px debe ser mayor que cero.")
        if self.min_color_score < 0.0:
            raise ValueError("min_color_score debe ser mayor o igual a cero.")

    def _parse_reference_point(self, value):
        if value is None:
            return None
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError("reference_point_px debe ser None o [x, y].")
        return np.array([float(value[0]), float(value[1])], dtype=np.float32)

    def _get_reference_point(self, frame):
        height, width = frame.shape[:2]
        if self.reference_point_px is None:
            return np.array([width - 1, height - 1], dtype=np.float32)

        x = min(max(float(self.reference_point_px[0]), 0.0), float(width - 1))
        y = min(max(float(self.reference_point_px[1]), 0.0), float(height - 1))
        return np.array([x, y], dtype=np.float32)

    def _apply_reference_valid_area(self, mask, reference_point):
        clipped = mask.copy()
        ref_x, ref_y = reference_point.astype(int)
        height, width = clipped.shape[:2]

        if ref_x + 1 < width:
            clipped[:, ref_x + 1:] = 0
        if ref_y + 1 < height:
            clipped[ref_y + 1:, :] = 0

        return clipped

    def _draw_invalid_zone_overlay(self, frame, reference_point):
        alpha = self.invalid_zone_overlay_alpha
        if alpha <= 0.0:
            return

        ref_x, ref_y = reference_point.astype(int)
        height, width = frame.shape[:2]
        overlay = frame.copy()

        if ref_x + 1 < width:
            cv2.rectangle(overlay, (ref_x + 1, 0), (width - 1, height - 1), (0, 0, 0), -1)
        if ref_y + 1 < height:
            cv2.rectangle(overlay, (0, ref_y + 1), (width - 1, height - 1), (0, 0, 0), -1)

        cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)

    def _on_camera_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.reference_point_px = np.array([x, y], dtype=np.float32)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.reference_point_px = None

    def _ensure_mouse_callback(self):
        if self.mouse_callback_ready:
            return

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._on_camera_mouse)
        self.mouse_callback_ready = True

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

        cv2.destroyAllWindows()
        self.mouse_callback_ready = False

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

        candidates.sort(key=lambda candidate: candidate["area"], reverse=True)
        return candidates[:self.max_light_candidates]

    def _build_candidate_mask(self, light_mask, red_mask, blue_mask):
        color_mask = cv2.bitwise_or(red_mask, blue_mask)

        if self.candidate_mask_mode == "light":
            return light_mask
        if self.candidate_mask_mode == "color":
            return color_mask

        return cv2.bitwise_or(light_mask, color_mask)

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

    def read(self):
        if self.cap is None:
            self.open()

        ret, frame = self.cap.read()
        now = time.time()
        if not ret:
            return None, None, now, "camera_read_failed"

        if self.flip_camera_x:
            frame = cv2.flip(frame, 1)

        reference_point = self._get_reference_point(frame)
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
        light_mask = self._apply_reference_valid_area(light_mask, reference_point)
        red_mask = self._apply_reference_valid_area(red_mask, reference_point)
        blue_mask = self._apply_reference_valid_area(blue_mask, reference_point)

        candidate_mask = self._build_candidate_mask(light_mask, red_mask, blue_mask)
        candidates = self._find_light_candidates(candidate_mask)
        centers = self._assign_colors_to_candidates(candidates, red_mask, blue_mask)

        if self.last_assignment_info.get("accepted"):
            self.last_centers = {
                color: centers[color].copy()
                for color in COLORS
                if centers[color] is not None
            }

        display = self._create_detection_image(frame, candidates, centers, reference_point)
        if self.show_debug_masks:
            cv2.imshow("Mascara candidatos", candidate_mask)
            cv2.imshow("Mascara luz", light_mask)
            cv2.imshow("Mascara rojo", red_mask)
            cv2.imshow("Mascara azul", blue_mask)

        self._print_positions(centers, now)
        return centers, display, now, None

    def _print_positions(self, centers, now):
        if now - self.last_print_time < self.print_every_seconds:
            return

        self.last_print_time = now
        red = self._format_center(centers["red"])
        blue = self._format_center(centers["blue"])
        reason = self.last_assignment_info.get("reason", "unknown")
        print(f"rojo={red}, azul={blue}, estado={reason}")

    def _format_center(self, center):
        if center is None:
            return "None"

        x, y = center.astype(int)
        return f"({x}, {y})"

    def _fold_led_angle_deg(self, red_center, blue_center):
        if red_center is None or blue_center is None:
            return None

        vector = blue_center - red_center
        angle_deg = math.degrees(math.atan2(float(vector[1]), float(vector[0])))
        angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
        if angle_deg > 90.0:
            angle_deg -= 180.0
        elif angle_deg <= -90.0:
            angle_deg += 180.0
        return angle_deg

    def _get_2d_geometry(self, frame, centers, reference_point):
        red_center = centers["red"]
        blue_center = centers["blue"]
        if red_center is None or blue_center is None:
            return None

        led_distance_px = float(np.linalg.norm(blue_center - red_center))
        if led_distance_px <= 1e-9:
            return None

        flute_center_px = reference_point
        cm_per_px = self.red_blue_distance / led_distance_px
        px_per_cm = 1.0 / cm_per_px
        angle_deg = self._fold_led_angle_deg(red_center, blue_center)
        angle_rad = math.radians(angle_deg)
        direction = np.array([math.cos(angle_rad), math.sin(angle_rad)], dtype=np.float32)
        robot_point = red_center - direction * (self.mouth_tip_offset_cm * px_per_cm)
        safety_radius_px = self.flute_safety_radius_cm * px_per_cm
        robot_distance_px = float(np.linalg.norm(robot_point - flute_center_px))

        return {
            "flute_center_px": flute_center_px,
            "robot_point_px": robot_point,
            "safety_radius_px": safety_radius_px,
            "robot_distance_cm": robot_distance_px * cm_per_px,
            "cm_per_px": cm_per_px,
            "angle_deg": angle_deg,
            "safe": robot_distance_px >= safety_radius_px,
        }

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

    def _draw_marker(self, frame, center, color_bgr, label):
        if center is None:
            return

        x, y = center.astype(int)
        cv2.circle(frame, (x, y), 9, color_bgr, 2)
        cv2.circle(frame, (x, y), 3, color_bgr, -1)
        cv2.putText(
            frame,
            f"{label} ({x}, {y})",
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

    def _draw_reference_geometry(self, frame, geometry):
        if geometry is None:
            return

        center = tuple(geometry["flute_center_px"].astype(int))
        robot = tuple(geometry["robot_point_px"].astype(int))
        safety_radius = int(round(geometry["safety_radius_px"]))
        safety_color = (60, 220, 60) if geometry["safe"] else (80, 80, 255)

        cv2.circle(frame, center, safety_radius, safety_color, 2)
        cv2.circle(frame, center, 7, (255, 255, 255), 2)
        cv2.circle(frame, robot, 7, safety_color, -1)
        cv2.line(frame, center, robot, safety_color, 1, cv2.LINE_AA)
        cv2.putText(
            frame,
            "Centro flauta",
            (max(0, center[0] - 130), max(18, center[1] - 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "X/Z robot",
            (robot[0] + 10, robot[1] + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            safety_color,
            2,
            cv2.LINE_AA,
        )

    def _create_detection_image(self, frame, candidates, centers, reference_point):
        display = frame.copy()
        self._draw_invalid_zone_overlay(display, reference_point)
        self._draw_candidate_contours(display, candidates)

        red_center = centers["red"]
        blue_center = centers["blue"]
        geometry = self._get_2d_geometry(display, centers, reference_point)

        self._draw_reference_geometry(display, geometry)
        self._draw_line(display, red_center, blue_center)
        self._draw_marker(display, red_center, DISPLAY_BGR["red"], "Rojo/punta")
        self._draw_marker(display, blue_center, DISPLAY_BGR["blue"], DISPLAY_NAMES["blue"])
        self._draw_status(display, centers, len(candidates), geometry, reference_point)
        return display

    def _draw_status(self, frame, centers, candidate_count, geometry, reference_point):
        info = self.last_assignment_info
        accepted = bool(info.get("accepted"))
        status_color = (60, 220, 60) if accepted else (80, 80, 255)
        angle_label = "Angulo: --"
        scale_label = "Escala: --"
        safety_label = "Seguridad: --"
        if geometry is not None:
            angle_label = f"Angulo: {geometry['angle_deg']:.2f} deg"
            scale_label = f"Escala: {geometry['cm_per_px']:.5f} cm/px"
            safety_label = f"Seguridad: {geometry['robot_distance_cm']:.2f} cm"
            if not geometry["safe"]:
                safety_label += " (limite)"
        rows = [
            f"Rojo: {self._format_center(centers['red'])}",
            f"Azul: {self._format_center(centers['blue'])}",
            f"Centro: {self._format_center(reference_point)}",
            angle_label,
            scale_label,
            safety_label,
            f"Candidatos: {candidate_count}",
            f"Estado: {info.get('reason', 'unknown')}",
        ]

        x0, y0 = 18, 24
        line_height = 24
        box_width = 390
        box_height = line_height * len(rows) + 18
        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (x0 - 10, y0 - 20),
            (x0 + box_width, y0 - 20 + box_height),
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
                0.62,
                color,
                2,
                cv2.LINE_AA,
            )

    def show(self, display):
        self._ensure_mouse_callback()
        cv2.imshow(self.window_name, display)
        return cv2.waitKey(1) & 0xFF

    def run(self):
        self.open()
        try:
            while True:
                _centers, display, _now, error = self.read()
                if error:
                    print(error)
                    break

                key = self.show(display)
                if key == 27 or key == ord("q"):
                    break
        finally:
            self.close()


def main():
    tracker = TwoLEDTracker(CONFIG)
    tracker.run()


if __name__ == "__main__":
    main()
