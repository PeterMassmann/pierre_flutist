import cv2
import numpy as np
import math
import time
import itertools


# ============================================================
# VARIABLES DE CONTROL
# ============================================================

CAMERA_INDEX = 1

# Distancias reales entre LEDs.
# La unidad que uses aquí será la unidad de la distancia impresa.
#
# Convención física actual:
#
#            verde
#
#      rojo          azul
#
RED_GREEN_DISTANCE = 9.0
RED_BLUE_DISTANCE = 12.2
GREEN_BLUE_DISTANCE = 9

# ------------------------------------------------------------
# Parámetros de cámara
# ------------------------------------------------------------

# Si tienes calibración real, pon aquí los valores correctos:
#
# CAMERA_MATRIX = np.array([
#     [fx,  0, cx],
#     [ 0, fy, cy],
#     [ 0,  0,  1]
# ], dtype=np.float32)
#
# DIST_COEFFS = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

CAMERA_MATRIX = np.array([
    [913.94,   0.00, 640.00],
    [  0.00, 913.94, 360.00],
    [  0.00,   0.00,   1.00]
], dtype=np.float32)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)

# Si no tienes calibración, el programa aproxima fx y fy.
# Esto permite que funcione, pero la distancia y el tilt no serán precisos.
APPROX_FOCAL_LENGTH_FACTOR = 1.0

# ------------------------------------------------------------
# Control de solvePnP temporal
# ------------------------------------------------------------

# Opción 2:
# Si hay una pose anterior válida, se usa como punto de partida para el
# cálculo nuevo con SOLVEPNP_ITERATIVE.
USE_PREVIOUS_POSE_GUESS = True

# Opción 3:
# Si el tilt cambia más que esto entre frames, se rechaza la medición nueva.
REJECT_TILT_JUMPS = True
MAX_TILT_JUMP_DEG = 6.0

# Si se rechazan demasiados frames seguidos, se acepta una nueva pose para
# evitar que el sistema quede "pegado" si la cabeza realmente se movió rápido.
MAX_CONSECUTIVE_REJECTS = 8

# Si quieres ver cuándo rechaza frames, cambia esto a True.
PRINT_REJECTED_FRAMES = False

# ------------------------------------------------------------
# Filtro bueno para encontrar los centros de luz
# ------------------------------------------------------------

LIGHT_LOW_HSV = np.array((80, 0, 200))
LIGHT_HIGH_HSV = np.array((100, 10, 255))

# ------------------------------------------------------------
# Filtros de color, usados solo para clasificar cada centro
# ------------------------------------------------------------

RED_LOW_HSV = np.array([0, 120, 120])
RED_HIGH_HSV = np.array([10, 255, 255])

RED_LOW_HSV_2 = np.array([170, 120, 120])
RED_HIGH_HSV_2 = np.array([179, 255, 255])

GREEN_LOW_HSV = np.array([35, 80, 80])
GREEN_HIGH_HSV = np.array([85, 255, 255])

BLUE_LOW_HSV = np.array([90, 80, 80])
BLUE_HIGH_HSV = np.array([130, 255, 255])

# ------------------------------------------------------------
# Parámetros de detección
# ------------------------------------------------------------

MIN_LIGHT_AREA = 20
MAX_LIGHT_CANDIDATES = 3

MIN_COLOR_AREA = 10

MORPH_KERNEL_SIZE = 5

MATCH_RADIUS = 25
MAX_MATCH_DISTANCE = 80

DISTANCE_WEIGHT = 0.5
OVERLAP_WEIGHT = 0.5

PRINT_EVERY_SECONDS = 0.1


# ============================================================
# FUNCIONES AUXILIARES
# ============================================================

def create_mask(hsv_frame, low_hsv, high_hsv, low_hsv_2=None, high_hsv_2=None):
    mask = cv2.inRange(hsv_frame, low_hsv, high_hsv)

    if low_hsv_2 is not None and high_hsv_2 is not None:
        mask_2 = cv2.inRange(hsv_frame, low_hsv_2, high_hsv_2)
        mask = cv2.bitwise_or(mask, mask_2)

    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def find_light_candidates(light_mask):
    contours, _ = cv2.findContours(
        light_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < MIN_LIGHT_AREA:
            continue

        moments = cv2.moments(contour)

        if moments["m00"] == 0:
            continue

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])

        candidates.append({
            "center": np.array([cx, cy], dtype=np.float32),
            "area": area,
            "contour": contour
        })

    candidates.sort(key=lambda c: c["area"], reverse=True)

    return candidates[:MAX_LIGHT_CANDIDATES]


def get_distance_to_mask_score(center, color_mask):
    x, y = center.astype(int)

    height, width = color_mask.shape

    if x < 0 or x >= width or y < 0 or y >= height:
        return 0.0, float("inf")

    binary = (color_mask > 0).astype(np.uint8)

    if np.count_nonzero(binary) < MIN_COLOR_AREA:
        return 0.0, float("inf")

    inverted = 1 - binary

    distance_map = cv2.distanceTransform(
        inverted,
        cv2.DIST_L2,
        cv2.DIST_MASK_PRECISE
    )

    distance_px = float(distance_map[y, x])

    score = 1.0 - min(distance_px / MAX_MATCH_DISTANCE, 1.0)

    return score, distance_px


def get_overlap_score(center, color_mask):
    height, width = color_mask.shape
    x, y = center.astype(int)

    circle_mask = np.zeros((height, width), dtype=np.uint8)

    cv2.circle(
        circle_mask,
        (x, y),
        MATCH_RADIUS,
        255,
        -1
    )

    circle_area = np.count_nonzero(circle_mask)

    if circle_area == 0:
        return 0.0

    overlap = cv2.bitwise_and(color_mask, circle_mask)
    overlap_count = np.count_nonzero(overlap)

    return float(overlap_count / circle_area)


def score_candidate_for_color(candidate, color_mask):
    center = candidate["center"]

    distance_score, distance_px = get_distance_to_mask_score(center, color_mask)
    overlap_score = get_overlap_score(center, color_mask)

    weight_sum = DISTANCE_WEIGHT + OVERLAP_WEIGHT

    if weight_sum <= 0:
        combined_score = 0.0
    else:
        combined_score = (
                                 DISTANCE_WEIGHT * distance_score +
                                 OVERLAP_WEIGHT * overlap_score
                         ) / weight_sum

    return {
        "combined_score": combined_score,
        "distance_score": distance_score,
        "overlap_score": overlap_score,
        "distance_px": distance_px
    }


def assign_colors_to_candidates(candidates, red_mask, green_mask, blue_mask):
    if len(candidates) < 3:
        return {
            "red": None,
            "green": None,
            "blue": None
        }

    color_masks = {
        "red": red_mask,
        "green": green_mask,
        "blue": blue_mask
    }

    colors = ["red", "green", "blue"]

    score_table = {}

    for color in colors:
        score_table[color] = []

        for candidate in candidates:
            score_info = score_candidate_for_color(candidate, color_masks[color])
            score_table[color].append(score_info)

    best_total_score = -1
    best_assignment = None

    for permutation in itertools.permutations(range(3), 3):
        total_score = 0.0

        for color, candidate_index in zip(colors, permutation):
            total_score += score_table[color][candidate_index]["combined_score"]

        if total_score > best_total_score:
            best_total_score = total_score
            best_assignment = dict(zip(colors, permutation))

    return {
        "red": candidates[best_assignment["red"]]["center"],
        "green": candidates[best_assignment["green"]]["center"],
        "blue": candidates[best_assignment["blue"]]["center"]
    }


def get_camera_matrix(frame):
    """
    Retorna la matriz interna de la cámara.

    Si CAMERA_MATRIX está definida arriba, usa esa.
    Si no, aproxima fx y fy usando el ancho de la imagen.
    """

    if CAMERA_MATRIX is not None:
        return CAMERA_MATRIX

    height, width = frame.shape[:2]

    fx = APPROX_FOCAL_LENGTH_FACTOR * width
    fy = APPROX_FOCAL_LENGTH_FACTOR * width

    cx = width / 2.0
    cy = height / 2.0

    return np.array([
        [fx, 0,  cx],
        [0,  fy, cy],
        [0,  0,  1]
    ], dtype=np.float32)


def validate_triangle_distances():
    """
    Verifica que las tres distancias formen un triángulo válido.
    """

    d_rg = float(RED_GREEN_DISTANCE)
    d_rb = float(RED_BLUE_DISTANCE)
    d_gb = float(GREEN_BLUE_DISTANCE)

    distances = {
        "RED_GREEN_DISTANCE": d_rg,
        "RED_BLUE_DISTANCE": d_rb,
        "GREEN_BLUE_DISTANCE": d_gb
    }

    for name, value in distances.items():
        if value <= 0:
            raise ValueError(f"{name} debe ser mayor que cero.")

    if d_rg + d_rb <= d_gb:
        raise ValueError(
            "Las distancias no forman un triángulo válido: "
            "RED_GREEN_DISTANCE + RED_BLUE_DISTANCE debe ser mayor que GREEN_BLUE_DISTANCE."
        )

    if d_rg + d_gb <= d_rb:
        raise ValueError(
            "Las distancias no forman un triángulo válido: "
            "RED_GREEN_DISTANCE + GREEN_BLUE_DISTANCE debe ser mayor que RED_BLUE_DISTANCE."
        )

    if d_rb + d_gb <= d_rg:
        raise ValueError(
            "Las distancias no forman un triángulo válido: "
            "RED_BLUE_DISTANCE + GREEN_BLUE_DISTANCE debe ser mayor que RED_GREEN_DISTANCE."
        )


def get_triangle_object_points():
    """
    Coordenadas reales del triángulo en su propio sistema local.

    Convención local:

            verde

        rojo      azul

    El orden de retorno sigue siendo:
        rojo, verde, azul
    """

    validate_triangle_distances()

    d_rg = float(RED_GREEN_DISTANCE)
    d_rb = float(RED_BLUE_DISTANCE)
    d_gb = float(GREEN_BLUE_DISTANCE)

    # Construimos primero:
    #
    #   rojo = (0, 0, 0)
    #   azul = (d_rb, 0, 0)
    #
    # Luego calculamos el verde para que:
    #
    #   distancia(verde, rojo) = d_rg
    #   distancia(verde, azul) = d_gb

    red = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    blue = np.array([d_rb, 0.0, 0.0], dtype=np.float32)

    green_x = (d_rg ** 2 - d_gb ** 2 + d_rb ** 2) / (2.0 * d_rb)
    green_y_squared = d_rg ** 2 - green_x ** 2
    green_y = math.sqrt(max(green_y_squared, 0.0))

    green = np.array([green_x, green_y, 0.0], dtype=np.float32)

    # Centrar el sistema local en el centro geométrico del triángulo.
    centroid = (red + green + blue) / 3.0

    red = red - centroid
    green = green - centroid
    blue = blue - centroid

    return np.array([red, green, blue], dtype=np.float32)


def angle_difference_deg(a, b):
    """
    Diferencia angular mínima entre dos ángulos en grados.
    Retorna un valor entre -180 y 180.
    """

    return (a - b + 180.0) % 360.0 - 180.0


def solve_pose_with_optional_guess(
        object_points,
        image_points,
        camera_matrix,
        previous_rvec,
        previous_tvec
):
    """
    Opción 2:

    Si hay una pose previa, se usa como guess para solvePnP ITERATIVE.
    Si no hay pose previa o falla, se cae a SQPNP.
    """

    # --------------------------------------------------------
    # Intento 1: solvePnP usando la pose anterior como guess
    # --------------------------------------------------------

    if (
            USE_PREVIOUS_POSE_GUESS and
            previous_rvec is not None and
            previous_tvec is not None
    ):
        try:
            rvec_guess = previous_rvec.copy()
            tvec_guess = previous_tvec.copy()

            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                DIST_COEFFS,
                rvec=rvec_guess,
                tvec=tvec_guess,
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                return {
                    "success": True,
                    "rvec": rvec,
                    "tvec": tvec,
                    "method": "ITERATIVE_WITH_PREVIOUS_GUESS"
                }

        except cv2.error:
            pass

    # --------------------------------------------------------
    # Intento 2: SQPNP sin guess
    # --------------------------------------------------------

    if not hasattr(cv2, "SOLVEPNP_SQPNP"):
        raise RuntimeError(
            "Tu versión de OpenCV no tiene SOLVEPNP_SQPNP. "
            "Con 3 puntos, este método es necesario como inicialización. "
            "Actualiza OpenCV con: pip install --upgrade opencv-python"
        )

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        DIST_COEFFS,
        flags=cv2.SOLVEPNP_SQPNP
    )

    return {
        "success": success,
        "rvec": rvec,
        "tvec": tvec,
        "method": "SQPNP_NO_GUESS"
    }


def pose_to_distance_and_tilt(rvec, tvec):
    """
    Convierte rvec/tvec a:
        - distancia Z
        - tilt en el plano vertical Y-Z de la cámara
    """

    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # Normal local del triángulo.
    normal_local = np.array([[0.0], [0.0], [1.0]], dtype=np.float32)

    # Normal del triángulo expresada en coordenadas de cámara.
    normal_camera = rotation_matrix @ normal_local
    normal_camera = normal_camera.flatten()

    # Evita saltos por normal invertido.
    if normal_camera[2] < 0:
        normal_camera = -normal_camera

    distance_z = float(tvec[2][0])

    # Proyección sobre plano vertical Y-Z.
    normal_y = normal_camera[1]
    normal_z = normal_camera[2]

    tilt_rad = math.atan2(normal_y, normal_z)
    tilt_deg = math.degrees(tilt_rad)

    return distance_z, tilt_deg, normal_camera


def estimate_distance_and_tilt(
        red_center,
        green_center,
        blue_center,
        frame,
        previous_rvec=None,
        previous_tvec=None
):
    """
    Estima pose, distancia y tilt.

    Usa la pose anterior como guess cuando está disponible.
    """

    if red_center is None or green_center is None or blue_center is None:
        return None

    object_points = get_triangle_object_points()

    image_points = np.array([
        red_center,
        green_center,
        blue_center
    ], dtype=np.float32)

    camera_matrix = get_camera_matrix(frame)

    pose_solution = solve_pose_with_optional_guess(
        object_points,
        image_points,
        camera_matrix,
        previous_rvec,
        previous_tvec
    )

    if not pose_solution["success"]:
        return None

    rvec = pose_solution["rvec"]
    tvec = pose_solution["tvec"]

    distance_z, tilt_deg, normal_camera = pose_to_distance_and_tilt(rvec, tvec)

    return {
        "distance_z": distance_z,
        "tilt_deg": tilt_deg,
        "normal_camera": normal_camera,
        "rvec": rvec,
        "tvec": tvec,
        "method": pose_solution["method"]
    }


def apply_tilt_jump_rejection(
        new_pose,
        last_accepted_pose,
        consecutive_rejects
):
    """
    Opción 3:

    Si el tilt nuevo salta demasiado respecto al último tilt aceptado,
    se rechaza la medición nueva y se conserva la pose anterior.

    Si hay muchos rechazos consecutivos, se acepta la nueva pose para permitir
    movimientos reales rápidos.
    """

    if new_pose is None:
        return last_accepted_pose, consecutive_rejects, False

    if not REJECT_TILT_JUMPS:
        return new_pose, 0, True

    if last_accepted_pose is None:
        return new_pose, 0, True

    new_tilt = new_pose["tilt_deg"]
    old_tilt = last_accepted_pose["tilt_deg"]

    tilt_jump = abs(angle_difference_deg(new_tilt, old_tilt))

    if tilt_jump <= MAX_TILT_JUMP_DEG:
        return new_pose, 0, True

    consecutive_rejects += 1

    if PRINT_REJECTED_FRAMES:
        print(
            f"rejected_tilt_jump={tilt_jump:.3f}, "
            f"new_tilt={new_tilt:.3f}, "
            f"old_tilt={old_tilt:.3f}, "
            f"rejects={consecutive_rejects}"
        )

    # Si se rechazaron demasiados frames seguidos, se acepta la nueva pose.
    # Esto evita quedarse pegado si realmente hubo un movimiento rápido.
    if consecutive_rejects >= MAX_CONSECUTIVE_REJECTS:
        return new_pose, 0, True

    return last_accepted_pose, consecutive_rejects, False


def draw_marker(frame, center, color_bgr, label):
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
        cv2.LINE_AA
    )


def draw_line(frame, p1, p2, color_bgr=(255, 255, 255)):
    if p1 is None or p2 is None:
        return

    p1 = tuple(p1.astype(int))
    p2 = tuple(p2.astype(int))

    cv2.line(frame, p1, p2, color_bgr, 2)


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

validate_triangle_distances()

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("No se pudo abrir la cámara.")

last_print_time = 0

# Pose aceptada más reciente.
last_accepted_pose = None

# rvec/tvec usados como guess para el siguiente frame.
last_rvec = None
last_tvec = None

# Contador de rechazos consecutivos por salto de tilt.
consecutive_rejects = 0

while True:
    ret, frame = cap.read()

    if not ret:
        break

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # --------------------------------------------------------
    # 1) Detectar centros candidatos de luz
    # --------------------------------------------------------

    light_mask = create_mask(
        hsv,
        LIGHT_LOW_HSV,
        LIGHT_HIGH_HSV
    )

    candidates = find_light_candidates(light_mask)

    # --------------------------------------------------------
    # 2) Filtros de color para clasificar los centros
    # --------------------------------------------------------

    red_mask = create_mask(
        hsv,
        RED_LOW_HSV,
        RED_HIGH_HSV,
        RED_LOW_HSV_2,
        RED_HIGH_HSV_2
    )

    green_mask = create_mask(
        hsv,
        GREEN_LOW_HSV,
        GREEN_HIGH_HSV
    )

    blue_mask = create_mask(
        hsv,
        BLUE_LOW_HSV,
        BLUE_HIGH_HSV
    )

    # --------------------------------------------------------
    # 3) Asignar color a cada centro candidato
    # --------------------------------------------------------

    assigned_centers = assign_colors_to_candidates(
        candidates,
        red_mask,
        green_mask,
        blue_mask
    )

    red_center = assigned_centers["red"]
    green_center = assigned_centers["green"]
    blue_center = assigned_centers["blue"]

    # --------------------------------------------------------
    # 4) Estimar distancia directa y tilt
    # --------------------------------------------------------

    new_pose = estimate_distance_and_tilt(
        red_center,
        green_center,
        blue_center,
        frame,
        previous_rvec=last_rvec,
        previous_tvec=last_tvec
    )

    accepted_pose, consecutive_rejects, was_accepted = apply_tilt_jump_rejection(
        new_pose,
        last_accepted_pose,
        consecutive_rejects
    )

    # Si la medición fue aceptada, actualizamos la pose usada como referencia.
    # Si fue rechazada, mantenemos la anterior.
    if was_accepted and accepted_pose is not None:
        last_accepted_pose = accepted_pose
        last_rvec = accepted_pose["rvec"]
        last_tvec = accepted_pose["tvec"]

    pose_data = last_accepted_pose

    # --------------------------------------------------------
    # 5) Imprimir únicamente distancia y tilt
    # --------------------------------------------------------

    now = time.time()

    if pose_data is not None and now - last_print_time > PRINT_EVERY_SECONDS:
        last_print_time = now

        distance_z = pose_data["distance_z"]
        tilt_deg = pose_data["tilt_deg"]

        print(f"distance={distance_z:.3f}, tilt={tilt_deg:.3f}")

    # --------------------------------------------------------
    # 6) Ventanas
    # --------------------------------------------------------

    display = frame.copy()

    draw_marker(display, red_center, (0, 0, 255), "Rojo")
    draw_marker(display, green_center, (0, 255, 0), "Verde")
    draw_marker(display, blue_center, (255, 0, 0), "Azul")

    draw_line(display, red_center, green_center)
    draw_line(display, green_center, blue_center)
    draw_line(display, blue_center, red_center)

    if pose_data is not None:
        distance_z = pose_data["distance_z"]
        tilt_deg = pose_data["tilt_deg"]

        cv2.putText(
            display,
            f"Distance Z: {distance_z:.2f}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            display,
            f"Tilt: {tilt_deg:.2f} deg",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

    cv2.imshow("Filtro rojo", red_mask)
    cv2.imshow("Filtro verde", green_mask)
    cv2.imshow("Filtro azul", blue_mask)
    cv2.imshow("Camara original con deteccion", display)

    key = cv2.waitKey(1) & 0xFF

    if key == 27 or key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()