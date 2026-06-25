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
GREEN_BLUE_DISTANCE = 9.0

# Cuarto LED amarillo:
# - Está a 9 cm del LED rojo.
# - Está a 9 cm del LED azul.
# - Está por debajo de rojo y azul.
# - Está 2.7 cm más atrás del plano formado por rojo, verde y azul.
#
# Convención local usada:
#   X: rojo -> azul
#   Y: rojo/azul -> verde
#   Z: normal del plano rojo-verde-azul
#
# Por defecto se usa Z negativo para representar "más atrás" del plano.
# Si al probar notas que el tilt queda con signo invertido, cambia el signo
# de YELLOW_PLANE_OFFSET_Z a +2.7.
YELLOW_RED_DISTANCE = 9.0
YELLOW_BLUE_DISTANCE = 9.0
YELLOW_PLANE_OFFSET_Z = -2.7

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

YELLOW_LOW_HSV = np.array([18, 80, 80])
YELLOW_HIGH_HSV = np.array([38, 255, 255])

# ------------------------------------------------------------
# Parámetros de detección
# ------------------------------------------------------------

MIN_LIGHT_AREA = 20
MAX_LIGHT_CANDIDATES = 4

MIN_COLOR_AREA = 10

MORPH_KERNEL_SIZE = 5

MATCH_RADIUS = 25
MAX_MATCH_DISTANCE = 80

DISTANCE_WEIGHT = 0.5
OVERLAP_WEIGHT = 0.5

PRINT_EVERY_SECONDS = 0.1

# ------------------------------------------------------------
# Promedio movil de mediciones
# ------------------------------------------------------------

# Los valores que se imprimen, se muestran en pantalla y se grafican
# son el promedio de las ultimas N mediciones aceptadas.
# Si N = 1, se usa la medicion instantanea sin promediar.
MOVING_AVERAGE_N = 10

# ------------------------------------------------------------
# Ventana de gráficos en tiempo real
# ------------------------------------------------------------

PLOT_WINDOW_NAME = "Graficos distancia y tilt"
PLOT_WIDTH = 900
PLOT_HEIGHT = 500
PLOT_HISTORY_SECONDS = 10.0
PLOT_MARGIN_LEFT = 65
PLOT_MARGIN_RIGHT = 25
PLOT_MARGIN_TOP = 35
PLOT_MARGIN_BOTTOM = 40

# Ejes Y fijos para que los graficos no cambien de escala.
# Distancia: siempre desde 0 hasta DISTANCE_PLOT_MAX.
# Si tu distancia real supera este valor, la linea se saturara arriba;
# en ese caso sube DISTANCE_PLOT_MAX.
DISTANCE_PLOT_MIN = 0.0
DISTANCE_PLOT_MAX = 50.0

# Tilt: siempre desde -90 hasta 90 grados.
TILT_PLOT_MIN = -90.0
TILT_PLOT_MAX = 90.0



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


def assign_colors_to_candidates(candidates, red_mask, green_mask, blue_mask, yellow_mask):
    """
    Asigna 4 centros candidatos a rojo, verde, azul y amarillo.

    La detección de centros se hace con LIGHT_LOW_HSV/LIGHT_HIGH_HSV,
    mientras que la identidad de cada centro se decide con los filtros de color.

    Retorna centros en el orden lógico del modelo:
        red, green, blue, yellow
    """

    if len(candidates) < 4:
        return {
            "red": None,
            "green": None,
            "blue": None,
            "yellow": None
        }

    color_masks = {
        "red": red_mask,
        "green": green_mask,
        "blue": blue_mask,
        "yellow": yellow_mask
    }

    colors = ["red", "green", "blue", "yellow"]

    score_table = {}

    for color in colors:
        score_table[color] = []

        for candidate in candidates:
            score_info = score_candidate_for_color(candidate, color_masks[color])
            score_table[color].append(score_info)

    best_total_score = -1
    best_assignment = None

    # 4 candidatos y 4 colores => 4! = 24 combinaciones, sigue siendo barato.
    for permutation in itertools.permutations(range(4), 4):
        total_score = 0.0

        for color, candidate_index in zip(colors, permutation):
            total_score += score_table[color][candidate_index]["combined_score"]

        if total_score > best_total_score:
            best_total_score = total_score
            best_assignment = dict(zip(colors, permutation))

    return {
        "red": candidates[best_assignment["red"]]["center"],
        "green": candidates[best_assignment["green"]]["center"],
        "blue": candidates[best_assignment["blue"]]["center"],
        "yellow": candidates[best_assignment["yellow"]]["center"]
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


def validate_led_geometry():
    """
    Verifica que la geometría configurada sea físicamente posible.
    """

    d_rg = float(RED_GREEN_DISTANCE)
    d_rb = float(RED_BLUE_DISTANCE)
    d_gb = float(GREEN_BLUE_DISTANCE)
    d_yr = float(YELLOW_RED_DISTANCE)
    d_yb = float(YELLOW_BLUE_DISTANCE)
    z_y = float(YELLOW_PLANE_OFFSET_Z)

    distances = {
        "RED_GREEN_DISTANCE": d_rg,
        "RED_BLUE_DISTANCE": d_rb,
        "GREEN_BLUE_DISTANCE": d_gb,
        "YELLOW_RED_DISTANCE": d_yr,
        "YELLOW_BLUE_DISTANCE": d_yb
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

    # Para que el LED amarillo esté a d_yr del rojo y a d_yb del azul,
    # con un offset Z fijo, debe existir una coordenada Y real.
    yellow_x = (d_yr ** 2 - d_yb ** 2 + d_rb ** 2) / (2.0 * d_rb)
    yellow_y_squared = d_yr ** 2 - yellow_x ** 2 - z_y ** 2

    if yellow_y_squared < -1e-6:
        raise ValueError(
            "La geometría del LED amarillo no es posible con las medidas dadas. "
            f"YELLOW_RED_DISTANCE={d_yr}, "
            f"YELLOW_BLUE_DISTANCE={d_yb}, "
            f"RED_BLUE_DISTANCE={d_rb}, "
            f"YELLOW_PLANE_OFFSET_Z={z_y}."
        )


def get_object_points():
    """
    Coordenadas reales de los LEDs en el sistema local del objeto.

    Convención física actual:

              verde

        rojo          azul

              amarillo

    con amarillo ubicado por debajo de rojo/azul y fuera del plano de
    rojo-verde-azul.

    Orden de retorno:
        rojo, verde, azul, amarillo

    Importante:
        Las coordenadas se centran usando el centro geométrico de los 3 LEDs
        originales: rojo, verde y azul. Así, tvec sigue representando
        aproximadamente la posición del centro del triángulo principal, no el
        centro de los 4 LEDs.
    """

    validate_led_geometry()

    d_rg = float(RED_GREEN_DISTANCE)
    d_rb = float(RED_BLUE_DISTANCE)
    d_gb = float(GREEN_BLUE_DISTANCE)
    d_yr = float(YELLOW_RED_DISTANCE)
    d_yb = float(YELLOW_BLUE_DISTANCE)
    z_y = float(YELLOW_PLANE_OFFSET_Z)

    # Plano principal RGB:
    #   rojo = (0, 0, 0)
    #   azul = (d_rb, 0, 0)
    #   verde queda arriba, con y positivo.
    red = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    blue = np.array([d_rb, 0.0, 0.0], dtype=np.float32)

    green_x = (d_rg ** 2 - d_gb ** 2 + d_rb ** 2) / (2.0 * d_rb)
    green_y_squared = d_rg ** 2 - green_x ** 2
    green_y = math.sqrt(max(green_y_squared, 0.0))
    green = np.array([green_x, green_y, 0.0], dtype=np.float32)

    # LED amarillo:
    # Está a YELLOW_RED_DISTANCE del rojo y a YELLOW_BLUE_DISTANCE del azul.
    # Además tiene un offset Z fijo respecto al plano RGB.
    # Como está "por debajo" de rojo y azul, usamos y negativo.
    yellow_x = (d_yr ** 2 - d_yb ** 2 + d_rb ** 2) / (2.0 * d_rb)
    yellow_y_squared = d_yr ** 2 - yellow_x ** 2 - z_y ** 2
    yellow_y = -math.sqrt(max(yellow_y_squared, 0.0))
    yellow = np.array([yellow_x, yellow_y, z_y], dtype=np.float32)

    # El origen se mantiene en el centro del triángulo RGB original.
    rgb_centroid = (red + green + blue) / 3.0

    red = red - rgb_centroid
    green = green - rgb_centroid
    blue = blue - rgb_centroid
    yellow = yellow - rgb_centroid

    return np.array([red, green, blue, yellow], dtype=np.float32)


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
    Resuelve la pose del objeto con 4 puntos.

    Si hay una pose previa, primero intenta SOLVEPNP_ITERATIVE usando esa pose
    como guess. Si no hay pose previa, o si falla, usa EPNP para obtener una
    inicialización y luego refina con ITERATIVE.
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
    # Intento 2: EPNP sin guess
    # --------------------------------------------------------

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        DIST_COEFFS,
        flags=cv2.SOLVEPNP_EPNP
    )

    if not success:
        return {
            "success": False,
            "rvec": None,
            "tvec": None,
            "method": "EPNP_FAILED"
        }

    # --------------------------------------------------------
    # Intento 3: refinar EPNP con ITERATIVE
    # --------------------------------------------------------

    try:
        rvec_guess = rvec.copy()
        tvec_guess = tvec.copy()

        refined_success, refined_rvec, refined_tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            DIST_COEFFS,
            rvec=rvec_guess,
            tvec=tvec_guess,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if refined_success:
            return {
                "success": True,
                "rvec": refined_rvec,
                "tvec": refined_tvec,
                "method": "EPNP_THEN_ITERATIVE"
            }

    except cv2.error:
        pass

    return {
        "success": True,
        "rvec": rvec,
        "tvec": tvec,
        "method": "EPNP_ONLY"
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
        yellow_center,
        frame,
        previous_rvec=None,
        previous_tvec=None
):
    """
    Estima pose, distancia y tilt.

    Usa la pose anterior como guess cuando está disponible.
    """

    if red_center is None or green_center is None or blue_center is None or yellow_center is None:
        return None

    object_points = get_object_points()

    image_points = np.array([
        red_center,
        green_center,
        blue_center,
        yellow_center
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




def append_capped_sample(samples, value, max_count):
    """
    Agrega una muestra a una lista y mantiene solo las ultimas max_count.
    """

    samples.append(float(value))

    while len(samples) > max_count:
        samples.pop(0)


def average_values(values):
    """
    Promedio aritmetico normal para magnitudes lineales, como distancia.
    """

    if len(values) == 0:
        return None

    return float(sum(values) / len(values))


def average_angles_deg(angles_deg):
    """
    Promedio circular de angulos en grados.

    Para tilt normalmente bastaria con promedio aritmetico, pero el promedio
    circular evita problemas si el angulo cruza cerca de -180/180 grados.
    """

    if len(angles_deg) == 0:
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


def get_averaged_measurement(distance_samples, tilt_samples):
    """
    Retorna el promedio de las ultimas N mediciones para distancia y tilt.
    """

    averaged_distance_z = average_values(distance_samples)
    averaged_tilt_deg = average_angles_deg(tilt_samples)

    if averaged_distance_z is None or averaged_tilt_deg is None:
        return None

    return {
        "distance_z": averaged_distance_z,
        "tilt_deg": averaged_tilt_deg
    }


def trim_history(history, current_time, history_seconds):
    """
    Mantiene solo muestras dentro de la ventana temporal visible.
    history es una lista de tuplas:
        (time_seconds, value)
    """

    min_time = current_time - history_seconds

    while len(history) > 0 and history[0][0] < min_time:
        history.pop(0)


def get_plot_limits(history, configured_min, configured_max):
    """
    Retorna límites Y para graficar.

    En la configuracion actual los limites se entregan fijos:
        - distancia: 0 a DISTANCE_PLOT_MAX
        - tilt: -90 a 90 grados

    La funcion conserva soporte para None por si despues quieres
    volver a autoescalado.
    """

    values = [value for _, value in history]

    if len(values) == 0:
        y_min = 0.0 if configured_min is None else float(configured_min)
        y_max = 1.0 if configured_max is None else float(configured_max)
        return y_min, y_max

    if configured_min is None:
        y_min = min(values)
    else:
        y_min = float(configured_min)

    if configured_max is None:
        y_max = max(values)
    else:
        y_max = float(configured_max)

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


def value_to_plot_y(value, y_min, y_max, top, bottom):
    """
    Convierte un valor a coordenada Y de pantalla.
    """

    alpha = (value - y_min) / (y_max - y_min)
    alpha = max(0.0, min(1.0, alpha))

    return int(bottom - alpha * (bottom - top))


def time_to_plot_x(sample_time, current_time, left, right):
    """
    Convierte un tiempo absoluto a coordenada X de pantalla.
    El borde derecho representa el instante actual.
    """

    start_time = current_time - PLOT_HISTORY_SECONDS
    alpha = (sample_time - start_time) / PLOT_HISTORY_SECONDS
    alpha = max(0.0, min(1.0, alpha))

    return int(left + alpha * (right - left))


def draw_time_series_panel(
        image,
        history,
        current_time,
        title,
        unit,
        configured_min,
        configured_max,
        top,
        bottom,
        line_color
):
    """
    Dibuja un gráfico temporal dentro de un panel de la imagen.
    """

    left = PLOT_MARGIN_LEFT
    right = PLOT_WIDTH - PLOT_MARGIN_RIGHT

    y_min, y_max = get_plot_limits(history, configured_min, configured_max)

    # Fondo del panel.
    cv2.rectangle(image, (left, top), (right, bottom), (28, 28, 28), -1)
    cv2.rectangle(image, (left, top), (right, bottom), (180, 180, 180), 1)

    # Grilla vertical: tiempo.
    vertical_grid_count = 5
    for i in range(vertical_grid_count + 1):
        x = int(left + i * (right - left) / vertical_grid_count)
        cv2.line(image, (x, top), (x, bottom), (55, 55, 55), 1)

        seconds_ago = PLOT_HISTORY_SECONDS * (1.0 - i / vertical_grid_count)
        label = f"-{seconds_ago:.0f}s" if seconds_ago > 0.01 else "0s"

        cv2.putText(
            image,
            label,
            (x - 15, bottom + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (190, 190, 190),
            1,
            cv2.LINE_AA
        )

    # Grilla horizontal: valores.
    horizontal_grid_count = 4
    for i in range(horizontal_grid_count + 1):
        y = int(top + i * (bottom - top) / horizontal_grid_count)
        cv2.line(image, (left, y), (right, y), (55, 55, 55), 1)

        value = y_max - i * (y_max - y_min) / horizontal_grid_count
        label = f"{value:.1f}"

        cv2.putText(
            image,
            label,
            (8, y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (190, 190, 190),
            1,
            cv2.LINE_AA
        )

    # Título del panel.
    cv2.putText(
        image,
        f"{title} [{unit}]",
        (left, top - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (230, 230, 230),
        2,
        cv2.LINE_AA
    )

    # Línea de datos.
    if len(history) >= 2:
        points = []

        for sample_time, value in history:
            x = time_to_plot_x(sample_time, current_time, left, right)
            y = value_to_plot_y(value, y_min, y_max, top, bottom)
            points.append([x, y])

        points = np.array(points, dtype=np.int32)
        cv2.polylines(image, [points], False, line_color, 2, cv2.LINE_AA)

    # Valor actual.
    if len(history) > 0:
        latest_value = history[-1][1]
        latest_label = f"actual: {latest_value:.3f} {unit}"
    else:
        latest_label = "actual: --"

    cv2.putText(
        image,
        latest_label,
        (right - 220, top + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (230, 230, 230),
        1,
        cv2.LINE_AA
    )


def create_realtime_plot_image(distance_history, tilt_history, current_time):
    """
    Crea una imagen BGR con dos gráficos:
        - distancia vs tiempo
        - tilt vs tiempo
    """

    plot_image = np.zeros((PLOT_HEIGHT, PLOT_WIDTH, 3), dtype=np.uint8)
    plot_image[:] = (18, 18, 18)

    cv2.putText(
        plot_image,
        f"Distancia y tilt vs tiempo | promedio N={MOVING_AVERAGE_N}",
        (20, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (240, 240, 240),
        2,
        cv2.LINE_AA
    )

    panel_gap = 55
    available_height = PLOT_HEIGHT - PLOT_MARGIN_TOP - PLOT_MARGIN_BOTTOM - panel_gap
    panel_height = available_height // 2

    distance_top = PLOT_MARGIN_TOP + 25
    distance_bottom = distance_top + panel_height

    tilt_top = distance_bottom + panel_gap
    tilt_bottom = tilt_top + panel_height

    draw_time_series_panel(
        plot_image,
        distance_history,
        current_time,
        "Distancia Z",
        "unidad",
        DISTANCE_PLOT_MIN,
        DISTANCE_PLOT_MAX,
        distance_top,
        distance_bottom,
        (0, 220, 255)
    )

    draw_time_series_panel(
        plot_image,
        tilt_history,
        current_time,
        "Tilt",
        "deg",
        TILT_PLOT_MIN,
        TILT_PLOT_MAX,
        tilt_top,
        tilt_bottom,
        (255, 120, 255)
    )

    return plot_image


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

validate_led_geometry()

if MOVING_AVERAGE_N < 1:
    raise ValueError("MOVING_AVERAGE_N debe ser mayor o igual a 1.")

if DISTANCE_PLOT_MIN is not None and DISTANCE_PLOT_MAX is not None:
    if DISTANCE_PLOT_MAX <= DISTANCE_PLOT_MIN:
        raise ValueError("DISTANCE_PLOT_MAX debe ser mayor que DISTANCE_PLOT_MIN.")

if TILT_PLOT_MIN is not None and TILT_PLOT_MAX is not None:
    if TILT_PLOT_MAX <= TILT_PLOT_MIN:
        raise ValueError("TILT_PLOT_MAX debe ser mayor que TILT_PLOT_MIN.")

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

# Historial para la ventana de gráficos.
distance_history = []
tilt_history = []

# Muestras recientes usadas para el promedio movil.
distance_average_samples = []
tilt_average_samples = []

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

    yellow_mask = create_mask(
        hsv,
        YELLOW_LOW_HSV,
        YELLOW_HIGH_HSV
    )

    # --------------------------------------------------------
    # 3) Asignar color a cada centro candidato
    # --------------------------------------------------------

    assigned_centers = assign_colors_to_candidates(
        candidates,
        red_mask,
        green_mask,
        blue_mask,
        yellow_mask
    )

    red_center = assigned_centers["red"]
    green_center = assigned_centers["green"]
    blue_center = assigned_centers["blue"]
    yellow_center = assigned_centers["yellow"]

    # --------------------------------------------------------
    # 4) Estimar distancia directa y tilt
    # --------------------------------------------------------

    new_pose = estimate_distance_and_tilt(
        red_center,
        green_center,
        blue_center,
        yellow_center,
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
    # 5) Promediar ultimas N mediciones y actualizar historial
    # --------------------------------------------------------

    now = time.time()

    averaged_pose_data = None

    if pose_data is not None:
        append_capped_sample(
            distance_average_samples,
            pose_data["distance_z"],
            MOVING_AVERAGE_N
        )

        append_capped_sample(
            tilt_average_samples,
            pose_data["tilt_deg"],
            MOVING_AVERAGE_N
        )

        averaged_pose_data = get_averaged_measurement(
            distance_average_samples,
            tilt_average_samples
        )

    if averaged_pose_data is not None:
        distance_history.append((now, averaged_pose_data["distance_z"]))
        tilt_history.append((now, averaged_pose_data["tilt_deg"]))

    trim_history(distance_history, now, PLOT_HISTORY_SECONDS)
    trim_history(tilt_history, now, PLOT_HISTORY_SECONDS)

    # --------------------------------------------------------
    # 6) Imprimir únicamente distancia y tilt promediados
    # --------------------------------------------------------

    if averaged_pose_data is not None and now - last_print_time > PRINT_EVERY_SECONDS:
        last_print_time = now

        distance_z = averaged_pose_data["distance_z"]
        tilt_deg = averaged_pose_data["tilt_deg"]

        print(f"distance={distance_z:.3f}, tilt={tilt_deg:.3f}")

    # --------------------------------------------------------
    # 7) Ventanas
    # --------------------------------------------------------

    display = frame.copy()

    draw_marker(display, red_center, (0, 0, 255), "Rojo")
    draw_marker(display, green_center, (0, 255, 0), "Verde")
    draw_marker(display, blue_center, (255, 0, 0), "Azul")
    draw_marker(display, yellow_center, (0, 255, 255), "Amarillo")

    draw_line(display, red_center, green_center)
    draw_line(display, green_center, blue_center)
    draw_line(display, blue_center, red_center)
    draw_line(display, red_center, yellow_center, (0, 255, 255))
    draw_line(display, blue_center, yellow_center, (0, 255, 255))
    draw_line(display, green_center, yellow_center, (160, 160, 160))

    if averaged_pose_data is not None:
        distance_z = averaged_pose_data["distance_z"]
        tilt_deg = averaged_pose_data["tilt_deg"]

        cv2.putText(
            display,
            f"Distance Z avg: {distance_z:.2f}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            display,
            f"Tilt avg: {tilt_deg:.2f} deg",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

    plot_image = create_realtime_plot_image(
        distance_history,
        tilt_history,
        now
    )

    cv2.imshow("Filtro rojo", red_mask)
    cv2.imshow("Filtro verde", green_mask)
    cv2.imshow("Filtro azul", blue_mask)
    cv2.imshow("Filtro amarillo", yellow_mask)
    cv2.imshow("Camara original con deteccion", display)
    cv2.imshow(PLOT_WINDOW_NAME, plot_image)

    key = cv2.waitKey(1) & 0xFF

    if key == 27 or key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()