"""Magnified flute mapping.

Pure math: pose de la cabeza (posición + orientación) -> parámetros del
embocador del robot (l, theta, offset), pasando por un bisel virtual fijo
frente al usuario.

Este módulo no depende de PyQt, OSC ni hardware. Es unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as R


MOUTH_OFFSET = np.array([0.0, 0.0, 0.0])


@dataclass
class HeadPose:
    position: np.ndarray
    quaternion: np.ndarray

    @classmethod
    def neutral(cls) -> "HeadPose":
        return cls(
            position=np.zeros(3),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        )

    def rotation(self) -> R:
        w, x, y, z = self.quaternion
        return R.from_quat([x, y, z, w])


@dataclass
class MappingConfig:
    bisel_offset_m: float = 0.20
    magnification_k: float = 10.0
    mouth_offset: np.ndarray = None

    def __post_init__(self):
        if self.mouth_offset is None:
            self.mouth_offset = MOUTH_OFFSET.copy()

    @property
    def P_bisel(self) -> np.ndarray:
        return np.array([0.0, 0.0, -self.bisel_offset_m])


@dataclass
class MappingResult:
    l: float
    theta: float
    offset: float
    P_mouth: np.ndarray
    v_jet: np.ndarray
    P_closest: np.ndarray
    miss_vector: np.ndarray
    s_closest: float

    def miss_distance(self) -> float:
        return float(np.linalg.norm(self.miss_vector))

    def aiming_error_deg(self) -> float:
        if self.s_closest <= 1e-6:
            return 180.0
        return float(np.degrees(np.arctan2(
            self.miss_distance(), self.s_closest)))


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_inv(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    return np.array([w, -x, -y, -z]) / n


def relative_pose(current: HeadPose, zero: HeadPose) -> HeadPose:
    q_rel = quat_mul(quat_inv(zero.quaternion), current.quaternion)
    P_rel = current.position - zero.position
    return HeadPose(position=P_rel, quaternion=q_rel)


def magnified_flute_mapping(
    current: HeadPose,
    zero: HeadPose,
    config: MappingConfig,
) -> MappingResult:
    """Compute (l, theta, offset) from current head pose.

    Frame convention:
    - X: lateral (positive = user's right)
    - Y: vertical (positive = up)
    - Z: depth (negative = in front of user)
    """
    rel = relative_pose(current, zero)
    rot = rel.rotation()

    P_mouth = rel.position + rot.apply(config.mouth_offset)
    v_jet = rot.apply(np.array([0.0, 0.0, -1.0]))

    to_bisel = config.P_bisel - P_mouth
    dist_mouth_bisel = float(np.linalg.norm(to_bisel))

    s_closest = float(np.dot(to_bisel, v_jet))
    P_closest = P_mouth + s_closest * v_jet
    miss_vector = P_closest - config.P_bisel

    l_robot = dist_mouth_bisel / config.magnification_k
    theta_robot = float(np.arctan2(miss_vector[1], max(s_closest, 1e-6)))
    offset_robot = float(miss_vector[0] / config.magnification_k)

    return MappingResult(
        l=l_robot,
        theta=theta_robot,
        offset=offset_robot,
        P_mouth=P_mouth,
        v_jet=v_jet,
        P_closest=P_closest,
        miss_vector=miss_vector,
        s_closest=s_closest,
    )


def euler_to_quat_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Build a quaternion from yaw/pitch/roll in a Y-up frame.

    - yaw: rotation around Y (vertical) — positive turns head LEFT
      (counterclockwise viewed from above).
    - pitch: rotation around X (lateral) — positive tilts head UP.
    - roll: rotation around Z (depth) — positive tilts head to the right.

    Returns quaternion [w, x, y, z].
    """
    qy = np.array([np.cos(yaw / 2), 0.0, np.sin(yaw / 2), 0.0])
    qx = np.array([np.cos(pitch / 2), np.sin(pitch / 2), 0.0, 0.0])
    qz = np.array([np.cos(roll / 2), 0.0, 0.0, np.sin(roll / 2)])
    return quat_mul(quat_mul(qy, qx), qz)
