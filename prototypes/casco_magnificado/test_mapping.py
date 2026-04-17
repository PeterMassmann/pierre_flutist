"""Tests del mapeo magnificado.

Valida las propiedades esperadas:
- Con cabeza en zero-pose y apuntando al bisel, miss = 0 y l = |bisel| / k
- Girar yaw positivo mueve el offset en la dirección correcta
- Inclinar pitch hacia arriba sube theta
- Acercar la cabeza reduce l
"""

from __future__ import annotations

import math
import numpy as np
import pytest

from mapping import (
    HeadPose,
    MappingConfig,
    euler_to_quat_zyx,
    magnified_flute_mapping,
)


def test_neutral_pose_aims_at_bisel():
    config = MappingConfig(bisel_offset_m=0.20, magnification_k=10.0)
    zero = HeadPose.neutral()
    current = HeadPose.neutral()

    result = magnified_flute_mapping(current, zero, config)

    assert result.aiming_error_deg() < 15.0
    expected_l = 0.20 / 10.0
    assert abs(result.l - expected_l) < 0.01


def test_yaw_right_shifts_offset_positive():
    config = MappingConfig()
    zero = HeadPose.neutral()

    yaw_right = math.radians(-10)
    q = euler_to_quat_zyx(yaw_right, 0.0, 0.0)
    current = HeadPose(position=np.zeros(3), quaternion=q)

    result = magnified_flute_mapping(current, zero, config)

    assert result.offset > 0.0


def test_pitch_up_increases_theta():
    config = MappingConfig()
    zero = HeadPose.neutral()

    q_up = euler_to_quat_zyx(0.0, math.radians(15), 0.0)
    result_up = magnified_flute_mapping(
        HeadPose(position=np.zeros(3), quaternion=q_up), zero, config)

    q_down = euler_to_quat_zyx(0.0, math.radians(-15), 0.0)
    result_down = magnified_flute_mapping(
        HeadPose(position=np.zeros(3), quaternion=q_down), zero, config)

    assert result_up.theta > result_down.theta


def test_moving_forward_reduces_l():
    config = MappingConfig(bisel_offset_m=0.20, magnification_k=10.0)
    zero = HeadPose.neutral()

    forward = HeadPose(
        position=np.array([0.0, 0.0, -0.10]),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    result = magnified_flute_mapping(forward, zero, config)

    assert result.l < 0.020
    assert result.l > 0.0


def test_magnification_scales_linearly():
    zero = HeadPose.neutral()
    q = euler_to_quat_zyx(math.radians(-5), 0.0, 0.0)
    pose = HeadPose(position=np.zeros(3), quaternion=q)

    result_k5 = magnified_flute_mapping(
        pose, zero, MappingConfig(magnification_k=5.0))
    result_k10 = magnified_flute_mapping(
        pose, zero, MappingConfig(magnification_k=10.0))

    assert result_k5.offset > result_k10.offset * 1.5


def test_zero_pose_calibration_cancels_offset():
    config = MappingConfig()
    q_tilted = euler_to_quat_zyx(math.radians(10), math.radians(5), 0.0)
    tilted_zero = HeadPose(position=np.zeros(3), quaternion=q_tilted)

    result = magnified_flute_mapping(tilted_zero, tilted_zero, config)

    assert result.aiming_error_deg() < 15.0


def test_result_fields_are_finite():
    config = MappingConfig()
    zero = HeadPose.neutral()

    for yaw in np.linspace(-math.pi / 3, math.pi / 3, 7):
        for pitch in np.linspace(-math.pi / 4, math.pi / 4, 5):
            q = euler_to_quat_zyx(yaw, pitch, 0.0)
            pose = HeadPose(position=np.zeros(3), quaternion=q)
            result = magnified_flute_mapping(pose, zero, config)
            assert np.isfinite(result.l)
            assert np.isfinite(result.theta)
            assert np.isfinite(result.offset)
