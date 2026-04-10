"""
Tests para las transformaciones cinemáticas del robot.
Verifica la consistencia entre el espacio de la tarea (l, theta, offset)
y el espacio de las articulaciones (x, z, alpha).
"""
import pytest
import numpy as np
from src.cinematica import (
    get_x_z_alpha, get_l_theta_of, get_pos_punta, get_x_z_from_punta,
    x_mm_to_units, alpha_angle_to_units, State, DATA,
)


class TestForwardInverse:
    """La cinemática directa e inversa deben ser inversas entre sí."""

    def test_roundtrip_task_to_joint_to_task(self):
        """(l, theta, o) -> (x, z, alpha) -> (l, theta, o) debe dar lo mismo."""
        l, theta, o = 50.0, 30.0, 5.0
        x, z, alpha = get_x_z_alpha(l, theta, o)
        l2, theta2, o2 = get_l_theta_of(x, z, alpha)
        assert abs(l - l2) < 0.01
        assert abs(theta - theta2) < 0.01
        assert abs(o - o2) < 0.01

    def test_roundtrip_joint_to_task_to_joint(self):
        """(x, z, alpha) -> (l, theta, o) -> (x, z, alpha) debe dar lo mismo."""
        x, z, alpha = 100.0, 50.0, 20.0
        l, theta, o = get_l_theta_of(x, z, alpha)
        x2, z2, alpha2 = get_x_z_alpha(l, theta, o)
        assert abs(x - x2) < 0.01
        assert abs(z - z2) < 0.01
        assert abs(alpha - alpha2) < 0.01

    @pytest.mark.parametrize("l,theta,o", [
        (30.0, 20.0, 0.0),
        (60.0, 45.0, 10.0),
        (80.0, 60.0, -5.0),
        (10.0, 10.0, 2.0),
    ])
    def test_roundtrip_parametric(self, l, theta, o):
        x, z, alpha = get_x_z_alpha(l, theta, o)
        l2, theta2, o2 = get_l_theta_of(x, z, alpha)
        assert abs(l - l2) < 0.01
        assert abs(theta - theta2) < 0.01
        assert abs(o - o2) < 0.01


class TestPuntaTransform:
    """La transformación de punta (boca) debe ser invertible."""

    def test_roundtrip_punta(self):
        x, z, alpha = 100.0, 50.0, 0.3
        x2, z2 = get_pos_punta(x, z, alpha)
        x_back, z_back = get_x_z_from_punta(x2, z2, alpha)
        assert abs(x - x_back) < 0.001
        assert abs(z - z_back) < 0.001

    def test_punta_zero_angle(self):
        """Con alpha=0, la punta se desplaza solo en x (dh) y z (dv)."""
        dh = DATA["physical_constants"]["dh"]
        dv = DATA["physical_constants"]["dv"]
        x2, z2 = get_pos_punta(0, 0, 0)
        assert abs(x2 - dh) < 0.001
        assert abs(z2 - dv) < 0.001


class TestUnitConversions:
    """Conversiones mm->pasos y °->pasos."""

    def test_x_mm_to_units(self):
        # 8mm por vuelta, 4000 pasos por vuelta => 500 pasos/mm
        assert x_mm_to_units(1.0) == 500
        assert x_mm_to_units(0.0) == 0
        assert x_mm_to_units(10.0) == 5000

    def test_alpha_angle_to_units(self):
        # 360° por vuelta, 4000 pasos por vuelta => 11.11 pasos/°
        assert alpha_angle_to_units(360) == 4000
        assert alpha_angle_to_units(0) == 0
        assert alpha_angle_to_units(180) == 2000

    def test_negative_values(self):
        assert x_mm_to_units(-1.0) == -500
        assert alpha_angle_to_units(-90) == -1000


class TestState:
    """La clase State mantiene consistencia entre coordenadas."""

    def test_state_creation(self):
        s = State(50, 30, 5, 10.0)
        assert not s.not_posible
        assert s.r == 50
        assert s.theta == 30
        assert s.o == 5
        assert s.flow == 10.0

    def test_state_set_x_updates_task_coords(self):
        s = State(50, 30, 5, 10.0)
        old_r = s.r
        s.x = s.x + 10  # cambiar x debería recalcular r, theta, o
        # No podemos saber el valor exacto, pero r debería haber cambiado
        assert s.r != old_r or s.theta != 30 or s.o != 5

    def test_state_set_r_updates_joint_coords(self):
        s = State(50, 30, 5, 10.0)
        old_x = s.x
        s.r = 60  # cambiar r debería recalcular x, z, alpha
        assert s.x != old_x

    def test_state_str(self):
        s = State(50, 30, 5, 10.0)
        text = str(s)
        assert "r:" in text
        assert "theta:" in text
