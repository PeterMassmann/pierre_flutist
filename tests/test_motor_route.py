"""
Tests para la generación de trayectorias.
Verifica perfiles de velocidad, interpolación y funciones auxiliares.
"""
import pytest
import numpy as np
from collections import deque
from src.motor_route import (
    get_min_T, get_1D_route, get_value_from_func, get_value_from_func_2d,
    third_order_time_scale,
)


class TestGetMinT:
    """Tiempo mínimo para perfil trapezoidal de velocidad."""

    def test_zero_distance(self):
        assert get_min_T(0, 100, 100) == 0

    def test_triangular_profile(self):
        """Distancia corta => perfil triangular (no alcanza v_max)."""
        T = get_min_T(10, 100, 100)
        assert T > 0

    def test_trapezoidal_profile(self):
        """Distancia larga => perfil trapezoidal (alcanza v_max)."""
        T = get_min_T(1000000, 100, 100, v_max=4000)
        assert T > 0
        # Con distancia suficiente, debería incluir un plateau a v_max
        t_acc = 4000 / 100
        t_dec = 4000 / 100
        assert T > t_acc + t_dec  # el tiempo total es mayor que solo acc+dec

    def test_higher_speed_means_shorter_time(self):
        T_slow = get_min_T(10000, 50, 50, v_max=2000)
        T_fast = get_min_T(10000, 50, 50, v_max=4000)
        assert T_fast <= T_slow


class TestGet1DRoute:
    """Trayectorias 1D con perfil trapezoidal."""

    def test_zero_distance_returns_empty(self):
        result = get_1D_route(100, 100, 50)
        assert result == []

    def test_basic_route_structure(self):
        temps, positions, speeds = get_1D_route(0, 5000, 50)
        assert len(temps) == len(positions) == len(speeds)
        assert len(temps) > 0
        # Debe empezar en 0 y terminar en 5000
        assert abs(positions[0] - 0) < 1
        assert abs(positions[-1] - 5000) < 1

    def test_starts_and_ends_at_rest(self):
        temps, positions, speeds = get_1D_route(0, 10000, 50)
        # La velocidad final debe ser 0
        assert speeds[-1] == 0

    def test_time_is_monotonic(self):
        temps, positions, speeds = get_1D_route(0, 10000, 50)
        for i in range(1, len(temps)):
            assert temps[i] >= temps[i-1]

    def test_higher_speed_param_means_shorter_time(self):
        t_slow, _, _ = get_1D_route(0, 10000, 20)
        t_fast, _, _ = get_1D_route(0, 10000, 80)
        assert t_fast[-1] < t_slow[-1]

    def test_negative_direction(self):
        temps, positions, speeds = get_1D_route(5000, 0, 50)
        assert len(temps) > 0
        assert abs(positions[0] - 5000) < 1
        assert abs(positions[-1] - 0) < 1


class TestInterpolation:
    """Funciones de interpolación usadas por los ejes virtuales."""

    def test_get_value_from_func_basic(self):
        """Interpolación lineal entre dos puntos."""
        func = [(0, 0), (10, 100)]
        assert get_value_from_func(5, func) == 50
        assert get_value_from_func(0, func) == 0

    def test_get_value_from_func_boundary(self):
        """En los extremos debe devolver el valor exacto."""
        func = [(0, 0), (10, 100)]
        assert get_value_from_func(0, func) == 0

    def test_get_value_from_func_beyond_end(self):
        """Más allá del final, devuelve el último valor."""
        func = [(0, 0), (10, 100)]
        assert get_value_from_func(20, func) == 100

    def test_get_value_from_func_2d_basic(self):
        """Interpolación lineal 2D (posición y velocidad)."""
        func = [(0, 0, 0), (10, 100, 20)]
        pos, vel = get_value_from_func_2d(5, func)
        assert abs(pos - 50) < 1
        assert abs(vel - 10) < 1

    def test_get_value_from_func_works_with_deque(self):
        """Debe funcionar con deque (cambio de Fase 0)."""
        func = deque([(0, 0), (10, 100), (20, 200)])
        result = get_value_from_func(5, func)
        assert abs(result - 50) < 1

    def test_get_value_from_func_2d_works_with_deque(self):
        """Debe funcionar con deque (cambio de Fase 0)."""
        func = deque([(0, 0, 0), (10, 100, 20), (20, 200, 20)])
        pos, vel = get_value_from_func_2d(5, func)
        assert abs(pos - 50) < 1


class TestTimeScaling:
    """Funciones de escalado temporal (polinomios de orden 3 y 5)."""

    def test_third_order_boundaries(self):
        """s(0) = 0, s(T) = 1 para escalado de tercer orden."""
        T = 5.0
        assert abs(third_order_time_scale(0, T)) < 0.001
        assert abs(third_order_time_scale(T, T) - 1.0) < 0.001

    def test_time_scale_monotonic(self):
        """El escalado debe ser monótono creciente."""
        T = 5.0
        prev = 0
        for t in np.linspace(0, T, 50):
            s = third_order_time_scale(t, T)
            assert s >= prev - 0.001
            prev = s
