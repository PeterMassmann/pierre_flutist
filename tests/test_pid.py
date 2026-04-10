"""
Tests para el controlador PID del AMCIDriver.
Se testea la lógica del PID de forma aislada, sin hardware.
"""
import pytest
from unittest.mock import MagicMock
from multiprocessing import Value


class FakePIDController:
    """
    Extrae la lógica PID de AMCIDriver para testarla sin instanciar
    todo el driver (que requiere hardware y pipes).
    Replica exactamente pid_control() de drivers.py.
    """
    def __init__(self, Kp=0, Ki=5, Kd=0.01, Kp_vel=0, Ki_vel=0, Kd_vel=0,
                 pos_min=-2000, pos_max=100000):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kp_vel = Kp_vel
        self.Ki_vel = Ki_vel
        self.Kd_vel = Kd_vel
        self.Ka = 1.0
        self.Ka_vel = 1.0
        self.e0 = 0
        self.MV_I0 = 0
        self.MV = 0
        self.e0_vel = 0
        self.MV_I0_vel = 0
        self.MV_vel = 0
        self.last_pos = 0
        self.pos_min = pos_min
        self.pos_max = pos_max
        self.MV_low = pos_min
        self.MV_high = pos_max
        self.MV_low_vel = -200000
        self.MV_high_vel = 200000

    def pid_control(self, ref_pos, ref_vel, dt, encoder_pos):
        SP = ref_pos
        CV = encoder_pos
        e = SP - CV
        MV_P = self.Kp * e

        SP_vel = ref_vel
        CV_vel = (CV - self.last_pos) / dt
        e_vel = SP_vel - CV_vel
        MV_P_vel = self.Kp_vel * e_vel

        MV_I_vel = self.MV_I0_vel + self.Ki_vel * dt * e_vel - self.Ka_vel * (self.MV_vel - max(self.MV_low_vel, min(self.MV_high_vel, self.MV_vel)))
        MV_D_vel = self.Kd_vel * (e_vel - self.e0_vel) / dt

        MV_I = self.MV_I0 + self.Ki * dt * e - self.Ka * (self.MV - max(self.MV_low, min(self.MV_high, self.MV)))
        MV_D = self.Kd * (e - self.e0) / dt

        raw_pos = int(round(SP + MV_P + MV_I + MV_D, 0))
        self.MV = max(self.pos_min, min(self.pos_max, raw_pos))
        self.e0 = e
        self.MV_I0 = MV_I

        raw_vel = int(round(SP_vel + MV_P_vel + MV_I_vel + MV_D_vel, 0))
        self.MV_vel = max(-200000, min(200000, raw_vel))
        self.e0_vel = e_vel
        self.MV_I0_vel = MV_I_vel

        self.last_pos = CV
        return self.MV, self.MV_vel


class TestPIDBasic:
    """Comportamiento básico del PID."""

    def test_zero_error_produces_no_correction(self):
        """Si la posición real == referencia, la salida debe ser ~referencia."""
        pid = FakePIDController(Kp=0, Ki=0, Kd=0)
        pos, vel = pid.pid_control(ref_pos=1000, ref_vel=0, dt=0.01, encoder_pos=1000)
        assert pos == 1000
        assert vel == 0

    def test_proportional_responds_to_error(self):
        """Con Kp > 0, un error de posición genera corrección."""
        pid = FakePIDController(Kp=1.0, Ki=0, Kd=0)
        pos, vel = pid.pid_control(ref_pos=1000, ref_vel=0, dt=0.01, encoder_pos=900)
        # Error = 100, MV_P = 100, output = ref + correction = 1000 + 100 = 1100
        assert pos == 1100

    def test_integral_accumulates_over_time(self):
        """Con Ki > 0, el error se acumula."""
        pid = FakePIDController(Kp=0, Ki=5, Kd=0)
        dt = 0.01

        # Primera iteración: error = 100, MV_I = 5 * 0.01 * 100 = 5
        pos1, _ = pid.pid_control(1000, 0, dt, 900)

        # Segunda iteración: error sigue, MV_I acumula
        pos2, _ = pid.pid_control(1000, 0, dt, 900)

        assert pos2 > pos1  # la integral debe haber crecido


class TestPIDSafety:
    """El PID debe respetar los límites de posición."""

    def test_position_clamping_upper(self):
        """La salida no debe exceder pos_max."""
        pid = FakePIDController(Kp=100, Ki=0, Kd=0, pos_max=5000)
        pos, _ = pid.pid_control(ref_pos=10000, ref_vel=0, dt=0.01, encoder_pos=0)
        assert pos <= 5000

    def test_position_clamping_lower(self):
        """La salida no debe ser menor que pos_min."""
        pid = FakePIDController(Kp=100, Ki=0, Kd=0, pos_min=-2000)
        pos, _ = pid.pid_control(ref_pos=-50000, ref_vel=0, dt=0.01, encoder_pos=0)
        assert pos >= -2000

    def test_velocity_clamping(self):
        """La velocidad no debe exceder SAFETY_MAX_VELOCITY."""
        pid = FakePIDController(Kp_vel=1000, Ki_vel=0, Kd_vel=0)
        _, vel = pid.pid_control(ref_pos=0, ref_vel=999999, dt=0.01, encoder_pos=0)
        assert abs(vel) <= 200000


class TestPIDWithMeasuredDt:
    """El PID usa dt medido — verificar que funciona con dt variables."""

    def test_different_dt_same_total_time(self):
        """
        Dos secuencias con distinto dt pero mismo tiempo total
        deben acumular integrales similares.
        """
        pid_a = FakePIDController(Kp=0, Ki=10, Kd=0)
        pid_b = FakePIDController(Kp=0, Ki=10, Kd=0)

        # Secuencia A: 10 pasos de 0.01s = 0.1s total
        for _ in range(10):
            pid_a.pid_control(1000, 0, 0.01, 900)

        # Secuencia B: 5 pasos de 0.02s = 0.1s total
        for _ in range(5):
            pid_b.pid_control(1000, 0, 0.02, 900)

        # Las integrales deben ser iguales (Ki * dt * e sumado)
        assert abs(pid_a.MV_I0 - pid_b.MV_I0) < 1.0

    def test_dt_clamp_protects_against_extreme_values(self):
        """Con dt muy pequeño, el cálculo no debe producir overflow."""
        pid = FakePIDController(Kp=0, Ki=5, Kd=0.01)
        # dt = 0.001 (mínimo del clamp en el main loop)
        pos, vel = pid.pid_control(1000, 0, 0.001, 900)
        assert isinstance(pos, int)
        assert isinstance(vel, int)
