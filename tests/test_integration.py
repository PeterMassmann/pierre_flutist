"""
Tests de integración: simulan el flujo completo de ejecución de partituras
sin hardware real. Verifican que los ejes virtuales consumen rutas
correctamente y que las trayectorias son deterministas.
"""
import pytest
import time
import threading
from multiprocessing import Event, Pipe
from collections import deque
from src.drivers import VirtualAxis, VirtualFlow, VirtualFingers
from src.motor_route import get_1D_route, get_value_from_func_2d


class TestVirtualAxisSimulated:
    """Simula un VirtualAxis sin hardware."""

    def _make_axis(self):
        running = Event()
        running.set()
        parent_conn, child_conn = Pipe()
        t0 = time.perf_counter()
        axis = VirtualAxis(running, 0.01, t0, child_conn)
        axis.daemon = True
        return axis, parent_conn, running

    def test_axis_starts_at_zero(self):
        axis, conn, running = self._make_axis()
        assert axis.pos == 0
        assert axis.vel == 0
        running.clear()

    def test_merge_ref_and_follow(self):
        """El eje virtual debe seguir una ruta cargada."""
        axis, conn, running = self._make_axis()
        axis.start()
        time.sleep(0.05)  # dejar que arranque

        # Crear ruta: ir de 0 a 1000 en 0.5s
        t_now = time.perf_counter() - axis.t0
        route = [
            (t_now, 0, 0),
            (t_now + 0.25, 500, 2000),
            (t_now + 0.5, 1000, 0),
        ]
        conn.send(["merge_ref", route])
        time.sleep(0.6)

        # Al final de la ruta, la posición debe estar en ~1000
        assert abs(axis.pos - 1000) < 50
        assert axis.vel == 0

        running.clear()
        axis.join(timeout=1)

    def test_stop_command(self):
        """El comando stop debe frenar el eje."""
        axis, conn, running = self._make_axis()
        axis.start()
        time.sleep(0.05)

        t_now = time.perf_counter() - axis.t0
        route = [(t_now, 0, 0), (t_now + 5.0, 50000, 10000)]
        conn.send(["merge_ref", route])
        time.sleep(0.2)

        conn.send(["stop"])
        time.sleep(0.1)
        pos_after_stop = axis.pos

        # La posición no debe seguir cambiando
        time.sleep(0.2)
        assert abs(axis.pos - pos_after_stop) < 10

        running.clear()
        axis.join(timeout=1)

    def test_deque_operations(self):
        """Verificar que merge_ref funciona con el deque."""
        axis, conn, running = self._make_axis()
        assert isinstance(axis.ref, deque)

        # merge_ref con extend
        t_now = time.perf_counter() - axis.t0
        new_ref = [(t_now + 1, 100, 0), (t_now + 2, 200, 0)]
        axis.merge_ref(new_ref)
        assert len(axis.ref) == 3  # original (0,0,0) + 2 nuevos

        running.clear()


class TestVirtualFlowSimulated:
    """Simula VirtualFlow sin hardware."""

    def _make_flow(self):
        running = Event()
        running.set()
        parent_conn, child_conn = Pipe()
        t0 = time.perf_counter()
        flow = VirtualFlow(running, 0.01, t0, child_conn)
        flow.daemon = True
        return flow, parent_conn, running

    def test_flow_follows_ramp(self):
        flow, conn, running = self._make_flow()
        flow.start()
        time.sleep(0.05)

        t_now = time.perf_counter() - flow.t0
        route = [(t_now, 0), (t_now + 0.3, 20.0)]
        conn.send(["merge_ref", route, 0.0, 0.0])
        time.sleep(0.4)

        assert abs(flow.flow - 20.0) < 2.0

        running.clear()
        flow.join(timeout=1)

    def test_flow_clamped_to_50(self):
        """El flujo no debe exceder 50."""
        flow, conn, running = self._make_flow()
        flow.start()
        time.sleep(0.05)

        t_now = time.perf_counter() - flow.t0
        route = [(t_now, 0), (t_now + 0.1, 80.0)]  # pide 80, pero max es 50
        conn.send(["merge_ref", route, 0.0, 0.0])
        time.sleep(0.2)

        assert flow.flow <= 50.0

        running.clear()
        flow.join(timeout=1)


class TestDeterminism:
    """
    Verifica que la misma ruta produce los mismos resultados.
    Este es el test más importante de la Fase 0.
    """

    def test_trajectory_is_deterministic(self):
        """
        Generar la misma ruta 1D dos veces debe dar resultados idénticos
        (la generación es puramente matemática, sin hardware).
        """
        route_a = get_1D_route(0, 10000, 50, acc=4000, dec=4000)
        route_b = get_1D_route(0, 10000, 50, acc=4000, dec=4000)

        temps_a, pos_a, vel_a = route_a
        temps_b, pos_b, vel_b = route_b

        assert len(temps_a) == len(temps_b)
        for i in range(len(temps_a)):
            assert temps_a[i] == temps_b[i]
            assert pos_a[i] == pos_b[i]
            assert vel_a[i] == vel_b[i]

    def test_interpolation_deterministic(self):
        """
        La interpolación sobre la misma ruta con el mismo t
        debe dar exactamente el mismo resultado.
        """
        func = deque([(0, 0, 0), (1, 1000, 1000), (2, 2000, 0)])
        results = []
        for _ in range(100):
            pos, vel = get_value_from_func_2d(0.5, func)
            results.append((pos, vel))

        # Todos los resultados deben ser idénticos
        assert all(r == results[0] for r in results)


class TestPipeBreakSafety:
    """Tests de seguridad: verificar que pipe roto no deja eje en movimiento."""

    def test_axis_stops_when_running_cleared(self):
        """Si running se limpia, el eje virtual debe parar."""
        running = Event()
        running.set()
        parent_conn, child_conn = Pipe()
        t0 = time.perf_counter()
        axis = VirtualAxis(running, 0.01, t0, child_conn)
        axis.daemon = True
        axis.start()
        time.sleep(0.05)

        # Enviar ruta larga
        t_now = time.perf_counter() - t0
        route = [(t_now, 0, 0), (t_now + 10.0, 100000, 10000)]
        parent_conn.send(["merge_ref", route])
        time.sleep(0.2)

        # Limpiar running (simula shutdown)
        running.clear()
        axis.join(timeout=2)

        # El proceso debe haber terminado
        assert not axis.is_alive()

    def test_axis_stops_safely_on_closed_pipe(self):
        """Si el pipe se cierra, el eje virtual debe parar limpiamente (no crashear)."""
        running = Event()
        running.set()
        parent_conn, child_conn = Pipe()
        t0 = time.perf_counter()
        axis = VirtualAxis(running, 0.01, t0, child_conn)
        axis.daemon = True
        axis.start()
        time.sleep(0.05)

        # Cerrar el pipe del lado del padre
        parent_conn.close()
        time.sleep(0.3)

        # El eje debe haberse detenido limpiamente (break del loop)
        axis.join(timeout=2)
        assert not axis.is_alive()

    def test_flow_stops_on_running_clear(self):
        """VirtualFlow debe parar cuando running se limpia."""
        running = Event()
        running.set()
        parent_conn, child_conn = Pipe()
        t0 = time.perf_counter()
        flow = VirtualFlow(running, 0.01, t0, child_conn)
        flow.daemon = True
        flow.start()
        time.sleep(0.05)

        t_now = time.perf_counter() - t0
        parent_conn.send(["merge_ref", [(t_now, 0), (t_now + 5, 30)], 0, 0])
        time.sleep(0.2)

        running.clear()
        flow.join(timeout=2)
        assert not flow.is_alive()
