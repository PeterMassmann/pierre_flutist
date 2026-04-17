"""Entrada del prototipo H.0.

Uso:
    python run.py osc              # escucha OSC en puerto 8000 (default)
    python run.py osc --port 9000  # puerto custom
    python run.py mouse            # modo mouse (sin hardware)
    python run.py sim              # secuencia simulada (regresión)
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from mapping import HeadPose, euler_to_quat_zyx
from sources import MouseSource, OSCSource, ScriptedSource
from visualizer import run_with_source


def make_sim_script():
    def script(t: float) -> HeadPose:
        yaw = 0.25 * math.sin(2 * math.pi * 0.2 * t)
        pitch = 0.15 * math.sin(2 * math.pi * 0.13 * t + 1.2)
        q = euler_to_quat_zyx(yaw, pitch, 0.0)
        z = 0.05 * math.sin(2 * math.pi * 0.07 * t)
        return HeadPose(
            position=np.array([0.0, 0.0, z]),
            quaternion=q,
        )
    return script


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode", choices=["osc", "mouse", "sim"],
        help="fuente de pose",
    )
    parser.add_argument("--host", default="0.0.0.0",
                        help="host OSC (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8000,
                        help="puerto OSC (default: 8000)")
    parser.add_argument("--verbose", action="store_true",
                        help="mostrar mensajes OSC recibidos")
    args = parser.parse_args()

    if args.mode == "osc":
        source = OSCSource(host=args.host, port=args.port, verbose=args.verbose)
    elif args.mode == "mouse":
        source = MouseSource()
    elif args.mode == "sim":
        source = ScriptedSource(make_sim_script())
    else:
        print(f"Modo desconocido: {args.mode}", file=sys.stderr)
        return 1

    return run_with_source(source)


if __name__ == "__main__":
    sys.exit(main())
