# Pierre the Flutist Robot

Software de control para el robot flautista Pierre: un sistema robótico que toca flauta traversa de forma autónoma.

El sistema coordina 3 ejes de movimiento (AMCI stepper motors), un controlador de flujo de aire (Alicat), servos para los dedos, sensor de presión y micrófono, todo comunicado por EtherNet/IP.

## Arquitectura

```
main.py
  ├── QApplication (GUI - PyQt5)
  │     └── Window ← pipe → Musician
  └── Musician (Process - cerebro del robot)
        ├── CommunicationCenter (EtherNet/IP hub)
        ├── AMCIDriver ×3 (ejes X, Z, Alpha)
        │     └── VirtualAxis (thread - generador de referencia)
        ├── FlowControllerDriver (Alicat)
        │     └── VirtualFlow (thread)
        ├── FingersDriver (servos)
        │     └── VirtualFingers (thread)
        ├── PressureSensor
        ├── Microphone
        └── Memory (logging)
```

**Comunicación entre procesos:** `multiprocessing.Pipe` para comandos, `Manager().dict()` para datos compartidos.

## Estructura del proyecto

```
pierre_flutist-claude/
├── main.py                  # Entry point
├── pyproject.toml           # Dependencias y config de pytest
├── src/
│   ├── GUI.py               # Ventana principal (PyQt5)
│   ├── musician.py          # Cerebro del robot (Process)
│   ├── drivers.py           # Drivers de todos los dispositivos
│   ├── communication.py     # Hub EtherNet/IP centralizado
│   ├── cinematica.py        # Cinemática: espacio tarea ↔ espacio articular
│   ├── motor_route.py       # Generación de trayectorias (straight lines)
│   ├── route.py             # Construcción de rutas desde partituras
│   ├── interactive_plot.py  # Widget de plot interactivo
│   ├── manual_move_win.py   # Ventana de control manual
│   ├── settings.json        # Parámetros del robot (dimensiones, límites, PID)
│   ├── look_up_table.json   # Tabla de notas → posiciones
│   ├── forms/               # Formularios PyQt5 (.py + .ui)
│   ├── plots/               # Ventanas de gráficos (live + pasivo)
│   ├── views/               # UI compiladas (mainwindow, startup)
│   ├── dynamics_dicts/      # Perfiles de dinámica (ff, pp, default)
│   ├── lib/ethernet_ip/     # Librería EtherNet/IP modificada
│   └── arduino/, codesys/, pcb/  # Firmware y diseño de hardware
├── scores/                  # Partituras en formato JSON
├── tests/                   # Suite de tests (49 tests)
│   ├── test_cinematica.py   # Cinemática directa/inversa, conversiones
│   ├── test_motor_route.py  # Trayectorias, interpolación, deque compat
│   ├── test_pid.py          # Control PID, clamping, dt medido
│   └── test_integration.py  # VirtualAxis/Flow simulados, determinismo
└── README.md
```

## Instalación

### Requisitos previos
- Python 3.10+
- Acceso a red EtherNet/IP (para hardware real)

### Instalación

```bash
# Clonar el repositorio
git clone <url-del-repo>
cd pierre_flutist-claude

# Instalar dependencias
pip install -e .

# Instalar dependencias de desarrollo (pytest)
pip install -e ".[dev]"
```

### Ejecución

```bash
python main.py
```

> **Nota:** Para operar sin hardware conectado, pasar `connect=False` en `main.py`.

### Tests

```bash
# Correr todos los tests
python -m pytest

# Con detalle
python -m pytest -v

# Con cobertura
python -m pytest --cov=src
```

## Cinemática

El robot tiene 3 ejes motorizados que se mapean al espacio de la tarea:

| Espacio tarea | Descripción | Espacio articular |
|:---:|---|:---:|
| `l` | Distancia labio-embocadura (mm) | `x`, `z` |
| `theta` | Ángulo de incidencia del aire (°) | `x`, `z`, `alpha` |
| `offset` | Desplazamiento lateral (mm) | `alpha` |

La transformación usa los parámetros geométricos de `settings.json` (dh, dv, posición de flauta).

## Partituras

Las partituras son archivos JSON en `scores/` que definen secuencias de estados (posición + flujo + dedos) en el tiempo. Cada estado incluye:
- Coordenadas en espacio de tarea (`l`, `theta`, `offset`)
- Flujo de aire (L/min)
- Configuración de dedos (nota musical)
- Tiempos y velocidades de transición

---

## Plan de Profesionalización

### Fase 0: Determinismo (COMPLETADA - 2026-04-09)

**Objetivo:** Hacer la ejecución de partituras determinista y repetible.

**Problema:** La misma partitura producía movimientos diferentes en cada ejecución debido a jitter temporal, resolución de reloj variable y estructuras de datos ineficientes.

**Cambios implementados:**

| Cambio | Archivos | Detalle |
|---|---|---|
| `time.perf_counter()` | drivers.py, musician.py, main.py | Reemplaza `time.time()` en todos los paths de control. Monotónico, resolución ~100ns |
| `collections.deque` | drivers.py (VirtualAxis, VirtualFlow, VirtualFingers) | `list.pop(0)` O(n) → `deque.popleft()` O(1) |
| Spin-wait híbrido | drivers.py (3 loops de control) | `sleep(grueso)` + spin fino para el último ms. Jitter <1% |
| dt real en PID | drivers.py (`pid_control`) | PID usa dt medido en vez de Ts hardcodeado |
| Sync de ejes | musician.py (`start_loaded_script`) | t_start + 0.2s para que todos los ejes arranquen sincronizados |

**No implementado (postergado):** Reemplazo de `Manager().dict()` por shared memory (impacto bajo, cambio invasivo).

---

### Fase 1: Calidad de Software (EN PROGRESO)

**Objetivo:** Tests, documentación, empaquetado y control de versiones.

| Tarea | Estado | Detalle |
|---|:---:|---|
| `pyproject.toml` | Done | Dependencias, config pytest, metadata |
| `.gitignore` | Done | Python, IDE, OS, recordings, 3D models |
| Tests unitarios (41) | Done | cinematica, motor_route, PID |
| Tests integración (8) | Done | VirtualAxis/Flow simulados, determinismo |
| README.md | Done | Este documento |
| Git: commit inicial | Done | Repositorio en GitHub (privado) |

---

### Fase 2: Rediseno de GUI (EN PROGRESO)

**Objetivo:** Interfaz moderna, clara y profesional para demos y uso diario.

| Tarea | Estado | Detalle |
|---|:---:|---|
| Tema QSS profesional | Done | `src/theme.qss` — paleta azul/verde/rojo, estilos para todos los widgets |
| Constantes visuales | Done | Colores de curvas/puntos/cursor centralizados en constantes |
| Reorganizar panel lateral | Done | Grupos: SOFT STOP, Estado, Transporte, Grabación, Edición, Herramientas |
| Panel de estado (LEDs) | Done | Indicadores: Conectado, En posición, Ejecutando |
| Barra de estado inferior | Done | Archivo activo, espacio (tarea/articular), tiempo del cursor |
| Tooltips descriptivos | Done | Todos los botones tienen tooltip explicativo |
| Secciones colapsables | Done | Lips, Tongue, Dinámica con expand/collapse clickeable |
| Mejorar visualización de rutas | Pendiente | Gráficos más claros con pyqtgraph |
| Editor de partituras mejorado | Pendiente | Timeline visual, drag & drop de estados |
| Manejo de errores en GUI | Pendiente | Mensajes claros al usuario, no crashes silenciosos |

**Criterio de éxito:** Un tesista nuevo puede operar el robot en <30 min con la GUI.

---

### Fase 3: Nuevas Funcionalidades

**Objetivo:** Capacidades avanzadas que aprovechan el determinismo de Fase 0.

| Tarea | Descripción | Prioridad |
|---|---|:---:|
| Auto-tuning | Optimización automática de parámetros (l, theta, offset, flujo) para cada nota | Alta |
| Importador MIDI | Convertir archivos MIDI a partituras JSON | Alta |
| Calibración automatizada | Rutina que encuentra la posición óptima de la flauta | Media |
| Perfiles de dinámica | ff, mf, pp, crescendo, diminuendo parametrizados | Media |
| Grabación + análisis | Grabar audio y comparar con referencia (pitch, timing) | Media |
| Reemplazo Manager().dict() | Shared memory para menor latencia IPC | Baja |

**Pre-requisito:** Fase 0 (determinismo) permite que auto-tuning converja de forma repetible.

---

## Guía para tesistas nuevos

### Conceptos clave

1. **Multiprocessing:** Cada dispositivo corre en su propio proceso. La comunicación es por `Pipe` (comandos) y `Manager().dict()` (datos compartidos). No usar variables globales.

2. **VirtualAxis/Flow/Fingers:** Threads dentro de cada driver que generan la referencia punto a punto. Consumen de un `deque` de tuplas `(t, pos, vel)`.

3. **Control loop:** Cada driver tiene un loop que corre a frecuencia fija (Ts) usando spin-wait. El PID usa el dt real medido, no el Ts nominal.

4. **Cinemática:** El robot piensa en espacio de tarea (l, theta, offset) pero se mueve en espacio articular (x, z, alpha). `cinematica.py` hace la conversión.

5. **Partituras:** Son JSON con una lista de estados. Cada estado define una posición en espacio de tarea + flujo + dedos + tiempo. `route.py` construye las trayectorias.

### Cómo modificar el código

- **Agregar una nota nueva a la look-up table:** Editar `src/look_up_table.json`
- **Cambiar parámetros PID:** Editar `src/settings.json` (secciones kp, ki, kd por eje)
- **Cambiar límites de seguridad:** Editar `src/settings.json` (max_pos, max_vel por eje)
- **Agregar un formulario nuevo:** Crear .ui en Qt Designer, compilar, agregar form en `src/forms/`
- **Correr tests después de cambios:** `python -m pytest -v`

### Seguridad

- Los drivers tienen límites de posición y velocidad hardcodeados en `settings.json`
- El watchdog detiene los motores si no hay comunicación
- Siempre hacer homing antes de operar
- No modificar los límites sin supervisión del profesor
