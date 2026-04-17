# Casco Magnificado — Prototipo H.0

Prueba conceptual offline del mapeo "cabeza del flautista → embocador del robot magnificado ×10". Valida la hipótesis central **antes** de construir hardware dedicado.

## Objetivo

Responder, en 1-2 sesiones cortas, a esta pregunta:

> ¿Se siente natural, para un flautista, apuntar con la cabeza a un bisel imaginario a 20 cm frente a la cara?

Si la respuesta es sí, se procede con los tests H.1-H.3 de la propuesta. Si es no, se reconsidera.

## Instalación

```bash
pip install python-osc numpy scipy PyQt5 pyqtgraph
```

(numpy, scipy, PyQt5, pyqtgraph ya son dependencias de Pierre — solo falta `python-osc`.)

## Tres modos de uso

### Modo 1 — mouse (sin teléfono, test rápido)

Para verificar que el visualizador funciona y entender el mapeo antes de conectar el teléfono.

```bash
python run.py mouse
```

El mouse es un placeholder; para sentir la propuesta **se necesita el teléfono**.

### Modo 2 — simulación (ciclo automático)

```bash
python run.py sim
```

Reproduce una secuencia de yaw/pitch/traslación sinusoidal. Útil para validar que la visualización refleja los movimientos esperados.

### Modo 3 — OSC desde el teléfono (el test real)

```bash
python run.py osc
```

Te dirá la IP y el puerto donde está escuchando. Luego configura una app de sensores IMU en tu celular para enviarle datos.

#### Apps de celular probadas

**Opción A — SensorLogger (iOS / Android, gratis)**
- Instala "Sensor Logger" de Kelvin Choi.
- Menú → *Sensors* → habilita "Orientation" (no solo accelerometer).
- Menú → *Settings* → *Data streaming* → activa HTTP streaming, apunta a `http://<IP_DEL_PC>:8000`.
- NOTA: SensorLogger envía JSON por HTTP, no OSC puro. Si prefieres esta app, podemos agregar un bridge (pedímelo). Por ahora recomiendo:

**Opción B — GyrOSC (iOS, ~US\$3)**
- Instala GyrOSC.
- Settings → Host = IP de tu laptop, Port = 8000.
- Enable "Quaternion" streaming.
- Envía mensajes `/gyrosc/quat qw qx qy qz` — ya los reconocemos.

**Opción C — SensorStream IMU+ (Android, gratis)**
- Configura UDP streaming al IP:8000 de tu laptop.
- Selecciona formato "Orientation (quaternion)".

**Opción D — TouchOSC + lua script (cualquier plataforma)**
- Más configurable pero requiere escribir un pequeño script lua.

**Si ninguna app funciona out-of-the-box**, pídeme un bridge: puedo hacer un pequeño servidor HTTP-a-OSC que acepte el formato JSON de SensorLogger y lo traduzca.

### Mensaje OSC custom (si escribes tu propio cliente)

```
/helmet/quat   qw qx qy qz           (quaternion — preferido)
/helmet/euler  yaw pitch roll        (grados o radianes — ver config de la app)
```

## Protocolo del test (5 min por flautista)

1. Conecta celular a la app OSC, valida que el panel diga "● Recibiendo pose".
2. Ponte el celular con una **vincha elástica** o diadema, con la pantalla pegada a la frente (el eje Y del celular alineado con la vertical).
3. Siéntate en posición cómoda de ejecución de flauta.
4. Mira al frente, con la cabeza en posición neutra, y pulsa **"Zero pose"**.
5. **Experimento 1 — yaw**: gira la cabeza suavemente izquierda/derecha. ¿El *impact marker* (punto blanco) se mueve lateralmente sobre el rectángulo naranja (bisel)? ¿Se siente intuitivo?
6. **Experimento 2 — pitch**: inclina la cabeza arriba/abajo. ¿El punto blanco sube y baja sobre el bisel?
7. **Experimento 3 — combinado**: intenta "apuntar" el chorro al centro del bisel moviendo cabeza. Mira el readout de *aiming error* — ¿puedes mantenerlo < 2°?
8. **Experimento 4 — rango de k**: ajusta el slider de magnificación entre 3 y 20. ¿Hay un valor donde se siente "justo"?
9. Reporte subjetivo (3 preguntas):
   - ¿Se siente natural?
   - ¿Hay algún gesto que **no** hace lo que esperarías?
   - ¿Cuánto te costaría construir un diccionario de notas con esta interfaz?

## Ejecutar tests unitarios

```bash
python -m pytest test_mapping.py -v
```

Validan la matemática del mapeo: zero-pose, direcciones de yaw/pitch, escala con k, calibración.

## Limitaciones conocidas del prototipo H.0

- **Sin tracking de posición**: el celular solo da orientación. La posición se asume fija en el origen. Para H.1 en adelante, se reemplaza por ArUco + cámara.
- **Sin captura de soplo**: para H.0 el flujo no se mapea. Lo agregaremos cuando tengamos el sensor de presión.
- **Sin conexión al robot real**: el prototipo solo visualiza; no envía comandos a Pierre. Por diseño — H.0 es una prueba perceptual, no funcional.
- **Drift del yaw**: sin magnetómetro calibrado, el yaw del celular puede derivar. Si notas que el readout se va de centro cuando la cabeza está quieta, vuelve a pulsar "Zero pose".

## Estructura del código

```
prototypes/casco_magnificado/
├── README.md            ← este archivo
├── mapping.py           ← matemática pura del mapeo magnificado
├── sources.py           ← fuentes de pose: OSC, mouse, scripted
├── visualizer.py        ← GUI 3D con readouts y controles
├── run.py               ← entry point con sub-comandos
└── test_mapping.py      ← tests unitarios de la matemática
```

`mapping.py` es la implementación de referencia que migraremos a `src/helmet.py` cuando construyamos el sistema real.

## Qué buscamos observar

Buenas señales (sigue con H.1+):
- El flautista ríe la primera vez que mueve la cabeza y el punto blanco responde.
- Encuentra rápidamente un valor de k que le acomoda (entre 5 y 15).
- Puede "cazar" el centro del bisel en menos de 3 segundos con precisión < 1°.
- Comenta algo como "si esto moviera el robot, yo podría hacer X".

Malas señales (reconsiderar):
- Fatiga cervical evidente en < 10 min.
- El flautista se confunde sobre qué dirección es qué.
- Ningún valor de k se siente cómodo (demasiado sensible o demasiado lento en todo el rango).
- Comentarios del tipo "preferiría mover un joystick".
