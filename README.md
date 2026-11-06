# Manipulador robótico — código del laboratorio

El resumen cronológico de la evolución y de las pruebas físicas está en
[`CAMBIOS.md`](CAMBIOS.md).

La configuración completa de Blynk está documentada en
[`docs/BLYNK.md`](docs/BLYNK.md) y respaldada de forma estructurada en
[`docs/blynk_template.json`](docs/blynk_template.json). Ninguno de esos archivos
contiene el token del dispositivo ni claves WiFi.

```
proyecto/
├── brazo_gui.py            ← interfaz (adaptada de la de Diego): DH en vivo + control + rutina
├── brazo/
│   ├── dh.py               ← modelo DH de ustedes (dh-2.py) + matrices A1..A5 / T0_1..T0_5
│   ├── inversa.py          ← su cinemática inversa + búsqueda automática de φ
│   ├── config.py           ← ★ puntos A/B/C, pila, cubo, sensor: SE MIDEN EN EL BRAZO REAL
│   ├── geometria.py        ← sensor en la pinza, pose de escaneo, trayectorias, escena
│   ├── comunicacion.py     ← serial con el ESP32 + simulador con la misma interfaz
│   ├── rutina.py           ← detectar → acercarse → tomar → apilar / ordenar
│   └── dibujo.py           ← vista 3D (también genera figuras para el informe)
├── Manipulador/Manipulador.ino   ← ESP32 modificado
├── platformio.ini                ← compilación y carga del ESP32 desde VS Code / PlatformIO
└── docs/diagrama_conexiones.png  ← diagrama eléctrico (+ .svg y el script que lo genera)
```

## Ver el DH sin abrir la interfaz

```
python -m brazo.dh                 # tabla DH, A1..A5, T0_1..T0_5 y TCP en la postura de calibración
python -m brazo.dh 0 62 -89 0 2    # lo mismo para los ángulos q1..q5 que quieras (grados)
```
(Ejecutar desde la carpeta `proyecto`.) Sirve para copiar al Anexo II del informe.

## Puesta en marcha

1. `pip install numpy matplotlib pyserial`
2. Copiar `Manipulador/secrets.example.h` como `Manipulador/secrets.h` y escribir ahí el
   token de Blynk y las redes WiFi. `secrets.h` está ignorado por Git.
3. Abrir la carpeta en VS Code con PlatformIO y ejecutar **Project Tasks → esp32dev → Upload**.
   PlatformIO instala automáticamente **Blynk** y **Adafruit PWM Servo Driver**.
   Desde una terminal también se puede usar:
   `pio run -e esp32dev -t upload --upload-port /dev/ttyUSB1`.
4. `python brazo_gui.py`
5. Elegir **Simulación** para probar sin brazo, o el puerto del ESP32 (COMx / /dev/ttyUSB0) → **Conectar**.
   Cerrar el Monitor Serie de Arduino antes: el puerto no se puede abrir dos veces.

## Qué hace cada parte

**Python (PC)** calcula el DH, la cinemática inversa y las rutinas generales 1–3.
Le manda al ESP32 porcentajes 0–100 % por serial. La rutina 4 de dos cajas está
también guardada en el firmware y no necesita que Python esté abierto.

**ESP32** mueve los servos con rampa (velocidad), lee el sensor, y habla con Blynk.
Blynk/WiFi corre en el núcleo 0, así una caída de WiFi no congela el brazo.

**App Blynk** mueve el brazo a mano, abre/cierra la pinza, pide rutinas y tiene el STOP.

El panel web de la plantilla **Manipulador Robotico** usa los 19 pines virtuales
V0–V18. Para repetir la secuencia validada no hace falta abrir el computador:
elegir **Apilado 2 cajas** en V8 y pulsar **EJECUTAR** en V9. El selector solo
elige la rutina; el movimiento comienza al pulsar V9. El STOP de V10 queda
disponible durante toda la ejecución.

### La rutina (V8 = 1, 2, 3 o 4 y luego V9, o el botón ▶ de la interfaz)

| N° | Qué hace |
|---|---|
| 1 | Escanea A, B y C con el sensor; mide la pila; toma cada cubo detectado y lo apila |
| 2 | Igual, pero deja los cubos en fila (`FILA_ORDEN`) |
| 3 | Solo escanea (para calibrar el sensor) |
| 4 | Repite el apilado A→B de dos cajas enseñado físicamente (`apilado_ensenado.json`) |

La rutina 4 trabaja como mínimo a velocidad 30/100 (V12 puede aumentarla). Espera una caja en A desde una postura
1 cm más alta que la calibración original, la confirma con dos
mediciones del HC-SR04, avisa que se retiren las manos y comienza tras 2 segundos.
Después de dejar la primera caja vuelve a A y espera la segunda. Su último movimiento
incluye la corrección validada de 5 mm alejándose de la base antes de abrir la pinza.

Detección: el brazo apunta el sensor al punto. Con la DH se sabe a qué distancia **debería**
estar la mesa; si el sensor mide claramente menos, hay un objeto. La altura medida
dice cuántos cubos hay (si hay una pila, los toma de arriba hacia abajo).
Bajada y subida son en línea recta (cartesianas, IK en cada tramo de 1,5 cm).

## Protocolo serial (115200 bd, una línea por mensaje)

| PC → ESP32 | Respuesta |
|---|---|
| `PING` | `LISTO,PONG` |
| `STATE` | `STATE,IDLE\|MOVING,p1,…,p6` |
| `ALL,p1,…,p6` / `SET,n,p` | `LISTO,ALL` / `LISTO,SET` |
| `OPEN` / `CLOSE` / `HOME` | `LISTO,…` |
| `SPEED,v` (1–100) | `LISTO,SPEED` |
| `DIST` | `DIST,cm` o `DIST,NA` (mediana de 5) |
| `STOP` / `RESUME` | `LISTO,STOP` / `LISTO,RESUME` |
| `XYZ,x,y,z` | `LISTO,XYZ` (se muestra en la app) |
| `REPORT,texto` | `LISTO,REPORT` (estado en la app) |

ESP32 → PC sin que se pida: `RUTINA,n` para las rutinas 1–3 (V9),
`STOP_REQUEST` (V10), `READY` (al reiniciar). La rutina 4 se ejecuta en el ESP32.

## Pines virtuales de Blynk

| Pin | Widget | Uso |
|---|---|---|
| V0–V4 | Slider 0–100 | Servos 1–5 |
| V5 / V6 | Botón | Abrir / cerrar pinza |
| V7 | Valor | Distancia del sensor [cm] |
| V8 | Menú (envía 0, 1, 2, 3) | 0 = Apilar, 1 = Ordenar, 2 = Escanear, 3 = Apilado enseñado |
| V9 | Botón | Iniciar rutina |
| V10 | Interruptor | 1 = STOP, 0 = reanudar |
| V11 | Etiqueta | Estado (EJECUTANDO, TOMANDO A, APILANDO 2…) |
| V12 | Slider 1–100 | Velocidad |
| V13–V15 | Valor | X, Y, Z del TCP calculados con DH |
| V16 | Slider 0–100 | Pinza (para probar cuánto abre/cierra) |
| V17 / V18 | Botón Push | Guardar la posición actual del V16 como «abierta» / «cerrada» |

La plantilla publicada usa `TMPL24p6M39YQ`. V12 tiene 30 como valor
predeterminado. Al reconectarse, el ESP32 sincroniza V8 y V12 desde Blynk sin
sobrescribir el valor almacenado en la nube.

## Medidas del brazo (pestaña «Modelo»)

Con regla, de centro a centro de los ejes (tornillo central de cada servo):

| Campo | Qué es |
|---|---|
| d₁ | altura del eje del hombro sobre {0} (cara de arriba de la base giratoria) |
| a₁ | cuánto está adelantado el eje del hombro respecto al eje de giro de la base |
| a₂ | eje hombro → eje codo |
| d₄ | eje codo → eje de la muñeca que sube/baja la pinza (q5) |
| a₅ | eje de esa muñeca → centro entre los dedos de la pinza |
| Base | altura de {0} sobre la mesa |

«Aplicar» actualiza el dibujo y las matrices. «Guardar» las deja en `brazo/medidas.json`.

## Calibrar con el brazo (pestaña «Calibrar»), sin regla

1. Conectar el brazo y poner un cubo en la mesa donde irá el punto A.
2. Mover el brazo (sliders o Blynk) hasta que la pinza **agarre** ese cubo.
3. Elegir «A», dejar marcada «Ajustar altura de mesa» y pulsar **Guardar aquí**.
   La DH calcula x, y del punto. Como el centro de la pinza queda en el centro
   del cubo, la altura de la mesa sale sola: `Z_MESA = z − cubo/2`.
4. Repetir con B, C y la Pila. Para la rutina 2, usar «Fila (agregar)» una vez
   por cada posición de la fila.
5. Anotar dónde quedó el sensor (x₅, y₅, z₅) y pulsar **💾 Guardar calibración**.
   Se guarda en `brazo/zona_trabajo.json` y se carga sola la próxima vez.

Si un punto queda muy cerca de la base, aparece un aviso. Conviene dejar los
puntos a 20–26 cm del eje de la base y con la base girada entre −65° y +40°.

## Lo que hay que medir / revisar con el brazo armado

En `brazo/config.py` (o con la pestaña Calibrar):

- `Z_MESA`: altura de la mesa en el sistema {0}. Si el origen de la DH está sobre la mesa, es negativa.
- `PUNTOS_RECOGIDA`, `PUNTO_APILADO`, `FILA_ORDEN`: deben quedar a 18–28 cm de la base
  (es la zona que alcanza la pinza con los límites de los servos).
- `CUBO`: lado del cubo.
- `SENSOR_OFFSET`: dónde quedó el HC-SR04 en la pinza (ver comentario en el archivo).

Para calibrar el sensor: rutina 3 con los puntos vacíos. La columna «Medido» debe salir
parecida a «Mesa». Si no, corregir `SENSOR_OFFSET` o `Z_MESA`.

En el hardware:

- **ECHO del HC-SR04 → GPIO33 con divisor 1 kΩ / 2 kΩ**. El ECHO sale a 5 V y el ESP32 es de 3,3 V.
- Canal del PCA ↔ servo: el código asume CH10 base … CH15 pinza.
- El VCC de lógica del PCA9685 va a 3,3 V del ESP32. El borne V+ queda sin usar, porque
  los servos se alimentan directo de la fuente.

> WiFi: las redes se configuran solamente en `Manipulador/secrets.h`. El ESP32 se conecta
> a la primera red guardada que encuentre y solo admite redes de 2,4 GHz.
>
> Para el informe o para compartir el código se usa `secrets.example.h`, que no contiene
> credenciales reales. Nunca agreguen `secrets.h` a Git.
