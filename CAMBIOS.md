# Historial de cambios

Este documento resume la evolución del repositorio desde su creación y deja
registrada la configuración que fue validada con el brazo físico.

## 2026-11-05 — Versión inicial

- Se creó el repositorio con su `README.md` y reglas básicas de `.gitignore`.
- Se incorporó el firmware del ESP32 para controlar seis servos mediante el
  PCA9685, medir distancia con el HC-SR04 y comunicarse por USB y Blynk.
- Se añadió la aplicación de escritorio en Python con:
  - control manual y visualización 3D;
  - modelo Denavit-Hartenberg y cinemática inversa;
  - comunicación con el brazo real y con un simulador;
  - rutinas genéricas para apilar, ordenar y escanear;
  - herramientas para calibrar puntos de trabajo y el sensor.
- Se añadieron el diagrama de conexiones y la configuración inicial de la zona
  de trabajo.

## 2026-11-06 — Seguridad, calibración y apilado validado

### Credenciales

- El token de Blynk y las redes WiFi se retiraron del código fuente actual.
- Se creó `Manipulador/secrets.example.h` con valores de ejemplo.
- El archivo local `Manipulador/secrets.h` quedó excluido mediante `.gitignore`.
- El `README.md` ahora explica cómo crear el archivo local de credenciales.

### Calibración física

- Se actualizaron en `brazo/zona_trabajo.json` las posturas reales de recogida A,
  el punto B y la pila.
- Se calibró una separación aproximada de 24 cm y un giro de base cercano a 84°
  entre A y B.
- El agarre en A se desplazó 1 cm hacia la base para sujetar mejor el cuerpo de
  la caja.
- Se calibró la pinza en 95 % abierta y 60 % cerrada.
- Las cajas utilizadas miden aproximadamente 8,45 × 3,44 × 3,04 cm.

### Rutina 4: apilado enseñado de dos cajas

- Se creó `brazo/apilado_ensenado.json`, que conserva todas las posturas
  comprobadas físicamente.
- Se añadió la opción **4 · Apilado enseñado (2 cajas)** a la interfaz.
- La rutina espera una caja en A, la toma, la traslada a B y vuelve a A para
  esperar la segunda.
- El HC-SR04 confirma la presencia mediante dos mediciones consecutivas.
- Tras detectar una caja, el sistema avisa que se retiren las manos y espera
  2 segundos antes de iniciar el movimiento.
- La espera máxima de cada caja es de 120 segundos y puede interrumpirse con
  **Detener/STOP**.
- La segunda caja se libera después de una corrección final de 5 mm alejándose
  de la base, que dejó ambas cajas alineadas durante el ensayo.
- La velocidad se aumentó de 10 a 20 sobre 100. La secuencia completa fue
  validada satisfactoriamente en simulación y con el brazo físico.

### Blynk

- El firmware ahora admite una cuarta opción en el menú V8.
- V8 acepta los valores 0–3 y V9 puede iniciar las rutinas 1–4.
- Se publicó el panel web de la plantilla `TMPL24p6M39YQ` en el dispositivo
  **Brazo ESP32** con los 19 controles y lecturas V0–V18.
- V8 ofrece **Apilar**, **Apilado 2 cajas**, **Ordenar fila** y **Solo escanear**;
  V9 ejecuta la opción elegida y V10 funciona como STOP/reanudación.
- Se añadieron al panel la velocidad, el estado de rutina, los seis controles
  manuales, la calibración de la pinza, el HC-SR04 y las coordenadas TCP.
- V12 quedó con 30 como valor predeterminado para acelerar el recorrido.
- Se corrigió la reconexión del firmware: ahora sincroniza V8 y V12 desde Blynk
  sin sobrescribir inmediatamente la velocidad almacenada en la nube.
- La definición reproducible del panel quedó respaldada en
  `docs/blynk_template.json`, sin tokens ni claves WiFi.

### Ajuste final de aproximación y velocidad

- La postura de espera del HC-SR04 en A se elevó 1 cm para impedir que los dedos
  rocen la caja antes de comenzar el agarre.
- El umbral de presencia se amplió de 10,5 a 12 cm para compensar la nueva altura.
- La velocidad de la rutina enseñada aumentó de 20 a 30 sobre 100, un 50 % más,
  manteniendo los puntos intermedios y la espera de seguridad de 2 segundos.
- Se añadió `platformio.ini` para compilar y cargar el ESP32 desde PlatformIO en
  VS Code con el entorno `esp32dev`.

### Corrección de ejecución desde Blynk

- Se eliminó la dependencia del programa `brazo_gui.py` para **Apilado 2 cajas**.
  La trayectoria A→B, sus puntos intermedios, la pinza 95/60 % y la espera del
  HC-SR04 quedaron implementados como una máquina de estados en el ESP32.
- V9 inicia esta secuencia directamente; las rutinas genéricas 1–3 conservan la
  comunicación con Python.
- El proceso sigue atendiendo Blynk durante todo el recorrido: V10 detiene y
  cancela la secuencia sin esperar a que termine un tramo.
- Se añadieron mensajes de estado para la colocación, detección, traslado y
  depósito de ambas cajas, además del timeout de 120 segundos por caja.
- Al iniciar, la rutina corrige automáticamente velocidades antiguas inferiores
  a 30 y respeta cualquier valor mayor configurado en V12.
- El firmware autónomo se compiló y cargó con PlatformIO; se verificó por serial
  que el ESP32 quedó `IDLE`, conectado a WiFi y conectado a Blynk.

## Archivos principales modificados

| Archivo | Motivo |
|---|---|
| `.gitignore` | Evitar que las credenciales locales se suban a Git |
| `Manipulador/Manipulador.ino` | Credenciales externas y soporte de Rutina 4 en Blynk |
| `Manipulador/secrets.example.h` | Plantilla segura para Blynk y WiFi |
| `platformio.ini` | Compilación y carga reproducibles del ESP32 desde VS Code |
| `brazo/zona_trabajo.json` | Puntos A, B y pila calibrados físicamente |
| `brazo/apilado_ensenado.json` | Trayectoria exacta y parámetros de la prueba exitosa |
| `brazo/rutina.py` | Detección, espera segura y ejecución automática de dos cajas |
| `README.md` | Instalación, credenciales y explicación de la nueva rutina |
| `docs/BLYNK.md` | Uso, verificación y reconstrucción del panel Blynk |
| `docs/blynk_template.json` | Respaldo estructurado de flujos y widgets V0–V18 |

## Alcance de la calibración

La Rutina 4 depende de la posición física de la base, las marcas A/B, la altura
de la mesa, el montaje del HC-SR04 y las dimensiones indicadas de las cajas. Si
alguno de esos elementos cambia, se deben volver a enseñar las posturas antes de
ejecutarla.
