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
- Para usar la rutina nueva desde Blynk es necesario cargar el firmware
  actualizado y añadir **Apilado enseñado** como cuarto elemento del menú V8.

## Archivos principales modificados

| Archivo | Motivo |
|---|---|
| `.gitignore` | Evitar que las credenciales locales se suban a Git |
| `Manipulador/Manipulador.ino` | Credenciales externas y soporte de Rutina 4 en Blynk |
| `Manipulador/secrets.example.h` | Plantilla segura para Blynk y WiFi |
| `brazo/zona_trabajo.json` | Puntos A, B y pila calibrados físicamente |
| `brazo/apilado_ensenado.json` | Trayectoria exacta y parámetros de la prueba exitosa |
| `brazo/rutina.py` | Detección, espera segura y ejecución automática de dos cajas |
| `README.md` | Instalación, credenciales y explicación de la nueva rutina |

## Alcance de la calibración

La Rutina 4 depende de la posición física de la base, las marcas A/B, la altura
de la mesa, el montaje del HC-SR04 y las dimensiones indicadas de las cajas. Si
alguno de esos elementos cambia, se deben volver a enseñar las posturas antes de
ejecutarla.
