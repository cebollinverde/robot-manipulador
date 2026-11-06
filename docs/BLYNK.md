# Blynk — configuración del manipulador

## Estado publicado

- Plantilla: **Manipulador Robotico**
- Template ID: `TMPL24p6M39YQ`
- Dispositivo: **Brazo ESP32**
- Flujos: V0–V18
- Panel web: publicado y aplicado al dispositivo
- Velocidad predeterminada: 30/100

El token del dispositivo y las redes WiFi no forman parte de este respaldo.
Solo deben existir en `Manipulador/secrets.h`, que Git ignora.

## Ejecutar el apilado validado

1. Encender la fuente y el ESP32.
2. No es necesario abrir `brazo_gui.py`: esta secuencia está almacenada en el
   firmware y el ESP32 la ejecuta de forma autónoma.
3. En Blynk, confirmar que **Brazo ESP32** aparece `Online`.
4. En **Seleccionar rutina**, elegir **Apilado 2 cajas**.
5. Dejar libre el recorrido, pulsar **EJECUTAR** y colocar la primera caja en A
   cuando el estado lo solicite.
6. La rutina confirma la caja con dos lecturas del HC-SR04, espera 2 segundos,
   la traslada a B y vuelve a A.
7. Colocar la segunda caja cuando el estado lo solicite. El brazo la apila y
   aplica la corrección final de 5 mm antes de abrir la pinza.

El interruptor **STOP / Reanudar** puede detener el proceso. STOP cancela la
secuencia actual; al volver el interruptor a cero se desbloquea el brazo y se
puede comenzar de nuevo con EJECUTAR. No se debe iniciar mientras alguien tenga
las manos en el rango de movimiento.

La rutina 4 fuerza un mínimo de velocidad 30 aunque V12 conserve un valor
antiguo menor. Un valor superior a 30 sí se respeta.

## Mapa del panel

| Pin | Control web | Función |
|---|---|---|
| V0 | Slider Base | Servo de la base |
| V1 | Slider Hombro | Servo del hombro |
| V2 | Slider Codo | Servo del codo |
| V3 | Slider Muñeca 1 | Giro de muñeca |
| V4 | Slider Muñeca 2 | Inclinación de muñeca |
| V5 | Botón Abrir pinza | Lleva la pinza a la apertura guardada |
| V6 | Botón Cerrar pinza | Lleva la pinza al cierre guardado |
| V7 | Gauge Distancia HC-SR04 | Distancia en centímetros |
| V8 | Selector de rutina | 0 Apilar; 3 Apilado 2 cajas; 1 Ordenar; 2 Escanear |
| V9 | Botón EJECUTAR | Inicia la rutina; la opción de dos cajas corre dentro del ESP32 |
| V10 | Switch STOP / Reanudar | 1 detiene; 0 reanuda |
| V11 | Etiqueta Estado de la rutina | Estado enviado por Python/ESP32 |
| V12 | Slider Velocidad | 1–100; valor predeterminado 30 |
| V13 | Etiqueta TCP X | Coordenada X calculada por la DH |
| V14 | Etiqueta TCP Y | Coordenada Y calculada por la DH |
| V15 | Etiqueta TCP Z | Coordenada Z calculada por la DH |
| V16 | Slider Calibrar pinza | Movimiento manual de la pinza |
| V17 | Botón Guardar como abierta | Guarda V16 como apertura |
| V18 | Botón Guardar como cerrada | Guarda V16 como cierre |

## Respaldo y reconstrucción

`docs/blynk_template.json` conserva tipos, límites, valores predeterminados,
nombres y widgets. Es una especificación reproducible; Blynk no ofrece desde el
panel web una exportación/importación directa de esta plantilla.

El panel de la aplicación móvil se organiza desde Blynk.App en **Developer
Mode**. Debe enlazarse a los mismos V0–V18 indicados arriba; los flujos y el
firmware ya son compartidos con el panel web.

Después de modificar flujos o widgets en la consola web, usar **Guardar y
aplicar** y comprobar que el cuadro de confirmación muestre **Brazo ESP32 —
Online**.
