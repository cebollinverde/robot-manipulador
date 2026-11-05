"""Genera docs/diagrama_conexiones.svg (y .png con rsvg-convert)."""
import os
import subprocess

W, H = 1540, 960
ROJO, NEGRO, AZUL, VERDE, NARANJO, MORADO, GRIS = (
    "#d62828", "#222222", "#1d6fd8", "#1a9e55", "#e07a00", "#7b3fbf", "#6b7280")
s = []


def caja(x, y, w, h, titulo, sub="", fondo="#f7f7f9", borde="#333"):
    s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fondo}" '
             f'stroke="{borde}" stroke-width="2"/>')
    s.append(f'<text x="{x + w / 2}" y="{y + 24}" class="t">{titulo}</text>')
    if sub:
        for k, linea in enumerate(sub.split("\n")):
            s.append(f'<text x="{x + w / 2}" y="{y + 44 + 16 * k}" class="s">{linea}</text>')


def pin(x, y, txt, lado="izq"):
    s.append(f'<circle cx="{x}" cy="{y}" r="4" fill="#fff" stroke="#333" stroke-width="1.5"/>')
    if lado == "izq":
        s.append(f'<text x="{x + 9}" y="{y + 4}" class="p" text-anchor="start">{txt}</text>')
    else:
        s.append(f'<text x="{x - 9}" y="{y + 4}" class="p" text-anchor="end">{txt}</text>')


def linea(puntos, color, ancho=2.4, guion=""):
    d = " ".join(f"{x},{y}" for x, y in puntos)
    extra = f' stroke-dasharray="{guion}"' if guion else ""
    s.append(f'<polyline points="{d}" fill="none" stroke="{color}" stroke-width="{ancho}"'
             f' stroke-linejoin="round"{extra}/>')


def punto(x, y, color):
    s.append(f'<circle cx="{x}" cy="{y}" r="4.5" fill="{color}"/>')


def texto(x, y, txt, color="#333", clase="s", anchor="middle"):
    s.append(f'<text x="{x}" y="{y}" class="{clase}" fill="{color}" '
             f'style="text-anchor:{anchor}">{txt}</text>')


def resistencia(x1, y1, x2, y2, etiqueta):
    """Resistencia en zigzag sobre un tramo horizontal o vertical."""
    if y1 == y2:
        n, paso = 6, (x2 - x1) / 8
        pts = [(x1, y1), (x1 + paso, y1)]
        for k in range(n):
            pts.append((x1 + paso * (1.5 + k), y1 + (7 if k % 2 == 0 else -7)))
        pts += [(x2 - paso, y1), (x2, y1)]
        linea(pts, NARANJO, 2)
        texto((x1 + x2) / 2, y1 - 12, etiqueta, NARANJO, "p")
    else:
        n, paso = 6, (y2 - y1) / 8
        pts = [(x1, y1), (x1, y1 + paso)]
        for k in range(n):
            pts.append((x1 + (7 if k % 2 == 0 else -7), y1 + paso * (1.5 + k)))
        pts += [(x1, y2 - paso), (x1, y2)]
        linea(pts, NARANJO, 2)
        texto(x1 + 14, (y1 + y2) / 2 + 4, etiqueta, NARANJO, "p", "start")


# ---------------------------------------------------------------- bloques
GND_Y = 905
caja(40, 70, 230, 110, "Fuente de poder", "5 V · 10 A\n(desde 220 V AC)", "#fff4f4", ROJO)
pin(270, 105, "+5V", "der")
pin(270, 150, "GND", "der")

caja(40, 330, 230, 120, "App Blynk", "Android\nsliders V0–V4, pinza V5/V6\nrutina V8/V9, STOP V10",
     "#f4f0ff", MORADO)
caja(40, 640, 230, 140, "PC · Python", "brazo_gui.py\nDH + cinemática inversa\nrutina detectar/apilar",
     "#f4f0ff", MORADO)

# ESP32
EX, EY, EW, EH = 360, 300, 210, 430
caja(EX, EY, EW, EH, "ESP32", "DevKit", "#eef6ff", AZUL)
esp = {"3V3": 380, "GND": 420, "GPIO26 (SDA)": 480, "GPIO27 (SCL)": 520,
       "GPIO25 (TRIG)": 600, "GPIO33 (ECHO)": 640}
for nombre, y in esp.items():
    pin(EX + EW, y, nombre, "der")
pin(EX, 700, "USB", "izq")

# PCA9685
PX, PY, PW, PH = 680, 230, 230, 470
caja(PX, PY, PW, PH, "PCA9685", "driver PWM 16 canales\ndirección I²C 0x40", "#eefaf2", VERDE)
pca_izq = {"VCC (lógica)": 380, "GND": 420, "SDA": 480, "SCL": 520}
for nombre, y in pca_izq.items():
    pin(PX, y, nombre, "izq")
texto(PX + 20, 650, "V+ (borne de servos): sin uso", GRIS, "p", "start")
texto(PX + 20, 668, "los servos se alimentan directo", GRIS, "p", "start")

# Servos
servos = [("MG995", "Servo 1 · Base (q1)"), ("MG995", "Servo 2 · Hombro (q2)"),
          ("MG995", "Servo 3 · Codo (q3)"), ("MG90S", "Servo 4 · Giro muñeca (q4)"),
          ("MG90S", "Servo 5 · Muñeca (q5)"), ("SG90", "Servo 6 · Pinza")]
SX, SW, SH = 1100, 320, 66
sy = [70, 160, 250, 400, 490, 580]
for k, ((modelo, nombre), y) in enumerate(zip(servos, sy)):
    grande = k < 3
    s.append(f'<rect x="{SX}" y="{y}" width="{SW}" height="{SH}" rx="8" '
             f'fill="{"#fff8ec" if grande else "#fffdf0"}" stroke="#555" stroke-width="2"/>')
    texto(SX + 62, y + 26, nombre, "#222", "t", "start")
    texto(SX + 62, y + 46, modelo + " · 3 cables", "#444", "s", "start")
    canal = 10 + k
    ys = y + 18
    # señal PWM desde el PCA
    pin(PX + PW, 270 + 42 * k, f"CH{canal}", "der")
    xv = 945 + 16 * k
    linea([(PX + PW, 270 + 42 * k), (xv, 270 + 42 * k), (xv, ys), (SX, ys)], AZUL, 2)
    texto(SX + 8, ys - 4, "señal", AZUL, "p", "start")

# ---------------------------------------------------------------- 5 V
RAIL_G, RAIL_C = 1040, 1065
linea([(270, 105), (320, 105), (320, 30), (RAIL_G, 30), (RAIL_G, sy[2] + 36)], ROJO, 3)
linea([(320, 30), (320, 45), (RAIL_C, 45), (RAIL_C, 785)], ROJO, 3)
punto(320, 30, ROJO)
texto(680, 22, "+5 V  línea servos grandes (3 × MG995)", ROJO, "p")
texto(680, 62, "+5 V  línea servos chicos (2 × MG90S + SG90)", ROJO, "p")
for k, y in enumerate(sy):
    xr = RAIL_G if k < 3 else RAIL_C
    linea([(xr, y + 36), (SX, y + 36)], ROJO, 2)
    punto(xr, y + 36, ROJO)
    texto(SX + 8, y + 32, "+5V", ROJO, "p", "start")

# ---------------------------------------------------------------- GND común
GX = 1455
for y in sy:
    linea([(SX + SW, y + 50), (GX, y + 50)], NEGRO, 2)
    punto(GX, y + 50, NEGRO)
    texto(SX + SW - 8, y + 54, "GND", NEGRO, "p", "end")
linea([(GX, sy[0] + 50), (GX, GND_Y)], NEGRO, 3)
texto(GX + 8, sy[3] - 30, "negativos", NEGRO, "p", "start")
texto(GX + 8, sy[3] - 16, "soldados", NEGRO, "p", "start")
texto(GX + 8, sy[3] - 2, "en 1 línea", NEGRO, "p", "start")
linea([(300, GND_Y), (GX, GND_Y)], NEGRO, 4)
texto(860, GND_Y + 26, "GND COMÚN (riel negativo de la protoboard) — fuente, ESP32, PCA9685, "
      "sensor y servos", NEGRO, "s")
linea([(270, 150), (300, 150), (300, GND_Y)], NEGRO, 2.4)
punto(300, GND_Y, NEGRO)
linea([(EX + EW, 420), (620, 420), (620, GND_Y)], NEGRO, 2.4)
punto(620, GND_Y, NEGRO)
linea([(PX, 420), (650, 420), (650, GND_Y)], NEGRO, 2.4)
punto(650, GND_Y, NEGRO)

# ---------------------------------------------------------------- I2C y lógica
linea([(EX + EW, 380), (PX, 380)], "#b45309", 2.4)
texto(625, 372, "3.3 V", "#b45309", "p")
linea([(EX + EW, 480), (PX, 480)], VERDE, 2.4)
linea([(EX + EW, 520), (PX, 520)], VERDE, 2.4)
texto(625, 472, "SDA", VERDE, "p")
texto(625, 512, "SCL", VERDE, "p")

# ---------------------------------------------------------------- HC-SR04
HX, HY, HW, HH = 700, 755, 300, 110
caja(HX, HY, HW, HH, "HC-SR04 (en la pinza)", "sensor ultrasónico", "#fff6ea", NARANJO)
pin(HX + HW, 785, "VCC", "der")
pin(HX + HW, 835, "GND", "der")
pin(HX, 790, "TRIG", "izq")
pin(HX, 835, "ECHO", "izq")
linea([(RAIL_C, 785), (HX + HW, 785)], ROJO, 2)
punto(RAIL_C, 785, ROJO)
linea([(HX + HW, 835), (1090, 835), (1090, GND_Y)], NEGRO, 2)
punto(1090, GND_Y, NEGRO)
linea([(EX + EW, 600), (600, 600), (600, 790), (HX, 790)], NARANJO, 2.4)
# divisor de voltaje en ECHO
resistencia(HX - 10, 835, 610, 835, "1 kΩ")
linea([(610, 835), (590, 835), (590, 640), (EX + EW, 640)], NARANJO, 2.4)
punto(590, 835, NARANJO)
resistencia(590, 835, 590, 895, "2 kΩ")
linea([(590, 895), (590, GND_Y)], NARANJO, 2)
punto(590, GND_Y, NEGRO)
texto(470, 810, "divisor 5 V → 3.3 V", NARANJO, "p")
texto(470, 826, "(el ECHO sale a 5 V)", NARANJO, "p")

# ---------------------------------------------------------------- comunicaciones
linea([(270, 710), (EX, 710)], MORADO, 3)
texto(315, 700, "USB serial", MORADO, "p")
texto(315, 730, "115200 bd", MORADO, "p")
linea([(270, 390), (EX, 390)], MORADO, 2.4, "7,5")
texto(315, 380, "WiFi", MORADO, "p")
texto(315, 410, "(nube Blynk)", MORADO, "p")

# leyenda
ly = 205
for k, (c, t) in enumerate(((ROJO, "+5 V potencia"), (NEGRO, "GND común"), (AZUL, "señal PWM"),
                            (VERDE, "I²C"), (NARANJO, "sensor"), (MORADO, "datos"))):
    linea([(48, ly + 22 * k), (78, ly + 22 * k)], c, 3)
    texto(86, ly + 22 * k + 4, t, "#333", "p", "start")

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<style>
 text {{ font-family: "DejaVu Sans", Arial, sans-serif; }}
 .t {{ font-size: 16px; font-weight: bold; text-anchor: middle; fill: #222; }}
 .s {{ font-size: 13px; text-anchor: middle; fill: #444; }}
 .p {{ font-size: 12px; fill: #333; }}
</style>
<rect width="100%" height="100%" fill="#ffffff"/>
{chr(10).join(s)}
</svg>'''

aqui = os.path.dirname(os.path.abspath(__file__))
ruta = os.path.join(aqui, "diagrama_conexiones.svg")
with open(ruta, "w", encoding="utf-8") as f:
    f.write(svg)
try:
    subprocess.run(["rsvg-convert", "-z", "1.5", "-o", ruta.replace(".svg", ".png"), ruta],
                   check=True)
except (FileNotFoundError, subprocess.CalledProcessError):
    pass
print("listo:", ruta)
