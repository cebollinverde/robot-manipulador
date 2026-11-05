"""
Configuracion de la zona de trabajo y de la rutina de traslado.

TODO lo que esta aqui se mide en el brazo real (cm, en el sistema {0} del DH:
x hacia adelante, y hacia la izquierda, z hacia arriba, origen en el eje de la base).
Los valores por defecto se eligieron dentro del espacio alcanzable del modelo,
pero hay que ajustarlos con el brazo armado.
"""

# ---------------------------------------------------------------- mesa y cubos
Z_MESA = 0.0          # altura de la mesa en {0}. Si {0} esta sobre la mesa, es negativa.
# Objeto a trasladar: caja rectangular ACOSTADA, con el lado largo apuntando
# hacia la base (radial). La pinza la agarra por el ANCHO.
CUBO = 3.04           # ALTO del objeto acostado [cm] (se usa para apilar)
LARGO_OBJ = 8.45      # largo, en direccion radial [cm]
ANCHO_OBJ = 3.44      # ancho, el lado que aprietan los dedos [cm]

# Puntos conocidos donde PUEDE haber un cubo (centro de la cara apoyada en la mesa)
PUNTOS_RECOGIDA = {
    "A": (14.1, -16.9),   # r = 22 cm, base a -50 grados
    "B": (19.9, -9.3),    # r = 22 cm, base a -25 grados
    "C": (22.0, 0.0),     # r = 22 cm, base a   0 grados
}

# Posturas ENSENADAS con el brazo real (pestana Calibrar). Para cada punto se guarda
# {"q": angulos q1..q5, "xyz": centro del objeto, "phi": inclinacion de la pinza}.
# La rutina vuelve EXACTAMENTE a esa postura para agarrar/dejar, asi las
# diferencias entre el modelo DH y el brazo real no afectan el agarre.
POSES = {}

# Rutina 1: todos los cubos se apilan aqui
PUNTO_APILADO = (17.8, 12.9)   # r = 22 cm, base a +36 grados

# Rutina 2: los cubos se dejan uno al lado del otro en estas posiciones
FILA_ORDEN = [
    (20.9, 6.8),     # +18 grados
    (19.1, 11.0),    # +30 grados
    (16.3, 14.7),    # +42 grados
]

# ---------------------------------------------------------------- movimiento
PHI_AGARRE = -55.0     # inclinacion preferida de la pinza al tomar/dejar [grados]
ALTURA_APROX = 5.0     # cm sobre el cubo antes de bajar en linea recta
ALTURA_SEGURA = 11.0   # z de traslado (sobre la mesa) para no chocar la pila
RADIO_TRASLADO = (19.0, 28.0)  # radio permitido al trasladarse en altura [cm]
HOLGURA_DEJAR = 0.3   # cm extra al soltar un cubo sobre otro
PASO_CM = 1.5          # largo de cada tramo en trayectorias cartesianas
TOL_INTERMEDIA = 3.0   # % de servo: tolerancia para encadenar tramos sin detenerse
VELOCIDAD = 45         # 1..100 (comando SPEED del ESP32)
ESPERA_PINZA = 0.5     # s extra para que la pinza termine de cerrar/abrir

# ---------------------------------------------------------------- sensor
# Posicion del HC-SR04 en el sistema {5} de la pinza (el ultimo del DH):
#   x5 = hacia la punta de la pinza (el sensor "mira" en esta direccion)
#   y5 = perpendicular, "encima" de la pinza cuando esta horizontal
#   z5 = hacia el costado (eje de la muneca). En las fotos el sensor esta corrido
#        hacia un lado de la garra: medir cuanto (+ o -, se prueba con la rutina 3)
# Valor estimado de las fotos: ~2.5 cm detras del centro de agarre, ~3 cm sobre la garra
SENSOR_OFFSET = (-2.5, 3.0, 0.0)

PHI_ESCANEO = -60.0      # inclinacion preferida al mirar un punto
DIST_ESCANEO = 12.0      # distancia sensor -> mesa al escanear [cm]
SENSOR_MIN = 2.0         # el HC-SR04 no mide bien por debajo de 2 cm
SENSOR_MAX = 60.0        # mas lejos que esto se considera "sin objeto"
MARGEN_DETECCION = 1.0   # cm minimos de diferencia con la mesa para decir "hay objeto"
LECTURAS_SENSOR = 3      # cuantas veces se pide DIST (el ESP ya filtra cada una)


# ---------------------------------------------------------------- calibracion guardada
# La interfaz (pestana "Calibrar") guarda aqui lo que se le ensena con el brazo real.
# Si el archivo existe, sus valores reemplazan a los de arriba.
import json as _json
import os as _os

ARCHIVO_ZONA = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "zona_trabajo.json")
_CLAVES = ("Z_MESA", "CUBO", "POSES", "LARGO_OBJ", "ANCHO_OBJ", "PUNTOS_RECOGIDA", "PUNTO_APILADO", "FILA_ORDEN", "SENSOR_OFFSET")


def cargar_zona():
    if not _os.path.exists(ARCHIVO_ZONA):
        return False
    with open(ARCHIVO_ZONA, encoding="utf-8") as f:
        datos = _json.load(f)
    g = globals()
    for k in _CLAVES:
        if k in datos:
            v = datos[k]
            if k == "PUNTOS_RECOGIDA":
                v = {n: tuple(p) for n, p in v.items()}
            elif k in ("PUNTO_APILADO", "SENSOR_OFFSET"):
                v = tuple(v)
            elif k == "FILA_ORDEN":
                v = [tuple(p) for p in v]
            g[k] = v
    return True


def guardar_zona():
    g = globals()
    with open(ARCHIVO_ZONA, "w", encoding="utf-8") as f:
        _json.dump({k: g[k] for k in _CLAVES}, f, indent=2)


cargar_zona()
