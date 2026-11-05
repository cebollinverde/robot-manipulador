"""
Modelo Denavit-Hartenberg del manipulador (5 articulaciones + pinza).

Basado en dh-2.py del grupo. Se mantienen las medidas y la calibracion;
solo se agregan funciones para obtener TODAS las matrices A_i y T0_i
(las usa la interfaz para mostrar que pasa en cada eslabon).

Tabla DH (convencion estandar, angulos en grados, largos en cm):

    i | theta     | d   | a   | alpha
    1 | q1        | H   | R0  |  90
    2 | q2        | 0   | L1  |   0
    3 | q3 + 90   | 0   | 0   |  90
    4 | q4        | L2  | 0   | -90
    5 | q5 - 90   | 0   | LG  |   0

Inclinacion de la pinza en el plano del brazo: phi = q2 + q3 + q5
"""
from math import cos, sin, radians, sqrt

# Medidas del brazo en centimetros
H = sqrt(6.3**2 - 1.3**2)  # Altura del hombro respecto a la base
R0 = 1.3                   # Separacion horizontal de la base
L1 = 12.3                  # Hombro a codo
L2 = 8.7 + 3.2             # Codo a muneca
LG = 11.1                  # Muneca al centro de la pinza

# Limites seguros medidos fisicamente (porcentaje ORIGINAL, servos 1..6)
MINIMO = [5, 15, 0, 10, 25, 20]
MAXIMO = [80, 93, 80, 78, 70, 85]

# Referencias angulares medidas al 50%
# IMPORTANTE: Estos datos son parte de la calibracion
Q50 = [0, 62, -89, 0, 2]
GRADOS_POR_PORCENTAJE = [1.6, 1.4, -1.4, 1.8, 1.4]

# Pinza (igual que en el ESP32): angulo de servo -> porcentaje original
PINZA_ABIERTA_ORIG = 110.0 * 100.0 / 180.0
PINZA_CERRADA_ORIG = 65.0 * 100.0 / 180.0

NOMBRES = ["q1 Base", "q2 Hombro", "q3 Codo", "q4 Giro muneca", "q5 Muneca"]

# ---------------------------------------------------------------- medidas editables
# La interfaz (pestana "Modelo") permite cambiar los largos y los guarda en
# brazo/medidas.json. Si ese archivo existe, reemplaza los valores de arriba.
import json as _json
import os as _os

ARCHIVO_MEDIDAS = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "medidas.json")
MEDIDAS_ORIGINALES = {"H": H, "R0": R0, "L1": L1, "L2": L2, "LG": LG}


def medidas():
    return {"H": H, "R0": R0, "L1": L1, "L2": L2, "LG": LG}


def fijar_medidas(**nuevas):
    """Cambia los largos del DH (cm). Ej: fijar_medidas(L1=13.0)."""
    g = globals()
    for k, v in nuevas.items():
        if k not in MEDIDAS_ORIGINALES:
            raise KeyError(k)
        v = float(v)
        if v < 0 or (k in ("L1", "L2", "LG") and v <= 0):
            raise ValueError(f"{k} debe ser positivo")
        g[k] = v


def guardar_medidas():
    with open(ARCHIVO_MEDIDAS, "w", encoding="utf-8") as f:
        _json.dump(medidas(), f, indent=2)


def cargar_medidas():
    if _os.path.exists(ARCHIVO_MEDIDAS):
        with open(ARCHIVO_MEDIDAS, encoding="utf-8") as f:
            fijar_medidas(**_json.load(f))
        return True
    return False


cargar_medidas()


def orig_a_normal(i, antiguo):
    """Porcentaje original (el que se midio) -> nuevo 0-100% normalizado."""
    return 100.0 * (antiguo - MINIMO[i]) / (MAXIMO[i] - MINIMO[i])


def normal_a_orig(i, nuevo):
    return MINIMO[i] + nuevo * (MAXIMO[i] - MINIMO[i]) / 100.0


PINZA_ABIERTA = orig_a_normal(5, PINZA_ABIERTA_ORIG)   # ~63.2 %
PINZA_CERRADA = orig_a_normal(5, PINZA_CERRADA_ORIG)   # ~24.8 %

# Postura de calibracion (todos los servos al 50% ORIGINAL) en la escala nueva
HOME = [orig_a_normal(i, 50.0) for i in range(6)]


def porcentajes_a_angulos(porcentajes):
    angulos = []
    for i in range(5):
        nuevo = float(porcentajes[i])
        # Pasamos del nuevo 0-100% a la escala que usamos al medir
        antiguo = MINIMO[i] + nuevo * (MAXIMO[i] - MINIMO[i]) / 100
        angulo = Q50[i] + (antiguo - 50) * GRADOS_POR_PORCENTAJE[i]
        angulos.append(angulo)
    return angulos


def angulos_a_porcentajes(angulos):
    porcentajes = []
    for i in range(5):
        angulo = float(angulos[i])
        # Recuperamos el porcentaje obtenido segun el y lo pasamos a la escala nueva
        antiguo = 50 + (angulo - Q50[i]) / GRADOS_POR_PORCENTAJE[i]
        nuevo = 100 * (antiguo - MINIMO[i]) / (MAXIMO[i] - MINIMO[i])

        # Tolerancia numerica: 100.0000001 sigue siendo 100
        if -1e-6 < nuevo < 0:
            nuevo = 0.0
        if 100 < nuevo < 100 + 1e-6:
            nuevo = 100.0

        # Si supera los limites, no devolvemos una posicion falsa
        if nuevo < 0 or nuevo > 100:
            return None

        porcentajes.append(nuevo)
    return porcentajes


def limites_angulos():
    """[(min, max)] en grados para q1..q5 (ordenados aunque la pendiente sea negativa)."""
    a = porcentajes_a_angulos([0] * 5)
    b = porcentajes_a_angulos([100] * 5)
    return [(min(a[i], b[i]), max(a[i], b[i])) for i in range(5)]


# Matriz D-H
def matriz_dh(theta, d, a, alpha):
    t = radians(theta)
    al = radians(alpha)

    ct, st = cos(t), sin(t)
    ca, sa = cos(al), sin(al)

    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ]


# Multiplicacion de matrices de 4x4
def multiplicar(A, B):
    return [
        [sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


IDENTIDAD = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]


def parametros_dh(angulos):
    """Filas (theta, d, a, alpha) en grados / cm para los angulos dados."""
    q1, q2, q3, q4, q5 = [float(q) for q in angulos]
    return [
        (q1,      H,  R0,  90),
        (q2,      0,  L1,   0),
        (q3 + 90, 0,   0,  90),
        (q4,     L2,   0, -90),
        (q5 - 90, 0,  LG,   0)
    ]


def matrices(angulos):
    """Devuelve (A, T): A = [A1..A5] y T = [T0_0 (identidad), T0_1, ..., T0_5]."""
    A = [matriz_dh(*fila) for fila in parametros_dh(angulos)]
    T = [IDENTIDAD]
    for Ai in A:
        T.append(multiplicar(T[-1], Ai))
    return A, T


# Unimos las transformaciones de las cinco articulaciones
def matriz_total(angulos):
    return matrices(angulos)[1][-1]


# La ultima columna de la matriz es X, Y, Z en centimetros
def cinematica_directa(angulos):
    T = matriz_total(angulos)
    return T[0][3], T[1][3], T[2][3]


def inclinacion_pinza(angulos):
    """phi = q2 + q3 + q5 (grados, respecto a la horizontal)."""
    return float(angulos[1]) + float(angulos[2]) + float(angulos[4])


# ---------------------------------------------------------------- prueba directa
# python -m brazo.dh                -> tabla DH y matrices en la postura de calibracion
# python -m brazo.dh 0 62 -89 0 2   -> lo mismo para esos angulos (grados)
if __name__ == "__main__":
    import sys

    if len(sys.argv) == 6:
        q = [float(v) for v in sys.argv[1:]]
    else:
        q = porcentajes_a_angulos(HOME)
    expr = ["q1", "q2", "q3 + 90", "q4", "q5 - 90"]

    print("Angulos q [grados]:", ", ".join(f"{v:.2f}" for v in q))
    p = angulos_a_porcentajes(q)
    print("Servos 1-5 [%]   :", "fuera de limites" if p is None
          else ", ".join(f"{v:.2f}" for v in p))
    print("\nTabla DH (cm, grados)")
    print(f"{'i':>2} | {'theta':>18} | {'d':>7} | {'a':>7} | {'alpha':>6}")
    for i, (th, d, a, al) in enumerate(parametros_dh(q), start=1):
        print(f"{i:>2} | {expr[i - 1] + ' = ' + format(th, '.2f'):>18} | {d:7.3f} | "
              f"{a:7.3f} | {al:6.0f}")

    A, T = matrices(q)

    def mostrar(nombre, M):
        print(f"\n{nombre} =")
        for fila in M:
            print("  [" + "  ".join(f"{(0.0 if abs(v) < 5e-5 else v):9.4f}" for v in fila) + " ]")

    for i in range(5):
        mostrar(f"A{i + 1}", A[i])
    for i in range(1, 6):
        mostrar(f"T0_{i}", T[i])
    x, y, z = cinematica_directa(q)
    print(f"\nTCP (centro de la pinza): x = {x:.2f} cm, y = {y:.2f} cm, z = {z:.2f} cm, "
          f"phi = {inclinacion_pinza(q):.1f} grados")
