"""
Geometria auxiliar: sensor montado en la pinza, poses de escaneo y una escena
simple (mesa + cubos) para el simulador y la vista 3D.
"""
from math import asin, atan2, cos, sin, radians, hypot, ceil

import numpy as np

from . import config as C
from .dh import matrices
from .inversa import cinematica_inversa, candidatos_phi


def pose_sensor(angulos):
    """(origen, direccion) del haz del sensor en {0}, usando T0_5 del DH."""
    T5 = np.array(matrices(angulos)[1][5], dtype=float)
    origen = T5 @ np.array([*C.SENSOR_OFFSET, 1.0])
    return origen[:3], T5[:3, 0]          # el sensor mira segun x5


def tcp_para_mirar(px, py, pz, phi, distancia):
    """TCP necesario para que el haz (con inclinacion phi) toque (px,py,pz) a
    'distancia' cm del sensor. Usa la geometria plana del brazo (q4 = 0)."""
    ox, oy, oz = C.SENSOR_OFFSET
    # Si el sensor esta corrido hacia el costado (oz), el haz va paralelo al plano
    # del brazo: hay que girar un poco la base para que pase por el punto.
    r = hypot(px, py)
    psi = atan2(py, px) + (asin(max(-1.0, min(1.0, oz / r))) if r > 1e-6 else 0.0)
    u = np.array([cos(psi), sin(psi), 0.0])           # direccion radial
    zv = np.array([0.0, 0.0, 1.0])
    z5 = np.array([sin(psi), -cos(psi), 0.0])         # eje de la muneca
    f = radians(phi)
    x5 = cos(f) * u + sin(f) * zv                     # hacia la punta
    y5 = -sin(f) * u + cos(f) * zv                    # "encima" de la pinza
    sensor = np.array([px, py, pz]) - distancia * x5
    return sensor - ox * x5 - oy * y5 - oz * z5


def pose_escaneo(px, py, actuales=None, z_mesa=None, phi_pref=None):
    """Busca (angulos, phi, distancia_esperada) para mirar el punto de mesa (px, py).
    z_mesa: altura de la mesa en ese punto (por defecto C.Z_MESA)."""
    z_mesa = C.Z_MESA if z_mesa is None else z_mesa
    phi_pref = C.PHI_ESCANEO if phi_pref is None else min(phi_pref, -30.0)
    distancias = sorted(range(7, 23), key=lambda d: abs(d - C.DIST_ESCANEO))
    for phi in candidatos_phi(phi_pref, -90, -20):
        for d in distancias:
            tcp = tcp_para_mirar(px, py, z_mesa, phi, d)
            q = cinematica_inversa(*tcp, phi, actuales)
            if q is not None:
                return q, phi, d
    return None, None, None


def punto_impacto(angulos, distancia):
    o, d = pose_sensor(angulos)
    return o + distancia * d


# ---------------------------------------------------------------- trayectorias
def a_cilindricas(p):
    return hypot(p[0], p[1]), atan2(p[1], p[0]), p[2]


def interpolar(p0, p1, paso=None):
    """Puntos intermedios en coordenadas cilindricas (r, angulo base, z).
    Mantiene el radio en los giros de base (un arco) y es una recta en
    las subidas/bajadas verticales."""
    paso = paso or C.PASO_CM
    r0, a0, z0 = a_cilindricas(p0)
    r1, a1, z1 = a_cilindricas(p1)
    largo = hypot(hypot(r1 - r0, ((r0 + r1) / 2) * (a1 - a0)), z1 - z0)
    n = max(1, ceil(largo / paso))
    puntos = []
    for k in range(1, n + 1):
        s = k / n
        r = r0 + (r1 - r0) * s
        a = a0 + (a1 - a0) * s
        puntos.append((r * cos(a), r * sin(a), z0 + (z1 - z0) * s))
    return puntos


# ---------------------------------------------------------------- escena
class Escena:
    """Mesa + cubos alineados con los ejes. Se usa para simular el sensor,
    simular el agarre y dibujar en 3D."""

    def __init__(self):
        self.cubos = []          # dicts: {"nombre", "p": np.array centro, "tomado": bool}

    def poner_cubos(self, nombres):
        self.cubos = []
        for n in nombres:
            if n in C.POSES:
                x, y, z = C.POSES[n]["xyz"]
            else:
                (x, y), z = C.PUNTOS_RECOGIDA[n], C.Z_MESA + C.CUBO / 2
            self.cubos.append({"nombre": n, "tomado": False, "p": np.array([x, y, z])})

    def centros(self, tcp=None):
        out = []
        for c in self.cubos:
            out.append(np.array(tcp, float) if (c["tomado"] and tcp is not None) else c["p"])
        return out

    def rayo(self, origen, direccion, tcp=None):
        """Distancia al primer obstaculo (mesa o cubo no tomado), o None."""
        o, d = np.asarray(origen, float), np.asarray(direccion, float)
        mejor = None
        if d[2] < -1e-6:
            t = (C.Z_MESA - o[2]) / d[2]
            if t > 0:
                mejor = t
        h = np.array([C.LARGO_OBJ / 2, C.ANCHO_OBJ / 2, C.CUBO / 2])
        for c in self.cubos:
            if c["tomado"]:
                continue
            # rayo en el sistema de la caja (eje x = direccion radial)
            a = atan2(c["p"][1], c["p"][0])
            R = np.array([[cos(a), sin(a), 0], [-sin(a), cos(a), 0], [0, 0, 1]])
            ol, dl = R @ (o - c["p"]), R @ d
            lo, hi = -h, h
            tmin, tmax = -np.inf, np.inf
            for k in range(3):
                if abs(dl[k]) < 1e-9:
                    if not lo[k] <= ol[k] <= hi[k]:
                        break
                    continue
                t1, t2 = (lo[k] - ol[k]) / dl[k], (hi[k] - ol[k]) / dl[k]
                tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            else:
                if tmax >= tmin and tmin > 0 and (mejor is None or tmin < mejor):
                    mejor = tmin
        return mejor

    def tomar(self, tcp, radio=2.0):
        """Cierra la pinza: engancha el cubo cuyo centro este cerca del TCP."""
        libres = [c for c in self.cubos if not c["tomado"]]
        if not libres:
            return None
        c = min(libres, key=lambda c: np.linalg.norm(c["p"] - tcp))
        if np.linalg.norm(c["p"] - tcp) <= radio:
            c["tomado"] = True
            return c["nombre"]
        return None

    def soltar(self, tcp):
        """Abre la pinza: el cubo cae sobre la mesa o sobre otro cubo."""
        for c in self.cubos:
            if c["tomado"]:
                c["tomado"] = False
                x, y = float(tcp[0]), float(tcp[1])
                apoyo = C.Z_MESA
                for o in self.cubos:
                    if o is c or o["tomado"]:
                        continue
                    if hypot(o["p"][0] - x, o["p"][1] - y) < 0.8 * C.ANCHO_OBJ:
                        apoyo = max(apoyo, o["p"][2] + C.CUBO / 2)
                c["p"] = np.array([x, y, apoyo + C.CUBO / 2])
                return c["nombre"]
        return None
