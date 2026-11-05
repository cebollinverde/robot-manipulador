"""
Escena 3D (matplotlib) del brazo: eslabones, sistemas {0}..{5}, pinza, haz del
sensor, cubos y puntos de la zona de trabajo. La usa la interfaz y tambien
sirve para generar figuras para el informe sin abrir ventanas.
"""

import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

from . import config as C
from . import dh

COLORES_EJES = ("#d62728", "#2ca02c", "#1f77b4")      # x, y, z
COLOR_BRAZO = "#4a5568"


def _set3d(linea, P):
    P = np.asarray(P, float).reshape(-1, 3)
    linea.set_data(P[:, 0], P[:, 1])
    linea.set_3d_properties(P[:, 2])


def caras_cubo(c, lado, largo=None, ancho=None):
    """Caja centrada en c, con el largo en direccion radial (como la deja la pinza)."""
    largo = lado if largo is None else largo
    ancho = lado if ancho is None else ancho
    x, y, z = c
    a = np.arctan2(y, x)
    u = np.array([np.cos(a), np.sin(a), 0.0])
    w = np.array([-np.sin(a), np.cos(a), 0.0])
    v = np.array([np.array([x, y, z]) + sx * largo / 2 * u + sy * ancho / 2 * w
                  + np.array([0, 0, sz * lado / 2])
                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    idx = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [[v[i] for i in cara] for cara in idx]


class Escena3D:
    def __init__(self, ax):
        self.ax = ax
        self._cubos = []
        self.construir()

    def construir(self, elev=24, azim=-60):
        ax = self.ax
        ax.cla()
        alcance = dh.R0 + dh.L1 + dh.L2 + dh.LG
        lim = max(32, alcance)
        z0 = C.Z_MESA - 2
        z1 = max(C.Z_MESA + 34, dh.H + dh.L1 + dh.L2 * 0.5 + 4)
        ax.set_xlim(-8, lim)
        ax.set_ylim(-lim * 0.8, lim * 0.8)
        ax.set_zlim(z0, z1)
        try:
            ax.set_box_aspect((lim + 8, lim * 1.6, z1 - z0))
        except AttributeError:
            pass
        ax.set_xlabel("x [cm]")
        ax.set_ylabel("y [cm]")
        ax.set_zlabel("z [cm]")
        for eje in (ax.xaxis, ax.yaxis, ax.zaxis):
            try:
                eje.set_pane_color((1, 1, 1, 0))
            except AttributeError:
                pass
        ax.view_init(elev, azim)

        # mesa: grilla cada 5 cm, zona alcanzable y base
        zm = C.Z_MESA
        segs = []
        for v in range(-25, 35, 5):
            segs.append([(v, -24, zm), (v, 24, zm)])
        for v in range(-20, 25, 5):
            segs.append([(-8, v, zm), (32, v, zm)])
        ax.add_collection3d(Line3DCollection(segs, colors="#e2e8f0", linewidths=0.7))
        lo, hi = dh.limites_angulos()[0]
        ang = np.radians(np.linspace(lo, hi, 80))
        for r in C.RADIO_TRASLADO:
            ax.plot(r * np.cos(ang), r * np.sin(ang), np.full_like(ang, zm),
                    ls="--", color="#cbd5e0", lw=1)
        # pedestal: cilindro desde la mesa hasta {0} (la base giratoria real)
        circ = np.linspace(0, 2 * np.pi, 40)
        rb = 4.5
        tapas = [list(zip(rb * np.cos(circ), rb * np.sin(circ), np.full_like(circ, z)))
                 for z in (zm, 0.0)]
        lados = []
        if zm < -0.05:
            for k in range(len(circ) - 1):
                a, b = circ[k], circ[k + 1]
                lados.append([(rb * np.cos(a), rb * np.sin(a), zm),
                              (rb * np.cos(b), rb * np.sin(b), zm),
                              (rb * np.cos(b), rb * np.sin(b), 0.0),
                              (rb * np.cos(a), rb * np.sin(a), 0.0)])
        ax.add_collection3d(Poly3DCollection(lados + tapas, facecolor="#e2e8f0",
                                             edgecolor="none", alpha=0.55))
        ax.plot(rb * np.cos(circ), rb * np.sin(circ), np.zeros_like(circ),
                color="#718096", lw=1)

        # puntos de trabajo
        self.pts = {}
        for n, (x, y) in C.PUNTOS_RECOGIDA.items():
            self.pts[n] = ax.plot([x], [y], [zm], ls="", marker="o", ms=9, mfc="none",
                                  mec="#a0aec0", mew=2)[0]
            ax.text(x + 1, y - 2.5, zm, n, fontsize=9, color="#4a5568")
        x, y = C.PUNTO_APILADO
        ax.plot([x], [y], [zm], ls="", marker="s", ms=10, mfc="none", mec="#3182ce", mew=2)
        ax.text(x + 1, y + 1, zm, "pila", fontsize=9, color="#3182ce")
        for x, y in C.FILA_ORDEN:
            ax.plot([x], [y], [zm], ls="", marker="+", ms=8, color="#805ad5")

        # elementos dinamicos
        self.l_sombra, = ax.plot([], [], [], color="#a0aec0", lw=3, alpha=0.5)
        self.l_rastro, = ax.plot([], [], [], color="#805ad5", lw=1.3, alpha=0.8)
        self.l_brazo, = ax.plot([], [], [], color=COLOR_BRAZO, lw=7, solid_capstyle="round")
        self.l_juntas, = ax.plot([], [], [], ls="", marker="o", ms=9, mfc="#f6ad55",
                                 mec="#2d3748")
        self.l_dedos = [ax.plot([], [], [], color="#2d3748", lw=3.5)[0] for _ in range(3)]
        self.l_tcp, = ax.plot([], [], [], ls="", marker="D", ms=6, color="#e53e3e")
        self.l_haz, = ax.plot([], [], [], color="#38a169", lw=1.6, ls="--")
        self.l_ejes = [[ax.plot([], [], [], color=c, lw=2)[0] for c in COLORES_EJES]
                       for _ in range(6)]
        self.t_etiq = [ax.text(0, 0, 0, f"{{{i}}}", fontsize=8, color="#2d3748")
                       for i in range(6)]
        self.txt_aviso = ax.text2D(0.02, 0.97, "", transform=ax.transAxes, color="#c53030",
                                   fontsize=10, weight="bold", va="top")
        self._cubos = []

    # ------------------------------------------------------------------
    def actualizar(self, T, pinza_pct=None, resaltados=(), ver_ejes=True, ver_etiq=True,
                   esc=4.0, sensor=None, cubos=(), deteccion=None, rastro=None):
        """T: lista [T0_0..T0_5] (listas o arrays 4x4).
        sensor: (origen, direccion, distancia o None).
        cubos: lista de centros. deteccion: {nombre: bool}."""
        T = [np.asarray(Ti, float) for Ti in T]
        P = np.array([Ti[:3, 3] for Ti in T])
        cadena = np.vstack([P[0], P[1], P[2], P[4], P[5]])
        _set3d(self.l_brazo, cadena)
        bajo = P[1:, 2].min() < C.Z_MESA - 1e-6
        self.l_brazo.set_color("#c53030" if bajo else COLOR_BRAZO)
        _set3d(self.l_juntas, np.vstack([P[1], P[2], P[4]]))
        _set3d(self.l_tcp, P[5:6])
        s = cadena.copy()
        s[:, 2] = C.Z_MESA
        _set3d(self.l_sombra, s)

        # pinza: dos dedos que se abren segun el porcentaje del servo 6
        T5 = T[5]
        o, x5, z5 = T5[:3, 3], T5[:3, 0], T5[:3, 2]
        ab = 0.6 + 1.4 * (0.5 if pinza_pct is None else min(max(pinza_pct / 100, 0), 1))
        base = o - 3.0 * x5
        for k, signo in enumerate((-1, 1)):
            _set3d(self.l_dedos[k], [base + signo * ab * z5, o + 1.2 * x5 + signo * ab * z5])
        _set3d(self.l_dedos[2], [base - ab * z5, base + ab * z5])

        for i, Ti in enumerate(T):
            oi = Ti[:3, 3]
            res = i in resaltados
            for k, ln in enumerate(self.l_ejes[i]):
                _set3d(ln, [oi, oi + Ti[:3, k] * esc * (1.4 if res else 1.0)])
                ln.set_linewidth(3.2 if res else 1.2)
                ln.set_alpha(1.0 if res else 0.45)
                ln.set_visible(ver_ejes)
            t = self.t_etiq[i]
            t.set_position((oi[0] + 0.8, oi[1] + 0.8))
            t.set_3d_properties(oi[2] + 1.2, None)
            t.set_visible(ver_etiq)
            t.set_fontweight("bold" if res else "normal")
            t.set_color("#c05621" if res else "#2d3748")

        if sensor is not None:
            so, sd, dist = sensor
            largo = dist if dist is not None else 15.0
            _set3d(self.l_haz, [so, so + np.asarray(sd) * largo])
            self.l_haz.set_color("#38a169" if dist is not None else "#a0aec0")
            self.l_haz.set_visible(True)
        else:
            self.l_haz.set_visible(False)

        for col in self._cubos:
            col.remove()
        self._cubos = []
        for c in cubos:
            col = Poly3DCollection(caras_cubo(c, C.CUBO, C.LARGO_OBJ, C.ANCHO_OBJ),
                                   facecolor="#f6e05e",
                                   edgecolor="#975a16", linewidths=0.8, alpha=0.95)
            self.ax.add_collection3d(col)
            self._cubos.append(col)

        for n, ln in self.pts.items():
            estado = None if not deteccion else deteccion.get(n)
            ln.set_markeredgecolor({None: "#a0aec0", True: "#38a169", False: "#e53e3e"}[estado])

        if rastro is not None and len(rastro) > 1:
            _set3d(self.l_rastro, np.array(rastro))
            self.l_rastro.set_visible(True)
        else:
            self.l_rastro.set_visible(False)
        self.txt_aviso.set_text("Parte del brazo bajo la mesa" if bajo else "")
