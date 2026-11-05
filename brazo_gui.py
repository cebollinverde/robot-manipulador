#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interfaz del manipulador (5 GDL + pinza) — adaptada del simulador DH de Diego.

* Muestra en vivo la tabla DH, las matrices A1..A5 y T0_1..T0_5 y el TCP.
* Se conecta al ESP32 por USB (o funciona en modo SIMULACION sin hardware).
* Si el brazo se mueve desde Blynk, la vista 3D y las matrices lo siguen.
* Control manual por articulacion, cinematica inversa y rutina automatica
  (detectar -> acercarse -> tomar -> apilar / ordenar).
* Atiende los eventos de la app: RUTINA,n (boton V9) y STOP_REQUEST (V10).

Requisitos:  pip install numpy matplotlib pyserial
Ejecutar:    python brazo_gui.py
"""
import math
import queue
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox, ttk

import numpy as np
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from brazo import config as C  # noqa: E402
from brazo import dh  # noqa: E402
from brazo.comunicacion import ESP32, ErrorRobot, Simulador, puertos_disponibles  # noqa: E402
from brazo.dibujo import Escena3D  # noqa: E402
from brazo.geometria import pose_sensor  # noqa: E402
from brazo.inversa import cinematica_inversa, inversa_auto  # noqa: E402
from brazo.rutina import NOMBRES_RUTINA, Rutina  # noqa: E402

LIMITES = dh.limites_angulos()
COLORES_Q = ("#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd", "#8c564b")
SUB = "₀₁₂₃₄₅"
SUP = "⁰¹²³⁴⁵"
EXPR_THETA = ("q1", "q2", "q3 + 90°", "q4", "q5 − 90°")
MODO_SIM = "Simulación (sin brazo)"


def _fmt(v):
    return f"{0.0 if abs(v) < 5e-5 else v: .3f}"


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Manipulador robótico · DH en vivo")
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{min(1560, sw - 40)}x{min(920, sh - 80)}+10+10")
        self.minsize(1150, 660)

        self.robot = None
        self.rutina = None
        self.hilo_rutina = None
        self.avisos = queue.Queue()        # avisos de la rutina (otro hilo)
        self._estado_robot = None          # (moviendo, pcts) del hilo lector
        self._lock = threading.Lock()
        self._vivo = True
        self._ultimo_toque = 0.0           # cuando el usuario movio un slider
        self._envio_pend = None
        self._vel_pend = None
        self._t_xyz = 0.0                  # último envío de X,Y,Z a Blynk
        self._xyz_pend = False
        self._sync = False
        self.q = dh.porcentajes_a_angulos(dh.HOME)
        self.pcts = list(dh.HOME)
        self.A, self.T = dh.matrices(self.q)
        self.dist_sensor = None
        self.deteccion = {}
        self.rastro = []
        self.historial = []                # (t, q1..q5)
        self.t0 = time.time()
        self._dirty = True

        self._estilos()
        self._construir_ui()
        self.escena = Escena3D(self.ax)
        self._fijar_sliders(self.q)
        self.protocol("WM_DELETE_WINDOW", self._cerrar)
        threading.Thread(target=self._hilo_estado, daemon=True).start()
        self.after(80, self._tick)

    # ------------------------------------------------------------------ estilo
    def _estilos(self):
        st = ttk.Style(self)
        if "clam" in st.theme_names():
            st.theme_use("clam")
        self.bg = st.lookup("TFrame", "background") or "#dcdad5"
        base = tkfont.nametofont("TkDefaultFont")
        self.f_bold = base.copy()
        self.f_bold.configure(weight="bold")
        self.f_tit = base.copy()
        self.f_tit.configure(weight="bold", size=base.cget("size") + 1)
        self.f_mono = tkfont.nametofont("TkFixedFont").copy()
        self.f_mono.configure(size=10)
        self.f_peq = base.copy()
        self.f_peq.configure(size=max(base.cget("size") - 1, 8))
        st.configure("Treeview", rowheight=22)
        st.configure("Titulo.TLabel", font=self.f_tit)
        st.configure("Nota.TLabel", foreground="#4a5568", font=self.f_peq)
        st.configure("Accion.TButton", font=self.f_bold)
        st.configure("Stop.TButton", font=self.f_bold, foreground="#c53030")

    # ------------------------------------------------------------------ UI
    def _construir_ui(self):
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        izq = ttk.Frame(self, padding=(8, 8, 4, 4))
        izq.grid(row=0, column=0, sticky="nsw")
        centro = ttk.Frame(self, padding=(4, 8, 4, 4))
        centro.grid(row=0, column=1, sticky="nsew")
        der = ttk.Frame(self, padding=(4, 8, 8, 4))
        der.grid(row=0, column=2, sticky="nse")

        self.var_ver_ejes = tk.BooleanVar(value=True)
        self.var_ver_etiq = tk.BooleanVar(value=True)
        self.var_ver_haz = tk.BooleanVar(value=True)
        self.var_rastro = tk.BooleanVar(value=True)
        self.var_esc = tk.DoubleVar(value=4.0)
        self.var_mat = tk.StringVar(value="T5")

        self._ui_conexion(izq)
        nb = ttk.Notebook(izq)
        nb.pack(fill="both", expand=True, pady=(8, 0))
        for nombre, fn in (("Control", self._ui_manual), ("Inversa", self._ui_inversa),
                           ("Rutina", self._ui_rutina), ("Modelo", self._ui_modelo),
                           ("Calibrar", self._ui_calibrar),
                           ("Vista", self._ui_vista_opc)):
            t = ttk.Frame(nb, padding=10)
            nb.add(t, text=nombre)
            fn(t)
        self._ui_vista(centro)
        self._ui_matrices(der)

        self.var_estado = tk.StringVar(value="Sin conexión. Elige un puerto o el modo simulación.")
        ttk.Label(self, textvariable=self.var_estado, anchor="w", relief="sunken",
                  padding=(8, 2)).grid(row=1, column=0, columnspan=3, sticky="ew")

    def _ui_conexion(self, f):
        caja = ttk.LabelFrame(f, text="Conexión", padding=8)
        caja.pack(fill="x")
        fila = ttk.Frame(caja)
        fila.pack(fill="x")
        self.var_puerto = tk.StringVar(value=MODO_SIM)
        self.cb_puerto = ttk.Combobox(fila, textvariable=self.var_puerto, width=24,
                                      state="readonly")
        self.cb_puerto.pack(side="left", fill="x", expand=True)
        ttk.Button(fila, text="↻", width=3, command=self._refrescar_puertos).pack(side="left",
                                                                               padx=2)
        self.btn_con = ttk.Button(fila, text="Conectar", style="Accion.TButton",
                                  command=self._conectar)
        self.btn_con.pack(side="left")
        self._refrescar_puertos()

        fila2 = ttk.Frame(caja)
        fila2.pack(fill="x", pady=(6, 0))
        ttk.Button(fila2, text="■ STOP", style="Stop.TButton", command=self._stop
                   ).pack(side="left", fill="x", expand=True, padx=(0, 2))
        ttk.Button(fila2, text="Reanudar", command=self._resume).pack(side="left", fill="x",
                                                                      expand=True)
        self.var_con_txt = tk.StringVar(value="● desconectado")
        self.lbl_con = ttk.Label(caja, textvariable=self.var_con_txt, foreground="#a0aec0")
        self.lbl_con.pack(anchor="w", pady=(4, 0))

    # ---------------- Control manual
    def _ui_manual(self, f):
        f.columnconfigure(0, weight=1)
        self.vars_q, self.vars_pct = [], []
        r = 0
        for i in range(5):
            lo, hi = LIMITES[i]
            var = tk.DoubleVar(value=0.0)
            vpct = tk.StringVar(value="")
            cab = ttk.Frame(f)
            cab.grid(row=r, column=0, sticky="ew", pady=(8 if i else 0, 0))
            tk.Label(cab, text="■", fg=COLORES_Q[i], bg=self.bg).pack(side="left")
            ttk.Label(cab, text=dh.NOMBRES[i], font=self.f_bold).pack(side="left", padx=(2, 0))
            ttk.Label(cab, textvariable=vpct, style="Nota.TLabel").pack(side="left", padx=6)
            ttk.Label(cab, text="°").pack(side="right")
            sp = ttk.Spinbox(cab, from_=lo, to=hi, increment=1, width=7, textvariable=var,
                             justify="right")
            sp.pack(side="right")
            sc = tk.Scale(f, from_=lo, to=hi, orient="horizontal", resolution=0.5,
                          variable=var, showvalue=False, length=290, sliderlength=18,
                          width=13, highlightthickness=0, bg=self.bg,
                          troughcolor="#e2e8f0", activebackground=COLORES_Q[i])
            sc.grid(row=r + 1, column=0, sticky="ew")
            var.trace_add("write", self._on_var_q)
            self.vars_q.append(var)
            self.vars_pct.append(vpct)
            r += 2

        self.var_enviar = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="Mover el brazo al cambiar los sliders",
                        variable=self.var_enviar).grid(row=r, column=0, sticky="w", pady=(8, 2))
        r += 1
        fb = ttk.Frame(f)
        fb.grid(row=r, column=0, sticky="ew", pady=(4, 0))
        for c in range(3):
            fb.columnconfigure(c, weight=1)
        botones = [("Home (calibración)", self._home), ("Abrir pinza", lambda: self._pinza(True)),
                   ("Cerrar pinza", lambda: self._pinza(False)),
                   ("Medir distancia", self._medir)]
        for k, (txt, cmd) in enumerate(botones):
            ttk.Button(fb, text=txt, command=cmd).grid(row=k // 2, column=k % 2,
                                                       sticky="ew", padx=2, pady=2)
        r += 1
        fv = ttk.Frame(f)
        fv.grid(row=r, column=0, sticky="ew", pady=(8, 0))
        ttk.Label(fv, text="Velocidad:").pack(side="left")
        self.var_vel = tk.IntVar(value=C.VELOCIDAD)
        tk.Scale(fv, from_=1, to=100, orient="horizontal", variable=self.var_vel,
                 length=170, sliderlength=16, width=12, highlightthickness=0, bg=self.bg,
                 troughcolor="#e2e8f0", command=lambda v: self._programar_velocidad()
                 ).pack(side="left", padx=4)
        r += 1
        self.var_dist = tk.StringVar(value="Sensor: —")
        ttk.Label(f, textvariable=self.var_dist, font=self.f_mono).grid(row=r, column=0,
                                                                         sticky="w", pady=(8, 0))
        r += 1
        ttk.Label(f, style="Nota.TLabel", wraplength=300, justify="left", text=(
            "• El % al lado de cada nombre es lo que recibe el ESP32 (0-100 % "
            "normalizado, igual que en Blynk).\n"
            "• Si mueves el brazo desde la app, los sliders y las matrices lo siguen.\n"
            "• q4 es el giro de la muñeca: la rutina lo deja en 0°.")
        ).grid(row=r, column=0, sticky="w", pady=(8, 0))

    # ---------------- Inversa
    def _ui_inversa(self, f):
        ttk.Label(f, text="Ir a un punto (x, y, z) en cm", font=self.f_bold).grid(
            row=0, column=0, columnspan=3, sticky="w")
        self.vars_xyz = []
        for k, (nom, v) in enumerate((("x", 22.0), ("y", 0.0), ("z", 6.0), ("φ [°]", -45.0))):
            ttk.Label(f, text=nom).grid(row=1 + k, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=f"{v:.1f}")
            ttk.Entry(f, textvariable=var, width=9, justify="right").grid(
                row=1 + k, column=1, sticky="w", padx=4)
            self.vars_xyz.append(var)
        self.var_phi_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="Si φ no alcanza, buscar la más cercana",
                        variable=self.var_phi_auto).grid(row=5, column=0, columnspan=3,
                                                         sticky="w", pady=(4, 0))
        fb = ttk.Frame(f)
        fb.grid(row=6, column=0, columnspan=3, sticky="ew", pady=8)
        ttk.Button(fb, text="Resolver", command=lambda: self._resolver_ik(False)
                   ).pack(side="left", padx=(0, 4))
        ttk.Button(fb, text="Resolver y mover", style="Accion.TButton",
                   command=lambda: self._resolver_ik(True)).pack(side="left")
        ttk.Button(fb, text="← TCP actual", command=self._xyz_actual).pack(side="left", padx=4)
        self.var_ik = tk.StringVar(value="")
        ttk.Label(f, textvariable=self.var_ik, font=self.f_mono, justify="left").grid(
            row=7, column=0, columnspan=3, sticky="w")
        ttk.Label(f, style="Nota.TLabel", wraplength=300, justify="left", text=(
            "Desacople de muñeca: con φ se calcula la muñeca W = P − LG·(cos φ, sin φ); "
            "con la ley de cosenos sale q3 (dos soluciones de codo), luego q2, y "
            "q5 = φ − q2 − q3. Se elige la solución dentro de los límites que "
            "menos se mueve, y se comprueba con la cinemática directa.")
        ).grid(row=8, column=0, columnspan=3, sticky="w", pady=(10, 0))

    # ---------------- Rutina
    def _ui_rutina(self, f):
        f.columnconfigure(0, weight=1)
        r = 0
        fr = ttk.Frame(f)
        fr.grid(row=r, column=0, sticky="ew")
        ttk.Label(fr, text="Rutina:").pack(side="left")
        self.var_num_rut = tk.StringVar(value="1 · " + NOMBRES_RUTINA[1])
        ttk.Combobox(fr, textvariable=self.var_num_rut, state="readonly", width=24,
                     values=[f"{k} · {v}" for k, v in NOMBRES_RUTINA.items()]
                     ).pack(side="left", padx=4, fill="x", expand=True)
        r += 1
        fe = ttk.Frame(f)
        fe.grid(row=r, column=0, sticky="ew", pady=(8, 2))
        fe.columnconfigure(0, weight=1)
        fe.columnconfigure(1, weight=1)
        ttk.Button(fe, text="▶ Ejecutar", style="Accion.TButton",
                   command=lambda: self._iniciar_rutina(int(self.var_num_rut.get()[0]))
                   ).grid(row=0, column=0, sticky="ew", padx=2)
        ttk.Button(fe, text="⏹ Detener", command=self._detener_rutina
                   ).grid(row=0, column=1, sticky="ew", padx=2)
        r += 1
        self.var_prog = tk.DoubleVar(value=0.0)
        ttk.Progressbar(f, variable=self.var_prog, maximum=100).grid(row=r, column=0,
                                                                      sticky="ew", pady=4)
        r += 1

        caja = ttk.LabelFrame(f, text="Cubos en la simulación", padding=6)
        caja.grid(row=r, column=0, sticky="ew", pady=(4, 4))
        self.vars_cubo = {}
        for n in C.PUNTOS_RECOGIDA:
            v = tk.BooleanVar(value=n != "B")
            ttk.Checkbutton(caja, text=n, variable=v).pack(side="left", padx=4)
            self.vars_cubo[n] = v
        ttk.Button(caja, text="Colocar", command=self._colocar_cubos).pack(side="right")
        r += 1

        ttk.Label(f, text="Detección", font=self.f_bold).grid(row=r, column=0, sticky="w",
                                                              pady=(6, 0))
        r += 1
        cols = ("p", "med", "esp", "res")
        self.tree_det = ttk.Treeview(f, columns=cols, show="headings", height=4)
        for c, txt, w in zip(cols, ("Punto", "Medido", "Mesa", "Resultado"), (55, 70, 60, 100)):
            self.tree_det.heading(c, text=txt)
            self.tree_det.column(c, width=w, anchor="center", stretch=False)
        self.tree_det.grid(row=r, column=0, sticky="ew")
        r += 1
        ttk.Label(f, text="Registro", font=self.f_bold).grid(row=r, column=0, sticky="w",
                                                             pady=(8, 0))
        r += 1
        fl = ttk.Frame(f)
        fl.grid(row=r, column=0, sticky="nsew")
        f.rowconfigure(r, weight=1)
        self.lst_log = tk.Listbox(fl, height=10, font=self.f_peq, activestyle="none")
        sb = ttk.Scrollbar(fl, orient="vertical", command=self.lst_log.yview)
        self.lst_log.configure(yscrollcommand=sb.set)
        self.lst_log.pack(side="left", fill="both", expand=True)
        sb.pack(side="left", fill="y")
        r += 1
        ttk.Label(f, style="Nota.TLabel", wraplength=300, justify="left", text=(
            "Los puntos, el tamaño del cubo y la posición del sensor se ajustan "
            "en brazo/config.py. El botón ▶ de Blynk (V9) también inicia la rutina "
            "elegida en V8 si esta ventana está conectada.")
        ).grid(row=r, column=0, sticky="w", pady=(6, 0))

    # ---------------- Modelo: largos del DH y altura de la base
    CAMPOS_MODELO = [
        ("H", "d₁  altura del eje del hombro sobre {0}"),
        ("R0", "a₁  adelanto del eje del hombro"),
        ("L1", "a₂  eje hombro → eje codo"),
        ("L2", "d₄  eje codo → eje muñeca (q5)"),
        ("LG", "a₅  eje muñeca → centro de la pinza"),
        ("BASE", "{0} sobre la mesa (alto de la base)"),
    ]

    def _ui_modelo(self, f):
        f.columnconfigure(1, weight=1)
        ttk.Label(f, style="Nota.TLabel", wraplength=300, justify="left", text=(
            "Medidas en cm, con regla, de CENTRO a CENTRO de los ejes (tornillo "
            "central de cada servo). {0} es el punto del eje de giro de la base "
            "desde donde se miden las alturas: la cara de arriba de la base giratoria.")
        ).grid(row=0, column=0, columnspan=2, sticky="w")
        self.vars_modelo = {}
        for k, (clave, txt) in enumerate(self.CAMPOS_MODELO):
            ttk.Label(f, text=txt, wraplength=210).grid(row=1 + k, column=0, sticky="w",
                                                        pady=2)
            var = tk.StringVar()
            ttk.Entry(f, textvariable=var, width=8, justify="right").grid(
                row=1 + k, column=1, sticky="w", padx=4)
            self.vars_modelo[clave] = var
        self._llenar_modelo()
        fb = ttk.Frame(f)
        fb.grid(row=8, column=0, columnspan=2, sticky="ew", pady=8)
        ttk.Button(fb, text="Aplicar", style="Accion.TButton",
                   command=self._aplicar_modelo).pack(side="left", padx=(0, 4))
        ttk.Button(fb, text="💾 Guardar", command=self._guardar_modelo).pack(side="left",
                                                                         padx=(0, 4))
        ttk.Button(fb, text="Originales", command=self._modelo_original).pack(side="left")
        ttk.Label(f, style="Nota.TLabel", wraplength=300, justify="left", text=(
            "Para comprobar: pulsa «Home (calibración)» en Control y compara de "
            "costado el brazo real con el dibujo. Brazo subiendo 62°, antebrazo casi "
            "en ángulo recto y pinza 25° hacia abajo. La pinza debe quedar a la "
            "misma altura sobre la mesa en el dibujo y en la realidad.")
        ).grid(row=9, column=0, columnspan=2, sticky="w")
        self.var_alcance = tk.StringVar()
        ttk.Label(f, textvariable=self.var_alcance, font=self.f_mono, justify="left").grid(
            row=10, column=0, columnspan=2, sticky="w", pady=(8, 0))
        self._mostrar_alcance()

    def _llenar_modelo(self):
        m = dh.medidas()
        m["BASE"] = -C.Z_MESA
        for k, var in self.vars_modelo.items():
            var.set(f"{m[k]:.2f}")

    def _mostrar_alcance(self):
        x, y, z = dh.cinematica_directa(dh.porcentajes_a_angulos(dh.HOME))
        self.var_alcance.set(
            f"Pinza en Home: {x:5.1f} cm adelante\n"
            f"               {z - C.Z_MESA:5.1f} cm sobre la mesa\n"
            f"Alcance máx.:  {dh.R0 + dh.L1 + dh.L2 + dh.LG:5.1f} cm")

    def _aplicar_modelo(self):
        try:
            v = {k: float(var.get().replace(",", ".")) for k, var in self.vars_modelo.items()}
            if v["BASE"] < 0:
                raise ValueError("la altura de la base no puede ser negativa")
            dh.fijar_medidas(**{k: v[k] for k in ("H", "R0", "L1", "L2", "LG")})
        except (ValueError, KeyError) as e:
            messagebox.showerror("Modelo", f"Revisa los valores: {e}")
            return False
        C.Z_MESA = -v["BASE"]
        if hasattr(self, "vars_cal"):
            self.vars_cal["zm"].set(f"{v['BASE']:.2f}")
        self.rastro.clear()
        self._rehacer_escena()
        self._mostrar_alcance()
        self.var_estado.set("Medidas aplicadas (falta «Guardar» para que queden).")
        return True

    def _guardar_modelo(self):
        if self._aplicar_modelo():
            dh.guardar_medidas()
            C.guardar_zona()
            self.var_estado.set(f"Medidas guardadas en {dh.ARCHIVO_MEDIDAS}")

    def _modelo_original(self):
        dh.fijar_medidas(**dh.MEDIDAS_ORIGINALES)
        self._llenar_modelo()
        self._aplicar_modelo()

    # ---------------- Calibración por enseñanza
    def _ui_calibrar(self, f):
        f.columnconfigure(1, weight=1)
        ttk.Label(f, style="Nota.TLabel", wraplength=300, justify="left", text=(
            "Lleva el brazo (sliders o Blynk) hasta que la pinza agarre un cubo apoyado "
            "en la mesa, en el lugar que quieres registrar, y pulsa «Guardar aquí». "
            "La DH calcula x, y del punto y, si marcas la casilla, la altura de la mesa.")
        ).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(f, text="Punto:").grid(row=1, column=0, sticky="w", pady=(8, 2))
        self.var_punto_cal = tk.StringVar(value="A")
        ttk.Combobox(f, textvariable=self.var_punto_cal, state="readonly", width=16,
                     values=["A", "B", "C", "Pila  (punto final, donde se apila)", "Fila (agregar)"]
                     ).grid(row=1, column=1, sticky="w", pady=(8, 2))
        self.var_cal_mesa = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="Ajustar altura de mesa con este punto",
                        variable=self.var_cal_mesa).grid(row=2, column=0, columnspan=2,
                                                         sticky="w")
        fb = ttk.Frame(f)
        fb.grid(row=3, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Button(fb, text="Guardar aquí", style="Accion.TButton",
                   command=self._ensenar_punto).pack(side="left", padx=(0, 4))
        ttk.Button(fb, text="Quitar punto", command=self._quitar_punto).pack(side="left",
                                                                            padx=(0, 4))
        ttk.Button(fb, text="Limpiar fila", command=self._limpiar_fila).pack(side="left")

        ttk.Separator(f).grid(row=4, column=0, columnspan=2, sticky="ew", pady=8)
        self.vars_cal = {}
        campos = [("Objeto: alto acostado [cm]", "cubo", C.CUBO),
                  ("Objeto: largo (hacia la base) [cm]", "largo", C.LARGO_OBJ),
                  ("Objeto: ancho (entre dedos) [cm]", "ancho", C.ANCHO_OBJ),
                  ("Sensor x₅ (adelante) [cm]", "sx", C.SENSOR_OFFSET[0]),
                  ("Sensor y₅ (arriba) [cm]", "sy", C.SENSOR_OFFSET[1]),
                  ("Sensor z₅ (costado) [cm]", "sz", C.SENSOR_OFFSET[2]),
                  ("Altura de {0} sobre la mesa [cm]", "zm", -C.Z_MESA)]
        for k, (txt, clave, v) in enumerate(campos):
            ttk.Label(f, text=txt).grid(row=5 + k, column=0, sticky="w", pady=1)
            var = tk.StringVar(value=f"{v:.2f}")
            ttk.Entry(f, textvariable=var, width=8, justify="right").grid(
                row=5 + k, column=1, sticky="w")
            self.vars_cal[clave] = var
        fg = ttk.Frame(f)
        fg.grid(row=5 + len(campos), column=0, columnspan=2, sticky="ew", pady=8)
        ttk.Button(fg, text="Aplicar", command=self._aplicar_cal).pack(side="left", padx=(0, 4))
        ttk.Button(fg, text="💾 Guardar calibración", command=self._guardar_cal
                   ).pack(side="left")
        self.var_zona = tk.StringVar()
        ttk.Label(f, textvariable=self.var_zona, font=self.f_mono, justify="left").grid(
            row=6 + len(campos), column=0, columnspan=2, sticky="w")
        self._mostrar_zona()

    def _mostrar_zona(self):
        def p(xy):
            return f"({xy[0]:5.1f}, {xy[1]:5.1f})"
        lineas = [f"{{0}} a {-C.Z_MESA:.2f} cm sobre la mesa",
                  f"objeto {C.LARGO_OBJ:.2f} × {C.ANCHO_OBJ:.2f} × {C.CUBO:.2f} cm"]
        def marca(n):
            return "  ✓ enseñado" if n in C.POSES else "  (sin enseñar)"
        lineas += [f"{n}: {p(xy)}{marca(n)}" for n, xy in C.PUNTOS_RECOGIDA.items()]
        lineas.append(f"Pila: {p(C.PUNTO_APILADO)}{marca('Pila')}")
        lineas.append("Fila: " + (" ".join(p(xy) for xy in C.FILA_ORDEN) or "—"))
        self.var_zona.set("\n".join(lineas))

    def _revisar_alcance(self, nombre):
        """Avisa si, desde la postura enseñada, el brazo no puede subir sobre el
        punto o mirarlo con el sensor. El agarre en sí siempre se puede: es la
        postura que se enseñó."""
        from brazo.geometria import pose_escaneo
        p = C.POSES.get(nombre)
        if p is None:
            return
        x, y, z = p["xyz"]
        problemas = []
        if inversa_auto(x, y, z + C.ALTURA_APROX, p["phi"], p["q"])[0] is None:
            problemas.append(f"no puede subir {C.ALTURA_APROX:.0f} cm sobre el punto")
        if nombre in "ABC" and pose_escaneo(x, y, p["q"], z - C.CUBO / 2, p["phi"])[0] is None:
            problemas.append("el sensor no alcanza a mirarlo")
        if problemas:
            messagebox.showwarning(
                "Calibración", f"Punto {nombre} guardado, pero: " + "; ".join(problemas)
                + ".\nPrueba poniendo el objeto un poco más lejos de la base, o enséñalo "
                "con la pinza menos inclinada.")

    def _rehacer_escena(self):
        self.escena.construir(self.ax.elev, self.ax.azim)
        self._mostrar_zona()
        for k, v in (("cubo", C.CUBO), ("zm", -C.Z_MESA)):
            self.vars_cal[k].set(f"{v:.2f}")
        if hasattr(self, "vars_modelo"):
            self.vars_modelo["BASE"].set(f"{-C.Z_MESA:.2f}")
            self._mostrar_alcance()
        self._dirty = True

    def _ensenar_punto(self):
        """Guarda la postura ACTUAL del brazo como el punto elegido."""
        if self.robot is None or self.robot.simulado:
            messagebox.showerror(
                "Calibración", "Primero conecten el brazo real (↻ → COM4 → Conectar).\n"
                "Sin conexión se guardaría la posición del dibujo, no la del brazo.")
            return
        x, y, z = dh.cinematica_directa(self.q)
        nombre = self.var_punto_cal.get().split()[0]
        xy = (round(x, 2), round(y, 2))
        if nombre.startswith("Fila"):
            C.FILA_ORDEN = list(C.FILA_ORDEN) + [xy]
        else:
            C.POSES = dict(C.POSES)
            C.POSES[nombre] = {"q": [round(float(a), 3) for a in self.q],
                               "xyz": [round(x, 2), round(y, 2), round(z, 2)],
                               "phi": round(dh.inclinacion_pinza(self.q), 2)}
            if nombre == "Pila":
                C.PUNTO_APILADO = xy
            else:
                C.PUNTOS_RECOGIDA = dict(C.PUNTOS_RECOGIDA, **{nombre: xy})
        if self.var_cal_mesa.get() and C.POSES:
            # mesa = promedio de lo enseñado (el centro de la pinza está en el centro del objeto)
            alturas = [p["xyz"][2] - C.CUBO / 2 for p in C.POSES.values()]
            C.Z_MESA = round(sum(alturas) / len(alturas), 2)
        self._rehacer_escena()
        # ¿quedó encima de otro punto? (típico si el brazo no se movió entre un punto y otro)
        if not nombre.startswith("Fila"):
            for otro, p in C.POSES.items():
                if otro != nombre and math.hypot(p["xyz"][0] - x, p["xyz"][1] - y) < 4:
                    messagebox.showwarning(
                        "Calibración", f"{nombre} quedó casi en el mismo lugar que {otro}.\n"
                        "¿Movieron el brazo hasta la otra caja antes de guardar? "
                        "Si no, muévanlo y vuelvan a guardar este punto.")
                    break
        if abs(self.q[3]) > 3:
            messagebox.showwarning(
                "Calibración", f"Muñeca 1 (giro) está en {self.q[3]:.0f}°. La rutina usa 0°, "
                "así que la pinza llegaría girada.\nDejen Muñeca 1 en 0° (servo 4 en "
                f"{dh.angulos_a_porcentajes([0, 62, -89, 0, 2])[3]:.1f} % en la app) y "
                "vuelvan a guardar este punto.")
        else:
            self._revisar_alcance(nombre)
        self.var_estado.set(f"{nombre} guardado: x = {x:.1f}, y = {y:.1f}, z = {z:.1f} cm "
                            "(falta «💾 Guardar calibración»)")

    def _quitar_punto(self):
        """Quita A, B o C de la rutina (por ejemplo, si solo usan 2 puntos)."""
        nombre = self.var_punto_cal.get().split()[0]
        if nombre not in C.PUNTOS_RECOGIDA:
            self.var_estado.set("Solo se pueden quitar los puntos de recogida (A, B, C).")
            return
        if len(C.PUNTOS_RECOGIDA) == 1:
            self.var_estado.set("Debe quedar al menos un punto de recogida.")
            return
        C.PUNTOS_RECOGIDA = {n: p for n, p in C.PUNTOS_RECOGIDA.items() if n != nombre}
        C.POSES = {n: p for n, p in C.POSES.items() if n != nombre}
        self._rehacer_escena()
        self.var_estado.set(f"Punto {nombre} quitado (falta «Guardar calibración»). "
                            "Para volver a usarlo, guárdalo de nuevo con «Guardar aquí».")

    def _limpiar_fila(self):
        C.FILA_ORDEN = []
        self._rehacer_escena()

    def _aplicar_cal(self):
        try:
            v = {k: float(var.get().replace(",", ".")) for k, var in self.vars_cal.items()}
        except ValueError:
            messagebox.showerror("Calibración", "Escribe números.")
            return False
        if min(v["cubo"], v["largo"], v["ancho"]) <= 0:
            messagebox.showerror("Calibración", "Las medidas del objeto deben ser > 0.")
            return False
        C.CUBO, C.Z_MESA = v["cubo"], -v["zm"]
        C.LARGO_OBJ, C.ANCHO_OBJ = v["largo"], v["ancho"]
        C.SENSOR_OFFSET = (v["sx"], v["sy"], v["sz"])
        self._rehacer_escena()
        return True

    def _guardar_cal(self):
        if self._aplicar_cal():
            C.guardar_zona()
            self.var_estado.set(f"Calibración guardada en {C.ARCHIVO_ZONA}")

    # ---------------- Opciones de vista
    def _ui_vista_opc(self, f):
        checks = [("Sistemas de referencia {i}", self.var_ver_ejes),
                  ("Etiquetas {i}", self.var_ver_etiq),
                  ("Haz del sensor", self.var_ver_haz),
                  ("Rastro del TCP", self.var_rastro)]
        for k, (txt, var) in enumerate(checks):
            ttk.Checkbutton(f, text=txt, variable=var, command=self._marcar).grid(
                row=k, column=0, columnspan=2, sticky="w")
        fe = ttk.Frame(f)
        fe.grid(row=4, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(fe, text="Largo de ejes:").pack(side="left")
        tk.Scale(fe, from_=1, to=8, resolution=0.5, orient="horizontal", variable=self.var_esc,
                 showvalue=False, length=140, sliderlength=16, width=12, highlightthickness=0,
                 bg=self.bg, troughcolor="#e2e8f0", command=lambda v: self._marcar()
                 ).pack(side="left", padx=4)
        ttk.Button(f, text="Borrar rastro", command=self._borrar_rastro).grid(
            row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Label(f, text="Vistas", font=self.f_bold).grid(row=6, column=0, columnspan=2,
                                                           sticky="w", pady=(12, 2))
        vistas = [("Isométrica", 24, -60), ("Frontal (XZ)", 0, -90),
                  ("Lateral (YZ)", 0, 0), ("Superior (XY)", 90, -90)]
        for k, (txt, el, az) in enumerate(vistas):
            ttk.Button(f, text=txt, command=lambda el=el, az=az: self._vista(el, az)).grid(
                row=7 + k // 2, column=k % 2, sticky="ew", padx=2, pady=2)

    # ---------------- Centro: 3D + historial
    def _ui_vista(self, f):
        self.nb_centro = ttk.Notebook(f)
        self.nb_centro.pack(fill="both", expand=True)
        t3d = ttk.Frame(self.nb_centro)
        tq = ttk.Frame(self.nb_centro)
        self.nb_centro.add(t3d, text="Vista 3D")
        self.nb_centro.add(tq, text="Historial q(t)")
        self.fig = Figure(figsize=(6.5, 6.5), dpi=100)
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas3d = FigureCanvasTkAgg(self.fig, master=t3d)
        self.canvas3d.get_tk_widget().pack(fill="both", expand=True)
        self.txt_q = self.fig.text(0.98, 0.97, "", color="#2d3748", fontsize=9, va="top",
                                   ha="right", family="monospace")
        self.fig.text(0.02, 0.02, "ejes: x rojo · y verde · z azul   ◆ TCP   ○ puntos A/B/C "
                      "(verde = hay cubo, rojo = vacío)   □ pila", fontsize=8, color="#718096")

        self.fig_q = Figure(figsize=(6.5, 6.5), dpi=100)
        self.ax_q, self.ax_z = self.fig_q.subplots(2, 1, sharex=True)
        self.canvas_q = FigureCanvasTkAgg(self.fig_q, master=tq)
        self.canvas_q.get_tk_widget().pack(fill="both", expand=True)

    # ---------------- Derecha: matrices
    def _ui_matrices(self, f):
        ttk.Label(f, text="Tabla DH (valores actuales)", style="Titulo.TLabel").pack(anchor="w")
        self.tree_dh = ttk.Treeview(f, columns=("i", "th", "d", "a", "al"), show="headings",
                                    height=5, selectmode="browse")
        for c, txt, w in zip(("i", "th", "d", "a", "al"),
                             ("i", "θᵢ", "dᵢ [cm]", "aᵢ [cm]", "αᵢ"), (26, 150, 62, 62, 44)):
            self.tree_dh.heading(c, text=txt)
            self.tree_dh.column(c, width=w, anchor="center", stretch=False)
        for i in range(1, 6):
            self.tree_dh.insert("", "end", iid=str(i), values=(i, "", "", "", ""))
        self.tree_dh.pack(anchor="w", pady=(2, 8))
        self.tree_dh.bind("<<TreeviewSelect>>", self._on_sel_dh)

        ttk.Label(f, text="Matriz seleccionada", style="Titulo.TLabel").pack(anchor="w")
        fs = ttk.Frame(f)
        fs.pack(anchor="w", pady=(2, 4))
        for k in range(1, 6):
            ttk.Radiobutton(fs, text=f"A{SUB[k]}", value=f"A{k}", variable=self.var_mat,
                            style="Toolbutton", width=5, command=self._on_sel_matriz
                            ).grid(row=0, column=k - 1, padx=1, pady=1)
            ttk.Radiobutton(fs, text=f"T{SUB[0]}{SUP[k]}", value=f"T{k}", variable=self.var_mat,
                            style="Toolbutton", width=5, command=self._on_sel_matriz
                            ).grid(row=1, column=k - 1, padx=1, pady=1)
        self.var_formula = tk.StringVar()
        ttk.Label(f, textvariable=self.var_formula, wraplength=360, justify="left").pack(
            anchor="w", pady=(2, 4))
        self.celdas_sel = self._grilla_matriz(f)

        ttk.Label(f, text=f"Efector final  T{SUB[0]}{SUP[5]} = A₁·A₂·A₃·A₄·A₅",
                  style="Titulo.TLabel").pack(anchor="w", pady=(6, 0))
        self.celdas_T = self._grilla_matriz(f)
        ley = ttk.Frame(f)
        ley.pack(anchor="w")
        for color, txt in (("#dbe9ff", "rotación R"), ("#fde3c0", "posición p [cm]"),
                           ("#e9edf2", "fila homogénea")):
            tk.Label(ley, text="  ", bg=color, relief="solid", bd=1).pack(side="left")
            ttk.Label(ley, text=txt, style="Nota.TLabel").pack(side="left", padx=(3, 10))

        ttk.Label(f, text="Herramienta (centro de la pinza)", style="Titulo.TLabel").pack(
            anchor="w", pady=(10, 0))
        self.var_tcp = tk.StringVar()
        ttk.Label(f, textvariable=self.var_tcp, font=self.f_mono, justify="left").pack(anchor="w")
        ttk.Button(f, text="Copiar matrices al portapapeles", command=self._copiar_matrices
                   ).pack(anchor="w", pady=(8, 0))

    def _grilla_matriz(self, padre):
        marco = tk.Frame(padre, bg="#a0aec0", padx=1, pady=1)
        marco.pack(anchor="w", pady=(2, 4))
        celdas = []
        for r in range(4):
            fila = []
            for c in range(4):
                bg = "#e9edf2" if r == 3 else ("#fde3c0" if c == 3 else "#dbe9ff")
                lb = tk.Label(marco, text="", width=9, font=self.f_mono, bg=bg, anchor="e",
                              padx=4, pady=2)
                lb.grid(row=r, column=c, padx=(0, 1 if c < 3 else 0),
                        pady=(0, 1 if r < 3 else 0))
                fila.append(lb)
            celdas.append(fila)
        return celdas

    # ================================================================== conexión
    def _refrescar_puertos(self):
        self.cb_puerto["values"] = [MODO_SIM] + puertos_disponibles()

    def _conectar(self):
        if self.robot is not None:
            self._desconectar()
            return
        destino = self.var_puerto.get()
        try:
            if destino == MODO_SIM:
                self.robot = Simulador()
                self._colocar_cubos()
            else:
                self.var_estado.set(f"Conectando a {destino}… (el ESP32 se reinicia)")
                self.update_idletasks()
                self.robot = ESP32(destino)
            self.robot.velocidad(self.var_vel.get())
        except (ErrorRobot, OSError) as e:
            self.robot = None
            messagebox.showerror("Conexión", str(e))
            return
        self.rutina = Rutina(self.robot, avisar=lambda *a: self.avisos.put(a))
        self.btn_con.configure(text="Desconectar")
        self.var_con_txt.set("● " + ("simulación" if self.robot.simulado else destino))
        self.lbl_con.configure(foreground="#38a169")
        self.var_estado.set("Conectado. La vista muestra la posición REAL del brazo.")

    def _desconectar(self):
        self._detener_rutina()
        r, self.robot = self.robot, None
        if r is not None:
            r.cerrar()
        self.btn_con.configure(text="Conectar")
        self.var_con_txt.set("● desconectado")
        self.lbl_con.configure(foreground="#a0aec0")
        self.var_estado.set("Desconectado.")

    def _hilo_estado(self):
        """Lee STATE ~10 veces por segundo (en otro hilo, para no congelar la ventana)."""
        while self._vivo:
            r = self.robot
            if r is not None:
                try:
                    est = r.estado()
                    with self._lock:
                        self._estado_robot = est
                except Exception:          # noqa: BLE001 (puerto cerrado al desconectar)
                    pass
            time.sleep(0.1)

    def _en_hilo(self, fn, *args, ok=None):
        """Ejecuta un comando al robot sin bloquear la interfaz."""
        if self.robot is None:
            self.var_estado.set("Primero conecta el brazo o el modo simulación.")
            return

        def tarea():
            try:
                res = fn(*args)
                if ok:      # el callback toca widgets: se ejecuta en el hilo de la ventana
                    self.avisos.put(("ok", ok, res))
            except ErrorRobot as e:
                self.avisos.put(("log", f"Error: {e}"))
        threading.Thread(target=tarea, daemon=True).start()

    def _rutina_activa(self):
        return self.hilo_rutina is not None and self.hilo_rutina.is_alive()

    # ================================================================== bucle de la interfaz
    def _tick(self):
        """Bucle de la ventana (cada 80 ms). Un error no lo detiene: se muestra abajo."""
        if not self._vivo:
            return
        try:
            self._tick_cuerpo()
        except Exception as e:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            self.var_estado.set(f"Error al actualizar: {e}")
        self.after(80, self._tick)

    def _tick_cuerpo(self):
        # 1) eventos del ESP32 (botones de Blynk)
        if self.robot is not None:
            while not self.robot.eventos.empty():
                self._evento(self.robot.eventos.get_nowait())
        # 2) avisos de la rutina
        while not self.avisos.empty():
            self._aviso(*self.avisos.get_nowait())
        # 3) posición real del brazo
        with self._lock:
            est, self._estado_robot = self._estado_robot, None
        if est is not None and self.robot is not None:
            _, pcts = est
            q = dh.porcentajes_a_angulos(pcts)
            usuario = time.time() - self._ultimo_toque < 1.0 and not self._rutina_activa()
            if not usuario and (np.max(np.abs(np.subtract(q, self.q))) > 0.05
                                or abs(pcts[5] - self.pcts[5]) > 0.05):
                self.q, self.pcts = q, pcts
                self._fijar_sliders(q)
                self._registrar()
                self._dirty = True
        if self._dirty:
            self._dirty = False
            self.actualizar()
        elif self._xyz_pend:
            self._publicar_xyz()

    def _evento(self, ev):
        if ev.startswith("RUTINA,"):
            try:
                n = int(ev.split(",")[1])
            except ValueError:
                return
            self._log(f"Blynk pidió la rutina {n}")
            self._iniciar_rutina(n)
        elif ev == "STOP_REQUEST":
            self._log("STOP desde Blynk")
            if self.rutina:
                self.rutina.parar.set()
        elif ev == "DESCONECTADO":
            self._log("Se perdió el puerto serial")
            self._desconectar()
        elif ev == "READY":
            self._log("El ESP32 se reinició")

    def _aviso(self, tipo, *datos):
        if tipo == "log":
            self._log(datos[0])
        elif tipo == "ok":
            msg = datos[0](datos[1])
            if msg:
                self.var_estado.set(msg)
        elif tipo == "deteccion":
            res = datos[0]
            self.deteccion[res["nombre"]] = res["presente"]
            med = "sin eco" if res["medido"] is None else f"{res['medido']:.1f} cm"
            txt = f"{res['cubos']} objeto(s)" if res["presente"] else "vacío"
            iid = res["nombre"]
            vals = (iid, med, f"{res['esperado']:.1f} cm", txt)
            if self.tree_det.exists(iid):
                self.tree_det.item(iid, values=vals)
            else:
                self.tree_det.insert("", "end", iid=iid, values=vals)
            self._dirty = True
        elif tipo == "progreso":
            self.var_prog.set(100 * datos[0])
        elif tipo == "fin":
            self.var_estado.set(f"Rutina {datos[0].lower()}.")
            self.var_prog.set(100 if datos[0] == "TERMINADA" else self.var_prog.get())

    def _log(self, txt):
        self.lst_log.insert("end", time.strftime("%H:%M:%S ") + txt)
        self.lst_log.see("end")
        if self.lst_log.size() > 400:
            self.lst_log.delete(0, 100)

    # ================================================================== acciones
    def _stop(self):
        if self.rutina:
            self.rutina.parar.set()
        self._en_hilo(lambda: self.robot.stop(), ok=lambda _: "DETENIDO. Pulsa Reanudar.")

    def _resume(self):
        self._en_hilo(lambda: self.robot.resume(), ok=lambda _: "Reanudado.")

    def _home(self):
        if self._rutina_activa():
            return
        self._en_hilo(lambda: self.robot.home(), ok=lambda _: "Volviendo a la postura de calibración.")

    def _pinza(self, abrir):
        if self._rutina_activa():
            return
        self._en_hilo(self.robot.pinza if self.robot else None, abrir)

    def _medir(self):
        def ok(d):
            self.dist_sensor = d
            self.var_dist.set("Sensor: sin eco" if d is None else f"Sensor: {d:6.2f} cm")
            self._dirty = True
            return None
        self._en_hilo(lambda: self.robot.distancia(), ok=ok)

    def _programar_velocidad(self):
        if self.robot is None:
            return
        if self._vel_pend:
            self.after_cancel(self._vel_pend)
        self._vel_pend = self.after(300, lambda: self._en_hilo(self.robot.velocidad,
                                                               self.var_vel.get()))

    def _colocar_cubos(self):
        if isinstance(self.robot, Simulador):
            self.robot.escena.poner_cubos([n for n, v in self.vars_cubo.items()
                                           if v.get() and n in C.PUNTOS_RECOGIDA])
            self.deteccion = {}
            self.tree_det.delete(*self.tree_det.get_children())
            self._dirty = True

    def _iniciar_rutina(self, n):
        if self.robot is None:
            self.var_estado.set("Conecta el brazo (o la simulación) antes de iniciar la rutina.")
            return
        if self._rutina_activa():
            self._log("Ya hay una rutina en curso.")
            return
        self.deteccion = {}
        self.tree_det.delete(*self.tree_det.get_children())
        self.var_prog.set(0)
        self.rastro.clear()
        self._log(f"Iniciando rutina {n}: {NOMBRES_RUTINA.get(n, '?')}")
        self.hilo_rutina = threading.Thread(target=self.rutina.ejecutar, args=(n,), daemon=True)
        self.hilo_rutina.start()

    def _detener_rutina(self):
        if self.rutina and self._rutina_activa():
            self.rutina.parar.set()

    # ---------------- sliders
    def _on_var_q(self, *_):
        if self._sync:
            return
        q = list(self.q)
        for i, v in enumerate(self.vars_q):
            try:
                q[i] = min(max(float(v.get()), LIMITES[i][0]), LIMITES[i][1])
            except (tk.TclError, ValueError):
                return
        if self._rutina_activa():
            self._fijar_sliders(self.q)
            return
        self._ultimo_toque = time.time()
        self.q = q
        self._registrar()
        self._dirty = True
        if self.robot is not None and self.var_enviar.get():
            if self._envio_pend:
                self.after_cancel(self._envio_pend)
            self._envio_pend = self.after(120, self._enviar_q)

    def _enviar_q(self):
        self._envio_pend = None
        p = dh.angulos_a_porcentajes(self.q)
        if p is None or self.robot is None:
            return
        pinza = self.robot.pinza_objetivo
        self._en_hilo(self.robot.mover_todos, p + [pinza])

    def _fijar_sliders(self, q):
        self._sync = True
        try:
            for v, qi in zip(self.vars_q, q):
                v.set(round(float(qi), 1))
        finally:
            self._sync = False

    # ---------------- inversa
    def _resolver_ik(self, mover):
        try:
            x, y, z, phi = (float(v.get().replace(",", ".")) for v in self.vars_xyz)
        except ValueError:
            self.var_ik.set("Escribe números.")
            return
        q = cinematica_inversa(x, y, z, phi, self.q)
        usado = phi
        if q is None and self.var_phi_auto.get():
            q, usado = inversa_auto(x, y, z, phi, self.q)
        if q is None:
            self.var_ik.set("Punto fuera de alcance\n(o fuera de los límites de los servos).")
            return
        p = dh.angulos_a_porcentajes(q)
        txt = [f"φ usada = {usado:.1f}°"] + [
            f"q{i + 1} = {q[i]:7.2f}°   servo {i + 1}: {p[i]:6.2f} %" for i in range(5)]
        self.var_ik.set("\n".join(txt))
        if mover:
            if self.robot is None or self._rutina_activa():
                self.var_estado.set("Sin robot conectado: solo se muestra la pose en 3D.")
                self.q = q
                self._fijar_sliders(q)
                self._dirty = True
                return
            self._ultimo_toque = time.time()
            self._en_hilo(self.robot.mover_todos, p + [self.robot.pinza_objetivo])

    def _xyz_actual(self):
        x, y, z = dh.cinematica_directa(self.q)
        for var, v in zip(self.vars_xyz, (x, y, z, dh.inclinacion_pinza(self.q))):
            var.set(f"{v:.1f}")

    # ================================================================== dibujo
    def _marcar(self):
        self._dirty = True

    def _registrar(self):
        self.historial.append((time.time() - self.t0, *self.q))
        if len(self.historial) > 3000:
            del self.historial[:500]
        if self.var_rastro.get():
            T5 = dh.matrices(self.q)[1][5]
            self.rastro.append([T5[0][3], T5[1][3], T5[2][3]])
            if len(self.rastro) > 3000:
                del self.rastro[:500]

    def _borrar_rastro(self):
        self.rastro.clear()
        self._dirty = True

    def _vista(self, el, az):
        self.ax.view_init(el, az)
        self.canvas3d.draw_idle()

    def _resaltados(self):
        sel = self.var_mat.get()
        i = int(sel[1])
        return {i - 1, i} if sel[0] == "A" else {0, i}

    def actualizar(self):
        self.A, self.T = dh.matrices(self.q)
        # Primero los números (tabla DH, matrices, TCP): no dependen del dibujo 3D
        p = dh.angulos_a_porcentajes(self.q)
        for i, v in enumerate(self.vars_pct):
            v.set("" if p is None else f"servo {i + 1}: {p[i]:5.1f} %")
        self._actualizar_tabla_dh()
        self._actualizar_matrices()
        self._actualizar_tcp()
        self._publicar_xyz()
        try:
            self._actualizar_3d()
            if self.nb_centro.index("current") == 1:
                self._graficar_historial()
        except Exception as e:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            self.var_estado.set(f"Error en el dibujo 3D (los valores siguen bien): {e}")

    def _actualizar_3d(self):
        sensor = None
        cubos = []
        if self.var_ver_haz.get():
            o, d = pose_sensor(self.q)
            dist = self.dist_sensor
            if isinstance(self.robot, Simulador):
                dist = self.robot.escena.rayo(o, d)
            sensor = (o, d, dist)
        if isinstance(self.robot, Simulador):
            cubos = self.robot.escena.centros(dh.cinematica_directa(self.q))
        self.escena.actualizar(
            self.T, pinza_pct=self.pcts[5], resaltados=self._resaltados(),
            ver_ejes=self.var_ver_ejes.get(), ver_etiq=self.var_ver_etiq.get(),
            esc=self.var_esc.get(), sensor=sensor, cubos=cubos, deteccion=self.deteccion,
            rastro=self.rastro if self.var_rastro.get() else None)
        self.txt_q.set_text("q = (" + ", ".join(f"{v:6.1f}" for v in self.q) + ")°")
        self.canvas3d.draw_idle()

    def _publicar_xyz(self):
        """Envía X, Y, Z del TCP (calculados con DH) a la app (V13-V15), máx. 2 veces/s."""
        if self.robot is None or self.robot.simulado:
            return
        ahora = time.time()
        if ahora - self._t_xyz < 0.5:
            self._xyz_pend = True
            return
        self._t_xyz, self._xyz_pend = ahora, False
        x, y, z = dh.cinematica_directa(self.q)
        self._en_hilo(self.robot.publicar_xyz, x, y, z)

    def _actualizar_tabla_dh(self):
        for i, (th, d, a, al) in enumerate(dh.parametros_dh(self.q), start=1):
            self.tree_dh.item(str(i), values=(i, f"{EXPR_THETA[i - 1]} = {th:.1f}°",
                                             f"{d:.2f}", f"{a:.2f}", f"{al:.0f}°"))

    def _poner_matriz(self, celdas, M):
        for r in range(4):
            for c in range(4):
                celdas[r][c].configure(text=_fmt(M[r][c]))

    def _actualizar_matrices(self):
        sel = self.var_mat.get()
        i = int(sel[1])
        M = self.A[i - 1] if sel[0] == "A" else self.T[i]
        self._poner_matriz(self.celdas_sel, M)
        self._poner_matriz(self.celdas_T, self.T[5])
        if sel[0] == "A":
            th, d, a, al = dh.parametros_dh(self.q)[i - 1]
            self.var_formula.set(
                f"A{SUB[i]} = Rz(θ{SUB[i]})·Tz(d{SUB[i]})·Tx(a{SUB[i]})·Rx(α{SUB[i]})\n"
                f"θ{SUB[i]} = {EXPR_THETA[i - 1]} = {th:.1f}°   d = {d:.2f}   a = {a:.2f}   "
                f"α = {al:.0f}°\nDescribe el sistema {{{i}}} visto desde {{{i - 1}}}.")
        else:
            prod = "·".join(f"A{SUB[k]}" for k in range(1, i + 1))
            self.var_formula.set(
                f"T{SUB[0]}{SUP[i]} = {prod}\n"
                f"Describe el sistema {{{i}}} visto desde la base {{0}}.\n"
                f"Columna 4 = posición del origen O{SUB[i]} [cm].")

    def _actualizar_tcp(self):
        T5 = np.array(self.T[5])
        p, x5 = T5[:3, 3], T5[:3, 0]
        phi = dh.inclinacion_pinza(self.q)
        self.var_tcp.set(
            f"x = {p[0]:7.2f} cm   φ = q2+q3+q5 = {phi:6.1f}°\n"
            f"y = {p[1]:7.2f} cm   ψ = q1       = {self.q[0]:6.1f}°\n"
            f"z = {p[2]:7.2f} cm   r = {math.hypot(p[0], p[1]):6.2f} cm\n"
            f"dirección pinza x₅ = ({x5[0]: .3f}, {x5[1]: .3f}, {x5[2]: .3f})")

    def _graficar_historial(self):
        axq, axz = self.ax_q, self.ax_z
        axq.cla()
        axz.cla()
        if len(self.historial) > 1:
            H = np.array(self.historial[-1500:])
            t = H[:, 0]
            for j in range(5):
                axq.plot(t, H[:, 1 + j], color=COLORES_Q[j], lw=1.6, label=f"q{j + 1}")
            z = [dh.cinematica_directa(fila[1:])[2] for fila in H]
            axz.plot(t, z, color="#e53e3e", lw=1.6)
            axq.legend(loc="upper left", ncol=5, fontsize=8)
        axq.set_ylabel("ángulo [°]")
        axz.set_ylabel("z del TCP [cm]")
        axz.set_xlabel("t [s]")
        for a in (axq, axz):
            a.grid(True, color="#edf2f7")
        self.fig_q.tight_layout()
        self.canvas_q.draw_idle()

    # ---------------- selección de matriz
    def _on_sel_matriz(self):
        sel = self.var_mat.get()
        actual = self.tree_dh.selection()
        if sel[0] == "A":
            if actual != (sel[1],):
                self.tree_dh.selection_set(sel[1])
        elif actual:
            self.tree_dh.selection_remove(*actual)
        self._dirty = True

    def _on_sel_dh(self, _e):
        sel = self.tree_dh.selection()
        if sel and self.var_mat.get() != f"A{sel[0]}":
            self.var_mat.set(f"A{sel[0]}")
            self._dirty = True

    def _copiar_matrices(self):
        def bloque(nombre, M):
            filas = "\n".join("  [" + "  ".join(f"{_fmt(v):>8}" for v in fila) + " ]"
                              for fila in M)
            return f"{nombre} =\n{filas}\n"
        partes = [f"q = ({', '.join(f'{v:.2f}' for v in self.q)}) grados\n"]
        partes += [bloque(f"A{i}", self.A[i - 1]) for i in range(1, 6)]
        partes += [bloque(f"T0_{i}", self.T[i]) for i in range(1, 6)]
        self.clipboard_clear()
        self.clipboard_append("\n".join(partes))
        self.var_estado.set("Matrices A1..A5 y T0_1..T0_5 copiadas al portapapeles.")

    def _cerrar(self):
        self._vivo = False
        self._detener_rutina()
        if self.robot is not None:
            self.robot.cerrar()
        self.quit()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
