"""
Comunicacion con el ESP32 por USB (serial) y un simulador con la MISMA interfaz.

Protocolo (una linea por mensaje, 115200 baudios):
  PC -> ESP32 : PING, STATE, ALL,p1..p6, SET,n,p, OPEN, CLOSE, HOME, SPEED,v,
                DIST, STOP, RESUME, XYZ,x,y,z, REPORT,texto
  ESP32 -> PC : LISTO,..., ERROR,..., STATE,IDLE|MOVING,p1..p6, DIST,d|NA
  Eventos     : RUTINA,n (boton de Blynk), STOP_REQUEST (interruptor de Blynk), READY

Los porcentajes que viajan son SIEMPRE los normalizados 0-100% (los mismos de la app).
"""
import queue
import random
import threading
import time

import numpy as np

from . import dh
from .geometria import Escena, pose_sensor

EVENTOS = ("RUTINA,", "STOP_REQUEST", "READY", "ERROR,PCA")


class ErrorRobot(Exception):
    pass


def puertos_disponibles():
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [p.device for p in list_ports.comports()]


# ======================================================================
# ESP32 real
# ======================================================================
class ESP32:
    simulado = False

    def __init__(self, puerto, baudios=115200):
        try:
            import serial
        except ImportError as e:
            raise ErrorRobot("Falta pyserial:  pip install pyserial") from e
        self.ser = serial.Serial(puerto, baudios, timeout=0.05)
        self.respuestas = queue.Queue()
        self.eventos = queue.Queue()
        self._lock = threading.Lock()
        self._vivo = True
        self.pinza_objetivo = dh.HOME[5]
        threading.Thread(target=self._leer, daemon=True).start()

        # Abrir el puerto reinicia el ESP32: esperamos READY o respondemos a PING
        limite = time.time() + 6
        while time.time() < limite:
            try:
                self.comando("PING", ("LISTO,PONG",), timeout=0.8)
                return
            except ErrorRobot:
                pass
        self.cerrar()
        raise ErrorRobot(f"El ESP32 no responde en {puerto}")

    def _leer(self):
        buf = b""
        while self._vivo:
            try:
                buf += self.ser.read(256)
            except Exception:          # puerto desconectado
                self.eventos.put("DESCONECTADO")
                return
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                txt = linea.decode("utf-8", "ignore").strip()
                if not txt:
                    continue
                if txt.startswith(EVENTOS):
                    self.eventos.put(txt)
                else:
                    self.respuestas.put(txt)

    def comando(self, texto, esperado, timeout=3.0):
        with self._lock:
            while not self.respuestas.empty():          # restos de un timeout anterior
                self.respuestas.get_nowait()
            self.ser.write((texto + "\n").encode())
            fin = time.time() + timeout
            while True:
                resto = fin - time.time()
                if resto <= 0:
                    raise ErrorRobot(f"Sin respuesta a {texto.split(',')[0]}")
                try:
                    r = self.respuestas.get(timeout=resto)
                except queue.Empty:
                    continue
                if r.startswith(esperado):
                    return r
                if r.startswith("ERROR"):
                    raise ErrorRobot(r)

    # ---- comandos -------------------------------------------------------
    def estado(self):
        partes = self.comando("STATE", ("STATE,",)).split(",")
        return partes[1] == "MOVING", [float(v) for v in partes[2:8]]

    def mover_todos(self, pcts6):
        self.comando("ALL," + ",".join(f"{p:.2f}" for p in pcts6), ("LISTO,ALL",))
        self.pinza_objetivo = pcts6[5]

    def pinza(self, abrir):
        r = self.comando("OPEN" if abrir else "CLOSE", ("LISTO,OPEN", "LISTO,CLOSE")).split(",")
        # El ESP32 responde con el % donde quedo la pinza (se ajusta desde la app, V16-V18)
        if len(r) > 2:
            self.pinza_objetivo = float(r[2])
        else:
            self.pinza_objetivo = dh.PINZA_ABIERTA if abrir else dh.PINZA_CERRADA

    def distancia(self):
        r = self.comando("DIST", ("DIST,",), timeout=4.0).split(",")[1]
        return None if r == "NA" else float(r)

    def velocidad(self, v):
        self.comando(f"SPEED,{int(v)}", ("LISTO,SPEED",))

    def home(self):
        self.comando("HOME", ("LISTO,HOME",))

    def stop(self):
        self.comando("STOP", ("LISTO,STOP",))

    def resume(self):
        self.comando("RESUME", ("LISTO,RESUME",))

    def publicar_xyz(self, x, y, z):
        self.comando(f"XYZ,{x:.2f},{y:.2f},{z:.2f}", ("LISTO,XYZ",))

    def reportar(self, texto):
        self.comando("REPORT," + texto.replace(",", " ")[:30], ("LISTO,REPORT",))

    def cerrar(self):
        self._vivo = False
        try:
            self.ser.close()
        except Exception:
            pass


# ======================================================================
# Simulador (mismo comportamiento que mover() del ESP32)
# ======================================================================
class Simulador:
    simulado = True

    def __init__(self):
        self.escena = Escena()
        self.eventos = queue.Queue()
        self._lock = threading.Lock()
        self.actual = [50.0] * 6          # porcentajes ORIGINALES, como el ESP32
        self.destino = [50.0] * 6
        self.vel = 50
        self.bloqueado = False
        self.factor_tiempo = 1.0          # >1 acelera la simulacion
        self._t = time.perf_counter()
        self.pinza_objetivo = dh.HOME[5]

    def _avanzar(self):
        ahora = time.perf_counter()
        ticks = int((ahora - self._t) * self.factor_tiempo / 0.02)
        if ticks <= 0:
            return
        self._t += ticks * 0.02 / self.factor_tiempo
        paso = (0.1 + self.vel * 0.012) * ticks
        for i in range(6):
            d = self.destino[i] - self.actual[i]
            self.actual[i] = self.destino[i] if abs(d) <= paso else self.actual[i] + paso * np.sign(d)

    def _normal(self):
        return [dh.orig_a_normal(i, self.actual[i]) for i in range(6)]

    def angulos(self):
        with self._lock:
            self._avanzar()
            return dh.porcentajes_a_angulos(self._normal())

    def tcp(self):
        return np.array(dh.cinematica_directa(self.angulos()))

    def estado(self):
        with self._lock:
            self._avanzar()
            moviendo = any(abs(self.destino[i] - self.actual[i]) > 0.01 for i in range(6))
            return moviendo, self._normal()

    def mover_todos(self, pcts6):
        if self.bloqueado:
            raise ErrorRobot("ERROR,ALL")
        if any(not 0 <= p <= 100 for p in pcts6):
            raise ErrorRobot("ERROR,LIMITE")
        with self._lock:
            self._avanzar()
            self.destino = [dh.normal_a_orig(i, p) for i, p in enumerate(pcts6)]
        self.pinza_objetivo = pcts6[5]

    def pinza(self, abrir):
        if self.bloqueado:
            raise ErrorRobot("ERROR,STOP")
        tcp = self.tcp()
        with self._lock:
            self.destino[5] = dh.PINZA_ABIERTA_ORIG if abrir else dh.PINZA_CERRADA_ORIG
        if abrir:
            self.escena.soltar(tcp)
        else:
            self.escena.tomar(tcp)
        self.pinza_objetivo = dh.PINZA_ABIERTA if abrir else dh.PINZA_CERRADA

    def distancia(self):
        time.sleep(0.15 / self.factor_tiempo)
        q = self.angulos()
        o, d = pose_sensor(q)
        t = self.escena.rayo(o, d, self.tcp())
        if t is None or t < 2 or t > 400:
            return None
        return round(t + random.gauss(0, 0.2), 2)

    def velocidad(self, v):
        self.vel = max(1, min(100, int(v)))

    def home(self):
        with self._lock:
            self._avanzar()
            self.destino = [50.0] * 6

    def stop(self):
        with self._lock:
            self._avanzar()
            self.destino = list(self.actual)
            self.bloqueado = True

    def resume(self):
        self.bloqueado = False

    def publicar_xyz(self, x, y, z):
        pass

    def reportar(self, texto):
        pass

    def cerrar(self):
        pass
