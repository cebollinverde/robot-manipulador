"""
Rutina automatica: detectar -> acercarse -> tomar -> trasladar -> apilar/ordenar.

Rutinas (mismo numero que el selector V8 de Blynk):
  1 = Apilar todos los cubos detectados en PUNTO_APILADO
  2 = Ordenar los cubos detectados en FILA_ORDEN
  3 = Solo escanear los puntos (prueba del sensor)
  4 = Repetir el apilado A -> B de dos cajas enseñado con el brazo real

Funciona igual con el ESP32 real o con el Simulador (misma interfaz).
Se ejecuta en un hilo aparte; la interfaz recibe avisos por el callback 'avisar'.
"""
import json
import os
import threading
import time
from math import atan2, cos, sin, hypot

import numpy as np

from . import config as C
from . import dh
from .comunicacion import ErrorRobot
from .geometria import interpolar, pose_escaneo, pose_sensor, punto_impacto
from .inversa import inversa_auto

NOMBRES_RUTINA = {
    1: "Apilar",
    2: "Ordenar en fila",
    3: "Solo escanear",
    4: "Apilado enseñado (2 cajas)",
}

ARCHIVO_APILADO_ENSENADO = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "apilado_ensenado.json"
)


class RutinaDetenida(Exception):
    pass


class Rutina:
    def __init__(self, robot, avisar=None):
        self.robot = robot
        self.avisar = avisar or (lambda tipo, *datos: print(tipo, *datos))
        self.parar = threading.Event()
        self.q = None            # ultimos angulos enviados (o leidos)
        self.deteccion = {}      # nombre -> dict con el resultado del escaneo

    # ------------------------------------------------------------------ bajo nivel
    def _log(self, txt):
        self.avisar("log", txt)

    def _revisar(self):
        if self.parar.is_set():
            raise RutinaDetenida()

    def _reportar(self, txt):
        self._log(txt)
        try:
            self.robot.reportar(txt)
        except ErrorRobot:
            pass

    def _leer_angulos(self):
        _, p = self.robot.estado()
        self.q = dh.porcentajes_a_angulos(p)
        return self.q

    def esperar(self, objetivo=None, tol=0.0, timeout=25.0):
        """Espera a que el brazo llegue (o quede a 'tol' % del objetivo)."""
        fin = time.time() + timeout
        while time.time() < fin:
            self._revisar()
            moviendo, p = self.robot.estado()
            if not moviendo:
                return
            if tol > 0 and objetivo is not None and \
                    max(abs(p[i] - objetivo[i]) for i in range(5)) <= tol:
                return
            time.sleep(0.03)
        raise ErrorRobot("El brazo no llego a la posicion (timeout)")

    def ir_angulos(self, q, tol=0.0):
        self._revisar()
        p = dh.angulos_a_porcentajes(q)
        if p is None:
            raise ErrorRobot("Angulos fuera de los limites")
        objetivo = p + [self.robot.pinza_objetivo]
        self.robot.mover_todos(objetivo)
        self.q = list(q)
        self.esperar(objetivo, tol)

    def _planificar(self, puntos, phi_final):
        """Resuelve la IK de cada punto del tramo. La inclinacion pasa
        suavemente desde la actual hasta la preferida en el destino; en cada
        punto se usa la alcanzable mas parecida."""
        phi0 = dh.inclinacion_pinza(self.q)
        qs, ref = [], self.q
        n = len(puntos)
        for k, p in enumerate(puntos, start=1):
            preferido = phi0 + (phi_final - phi0) * k / n
            q, _ = inversa_auto(*p, phi_preferido=preferido, actuales=ref)
            if q is None:
                return None
            qs.append(q)
            ref = q
        return qs

    def trayecto(self, destino, phi=C.PHI_AGARRE):
        """Mueve el TCP hasta 'destino' por una trayectoria cartesiana suave."""
        origen = dh.cinematica_directa(self.q)
        qs = self._planificar(interpolar(origen, destino), phi)
        if qs is None:
            raise ErrorRobot(f"Trayectoria fuera de alcance hacia {np.round(destino, 1)}")
        for k, q in enumerate(qs):
            ultimo = k == len(qs) - 1
            self.ir_angulos(q, tol=0.0 if ultimo else C.TOL_INTERMEDIA)
        try:
            self.robot.publicar_xyz(*destino)
        except ErrorRobot:
            pass

    def ir_seguro(self, x, y, z, phi=C.PHI_AGARRE):
        """Sube a la altura segura, se traslada en arco y baja en linea recta.
        En altura el radio se mantiene dentro de la zona alcanzable. Si el
        camino cartesiano no es posible, va por movimiento articular via Home."""
        zs = C.Z_MESA + C.ALTURA_SEGURA
        try:
            self.subir(phi)
            r_fin = min(max(hypot(x, y), C.RADIO_TRASLADO[0]), C.RADIO_TRASLADO[1])
            a_fin = atan2(y, x)
            self.trayecto((r_fin * cos(a_fin), r_fin * sin(a_fin), zs), phi)
            self.trayecto((x, y, z), phi)
        except ErrorRobot as e:
            if "fuera de alcance" not in str(e):
                raise
            q_fin, _ = inversa_auto(x, y, z, phi_preferido=phi, actuales=self.q)
            if q_fin is None:
                raise
            self._log("  (camino recto no posible: movimiento articular por Home)")
            self.ir_articular(dh.porcentajes_a_angulos(dh.HOME))
            self.ir_articular(q_fin)

    def ir_articular(self, q_fin, pasos=12):
        """Interpola en el espacio articular (todas las articulaciones a la vez)."""
        q0 = list(self.q)
        for k in range(1, pasos + 1):
            q = [a + (b - a) * k / pasos for a, b in zip(q0, q_fin)]
            self.ir_angulos(q, tol=0.0 if k == pasos else C.TOL_INTERMEDIA)

    def subir(self, phi=C.PHI_AGARRE):
        """Lleva el TCP a la altura segura (corrigiendo el radio si hace falta)."""
        zs = C.Z_MESA + C.ALTURA_SEGURA
        px, py, pz = dh.cinematica_directa(self.q)
        r = hypot(px, py)
        r_ok = min(max(r, C.RADIO_TRASLADO[0]), C.RADIO_TRASLADO[1])
        if pz < zs - 0.5 or abs(r - r_ok) > 0.2:
            a = atan2(py, px)
            self.trayecto((r_ok * cos(a), r_ok * sin(a), zs), phi)

    def pinza(self, abrir):
        self._revisar()
        self.robot.pinza(abrir)
        self.esperar()
        time.sleep(C.ESPERA_PINZA)

    # ------------------------------------------------------------------ sensor
    def medir(self):
        valores = []
        for _ in range(C.LECTURAS_SENSOR):
            self._revisar()
            d = self.robot.distancia()
            if d is not None:
                valores.append(d)
        return float(np.median(valores)) if valores else None

    def _pose(self, nombre):
        """(x, y, z_centro, phi, q) de un punto. q solo existe si fue ENSENADO."""
        p = C.POSES.get(nombre)
        if p:
            x, y, z = p["xyz"]
            return x, y, z, p["phi"], list(p["q"])
        if nombre == "Pila":
            x, y = C.PUNTO_APILADO
        else:
            x, y = C.PUNTOS_RECOGIDA[nombre]
        return x, y, C.Z_MESA + C.CUBO / 2, C.PHI_AGARRE, None

    def escanear(self, nombre):
        """Mira el punto con el sensor. Devuelve dict con presente, altura, etc."""
        x, y, zc, phi_e, q_e = self._pose(nombre)
        z_mesa = zc - C.CUBO / 2                       # altura de la mesa en ESE punto
        self._reportar(f"ESCANEANDO {nombre}")
        q, phi, _ = pose_escaneo(x, y, self.q, z_mesa, phi_e)
        if q is None:
            raise ErrorRobot(f"No hay pose de escaneo para {nombre}")
        tcp = dh.cinematica_directa(q)
        self.ir_seguro(*tcp, phi=phi)
        self.ir_angulos(q)                       # corrige el ultimo milimetro con la phi exacta
        time.sleep(0.3)                          # que se calme la vibracion
        d = self.medir()

        o, v = pose_sensor(q)                    # distancia a la que DEBERIA estar la mesa
        esperado = (z_mesa - o[2]) / v[2] if v[2] < -0.05 else float("inf")
        res = {"nombre": nombre, "x": x, "y": y, "medido": d, "esperado": esperado,
               "presente": False, "z_mesa": z_mesa, "z_tope": z_mesa, "cubos": 0,
               "phi": phi_e, "q": q_e, "zc": zc}
        if d is not None and C.SENSOR_MIN <= d <= C.SENSOR_MAX \
                and d < esperado - C.MARGEN_DETECCION:
            z_tope = float(punto_impacto(q, d)[2])
            res.update(presente=True, z_tope=z_tope,
                       cubos=max(1, round((z_tope - z_mesa) / C.CUBO)))
        txt_d = "sin eco" if d is None else f"{d:.1f} cm"
        self._log(f"  {nombre}: medido {txt_d}, mesa a {esperado:.1f} cm -> "
                  f"{'HAY OBJETO' if res['presente'] else 'vacio'}")
        self.deteccion[nombre] = res
        self.avisar("deteccion", res)
        return res

    # ------------------------------------------------------------------ tomar / dejar
    def _bajar_a(self, x, y, z, phi, q_exacto=None):
        """Baja en linea recta hasta (x,y,z). Si hay postura ensenada, termina en ella."""
        self.trayecto((x, y, z), phi)
        if q_exacto is not None:
            self.ir_angulos(q_exacto)

    def tomar(self, res):
        x, y, phi = res["x"], res["y"], res["phi"]
        z = res["z_tope"] - C.CUBO / 2                   # centro del objeto de mas arriba
        z = max(z, res["z_mesa"] + C.CUBO / 2)
        # El objeto de abajo se toma con la postura EXACTA que se enseno
        exacto = res["q"] if (res["q"] is not None and abs(z - res["zc"]) < C.CUBO / 2) else None
        if exacto is not None:
            z = res["zc"]
        self._reportar(f"TOMANDO {res['nombre']}")
        self.pinza(True)
        self.ir_seguro(x, y, z + C.ALTURA_APROX, phi)
        self._bajar_a(x, y, z, phi, exacto)
        self.pinza(False)
        self.trayecto((x, y, z + C.ALTURA_APROX), phi)   # subida recta
        self.subir(phi)

    def dejar(self, destino, nivel, texto):
        """Deja el objeto en destino=(x, y, z_centro, phi, q) en el nivel indicado
        (0 = sobre la mesa, 1 = encima de otro, ...)."""
        x, y, zc, phi, q_e = destino
        z = zc + C.CUBO * nivel + (C.HOLGURA_DEJAR if nivel > 0 else 0.0)
        exacto = q_e if nivel == 0 else None
        self._reportar(texto)
        self.ir_seguro(x, y, z + C.ALTURA_APROX, phi)
        self._bajar_a(x, y, z, phi, exacto)
        self.pinza(True)
        self.trayecto((x, y, z + C.ALTURA_APROX), phi)
        self.subir(phi)

    def a_home(self):
        self.subir()
        self.ir_angulos(dh.porcentajes_a_angulos(dh.HOME))

    # ------------------------------------------------------------------ rutinas
    def escanear_todos(self):
        encontrados = []
        for nombre in C.PUNTOS_RECOGIDA:
            x, y, zc, phi, _ = self._pose(nombre)
            if pose_escaneo(x, y, self.q, zc - C.CUBO / 2, phi)[0] is None:
                self._log(f"  {nombre}: el sensor no alcanza a mirarlo. Se salta.")
                continue
            res = self.escanear(nombre)
            if res["presente"]:
                encontrados.append(res)
        return encontrados

    def _rutina_apilar(self):
        encontrados = self.escanear_todos()
        if not encontrados:
            self._log("No se detectaron cubos.")
            return
        # La pila se asume vacia al empezar (el punto final debe estar despejado)
        destino = self._pose("Pila")
        nivel = 0
        total = len(encontrados)
        for k, res in enumerate(encontrados):
            self.avisar("progreso", k / total)
            # Si un punto tenia una pila, se toman sus cubos de arriba hacia abajo
            for _ in range(res["cubos"]):
                self.tomar(res)
                self.dejar(destino, nivel, f"APILANDO {nivel + 1}")
                nivel += 1
                res["z_tope"] -= C.CUBO
        self.avisar("progreso", 1.0)

    def _rutina_ordenar(self):
        encontrados = self.escanear_todos()
        if not encontrados:
            self._log("No se detectaron cubos.")
            return
        libres = list(C.FILA_ORDEN)
        total = sum(r["cubos"] for r in encontrados)
        hechos = 0
        for res in encontrados:
            for _ in range(res["cubos"]):
                if not libres:
                    self._log("No quedan posiciones libres en la fila.")
                    return
                self.avisar("progreso", hechos / total)
                self.tomar(res)
                x, y = libres.pop(0)
                self.dejar((x, y, C.Z_MESA + C.CUBO / 2, C.PHI_AGARRE, None), 0,
                           f"ORDENANDO {hechos + 1}")
                hechos += 1
                res["z_tope"] -= C.CUBO
        self.avisar("progreso", 1.0)

    def _cargar_apilado_ensenado(self):
        try:
            with open(ARCHIVO_APILADO_ENSENADO, encoding="utf-8") as f:
                datos = json.load(f)
        except (OSError, ValueError) as e:
            raise ErrorRobot(f"No se pudo cargar la secuencia enseñada: {e}") from e
        if datos.get("numero_cajas") != 2 or "traslado_a_b" not in datos:
            raise ErrorRobot("La secuencia enseñada no contiene dos cajas A-B")
        return datos

    def _ir_ensenado(self, movimientos, nombre, pasos=6):
        """Reproduce una postura validada, guardada como porcentajes normalizados."""
        clave = nombre + "_porcentajes"
        porcentajes = movimientos.get(clave)
        if not isinstance(porcentajes, list) or len(porcentajes) != 5:
            raise ErrorRobot(f"Falta el punto enseñado {clave}")
        q = dh.porcentajes_a_angulos(porcentajes)
        self.ir_articular(q, pasos=pasos)
        return q

    def _esperar_caja_en_a(self, datos, movimientos, numero):
        """Espera dos mediciones consecutivas y da tiempo para retirar las manos."""
        umbral = float(movimientos.get("umbral_presencia_a_cm", 10.5))
        timeout = float(datos.get("espera_caja_s", 120.0))
        seguridad = float(datos.get("espera_seguridad_s", 2.0))
        self._reportar(f"COLOQUE CAJA {numero} EN A")
        limite = time.time() + timeout
        confirmaciones = 0
        ultima = None
        while time.time() < limite:
            self._revisar()
            ultima = self.medir()
            if ultima is not None and C.SENSOR_MIN <= ultima <= umbral:
                confirmaciones += 1
                if confirmaciones >= 2:
                    self._reportar(f"CAJA {numero} DETECTADA - RETIRE MANOS")
                    fin_seguridad = time.time() + seguridad
                    while time.time() < fin_seguridad:
                        self._revisar()
                        time.sleep(0.05)
                    return ultima
            else:
                confirmaciones = 0
            time.sleep(0.2)
        txt = "sin eco" if ultima is None else f"{ultima:.1f} cm"
        raise ErrorRobot(f"No se detectó la caja {numero} en A ({txt})")

    def _rutina_apilado_ensenado(self):
        """Reproduce la trayectoria A-B validada físicamente para exactamente dos cajas."""
        datos = self._cargar_apilado_ensenado()
        m = datos["traslado_a_b"]
        self.robot.velocidad(int(datos.get("velocidad", 30)))

        # Primero se alcanza la postura alta más cercana. Esto evita barrer la mesa
        # si la rutina comienza cerca de A o cerca de la pila B.
        qa = dh.porcentajes_a_angulos(m["a_alta_porcentajes"])
        qb = dh.porcentajes_a_angulos(m["b_alta_porcentajes"])
        distancia_a = sum(abs(a - b) for a, b in zip(self.q, qa))
        distancia_b = sum(abs(a - b) for a, b in zip(self.q, qb))
        empieza_en_b = distancia_b < distancia_a
        q_alta = qb if empieza_en_b else qa
        self.ir_articular(q_alta, pasos=8)
        if empieza_en_b:
            self.ir_articular(qa, pasos=10)

        for numero in (1, 2):
            self.avisar("progreso", (numero - 1) / 2)
            self._ir_ensenado(m, "a_aproximacion_sensor", pasos=6)
            distancia = self._esperar_caja_en_a(datos, m, numero)
            self._log(f"  Caja {numero} confirmada a {distancia:.2f} cm")

            self.pinza(True)
            self._ir_ensenado(m, "a_aproximacion_agarre", pasos=3)
            self._ir_ensenado(m, "a_agarre", pasos=6)
            self.pinza(False)
            self._reportar(f"TRASLADANDO CAJA {numero}")
            self._ir_ensenado(m, "a_elevacion_1", pasos=3)
            self._ir_ensenado(m, "a_elevacion_2", pasos=3)
            self._ir_ensenado(m, "a_elevacion_3", pasos=3)
            self._ir_ensenado(m, "a_alta", pasos=4)
            self._ir_ensenado(m, "b_alta", pasos=10)

            if numero == 1:
                self._ir_ensenado(m, "b_aproximacion", pasos=5)
                self._ir_ensenado(m, "b_deposito", pasos=6)
                self.pinza(True)
                self._ir_ensenado(m, "b_aproximacion", pasos=5)
                self._ir_ensenado(m, "b_alta", pasos=5)
                self._ir_ensenado(m, "a_alta", pasos=10)
            else:
                self._ir_ensenado(m, "b_segundo_nivel_aproximacion", pasos=5)
                self._ir_ensenado(m, "b_segundo_nivel_precolocacion", pasos=5)
                # Este último punto incorpora los 5 mm alejándose de la base que
                # dejaron ambas cajas alineadas en el ensayo físico.
                self._ir_ensenado(m, "b_segundo_nivel_suelta", pasos=3)
                self.pinza(True)
                self._ir_ensenado(m, "b_segundo_nivel_aproximacion", pasos=5)
                self._ir_ensenado(m, "b_alta", pasos=5)

        self.avisar("progreso", 1.0)
        self._reportar("DOS CAJAS APILADAS")

    def ejecutar(self, numero):
        """Corre la rutina completa. Devuelve 'TERMINADA', 'DETENIDA' o 'ERROR'."""
        self.parar.clear()
        self.deteccion = {}
        final = "TERMINADA"
        try:
            self.robot.velocidad(C.VELOCIDAD)
            self._leer_angulos()
            self._reportar(f"EJECUTANDO {numero}")
            if numero == 1:
                self._rutina_apilar()
            elif numero == 2:
                self._rutina_ordenar()
            elif numero == 3:
                self.escanear_todos()
            elif numero == 4:
                self._rutina_apilado_ensenado()
            else:
                raise ErrorRobot(f"Rutina {numero} no existe")
            if numero != 4:
                self.a_home()
        except RutinaDetenida:
            final = "DETENIDA"
            self._log("Rutina detenida.")
        except ErrorRobot as e:
            final = "ERROR"
            self._log(f"Error: {e}")
        try:
            self.robot.reportar(final)
        except ErrorRobot:
            pass
        self.avisar("fin", final)
        return final
