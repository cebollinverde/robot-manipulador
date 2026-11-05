"""
Cinematica inversa geometrica (desacople de muneca).

cinematica_inversa() es la funcion original del grupo (phi fijo).
inversa_auto() la usa probando varias inclinaciones phi, porque con los
limites de los servos la pinza no siempre puede llegar con -25 grados.
"""
from math import atan2, cos, sin, radians, degrees, sqrt, hypot

from . import dh
from .dh import cinematica_directa, porcentajes_a_angulos


# Calcula los angulos necesarios para llegar a un punto.
# actuales son los 5 angulos que tiene el brazo en ese momento.
def cinematica_inversa(x, y, z, phi=-25, actuales=None):

    # Medidas actuales del DH (se pueden cambiar desde la interfaz)
    H, R0, L1, L2, LG = dh.H, dh.R0, dh.L1, dh.L2, dh.LG

    # Calculamos el giro de la base y la posicion de la muneca.
    q1 = degrees(atan2(y, x))
    r = hypot(x, y) - R0 - LG * cos(radians(phi))
    h = z - H - LG * sin(radians(phi))

    # Ley de cosenos para encontrar el codo.
    D = (r**2 + h**2 - L1**2 - L2**2) / (2 * L1 * L2)

    # Si el punto queda muy lejos, no podemos llegar
    if D < -1 or D > 1:
        return None

    # Sacamos los angulos permitidos con los limites del DH.
    minimo = porcentajes_a_angulos([0] * 5)
    maximo = porcentajes_a_angulos([100] * 5)
    opciones = []

    # Un punto puede alcanzarse con el codo en dos posiciones.
    for lado in (-1, 1):

        q3 = degrees(atan2(lado * sqrt(1 - D**2), D))
        q2 = degrees(atan2(h, r) - atan2(
            L2 * sin(radians(q3)),
            L1 + L2 * cos(radians(q3))
        ))

        q4 = 0
        q5 = phi - q2 - q3

        angulos = [q1, q2, q3, q4, q5]

        # Revisamos que los angulos esten dentro de los limites.
        permitido = all(
            min(minimo[i], maximo[i]) <= angulos[i] <=
            max(minimo[i], maximo[i])
            for i in range(5)
        )

        if permitido:
            # Comprobamos el resultado con la cinematica directa.
            px, py, pz = cinematica_directa(angulos)
            error = sqrt((px-x)**2 + (py-y)**2 + (pz-z)**2)

            if error < 0.05:
                opciones.append(angulos)

    # Si no encontramos una solucion, no enviamos nada al brazo
    if not opciones:
        return None

    # Elegimos la posicion que requiera menos movimiento.
    if actuales is not None:
        return min(
            opciones,
            key=lambda q: sum(
                (q[i] - actuales[i])**2 for i in range(5)
            )
        )

    return opciones[0]


def candidatos_phi(preferido=-45, minimo=-120, maximo=10, paso=2.5):
    """Inclinaciones a probar, ordenadas desde la mas cercana a la preferida."""
    n = int(round((maximo - minimo) / paso))
    valores = [minimo + k * paso for k in range(n + 1)]
    return sorted(valores, key=lambda p: (abs(p - preferido), p))


def inversa_auto(x, y, z, phi_preferido=-45, actuales=None):
    """Busca la inclinacion phi alcanzable mas parecida a la preferida.

    Devuelve (angulos, phi) o (None, None) si el punto no se alcanza.
    """
    for phi in candidatos_phi(phi_preferido):
        q = cinematica_inversa(x, y, z, phi, actuales)
        if q is not None:
            return q, phi
    return None, None
