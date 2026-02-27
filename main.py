import time
import cv2

from Vision import BallTracker
from SerialLink import SerialSender

# =========================
# CONFIG
# =========================
PORT = None
BAUD = 115200

# Tolerancias (pixeles)
TOL_X = 25
TOL_Y = 25

# Control
KP_X = 0.006
UT_MAX = 2.0

KP_Y = 0.004
UY_MAX = 0.35

# Velocidades
UY_AVANZAR = 0.20
UY_EXTRA = 0.18
UY_CAPTURA = 0.25

# Tiempos
CAPTURA_TIMEOUT = 4.0
ESPERA_TIRO = 10.0
TIEMPO_PATADA = 1.0

# Arduino PI params
KP_ARD = 20.0
KI_ARD = 5.0

STABLE_FRAMES = 3

# =========================
# HELPERS
# =========================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def build_cmd(Ux, Uy, Ut, patada=0, cilindro=0):
    return f"M,{Ux:.3f},{Uy:.3f},{Ut:.3f},{int(patada)},{int(cilindro)},{KP_ARD:.2f},{KI_ARD:.2f}"

def ut_from_errorx(error_x):
    Ut = KP_X * float(error_x)  # si gira al revés invierte signo aquí
    return clamp(Ut, -UT_MAX, UT_MAX)

def uy_from_errory(error_y):
    Uy = KP_Y * float(-error_y)  # si avanza al revés invierte signo aquí
    return clamp(Uy, -UY_MAX, UY_MAX)

# =========================
# MAQUINA DE ESTADOS
# =========================
BUSCAR   = 0
ALINEAR  = 1
AVANZAR  = 2
CAPTURAR = 3
TIRAR    = 4
PATEAR   = 5

state = BUSCAR

stable_x = 0

t_captura_ini = None
t_tirar_ini = None
t_patear_ini = None

# Sensor pelota (activo LOW: con pelota = 0)
sensor_pelota = 1

# =========================
# INIT
# =========================
tracker = BallTracker(cam_index=0, width=640, height=480, show_windows=True)
sender = SerialSender(port=PORT, baud=BAUD, min_interval=0.03, auto_reconnect=True, verbose=True)

try:
    while True:

        # -------- Vision --------
        x, y, r, found, capture, debug, error_x, error_y = tracker.read()
        if debug is None:
            break

        # -------- Serial Arduino --------
        for line in sender.read_available():
            if line.startswith("P,"):
                try:
                    sensor_pelota = int(line.split(",")[1])
                except:
                    pass

        lost_visual = (not found)

        Ux = 0.0
        Uy = 0.0
        Ut = 0.0
        patada = 0
        cilindro = 0

        now = time.time()

        # =========================
        # BUSCAR
        # =========================
        if state == BUSCAR:
            stable_x = 0
            t_captura_ini = None
            t_tirar_ini = None
            t_patear_ini = None

            if found:
                state = ALINEAR

        # =========================
        # ALINEAR
        # =========================
        elif state == ALINEAR:
            if lost_visual:
                state = BUSCAR
            else:
                if abs(error_x) <= TOL_X:
                    stable_x += 1
                else:
                    stable_x = 0

                Ut = ut_from_errorx(error_x)

                if stable_x >= STABLE_FRAMES:
                    state = AVANZAR

        # =========================
        # AVANZAR
        # =========================
        elif state == AVANZAR:
            if lost_visual:
                state = BUSCAR
            else:
                Ut = ut_from_errorx(error_x) * 0.6
                Ut = clamp(Ut, -UT_MAX, UT_MAX)

                near_origin = (abs(error_x) <= TOL_X) and (abs(error_y) <= TOL_Y)

                if near_origin:
                    Uy = UY_EXTRA
                else:
                    Uy = uy_from_errory(error_y)
                    if abs(Uy) < 0.08:
                        Uy = 0.08 if Uy >= 0 else -0.08

                if capture:
                    state = CAPTURAR
                    t_captura_ini = now

        # =========================
        # CAPTURAR
        # =========================
        elif state == CAPTURAR:
            if t_captura_ini is None:
                t_captura_ini = now

            if lost_visual:
                state = BUSCAR
            else:
                cilindro = 1
                Uy = UY_CAPTURA
                Ut = ut_from_errorx(error_x) * 0.4
                Ut = clamp(Ut, -UT_MAX, UT_MAX)

                # Sensor activo LOW → con pelota = 0
                if sensor_pelota == 0:
                    state = TIRAR
                    t_tirar_ini = now

                if (now - t_captura_ini) >= CAPTURA_TIMEOUT and sensor_pelota == 1:
                    state = BUSCAR

        # =========================
        # TIRAR
        # =========================
        elif state == TIRAR:
            if t_tirar_ini is None:
                t_tirar_ini = now

            if (now - t_tirar_ini) >= ESPERA_TIRO:
                state = PATEAR
                t_patear_ini = now

        # =========================
        # PATEAR
        # =========================
        elif state == PATEAR:
            if t_patear_ini is None:
                t_patear_ini = now

            if (now - t_patear_ini) < TIEMPO_PATADA:
                patada = 1
            else:
                patada = 0
                state = BUSCAR

        # -------- Enviar --------
        cmd = build_cmd(Ux, Uy, Ut, patada=patada, cilindro=cilindro)
        sender.send_line(cmd)

        print(
            f"[{state}] found={found} cap={capture} P9={sensor_pelota} "
            f"ex={error_x} ey={error_y} -> {cmd}"
        )

        cv2.imshow("Salida", debug)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    sender.close()
    tracker.release()

