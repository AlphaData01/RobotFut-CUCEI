
import cv2
import time
from Vision import BallTracker
from enum import Enum, auto
import Control

# Camara a usar, Resolucion de la camara
tracker = BallTracker(cam_index=1, width=640, height=480, show_windows=True)

class Estado(Enum):
    BUSQUEDA = auto()
    ALINEAR = auto()
    AVANZAR = auto()
    CAPTURAR = auto()

# Tolerancias en pixeles
TOL_X = 30
TOL_Y = 30

def main():
    estado = Estado.BUSQUEDA

    # Conectar con Arduino
    Control.connect("COM5")

    # Limitar frecuencia de envío (evita Write timeout)
    last_send = 0.0
    SEND_DT = 0.03  # 33 Hz (si aún falla, prueba 0.05)

    try:
        while True:
            #Valores de la vision
            x, y, r, found, capture, debug, error_x, error_y = tracker.read()

            # Fallo camara
            if debug is None:
                break
            # Camara
            cv2.imshow("Salida", debug)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Valores Arduino
            Ux = 0
            Uy = 0
            Ut = 0
            patada = 0
            cilindro = 0

            # ---- MAQUINA DE ESTADOS----

            # ========== BUSQUEDA ==========
            if estado == Estado.BUSQUEDA:
                print("BUSQUEDA...")
                # ---- TRANSICIONES ----
                if found:
                    estado = Estado.ALINEAR
                else:
                    #Ut = 4.0 # Girar Buscando
                    pass
            # ========== ALINEAR ==========
            elif estado == Estado.ALINEAR:
                print(f"ALINEAR.")

                # ---- Si perdio la pelota, regresa a BUSQUEDA ----
                if not found:
                    print("Perdí pelota en ALINEAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue

                Ut = 0.0095 * error_x

                # ---- TRANSICIONES ----
                if abs(error_x) <= TOL_X:
                    estado = Estado.AVANZAR

            # ========== AVANZAR ==========
            elif estado == Estado.AVANZAR:
                print("AVANZAR.")

                # ---- Si perdio la pelota, regresa a BUSQUEDA ----
                if not found:
                    print("Perdí pelota en AVANZAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue

                # Mantener giro pequeño
                Ut = 0.009 * error_x

                # Avanzar según error Y
                Uy = - 0.0025 * error_y

                # ---- TRANSICIONES ----
                if abs(error_y) <= TOL_Y:
                    estado = Estado.CAPTURAR

            # ========== CAPTURAR ==========
            elif estado == Estado.CAPTURAR:
                print("CAPTURAR.")

                #if capture == True:
                Uy = - 0.02
                cilindro = 1

                # Leer sensor de pelota del Arduino
                pelota = Control.read()

                # ---- TRANSICIONES ----
                if pelota == 1:
                    print("Pelota detectada por Arduino")
                    #time.sleep(5)
                    estado = Estado.BUSQUEDA
                else:
                    #time.sleep(5)
                    estado = Estado.BUSQUEDA

            # print("ENVIO:", Ux, Uy, Ut, patada, cilindro)
            # ENVIAR A 33 Hz (no cada frame)
            now = time.time()
            if (now - last_send) >= SEND_DT:
                Control.send(Ux, Uy, Ut, patada, cilindro)
                last_send = now

    finally:
        tracker.release()

if __name__ == "__main__":
    main()

