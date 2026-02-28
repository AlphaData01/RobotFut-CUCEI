
import cv2
import time
from Vision import BallTracker
from enum import Enum, auto

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

            # ---- MAQUINA DE ESTADOS----
            if estado == Estado.BUSQUEDA:
                print("BUSQUEDA...")
                # ---- TRANSICIONES ----
                if found:
                    estado = Estado.ALINEAR
                else:
                    # aquí podrías girar para buscar (por ahora solo print)
                    pass

            elif estado == Estado.ALINEAR:
                print(f"ALINEAR:")

                # ---- Si perdio la pelota, regresa a BUSQUEDA ----
                if not found or error_x is None:
                    print("Perdí pelota en ALINEAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue

                # ---- TRANSICIONES ----
                print(f"error_x={error_x}  error_y={error_y}  capture={capture}")

                if abs(error_x) <= TOL_X:
                    estado = Estado.AVANZAR

            elif estado == Estado.AVANZAR:
                print("AVANZAR.")

                # ---- Si perdio la pelota, regresa a BUSQUEDA ----
                if not found or error_y is None:
                    print("Perdí pelota en AVANZAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue

                # ---- TRANSICIONES ----
                print(f"error_x={error_x}  error_y={error_y}  capture={capture}")
                if abs(error_y) <= TOL_Y:
                    estado = Estado.CAPTURAR

            elif estado == Estado.CAPTURAR:
                print("CAPTURAR.")

                # ---- Si perdio la pelota, regresa a BUSQUEDA ----
                if not found:
                    print("Perdí pelota en CAPTURAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue

                # ---- TRANSICIONES ----
                print(f"error_x={error_x}  error_y={error_y}  capture={capture}")
                if capture:
                    print("CAPTURA = TRUE, esperando 10s y regreso a BUSQUEDA")
                    time.sleep(10)
                    estado = Estado.BUSQUEDA

    finally:
        tracker.release()

if __name__ == "__main__":
    main()