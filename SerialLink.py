# SerialLink.py
import time
from typing import Optional

import serial
from serial.tools import list_ports


def find_arduino_port(prefer: Optional[str] = None) -> Optional[str]:
    """
    Busca un puerto tipo Arduino.
    - Si 'prefer' no es None (ej. "COM6" o "/dev/ttyACM0"), lo usa si existe.
    - Si no, intenta detectar por descripción/hwid.
    """
    ports = list(list_ports.comports())

    if prefer:
        for p in ports:
            if p.device == prefer:
                return p.device

    # Heurística típica para Arduino
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "arduino" in desc or "ch340" in desc or "wch" in desc or "cp210" in desc or "usb serial" in desc:
            return p.device

    # Si no detecta, regresa el primero disponible
    if ports:
        return ports[0].device

    return None


class SerialSender:
    """
    Abre un puerto serial y envía líneas tipo:
        M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki\\n
    con control de frecuencia (rate limit) para no saturar Arduino.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = 115200,
        min_interval: float = 0.03,  # ~33 Hz
        auto_reconnect: bool = True,
        verbose: bool = True,
    ):
        self.baud = baud
        self.min_interval = float(min_interval)
        self.auto_reconnect = auto_reconnect
        self.verbose = verbose

        self.port = port or find_arduino_port()
        if self.port is None:
            raise RuntimeError("No encontré ningún puerto serial. Conecta el Arduino o especifica port='COMx'.")

        self.ser: Optional[serial.Serial] = None
        self._last_send = 0.0

        self.connect()

    def connect(self):
        if self.ser and self.ser.is_open:
            return

        if self.verbose:
            print(f"[SERIAL] Abriendo {self.port} @ {self.baud}...")

        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        # Arduino se reinicia al abrir serial: espera un poco
        time.sleep(2.0)

        # Limpia buffers
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

        if self.verbose:
            print("[SERIAL] Listo.")

    def close(self):
        if self.ser and self.ser.is_open:
            if self.verbose:
                print("[SERIAL] Cerrando puerto...")
            self.ser.close()

    def send_line(self, line: str) -> bool:
        """
        Envía una línea. Devuelve True si envió, False si saltó por rate limit.
        """
        now = time.time()
        if (now - self._last_send) < self.min_interval:
            return False

        self._last_send = now

        if not line.endswith("\n"):
            line = line + "\n"

        data = line.encode("ascii", errors="ignore")

        try:
            if self.ser is None or not self.ser.is_open:
                if self.auto_reconnect:
                    self.connect()
                else:
                    raise RuntimeError("Serial no está abierto.")

            self.ser.write(data)
            return True

        except Exception as e:
            if self.verbose:
                print(f"[SERIAL] Error enviando: {e}")

            # Intento de reconexión
            if self.auto_reconnect:
                try:
                    time.sleep(0.2)
                    self.close()
                    time.sleep(0.2)
                    self.connect()
                except Exception as e2:
                    if self.verbose:
                        print(f"[SERIAL] Falló reconexión: {e2}")

            return False

    def read_available(self) -> list[str]:
        """
        Lee líneas que el Arduino haya mandado (debug).
        """
        out = []
        if self.ser is None or not self.ser.is_open:
            return out
        try:
            while self.ser.in_waiting:
                s = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if s:
                    out.append(s)
        except Exception:
            pass
        return out