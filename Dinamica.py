# Dinamica.py
import time

class Dinamica:

    BUSCAR = 0
    ALINEAR_X = 1
    IR_A_ORIGEN = 2
    CAPTURA_5S = 3

    def __init__(
        self,
        tol_x_px=25,
        ut_max=2.0,
        uy_max=0.35,
        kp_x=0.006,
        kp_y=0.004,
        captura_segundos=5.0,
        uy_captura=0.25,
        kp_arduino=20.0,
        ki_arduino=5.0,
        stable_frames=3
    ):
        self.tol_x_px = tol_x_px
        self.ut_max = ut_max
        self.uy_max = uy_max
        self.kp_x = kp_x
        self.kp_y = kp_y

        self.captura_segundos = float(captura_segundos)
        self.uy_captura = float(uy_captura)

        self.kp_arduino = float(kp_arduino)
        self.ki_arduino = float(ki_arduino)

        self.state = self.BUSCAR
        self._stable_frames = int(stable_frames)
        self._stable_x = 0
        self._t_captura_ini = None

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _cmd(self, Ux, Uy, Ut, patada=0, cilindro=0):
        # Formato exacto que tu Arduino parsea: M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki
        return f"M,{Ux:.3f},{Uy:.3f},{Ut:.3f},{int(patada)},{int(cilindro)},{self.kp_arduino:.2f},{self.ki_arduino:.2f}"

    def _ut_from_errorx(self, error_x):
        # Si gira al revés, invierte el signo aquí:
        Ut = self.kp_x * float(error_x)
        return self._clamp(Ut, -self.ut_max, self.ut_max)

    def _uy_from_errory(self, error_y):
        # Avance proporcional hacia la pelota / hacia el origen
        # Si avanza al revés, invierte el signo aquí:
        Uy = self.kp_y * float(-error_y)
        return self._clamp(Uy, -self.uy_max, self.uy_max)

    def step(self, found, capture, error_x, error_y):
        now = time.time()

        # ================= BUSCAR =================
        if self.state == self.BUSCAR:
            self._stable_x = 0
            self._t_captura_ini = None

            if not found:
                return self._cmd(0, 0, 0, cilindro=0), "BUSCAR"

            self.state = self.ALINEAR_X

        # Si pierde pelota en cualquier estado -> BUSCAR
        if not found:
            self.state = self.BUSCAR
            self._stable_x = 0
            self._t_captura_ini = None
            return self._cmd(0, 0, 0, cilindro=0), "BUSCAR"

        # ================= ALINEAR_X =================
        if self.state == self.ALINEAR_X:

            if abs(error_x) <= self.tol_x_px:
                self._stable_x += 1
            else:
                self._stable_x = 0

            Ut = self._ut_from_errorx(error_x)

            cmd = self._cmd(0, 0, Ut, cilindro=0)

            if self._stable_x >= self._stable_frames:
                self.state = self.IR_A_ORIGEN

            return cmd, "ALINEAR_X"

        # ================= IR_A_ORIGEN =================
        if self.state == self.IR_A_ORIGEN:

            Ut = self._ut_from_errorx(error_x)
            Uy = self._uy_from_errory(error_y)

            # 🔵 CAPTURA se activa por zona azul
            if capture:
                self.state = self.CAPTURA_5S
                self._t_captura_ini = now

            return self._cmd(0, Uy, Ut, cilindro=0), "IR_A_ORIGEN"

        # ================= CAPTURA_5S =================
        if self.state == self.CAPTURA_5S:

            if self._t_captura_ini is None:
                self._t_captura_ini = now

            t = now - self._t_captura_ini

            if t >= self.captura_segundos:
                self.state = self.BUSCAR
                return self._cmd(0, 0, 0, cilindro=0), "BUSCAR"

            # Rodillo ON + avance fijo 5s
            # Si avanza al revés, cambia el signo de uy_captura
            return self._cmd(0, self.uy_captura, 0, cilindro=1), f"CAPTURA_5S ({t:.1f}s)"

        # Fallback
        self.state = self.BUSCAR
        return self._cmd(0, 0, 0, cilindro=0), "BUSCAR"