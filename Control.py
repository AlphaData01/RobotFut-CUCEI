# Control.py
import time
import serial

ser = None
pelota = None

# Windows: "COM5"   Ubuntu: "/dev/ttyACM0"
def connect(port="COM5", baud=115200):
    global ser
    ser = serial.Serial(port, baud, timeout=0.01, write_timeout=0.01)
    time.sleep(2)  # reset típico del Arduino al abrir el puerto

def close():
    global ser
    try:
        if ser:
            ser.close()
    except:
        pass
    ser = None

def send(Ux=0, Uy=0, Ut=0, patada=0, cilindro=0, Kp=20, Ki=5):
    global ser
    if not ser:
        return
    try:
        line = f"M,{Ux:.3f},{Uy:.3f},{Ut:.3f},{int(patada)},{int(cilindro)},{Kp:.3f},{Ki:.3f}\n"
        ser.write(line.encode("ascii"))
    except Exception as e:
        print("SERIAL SEND ERROR:", e)
        close()

def read():
    global ser, pelota
    if not ser:
        return pelota
    try:
        # no bloquea: solo procesa lo que ya llegó
        while ser.in_waiting > 0:
            s = ser.readline().decode("utf-8", errors="ignore").strip()
            if s.startswith("P,"):
                pelota = int(s.split(",")[1])
    except Exception as e:
        print("SERIAL READ ERROR:", e)
        close()
    return pelota

