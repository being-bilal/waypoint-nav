# accelerometer.py
import threading
from dispatcher import get_message

GRAVITY = 9.81

class Accelerometer:
    def __init__(self):
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0
        self._lock       = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="accel-worker")
        self._thread.start()

    def _run(self):
        while not self._stop_event.is_set():
            try:
                msg = get_message("RAW_IMU")
            except Exception:
                continue
            ax_raw = msg.xacc * GRAVITY / 1000.0
            ay_raw = msg.yacc * GRAVITY / 1000.0
            az_raw = msg.zacc * GRAVITY / 1000.0
            with self._lock:
                self.ax = ax_raw - self.bias_x
                self.ay = ay_raw - self.bias_y
                self.az = az_raw - self.bias_z

    def calibrate(self, samples: int = 200):
        total_x = total_y = total_z = 0.0
        collected = 0
        while collected < samples:
            try:
                msg = get_message("RAW_IMU")
            except Exception:
                continue
            total_x += msg.xacc * GRAVITY / 1000.0
            total_y += msg.yacc * GRAVITY / 1000.0
            total_z += msg.zacc * GRAVITY / 1000.0
            collected += 1
        with self._lock:
            self.bias_x = total_x / samples
            self.bias_y = total_y / samples
            self.bias_z = (total_z / samples) - GRAVITY

    def get(self) -> dict:
        with self._lock:
            return {"ax": self.ax, "ay": self.ay, "az": self.az}

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)