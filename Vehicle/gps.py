import threading
from dispatcher import connect, get_message


class GPS:
    def __init__(self):
        connect()   # ensures dispatcher is up, no-op if already connected

        self.lat        = 0.0
        self.lon        = 0.0
        self.alt        = 0.0
        self.fix_type   = 0
        self.satellites = 0

        self._lock       = threading.Lock()
        self._stop_event = threading.Event()

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="gps-worker"
        )
        self._thread.start()

    def _run(self):
        while not self._stop_event.is_set():
            try:
                msg = get_message("GPS_RAW_INT")
            except Exception:
                continue

            with self._lock:
                self.lat        = msg.lat / 1e7
                self.lon        = msg.lon / 1e7
                self.alt        = msg.alt / 1000.0
                self.fix_type   = msg.fix_type
                self.satellites = msg.satellites_visible

    def get(self) -> dict:
        with self._lock:
            return {
                "lat":        self.lat,
                "lon":        self.lon,
                "alt":        self.alt,
                "fix_type":   self.fix_type,
                "satellites": self.satellites,
            }

    def has_fix(self) -> bool:
        with self._lock:
            return self.fix_type >= 3

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)