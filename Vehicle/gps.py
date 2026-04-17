import threading                    # For running the GPS reader in a background thread
import time                         # For sleep on error recovery
from pymavlink import mavutil       # MAVLink protocol library to talk to the Pixhawk
from constants import (GPS_CONNECTION_STRING, GPS_BAUD_RATE,        # Serial port and baud rate for the Pixhawk
                       GPS_HEARTBEAT_TIMEOUT, GPS_MIN_FIX_TYPE)    # Heartbeat wait time and minimum fix quality

class GPS:
    def __init__(self, connection_string=GPS_CONNECTION_STRING, baud=GPS_BAUD_RATE):
        """
        Initializes the connection to the Pixhawk.
        :param connection_string: '/dev/ttyACM0' for USB or '/dev/ttyTHS1' for UART (J41 Pins)
        :param baud: Connection speed (default 115200 for Pixhawk Telem ports)
        """
        try:
            # Create a MAVLink serial connection to the Pixhawk flight controller
            self.master = mavutil.mavlink_connection(connection_string, baud=baud)

            # The Pixhawk sends periodic "heartbeat" messages; we wait for one
            # to confirm the connection is alive before continuing
            print(f"Waiting for heartbeat from Pixhawk on {connection_string}...")
            self.master.wait_heartbeat(timeout=GPS_HEARTBEAT_TIMEOUT)  # blocks until heartbeat or timeout
            print("Heartbeat received!")
        except Exception as e:
            print(f"Failed to connect to Pixhawk: {e}")
            raise  # re-raise so the caller knows initialisation failed

        # ── Shared state: latest GPS readings ────────────────────────────
        # These are written by the background thread and read by get()/has_fix()
        self.lat        = 0.0      # Latitude  in decimal degrees (e.g. 27.914731)
        self.lon        = 0.0      # Longitude in decimal degrees (e.g. 78.0766382)
        self.alt        = 0.0      # Altitude  in metres above mean sea level
        self.fix_type   = 0        # GPS fix quality (0=none, 2=2D, 3=3D, 4+=DGPS/RTK)
        self.satellites = 0        # Number of satellites the receiver can see

        # ── Thread synchronisation ───────────────────────────────────────
        self._lock       = threading.Lock()   # Protects the shared state above
        self._stop_event = threading.Event()  # Set this to signal the thread to exit

        # ── Background worker thread ─────────────────────────────────────
        # daemon=True means the thread dies automatically when the main program exits
        self._thread = threading.Thread(
            target=self._run,       # the method that loops and reads GPS messages
            daemon=True,
            name="gps-worker"       # shows up in debugger / thread listings
        )
        self._thread.start()        # kick off the background reader immediately

    # ─────────────────────────────────────────────────────────────────────
    # BACKGROUND LOOP  –  runs in its own thread, continuously reads GPS
    # ─────────────────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────
    # BACKGROUND LOOP  –  runs in its own thread, continuously reads GPS
    # ─────────────────────────────────────────────────────────────────────
    def _run(self):
        # Keep looping until someone calls stop() which sets the event
        while not self._stop_event.is_set():
            try:
                # Listen for BOTH message types simultaneously
                msg = self.master.recv_match(
                    type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT'], 
                    blocking=True, 
                    timeout=1.0
                )
                
                if msg is None:
                    continue  # timeout fired, no message yet → try again

                # Update the shared state under lock
                with self._lock:
                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        # Use the Pixhawk's smooth, EKF-fused coordinates
                        self.lat = msg.lat / 1e7    
                        self.lon = msg.lon / 1e7    
                        self.alt = msg.alt / 1000.0 
                        
                    elif msg.get_type() == 'GPS_RAW_INT':
                        # Keep pulling fix quality and satellites from the raw message
                        self.fix_type   = msg.fix_type      
                        self.satellites = msg.satellites_visible  

            except Exception as e:
                print(f"Error reading GPS: {e}")
                time.sleep(0.1)  # brief pause before retrying to avoid a tight error loop
    # ─────────────────────────────────────────────────────────────────────
    # PUBLIC API  –  called by nav.py / main.py from the main thread
    # ─────────────────────────────────────────────────────────────────────
    def get(self) -> dict:
        """Returns a snapshot of the latest GPS readings as a dict."""
        with self._lock:   # acquire lock so we don't read mid-update
            return {
                "lat":        self.lat,          # decimal degrees
                "lon":        self.lon,          # decimal degrees
                "alt":        self.alt,          # metres
                "fix_type":   self.fix_type,     # integer code (see MAVLink docs)
                "satellites": self.satellites,   # count
            }

    def has_fix(self) -> bool:
        """Returns True if the GPS fix quality meets our minimum threshold."""
        with self._lock:
            # 3 = 3D Fix, 4 = DGPS, 5 = RTK Float, 6 = RTK Fixed
            return self.fix_type >= GPS_MIN_FIX_TYPE  # from constants.py (default 3)

    def stop(self):
        """Signal the background thread to stop, then wait for it to finish."""
        self._stop_event.set()            # tell _run() loop to exit
        self._thread.join(timeout=2.0)    # wait up to 2 seconds for it to actually quit

# ─────────────────────────────────────────────────────────────────────────────
# STANDALONE TEST  –  run this file directly to verify GPS is working
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    gps = GPS(connection_string='/dev/ttyACM0')  # Change port as needed
    try:
        while True:
            if gps.has_fix():
                print(f"Position: {gps.get()}")     # print the full dict
            else:
                print("Waiting for GPS fix...")      # no fix yet
            time.sleep(1)                            # poll once per second
    except KeyboardInterrupt:
        gps.stop()                                   # clean shutdown on Ctrl+C