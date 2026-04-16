import time                         # For sleep in the streaming generator
import threading                    # For lock protecting IMU data between callback and main thread
import xspublic as xda              # Xsens Device API - talks to the MTi IMU sensor
from constants import (
    IMU_TARGET_PORT, IMU_HZ,        # Serial port and output rate for the Xsens sensor
    # Euler alignment constants  (constants.py -> IMU <-> NAV FRAME AXIS ALIGNMENT section)
    IMU_YAW_SIGN, IMU_YAW_NAV_OFFSET, YAW_MOUNTING_OFFSET,   # Yaw: sign flip + convention offset + physical offset
    IMU_ROLL_SIGN, IMU_ROLL_OFFSET,                            # Roll: sign flip + offset
    IMU_PITCH_SIGN, IMU_PITCH_OFFSET,                          # Pitch: sign flip + offset
    # Accelerometer axis remapping
    IMU_ACCEL_REMAP,                # Tuple like ('x','y','z') mapping Xsens axes -> nav axes
)


def _remap_accel(raw_x, raw_y, raw_z, remap=IMU_ACCEL_REMAP):
    """
    Remap raw Xsens accelerometer axes to the navigation coordinate frame.

    Each element of `remap` is one of 'x','y','z','-x','-y','-z'.
    The tuple is (nav_x_source, nav_y_source, nav_z_source).

    Example:
        ('y', '-x', 'z')  ->  nav_x = raw_y,  nav_y = -raw_x,  nav_z = raw_z
    """
    # Build a lookup table mapping axis name -> value (including negated versions)
    axis_map = {'x': raw_x, 'y': raw_y, 'z': raw_z,
                '-x': -raw_x, '-y': -raw_y, '-z': -raw_z}
    # Return the three nav-frame accelerations by looking up each remap entry
    return (axis_map[remap[0]], axis_map[remap[1]], axis_map[remap[2]])


class SimpleXsens:
    def __init__(self, hz=IMU_HZ, target_port=None):
        # XsControl is the top-level Xsens API object that manages devices
        self.control = xda.XsControl()

        # Thread-safe storage for the latest extracted values.
        # The callback thread writes these, the main thread reads them.
        self._lock = threading.Lock()
        self._euler = None    # (roll, pitch, yaw) as Python floats, or None
        self._accel = None    # (ax, ay, az)       as Python floats, or None
        self._gyro = None     # (gx, gy, gz)       as Python floats, or None    

        # Scan all serial ports for connected Xsens MTi devices
        print(f"Scanning for Xsens devices... (Target: {target_port if target_port else 'Any'})")
        ports = xda.XsScanner.scan_ports()   # returns a list of XsPortInfo objects

        # If no devices found at all, bail out immediately
        if not ports:
            raise RuntimeError("No MTi devices found! Check USB connection and dialout permissions.")

        # Try to find a device on the requested port (or take the first one found)
        self.port = None
        for p in ports:
            current_port = p.port_name()           # e.g. '/dev/ttyUSB0'
            if target_port is None or target_port in current_port:
                self.port = p                      # found it!
                break

        # If we found Xsens devices but none on the requested port, error out
        if self.port is None:
            available = [p.port_name() for p in ports]   # list all ports we DID find
            raise RuntimeError(f"Xsens found, but NOT on {target_port}. Available: {available}")

        print(f"Connected successfully to Xsens on {self.port.port_name()} at {self.port.baud_rate()} bps")

        # Open the serial connection to the device at the detected baud rate
        self.control.open_port(self.port.port_name(), self.port.baud_rate())
        # Obtain a device handle so we can configure it
        self.device = self.control.device(self.port.device_id())

        # Switch the device from measurement to config mode so we can change settings
        self.device.goto_config()

        # Configure the two data streams we need:
        config = []
        # EulerAngles gives us roll, pitch, yaw at the requested Hz
        config.append(xda.XsOutputConfiguration(xda.XsDataIdentifier.EulerAngles, hz))
        # Acceleration gives us calibrated accelerometer readings at the requested Hz
        config.append(xda.XsOutputConfiguration(xda.XsDataIdentifier.Acceleration, hz))

        # Push the config to the device and switch back to measurement mode
        self.device.set_output_configuration(config)
        self.device.goto_measurement()    # device starts streaming data now

        # ── Callback class ───────────────────────────────────────────────
        # The Xsens API is callback-driven: it calls our function whenever
        # a new data packet arrives from the sensor.
        #
        # CRITICAL FIX: We extract Python float values HERE inside the
        # callback (running on the Xsens C++ thread) and store them
        # under a lock. We do NOT pass the raw C++ packet object to the
        # main thread. Accessing a C++ object from another thread while
        # the callback replaces it causes a Segmentation Fault.
        parent = self  # reference to the outer SimpleXsens instance

        class Callback(xda.XsCallback):
            def __init__(self):
                xda.XsCallback.__init__(self)   # must call parent constructor

            # C++ native callback name (Xsens C++ SDK naming convention)
            def onLiveDataAvailable(self, dev, packet):
                self._extract(packet)

            # Pythonic callback name (some Xsens Python wrappers use this instead)
            def on_live_data_available(self, dev, packet):
                self._extract(packet)

            def _extract(self, packet):
                """Extract values from the C++ packet and store as plain Python floats."""
                with parent._lock:
                    # Extract orientation if this packet contains it
                    if packet.contains_orientation():
                        e = packet.orientation_euler()
                        parent._euler = (e.roll, e.pitch, e.yaw)

                    # Extract acceleration if this packet contains it
                    if hasattr(packet, 'contains_calibrated_acc') and packet.contains_calibrated_acc():
                        a = packet.calibrated_acc()
                        parent._accel = (a[0], a[1], a[2])

                    # Extract gyroscope if this packet contains it
                    if hasattr(packet, 'contains_calibrated_gyr') and packet.contains_calibrated_gyr():
                        g = packet.calibrated_gyr()
                        parent._gyr = (g[0], g[1], g[2])

        # Instantiate the callback and register it with the device
        self.cb = Callback()
        self.device.add_callback_handler(self.cb)   # Xsens will now call our callback

    def get_imu_packet(self):
        """
        Returns the latest IMU data as a dict with axis alignment applied,
        or None if no data is available yet.

        Thread-safe: reads only Python floats that were extracted by the
        callback under a lock. No C++ objects are accessed here.

        Alignment pipeline:
          Yaw:   nav_yaw   = IMU_YAW_SIGN   * xsens_yaw   + IMU_YAW_NAV_OFFSET + YAW_MOUNTING_OFFSET
          Roll:  nav_roll  = IMU_ROLL_SIGN   * xsens_roll  + IMU_ROLL_OFFSET
          Pitch: nav_pitch = IMU_PITCH_SIGN  * xsens_pitch + IMU_PITCH_OFFSET
          Accel: remapped via IMU_ACCEL_REMAP tuple
        """
        with self._lock:
            # Need BOTH orientation AND acceleration to produce a complete packet
            if self._euler is None or self._accel is None or self._gyro is None:
                return None

            # Copy the cached Python floats (safe - no C++ access)
            roll_raw, pitch_raw, yaw_raw = self._euler
            ax_raw, ay_raw, az_raw = self._accel
            gx_raw, gy_raw, gz_raw = self._gyro

        # ── Euler alignment (outside lock - pure Python math) ────────
        aligned_yaw = (IMU_YAW_SIGN * yaw_raw
                       + IMU_YAW_NAV_OFFSET
                       + YAW_MOUNTING_OFFSET)
        aligned_yaw = (aligned_yaw + 180) % 360 - 180          # wrap to [-180, +180]

        aligned_roll = IMU_ROLL_SIGN * roll_raw + IMU_ROLL_OFFSET
        aligned_roll = (aligned_roll + 180) % 360 - 180

        aligned_pitch = IMU_PITCH_SIGN * pitch_raw + IMU_PITCH_OFFSET
        aligned_pitch = (aligned_pitch + 180) % 360 - 180

        # ── Accelerometer remapping ──────────────────────────────────
        ax, ay, az = _remap_accel(ax_raw, ay_raw, az_raw)
        yaw_rate = IMU_YAW_SIGN * gz_raw

        # Return a flat dict with all values now in the nav coordinate frame
        return {
            "accel_x": ax,             # nav-frame X acceleration (m/s^2)
            "accel_y": ay,             # nav-frame Y acceleration (m/s^2)
            "accel_z": az,             # nav-frame Z acceleration (m/s^2)
            "roll":    aligned_roll,   # nav-frame roll  (degrees, [-180, 180])
            "pitch":   aligned_pitch,  # nav-frame pitch (degrees, [-180, 180])
            "yaw":     aligned_yaw,    # nav-frame yaw   (degrees, [-180, 180])
            "yaw_rate": yaw_rate,       # nav-frame yaw rate (degrees/s)
        }

    def close(self):
        """Safely shuts down the Xsens connection."""
        self.device.remove_callback_handler(self.cb)       # stop receiving callbacks
        self.control.close_port(self.port.port_name())     # release the serial port

# ---------------------------------------------------------------------------
# STREAMING GENERATOR  -  yields aligned IMU packets in a loop
# ---------------------------------------------------------------------------
def stream_imu():
    """Live IMU stream generator. Use in a for-loop: `for pkt in stream_imu():`"""
    sensor = SimpleXsens(hz=IMU_HZ, target_port=IMU_TARGET_PORT)  # open the sensor
    try:
        while True:
            imu_pkt = sensor.get_imu_packet()   # poll for the latest aligned packet
            if imu_pkt is not None:
                yield imu_pkt                    # yield to the caller (generator pattern)
            time.sleep(0.01)                     # ~100 Hz polling rate
    finally:
        print("Closing Xsens IMU connection...")
        sensor.close()                           # always close on exit

# ---------------------------------------------------------------------------
# STANDALONE TEST  -  run this file directly to verify IMU is working
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Running standalone IMU test...")
    try:
        for packet in stream_imu():   # blocks and yields packets indefinitely
            # Print the three Euler angles (already aligned to nav frame)
            print(f"Roll: {packet['roll']:>6.1f} | Pitch: {packet['pitch']:>6.1f} | Yaw: {packet['yaw']:>6.1f}")
    except KeyboardInterrupt:
        print("\nTest stopped by user.")