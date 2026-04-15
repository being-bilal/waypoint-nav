import time                         # For sleep in the streaming generator
import xspublic as xda              # Xsens Device API – talks to the MTi IMU sensor
from constants import (
    IMU_TARGET_PORT, IMU_HZ,        # Serial port and output rate for the Xsens sensor
    # Euler alignment constants  (constants.py → IMU ↔ NAV FRAME AXIS ALIGNMENT section)
    IMU_YAW_SIGN, IMU_YAW_NAV_OFFSET, YAW_MOUNTING_OFFSET,   # Yaw: sign flip + convention offset + physical offset
    IMU_ROLL_SIGN, IMU_ROLL_OFFSET,                            # Roll: sign flip + offset
    IMU_PITCH_SIGN, IMU_PITCH_OFFSET,                          # Pitch: sign flip + offset
    # Accelerometer axis remapping
    IMU_ACCEL_REMAP,                # Tuple like ('x','y','z') mapping Xsens axes → nav axes
)


def _remap_accel(raw_x, raw_y, raw_z, remap=IMU_ACCEL_REMAP):
    """
    Remap raw Xsens accelerometer axes to the navigation coordinate frame.

    Each element of `remap` is one of 'x','y','z','-x','-y','-z'.
    The tuple is (nav_x_source, nav_y_source, nav_z_source).

    Example:
        ('y', '-x', 'z')  →  nav_x = raw_y,  nav_y = -raw_x,  nav_z = raw_z
    """
    # Build a lookup table mapping axis name → value (including negated versions)
    axis_map = {'x': raw_x, 'y': raw_y, 'z': raw_z,
                '-x': -raw_x, '-y': -raw_y, '-z': -raw_z}
    # Return the three nav-frame accelerations by looking up each remap entry
    return (axis_map[remap[0]], axis_map[remap[1]], axis_map[remap[2]])


class SimpleXsens:
    def __init__(self, hz=IMU_HZ, target_port=None):
        # XsControl is the top-level Xsens API object that manages devices
        self.control = xda.XsControl()
        
        # The Xsens sometimes sends orientation and acceleration in separate
        # packets, so we buffer the latest of each and combine them
        self.latest_euler = None   # will hold the last orientation reading
        self.latest_accel = None   # will hold the last acceleration reading
        
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
        # a new data packet arrives from the sensor
        class Callback(xda.XsCallback):
            def __init__(self):
                xda.XsCallback.__init__(self)   # must call parent constructor
                self.packet = None              # will hold the most recent data packet
                
            # C++ native callback name (Xsens C++ SDK naming convention)
            def onLiveDataAvailable(self, dev, packet):
                print(".", end="", flush=True)  # print a dot as a heartbeat indicator
                self.packet = packet            # store the latest packet for get_imu_packet()

            # Pythonic callback name (some Xsens Python wrappers use this instead)
            def on_live_data_available(self, dev, packet):
                print(".", end="", flush=True)  # same heartbeat dot
                self.packet = packet            # same storage
        
        # Instantiate the callback and register it with the device
        self.cb = Callback()
        self.device.add_callback_handler(self.cb)   # Xsens will now call our callback

    def get_imu_packet(self):
        """
        Extracts Accel and Euler data, handling split packets.
        Applies full axis alignment from constants.py so the returned
        values are in the NAVIGATION coordinate frame.

        Alignment pipeline:
          Yaw:   nav_yaw   = IMU_YAW_SIGN   * xsens_yaw   + IMU_YAW_NAV_OFFSET + YAW_MOUNTING_OFFSET
          Roll:  nav_roll  = IMU_ROLL_SIGN   * xsens_roll  + IMU_ROLL_OFFSET
          Pitch: nav_pitch = IMU_PITCH_SIGN  * xsens_pitch + IMU_PITCH_OFFSET
          Accel: remapped via IMU_ACCEL_REMAP tuple
        """
        # Only proceed if the callback has received at least one packet
        if self.cb.packet:
            
            # --- Step 1: Extract orientation if this packet contains it ---
            # (some packets may only have accel, not orientation, hence the check)
            if self.cb.packet.contains_orientation():
                self.latest_euler = self.cb.packet.orientation_euler()  # .roll, .pitch, .yaw
                
            # --- Step 2: Extract acceleration if this packet contains it ---
            # We check both hasattr (API compatibility) and the method itself
            if hasattr(self.cb.packet, 'contains_calibrated_acc') and self.cb.packet.contains_calibrated_acc():
                self.latest_accel = self.cb.packet.calibrated_acc()  # [ax, ay, az] in m/s²

            # --- Step 3: Combine, align to nav frame, and return ---
            # Only return a packet if we have BOTH orientation AND acceleration
            if self.latest_euler is not None and self.latest_accel is not None:

                # ── Euler alignment ──────────────────────────────────────
                # Apply sign flip (CW↔CCW), convention offset (North→East), and mounting offset
                aligned_yaw = (IMU_YAW_SIGN * self.latest_euler.yaw    # flip sign if needed
                               + IMU_YAW_NAV_OFFSET                    # convert 0°-reference
                               + YAW_MOUNTING_OFFSET)                   # physical rotation on the boat
                aligned_yaw = (aligned_yaw + 180) % 360 - 180          # wrap result to [-180°, +180°]

                # Same formula for roll: sign flip + mounting offset
                aligned_roll = IMU_ROLL_SIGN * self.latest_euler.roll + IMU_ROLL_OFFSET
                aligned_roll = (aligned_roll + 180) % 360 - 180        # wrap to [-180°, +180°]

                # Same formula for pitch: sign flip + mounting offset
                aligned_pitch = IMU_PITCH_SIGN * self.latest_euler.pitch + IMU_PITCH_OFFSET
                aligned_pitch = (aligned_pitch + 180) % 360 - 180      # wrap to [-180°, +180°]

                # ── Accelerometer remapping ──────────────────────────────
                # Swap/negate the raw Xsens accel axes to match the nav coordinate frame
                ax, ay, az = _remap_accel(
                    self.latest_accel[0],    # raw Xsens X acceleration
                    self.latest_accel[1],    # raw Xsens Y acceleration
                    self.latest_accel[2],    # raw Xsens Z acceleration
                )

                # Return a flat dict with all values now in the nav coordinate frame
                return {
                    "accel_x": ax,             # nav-frame X acceleration (m/s²)
                    "accel_y": ay,             # nav-frame Y acceleration (m/s²)
                    "accel_z": az,             # nav-frame Z acceleration (m/s²)
                    "roll":    aligned_roll,   # nav-frame roll  (degrees, [-180, 180])
                    "pitch":   aligned_pitch,  # nav-frame pitch (degrees, [-180, 180])
                    "yaw":     aligned_yaw,    # nav-frame yaw   (degrees, [-180, 180])
                }
        return None  # no data available yet (or packets were split and incomplete)

    def close(self):
        """Safely shuts down the Xsens connection."""
        self.device.remove_callback_handler(self.cb)       # stop receiving callbacks
        self.control.close_port(self.port.port_name())     # release the serial port

# ─────────────────────────────────────────────────────────────────────────────
# STREAMING GENERATOR  –  yields aligned IMU packets in a loop
# ─────────────────────────────────────────────────────────────────────────────
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

# ─────────────────────────────────────────────────────────────────────────────
# STANDALONE TEST  –  run this file directly to verify IMU is working
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Running standalone IMU test...")
    try:
        for packet in stream_imu():   # blocks and yields packets indefinitely
            # Print the three Euler angles (already aligned to nav frame)
            print(f"Roll: {packet['roll']:>6.1f} | Pitch: {packet['pitch']:>6.1f} | Yaw: {packet['yaw']:>6.1f}")
    except KeyboardInterrupt:
        print("\nTest stopped by user.")