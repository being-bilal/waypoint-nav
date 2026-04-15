# =============================================================================
# VEHICLE CONSTANTS
# =============================================================================
# Central configuration file for all tunable constants used across the Vehicle
# subsystem. Each section is tagged with the module it belongs to.
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
# GPS  (gps.py)
# ─────────────────────────────────────────────────────────────────────────────
GPS_CONNECTION_STRING = '/dev/ttyACM0'      # USB: '/dev/ttyACM0'  |  UART (J41): '/dev/ttyTHS1'
GPS_BAUD_RATE         = 115200              # Pixhawk telemetry port baud rate
GPS_HEARTBEAT_TIMEOUT = 10                  # Seconds to wait for Pixhawk heartbeat
GPS_MIN_FIX_TYPE      = 3                   # Minimum fix quality (3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed)

# ─────────────────────────────────────────────────────────────────────────────
# IMU / Xsens  (imu.py)
# ─────────────────────────────────────────────────────────────────────────────
IMU_TARGET_PORT       = '/dev/ttyUSB0'      # Serial port for Xsens MTi sensor
IMU_HZ                = 100                 # IMU output rate in Hz

# ─────────────────────────────────────────────────────────────────────────────
# IMU ↔ NAV FRAME AXIS ALIGNMENT  (imu.py)
# ─────────────────────────────────────────────────────────────────────────────
#
#  Your navigation coordinate frame  (UTM from lat/lon → local x, y):
#
#        North (+Y)
#           ↑
#           |
#   West ←──┼──→ East (+X)
#           |
#        South
#
#   Heading from arctan2(dy, dx):
#     0° = East,  90° = North,  -90° = South,  ±180° = West
#     Positive = counter-clockwise
#
#  Xsens default (ENU mode):
#     Yaw:   0° = North,  positive = East (CW from above)
#     Roll:  rotation about X (forward)
#     Pitch: rotation about Y (right)
#
#  Xsens default (NED mode):
#     Yaw:   0° = North,  positive = East (CW from above)
#
#  ── HOW TO CONFIGURE ──
#
#  The alignment is applied as:
#     nav_yaw = (IMU_YAW_SIGN × xsens_yaw) + IMU_YAW_NAV_OFFSET + YAW_MOUNTING_OFFSET
#
#  Step 1: Set IMU_YAW_SIGN
#     If your Xsens yaw increases clockwise (CW) but your nav frame is CCW-positive:
#       → set IMU_YAW_SIGN = -1
#     If both use the same rotation direction:
#       → set IMU_YAW_SIGN = +1
#
#  Step 2: Set IMU_YAW_NAV_OFFSET
#     This converts the Xsens "zero" reference to match the nav frame "zero".
#     Example:  Xsens 0° = North,  Nav 0° = East
#       → North is 90° in the nav frame, so IMU_YAW_NAV_OFFSET = 90.0
#     Example:  Xsens 0° = East,  Nav 0° = East
#       → IMU_YAW_NAV_OFFSET = 0.0
#
#  Step 3: Set YAW_MOUNTING_OFFSET
#     If the IMU is physically rotated on the boat (e.g. Xsens "forward"
#     points 30° to the right of the boat's bow), set this to -30.0
#
#  ── QUICK-START CHEAT SHEET ──
#     Xsens NED/ENU,  0°=North, CW+  →  Nav 0°=East, CCW+:
#       IMU_YAW_SIGN       = -1
#       IMU_YAW_NAV_OFFSET =  90.0
#       YAW_MOUNTING_OFFSET=  0.0   (adjust for physical rotation)
#
#     Xsens ENU,  0°=East, CCW+  →  Nav 0°=East, CCW+  (already matched):
#       IMU_YAW_SIGN       =  1
#       IMU_YAW_NAV_OFFSET =  0.0
#       YAW_MOUNTING_OFFSET=  0.0
# ─────────────────────────────────────────────────────────────────────────────

# Euler angle alignment
IMU_YAW_SIGN          = 1                   # +1 = same rotation sense, -1 = flip CW↔CCW
IMU_YAW_NAV_OFFSET    = 0.0                 # Degrees to add to convert Xsens 0° to nav frame 0°
YAW_MOUNTING_OFFSET   = 0.0                 # Physical mounting offset on the boat (degrees)

IMU_ROLL_SIGN         = 1                   # +1 or -1 to match roll axis direction
IMU_ROLL_OFFSET       = 0.0                 # Roll mounting offset (degrees)

IMU_PITCH_SIGN        = 1                   # +1 or -1 to match pitch axis direction
IMU_PITCH_OFFSET      = 0.0                 # Pitch mounting offset (degrees)

# Accelerometer axis remapping
# Maps Xsens accel axes → nav frame axes.
# Each entry is one of: 'x', 'y', 'z', '-x', '-y', '-z'
#   Meaning: nav_accel_x = xsens_accel[<first entry>],  etc.
#
# Example: Xsens is mounted 90° rotated (Xsens X points nav Y):
#   IMU_ACCEL_REMAP = ('y', '-x', 'z')  →  nav_x = xsens_y, nav_y = -xsens_x
IMU_ACCEL_REMAP       = ('x', 'y', 'z')    # Default: no remapping

# ─────────────────────────────────────────────────────────────────────────────
# NAVIGATION  (nav.py)
# ─────────────────────────────────────────────────────────────────────────────
LOOK_AHEAD_DELTA      = 5.0                 # Look-ahead distance along the path (metres)
WAYPOINT_FILE         = 'waypoints.json'    # Default waypoint file path

# ─────────────────────────────────────────────────────────────────────────────
# CONTROL / PID  (control.py)
# ─────────────────────────────────────────────────────────────────────────────
PWM_NEUTRAL           = 1500                # Neutral ESC PWM value (microseconds)
MAX_AXIAL_FORCE       = 10.0                # Max forward/reverse force (kgf) — adjust per thruster
MAX_YAW_TORQUE        = 5.0                 # Max rotational torque (N·m) — adjust per boat width
MAX_THRUST            = 5.2                 # Max thrust per motor (e.g. T200 @ 16V ≈ 5.2 kgf)

# PID gains for steering
STEERING_KP           = 0.05
STEERING_KI           = 0.001
STEERING_KD           = 0.01
STEERING_MIN_OUT      = -1.0                # PID output lower bound (full reverse rotation)
STEERING_MAX_OUT      = 1.0                 # PID output upper bound (full forward rotation)

BASE_SURGE            = 0.4                 # Base forward speed [−1 to 1]  (0.4 = 40%)

# ─────────────────────────────────────────────────────────────────────────────
# ACCELEROMETER  (accelerometer_data.py)
# ─────────────────────────────────────────────────────────────────────────────
GRAVITY               = 9.81                # Gravitational acceleration (m/s²)
ACCEL_CALIBRATION_SAMPLES = 200             # Number of samples for bias calibration

# ─────────────────────────────────────────────────────────────────────────────
# NETWORKING / COMMS  (main.py)
# ─────────────────────────────────────────────────────────────────────────────
BASE_IP               = "192.168.0.109"         # Base station IP address
UDP_PORT_OUT          = 5006                # Port to send telemetry TO the base station
UDP_PORT_IN           = 5005                # Port to receive waypoints FROM the base station

# ─────────────────────────────────────────────────────────────────────────────
# NAVIGATION LOOP  (main.py)
# ─────────────────────────────────────────────────────────────────────────────
NAV_LOOP_RATE         = 0.01                # Main navigation loop sleep (seconds) — ~100 Hz
