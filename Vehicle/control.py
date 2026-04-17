import time                              # For dt calculation in PID and arming delay
import numpy as np                       # For matrix math in thrust allocation
from pymavlink import mavutil            # MAVLink protocol to talk to the Pixhawk
from constants import (
    PWM_NEUTRAL,                         # 1500 µs — ESC neutral (no thrust)
    MAX_AXIAL_FORCE, MAX_YAW_TORQUE,     # Max force/torque the boat can produce
    MAX_THRUST,                          # Max thrust per individual motor (kgf)
    STEERING_KP, STEERING_KI, STEERING_KD,  # PID gains for yaw steering
    STEERING_MIN_OUT, STEERING_MAX_OUT,  # PID output clamp range [-1, 1]
    BASE_SURGE,                          # Default forward speed (fraction, 0.4 = 40%)
    GPS_CONNECTION_STRING, GPS_BAUD_RATE ,# Pixhawk serial port (shared with GPS)
    PWM_MAX, PWM_MIN 
)

# =============================================================================
# PID CONTROLLER  –  generic Proportional-Integral-Derivative controller
# =============================================================================
class PIDController:
    def __init__(self, Kp, Ki, Kd, min_out, max_out):
        self.Kp = Kp            # Proportional gain — reacts to current error
        self.Ki = Ki            # Integral gain — eliminates steady-state error
        self.Kd = Kd            # Derivative gain — damps oscillations
        self.min_out = min_out  # Lower clamp for the output (e.g. -1.0)
        self.max_out = max_out  # Upper clamp for the output (e.g. +1.0)

        self.prev_error = 0.0   # Error from the previous call (for D term)
        self.integral = 0.0     # Running sum of error×dt (for I term)
        self.last_time = time.time()  # Timestamp of the last call

    def compute(self, error):
        """
        Compute one PID iteration.
        :param error: The current error value (e.g. yaw_error in degrees)
        :returns: Control output in [min_out, max_out]
        """
        current_time = time.time()
        dt = current_time - self.last_time       # time elapsed since last call (seconds)
        if dt <= 0.0: dt = 0.01                  # safety: avoid division by zero

        # P term: directly proportional to the current error
        P = self.Kp * error

        # I term: accumulate error over time to fix persistent bias
        self.integral += error * dt
        # Anti-windup: clamp the integral so it can't grow unbounded
        self.integral = max(self.min_out, min(self.integral, self.max_out))
        I = self.Ki * self.integral

        # D term: rate of change of error — predicts where error is going
        D = self.Kd * ((error - self.prev_error) / dt)

        # Combine P + I + D and clamp to the output range
        output = max(self.min_out, min(P + I + D, self.max_out))
        
        # Save state for next iteration
        self.prev_error = error
        self.last_time = current_time
        return output  # value in [-1.0, +1.0] representing rotational effort

# =============================================================================
# PIXHAWK BOAT CONTROLLER  –  thrust allocation + MAVLink commands
# =============================================================================
class PixhawkBoatController:
    def __init__(self, master=None, connection_string=GPS_CONNECTION_STRING, baud=GPS_BAUD_RATE):
        """
        :param master: An existing mavutil.mavlink_connection (e.g. gps.master).
                       If provided, this connection is SHARED — no new port is opened.
                       If None, a new connection is created (for standalone testing only).
        """
        # ── 1. Use shared connection or create a new one ─────────────────
        if master is not None:
            # SHARED MODE: reuse the MAVLink connection that GPS already opened
            print(f"Controller sharing existing MAVLink connection.")
            self.master = master
        else:
            # STANDALONE MODE: open our own connection (only for isolated testing)
            print(f"Connecting to Pixhawk on {connection_string}...")
            self.master = mavutil.mavlink_connection(connection_string, baud=baud)
            self.master.wait_heartbeat()
            print("Heartbeat received!")

        # ── 2. Configure Pixhawk hardware ────────────────────────────────
        self.set_pixhawk_passthrough()   # set SERVO outputs to pass-through mode
        self.arm_vehicle()               # arm the ESCs so they accept PWM commands

        # ── 3. Create the steering PID controller ────────────────────────
        # Output is in [-1, 1]: -1 = full left turn, +1 = full right turn
        self.steering_pid = PIDController(
            Kp=STEERING_KP, Ki=STEERING_KI, Kd=STEERING_KD,       # gains from constants
            min_out=STEERING_MIN_OUT, max_out=STEERING_MAX_OUT     # clamp range
        )
        
        # Base forward speed as a fraction [-1, 1]; 0.4 = 40% forward
        self.base_surge = BASE_SURGE

    # ─────────────────────────────────────────────────────────────────────
    # MATH & ALLOCATION METHODS
    # ─────────────────────────────────────────────────────────────────────

    def invert_pwm(self, PWM, invert=True):
        """Mirror a PWM value around neutral. Use if a thruster is mounted backwards."""
        if invert: return PWM_NEUTRAL - (PWM - PWM_NEUTRAL)  # e.g. 1600 → 1400
        return PWM

    def compute_thruster_forces(self, raw_surge, raw_yaw):
        """
        Convert normalised surge/yaw commands into per-thruster forces [f_left, f_right].
        
        Uses a simple 2×2 allocation matrix for a differential-thrust boat:
          B × [f_left, f_right]^T = [desired_surge, desired_yaw]^T
        where B = [[1, 1], [-1, 1]]
          → both thrusters contribute equally to surge
          → opposite thrusters create yaw torque
        """
        # Scale normalised [-1,1] commands to physical units
        desired_surge = raw_surge * MAX_AXIAL_FORCE    # (kgf) total forward force
        desired_yaw = raw_yaw * MAX_YAW_TORQUE         # (N·m)  rotational torque

        # Allocation matrix: row 1 = surge, row 2 = yaw
        B = np.array([[1, 1], [-1, 1]])
        t = np.array([desired_surge, desired_yaw])     # desired [surge, yaw] vector
        
        # Solve for per-thruster forces: f = B⁻¹ × t
        f = np.linalg.inv(B) @ t                       # [f_left, f_right] in kgf
        max_force = np.max(np.abs(f))                  # find the largest individual force

        # If any thruster would exceed its physical max, scale both down proportionally
        # (preserves the surge/yaw ratio while staying within hardware limits)
        if max_force > MAX_THRUST: 
            f /= (max_force / MAX_THRUST)
            
        return f  # numpy array [f_left, f_right] in kgf

    def map_force_to_pwm(self, thrust):
        """
        Convert a physical thrust force (kgf) to a PWM value (µs) using
        polynomial curves fitted to the actual thruster characterisation data.
        
        The T200 thruster has separate curves for forward and reverse:
        - Forward (thrust > 0): cubic polynomial → PWM ~1535–1900
        - Reverse (thrust < 0): cubic polynomial → PWM ~1100–1464
        - Dead zone (|thrust| < 0.01): PWM = 1500 (neutral)
        """
        if abs(thrust) < 1e-2: 
            pwm_value = 1500                           # dead zone → no thrust
        elif thrust < 0:
            # Reverse thrust polynomial coefficients (from T200 characterisation)
            coeffs = np.array([4.58585333,  35.21660561,  169.73509491, 1464.33710736])
            # Evaluate: coeffs[0]×t³ + coeffs[1]×t² + coeffs[2]×t + coeffs[3]
            pwm_value = float(np.array([thrust**3, thrust**2, thrust, 1]) @ coeffs)
        elif thrust > 0:
            # Forward thrust polynomial coefficients
            coeffs = np.array([2.22716503, -22.41358258,  135.44774899, 1535.90291842])
            pwm_value = float(np.array([thrust**3, thrust**2, thrust, 1]) @ coeffs)            
        
        pwm_int = int(round(pwm_value))  # PWM must be an integer (µs)
        return max(PWM_MIN, min(PWM_MAX, pwm_int))

    # ─────────────────────────────────────────────────────────────────────
    # MAVLINK METHODS  –  hardware setup commands sent to the Pixhawk
    # ─────────────────────────────────────────────────────────────────────

    def set_pixhawk_passthrough(self):
        """Set SERVO 1 and 3 to 0 (Disabled) so they accept DO_SET_SERVO commands."""
        print("Configuring SERVO 1 and 3 for Direct MAVLink Override...")
        # CRITICAL: To use DO_SET_SERVO, the servo function MUST be set to 0. 
        # If it is set to 1 (RCPassThru), DO_SET_SERVO commands are ignored!
        for i in [1, 3]:  # Channel 1 = left motor, Channel 3 = right motor
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                f"SERVO{i}_FUNCTION".encode('utf-8'),   # parameter name
                0,                                       # value = 0 (Disabled)
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32    # data type
            )
        
        # Request all data streams at 50 Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1   
        )

    def arm_vehicle(self):
        """Arm the Pixhawk and bypass pre-arm checks for companion computer control."""
        print("Arming vehicle (forcing bypass of pre-arm checks)...")
        # Set flight mode to "custom mode 0" (Manual)
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0
        )
        time.sleep(0.5) 
        
        # Send the ARM command with the Force flag (21196) 
        # This forces the Pixhawk to arm even if GPS is slightly drifty indoors
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,                  # confirmation count
            1,                  # param1 = 1 → ARM  
            21196,              # param2 = 21196 → FORCE ARM (bypass safety checks)
            0, 0, 0, 0, 0       # params 3–7
        )
        print("Vehicle ARM Command Sent!")

    def send_direct_pwm(self, pwm_left, pwm_right):
        """
        Directly sets the PWM output on the SERVO channels using DO_SET_SERVO.
        This bypasses RC Failsafes completely.
        """
        # Command Left Motor (Servo 1)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,          # confirmation
            1,          # param1: Servo instance (Channel 1)
            pwm_left,   # param2: PWM value
            0, 0, 0, 0, 0
        )
        
        # Command Right Motor (Servo 3)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,          # confirmation
            3,          # param1: Servo instance (Channel 3)
            pwm_right,  # param2: PWM value
            0, 0, 0, 0, 0
        )
    # ─────────────────────────────────────────────────────────────────────
    # MAIN UPDATE  –  called every loop iteration from main.py
    # ─────────────────────────────────────────────────────────────────────

    def update(self, yaw_error):
        """
        Full control pipeline: yaw_error → PID → thrust allocation → PWM → Pixhawk.
        
        :param yaw_error: Heading error in degrees (positive = turn right)
        :returns: (pwm_left, pwm_right) tuple of PWM values sent to the ESCs
        """
        # 1. Run the PID controller on the yaw error → get desired rotational effort [-1, 1]
        raw_yaw = self.steering_pid.compute(yaw_error)

        # 2. Allocate thrust: convert (base_surge, raw_yaw) → [f_left, f_right] in kgf
        forces = self.compute_thruster_forces(self.base_surge, raw_yaw)

        # 3. Convert physical forces (kgf) to PWM microseconds (1100–1900)
        pwm_left = self.map_force_to_pwm(forces[0])    # left motor
        pwm_right = self.map_force_to_pwm(forces[1])   # right motor

        # If one thruster is physically mounted backwards, uncomment this:
        # pwm_left = self.invert_pwm(pwm_left)

        # 4. Send the PWM commands to the Pixhawk → ESCs → motors
        self.send_direct_pwm(pwm_left, pwm_right)

        return pwm_left, pwm_right  # return for logging/telemetry
    
    