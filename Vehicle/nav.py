# =============================================================================
# NAVIGATION LOGIC  –  Waypoint-following algorithm
# =============================================================================
#
# ALGORITHM OVERVIEW (Line-of-Sight with Look-Ahead):
#
# 1. COORDINATE TRANSFORMATION
#    Convert GPS coordinates (lat, lon) to local Cartesian (x, y) using UTM.
#    The first waypoint becomes the origin (0, 0).
#
# 2. CALCULATE DISTANCE 's' ALONG THE PATH
#    s = scalar projection of vector AP onto the path vector AB.
#    s = dot(AP, AB_unit)  →  how far along the path we've traveled.
#
# 3. CALCULATE THE TARGET POINT T
#    Place a target point further down the path by a "look-ahead" distance:
#    T = A + (s + Δ) × AB_unit
#
# 4. CALCULATE TARGET HEADING (Qt)
#    Qt = arctan2(yt - y, xt - x)  →  the direction we SHOULD face.
#
# 5. GET ACTUAL HEADING (Q)
#    Read the current yaw from the IMU (already aligned to nav frame).
#
# 6. HEADING ERROR
#    yaw_error = Qt - Q   (wrapped to [-180, 180])
#
# 7. CROSS-TRACK ERROR (optional)
#    Perpendicular distance from current position to the path line.
#    e = cross(AB_unit, AP)
#
# 8. The yaw_error and cross-track error are fed into the PID controller
#    (control.py) to produce thruster PWM commands.
# =============================================================================

import numpy as np                   # Linear algebra: vectors, dot/cross products, arctan2
import json                          # Reading the waypoints JSON file
import utm                           # Converting lat/lon ↔ UTM (x, y in metres)
import time                          # Sleeping between navigation loop iterations
import matplotlib.pyplot as plt      # (available for offline plotting/debugging)

# Import constants from the central config file
from constants import (GPS_CONNECTION_STRING, IMU_TARGET_PORT, IMU_HZ,   # sensor ports (for standalone mode)
                       LOOK_AHEAD_DELTA, WAYPOINT_FILE, NAV_LOOP_RATE , ACCEPTANCE_RADIUS)  # nav algorithm tuning
# Import sensor classes (used only in standalone __main__ mode)
from gps import GPS 
from imu import SimpleXsens
from ekf import GPS_IMU_EKF

class Navigator:
    def __init__(self, gps, imu, waypoint_file=WAYPOINT_FILE):
        """
        :param gps: A GPS instance (created in main.py and injected here)
        :param imu: A SimpleXsens instance (created in main.py and injected here)
        :param waypoint_file: Path to the JSON waypoints file
        """
        print("Initializing Navigator...")
        self.gps = gps     # GPS sensor handle – we call gps.get() and gps.has_fix()
        self.imu = imu     # IMU sensor handle – we call imu.get_imu_packet()
        self.ekf = None
        self.last_time = time.time()
        self.estimated_velocity = 0.0 # Will calculate from GPS diffs
        
        # Load waypoints from the JSON file as a list of (lat, lon) tuples
        self.waypoints_geo = self.load_waypoints(waypoint_file)
        
        # We need at least 2 waypoints to define a path segment A→B
        if len(self.waypoints_geo) < 2:
            raise ValueError("Need at least 2 waypoints to navigate!")
            
        # Use the FIRST waypoint as the UTM origin so all local coordinates
        # are small numbers relative to the start point
        lat_ref, lon_ref = self.waypoints_geo[0]
        # utm.from_latlon returns (easting, northing, zone_number, zone_letter)
        self.x_ref, self.y_ref, self.zone_num, self.zone_let = utm.from_latlon(lat_ref, lon_ref)
        
        # Convert ALL waypoints from geographic → local (x, y) relative to the origin
        self.waypoints_local = self.to_local_coordinates()
        
        # Index of the CURRENT path segment: we navigate from waypoint[i] → waypoint[i+1]
        self.current_wp_index = 0 
        # Look-ahead distance (metres) – how far ahead of our projection on the path
        # we place the target point T
        self.LOOK_AHEAD_DELTA = LOOK_AHEAD_DELTA
        self.Acceptance_radius = ACCEPTANCE_RADIUS
        # ── State variables: last-known-good sensor readings ─────────────
        # GPS and IMU update at different rates and may have gaps, so we
        # cache the most recent valid reading and reuse it until a new one arrives
        self.last_pos = None        # last known local position [x, y] (numpy array)
        self.last_yaw = None        # last known yaw angle in degrees
        self.last_imu_data = None   # last known full IMU packet (roll, pitch, yaw, accel)

    def load_waypoints(self, filename):
        """Read the waypoints JSON file and return a list of (lat, lon) tuples."""
        with open(filename, 'r') as f:
            data = json.load(f)              # parse the JSON contents
            # Extract each waypoint's lat/lon into a tuple
            return [(wp["lat"], wp["lon"]) for wp in data.get("waypoints", [])]

    def to_local_coordinates(self):
        """Convert all geographic waypoints to local (x, y) coordinates in metres."""
        local_pts = []
        for lat, lon in self.waypoints_geo:
            # Convert each (lat, lon) to UTM (easting, northing)
            x, y, _, _ = utm.from_latlon(lat, lon)
            # Subtract the origin so the first waypoint is at (0, 0)
            local_pts.append(np.array([x - self.x_ref, y - self.y_ref]))
        return local_pts

    def get_local_position(self):
        """
        Read the current GPS fix and convert to local (x, y).
        Returns None if GPS doesn't have a valid fix.
        """
        geo = self.gps.get()                   # get the latest GPS snapshot dict
        if self.gps.has_fix():                  # only proceed if fix quality is good enough
            x, y, _, _ = utm.from_latlon(geo['lat'], geo['lon'])   # lat/lon → UTM
            return np.array([x - self.x_ref, y - self.y_ref])      # subtract origin
        return None  # no fix → caller should use last_pos instead

    # ─────────────────────────────────────────────────────────────────────
    # CORE NAVIGATION CALCULATION  –  called once per loop iteration
    # ─────────────────────────────────────────────────────────────────────
    def calculate_navigation(self):
        """
        One iteration of the navigation algorithm.
        Returns a dict with status and computed navigation values.
        """
        # If we've passed the last segment, navigation is complete
        if self.current_wp_index >= len(self.waypoints_local) - 1:
            return {"status": "DONE"}

        # ── 1. Fetch new sensor data (may be None if no new reading yet) ─
        new_P = self.get_local_position()       
        new_imu_data = self.imu.get_imu_packet() 
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0: dt = 0.01

        # Flags for telemetry (so the GUI knows when we get fresh data)
        gps_updated = new_P is not None
        imu_updated = new_imu_data is not None

        # ── 2. INITIALIZE EKF (Run once when we get our first data) ──
        if self.ekf is None:
            if gps_updated and imu_updated:
                # Initialize EKF with first GPS pos and IMU yaw (converted to radians)
                self.ekf = GPS_IMU_EKF(new_P[0], new_P[1], math.radians(new_imu_data['yaw']))
                self.last_pos = new_P
                self.last_imu_data = new_imu_data
            return {"status": "WAITING", "gps_updated": gps_updated, "imu_updated": imu_updated}

        # ── 3. EKF PREDICTION (Using IMU) ──
        if imu_updated:
            self.last_imu_data = new_imu_data
            # Convert yaw rate from deg/s to rad/s
            yaw_rate_rad = math.radians(new_imu_data['yaw_rate'])
            # Predict where we are based on velocity and yaw rate
            self.ekf.predict(self.estimated_velocity, yaw_rate_rad, dt)

        # ── 4. EKF UPDATE (Using GPS) ──
        if gps_updated:
            # Calculate simple velocity for the prediction step (distance / time)
            dist_moved = np.linalg.norm(new_P - self.last_pos)
            self.estimated_velocity = dist_moved / dt
            self.last_pos = new_P
            
            # Correct the EKF drift with absolute GPS coordinates
            self.ekf.update(new_P[0], new_P[1])

        # ── 5. GET FILTERED STATE FOR NAVIGATION ──
        ekf_x, ekf_y, ekf_yaw_deg = self.ekf.get_state()
        P = np.array([ekf_x, ekf_y])   # Smooth, filtered position
        actual_yaw = ekf_yaw_deg       # Smooth, filtered heading
        
        # A = start of the current path segment, B = end of the current path segment
        A = self.waypoints_local[self.current_wp_index]      
        B = self.waypoints_local[self.current_wp_index + 1]  
        
        # ── Vector math ──────────────────────────────────────────────────
        AB = B - A                       
        AP = P - A                       
        AB_length = np.linalg.norm(AB)   
        
        # Guard: two identical waypoints would cause a division-by-zero
        if AB_length == 0:
            return {"status": "ERROR", "msg": "Identical waypoints"}
            
        AB_unit = AB / AB_length         
        
        # s = how far along the path we've traveled (scalar projection of AP onto AB)
        s = np.dot(AP, AB_unit)
        
        # Calculate straight-line distance to the target waypoint (B)
        dist_to_wp = np.linalg.norm(B - P)
        
        # If s >= AB_length, we've passed waypoint B → switch to the next segment
        if dist_to_wp <= self.ACCEPTANCE_RADIUS or s >= AB_length:
            print(f"\n--- Reached Waypoint {self.current_wp_index + 1}! Switching to next segment. ---")
            self.current_wp_index += 1   
            return {"status": "WAYPOINT_SWITCH"}
        
        # ── Calculate the target point T (look-ahead) ────────────────────
        T = A + (s + self.LOOK_AHEAD_DELTA) * AB_unit
        target_easting = T[0] + self.x_ref
        target_northing = T[1] + self.y_ref
        target_lat, target_lon = utm.to_latlon(target_easting, target_northing, self.zone_num, self.zone_let)
        
        # ── Calculate target heading ─────────────────────────────────────
        PT = T - P                                        
        target_yaw_rad = np.arctan2(PT[1], PT[0])         
        target_yaw_deg = np.degrees(target_yaw_rad)       
        
        # ── Calculate yaw error ──────────────────────────────────────────
        yaw_error = target_yaw_deg - actual_yaw           
        yaw_error = (yaw_error + 180) % 360 - 180         
        
        # ── Cross-track error ────────────────────────────────────────────
        cross_track_error = float(np.cross(AB_unit, AP))

        # ── Return all computed values ───────────────────────────────────
        return {
            "status": "NAVIGATING",
            "gps_updated": gps_updated,           
            "imu_updated": imu_updated,           
            "pos": P,                             
            "target": T,                          
            "target_yaw": target_yaw_deg,         
            "actual_yaw": actual_yaw,             
            "yaw_error": yaw_error,               
            "xtrack_error": cross_track_error,    
            "imu_data": self.last_imu_data,       # FIXED: Added missing comma
            "target_lat": target_lat,             
            "target_lon": target_lon,             
        }

    # ─────────────────────────────────────────────────────────────────────
    # STANDALONE RUN LOOP  –  used when running nav.py directly
    # (When using main.py, the main loop lives there instead)
    # ─────────────────────────────────────────────────────────────────────
    def run(self):
        try:
            print("Navigation Started. Press Ctrl+C to stop.")
            while True:
                nav = self.calculate_navigation()  # one iteration of the nav algorithm
                
                status = nav.get("status")

                if status == "DONE":
                    print("\n🎉 DESTINATION REACHED! Navigation Complete.")
                    break                          # exit the loop — we're there!
                    
                elif status == "WAITING":
                    # Show sensor status indicators while waiting for initial data
                    g_flag = "🟢" if nav["gps_updated"] or self.last_pos is not None else "🔴"
                    i_flag = "🟢" if nav["imu_updated"] or self.last_yaw is not None else "🔴"
                    
                    msg = f"Waiting for initial data... GPS: {g_flag} | IMU: {i_flag}"
                    
                    # If IMU is already streaming, show its values while we wait for GPS
                    if self.last_imu_data is not None:
                        imu = self.last_imu_data
                        msg += f" | RAW IMU -> R:{imu['roll']:>6.1f} P:{imu['pitch']:>6.1f} Y:{imu['yaw']:>6.1f}"
                        
                    # ljust(100) pads the line with spaces so decreasing numbers don't leave artifacts
                    print(msg.ljust(100), end="\r")  # \r = overwrite the same line
                    
                elif status == "NAVIGATING":
                    # Show which data sources updated this cycle
                    g_flag = "G" if nav["gps_updated"] else "-"   # G = fresh GPS, - = stale
                    i_flag = "I" if nav["imu_updated"] else "-"   # I = fresh IMU, - = stale
                    
                    imu = nav['imu_data']
                    
                    # Print a scrolling log line (no \r) with full navigation state
                    print(f"[{g_flag}{i_flag}] WP {self.current_wp_index}->{self.current_wp_index+1} | "
                          f"Yaw Err: {nav['yaw_error']:>7.2f}° | "
                          f"Target: {nav['target_yaw']:>6.1f}° | "
                          f"X-Track: {nav['xtrack_error']:>6.2f}m | "
                          f"RAW IMU -> R:{imu['roll']:>6.1f} P:{imu['pitch']:>6.1f} Y:{imu['yaw']:>6.1f} | "
                          f"Acc(x:{imu['accel_x']:>5.2f} y:{imu['accel_y']:>5.2f} z:{imu['accel_z']:>5.2f})")
                          
                elif status == "WAYPOINT_SWITCH":
                    pass   # the switch message was already printed in calculate_navigation()

                time.sleep(NAV_LOOP_RATE)  # ~100 Hz (0.01s) — don't spin-loop at 100% CPU
                
        except KeyboardInterrupt:
            print("\nStopping Navigation...")
        finally:
            # Clean up sensor connections regardless of how we exited
            self.gps.stop()     # stop the GPS background thread
            self.imu.close()    # release the Xsens serial port
        

# ─────────────────────────────────────────────────────────────────────────────
# STANDALONE MODE  –  run nav.py directly to navigate without main.py
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Create sensor instances directly (normally main.py does this)
    gps = GPS(connection_string=GPS_CONNECTION_STRING)
    imu = SimpleXsens(hz=IMU_HZ, target_port=IMU_TARGET_PORT)
    # Create the navigator with injected sensors and start the run loop
    nav_system = Navigator(gps, imu, WAYPOINT_FILE)
    nav_system.run()