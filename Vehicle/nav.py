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
# =============================================================================
# NAVIGATION LOGIC  –  Waypoint-following algorithm with EKF
# =============================================================================

import numpy as np                   
import math                          
import json                          
import utm                           
import time                          

from constants import (GPS_CONNECTION_STRING, IMU_TARGET_PORT, IMU_HZ,   
                       LOOK_AHEAD_DELTA, WAYPOINT_FILE, NAV_LOOP_RATE, ACCEPTANCE_RADIUS)  
from ekf import GPS_IMU_EKF
from gps import GPS
from imu import SimpleXsens

class Navigator:
    def __init__(self, gps, imu, waypoint_file=WAYPOINT_FILE):
        print("Initializing Navigator...")
        self.gps = gps     
        self.imu = imu     
        self.ekf = None
        self.last_time = time.time()
        self.estimated_velocity = 0.0 
        
        self.waypoints_geo = self.load_waypoints(waypoint_file)
        
        if len(self.waypoints_geo) < 2:
            raise ValueError("Need at least 2 waypoints to navigate!")
            
        lat_ref, lon_ref = self.waypoints_geo[0]
        self.x_ref, self.y_ref, self.zone_num, self.zone_let = utm.from_latlon(lat_ref, lon_ref)
        
        self.waypoints_local = self.to_local_coordinates()
        self.current_wp_index = 0 
        
        self.LOOK_AHEAD_DELTA = LOOK_AHEAD_DELTA
        self.ACCEPTANCE_RADIUS = ACCEPTANCE_RADIUS
        
        self.last_pos = None        
        self.last_yaw = None        
        self.last_imu_data = None   

    def load_waypoints(self, filename):
        with open(filename, 'r') as f:
            data = json.load(f)              
            return [(wp["lat"], wp["lon"]) for wp in data.get("waypoints", [])]

    def to_local_coordinates(self):
        local_pts = []
        for lat, lon in self.waypoints_geo:
            x, y, _, _ = utm.from_latlon(lat, lon)
            local_pts.append(np.array([x - self.x_ref, y - self.y_ref]))
        return local_pts

    def get_local_position(self):
        geo = self.gps.get()                   
        if self.gps.has_fix():                  
            x, y, _, _ = utm.from_latlon(geo['lat'], geo['lon'])   
            return np.array([x - self.x_ref, y - self.y_ref])      
        return None  

    def calculate_navigation(self):
        if self.current_wp_index >= len(self.waypoints_local) - 1:
            return {"status": "DONE"}

        # ── 1. Fetch new sensor data ──
        new_P = self.get_local_position()       
        new_imu_data = self.imu.get_imu_packet() 
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0.0: dt = 0.01

        gps_updated = new_P is not None
        imu_updated = new_imu_data is not None

        if imu_updated:
            self.last_imu_data = new_imu_data
            self.last_yaw = new_imu_data['yaw']

        # ── 2. INITIALIZE EKF ──
        if self.ekf is None:
            if gps_updated:
                self.last_pos = new_P
                
            if self.last_pos is not None and self.last_imu_data is not None:
                self.ekf = GPS_IMU_EKF(self.last_pos[0], self.last_pos[1], math.radians(self.last_imu_data['yaw']))
            
            return {"status": "WAITING", "gps_updated": gps_updated, "imu_updated": imu_updated}

        # ── 3. EKF PREDICTION ──
        if imu_updated:
            yaw_rate_rad = math.radians(new_imu_data['yaw_rate'])
            self.ekf.predict(self.estimated_velocity, yaw_rate_rad, dt)

        # ── 4. EKF UPDATE ──
        if gps_updated:
            dist_moved = float(np.linalg.norm(new_P - self.last_pos))
            
            # Deadband to prevent GPS drift from creating massive fake velocity
            if dist_moved < 0.4:
                self.estimated_velocity = 0.0
            else:
                self.estimated_velocity = dist_moved / dt
            
            self.last_pos = new_P
            self.ekf.update(new_P[0], new_P[1])

        # ── 5. GET FILTERED STATE FOR NAVIGATION ──
        ekf_x, ekf_y, ekf_yaw_deg = self.ekf.get_state()
        P = np.array([ekf_x, ekf_y])   
        actual_yaw = ekf_yaw_deg       
        
        A = self.waypoints_local[self.current_wp_index]      
        B = self.waypoints_local[self.current_wp_index + 1]  
        
        # ── Vector math ──────────────────────────────────────────────────
        AB = B - A                       
        AP = P - A                       
        AB_length = np.linalg.norm(AB)   
        
        if AB_length == 0:
            return {"status": "ERROR", "msg": "Identical waypoints"}
            
        AB_unit = AB / AB_length         
        s = np.dot(AP, AB_unit)
        dist_to_wp = np.linalg.norm(B - P)
        
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
        
        yaw_error = target_yaw_deg - actual_yaw           
        yaw_error = (yaw_error + 180) % 360 - 180         
        
        cross_track_error = float(np.cross(AB_unit, AP))

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
            "imu_data": self.last_imu_data,       
            "target_lat": target_lat,             
            "target_lon": target_lon,             
        }

    def run(self):
        try:
            print("Navigation Started. Press Ctrl+C to stop.")
            while True:
                nav = self.calculate_navigation()
                status = nav.get("status")

                if status == "DONE":
                    print("\n🎉 DESTINATION REACHED! Navigation Complete.")
                    break
                    
                elif status == "WAITING":
                    g_flag = "🟢" if nav["gps_updated"] or self.last_pos is not None else "🔴"
                    i_flag = "🟢" if nav["imu_updated"] or self.last_yaw is not None else "🔴"
                    msg = f"Waiting for initial data... GPS: {g_flag} | IMU: {i_flag}"
                    if self.last_imu_data is not None:
                        imu = self.last_imu_data
                        msg += f" | RAW IMU -> R:{imu['roll']:>6.1f} P:{imu['pitch']:>6.1f} Y:{imu['yaw']:>6.1f}"
                    print(msg.ljust(100), end="\r")
                    
                elif status == "NAVIGATING":
                    g_flag = "G" if nav["gps_updated"] else "-"
                    i_flag = "I" if nav["imu_updated"] else "-"
                    imu = nav['imu_data']
                    print(f"[{g_flag}{i_flag}] WP {self.current_wp_index}->{self.current_wp_index+1} | "
                          f"Yaw Err: {nav['yaw_error']:>7.2f}° | "
                          f"Target: {nav['target_yaw']:>6.1f}° | "
                          f"X-Track: {nav['xtrack_error']:>6.2f}m | "
                          f"RAW IMU -> R:{imu['roll']:>6.1f} P:{imu['pitch']:>6.1f} Y:{imu['yaw']:>6.1f} | "
                          f"Acc(x:{imu['accel_x']:>5.2f} y:{imu['accel_y']:>5.2f} z:{imu['accel_z']:>5.2f})")
                          
                time.sleep(NAV_LOOP_RATE)
                
        except KeyboardInterrupt:
            print("\nStopping Navigation...")
        finally:
            self.gps.stop()
            self.imu.close()

if __name__ == "__main__":
    gps = GPS(connection_string=GPS_CONNECTION_STRING)
    imu = SimpleXsens(hz=IMU_HZ, target_port=IMU_TARGET_PORT)
    nav_system = Navigator(gps, imu, WAYPOINT_FILE)
    nav_system.run()