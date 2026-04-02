# ASV Waypoint Navigation System
This is the step by step procedure to achieve waypoint navigation for the Autonomous surface vehicle using GPS and IMU data.

------------------------------------------------------------------------

# PHASE 1 --- Sensor Setup and Raw Data

## Step 1 --- Establish What Each Sensor Outputs

**GPS outputs:**

-   Latitude 
-   Longitude

**IMU outputs:**

-   Accelerometer: acceleration in X, Y, Z axes 
-   Gyroscope: rotation rate around X, Y, Z axes 

**Magnetometer outputs:**

-   Magnetic field strength in X, Y, Z axes 

------------------------------------------------------------------------

## Step 2 --- Define Your Body Frame

Before anything else, physically establish your ASV's coordinate frame:

    X axis → points toward ASV nose (forward)
    Y axis → points to the left of ASV
    Z axis → points upward

    This is standard ROS REP-103 convention

Make sure your IMU is mounted aligned with this frame. If it's rotated,
apply a fixed rotation correction to all IMU readings first.

------------------------------------------------------------------------

## Step 3 --- Remove Gravity From Accelerometer

Raw accelerometer reads gravity (9.81 m/s²) along with actual motion.
You must subtract gravity before using acceleration for dead reckoning.

At rest on flat ground:

    raw_accel_z = +9.81 m/s²
    actual_motion_accel = raw_accel - gravity_vector

The gravity vector direction changes as your ASV pitches and rolls, so
you need your orientation estimate to know which direction gravity is
pointing at each moment.

------------------------------------------------------------------------

# PHASE 2 --- Sensor Fusion for Heading

## Step 4 --- Get Raw Heading From Magnetometer

The magnetometer gives you field strength in X and Y axes.

    raw_heading = atan2(mag_y, mag_x)
    convert to degrees
    normalize to 0–360°

This gives magnetic north heading.

------------------------------------------------------------------------

## Step 5 --- Apply Hard Iron and Soft Iron Calibration

Your ASV's motors, metal frame, and electronics distort the magnetic
field around the magnetometer.

**Hard iron** --- shifts the center of magnetometer readings.

**Soft iron** --- stretches and tilts the ellipse formed by readings.

### Calibration procedure

1.  Slowly rotate the ASV 360°.
2.  Record magnetometer X,Y readings.
3.  Ideal result is a circle centered at (0,0).
4.  Real result is an offset ellipse.
5.  Compute offset and scaling.
6.  Apply corrections.


------------------------------------------------------------------------

# PHASE 3 --- Sensor Fusion for Position

## Step 8 --- GPS Preprocessing

Apply smoothing:

    filtered_lat = β * new_gps_lat + (1 - β) * previous_lat
    filtered_lon = β * new_gps_lon + (1 - β) * previous_lon

Typical:

    β = 0.3–0.5

Reject unrealistic jumps:

    if haversine(new_gps, previous_gps)/dt > max_speed:
        reject reading

------------------------------------------------------------------------

## Step 9 --- IMU Dead Reckoning

    accel_world_x = accel_body_x * cos(heading) - accel_body_y * sin(heading)
    accel_world_y = accel_body_x * sin(heading) + accel_body_y * cos(heading)

    velocity_x += accel_world_x * dt
    velocity_y += accel_world_y * dt

    displacement_x += velocity_x * dt
    displacement_y += velocity_y * dt

Convert displacement to lat/lon:

    estimated_lat = last_gps_lat + (displacement_y / R)
    estimated_lon = last_gps_lon + (displacement_x / (R * cos(last_gps_lat)))

------------------------------------------------------------------------

## Step 10 --- Correct With GPS

    corrected_lat = estimated_lat + K * (gps_lat - estimated_lat)
    corrected_lon = estimated_lon + K * (gps_lon - estimated_lon)

    velocity_x = velocity_x * (1 - K)
    velocity_y = velocity_y * (1 - K)

Typical:

    K = 0.5–0.8

------------------------------------------------------------------------

# PHASE 4 --- Navigation Geometry

## Step 11 --- Load Waypoints

    waypoints = [
        (lat_A, lon_A),
        (lat_B, lon_B),
        (lat_C, lon_C)
    ]

------------------------------------------------------------------------

## Step 12 --- Compute Bearing

    Δlon = target_lon - current_lon

    x = cos(target_lat) * sin(Δlon)
    y = cos(current_lat) * sin(target_lat) - sin(current_lat) * cos(target_lat) * cos(Δlon)

    bearing = atan2(x, y)
    bearing = degrees(bearing)
    bearing = (bearing + 360) % 360

------------------------------------------------------------------------

## Step 13 --- Compute Distance (Haversine)

    Δlat = target_lat - current_lat
    Δlon = target_lon - current_lon

    a = sin²(Δlat/2) + cos(current_lat)*cos(target_lat)*sin²(Δlon/2)
    c = 2 * atan2(√a, √(1-a))

    distance = 6371000 * c

------------------------------------------------------------------------

## Step 14 --- Convert to Local XY

    P.x = (current_lon - A.lon) * cos(A.lat) * 6371000
    P.y = (current_lat - A.lat) * 6371000

    B.x = (B.lon - A.lon) * cos(A.lat) * 6371000
    B.y = (B.lat - A.lat) * 6371000

------------------------------------------------------------------------

## Step 15 --- Cross Track Error

    XTE = (AB.x * AP.y - AB.y * AP.x) / sqrt(B.x² + B.y²)

------------------------------------------------------------------------

## Step 16 --- Along Track Distance

    along_track = (P.x * B.x + P.y * B.y) / sqrt(B.x² + B.y²)

------------------------------------------------------------------------

# PHASE 5 --- Control

## Step 17 --- Heading Error

    heading_error = desired_bearing - current_heading

    if heading_error > 180:
        heading_error -= 360
    if heading_error < -180:
        heading_error += 360

------------------------------------------------------------------------

## Step 18 --- Blend XTE

    xte_correction = k_xte * XTE

    total_heading_error = heading_error + xte_correction

Typical:

    k_xte = 0.5

------------------------------------------------------------------------

## Step 19 --- PID Controller

    proportional = Kp * error

    integral += error * dt
    integral_term = Ki * integral

    derivative = (error - previous_error)/dt
    derivative_term = Kd * derivative

    yaw_command = proportional + integral_term + derivative_term

Clamp values to prevent windup.

------------------------------------------------------------------------

## Step 20 --- Forward Thrust

    forward_thrust = constant_value

------------------------------------------------------------------------

# PHASE 6 --- Waypoint Management

## Step 21 --- Waypoint Arrival

    if distance < acceptance_radius:

        current_waypoint_index += 1

        if current_waypoint_index >= len(waypoints):
            stop motors
        else:
            previous_waypoint = target_waypoint
            target_waypoint = waypoints[current_waypoint_index]

------------------------------------------------------------------------

# PHASE 7 --- Main Loop

## Step 22 --- Complete Loop

    1. Read sensors
    2. Fuse magnetometer + gyro → heading
    3. Fuse GPS + IMU → position
    4. Compute bearing
    5. Compute distance
    6. Compute XTE
    7. Compute heading error
    8. Run PID
    9. Send commands to motors
    10. Check waypoint arrival
    11. Repeat

------------------------------------------------------------------------

# System Flow Summary

Sensors          →   Raw lat/lon, accel, gyro, mag
       ↓
Calibration      →   Corrected mag, gravity-removed accel
       ↓
Fusion           →   Clean heading (°), clean position (lat/lon)
       ↓
Geometry         →   bearing (°), distance (m), XTE (m), along_track (m)
       ↓
Control          →   heading_error (°), total_correction (°)
       ↓
PID              →   yaw_command
       ↓
Actuators        →   motor left/right commands
       ↓
Waypoint logic   →   advance to next waypoint when arrived