# NAVIGATION LOGIC

# 1. COORDINATE TRANSFORMATION
# Change the coordinates of points A (start), B (end), and P (current position) 
# from geographic (latitude, longitude) to local Cartesian (x, y) coordinates.
# This is usually done using UTM projection or a local tangent plane.

# 2. CALCULATE DISTANCE 's' ALONG THE PATH
# Calculate 's', which represents the distance traveled along the path vector AB.
# This is the scalar projection of vector AP onto vector AB:
# s = (vector(AB) . vector(AP)) / |AB|
# s = ((x - x1)(x2 - x1) + (y - y1)(y2 - y1)) / sqrt((x2 - x1)^2 + (y2 - y1)^2)

# 3. CALCULATE THE TARGET POINT T(xt, yt)
# Define a target point 'T' further down the path based on a 'Look-ahead distance' (L).
# L = s + Delta (where Delta is the specific look-ahead interval).
# xt = x_A + (s + Delta) * (dx / L_total)
# yt = y_A + (s + Delta) * (dy / L_total)

# 4. CALCULATE TARGET HEADING (Qt)
# Calculate the required heading (angle) between current position P and target T.
# This represents the direction the vehicle SHOULD be facing.
# Qt = arctan2(yt - y, xt - x)

# 5. GET ACTUAL HEADING (Q)
# Retrieve the current orientation of the vehicle from the onboard magnetometer.

# 6. CALCULATE HEADING ERROR
# Determine the difference between the target heading and the actual heading.
# Heading Error (Q_error) = Qt - Q

# OTIONAL: 
# 7. CALCULATE LATERAL DISTANCE 'e' (CROSS-TRACK ERROR)
# Calculate the perpendicular distance 'e' from the current position P 
# to the path line AB.
# e = sqrt(D^2 - s^2), where D is the total distance from A to P.

# 8. IMPLEMENT PID CONTROL TO MINIMIZE ERRORS
# Use a Proportional-Integral-Derivative controller to minimize both 
# the heading error and the cross-track error.
#
# Line-of-Sight Heading (Q_los):
# Q_los = Q_target + arctan(-e / Delta)
#
# Final Error for PID input:
# E = Q_los - Q_actual
# =============================================================================


import numpy as np
from accelerometer_data import Accelerometer
import json 
import utm 
import matplotlib.pyplot as plt


def transmitData():
    accel = Accelerometer()

    try:
        while True:
            accel_data = accel.get()

            packet = {
                "accel": {
                    "ax": accel_data["ax"],
                    "ay": accel_data["ay"],
                    "az": accel_data["az"],
                },
            }
            

            yield packet

    finally:
        accel.stop()

        

def get_waypoints():
    with open('waypoints.json', 'r') as f:
        data = json.load(f)
        waypoints = data.get("waypoints", [])
    return [(wp["lat"], wp["lon"]) for wp in waypoints]        
        
import utm

def to_local_coordinates():
    waypoints = get_waypoints()
    lat_ref, lon_ref = waypoints[0]
    x_ref, y_ref, _, _ = utm.from_latlon(lat_ref, lon_ref)
    xy_waypoints = []
    for lat, lon in waypoints:
        x, y, _, _ = utm.from_latlon(lat, lon)
        # Shift origin to first waypoint
        x_local = x - x_ref
        y_local = y - y_ref
        xy_waypoints.append((round(x_local, 4), round(y_local, 4)))
    return xy_waypoints

def get_current_position():
    # In a real implementation, this would interface with the GPS module to get the current latitude and longitude
    # Assuming the lat, lon of the mid point between the first two waypoints for testing
    points = to_local_coordinates()
    xm = ((points[0][0] + points[1][0])/2) 
    ym = ((points[0][1] + points[1][1])/2) + 2 # Adding 2 meters to y to simulate being off the path
    return xm, ym

def calculate_target_point():
    waypoints = to_local_coordinates()
    A = np.array(waypoints[0])
    B = np.array(waypoints[1])
    P = np.array(get_current_position())

    # Vector AB and AP
    AB = B - A
    AP = P - A
    AB_length = np.linalg.norm(AB)

    # Projection distance along AB
    s = np.dot(AP, AB) / AB_length

    # Look-ahead distance
    Delta = 5.0  # meters

    delta_AB = AB / AB_length

    # Target point
    T = A + (s + Delta) * delta_AB

    return tuple(np.round(T, 4))

print("Target point (x, y):", calculate_target_point())

def get_current_heading():
    # In a real implementation, this would interface with the magnetometer to get the current heading
    # For testing, we can assume a fixed heading (e.g., 0 degrees)
    return 0.0


waypoints = to_local_coordinates()
A = waypoints[0]
B = waypoints[1]
P = get_current_position()
T = calculate_target_point()
# Define three points
x = [A[0], B[0], P[0], T[0]]
y = [A[1], B[1], P[1], T[1]]

# Plot points
plt.scatter(x, y)


# Labels and title
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("Plot of Three Points")

# Show grid
plt.grid()

# Display the plot
plt.show()