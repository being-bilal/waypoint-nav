import numpy as np
import folium
import matplotlib.pyplot as plt
"""
Dummy waypoints for testing:
Waypoint 0 (Start): 27.915800°N, 78.078200°E
Waypoint 1:         27.916150°N, 78.078600°E  
Waypoint 2:         27.916400°N, 78.079100°E  
Waypoint 3:         27.916100°N, 78.079400°E  
Waypoint 4 (End):   27.915800°N, 78.079200°E  
"""
waypoints = [
    (27.915800, 78.078200),  
    (27.916150, 78.078600),  
    (27.916400, 78.079100),  
    (27.916100, 78.079400),  
    (27.915800, 78.079200)   
]

m = folium.Map(location=[waypoints[0][0], waypoints[0][1]], zoom_start=20)

folium.Marker([waypoints[0][0], waypoints[0][1]], popup="Start").add_to(m)
folium.Marker([waypoints[-1][0], waypoints[-1][1]], popup="End").add_to(m)

folium.PolyLine(waypoints, color="red", weight=3).add_to(m)

m.save("map.html")

# Turning lat/lon into x/y for plotting
R = 6378137
lat0 = waypoints[0][0]
lon0 = waypoints[0][1]
x_vals = []
y_vals = []

for lat, lon in waypoints:
    x = R * np.radians(lon - lon0) * np.cos(np.radians(lat0))
    y = R * np.radians(lat - lat0)

    x_vals.append(x/1000)  
    y_vals.append(y/1000)

plt.figure(figsize=(8,6))

plt.plot(
    x_vals,
    y_vals,
    marker='o',
    linewidth=2,
    label="Target path"
)


plt.scatter(x_vals[0], y_vals[0], s=100, label="Start")
plt.scatter(x_vals[-1], y_vals[-1], s=100, label="End")

plt.title("Target Path")
plt.xlabel("X (km)")
plt.ylabel("Y (km)")

plt.axis("equal")
plt.grid(True)
plt.legend()

plt.show()