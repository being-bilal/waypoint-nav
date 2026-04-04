import numpy as np
import folium
import matplotlib.pyplot as plt

def show(waypoints):
    m = folium.Map(location=[waypoints[0][0], waypoints[0][1]], zoom_start=30)
    folium.Marker([waypoints[0][0],  waypoints[0][1]],  popup="Start").add_to(m)
    folium.Marker([waypoints[-1][0], waypoints[-1][1]], popup="End").add_to(m)
    folium.PolyLine(waypoints, color="red", weight=3).add_to(m)
    m.save("map.html")

    R    = 6378137
    lat0 = waypoints[0][0]
    lon0 = waypoints[0][1]
    x_vals, y_vals = [], []

    for lat, lon in waypoints:
        x = R * np.radians(lon - lon0) * np.cos(np.radians(lat0))
        y = R * np.radians(lat - lat0)
        x_vals.append(x / 1000)
        y_vals.append(y / 1000)
