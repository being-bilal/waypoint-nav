"""
Reads the three dummy CSV files row by row and sends RAW data
to the base station.

Packet types sent:
  - type: "imu" → raw IMU data
  - type: "gps" → latitude, longitude
  - type: "mag" → raw magnetometer heading

Usage:
    python pi_sender.py
"""

import socket
import threading
import json
import time
import csv
import os
import logging
from nav import transmitData

BASE_IP      = "127.0.0.1"
UDP_PORT_OUT = 5006
UDP_PORT_IN  = 5005

CSV_DIR      = "/Users/mohammadbilal/Documents/Projects/waypoint-nav/dummy_data"
GPS_CSV      = os.path.join(CSV_DIR, "gps_data.csv")
IMU_CSV      = os.path.join(CSV_DIR, "imu_data.csv")
MAG_CSV      = os.path.join(CSV_DIR, "magnetometer_data.csv")

running = True
waypoints = []

def load_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def waypoint_receiver():
    global waypoints
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT_IN))
    sock.settimeout(1.0)

    print(f"[RX] Listening for waypoints on port {UDP_PORT_IN}...")

    while running:
        try:
            data, _ = sock.recvfrom(4096)
            pkt = json.loads(data.decode())

            if pkt.get("type") == "waypoints":
                waypoints = pkt.get("waypoints", [])
                waypoints = [(float(lat), float(lon)) for lat, lon in waypoints]

                print(f"[RX] Received {len(waypoints)} waypoints.")
                # Save waypoints to JSON
                waypoint_data = {
                    "waypoints": [
                        {"lat": lat, "lon": lon} for lat, lon in waypoints
                    ]
                }

                with open("waypoints.json", "w") as f:
                    json.dump(waypoint_data, f, indent=4)

                print("[RX] Waypoints saved ")

                break

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[RX] Error: {e}")
    

    sock.close()
    
def main():

    global running

    # Start waypoint listener thread
    wp_thread = threading.Thread(target=waypoint_receiver, daemon=True)
    wp_thread.start()

    print("[TX] Starting NAV data stream. Press Ctrl+C to stop.\n")

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Loop timing
    dt = 0.01   # 100 Hz

    try:

        for nav_data in transmitData():

            if not running:
                break

            print(f"[TX] Sending nav packet: {nav_data}")

            sock.sendto(json.dumps(nav_data).encode(), (BASE_IP, UDP_PORT_OUT))

            time.sleep(dt)

    except KeyboardInterrupt:
        running = False
        print("\n[INFO] Stopped.")

    finally:
        sock.close()
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    main()