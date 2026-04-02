import socket
import threading
import json
import time

ASV_IP       = "127.0.0.1"   
UDP_PORT_OUT = 5005             
UDP_PORT_IN  = 5006              

WAYPOINTS = [
    (27.915800, 78.078200),
    (27.916150, 78.078600),
    (27.916400, 78.079100),
    (27.916100, 78.079400),
    (27.915800, 78.079200),
]

telemetry = {
    "gps_lat":           0.0,
    "gps_lon":           0.0,
    "heading":           0.0,
    "roll":              0.0,
    "pitch":             0.0,
    "yaw":               0.0,
    "desired_bearing":   0.0,
    "heading_error":     0.0,
    "cross_track_error": 0.0,
    "dist_to_waypoint":  0.0,
    "active_wp":         0,
    "nav_status":        "WAITING",
}

running = True

def send_waypoints():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = {
        "type":             "waypoints",
        "waypoints":        WAYPOINTS,
    }
    data = json.dumps(packet).encode()
    # Send a few times to survive packet loss at startup
    for _ in range(5):
        sock.sendto(data, (ASV_IP, UDP_PORT_OUT))
        time.sleep(0.2)
    print(f"[TX] Waypoints sent to {ASV_IP}:{UDP_PORT_OUT}  ({len(WAYPOINTS)} points)")
    sock.close()

# Thread: listen for telemetry from Pi

def telemetry_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT_IN))
    sock.settimeout(1.0)

    print(f"[RX] Listening for telemetry on port {UDP_PORT_IN}...")

    while running:
        try:
            data, _ = sock.recvfrom(4096)
            pkt = json.loads(data.decode())

            # -------- IMU --------
            imu = pkt.get("imu", {})
            telemetry["roll"]  = imu.get("gyro_x", 0.0)
            telemetry["pitch"] = imu.get("gyro_y", 0.0)
            telemetry["yaw"]   = imu.get("gyro_z", 0.0)

            # -------- GPS --------
            gps = pkt.get("gps", {})
            telemetry["gps_lat"] = gps.get("lat", 0.0)
            telemetry["gps_lon"] = gps.get("lon", 0.0)

            # -------- Magnetometer --------
            mag = pkt.get("mag", {})
            telemetry["heading"] = mag.get("heading_deg", 0.0)

        except socket.timeout:
            continue

        except Exception as e:
            print(f"[RX] Error: {e}")

    sock.close()

def main():
    global running

    send_waypoints()

    listener = threading.Thread(target=telemetry_listener, daemon=True)
    listener.start()

    print("\033[2J")  # clear screen once
    try:
        while True:
            t = telemetry
            wp_idx   = t["active_wp"]
            wp_total = len(WAYPOINTS)
            wp_lat   = WAYPOINTS[min(wp_idx, wp_total - 1)][0]
            wp_lon   = WAYPOINTS[min(wp_idx, wp_total - 1)][1]

            print(
                f"\033[H"   # move cursor to top
                f"\n"
                f"{'='*55}\n"
                f"  ASV LIVE TELEMETRY\n"
                f"{'='*55}\n"
                f"\n"
                f"  GPS\n"
                f"    Lat : {t['gps_lat']:>12.6f} °N\n"
                f"    Lon : {t['gps_lon']:>12.6f} °E\n"
                f"\n"
                f"  IMU\n"
                f"    Roll  : {t['roll']:>7.2f} °\n"
                f"    Pitch : {t['pitch']:>7.2f} °\n"
                f"    Yaw   : {t['yaw']:>7.2f} °\n"
                f"\n"
                f"  NAVIGATION   [{t['nav_status']}]   WP {wp_idx}/{wp_total-1}\n"
                f"    Target      : {wp_lat:.6f} °N  {wp_lon:.6f} °E\n"
                f"    Heading     : {t['heading']:>7.1f} °   (desired {t['desired_bearing']:.1f} °)\n"
                f"    Heading err : {t['heading_error']:>+7.1f} °\n"
                f"    Cross-track : {t['cross_track_error']:>+7.2f} m\n"
                f"    Dist to WP  : {t['dist_to_waypoint']:>7.2f} m\n"
                f"\n"
                f"{'='*55}\n"
                f"  Ctrl+C to quit\n",
                end="", flush=True
            )
            time.sleep(0.1)   # 10 Hz display refresh

    except KeyboardInterrupt:
        running = False
        print("\n[INFO] Stopped.")

main()