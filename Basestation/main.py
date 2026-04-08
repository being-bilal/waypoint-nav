import socket
import threading
import json
import time
import gui
import logging

ASV_IP       = "127.0.0.1"
UDP_PORT_OUT = 5005
UDP_PORT_IN  = 5006

WAYPOINTS = [
    (27.914731,78.0766382),
    (27.9147516,78.0768118),
    (27.9146165,78.0767982),
    (27.9146257,78.076599),
    (27.914731,78.0766382)
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

logging.basicConfig(
    filename="info.log",
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

running = True
def send_waypoints():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = {
        "type":      "waypoints",
        "waypoints": WAYPOINTS,
    }
    data = json.dumps(packet).encode()
    for _ in range(5):
        sock.sendto(data, (ASV_IP, UDP_PORT_OUT))
        time.sleep(0.2)
    print(f"[TX] Waypoints sent to {ASV_IP}:{UDP_PORT_OUT}  ({len(WAYPOINTS)} points)")
    logging.info("Waypoints sent to %s:%d  (%d points)", ASV_IP, UDP_PORT_OUT, len(WAYPOINTS))
    sock.close()

def telemetry_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT_IN))
    sock.settimeout(1.0)

    print(f"[RX] Listening for telemetry on port {UDP_PORT_IN}...")
    logging.info("Telemetry listener started on port %d", UDP_PORT_IN)
    

    while running:
        try:
            data, _ = sock.recvfrom(4096)
            pkt = json.loads(data.decode())

            accel = pkt.get("accel", {})
            telemetry["ax"] = accel.get("ax", 0.0)
            telemetry["ay"] = accel.get("ay", 0.0)
            telemetry["az"] = accel.get("az", 0.0)

            print(f"\rACCEL  x={telemetry['ax']:>+7.3f}  y={telemetry['ay']:>+7.3f}  z={telemetry['az']:>+7.3f} m/s²",
                  end="", flush=True)

        except socket.timeout:
            continue

        except Exception as e:
            print(f"[RX] Error: {e}")
            logging.error("Error in telemetry listener: %s", str(e))

    sock.close()

def main():
    global running

    send_waypoints()

    listener = threading.Thread(target=telemetry_listener, daemon=True)
    listener.start()

    gui_thread = threading.Thread(target=gui.show, args=(WAYPOINTS, telemetry), daemon=True)
    gui_thread.start()

    try:
        while True:
            t = telemetry
            wp_idx   = t["active_wp"]
            wp_total = len(WAYPOINTS)
            wp_lat   = WAYPOINTS[min(wp_idx, wp_total - 1)][0]
            wp_lon   = WAYPOINTS[min(wp_idx, wp_total - 1)][1]
            time.sleep(0.1)

    except KeyboardInterrupt:
        running = False
        print("\n[INFO] Stopped.")
        logging.info("Basestation stopped by user.")

main()