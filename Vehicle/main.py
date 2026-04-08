import socket
import threading
import json
import time
import logging
from pymavlink import mavutil
from nav import transmitData
import dispatcher

BASE_IP      = "127.0.0.1"
UDP_PORT_OUT = 5006
UDP_PORT_IN  = 5005

PIXHAWK_CONNECTION = "/dev/tty.usbmodem1101"
PIXHAWK_BAUD       = 115200

running   = True
waypoints = []

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger(__name__)


def connect_pixhawk():
    log.info(f"Connecting to Pixhawk on {PIXHAWK_CONNECTION}...")
    master = mavutil.mavlink_connection(PIXHAWK_CONNECTION, baud=PIXHAWK_BAUD)
    master.wait_heartbeat()
    log.info(f"Heartbeat received (system {master.target_system}, "
             f"component {master.target_component})")

    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 50, 1)   # RAW_IMU  @ 50 Hz

    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1)        # GPS      @  2 Hz

    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50, 1)         # ATTITUDE @ 50 Hz

    return master


def waypoint_receiver():
    global waypoints
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT_IN))
    sock.settimeout(1.0)
    log.info(f"Listening for waypoints on port {UDP_PORT_IN}...")

    while running:
        try:
            data, _ = sock.recvfrom(4096)
            pkt = json.loads(data.decode())
            if pkt.get("type") == "waypoints":
                waypoints = [
                    (float(lat), float(lon))
                    for lat, lon in pkt.get("waypoints", [])
                ]
                log.info(f"Received {len(waypoints)} waypoints.")
                with open("waypoints.json", "w") as f:
                    json.dump({
                        "waypoints": [
                            {"lat": lat, "lon": lon} for lat, lon in waypoints
                        ]
                    }, f, indent=4)
                log.info("Waypoints saved.")
                break
        except socket.timeout:
            continue
        except Exception as e:
            log.error(f"Waypoint receiver error: {e}")

    sock.close()


def main():
    global running

    # 1. Single MAVLink connection — shared via dispatcher
    master = connect_pixhawk()
    dispatcher.start(master)

    # 2. Waypoint listener
    wp_thread = threading.Thread(target=waypoint_receiver, daemon=True)
    wp_thread.start()

    # 3. UDP send socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log.info("Starting NAV data stream. Press Ctrl+C to stop.")

    dt = 0.01   # 100 Hz

    try:
        for nav_data in transmitData():
            if not running:
                break
            sock.sendto(json.dumps(nav_data).encode(), (BASE_IP, UDP_PORT_OUT))
            time.sleep(dt)

    except KeyboardInterrupt:
        running = False
        log.info("Stopped by user.")

    finally:
        sock.close()
        log.info("Socket closed.")


if __name__ == "__main__":
    main()