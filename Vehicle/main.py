import socket
import threading
import json
import logging

from imu import stream_imu

BASE_IP      = "127.0.0.1"
UDP_PORT_OUT = 5006
UDP_PORT_IN  = 5005

running   = True
waypoints = []

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger(__name__)

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

    wp_thread = threading.Thread(target=waypoint_receiver, daemon=True)
    wp_thread.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log.info("Starting Xsens IMU data stream. Press Ctrl+C to stop.")

    try:
        for imu_data in stream_imu():
            if not running:
                break
            
            # This line will print the data to your terminal so you know it's working
            log.info(f"Sending: Roll={imu_data['roll']:.1f}, Pitch={imu_data['pitch']:.1f}, Yaw={imu_data['yaw']:.1f}")
            
            sock.sendto(json.dumps(imu_data).encode(), (BASE_IP, UDP_PORT_OUT))

    except KeyboardInterrupt:
        running = False
        log.info("Stopped by user.")

    except Exception as e:
        log.error(f"Main loop error: {e}")

    finally:
        sock.close()
        log.info("Socket closed.")

if __name__ == "__main__":
    main()