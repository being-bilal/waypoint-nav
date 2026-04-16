# =============================================================================
# MAIN.PY  –  Vehicle Entry Point
# =============================================================================
# Stitches together GPS, IMU, Navigation, and Control into a single run loop.
#
#   1. Receives waypoints from the base station over UDP  (networking)
#   2. Initialises GPS (Pixhawk) and IMU (Xsens)          (sensors)
#   3. Runs the Navigator to compute heading/cross-track   (nav algorithm)
#   4. Feeds yaw error into the PID boat controller        (control)
#   5. Streams live telemetry back to the base station     (networking)
# =============================================================================

import socket       # UDP networking for receiving waypoints and sending telemetry
import threading    # Background thread for the waypoint listener
import json         # Serialising/deserialising waypoint and telemetry JSON packets
import logging      # Structured log messages
import time         # Sleep in the main loop

# ── Import constants from the central config file ────────────────────────────
from constants import (
    # Networking / comms  (constants.py → NETWORKING section)
    BASE_IP, UDP_PORT_OUT, UDP_PORT_IN,       # base station IP, ports for telemetry out and waypoints in
    # GPS  (constants.py → GPS section)
    GPS_CONNECTION_STRING, GPS_BAUD_RATE,      # Pixhawk serial port and baud rate
    # IMU  (constants.py → IMU section)
    IMU_TARGET_PORT, IMU_HZ,                  # Xsens serial port and output rate
    # Navigation  (constants.py → NAVIGATION section)
    WAYPOINT_FILE, NAV_LOOP_RATE,             # waypoint JSON path and loop sleep time
)
# ── Import the four subsystem classes ────────────────────────────────────────
from gps import GPS                           # reads lat/lon from Pixhawk GPS
from imu import SimpleXsens                   # reads roll/pitch/yaw from Xsens MTi
from nav import Navigator                     # waypoint-following algorithm
from control import PixhawkBoatController     # PID + thrust allocation + MAVLink PWM

# ── Logging setup ────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger(__name__)  # logger for this module

# ── Global state ─────────────────────────────────────────────────────────────
running   = True                        # set to False to stop all loops gracefully
waypoints_received = threading.Event()  # signalled when waypoints are available


# =============================================================================
# WAYPOINT RECEIVER  – listens for waypoints from the base station via UDP
# =============================================================================
def waypoint_receiver():
    """
    Runs in a background thread. Blocks on a UDP socket waiting for the
    base station to send a 'waypoints' packet. Saves them to WAYPOINT_FILE
    and sets the waypoints_received event so the main thread can proceed.
    """
    global running

    # Create a UDP socket (SOCK_DGRAM = UDP, not TCP)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Allow multiple processes to bind to the same port (useful during restarts)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to all interfaces (0.0.0.0) on the waypoint port
    sock.bind(("0.0.0.0", UDP_PORT_IN))
    # Set a 1-second timeout so the loop can check the `running` flag periodically
    sock.settimeout(1.0)
    log.info(f"Listening for waypoints on port {UDP_PORT_IN}...")

    while running:
        try:
            # Block until we receive a UDP datagram (up to 4096 bytes)
            data, _ = sock.recvfrom(4096)
            # Decode the raw bytes to a string, then parse as JSON
            pkt = json.loads(data.decode())

            # Only process packets with type="waypoints"
            if pkt.get("type") == "waypoints":
                # Extract the waypoint list as (lat, lon) tuples
                wps = [
                    (float(lat), float(lon))
                    for lat, lon in pkt.get("waypoints", [])
                ]
                log.info(f"Received {len(wps)} waypoints.")

                # Save to disk so the Navigator can load them
                with open(WAYPOINT_FILE, "w") as f:
                    json.dump(
                        {"waypoints": [{"lat": lat, "lon": lon} for lat, lon in wps]},
                        f, indent=4,
                    )
                log.info("Waypoints saved to %s.", WAYPOINT_FILE)
                waypoints_received.set()  # signal the main thread that waypoints are ready
                break                     # our job is done — exit the listener loop

        except socket.timeout:
            continue        # timed out — go back and check `running`, then try again
        except Exception as e:
            log.error(f"Waypoint receiver error: {e}")

    sock.close()  # release the socket


# =============================================================================
# TELEMETRY  – builds and sends navigation data to the base station over UDP
# =============================================================================
def build_telemetry_packet(nav_data: dict, gps_obj, navigator) -> dict:
    """
    Build a flat, JSON-safe telemetry dict that the Base Station can consume.
    
    Maps internal nav fields → base station expected fields:
      - gps_lat / gps_lon          ← GPS.get()               (geographic position)
      - heading / actual_yaw       ← IMU yaw                  (current heading)
      - desired_bearing / target_yaw  ← nav algorithm          (where we should face)
      - heading_error / yaw_error  ← difference                (how far off we are)
      - cross_track_error          ← perpendicular to path     (lateral offset)
      - active_wp                  ← Navigator index           (which segment we're on)
      - dist_to_waypoint           ← Euclidean P→T distance    (how far to the target point)
      - roll / pitch               ← IMU data                  (boat tilt)
      - nav_status                 ← NAVIGATING / WAITING / DONE
    """
    import math

    # GPS geographic position — the GUI map needs lat/lon, not UTM coords
    geo = gps_obj.get()  # returns {"lat": ..., "lon": ..., "alt": ..., ...}

    # Extract the IMU sub-dict (may be None if not navigating yet)
    imu_data = nav_data.get("imu_data") or {}

    # Calculate Euclidean distance from current position P to look-ahead target T
    pos    = nav_data.get("pos")       # current position [x, y] in local UTM metres
    target = nav_data.get("target")    # target point [x, y] in local UTM metres
    if pos is not None and target is not None:
        dx = float(target[0] - pos[0])     # x-distance to target
        dy = float(target[1] - pos[1])     # y-distance to target
        dist = math.sqrt(dx * dx + dy * dy)  # straight-line distance
    else:
        dist = 0.0  # no position data available yet

    # Assemble the flat packet dict with all fields the base station GUI expects
    packet = {
        # ── GPS (geographic coordinates for the map) ─────────────────────
        "gps_lat":           geo["lat"],           # decimal degrees latitude
        "gps_lon":           geo["lon"],           # decimal degrees longitude
        "gps_alt":           geo["alt"],           # altitude in metres
        "gps_fix":           geo["fix_type"],      # fix quality (3=3D, 5=RTK float, etc.)
        "gps_sats":          geo["satellites"],    # number of visible satellites

        # ── IMU (orientation and acceleration) ───────────────────────────
        "roll":              imu_data.get("roll", 0.0),     # degrees, nav frame
        "pitch":             imu_data.get("pitch", 0.0),    # degrees, nav frame
        "yaw":               imu_data.get("yaw", 0.0),      # degrees, nav frame
        "accel_x":           imu_data.get("accel_x", 0.0),  # m/s², nav frame
        "accel_y":           imu_data.get("accel_y", 0.0),  # m/s², nav frame
        "accel_z":           imu_data.get("accel_z", 0.0),  # m/s², nav frame

        # ── Navigation state ─────────────────────────────────────────────
        "heading":           nav_data.get("actual_yaw", 0.0),       # current heading (°)
        "desired_bearing":   nav_data.get("target_yaw", 0.0),       # target heading (°)
        "heading_error":     nav_data.get("yaw_error", 0.0),        # heading error (°)
        "cross_track_error": nav_data.get("xtrack_error", 0.0),     # lateral error (m)
        "dist_to_waypoint":  round(dist, 3),                        # distance to target (m)
        "active_wp":         navigator.current_wp_index,            # current segment index
        "target_lat":        nav_data.get("target_lat", 0.0),       # Target latitude
        "target_lon":        nav_data.get("target_lon", 0.0),       # Target longitude
        "nav_status":        nav_data.get("status", "UNKNOWN"),     # status string
    }
    return packet


def send_telemetry(sock, packet: dict):
    """Serialise the telemetry packet to JSON and send it to the base station via UDP."""
    try:
        # Encode dict → JSON string → UTF-8 bytes, then send via UDP
        sock.sendto(json.dumps(packet).encode(), (BASE_IP, UDP_PORT_OUT))
    except Exception as e:
        log.error(f"Telemetry send error: {e}")


# =============================================================================
# MAIN  –  the orchestrator that ties everything together
# =============================================================================
def main():
    global running

    # ── 1. Start the waypoint listener in a background thread ────────────
    # This thread blocks until the base station sends waypoints over UDP
    wp_thread = threading.Thread(target=waypoint_receiver, daemon=True)
    wp_thread.start()

    # ── 2. Initialise sensors ────────────────────────────────────────────
    # GPS talks to the Pixhawk flight controller over USB serial
    log.info("Initialising GPS (Pixhawk) on %s ...", GPS_CONNECTION_STRING)
    gps = GPS(connection_string=GPS_CONNECTION_STRING, baud=GPS_BAUD_RATE)

    # IMU talks to the Xsens MTi sensor over USB serial
    log.info("Initialising IMU (Xsens) on %s @ %d Hz ...", IMU_TARGET_PORT, IMU_HZ)
    imu = SimpleXsens(hz=IMU_HZ, target_port=IMU_TARGET_PORT)

    # ── 3. Wait for waypoints (from file or UDP) ────────────────────────
    log.info("Waiting for waypoints (file or UDP) ...")
    # Check if we already have a valid waypoints file from a previous run
    try:
        with open(WAYPOINT_FILE, "r") as f:
            existing = json.load(f)
            # Need at least 2 waypoints to define one path segment
            if len(existing.get("waypoints", [])) >= 2:
                log.info("Loaded existing %s with %d waypoints.",
                         WAYPOINT_FILE, len(existing["waypoints"]))
                waypoints_received.set()   # skip waiting for UDP
    except (FileNotFoundError, json.JSONDecodeError):
        pass  # no valid file — we'll wait for the base station to send them

    # Block here until waypoints are available (either from the file check above
    # or from the UDP listener thread)
    waypoints_received.wait()

    # ── 4. Initialise the Navigator and Controller ───────────────────────
    # Navigator receives the GPS and IMU instances (dependency injection)
    navigator  = Navigator(gps, imu, waypoint_file=WAYPOINT_FILE)
    # Controller shares the SAME MAVLink connection that GPS already opened
    # (opening /dev/ttyACM0 twice causes a serial port conflict)
    controller = PixhawkBoatController(master=gps.master)

    # ── 5. Open a UDP socket for outgoing telemetry to the base station ──
    telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log.info("Navigation loop starting. Press Ctrl+C to stop.")

    # ── 6. MAIN LOOP  –  runs at ~100 Hz ────────────────────────────────
    try:
        while running:
            # Run one iteration of the navigation algorithm
            nav = navigator.calculate_navigation()
            status = nav.get("status")

            # ── DONE: all waypoints reached ──────────────────────────────
            if status == "DONE":
                log.info("🎉 DESTINATION REACHED! Navigation complete.")
                break

            # ── WAITING: sensors not ready yet ───────────────────────────
            elif status == "WAITING":
                # Show green/red indicators for which sensors have data
                g_flag = "G" if nav.get("gps_updated") or navigator.last_pos is not None else "-"
                i_flag = "I" if nav.get("imu_updated") or navigator.last_yaw is not None else "-"

                msg = f"[{g_flag}{i_flag}] WAITING for sensors | GPS: {g_flag} | IMU: {i_flag}"
                # If IMU is already streaming, show its values while we wait for GPS
                if navigator.last_imu_data is not None:
                    imu_d = navigator.last_imu_data
                    msg += (f" | IMU -> R:{imu_d['roll']:>6.1f} "
                            f"P:{imu_d['pitch']:>6.1f} Y:{imu_d['yaw']:>6.1f}")
                # \r = carriage return -> overwrite the same line (no scroll)
                print(msg.ljust(120), end="\r")

                # Stream whatever data we have to the base station so the
                # dashboard shows live IMU readings even before GPS locks on.
                # build_telemetry_packet handles missing nav fields gracefully.
                waiting_nav = {
                    "status": "WAITING",
                    "imu_data": navigator.last_imu_data,   # may be None if IMU hasn't fired yet
                    "actual_yaw": navigator.last_yaw or 0.0,
                }
                telem_packet = build_telemetry_packet(waiting_nav, gps, navigator)
                send_telemetry(telem_sock, telem_packet)

            # ── NAVIGATING: full control loop ────────────────────────────
            elif status == "NAVIGATING":
                # Feed the yaw error from the navigator into the PID controller
                # → controller computes thrust allocation → sends PWM to Pixhawk
                yaw_error = nav["yaw_error"]
                pwm_left, pwm_right = controller.update(yaw_error)

                # Print a scrolling status line showing all key values
                g_flag = "G" if nav["gps_updated"] else "-"   # G = fresh GPS this cycle
                i_flag = "I" if nav["imu_updated"] else "-"   # I = fresh IMU this cycle
                imu_d  = nav["imu_data"]

                print(
                    f"[{g_flag}{i_flag}] WP {navigator.current_wp_index}"
                    f"→{navigator.current_wp_index + 1} | "
                    f"Yaw Err: {yaw_error:>7.2f}° | "
                    f"Target: {nav['target_yaw']:>6.1f}° | "
                    f"X-Track: {nav['xtrack_error']:>6.2f}m | "
                    f"PWM L:{pwm_left} R:{pwm_right} | "
                    f"IMU → R:{imu_d['roll']:>6.1f} P:{imu_d['pitch']:>6.1f} Y:{imu_d['yaw']:>6.1f}"
                )

                # Build the telemetry packet and stream it to the base station
                telem_packet = build_telemetry_packet(nav, gps, navigator)
                send_telemetry(telem_sock, telem_packet)

            # ── WAYPOINT_SWITCH: advanced to the next path segment ───────
            elif status == "WAYPOINT_SWITCH":
                log.info("Switched to next waypoint segment.")

            # ── ERROR: something went wrong in the nav calculation ───────
            elif status == "ERROR":
                log.error("Nav error: %s", nav.get("msg", "unknown"))

            # Sleep to maintain the target loop rate (~100 Hz = 0.01s)
            time.sleep(NAV_LOOP_RATE)

    except KeyboardInterrupt:
        running = False
        log.info("Stopped by user.")

    except Exception as e:
        log.error(f"Main loop error: {e}")
        running = False

    finally:
        # ── Cleanup: release all hardware resources ──────────────────────
        log.info("Shutting down ...")
        gps.stop()           # stop the GPS background thread
        imu.close()          # release the Xsens serial port
        telem_sock.close()   # close the UDP socket
        log.info("All resources released. Goodbye.")


# ── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()