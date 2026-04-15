# =============================================================================
# BASESTATION MAIN.PY  –  Ground Control Station Entry Point
# =============================================================================
# Communication flow with the Vehicle (both on the same Wi-Fi network):
#
#   ┌──────────────┐    UDP :5005    ┌──────────────┐
#   │  Base Station │ ─────────────→ │   Vehicle     │   (waypoints)
#   │  (this file)  │ ←───────────── │  (Vehicle/)   │   (telemetry)
#   └──────────────┘    UDP :5006    └──────────────┘
#
#   1. On startup, base station sends waypoints to the vehicle on port 5005.
#   2. Vehicle starts navigating and streams telemetry back on port 5006.
#   3. The telemetry listener thread parses each packet and updates the
#      shared `telemetry` dict that the GUI reads via HTTP polling.
#   4. The GUI web server (gui.py) runs on localhost:8080 and serves a
#      live dashboard with map, charts, and header stats.
#
# IMPORTANT – Port agreement:
#   Vehicle.UDP_PORT_IN  = 5005  ←  Basestation.UDP_PORT_OUT = 5005
#   Vehicle.UDP_PORT_OUT = 5006  →  Basestation.UDP_PORT_IN  = 5006
# =============================================================================

import socket       # UDP networking for sending waypoints and receiving telemetry
import threading    # Background threads for telemetry listener and GUI server
import json         # Serialising waypoint packets and parsing telemetry JSON
import time         # Sleep in the main loop and between waypoint retries
import logging      # Structured logging to info.log

import gui          # The web-based dashboard module (serves HTML + /telemetry JSON endpoint)

# ── Import all constants from the central config file ────────────────────────
from constants import (
    # Networking  (constants.py → NETWORKING section)
    ASV_IP, UDP_PORT_OUT, UDP_PORT_IN,        # vehicle IP, port to send waypoints, port to receive telemetry
    # Waypoint TX  (constants.py → WAYPOINT TRANSMISSION section)
    WP_SEND_RETRIES, WP_SEND_INTERVAL,       # how many times to re-send and the delay between sends
    # Telemetry RX  (constants.py → TELEMETRY LISTENER section)
    TELEM_RECV_TIMEOUT, TELEM_BUFFER_SIZE,    # socket timeout and max UDP datagram size
    # GUI  (constants.py → GUI section)
    GUI_HTTP_PORT,                            # HTTP port for the dashboard (default 8080)
    # Default waypoints  (constants.py → DEFAULT WAYPOINTS section)
    DEFAULT_WAYPOINTS,                        # list of (lat, lon) tuples
)

# ── Logging setup ────────────────────────────────────────────────────────────
# Logs go to a file so they persist across sessions
logging.basicConfig(
    filename="info.log",                                    # log file in the Basestation folder
    level=logging.INFO,                                     # log INFO and above
    format="%(asctime)s - [%(levelname)s] - %(message)s",   # timestamp + level + message
    datefmt="%Y-%m-%d %H:%M:%S",                            # human-readable date/time
)
log = logging.getLogger(__name__)  # logger for this module

# ── Shared telemetry state ───────────────────────────────────────────────────
# This dict is the bridge between the telemetry listener thread and the GUI.
# The listener WRITES to it, the GUI's /telemetry endpoint READS from it.
# In CPython, simple dict key assignments are atomic, so no lock is needed.
telemetry = {
    # ── GPS fields ───────────────────────────────────────────────────────
    "gps_lat":           0.0,     # decimal degrees latitude from the vehicle
    "gps_lon":           0.0,     # decimal degrees longitude from the vehicle
    "gps_alt":           0.0,     # altitude in metres
    "gps_fix":           0,       # fix type (0=none, 3=3D, 5=RTK float, 6=RTK fixed)
    "gps_sats":          0,       # number of satellites visible

    # ── IMU fields ───────────────────────────────────────────────────────
    "roll":              0.0,     # roll angle in degrees (nav frame)
    "pitch":             0.0,     # pitch angle in degrees (nav frame)
    "yaw":               0.0,     # yaw angle in degrees (nav frame)
    "accel_x":           0.0,     # X acceleration in m/s² (nav frame)
    "accel_y":           0.0,     # Y acceleration in m/s² (nav frame)
    "accel_z":           0.0,     # Z acceleration in m/s² (nav frame)

    # ── Navigation fields ────────────────────────────────────────────────
    "heading":           0.0,     # current heading from IMU yaw (degrees)
    "desired_bearing":   0.0,     # target heading from nav algorithm (degrees)
    "heading_error":     0.0,     # difference between target and actual heading (degrees)
    "cross_track_error": 0.0,     # perpendicular distance from the planned path (metres)
    "dist_to_waypoint":  0.0,     # distance to the look-ahead target point (metres)
    "active_wp":         0,       # current waypoint segment index (0-based)
    "nav_status":        "WAITING",  # status string: WAITING / NAVIGATING / DONE / ERROR
}

running = True   # global flag — set to False to stop all threads gracefully


# =============================================================================
# WAYPOINT SENDER  – pushes waypoints to the Vehicle over UDP
# =============================================================================
def send_waypoints(waypoints):
    """
    Sends a 'waypoints' JSON packet to the vehicle.
    UDP is unreliable (no delivery guarantee), so we send multiple copies
    spaced WP_SEND_INTERVAL seconds apart (default: 5 retries × 0.2s).
    The vehicle listens on port 5005 (UDP_PORT_OUT from our perspective).
    """
    # Create a throwaway UDP socket just for sending
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Build the JSON payload: the vehicle expects {"type": "waypoints", "waypoints": [[lat,lon], ...]}
    packet = {
        "type":      "waypoints",
        "waypoints": waypoints,     # list of (lat, lon) tuples
    }
    # Encode dict → JSON string → UTF-8 bytes
    data = json.dumps(packet).encode()

    # Send the packet multiple times for redundancy
    for attempt in range(WP_SEND_RETRIES):
        sock.sendto(data, (ASV_IP, UDP_PORT_OUT))   # send one copy to the vehicle
        time.sleep(WP_SEND_INTERVAL)                 # wait between retries

    print(f"[TX] Waypoints sent to {ASV_IP}:{UDP_PORT_OUT}  ({len(waypoints)} points)")
    log.info("Waypoints sent to %s:%d  (%d points)", ASV_IP, UDP_PORT_OUT, len(waypoints))
    sock.close()   # done sending — release the socket


# =============================================================================
# TELEMETRY LISTENER  – receives navigation state from the Vehicle over UDP
# =============================================================================
def telemetry_listener():
    """
    Runs in a background thread. Binds to UDP port 5006 and continuously
    receives telemetry JSON packets from the vehicle.

    Expected packet fields (sent by Vehicle/main.py → build_telemetry_packet):
      GPS:  gps_lat, gps_lon, gps_alt, gps_fix, gps_sats
      IMU:  roll, pitch, yaw, accel_x, accel_y, accel_z
      NAV:  heading, desired_bearing, heading_error, cross_track_error,
            dist_to_waypoint, active_wp, nav_status
    """
    # Create and configure the receive socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)      # UDP socket
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)   # allow reuse on restart
    sock.bind(("0.0.0.0", UDP_PORT_IN))                          # listen on all interfaces
    sock.settimeout(TELEM_RECV_TIMEOUT)                          # 1s timeout so we can check `running`

    print(f"[RX] Listening for telemetry on port {UDP_PORT_IN}...")
    log.info("Telemetry listener started on port %d", UDP_PORT_IN)

    while running:
        try:
            # Block until a UDP datagram arrives (up to TELEM_BUFFER_SIZE bytes)
            data, addr = sock.recvfrom(TELEM_BUFFER_SIZE)
            # Decode bytes → string → dict
            pkt = json.loads(data.decode())

            # ── Update the shared telemetry dict with GPS values ─────────
            telemetry["gps_lat"]  = pkt.get("gps_lat",  0.0)   # latitude from the vehicle
            telemetry["gps_lon"]  = pkt.get("gps_lon",  0.0)   # longitude from the vehicle
            telemetry["gps_alt"]  = pkt.get("gps_alt",  0.0)   # altitude from the vehicle
            telemetry["gps_fix"]  = pkt.get("gps_fix",  0)     # fix type code
            telemetry["gps_sats"] = pkt.get("gps_sats", 0)     # satellite count

            # ── Update IMU values ────────────────────────────────────────
            telemetry["roll"]    = pkt.get("roll",    0.0)      # roll angle (degrees)
            telemetry["pitch"]   = pkt.get("pitch",   0.0)      # pitch angle (degrees)
            telemetry["yaw"]     = pkt.get("yaw",     0.0)      # yaw angle (degrees)
            telemetry["accel_x"] = pkt.get("accel_x", 0.0)     # X acceleration (m/s²)
            telemetry["accel_y"] = pkt.get("accel_y", 0.0)     # Y acceleration (m/s²)
            telemetry["accel_z"] = pkt.get("accel_z", 0.0)     # Z acceleration (m/s²)

            # ── Update navigation values ─────────────────────────────────
            telemetry["heading"]           = pkt.get("heading",           0.0)   # current heading (°)
            telemetry["desired_bearing"]   = pkt.get("desired_bearing",   0.0)   # target heading (°)
            telemetry["heading_error"]     = pkt.get("heading_error",     0.0)   # heading error (°)
            telemetry["cross_track_error"] = pkt.get("cross_track_error", 0.0)   # lateral error (m)
            telemetry["dist_to_waypoint"]  = pkt.get("dist_to_waypoint",  0.0)   # distance to target (m)
            telemetry["active_wp"]         = pkt.get("active_wp",         0)     # segment index
            telemetry["nav_status"]        = pkt.get("nav_status",        "UNKNOWN")  # status string

            # ── Print a single-line summary to the console ───────────────
            # \r = carriage return → overwrites the same line each update
            print(
                f"\r[{telemetry['nav_status']:>10s}] "                              # right-aligned status
                f"GPS({telemetry['gps_lat']:.6f}, {telemetry['gps_lon']:.6f}) "     # lat/lon
                f"Hdg:{telemetry['heading']:>+7.1f} -> {telemetry['desired_bearing']:>+7.1f} "  # actual -> target
                f"Err:{telemetry['heading_error']:>+7.2f}° "                        # heading error
                f"XTE:{telemetry['cross_track_error']:>+6.2f}m "                    # cross-track error
                f"WP:{telemetry['active_wp']} "                                     # waypoint index
                f"Dist:{telemetry['dist_to_waypoint']:.1f}m "                       # distance to target
                f"R:{telemetry['roll']:>+6.1f} P:{telemetry['pitch']:>+6.1f}",      # roll / pitch
                end="", flush=True,  # don't add newline, flush immediately
            )

        except socket.timeout:
            continue   # timeout is normal — just loop back and try again

        except json.JSONDecodeError as e:
            # Received data that isn't valid JSON — log it and move on
            log.warning("Received malformed telemetry packet: %s", e)

        except Exception as e:
            # Any other error — log it and continue
            print(f"\n[RX] Error: {e}")
            log.error("Error in telemetry listener: %s", str(e))

    sock.close()   # release the socket when the loop exits


# =============================================================================
# MAIN  –  orchestrates the base station startup sequence
# =============================================================================
def main():
    global running

    # The default waypoints are pre-loaded on the planning map.
    # The user can edit them before starting the mission.
    default_waypoints = DEFAULT_WAYPOINTS

    # ── 1. Start telemetry listener IMMEDIATELY ──────────────────────────
    # This runs from the start so the dashboard shows vehicle data
    # even while the user is still on the planning page.
    listener = threading.Thread(target=telemetry_listener, daemon=True)
    listener.start()

    # ── 2. Callback for when user clicks "Start Mission" ─────────────────
    def on_waypoints_selected(wps):
        """
        Called by gui.py when the user submits waypoints from the planning page.
        :param wps: list of (lat, lon) tuples selected by the user
        """
        print(f"\n[MISSION] User selected {len(wps)} waypoints. Sending to vehicle...")
        log.info("User selected %d waypoints - sending to vehicle", len(wps))

        # Send waypoints to the vehicle via UDP
        send_waypoints(wps)

        print("[MISSION] Waypoints sent. Dashboard is live.")
        log.info("Waypoints sent after mission launch")

    # ── 3. Launch the GUI in a background thread ─────────────────────────
    # In planning mode: shows the interactive map for waypoint selection.
    # After the user clicks "Start Mission", on_waypoints_selected() fires,
    # which sends the waypoints to the vehicle.
    # The page then redirects to /dashboard for live monitoring.
    gui_thread = threading.Thread(
        target=gui.show,
        args=(default_waypoints, telemetry),
        kwargs={"port": GUI_HTTP_PORT, "on_waypoints": on_waypoints_selected},
        daemon=True,
    )
    gui_thread.start()

    # ── 4. Keep the main thread alive until Ctrl+C ───────────────────────
    try:
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        running = False
        print("\n[INFO] Stopped by user.")
        log.info("Basestation stopped by user.")


# ── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()