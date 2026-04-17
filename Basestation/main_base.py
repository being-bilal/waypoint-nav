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
import platform     # ICMP ping command differs by OS
import shutil       # Locate ping binary
import subprocess   # Run ICMP probe after waypoint TX

import gui          # The web-based dashboard module (serves HTML + /telemetry JSON endpoint)

# ── Import all constants from the central config file ────────────────────────
from constants_base import (
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
logging.basicConfig(
    filename="info.log",
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger(__name__)

# ── Shared telemetry state ───────────────────────────────────────────────────
telemetry = {
    # ── GPS fields ───────────────────────────────────────────────────────
    "gps_lat":           0.0,
    "gps_lon":           0.0,
    "gps_alt":           0.0,
    "gps_fix":           0,
    "gps_sats":          0,

    # ── IMU fields ───────────────────────────────────────────────────────
    "roll":              0.0,
    "pitch":             0.0,
    "yaw":               0.0,
    "accel_x":           0.0,
    "accel_y":           0.0,
    "accel_z":           0.0,

    # ── Navigation fields ────────────────────────────────────────────────
    "heading":           0.0,
    "desired_bearing":   0.0,
    "heading_error":     0.0,
    "cross_track_error": 0.0,
    "dist_to_waypoint":  0.0,
    "target_lat":        0.0,
    "target_lon":        0.0,
    "active_wp":         0,
    "nav_status":        "WAITING",
}

running = True   # global flag — set to False to stop all threads gracefully

# ── Telemetry tracking (for change-detection logging) ───────────────────────
_last_nav_status = None   # detect nav_status transitions (e.g. WAITING → NAVIGATING)
_last_active_wp  = None   # detect waypoint advances
_last_gps_fix    = None   # detect GPS fix quality changes
_packet_count    = 0      # total packets received — logged periodically
_bad_packet_count = 0     # total malformed packets received
_last_periodic_log = 0.0  # timestamp of last periodic telemetry log

# Telemetry gap: ERROR repeats every ALERT_INTERVAL_S while no valid packet for TELEMETRY_GAP_THRESHOLD_S
TELEMETRY_GAP_THRESHOLD_S = 5.0
ALERT_INTERVAL_S = 30.0
_listener_start_mono = None
_last_valid_telemetry_mono = None
_last_telemetry_gap_error_log_mono = None

_vehicle_watch_started = False
_vehicle_watch_lock = threading.Lock()
_vehicle_unreachable_error_active = False


def log_sensor_status_at_startup():
    """Log current telemetry dict (sensor fields) once at program start — values are defaults until RX."""
    fix_names = {0: "NO FIX", 3: "3D FIX", 5: "RTK FLOAT", 6: "RTK FIXED"}
    gf = telemetry["gps_fix"]
    fix_label = fix_names.get(gf, f"UNKNOWN ({gf})")
    log.info(
        "Sensor status — GPS: lat=%.7f lon=%.7f alt=%.2fm fix=%s sats=%d",
        telemetry["gps_lat"],
        telemetry["gps_lon"],
        telemetry["gps_alt"],
        fix_label,
        telemetry["gps_sats"],
    )
    log.info(
        "Sensor status — IMU: roll=%.2f° pitch=%.2f° yaw=%.2f°  accel=(%.3f, %.3f, %.3f)",
        telemetry["roll"],
        telemetry["pitch"],
        telemetry["yaw"],
        telemetry["accel_x"],
        telemetry["accel_y"],
        telemetry["accel_z"],
    )
    log.info(
        "Sensor status — NAV: heading=%.2f° desired_bearing=%.2f° heading_error=%.2f° "
        "XTE=%.2fm dist_to_waypoint=%.1fm active_wp=%d nav_status=%s "
        "target_lat=%.7f target_lon=%.7f",
        telemetry["heading"],
        telemetry["desired_bearing"],
        telemetry["heading_error"],
        telemetry["cross_track_error"],
        telemetry["dist_to_waypoint"],
        telemetry["active_wp"],
        telemetry["nav_status"],
        telemetry["target_lat"],
        telemetry["target_lon"],
    )


def _check_telemetry_gap():
    """While no valid telemetry for TELEMETRY_GAP_THRESHOLD_S, log ERROR every ALERT_INTERVAL_S."""
    global _last_telemetry_gap_error_log_mono
    now = time.monotonic()
    ref = (
        _last_valid_telemetry_mono
        if _last_valid_telemetry_mono is not None
        else _listener_start_mono
    )
    if ref is None:
        return
    elapsed = now - ref
    if elapsed >= TELEMETRY_GAP_THRESHOLD_S:
        if _last_telemetry_gap_error_log_mono is None or (
            now - _last_telemetry_gap_error_log_mono
        ) >= ALERT_INTERVAL_S:
            log.error(
                "Lost contact with the vehicle. "
                "Check that the vehicle is on and in range.",
            )
            _last_telemetry_gap_error_log_mono = now
    else:
        _last_telemetry_gap_error_log_mono = None


def _vehicle_host_reachable_icmp(ip):
    """
    Best-effort ICMP ping to the vehicle IP (ASV_IP from constants_base).
    Returns True if ping succeeds, False if it fails, None if ping could not be run.
    UDP does not confirm delivery; this is a separate L3 reachability hint.
    """
    ping_bin = shutil.which("ping")
    if not ping_bin:
        return None
    system = platform.system().lower()
    try:
        if system == "windows":
            cmd = [ping_bin, "-n", "1", "-w", "1000", ip]
        elif system == "darwin":
            cmd = [ping_bin, "-c", "1", "-W", "1000", ip]
        else:
            cmd = [ping_bin, "-c", "1", "-W", "1", ip]
        r = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        return r.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return None


def _vehicle_unreachable_watch_loop():
    """Background: ICMP ping ASV_IP every ALERT_INTERVAL_S; ERROR while host is unreachable."""
    global _vehicle_unreachable_error_active
    while running:
        time.sleep(ALERT_INTERVAL_S)
        if not running:
            break
        r = _vehicle_host_reachable_icmp(ASV_IP)
        if r is False:
            _vehicle_unreachable_error_active = True
            log.error(
                "vehicle unreachable. It may be off or out of range.",
            )
        elif r is True:
            if _vehicle_unreachable_error_active:
                log.info(
                    "Success: Connection to the vehicle restored.",
                )
                _vehicle_unreachable_error_active = False


def _start_vehicle_unreachable_watch_if_needed():
    """Start a single daemon thread after first waypoint send to repeat unreachable ERRORs."""
    global _vehicle_watch_started
    with _vehicle_watch_lock:
        if _vehicle_watch_started:
            return
        _vehicle_watch_started = True
    threading.Thread(
        target=_vehicle_unreachable_watch_loop,
        daemon=True,
        name="VehicleHostWatch",
    ).start()


# =============================================================================
# WAYPOINT SENDER  – pushes waypoints to the Vehicle over UDP
# =============================================================================
def send_waypoints(waypoints):
    """
    Sends a 'waypoints' JSON packet to the vehicle.
    UDP is unreliable (no delivery guarantee), so we send multiple copies
    spaced WP_SEND_INTERVAL seconds apart.
    """
    global _vehicle_unreachable_error_active
    log.info(
        "Preparing to send %d waypoints to %s:%d  (retries=%d, interval=%.2fs)",
        len(waypoints), ASV_IP, UDP_PORT_OUT, WP_SEND_RETRIES, WP_SEND_INTERVAL,
    )

    # Log each waypoint coordinate for traceability
    for i, (lat, lon) in enumerate(waypoints):
        log.info("  WP[%d]: lat=%.7f  lon=%.7f", i, lat, lon)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = {
        "type":      "waypoints",
        "waypoints": waypoints,
    }
    data = json.dumps(packet).encode()
    log.info("Waypoint packet encoded — %d bytes", len(data))

    for attempt in range(WP_SEND_RETRIES):
        try:
            sock.sendto(data, (ASV_IP, UDP_PORT_OUT))
            log.info("Waypoint TX attempt %d/%d → %s:%d", attempt + 1, WP_SEND_RETRIES, ASV_IP, UDP_PORT_OUT)
        except OSError as e:
            log.error("Waypoint TX attempt %d/%d failed: %s", attempt + 1, WP_SEND_RETRIES, e)
        time.sleep(WP_SEND_INTERVAL)

    print(f"[TX] Waypoints sent to {ASV_IP}:{UDP_PORT_OUT}  ({len(waypoints)} points)")
    log.info("All waypoint TX attempts complete — %d points sent", len(waypoints))

    reachable = _vehicle_host_reachable_icmp(ASV_IP)
    if reachable is False:
        log.error(
            "Waypoints sent but vehicle host %s (ASV_IP) appears unreachable — ICMP ping failed; "
            "UDP datagrams were transmitted but the vehicle may be down or on another network.",
            ASV_IP,
        )
    elif reachable is None:
        log.debug("Vehicle reachability not verified (ping unavailable or failed to run)")

    _start_vehicle_unreachable_watch_if_needed()

    sock.close()
    log.debug("Waypoint TX socket closed")


# =============================================================================
# TELEMETRY LISTENER  – receives navigation state from the Vehicle over UDP
# =============================================================================
def telemetry_listener():
    """
    Runs in a background thread. Binds to UDP port 5006 and continuously
    receives telemetry JSON packets from the vehicle.
    """
    global _last_nav_status, _last_active_wp, _last_gps_fix
    global _packet_count, _bad_packet_count, _last_periodic_log
    global _listener_start_mono, _last_valid_telemetry_mono, _last_telemetry_gap_error_log_mono

    log.info("Telemetry listener thread starting — binding to 0.0.0.0:%d", UDP_PORT_IN)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind(("0.0.0.0", UDP_PORT_IN))
        log.info("Telemetry socket bound successfully on port %d", UDP_PORT_IN)
    except OSError as e:
        log.critical("Failed to bind telemetry socket on port %d: %s", UDP_PORT_IN, e)
        print(f"[RX] CRITICAL: Cannot bind to port {UDP_PORT_IN}: {e}")
        return

    sock.settimeout(TELEM_RECV_TIMEOUT)

    _listener_start_mono = time.monotonic()
    _last_valid_telemetry_mono = None
    _last_telemetry_gap_error_log_mono = None

    print(f"[RX] Listening for telemetry on port {UDP_PORT_IN}...")
    log.info("Telemetry listener ready — timeout=%.1fs, buffer=%d bytes", TELEM_RECV_TIMEOUT, TELEM_BUFFER_SIZE)

    while running:
        try:
            data, addr = sock.recvfrom(TELEM_BUFFER_SIZE)
            _packet_count += 1

            # ── Log first-ever packet (confirms comms link is up) ────────
            if _packet_count == 1:
                log.info("First telemetry packet received from %s:%d — comms link established", addr[0], addr[1])


            pkt = json.loads(data.decode())

            # ── Update shared telemetry dict ─────────────────────────────
            telemetry["gps_lat"]  = pkt.get("gps_lat",  0.0)
            telemetry["gps_lon"]  = pkt.get("gps_lon",  0.0)
            telemetry["gps_alt"]  = pkt.get("gps_alt",  0.0)
            telemetry["gps_fix"]  = pkt.get("gps_fix",  0)
            telemetry["gps_sats"] = pkt.get("gps_sats", 0)

            telemetry["roll"]    = pkt.get("roll",    0.0)
            telemetry["pitch"]   = pkt.get("pitch",   0.0)
            telemetry["yaw"]     = pkt.get("yaw",     0.0)
            telemetry["accel_x"] = pkt.get("accel_x", 0.0)
            telemetry["accel_y"] = pkt.get("accel_y", 0.0)
            telemetry["accel_z"] = pkt.get("accel_z", 0.0)

            telemetry["heading"]           = pkt.get("heading",           0.0)
            telemetry["desired_bearing"]   = pkt.get("desired_bearing",   0.0)
            telemetry["heading_error"]     = pkt.get("heading_error",     0.0)
            telemetry["cross_track_error"] = pkt.get("cross_track_error", 0.0)
            telemetry["dist_to_waypoint"]  = pkt.get("dist_to_waypoint",  0.0)
            telemetry["target_lat"]        = pkt.get("target_lat",        0.0)
            telemetry["target_lon"]        = pkt.get("target_lon",        0.0)
            telemetry["active_wp"]         = pkt.get("active_wp",         0)
            telemetry["nav_status"]        = pkt.get("nav_status",        "UNKNOWN")

            had_gap_errors = _last_telemetry_gap_error_log_mono is not None
            _last_valid_telemetry_mono = time.monotonic()
            _last_telemetry_gap_error_log_mono = None
            if had_gap_errors:
                log.info("Telemetry stream resumed after gap (valid packet received)")

            # ── Log nav_status transitions ───────────────────────────────
            current_status = telemetry["nav_status"]
            if current_status != _last_nav_status:
                log.info(
                    "Nav status changed: %s → %s  (WP=%d, dist=%.1fm)",
                    _last_nav_status, current_status,
                    telemetry["active_wp"], telemetry["dist_to_waypoint"],
                )
                _last_nav_status = current_status

            # ── Log waypoint advances ────────────────────────────────────
            current_wp = telemetry["active_wp"]
            if current_wp != _last_active_wp:
                log.info(
                    "Waypoint advanced: %s → %d  (GPS=%.6f, %.6f  dist=%.1fm)",
                    _last_active_wp, current_wp,
                    telemetry["gps_lat"], telemetry["gps_lon"],
                    telemetry["dist_to_waypoint"],
                )
                _last_active_wp = current_wp

            # ── Log GPS fix quality changes ──────────────────────────────
            current_fix = telemetry["gps_fix"]
            if current_fix != _last_gps_fix:
                fix_names = {0: "NO FIX", 3: "3D FIX", 5: "RTK FLOAT", 6: "RTK FIXED"}
                fix_label = fix_names.get(current_fix, f"UNKNOWN ({current_fix})")
                log.info(
                    "GPS fix changed: %s → %s  (sats=%d)",
                    _last_gps_fix, fix_label, telemetry["gps_sats"],
                )
                _last_gps_fix = current_fix

            # ── Periodic snapshot log every 30 seconds ───────────────────
            now = time.time()
            if now - _last_periodic_log >= 30.0:
                log.info(
                    "Telemetry snapshot — pkts=%d  bad=%d  "
                    "GPS(%.6f, %.6f, alt=%.1fm, fix=%d, sats=%d)  "
                    "IMU(roll=%.1f, pitch=%.1f, yaw=%.1f)  "
                    "NAV(hdg=%.1f→%.1f, err=%.2f°, XTE=%.2fm, dist=%.1fm, WP=%d, status=%s)  "
                    "target(%.6f, %.6f)",
                    _packet_count, _bad_packet_count,
                    telemetry["gps_lat"], telemetry["gps_lon"], telemetry["gps_alt"],
                    telemetry["gps_fix"], telemetry["gps_sats"],
                    telemetry["roll"], telemetry["pitch"], telemetry["yaw"],
                    telemetry["heading"], telemetry["desired_bearing"],
                    telemetry["heading_error"], telemetry["cross_track_error"],
                    telemetry["dist_to_waypoint"], telemetry["active_wp"],
                    telemetry["nav_status"],
                    telemetry["target_lat"], telemetry["target_lon"],
                )
                _last_periodic_log = now

            # ── Warn on large cross-track error (> 5 m) ──────────────────
            xte = abs(telemetry["cross_track_error"])
            if xte > 5.0:
                log.warning(
                    "Large cross-track error: %.2fm  (hdg=%.1f°, desired=%.1f°, WP=%d)",
                    xte, telemetry["heading"], telemetry["desired_bearing"], telemetry["active_wp"],
                )

            # ── Console single-line summary ───────────────────────────────
            print(
                f"\r[{telemetry['nav_status']:>10s}] "
                f"GPS({telemetry['gps_lat']:.6f}, {telemetry['gps_lon']:.6f}) "
                f"Hdg:{telemetry['heading']:>+7.1f} -> {telemetry['desired_bearing']:>+7.1f} "
                f"Err:{telemetry['heading_error']:>+7.2f}° "
                f"XTE:{telemetry['cross_track_error']:>+6.2f}m "
                f"WP:{telemetry['active_wp']} "
                f"Dist:{telemetry['dist_to_waypoint']:.1f}m "
                f"R:{telemetry['roll']:>+6.1f} P:{telemetry['pitch']:>+6.1f}",
                end="", flush=True,
            )

        except socket.timeout:
            _check_telemetry_gap()
            continue

        except json.JSONDecodeError as e:
            _bad_packet_count += 1
            log.warning(
                "Malformed telemetry packet #%d (total bad=%d): %s",
                _packet_count, _bad_packet_count, e,
            )
            _check_telemetry_gap()

        except Exception as e:
            print(f"\n[RX] Error: {e}")
            log.error("Unexpected error in telemetry listener: %s", str(e), exc_info=True)
            _check_telemetry_gap()

    log.info(
        "Telemetry listener stopping — total packets received: %d  bad: %d",
        _packet_count, _bad_packet_count,
    )
    sock.close()
    log.info("Telemetry socket closed")


# =============================================================================
# MAIN  –  orchestrates the base station startup sequence
# =============================================================================
def main():
    global running

    log.info("=" * 60)
    log.info("Basestation starting up")
    log_sensor_status_at_startup()
    log.info("Config — vehicle: %s  TX port: %d  RX port: %d  GUI port: %d",
             ASV_IP, UDP_PORT_OUT, UDP_PORT_IN, GUI_HTTP_PORT)
    log.info("Default waypoints loaded: %d points", len(DEFAULT_WAYPOINTS))
    for i, (lat, lon) in enumerate(DEFAULT_WAYPOINTS):
        log.info("  Default WP[%d]: %.7f, %.7f", i, lat, lon)

    default_waypoints = DEFAULT_WAYPOINTS

    # ── 1. Start telemetry listener ──────────────────────────────────────
    log.info("Starting telemetry listener thread...")
    listener = threading.Thread(target=telemetry_listener, daemon=True, name="TelemetryListener")
    listener.start()
    log.info("Telemetry listener thread started (tid=%s)", listener.ident)

    # ── 2. Callback for when user clicks "Start Mission" ─────────────────
    def on_waypoints_selected(wps):
        log.info("Mission start triggered by user — %d waypoints received from GUI", len(wps))
        for i, (lat, lon) in enumerate(wps):
            log.info("  User WP[%d]: %.7f, %.7f", i, lat, lon)

        print(f"\n[MISSION] User selected {len(wps)} waypoints. Sending to vehicle...")

        t0 = time.time()
        send_waypoints(wps)
        elapsed = time.time() - t0

        log.info("Waypoints dispatched to vehicle in %.2fs", elapsed)
        print("[MISSION] Waypoints sent. Dashboard is live.")

    # ── 3. Launch the GUI thread ─────────────────────────────────────────
    log.info("Starting GUI thread on port %d...", GUI_HTTP_PORT)
    gui_thread = threading.Thread(
        target=gui.show,
        args=(default_waypoints, telemetry),
        kwargs={"port": GUI_HTTP_PORT, "on_waypoints": on_waypoints_selected},
        daemon=True,
        name="GUIServer",
    )
    gui_thread.start()
    log.info("GUI thread started — open http://localhost:%d in your browser", GUI_HTTP_PORT)

    # ── 4. Keep the main thread alive until Ctrl+C ───────────────────────
    log.info("Basestation ready — press Ctrl+C to stop")
    try:
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        running = False
        log.info("KeyboardInterrupt received — shutting down")
        log.info(
            "Shutdown stats — telemetry packets received: %d  bad: %d",
            _packet_count, _bad_packet_count,
        )
        print("\n[INFO] Stopped by user.")
        log.info("Basestation stopped cleanly")
        log.info("=" * 60)


# ── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()