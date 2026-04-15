# =============================================================================
# BASESTATION CONSTANTS
# =============================================================================
# Central configuration file for the Base Station.
# These must stay in sync with Vehicle/constants.py networking section.
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
# NETWORKING  –  must MIRROR the Vehicle's port assignments
# ─────────────────────────────────────────────────────────────────────────────
#   Vehicle sends telemetry on Vehicle.UDP_PORT_OUT = 5006
#   → Base station listens on UDP_PORT_IN = 5006  (same port)
#
#   Base station sends waypoints on UDP_PORT_OUT = 5005
#   → Vehicle listens on Vehicle.UDP_PORT_IN = 5005  (same port)
# ─────────────────────────────────────────────────────────────────────────────
ASV_IP            = "103.55.109.21"     # Vehicle IP on the shared Wi-Fi network
UDP_PORT_OUT      = 5005            # Port to SEND waypoints TO the vehicle
UDP_PORT_IN       = 5006            # Port to RECEIVE telemetry FROM the vehicle

# ─────────────────────────────────────────────────────────────────────────────
# WAYPOINT TRANSMISSION  (main.py)
# ─────────────────────────────────────────────────────────────────────────────
WP_SEND_RETRIES   = 5               # Number of times to re-send the waypoint packet
WP_SEND_INTERVAL  = 0.2             # Seconds between each retry

# ─────────────────────────────────────────────────────────────────────────────
# TELEMETRY LISTENER  (main.py)
# ─────────────────────────────────────────────────────────────────────────────
TELEM_RECV_TIMEOUT = 1.0            # Socket recv timeout (seconds)
TELEM_BUFFER_SIZE  = 4096           # UDP receive buffer size (bytes)

# ─────────────────────────────────────────────────────────────────────────────
# GUI  (gui.py)
# ─────────────────────────────────────────────────────────────────────────────
GUI_HTTP_PORT     = 8080            # HTTP port for the dashboard web UI

# ─────────────────────────────────────────────────────────────────────────────
# DEFAULT WAYPOINTS  (main.py)
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_WAYPOINTS = [
    (27.914731,  78.0766382),
    (27.9147516, 78.0768118),
    (27.9146165, 78.0767982),
    (27.9146257, 78.076599),
    (27.914731,  78.0766382),
]
