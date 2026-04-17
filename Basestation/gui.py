import numpy as np
import json
import logging
import threading
import base64
import os
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

logging.basicConfig(
    filename="info.log",
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

_telemetry_ref   = {}
_dashboard_html  = ""
_planning_html   = ""
_mission_started = False
_on_waypoints_cb = None
_waypoints_event = threading.Event()
_http_port = 8080  # set by show() to match the HTTPServer bind port

# ── Embed the Andromeida logo as base64 PNG ──────────────────────────────────
_LOGO_B64 = ""  # empty string = no logo loaded yet

def _load_logo():
    global _LOGO_B64
    logo_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logo.PNG")
    if os.path.exists(logo_path):
        with open(logo_path, "rb") as f:
            _LOGO_B64 = base64.b64encode(f.read()).decode()
    else:
        _LOGO_B64 = ""
        print("[GUI] Warning: logo.PNG not found next to script")

def _logo_img(height=140):
    if _LOGO_B64:
        return f'<img src="data:image/png;base64,{_LOGO_B64}" alt="Andromeida" style="height:{height}px;width:auto;display:block;" />'
    return ""

# ── Shared tile-layer JS (Leaflet) ───────────────────────────────────────────
_TILE_LAYERS_JS = """
const tileLayers = {
  'OpenStreetMap': L.tileLayer(
    'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    { attribution: '&copy; OpenStreetMap contributors', maxZoom: 22 }
  ),
  'Satellite (Esri)': L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    { attribution: 'Tiles &copy; Esri', maxZoom: 12 }
  ),
  'Esri Topo': L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}',
    { attribution: 'Tiles &copy; Esri', maxZoom: 12 }
  ),
  'Esri Street': L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}',
    { attribution: 'Tiles &copy; Esri', maxZoom: 12 }
  ),
  'CartoDB Light': L.tileLayer(
    'https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png',
    { attribution: '&copy; OpenStreetMap contributors &copy; CARTO', subdomains: 'abcd', maxZoom: 22 }
  ),
  'CartoDB Dark': L.tileLayer(
    'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
    { attribution: '&copy; OpenStreetMap contributors &copy; CARTO', subdomains: 'abcd', maxZoom: 22 }
  ),
  'OpenTopoMap': L.tileLayer(
    'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
    { attribution: 'Map data: &copy; OpenStreetMap contributors | Map style: &copy; OpenTopoMap (CC-BY-SA)', maxZoom: 17 }
  )
};
tileLayers['OpenStreetMap'].addTo(map);
L.control.layers(tileLayers, null, { position: 'topright', collapsed: true }).addTo(map);
"""

_LEAFLET_CSS = '<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/leaflet@1.9.3/dist/leaflet.css"/>'
_LEAFLET_JS  = '<script src="https://cdn.jsdelivr.net/npm/leaflet@1.9.3/dist/leaflet.js"></script>'

_TILE_CONTROL_CSS = """
  .leaflet-control-layers {
    font-family: var(--mono) !important;
    font-size: 11px !important;
    border: 1px solid var(--border) !important;
    border-radius: 4px !important;
    box-shadow: 0 2px 8px rgba(0,0,0,0.1) !important;
  }
  .leaflet-control-layers-toggle { background-color: var(--white) !important; }
  .leaflet-control-layers-expanded {
    padding: 8px 12px !important;
    background: var(--white) !important;
    color: var(--text) !important;
  }
  .leaflet-control-layers label {
    font-family: var(--mono) !important;
    font-size: 11px !important;
    color: var(--text-mid) !important;
    letter-spacing: 0.04em !important;
  }
  .leaflet-control-layers-separator {
    border-top: 1px solid var(--border) !important;
    margin: 6px 0 !important;
  }
"""


def _build_dashboard(waypoints, _map_html_unused=None):
    """Build the monitoring dashboard with a native Leaflet map (supports tile switching)."""
    try:
        from constants_base import ASV_IP, UDP_PORT_OUT, UDP_PORT_IN
    except ImportError:
        ASV_IP = "—"
        UDP_PORT_OUT = 5005
        UDP_PORT_IN = 5006

    R    = 6378137
    lat0 = waypoints[0][0]
    lon0 = waypoints[0][1]
    wp_xy = []
    for lat, lon in waypoints:
        x = R * np.radians(lon - lon0) * np.cos(np.radians(lat0))
        y = R * np.radians(lat - lat0)
        wp_xy.append({"x": round(x, 3), "y": round(y, 3)})

    waypoints_js  = json.dumps(wp_xy)
    waypoints_raw = json.dumps(waypoints)
    center_lat    = waypoints[0][0]
    center_lon    = waypoints[0][1]
    logo_html     = _logo_img(height=140)

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>ASV Ground Control Station</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@300;400;500;600&display=swap" rel="stylesheet">
{_LEAFLET_CSS}
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {{
    --white:      #ffffff;
    --off-white:  #f8f9fb;
    --bg:         #f2f4f7;
    --border:     #dde1e9;
    --border-dark:#bcc3d0;
    --text:       #0f1623;
    --text-mid:   #4a5568;
    --text-dim:   #8a94a6;
    --blue:       #1a56db;
    --blue-light: #e8effe;
    --blue-mid:   #93b4f7;
    --red:        #c0392b;
    --green:      #1a7a4a;
    --amber:      #b45309;
    --mono:       'IBM Plex Mono', monospace;
    --sans:       'IBM Plex Sans', sans-serif;
  }}

  * {{ box-sizing: border-box; margin: 0; padding: 0; }}

  body {{
    background: var(--bg);
    color: var(--text);
    font-family: var(--sans);
    font-size: 13px;
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }}

  /* ── HEADER ── */
  header {{
    background: var(--white);
    border-bottom: 1px solid var(--border);
    padding: 0 24px;
    height: 56px;
    display: flex;
    align-items: center;
    gap: 20px;
    flex-shrink: 0;
  }}

  .logo-img {{ height: 32px; width: auto; display: block; flex-shrink: 0; }}

  .logo-block {{
    display: flex;
    align-items: center;
    gap: 10px;
  }}

  .logo {{
    font-family: var(--mono);
    font-size: 14px;
    font-weight: 600;
    color: var(--text);
    letter-spacing: 0.08em;
  }}

  .logo-sub {{
    font-family: var(--mono);
    font-size: 10px;
    color: var(--text-dim);
    letter-spacing: 0.12em;
    text-transform: uppercase;
  }}

  .divider {{
    width: 1px;
    height: 24px;
    background: var(--border);
  }}

  .status-badge {{
    font-family: var(--mono);
    font-size: 10px;
    font-weight: 600;
    letter-spacing: 0.1em;
    padding: 4px 10px;
    border-radius: 3px;
    background: var(--blue-light);
    color: var(--blue);
    border: 1px solid var(--blue-mid);
  }}

  .header-stats {{
    margin-left: auto;
    display: flex;
    gap: 0;
  }}

  .hstat {{
    display: flex;
    flex-direction: column;
    align-items: flex-end;
    padding: 0 20px;
    border-left: 1px solid var(--border);
  }}

  .hstat-label {{
    font-family: var(--mono);
    font-size: 9px;
    font-weight: 500;
    color: var(--text-dim);
    letter-spacing: 0.12em;
    text-transform: uppercase;
    margin-bottom: 2px;
  }}

  .hstat-value {{
    font-family: var(--mono);
    font-size: 13px;
    font-weight: 600;
    color: var(--text);
  }}

  /* ── TABS ── */
  .tab-bar {{
    background: var(--white);
    border-bottom: 1px solid var(--border);
    padding: 0 24px;
    display: flex;
    gap: 0;
    flex-shrink: 0;
  }}

  .tab {{
    font-family: var(--mono);
    font-size: 11px;
    font-weight: 500;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    padding: 10px 20px;
    border: none;
    background: transparent;
    color: var(--text-dim);
    cursor: pointer;
    border-bottom: 2px solid transparent;
    transition: color 0.15s, border-color 0.15s;
    margin-bottom: -1px;
  }}

  .tab:hover {{ color: var(--text); }}
  .tab.active {{ color: var(--blue); border-bottom-color: var(--blue); }}

  /* ── PANELS ── */
  .panel {{ display: none; flex: 1; overflow: hidden; }}
  .panel.active {{ display: flex; }}

  /* MAP + monitoring sidebar */
  #panel-map {{
    flex-direction: row;
    align-items: stretch;
    min-height: 0;
  }}
  .monitor-sidebar {{
    width: 300px;
    min-width: 260px;
    flex-shrink: 0;
    background: var(--white);
    border-right: 1px solid var(--border);
    overflow-y: auto;
    display: flex;
    flex-direction: column;
  }}
  .monitor-sidebar .sidebar-head {{
    padding: 14px 16px 12px;
    border-bottom: 1px solid var(--border);
  }}
  .monitor-sidebar .sidebar-head p {{
    font-size: 11px;
    color: var(--text-mid);
    line-height: 1.45;
    margin-top: 4px;
  }}
  .monitor-sidebar h2 {{
    font-family: var(--mono);
    font-size: 10px;
    font-weight: 600;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    color: var(--text-dim);
    margin-bottom: 8px;
  }}
  .monitor-sidebar .sidebar-section {{
    padding: 12px 16px 14px;
    border-bottom: 1px solid #f0f1f4;
  }}
  .monitor-sidebar .sidebar-section:last-child {{ border-bottom: none; }}
  .monitor-row {{
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    gap: 10px;
    padding: 3px 0;
  }}
  .monitor-row .lbl {{
    font-family: var(--mono);
    font-size: 9px;
    font-weight: 500;
    color: var(--text-mid);
    letter-spacing: 0.06em;
    text-transform: uppercase;
    flex-shrink: 0;
  }}
  .monitor-row .val {{
    font-family: var(--mono);
    font-size: 11px;
    font-weight: 500;
    color: var(--text);
    text-align: right;
    word-break: break-word;
  }}
  .map-wrap {{
    flex: 1;
    min-width: 0;
    min-height: 0;
    display: flex;
    flex-direction: column;
  }}
  #map {{ flex: 1; min-height: 0; }}

  /* GRAPHS */
  #panel-graphs {{
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 1fr;
    gap: 1px;
    background: var(--border);
    overflow: hidden;
  }}
  #panel-graphs.panel {{ display: none; }}
  #panel-graphs.panel.active {{ display: grid; }}

  /* LOGS */
  #panel-logs {{
    background: #0f172a;
    color: #e2e8f0;
    padding: 0;
  }}
  #panel-logs.panel {{ display: none; }}
  #panel-logs.panel.active {{ display: flex; }}
  .logs-wrap {{
    display: flex;
    flex-direction: column;
    width: 100%;
    min-height: 0;
  }}
  .logs-header {{
    background: #111827;
    border-bottom: 1px solid #243041;
    padding: 10px 14px;
    font-family: var(--mono);
    font-size: 10px;
    color: #94a3b8;
    letter-spacing: 0.08em;
    text-transform: uppercase;
  }}
  .logs-view {{
    flex: 1;
    min-height: 0;
    margin: 0;
    padding: 12px 14px 18px;
    overflow: auto;
    white-space: pre-wrap;
    word-break: break-word;
    font-family: var(--mono);
    font-size: 11px;
    line-height: 1.45;
  }}
  .log-line {{
    margin: 0;
    padding: 1px 0;
  }}
  .log-line.log-level-default {{ color: #e2e8f0; }}
  .log-line.log-level-error {{ color: #f87171; }}
  .log-line.log-level-warning {{ color: #fbbf24; }}
  .log-line.log-level-success {{ color: #34d399; }}

  /* Error toasts (Monitoring + Graphs only) */
  .toast-stack {{
    position: fixed;
    top: 100px;
    right: 20px;
    width: min(360px, calc(100vw - 40px));
    z-index: 10050;
    display: flex;
    flex-direction: column;
    gap: 10px;
    pointer-events: none;
  }}
  .toast-stack.toast-stack--hidden {{ visibility: hidden; opacity: 0; }}
  .toast {{
    pointer-events: auto;
    background: #ffffff;
    border: 1px solid #fecaca;
    border-left: 4px solid #dc2626;
    border-radius: 6px;
    padding: 10px 36px 10px 12px;
    box-shadow: 0 8px 24px rgba(15, 22, 34, 0.18);
    position: relative;
    animation: toast-in 0.2s ease-out;
  }}
  @keyframes toast-in {{
    from {{ opacity: 0; transform: translateX(12px); }}
    to {{ opacity: 1; transform: translateX(0); }}
  }}
  .toast-title {{
    font-family: var(--mono);
    font-size: 10px;
    font-weight: 600;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: #dc2626;
    margin-bottom: 6px;
  }}
  .toast-body {{
    font-family: var(--mono);
    font-size: 11px;
    line-height: 1.45;
    color: #0f1623;
    word-break: break-word;
  }}
  .toast-close {{
    position: absolute;
    top: 6px;
    right: 8px;
    width: 26px;
    height: 26px;
    border: none;
    background: transparent;
    color: #8a94a6;
    font-size: 18px;
    line-height: 1;
    cursor: pointer;
    border-radius: 4px;
  }}
  .toast-close:hover {{ background: #f2f4f7; color: #0f1623; }}

  .card {{
    background: var(--white);
    padding: 16px 18px 12px;
    display: flex;
    flex-direction: column;
    min-height: 0;
  }}

  .card.tall {{ grid-row: span 2; }}

  .card-header {{
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 10px;
    flex-shrink: 0;
  }}

  .card-title {{
    font-family: var(--mono);
    font-size: 9px;
    font-weight: 600;
    letter-spacing: 0.14em;
    text-transform: uppercase;
    color: var(--text-dim);
  }}

  .card-live {{
    width: 6px; height: 6px;
    border-radius: 50%;
    background: var(--green);
    animation: pulse 2s infinite;
  }}

  @keyframes pulse {{
    0%, 100% {{ opacity: 1; }}
    50% {{ opacity: 0.3; }}
  }}

  .card canvas {{ flex: 1; min-height: 0; }}

  {_TILE_CONTROL_CSS}
</style>
</head>
<body>

<header>
  <div class="logo-block">
    {logo_html}
    <div style="display:flex;flex-direction:column;gap:1px;">
      <span class="logo">ASV · GCS</span>
      <span class="logo-sub">Ground Control Station</span>
    </div>
  </div>
  <div class="divider"></div>
  <div class="status-badge" id="hdr-status">WAITING</div>
  <div class="hstat">
      <span class="hstat-label">Look-Ahead</span>
      <span class="hstat-value" id="hdr-tgt">—</span>
    </div>
  <div class="header-stats">
    <div class="hstat">
      <span class="hstat-label">Latitude</span>
      <span class="hstat-value" id="hdr-lat">—</span>
    </div>
    <div class="hstat">
      <span class="hstat-label">Longitude</span>
      <span class="hstat-value" id="hdr-lon">—</span>
    </div>
    <div class="hstat">
      <span class="hstat-label">Heading</span>
      <span class="hstat-value" id="hdr-hdg">—</span>
    </div>
    <div class="hstat">
      <span class="hstat-label">Waypoint</span>
      <span class="hstat-value" id="hdr-wp">—</span>
    </div>
    <div class="hstat">
      <span class="hstat-label">Distance</span>
      <span class="hstat-value" id="hdr-dist">—</span>
    </div>
  </div>
</header>

<div class="tab-bar">
  <button class="tab active" onclick="switchTab('map', this)">Monitoring</button>
  <button class="tab"        onclick="switchTab('graphs', this)">Graphs</button>
  <button class="tab"        onclick="switchTab('logs', this)">Logs</button>
</div>

<div id="toast-stack" class="toast-stack" aria-live="assertive"></div>

<div class="panel active" id="panel-map">
  <aside class="monitor-sidebar">
    <div class="sidebar-head">
      <h2>Sensor &amp; navigation</h2>
    </div>
    <div class="sidebar-section">
      <h2>GPS</h2>
      <div class="monitor-row"><span class="lbl">Latitude</span><span class="val" id="sb-gps-lat">—</span></div>
      <div class="monitor-row"><span class="lbl">Longitude</span><span class="val" id="sb-gps-lon">—</span></div>
      <div class="monitor-row"><span class="lbl">Altitude</span><span class="val" id="sb-gps-alt">—</span></div>
      <div class="monitor-row"><span class="lbl">Fix</span><span class="val" id="sb-gps-fix">—</span></div>
      <div class="monitor-row"><span class="lbl">Satellites</span><span class="val" id="sb-gps-sats">—</span></div>
    </div>
    <div class="sidebar-section">
      <h2>IMU</h2>
      <div class="monitor-row"><span class="lbl">Roll</span><span class="val" id="sb-roll">—</span></div>
      <div class="monitor-row"><span class="lbl">Pitch</span><span class="val" id="sb-pitch">—</span></div>
      <div class="monitor-row"><span class="lbl">Yaw</span><span class="val" id="sb-yaw">—</span></div>
      <div class="monitor-row"><span class="lbl">Accel X</span><span class="val" id="sb-accel-x">—</span></div>
      <div class="monitor-row"><span class="lbl">Accel Y</span><span class="val" id="sb-accel-y">—</span></div>
      <div class="monitor-row"><span class="lbl">Accel Z</span><span class="val" id="sb-accel-z">—</span></div>
    </div>
    <div class="sidebar-section">
      <h2>Navigation</h2>
      <div class="monitor-row"><span class="lbl">Status</span><span class="val" id="sb-nav-status">—</span></div>
      <div class="monitor-row"><span class="lbl">Active WP</span><span class="val" id="sb-active-wp">—</span></div>
      <div class="monitor-row"><span class="lbl">Heading</span><span class="val" id="sb-heading">—</span></div>
      <div class="monitor-row"><span class="lbl">Desired brg</span><span class="val" id="sb-desired-bearing">—</span></div>
      <div class="monitor-row"><span class="lbl">Hdg error</span><span class="val" id="sb-heading-error">—</span></div>
      <div class="monitor-row"><span class="lbl">XTE</span><span class="val" id="sb-xte">—</span></div>
      <div class="monitor-row"><span class="lbl">Dist to WP</span><span class="val" id="sb-dist-wp">—</span></div>
      <div class="monitor-row"><span class="lbl">Target lat</span><span class="val" id="sb-target-lat">—</span></div>
      <div class="monitor-row"><span class="lbl">Target lon</span><span class="val" id="sb-target-lon">—</span></div>
    </div>
    <div class="sidebar-section">
      <h2>Link</h2>
      <div class="monitor-row"><span class="lbl">Vehicle IP</span><span class="val">{ASV_IP}</span></div>
      <div class="monitor-row"><span class="lbl">WP TX port</span><span class="val">{UDP_PORT_OUT}</span></div>
      <div class="monitor-row"><span class="lbl">Telem RX port</span><span class="val">{UDP_PORT_IN}</span></div>
      <div class="monitor-row"><span class="lbl">Dashboard</span><span class="val">{_http_port}</span></div>
    </div>
  </aside>
  <div class="map-wrap">
    <div id="map"></div>
  </div>
</div>

<div class="panel" id="panel-graphs">
  <div class="card tall">
    <div class="card-header">
      <span class="card-title">Actual path vs planned</span>
      <span class="card-live"></span>
    </div>
    <canvas id="c-path"></canvas>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="card-title">Heading vs desired bearing (°)</span>
      <span class="card-live"></span>
    </div>
    <canvas id="c-heading"></canvas>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="card-title">Cross-track error (m)</span>
      <span class="card-live"></span>
    </div>
    <canvas id="c-xte"></canvas>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="card-title">Roll &amp; Pitch (°)</span>
      <span class="card-live"></span>
    </div>
    <canvas id="c-rp"></canvas>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="card-title">Distance to waypoint (m)</span>
      <span class="card-live"></span>
    </div>
    <canvas id="c-dist"></canvas>
  </div>
</div>

<div class="panel" id="panel-logs">
  <div class="logs-wrap">
    <div class="logs-header">Runtime logs (info.log)</div>
    <div class="logs-view" id="logs-view">Waiting for logs...</div>
  </div>
</div>

{_LEAFLET_JS}
<script>
// ── Dashboard Leaflet Map ──────────────────────────────────────────────
const WP_RAW = {waypoints_raw};
const WP_XY  = {waypoints_js};

const map = L.map('map', {{ zoomControl: true }}).setView([{center_lat}, {center_lon}], 19);

{_TILE_LAYERS_JS}

// Draw planned path
const plannedCoords = WP_RAW.map(w => [w[0], w[1]]);
L.polyline(plannedCoords, {{ color: '#bcc3d0', weight: 2, dashArray: '6 4' }}).addTo(map);

// Waypoint markers
WP_RAW.forEach(([lat, lon], i) => {{
  const isFirst = i === 0;
  const isLast  = i === WP_RAW.length - 1;
  if (isFirst || isLast) {{
    L.marker([lat, lon], {{
      icon: L.divIcon({{
        className: '',
        html: `<div style="background:${{isFirst ? '#1a7a4a' : '#c0392b'}};color:white;border-radius:50%;width:22px;height:22px;display:flex;align-items:center;justify-content:center;font-family:'IBM Plex Mono',monospace;font-size:10px;font-weight:600;border:2px solid white;box-shadow:0 2px 6px rgba(0,0,0,0.3);">${{isFirst ? 'S' : 'E'}}</div>`,
        iconSize: [22, 22], iconAnchor: [11, 11]
      }})
    }}).addTo(map).bindPopup(isFirst ? 'Start' : 'End');
  }} else {{
    L.circleMarker([lat, lon], {{
      radius: 5, color: '#1a56db', fillColor: '#1a56db', fillOpacity: 0.7, weight: 1.5
    }}).addTo(map).bindPopup(`WP ${{i}}`);
  }}
}});

// ── Live boat marker (arrow) ───────────────────────────────────────────
let boatMarker = null;
let targetMarker = null;
let actualPath = [];
let actualPolyline = L.polyline([], {{ color: '#1a56db', weight: 2 }}).addTo(map);

const boatIcon = L.divIcon({{
  className: '',
  html: `<div id="boat-icon-container" style="transition:transform 0.2s linear;width:24px;height:24px;display:flex;align-items:center;justify-content:center;">
    <svg width="24" height="24" viewBox="0 0 24 24" style="filter:drop-shadow(0px 2px 3px rgba(0,0,0,0.4));">
      <path d="M12 2L22 20L12 17L2 20L12 2Z" fill="#e74c3c" stroke="white" stroke-width="2"/>
    </svg>
  </div>`,
  iconSize: [24, 24], iconAnchor: [12, 12]
}});

const targetIcon = L.divIcon({{
  className: '',
  html: `<div style="background:#b45309;width:12px;height:12px;transform:rotate(45deg);border:1.5px solid white;box-shadow:0 2px 4px rgba(0,0,0,0.3);"></div>`,
  iconSize: [12, 12], iconAnchor: [6, 6]
}});

function switchTab(name, btn) {{
  document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
  document.getElementById('panel-' + name).classList.add('active');
  btn.classList.add('active');
  if (name === 'map') {{ setTimeout(() => map.invalidateSize(), 50); }}
  updateToastStackVisibility();
}}

function updateToastStackVisibility() {{
  const stack = document.getElementById('toast-stack');
  if (!stack) return;
  const logsActive = document.getElementById('panel-logs')?.classList.contains('active');
  stack.classList.toggle('toast-stack--hidden', !!logsActive);
}}

let _lastLogsBlob = "";
const _activeErrorToastKeys = new Set();
let _logsSnapshotInitialized = false;
let _prevLogLinesSnapshot = [];

function _isLogsPinnedToBottom(el) {{
  return (el.scrollHeight - el.scrollTop - el.clientHeight) < 24;
}}

function _logLineClass(line) {{
  const u = line.toUpperCase();
  if (u.includes(' - [ERROR] - ') || u.includes(' - [CRITICAL] - ')) return 'log-line log-level-error';
  if (u.includes(' - [WARNING] - ')) return 'log-line log-level-warning';
  if (u.includes(' - [INFO] - ') && u.includes('SUCCESS:')) return 'log-line log-level-success';
  return 'log-line log-level-default';
}}

function _errorToastDedupeKey(line) {{
  const m = line.match(/ - \\[(ERROR|CRITICAL)\\] - (.*)$/i);
  return m ? m[2].trim() : line.trim();
}}

function _renderLogLines(container, lines) {{
  container.textContent = '';
  if (!lines.length) {{
    const empty = document.createElement('div');
    empty.className = 'log-line log-level-default';
    empty.textContent = 'No log lines yet.';
    container.appendChild(empty);
    return;
  }}
  for (const line of lines) {{
    const div = document.createElement('div');
    div.className = _logLineClass(line);
    div.textContent = line;
    container.appendChild(div);
  }}
}}

const MAX_ERROR_TOASTS = 2;

function _showErrorToast(message, dedupeKey) {{
  if (_activeErrorToastKeys.has(dedupeKey)) return;
  const stack = document.getElementById('toast-stack');
  if (!stack) return;
  _activeErrorToastKeys.add(dedupeKey);

  const toast = document.createElement('div');
  toast.className = 'toast';
  toast.setAttribute('role', 'alert');
  toast.dataset.dedupeKey = dedupeKey;

  const title = document.createElement('div');
  title.className = 'toast-title';
  title.textContent = 'Error';

  const body = document.createElement('div');
  body.className = 'toast-body';
  body.textContent = message;

  const close = document.createElement('button');
  close.type = 'button';
  close.className = 'toast-close';
  close.setAttribute('aria-label', 'Dismiss');
  close.textContent = '×';
  close.addEventListener('click', () => {{
    _activeErrorToastKeys.delete(dedupeKey);
    toast.remove();
  }});

  toast.appendChild(close);
  toast.appendChild(title);
  toast.appendChild(body);
  stack.appendChild(toast);

  while (stack.children.length > MAX_ERROR_TOASTS) {{
    const oldest = stack.firstElementChild;
    const key = oldest && oldest.dataset ? oldest.dataset.dedupeKey : null;
    if (key) _activeErrorToastKeys.delete(key);
    oldest.remove();
  }}
}}

function _processNewErrorToasts(lines) {{
  if (!_logsSnapshotInitialized) {{
    _prevLogLinesSnapshot = lines.slice();
    _logsSnapshotInitialized = true;
    return;
  }}
  const prevSet = new Set(_prevLogLinesSnapshot);
  const newLinesOnly = lines.filter(l => !prevSet.has(l));
  _prevLogLinesSnapshot = lines.slice();

  const mapActive = document.getElementById('panel-map')?.classList.contains('active');
  const graphsActive = document.getElementById('panel-graphs')?.classList.contains('active');
  if (!mapActive && !graphsActive) return;

  for (const line of newLinesOnly) {{
    const u = line.toUpperCase();
    if (!u.includes(' - [ERROR] - ') && !u.includes(' - [CRITICAL] - ')) continue;
    const msg = _errorToastDedupeKey(line);
    if (!msg) continue;
    _showErrorToast(msg, msg);
  }}
}}

async function pollLogs() {{
  const view = document.getElementById('logs-view');
  if (!view) return;
  try {{
    const r = await fetch('/logs?tail=600');
    const d = await r.json();
    const lines = Array.isArray(d.lines) ? d.lines : [];
    const blob = lines.length ? lines.join('\\n') : '';
    if (blob === _lastLogsBlob) return;
    const stickBottom = _isLogsPinnedToBottom(view);
    _lastLogsBlob = blob;
    _renderLogLines(view, lines);
    _processNewErrorToasts(lines);
    if (stickBottom) view.scrollTop = view.scrollHeight;
  }} catch (e) {{
    view.textContent = '';
    const err = document.createElement('div');
    err.className = 'log-line log-level-error';
    err.textContent = 'Unable to load logs from info.log';
    view.appendChild(err);
  }}
}}

// ── Chart.js setup ─────────────────────────────────────────────────────
const GRID = '#edf0f5';
const TICK = {{ color: '#8a94a6', font: {{ family: 'IBM Plex Mono', size: 10 }} }};
const SCALE = {{
  x: {{ ticks: TICK, grid: {{ color: GRID }}, border: {{ color: '#dde1e9' }} }},
  y: {{ ticks: TICK, grid: {{ color: GRID }}, border: {{ color: '#dde1e9' }} }}
}};
const BASE = {{
  responsive: true, maintainAspectRatio: false, animation: false,
  plugins: {{ legend: {{ display: false }} }}, scales: SCALE
}};
const MAX = 120;

function ds(color, label='') {{
  return {{ label, data: [], borderColor: color, borderWidth: 1.5,
            pointRadius: 0, tension: 0.3, fill: false }};
}}
function push(chart, dsIdx, val) {{
  chart.data.datasets[dsIdx].data.push(val);
  if (chart.data.datasets[dsIdx].data.length > MAX)
    chart.data.datasets[dsIdx].data.shift();
}}
function pushLabel(chart, lbl) {{
  chart.data.labels.push(lbl);
  if (chart.data.labels.length > MAX) chart.data.labels.shift();
}}

const pathChart = new Chart(document.getElementById('c-path'), {{
  type: 'scatter',
  data: {{ datasets: [
    {{ label: 'Planned', data: WP_XY, borderColor: '#bcc3d0',
       backgroundColor: 'transparent', showLine: true, borderWidth: 1.5,
       borderDash: [5,4], pointRadius: 4, pointBackgroundColor: '#bcc3d0' }},
    {{ label: 'Actual',  data: [], borderColor: '#1a56db',
       backgroundColor: 'transparent', showLine: true, borderWidth: 1.5,
       pointRadius: 2, pointBackgroundColor: '#1a56db' }},
    {{ label: 'Target', data: [], backgroundColor: '#b45309',
       pointRadius: 6, pointStyle: 'rectRot', showLine: false }},
    {{ label: 'ASV', data: [], backgroundColor: '#e74c3c', borderColor: '#c0392b',
       pointRadius: 8, pointHoverRadius: 8, pointStyle: 'triangle', rotation: 0, showLine: false }}
  ]}},
  options: {{
    ...BASE,
    plugins: {{ legend: {{ display: true,
      labels: {{ color: '#8a94a6', font: {{ family: 'IBM Plex Mono', size: 10 }} }} }} }},
    scales: {{
      x: {{ ...SCALE.x, title: {{ display: true, text: 'X (m)', color: '#8a94a6',
             font: {{ family: 'IBM Plex Mono', size: 10 }} }} }},
      y: {{ ...SCALE.y, title: {{ display: true, text: 'Y (m)', color: '#8a94a6',
             font: {{ family: 'IBM Plex Mono', size: 10 }} }} }}
    }}
  }}
}});

const headingChart = new Chart(document.getElementById('c-heading'), {{
  type: 'line',
  data: {{ labels: [], datasets: [ ds('#1a56db','Heading'), ds('#b45309','Desired') ] }},
  options: {{ ...BASE, plugins: {{ legend: {{ display: true,
    labels: {{ color: '#8a94a6', font: {{ family: 'IBM Plex Mono', size: 10 }} }} }} }} }}
}});

const xteChart = new Chart(document.getElementById('c-xte'), {{
  type: 'line',
  data: {{ labels: [], datasets: [ ds('#c0392b','XTE') ] }},
  options: BASE
}});

const rpChart = new Chart(document.getElementById('c-rp'), {{
  type: 'line',
  data: {{ labels: [], datasets: [ ds('#1a7a4a','Roll'), ds('#b45309','Pitch') ] }},
  options: {{ ...BASE, plugins: {{ legend: {{ display: true,
    labels: {{ color: '#8a94a6', font: {{ family: 'IBM Plex Mono', size: 10 }} }} }} }} }}
}});

const distChart = new Chart(document.getElementById('c-dist'), {{
  type: 'line',
  data: {{ labels: [], datasets: [ ds('#6366f1','Dist') ] }},
  options: BASE
}});

const R_E  = 6378137;
const lat0 = WP_RAW[0][0];
const lon0 = WP_RAW[0][1];

function toXY(lat, lon) {{
  const dLon = (lon - lon0) * Math.PI / 180;
  const dLat = (lat - lat0) * Math.PI / 180;
  const lat0r = lat0 * Math.PI / 180;
  return {{
    x: +(R_E * dLon * Math.cos(lat0r)).toFixed(3),
    y: +(R_E * dLat).toFixed(3)
  }};
}}

function updateStatusBadge(status) {{
  const el = document.getElementById('hdr-status');
  el.textContent = status;
  el.style.background = status === 'NAVIGATING' ? '#e8faf0' :
                         status === 'REACHED'    ? '#e8effe' :
                         status === 'ERROR'      ? '#fdecea' : '#f2f4f7';
  el.style.color       = status === 'NAVIGATING' ? '#1a7a4a' :
                         status === 'REACHED'    ? '#1a56db' :
                         status === 'ERROR'      ? '#c0392b' : '#4a5568';
  el.style.borderColor = status === 'NAVIGATING' ? '#6fcf97' :
                         status === 'REACHED'    ? '#93b4f7' :
                         status === 'ERROR'      ? '#f5a9a3' : '#dde1e9';
}}

function gpsFixLabel(fix) {{
  const n = parseInt(fix, 10);
  const names = {{0: 'NO FIX', 3: '3D FIX', 5: 'RTK FLOAT', 6: 'RTK FIXED'}};
  return Object.prototype.hasOwnProperty.call(names, n) ? names[n] : 'UNKNOWN (' + n + ')';
}}

function setSb(id, text) {{
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}}

function updateMonitoringSidebar(d) {{
  const nz = (v, def = 0) => {{
    if (v === undefined || v === null) return def;
    const x = Number(v);
    return Number.isFinite(x) ? x : def;
  }};
  setSb('sb-gps-lat', nz(d.gps_lat).toFixed(7));
  setSb('sb-gps-lon', nz(d.gps_lon).toFixed(7));
  setSb('sb-gps-alt', nz(d.gps_alt).toFixed(2) + ' m');
  setSb('sb-gps-fix', d.gps_fix !== undefined && d.gps_fix !== null ? gpsFixLabel(d.gps_fix) : '—');
  setSb('sb-gps-sats', d.gps_sats !== undefined && d.gps_sats !== null ? String(d.gps_sats) : '—');
  setSb('sb-roll', nz(d.roll).toFixed(2) + '°');
  setSb('sb-pitch', nz(d.pitch).toFixed(2) + '°');
  setSb('sb-yaw', nz(d.yaw).toFixed(2) + '°');
  setSb('sb-accel-x', nz(d.accel_x).toFixed(3));
  setSb('sb-accel-y', nz(d.accel_y).toFixed(3));
  setSb('sb-accel-z', nz(d.accel_z).toFixed(3));
  setSb('sb-nav-status', d.nav_status !== undefined && d.nav_status !== null ? String(d.nav_status) : '—');
  setSb('sb-active-wp', d.active_wp !== undefined && d.active_wp !== null ? String(d.active_wp) : '—');
  setSb('sb-heading', nz(d.heading).toFixed(2) + '°');
  setSb('sb-desired-bearing', nz(d.desired_bearing).toFixed(2) + '°');
  setSb('sb-heading-error', nz(d.heading_error).toFixed(2) + '°');
  setSb('sb-xte', (nz(d.cross_track_error) >= 0 ? '+' : '') + nz(d.cross_track_error).toFixed(2) + ' m');
  setSb('sb-dist-wp', nz(d.dist_to_waypoint).toFixed(1) + ' m');
  const tlat = nz(d.target_lat), tlon = nz(d.target_lon);
  setSb('sb-target-lat', tlat !== 0 ? tlat.toFixed(7) : '—');
  setSb('sb-target-lon', tlon !== 0 ? tlon.toFixed(7) : '—');
}}

async function poll() {{
  try {{
    const d  = await (await fetch('/telemetry')).json();
    const ts = new Date().toLocaleTimeString('en', {{hour12: false}});

    updateStatusBadge(d.nav_status);
    updateMonitoringSidebar(d);
    document.getElementById('hdr-lat').textContent  = d.gps_lat.toFixed(6);
    document.getElementById('hdr-lon').textContent  = d.gps_lon.toFixed(6);
    document.getElementById('hdr-hdg').textContent  = d.heading.toFixed(1) + '°';
    document.getElementById('hdr-wp').textContent   = d.active_wp;
    document.getElementById('hdr-dist').textContent = d.dist_to_waypoint.toFixed(1) + ' m';

    if (d.target_lat !== 0) {{
      document.getElementById('hdr-tgt').textContent = d.target_lat.toFixed(5) + ', ' + d.target_lon.toFixed(5);
      pathChart.data.datasets[2].data = [toXY(d.target_lat, d.target_lon)];
      // Update target marker on Leaflet map
      if (!targetMarker) {{
        targetMarker = L.marker([d.target_lat, d.target_lon], {{icon: targetIcon}}).addTo(map);
        targetMarker.bindTooltip("Look-Ahead Target", {{permanent: false, direction: 'right'}});
      }} else {{
        targetMarker.setLatLng([d.target_lat, d.target_lon]);
      }}
    }}

    if (d.gps_lat !== 0 || d.gps_lon !== 0) {{
      const currentPos = toXY(d.gps_lat, d.gps_lon);

      // Chart.js: actual path + ASV triangle
      pathChart.data.datasets[1].data.push(currentPos);
      pathChart.data.datasets[3].data = [currentPos];
      pathChart.data.datasets[3].rotation = 90 - d.heading;
      pathChart.update();

      // Leaflet map: live boat position & heading
      if (!boatMarker) {{
        boatMarker = L.marker([d.gps_lat, d.gps_lon], {{icon: boatIcon, zIndexOffset: 1000}}).addTo(map);
        boatMarker.bindTooltip("ASV Position", {{permanent: false, direction: 'top'}});
      }} else {{
        boatMarker.setLatLng([d.gps_lat, d.gps_lon]);
      }}
      const iconContainer = document.getElementById('boat-icon-container');
      if (iconContainer && d.heading !== undefined) {{
        iconContainer.style.transform = `rotate(${{90 - d.heading}}deg)`;
      }}

      // Leaflet map: draw actual trail
      actualPath.push([d.gps_lat, d.gps_lon]);
      actualPolyline.setLatLngs(actualPath);
    }}

    pushLabel(headingChart, ts);
    push(headingChart, 0, d.heading);
    push(headingChart, 1, d.desired_bearing);
    headingChart.update();

    pushLabel(xteChart, ts);
    push(xteChart, 0, d.cross_track_error);
    xteChart.update();

    pushLabel(rpChart, ts);
    push(rpChart, 0, d.roll);
    push(rpChart, 1, d.pitch);
    rpChart.update();

    pushLabel(distChart, ts);
    push(distChart, 0, d.dist_to_waypoint);
    distChart.update();

  }} catch(e) {{}}
}}

setInterval(poll, 200);
poll();
setInterval(pollLogs, 1000);
pollLogs();
updateToastStackVisibility();
</script>
</body>
</html>"""

def _build_planning_html(default_center, default_waypoints):
    default_wp_js = json.dumps([[lat, lon] for lat, lon in default_waypoints])
    logo_html = _logo_img(height=140)

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>ASV · Mission Planner</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@300;400;500;600&display=swap" rel="stylesheet">
{_LEAFLET_CSS}
<style>
  :root {{
    --white:     #ffffff;
    --bg:        #f2f4f7;
    --border:    #dde1e9;
    --text:      #0f1623;
    --text-mid:  #4a5568;
    --text-dim:  #8a94a6;
    --blue:      #1a56db;
    --blue-light:#e8effe;
    --green:     #1a7a4a;
    --red:       #c0392b;
    --mono:      'IBM Plex Mono', monospace;
    --sans:      'IBM Plex Sans', sans-serif;
  }}

  * {{ box-sizing: border-box; margin: 0; padding: 0; }}

  body {{
    background: var(--bg);
    color: var(--text);
    font-family: var(--sans);
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }}

  header {{
    background: var(--white);
    border-bottom: 1px solid var(--border);
    padding: 0 24px;
    height: 56px;
    display: flex;
    align-items: center;
    gap: 16px;
    flex-shrink: 0;
    z-index: 1000;
  }}

  .logo-block {{
    display: flex;
    align-items: center;
    gap: 12px;
  }}

  .logo {{
    font-family: var(--mono);
    font-size: 14px;
    font-weight: 600;
    letter-spacing: 0.08em;
  }}

  .logo-sub {{
    font-family: var(--mono);
    font-size: 10px;
    color: var(--text-dim);
    letter-spacing: 0.12em;
    text-transform: uppercase;
  }}

  .main-area {{
    flex: 1;
    display: flex;
    overflow: hidden;
  }}

  /* ── Sidebar ── */
  .sidebar {{
    width: 320px;
    background: var(--white);
    border-right: 1px solid var(--border);
    display: flex;
    flex-direction: column;
    flex-shrink: 0;
    overflow: hidden;
  }}

  .sidebar-header {{
    padding: 16px 20px 12px;
    border-bottom: 1px solid var(--border);
  }}

  .sidebar-header h2 {{
    font-family: var(--mono);
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--text-dim);
    margin-bottom: 6px;
  }}

  .sidebar-header p {{
    font-size: 12px;
    color: var(--text-mid);
    line-height: 1.5;
  }}

  .wp-list {{
    flex: 1;
    overflow-y: auto;
    padding: 8px 0;
  }}

  .wp-item {{
    display: flex;
    align-items: center;
    padding: 8px 20px;
    gap: 12px;
    font-family: var(--mono);
    font-size: 12px;
    border-bottom: 1px solid #f0f1f4;
    transition: background 0.15s;
  }}

  .wp-item:hover {{ background: #f8f9fb; }}

  .wp-num {{
    width: 24px;
    height: 24px;
    border-radius: 50%;
    background: var(--blue);
    color: white;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 10px;
    font-weight: 600;
    flex-shrink: 0;
  }}

  .wp-coords {{
    flex: 1;
    color: var(--text);
  }}

  .wp-remove {{
    width: 20px;
    height: 20px;
    border: none;
    background: transparent;
    color: var(--text-dim);
    cursor: pointer;
    font-size: 14px;
    border-radius: 3px;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: all 0.15s;
  }}

  .wp-remove:hover {{ background: #fdecea; color: var(--red); }}

  .wp-empty {{
    padding: 40px 20px;
    text-align: center;
    color: var(--text-dim);
    font-size: 12px;
    font-style: italic;
  }}

  .sidebar-footer {{
    padding: 16px 20px;
    border-top: 1px solid var(--border);
    display: flex;
    flex-direction: column;
    gap: 8px;
  }}

  .btn {{
    font-family: var(--mono);
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    padding: 10px 16px;
    border-radius: 4px;
    border: 1px solid var(--border);
    cursor: pointer;
    transition: all 0.15s;
    text-align: center;
  }}

  .btn-primary {{
    background: var(--blue);
    color: white;
    border-color: var(--blue);
  }}

  .btn-primary:hover {{ background: #1648b8; }}
  .btn-primary:disabled {{ background: #93b4f7; border-color: #93b4f7; cursor: not-allowed; }}

  .btn-secondary {{
    background: var(--white);
    color: var(--text-mid);
  }}

  .btn-secondary:hover {{ background: var(--bg); }}

  .wp-count {{
    font-family: var(--mono);
    font-size: 10px;
    color: var(--text-dim);
    text-align: center;
  }}

  /* ── Map ── */
  #map {{ flex: 1; }}

  .wp-label {{
    background: var(--blue);
    color: white;
    border: 2px solid white;
    border-radius: 50%;
    width: 26px;
    height: 26px;
    display: flex;
    align-items: center;
    justify-content: center;
    font-family: var(--mono);
    font-size: 11px;
    font-weight: 600;
    box-shadow: 0 2px 6px rgba(0,0,0,0.3);
  }}

  {_TILE_CONTROL_CSS}
</style>
</head>
<body>

<header>
  <div class="logo-block">
    {logo_html}
    <div style="display:flex;flex-direction:column;gap:1px;">
      <span class="logo">ASV · MISSION PLANNER</span>
      <span class="logo-sub">Click the map to place waypoints</span>
    </div>
  </div>
</header>

<div class="main-area">
  <div class="sidebar">
    <div class="sidebar-header">
      <h2>Waypoints</h2>
      <p>Click on the map to add waypoints. Click the ✕ to remove one. You need at least 2 to start.</p>
    </div>
    <div class="wp-list" id="wp-list">
      <div class="wp-empty" id="wp-empty">No waypoints yet — click the map!</div>
    </div>
    <div class="sidebar-footer">
      <div class="wp-count" id="wp-count">0 waypoints</div>
      <button class="btn btn-secondary" onclick="clearAll()">Clear All</button>
      <button class="btn btn-primary" id="btn-start" onclick="startMission()" disabled>Start Mission</button>
    </div>
  </div>
  <div id="map"></div>
</div>

{_LEAFLET_JS}
<script>
const waypoints = [];
let segmentLayers = [];
let polyline = null;
const defaultWps = {default_wp_js};

const center = defaultWps.length > 0 ? defaultWps[0] : [27.9147, 78.0766];
const map = L.map('map', {{ zoomControl: true }}).setView(center, 18);

{_TILE_LAYERS_JS}

// ── Haversine distance ─────────────────────────────────────────────────
function haversineDistance(lat1, lon1, lat2, lon2) {{
  const R = 6378137;
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
            Math.sin(dLon / 2) * Math.sin(dLon / 2);
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}}

function formatDist(m) {{
  return m >= 1000 ? (m / 1000).toFixed(2) + ' km' : m.toFixed(1) + ' m';
}}

function wpIcon(n) {{
  return L.divIcon({{
    className: '',
    html: `<div class="wp-label">${{n}}</div>`,
    iconSize: [26, 26],
    iconAnchor: [13, 13]
  }});
}}

function addWaypoint(lat, lon) {{
  const idx = waypoints.length;
  const marker = L.marker([lat, lon], {{ icon: wpIcon(idx + 1), draggable: true }}).addTo(map);

  // ── Popup showing lat/lon on click ────────────────────────────────
  marker.bindPopup(
    `<div style="font-family:'IBM Plex Mono',monospace;font-size:11px;line-height:1.8;min-width:160px;">
      <span style="font-weight:600;color:#1a56db;">WP ${{idx + 1}}</span><br/>
      <span style="color:#4a5568;">Lat:</span> <span style="color:#0f1623;">${{lat.toFixed(7)}}</span><br/>
      <span style="color:#4a5568;">Lon:</span> <span style="color:#0f1623;">${{lon.toFixed(7)}}</span>
    </div>`,
    {{ maxWidth: 220 }}
  );

  marker.on('dragend', function() {{
    const pos = marker.getLatLng();
    const wp = waypoints.find(w => w.marker === marker);
    if (wp) {{
      wp.lat = pos.lat;
      wp.lon = pos.lng;
      // Update popup content with new coordinates after drag
      const wpIdx = waypoints.indexOf(wp);
      marker.setPopupContent(
        `<div style="font-family:'IBM Plex Mono',monospace;font-size:11px;line-height:1.8;min-width:160px;">
          <span style="font-weight:600;color:#1a56db;">WP ${{wpIdx + 1}}</span><br/>
          <span style="color:#4a5568;">Lat:</span> <span style="color:#0f1623;">${{pos.lat.toFixed(7)}}</span><br/>
          <span style="color:#4a5568;">Lon:</span> <span style="color:#0f1623;">${{pos.lng.toFixed(7)}}</span>
        </div>`
      );
    }}
    updateUI();
  }});

  waypoints.push({{ lat, lon, marker }});
  updateUI();
}}

function removeWaypoint(idx) {{
  if (idx < 0 || idx >= waypoints.length) return;
  map.removeLayer(waypoints[idx].marker);
  waypoints.splice(idx, 1);
  waypoints.forEach((wp, i) => {{ wp.marker.setIcon(wpIcon(i + 1)); }});
  updateUI();
}}

function clearAll() {{
  waypoints.forEach(wp => map.removeLayer(wp.marker));
  waypoints.length = 0;
  updateUI();
}}

const DIST_LABEL_ZOOM = 19;  // zoom level at which distance labels appear

// ── Clear old segment polylines and distance labels ────────────────────
function clearSegmentLayers() {{
  segmentLayers.forEach(l => map.removeLayer(l));
  segmentLayers = [];
}}

// ── Show/hide only the distance label markers based on current zoom ────
function updateDistLabelVisibility() {{
  const show = map.getZoom() >= DIST_LABEL_ZOOM;
  // distMarkers are stored at odd indices: [seg0, lbl0, seg1, lbl1, …]
  for (let i = 1; i < segmentLayers.length; i += 2) {{
    const el = segmentLayers[i].getElement();
    if (el) el.style.opacity = show ? '1' : '0';
  }}
}}

map.on('zoomend', updateDistLabelVisibility);

function updateUI() {{
  const list   = document.getElementById('wp-list');
  const empty  = document.getElementById('wp-empty');
  const count  = document.getElementById('wp-count');
  const btn    = document.getElementById('btn-start');

  list.querySelectorAll('.wp-item').forEach(el => el.remove());

  if (waypoints.length === 0) {{
    empty.style.display = 'block';
  }} else {{
    empty.style.display = 'none';
    waypoints.forEach((wp, i) => {{
      const div = document.createElement('div');
      div.className = 'wp-item';
      div.innerHTML = `
        <span class="wp-num">${{i + 1}}</span>
        <span class="wp-coords">${{wp.lat.toFixed(7)}}, ${{wp.lon.toFixed(7)}}</span>
        <button class="wp-remove" onclick="removeWaypoint(${{i}})">✕</button>
      `;
      list.appendChild(div);
    }});
  }}

  count.textContent = waypoints.length + ' waypoint' + (waypoints.length !== 1 ? 's' : '');
  btn.disabled = waypoints.length < 2;

  // ── Redraw segments with distance labels ──────────────────────────
  clearSegmentLayers();

  if (waypoints.length >= 2) {{
    const showLabels = map.getZoom() >= DIST_LABEL_ZOOM;

    for (let i = 0; i < waypoints.length - 1; i++) {{
      const A = waypoints[i];
      const B = waypoints[i + 1];

      // Segment polyline
      const seg = L.polyline(
        [[A.lat, A.lon], [B.lat, B.lon]],
        {{ color: '#1a56db', weight: 2.5, dashArray: '8 5' }}
      ).addTo(map);

      // Distance label at midpoint
      const midLat = (A.lat + B.lat) / 2;
      const midLon = (A.lon + B.lon) / 2;
      const dist   = haversineDistance(A.lat, A.lon, B.lat, B.lon);

      const distMarker = L.marker([midLat, midLon], {{
        icon: L.divIcon({{
          className: '',
          html: `<div style="
            font-family: 'IBM Plex Mono', monospace;
            font-size: 10px;
            font-weight: 600;
            color: #1a56db;
            white-space: nowrap;
            pointer-events: none;
            opacity: ${{showLabels ? 1 : 0}};
            transition: opacity 0.2s ease;
            text-shadow: 0 1px 3px rgba(255,255,255,0.9), 0 -1px 3px rgba(255,255,255,0.9), 1px 0 3px rgba(255,255,255,0.9), -1px 0 3px rgba(255,255,255,0.9);
          ">${{formatDist(dist)}}</div>`,
          iconAnchor: [28, 10]
        }}),
        interactive: false,
        zIndexOffset: 500
      }}).addTo(map);

      // always push as pair [seg, label] so indices stay consistent
      segmentLayers.push(seg, distMarker);
    }}
  }}
}}

map.on('click', function(e) {{
  addWaypoint(e.latlng.lat, e.latlng.lng);
}});

defaultWps.forEach(([lat, lon]) => addWaypoint(lat, lon));

// ── Live telemetry on planning map ────────────────────────────────────
let boatMarker = null;
let targetMarker = null;

const boatIconHtml = `
  <div id="boat-icon-container" style="transition: transform 0.2s linear; width: 24px; height: 24px; display: flex; align-items: center; justify-content: center;">
    <svg width="24" height="24" viewBox="0 0 24 24" style="filter: drop-shadow(0px 2px 3px rgba(0,0,0,0.4));">
      <path d="M12 2L22 20L12 17L2 20L12 2Z" fill="#e74c3c" stroke="white" stroke-width="2"/>
    </svg>
  </div>
`;
const boatIcon = L.divIcon({{ className: '', html: boatIconHtml, iconSize: [24, 24], iconAnchor: [12, 12] }});
const targetIcon = L.divIcon({{
  className: '',
  html: `<div style="background: #b45309; width: 12px; height: 12px; transform: rotate(45deg); border: 1.5px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>`,
  iconSize: [12, 12], iconAnchor: [6, 6]
}});

async function pollTelemetryPos() {{
  try {{
    const resp = await fetch('/telemetry');
    const d = await resp.json();
    if (d.gps_lat && d.gps_lat !== 0) {{
      if (!boatMarker) {{
        boatMarker = L.marker([d.gps_lat, d.gps_lon], {{icon: boatIcon, zIndexOffset: 1000}}).addTo(map);
        boatMarker.bindTooltip("ASV Position", {{permanent: false, direction: 'top'}});
        if (waypoints.length === 0) {{ map.setView([d.gps_lat, d.gps_lon], 18); }}
      }} else {{
        boatMarker.setLatLng([d.gps_lat, d.gps_lon]);
      }}
      const iconContainer = document.getElementById('boat-icon-container');
      if (iconContainer && d.heading !== undefined) {{
        iconContainer.style.transform = `rotate(${{90 - d.heading}}deg)`;
      }}
    }}
    if (d.target_lat && d.target_lat !== 0) {{
      if (!targetMarker) {{
        targetMarker = L.marker([d.target_lat, d.target_lon], {{icon: targetIcon}}).addTo(map);
        targetMarker.bindTooltip("Look-Ahead Target", {{permanent: false, direction: 'right'}});
      }} else {{
        targetMarker.setLatLng([d.target_lat, d.target_lon]);
      }}
    }}
  }} catch(e) {{}} finally {{
    setTimeout(pollTelemetryPos, 500);
  }}
}}
pollTelemetryPos();

async function startMission() {{
  const btn = document.getElementById('btn-start');
  btn.disabled = true;
  btn.textContent = 'SENDING...';
  const wps = waypoints.map(wp => [wp.lat, wp.lon]);
  try {{
    const resp = await fetch('/start', {{
      method: 'POST',
      headers: {{ 'Content-Type': 'application/json' }},
      body: JSON.stringify({{ waypoints: wps }})
    }});
    if (resp.ok) {{
      btn.textContent = 'MISSION STARTED ✓';
      btn.style.background = '#1a7a4a';
      btn.style.borderColor = '#1a7a4a';
      setTimeout(() => {{ window.location.href = '/dashboard'; }}, 800);
    }} else {{
      btn.textContent = 'ERROR — RETRY';
      btn.style.background = '#c0392b';
      btn.disabled = false;
    }}
  }} catch(e) {{
    btn.textContent = 'ERROR — RETRY';
    btn.style.background = '#c0392b';
    btn.disabled = false;
  }}
}}
</script>
</body>
</html>"""


# =============================================================================
# HTTP HANDLER
# =============================================================================
def _read_log_tail(file_path, max_lines=500):
    max_lines = max(1, min(int(max_lines), 2000))
    try:
        with open(file_path, "r", encoding="utf-8", errors="replace") as fh:
            return list(deque(fh, maxlen=max_lines))
    except FileNotFoundError:
        return []
    except Exception as e:
        logging.error("Error reading log tail from %s: %s", file_path, e)
        return [f"[GUI] Failed to read logs: {e}"]


def _read_latest_basestation_session(file_path, max_lines=2000):
    """Return only latest Basestation session logs.

    Session starts at "Basestation starting up" and ends at
    "Basestation stopped cleanly". If stop marker is not present yet,
    returns from latest start marker to EOF.
    """
    lines = _read_log_tail(file_path, max_lines=max_lines)
    if not lines:
        return []

    start_marker = "Basestation starting up"
    stop_marker = "Basestation stopped cleanly"

    start_idx = -1
    for i, line in enumerate(lines):
        if start_marker in line:
            start_idx = i

    if start_idx == -1:
        return []

    stop_idx = -1
    for i in range(start_idx, len(lines)):
        if stop_marker in lines[i]:
            stop_idx = i

    if stop_idx != -1:
        return lines[start_idx:stop_idx + 1]
    return lines[start_idx:]


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        parsed = urlparse(self.path)
        route = parsed.path

        if route == '/telemetry':
            body = json.dumps(_telemetry_ref).encode()
            self._respond(200, 'application/json', body)
        elif route == '/logs':
            qs = parse_qs(parsed.query)
            tail = qs.get('tail', ['500'])[0]
            try:
                tail = int(tail)
            except ValueError:
                tail = 500
            log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "info.log")
            lines = [line.rstrip("\n") for line in _read_latest_basestation_session(log_path, tail)]
            body = json.dumps({'lines': lines}).encode()
            self._respond(200, 'application/json', body)
        elif route == '/dashboard':
            body = _dashboard_html.encode()
            self._respond(200, 'text/html; charset=utf-8', body)
        else:
            if _mission_started:
                body = _dashboard_html.encode()
            else:
                body = _planning_html.encode()
            self._respond(200, 'text/html; charset=utf-8', body)

    def do_POST(self):
        global _mission_started, _dashboard_html

        if self.path == '/start':
            length = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(length)
            try:
                data = json.loads(raw.decode())
                wps = [(lat, lon) for lat, lon in data.get('waypoints', [])]

                if len(wps) < 2:
                    self._respond(400, 'application/json',
                                  json.dumps({'error': 'Need at least 2 waypoints'}).encode())
                    return

                logging.info("User submitted %d waypoints via GUI", len(wps))

                _dashboard_html = _build_dashboard(wps)
                _mission_started = True

                if _on_waypoints_cb:
                    _on_waypoints_cb(wps)

                _waypoints_event.set()

                self._respond(200, 'application/json',
                              json.dumps({'ok': True, 'count': len(wps)}).encode())

            except Exception as e:
                logging.error("Error processing /start: %s", e)
                self._respond(500, 'application/json',
                              json.dumps({'error': str(e)}).encode())
        else:
            self._respond(404, 'text/plain', b'Not Found')

    def _respond(self, code, content_type, body):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)


# =============================================================================
# PUBLIC API
# =============================================================================
def show(waypoints, telemetry, port=8080, on_waypoints=None):
    global _telemetry_ref, _dashboard_html, _planning_html
    global _mission_started, _on_waypoints_cb, _http_port

    _load_logo()

    _telemetry_ref   = telemetry
    _on_waypoints_cb = on_waypoints
    _http_port       = port

    if on_waypoints is not None:
        _mission_started = False
        center = waypoints[0] if waypoints else (27.9147, 78.0766)
        _planning_html  = _build_planning_html(center, waypoints)
        _dashboard_html = _build_dashboard(waypoints)
        logging.info("Planning mode - open http://localhost:%d to select waypoints", port)
        print(f"[GUI] Open -> http://localhost:{port}  (select waypoints on the map)")
    else:
        _mission_started = True
        _dashboard_html = _build_dashboard(waypoints)
        logging.info("Dashboard running at http://localhost:%d", port)
        print(f"[GUI] Open -> http://localhost:{port}")

    HTTPServer(("0.0.0.0", port), _Handler).serve_forever()


def wait_for_waypoints(timeout=None):
    return _waypoints_event.wait(timeout=timeout)