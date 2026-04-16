import numpy as np
import json
import logging
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

logging.basicConfig(
    filename="info.log",
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

_telemetry_ref   = {}          # shared dict written by telemetry listener, read by /telemetry
_dashboard_html  = ""          # cached HTML for the monitoring dashboard
_planning_html   = ""          # cached HTML for the waypoint planning page
_mission_started = False       # flips to True after the user clicks Start Mission
_on_waypoints_cb = None        # callback function: called with [(lat,lon), ...] when user submits
_waypoints_event = threading.Event()  # signalled when waypoints are submitted


def _build_map_html(waypoints):
    import folium
    m = folium.Map(
        location=[waypoints[0][0], waypoints[0][1]],
        zoom_start=20,
        tiles="CartoDB positron"
    )
    folium.Marker(
        [waypoints[0][0], waypoints[0][1]], popup="Start",
        icon=folium.Icon(color="green", icon="play", prefix="fa")
    ).add_to(m)
    folium.Marker(
        [waypoints[-1][0], waypoints[-1][1]], popup="End",
        icon=folium.Icon(color="red", icon="flag", prefix="fa")
    ).add_to(m)
    for i, (lat, lon) in enumerate(waypoints):
        folium.CircleMarker(
            [lat, lon], radius=5, color="#1a56db",
            fill=True, fill_color="#1a56db", fill_opacity=0.7,
            popup=f"WP {i}"
        ).add_to(m)
    folium.PolyLine(
        waypoints, color="#1a56db", weight=2,
        dash_array="6 4", tooltip="Planned path"
    ).add_to(m)
    return m._repr_html_()


def _build_dashboard(waypoints, map_html):
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
    map_escaped   = map_html.replace("\\", "\\\\").replace("`", "\\`")

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>ASV Ground Control Station</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@300;400;500;600&display=swap" rel="stylesheet">
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

  .logo-block {{
    display: flex;
    align-items: baseline;
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

  .hstat:last-child {{ border-right: none; }}

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

  /* MAP */
  #panel-map {{ flex-direction: column; }}
  #map-container {{ flex: 1; }}
  #map-container > * {{ width: 100% !important; height: 100% !important; }}

  /* GRAPHS */
  #panel-graphs {{
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 1fr;
    gap: 1px;
    background: var(--border);
    overflow: hidden;
  }}

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
</style>
</head>
<body>

<header>
  <div class="logo-block">
    <span class="logo">ASV · GCS</span>
    <span class="logo-sub">Ground Control Station</span>
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
  <button class="tab active" onclick="switchTab('map', this)">Map</button>
  <button class="tab"        onclick="switchTab('graphs', this)">Graphs</button>
</div>

<div class="panel active" id="panel-map">
  <div id="map-container"></div>
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

<script>
document.getElementById('map-container').innerHTML = `{map_escaped}`;
const mapEl = document.getElementById('map-container').firstElementChild;
if (mapEl) {{ mapEl.style.cssText = 'width:100%;height:100%;border:none;'; }}

function switchTab(name, btn) {{
  document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
  document.getElementById('panel-' + name).classList.add('active');
  btn.classList.add('active');
  if (name === 'map') window.dispatchEvent(new Event('resize'));
}}

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

const WP_XY  = {waypoints_js};
const WP_RAW = {waypoints_raw};

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
    // ── NEW: Live ASV Triangle on the Dashboard ──
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

async function poll() {{
  try {{
    const d  = await (await fetch('/telemetry')).json();
    const ts = new Date().toLocaleTimeString('en', {{hour12: false}});

    updateStatusBadge(d.nav_status);
    document.getElementById('hdr-lat').textContent  = d.gps_lat.toFixed(6);
    document.getElementById('hdr-lon').textContent  = d.gps_lon.toFixed(6);
    document.getElementById('hdr-hdg').textContent  = d.heading.toFixed(1) + '°';
    document.getElementById('hdr-wp').textContent   = d.active_wp;
    document.getElementById('hdr-dist').textContent = d.dist_to_waypoint.toFixed(1) + ' m';
    
    if (d.target_lat !== 0) {{
      document.getElementById('hdr-tgt').textContent = d.target_lat.toFixed(5) + ', ' + d.target_lon.toFixed(5);
      pathChart.data.datasets[2].data = [toXY(d.target_lat, d.target_lon)];
    }}
    
    if (d.gps_lat !== 0 || d.gps_lon !== 0) {{
      const currentPos = toXY(d.gps_lat, d.gps_lon);
      
      // Update the trailing blue line
      pathChart.data.datasets[1].data.push(currentPos);
      
      // ── Update the Live ASV Triangle ──
      pathChart.data.datasets[3].data = [currentPos];
      // Chart.js 0° points UP (+Y). Nav 0° points EAST (+X). 
      // 90 - heading accurately maps Nav Frame to Chart Rotation.
      pathChart.data.datasets[3].rotation = 90 - d.heading;
      
      pathChart.update();
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
</script>
</body>
</html>"""


# =============================================================================
# WAYPOINT PLANNING PAGE  –  interactive Leaflet map for selecting waypoints
# =============================================================================
def _build_planning_html(default_center, default_waypoints):
    """
    Returns a full HTML page with an interactive Leaflet map.
    The user clicks to place waypoints, then clicks 'Start Mission'
    to POST them to /start.
    """
    # Convert default waypoints to JSON for the JS code
    default_wp_js = json.dumps([[lat, lon] for lat, lon in default_waypoints])

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>ASV · Mission Planner</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@300;400;500;600&display=swap" rel="stylesheet">
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/leaflet@1.9.3/dist/leaflet.css"/>
<script src="https://cdn.jsdelivr.net/npm/leaflet@1.9.3/dist/leaflet.js"></script>
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
</style>
</head>
<body>

<header>
  <span class="logo">ASV · MISSION PLANNER</span>
  <span class="logo-sub">Click the map to place waypoints</span>
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

<script>
// ── State ──────────────────────────────────────────────────────────────
const waypoints = [];   // array of {{lat, lon, marker, idx}}
let polyline = null;
const defaultWps = {default_wp_js};

// ── Map setup ──────────────────────────────────────────────────────────
const center = defaultWps.length > 0 ? defaultWps[0] : [27.9147, 78.0766];
const map = L.map('map', {{ zoomControl: true }}).setView(center, 18);

L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  attribution: '&copy; OpenStreetMap contributors',
  maxZoom: 22
}}).addTo(map);

// ── Custom numbered icon ───────────────────────────────────────────────
function wpIcon(n) {{
  return L.divIcon({{
    className: '',
    html: `<div class="wp-label">${{n}}</div>`,
    iconSize: [26, 26],
    iconAnchor: [13, 13]
  }});
}}

// ── Add a waypoint ─────────────────────────────────────────────────────
function addWaypoint(lat, lon) {{
  const idx = waypoints.length;
  const marker = L.marker([lat, lon], {{ icon: wpIcon(idx + 1), draggable: true }}).addTo(map);

  // Drag updates the coordinates
  marker.on('dragend', function() {{
    const pos = marker.getLatLng();
    const wp = waypoints.find(w => w.marker === marker);
    if (wp) {{ wp.lat = pos.lat; wp.lon = pos.lng; }}
    updateUI();
  }});

  waypoints.push({{ lat, lon, marker }});
  updateUI();
}}

// ── Remove a waypoint by index ─────────────────────────────────────────
function removeWaypoint(idx) {{
  if (idx < 0 || idx >= waypoints.length) return;
  map.removeLayer(waypoints[idx].marker);
  waypoints.splice(idx, 1);
  // Re-number all remaining markers
  waypoints.forEach((wp, i) => {{ wp.marker.setIcon(wpIcon(i + 1)); }});
  updateUI();
}}

// ── Clear all ──────────────────────────────────────────────────────────
function clearAll() {{
  waypoints.forEach(wp => map.removeLayer(wp.marker));
  waypoints.length = 0;
  updateUI();
}}

// ── Redraw the sidebar list, polyline, and button state ────────────────
function updateUI() {{
  const list = document.getElementById('wp-list');
  const empty = document.getElementById('wp-empty');
  const count = document.getElementById('wp-count');
  const btn   = document.getElementById('btn-start');

  // Rebuild the sidebar list
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

  // Redraw the path polyline
  if (polyline) map.removeLayer(polyline);
  if (waypoints.length >= 2) {{
    polyline = L.polyline(
      waypoints.map(wp => [wp.lat, wp.lon]),
      {{ color: '#1a56db', weight: 2.5, dashArray: '8 5' }}
    ).addTo(map);
  }}
}}

// ── Map click → add waypoint ───────────────────────────────────────────
map.on('click', function(e) {{
  addWaypoint(e.latlng.lat, e.latlng.lng);
}});

// ── Load default waypoints ─────────────────────────────────────────────
defaultWps.forEach(([lat, lon]) => addWaypoint(lat, lon));


// — Live Boat Position & Orientation —
let boatMarker = null;

// Use an SVG arrow pointing UP (North) by default, rather than a circle
const boatIconHtml = `
  <div id="boat-icon-container" style="transition: transform 0.2s linear; width: 24px; height: 24px; display: flex; align-items: center; justify-content: center;">
    <svg width="24" height="24" viewBox="0 0 24 24" style="filter: drop-shadow(0px 2px 3px rgba(0,0,0,0.4));">
      <path d="M12 2L22 20L12 17L2 20L12 2Z" fill="#e74c3c" stroke="white" stroke-width="2"/>
    </svg>
  </div>
`;

const boatIcon = L.divIcon({{
  className: '',
  html: boatIconHtml,
  iconSize: [24, 24],
  iconAnchor: [12, 12]
}});

// — Target Point Marker —
let targetMarker = null;
const targetIcon = L.divIcon({{
  className: '',
  html: `<div style="background: #b45309; width: 12px; height: 12px; transform: rotate(45deg); border: 1.5px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>`,
  iconSize: [12, 12],
  iconAnchor: [6, 6]
}});


async function pollTelemetryPos() {{
  try {{
    const resp = await fetch('/telemetry');
    const d = await resp.json();
    
    // 1. UPDATE BOAT POSITION & ORIENTATION
    if (d.gps_lat && d.gps_lat !== 0) {{
      if (!boatMarker) {{
        boatMarker = L.marker([d.gps_lat, d.gps_lon], {{icon: boatIcon, zIndexOffset: 1000}}).addTo(map);
        boatMarker.bindTooltip("ASV Position", {{permanent: false, direction: 'top'}});
        
        if (waypoints.length === 0) {{
          map.setView([d.gps_lat, d.gps_lon], 18);
        }}
      }} else {{
        boatMarker.setLatLng([d.gps_lat, d.gps_lon]);
      }}

      // Rotate the boat icon based on heading
      const iconContainer = document.getElementById('boat-icon-container');
      if (iconContainer && d.heading !== undefined) {{
        // CSS rotation: 0deg points UP. Our Nav 0 is EAST. 90 - heading fixes this!
        iconContainer.style.transform = `rotate(${{90 - d.heading}}deg)`;
      }}
    }}

    // 2. UPDATE TARGET POINT
    if (d.target_lat && d.target_lat !== 0) {{
      if (!targetMarker) {{
        targetMarker = L.marker([d.target_lat, d.target_lon], {{icon: targetIcon}}).addTo(map);
        targetMarker.bindTooltip("Look-Ahead Target", {{permanent: false, direction: 'right'}});
      }} else {{
        targetMarker.setLatLng([d.target_lat, d.target_lon]);
      }}
    }}

  }} catch(e) {{
    // console.warn("Telemetry poll failed:", e);
  }} finally {{
    setTimeout(pollTelemetryPos, 500);
  }}
}}

// Kick off the continuous polling loop
pollTelemetryPos();

// ── Start Mission ──────────────────────────────────────────────────────
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
      // Redirect to the dashboard after a brief delay
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
# HTTP HANDLER  –  serves planning page, dashboard, and API endpoints
# =============================================================================
class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass  # suppress default HTTP logs

    def do_GET(self):
        if self.path == '/telemetry':
            # JSON API endpoint polled by the dashboard every 200ms
            body = json.dumps(_telemetry_ref).encode()
            self._respond(200, 'application/json', body)

        elif self.path == '/dashboard':
            # The monitoring dashboard (shown after mission starts)
            body = _dashboard_html.encode()
            self._respond(200, 'text/html; charset=utf-8', body)

        else:
            # Root path '/' → show the planning page (or dashboard if mission already started)
            if _mission_started:
                body = _dashboard_html.encode()
            else:
                body = _planning_html.encode()
            self._respond(200, 'text/html; charset=utf-8', body)

    def do_POST(self):
        global _mission_started, _dashboard_html

        if self.path == '/start':
            # Read the POST body
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

                # Rebuild the dashboard with the user-selected waypoints
                map_html = _build_map_html(wps)
                _dashboard_html = _build_dashboard(wps, map_html)

                _mission_started = True

                # Call the callback so main.py can send waypoints to the vehicle
                if _on_waypoints_cb:
                    _on_waypoints_cb(wps)

                # Signal the event so main.py's blocking wait can proceed
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
# PUBLIC API  –  called by main.py
# =============================================================================
def show(waypoints, telemetry, port=8080, on_waypoints=None):
    """
    Start the GUI HTTP server.

    :param waypoints: Default waypoints to pre-load on the planning map
    :param telemetry: Shared telemetry dict (updated by telemetry listener)
    :param port: HTTP port for the web server
    :param on_waypoints: Callback function(wps) called when user submits waypoints.
                         If None, the planning page is skipped and the dashboard
                         is shown immediately with the provided waypoints.
    """
    global _telemetry_ref, _dashboard_html, _planning_html
    global _mission_started, _on_waypoints_cb

    _telemetry_ref  = telemetry
    _on_waypoints_cb = on_waypoints

    if on_waypoints is not None:
        # Planning mode: show the interactive waypoint selector first
        _mission_started = False
        center = waypoints[0] if waypoints else (27.9147, 78.0766)
        _planning_html = _build_planning_html(center, waypoints)
        # Pre-build a dashboard with default waypoints (will be rebuilt on /start)
        map_html = _build_map_html(waypoints)
        _dashboard_html = _build_dashboard(waypoints, map_html)
        logging.info("Planning mode - open http://localhost:%d to select waypoints", port)
        print(f"[GUI] Open -> http://localhost:{port}  (select waypoints on the map)")
    else:
        # Direct mode: skip planning, go straight to monitoring dashboard
        _mission_started = True
        map_html = _build_map_html(waypoints)
        _dashboard_html = _build_dashboard(waypoints, map_html)
        logging.info("Dashboard running at http://localhost:%d", port)
        print(f"[GUI] Open -> http://localhost:{port}")

    HTTPServer(("0.0.0.0", port), _Handler).serve_forever()


def wait_for_waypoints(timeout=None):
    """
    Block until the user submits waypoints from the planning page.
    Returns True if waypoints were received, False on timeout.
    """
    return _waypoints_event.wait(timeout=timeout)