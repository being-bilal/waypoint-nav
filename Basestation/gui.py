import numpy as np
import json
import logging
from http.server import HTTPServer, BaseHTTPRequestHandler

logging.basicConfig(
    filename="info.log",
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

_telemetry_ref  = {}
_dashboard_html = ""


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
       pointRadius: 2, pointBackgroundColor: '#1a56db' }}
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

    if (d.gps_lat !== 0 || d.gps_lon !== 0) {{
      pathChart.data.datasets[1].data.push(toXY(d.gps_lat, d.gps_lon));
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


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        if self.path == '/telemetry':
            body = json.dumps(_telemetry_ref).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(body))
            self.end_headers()
            self.wfile.write(body)
        else:
            body = _dashboard_html.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', len(body))
            self.end_headers()
            self.wfile.write(body)


def show(waypoints, telemetry, port=8080):
    global _telemetry_ref, _dashboard_html

    _telemetry_ref = telemetry

    logging.info("Building dashboard with %d waypoints", len(waypoints))
    map_html        = _build_map_html(waypoints)
    _dashboard_html = _build_dashboard(waypoints, map_html)

    logging.info("Dashboard running at http://localhost:%d", port)
    print(f"[GUI] Open → http://localhost:{port}")

    HTTPServer(("0.0.0.0", port), _Handler).serve_forever()