import numpy as np
import folium
import logging
import json
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
    m = folium.Map(location=[waypoints[0][0], waypoints[0][1]], zoom_start=20)
    folium.Marker([waypoints[0][0],  waypoints[0][1]],  popup="Start",
                  icon=folium.Icon(color="green")).add_to(m)
    folium.Marker([waypoints[-1][0], waypoints[-1][1]], popup="End",
                  icon=folium.Icon(color="red")).add_to(m)
    for i, (lat, lon) in enumerate(waypoints):
        folium.CircleMarker([lat, lon], radius=5, color="#378ADD",
                            fill=True, fill_opacity=0.8,
                            popup=f"WP {i}").add_to(m)
    folium.PolyLine(waypoints, color="#E24B4A", weight=3,
                    dash_array="8 4", tooltip="Planned path").add_to(m)
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
<title>ASV Ground Control</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600&family=Syne:wght@700&display=swap');
  :root {{
    --bg:      #0a0c10;
    --surface: #111318;
    --border:  #1e2330;
    --accent:  #00d4ff;
    --red:     #ff4757;
    --green:   #2ed573;
    --amber:   #ffa502;
    --text:    #c8cdd8;
    --dim:     #4a5068;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    background: var(--bg); color: var(--text);
    font-family: 'JetBrains Mono', monospace; font-size: 13px;
    height: 100vh; display: flex; flex-direction: column; overflow: hidden;
  }}
  header {{
    display: flex; align-items: center; gap: 16px;
    padding: 10px 20px; border-bottom: 1px solid var(--border);
    background: var(--surface); flex-shrink: 0;
  }}
  .logo {{
    font-family: 'Syne', sans-serif; font-size: 16px; font-weight: 700;
    color: var(--accent); letter-spacing: 3px;
  }}
  .pill {{
    padding: 3px 10px; border-radius: 20px; font-size: 11px;
    font-weight: 600; letter-spacing: 1px;
    background: #00d4ff18; border: 1px solid var(--accent); color: var(--accent);
  }}
  .stat-bar {{ margin-left: auto; display: flex; gap: 24px; }}
  .stat {{ display: flex; flex-direction: column; align-items: flex-end; }}
  .stat-label {{ font-size: 10px; color: var(--dim); letter-spacing: 1px; }}
  .stat-value {{ font-size: 13px; font-weight: 600; }}
  .tabs {{
    display: flex; gap: 2px; padding: 8px 20px 0;
    background: var(--surface); border-bottom: 1px solid var(--border); flex-shrink: 0;
  }}
  .tab {{
    padding: 7px 20px; border: none; background: transparent;
    color: var(--dim); font-family: 'JetBrains Mono', monospace;
    font-size: 12px; cursor: pointer; border-bottom: 2px solid transparent;
    transition: all 0.2s; letter-spacing: 1px;
  }}
  .tab:hover {{ color: var(--text); }}
  .tab.active {{ color: var(--accent); border-bottom-color: var(--accent); }}
  .panel {{ display: none; flex: 1; overflow: hidden; }}
  .panel.active {{ display: flex; }}
  #panel-map {{ flex-direction: column; }}
  #map-container {{ flex: 1; border: none; width: 100%; }}
  #panel-graphs {{
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 1fr;
    gap: 1px; background: var(--border); overflow: hidden;
  }}
  .card {{
    background: var(--surface); padding: 14px 16px 10px;
    display: flex; flex-direction: column; min-height: 0;
  }}
  .card-title {{
    font-size: 10px; letter-spacing: 2px; color: var(--dim);
    text-transform: uppercase; margin-bottom: 8px; flex-shrink: 0;
  }}
  .card canvas {{ flex: 1; min-height: 0; }}
  .card.tall {{ grid-row: span 2; }}
</style>
</head>
<body>

<header>
  <div class="logo">ASV · GCS</div>
  <div class="pill" id="hdr-status">WAITING</div>
  <div class="stat-bar">
    <div class="stat"><span class="stat-label">LAT</span><span class="stat-value" id="hdr-lat">—</span></div>
    <div class="stat"><span class="stat-label">LON</span><span class="stat-value" id="hdr-lon">—</span></div>
    <div class="stat"><span class="stat-label">HEADING</span><span class="stat-value" id="hdr-hdg">—</span></div>
    <div class="stat"><span class="stat-label">WP</span><span class="stat-value" id="hdr-wp">—</span></div>
    <div class="stat"><span class="stat-label">DIST</span><span class="stat-value" id="hdr-dist">—</span></div>
  </div>
</header>

<div class="tabs">
  <button class="tab active" onclick="switchTab('map',this)">MAP</button>
  <button class="tab"        onclick="switchTab('graphs',this)">GRAPHS</button>
</div>

<div class="panel active" id="panel-map">
  <div id="map-container"></div>
</div>

<div class="panel" id="panel-graphs">
  <div class="card tall">
    <div class="card-title">Actual path vs planned</div>
    <canvas id="c-path"></canvas>
  </div>
  <div class="card">
    <div class="card-title">Heading vs desired bearing (°)</div>
    <canvas id="c-heading"></canvas>
  </div>
  <div class="card">
    <div class="card-title">Cross-track error (m)</div>
    <canvas id="c-xte"></canvas>
  </div>
  <div class="card">
    <div class="card-title">Roll &amp; Pitch (°)</div>
    <canvas id="c-rp"></canvas>
  </div>
  <div class="card">
    <div class="card-title">Distance to waypoint (m)</div>
    <canvas id="c-dist"></canvas>
  </div>
</div>

<script>
// ── Inject map HTML ────────────────────────────────────────────
document.getElementById('map-container').innerHTML = `{map_escaped}`;
const mapEl = document.getElementById('map-container').firstElementChild;
if (mapEl) {{ mapEl.style.cssText = 'width:100%;height:100%;border:none;'; }}

// ── Tab switching ──────────────────────────────────────────────
function switchTab(name, btn) {{
  document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
  document.getElementById('panel-' + name).classList.add('active');
  btn.classList.add('active');
  if (name === 'map') window.dispatchEvent(new Event('resize'));
}}

// ── Chart helpers ──────────────────────────────────────────────
const GRID  = '#1e2330';
const TICK  = {{ color: '#4a5068', font: {{ family: 'JetBrains Mono', size: 10 }} }};
const SCALE = {{ x: {{ ticks: TICK, grid: {{ color: GRID }} }},
                 y: {{ ticks: TICK, grid: {{ color: GRID }} }} }};
const BASE  = {{
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

// ── Charts ────────────────────────────────────────────────────
const WP_XY  = {waypoints_js};
const WP_RAW = {waypoints_raw};

const pathChart = new Chart(document.getElementById('c-path'), {{
  type: 'scatter',
  data: {{ datasets: [
    {{ label: 'Planned', data: WP_XY, borderColor: '#ff4757',
       backgroundColor: 'transparent', showLine: true, borderWidth: 2,
       borderDash: [6,3], pointRadius: 5, pointBackgroundColor: '#ff4757' }},
    {{ label: 'Actual',  data: [], borderColor: '#00d4ff',
       backgroundColor: 'transparent', showLine: true, borderWidth: 1.5,
       pointRadius: 2, pointBackgroundColor: '#00d4ff' }}
  ]}},
  options: {{
    ...BASE,
    plugins: {{ legend: {{ display: true,
      labels: {{ color: '#4a5068', font: {{ family: 'JetBrains Mono', size: 10 }} }} }} }},
    scales: {{
      x: {{ ...SCALE.x, title: {{ display: true, text: 'X (m)', color: '#4a5068' }} }},
      y: {{ ...SCALE.y, title: {{ display: true, text: 'Y (m)', color: '#4a5068' }} }}
    }}
  }}
}});

const headingChart = new Chart(document.getElementById('c-heading'), {{
  type: 'line',
  data: {{ labels: [], datasets: [ ds('#00d4ff','Heading'), ds('#ffa502','Desired') ] }},
  options: {{ ...BASE, plugins: {{ legend: {{ display: true,
    labels: {{ color: '#4a5068', font: {{ family: 'JetBrains Mono', size: 10 }} }} }} }} }}
}});

const xteChart = new Chart(document.getElementById('c-xte'), {{
  type: 'line',
  data: {{ labels: [], datasets: [ ds('#ff4757','XTE') ] }},
  options: BASE
}});

const rpChart = new Chart(document.getElementById('c-rp'), {{
  type: 'line',
  data: {{ labels: [], datasets: [ ds('#2ed573','Roll'), ds('#ffa502','Pitch') ] }},
  options: {{ ...BASE, plugins: {{ legend: {{ display: true,
    labels: {{ color: '#4a5068', font: {{ family: 'JetBrains Mono', size: 10 }} }} }} }} }}
}});

const distChart = new Chart(document.getElementById('c-dist'), {{
  type: 'line',
  data: {{ labels: [], datasets: [ ds('#a29bfe','Dist') ] }},
  options: BASE
}});

// ── XY conversion ──────────────────────────────────────────────
const R_E = 6378137, lat0 = WP_RAW[0][0], lon0 = WP_RAW[0][1];
function toXY(lat, lon) {{
  return {{
    x: +(R_E*(lon-lon0)*Math.PI/180*Math.cos(lat0*Math.PI/180)).toFixed(3),
    y: +(R_E*(lat-lat0)*Math.PI/180).toFixed(3)
  }};
}}

// ── Poll /telemetry ────────────────────────────────────────────
async function poll() {{
  try {{
    const d   = await (await fetch('/telemetry')).json();
    const ts  = new Date().toLocaleTimeString('en', {{hour12:false}});

    document.getElementById('hdr-status').textContent = d.nav_status;
    document.getElementById('hdr-lat').textContent    = d.gps_lat.toFixed(6);
    document.getElementById('hdr-lon').textContent    = d.gps_lon.toFixed(6);
    document.getElementById('hdr-hdg').textContent    = d.heading.toFixed(1) + '°';
    document.getElementById('hdr-wp').textContent     = d.active_wp;
    document.getElementById('hdr-dist').textContent   = d.dist_to_waypoint.toFixed(1) + 'm';

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

  }} catch(e) {{ /* server not ready, retry silently */ }}
}}

setInterval(poll, 200);
poll();
</script>
</body>
</html>"""

# ── Minimal HTTP server ───────────────────────────────────────────────────────
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