# dashboard.py
# ---------------------------------------------------------------------------
# Web interface (arayuz) for route_nav.
#
# Serves a single HTML page over HTTP that shows:
#   * a large live CAMERA feed (the debug overlay frame)
#   * a small live MASK feed
#   * a schematic MAP drawn as horizontal/vertical lines (auto-built from GRAPH)
#     showing the car, its current node, heading arrow, destination, the
#     predicted (planned) path, the next node and any blocked edges
#   * a control panel to type a START node, HEADING (N/E/S/W) and DESTINATION,
#     plus closed-road buttons (LEFT / RIGHT / FORWARD / CLEAR) and RESET
#   * a live telemetry readout
#
# Only the Python standard library + OpenCV (cv2) are used, so nothing extra
# needs to be installed on the Raspberry Pi.
#
# Public API used by route_nav:
#   dashboard.set_graph(GRAPH)          -> give it the map so it can lay it out
#   dashboard.set_control_callback(cb)  -> cb(dict) is called for each UI action
#   dashboard.start(port=8080)          -> start the server in a daemon thread
#   dashboard.push_frame(bgr_frame)     -> newest camera frame
#   dashboard.push_mask(mask_gray)      -> newest mask frame
#   dashboard.push_state(**fields)      -> newest navigation state (free-form)
# ---------------------------------------------------------------------------

import json
import threading
import time

import cv2

try:
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
except ImportError:  # very old Python fallback
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn

    class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True


JPEG_QUALITY = 70
STREAM_PERIOD = 0.04   # ~25 fps cap per video stream

# ---- shared state (written by route_nav, read by HTTP handlers) ----
_state_lock = threading.Lock()
_state = {}

_cam_lock = threading.Lock()
_cam_jpeg = None

_mask_lock = threading.Lock()
_mask_jpeg = None

_control_callback = None
_graph_layout = {"coords": {}, "edges": [], "nodes": []}

DIR_DELTA = {"NORTH": (0, -1), "SOUTH": (0, 1), "EAST": (1, 0), "WEST": (-1, 0)}


# ===========================================================================
# Public API
# ===========================================================================
def set_control_callback(cb):
    """Register a function cb(command_dict) called whenever the UI posts an
    action. It runs on the HTTP thread, so keep it light (route_nav just drops
    the dict into a queue and applies it on its own main thread)."""
    global _control_callback
    _control_callback = cb


def set_graph(graph):
    """Provide the navigation GRAPH so the map can be laid out automatically."""
    global _graph_layout
    _graph_layout = _compute_layout(graph)


def push_frame(frame):
    """Newest camera frame (BGR numpy array, as produced by OpenCV)."""
    global _cam_jpeg
    if frame is None:
        return
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    if ok:
        with _cam_lock:
            _cam_jpeg = buf.tobytes()


def push_mask(mask):
    """Newest mask frame (single-channel grayscale numpy array)."""
    global _mask_jpeg
    if mask is None:
        return
    ok, buf = cv2.imencode(".jpg", mask, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    if ok:
        with _mask_lock:
            _mask_jpeg = buf.tobytes()


def push_state(**fields):
    """Newest navigation state. Any keyword fields are accepted and forwarded
    to the browser verbatim as JSON."""
    with _state_lock:
        _state.update(fields)
        _state["_ts"] = time.time()


def start(port=8080):
    """Start the HTTP server on a background daemon thread."""
    server = ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    server.daemon_threads = True
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"[dashboard] interface running at http://<this-pi-ip>:{port}/")
    return server


# ===========================================================================
# Graph layout: assign grid (col,row) to every node by walking the directions.
# ===========================================================================
def _compute_layout(graph):
    coords = {}
    if not graph:
        return {"coords": {}, "edges": [], "nodes": []}

    start = "CROSS_CENTER" if "CROSS_CENTER" in graph else next(iter(graph))
    coords[start] = (0, 0)
    queue = [start]
    while queue:
        node = queue.pop(0)
        cx, cy = coords[node]
        for direction, edge in graph.get(node, {}).items():
            nxt = edge.get("to")
            if direction not in DIR_DELTA or nxt is None:
                continue
            dx, dy = DIR_DELTA[direction]
            if nxt not in coords:
                coords[nxt] = (cx + dx, cy + dy)
                queue.append(nxt)

    # also place any nodes the BFS never reached (defensive)
    spare = 0
    for node in graph:
        if node not in coords:
            coords[node] = (spare, 4)
            spare += 1

    edges = []
    seen = set()
    for node, nbrs in graph.items():
        for _direction, edge in nbrs.items():
            nxt = edge.get("to")
            if nxt is None or node not in coords or nxt not in coords:
                continue
            key = tuple(sorted((node, nxt)))
            if key in seen:
                continue
            seen.add(key)
            edges.append({"a": node, "b": nxt})

    return {
        "coords": {n: {"col": c[0], "row": c[1]} for n, c in coords.items()},
        "edges": edges,
        "nodes": sorted(coords.keys()),
    }


# ===========================================================================
# HTTP handler
# ===========================================================================
class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass  # stay quiet

    # ---- routing ----
    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            self._send_bytes(_PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/state":
            self._send_state()
        elif path == "/graph":
            self._send_json(_graph_layout)
        elif path == "/video/cam":
            self._stream("cam")
        elif path == "/video/mask":
            self._stream("mask")
        else:
            self.send_error(404)

    def do_POST(self):
        path = self.path.split("?")[0]
        if path != "/control":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", 0) or 0)
        body = self.rfile.read(length) if length else b"{}"
        try:
            data = json.loads(body.decode("utf-8"))
        except Exception:
            data = {}
        if _control_callback is not None:
            try:
                _control_callback(data)
            except Exception as e:
                print("[dashboard] control callback error:", e)
        self._send_json({"ok": True})

    # ---- helpers ----
    def _send_bytes(self, payload, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_json(self, obj):
        self._send_bytes(json.dumps(obj).encode("utf-8"), "application/json")

    def _send_state(self):
        with _state_lock:
            snapshot = dict(_state)
        snapshot["_now"] = time.time()
        self._send_json(snapshot)

    def _stream(self, which):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
        )
        self.end_headers()
        try:
            while True:
                if which == "cam":
                    with _cam_lock:
                        jpg = _cam_jpeg
                else:
                    with _mask_lock:
                        jpg = _mask_jpeg
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    ("Content-Length: %d\r\n\r\n" % len(jpg)).encode("ascii")
                )
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(STREAM_PERIOD)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            pass


# ===========================================================================
# The single-page interface
# ===========================================================================
_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>Kapsule Nav HUD</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Chakra+Petch:wght@500;700&family=JetBrains+Mono:wght@400;600&display=swap" rel="stylesheet">
<style>
  :root{
    --bg:#070b10; --panel:#0f1620; --panel2:#0b121b; --edge:#1d2a38;
    --ink:#cfe3f2; --muted:#65788a;
    --amber:#ffb000; --cyan:#36e0d0; --green:#52e36a; --red:#ff4d57;
    --grid:#142231;
    --mono:'JetBrains Mono',ui-monospace,Menlo,Consolas,monospace;
    --disp:'Chakra Petch',var(--mono);
  }
  *{box-sizing:border-box}
  html,body{margin:0;height:100%}
  body{
    background:
      radial-gradient(1200px 600px at 70% -10%, #0d1722 0%, transparent 60%),
      var(--bg);
    color:var(--ink); font-family:var(--mono); font-size:13px;
    -webkit-font-smoothing:antialiased;
  }
  a{color:var(--cyan)}
  .wrap{max-width:1480px;margin:0 auto;padding:14px 16px 28px}

  header{
    display:flex;align-items:center;gap:16px;flex-wrap:wrap;
    border:1px solid var(--edge);border-radius:10px;
    background:linear-gradient(180deg,#101a26,#0b131c);
    padding:12px 16px;margin-bottom:14px;
  }
  .brand{font-family:var(--disp);font-weight:700;letter-spacing:.14em;
    font-size:18px;text-transform:uppercase;color:#eaf4ff}
  .brand b{color:var(--amber)}
  .tagline{color:var(--muted);letter-spacing:.18em;font-size:11px;text-transform:uppercase}
  .spacer{flex:1}
  .pill{display:inline-flex;align-items:center;gap:7px;padding:5px 11px;
    border:1px solid var(--edge);border-radius:999px;background:#0c141d;
    font-size:11px;letter-spacing:.12em;text-transform:uppercase;color:var(--muted)}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--red);
    box-shadow:0 0 8px currentColor}
  .pill.live .dot{background:var(--green)}
  .pill.live{color:#bfe9c9}
  /* engel pill in the header */
  .pill.engel{border-color:var(--red);color:var(--red);display:none}
  .pill.engel.show{display:inline-flex;animation:engelblink 0.8s steps(1) infinite}
  .pill.engel .dot{background:var(--red)}
  @keyframes engelblink{50%{opacity:.35}}
  .statebadge{font-family:var(--disp);font-weight:700;letter-spacing:.12em;
    color:#06222a;background:var(--cyan);padding:6px 14px;border-radius:7px;
    text-transform:uppercase}

  .grid{display:grid;grid-template-columns:1.45fr 1fr;gap:14px;align-items:start}
  @media (max-width:980px){.grid{grid-template-columns:1fr}}

  .card{border:1px solid var(--edge);border-radius:10px;background:var(--panel);
    overflow:hidden}
  .card .hd{display:flex;align-items:center;gap:10px;padding:9px 13px;
    border-bottom:1px solid var(--edge);background:var(--panel2);
    font-family:var(--disp);letter-spacing:.16em;text-transform:uppercase;
    font-size:11px;color:#9fb6c9}
  .card .hd .accent{width:9px;height:9px;border-radius:2px;background:var(--amber)}
  .card .bd{padding:13px}

  .feedwrap{position:relative;background:#05080c;line-height:0}
  .feedwrap img{width:100%;display:block;background:#05080c}
  .feedtag{position:absolute;top:9px;left:9px;font-size:10px;letter-spacing:.18em;
    padding:3px 8px;background:rgba(7,11,16,.72);border:1px solid var(--edge);
    border-radius:5px;color:var(--cyan);text-transform:uppercase}
  /* ENGEL overlay banner on the camera feed */
  .engel-banner{position:absolute;left:50%;top:14px;transform:translateX(-50%);
    display:none;align-items:center;gap:9px;
    font-family:var(--disp);font-weight:700;letter-spacing:.18em;
    text-transform:uppercase;font-size:16px;color:#fff;
    padding:8px 18px;border-radius:8px;border:2px solid var(--red);
    background:rgba(255,77,87,.22);box-shadow:0 0 22px rgba(255,77,87,.6)}
  .engel-banner.show{display:flex;animation:engelblink 0.8s steps(1) infinite}
  .engel-banner .ic{font-size:18px}
  .feedwrap.engelframe{outline:3px solid var(--red);outline-offset:-3px}

  /* right column stacks */
  .stack{display:flex;flex-direction:column;gap:14px}
  .maskrow{display:grid;grid-template-columns:1fr;gap:14px}

  svg.map{width:100%;height:auto;display:block;background:
    repeating-linear-gradient(0deg,transparent 0 23px,#0a121b 23px 24px),
    repeating-linear-gradient(90deg,transparent 0 23px,#0a121b 23px 24px),
    #070d14}

  .legend{display:flex;flex-wrap:wrap;gap:12px 18px;padding:11px 13px 2px;
    color:var(--muted);font-size:11px;letter-spacing:.04em}
  .legend i{display:inline-block;width:14px;height:4px;border-radius:2px;
    margin-right:6px;vertical-align:middle}
  .lg-car{background:var(--amber)}.lg-dest{background:var(--red)}
  .lg-path{background:var(--cyan)}.lg-blk{background:#6b7787}
  .lg-next{background:var(--green)}

  /* controls */
  .controls{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
  @media (max-width:560px){.controls{grid-template-columns:1fr}}
  label{display:block;font-size:10px;letter-spacing:.16em;text-transform:uppercase;
    color:var(--muted);margin:0 0 5px 2px}
  select{width:100%;padding:9px 10px;background:#0a121b;color:var(--ink);
    border:1px solid var(--edge);border-radius:7px;font-family:var(--mono);
    font-size:13px;appearance:none}
  select:focus{outline:none;border-color:var(--cyan)}
  .btnrow{display:flex;flex-wrap:wrap;gap:9px;margin-top:13px}
  button{font-family:var(--disp);font-weight:700;letter-spacing:.1em;
    text-transform:uppercase;font-size:12px;cursor:pointer;
    border:1px solid var(--edge);border-radius:8px;padding:11px 16px;
    background:#0c141d;color:var(--ink);transition:.12s}
  button:hover{border-color:var(--cyan);color:#fff}
  button:active{transform:translateY(1px)}
  .b-go{background:var(--amber);color:#1a1200;border-color:var(--amber)}
  .b-go:hover{background:#ffc23a;color:#1a1200}
  .b-reset{border-color:#3a2530;color:#ffc0c8}
  .b-reset:hover{border-color:var(--red);color:#fff;background:#2a1418}
  .sub{font-size:10px;letter-spacing:.16em;text-transform:uppercase;
    color:var(--muted);margin:16px 0 8px}
  .chiprow{display:flex;flex-wrap:wrap;gap:8px}
  .chip{padding:8px 13px;border-radius:7px}
  .chip.on{outline:2px solid var(--amber);outline-offset:-2px}

  /* telemetry */
  .tele{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:var(--edge);
    border-radius:8px;overflow:hidden}
  .tele div{background:var(--panel);padding:9px 11px}
  .tele .k{color:var(--muted);font-size:10px;letter-spacing:.14em;text-transform:uppercase}
  .tele .v{color:#eaf4ff;font-size:14px;font-weight:600;margin-top:2px;word-break:break-word}
  .v.amber{color:var(--amber)}.v.cyan{color:var(--cyan)}.v.green{color:var(--green)}.v.red{color:var(--red)}
  .flags{display:flex;gap:7px;flex-wrap:wrap;margin-top:4px}
  .flag{font-size:10px;letter-spacing:.1em;padding:3px 8px;border-radius:5px;
    border:1px solid var(--edge);color:var(--muted);text-transform:uppercase}
  .flag.act{border-color:var(--cyan);color:var(--cyan)}
  .flag.danger{border-color:var(--red);color:var(--red)}
  .pathline{font-size:13px;color:#eaf4ff;letter-spacing:.02em}
  .pathline .arr{color:var(--muted);margin:0 4px}
  .pathline .now{color:var(--amber);font-weight:700}
  .pathline .end{color:var(--red);font-weight:700}
</style>
</head>
<body>
<div class="wrap">

  <header>
    <div>
      <div class="brand">KAPSUL <b>&#9679;</b> NAV HUD</div>
      <div class="tagline">graph navigation &middot; line follower</div>
    </div>
    <div class="spacer"></div>
    <span class="pill engel" id="engelpill"><span class="dot"></span>ENGEL</span>
    <span class="pill" id="link"><span class="dot"></span><span id="linktxt">connecting</span></span>
    <span class="pill"><span id="fps">--</span> fps</span>
    <span class="statebadge" id="state">--</span>
  </header>

  <div class="grid">
    <!-- LEFT: big camera + controls -->
    <div class="stack">
      <div class="card">
        <div class="hd"><span class="accent"></span>Camera &mdash; debug overlay</div>
        <div class="feedwrap" id="camwrap">
          <span class="feedtag">CAM</span>
          <div class="engel-banner" id="engelBanner"><span class="ic">&#9888;</span><span id="engelText">ENGEL - DUR</span></div>
          <img id="cam" src="/video/cam" alt="camera"/>
        </div>
      </div>

      <div class="card">
        <div class="hd"><span class="accent"></span>Route control</div>
        <div class="bd">
          <div class="controls">
            <div>
              <label>Start node</label>
              <select id="start"></select>
            </div>
            <div>
              <label>Heading</label>
              <select id="heading">
                <option>NORTH</option><option>EAST</option>
                <option selected>SOUTH</option><option>WEST</option>
              </select>
            </div>
            <div>
              <label>Destination</label>
              <select id="dest"></select>
            </div>
          </div>
          <div class="btnrow">
            <button class="b-go" onclick="setRoute()">Set route</button>
            <button class="b-reset" onclick="cmd({cmd:'reset'})">Reset</button>
          </div>

          <div class="sub">Quick destination</div>
          <div class="chiprow" id="quickdest"></div>

          <div class="sub">Closed road (applied at next junction)</div>
          <div class="chiprow">
            <button class="chip" id="fb-LEFT"    onclick="forbid('LEFT')">Left &times;</button>
            <button class="chip" id="fb-RIGHT"   onclick="forbid('RIGHT')">Right &times;</button>
            <button class="chip" id="fb-FORWARD" onclick="forbid('FORWARD')">Forward &times;</button>
            <button class="chip" onclick="forbid('CLEAR')">Clear</button>
          </div>
        </div>
      </div>
    </div>

    <!-- RIGHT: map + mask + telemetry -->
    <div class="stack">
      <div class="card">
        <div class="hd"><span class="accent"></span>Map &amp; route</div>
        <svg class="map" id="map" viewBox="0 0 400 300" preserveAspectRatio="xMidYMid meet"></svg>
        <div class="legend">
          <span><i class="lg-car"></i>Car / now</span>
          <span><i class="lg-next"></i>Next node</span>
          <span><i class="lg-dest"></i>Destination</span>
          <span><i class="lg-path"></i>Predicted path</span>
          <span><i class="lg-blk"></i>Blocked</span>
        </div>
      </div>

      <div class="card">
        <div class="hd"><span class="accent"></span>Mask</div>
        <div class="feedwrap">
          <span class="feedtag">MASK</span>
          <img id="mask" src="/video/mask" alt="mask"/>
        </div>
      </div>

      <div class="card">
        <div class="hd"><span class="accent"></span>Telemetry</div>
        <div class="bd">
          <div style="margin-bottom:11px">
            <div class="tele div k" style="background:none;padding:0 0 6px 0;color:var(--muted);font-size:10px;letter-spacing:.14em">PLAN</div>
            <div class="pathline" id="path">&mdash;</div>
          </div>
          <div class="tele">
            <div><div class="k">Current node</div><div class="v amber" id="t-node">--</div></div>
            <div><div class="k">Heading</div><div class="v cyan" id="t-head">--</div></div>
            <div><div class="k">Next node</div><div class="v green" id="t-next">--</div></div>
            <div><div class="k">Turn / dir</div><div class="v" id="t-act">--</div></div>
            <div><div class="k">Destination</div><div class="v red" id="t-dest">--</div></div>
            <div><div class="k">State</div><div class="v" id="t-state">--</div></div>
            <div><div class="k">vx / vy / wz</div><div class="v" id="t-vel">--</div></div>
            <div><div class="k">Error / yaw</div><div class="v" id="t-err">--</div></div>
            <div><div class="k">YOLO sign</div><div class="v" id="t-yolo">--</div></div>
            <div><div class="k">Sign rule</div><div class="v" id="t-rule">--</div></div>
            <div><div class="k">Engel</div><div class="v" id="t-engel">--</div></div>
            <div><div class="k">Engel alan</div><div class="v" id="t-engelarea">--</div></div>
          </div>
          <div class="sub">Line sensing</div>
          <div class="flags" id="lineflags"></div>
          <div class="sub">Forbidden / blocked</div>
          <div class="flags" id="blkflags"></div>
        </div>
      </div>
    </div>
  </div>
</div>

<script>
const DIR_DELTA = {NORTH:[0,-1],SOUTH:[0,1],EAST:[1,0],WEST:[-1,0]};
const HEAD_ANGLE = {EAST:0,SOUTH:90,WEST:180,NORTH:-90};
let LAYOUT = {coords:{},edges:[],nodes:[]};
let VIEW = null;
const SVGNS = "http://www.w3.org/2000/svg";

function el(tag, attrs){const e=document.createElementNS(SVGNS,tag);
  for(const k in attrs) e.setAttribute(k, attrs[k]); return e;}

async function loadGraph(){
  try{
    LAYOUT = await (await fetch('/graph')).json();
  }catch(e){ return; }
  // populate selects
  const nodes = LAYOUT.nodes || [];
  const startSel=document.getElementById('start');
  const destSel=document.getElementById('dest');
  startSel.innerHTML=''; destSel.innerHTML='';
  nodes.forEach(n=>{
    startSel.appendChild(new Option(n,n));
    destSel.appendChild(new Option(n,n));
  });
  // quick-dest chips for short single-letter nodes (A/B/C ...)
  const qd=document.getElementById('quickdest'); qd.innerHTML='';
  nodes.filter(n=>n.length<=2).forEach(n=>{
    const b=document.createElement('button');
    b.className='chip'; b.textContent=n;
    b.onclick=()=>cmd({cmd:'set_dest',dest:n});
    qd.appendChild(b);
  });
  computeView();
}

function computeView(){
  const c=LAYOUT.coords||{}; const keys=Object.keys(c);
  if(!keys.length){VIEW=null;return;}
  let cols=keys.map(k=>c[k].col), rows=keys.map(k=>c[k].row);
  const minC=Math.min(...cols),maxC=Math.max(...cols);
  const minR=Math.min(...rows),maxR=Math.max(...rows);
  const CELL=120, M=64;
  const W=M*2+(maxC-minC)*CELL, H=M*2+(maxR-minR)*CELL;
  VIEW={minC,minR,CELL,M,W,H};
  document.getElementById('map').setAttribute('viewBox',`0 0 ${W} ${H}`);
}
function px(node){const c=LAYOUT.coords[node];if(!c||!VIEW)return null;
  return [VIEW.M+(c.col-VIEW.minC)*VIEW.CELL, VIEW.M+(c.row-VIEW.minR)*VIEW.CELL];}
function neighborPx(node,dir){
  const c=LAYOUT.coords[node]; const d=DIR_DELTA[dir]; if(!c||!d||!VIEW)return null;
  return [VIEW.M+(c.col+d[0]-VIEW.minC)*VIEW.CELL, VIEW.M+(c.row+d[1]-VIEW.minR)*VIEW.CELL];
}

function drawMap(s){
  const svg=document.getElementById('map');
  if(!VIEW){return;}
  svg.innerHTML='';

  const path=(s.path||[]);
  const pathSet=new Set();
  for(let i=0;i<path.length-1;i++){pathSet.add(path[i]+'|'+path[i+1]);pathSet.add(path[i+1]+'|'+path[i]);}
  const blocked=(s.blocked_edges||s.blocked||[]);

  // faint backbone grid through every node row/col
  const cols=new Set(),rows=new Set();
  for(const n in LAYOUT.coords){cols.add(LAYOUT.coords[n].col);rows.add(LAYOUT.coords[n].row);}
  cols.forEach(col=>{const x=VIEW.M+(col-VIEW.minC)*VIEW.CELL;
    svg.appendChild(el('line',{x1:x,y1:14,x2:x,y2:VIEW.H-14,stroke:'#142231','stroke-width':1}));});
  rows.forEach(r=>{const y=VIEW.M+(r-VIEW.minR)*VIEW.CELL;
    svg.appendChild(el('line',{x1:14,y1:y,x2:VIEW.W-14,y2:y,stroke:'#142231','stroke-width':1}));});

  // roads (graph edges)
  (LAYOUT.edges||[]).forEach(e=>{
    const a=px(e.a),b=px(e.b); if(!a||!b)return;
    const onPath=pathSet.has(e.a+'|'+e.b);
    svg.appendChild(el('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],
      stroke:onPath?'#36e0d0':'#2b3e52','stroke-width':onPath?7:5,
      'stroke-linecap':'round',opacity:onPath?1:.85}));
  });

  // blocked edges (red dashed)
  blocked.forEach(tok=>{
    const parts=String(tok).split(':'); if(parts.length<2)return;
    const a=px(parts[0]); const b=neighborPx(parts[0],parts[1]); if(!a||!b)return;
    svg.appendChild(el('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],
      stroke:'#6b7787','stroke-width':6,'stroke-dasharray':'4 7','stroke-linecap':'round'}));
    const mx=(a[0]+b[0])/2,my=(a[1]+b[1])/2;
    svg.appendChild(el('circle',{cx:mx,cy:my,r:10,fill:'#16202c',stroke:'#ff4d57','stroke-width':2}));
    const t=el('text',{x:mx,y:my+4,'text-anchor':'middle',fill:'#ff4d57','font-size':13,'font-weight':700});
    t.textContent='\u2715'; svg.appendChild(t);
  });

  // nodes
  for(const n of (LAYOUT.nodes||[])){
    const p=px(n); if(!p)continue;
    const isNow=n===s.current_node, isDest=n===s.destination, isNext=n===s.next_node;
    svg.appendChild(el('circle',{cx:p[0],cy:p[1],r:13,fill:'#0b141e',
      stroke:isDest?'#ff4d57':(isNext?'#52e36a':'#3a516a'),'stroke-width':isDest||isNext?3:2}));
    const label=el('text',{x:p[0],y:p[1]-22,'text-anchor':'middle',
      fill:isNow?'#ffb000':(isDest?'#ff8a90':'#9fb6c9'),'font-size':13,
      'font-family':'Chakra Petch, monospace','font-weight':700});
    label.textContent=n; svg.appendChild(label);
  }

  // destination ring
  if(s.destination){const p=px(s.destination);
    if(p)svg.appendChild(el('circle',{cx:p[0],cy:p[1],r:21,fill:'none',
      stroke:'#ff4d57','stroke-width':2,'stroke-dasharray':'3 5'}));}

  // next-node pulse
  if(s.next_node){const p=px(s.next_node);
    if(p){const c=el('circle',{cx:p[0],cy:p[1],r:19,fill:'none',stroke:'#52e36a','stroke-width':2,opacity:.9});
      const an=el('animate',{attributeName:'r',values:'17;25;17',dur:'1.6s',repeatCount:'indefinite'});
      c.appendChild(an); svg.appendChild(c);}}

  // CAR at current node with heading arrow
  if(s.current_node){
    const p=px(s.current_node);
    if(p){
      const ang=HEAD_ANGLE[s.current_heading]!==undefined?HEAD_ANGLE[s.current_heading]:-90;
      const g=el('g',{transform:`translate(${p[0]},${p[1]}) rotate(${ang})`});
      g.appendChild(el('circle',{cx:0,cy:0,r:15,fill:'#ffb000',opacity:.16}));
      // arrow/chevron pointing +x (then rotated by heading)
      const tri=el('polygon',{points:'16,0 -9,-10 -3,0 -9,10',fill:'#ffb000',
        stroke:'#1a1200','stroke-width':1.5,'stroke-linejoin':'round'});
      g.appendChild(tri);
      svg.appendChild(g);
    }
  }
}

// ---------- telemetry ----------
function flag(txt,on){return `<span class="flag ${on?'act':''}">${txt}</span>`;}
function fmtPath(p){
  if(!p||!p.length)return '&mdash;';
  return p.map((n,i)=>{
    let cls=''; if(i===0)cls='now'; if(i===p.length-1&&p.length>1)cls='end';
    return `<span class="${cls}">${n}</span>`;
  }).join('<span class="arr">&rarr;</span>');
}

function applyState(s){
  document.getElementById('state').textContent=s.state_name||'--';
  document.getElementById('fps').textContent=(s.fps!=null)?Number(s.fps).toFixed(0):'--';
  document.getElementById('path').innerHTML=fmtPath(s.path||s.route);
  document.getElementById('t-node').textContent=s.current_node||'--';
  document.getElementById('t-head').textContent=s.current_heading||'--';
  document.getElementById('t-next').textContent=s.next_node||'--';
  document.getElementById('t-act').textContent=(s.next_action||'--')+' '+(s.target_direction?('/ '+s.target_direction):'');
  document.getElementById('t-dest').textContent=s.destination||'--';
  document.getElementById('t-state').textContent=s.state_name||'--';
  document.getElementById('t-vel').textContent=`${s.vx??0} / ${s.vy??0} / ${s.wz??0}`;
  document.getElementById('t-err').textContent=`${(s.error!=null)?Number(s.error).toFixed(1):'--'} / ${(s.yaw!=null)?Number(s.yaw).toFixed(1):'--'}`;
  document.getElementById('t-yolo').textContent=`${s.yolo_label||'YOK'} (${(s.yolo_conf!=null)?Number(s.yolo_conf).toFixed(2):'0.00'})`;
  document.getElementById('t-rule').textContent=s.yolo_rule||'NONE';

  // ----- ENGEL (obstacle) indicators -----
  const stopped = !!s.obstacle;          // confirmed -> car is held
  const seen    = !!s.obstacle_seen;     // a blob is visible this frame
  const area    = (s.obstacle_area!=null)?Number(s.obstacle_area):0;
  const engelEl = document.getElementById('t-engel');
  engelEl.textContent = stopped ? 'DUR (STOP)' : (seen ? 'gorundu' : 'yok');
  engelEl.className = 'v ' + (stopped ? 'red' : (seen ? 'amber' : 'green'));
  document.getElementById('t-engelarea').textContent = area ? area.toFixed(0) : '--';

  const banner = document.getElementById('engelBanner');
  const camwrap = document.getElementById('camwrap');
  const engelpill = document.getElementById('engelpill');
  banner.classList.toggle('show', stopped);
  camwrap.classList.toggle('engelframe', stopped);
  engelpill.classList.toggle('show', stopped);
  document.getElementById('engelText').textContent =
    'ENGEL - DUR' + (area ? ('  (' + area.toFixed(0) + ')') : '');

  document.getElementById('lineflags').innerHTML=
    flag('TWO',s.two)+flag('LEFT-ONLY',s.only_left)+flag('RIGHT-ONLY',s.only_right)+
    flag('NO-LINE',s.no_line)+(s.no_path?flag('NO-PATH',true):'')+
    (stopped?'<span class="flag danger">ENGEL</span>':'');

  const forb=s.forbidden||[]; const blk=s.blocked_edges||s.blocked||[];
  let bh='';
  ['LEFT','RIGHT','FORWARD'].forEach(r=>bh+=flag(r,forb.includes(r)));
  blk.forEach(b=>bh+=`<span class="flag" style="border-color:#3a2530;color:#ff8a90">${b}</span>`);
  document.getElementById('blkflags').innerHTML=bh||'<span class="flag">none</span>';

  // sync forbidden chip highlight
  ['LEFT','RIGHT','FORWARD'].forEach(r=>{
    const b=document.getElementById('fb-'+r); if(b)b.classList.toggle('on',forb.includes(r));
  });

  drawMap(s);
}

// ---------- polling ----------
async function poll(){
  try{
    const s=await (await fetch('/state',{cache:'no-store'})).json();
    const fresh=(s._now-s._ts)<2.0;
    const link=document.getElementById('link');
    link.classList.toggle('live',fresh);
    document.getElementById('linktxt').textContent=fresh?'live':'stale';
    applyState(s);
  }catch(e){
    document.getElementById('link').classList.remove('live');
    document.getElementById('linktxt').textContent='offline';
  }
}

// ---------- commands ----------
async function cmd(obj){
  try{await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(obj)});}catch(e){}
  setTimeout(poll,120);
}
function setRoute(){
  cmd({cmd:'set_route',
    start:document.getElementById('start').value,
    heading:document.getElementById('heading').value,
    dest:document.getElementById('dest').value});
}
function forbid(rule){cmd({cmd:'forbid',rule:rule});}

loadGraph().then(()=>{poll();setInterval(poll,220);});
</script>
</body>
</html>
"""
