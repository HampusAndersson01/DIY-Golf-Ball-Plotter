from __future__ import annotations

from flask import Flask, request, jsonify, render_template_string
import serial
import time
import threading
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, asdict
from typing import Optional, Any

# Optional dependency for real SVG path support:
#   pip install svgpathtools pyserial flask
try:
    from svgpathtools import parse_path
except Exception:
    parse_path = None

# ============================================================
# Machine / serial setup
# ============================================================

SERIAL_PORT = "COM12"
BAUD_RATE = 115200

MOTOR_FULL_STEPS_PER_REV = 200
X_MICROSTEPS = 16
Y_MICROSTEPS = 16

X_STEPS_PER_DEGREE = (MOTOR_FULL_STEPS_PER_REV * X_MICROSTEPS) / 360.0
Y_STEPS_PER_DEGREE = (MOTOR_FULL_STEPS_PER_REV * Y_MICROSTEPS) / 360.0

# X is ball rotation. Y is arm tilt.
# Drawing area on the ball is centered on the origin so calibration at 0,0
# means the pen is physically in the middle of the ball.
#   X: -180..+180 degrees
#   Y: -45..+45 degrees = 90 degree drawing band
X_DRAW_MIN = -180.0
X_DRAW_MAX = 180.0
Y_DRAW_MIN = -45.0
Y_DRAW_MAX = 45.0

BALL_CENTER_X = 0.0
BALL_CENTER_Y = 0.0
BALL_DIAMETER_MM = 42.67

DEFAULT_X_MAX_FEED = 6000       # degrees/min
DEFAULT_Y_MAX_FEED = 6000       # degrees/min
DEFAULT_X_ACCELERATION = 100    # degrees/sec^2
DEFAULT_Y_ACCELERATION = 100    # degrees/sec^2
DEFAULT_DRAW_FEED = 1200        # degrees/min for drawing
DEFAULT_TRAVEL_FEED = 3000      # degrees/min for pen-up travel
DEFAULT_LINE_THICKNESS_MM = 0.0  # pen stroke width on the ball in mm

# Servo via GRBL spindle PWM M3 S...
DEFAULT_PEN_UP_S = 400
DEFAULT_PEN_DOWN_S = 900
DEFAULT_SERVO_DWELL = 0.25
DEFAULT_PEN_LIFT_STEPS = 8
DEFAULT_PEN_LIFT_STEP_DWELL = 0.03
MIN_SERVO_S = 0
MAX_SERVO_S = 1000

# SVG flattening defaults
DEFAULT_SAMPLE_STEP_DEG = 1.0       # max angular spacing between sampled points
DEFAULT_CURVE_SAMPLES = 80          # fallback per curve/path segment
DEFAULT_MARGIN_PERCENT = 4.0        # keep SVG away from extreme edges

app = Flask(__name__)

serial_lock = threading.Lock()
grbl: Optional[serial.Serial] = None

job_lock = threading.Lock()
job_thread: Optional[threading.Thread] = None
job_stop_requested = False
job_pause_requested = False

state: dict[str, Any] = {
    "connected": False,
    "calibrated": False,
    "running": False,
    "paused": False,
    "status": "Not connected",
    "last_svg_name": None,
    "last_gcode": [],
    "last_preview": [],
    "last_error": None,
    "progress_total": 0,
    "progress_done": 0,
}


# ============================================================
# Data models
# ============================================================

@dataclass
class Point:
    x: float
    y: float


@dataclass
class Segment:
    points: list[Point]
    closed: bool = False


@dataclass
class SvgBounds:
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @property
    def width(self) -> float:
        return max(0.000001, self.max_x - self.min_x)

    @property
    def height(self) -> float:
        return max(0.000001, self.max_y - self.min_y)


# ============================================================
# HTML UI
# ============================================================

HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>SVG → G-code Golf Ball Plotter</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0f172a;
      --panel: #111827;
      --panel2: #1f2937;
      --border: #374151;
      --text: #f9fafb;
      --muted: #9ca3af;
      --blue: #2563eb;
      --green: #16a34a;
      --red: #dc2626;
      --orange: #d97706;
      --purple: #7c3aed;
    }

    * { box-sizing: border-box; }

    body {
      font-family: Arial, sans-serif;
      background: radial-gradient(circle at top, #1e293b 0, var(--bg) 45%);
      color: var(--text);
      margin: 0;
      padding: 24px;
    }

    .container { max-width: 1480px; margin: 0 auto; }
    h1 { margin: 0 0 8px; }
    h2 { margin: 0 0 12px; font-size: 1.15rem; }
    h3 { margin: 14px 0 8px; font-size: 1rem; }
    .subtitle { color: var(--muted); margin-bottom: 18px; line-height: 1.45; }

    .statusbar {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(190px, 1fr));
      gap: 10px;
      margin-bottom: 16px;
    }

    .statusbox {
      background: rgba(17, 24, 39, 0.85);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 12px;
    }

    .statusbox .label { color: var(--muted); font-size: 0.85rem; }
    .statusbox .value { font-size: 1.05rem; margin-top: 4px; font-weight: bold; }

    .grid {
      display: grid;
      grid-template-columns: 410px 1fr;
      gap: 16px;
      align-items: start;
    }

    .left, .right { display: grid; gap: 16px; }

    .card {
      background: rgba(31, 41, 55, 0.92);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 18px;
      box-shadow: 0 18px 50px rgba(0,0,0,0.18);
    }

    label {
      display: block;
      margin-top: 12px;
      margin-bottom: 6px;
      color: #d1d5db;
      font-size: 0.92rem;
    }

    input, select, textarea {
      width: 100%;
      padding: 10px;
      border-radius: 9px;
      border: 1px solid #4b5563;
      background: #030712;
      color: var(--text);
    }

    input[type="file"] { background: #111827; }
    input[type="checkbox"] { width: auto; }

    textarea {
      min-height: 280px;
      font-family: Consolas, Monaco, monospace;
      font-size: 12px;
      white-space: pre;
    }

    button {
      padding: 10px 14px;
      margin: 6px 4px 6px 0;
      border: none;
      border-radius: 9px;
      background: var(--blue);
      color: white;
      cursor: pointer;
      font-weight: bold;
    }

    button:hover { filter: brightness(0.92); }
    button:disabled { opacity: 0.45; cursor: not-allowed; }
    button.danger { background: var(--red); }
    button.secondary { background: #4b5563; }
    button.success { background: var(--green); }
    button.warning-btn { background: var(--orange); }
    button.purple { background: var(--purple); }

    .row { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
    .two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .three-col { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; }

    .small { color: var(--muted); font-size: 0.9rem; line-height: 1.45; }
    .warning { color: #fbbf24; }
    .danger-text { color: #fca5a5; }
    .ok-text { color: #86efac; }

    .pill {
      display: inline-block;
      background: #030712;
      border: 1px solid var(--border);
      padding: 6px 9px;
      border-radius: 999px;
      margin: 4px 4px 0 0;
      font-size: 0.86rem;
      color: #e5e7eb;
    }

    pre {
      background: #030712;
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px;
      min-height: 260px;
      max-height: 520px;
      overflow: auto;
      white-space: pre-wrap;
      font-family: Consolas, Monaco, monospace;
      font-size: 12px;
    }

    .preview-wrap {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 16px;
    }

    .preview-panel {
      background: #030712;
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 12px;
      min-height: 420px;
    }

    .preview-title { color: var(--muted); font-size: 0.9rem; margin-bottom: 8px; }

    #svgPreview, #flatPreview, #ballPreview {
      width: 100%;
      height: 380px;
      background: #020617;
      border-radius: 10px;
      overflow: hidden;
    }

    #svgPreview {
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 10px;
    }

    #svgPreview svg {
      width: 100%;
      height: 100%;
      max-width: 100%;
      max-height: 100%;
      object-fit: contain;
    }

    #svgPreview svg, #svgPreview img {
      display: block;
    }

    canvas { width: 100%; height: 380px; display: block; border-radius: 10px; background: #020617; }

    .progress {
      width: 100%;
      height: 14px;
      background: #030712;
      border: 1px solid var(--border);
      border-radius: 999px;
      overflow: hidden;
      margin-top: 8px;
    }

    .progress-fill {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, #2563eb, #7c3aed);
      transition: width 0.2s ease;
    }

    @media (max-width: 980px) {
      .grid { grid-template-columns: 1fr; }
      .preview-wrap { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>SVG → G-code Golf Ball Plotter</h1>
    <div class="subtitle">
      Upload SVG → flatten paths → map to centered ball coordinates → manually calibrate/zero → run on GRBL.
      This uses X and Y as angular degrees, not millimeters.
    </div>

    <div class="statusbar">
      <div class="statusbox"><div class="label">Serial</div><div id="serialStatus" class="value">-</div></div>
      <div class="statusbox"><div class="label">Calibration</div><div id="calStatus" class="value">-</div></div>
      <div class="statusbox"><div class="label">Runner</div><div id="runStatus" class="value">-</div></div>
      <div class="statusbox"><div class="label">Progress</div><div id="progressStatus" class="value">0 / 0</div></div>
    </div>

    <div class="grid">
      <div class="left">
        <div class="card">
          <h2>1. Connect / setup</h2>
          <div class="row">
            <button onclick="api('/connect')" class="success">Connect</button>
            <button onclick="sendCommand('?')">Status ?</button>
            <button onclick="sendCommand('$$')">$$ Settings</button>
            <button onclick="sendCommand('$I')">$I Firmware</button>
            <button onclick="softReset()" class="danger">Soft Reset</button>
          </div>

          <div class="two-col">
            <div>
              <label>X max feed</label>
              <input id="xMaxFeed" type="number" value="{{ default_x_max_feed }}" step="100" />
            </div>
            <div>
              <label>Y max feed</label>
              <input id="yMaxFeed" type="number" value="{{ default_y_max_feed }}" step="100" />
            </div>
          </div>

          <div class="two-col">
            <div>
              <label>X acceleration</label>
              <input id="xAcceleration" type="number" value="{{ default_x_acceleration }}" step="10" />
            </div>
            <div>
              <label>Y acceleration</label>
              <input id="yAcceleration" type="number" value="{{ default_y_acceleration }}" step="10" />
            </div>
          </div>

          <button onclick="applyMachineConfig()" class="success">Apply GRBL Settings</button>
          <p class="small warning">
            Testing mode disables soft limits, hard limits and homing. Current position is manually zeroed.
          </p>
          <p>
            <span class="pill">$100 {{ x_steps_per_degree }}</span>
            <span class="pill">$101 {{ y_steps_per_degree }}</span>
            <span class="pill">$30 1000</span>
          </p>
        </div>

        <div class="card">
          <h2>2. Manual calibration</h2>
          <p class="small">
            Jog the machine until the pen is physically at the center of the ball. Then set zero and mark calibrated.
            The runner will not start until this is done.
          </p>

          <div class="row">
            <button onclick="penUp()" class="success">Pen Up</button>
            <button onclick="penDown()" class="warning-btn">Pen Down</button>
            <button onclick="penTest()" class="secondary">Pen Test</button>
            <button onclick="servoOff()" class="danger">Servo Off</button>
          </div>

          <div class="two-col">
            <div><label>Pen up S</label><input id="penUpS" type="number" value="{{ default_pen_up_s }}" min="0" max="1000" step="10" /></div>
            <div><label>Pen down S</label><input id="penDownS" type="number" value="{{ default_pen_down_s }}" min="0" max="1000" step="10" /></div>
          </div>
          <label>Servo dwell seconds</label>
          <input id="servoDwell" type="number" value="{{ default_servo_dwell }}" min="0" max="5" step="0.05" />

          <h3>Jog</h3>
          <div class="two-col">
            <div>
              <label>X jog degrees</label>
              <input id="xJog" type="number" value="1" step="0.1" />
            </div>
            <div>
              <label>Y jog degrees</label>
              <input id="yJog" type="number" value="1" step="0.1" />
            </div>
          </div>
          <div class="row">
            <button onclick="jogX(-getNum('xJog'))" class="secondary">X -</button>
            <button onclick="jogX(getNum('xJog'))" class="secondary">X +</button>
            <button onclick="jogY(-getNum('yJog'))" class="secondary">Y -</button>
            <button onclick="jogY(getNum('yJog'))" class="secondary">Y +</button>
          </div>
          <div class="row">
            <button onclick="zeroPosition()" class="warning-btn">Zero X/Y Here</button>
            <button onclick="markCalibrated()" class="success">I Have Calibrated</button>
            <button onclick="clearCalibrated()" class="danger">Clear Calibration</button>
          </div>
        </div>

        <div class="card">
          <h2>3. SVG import</h2>
          <input id="svgFile" type="file" accept=".svg,image/svg+xml" />

          <div class="three-col">
            <div>
              <label>Placement scale %</label>
              <input id="placementScale" type="number" value="100" step="1" min="5" max="300" />
            </div>
            <div>
              <label>Placement X offset</label>
              <input id="placementOffsetX" type="number" value="0" step="1" min="-180" max="180" />
            </div>
            <div>
              <label>Placement Y offset</label>
              <input id="placementOffsetY" type="number" value="0" step="1" min="-45" max="45" />
            </div>
          </div>

          <div class="two-col">
            <div>
              <label>Draw feed</label>
              <input id="drawFeed" type="number" value="{{ default_draw_feed }}" step="100" />
            </div>
            <div>
              <label>Travel feed</label>
              <input id="travelFeed" type="number" value="{{ default_travel_feed }}" step="100" />
            </div>
          </div>

          <div class="two-col">
            <div>
              <label>Sample step degrees</label>
              <input id="sampleStepDeg" type="number" value="{{ default_sample_step_deg }}" step="0.1" min="0.05" />
            </div>
            <div>
              <label>Margin %</label>
              <input id="marginPercent" type="number" value="{{ default_margin_percent }}" step="0.5" min="0" max="25" />
            </div>
          </div>

          <label>Line spacing / nozzle diameter (mm on ball)</label>
          <input id="lineThicknessMm" type="number" value="{{ default_line_thickness_mm }}" step="0.01" min="0" max="10" />
          <p class="small">
            Example: 0.75 mm sets the spacing between contour passes. Open lines stay single-pass; closed shapes can be filled with nested contours.
          </p>

          <label>Fit mode</label>
          <select id="fitMode">
            <option value="contain" selected>Contain entire SVG inside ball drawing area</option>
            <option value="stretch">Stretch to full 360 x 90 area</option>
          </select>

          <label><input id="invertY" type="checkbox" checked /> Invert SVG Y so top of SVG becomes +Y</label>
          <label><input id="includeComments" type="checkbox" checked /> Include comments in G-code</label>

          <button onclick="uploadAndGenerate()" class="purple">Upload SVG + Generate G-code</button>
          <button onclick="downloadGcode()" class="secondary">Download G-code</button>
        </div>

        <div class="card">
          <h2>4. Run</h2>
          <p class="small warning">
            Run is locked until calibration is marked complete. Keep hand near power/USB disconnect during early tests.
          </p>
          <div class="row">
            <button onclick="runGcode()" class="success">Run Generated G-code</button>
            <button onclick="pauseRun()" class="warning-btn">Pause / Feed Hold</button>
            <button onclick="resumeRun()" class="success">Resume</button>
            <button onclick="stopRun()" class="danger">Stop</button>
          </div>
          <div class="progress"><div id="progressFill" class="progress-fill"></div></div>
        </div>

        <div class="card">
          <h2>Raw command</h2>
          <input id="rawCommand" type="text" value="M3 S{{ default_pen_up_s }}" />
          <button onclick="sendRaw()">Send</button>
          <p class="small">
            Example: G90 G1 X180 Y0 F1200<br>
            Pen: M3 S{{ default_pen_up_s }} / M3 S{{ default_pen_down_s }}
          </p>
        </div>
      </div>

      <div class="right">
        <div class="card">
          <h2>Preview</h2>
          <div class="preview-wrap">
            <div class="preview-panel">
              <div class="preview-title">Original SVG</div>
              <div id="svgPreview"><span class="small">No SVG loaded</span></div>
            </div>
            <div class="preview-panel">
              <div class="preview-title">Flat coordinate map</div>
              <canvas id="flatPreview"></canvas>
            </div>
            <div class="preview-panel">
              <div class="preview-title">Ball preview</div>
              <canvas id="ballPreview"></canvas>
            </div>
          </div>
        </div>

        <div class="card">
          <h2>Generated G-code</h2>
          <textarea id="gcodeBox" spellcheck="false"></textarea>
        </div>

        <div class="card">
          <h2>Log</h2>
          <pre id="log"></pre>
        </div>
      </div>
    </div>
  </div>

<script>
  let latestGcode = [];
  let latestPreview = [];

  function appendLog(text) {
    const log = document.getElementById("log");
    const now = new Date().toLocaleTimeString();
    log.textContent += `[${now}] ${text}\n`;
    log.scrollTop = log.scrollHeight;
  }

  function getNum(id) {
    return parseFloat(document.getElementById(id).value || "0");
  }

  function getInt(id) {
    return parseInt(document.getElementById(id).value || "0");
  }

  async function api(url, data = {}) {
    try {
      const res = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(data)
      });
      const json = await res.json();
      if (json.ok) {
        if (json.command) appendLog(`> ${json.command}`);
        if (json.response) appendLog(`< ${json.response}`);
      } else {
        appendLog(`ERROR: ${json.error}`);
      }
      await refreshState();
      return json;
    } catch (err) {
      appendLog(`FETCH ERROR: ${err}`);
      return { ok: false, error: String(err) };
    }
  }

  async function refreshState() {
    try {
      const res = await fetch('/state');
      const s = await res.json();
      document.getElementById('serialStatus').textContent = s.connected ? 'Connected' : 'Disconnected';
      document.getElementById('serialStatus').className = s.connected ? 'value ok-text' : 'value danger-text';
      document.getElementById('calStatus').textContent = s.calibrated ? 'Ready' : 'Not calibrated';
      document.getElementById('calStatus').className = s.calibrated ? 'value ok-text' : 'value warning';
      document.getElementById('runStatus').textContent = s.status || '-';
      document.getElementById('progressStatus').textContent = `${s.progress_done || 0} / ${s.progress_total || 0}`;

      const pct = s.progress_total ? Math.round((s.progress_done / s.progress_total) * 100) : 0;
      document.getElementById('progressFill').style.width = `${pct}%`;
    } catch (_) {}
  }

  function sendCommand(command) { return api('/command', { command }); }
  function sendRaw() { return sendCommand(document.getElementById('rawCommand').value); }
  function softReset() { return api('/reset'); }

  function applyMachineConfig() {
    return api('/apply-config', {
      x_max_feed: getNum('xMaxFeed'),
      y_max_feed: getNum('yMaxFeed'),
      x_acceleration: getNum('xAcceleration'),
      y_acceleration: getNum('yAcceleration')
    });
  }

  function penUp() {
    return api('/pen-up', {
      s: getInt('penUpS'),
      dwell: getNum('servoDwell')
    });
  }

  function penDown() {
    return api('/pen-down', {
      s: getInt('penDownS'),
      dwell: getNum('servoDwell')
    });
  }

  function penTest() {
    return api('/pen-test', {
      up_s: getInt('penUpS'),
      down_s: getInt('penDownS'),
      dwell: getNum('servoDwell')
    });
  }

  function servoOff() { return api('/servo-off'); }
  function jogX(degrees) { return api('/jog', { axis: 'X', degrees, feed: getNum('travelFeed') }); }
  function jogY(degrees) { return api('/jog', { axis: 'Y', degrees, feed: getNum('travelFeed') }); }
  function zeroPosition() { return api('/zero-position'); }
  function markCalibrated() { return api('/mark-calibrated'); }
  function clearCalibrated() { return api('/clear-calibrated'); }
  function runGcode() { return api('/run-gcode'); }
  function pauseRun() { return api('/pause'); }
  function resumeRun() { return api('/resume'); }
  function stopRun() { return api('/stop'); }

  async function uploadAndGenerate() {
    const file = document.getElementById('svgFile').files[0];
    if (!file) {
      appendLog('ERROR: Choose an SVG file first.');
      return;
    }

    const text = await file.text();
    document.getElementById('svgPreview').innerHTML = text;

    const form = new FormData();
    form.append('svg', file);
    form.append('draw_feed', getNum('drawFeed'));
    form.append('travel_feed', getNum('travelFeed'));
    form.append('sample_step_deg', getNum('sampleStepDeg'));
    form.append('margin_percent', getNum('marginPercent'));
    form.append('fit_mode', document.getElementById('fitMode').value);
    form.append('invert_y', document.getElementById('invertY').checked ? '1' : '0');
    form.append('include_comments', document.getElementById('includeComments').checked ? '1' : '0');
    form.append('placement_scale', getNum('placementScale'));
    form.append('placement_offset_x', getNum('placementOffsetX'));
    form.append('placement_offset_y', getNum('placementOffsetY'));
    form.append('line_thickness_mm', getNum('lineThicknessMm'));
    form.append('pen_up_s', getInt('penUpS'));
    form.append('pen_down_s', getInt('penDownS'));
    form.append('servo_dwell', getNum('servoDwell'));

    try {
      const res = await fetch('/generate-gcode', { method: 'POST', body: form });
      const json = await res.json();
      if (!json.ok) {
        appendLog(`ERROR: ${json.error}`);
        return;
      }

      latestGcode = json.gcode;
      latestPreview = json.preview;
      document.getElementById('gcodeBox').value = latestGcode.join('\n');
      drawFlatPreview(latestPreview);
      drawBallPreview(latestPreview);
      appendLog(`Generated ${latestGcode.length} G-code lines from ${json.segment_count} SVG segments / ${json.point_count} points.`);
      await refreshState();
    } catch (err) {
      appendLog(`GENERATE ERROR: ${err}`);
    }
  }

  function setupCanvas(canvasId) {
    const canvas = document.getElementById(canvasId);
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.max(300, Math.floor(rect.width * window.devicePixelRatio));
    canvas.height = Math.max(300, Math.floor(rect.height * window.devicePixelRatio));
    return canvas.getContext('2d');
  }

  function drawFlatPreview(paths) {
    const ctx = setupCanvas('flatPreview');
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#020617';
    ctx.fillRect(0, 0, w, h);

    const pad = 24 * window.devicePixelRatio;
    const plotW = w - pad * 2;
    const plotH = h - pad * 2;
    const scale = Math.min(plotW / 360, plotH / 90);
    const drawW = 360 * scale;
    const drawH = 90 * scale;
    const left = (w - drawW) / 2;
    const top = (h - drawH) / 2;
    const sx = (x) => left + (x + 180) * scale;
    const sy = (y) => top + (45 - y) * scale;

    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1 * window.devicePixelRatio;
    ctx.strokeRect(left, top, drawW, drawH);

    ctx.fillStyle = 'rgba(37, 99, 235, 0.45)';
    ctx.fillRect(sx(0) - 1 * window.devicePixelRatio, top, 2 * window.devicePixelRatio, drawH);
    ctx.fillRect(left, sy(0) - 1 * window.devicePixelRatio, drawW, 2 * window.devicePixelRatio);

    ctx.fillStyle = '#94a3b8';
    ctx.font = `${11 * window.devicePixelRatio}px Arial`;
    ctx.fillText('X -180°', left, h - 7 * window.devicePixelRatio);
    ctx.fillText('X 0°', cxLabel(w, scale, left), h - 7 * window.devicePixelRatio);
    ctx.fillText('X +180°', w - left - 48 * window.devicePixelRatio, h - 7 * window.devicePixelRatio);
    ctx.fillText('Y +45°', 8 * window.devicePixelRatio, top + 4 * window.devicePixelRatio);
    ctx.fillText('Y 0°', 8 * window.devicePixelRatio, cyLabel(h, scale, top));
    ctx.fillText('Y -45°', 8 * window.devicePixelRatio, h - top);

    ctx.lineWidth = 1.7 * window.devicePixelRatio;
    ctx.strokeStyle = '#a78bfa';
    for (const path of paths) {
      if (!path.length) continue;
      ctx.beginPath();
      ctx.moveTo(sx(path[0].x), sy(path[0].y));
      for (let i = 1; i < path.length; i++) {
        ctx.lineTo(sx(path[i].x), sy(path[i].y));
      }
      ctx.stroke();
    }
  }

  function cxLabel(width, scale, left) {
    return left + (180 * scale) - 18 * window.devicePixelRatio;
  }

  function cyLabel(height, scale, top) {
    return top + (45 * scale) + 4 * window.devicePixelRatio;
  }

  function projectBallPoint(point, cx, cy, radius) {
    const lon = (point.x * Math.PI) / 180;
    const lat = (point.y * Math.PI) / 180;
    const x = Math.cos(lat) * Math.sin(lon);
    const y = Math.sin(lat);
    const z = Math.cos(lat) * Math.cos(lon);
    const perspective = 0.72 + (z * 0.28);
    return {
      x: cx + x * radius * perspective,
      y: cy - y * radius * perspective,
      z
    };
  }

  function drawBallPreview(paths) {
    const ctx = setupCanvas('ballPreview');
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#020617';
    ctx.fillRect(0, 0, w, h);

    const radius = Math.min(w, h) * 0.38;
    const cx = w / 2;
    const cy = h / 2;

    const sphere = ctx.createRadialGradient(cx - radius * 0.35, cy - radius * 0.35, radius * 0.1, cx, cy, radius);
    sphere.addColorStop(0, '#e2e8f0');
    sphere.addColorStop(0.22, '#94a3b8');
    sphere.addColorStop(0.65, '#334155');
    sphere.addColorStop(1, '#0f172a');
    ctx.fillStyle = sphere;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1 * window.devicePixelRatio;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.stroke();

    ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
    ctx.lineWidth = 1.2 * window.devicePixelRatio;
    ctx.beginPath();
    ctx.arc(cx, cy, radius * 0.56, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cx - radius, cy);
    ctx.lineTo(cx + radius, cy);
    ctx.moveTo(cx, cy - radius);
    ctx.lineTo(cx, cy + radius);
    ctx.stroke();

    ctx.strokeStyle = 'rgba(148, 163, 184, 0.18)';
    ctx.lineWidth = 1 * window.devicePixelRatio;
    for (const lonDeg of [-120, -60, 0, 60, 120]) {
      ctx.beginPath();
      for (let latDeg = -45; latDeg <= 45; latDeg += 3) {
        const p = projectBallPoint({ x: lonDeg, y: latDeg }, cx, cy, radius);
        if (latDeg === -45) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
    }
    ctx.beginPath();
    for (let lonDeg = -180; lonDeg <= 180; lonDeg += 4) {
      const p = projectBallPoint({ x: lonDeg, y: 0 }, cx, cy, radius);
      if (lonDeg === -180) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();

    const projectedPaths = [];
    for (const path of paths) {
      if (!path.length) continue;
      projectedPaths.push(path.map((point) => projectBallPoint(point, cx, cy, radius)));
    }
    projectedPaths.sort((a, b) => (a[0]?.z || 0) - (b[0]?.z || 0));

    for (const projected of projectedPaths) {
      ctx.beginPath();
      for (let i = 0; i < projected.length; i++) {
        const p = projected[i];
        if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      }
      const depth = projected.reduce((sum, p) => sum + p.z, 0) / projected.length;
      const alpha = Math.max(0.2, Math.min(1, 0.35 + (depth + 1) * 0.325));
      ctx.lineWidth = 2.2 * window.devicePixelRatio;
      ctx.strokeStyle = `rgba(167, 139, 250, ${alpha})`;
      ctx.stroke();
    }

    ctx.fillStyle = 'rgba(255, 255, 255, 0.38)';
    ctx.beginPath();
    ctx.arc(cx - radius * 0.32, cy - radius * 0.34, radius * 0.16, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#cbd5e1';
    ctx.font = `${11 * window.devicePixelRatio}px Arial`;
    ctx.fillText('Front', cx - 16 * window.devicePixelRatio, cy + radius + 16 * window.devicePixelRatio);
    ctx.fillText('0° / 0°', cx - 20 * window.devicePixelRatio, cy + 4 * window.devicePixelRatio);
  }

  function downloadGcode() {
    const text = document.getElementById('gcodeBox').value;
    if (!text.trim()) {
      appendLog('ERROR: No G-code generated.');
      return;
    }
    const blob = new Blob([text], { type: 'text/plain' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'golfball_plotter_output.gcode';
    a.click();
    URL.revokeObjectURL(a.href);
  }

  window.addEventListener('resize', () => {
    drawFlatPreview(latestPreview);
    drawBallPreview(latestPreview);
  });
  setInterval(refreshState, 750);
  refreshState();
  appendLog('Controller loaded. Connect, apply settings, upload SVG, calibrate, then run.');
</script>
</body>
</html>
"""


# ============================================================
# Serial helpers
# ============================================================

def connect_grbl() -> serial.Serial:
    global grbl

    if grbl and grbl.is_open:
        state["connected"] = True
        return grbl

    grbl = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=3)
    time.sleep(2)

    grbl.write(b"\r\n\r\n")
    time.sleep(1)

    while grbl.in_waiting:
        line = grbl.readline().decode(errors="ignore").strip()
        if line:
            print(line)

    state["connected"] = True
    state["status"] = "Connected"
    return grbl


def read_available_lines(ser: serial.Serial) -> list[str]:
    lines: list[str] = []
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            lines.append(line)
    return lines


def read_until_ok_or_error(ser: serial.Serial, timeout: float = 15) -> str:
    end_time = time.time() + timeout
    lines: list[str] = []

    while time.time() < end_time:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        lines.append(line)
        if line == "ok" or line.startswith("error:") or line.startswith("ALARM:"):
            break

    return "\n".join(lines) if lines else "NO RESPONSE"


def wait_until_idle_unlocked(ser: serial.Serial, timeout: float = 60) -> bool:
    end_time = time.time() + timeout

    while time.time() < end_time:
        ser.write(b"?")
        time.sleep(0.12)
        lines = read_available_lines(ser)
        for line in lines:
            if line.startswith("<Idle"):
                return True
            if line.startswith("<Alarm") or line.startswith("ALARM:"):
                return False
        time.sleep(0.05)

    return False


def send_to_grbl_unlocked(ser: serial.Serial, command: str, timeout: float = 15) -> str:
    command = command.strip()
    if not command:
        raise ValueError("Empty command")

    if command == "?":
        ser.write(b"?")
        time.sleep(0.2)
        lines = read_available_lines(ser)
        return "\n".join(lines) if lines else "NO STATUS RESPONSE"

    ser.write((command + "\n").encode("utf-8"))
    response = read_until_ok_or_error(ser, timeout=timeout)
    if "error:" in response or "ALARM:" in response:
        raise RuntimeError(f"GRBL rejected command {command}: {response}")
    return response


def send_to_grbl(command: str, timeout: float = 15) -> str:
    with serial_lock:
        ser = connect_grbl()
        return send_to_grbl_unlocked(ser, command, timeout=timeout)


def send_many(commands: list[str], delay: float = 0.04, wait_idle_between: bool = True) -> str:
    results: list[str] = []
    with serial_lock:
        ser = connect_grbl()
        for cmd in commands:
            if wait_idle_between:
                wait_until_idle_unlocked(ser)
            response = send_to_grbl_unlocked(ser, cmd)
            results.append(f"{cmd} -> {response}")
            time.sleep(delay)
    return "\n".join(results)


# ============================================================
# Validation helpers
# ============================================================

def validate_feed(feed: Any) -> float:
    value = float(feed)
    if value <= 0:
        raise ValueError("Feed rate must be greater than 0")
    if value > 100000:
        raise ValueError("Feed rate is too high")
    return value


def validate_degrees(degrees: Any) -> float:
    value = float(degrees)
    if abs(value) > 100000:
        raise ValueError("Degree value is too large")
    return value


def validate_y_degrees(degrees: Any) -> float:
    value = float(degrees)
    if value < Y_DRAW_MIN or value > Y_DRAW_MAX:
        raise ValueError(f"Y angle must be between {Y_DRAW_MIN} and {Y_DRAW_MAX} degrees")
    return value


def validate_servo_s(s_value: Any) -> int:
    value = int(s_value)
    if value < MIN_SERVO_S or value > MAX_SERVO_S:
        raise ValueError(f"Servo S value must be between {MIN_SERVO_S} and {MAX_SERVO_S}")
    return value


def validate_dwell(dwell: Any) -> float:
    value = float(dwell)
    if value < 0:
        raise ValueError("Dwell must not be negative")
    if value > 5:
        raise ValueError("Dwell is too long")
    return value


def build_servo_transition_commands(start_s: int, end_s: int, steps: int = DEFAULT_PEN_LIFT_STEPS) -> list[str]:
  if steps <= 1 or start_s == end_s:
    return [f"M3 S{end_s}"]

  commands: list[str] = []
  last_s: Optional[int] = None
  for step in range(1, steps + 1):
    s_value = round(start_s + ((end_s - start_s) * step / steps))
    if s_value == last_s:
      continue
    commands.append(f"M3 S{s_value}")
    if step != steps:
      commands.append(f"G4 P{DEFAULT_PEN_LIFT_STEP_DWELL:.3f}")
    last_s = s_value

  return commands


def mm_to_ball_degrees(mm: float) -> float:
  if mm < 0:
    raise ValueError("Line thickness must not be negative")
  return (mm / BALL_DIAMETER_MM) * 360.0


# ============================================================
# SVG parsing and flattening
# ============================================================

def strip_namespace(tag: str) -> str:
    return tag.split("}", 1)[-1] if "}" in tag else tag


def parse_float(value: Optional[str], default: float = 0.0) -> float:
    if value is None:
        return default
    match = re.search(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", value)
    return float(match.group(0)) if match else default


def parse_points_attr(points: str) -> list[Point]:
    nums = [float(n) for n in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", points or "")]
    pts: list[Point] = []
    for i in range(0, len(nums) - 1, 2):
        pts.append(Point(nums[i], nums[i + 1]))
    return pts


def path_d_to_segments(d: str, curve_samples: int) -> list[Segment]:
    if parse_path is None:
        raise RuntimeError("Install svgpathtools for <path> support: pip install svgpathtools")

    path = parse_path(d)
    if len(path) == 0:
        return []

    segments: list[Segment] = []
    current: list[Point] = []
    last_end: Optional[complex] = None

    for part in path:
        start = part.start
        end = part.end

        if last_end is not None and abs(start - last_end) > 1e-6:
            if len(current) >= 2:
                segments.append(Segment(current, closed=False))
            current = []

        # Use length-aware sampling when possible.
        try:
            length = max(1.0, float(part.length(error=1e-4)))
            samples = max(2, min(300, int(length / 2.0) + 2))
        except Exception:
            samples = curve_samples

        for i in range(samples):
            t = i / (samples - 1)
            p = part.point(t)
            pt = Point(float(p.real), float(p.imag))
            if current and abs(current[-1].x - pt.x) < 1e-9 and abs(current[-1].y - pt.y) < 1e-9:
                continue
            current.append(pt)

        last_end = end

    if len(current) >= 2:
        closed = abs(complex(current[0].x, current[0].y) - complex(current[-1].x, current[-1].y)) < 1e-6
        segments.append(Segment(current, closed=closed))

    return segments


def circle_to_segment(cx: float, cy: float, r: float, samples: int = 120) -> Segment:
    pts = []
    for i in range(samples + 1):
        a = (i / samples) * math.tau
        pts.append(Point(cx + math.cos(a) * r, cy + math.sin(a) * r))
    return Segment(pts, closed=True)


def ellipse_to_segment(cx: float, cy: float, rx: float, ry: float, samples: int = 120) -> Segment:
    pts = []
    for i in range(samples + 1):
        a = (i / samples) * math.tau
        pts.append(Point(cx + math.cos(a) * rx, cy + math.sin(a) * ry))
    return Segment(pts, closed=True)


def rect_to_segment(x: float, y: float, w: float, h: float) -> Segment:
    return Segment([
        Point(x, y), Point(x + w, y), Point(x + w, y + h),
        Point(x, y + h), Point(x, y)
    ], closed=True)


def extract_svg_segments(svg_text: str) -> tuple[list[Segment], Optional[SvgBounds]]:
    root = ET.fromstring(svg_text)
    segments: list[Segment] = []

    viewbox_bounds: Optional[SvgBounds] = None
    viewbox = root.attrib.get("viewBox")
    if viewbox:
        nums = [float(n) for n in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", viewbox)]
        if len(nums) == 4:
            viewbox_bounds = SvgBounds(nums[0], nums[1], nums[0] + nums[2], nums[1] + nums[3])

    for elem in root.iter():
        tag = strip_namespace(elem.tag)

        # Ignore hidden elements where possible.
        style = elem.attrib.get("style", "")
        if elem.attrib.get("display") == "none" or "display:none" in style.replace(" ", ""):
            continue

        if tag == "path":
            d = elem.attrib.get("d", "").strip()
            if d:
                segments.extend(path_d_to_segments(d, DEFAULT_CURVE_SAMPLES))

        elif tag == "polyline":
            pts = parse_points_attr(elem.attrib.get("points", ""))
            if len(pts) >= 2:
                segments.append(Segment(pts, closed=False))

        elif tag == "polygon":
            pts = parse_points_attr(elem.attrib.get("points", ""))
            if len(pts) >= 2:
                if pts[0] != pts[-1]:
                    pts.append(Point(pts[0].x, pts[0].y))
                segments.append(Segment(pts, closed=True))

        elif tag == "line":
            x1 = parse_float(elem.attrib.get("x1"))
            y1 = parse_float(elem.attrib.get("y1"))
            x2 = parse_float(elem.attrib.get("x2"))
            y2 = parse_float(elem.attrib.get("y2"))
            segments.append(Segment([Point(x1, y1), Point(x2, y2)], closed=False))

        elif tag == "rect":
            x = parse_float(elem.attrib.get("x"))
            y = parse_float(elem.attrib.get("y"))
            w = parse_float(elem.attrib.get("width"))
            h = parse_float(elem.attrib.get("height"))
            if w > 0 and h > 0:
                segments.append(rect_to_segment(x, y, w, h))

        elif tag == "circle":
            cx = parse_float(elem.attrib.get("cx"))
            cy = parse_float(elem.attrib.get("cy"))
            r = parse_float(elem.attrib.get("r"))
            if r > 0:
                segments.append(circle_to_segment(cx, cy, r))

        elif tag == "ellipse":
            cx = parse_float(elem.attrib.get("cx"))
            cy = parse_float(elem.attrib.get("cy"))
            rx = parse_float(elem.attrib.get("rx"))
            ry = parse_float(elem.attrib.get("ry"))
            if rx > 0 and ry > 0:
                segments.append(ellipse_to_segment(cx, cy, rx, ry))

    return segments, viewbox_bounds


def bounds_from_segments(segments: list[Segment]) -> SvgBounds:
    pts = [p for seg in segments for p in seg.points]
    if not pts:
        raise ValueError("SVG contains no drawable paths/shapes")
    return SvgBounds(
        min_x=min(p.x for p in pts),
        min_y=min(p.y for p in pts),
        max_x=max(p.x for p in pts),
        max_y=max(p.y for p in pts),
    )


def resample_segment(points: list[Point], max_step: float) -> list[Point]:
    if len(points) < 2:
        return points

    out = [points[0]]
    for a, b in zip(points, points[1:]):
        dx = b.x - a.x
        dy = b.y - a.y
        dist = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(dist / max_step)))
        for i in range(1, steps + 1):
            t = i / steps
            out.append(Point(a.x + dx * t, a.y + dy * t))
    return out


def map_svg_to_angles(
    segments: list[Segment],
    bounds: SvgBounds,
    fit_mode: str,
    invert_y: bool,
    margin_percent: float,
) -> list[Segment]:
    margin_x = (X_DRAW_MAX - X_DRAW_MIN) * (margin_percent / 100.0)
    margin_y = (Y_DRAW_MAX - Y_DRAW_MIN) * (margin_percent / 100.0)

    target_w = (X_DRAW_MAX - X_DRAW_MIN) - margin_x * 2
    target_h = (Y_DRAW_MAX - Y_DRAW_MIN) - margin_y * 2

    if target_w <= 0 or target_h <= 0:
        raise ValueError("Margin is too large")

    if fit_mode == "stretch":
        scale_x = target_w / bounds.width
        scale_y = target_h / bounds.height
        offset_x = X_DRAW_MIN + margin_x
        offset_y = Y_DRAW_MIN + margin_y
    else:
        scale = min(target_w / bounds.width, target_h / bounds.height)
        scale_x = scale_y = scale
        used_w = bounds.width * scale
        used_h = bounds.height * scale
        offset_x = X_DRAW_MIN + margin_x + (target_w - used_w) / 2.0
        offset_y = Y_DRAW_MIN + margin_y + (target_h - used_h) / 2.0

    mapped_segments: list[Segment] = []
    for seg in segments:
        mapped: list[Point] = []
        for p in seg.points:
            nx = p.x - bounds.min_x
            ny = p.y - bounds.min_y
            x_deg = offset_x + nx * scale_x

            if invert_y:
                y_deg = offset_y + (bounds.height - ny) * scale_y
            else:
                y_deg = offset_y + ny * scale_y

            # Clamp tiny float overshoots.
            x_deg = min(X_DRAW_MAX, max(X_DRAW_MIN, x_deg))
            y_deg = min(Y_DRAW_MAX, max(Y_DRAW_MIN, y_deg))
            mapped.append(Point(x_deg, y_deg))

        if len(mapped) >= 2:
            mapped_segments.append(Segment(mapped, closed=seg.closed))

    return mapped_segments


def apply_placement_transform(
    segments: list[Segment],
    scale_percent: float,
    offset_x: float,
    offset_y: float,
) -> list[Segment]:
    if scale_percent <= 0:
        raise ValueError("Placement scale must be greater than 0")

    if not segments:
        return []

    scale = scale_percent / 100.0
    bounds = bounds_from_segments(segments)
    center_x = (bounds.min_x + bounds.max_x) / 2.0
    center_y = (bounds.min_y + bounds.max_y) / 2.0

    placed_segments: list[Segment] = []
    for seg in segments:
        placed_points: list[Point] = []
        for p in seg.points:
            x = center_x + (p.x - center_x) * scale + offset_x
            y = center_y + (p.y - center_y) * scale + offset_y
            y = min(Y_DRAW_MAX, max(Y_DRAW_MIN, y))
            placed_points.append(Point(x, y))

        if len(placed_points) >= 2:
            placed_segments.append(Segment(placed_points, closed=seg.closed))

    return placed_segments


def expand_segment_for_thickness(segment: Segment, line_thickness_deg: float) -> list[Segment]:
  if line_thickness_deg <= 0:
    return [segment]

  points = list(segment.points)
  if len(points) < 2:
    return [segment]

  if segment.closed and len(points) >= 3:
    first = points[0]
    last = points[-1]
    if abs(first.x - last.x) < 1e-9 and abs(first.y - last.y) < 1e-9:
      points = points[:-1]

  if len(points) < 2:
    return [segment]

  if not segment.closed:
    return [Segment(points, closed=False)]

  bounds = bounds_from_segments([Segment(points, closed=True)])
  center_x = (bounds.min_x + bounds.max_x) / 2.0
  center_y = (bounds.min_y + bounds.max_y) / 2.0
  average_radius = sum(math.hypot(p.x - center_x, p.y - center_y) for p in points) / len(points)

  if average_radius <= 1e-9:
    return [Segment(points + [Point(points[0].x, points[0].y)], closed=True)]

  scale_step = min(0.35, max(0.02, line_thickness_deg / average_radius))
  expanded: list[Segment] = []
  scale = 1.0

  while scale > 0.02:
    scaled_points = [
      Point(
        center_x + (p.x - center_x) * scale,
        center_y + (p.y - center_y) * scale,
      )
      for p in points
    ]
    scaled_points.append(Point(scaled_points[0].x, scaled_points[0].y))
    expanded.append(Segment(scaled_points, closed=True))

    scale -= scale_step

  return expanded


def expand_segments_for_thickness(segments: list[Segment], line_thickness_deg: float) -> list[Segment]:
  if line_thickness_deg <= 0:
    return segments

  expanded: list[Segment] = []
  for segment in segments:
    expanded.extend(expand_segment_for_thickness(segment, line_thickness_deg))
  return expanded


def generate_gcode_from_segments(
    segments: list[Segment],
    draw_feed: float,
    travel_feed: float,
    sample_step_deg: float,
    pen_up_s: int,
    pen_down_s: int,
    servo_dwell: float,
    line_thickness_deg: float,
    include_comments: bool,
) -> tuple[list[str], list[list[dict[str, float]]]]:
    g: list[str] = []
    preview: list[list[dict[str, float]]] = []

    def comment(text: str) -> None:
        if include_comments:
            g.append(f"({text})")

    comment("Generated for golf ball plotter")
    comment("Units are angular degrees. X=-180..180 ball rotation, Y=-45..45 arm tilt")
    g.extend([
        "$X",
        "G21",
        "G90",
        f"M3 S{pen_up_s}",
        f"G4 P{servo_dwell:.3f}",
    ])

    drawable_segments = expand_segments_for_thickness(segments, line_thickness_deg)

    for index, seg in enumerate(drawable_segments, start=1):
        pts = resample_segment(seg.points, max_step=max(0.05, sample_step_deg))
        if len(pts) < 2:
            continue

        preview.append([asdict(p) for p in pts])
        start = pts[0]

        comment(f"Segment {index}, {len(pts)} points")
        g.append(f"G1 X{start.x:.4f} Y{start.y:.4f} F{travel_feed:.3f}")
        g.append(f"M3 S{pen_down_s}")
        g.append(f"G4 P{servo_dwell:.3f}")

        for p in pts[1:]:
            g.append(f"G1 X{p.x:.4f} Y{p.y:.4f} F{draw_feed:.3f}")

        g.extend(build_servo_transition_commands(pen_down_s, pen_up_s))
        g.append(f"G4 P{servo_dwell:.3f}")

    comment("Return to zero with pen up")
    g.append(f"G1 X0.0000 Y0.0000 F{travel_feed:.3f}")
    g.append(f"M3 S{pen_up_s}")
    g.append(f"G4 P{servo_dwell:.3f}")

    return g, preview


def is_streamable_gcode_line(line: str) -> bool:
    s = line.strip()
    if not s:
        return False
    if s.startswith("(") and s.endswith(")"):
        return False
    return True


# ============================================================
# Runner
# ============================================================

def run_gcode_worker(gcode: list[str]) -> None:
    global job_stop_requested, job_pause_requested

    stream_lines = [line for line in gcode if is_streamable_gcode_line(line)]

    try:
        state["running"] = True
        state["paused"] = False
        state["status"] = "Running"
        state["progress_total"] = len(stream_lines)
        state["progress_done"] = 0
        state["last_error"] = None

        with serial_lock:
            ser = connect_grbl()

            for line in stream_lines:
                with job_lock:
                    if job_stop_requested:
                        state["status"] = "Stopped"
                        break
                    paused = job_pause_requested

                while paused:
                    state["paused"] = True
                    state["status"] = "Paused"
                    time.sleep(0.1)
                    with job_lock:
                        if job_stop_requested:
                            state["status"] = "Stopped"
                            return
                        paused = job_pause_requested

                state["paused"] = False
                state["status"] = f"Running: {line}"

                send_to_grbl_unlocked(ser, line, timeout=20)
                state["progress_done"] += 1

            wait_until_idle_unlocked(ser, timeout=120)

        if state["status"] != "Stopped":
            state["status"] = "Finished"

    except Exception as e:
        state["last_error"] = str(e)
        state["status"] = f"Error: {e}"

    finally:
        state["running"] = False
        state["paused"] = False
        with job_lock:
            job_stop_requested = False
            job_pause_requested = False


# ============================================================
# Routes: UI / state
# ============================================================

@app.route("/")
def index():
    return render_template_string(
        HTML,
        x_steps_per_degree=f"{X_STEPS_PER_DEGREE:.6f}",
        y_steps_per_degree=f"{Y_STEPS_PER_DEGREE:.6f}",
        default_x_max_feed=DEFAULT_X_MAX_FEED,
        default_y_max_feed=DEFAULT_Y_MAX_FEED,
        default_x_acceleration=DEFAULT_X_ACCELERATION,
        default_y_acceleration=DEFAULT_Y_ACCELERATION,
        default_draw_feed=DEFAULT_DRAW_FEED,
        default_travel_feed=DEFAULT_TRAVEL_FEED,
        default_line_thickness_mm=DEFAULT_LINE_THICKNESS_MM,
        default_pen_up_s=DEFAULT_PEN_UP_S,
        default_pen_down_s=DEFAULT_PEN_DOWN_S,
        default_servo_dwell=DEFAULT_SERVO_DWELL,
        default_sample_step_deg=DEFAULT_SAMPLE_STEP_DEG,
        default_margin_percent=DEFAULT_MARGIN_PERCENT,
    )


@app.route("/state")
def get_state():
    return jsonify(state)


# ============================================================
# Routes: GRBL commands
# ============================================================

@app.route("/connect", methods=["POST"])
def connect_route():
    try:
        with serial_lock:
            connect_grbl()
        return jsonify({"ok": True, "command": "CONNECT", "response": "Connected"})
    except Exception as e:
        state["connected"] = False
        state["status"] = "Connection failed"
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/command", methods=["POST"])
def command():
    data = request.get_json(force=True)
    cmd = data.get("command", "").strip()
    try:
        response = send_to_grbl(cmd)
        return jsonify({"ok": True, "command": cmd, "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/reset", methods=["POST"])
def reset():
    global job_stop_requested, job_pause_requested
    try:
        with job_lock:
            job_stop_requested = True
            job_pause_requested = False
        with serial_lock:
            ser = connect_grbl()
            ser.write(b"\x18")
            time.sleep(1)
            lines = read_available_lines(ser)
        state["calibrated"] = False
        state["status"] = "Soft reset sent - calibration cleared"
        return jsonify({"ok": True, "command": "CTRL-X RESET", "response": "\n".join(lines) if lines else "RESET SENT"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/apply-config", methods=["POST"])
def apply_config():
    data = request.get_json(force=True)
    try:
        x_max_feed = validate_feed(data.get("x_max_feed", DEFAULT_X_MAX_FEED))
        y_max_feed = validate_feed(data.get("y_max_feed", DEFAULT_Y_MAX_FEED))
        x_acceleration = float(data.get("x_acceleration", DEFAULT_X_ACCELERATION))
        y_acceleration = float(data.get("y_acceleration", DEFAULT_Y_ACCELERATION))

        if x_acceleration <= 0 or y_acceleration <= 0:
            raise ValueError("Acceleration must be greater than 0")
        if x_acceleration > 10000 or y_acceleration > 10000:
            raise ValueError("Acceleration is too high")

        commands = [
            "$X",
            "$30=1000",
            "$31=0",
            "$32=0",
            "$22=0",
            "$20=0",
            "$21=0",
            f"$100={X_STEPS_PER_DEGREE:.6f}",
            f"$110={x_max_feed:.3f}",
            f"$120={x_acceleration:.3f}",
            "$130=100000",
            f"$101={Y_STEPS_PER_DEGREE:.6f}",
            f"$111={y_max_feed:.3f}",
            f"$121={y_acceleration:.3f}",
            "$131=90",
            "$102=80.000",
            "$112=500.000",
            "$122=50.000",
            "$132=10",
            "G21",
            "G90",
        ]
        response = send_many(commands, wait_idle_between=False)
        state["status"] = "GRBL settings applied"
        return jsonify({"ok": True, "command": "APPLY GRBL SETTINGS", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/pen-up", methods=["POST"])
def pen_up():
    data = request.get_json(force=True)
    try:
        s_value = validate_servo_s(data.get("s", DEFAULT_PEN_UP_S))
        dwell = validate_dwell(data.get("dwell", DEFAULT_SERVO_DWELL))
        response = send_many(["$X", f"M3 S{s_value}", f"G4 P{dwell:.3f}"], wait_idle_between=True)
        return jsonify({"ok": True, "command": f"PEN UP M3 S{s_value}", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/pen-down", methods=["POST"])
def pen_down():
    data = request.get_json(force=True)
    try:
        s_value = validate_servo_s(data.get("s", DEFAULT_PEN_DOWN_S))
        dwell = validate_dwell(data.get("dwell", DEFAULT_SERVO_DWELL))
        response = send_many(["$X", f"M3 S{s_value}", f"G4 P{dwell:.3f}"], wait_idle_between=True)
        return jsonify({"ok": True, "command": f"PEN DOWN M3 S{s_value}", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/pen-test", methods=["POST"])
def pen_test():
    data = request.get_json(force=True)
    try:
        up_s = validate_servo_s(data.get("up_s", DEFAULT_PEN_UP_S))
        down_s = validate_servo_s(data.get("down_s", DEFAULT_PEN_DOWN_S))
        dwell = validate_dwell(data.get("dwell", DEFAULT_SERVO_DWELL))
        commands = [
            "$X",
            f"M3 S{up_s}", f"G4 P{dwell:.3f}",
            f"M3 S{down_s}", f"G4 P{dwell:.3f}",
            f"M3 S{up_s}", f"G4 P{dwell:.3f}",
        ]
        response = send_many(commands, wait_idle_between=True)
        return jsonify({"ok": True, "command": f"PEN TEST S{up_s}/S{down_s}", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/servo-off", methods=["POST"])
def servo_off():
    try:
        response = send_many(["$X", "M5"], wait_idle_between=True)
        return jsonify({"ok": True, "command": "SERVO OFF M5", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/jog", methods=["POST"])
def jog():
    data = request.get_json(force=True)
    try:
        axis = str(data.get("axis", "X")).upper()
        if axis not in ["X", "Y"]:
            raise ValueError("Axis must be X or Y")
        degrees = validate_degrees(data.get("degrees", 0))
        feed = validate_feed(data.get("feed", DEFAULT_TRAVEL_FEED))
        commands = ["$X", "G21", "G91", f"G1 {axis}{degrees:.6f} F{feed:.3f}", "G4 P0.01", "G90"]
        response = send_many(commands, wait_idle_between=True)
        state["calibrated"] = False
        state["status"] = "Jogged - calibration cleared until you confirm again"
        return jsonify({"ok": True, "command": f"JOG {axis}{degrees:.3f}", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/zero-position", methods=["POST"])
def zero_position():
    try:
        response = send_many(["$X", "G21", "G92 X0 Y0", "G90"], wait_idle_between=True)
        state["calibrated"] = False
        state["status"] = "Zero set - click calibrated when physically ready"
        return jsonify({"ok": True, "command": "G92 X0 Y0", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/mark-calibrated", methods=["POST"])
def mark_calibrated():
    state["calibrated"] = True
    state["status"] = "Calibrated and ready"
    return jsonify({"ok": True, "command": "MARK CALIBRATED", "response": "Runner unlocked"})


@app.route("/clear-calibrated", methods=["POST"])
def clear_calibrated():
    state["calibrated"] = False
    state["status"] = "Calibration cleared"
    return jsonify({"ok": True, "command": "CLEAR CALIBRATION", "response": "Runner locked"})


# ============================================================
# Routes: SVG generation and runner
# ============================================================

@app.route("/generate-gcode", methods=["POST"])
def generate_gcode_route():
    try:
        if "svg" not in request.files:
            raise ValueError("No SVG file uploaded")

        file = request.files["svg"]
        svg_text = file.read().decode("utf-8", errors="ignore")

        draw_feed = validate_feed(request.form.get("draw_feed", DEFAULT_DRAW_FEED))
        travel_feed = validate_feed(request.form.get("travel_feed", DEFAULT_TRAVEL_FEED))
        sample_step_deg = float(request.form.get("sample_step_deg", DEFAULT_SAMPLE_STEP_DEG))
        margin_percent = float(request.form.get("margin_percent", DEFAULT_MARGIN_PERCENT))
        placement_scale = float(request.form.get("placement_scale", 100.0))
        placement_offset_x = float(request.form.get("placement_offset_x", 0.0))
        placement_offset_y = float(request.form.get("placement_offset_y", 0.0))
        line_thickness_mm = float(request.form.get("line_thickness_mm", DEFAULT_LINE_THICKNESS_MM))
        fit_mode = request.form.get("fit_mode", "contain")
        invert_y = request.form.get("invert_y", "1") == "1"
        include_comments = request.form.get("include_comments", "1") == "1"
        pen_up_s = validate_servo_s(request.form.get("pen_up_s", DEFAULT_PEN_UP_S))
        pen_down_s = validate_servo_s(request.form.get("pen_down_s", DEFAULT_PEN_DOWN_S))
        servo_dwell = validate_dwell(request.form.get("servo_dwell", DEFAULT_SERVO_DWELL))

        if sample_step_deg <= 0:
            raise ValueError("Sample step must be greater than 0")
        if margin_percent < 0 or margin_percent > 25:
            raise ValueError("Margin percent must be between 0 and 25")
        if line_thickness_mm < 0 or line_thickness_mm > 10:
          raise ValueError("Line thickness must be between 0 and 10 mm")
        if fit_mode not in ["contain", "stretch"]:
            raise ValueError("Invalid fit mode")

        line_thickness_deg = mm_to_ball_degrees(line_thickness_mm)

        segments, viewbox_bounds = extract_svg_segments(svg_text)
        if not segments:
            raise ValueError("No drawable SVG elements found. Try converting text/strokes to paths first.")

        bounds = viewbox_bounds or bounds_from_segments(segments)
        mapped = map_svg_to_angles(segments, bounds, fit_mode, invert_y, margin_percent)
        placed = apply_placement_transform(mapped, placement_scale, placement_offset_x, placement_offset_y)

        gcode, preview = generate_gcode_from_segments(
          placed,
            draw_feed=draw_feed,
            travel_feed=travel_feed,
            sample_step_deg=sample_step_deg,
            pen_up_s=pen_up_s,
            pen_down_s=pen_down_s,
            servo_dwell=servo_dwell,
            line_thickness_deg=line_thickness_deg,
            include_comments=include_comments,
        )

        point_count = sum(len(seg.points) for seg in placed)
        state["last_svg_name"] = file.filename
        state["last_gcode"] = gcode
        state["last_preview"] = preview
        state["progress_total"] = 0
        state["progress_done"] = 0
        state["status"] = "G-code generated - calibrate before run"

        return jsonify({
            "ok": True,
            "gcode": gcode,
            "preview": preview,
            "segment_count": len(placed),
            "point_count": point_count,
            "bounds": asdict(bounds),
        })
    except Exception as e:
        state["last_error"] = str(e)
        state["status"] = f"Generate error: {e}"
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/run-gcode", methods=["POST"])
def run_gcode_route():
    global job_thread, job_stop_requested, job_pause_requested

    try:
        if state["running"]:
            raise ValueError("A job is already running")
        if not state["calibrated"]:
            raise ValueError("Machine is not calibrated. Jog/zero first, then click 'I Have Calibrated'.")
        if not state["last_gcode"]:
            raise ValueError("No G-code generated yet")

        with job_lock:
            job_stop_requested = False
            job_pause_requested = False

        job_thread = threading.Thread(target=run_gcode_worker, args=(list(state["last_gcode"]),), daemon=True)
        job_thread.start()

        return jsonify({"ok": True, "command": "RUN GENERATED G-CODE", "response": "Started"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/pause", methods=["POST"])
def pause():
    global job_pause_requested
    try:
        with job_lock:
            job_pause_requested = True
        with serial_lock:
            ser = connect_grbl()
            ser.write(b"!")  # GRBL feed hold
        state["paused"] = True
        state["status"] = "Feed hold requested"
        return jsonify({"ok": True, "command": "FEED HOLD !", "response": "Pause requested"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/resume", methods=["POST"])
def resume():
    global job_pause_requested
    try:
        with job_lock:
            job_pause_requested = False
        with serial_lock:
            ser = connect_grbl()
            ser.write(b"~")  # GRBL cycle start
        state["paused"] = False
        state["status"] = "Resume requested"
        return jsonify({"ok": True, "command": "CYCLE START ~", "response": "Resume requested"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/stop", methods=["POST"])
def stop():
    global job_stop_requested, job_pause_requested
    try:
        with job_lock:
            job_stop_requested = True
            job_pause_requested = False
        with serial_lock:
            ser = connect_grbl()
            ser.write(b"!")
            time.sleep(0.1)
            ser.write(b"\x18")  # soft reset to clear planner
            time.sleep(1)
            lines = read_available_lines(ser)
        state["calibrated"] = False
        state["status"] = "Stopped - calibration cleared"
        return jsonify({"ok": True, "command": "STOP + SOFT RESET", "response": "\n".join(lines) if lines else "Stopped"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    print("Starting SVG -> G-code Golf Ball Plotter Controller...")
    print(f"Serial port: {SERIAL_PORT}")
    print(f"X microsteps: {X_MICROSTEPS}")
    print(f"Y microsteps: {Y_MICROSTEPS}")
    print(f"X steps/degree: {X_STEPS_PER_DEGREE:.6f}")
    print(f"Y steps/degree: {Y_STEPS_PER_DEGREE:.6f}")
    print("Install dependencies if needed:")
    print("  pip install flask pyserial svgpathtools")
    print("Open:")
    print("  http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=False)
