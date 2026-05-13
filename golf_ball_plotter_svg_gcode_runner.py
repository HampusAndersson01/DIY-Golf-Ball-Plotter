from __future__ import annotations

from flask import Flask, request, jsonify, render_template_string
import serial
import time
import threading
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, asdict, field
from typing import Optional, Any

# Optional dependency for real SVG path support:
#   pip install svgpathtools pyserial flask
try:
    from svgpathtools import parse_path
except Exception:
    parse_path = None

try:
    from shapely import affinity
    from shapely.geometry import GeometryCollection, LineString, MultiLineString, MultiPolygon, Point as ShapelyPoint, Polygon
    from shapely.ops import polygonize, unary_union
    from shapely.validation import make_valid
except Exception:
    affinity = None
    GeometryCollection = None
    LineString = None
    MultiLineString = None
    MultiPolygon = None
    ShapelyPoint = None
    Polygon = None
    polygonize = None
    unary_union = None
    make_valid = None

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
DEFAULT_LINE_THICKNESS_MM = 0.75

# Servo via GRBL spindle PWM M3 S...
DEFAULT_PEN_UP_S = 575
DEFAULT_PEN_DOWN_S = 700
DEFAULT_SERVO_DWELL = 0.06
DEFAULT_SERVO_RAMP_ENABLED = True
DEFAULT_SERVO_RAMP_STEP = 20
DEFAULT_SERVO_RAMP_DELAY_MS = 10
DEFAULT_PEN_UP_DWELL_MS = 30
DEFAULT_PEN_DOWN_DWELL_MS = 60
MIN_SERVO_S = 500
MAX_SERVO_S = 1000

# SVG flattening defaults
DEFAULT_SAMPLE_STEP_DEG = 1.0       # max angular spacing between sampled points
DEFAULT_CURVE_SAMPLES = 80          # fallback per curve/path segment
DEFAULT_MARGIN_PERCENT = 4.0        # keep SVG away from extreme edges
DEFAULT_ROTATION_DEG = 0.0
DEFAULT_ENABLE_FILL = True
DEFAULT_FILL_MODE = "slicer"
DEFAULT_PARSER_MODE = "visible_geometry"
DEFAULT_COLOR_MAPPING_MODE = False
DEFAULT_TRACE_STROKE_ONLY_PATHS = True
DEFAULT_FILL_ONLY_DARK_SVG_FILLS = True
DEFAULT_WALL_COUNT = 1
DEFAULT_INFILL_PATTERN = "zigzag"
DEFAULT_INFILL_SPACING_MM = DEFAULT_LINE_THICKNESS_MM
DEFAULT_INFILL_ANGLE_DEG = 0.0
DEFAULT_OUTLINE_AFTER_FILL = False
DEFAULT_MIN_FILL_AREA_MM2 = 1.0
DEFAULT_MIN_FILL_WIDTH_MM = DEFAULT_LINE_THICKNESS_MM
DEFAULT_SIMPLIFY_TOLERANCE_MM = 0.05
DEFAULT_REMOVE_DUPLICATE_PATHS = True
DEFAULT_SMALL_SHAPE_MODE = "single-wall"
DEFAULT_MIN_SEGMENT_LENGTH_MM = 0.5
SVG_DARK_FILL_LUMINANCE_THRESHOLD = 0.42
SVG_LIGHT_CUTOUT_LUMINANCE_THRESHOLD = 0.82
SVG_MIN_PRINT_OPACITY = 0.99

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
    "current_servo_s": DEFAULT_PEN_UP_S,
    "server_pid": os.getpid(),
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
class SvgFillShape:
    geometry: Any
    fill_rule: str = "nonzero"
    source_tag: str = "path"


@dataclass
class GeometryBundle:
    outline_segments: list[Segment] = field(default_factory=list)
    fill_boundary_segments: list[Segment] = field(default_factory=list)
    fill_shapes: list[SvgFillShape] = field(default_factory=list)


@dataclass
class NormalizedFillRegion:
    type: str
    paths: list[str]
    fillColor: str
    fillRule: str
    holes: list[str] = field(default_factory=list)
    source: str = "path"


@dataclass
class NormalizedStrokePath:
    type: str
    path: str
    strokeColor: str
    strokeWidth: float
    source: str = "path"


@dataclass
class NormalizedDetailPath:
    type: str
    path: str
    source: str


@dataclass
class IgnoredSvgElement:
    reason: str
    element: str


@dataclass
class SvgPrintModel:
    fills: list[NormalizedFillRegion] = field(default_factory=list)
    cutouts: list[NormalizedFillRegion] = field(default_factory=list)
    strokes: list[NormalizedStrokePath] = field(default_factory=list)
    details: list[NormalizedDetailPath] = field(default_factory=list)
    ignored: list[IgnoredSvgElement] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    diagnostics: list[str] = field(default_factory=list)
    computed_bounds: Optional[dict[str, float]] = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvgAnalysisResult:
    bundle: GeometryBundle
    print_model: SvgPrintModel
    viewbox_bounds: Optional["SvgBounds"]


@dataclass
class ClassifiedSvgElement:
    element: str
    tag: str
    computed_style: dict[str, str]
    fill_geometry: Optional[Any] = None
    stroke_segments: list[Segment] = field(default_factory=list)
    fill_rule: str = "nonzero"
    fill_classification: str = "none"
    has_fill: bool = False
    is_cutout_fill: bool = False
    has_stroke: bool = False
    is_stroke_only: bool = False


@dataclass
class Toolpath:
    points: list[Point]
    kind: str
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

    #svgPreview, #classifiedPreview, #flatPreview, #ballPreview {
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

    #classifiedPreview {
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 10px;
    }

    #svgPreview svg, #classifiedPreview svg {
      width: 100%;
      height: 100%;
      max-width: 100%;
      max-height: 100%;
      object-fit: contain;
    }

    #svgPreview svg, #svgPreview img, #classifiedPreview svg {
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
          <p class="small" id="servoDefaultsInfo">
            Server defaults: pen up S{{ default_pen_up_s }}, pen down S{{ default_pen_down_s }}
          </p>

          <div class="row">
            <button onclick="penUp()" class="success">Pen Up</button>
            <button onclick="penDown()" class="warning-btn">Pen Down</button>
            <button onclick="penTest()" class="secondary">Pen Test</button>
            <button onclick="goHome()" class="secondary">Home X/Y</button>
            <button onclick="servoOff()" class="danger">Servo Off</button>
            <button onclick="resetServoUiDefaults()" class="secondary">Reset UI Defaults</button>
          </div>

          <div class="two-col">
            <div><label>Pen up S</label><input id="penUpS" type="number" value="{{ default_pen_up_s }}" min="0" max="1000" step="10" /></div>
            <div><label>Pen down S</label><input id="penDownS" type="number" value="{{ default_pen_down_s }}" min="0" max="1000" step="10" /></div>
          </div>
          <div class="two-col">
            <div>
              <label>Pen up dwell ms</label>
              <input id="penUpDwellMs" type="number" value="{{ default_pen_up_dwell_ms }}" min="0" max="1000" step="5" />
            </div>
            <div>
              <label>Pen down dwell ms</label>
              <input id="penDownDwellMs" type="number" value="{{ default_pen_down_dwell_ms }}" min="0" max="1000" step="5" />
            </div>
          </div>
          <label><input id="servoRampEnabled" type="checkbox" {% if default_servo_ramp_enabled %}checked{% endif %} /> Servo ramping enabled</label>
          <div class="two-col">
            <div>
              <label>Servo ramp step S</label>
              <input id="servoRampStep" type="number" value="{{ default_servo_ramp_step }}" min="1" max="200" step="1" />
            </div>
            <div>
              <label>Servo ramp delay ms</label>
              <input id="servoRampDelayMs" type="number" value="{{ default_servo_ramp_delay_ms }}" min="0" max="1000" step="1" />
            </div>
          </div>

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
          <div>
            <label>Rotation degrees</label>
            <input id="rotationDeg" type="number" value="{{ default_rotation_deg }}" step="1" min="-360" max="360" />
          </div>

          <div class="two-col">
            <div>
              <label>SVG parser mode</label>
              <select id="parserMode">
                <option value="visible_geometry" {% if default_parser_mode == "visible_geometry" %}selected{% endif %}>Visible geometry</option>
                <option value="detect_visible_print_areas" {% if default_parser_mode == "detect_visible_print_areas" %}selected{% endif %}>Detect visible print areas</option>
              </select>
            </div>
            <div>
              <label><input id="colorMappingMode" type="checkbox" {% if default_color_mapping_mode %}checked{% endif %} /> Enable optional color mapping</label>
              <p class="small">Black/dark = printable, white/light = cutout/ignored, blue = outline, orange = fill, teal = detail.</p>
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

          <label>Line width / nozzle diameter (mm on ball)</label>
          <input id="lineThicknessMm" type="number" value="{{ default_line_thickness_mm }}" step="0.01" min="0" max="10" />
          <p class="small">
            Example: 0.75 mm makes the fill wall offset inward by 0.375 mm and defaults infill spacing to 0.75 mm.
          </p>

          <h3>Fill</h3>
          <label><input id="enableFill" type="checkbox" {% if default_enable_fill %}checked{% endif %} /> Enable slicer-style fill for filled SVG regions</label>
          <label><input id="fillOnlyDarkSvgFills" type="checkbox" {% if default_fill_only_dark_svg_fills %}checked{% endif %} /> Fill only dark SVG fills</label>
          <label><input id="traceStrokeOnlyPaths" type="checkbox" {% if default_trace_stroke_only_paths %}checked{% endif %} /> Trace stroke-only SVG paths</label>
          <div class="two-col">
            <div>
              <label>Fill mode</label>
              <select id="fillMode">
                <option value="slicer" selected>Slicer</option>
              </select>
            </div>
            <div>
              <label>Wall count</label>
              <input id="wallCount" type="number" value="{{ default_wall_count }}" min="1" max="8" step="1" />
            </div>
          </div>
          <div class="two-col">
            <div>
              <label>Infill pattern</label>
              <select id="infillPattern">
                <option value="zigzag" selected>Zigzag</option>
                <option value="hatch">Hatch</option>
              </select>
            </div>
            <div>
              <label>Infill spacing mm</label>
              <input id="infillSpacingMm" type="number" value="{{ default_infill_spacing_mm }}" min="0.1" max="10" step="0.01" />
            </div>
          </div>
          <div class="two-col">
            <div>
              <label>Infill angle degrees</label>
              <input id="infillAngleDeg" type="number" value="{{ default_infill_angle_deg }}" min="-180" max="180" step="1" />
            </div>
            <div>
              <label>Small shape mode</label>
              <select id="smallShapeMode">
                <option value="single-wall" selected>Single wall</option>
                <option value="skip">Skip</option>
                <option value="centerline-todo">Centerline TODO</option>
              </select>
            </div>
          </div>
          <div class="two-col">
            <div>
              <label>Min fill area mm²</label>
              <input id="minFillAreaMm2" type="number" value="{{ default_min_fill_area_mm2 }}" min="0" max="1000" step="0.1" />
            </div>
            <div>
              <label>Min fill width mm</label>
              <input id="minFillWidthMm" type="number" value="{{ default_min_fill_width_mm }}" min="0" max="10" step="0.01" />
            </div>
          </div>
          <div class="two-col">
            <div>
              <label>Simplify tolerance mm</label>
              <input id="simplifyToleranceMm" type="number" value="{{ default_simplify_tolerance_mm }}" min="0" max="2" step="0.01" />
            </div>
            <div>
              <label>Min segment length mm</label>
              <input id="minSegmentLengthMm" type="number" value="{{ default_min_segment_length_mm }}" min="0" max="10" step="0.01" />
            </div>
          </div>
          <label><input id="outlineAfterFill" type="checkbox" {% if default_outline_after_fill %}checked{% endif %} /> Draw original outline after fill</label>
          <label><input id="removeDuplicatePaths" type="checkbox" {% if default_remove_duplicate_paths %}checked{% endif %} /> Remove duplicate / tiny paths</label>

          <label>Fit mode</label>
          <select id="fitMode">
            <option value="contain" selected>Contain entire SVG inside ball drawing area</option>
            <option value="stretch">Stretch to full 360 x 90 area</option>
          </select>

          <label><input id="invertY" type="checkbox" checked /> Invert SVG Y so top of SVG becomes +Y</label>
          <label><input id="includeComments" type="checkbox" checked /> Include comments in G-code</label>
          <label><input id="debugPipeline" type="checkbox" /> Include debug geometry snapshots in `/generate-gcode` response</label>

          <button onclick="uploadAndGenerate()" class="purple">Upload SVG + Generate G-code</button>
          <button onclick="downloadGcode()" class="secondary">Download G-code</button>
          <button onclick="runSvgPipelineSelfTest()" class="secondary">Run Integrated SVG Self-Test</button>
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
              <div class="preview-title">Classified SVG Regions</div>
              <div class="small">Amber = printable dark fills, red = cutouts, blue = traced stroke paths.</div>
              <div id="classifiedPreview"><span class="small">Generate G-code to inspect parsed SVG regions</span></div>
            </div>
            <div class="preview-panel">
              <div class="preview-title">Flat coordinate map</div>
              <div class="small">Outline = blue, fill wall = amber, infill = teal, travel = slate</div>
              <canvas id="flatPreview"></canvas>
            </div>
            <div class="preview-panel">
              <div class="preview-title">Ball preview</div>
              <div class="small">Preview is generated from the same final toolpaths as the G-code.</div>
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
  let latestPrintModel = null;
  let latestViewboxBounds = null;

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

  function getBool(id) {
    return document.getElementById(id).checked;
  }

  const SERVER_DEFAULTS = {
    penUpS: {{ default_pen_up_s }},
    penDownS: {{ default_pen_down_s }},
    penUpDwellMs: {{ default_pen_up_dwell_ms }},
    penDownDwellMs: {{ default_pen_down_dwell_ms }},
    servoRampEnabled: {{ 'true' if default_servo_ramp_enabled else 'false' }},
    servoRampStep: {{ default_servo_ramp_step }},
    servoRampDelayMs: {{ default_servo_ramp_delay_ms }}
  };

  function resetServoUiDefaults() {
    document.getElementById('penUpS').value = SERVER_DEFAULTS.penUpS;
    document.getElementById('penDownS').value = SERVER_DEFAULTS.penDownS;
    document.getElementById('penUpDwellMs').value = SERVER_DEFAULTS.penUpDwellMs;
    document.getElementById('penDownDwellMs').value = SERVER_DEFAULTS.penDownDwellMs;
    document.getElementById('servoRampEnabled').checked = SERVER_DEFAULTS.servoRampEnabled;
    document.getElementById('servoRampStep').value = SERVER_DEFAULTS.servoRampStep;
    document.getElementById('servoRampDelayMs').value = SERVER_DEFAULTS.servoRampDelayMs;
    document.getElementById('rawCommand').value = `M3 S${SERVER_DEFAULTS.penUpS}`;
    appendLog(`Reset servo UI fields to server defaults S${SERVER_DEFAULTS.penUpS}/S${SERVER_DEFAULTS.penDownS}.`);
  }

  function appendServoSettings(target) {
    target.servo_ramp_enabled = getBool('servoRampEnabled');
    target.servo_ramp_step = getInt('servoRampStep');
    target.servo_ramp_delay_ms = getNum('servoRampDelayMs');
    target.pen_up_dwell_ms = getNum('penUpDwellMs');
    target.pen_down_dwell_ms = getNum('penDownDwellMs');
    return target;
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
      if (s.defaults) {
        document.getElementById('servoDefaultsInfo').textContent =
          `Server defaults: pen up S${s.defaults.pen_up_s}, pen down S${s.defaults.pen_down_s}, PID ${s.server_pid}`;
      }

      const pct = s.progress_total ? Math.round((s.progress_done / s.progress_total) * 100) : 0;
      document.getElementById('progressFill').style.width = `${pct}%`;
    } catch (_) {}
  }

  function sendCommand(command) { return api('/command', { command }); }
  function sendRaw() { return sendCommand(document.getElementById('rawCommand').value); }
  function softReset() { return api('/reset'); }

  async function runSvgPipelineSelfTest() {
    try {
      const res = await fetch('/self-test-svg-pipeline', { method: 'POST' });
      const json = await res.json();
      if (!json.ok) {
        appendLog(`SVG SELF-TEST ERROR: ${json.error}`);
        return;
      }
      appendLog(`SVG self-test passed: ${json.summary.passed} checks.`);
      for (const line of (json.summary.messages || [])) {
        appendLog(`SELF-TEST: ${line}`);
      }
    } catch (err) {
      appendLog(`SVG SELF-TEST FETCH ERROR: ${err}`);
    }
  }

  function applyMachineConfig() {
    return api('/apply-config', {
      x_max_feed: getNum('xMaxFeed'),
      y_max_feed: getNum('yMaxFeed'),
      x_acceleration: getNum('xAcceleration'),
      y_acceleration: getNum('yAcceleration')
    });
  }

  function penUp() {
    return api('/pen-up', appendServoSettings({
      s: getInt('penUpS'),
    }));
  }

  function penDown() {
    return api('/pen-down', appendServoSettings({
      s: getInt('penDownS'),
    }));
  }

  function penTest() {
    return api('/pen-test', appendServoSettings({
      up_s: getInt('penUpS'),
      down_s: getInt('penDownS'),
    }));
  }

  function servoOff() { return api('/servo-off'); }
  function jogX(degrees) { return api('/jog', { axis: 'X', degrees, feed: getNum('travelFeed') }); }
  function jogY(degrees) { return api('/jog', { axis: 'Y', degrees, feed: getNum('travelFeed') }); }
  function zeroPosition() { return api('/zero-position'); }
  function goHome() {
    return api('/go-home', appendServoSettings({
      pen_up_s: getInt('penUpS'),
      travel_feed: getNum('travelFeed')
    }));
  }
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
    form.append('parser_mode', document.getElementById('parserMode').value);
    form.append('color_mapping_mode', getBool('colorMappingMode') ? '1' : '0');
    form.append('invert_y', document.getElementById('invertY').checked ? '1' : '0');
    form.append('include_comments', document.getElementById('includeComments').checked ? '1' : '0');
    form.append('debug_pipeline', getBool('debugPipeline') ? '1' : '0');
    form.append('placement_scale', getNum('placementScale'));
    form.append('placement_offset_x', getNum('placementOffsetX'));
    form.append('placement_offset_y', getNum('placementOffsetY'));
    form.append('rotation_deg', getNum('rotationDeg'));
    form.append('line_thickness_mm', getNum('lineThicknessMm'));
    form.append('enable_fill', getBool('enableFill') ? '1' : '0');
    form.append('fill_only_dark_svg_fills', getBool('fillOnlyDarkSvgFills') ? '1' : '0');
    form.append('trace_stroke_only_paths', getBool('traceStrokeOnlyPaths') ? '1' : '0');
    form.append('fill_mode', document.getElementById('fillMode').value);
    form.append('wall_count', getInt('wallCount'));
    form.append('infill_pattern', document.getElementById('infillPattern').value);
    form.append('infill_spacing_mm', getNum('infillSpacingMm'));
    form.append('infill_angle_deg', getNum('infillAngleDeg'));
    form.append('outline_after_fill', getBool('outlineAfterFill') ? '1' : '0');
    form.append('min_fill_area_mm2', getNum('minFillAreaMm2'));
    form.append('min_fill_width_mm', getNum('minFillWidthMm'));
    form.append('simplify_tolerance_mm', getNum('simplifyToleranceMm'));
    form.append('remove_duplicate_paths', getBool('removeDuplicatePaths') ? '1' : '0');
    form.append('small_shape_mode', document.getElementById('smallShapeMode').value);
    form.append('min_segment_length_mm', getNum('minSegmentLengthMm'));
    form.append('pen_up_s', getInt('penUpS'));
    form.append('pen_down_s', getInt('penDownS'));
    form.append('servo_ramp_enabled', getBool('servoRampEnabled') ? '1' : '0');
    form.append('servo_ramp_step', getInt('servoRampStep'));
    form.append('servo_ramp_delay_ms', getNum('servoRampDelayMs'));
    form.append('pen_up_dwell_ms', getNum('penUpDwellMs'));
    form.append('pen_down_dwell_ms', getNum('penDownDwellMs'));

    try {
      const res = await fetch('/generate-gcode', { method: 'POST', body: form });
      const json = await res.json();
      if (!json.ok) {
        appendLog(`ERROR: ${json.error}`);
        return;
      }

      latestGcode = json.gcode;
      latestPreview = json.preview || [];
      latestPrintModel = json.print_model || null;
      latestViewboxBounds = json.viewbox_bounds || null;
      document.getElementById('gcodeBox').value = latestGcode.join('\n');
      renderClassifiedPreview(latestPrintModel, latestViewboxBounds);
      drawFlatPreview(latestPreview);
      drawBallPreview(latestPreview);
      appendLog(`Generated ${latestGcode.length} G-code lines from ${json.toolpath_count} toolpaths / ${json.point_count} plotted points.`);
      const classificationCounts = json.print_model?.metadata?.classificationCounts || {};
      appendLog(`SVG classification: dark fills ${classificationCounts.dark_filled_polygons || 0}, light cutouts ${classificationCounts.light_cutout_polygons || 0}, transparent cutouts ${classificationCounts.transparent_cutout_polygons || 0}, stroke-only ${classificationCounts.stroke_only_paths || 0}, ignored ${classificationCounts.ignored_paths || 0}.`);
      if (json.debug?.toolpath_counts) {
        const counts = json.debug.toolpath_counts;
        appendLog(`Generated toolpaths: walls ${counts.generated_fill_walls || 0}, infill ${counts.generated_infill_paths || 0}, outlines ${counts.generated_outline_paths || 0}.`);
      }
      for (const warning of (json.print_model?.warnings || [])) {
        appendLog(`WARNING: ${warning}`);
      }
      for (const diagnostic of (json.print_model?.diagnostics || [])) {
        appendLog(`DIAGNOSTIC: ${diagnostic}`);
      }
      await refreshState();
    } catch (err) {
      appendLog(`GENERATE ERROR: ${err}`);
    }
  }

  function renderClassifiedPreview(printModel, viewboxBounds) {
    const host = document.getElementById('classifiedPreview');
    if (!host) return;
    if (!printModel) {
      host.innerHTML = '<span class="small">No classified SVG regions available</span>';
      return;
    }

    const fills = printModel.fills || [];
    const cutouts = printModel.cutouts || [];
    const strokes = printModel.strokes || [];
    const computed = printModel.computed_bounds || {};
    const vb = viewboxBounds || {};
    const minX = Number.isFinite(vb.min_x) ? vb.min_x : (Number.isFinite(computed.min_x) ? computed.min_x : 0);
    const minY = Number.isFinite(vb.min_y) ? vb.min_y : (Number.isFinite(computed.min_y) ? computed.min_y : 0);
    const maxX = Number.isFinite(vb.max_x) ? vb.max_x : (Number.isFinite(computed.max_x) ? computed.max_x : minX + 100);
    const maxY = Number.isFinite(vb.max_y) ? vb.max_y : (Number.isFinite(computed.max_y) ? computed.max_y : minY + 100);
    const width = Math.max(1, maxX - minX);
    const height = Math.max(1, maxY - minY);

    const fillMarkup = fills.map((region) => {
      const mergedPath = [...(region.paths || []), ...(region.holes || [])].join(' ');
      return `<path d="${escapeHtml(mergedPath)}" fill="rgba(245, 158, 11, 0.7)" stroke="#f59e0b" stroke-width="${Math.max(width, height) * 0.0015}" fill-rule="evenodd"/>`;
    }).join('');

    const cutoutMarkup = cutouts.map((region) => {
      const mergedPath = [...(region.paths || []), ...(region.holes || [])].join(' ');
      return `<path d="${escapeHtml(mergedPath)}" fill="rgba(239, 68, 68, 0.18)" stroke="#ef4444" stroke-width="${Math.max(width, height) * 0.0015}" fill-rule="evenodd"/>`;
    }).join('');

    const strokeMarkup = strokes.map((region) =>
      `<path d="${escapeHtml(region.path || '')}" fill="none" stroke="rgba(59, 130, 246, 0.95)" stroke-width="${Math.max(region.strokeWidth || 1, Math.max(width, height) * 0.0012)}" stroke-linecap="round" stroke-linejoin="round"/>`
    ).join('');

    host.innerHTML = `
      <svg viewBox="${minX} ${minY} ${width} ${height}" xmlns="http://www.w3.org/2000/svg" aria-label="Classified SVG regions">
        <rect x="${minX}" y="${minY}" width="${width}" height="${height}" fill="#020617"/>
        <g>${fillMarkup}</g>
        <g>${cutoutMarkup}</g>
        <g>${strokeMarkup}</g>
      </svg>
    `;
  }

  function escapeHtml(text) {
    return String(text || '')
      .replaceAll('&', '&amp;')
      .replaceAll('<', '&lt;')
      .replaceAll('>', '&gt;')
      .replaceAll('\"', '&quot;');
  }

  function setupCanvas(canvasId) {
    const canvas = document.getElementById(canvasId);
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.max(300, Math.floor(rect.width * window.devicePixelRatio));
    canvas.height = Math.max(300, Math.floor(rect.height * window.devicePixelRatio));
    return canvas.getContext('2d');
  }

  function previewStyle(kind, depth = 1) {
    const alpha = Math.max(0.18, Math.min(1, depth));
    if (kind === 'outline') return { stroke: `rgba(59, 130, 246, ${alpha})`, width: 2.1, dash: [] };
    if (kind === 'fill-wall') return { stroke: `rgba(245, 158, 11, ${alpha})`, width: 2.4, dash: [] };
    if (kind === 'fill-infill') return { stroke: `rgba(45, 212, 191, ${alpha})`, width: 1.6, dash: [] };
    return { stroke: `rgba(148, 163, 184, ${alpha * 0.9})`, width: 1.1, dash: [6, 6] };
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

    for (const entry of paths) {
      const path = entry.points || [];
      if (!path.length) continue;
      const style = previewStyle(entry.kind || 'outline');
      ctx.beginPath();
      ctx.moveTo(sx(path[0].x), sy(path[0].y));
      for (let i = 1; i < path.length; i++) {
        ctx.lineTo(sx(path[i].x), sy(path[i].y));
      }
      ctx.setLineDash((style.dash || []).map((value) => value * window.devicePixelRatio));
      ctx.lineWidth = style.width * window.devicePixelRatio;
      ctx.strokeStyle = style.stroke;
      ctx.stroke();
    }
    ctx.setLineDash([]);
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
      x: cx + y * radius * perspective,
      y: cy + x * radius * perspective,
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
    for (const entry of paths) {
      const path = entry.points || [];
      if (!path.length) continue;
      projectedPaths.push({
        kind: entry.kind || 'outline',
        points: path.map((point) => projectBallPoint(point, cx, cy, radius))
      });
    }
    projectedPaths.sort((a, b) => ((a.points[0]?.z || 0) - (b.points[0]?.z || 0)));

    for (const entry of projectedPaths) {
      const projected = entry.points;
      ctx.beginPath();
      for (let i = 0; i < projected.length; i++) {
        const p = projected[i];
        if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      }
      const depth = projected.reduce((sum, p) => sum + p.z, 0) / projected.length;
      const style = previewStyle(entry.kind, 0.35 + ((depth + 1) * 0.325));
      ctx.setLineDash((style.dash || []).map((value) => value * window.devicePixelRatio));
      ctx.lineWidth = style.width * window.devicePixelRatio;
      ctx.strokeStyle = style.stroke;
      ctx.stroke();
    }
    ctx.setLineDash([]);

    ctx.fillStyle = 'rgba(255, 255, 255, 0.38)';
    ctx.beginPath();
    ctx.arc(cx - radius * 0.32, cy - radius * 0.34, radius * 0.16, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#cbd5e1';
    ctx.font = `${11 * window.devicePixelRatio}px Arial`;
    ctx.fillText('Printer view rotation', cx - 38 * window.devicePixelRatio, cy + radius + 16 * window.devicePixelRatio);
    ctx.fillText('Right side = bottom', cx - 42 * window.devicePixelRatio, cy + radius + 30 * window.devicePixelRatio);
    ctx.fillText('X0° / Y0°', cx - 24 * window.devicePixelRatio, cy + 4 * window.devicePixelRatio);
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


def validate_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def validate_non_negative_float(value: Any, label: str, maximum: Optional[float] = None) -> float:
    out = float(value)
    if out < 0:
        raise ValueError(f"{label} must not be negative")
    if maximum is not None and out > maximum:
        raise ValueError(f"{label} must be <= {maximum}")
    return out


def validate_non_negative_int(value: Any, label: str, minimum: int = 0, maximum: Optional[int] = None) -> int:
    out = int(value)
    if out < minimum:
        raise ValueError(f"{label} must be >= {minimum}")
    if maximum is not None and out > maximum:
        raise ValueError(f"{label} must be <= {maximum}")
    return out


def ms_to_seconds(ms: float) -> float:
    return max(0.0, ms) / 1000.0


def get_tracked_servo_s(fallback: int) -> int:
    tracked = state.get("current_servo_s", fallback)
    try:
        return validate_servo_s(tracked)
    except Exception:
        return fallback


def set_tracked_servo_s(s_value: int) -> None:
    state["current_servo_s"] = validate_servo_s(s_value)


def build_pen_position_commands(
    start_s: int,
    end_s: int,
    *,
    ramp_enabled: bool,
    ramp_step: int,
    ramp_delay_ms: float,
    dwell_ms: float,
) -> list[str]:
    commands: list[str] = [f"M3 S{end_s}"]

    dwell_seconds = ms_to_seconds(dwell_ms)
    if dwell_seconds > 0:
        commands.append(f"G4 P{dwell_seconds:.3f}")

    return commands


def mm_to_ball_degrees(mm: float) -> float:
  if mm < 0:
    raise ValueError("Line thickness must not be negative")
  return (mm / BALL_DIAMETER_MM) * 360.0


# ============================================================
# SVG parsing and flattening
# ============================================================

SVG_MATRIX_IDENTITY = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def require_geometry_support() -> None:
    if parse_path is None:
        raise RuntimeError("Install svgpathtools for SVG path support: pip install svgpathtools")
    if Polygon is None or affinity is None:
        raise RuntimeError("Install shapely for slicer fill support: pip install shapely")


def strip_namespace(tag: str) -> str:
    return tag.split("}", 1)[-1] if "}" in tag else tag


def parse_float(value: Optional[str], default: float = 0.0) -> float:
    if value is None:
        return default
    match = re.search(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", value)
    return float(match.group(0)) if match else default


def parse_points_attr(points: str) -> list[Point]:
    nums = [float(n) for n in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", points or "")]
    out: list[Point] = []
    for i in range(0, len(nums) - 1, 2):
        out.append(Point(nums[i], nums[i + 1]))
    return out


def parse_style_map(style: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in (style or "").split(";"):
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        out[key.strip()] = value.strip()
    return out


def parse_svg_length(value: Optional[str], default: float = 0.0) -> float:
    return parse_float(value, default)


def format_svg_path(points: list[Point], closed: bool) -> str:
    if not points:
        return ""
    commands = [f"M {points[0].x:.6f} {points[0].y:.6f}"]
    for point in points[1:]:
        commands.append(f"L {point.x:.6f} {point.y:.6f}")
    if closed:
        commands.append("Z")
    return " ".join(commands)


def debug_append_parser_entry(debug: Optional[dict[str, Any]], entry: dict[str, Any]) -> None:
    if debug is None:
        return
    debug.setdefault("parser_elements", []).append(entry)


def debug_append_warning(debug: Optional[dict[str, Any]], message: str) -> None:
    if debug is None:
        return
    debug.setdefault("parser_warnings", []).append(message)


def debug_set_counts(debug: Optional[dict[str, Any]], key: str, counts: dict[str, int]) -> None:
    if debug is None:
        return
    debug[key] = {name: int(value) for name, value in counts.items()}


def parse_svg_stylesheet(root: ET.Element, debug: Optional[dict[str, Any]] = None) -> dict[str, dict[str, str]]:
    rules: dict[str, dict[str, str]] = {}
    for elem in root.iter():
        if strip_namespace(elem.tag) != "style":
            continue
        css_text = "".join(elem.itertext())
        for selectors_text, body in re.findall(r"([^{}]+)\{([^{}]+)\}", css_text, flags=re.S):
            declarations = parse_style_map(body)
            if not declarations:
                continue
            for raw_selector in selectors_text.split(","):
                selector = raw_selector.strip()
                if not selector:
                    continue
                if re.fullmatch(r"[.#]?[A-Za-z_][\w:-]*", selector):
                    rules.setdefault(selector, {}).update(declarations)
                else:
                    debug_append_warning(debug, f"Unsupported CSS selector ignored: {selector}")
    return rules


def stylesheet_props_for_elem(elem: ET.Element, stylesheet_rules: dict[str, dict[str, str]]) -> dict[str, str]:
    props: dict[str, str] = {}
    tag = strip_namespace(elem.tag)
    for selector in [tag]:
        props.update(stylesheet_rules.get(selector, {}))

    elem_id = elem.attrib.get("id", "").strip()
    if elem_id:
        props.update(stylesheet_rules.get(f"#{elem_id}", {}))

    classes = [class_name for class_name in elem.attrib.get("class", "").split() if class_name]
    for class_name in classes:
        props.update(stylesheet_rules.get(f".{class_name}", {}))
    return props


def parse_svg_transform(transform_text: Optional[str]) -> tuple[float, float, float, float, float, float]:
    matrix = SVG_MATRIX_IDENTITY
    for name, raw_args in re.findall(r"([a-zA-Z]+)\s*\(([^)]*)\)", transform_text or ""):
        args = [float(n) for n in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", raw_args)]
        name = name.lower()
        if name == "matrix" and len(args) == 6:
            op = tuple(args)
        elif name == "translate":
            tx = args[0] if args else 0.0
            ty = args[1] if len(args) > 1 else 0.0
            op = (1.0, 0.0, 0.0, 1.0, tx, ty)
        elif name == "scale":
            sx = args[0] if args else 1.0
            sy = args[1] if len(args) > 1 else sx
            op = (sx, 0.0, 0.0, sy, 0.0, 0.0)
        elif name == "rotate":
            angle = math.radians(args[0] if args else 0.0)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            if len(args) >= 3:
                cx, cy = args[1], args[2]
                op = (
                    cos_a,
                    sin_a,
                    -sin_a,
                    cos_a,
                    cx - (cos_a * cx) + (sin_a * cy),
                    cy - (sin_a * cx) - (cos_a * cy),
                )
            else:
                op = (cos_a, sin_a, -sin_a, cos_a, 0.0, 0.0)
        elif name == "skewx" and args:
            op = (1.0, 0.0, math.tan(math.radians(args[0])), 1.0, 0.0, 0.0)
        elif name == "skewy" and args:
            op = (1.0, math.tan(math.radians(args[0])), 0.0, 1.0, 0.0, 0.0)
        else:
            continue
        # SVG transform lists are applied in the order they are written.
        # With column-vector points that means each new operation must pre-multiply
        # the accumulated matrix, otherwise `translate(...) scale(...)` becomes
        # `scale` then `translate`, which misplaces outlines and fill clipping.
        matrix = multiply_svg_matrices(op, matrix)
    return matrix


def multiply_svg_matrices(
    left: tuple[float, float, float, float, float, float],
    right: tuple[float, float, float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    a1, b1, c1, d1, e1, f1 = left
    a2, b2, c2, d2, e2, f2 = right
    return (
        (a1 * a2) + (c1 * b2),
        (b1 * a2) + (d1 * b2),
        (a1 * c2) + (c1 * d2),
        (b1 * c2) + (d1 * d2),
        (a1 * e2) + (c1 * f2) + e1,
        (b1 * e2) + (d1 * f2) + f1,
    )


def apply_svg_matrix(point: Point, matrix: tuple[float, float, float, float, float, float]) -> Point:
    a, b, c, d, e, f = matrix
    return Point(
        (a * point.x) + (c * point.y) + e,
        (b * point.x) + (d * point.y) + f,
    )


def shapely_affine_from_svg_matrix(matrix: tuple[float, float, float, float, float, float]) -> list[float]:
    a, b, c, d, e, f = matrix
    return [a, c, b, d, e, f]


def path_d_to_segments(d: str, curve_samples: int) -> list[Segment]:
    require_geometry_support()
    path = parse_path(d)
    if len(path) == 0:
        return []

    segments: list[Segment] = []
    for subpath in path.continuous_subpaths():
        current: list[Point] = []
        for part in subpath:
            try:
                length = max(1.0, float(part.length(error=1e-4)))
                samples = max(2, min(300, int(length / 2.0) + 2))
            except Exception:
                samples = curve_samples

            for i in range(samples):
                t = i / (samples - 1)
                p = part.point(t)
                pt = Point(float(p.real), float(p.imag))
                if current and nearly_same_point(current[-1], pt):
                    continue
                current.append(pt)

        if len(current) < 2:
            continue
        closed = nearly_same_point(current[0], current[-1])
        if closed and not nearly_same_point(current[0], current[-1]):
            current.append(Point(current[0].x, current[0].y))
        elif not closed and len(subpath) > 0 and abs(subpath[0].start - subpath[-1].end) <= 1e-6:
            current.append(Point(current[0].x, current[0].y))
            closed = True
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


def nearly_same_point(a: Point, b: Point, tolerance: float = 1e-6) -> bool:
    return abs(a.x - b.x) <= tolerance and abs(a.y - b.y) <= tolerance


def ensure_segment_closed(segment: Segment) -> Segment:
    if not segment.points:
        return segment
    points = list(segment.points)
    if not nearly_same_point(points[0], points[-1]):
        points.append(Point(points[0].x, points[0].y))
    return Segment(points, closed=True)


def transform_segment(segment: Segment, matrix: tuple[float, float, float, float, float, float]) -> Segment:
    return Segment([apply_svg_matrix(point, matrix) for point in segment.points], closed=segment.closed)


def debug_append_segments(debug: Optional[dict[str, Any]], key: str, segments: list[Segment], kind: str) -> None:
    if debug is None or not segments:
        return
    entries = debug.setdefault(key, [])
    for segment in segments:
        if len(segment.points) < 2:
            continue
        entries.append({
            "kind": kind,
            "closed": segment.closed,
            "points": [asdict(point) for point in segment.points],
        })


def debug_append_toolpaths(debug: Optional[dict[str, Any]], key: str, toolpaths: list["Toolpath"]) -> None:
    if debug is None or not toolpaths:
        return
    entries = debug.setdefault(key, [])
    for toolpath in toolpaths:
        if len(toolpath.points) < 2:
            continue
        entries.append({
            "kind": toolpath.kind,
            "closed": toolpath.closed,
            "points": [asdict(point) for point in toolpath.points],
        })


def element_segments(tag: str, elem: ET.Element) -> list[Segment]:
    if tag == "path":
        d = elem.attrib.get("d", "").strip()
        return path_d_to_segments(d, DEFAULT_CURVE_SAMPLES) if d else []
    if tag == "polyline":
        pts = parse_points_attr(elem.attrib.get("points", ""))
        if len(pts) >= 2:
            closed = nearly_same_point(pts[0], pts[-1])
            return [Segment(pts, closed=closed)]
        return []
    if tag == "polygon":
        pts = parse_points_attr(elem.attrib.get("points", ""))
        if len(pts) >= 2:
            return [ensure_segment_closed(Segment(pts, closed=True))]
        return []
    if tag == "line":
        x1 = parse_float(elem.attrib.get("x1"))
        y1 = parse_float(elem.attrib.get("y1"))
        x2 = parse_float(elem.attrib.get("x2"))
        y2 = parse_float(elem.attrib.get("y2"))
        return [Segment([Point(x1, y1), Point(x2, y2)], closed=False)]
    if tag == "rect":
        x = parse_float(elem.attrib.get("x"))
        y = parse_float(elem.attrib.get("y"))
        w = parse_float(elem.attrib.get("width"))
        h = parse_float(elem.attrib.get("height"))
        return [rect_to_segment(x, y, w, h)] if w > 0 and h > 0 else []
    if tag == "circle":
        cx = parse_float(elem.attrib.get("cx"))
        cy = parse_float(elem.attrib.get("cy"))
        r = parse_float(elem.attrib.get("r"))
        return [circle_to_segment(cx, cy, r)] if r > 0 else []
    if tag == "ellipse":
        cx = parse_float(elem.attrib.get("cx"))
        cy = parse_float(elem.attrib.get("cy"))
        rx = parse_float(elem.attrib.get("rx"))
        ry = parse_float(elem.attrib.get("ry"))
        return [ellipse_to_segment(cx, cy, rx, ry)] if rx > 0 and ry > 0 else []
    return []


def signed_ring_area(points: list[Point]) -> float:
    area = 0.0
    ring = points[:-1] if len(points) >= 2 and nearly_same_point(points[0], points[-1]) else points
    if len(ring) < 3:
        return 0.0
    for current, nxt in zip(ring, ring[1:] + [ring[0]]):
        area += (current.x * nxt.y) - (nxt.x * current.y)
    return area / 2.0


def closed_segment_to_ring(segment: Segment) -> Optional[list[Point]]:
    closed = ensure_segment_closed(segment)
    deduped: list[Point] = []
    for point in closed.points:
        if deduped and nearly_same_point(deduped[-1], point):
            continue
        deduped.append(point)
    if len(deduped) < 4:
        return None
    if not nearly_same_point(deduped[0], deduped[-1]):
        deduped.append(Point(deduped[0].x, deduped[0].y))
    return deduped


def make_polygon_from_ring(ring: list[Point]) -> Optional[Any]:
    if len(ring) < 4:
        return None
    polygon = Polygon([(point.x, point.y) for point in ring])
    if polygon.is_empty:
        return None
    if not polygon.is_valid:
        polygon = make_valid(polygon) if make_valid is not None else polygon.buffer(0)
    if polygon.is_empty:
        return None
    return polygon


def parse_svg_color(value: str) -> Optional[tuple[float, float, float]]:
    color = (value or "").strip().lower()
    if not color or color in {"none", "transparent"}:
        return None
    named = {
        "black": (0.0, 0.0, 0.0),
        "white": (1.0, 1.0, 1.0),
        "red": (1.0, 0.0, 0.0),
        "green": (0.0, 0.502, 0.0),
        "yellow": (1.0, 1.0, 0.0),
        "navy": (0.0, 0.0, 0.502),
        "blue": (0.0, 0.0, 1.0),
        "orange": (1.0, 0.647, 0.0),
        "amber": (1.0, 0.749, 0.0),
        "teal": (0.0, 0.502, 0.502),
        "gray": (0.502, 0.502, 0.502),
        "grey": (0.502, 0.502, 0.502),
        "slategray": (0.439, 0.502, 0.565),
        "slategrey": (0.439, 0.502, 0.565),
    }
    if color in named:
        return named[color]
    if color.startswith("#"):
        hex_value = color[1:]
        if len(hex_value) == 3:
            try:
                return tuple(int(ch * 2, 16) / 255.0 for ch in hex_value)
            except ValueError:
                return None
        if len(hex_value) == 6:
            try:
                return (
                    int(hex_value[0:2], 16) / 255.0,
                    int(hex_value[2:4], 16) / 255.0,
                    int(hex_value[4:6], 16) / 255.0,
                )
            except ValueError:
                return None
    if color.startswith("rgb"):
        raw_parts = re.findall(r"[-+]?\d*\.?\d+%?", color)
        if len(raw_parts) >= 3:
            channels = []
            for part in raw_parts[:3]:
                if part.endswith("%"):
                    channels.append(max(0.0, min(1.0, float(part[:-1]) / 100.0)))
                else:
                    channels.append(max(0.0, min(1.0, float(part) / 255.0)))
            if len(channels) == 3:
                return tuple(channels)  # type: ignore[return-value]
    return None


def parse_svg_color_alpha(value: str) -> float:
    color = (value or "").strip().lower()
    if not color or color in {"none", "transparent"}:
        return 0.0
    if color.startswith("rgba"):
        raw_parts = re.findall(r"[-+]?\d*\.?\d+%?", color)
        if len(raw_parts) >= 4:
            alpha = raw_parts[3]
            try:
                if alpha.endswith("%"):
                    return max(0.0, min(1.0, float(alpha[:-1]) / 100.0))
                return max(0.0, min(1.0, float(alpha)))
            except ValueError:
                return 1.0
    return 1.0


def is_unsupported_paint_server(value: str) -> bool:
    paint = (value or "").strip().lower()
    return paint.startswith("url(") or paint in {"context-fill", "context-stroke", "currentcolor", "inherit"}


def color_luminance(value: str) -> Optional[float]:
    rgb = parse_svg_color(value)
    if rgb is None:
        return None
    return (0.2126 * rgb[0]) + (0.7152 * rgb[1]) + (0.0722 * rgb[2])


def color_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def classify_color_role(value: str) -> Optional[str]:
    rgb = parse_svg_color(value)
    if rgb is None:
        return None
    palette = {
        "print": (0.0, 0.0, 0.0),
        "cutout": (1.0, 1.0, 1.0),
        "outline": (0.0, 0.0, 1.0),
        "fill": (1.0, 0.647, 0.0),
        "detail": (0.0, 0.502, 0.502),
        "ignored": (0.5, 0.5, 0.5),
    }
    best_name = None
    best_distance = float("inf")
    for name, target in palette.items():
        distance = color_distance(rgb, target)
        if distance < best_distance:
            best_distance = distance
            best_name = name
    return best_name


def detect_fill_operation_kind(props: dict[str, str], parser_mode: str, color_mapping_mode: bool) -> str:
    fill = props.get("fill", "")
    if color_mapping_mode:
        role = classify_color_role(fill)
        if role in {"cutout", "ignored"}:
            return "subtract"
        return "add"
    if parser_mode == "detect_visible_print_areas":
        luminance = color_luminance(fill)
        if luminance is None:
            return "add"
        return "subtract" if luminance >= 0.97 else "add"
    return "add"


def build_fill_geometry(segments: list[Segment], fill_rule: str) -> Optional[Any]:
    require_geometry_support()
    rings: list[dict[str, Any]] = []
    for segment in segments:
        ring = closed_segment_to_ring(segment)
        if ring is None:
            continue
        polygon = make_polygon_from_ring(ring)
        if polygon is None:
            continue
        rings.append({
            "ring": ring,
            "polygon": polygon,
            "winding": 1 if signed_ring_area(ring) >= 0 else -1,
            "area": abs(signed_ring_area(ring)),
            "sample": polygon.representative_point(),
        })

    if not rings:
        return None

    rings.sort(key=lambda item: item["area"], reverse=True)
    for index, ring_info in enumerate(rings):
        parent_index = None
        for candidate_index in range(index - 1, -1, -1):
            candidate = rings[candidate_index]
            if candidate["polygon"].covers(ring_info["sample"]):
                if parent_index is None or candidate["area"] < rings[parent_index]["area"]:
                    parent_index = candidate_index
        ring_info["parent"] = parent_index

    def state_for_ring(index: int) -> tuple[int, bool]:
        ring_info = rings[index]
        cached = ring_info.get("state")
        if cached is not None:
            return cached
        parent_index = ring_info["parent"]
        if fill_rule == "evenodd":
            depth = 1 if parent_index is None else state_for_ring(parent_index)[0] + 1
            state = (depth, (depth % 2) == 1)
        else:
            winding = ring_info["winding"] if parent_index is None else state_for_ring(parent_index)[0] + ring_info["winding"]
            state = (winding, winding != 0)
        ring_info["state"] = state
        return state

    shell_indices: list[int] = []
    holes_for_shell: dict[int, list[int]] = {}
    for index, ring_info in enumerate(rings):
        parent_index = ring_info["parent"]
        current_value, current_filled = state_for_ring(index)
        parent_filled = state_for_ring(parent_index)[1] if parent_index is not None else False
        if current_filled and not parent_filled:
            shell_indices.append(index)
            holes_for_shell.setdefault(index, [])
        elif parent_filled and not current_filled:
            owner_index = parent_index
            while owner_index is not None:
                owner_value, owner_filled = state_for_ring(owner_index)
                owner_parent_filled = state_for_ring(rings[owner_index]["parent"])[1] if rings[owner_index]["parent"] is not None else False
                if owner_filled and not owner_parent_filled:
                    holes_for_shell.setdefault(owner_index, []).append(index)
                    break
                owner_index = rings[owner_index]["parent"]

    polygons: list[Any] = []
    for shell_index in shell_indices:
        shell_ring = rings[shell_index]["ring"]
        hole_rings = [rings[hole_index]["ring"] for hole_index in holes_for_shell.get(shell_index, [])]
        polygon = Polygon(
            [(point.x, point.y) for point in shell_ring],
            [[(point.x, point.y) for point in hole_ring] for hole_ring in hole_rings],
        )
        if not polygon.is_valid:
            polygon = make_valid(polygon) if make_valid is not None else polygon.buffer(0)
        if not polygon.is_empty:
            polygons.append(polygon)

    if not polygons:
        return None

    geometry = unary_union(polygons)
    if not geometry.is_valid:
        geometry = make_valid(geometry) if make_valid is not None else geometry.buffer(0)
    return geometry if not geometry.is_empty else None


def extract_local_href(elem: ET.Element) -> Optional[str]:
    href = (
        elem.attrib.get("href")
        or elem.attrib.get("{http://www.w3.org/1999/xlink}href")
        or elem.attrib.get("xlink:href")
        or ""
    ).strip()
    if not href:
        return None
    if href.startswith("#"):
        return href[1:]
    return "__external__"


def parse_url_reference(value: str) -> Optional[str]:
    match = re.fullmatch(r"url\(\s*#([^)]+)\s*\)", (value or "").strip())
    return match.group(1) if match else None


def merge_presentation_attrs(
    inherited: dict[str, str],
    elem: ET.Element,
    stylesheet_rules: Optional[dict[str, dict[str, str]]] = None,
) -> dict[str, str]:
    props = dict(inherited)
    stylesheet_map = stylesheet_props_for_elem(elem, stylesheet_rules or {})
    explicit_fill = inherited.get("__has_explicit_fill") == "1"
    explicit_stroke = inherited.get("__has_explicit_stroke") == "1"
    if "fill" in stylesheet_map:
        explicit_fill = True
    if "stroke" in stylesheet_map:
        explicit_stroke = True
    props.update(stylesheet_map)
    style_map = parse_style_map(elem.attrib.get("style", ""))
    if "fill" in style_map or "fill-opacity" in style_map:
        explicit_fill = True
    if "stroke" in style_map or "stroke-opacity" in style_map or "stroke-width" in style_map:
        explicit_stroke = True
    props.update(style_map)
    for key in [
        "fill", "stroke", "stroke-width", "opacity", "fill-opacity", "stroke-opacity",
        "display", "visibility", "fill-rule", "clip-rule", "clip-path", "mask",
    ]:
        if key in elem.attrib:
            props[key] = elem.attrib[key]
    if "fill" in elem.attrib or "fill-opacity" in elem.attrib:
        explicit_fill = True
    if "stroke" in elem.attrib or "stroke-opacity" in elem.attrib or "stroke-width" in elem.attrib:
        explicit_stroke = True
    props["__has_explicit_fill"] = "1" if explicit_fill else "0"
    props["__has_explicit_stroke"] = "1" if explicit_stroke else "0"
    return props


def get_computed_style(
    elem: ET.Element,
    inherited_style: dict[str, str],
    stylesheet_rules: Optional[dict[str, dict[str, str]]] = None,
) -> dict[str, str]:
    return merge_presentation_attrs(inherited_style, elem, stylesheet_rules)


def combined_opacity(props: dict[str, str], opacity_key: str) -> float:
    value = 1.0
    for key in ["opacity", opacity_key]:
        if key in props:
            try:
                value *= float(props[key])
            except Exception:
                continue
    return value


def effective_paint_opacity(props: dict[str, str], paint_key: str, opacity_key: str) -> float:
    return combined_opacity(props, opacity_key) * parse_svg_color_alpha(props.get(paint_key, ""))


def is_print_opaque(props: dict[str, str], paint_key: str, opacity_key: str) -> bool:
    return effective_paint_opacity(props, paint_key, opacity_key) >= SVG_MIN_PRINT_OPACITY


def parse_stroke_width(props: dict[str, str]) -> float:
    return max(0.0, parse_svg_length(props.get("stroke-width"), 1.0))


def has_visible_fill(props: dict[str, str]) -> bool:
    fill = props.get("fill", "").strip().lower()
    if fill in {"", "none", "transparent"}:
        return False
    if is_unsupported_paint_server(fill):
        return False
    return effective_paint_opacity(props, "fill", "fill-opacity") > 0


def has_visible_stroke(props: dict[str, str]) -> bool:
    stroke = props.get("stroke", "").strip().lower()
    if stroke in {"", "none", "transparent"}:
        return False
    return is_print_opaque(props, "stroke", "stroke-opacity") and parse_stroke_width(props) > 0


def is_hidden(props: dict[str, str]) -> bool:
    display = props.get("display", "").strip().lower()
    visibility = props.get("visibility", "").strip().lower()
    try:
        opacity = float(props.get("opacity", "1") or "1")
    except Exception:
        opacity = 1.0
    return display == "none" or visibility == "hidden" or opacity <= 0.0


def is_dark_visible_color(value: str) -> bool:
    luminance = color_luminance(value)
    return luminance is not None and luminance <= 0.45


def is_dark_fill(style: dict[str, str]) -> bool:
    if style.get("__has_explicit_fill") != "1" or not has_visible_fill(style) or not is_print_opaque(style, "fill", "fill-opacity"):
        return False
    luminance = color_luminance(style.get("fill", ""))
    return luminance is not None and luminance <= SVG_DARK_FILL_LUMINANCE_THRESHOLD


def is_light_cutout(style: dict[str, str]) -> bool:
    if style.get("__has_explicit_fill") != "1" or not has_visible_fill(style) or not is_print_opaque(style, "fill", "fill-opacity"):
        return False
    luminance = color_luminance(style.get("fill", ""))
    return luminance is not None and luminance >= SVG_LIGHT_CUTOUT_LUMINANCE_THRESHOLD


def is_transparent_cutout(style: dict[str, str]) -> bool:
    if style.get("__has_explicit_fill") != "1" or not has_visible_fill(style):
        return False
    return not is_print_opaque(style, "fill", "fill-opacity")


def has_printable_fill(style: dict[str, str], fill_only_dark_svg_fills: bool = True) -> bool:
    if style.get("__has_explicit_fill") != "1" or not has_visible_fill(style) or not is_print_opaque(style, "fill", "fill-opacity"):
        return False
    if fill_only_dark_svg_fills:
        return is_dark_fill(style)
    return not is_light_cutout(style)


def classify_fill_style(style: dict[str, str], fill_only_dark_svg_fills: bool = True) -> str:
    fill = style.get("fill", "").strip().lower()
    if style.get("__has_explicit_fill") != "1":
        return "none"
    if fill in {"", "none", "transparent"}:
        return "none"
    if is_unsupported_paint_server(fill):
        return "unsupported"
    if effective_paint_opacity(style, "fill", "fill-opacity") <= 0:
        return "transparent"
    if is_transparent_cutout(style):
        return "transparent-cutout"
    if is_dark_fill(style):
        return "dark-fill"
    if is_light_cutout(style):
        return "light-cutout"
    if has_printable_fill(style, fill_only_dark_svg_fills):
        return "printable-fill"
    return "decorative-fill"


def should_promote_stroke_to_detail(
    props: dict[str, str],
    source_tag: str,
    parser_mode: str,
    color_mapping_mode: bool,
) -> bool:
    if source_tag == "text":
        return True
    width = parse_stroke_width(props)
    if color_mapping_mode:
        return classify_color_role(props.get("stroke", "")) == "detail"
    if parser_mode == "detect_visible_print_areas":
        return width <= 2.0 and is_dark_visible_color(props.get("stroke", ""))
    return width <= 1.5


def geometry_to_line_segments(geometry: Any, closed: bool = False) -> list[Segment]:
    segments: list[Segment] = []
    for line in extract_lines(geometry):
        points = [Point(x, y) for x, y in line.coords]
        if len(points) >= 2:
            segments.append(Segment(points, closed=closed))
    return segments


def clip_segments_to_geometry(segments: list[Segment], clip_geometry: Any) -> list[Segment]:
    if clip_geometry is None or clip_geometry.is_empty:
        return []
    clipped: list[Segment] = []
    for segment in segments:
        if len(segment.points) < 2:
            continue
        line = LineString([(point.x, point.y) for point in segment.points])
        clipped_geometry = line.intersection(clip_geometry)
        clipped.extend(geometry_to_line_segments(clipped_geometry, closed=segment.closed))
    return clipped


def path_strings_from_geometry(geometry: Any) -> tuple[list[str], list[str]]:
    outers: list[str] = []
    holes: list[str] = []
    for polygon in normalize_geometry(geometry):
        outers.append(format_svg_path([Point(x, y) for x, y in polygon.exterior.coords], True))
        for interior in polygon.interiors:
            holes.append(format_svg_path([Point(x, y) for x, y in interior.coords], True))
    return outers, holes


def build_diagnostic_message(model: SvgPrintModel) -> str:
    if model.fills or model.strokes or model.details:
        return ""
    reasons = {entry.reason for entry in model.ignored}
    if any("text" in reason.lower() for reason in reasons):
        return "Only text found; convert text to paths or enable text outlining."
    if any("stroke" in reason.lower() for reason in reasons):
        return "Only strokes found; enable stroke plotting or stroke-to-path conversion."
    if any("hidden" in reason.lower() for reason in reasons):
        return "Elements hidden by display:none or opacity:0."
    if any("mask" in reason.lower() or "clip" in reason.lower() for reason in reasons):
        return "Unsupported masks/clips detected."
    if any("css" in reason.lower() for reason in reasons):
        return "CSS styles could not be resolved."
    if any("use" in reason.lower() or "symbol" in reason.lower() for reason in reasons):
        return "All geometry is inside unsupported <use>/<symbol> references."
    return "Visible SVG content could not be normalized into drawable geometry."


def geometry_to_boundary_segments(geometry: Any) -> list[Segment]:
    segments: list[Segment] = []
    for polygon in normalize_geometry(geometry):
        segments.append(Segment([Point(x, y) for x, y in polygon.exterior.coords], closed=True))
        for interior in polygon.interiors:
            segments.append(Segment([Point(x, y) for x, y in interior.coords], closed=True))
    return segments


def compose_classified_elements(
    elements: list[ClassifiedSvgElement],
    *,
    trace_stroke_only_paths: bool,
    parser_mode: str,
    color_mapping_mode: bool,
) -> tuple[GeometryBundle, SvgPrintModel, dict[str, int]]:
    bundle = GeometryBundle()
    model = SvgPrintModel()
    classification_counts = {
        "dark_filled_polygons": 0,
        "light_cutout_polygons": 0,
        "transparent_cutout_polygons": 0,
        "stroke_only_paths": 0,
        "ignored_paths": 0,
    }
    dark_fill_geometries: list[Any] = []
    cutout_geometries: list[Any] = []

    for item in elements:
        props = item.computed_style
        if item.has_stroke:
            if not item.is_stroke_only or trace_stroke_only_paths:
                bundle.outline_segments.extend(item.stroke_segments)
                for segment in item.stroke_segments:
                    path_text = format_svg_path(segment.points, segment.closed)
                    if not path_text:
                        continue
                    model.strokes.append(NormalizedStrokePath(
                        type="stroke_path",
                        path=path_text,
                        strokeColor=props.get("stroke", ""),
                        strokeWidth=parse_stroke_width(props),
                        source="stroke",
                    ))
                    if should_promote_stroke_to_detail(props, item.tag, parser_mode, color_mapping_mode):
                        model.details.append(NormalizedDetailPath(type="detail_path", path=path_text, source="stroke"))
            if item.is_stroke_only:
                classification_counts["stroke_only_paths"] += 1

        if item.has_fill and item.fill_geometry is not None and not item.fill_geometry.is_empty:
            dark_fill_geometries.append(item.fill_geometry)
            classification_counts["dark_filled_polygons"] += len(normalize_geometry(item.fill_geometry))
            outer_paths, holes = path_strings_from_geometry(item.fill_geometry)
            model.fills.append(NormalizedFillRegion(
                type="filled_region",
                paths=outer_paths,
                fillColor=props.get("fill", ""),
                fillRule=item.fill_rule,
                holes=holes,
                source="path" if item.tag == "path" else "shape",
            ))
        elif item.is_cutout_fill and item.fill_geometry is not None and not item.fill_geometry.is_empty:
            cutout_geometries.append(item.fill_geometry)
            polygon_count = len(normalize_geometry(item.fill_geometry))
            outer_paths, holes = path_strings_from_geometry(item.fill_geometry)
            model.cutouts.append(NormalizedFillRegion(
                type="cutout_region",
                paths=outer_paths,
                fillColor=props.get("fill", ""),
                fillRule=item.fill_rule,
                holes=holes,
                source="path" if item.tag == "path" else "shape",
            ))
            if item.fill_classification == "transparent-cutout":
                classification_counts["transparent_cutout_polygons"] += polygon_count
            else:
                classification_counts["light_cutout_polygons"] += polygon_count
        elif not item.has_stroke:
            classification_counts["ignored_paths"] += 1
            model.ignored.append(IgnoredSvgElement(
                reason="Visible geometry resolved to no printable fill or stroke",
                element=item.element,
            ))

    composed_fill = unary_union(dark_fill_geometries) if dark_fill_geometries else None
    if composed_fill is not None and not composed_fill.is_empty and cutout_geometries:
        cutout_union = unary_union(cutout_geometries)
        if cutout_union is not None and not cutout_union.is_empty:
            composed_fill = composed_fill.difference(cutout_union)
    if composed_fill is not None and not composed_fill.is_empty and not composed_fill.is_valid:
        composed_fill = make_valid(composed_fill) if make_valid is not None else composed_fill.buffer(0)
    if composed_fill is not None and not composed_fill.is_empty:
        bundle.fill_shapes.append(SvgFillShape(geometry=composed_fill, fill_rule="evenodd", source_tag="composited-visible-fill"))
        bundle.fill_boundary_segments.extend(geometry_to_boundary_segments(composed_fill))

    model.metadata["classificationCounts"] = classification_counts
    return bundle, model, classification_counts


def analyze_svg(
    svg_text: str,
    parser_mode: str = "visible_geometry",
    color_mapping_mode: bool = False,
    trace_stroke_only_paths: bool = True,
    fill_only_dark_svg_fills: bool = True,
    debug: Optional[dict[str, Any]] = None,
) -> SvgAnalysisResult:
    require_geometry_support()
    try:
        root = ET.fromstring(svg_text)
    except ET.ParseError as exc:
        raise ValueError(f"Invalid SVG XML: {exc}") from exc

    if strip_namespace(root.tag) != "svg":
        raise ValueError("Uploaded file is not an SVG document")

    stylesheet_rules = parse_svg_stylesheet(root, debug=debug)
    id_map: dict[str, ET.Element] = {}
    for elem in root.iter():
        elem_id = elem.attrib.get("id", "").strip()
        if elem_id:
            id_map[elem_id] = elem

    default_props = {
        "fill": "#000000",
        "stroke": "none",
        "stroke-width": "1",
        "opacity": "1",
        "fill-opacity": "1",
        "stroke-opacity": "1",
        "visibility": "visible",
        "display": "inline",
        "fill-rule": "nonzero",
        "clip-rule": "nonzero",
    }

    viewbox_bounds: Optional[SvgBounds] = None
    viewbox = root.attrib.get("viewBox")
    if viewbox:
        nums = [float(n) for n in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", viewbox)]
        if len(nums) == 4:
            viewbox_bounds = SvgBounds(nums[0], nums[1], nums[0] + nums[2], nums[1] + nums[3])

    model_metadata = {
        "viewBox": viewbox or "",
        "width": parse_svg_length(root.attrib.get("width")),
        "height": parse_svg_length(root.attrib.get("height")),
        "units": {
            "width": root.attrib.get("width", ""),
            "height": root.attrib.get("height", ""),
        },
        "parserMode": parser_mode,
        "colorMappingMode": color_mapping_mode,
        "traceStrokeOnlyPaths": trace_stroke_only_paths,
        "fillOnlyDarkSvgFills": fill_only_dark_svg_fills,
    }
    warnings: list[str] = []
    ignored_non_drawable: list[IgnoredSvgElement] = []
    classified_elements: list[ClassifiedSvgElement] = []

    for elem in root.iter():
        tag = strip_namespace(elem.tag)
        if tag in {"script", "foreignObject"}:
            raise ValueError(f"Unsafe SVG element rejected: <{tag}>")
        href = extract_local_href(elem)
        if href == "__external__":
            raise ValueError(f"External SVG references are not allowed on <{tag}>")

    clip_cache: dict[str, Any] = {}
    warning_seen: set[str] = set()

    def push_warning(key: str, message: str) -> None:
        if key in warning_seen:
            return
        warning_seen.add(key)
        warnings.append(message)
        debug_append_warning(debug, message)

    def element_label(elem: ET.Element, tag_override: Optional[str] = None) -> str:
        tag_name = tag_override or strip_namespace(elem.tag)
        elem_id = elem.attrib.get("id", "").strip()
        return f"<{tag_name}#{elem_id}>" if elem_id else f"<{tag_name}>"

    def add_ignored(reason: str, elem: ET.Element, tag_override: Optional[str] = None) -> None:
        ignored_non_drawable.append(IgnoredSvgElement(reason=reason, element=element_label(elem, tag_override)))

    def clip_geometry_for_reference(ref_id: str, stack: set[str]) -> Optional[Any]:
        if ref_id in clip_cache:
            return clip_cache[ref_id]
        clip_elem = id_map.get(ref_id)
        if clip_elem is None or strip_namespace(clip_elem.tag) != "clipPath":
            return None
        if ref_id in stack:
            push_warning("recursive-clip", f"Recursive clipPath ignored: #{ref_id}")
            return None

        collected: list[Any] = []

        def walk_clip(elem: ET.Element, parent_matrix: tuple[float, float, float, float, float, float]) -> None:
            tag = strip_namespace(elem.tag)
            matrix = multiply_svg_matrices(parent_matrix, parse_svg_transform(elem.attrib.get("transform")))
            if tag == "use":
                href_id = extract_local_href(elem)
                if href_id and href_id not in {"__external__", ref_id}:
                    target = id_map.get(href_id)
                    if target is not None:
                        translated = multiply_svg_matrices(
                            matrix,
                            parse_svg_transform(f"translate({parse_svg_length(elem.attrib.get('x'))},{parse_svg_length(elem.attrib.get('y'))})"),
                        )
                        walk_clip(target, translated)
                return
            if tag in {"path", "polyline", "polygon", "line", "rect", "circle", "ellipse"}:
                segments = [transform_segment(segment, matrix) for segment in element_segments(tag, elem)]
                fill_segments = [ensure_segment_closed(segment) for segment in segments if len(segment.points) >= 3]
                geometry = build_fill_geometry(fill_segments, elem.attrib.get("clip-rule", "nonzero").strip().lower() or "nonzero") if fill_segments else None
                if geometry is not None and not geometry.is_empty:
                    collected.append(geometry)
            for child in list(elem):
                walk_clip(child, matrix)

        walk_clip(clip_elem, SVG_MATRIX_IDENTITY)
        geometry = unary_union(collected) if collected else None
        if geometry is not None and not geometry.is_empty and not geometry.is_valid:
            geometry = make_valid(geometry) if make_valid is not None else geometry.buffer(0)
        clip_cache[ref_id] = geometry
        return geometry

    def walk(
        elem: ET.Element,
        parent_matrix: tuple[float, float, float, float, float, float],
        inherited_props: dict[str, str],
        active_clip: Any = None,
        ref_stack: Optional[set[str]] = None,
    ) -> None:
        tag = strip_namespace(elem.tag)
        if tag in {"defs", "symbol", "clipPath"}:
            return

        props = get_computed_style(elem, inherited_props, stylesheet_rules)
        if is_hidden(props):
            add_ignored("Element hidden by display:none, visibility:hidden, or opacity:0", elem)
            return

        matrix = multiply_svg_matrices(parent_matrix, parse_svg_transform(elem.attrib.get("transform")))

        if tag == "use":
            href_id = extract_local_href(elem)
            if not href_id:
                add_ignored("Unresolved <use> without href", elem)
                return
            if href_id == "__external__":
                add_ignored("External <use> reference ignored", elem)
                return
            target = id_map.get(href_id)
            if target is None:
                add_ignored(f"Unresolved <use> reference #{href_id}", elem)
                return
            current_stack = ref_stack or set()
            if href_id in current_stack:
                add_ignored(f"Recursive <use> reference #{href_id} ignored", elem)
                return
            translated = multiply_svg_matrices(
                matrix,
                parse_svg_transform(f"translate({parse_svg_length(elem.attrib.get('x'))},{parse_svg_length(elem.attrib.get('y'))})"),
            )
            walk(target, translated, props, active_clip=active_clip, ref_stack=current_stack | {href_id})
            return

        if tag == "mask":
            push_warning("mask", "Masks are present but not fully supported; convert masked content to paths if output looks wrong.")
            add_ignored("Unsupported mask definition", elem)
            return

        if tag == "text":
            push_warning("text", "Text must be converted to paths before plotting.")
            add_ignored("Visible text requires conversion to paths before plotting", elem)
            debug_append_parser_entry(debug, {
                "element": element_label(elem),
                "tag": tag,
                "source": "text",
                "computedStyle": {key: value for key, value in props.items() if not key.startswith("__")},
                "styleSource": "computed",
                "ignored": True,
                "reason": "Text requires outline conversion",
            })
            return

        if "filter" in elem.attrib:
            push_warning("filter", "SVG filters are not supported by the vector parser; visible output may differ.")

        element_clip = active_clip
        clip_ref = parse_url_reference(props.get("clip-path", ""))
        if clip_ref:
            clip_geometry = clip_geometry_for_reference(clip_ref, ref_stack or set())
            if clip_geometry is None:
                push_warning("clip", "Unsupported clip path detected; some geometry may be unclipped.")
                add_ignored(f"Unsupported or unresolved clip-path #{clip_ref}", elem)
            elif element_clip is None:
                element_clip = clip_geometry
            else:
                element_clip = element_clip.intersection(clip_geometry)

        if props.get("mask"):
            push_warning("mask", "Masks are present but not fully supported; convert masked content to paths if output looks wrong.")
            add_ignored("Unsupported mask on visible element", elem)

        if tag in {"path", "polyline", "polygon", "line", "rect", "circle", "ellipse"}:
            raw_segments = element_segments(tag, elem)
            debug_append_segments(debug, "parsed_paths", raw_segments, f"{tag}-parsed")
            debug_append_segments(debug, "flattened_paths", raw_segments, f"{tag}-flattened")
            transformed_segments = [transform_segment(segment, matrix) for segment in raw_segments]

            fill_segments = [ensure_segment_closed(segment) for segment in transformed_segments if len(segment.points) >= 3]
            fill_rule = props.get("fill-rule", "nonzero").strip().lower() or "nonzero"
            fill_geometry = build_fill_geometry(fill_segments, fill_rule) if fill_segments else None
            if fill_geometry is not None and element_clip is not None and not element_clip.is_empty:
                fill_geometry = fill_geometry.intersection(element_clip)

            stroke_segments = transformed_segments
            if element_clip is not None and stroke_segments:
                stroke_segments = clip_segments_to_geometry(stroke_segments, element_clip)

            debug_append_segments(debug, "transformed_paths", stroke_segments or transformed_segments, f"{tag}-transformed")

            if is_unsupported_paint_server(props.get("fill", "")):
                push_warning("fill-paint", "Unsupported SVG fill paint server detected; non-solid fills are ignored for slicer regions.")
            fill_classification = classify_fill_style(props, fill_only_dark_svg_fills)
            has_fill_geometry = fill_geometry is not None and not fill_geometry.is_empty
            has_fill = fill_classification in {"dark-fill", "printable-fill"} and has_fill_geometry
            is_cutout_fill = fill_classification in {"light-cutout", "transparent-cutout"} and has_fill_geometry

            stroke_only_candidate = has_visible_stroke(props) and bool(stroke_segments)
            has_stroke = stroke_only_candidate
            if parser_mode == "detect_visible_print_areas":
                has_stroke = has_stroke and is_dark_visible_color(props.get("stroke", ""))
            elif color_mapping_mode:
                stroke_role = classify_color_role(props.get("stroke", ""))
                has_stroke = has_stroke and stroke_role not in {"cutout", "ignored"}

            public_props = {key: value for key, value in props.items() if not key.startswith("__")}
            parser_entry = {
                "element": element_label(elem),
                "tag": tag,
                "computedStyle": public_props,
                "styleSource": "computed",
                "fillVisible": has_fill,
                "cutoutFill": is_cutout_fill,
                "strokeVisible": has_stroke,
                "strokeWidth": parse_stroke_width(props),
                "fillClassification": fill_classification,
                "clipApplied": bool(element_clip is not None),
            }

            classified_elements.append(ClassifiedSvgElement(
                element=element_label(elem),
                tag=tag,
                computed_style=public_props,
                fill_geometry=fill_geometry,
                stroke_segments=stroke_segments,
                fill_rule=fill_rule,
                fill_classification=fill_classification,
                has_fill=has_fill,
                is_cutout_fill=is_cutout_fill,
                has_stroke=has_stroke,
                is_stroke_only=not has_fill and not is_cutout_fill and has_stroke,
            ))

            debug_append_parser_entry(debug, parser_entry)
        elif tag in {"image", "pattern"}:
            add_ignored(f"Unsupported drawable element <{tag}>", elem)

        for child in list(elem):
            walk(child, matrix, props, active_clip=element_clip, ref_stack=ref_stack)

    walk(root, SVG_MATRIX_IDENTITY, default_props)
    bundle, model, classification_counts = compose_classified_elements(
        classified_elements,
        trace_stroke_only_paths=trace_stroke_only_paths,
        parser_mode=parser_mode,
        color_mapping_mode=color_mapping_mode,
    )
    model.metadata.update(model_metadata)
    model.warnings.extend(warnings)
    model.ignored = ignored_non_drawable + model.ignored
    debug_set_counts(debug, "classification_counts", classification_counts)

    try:
        model.computed_bounds = asdict(bounds_from_bundle(bundle))
    except Exception:
        model.computed_bounds = None

    diagnostic = build_diagnostic_message(model)
    if diagnostic:
        model.diagnostics.append(diagnostic)

    return SvgAnalysisResult(bundle=bundle, print_model=model, viewbox_bounds=viewbox_bounds)


def extract_svg_bundle(
    svg_text: str,
    debug: Optional[dict[str, Any]] = None,
    parser_mode: str = "visible_geometry",
    color_mapping_mode: bool = False,
    trace_stroke_only_paths: bool = True,
    fill_only_dark_svg_fills: bool = True,
) -> tuple[GeometryBundle, Optional[SvgBounds], SvgPrintModel]:
    result = analyze_svg(
        svg_text,
        parser_mode=parser_mode,
        color_mapping_mode=color_mapping_mode,
        trace_stroke_only_paths=trace_stroke_only_paths,
        fill_only_dark_svg_fills=fill_only_dark_svg_fills,
        debug=debug,
    )
    return result.bundle, result.viewbox_bounds, result.print_model


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


def bounds_from_bundle(bundle: GeometryBundle) -> SvgBounds:
    min_x = min_y = float("inf")
    max_x = max_y = float("-inf")

    all_segments = bundle.outline_segments + bundle.fill_boundary_segments
    if all_segments:
        seg_bounds = bounds_from_segments(all_segments)
        min_x = min(min_x, seg_bounds.min_x)
        min_y = min(min_y, seg_bounds.min_y)
        max_x = max(max_x, seg_bounds.max_x)
        max_y = max(max_y, seg_bounds.max_y)

    for fill_shape in bundle.fill_shapes:
        if fill_shape.geometry.is_empty:
            continue
        gx1, gy1, gx2, gy2 = fill_shape.geometry.bounds
        min_x = min(min_x, gx1)
        min_y = min(min_y, gy1)
        max_x = max(max_x, gx2)
        max_y = max(max_y, gy2)

    if not math.isfinite(min_x):
        raise ValueError("SVG contains no drawable paths/shapes")
    return SvgBounds(min_x=min_x, min_y=min_y, max_x=max_x, max_y=max_y)


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


def map_bundle_to_angles(
    bundle: GeometryBundle,
    bounds: SvgBounds,
    fit_mode: str,
    invert_y: bool,
    margin_percent: float,
) -> GeometryBundle:
    margin_x = (X_DRAW_MAX - X_DRAW_MIN) * (margin_percent / 100.0)
    margin_y = (Y_DRAW_MAX - Y_DRAW_MIN) * (margin_percent / 100.0)

    target_w = (X_DRAW_MAX - X_DRAW_MIN) - margin_x * 2
    target_h = (Y_DRAW_MAX - Y_DRAW_MIN) - margin_y * 2
    if target_w <= 0 or target_h <= 0:
        raise ValueError("Margin is too large")

    if fit_mode == "stretch":
        scale_x = target_w / bounds.width
        scale_y = target_h / bounds.height
        base_x = X_DRAW_MIN + margin_x - (bounds.min_x * scale_x)
        if invert_y:
            base_y = Y_DRAW_MIN + margin_y + (bounds.max_y * scale_y)
            matrix = [scale_x, 0.0, 0.0, -scale_y, base_x, base_y]
        else:
            base_y = Y_DRAW_MIN + margin_y - (bounds.min_y * scale_y)
            matrix = [scale_x, 0.0, 0.0, scale_y, base_x, base_y]
    else:
        scale = min(target_w / bounds.width, target_h / bounds.height)
        used_w = bounds.width * scale
        used_h = bounds.height * scale
        offset_x = X_DRAW_MIN + margin_x + (target_w - used_w) / 2.0
        offset_y = Y_DRAW_MIN + margin_y + (target_h - used_h) / 2.0
        base_x = offset_x - (bounds.min_x * scale)
        if invert_y:
            base_y = offset_y + (bounds.max_y * scale)
            matrix = [scale, 0.0, 0.0, -scale, base_x, base_y]
        else:
            base_y = offset_y - (bounds.min_y * scale)
            matrix = [scale, 0.0, 0.0, scale, base_x, base_y]

    outline_segments = [
        Segment([Point((matrix[0] * p.x) + matrix[4], (matrix[3] * p.y) + matrix[5]) for p in seg.points], closed=seg.closed)
        for seg in bundle.outline_segments
    ]
    fill_boundary_segments = [
        Segment([Point((matrix[0] * p.x) + matrix[4], (matrix[3] * p.y) + matrix[5]) for p in seg.points], closed=seg.closed)
        for seg in bundle.fill_boundary_segments
    ]
    fill_shapes = [
        SvgFillShape(
            geometry=affinity.affine_transform(fill_shape.geometry, matrix),
            fill_rule=fill_shape.fill_rule,
            source_tag=fill_shape.source_tag,
        )
        for fill_shape in bundle.fill_shapes
    ]
    return GeometryBundle(outline_segments=outline_segments, fill_boundary_segments=fill_boundary_segments, fill_shapes=fill_shapes)


def apply_placement_transform(
    bundle: GeometryBundle,
    scale_percent: float,
    rotation_deg: float,
    offset_x: float,
    offset_y: float,
) -> GeometryBundle:
    if scale_percent <= 0:
        raise ValueError("Placement scale must be greater than 0")

    if not bundle.outline_segments and not bundle.fill_boundary_segments and not bundle.fill_shapes:
        return GeometryBundle()

    scale = scale_percent / 100.0
    bounds = bounds_from_bundle(bundle)
    center_x = (bounds.min_x + bounds.max_x) / 2.0
    center_y = (bounds.min_y + bounds.max_y) / 2.0
    angle = math.radians(rotation_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    def place_point(point: Point) -> Point:
        scaled_x = center_x + ((point.x - center_x) * scale)
        scaled_y = center_y + ((point.y - center_y) * scale)
        rel_x = scaled_x - center_x
        rel_y = scaled_y - center_y
        rotated_x = center_x + (rel_x * cos_a) - (rel_y * sin_a)
        rotated_y = center_y + (rel_x * sin_a) + (rel_y * cos_a)
        return Point(rotated_x + offset_x, rotated_y + offset_y)

    outline_segments = [
        Segment([place_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.outline_segments
    ]
    fill_boundary_segments = [
        Segment([place_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.fill_boundary_segments
    ]

    fill_shapes = []
    for fill_shape in bundle.fill_shapes:
        geometry = affinity.scale(fill_shape.geometry, xfact=scale, yfact=scale, origin=(center_x, center_y))
        geometry = affinity.rotate(geometry, rotation_deg, origin=(center_x, center_y))
        geometry = affinity.translate(geometry, xoff=offset_x, yoff=offset_y)
        fill_shapes.append(SvgFillShape(geometry=geometry, fill_rule=fill_shape.fill_rule, source_tag=fill_shape.source_tag))

    return GeometryBundle(outline_segments=outline_segments, fill_boundary_segments=fill_boundary_segments, fill_shapes=fill_shapes)


def segment_length(points: list[Point]) -> float:
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))


def mm_area_to_ball_degree_area(area_mm2: float) -> float:
    scale = 360.0 / BALL_DIAMETER_MM
    return area_mm2 * scale * scale


def simplify_segment_points(points: list[Point], tolerance: float, closed: bool) -> list[Point]:
    if len(points) < 2 or tolerance <= 0:
        return points
    coords = [(point.x, point.y) for point in points]
    geometry = Polygon(coords) if closed and len(coords) >= 4 else LineString(coords)
    simplified = geometry.simplify(tolerance, preserve_topology=True)
    if closed:
        if isinstance(simplified, Polygon):
            out = [Point(x, y) for x, y in simplified.exterior.coords]
        else:
            out = [Point(x, y) for x, y in geometry.exterior.coords]
    else:
        if isinstance(simplified, LineString):
            out = [Point(x, y) for x, y in simplified.coords]
        else:
            out = [Point(x, y) for x, y in geometry.coords]
    return out if len(out) >= 2 else points


def normalize_geometry(geometry: Any) -> list[Polygon]:
    if geometry is None or geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, MultiPolygon):
        return [geom for geom in geometry.geoms if not geom.is_empty]
    if hasattr(geometry, "geoms"):
        polygons: list[Polygon] = []
        for geom in geometry.geoms:
            polygons.extend(normalize_geometry(geom))
        return polygons
    return []


def debug_append_bundle(debug: Optional[dict[str, Any]], key: str, bundle: GeometryBundle) -> None:
    if debug is None:
        return
    debug_append_segments(debug, key, bundle.outline_segments, f"{key}-outline")
    debug_append_segments(debug, key, bundle.fill_boundary_segments, f"{key}-fill-boundary")


def geometry_to_closed_toolpaths(geometry: Any, kind: str, tolerance: float) -> list[Toolpath]:
    paths: list[Toolpath] = []
    for polygon in normalize_geometry(geometry):
        paths.append(Toolpath(
            points=simplify_segment_points([Point(x, y) for x, y in polygon.exterior.coords], tolerance, True),
            kind=kind,
            closed=True,
        ))
        for interior in polygon.interiors:
            paths.append(Toolpath(
                points=simplify_segment_points([Point(x, y) for x, y in interior.coords], tolerance, True),
                kind=kind,
                closed=True,
            ))
    return paths


def extract_lines(geometry: Any) -> list[LineString]:
    if geometry is None or geometry.is_empty:
        return []
    if isinstance(geometry, LineString):
        return [geometry]
    if isinstance(geometry, MultiLineString):
        return [line for line in geometry.geoms if not line.is_empty]
    if isinstance(geometry, GeometryCollection):
        out: list[LineString] = []
        for geom in geometry.geoms:
            out.extend(extract_lines(geom))
        return out
    return []


def generate_zigzag_infill(
    region: Any,
    spacing: float,
    angle_deg: float,
    min_segment_length: float,
    tolerance: float,
    debug: Optional[dict[str, Any]] = None,
) -> list[Toolpath]:
    if region is None or region.is_empty or spacing <= 0:
        return []

    origin = region.centroid.coords[0]
    rotated = affinity.rotate(region, -angle_deg, origin=origin)
    min_x, min_y, max_x, max_y = rotated.bounds
    if not all(math.isfinite(value) for value in [min_x, min_y, max_x, max_y]):
        return []

    paths: list[Toolpath] = []
    row = 0
    y = min_y
    while y <= max_y + 1e-6:
        scan = LineString([(min_x - spacing, y), (max_x + spacing, y)])
        if debug is not None:
            scan_unrotated = affinity.rotate(scan, angle_deg, origin=origin)
            debug_append_toolpaths(debug, "hatch_before_clipping", [
                Toolpath(
                    points=[Point(x, y2) for x, y2 in scan_unrotated.coords],
                    kind="debug-hatch-before",
                    closed=False,
                )
            ])
        clipped = rotated.intersection(scan)
        row_lines = []
        for line in extract_lines(clipped):
            if line.length < min_segment_length:
                continue
            coords = list(line.coords)
            if row % 2 == 1:
                coords = list(reversed(coords))
            unrotated = affinity.rotate(LineString(coords), angle_deg, origin=origin)
            row_lines.append(Toolpath(
                points=simplify_segment_points([Point(x, y2) for x, y2 in unrotated.coords], tolerance, False),
                kind="fill-infill",
                closed=False,
            ))
        row_lines.sort(key=lambda path: (path.points[0].y if path.points else 0.0, path.points[0].x if path.points else 0.0))
        debug_append_toolpaths(debug, "hatch_after_clipping", row_lines)
        paths.extend(row_lines)
        y += spacing
        row += 1
    return paths


def path_signature(path: Toolpath, decimals: int = 4) -> tuple[Any, ...]:
    return (
        path.kind,
        path.closed,
        tuple((round(point.x, decimals), round(point.y, decimals)) for point in path.points),
    )


def dedupe_toolpaths(paths: list[Toolpath], minimum_length: float) -> list[Toolpath]:
    seen: set[tuple[Any, ...]] = set()
    deduped: list[Toolpath] = []
    for path in paths:
        if len(path.points) < 2:
            continue
        if segment_length(path.points) < minimum_length:
            continue
        signature = path_signature(path)
        if signature in seen:
            continue
        seen.add(signature)
        deduped.append(path)
    return deduped


def generate_toolpaths(
    bundle: GeometryBundle,
    *,
    enable_fill: bool,
    line_width_mm: float,
    wall_count: int,
    infill_spacing_mm: float,
    infill_angle_deg: float,
    outline_after_fill: bool,
    min_fill_area_mm2: float,
    min_fill_width_mm: float,
    simplify_tolerance_mm: float,
    remove_duplicate_paths: bool,
    small_shape_mode: str,
    min_segment_length_mm: float,
    debug: Optional[dict[str, Any]] = None,
) -> list[Toolpath]:
    toolpaths: list[Toolpath] = []
    line_width_deg = mm_to_ball_degrees(line_width_mm)
    infill_spacing_deg = mm_to_ball_degrees(infill_spacing_mm)
    simplify_tolerance_deg = mm_to_ball_degrees(simplify_tolerance_mm)
    min_segment_length_deg = mm_to_ball_degrees(min_segment_length_mm)
    min_fill_area_deg2 = mm_area_to_ball_degree_area(min_fill_area_mm2)
    min_fill_width_deg = mm_to_ball_degrees(min_fill_width_mm)

    outline_segments = list(bundle.outline_segments)
    if not enable_fill or outline_after_fill:
        outline_segments.extend(bundle.fill_boundary_segments)

    for segment in outline_segments:
        simplified = simplify_segment_points(segment.points, simplify_tolerance_deg, segment.closed)
        toolpaths.append(Toolpath(points=simplified, kind="outline", closed=segment.closed))

    if enable_fill and line_width_deg > 0:
        sorted_shapes = sorted(
            bundle.fill_shapes,
            key=lambda item: (
                round(item.geometry.centroid.y, 5),
                round(item.geometry.centroid.x, 5),
                round(item.geometry.area, 5),
            ),
        )
        for fill_shape in sorted_shapes:
            geometry = fill_shape.geometry
            if geometry.is_empty:
                continue
            if not geometry.is_valid:
                geometry = make_valid(geometry) if make_valid is not None else geometry.buffer(0)
            if geometry.is_empty:
                continue
            if simplify_tolerance_deg > 0:
                geometry = geometry.simplify(simplify_tolerance_deg, preserve_topology=True)
            polygons = normalize_geometry(geometry)
            for polygon in polygons:
                area = polygon.area
                bounds = polygon.bounds
                min_width = min(bounds[2] - bounds[0], bounds[3] - bounds[1])
                too_small = area < min_fill_area_deg2 or min_width < min_fill_width_deg
                if too_small and small_shape_mode == "skip":
                    continue

                wall_geometries = []
                for wall_index in range(max(1, wall_count)):
                    wall_offset = line_width_deg * (0.5 + wall_index)
                    wall_geometry = polygon.buffer(-wall_offset, join_style=1)
                    if not wall_geometry.is_empty:
                        wall_geometries.append(wall_geometry)

                if not wall_geometries:
                    if small_shape_mode == "single-wall":
                        wall_paths = geometry_to_closed_toolpaths(polygon, "fill-wall", simplify_tolerance_deg)
                    else:
                        continue
                else:
                    wall_paths = []
                    for wall_geometry in wall_geometries:
                        wall_paths.extend(geometry_to_closed_toolpaths(wall_geometry, "fill-wall", simplify_tolerance_deg))

                infill_paths: list[Toolpath] = []
                infill_region = polygon.buffer(-(line_width_deg * max(1, wall_count)), join_style=1)
                if not infill_region.is_empty:
                    infill_paths = generate_zigzag_infill(
                        region=infill_region,
                        spacing=max(infill_spacing_deg, line_width_deg),
                        angle_deg=infill_angle_deg,
                        min_segment_length=min_segment_length_deg,
                        tolerance=simplify_tolerance_deg,
                        debug=debug,
                    )

                toolpaths.extend(infill_paths)
                toolpaths.extend(wall_paths)

    if remove_duplicate_paths:
        toolpaths = dedupe_toolpaths(toolpaths, min_segment_length_deg)
    else:
        toolpaths = [path for path in toolpaths if segment_length(path.points) >= min_segment_length_deg]
    toolpath_counts = {
        "generated_fill_walls": sum(1 for path in toolpaths if path.kind == "fill-wall"),
        "generated_infill_paths": sum(1 for path in toolpaths if path.kind == "fill-infill"),
        "generated_outline_paths": sum(1 for path in toolpaths if path.kind == "outline"),
        "generated_travel_paths": sum(1 for path in toolpaths if path.kind == "travel"),
    }
    debug_set_counts(debug, "toolpath_counts", toolpath_counts)
    debug_append_toolpaths(debug, "final_toolpaths", toolpaths)
    return toolpaths


def generate_gcode_from_toolpaths(
    toolpaths: list[Toolpath],
    draw_feed: float,
    travel_feed: float,
    sample_step_deg: float,
    pen_up_s: int,
    pen_down_s: int,
    servo_ramp_enabled: bool,
    servo_ramp_step: int,
    servo_ramp_delay_ms: float,
    pen_up_dwell_ms: float,
    pen_down_dwell_ms: float,
    include_comments: bool,
) -> tuple[list[str], list[dict[str, Any]]]:
    g: list[str] = []
    preview: list[dict[str, Any]] = []
    current_servo = pen_up_s
    current_position = Point(0.0, 0.0)
    current_pen_down = False

    def comment(text: str) -> None:
        if include_comments:
            g.append(f"({text})")

    comment("Generated for golf ball plotter")
    comment("Units are angular degrees. X=-180..180 ball rotation, Y=-45..45 arm tilt")
    g.extend(["$X", "G21", "G90"])
    g.extend(build_pen_position_commands(
        pen_up_s,
        pen_up_s,
        ramp_enabled=False,
        ramp_step=servo_ramp_step,
        ramp_delay_ms=servo_ramp_delay_ms,
        dwell_ms=pen_up_dwell_ms,
    ))

    for index, toolpath in enumerate(toolpaths, start=1):
        pts = resample_segment(toolpath.points, max_step=max(0.05, sample_step_deg))
        if len(pts) < 2:
            continue

        start = pts[0]
        if not nearly_same_point(current_position, start):
            preview.append({"kind": "travel", "closed": False, "points": [asdict(current_position), asdict(start)]})
            if current_pen_down:
                g.extend(build_pen_position_commands(
                    current_servo,
                    pen_up_s,
                    ramp_enabled=servo_ramp_enabled,
                    ramp_step=servo_ramp_step,
                    ramp_delay_ms=servo_ramp_delay_ms,
                    dwell_ms=pen_up_dwell_ms,
                ))
                current_servo = pen_up_s
                current_pen_down = False
            comment(f"Travel to {toolpath.kind} path {index}")
            g.append(f"G1 X{start.x:.4f} Y{start.y:.4f} F{travel_feed:.3f}")
            current_position = start

        if not current_pen_down:
            g.extend(build_pen_position_commands(
                current_servo,
                pen_down_s,
                ramp_enabled=servo_ramp_enabled,
                ramp_step=servo_ramp_step,
                ramp_delay_ms=servo_ramp_delay_ms,
                dwell_ms=pen_down_dwell_ms,
            ))
            current_servo = pen_down_s
            current_pen_down = True

        preview.append({"kind": toolpath.kind, "closed": toolpath.closed, "points": [asdict(point) for point in pts]})
        comment(f"{toolpath.kind} path {index}, {len(pts)} points")
        for point in pts[1:]:
            g.append(f"G1 X{point.x:.4f} Y{point.y:.4f} F{draw_feed:.3f}")
            current_position = point

        g.extend(build_pen_position_commands(
            current_servo,
            pen_up_s,
            ramp_enabled=servo_ramp_enabled,
            ramp_step=servo_ramp_step,
            ramp_delay_ms=servo_ramp_delay_ms,
            dwell_ms=pen_up_dwell_ms,
        ))
        current_servo = pen_up_s
        current_pen_down = False

    comment("Return to zero with pen up")
    if not nearly_same_point(current_position, Point(0.0, 0.0)):
        preview.append({"kind": "travel", "closed": False, "points": [asdict(current_position), asdict(Point(0.0, 0.0))]})
        g.append(f"G1 X0.0000 Y0.0000 F{travel_feed:.3f}")
    g.extend(build_pen_position_commands(
        current_servo,
        pen_up_s,
        ramp_enabled=False,
        ramp_step=servo_ramp_step,
        ramp_delay_ms=servo_ramp_delay_ms,
        dwell_ms=pen_up_dwell_ms,
    ))

    return g, preview


def _svg_pipeline_assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def run_integrated_svg_pipeline_self_test() -> dict[str, Any]:
    checks: list[str] = []

    def expect(condition: bool, message: str) -> None:
        _svg_pipeline_assert(condition, message)
        checks.append(message)

    compositing_svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <rect x="0" y="0" width="100" height="100" fill="#111111"/>
      <rect x="30" y="30" width="40" height="40" fill="white"/>
      <rect x="10" y="10" width="80" height="80" fill="none" stroke="blue" stroke-width="0.5"/>
    </svg>
    """
    result = analyze_svg(compositing_svg, trace_stroke_only_paths=True, fill_only_dark_svg_fills=True, debug={})
    counts = result.print_model.metadata["classificationCounts"]
    expect(len(result.bundle.fill_shapes) == 1, "dark fill plus cutout composes into one printable region")
    expect(abs(result.bundle.fill_shapes[0].geometry.area - 8400.0) < 0.01, "light cutout subtracts from dark fill area")
    expect(counts["dark_filled_polygons"] == 1 and counts["light_cutout_polygons"] == 1, "dark fill and light cutout counts are classified correctly")
    expect(counts["stroke_only_paths"] == 1, "stroke-only geometry is tracked separately from fill geometry")
    toolpaths = generate_toolpaths(
        result.bundle,
        enable_fill=True,
        line_width_mm=0.75,
        wall_count=1,
        infill_spacing_mm=0.75,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        debug={},
    )
    expect(any(path.kind == "fill-wall" for path in toolpaths), "printable regions generate fill walls")
    expect(any(path.kind == "fill-infill" for path in toolpaths), "printable regions generate infill when geometry is large enough")

    stroke_only_svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10">
      <rect x="1" y="1" width="8" height="8" stroke="#000000" stroke-width="0.5"/>
    </svg>
    """
    result = analyze_svg(stroke_only_svg, trace_stroke_only_paths=True, fill_only_dark_svg_fills=True, debug={})
    counts = result.print_model.metadata["classificationCounts"]
    expect(len(result.bundle.fill_shapes) == 0, "closed stroke-only geometry does not create fill regions")
    expect(len(result.bundle.outline_segments) > 0, "stroke-only geometry still creates traceable outline segments")
    expect(counts["stroke_only_paths"] == 1, "stroke-only path count increments for closed outlines")

    inherited_svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <g style="fill: rgb(0,0,0); fill-opacity: 1;">
        <rect x="0" y="0" width="40" height="40"/>
      </g>
      <g style="fill: rgba(255,255,255,0.9);">
        <rect x="10" y="10" width="20" height="20"/>
      </g>
    </svg>
    """
    result = analyze_svg(inherited_svg, trace_stroke_only_paths=True, fill_only_dark_svg_fills=True, debug={})
    counts = result.print_model.metadata["classificationCounts"]
    expect(abs(result.bundle.fill_shapes[0].geometry.area - 1200.0) < 0.01, "group-inherited fill styles affect geometry classification")
    expect(counts["transparent_cutout_polygons"] == 1, "semi-transparent inherited fills become transparent cutouts")

    transparent_svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <rect x="0" y="0" width="100" height="100" fill="#000000"/>
      <rect x="30" y="30" width="40" height="40" fill="#000000" fill-opacity="0.2"/>
      <rect x="10" y="10" width="80" height="80" fill="none" stroke="#000000" stroke-opacity="0.2" stroke-width="2"/>
    </svg>
    """
    result = analyze_svg(transparent_svg, trace_stroke_only_paths=True, fill_only_dark_svg_fills=True, debug={})
    counts = result.print_model.metadata["classificationCounts"]
    expect(abs(result.bundle.fill_shapes[0].geometry.area - 8400.0) < 0.01, "transparent fills subtract from printable geometry")
    expect(counts["transparent_cutout_polygons"] == 1, "transparent fill cutouts are counted explicitly")
    expect(len(result.bundle.outline_segments) == 0, "transparent strokes are ignored for tracing")

    compound_path_svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <path fill="#000000" d="M 0 0 L 100 0 L 100 100 L 0 100 Z M 30 30 L 30 70 L 70 70 L 70 30 Z"/>
    </svg>
    """
    result = analyze_svg(compound_path_svg, trace_stroke_only_paths=True, fill_only_dark_svg_fills=True, debug={})
    expect(abs(result.bundle.fill_shapes[0].geometry.area - 8400.0) < 0.01, "compound path holes are preserved in fill geometry")

    arsenal_path = os.path.join(os.getcwd(), "Arsenal.svg")
    if os.path.exists(arsenal_path):
        arsenal_svg = open(arsenal_path, "r", encoding="utf-8", errors="ignore").read()
        arsenal_result = analyze_svg(arsenal_svg, trace_stroke_only_paths=True, fill_only_dark_svg_fills=True, debug={})
        arsenal_polygons = normalize_geometry(arsenal_result.bundle.fill_shapes[0].geometry) if arsenal_result.bundle.fill_shapes else []
        arsenal_counts = arsenal_result.print_model.metadata["classificationCounts"]
        expect(bool(arsenal_polygons), "Arsenal.svg produces printable polygons after parsing")
        expect(len(arsenal_polygons) >= 10, "Arsenal.svg resolves into multiple separated printable polygons")
        expect(arsenal_counts["transparent_cutout_polygons"] == 0, "Arsenal.svg contains no explicit transparent cutout fills")

    return {
        "passed": len(checks),
        "messages": checks,
    }


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
        default_servo_ramp_enabled=DEFAULT_SERVO_RAMP_ENABLED,
        default_servo_ramp_step=DEFAULT_SERVO_RAMP_STEP,
        default_servo_ramp_delay_ms=DEFAULT_SERVO_RAMP_DELAY_MS,
        default_pen_up_dwell_ms=DEFAULT_PEN_UP_DWELL_MS,
        default_pen_down_dwell_ms=DEFAULT_PEN_DOWN_DWELL_MS,
        default_sample_step_deg=DEFAULT_SAMPLE_STEP_DEG,
        default_margin_percent=DEFAULT_MARGIN_PERCENT,
        default_rotation_deg=DEFAULT_ROTATION_DEG,
        default_parser_mode=DEFAULT_PARSER_MODE,
        default_color_mapping_mode=DEFAULT_COLOR_MAPPING_MODE,
        default_enable_fill=DEFAULT_ENABLE_FILL,
        default_trace_stroke_only_paths=DEFAULT_TRACE_STROKE_ONLY_PATHS,
        default_fill_only_dark_svg_fills=DEFAULT_FILL_ONLY_DARK_SVG_FILLS,
        default_wall_count=DEFAULT_WALL_COUNT,
        default_infill_spacing_mm=DEFAULT_INFILL_SPACING_MM,
        default_infill_angle_deg=DEFAULT_INFILL_ANGLE_DEG,
        default_min_fill_area_mm2=DEFAULT_MIN_FILL_AREA_MM2,
        default_min_fill_width_mm=DEFAULT_MIN_FILL_WIDTH_MM,
        default_simplify_tolerance_mm=DEFAULT_SIMPLIFY_TOLERANCE_MM,
        default_outline_after_fill=DEFAULT_OUTLINE_AFTER_FILL,
        default_remove_duplicate_paths=DEFAULT_REMOVE_DUPLICATE_PATHS,
        default_min_segment_length_mm=DEFAULT_MIN_SEGMENT_LENGTH_MM,
    )


@app.route("/state")
def get_state():
    return jsonify({
        **state,
        "defaults": {
            "pen_up_s": DEFAULT_PEN_UP_S,
            "pen_down_s": DEFAULT_PEN_DOWN_S,
            "pen_up_dwell_ms": DEFAULT_PEN_UP_DWELL_MS,
            "pen_down_dwell_ms": DEFAULT_PEN_DOWN_DWELL_MS,
            "servo_ramp_enabled": DEFAULT_SERVO_RAMP_ENABLED,
            "servo_ramp_step": DEFAULT_SERVO_RAMP_STEP,
            "servo_ramp_delay_ms": DEFAULT_SERVO_RAMP_DELAY_MS,
        },
    })


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
        start_s = validate_servo_s(data.get("start_s", get_tracked_servo_s(DEFAULT_PEN_DOWN_S)))
        ramp_enabled = validate_bool(data.get("servo_ramp_enabled", DEFAULT_SERVO_RAMP_ENABLED))
        ramp_step = validate_non_negative_int(data.get("servo_ramp_step", DEFAULT_SERVO_RAMP_STEP), "Servo ramp step", minimum=1, maximum=200)
        ramp_delay_ms = validate_non_negative_float(data.get("servo_ramp_delay_ms", DEFAULT_SERVO_RAMP_DELAY_MS), "Servo ramp delay", maximum=1000)
        dwell_ms = validate_non_negative_float(data.get("pen_up_dwell_ms", DEFAULT_PEN_UP_DWELL_MS), "Pen up dwell", maximum=5000)
        commands = ["$X", *build_pen_position_commands(start_s, s_value, ramp_enabled=ramp_enabled, ramp_step=ramp_step, ramp_delay_ms=ramp_delay_ms, dwell_ms=dwell_ms)]
        response = send_many(commands, wait_idle_between=True)
        set_tracked_servo_s(s_value)
        return jsonify({"ok": True, "command": f"PEN UP M3 S{s_value}", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/pen-down", methods=["POST"])
def pen_down():
    data = request.get_json(force=True)
    try:
        s_value = validate_servo_s(data.get("s", DEFAULT_PEN_DOWN_S))
        start_s = validate_servo_s(data.get("start_s", get_tracked_servo_s(DEFAULT_PEN_UP_S)))
        ramp_enabled = validate_bool(data.get("servo_ramp_enabled", DEFAULT_SERVO_RAMP_ENABLED))
        ramp_step = validate_non_negative_int(data.get("servo_ramp_step", DEFAULT_SERVO_RAMP_STEP), "Servo ramp step", minimum=1, maximum=200)
        ramp_delay_ms = validate_non_negative_float(data.get("servo_ramp_delay_ms", DEFAULT_SERVO_RAMP_DELAY_MS), "Servo ramp delay", maximum=1000)
        dwell_ms = validate_non_negative_float(data.get("pen_down_dwell_ms", DEFAULT_PEN_DOWN_DWELL_MS), "Pen down dwell", maximum=5000)
        commands = ["$X", *build_pen_position_commands(start_s, s_value, ramp_enabled=ramp_enabled, ramp_step=ramp_step, ramp_delay_ms=ramp_delay_ms, dwell_ms=dwell_ms)]
        response = send_many(commands, wait_idle_between=True)
        set_tracked_servo_s(s_value)
        return jsonify({"ok": True, "command": f"PEN DOWN M3 S{s_value}", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/pen-test", methods=["POST"])
def pen_test():
    data = request.get_json(force=True)
    try:
        up_s = validate_servo_s(data.get("up_s", DEFAULT_PEN_UP_S))
        down_s = validate_servo_s(data.get("down_s", DEFAULT_PEN_DOWN_S))
        start_s = validate_servo_s(data.get("start_s", get_tracked_servo_s(up_s)))
        ramp_enabled = validate_bool(data.get("servo_ramp_enabled", DEFAULT_SERVO_RAMP_ENABLED))
        ramp_step = validate_non_negative_int(data.get("servo_ramp_step", DEFAULT_SERVO_RAMP_STEP), "Servo ramp step", minimum=1, maximum=200)
        ramp_delay_ms = validate_non_negative_float(data.get("servo_ramp_delay_ms", DEFAULT_SERVO_RAMP_DELAY_MS), "Servo ramp delay", maximum=1000)
        up_dwell_ms = validate_non_negative_float(data.get("pen_up_dwell_ms", DEFAULT_PEN_UP_DWELL_MS), "Pen up dwell", maximum=5000)
        down_dwell_ms = validate_non_negative_float(data.get("pen_down_dwell_ms", DEFAULT_PEN_DOWN_DWELL_MS), "Pen down dwell", maximum=5000)
        commands = ["$X"]
        commands.extend(build_pen_position_commands(start_s, down_s, ramp_enabled=ramp_enabled, ramp_step=ramp_step, ramp_delay_ms=ramp_delay_ms, dwell_ms=down_dwell_ms))
        commands.extend(build_pen_position_commands(down_s, up_s, ramp_enabled=ramp_enabled, ramp_step=ramp_step, ramp_delay_ms=ramp_delay_ms, dwell_ms=up_dwell_ms))
        response = send_many(commands, wait_idle_between=True)
        set_tracked_servo_s(up_s)
        return jsonify({"ok": True, "command": f"PEN TEST S{up_s}/S{down_s}", "response": response})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/servo-off", methods=["POST"])
def servo_off():
    try:
        response = send_many(["$X", "M5"], wait_idle_between=True)
        set_tracked_servo_s(DEFAULT_PEN_UP_S)
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


@app.route("/go-home", methods=["POST"])
def go_home():
    data = request.get_json(force=True)
    try:
        pen_up_s = validate_servo_s(data.get("pen_up_s", DEFAULT_PEN_UP_S))
        travel_feed = validate_feed(data.get("travel_feed", DEFAULT_TRAVEL_FEED))
        start_s = validate_servo_s(data.get("start_s", get_tracked_servo_s(DEFAULT_PEN_DOWN_S)))
        ramp_enabled = validate_bool(data.get("servo_ramp_enabled", DEFAULT_SERVO_RAMP_ENABLED))
        ramp_step = validate_non_negative_int(data.get("servo_ramp_step", DEFAULT_SERVO_RAMP_STEP), "Servo ramp step", minimum=1, maximum=200)
        ramp_delay_ms = validate_non_negative_float(data.get("servo_ramp_delay_ms", DEFAULT_SERVO_RAMP_DELAY_MS), "Servo ramp delay", maximum=1000)
        pen_up_dwell_ms = validate_non_negative_float(data.get("pen_up_dwell_ms", DEFAULT_PEN_UP_DWELL_MS), "Pen up dwell", maximum=5000)

        commands = ["$X"]
        commands.extend(build_pen_position_commands(
            start_s,
            pen_up_s,
            ramp_enabled=ramp_enabled,
            ramp_step=ramp_step,
            ramp_delay_ms=ramp_delay_ms,
            dwell_ms=pen_up_dwell_ms,
        ))
        commands.extend([
            "G21",
            "G90",
            f"G1 X0.0000 Y0.0000 F{travel_feed:.3f}",
        ])

        response = send_many(commands, wait_idle_between=True)
        set_tracked_servo_s(pen_up_s)
        state["status"] = "Returned to X0 Y0 with pen up"
        return jsonify({"ok": True, "command": "GO HOME X0 Y0 WITH PEN UP", "response": response})
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
        rotation_deg = float(request.form.get("rotation_deg", DEFAULT_ROTATION_DEG))
        parser_mode = request.form.get("parser_mode", DEFAULT_PARSER_MODE)
        color_mapping_mode = validate_bool(request.form.get("color_mapping_mode", DEFAULT_COLOR_MAPPING_MODE))
        line_thickness_mm = float(request.form.get("line_thickness_mm", DEFAULT_LINE_THICKNESS_MM))
        enable_fill = validate_bool(request.form.get("enable_fill", DEFAULT_ENABLE_FILL))
        trace_stroke_only_paths = validate_bool(request.form.get("trace_stroke_only_paths", DEFAULT_TRACE_STROKE_ONLY_PATHS))
        fill_only_dark_svg_fills = validate_bool(request.form.get("fill_only_dark_svg_fills", DEFAULT_FILL_ONLY_DARK_SVG_FILLS))
        fill_mode = request.form.get("fill_mode", DEFAULT_FILL_MODE)
        wall_count = validate_non_negative_int(request.form.get("wall_count", DEFAULT_WALL_COUNT), "Wall count", minimum=1, maximum=8)
        infill_pattern = request.form.get("infill_pattern", DEFAULT_INFILL_PATTERN)
        infill_spacing_mm = validate_non_negative_float(request.form.get("infill_spacing_mm", DEFAULT_INFILL_SPACING_MM), "Infill spacing", maximum=10)
        infill_angle_deg = float(request.form.get("infill_angle_deg", DEFAULT_INFILL_ANGLE_DEG))
        outline_after_fill = validate_bool(request.form.get("outline_after_fill", DEFAULT_OUTLINE_AFTER_FILL))
        min_fill_area_mm2 = validate_non_negative_float(request.form.get("min_fill_area_mm2", DEFAULT_MIN_FILL_AREA_MM2), "Minimum fill area", maximum=10000)
        min_fill_width_mm = validate_non_negative_float(request.form.get("min_fill_width_mm", DEFAULT_MIN_FILL_WIDTH_MM), "Minimum fill width", maximum=10)
        simplify_tolerance_mm = validate_non_negative_float(request.form.get("simplify_tolerance_mm", DEFAULT_SIMPLIFY_TOLERANCE_MM), "Simplify tolerance", maximum=5)
        remove_duplicate_paths = validate_bool(request.form.get("remove_duplicate_paths", DEFAULT_REMOVE_DUPLICATE_PATHS))
        small_shape_mode = request.form.get("small_shape_mode", DEFAULT_SMALL_SHAPE_MODE)
        min_segment_length_mm = validate_non_negative_float(request.form.get("min_segment_length_mm", DEFAULT_MIN_SEGMENT_LENGTH_MM), "Minimum segment length", maximum=20)
        fit_mode = request.form.get("fit_mode", "contain")
        invert_y = request.form.get("invert_y", "1") == "1"
        include_comments = request.form.get("include_comments", "1") == "1"
        pen_up_s = validate_servo_s(request.form.get("pen_up_s", DEFAULT_PEN_UP_S))
        pen_down_s = validate_servo_s(request.form.get("pen_down_s", DEFAULT_PEN_DOWN_S))
        servo_ramp_enabled = validate_bool(request.form.get("servo_ramp_enabled", DEFAULT_SERVO_RAMP_ENABLED))
        servo_ramp_step = validate_non_negative_int(request.form.get("servo_ramp_step", DEFAULT_SERVO_RAMP_STEP), "Servo ramp step", minimum=1, maximum=200)
        servo_ramp_delay_ms = validate_non_negative_float(request.form.get("servo_ramp_delay_ms", DEFAULT_SERVO_RAMP_DELAY_MS), "Servo ramp delay", maximum=1000)
        pen_up_dwell_ms = validate_non_negative_float(request.form.get("pen_up_dwell_ms", DEFAULT_PEN_UP_DWELL_MS), "Pen up dwell", maximum=5000)
        pen_down_dwell_ms = validate_non_negative_float(request.form.get("pen_down_dwell_ms", DEFAULT_PEN_DOWN_DWELL_MS), "Pen down dwell", maximum=5000)
        debug_pipeline = validate_bool(request.form.get("debug_pipeline", "0"))

        if sample_step_deg <= 0:
            raise ValueError("Sample step must be greater than 0")
        if margin_percent < 0 or margin_percent > 25:
            raise ValueError("Margin percent must be between 0 and 25")
        if line_thickness_mm < 0 or line_thickness_mm > 10:
            raise ValueError("Line thickness must be between 0 and 10 mm")
        if fit_mode not in ["contain", "stretch"]:
            raise ValueError("Invalid fit mode")
        if parser_mode not in {"visible_geometry", "detect_visible_print_areas"}:
            raise ValueError("Invalid parser mode")
        if fill_mode != "slicer":
            raise ValueError("Only slicer fill mode is currently supported")
        if infill_pattern not in {"zigzag", "hatch"}:
            raise ValueError("Infill pattern must be zigzag or hatch")
        if small_shape_mode not in {"single-wall", "skip", "centerline-todo"}:
            raise ValueError("Invalid small shape mode")

        debug_data: Optional[dict[str, Any]] = {} if debug_pipeline else None
        bundle, viewbox_bounds, print_model = extract_svg_bundle(
            svg_text,
            debug=debug_data,
            parser_mode=parser_mode,
            color_mapping_mode=color_mapping_mode,
            trace_stroke_only_paths=trace_stroke_only_paths,
            fill_only_dark_svg_fills=fill_only_dark_svg_fills,
        )
        if not bundle.outline_segments and not bundle.fill_boundary_segments and not bundle.fill_shapes:
            raise ValueError("; ".join(print_model.diagnostics or ["Visible SVG content could not be normalized into drawable geometry."]))

        bounds = viewbox_bounds or bounds_from_bundle(bundle)
        mapped = map_bundle_to_angles(bundle, bounds, fit_mode, invert_y, margin_percent)
        debug_append_bundle(debug_data, "mapped_paths", mapped)
        placed = apply_placement_transform(mapped, placement_scale, rotation_deg, placement_offset_x, placement_offset_y)
        debug_append_bundle(debug_data, "placed_paths", placed)
        toolpaths = generate_toolpaths(
            placed,
            enable_fill=enable_fill,
            line_width_mm=line_thickness_mm,
            wall_count=wall_count,
            infill_spacing_mm=infill_spacing_mm if infill_spacing_mm > 0 else line_thickness_mm,
            infill_angle_deg=infill_angle_deg,
            outline_after_fill=outline_after_fill,
            min_fill_area_mm2=min_fill_area_mm2,
            min_fill_width_mm=min_fill_width_mm,
            simplify_tolerance_mm=simplify_tolerance_mm,
            remove_duplicate_paths=remove_duplicate_paths,
            small_shape_mode=small_shape_mode,
            min_segment_length_mm=min_segment_length_mm,
            debug=debug_data,
        )
        if not toolpaths:
            raise ValueError("No toolpaths were generated from the current SVG/settings")

        gcode, preview = generate_gcode_from_toolpaths(
            toolpaths,
            draw_feed=draw_feed,
            travel_feed=travel_feed,
            sample_step_deg=sample_step_deg,
            pen_up_s=pen_up_s,
            pen_down_s=pen_down_s,
            servo_ramp_enabled=servo_ramp_enabled,
            servo_ramp_step=servo_ramp_step,
            servo_ramp_delay_ms=servo_ramp_delay_ms,
            pen_up_dwell_ms=pen_up_dwell_ms,
            pen_down_dwell_ms=pen_down_dwell_ms,
            include_comments=include_comments,
        )
        if debug_data is not None:
            debug_data["gcode_preview"] = preview

        point_count = sum(len(path["points"]) for path in preview if path["kind"] != "travel")
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
            "toolpath_count": len(toolpaths),
            "point_count": point_count,
            "bounds": asdict(bounds),
            "viewbox_bounds": asdict(viewbox_bounds) if viewbox_bounds else None,
            "print_model": asdict(print_model),
            "debug": debug_data,
        })
    except Exception as e:
        state["last_error"] = str(e)
        state["status"] = f"Generate error: {e}"
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/analyze-svg", methods=["POST"])
def analyze_svg_route():
    try:
        if "svg" not in request.files:
            raise ValueError("No SVG file uploaded")

        file = request.files["svg"]
        svg_text = file.read().decode("utf-8", errors="ignore")
        parser_mode = request.form.get("parser_mode", DEFAULT_PARSER_MODE)
        color_mapping_mode = validate_bool(request.form.get("color_mapping_mode", DEFAULT_COLOR_MAPPING_MODE))
        trace_stroke_only_paths = validate_bool(request.form.get("trace_stroke_only_paths", DEFAULT_TRACE_STROKE_ONLY_PATHS))
        fill_only_dark_svg_fills = validate_bool(request.form.get("fill_only_dark_svg_fills", DEFAULT_FILL_ONLY_DARK_SVG_FILLS))
        debug_pipeline = validate_bool(request.form.get("debug_pipeline", "0"))

        if parser_mode not in {"visible_geometry", "detect_visible_print_areas"}:
            raise ValueError("Invalid parser mode")

        debug_data: Optional[dict[str, Any]] = {} if debug_pipeline else None
        result = analyze_svg(
            svg_text,
            parser_mode=parser_mode,
            color_mapping_mode=color_mapping_mode,
            trace_stroke_only_paths=trace_stroke_only_paths,
            fill_only_dark_svg_fills=fill_only_dark_svg_fills,
            debug=debug_data,
        )

        return jsonify({
            "ok": True,
            "print_model": asdict(result.print_model),
            "viewbox_bounds": asdict(result.viewbox_bounds) if result.viewbox_bounds else None,
            "debug": debug_data,
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/self-test-svg-pipeline", methods=["POST"])
def self_test_svg_pipeline_route():
    try:
        summary = run_integrated_svg_pipeline_self_test()
        return jsonify({"ok": True, "summary": summary})
    except Exception as e:
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
    if "--self-test" in sys.argv:
        summary = run_integrated_svg_pipeline_self_test()
        print(f"SVG pipeline self-test passed: {summary['passed']} checks")
        for message in summary["messages"]:
            print(f" - {message}")
        raise SystemExit(0)
    print("Starting SVG -> G-code Golf Ball Plotter Controller...")
    print(f"Serial port: {SERIAL_PORT}")
    print(f"X microsteps: {X_MICROSTEPS}")
    print(f"Y microsteps: {Y_MICROSTEPS}")
    print(f"X steps/degree: {X_STEPS_PER_DEGREE:.6f}")
    print(f"Y steps/degree: {Y_STEPS_PER_DEGREE:.6f}")
    print("Install dependencies if needed:")
    print("  pip install flask pyserial svgpathtools shapely")
    print("Open:")
    print("  http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=False)
