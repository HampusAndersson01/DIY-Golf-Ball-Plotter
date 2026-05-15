from __future__ import annotations

# Internal core module extracted from the previous single-file app.
# It intentionally contains the parser, geometry, serial, toolpath, G-code, and self-test logic
# used by the refactored package, without the old Flask UI/routes layer.

RUNTIME_KEYS = [
    "SERIAL_PORT",
    "BAUD_RATE",
    "MOTOR_FULL_STEPS_PER_REV",
    "X_MICROSTEPS",
    "Y_MICROSTEPS",
    "X_DRAW_MIN",
    "X_DRAW_MAX",
    "Y_DRAW_MIN",
    "Y_DRAW_MAX",
    "BALL_CENTER_X",
    "BALL_CENTER_Y",
    "BALL_DIAMETER_MM",
    "DEFAULT_X_MAX_FEED",
    "DEFAULT_Y_MAX_FEED",
    "DEFAULT_X_ACCELERATION",
    "DEFAULT_Y_ACCELERATION",
    "DEFAULT_DRAW_FEED",
    "DEFAULT_TRAVEL_FEED",
    "DEFAULT_LINE_THICKNESS_MM",
    "DEFAULT_PEN_UP_S",
    "DEFAULT_PEN_DOWN_S",
    "DEFAULT_SERVO_DWELL",
    "DEFAULT_SERVO_RAMP_ENABLED",
    "DEFAULT_SERVO_RAMP_STEP",
    "DEFAULT_SERVO_RAMP_DELAY_MS",
    "DEFAULT_PEN_UP_DWELL_MS",
    "DEFAULT_PEN_DOWN_DWELL_MS",
    "DEFAULT_GCODE_MODE",
    "MIN_SERVO_S",
    "MAX_SERVO_S",
    "DEFAULT_SAMPLE_STEP_DEG",
    "DEFAULT_CURVE_SAMPLES",
    "DEFAULT_MARGIN_PERCENT",
    "DEFAULT_ROTATION_DEG",
    "DEFAULT_ENABLE_FILL",
    "DEFAULT_FILL_MODE",
    "DEFAULT_PARSER_MODE",
    "DEFAULT_COLOR_MAPPING_MODE",
    "DEFAULT_TRACE_STROKE_ONLY_PATHS",
    "DEFAULT_FILL_ONLY_DARK_SVG_FILLS",
    "DEFAULT_WALL_COUNT",
    "DEFAULT_INFILL_PATTERN",
    "DEFAULT_INFILL_DENSITY",
    "DEFAULT_INFILL_SPACING_MM",
    "DEFAULT_INFILL_ANGLE_DEG",
    "DEFAULT_OUTLINE_AFTER_FILL",
    "DEFAULT_MIN_FILL_AREA_MM2",
    "DEFAULT_MIN_FILL_WIDTH_MM",
    "DEFAULT_SIMPLIFY_TOLERANCE_MM",
    "DEFAULT_REMOVE_DUPLICATE_PATHS",
    "DEFAULT_SMALL_SHAPE_MODE",
    "DEFAULT_THIN_DETAIL_MODE",
    "DEFAULT_THIN_DETAIL_MIN_AREA_MM2",
    "DEFAULT_THIN_DETAIL_SIMPLIFY_MM",
    "DEFAULT_THIN_DETAIL_OVERLAP",
    "DEFAULT_MIN_SEGMENT_LENGTH_MM",
    "DEFAULT_TRAVEL_OPTIMIZATION",
    "DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS",
    "SVG_DARK_FILL_LUMINANCE_THRESHOLD",
    "SVG_LIGHT_CUTOUT_LUMINANCE_THRESHOLD",
    "SVG_MIN_PRINT_OPACITY",
]


def configure_runtime(config, state_dict, serial_lock_obj):
    global serial_lock, state, X_STEPS_PER_DEGREE, Y_STEPS_PER_DEGREE
    for key in RUNTIME_KEYS:
        if key in config:
            globals()[key] = config[key]
    X_STEPS_PER_DEGREE = (config["MOTOR_FULL_STEPS_PER_REV"] * config["X_MICROSTEPS"]) / 360.0
    Y_STEPS_PER_DEGREE = (config["MOTOR_FULL_STEPS_PER_REV"] * config["Y_MICROSTEPS"]) / 360.0
    state = state_dict
    serial_lock = serial_lock_obj


from collections import deque
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
from typing import Optional, Any, Callable

# Optional dependency for real SVG path support:
#   pip install svgpathtools pyserial flask
try:
    from svgpathtools import parse_path
except Exception:
    parse_path = None

try:
    from shapely import affinity
    from shapely.geometry import GeometryCollection, LineString, MultiLineString, MultiPolygon, Point as ShapelyPoint, Polygon
    from shapely.ops import polygonize, substring, unary_union
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
    substring = None
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
DEFAULT_GCODE_MODE = "simple"
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
DEFAULT_INFILL_DENSITY = 100.0
DEFAULT_INFILL_SPACING_MM = DEFAULT_LINE_THICKNESS_MM
DEFAULT_INFILL_ANGLE_DEG = 0.0
DEFAULT_OUTLINE_AFTER_FILL = True
DEFAULT_MIN_FILL_AREA_MM2 = 1.0
DEFAULT_MIN_FILL_WIDTH_MM = DEFAULT_LINE_THICKNESS_MM
DEFAULT_SIMPLIFY_TOLERANCE_MM = 0.05
DEFAULT_REMOVE_DUPLICATE_PATHS = True
DEFAULT_SMALL_SHAPE_MODE = "single-wall"
DEFAULT_THIN_DETAIL_MODE = True
DEFAULT_THIN_DETAIL_MIN_AREA_MM2 = 0.05
DEFAULT_THIN_DETAIL_SIMPLIFY_MM = 0.1
DEFAULT_THIN_DETAIL_OVERLAP = True
DEFAULT_MIN_SEGMENT_LENGTH_MM = 0.5
DEFAULT_TRAVEL_OPTIMIZATION = "nearest-neighbor"
DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS = True
SVG_DARK_FILL_LUMINANCE_THRESHOLD = 0.42
SVG_LIGHT_CUTOUT_LUMINANCE_THRESHOLD = 0.82
SVG_MIN_PRINT_OPACITY = 0.99

app = Flask(__name__)

serial_lock = threading.Lock()
grbl: Optional[serial.Serial] = None
GRBL_RX_BUFFER_SIZE = 128

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
    detail_segments: list[Segment] = field(default_factory=list)
    fill_shapes: list[SvgFillShape] = field(default_factory=list)
    printable_geometry: Any = None
    cutout_geometry: Any = None


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
class SlicerSettings:
    line_width_mm: float
    wall_count: int
    infill_density: float = 100.0
    infill_spacing_mm: float = DEFAULT_INFILL_SPACING_MM
    infill_angle_deg: float = 0.0
    outline_after_fill: bool = False
    min_fill_area_mm2: float = 1.0
    min_fill_width_mm: float = DEFAULT_MIN_FILL_WIDTH_MM
    simplify_tolerance_mm: float = DEFAULT_SIMPLIFY_TOLERANCE_MM
    remove_duplicate_paths: bool = True
    small_shape_mode: str = DEFAULT_SMALL_SHAPE_MODE
    thin_detail_mode: bool = DEFAULT_THIN_DETAIL_MODE
    thin_detail_min_area_mm2: float = DEFAULT_THIN_DETAIL_MIN_AREA_MM2
    thin_detail_simplify_mm: float = DEFAULT_THIN_DETAIL_SIMPLIFY_MM
    thin_detail_overlap: bool = DEFAULT_THIN_DETAIL_OVERLAP
    min_segment_length_mm: float = DEFAULT_MIN_SEGMENT_LENGTH_MM
    travel_optimization: str = DEFAULT_TRAVEL_OPTIMIZATION
    allow_pen_down_infill_connectors: bool = DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS


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


def read_next_grbl_line(ser: serial.Serial, timeout: float = 15) -> str:
    end_time = time.time() + timeout

    while time.time() < end_time:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            return line

    raise TimeoutError("Timed out waiting for GRBL response")


def process_streaming_ack_unlocked(
    ser: serial.Serial,
    pending_lengths: deque[int],
    timeout: float = 15,
) -> int:
    while True:
        line = read_next_grbl_line(ser, timeout=timeout)
        if line == "ok":
            if not pending_lengths:
                raise RuntimeError("Received unexpected GRBL ok with no pending commands")
            return pending_lengths.popleft()
        if line.startswith("error:") or line.startswith("ALARM:"):
            raise RuntimeError(f"GRBL streaming error: {line}")
        if line.startswith("<") or line.startswith("["):
            continue
        if line.lower().startswith("grbl"):
            continue


def stream_gcode_lines_unlocked(
    ser: serial.Serial,
    lines: list[str],
    *,
    rx_buffer_size: int = GRBL_RX_BUFFER_SIZE,
    response_timeout: float = 20,
    should_stop: Optional[Callable[[], bool]] = None,
    wait_while_paused: Optional[Callable[[], None]] = None,
    on_line_sent: Optional[Callable[[str, int], None]] = None,
) -> int:
    pending_lengths: deque[int] = deque()
    pending_bytes = 0
    sent_count = 0

    for raw_line in lines:
        if should_stop and should_stop():
            return sent_count
        if wait_while_paused:
            wait_while_paused()
        if should_stop and should_stop():
            return sent_count

        line = raw_line.strip()
        if not line:
            continue

        payload = (line + "\n").encode("ascii")
        if len(payload) >= rx_buffer_size:
            raise ValueError(f"G-code line exceeds GRBL RX buffer size: {line}")

        while pending_bytes + len(payload) >= rx_buffer_size:
            pending_bytes -= process_streaming_ack_unlocked(ser, pending_lengths, timeout=response_timeout)
            if should_stop and should_stop():
                return sent_count
            if wait_while_paused:
                wait_while_paused()

        ser.write(payload)
        pending_lengths.append(len(payload))
        pending_bytes += len(payload)
        sent_count += 1
        if on_line_sent:
            on_line_sent(line, sent_count)

    while pending_lengths:
        pending_bytes -= process_streaming_ack_unlocked(ser, pending_lengths, timeout=response_timeout)

    return sent_count


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
    circumference_mm = math.pi * BALL_DIAMETER_MM
    return (mm / circumference_mm) * 360.0


def ball_radius_mm(ball_diameter_mm: float = BALL_DIAMETER_MM) -> float:
    return ball_diameter_mm / 2.0


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


def summarize_toolpaths(toolpaths: list["Toolpath"]) -> dict[str, Any]:
    paths_by_kind: dict[str, int] = {}
    points_by_kind: dict[str, int] = {}
    one_move_toolpaths = 0
    total_points = 0
    for toolpath in toolpaths:
        kind = toolpath.kind
        point_count = len(toolpath.points)
        total_points += point_count
        paths_by_kind[kind] = paths_by_kind.get(kind, 0) + 1
        points_by_kind[kind] = points_by_kind.get(kind, 0) + point_count
        if point_count == 2:
            one_move_toolpaths += 1

    return {
        "total_toolpaths": len(toolpaths),
        "one_move_toolpaths": one_move_toolpaths,
        "average_points_per_toolpath": (total_points / len(toolpaths)) if toolpaths else 0.0,
        "paths_by_kind": paths_by_kind,
        "points_by_kind": points_by_kind,
    }


def debug_append_geometry(debug: Optional[dict[str, Any]], key: str, geometry: Any, kind: str) -> None:
    if debug is None or geometry is None or geometry.is_empty:
        return
    entries = debug.setdefault(key, [])
    for polygon in normalize_geometry(geometry):
        entries.append({
            "kind": kind,
            "closed": True,
            "points": [asdict(Point(x, y)) for x, y in polygon.exterior.coords],
        })
        for interior in polygon.interiors:
            entries.append({
                "kind": f"{kind}-hole",
                "closed": True,
                "points": [asdict(Point(x, y)) for x, y in interior.coords],
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
    bundle.printable_geometry = composed_fill
    bundle.cutout_geometry = unary_union(cutout_geometries) if cutout_geometries else None
    if composed_fill is not None and not composed_fill.is_empty and cutout_geometries:
        cutout_union = bundle.cutout_geometry
        if cutout_union is not None and not cutout_union.is_empty:
            composed_fill = composed_fill.difference(cutout_union)
    if composed_fill is not None and not composed_fill.is_empty and not composed_fill.is_valid:
        composed_fill = make_valid(composed_fill) if make_valid is not None else composed_fill.buffer(0)
    if composed_fill is not None and not composed_fill.is_empty:
        bundle.fill_shapes.append(SvgFillShape(geometry=composed_fill, fill_rule="evenodd", source_tag="composited-visible-fill"))
        bundle.fill_boundary_segments.extend(geometry_to_boundary_segments(composed_fill))
    bundle.printable_geometry = composed_fill

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
    debug_append_geometry(debug, "printable_polygons", unary_union([item.fill_geometry for item in classified_elements if item.has_fill and item.fill_geometry is not None]) if any(item.has_fill and item.fill_geometry is not None for item in classified_elements) else None, "printable-polygon")
    debug_append_geometry(debug, "cutout_polygons", unary_union([item.fill_geometry for item in classified_elements if item.is_cutout_fill and item.fill_geometry is not None]) if any(item.is_cutout_fill and item.fill_geometry is not None for item in classified_elements) else None, "cutout-polygon")
    debug_append_geometry(debug, "composed_fill_region", bundle.printable_geometry, "composed-fill-region")
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

    all_segments = bundle.outline_segments + bundle.fill_boundary_segments + bundle.detail_segments
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


def map_bundle_to_surface_mm(
    bundle: GeometryBundle,
    bounds: SvgBounds,
    fit_mode: str,
    invert_y: bool,
    margin_percent: float,
) -> GeometryBundle:
    radius_mm = ball_radius_mm()
    full_width_mm = radius_mm * math.radians(X_DRAW_MAX - X_DRAW_MIN)
    full_height_mm = radius_mm * math.radians(Y_DRAW_MAX - Y_DRAW_MIN)
    margin_x = full_width_mm * (margin_percent / 100.0)
    margin_y = full_height_mm * (margin_percent / 100.0)

    target_w = full_width_mm - margin_x * 2
    target_h = full_height_mm - margin_y * 2
    if target_w <= 0 or target_h <= 0:
        raise ValueError("Margin is too large")

    target_min_x = -(target_w / 2.0)
    target_min_y = -(target_h / 2.0)

    if fit_mode == "stretch":
        scale_x = target_w / bounds.width
        scale_y = target_h / bounds.height
        base_x = target_min_x - (bounds.min_x * scale_x)
        if invert_y:
            base_y = target_min_y + (bounds.max_y * scale_y)
            matrix = [scale_x, 0.0, 0.0, -scale_y, base_x, base_y]
        else:
            base_y = target_min_y - (bounds.min_y * scale_y)
            matrix = [scale_x, 0.0, 0.0, scale_y, base_x, base_y]
    else:
        scale = min(target_w / bounds.width, target_h / bounds.height)
        used_w = bounds.width * scale
        used_h = bounds.height * scale
        offset_x = target_min_x + (target_w - used_w) / 2.0
        offset_y = target_min_y + (target_h - used_h) / 2.0
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
    detail_segments = [
        Segment([Point((matrix[0] * p.x) + matrix[4], (matrix[3] * p.y) + matrix[5]) for p in seg.points], closed=seg.closed)
        for seg in bundle.detail_segments
    ]
    fill_shapes = [
        SvgFillShape(
            geometry=affinity.affine_transform(fill_shape.geometry, matrix),
            fill_rule=fill_shape.fill_rule,
            source_tag=fill_shape.source_tag,
        )
        for fill_shape in bundle.fill_shapes
    ]
    printable_geometry = affinity.affine_transform(bundle.printable_geometry, matrix) if bundle.printable_geometry is not None and not bundle.printable_geometry.is_empty else bundle.printable_geometry
    cutout_geometry = affinity.affine_transform(bundle.cutout_geometry, matrix) if bundle.cutout_geometry is not None and not bundle.cutout_geometry.is_empty else bundle.cutout_geometry
    return GeometryBundle(
        outline_segments=outline_segments,
        fill_boundary_segments=fill_boundary_segments,
        detail_segments=detail_segments,
        fill_shapes=fill_shapes,
        printable_geometry=printable_geometry,
        cutout_geometry=cutout_geometry,
    )


def apply_surface_placement_transform(
    bundle: GeometryBundle,
    scale_percent: float,
    rotation_deg: float,
) -> GeometryBundle:
    if scale_percent <= 0:
        raise ValueError("Placement scale must be greater than 0")

    if not bundle.outline_segments and not bundle.fill_boundary_segments and not bundle.detail_segments and not bundle.fill_shapes:
        return GeometryBundle()

    scale = scale_percent / 100.0
    angle = math.radians(rotation_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    def place_point(point: Point) -> Point:
        scaled_x = point.x * scale
        scaled_y = point.y * scale
        return Point(
            (scaled_x * cos_a) - (scaled_y * sin_a),
            (scaled_x * sin_a) + (scaled_y * cos_a),
        )

    outline_segments = [
        Segment([place_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.outline_segments
    ]
    fill_boundary_segments = [
        Segment([place_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.fill_boundary_segments
    ]
    detail_segments = [
        Segment([place_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.detail_segments
    ]

    fill_shapes = []
    for fill_shape in bundle.fill_shapes:
        geometry = affinity.scale(fill_shape.geometry, xfact=scale, yfact=scale, origin=(0.0, 0.0))
        geometry = affinity.rotate(geometry, rotation_deg, origin=(0.0, 0.0))
        fill_shapes.append(SvgFillShape(geometry=geometry, fill_rule=fill_shape.fill_rule, source_tag=fill_shape.source_tag))
    printable_geometry = bundle.printable_geometry
    if printable_geometry is not None and not printable_geometry.is_empty:
        printable_geometry = affinity.scale(printable_geometry, xfact=scale, yfact=scale, origin=(0.0, 0.0))
        printable_geometry = affinity.rotate(printable_geometry, rotation_deg, origin=(0.0, 0.0))
    cutout_geometry = bundle.cutout_geometry
    if cutout_geometry is not None and not cutout_geometry.is_empty:
        cutout_geometry = affinity.scale(cutout_geometry, xfact=scale, yfact=scale, origin=(0.0, 0.0))
        cutout_geometry = affinity.rotate(cutout_geometry, rotation_deg, origin=(0.0, 0.0))

    return GeometryBundle(
        outline_segments=outline_segments,
        fill_boundary_segments=fill_boundary_segments,
        detail_segments=detail_segments,
        fill_shapes=fill_shapes,
        printable_geometry=printable_geometry,
        cutout_geometry=cutout_geometry,
    )


def segment_length(points: list[Point]) -> float:
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))


def mm_area_to_ball_degree_area(area_mm2: float) -> float:
    scale = 360.0 / (math.pi * BALL_DIAMETER_MM)
    return area_mm2 * scale * scale


def surface_mm_to_ball_angles(
    point: Point,
    *,
    center_lon_deg: float,
    center_lat_deg: float,
    ball_diameter_mm: float = BALL_DIAMETER_MM,
    min_cos_lat: float = 0.1,
) -> Point:
    radius = ball_radius_mm(ball_diameter_mm)
    center_lon = math.radians(center_lon_deg)
    center_lat = math.radians(center_lat_deg)
    lat = center_lat + (point.y / radius)
    cos_lat = math.cos(lat)
    if abs(cos_lat) < min_cos_lat:
        raise ValueError("Toolpath approaches the ball pole too closely for stable longitude mapping")
    lon = center_lon + (point.x / (radius * cos_lat))
    return Point(math.degrees(lon), math.degrees(lat))


def project_toolpaths_to_ball_angles(
    toolpaths: list[Toolpath],
    *,
    center_lon_deg: float,
    center_lat_deg: float,
    ball_diameter_mm: float = BALL_DIAMETER_MM,
    min_cos_lat: float = 0.1,
) -> list[Toolpath]:
    projected: list[Toolpath] = []
    for toolpath in toolpaths:
        points = [
            surface_mm_to_ball_angles(
                point,
                center_lon_deg=center_lon_deg,
                center_lat_deg=center_lat_deg,
                ball_diameter_mm=ball_diameter_mm,
                min_cos_lat=min_cos_lat,
            )
            for point in toolpath.points
        ]
        projected.append(Toolpath(points=points, kind=toolpath.kind, closed=toolpath.closed))
    return projected


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
    debug_append_segments(debug, key, bundle.detail_segments, f"{key}-detail")
    debug_append_geometry(debug, f"{key}_printable_geometry", bundle.printable_geometry, f"{key}-printable-geometry")
    debug_append_geometry(debug, f"{key}_cutout_geometry", bundle.cutout_geometry, f"{key}-cutout-geometry")


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


def _line_coords(geometry: Any) -> list[tuple[float, float]]:
    if geometry is None or geometry.is_empty:
        return []
    if isinstance(geometry, LineString):
        return list(geometry.coords)
    if isinstance(geometry, MultiLineString):
        coords: list[tuple[float, float]] = []
        for line in geometry.geoms:
            line_coords = list(line.coords)
            if not line_coords:
                continue
            if coords and coords[-1] == line_coords[0]:
                coords.extend(line_coords[1:])
            else:
                coords.extend(line_coords)
        return coords
    return []


def _concat_coords(*parts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    merged: list[tuple[float, float]] = []
    for part in parts:
        if not part:
            continue
        if merged and merged[-1] == part[0]:
            merged.extend(part[1:])
        else:
            merged.extend(part)
    return merged


def boundary_connector_coords(
    polygon: Polygon,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    tolerance: float = 1e-6,
) -> list[tuple[float, float]]:
    if substring is None:
        return []

    candidate_rings = [LineString(polygon.exterior.coords)]
    candidate_rings.extend(LineString(interior.coords) for interior in polygon.interiors)

    start_pt = ShapelyPoint(start)
    end_pt = ShapelyPoint(end)
    best_coords: list[tuple[float, float]] = []
    best_length = float("inf")

    for ring in candidate_rings:
        if ring.distance(start_pt) > tolerance or ring.distance(end_pt) > tolerance:
            continue

        ring_length = ring.length
        if ring_length <= tolerance:
            continue

        start_d = ring.project(start_pt)
        end_d = ring.project(end_pt)

        forward = _line_coords(substring(ring, start_d, end_d))
        if start_d <= end_d:
            wrap = _concat_coords(
                _line_coords(substring(ring, end_d, ring_length)),
                _line_coords(substring(ring, 0.0, start_d)),
            )
        else:
            wrap = _concat_coords(
                _line_coords(substring(ring, start_d, ring_length)),
                _line_coords(substring(ring, 0.0, end_d)),
            )

        backward = list(reversed(wrap))
        for coords in (forward, backward):
            if len(coords) < 2:
                continue
            length = LineString(coords).length
            if length < best_length:
                best_length = length
                best_coords = coords

    return best_coords


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


def filter_toolpaths_by_length(paths: list[Toolpath], minimum_length: float) -> list[Toolpath]:
    filtered: list[Toolpath] = []
    relaxed_minimum = max(0.0, minimum_length * 0.25)
    for path in paths:
        if len(path.points) < 2:
            continue
        threshold = relaxed_minimum if path.kind == "detail-trace" else minimum_length
        if segment_length(path.points) < threshold:
            continue
        filtered.append(path)
    return filtered


def rotate_closed_toolpath(path: Toolpath, anchor: Point) -> Toolpath:
    if not path.closed or len(path.points) <= 2:
        return path
    core = path.points[:-1]
    if len(core) < 2:
        return path
    best_index = min(range(len(core)), key=lambda i: math.hypot(core[i].x - anchor.x, core[i].y - anchor.y))
    rotated = core[best_index:] + core[:best_index] + [core[best_index]]
    return Toolpath(points=rotated, kind=path.kind, closed=True)


def optimize_toolpath_order(
    toolpaths: list[Toolpath],
    *,
    strategy: str,
    start_point: Optional[Point] = None,
) -> list[Toolpath]:
    if strategy != "nearest-neighbor" or len(toolpaths) <= 1:
        return toolpaths

    pending = list(toolpaths)
    ordered: list[Toolpath] = []
    current = start_point or Point(0.0, 0.0)
    while pending:
        best_index = 0
        best_path = pending[0]
        best_score = float("inf")
        best_reversed = False
        best_rotated = best_path

        for index, path in enumerate(pending):
            candidate = rotate_closed_toolpath(path, current)
            if candidate.points:
                start_distance = math.hypot(candidate.points[0].x - current.x, candidate.points[0].y - current.y)
                if start_distance < best_score:
                    best_index = index
                    best_path = path
                    best_score = start_distance
                    best_reversed = False
                    best_rotated = candidate
            if not path.closed and len(path.points) >= 2:
                reversed_points = list(reversed(path.points))
                reverse_distance = math.hypot(reversed_points[0].x - current.x, reversed_points[0].y - current.y)
                if reverse_distance < best_score:
                    best_index = index
                    best_path = path
                    best_score = reverse_distance
                    best_reversed = True
                    best_rotated = Toolpath(points=reversed_points, kind=path.kind, closed=False)

        pending.pop(best_index)
        selected = best_rotated
        if best_reversed and best_path.closed:
            selected = best_path
        ordered.append(selected)
        if selected.points:
            current = selected.points[-1]
    return ordered


def merge_connected_toolpaths(
    toolpaths: list[Toolpath],
    *,
    tolerance: float = 1e-6,
) -> list[Toolpath]:
    if len(toolpaths) <= 1:
        return toolpaths

    merged: list[Toolpath] = []
    current = toolpaths[0]
    for candidate in toolpaths[1:]:
        can_merge = (
            current.kind == candidate.kind
            and not current.closed
            and not candidate.closed
            and len(current.points) >= 2
            and len(candidate.points) >= 2
        )
        if can_merge and nearly_same_point(current.points[-1], candidate.points[0], tolerance):
            current = Toolpath(
                points=current.points + candidate.points[1:],
                kind=current.kind,
                closed=False,
            )
            continue
        if can_merge and nearly_same_point(current.points[-1], candidate.points[-1], tolerance):
            current = Toolpath(
                points=current.points + list(reversed(candidate.points[:-1])),
                kind=current.kind,
                closed=False,
            )
            continue
        merged.append(current)
        current = candidate

    merged.append(current)
    return merged


class SlicerService:
    def _scanline_spacing_deg(self, settings: SlicerSettings) -> float:
        base_spacing_mm = settings.infill_spacing_mm if settings.infill_spacing_mm > 0 else settings.line_width_mm
        density_scale = max(0.01, settings.infill_density / 100.0)
        return max(mm_to_ball_degrees(settings.line_width_mm) * 0.25, mm_to_ball_degrees(base_spacing_mm) / density_scale)

    def _emit_debug_connector(
        self,
        debug: Optional[dict[str, Any]],
        key: str,
        start: tuple[float, float],
        end: tuple[float, float],
        angle_deg: float,
        origin: tuple[float, float],
        kind: str,
    ) -> None:
        if debug is None or nearly_same_point(Point(*start), Point(*end)):
            return
        world_line = affinity.rotate(LineString([start, end]), angle_deg, origin=origin)
        debug_append_toolpaths(
            debug,
            key,
            [Toolpath(points=[Point(x, y) for x, y in world_line.coords], kind=kind, closed=False)],
        )

    def _generate_scanline_infill(
        self,
        region: Any,
        *,
        spacing_deg: float,
        angle_deg: float,
        min_segment_length_deg: float,
        tolerance_deg: float,
        kind: str = "fill-infill",
        allow_pen_down_infill_connectors: bool = DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if region is None or region.is_empty or spacing_deg <= 0:
            return []

        origin = region.centroid.coords[0]
        rotated = affinity.rotate(region, -angle_deg, origin=origin)
        min_x, min_y, max_x, max_y = rotated.bounds
        if not all(math.isfinite(value) for value in [min_x, min_y, max_x, max_y]):
            return []

        toolpaths: list[Toolpath] = []
        epsilon = max(tolerance_deg, 1e-6)
        for polygon in normalize_geometry(rotated):
            cover_region = polygon.buffer(epsilon, join_style=1)
            poly_min_x, poly_min_y, poly_max_x, poly_max_y = polygon.bounds
            rows: list[list[list[tuple[float, float]]]] = []
            row = 0
            y = poly_min_y
            while y <= poly_max_y + 1e-6:
                raw_scan = LineString([(poly_min_x - spacing_deg, y), (poly_max_x + spacing_deg, y)])
                if debug is not None:
                    raw_scan_world = affinity.rotate(raw_scan, angle_deg, origin=origin)
                    debug_append_toolpaths(debug, "raw_scanlines", [
                        Toolpath(points=[Point(x, y2) for x, y2 in raw_scan_world.coords], kind="debug-raw-scanline", closed=False)
                    ])

                clipped = polygon.intersection(raw_scan)
                clipped_segments: list[list[tuple[float, float]]] = []
                for line in extract_lines(clipped):
                    if line.length < min_segment_length_deg:
                        continue
                    coords = list(line.coords)
                    clipped_segments.append(coords)
                clipped_segments.sort(
                    key=lambda coords: min(point[0] for point in coords),
                )
                if row % 2 == 1:
                    clipped_segments = [list(reversed(coords)) for coords in reversed(clipped_segments)]

                rows.append(clipped_segments)

                row_paths: list[Toolpath] = []
                for coords in clipped_segments:
                    world_line = affinity.rotate(LineString(coords), angle_deg, origin=origin)
                    points = simplify_segment_points([Point(x, y2) for x, y2 in world_line.coords], tolerance_deg, False)
                    if len(points) >= 2:
                        row_paths.append(Toolpath(points=points, kind=kind, closed=False))
                debug_append_toolpaths(debug, "clipped_infill_lines", row_paths)
                y += spacing_deg
                row += 1

            if not allow_pen_down_infill_connectors:
                for row_segments in rows:
                    for coords in row_segments:
                        world_line = affinity.rotate(LineString(coords), angle_deg, origin=origin)
                        points = simplify_segment_points([Point(x, y2) for x, y2 in world_line.coords], tolerance_deg, False)
                        if len(points) >= 2:
                            toolpaths.append(Toolpath(points=points, kind=kind, closed=False))
                continue

            used = [[False for _ in row_segments] for row_segments in rows]
            for row_index, row_segments in enumerate(rows):
                for segment_index, coords in enumerate(row_segments):
                    if used[row_index][segment_index]:
                        continue
                    current_coords = list(coords)
                    used[row_index][segment_index] = True
                    current_row = row_index

                    while True:
                        next_row = current_row + 1
                        while next_row < len(rows) and not any(not flag for flag in used[next_row]):
                            next_row += 1
                        if next_row >= len(rows):
                            break

                        candidates: list[tuple[float, int, list[tuple[float, float]]]] = []
                        for next_index, next_coords in enumerate(rows[next_row]):
                            if used[next_row][next_index]:
                                continue
                            connector = LineString([current_coords[-1], next_coords[0]])
                            if cover_region.covers(connector):
                                candidates.append((connector.length, next_index, next_coords))
                            else:
                                self._emit_debug_connector(
                                    debug,
                                    "rejected_infill_connectors",
                                    current_coords[-1],
                                    next_coords[0],
                                    angle_deg,
                                    origin,
                                    "debug-rejected-connector",
                                )

                        if not candidates:
                            break

                        _, next_index, next_coords = min(candidates, key=lambda item: item[0])
                        self._emit_debug_connector(
                            debug,
                            "valid_infill_connectors",
                            current_coords[-1],
                            next_coords[0],
                            angle_deg,
                            origin,
                            "debug-valid-connector",
                        )
                        current_coords = _concat_coords(current_coords, next_coords)
                        used[next_row][next_index] = True
                        current_row = next_row

                    world_line = affinity.rotate(LineString(current_coords), angle_deg, origin=origin)
                    points = simplify_segment_points([Point(x, y2) for x, y2 in world_line.coords], tolerance_deg, False)
                    if len(points) >= 2:
                        toolpaths.append(Toolpath(points=points, kind=kind, closed=False))

        return toolpaths

    def _generate_centerline_fallback(
        self,
        region: Any,
        *,
        angle_deg: float,
        min_segment_length_deg: float,
        tolerance_deg: float,
        kind: str = "fill-infill",
    ) -> list[Toolpath]:
        if region is None or region.is_empty:
            return []

        origin = region.centroid.coords[0]
        candidate_angles = [angle_deg, angle_deg + 90.0]
        try:
            oriented = region.minimum_rotated_rectangle
            coords = list(oriented.exterior.coords)
            if len(coords) >= 3:
                edges = []
                for start, end in zip(coords, coords[1:]):
                    dx = end[0] - start[0]
                    dy = end[1] - start[1]
                    length = math.hypot(dx, dy)
                    if length > 1e-6:
                        edges.append((length, math.degrees(math.atan2(dy, dx))))
                if edges:
                    edges.sort(reverse=True)
                    candidate_angles.append(edges[0][1])
        except Exception:
            pass

        best_line = None
        best_length = -1.0
        for candidate_angle in candidate_angles:
            rotated = affinity.rotate(region, -candidate_angle, origin=origin)
            min_x, _, max_x, _ = rotated.bounds
            center_y = rotated.centroid.y
            probe = LineString([(min_x - 1.0, center_y), (max_x + 1.0, center_y)])
            clipped = rotated.intersection(probe)
            lines = sorted(extract_lines(clipped), key=lambda line: line.length, reverse=True)
            if lines and lines[0].length > best_length:
                best_line = affinity.rotate(lines[0], candidate_angle, origin=origin)
                best_length = lines[0].length

        if best_line is None or best_length < min_segment_length_deg:
            return []

        points = simplify_segment_points([Point(x, y) for x, y in best_line.coords], tolerance_deg, False)
        if len(points) < 2:
            return []
        return [Toolpath(points=points, kind=kind, closed=False)]

    def _generate_detail_fill(
        self,
        region: Any,
        *,
        line_width_deg: float,
        scanline_spacing_deg: float,
        angle_deg: float,
        min_segment_length_deg: float,
        tolerance_deg: float,
        detail_tolerance_deg: float,
        allow_overlap: bool,
        allow_pen_down_infill_connectors: bool = DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if region is None or region.is_empty:
            return []

        detail_spacing = max(line_width_deg * 0.35, scanline_spacing_deg * (0.5 if allow_overlap else 1.0))
        inset_amount = min(line_width_deg * 0.15, line_width_deg * 0.5)
        detail_region = region.buffer(-inset_amount, join_style=1)
        if detail_region.is_empty:
            detail_region = region

        detail_paths = self._generate_scanline_infill(
            detail_region,
            spacing_deg=detail_spacing,
            angle_deg=angle_deg,
            min_segment_length_deg=min_segment_length_deg,
            tolerance_deg=max(tolerance_deg, detail_tolerance_deg),
            kind="detail-trace",
            allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
            debug=debug,
        )
        if detail_paths:
            return detail_paths

        return self._generate_centerline_fallback(
            detail_region,
            angle_deg=angle_deg,
            min_segment_length_deg=min_segment_length_deg,
            tolerance_deg=max(tolerance_deg, detail_tolerance_deg),
            kind="detail-trace",
        )

    def slice_one_layer(
        self,
        printable_geometry: Any,
        *,
        line_width_mm: float,
        wall_count: int,
        infill_density: float = 100.0,
        infill_angle_deg: float = 0.0,
        outline_after_fill: bool = False,
        min_fill_area_mm2: float = 1.0,
        min_segment_length_mm: float = 0.5,
        infill_spacing_mm: float = DEFAULT_INFILL_SPACING_MM,
        min_fill_width_mm: float = DEFAULT_MIN_FILL_WIDTH_MM,
        simplify_tolerance_mm: float = DEFAULT_SIMPLIFY_TOLERANCE_MM,
        remove_duplicate_paths: bool = True,
        small_shape_mode: str = DEFAULT_SMALL_SHAPE_MODE,
        thin_detail_mode: bool = DEFAULT_THIN_DETAIL_MODE,
        thin_detail_min_area_mm2: float = DEFAULT_THIN_DETAIL_MIN_AREA_MM2,
        thin_detail_simplify_mm: float = DEFAULT_THIN_DETAIL_SIMPLIFY_MM,
        thin_detail_overlap: bool = DEFAULT_THIN_DETAIL_OVERLAP,
        travel_optimization: str = DEFAULT_TRAVEL_OPTIMIZATION,
        allow_pen_down_infill_connectors: bool = DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if printable_geometry is None or printable_geometry.is_empty or line_width_mm <= 0:
            return []

        line_width_deg = mm_to_ball_degrees(line_width_mm)
        simplify_tolerance_deg = mm_to_ball_degrees(simplify_tolerance_mm)
        thin_detail_tolerance_deg = mm_to_ball_degrees(thin_detail_simplify_mm)
        min_segment_length_deg = mm_to_ball_degrees(min_segment_length_mm)
        min_fill_area_deg2 = mm_area_to_ball_degree_area(min_fill_area_mm2)
        thin_detail_min_area_deg2 = mm_area_to_ball_degree_area(thin_detail_min_area_mm2)
        scanline_spacing_deg = self._scanline_spacing_deg(SlicerSettings(
            line_width_mm=line_width_mm,
            wall_count=wall_count,
            infill_density=infill_density,
            infill_spacing_mm=infill_spacing_mm,
            allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
        ))

        debug_append_geometry(debug, "final_composed_fill_region", printable_geometry, "final-composed-fill")
        polygons = sorted(
            normalize_geometry(printable_geometry),
            key=lambda poly: (-round(poly.area, 5), -round(poly.centroid.y, 5), round(poly.centroid.x, 5)),
        )

        ordered: list[Toolpath] = []
        slicer_counts = {
            "normal_slicer_region_count": 0,
            "outline_buffer_empty_region_count": 0,
            "normal_infill_empty_region_count": 0,
            "thin_detail_fallback_region_count": 0,
            "thin_detail_path_count": 0,
            "detail_trace_path_count": 0,
        }
        for polygon in polygons:
            printable_outline_region = polygon.buffer(-(line_width_deg * 0.5), join_style=1)
            can_fit_outline = not printable_outline_region.is_empty
            if not can_fit_outline:
                slicer_counts["outline_buffer_empty_region_count"] += 1
            if not can_fit_outline and not thin_detail_mode:
                continue

            debug_append_geometry(debug, "detected_printable_polygons", polygon, "detected-printable-polygon")
            region_paths: list[Toolpath] = []

            if can_fit_outline:
                slicer_counts["normal_slicer_region_count"] += 1
                outer_walls = geometry_to_closed_toolpaths(printable_outline_region, "fill-wall", simplify_tolerance_deg)
                debug_append_toolpaths(debug, "outer_walls", outer_walls)
                region_paths.extend(optimize_toolpath_order(outer_walls, strategy=travel_optimization))

            anchor = region_paths[-1].points[-1] if region_paths and region_paths[-1].points else Point(0.0, 0.0)
            inner_wall_paths: list[Toolpath] = []
            if can_fit_outline:
                for wall_index in range(1, max(1, wall_count)):
                    wall_geometry = polygon.buffer(-(line_width_deg * (wall_index + 0.5)), join_style=1)
                    if wall_geometry.is_empty:
                        continue
                    debug_append_geometry(debug, "inner_wall_regions", wall_geometry, f"inner-wall-region-{wall_index}")
                    inner_wall_paths.extend(geometry_to_closed_toolpaths(wall_geometry, "fill-wall", simplify_tolerance_deg))
                debug_append_toolpaths(debug, "inner_walls", inner_wall_paths)
                inner_wall_paths = optimize_toolpath_order(inner_wall_paths, strategy=travel_optimization, start_point=anchor)
                region_paths.extend(inner_wall_paths)

            infill_paths: list[Toolpath] = []
            infill_region = None
            fill_threshold_failed = True
            if can_fit_outline:
                infill_offset = line_width_deg * max(1, wall_count)
                infill_region = polygon.buffer(-infill_offset, join_style=1)
                if not infill_region.is_empty:
                    fill_area = infill_region.area
                    fill_threshold_failed = fill_area < min_fill_area_deg2
                else:
                    slicer_counts["normal_infill_empty_region_count"] += 1
            if not fill_threshold_failed and infill_region is not None and not infill_region.is_empty:
                debug_append_geometry(debug, "infill_regions", infill_region, "infill-region")
                infill_paths = self._generate_scanline_infill(
                    infill_region,
                    spacing_deg=scanline_spacing_deg,
                    angle_deg=infill_angle_deg,
                    min_segment_length_deg=min_segment_length_deg,
                    tolerance_deg=simplify_tolerance_deg,
                    allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
                    debug=debug,
                )
            multi_pass_infill = len(infill_paths) >= 2 or any(len(path.points) >= 4 for path in infill_paths)
            single_pass_infill = len(infill_paths) == 1 and not multi_pass_infill
            if not multi_pass_infill:
                detail_region = printable_outline_region if can_fit_outline else polygon
                if thin_detail_mode and polygon.area >= thin_detail_min_area_deg2:
                    slicer_counts["thin_detail_fallback_region_count"] += 1
                    infill_paths = self._generate_detail_fill(
                        detail_region,
                        line_width_deg=line_width_deg,
                        scanline_spacing_deg=scanline_spacing_deg,
                        angle_deg=infill_angle_deg,
                        min_segment_length_deg=min_segment_length_deg,
                        tolerance_deg=simplify_tolerance_deg,
                        detail_tolerance_deg=thin_detail_tolerance_deg,
                        allow_overlap=thin_detail_overlap,
                        allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
                        debug=debug,
                    )
                elif small_shape_mode == "centerline":
                    slicer_counts["thin_detail_fallback_region_count"] += 1
                    centerline_region = infill_region if infill_region is not None and not infill_region.is_empty else detail_region
                    infill_paths = self._generate_centerline_fallback(
                        centerline_region,
                        angle_deg=infill_angle_deg,
                        min_segment_length_deg=min_segment_length_deg,
                        tolerance_deg=max(simplify_tolerance_deg, thin_detail_tolerance_deg),
                        kind="detail-trace",
                    )
                elif small_shape_mode == "skip":
                    continue
                else:
                    infill_paths = []
            slicer_counts["thin_detail_path_count"] += sum(1 for path in infill_paths if path.kind == "detail-trace")
            slicer_counts["detail_trace_path_count"] = slicer_counts["thin_detail_path_count"]

            infill_paths = optimize_toolpath_order(
                infill_paths,
                strategy=travel_optimization,
                start_point=region_paths[-1].points[-1] if region_paths and region_paths[-1].points else anchor,
            )
            region_paths.extend(infill_paths)

            if outline_after_fill and can_fit_outline:
                cleanup_paths = geometry_to_closed_toolpaths(printable_outline_region, "outline", simplify_tolerance_deg)
                debug_append_toolpaths(debug, "cleanup_outlines", cleanup_paths)
                cleanup_paths = optimize_toolpath_order(
                    cleanup_paths,
                    strategy=travel_optimization,
                    start_point=region_paths[-1].points[-1] if region_paths and region_paths[-1].points else anchor,
                )
                region_paths.extend(cleanup_paths)

            ordered.extend(region_paths)

        non_detail_paths = [path for path in ordered if path.kind != "detail-trace"]
        thin_detail_paths = [path for path in ordered if path.kind == "detail-trace"]
        if remove_duplicate_paths:
            non_detail_paths = dedupe_toolpaths(non_detail_paths, min_segment_length_deg)
        else:
            non_detail_paths = filter_toolpaths_by_length(non_detail_paths, min_segment_length_deg)
        thin_detail_paths = filter_toolpaths_by_length(thin_detail_paths, min_segment_length_deg)
        ordered = merge_connected_toolpaths(non_detail_paths + thin_detail_paths)
        debug_set_counts(debug, "slicer_counts", slicer_counts)
        return ordered


def generate_toolpaths(
    bundle: GeometryBundle,
    *,
    enable_fill: bool,
    line_width_mm: float,
    wall_count: int,
    infill_density: float,
    infill_spacing_mm: float,
    infill_angle_deg: float,
    outline_after_fill: bool,
    min_fill_area_mm2: float,
    min_fill_width_mm: float,
    simplify_tolerance_mm: float,
    remove_duplicate_paths: bool,
    small_shape_mode: str,
    thin_detail_mode: bool = DEFAULT_THIN_DETAIL_MODE,
    thin_detail_min_area_mm2: float = DEFAULT_THIN_DETAIL_MIN_AREA_MM2,
    thin_detail_simplify_mm: float = DEFAULT_THIN_DETAIL_SIMPLIFY_MM,
    thin_detail_overlap: bool = DEFAULT_THIN_DETAIL_OVERLAP,
    min_segment_length_mm: float = DEFAULT_MIN_SEGMENT_LENGTH_MM,
    travel_optimization: str = DEFAULT_TRAVEL_OPTIMIZATION,
    allow_pen_down_infill_connectors: bool = DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS,
    debug: Optional[dict[str, Any]] = None,
) -> list[Toolpath]:
    toolpaths: list[Toolpath] = []
    simplify_tolerance_deg = mm_to_ball_degrees(simplify_tolerance_mm)
    detail_tolerance_deg = mm_to_ball_degrees(max(simplify_tolerance_mm, thin_detail_simplify_mm))

    outline_segments = list(bundle.outline_segments)
    if not enable_fill:
        outline_segments.extend(bundle.fill_boundary_segments)

    for segment in outline_segments:
        simplified = simplify_segment_points(segment.points, simplify_tolerance_deg, segment.closed)
        toolpaths.append(Toolpath(points=simplified, kind="outline", closed=segment.closed))

    if enable_fill and bundle.printable_geometry is not None and not bundle.printable_geometry.is_empty:
        slicer = SlicerService()
        toolpaths.extend(slicer.slice_one_layer(
            bundle.printable_geometry,
            line_width_mm=line_width_mm,
            wall_count=wall_count,
            infill_density=infill_density,
            infill_angle_deg=infill_angle_deg,
            outline_after_fill=outline_after_fill,
            min_fill_area_mm2=min_fill_area_mm2,
            min_segment_length_mm=min_segment_length_mm,
            infill_spacing_mm=infill_spacing_mm,
            min_fill_width_mm=min_fill_width_mm,
            simplify_tolerance_mm=simplify_tolerance_mm,
            remove_duplicate_paths=remove_duplicate_paths,
            small_shape_mode=small_shape_mode,
            thin_detail_mode=thin_detail_mode,
            thin_detail_min_area_mm2=thin_detail_min_area_mm2,
            thin_detail_simplify_mm=thin_detail_simplify_mm,
            thin_detail_overlap=thin_detail_overlap,
            travel_optimization=travel_optimization,
            allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
            debug=debug,
        ))

    detail_paths: list[Toolpath] = []
    for segment in bundle.detail_segments:
        simplified = simplify_segment_points(segment.points, detail_tolerance_deg, segment.closed)
        if len(simplified) < 2:
            continue
        detail_paths.append(Toolpath(points=simplified, kind="detail-trace", closed=segment.closed))
    if detail_paths:
        toolpaths.extend(merge_connected_toolpaths(
            optimize_toolpath_order(detail_paths, strategy=travel_optimization)
        ))

    toolpaths = merge_connected_toolpaths(toolpaths)

    toolpath_counts = {
        "generated_fill_walls": sum(1 for path in toolpaths if path.kind == "fill-wall"),
        "generated_infill_paths": sum(1 for path in toolpaths if path.kind == "fill-infill"),
        "generated_thin_detail_paths": sum(1 for path in toolpaths if path.kind == "detail-trace"),
        "generated_detail_trace_paths": sum(1 for path in toolpaths if path.kind == "detail-trace"),
        "generated_outline_paths": sum(1 for path in toolpaths if path.kind == "outline"),
        "generated_travel_paths": sum(1 for path in toolpaths if path.kind == "travel"),
    }
    debug_set_counts(debug, "toolpath_counts", toolpath_counts)
    if debug is not None:
        debug["toolpath_diagnostics"] = summarize_toolpaths(toolpaths)
    debug_append_toolpaths(debug, "final_toolpaths", toolpaths)
    return toolpaths


def generate_gcode_from_toolpaths(
    toolpaths: list[Toolpath],
    draw_feed: float,
    travel_feed: float,
    sample_step_deg: float,
    placement_offset_x: float,
    placement_offset_y: float,
    pen_up_s: int,
    pen_down_s: int,
    servo_ramp_enabled: bool,
    servo_ramp_step: int,
    servo_ramp_delay_ms: float,
    pen_up_dwell_ms: float,
    pen_down_dwell_ms: float,
    gcode_mode: str,
    include_comments: bool,
) -> tuple[list[str], list[dict[str, Any]]]:
    if gcode_mode != "simple":
        raise ValueError("Invalid G-code mode")

    g: list[str] = []
    preview: list[dict[str, Any]] = []
    current_servo = pen_up_s
    current_position = Point(0.0, 0.0)
    current_pen_down = False
    sample_step_mm = ball_radius_mm() * math.radians(max(0.05, sample_step_deg)) * 0.5
    stream_line_number = 0

    def comment(text: str) -> None:
        if include_comments:
            g.append(f"({text})")

    def append_gcode(line: str) -> int | None:
        nonlocal stream_line_number
        g.append(line)
        if is_streamable_gcode_line(line):
            stream_line_number += 1
            return stream_line_number
        return None

    comment("Generated for golf ball plotter")
    comment("Units are angular degrees. X=-180..180 ball rotation, Y=-45..45 arm tilt")
    for command in ["$X", "G21", "G90"]:
        append_gcode(command)
    for command in build_pen_position_commands(
        pen_up_s,
        pen_up_s,
        ramp_enabled=False,
        ramp_step=servo_ramp_step,
        ramp_delay_ms=servo_ramp_delay_ms,
        dwell_ms=pen_up_dwell_ms,
    ):
        append_gcode(command)

    for index, toolpath in enumerate(toolpaths, start=1):
        sampled_surface_points = resample_segment(toolpath.points, max_step=sample_step_mm)
        pts = [
            surface_mm_to_ball_angles(
                point,
                center_lon_deg=placement_offset_x,
                center_lat_deg=placement_offset_y,
            )
            for point in sampled_surface_points
        ]
        if len(pts) < 2:
            continue

        for point in pts:
            if point.y < (Y_DRAW_MIN - 1e-6) or point.y > (Y_DRAW_MAX + 1e-6):
                raise ValueError(f"Projected toolpath exceeds Y drawing limits at {point.y:.3f} degrees")
            if point.x < (X_DRAW_MIN - 1e-6) or point.x > (X_DRAW_MAX + 1e-6):
                raise ValueError(f"Projected toolpath exceeds X drawing limits at {point.x:.3f} degrees")

        start = pts[0]
        if not nearly_same_point(current_position, start):
            travel_id = f"travel-{index:04d}"
            if current_pen_down:
                for command in build_pen_position_commands(
                    current_servo,
                    pen_up_s,
                    ramp_enabled=servo_ramp_enabled,
                    ramp_step=servo_ramp_step,
                    ramp_delay_ms=servo_ramp_delay_ms,
                    dwell_ms=pen_up_dwell_ms,
                ):
                    append_gcode(command)
                current_servo = pen_up_s
                current_pen_down = False
            comment(f"Travel to {toolpath.kind} path {index}")
            travel_line = append_gcode(f"G1 X{start.x:.4f} Y{start.y:.4f} F{travel_feed:.3f}")
            preview.append({
                "id": travel_id,
                "kind": "travel",
                "closed": False,
                "points": [asdict(current_position), asdict(start)],
                "gcode_start_line": travel_line,
                "gcode_end_line": travel_line,
            })
            current_position = start

        if not current_pen_down:
            for command in build_pen_position_commands(
                current_servo,
                pen_down_s,
                ramp_enabled=servo_ramp_enabled,
                ramp_step=servo_ramp_step,
                ramp_delay_ms=servo_ramp_delay_ms,
                dwell_ms=pen_down_dwell_ms,
            ):
                append_gcode(command)
            current_servo = pen_down_s
            current_pen_down = True

        path_id = f"path-{index:04d}"
        draw_start_line = None
        draw_end_line = None
        comment(f"{toolpath.kind} path {index}, {len(pts)} points")
        for point in pts[1:]:
            line_number = append_gcode(f"G1 X{point.x:.4f} Y{point.y:.4f} F{draw_feed:.3f}")
            if line_number is not None:
                if draw_start_line is None:
                    draw_start_line = line_number
                draw_end_line = line_number
            current_position = point
        preview.append({
            "id": path_id,
            "kind": toolpath.kind,
            "closed": toolpath.closed,
            "points": [asdict(point) for point in pts],
            "gcode_start_line": draw_start_line,
            "gcode_end_line": draw_end_line,
        })

        for command in build_pen_position_commands(
            current_servo,
            pen_up_s,
            ramp_enabled=servo_ramp_enabled,
            ramp_step=servo_ramp_step,
            ramp_delay_ms=servo_ramp_delay_ms,
            dwell_ms=pen_up_dwell_ms,
        ):
            append_gcode(command)
        current_servo = pen_up_s
        current_pen_down = False

    comment("Return to zero with pen up")
    if not nearly_same_point(current_position, Point(0.0, 0.0)):
        return_home_line = append_gcode(f"G1 X0.0000 Y0.0000 F{travel_feed:.3f}")
        preview.append({
            "id": "travel-home",
            "kind": "travel",
            "closed": False,
            "points": [asdict(current_position), asdict(Point(0.0, 0.0))],
            "gcode_start_line": return_home_line,
            "gcode_end_line": return_home_line,
        })
    for command in build_pen_position_commands(
        current_servo,
        pen_up_s,
        ramp_enabled=False,
        ramp_step=servo_ramp_step,
        ramp_delay_ms=servo_ramp_delay_ms,
        dwell_ms=pen_up_dwell_ms,
    ):
        append_gcode(command)

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
        infill_density=100.0,
        infill_spacing_mm=0.75,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        debug={},
    )
    expect(any(path.kind == "fill-wall" for path in toolpaths), "printable regions generate fill walls")
    expect(any(path.kind == "fill-infill" for path in toolpaths), "printable regions generate infill when geometry is large enough")
    hole_box = Polygon([(30, 30), (70, 30), (70, 70), (30, 70)])
    infill_points = [point for path in toolpaths if path.kind == "fill-infill" for point in path.points]
    expect(not any(hole_box.buffer(-0.01).contains(ShapelyPoint(point.x, point.y)) for point in infill_points), "infill lines do not cross cutout holes")
    equator = surface_mm_to_ball_angles(Point(10.0, 0.0), center_lon_deg=0.0, center_lat_deg=0.0)
    at_45 = surface_mm_to_ball_angles(Point(10.0, 0.0), center_lon_deg=0.0, center_lat_deg=45.0)
    expect(abs(at_45.x) > abs(equator.x) * 1.39, "surface mapping expands longitude away from the equator to preserve physical width")

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
            def should_stop() -> bool:
                with job_lock:
                    if job_stop_requested:
                        state["status"] = "Stopped"
                        return True
                return False

            def wait_while_paused() -> None:
                with job_lock:
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

            def on_line_sent(line: str, sent_count: int) -> None:
                state["status"] = f"Running: {line}"
                state["progress_done"] = sent_count

            stream_gcode_lines_unlocked(
                ser,
                stream_lines,
                response_timeout=20,
                should_stop=should_stop,
                wait_while_paused=wait_while_paused,
                on_line_sent=on_line_sent,
            )

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
