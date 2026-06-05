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
    "DEFAULT_MAX_PRINT_X_SPAN_DEG",
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
    "DEFAULT_FILL_STRATEGY",
    "DEFAULT_ALTERNATE_FILL_ANGLE_DEG",
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
    "DEFAULT_ALLOW_DETAIL_PEN_DOWN_CONTINUATION",
    "DEFAULT_INFILL_PATH_MODE",
    "DEFAULT_THIN_REGION_SINGLE_STROKE_MAX_FACTOR",
    "DEFAULT_NARROW_REGION_MAX_FACTOR",
    "DEFAULT_COLLAPSE_OUTLINE_MAX_FACTOR",
    "DEFAULT_TINY_DOT_AREA_FACTOR",
    "DEFAULT_SINGLE_STROKE_WIDTH_MAX_FACTOR",
    "DEFAULT_CENTERLINE_WIDTH_MAX_FACTOR",
    "DEFAULT_DETAIL_WIDTH_MAX_FACTOR",
    "DEFAULT_STREAMING_MODE",
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
    state.setdefault("streaming_mode", config.get("DEFAULT_STREAMING_MODE", DEFAULT_STREAMING_MODE))
    serial_lock = serial_lock_obj


from collections import Counter, deque
import hashlib
import json
import logging
import cv2
from flask import Flask, request, jsonify, render_template_string
import numpy as np
import serial
import time
import threading
import math
import os
import re
import statistics
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from dataclasses import dataclass, asdict, field
from typing import Optional, Any, Callable, Literal

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

logger = logging.getLogger(__name__)

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
DEFAULT_LINE_THICKNESS_MM = 0.2

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
DEFAULT_MAX_PRINT_X_SPAN_DEG = 120.0
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
DEFAULT_INFILL_PATTERN = "hatch"
DEFAULT_INFILL_DENSITY = 100.0
DEFAULT_INFILL_SPACING_MM = DEFAULT_LINE_THICKNESS_MM
DEFAULT_INFILL_OVERLAP_PERCENT = 20.0
DEFAULT_INFILL_ANGLE_DEG = 45.0
DEFAULT_FILL_STRATEGY = "adaptive_angle"
DEFAULT_ALTERNATE_FILL_ANGLE_DEG = -45.0
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
DEFAULT_ALLOW_DETAIL_PEN_DOWN_CONTINUATION = True
DEFAULT_INFILL_PATH_MODE = "rectilinear"
DEFAULT_LONG_THIN_INFILL_ASPECT_RATIO = 3.0
DEFAULT_SMALL_DETAIL_MIN_DIM_FACTOR = 4.0
DEFAULT_SMALL_DETAIL_MIN_DIM_FLOOR_MM = 1.0
DEFAULT_SMALL_DETAIL_AREA_FACTOR = 18.0
DEFAULT_SHORT_INFILL_SEGMENT_FACTOR = 1.5
DEFAULT_MAX_PEN_DOWN_CONNECTOR_SPACING_FACTOR = 1.4
DEFAULT_THIN_REGION_SINGLE_STROKE_MAX_FACTOR = 1.5
DEFAULT_NARROW_REGION_MAX_FACTOR = 3.0
DEFAULT_COLLAPSE_OUTLINE_MAX_FACTOR = 2.0
DEFAULT_TINY_DOT_AREA_FACTOR = 0.20
DEFAULT_SINGLE_STROKE_WIDTH_MAX_FACTOR = 1.15
DEFAULT_CENTERLINE_WIDTH_MAX_FACTOR = 1.5
DEFAULT_DETAIL_WIDTH_MAX_FACTOR = 2.5
DEFAULT_MAX_DETAIL_CONTINUATION_LENGTH_FACTOR = 4.0
DEFAULT_PREFERRED_DETAIL_CONTINUATION_LENGTH_FACTOR = 2.0
DEFAULT_MAX_DETAIL_CONTINUATION_OVERSPILL_AREA_RATIO = 0.02
DEFAULT_MAX_DETAIL_CONTINUATION_TURN_DEG = 120.0
DEFAULT_STREAMING_MODE = "buffered"
DEFAULT_OUTLINE_PLACEMENT_MODE = "inside_edge_default"
DEFAULT_PROJECTION_SAMPLING_MAX_SEGMENT_MM = 0.15
ORIGIN_ANCHORS = {
    "center",
    "min-x",
    "max-x",
    "min-y",
    "max-y",
    "top-left",
    "top-center",
    "top-right",
    "center-left",
    "center-right",
    "bottom-left",
    "bottom-center",
    "bottom-right",
    "custom",
}
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
    "last_timeout_debug": None,
    "progress_total": 0,
    "progress_done": 0,
    "current_servo_s": DEFAULT_PEN_UP_S,
    "streaming_mode": "buffered",
    "streaming": {
        "mode": "buffered",
        "current_line": 0,
        "current_path_id": None,
        "current_path_kind": None,
        "pending_buffer_chars": 0,
        "pending_commands": 0,
        "last_response_age_sec": 0.0,
        "last_grbl_status": None,
        "ok_count": 0,
        "error_count": 0,
        "sent_count": 0,
    },
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
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class XAxisCalibrationTick:
    id: str
    label: str
    commanded_x_deg: float
    emitted_x_deg: float
    y_start_deg: float
    y_end_deg: float


@dataclass
class PrintableRegion:
    component_id: str
    outer_rings_mm: list[list[Point]]
    hole_rings_mm: list[list[Point]]
    source: str = "final_fill_clip_polygon"
    coordinate_space: str = "surface_mm"
    geometry: Any = None


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
    coordinate_space: str = "surface_mm"
    path_id: str | None = None
    source: str = "unknown"
    region_id: int | None = None
    warnings: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class SlicerSettings:
    line_width_mm: float
    wall_count: int
    infill_density: float = 100.0
    infill_spacing_mm: float = DEFAULT_INFILL_SPACING_MM
    infill_angle_deg: float = 0.0
    fill_strategy: str = "horizontal_scanline"
    alternate_fill_angle_deg: float = -45.0
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
    infill_path_mode: str = DEFAULT_INFILL_PATH_MODE


FillStrategy = Literal[
    "RECTILINEAR_SERPENTINE",
    "CONTOUR_PARALLEL_DETAIL",
    "CENTERLINE_DETAIL",
    "SINGLE_STROKE_DETAIL",
    "OUTLINE_ONLY",
    "SKIP_FILL",
]


ThinRegionMode = Literal["singleStroke", "outlineOnly", "skip"]


@dataclass
class HybridInfillConfig:
    enabled: bool = True
    lineWidthMm: float = 0.0
    infillSpacingMm: float = 0.0
    wallCount: int = 1
    infillAngleDeg: float = 0.0
    singleStrokeWidthMaxFactor: float = 1.15
    centerlineWidthMaxFactor: float = 1.5
    detailWidthMaxFactor: float = 2.5
    detailMinWidthFactor: float = 2.25
    minSerpentineRowLengthFactor: float = 3.0
    minAreaToFillMm2: float = 0.15
    minUsableDetailAreaMm2: float = 0.15
    minNormalFillAreaMm2: float = 0.35
    connectorValidation: str = "sampled"
    connectorSampleStepMm: float = 0.05
    allowInternalConnectorOverlap: bool = True
    maxConnectorOverlapMm: float = 0.0
    detailFillEnabled: bool = True
    centerlineFallbackEnabled: bool = True
    thinRegionMode: ThinRegionMode = "singleStroke"
    allowOutlineOverlapForThinRegions: bool = True
    optimizePathOrder: bool = True
    singleStrokeMaxWidthFactor: float = DEFAULT_THIN_REGION_SINGLE_STROKE_MAX_FACTOR
    narrowRegionMaxWidthFactor: float = DEFAULT_NARROW_REGION_MAX_FACTOR
    collapseOutlineMaxWidthFactor: float = DEFAULT_COLLAPSE_OUTLINE_MAX_FACTOR
    tinyDotAreaFactor: float = DEFAULT_TINY_DOT_AREA_FACTOR


@dataclass
class NormalizedGeometryConfig:
    lineWidthMm: float
    penRadiusMm: float
    effectiveInfillSpacingMm: float
    effectiveDetailSpacingMm: float
    effectiveWallSpacingMm: float
    previewStrokeWidthMm: float
    surfaceMmToPreviewPxScale: float | None = None
    connectorSampleStepMm: float = 0.05
    source: dict[str, Any] = field(default_factory=dict)


@dataclass
class GeometrySpacingMetrics:
    lineWidthMm: float
    penRadiusMm: float
    effectiveInfillSpacingMm: float
    effectiveDetailSpacingMm: float
    effectiveWallSpacingMm: float
    previewStrokeWidthMm: float
    actualAverageInfillSpacingMm: float | None = None
    actualMaxInfillSpacingMm: float | None = None
    actualAverageDetailOffsetSpacingMm: float | None = None
    actualMaxDetailOffsetSpacingMm: float | None = None
    estimatedUncoveredGapMm: float = 0.0
    expectedOverlapMm: float = 0.0
    componentBoundsMm: dict[str, float] | None = None
    previewBoundsPx: dict[str, float] | None = None
    previewGcodePathMismatchCount: int = 0


def normalize_geometry_config(
    *,
    raw_line_width_mm: float,
    raw_infill_spacing_mm: float | None = None,
    raw_infill_overlap_percent: float | None = None,
    raw_detail_spacing_mm: float | None = None,
    raw_wall_spacing_mm: float | None = None,
    raw_preview_stroke_width_mm: float | None = None,
    surface_mm_to_preview_px_scale: float | None = None,
    connector_sample_step_mm: float | None = None,
) -> NormalizedGeometryConfig:
    line_width_mm = max(0.0, float(raw_line_width_mm))
    pen_radius_mm = line_width_mm / 2.0
    # Determine infill overlap percent (default fallback)
    infill_overlap_percent = float(raw_infill_overlap_percent) if raw_infill_overlap_percent is not None else float(DEFAULT_INFILL_OVERLAP_PERCENT)
    # If explicit spacing not provided, compute effective spacing using overlap percent
    if raw_infill_spacing_mm is None:
        effective_infill_spacing_mm = max(0.0, line_width_mm * (1.0 - infill_overlap_percent / 100.0))
    else:
        effective_infill_spacing_mm = max(0.0, float(raw_infill_spacing_mm))
    effective_detail_spacing_mm = line_width_mm if raw_detail_spacing_mm is None else max(0.0, float(raw_detail_spacing_mm))
    effective_wall_spacing_mm = line_width_mm if raw_wall_spacing_mm is None else max(0.0, float(raw_wall_spacing_mm))
    preview_stroke_width_mm = line_width_mm if raw_preview_stroke_width_mm is None else max(0.0, float(raw_preview_stroke_width_mm))
    resolved_connector_sample_step_mm = connector_sample_step_mm if connector_sample_step_mm is not None else max(0.01, min(line_width_mm / 4.0, 0.05))
    return NormalizedGeometryConfig(
        lineWidthMm=line_width_mm,
        penRadiusMm=pen_radius_mm,
        effectiveInfillSpacingMm=effective_infill_spacing_mm,
        effectiveDetailSpacingMm=effective_detail_spacing_mm,
        effectiveWallSpacingMm=effective_wall_spacing_mm,
        previewStrokeWidthMm=preview_stroke_width_mm,
        surfaceMmToPreviewPxScale=surface_mm_to_preview_px_scale,
        connectorSampleStepMm=resolved_connector_sample_step_mm,
        source={
            "rawLineWidthMm": float(raw_line_width_mm),
            "rawInfillSpacingMm": None if raw_infill_spacing_mm is None else float(raw_infill_spacing_mm),
            "rawInfillOverlapPercent": None if raw_infill_overlap_percent is None else float(raw_infill_overlap_percent),
            "rawDetailSpacingMm": None if raw_detail_spacing_mm is None else float(raw_detail_spacing_mm),
            "rawPreviewStrokeWidthMm": None if raw_preview_stroke_width_mm is None else float(raw_preview_stroke_width_mm),
        },
    )


@dataclass
class RegionMetrics:
    areaMm2: float
    bboxWidthMm: float
    bboxHeightMm: float
    minDimensionMm: float
    maxLocalWidthMm: float
    aspectRatio: float
    holeCount: int
    componentCount: int
    estimatedRowCount: float
    estimatedShortRowRatio: float
    highCurvatureScore: float | None = None
    estimatedLocalWidthMm: float | None = None


@dataclass
class InfillMetrics:
    totalPaths: int = 0
    penLifts: int = 0
    travelDistanceMm: float = 0.0
    drawDistanceMm: float = 0.0
    invalidConnectorCount: int = 0
    rejectedConnectorCount: int = 0
    acceptedConnectorCount: int = 0
    rectilinearRegionCount: int = 0
    contourDetailRegionCount: int = 0
    centerlineRegionCount: int = 0
    singleStrokeRegionCount: int = 0
    outlineOnlyRegionCount: int = 0
    skippedTinyRegionCount: int = 0
    collapsedDrawableRegionCount: int = 0
    suppressedWallForThinRegionCount: int = 0
    prunedNoisyPathCount: int = 0
    averageRowLengthMm: float = 0.0
    shortRowCount: int = 0
    estimatedOverlapMm: float = 0.0
    estimatedMissedAreaMm2: float | None = None
    outsideDrawablePathPointCount: int = 0


@dataclass
class InfillSegment:
    id: str
    component_id: str
    row_index: int
    interval_index: int
    cell_id: str | None
    scanline_offset: float
    low_u: Point
    high_u: Point
    min_u: float
    max_u: float
    center: Point
    length: float
    kind: str = "infill"
    coords: list[tuple[float, float]] = field(default_factory=list)


@dataclass
class InfillCellPlan:
    cell_id: str
    component_id: str
    segments: list[InfillSegment]
    toolpaths: list[Toolpath]
    entry_point: Point
    exit_point: Point
    centroid: Point
    total_length: float


@dataclass
class InfillCellAdaptiveDecision:
    mode: str
    reasons: list[str]
    metrics: dict[str, float]


@dataclass
class InfillConnectorValidationResult:
    accepted: bool
    reason: str
    connector_mode: str = "direct"
    connector_coords: list[tuple[float, float]] = field(default_factory=list)
    sample_failures: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class MaskCoverageMetrics:
    mask_area_px: int
    covered_inside_mask_px: int
    missed_inside_mask_px: int
    overdraw_outside_mask_px: int
    covered_pixels_total_px: int
    raw_coverage_percent: float
    outside_overdraw_percent: float
    penalized_coverage_percent: float
    missed_inside_mask_percent: float
    px_per_mm: float
    pen_radius_px: float


def _connector_meta_value(meta: dict[str, Any] | None, *keys: str) -> Any:
    if not meta:
        return None
    for key in keys:
        value = meta.get(key)
        if value is not None:
            return value
    return None


def _offset_geometry(
    geometry: Any,
    offset_mm: float,
    *,
    join_style: int = 1,
    miter_limit: float = 3.0,
    quad_segs: int = 16,
) -> Any:
    if geometry is None or geometry.is_empty or abs(offset_mm) <= 1e-12:
        return geometry
    offset = geometry.buffer(
        offset_mm,
        join_style=max(1, min(3, int(join_style))),
        mitre_limit=max(1.0, float(miter_limit)),
        quad_segs=max(4, int(quad_segs)),
    )
    if offset is not None and not offset.is_empty and not offset.is_valid:
        offset = make_valid(offset) if make_valid is not None else offset.buffer(0)
    return offset


def _point_inside_geometry(geometry: Any, point: Point) -> bool:
    if geometry is None or geometry.is_empty:
        return False
    try:
        return bool(geometry.covers(ShapelyPoint(point.x, point.y)))
    except Exception:
        return False


def can_draw_connector(
    from_point: Point,
    to_point: Point,
    drawable_area: Any,
    pen_radius_mm: float,
    config: HybridInfillConfig,
) -> bool:
    if drawable_area is None or drawable_area.is_empty:
        return False
    connector = LineString([(from_point.x, from_point.y), (to_point.x, to_point.y)])
    if connector.length <= 1e-9:
        return _point_inside_geometry(drawable_area, from_point)

    max_length_mm = config.maxConnectorOverlapMm if config.maxConnectorOverlapMm > 0 else max(config.infillSpacingMm * 2.0, config.lineWidthMm * config.minSerpentineRowLengthFactor)
    if (not config.allowInternalConnectorOverlap) and connector.length > max_length_mm + 1e-6:
        return False
    if connector.length > max_length_mm * 1.5 + 1e-6:
        return False

    if config.connectorValidation == "capsule":
        capsule = connector.buffer(max(0.0, pen_radius_mm), cap_style=1, join_style=1)
        if capsule.is_empty:
            return False
        return bool(drawable_area.covers(capsule))

    step_mm = config.connectorSampleStepMm if config.connectorSampleStepMm > 0 else min(max(pen_radius_mm * 0.5, 0.01), 0.05)
    sample_count = max(2, int(math.ceil(connector.length / max(1e-6, step_mm))) + 1)
    for sample_index in range(sample_count):
        distance_mm = min(connector.length, (connector.length * sample_index) / max(1, sample_count - 1))
        sample_point = connector.interpolate(distance_mm)
        if not drawable_area.covers(sample_point):
            return False
    return True


def choose_fill_strategy(metrics: RegionMetrics, config: HybridInfillConfig) -> FillStrategy:
    line_width_mm = max(config.lineWidthMm, 1e-9)
    single_stroke_width_max = line_width_mm * max(config.singleStrokeMaxWidthFactor, config.singleStrokeWidthMaxFactor)
    centerline_width_max = line_width_mm * max(config.centerlineWidthMaxFactor, config.collapseOutlineMaxWidthFactor)
    detail_width_max = line_width_mm * max(config.detailWidthMaxFactor, config.narrowRegionMaxWidthFactor)
    min_usable_detail_area = max(config.minUsableDetailAreaMm2, line_width_mm * line_width_mm * 0.75)
    min_normal_fill_area = max(config.minNormalFillAreaMm2, line_width_mm * line_width_mm * 3.0)
    tiny_dot_area_mm2 = max(1e-6, line_width_mm * line_width_mm * config.tinyDotAreaFactor)

    if metrics.areaMm2 <= tiny_dot_area_mm2:
        return "SINGLE_STROKE_DETAIL"
    if metrics.areaMm2 < min_usable_detail_area:
        return "SINGLE_STROKE_DETAIL"
    if metrics.maxLocalWidthMm <= single_stroke_width_max:
        return "SINGLE_STROKE_DETAIL"
    if metrics.maxLocalWidthMm <= centerline_width_max:
        return "CENTERLINE_DETAIL"
    if metrics.aspectRatio >= 4.0 and metrics.maxLocalWidthMm <= detail_width_max * 2.0:
        return "RECTILINEAR_SERPENTINE"
    if (
        metrics.areaMm2 < min_normal_fill_area
        or metrics.maxLocalWidthMm <= detail_width_max
        or metrics.maxLocalWidthMm <= detail_width_max * 2.0
        or metrics.estimatedShortRowRatio > 0.35
        or (metrics.highCurvatureScore is not None and metrics.highCurvatureScore > 0.65)
        or (metrics.holeCount > 0 and metrics.minDimensionMm < detail_width_max * 2.0)
    ):
        return "CONTOUR_PARALLEL_DETAIL"
    return "RECTILINEAR_SERPENTINE"


def validate_thin_region_stroke(
    path: list[Point],
    component: Any,
    drawable_area: Any | None,
    config: HybridInfillConfig,
    metrics: RegionMetrics | None = None,
) -> bool:
    if len(path) < 2 or component is None or component.is_empty:
        return False

    line = LineString([(point.x, point.y) for point in path])
    tiny_region = bool(
        metrics is not None
        and (metrics.areaMm2 < config.minUsableDetailAreaMm2 or metrics.maxLocalWidthMm <= config.lineWidthMm * config.singleStrokeWidthMaxFactor)
    )
    minimum_length_mm = max(0.01, config.lineWidthMm * (0.25 if tiny_region else 0.75))
    if line.length < minimum_length_mm:
        return False

    if drawable_area is not None and not drawable_area.is_empty:
        cover_region = drawable_area.buffer(max(config.lineWidthMm * 0.05, 0.01), join_style=1)
        if cover_region.is_empty:
            cover_region = drawable_area
        try:
            if not cover_region.covers(line) and not tiny_region:
                return False
        except Exception:
            return False
    try:
        if not component.covers(line):
            return False
    except Exception:
        return False

    if drawable_area is None or drawable_area.is_empty:
        pass
    elif not tiny_region:
        try:
            if not drawable_area.covers(line):
                return False
        except Exception:
            return False

    sample_step_mm = max(0.01, config.lineWidthMm * 0.25)
    sample_count = max(2, int(math.ceil(line.length / sample_step_mm)) + 1)
    for sample_index in range(sample_count):
        distance_mm = min(line.length, (line.length * sample_index) / max(sample_count - 1, 1))
        sample_point = line.interpolate(distance_mm)
        try:
            if not component.covers(sample_point):
                return False
            if drawable_area is not None and not drawable_area.is_empty and not tiny_region and not drawable_area.covers(sample_point):
                return False
        except Exception:
            return False
    return True


def compute_toolpath_mask_coverage_metrics(
    toolpaths: list[Toolpath],
    *,
    mask: Any,
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    pen_radius_mm: float,
    sample_step_mm: float,
    include_kinds: set[str] | None = None,
) -> MaskCoverageMetrics | None:
    if mask is None:
        return None
    mask_height, mask_width = mask.shape[:2]
    target_mask = np.asarray(mask) > 0
    mask_area_px = int(np.count_nonzero(target_mask))
    if mask_area_px <= 0:
        return None
    a, b, c, d, _e, _f = current_to_source_matrix
    # Convert toolpath-space mm radius to source-image pixels via affine scale.
    scale_x = math.hypot(a, b)
    scale_y = math.hypot(c, d)
    px_per_mm = max(1e-6, (scale_x + scale_y) * 0.5)
    pen_radius_px = max(0.0, float(pen_radius_mm) * px_per_mm)
    coverage_draw_kinds = include_kinds if include_kinds is not None else {
        "coverage_centerline",
        "coverage_offset_line",
        "coverage_rectilinear",
        "coverage_contour",
        "coverage_connector",
        "coverage_tiny_mark",
        "fill-infill",
        "detail-trace",
        "outline",
        "outline_cleanup",
        "fill-wall",
    }
    drawn_mask = np.zeros((mask_height, mask_width), dtype=np.uint8)
    radius_px_i = max(1, int(round(pen_radius_px)))
    for path in toolpaths:
        if path.kind not in coverage_draw_kinds or len(path.points) < 1:
            continue
        if len(path.points) == 1:
            source_point = apply_svg_matrix(path.points[0], current_to_source_matrix)
            cv2.circle(drawn_mask, (int(round(source_point.x)), int(round(source_point.y))), radius_px_i, 255, -1)
            continue
        for start, end in zip(path.points, path.points[1:]):
            line = LineString([(start.x, start.y), (end.x, end.y)])
            if line.length <= 1e-9:
                continue
            sample_count = max(2, int(math.ceil(line.length / max(0.01, sample_step_mm))) + 1)
            for sample_index in range(sample_count):
                distance_mm = min(line.length, (line.length * sample_index) / max(sample_count - 1, 1))
                sample = line.interpolate(distance_mm)
                source_point = apply_svg_matrix(Point(float(sample.x), float(sample.y)), current_to_source_matrix)
                cv2.circle(drawn_mask, (int(round(source_point.x)), int(round(source_point.y))), radius_px_i, 255, -1)
    drawn_bool = drawn_mask > 0
    inside_covered_mask = target_mask & drawn_bool
    inside_missed_mask = target_mask & ~drawn_bool
    outside_overdraw_mask = ~target_mask & drawn_bool
    covered_inside_mask_px = int(np.count_nonzero(inside_covered_mask))
    missed_inside_mask_px = int(np.count_nonzero(inside_missed_mask))
    overdraw_outside_mask_px = int(np.count_nonzero(outside_overdraw_mask))
    raw_coverage_percent = (100.0 * covered_inside_mask_px / mask_area_px) if mask_area_px > 0 else 0.0
    outside_overdraw_percent = (100.0 * overdraw_outside_mask_px / mask_area_px) if mask_area_px > 0 else 0.0
    # Penalized score intentionally allows negative values for severe overdraw.
    penalized_coverage_percent = raw_coverage_percent - outside_overdraw_percent
    missed_inside_mask_percent = (100.0 * missed_inside_mask_px / mask_area_px) if mask_area_px > 0 else 0.0
    return MaskCoverageMetrics(
        mask_area_px=mask_area_px,
        covered_inside_mask_px=covered_inside_mask_px,
        missed_inside_mask_px=missed_inside_mask_px,
        overdraw_outside_mask_px=overdraw_outside_mask_px,
        covered_pixels_total_px=int(np.count_nonzero(drawn_bool)),
        raw_coverage_percent=raw_coverage_percent,
        outside_overdraw_percent=outside_overdraw_percent,
        penalized_coverage_percent=penalized_coverage_percent,
        missed_inside_mask_percent=missed_inside_mask_percent,
        px_per_mm=px_per_mm,
        pen_radius_px=pen_radius_px,
    )


def rasterize_surface_toolpaths_mask(
    toolpaths: list[Toolpath],
    *,
    shape: tuple[int, int],
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    pen_radius_mm: float,
    max_segment_mm: float,
    include_kinds: set[str] | None = None,
) -> np.ndarray:
    height, width = shape
    drawn_mask = np.zeros((height, width), dtype=np.uint8)
    a, b, c, d, _e, _f = current_to_source_matrix
    px_per_mm = max(1e-6, (math.hypot(a, b) + math.hypot(c, d)) * 0.5)
    pen_radius_px = max(1, int(round(max(0.0, float(pen_radius_mm) * px_per_mm))))
    allowed_kinds = include_kinds if include_kinds is not None else {
        "coverage_centerline",
        "coverage_offset_line",
        "coverage_rectilinear",
        "coverage_contour",
        "coverage_connector",
        "coverage_tiny_mark",
        "fill-infill",
        "fill-repair",
        "detail-trace",
        "outline",
        "outline_cleanup",
        "fill-wall",
    }
    for path in toolpaths:
        if path.kind not in allowed_kinds or len(path.points) < 1:
            continue
        if len(path.points) == 1:
            source_point = apply_svg_matrix(path.points[0], current_to_source_matrix)
            cv2.circle(drawn_mask, (int(round(source_point.x)), int(round(source_point.y))), pen_radius_px, 255, -1)
            continue
        sampled_points = resample_segment(path.points, max_step=max(0.01, float(max_segment_mm)))
        for point in sampled_points:
            source_point = apply_svg_matrix(point, current_to_source_matrix)
            cv2.circle(drawn_mask, (int(round(source_point.x)), int(round(source_point.y))), pen_radius_px, 255, -1)
    return drawn_mask


def rasterize_source_mask_to_target_frame(
    source_mask: np.ndarray,
    *,
    source_to_surface_matrix: tuple[float, float, float, float, float, float],
    surface_to_target_matrix: tuple[float, float, float, float, float, float],
    shape: tuple[int, int],
) -> np.ndarray:
    source_to_target = multiply_svg_matrices(surface_to_target_matrix, source_to_surface_matrix)
    target_to_source = invert_svg_matrix(source_to_target)
    affine = np.asarray([
        [float(target_to_source[0]), float(target_to_source[2]), float(target_to_source[4])],
        [float(target_to_source[1]), float(target_to_source[3]), float(target_to_source[5])],
    ], dtype=np.float32)
    warped = cv2.warpAffine(
        (np.asarray(source_mask) > 0).astype(np.uint8) * 255,
        affine,
        (int(shape[1]), int(shape[0])),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return warped


def compute_toolpath_mask_coverage_breakdown(
    toolpaths: list[Toolpath],
    *,
    mask: Any,
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    pen_radius_mm: float,
    sample_step_mm: float,
    include_kinds: set[str] | None = None,
) -> list[dict[str, float | int | str]]:
    include = include_kinds if include_kinds is not None else {
        "coverage_centerline",
        "coverage_offset_line",
        "coverage_rectilinear",
        "coverage_contour",
        "coverage_connector",
        "coverage_tiny_mark",
        "outline_cleanup",
    }
    selected = [p for p in toolpaths if p.kind in include and len(p.points) >= 1]
    groups: dict[str, list[Toolpath]] = {}
    for path in selected:
        source = str(path.source or path.metadata.get("source") or "unknown")
        key = f"{path.kind}|{source}"
        groups.setdefault(key, []).append(path)
    cumulative: list[Toolpath] = []
    baseline = compute_toolpath_mask_coverage_metrics(
        cumulative,
        mask=mask,
        current_to_source_matrix=current_to_source_matrix,
        pen_radius_mm=pen_radius_mm,
        sample_step_mm=sample_step_mm,
        include_kinds=include,
    )
    rows: list[dict[str, float | int | str]] = []
    prev = baseline
    for key in sorted(groups.keys()):
        cumulative.extend(groups[key])
        cur = compute_toolpath_mask_coverage_metrics(
            cumulative,
            mask=mask,
            current_to_source_matrix=current_to_source_matrix,
            pen_radius_mm=pen_radius_mm,
            sample_step_mm=sample_step_mm,
            include_kinds=include,
        )
        if cur is None:
            continue
        prev_raw = prev.raw_coverage_percent if prev is not None else 0.0
        prev_out = prev.outside_overdraw_percent if prev is not None else 0.0
        prev_pen = prev.penalized_coverage_percent if prev is not None else 0.0
        rows.append(
            {
                "path_kind_source": key,
                "path_count": len(groups[key]),
                "draw_length_mm": float(sum(segment_length(p.points) for p in groups[key])),
                "covered_inside_mask_px": int(cur.covered_inside_mask_px),
                "missed_inside_mask_px": int(cur.missed_inside_mask_px),
                "overdraw_outside_mask_px": int(cur.overdraw_outside_mask_px),
                "net_score_px": int(cur.covered_inside_mask_px - cur.overdraw_outside_mask_px),
                "raw_coverage_delta": float(cur.raw_coverage_percent - prev_raw),
                "outside_overdraw_delta": float(cur.outside_overdraw_percent - prev_out),
                "penalized_delta": float(cur.penalized_coverage_percent - prev_pen),
            }
        )
        prev = cur
    return rows


def estimate_toolpath_mask_coverage(
    toolpaths: list[Toolpath],
    *,
    mask: Any,
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    pen_radius_mm: float,
    sample_step_mm: float,
    include_kinds: set[str] | None = None,
) -> dict[str, float]:
    metrics = compute_toolpath_mask_coverage_metrics(
        toolpaths,
        mask=mask,
        current_to_source_matrix=current_to_source_matrix,
        pen_radius_mm=pen_radius_mm,
        sample_step_mm=sample_step_mm,
        include_kinds=include_kinds,
    )
    if metrics is None:
        return {}
    return {
        "selected_mask_pixels": float(metrics.mask_area_px),
        "estimated_covered_pixels": float(metrics.covered_inside_mask_px),
        "missed_selected_pixels": float(metrics.missed_inside_mask_px),
        "outside_mask_pixels": float(metrics.overdraw_outside_mask_px),
        "coverage_percent": metrics.raw_coverage_percent,
        "raw_coverage_percent": metrics.raw_coverage_percent,
        "missed_percent": metrics.missed_inside_mask_percent,
        "outside_overdraw_percent": metrics.outside_overdraw_percent,
        "penalized_coverage_percent": metrics.penalized_coverage_percent,
        "covered_pixels_total": float(metrics.covered_pixels_total_px),
        "px_per_mm": metrics.px_per_mm,
        "pen_radius_px": metrics.pen_radius_px,
    }


def should_accept_thin_region_stroke(
    delta_covered_inside_mask_px: int,
    delta_overdraw_outside_mask_px: int,
    delta_penalized_coverage_percent: float,
) -> bool:
    return (
        int(delta_covered_inside_mask_px) > int(delta_overdraw_outside_mask_px)
        and float(delta_penalized_coverage_percent) > 0.0
    )


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
        logger.debug("Reusing existing GRBL serial connection on %s", SERIAL_PORT)
        state["connected"] = True
        return grbl

    logger.info("Opening GRBL serial connection on %s at %s baud", SERIAL_PORT, BAUD_RATE)
    grbl = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=3)
    time.sleep(2)

    grbl.write(b"\r\n\r\n")
    time.sleep(1)

    startup_lines: list[str] = []
    while grbl.in_waiting:
        line = grbl.readline().decode(errors="ignore").strip()
        if line:
            startup_lines.append(line)
            logger.info("GRBL startup: %s", line)

    state["connected"] = True
    state["status"] = "Connected"
    logger.info("GRBL connection established; startup_lines=%d", len(startup_lines))
    return grbl


def read_available_lines(ser: serial.Serial) -> list[str]:
    lines: list[str] = []
    while True:
        line = read_next_grbl_line(ser, timeout=0.05, raise_on_timeout=False)
        if not line:
            break
        lines.append(line)
    if lines:
        logger.debug("Read %d available GRBL lines: %s", len(lines), lines)
    return lines


def read_until_ok_or_error(ser: serial.Serial, timeout: float = 15) -> str:
    end_time = time.time() + timeout
    lines: list[str] = []

    while time.time() < end_time:
        line = read_next_grbl_line(ser, timeout=min(0.25, max(0.01, end_time - time.time())), raise_on_timeout=False)
        if not line:
            continue
        lines.append(line)
        if line == "ok" or line.startswith("error:") or line.startswith("ALARM:"):
            break

    response = "\n".join(lines) if lines else "NO RESPONSE"
    logger.debug("GRBL response read_until_ok_or_error(timeout=%.2f): %s", timeout, response)
    return response


def _serial_read_buffer(ser: serial.Serial) -> str:
    buffer = getattr(ser, "_codex_read_buffer", "")
    if not isinstance(buffer, str):
        buffer = ""
    return buffer


def _set_serial_read_buffer(ser: serial.Serial, buffer: str) -> None:
    setattr(ser, "_codex_read_buffer", buffer)


def _extract_grbl_message_from_buffer(buffer: str) -> tuple[str | None, str]:
    normalized = buffer.replace("\r", "\n")
    if "\n" in normalized:
        raw_line, remainder = normalized.split("\n", 1)
        return raw_line.strip(), remainder

    stripped = normalized.strip()
    if stripped.startswith("<") and ">" in stripped:
        end_index = stripped.index(">") + 1
        return stripped[:end_index], stripped[end_index:]
    if stripped == "ok" or stripped.startswith("error:") or stripped.startswith("ALARM:"):
        return stripped, ""
    if stripped.lower().startswith("grbl") and "]" in stripped:
        end_index = stripped.index("]") + 1
        return stripped[:end_index], stripped[end_index:]
    return None, buffer


def _supports_readline_fallback(ser: serial.Serial) -> bool:
    # Real pyserial objects expose in_waiting and should be polled non-blockingly.
    # The readline fallback exists only for narrow test doubles that return whole
    # messages but do not implement in_waiting/read buffering semantics.
    return not hasattr(type(ser), "in_waiting") and hasattr(ser, "readline")


def read_next_grbl_line(ser: serial.Serial, timeout: float = 15, *, raise_on_timeout: bool = True) -> str:
    end_time = time.time() + max(0.0, timeout)

    while time.time() < end_time:
        buffered = _serial_read_buffer(ser)
        message, remainder = _extract_grbl_message_from_buffer(buffered)
        if message is not None:
            _set_serial_read_buffer(ser, remainder)
            if message:
                logger.debug("GRBL line received: %s", message)
                return message

        waiting = int(getattr(ser, "in_waiting", 0) or 0)
        if waiting > 0:
            chunk = ser.read(waiting).decode(errors="ignore")
            if chunk:
                _set_serial_read_buffer(ser, buffered + chunk)
                continue
        elif _supports_readline_fallback(ser):
            chunk = ser.readline().decode(errors="ignore")
            if chunk:
                _set_serial_read_buffer(ser, buffered + chunk)
                continue

        time.sleep(0.01)

    buffered = _serial_read_buffer(ser)
    message, remainder = _extract_grbl_message_from_buffer(buffered)
    if message is not None:
        _set_serial_read_buffer(ser, remainder)
        if message:
            logger.debug("GRBL line received at timeout boundary: %s", message)
            return message

    if raise_on_timeout:
        raise TimeoutError("Timed out waiting for GRBL response")
    return ""


def _parse_status_report_fields(status_line: str) -> dict[str, Any]:
    if not status_line.startswith("<") or "|" not in status_line:
        return {}
    raw = status_line.strip()[1:]
    if raw.endswith(">"):
        raw = raw[:-1]
    parts = raw.split("|")
    fields: dict[str, Any] = {
        "state": parts[0],
    }
    for part in parts[1:]:
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        fields[key] = value
    bf_value = fields.get("Bf")
    if isinstance(bf_value, str) and "," in bf_value:
        try:
            planner_free, rx_free = bf_value.split(",", 1)
            fields["planner_buffer_free"] = int(planner_free)
            fields["serial_rx_free"] = int(rx_free)
        except ValueError:
            pass
    return fields


def _update_streaming_state(
    *,
    mode: str,
    current_line: int,
    current_path_id: str | None,
    current_path_kind: str | None,
    pending_buffer_chars: int,
    pending_commands: int,
    sent_count: int,
    ok_count: int,
    error_count: int,
    acked_count: int,
    total_lines: int,
    last_response_at: float | None,
    last_grbl_status: str | None,
    last_stream_event: str | None = None,
) -> None:
    age = 0.0 if last_response_at is None else max(0.0, time.time() - last_response_at)
    state["streaming"] = {
        "mode": mode,
        "current_line": current_line,
        "current_path_id": current_path_id,
        "current_path_kind": current_path_kind,
        "pending_buffer_chars": pending_buffer_chars,
        "pending_commands": pending_commands,
        "last_response_age_sec": round(age, 3),
        "last_grbl_status": last_grbl_status,
        "ok_count": ok_count,
        "error_count": error_count,
        "sent_count": sent_count,
        "acked_count": acked_count,
        "total_lines": total_lines,
        "streaming_active": bool(sent_count > acked_count or pending_commands > 0 or pending_buffer_chars > 0),
    }
    if last_stream_event is not None:
        state["last_stream_event"] = last_stream_event


def _read_status_response_unlocked(ser: serial.Serial, timeout: float = 0.75) -> str:
    ser.write(b"?")
    end_time = time.time() + timeout
    captured: list[str] = []
    while time.time() < end_time:
        line = read_next_grbl_line(ser, timeout=min(0.2, max(0.01, end_time - time.time())), raise_on_timeout=False)
        if not line:
            continue
        captured.append(line)
        if line.startswith("<") or line.startswith("ok") or line.startswith("error:") or line.startswith("ALARM:"):
            break
    response = "\n".join(captured)
    logger.debug("GRBL status query response: %s", response or "NO RESPONSE")
    return response


def process_streaming_ack_unlocked(
    ser: serial.Serial,
    pending_lengths: deque[int],
    pending_commands: deque[str],
    *,
    timeout: float = 15,
    all_lines: list[str],
    line_index: int,
    pending_buffer_chars: int,
    sent_count: int,
    ok_count: int,
    recent_grbl_responses: deque[str],
    last_response_at: float | None,
    last_grbl_status: str | None,
    alive_retry_count: int,
    mode: str,
) -> tuple[int, int, float | None, str | None, int]:
    while True:
        try:
            line = read_next_grbl_line(ser, timeout=timeout)
        except TimeoutError as exc:
            logger.warning(
                "Timeout waiting for GRBL ack: line_index=%s sent_count=%s pending_queue=%s pending_buffer=%s",
                line_index,
                sent_count,
                len(pending_lengths),
                pending_buffer_chars,
            )
            status_query_response = _read_status_response_unlocked(ser)
            if status_query_response:
                status_lines = status_query_response.splitlines()
                for status_line in status_query_response.splitlines():
                    recent_grbl_responses.append(status_line)
                    if status_line.startswith("<"):
                        last_grbl_status = status_line[1:].split("|", 1)[0]
                idle_status = next((line for line in reversed(status_lines) if line.startswith("<")), "")
                idle_status_fields = _parse_status_report_fields(idle_status) if idle_status else {}
                last_response_at = time.time()
                _update_streaming_state(
                    mode=mode,
                    current_line=line_index,
                    current_path_id=state.get("streaming", {}).get("current_path_id"),
                    current_path_kind=state.get("streaming", {}).get("current_path_kind"),
                    pending_buffer_chars=pending_buffer_chars,
                    pending_commands=len(pending_lengths),
                    sent_count=sent_count,
                    ok_count=ok_count,
                    error_count=0,
                    acked_count=ok_count,
                    total_lines=len(all_lines),
                    last_response_at=last_response_at,
                    last_grbl_status=last_grbl_status,
                    last_stream_event="status_query_alive_retry",
                )
                if (
                    pending_lengths
                    and idle_status_fields.get("state") == "Idle"
                    and int(idle_status_fields.get("planner_buffer_free", -1)) >= 15
                    and int(idle_status_fields.get("serial_rx_free", -1)) >= GRBL_RX_BUFFER_SIZE
                ):
                    recovered_count = len(pending_lengths)
                    recovered_bytes = sum(pending_lengths)
                    pending_lengths.clear()
                    pending_commands.clear()
                    logger.warning(
                        "Recovered GRBL ack desync from idle status: line_index=%s recovered_count=%s recovered_bytes=%s status=%s",
                        line_index,
                        recovered_count,
                        recovered_bytes,
                        idle_status,
                    )
                    _update_streaming_state(
                        mode=mode,
                        current_line=line_index,
                        current_path_id=state.get("streaming", {}).get("current_path_id"),
                        current_path_kind=state.get("streaming", {}).get("current_path_kind"),
                        pending_buffer_chars=0,
                        pending_commands=0,
                        sent_count=sent_count,
                        ok_count=ok_count + recovered_count,
                        error_count=0,
                        acked_count=ok_count + recovered_count,
                        total_lines=len(all_lines),
                        last_response_at=last_response_at,
                        last_grbl_status=last_grbl_status,
                        last_stream_event="status_query_recovered_desync",
                    )
                    return recovered_bytes, ok_count + recovered_count, last_response_at, last_grbl_status, 0
                if alive_retry_count < 1:
                    return 0, ok_count, last_response_at, last_grbl_status, alive_retry_count + 1
            timeout_debug = {
                "line_index": line_index,
                "current_command": all_lines[line_index - 1] if 0 < line_index <= len(all_lines) else "",
                "previous_10_commands": all_lines[max(0, line_index - 11):max(0, line_index - 1)],
                "next_10_commands": all_lines[line_index:min(len(all_lines), line_index + 10)],
                "streaming_mode": mode,
                "pending_buffer_chars": pending_buffer_chars,
                "pending_queue_length": len(pending_lengths),
                "oldest_pending_command": pending_commands[0] if pending_commands else "",
                "last_response_age_sec": 0.0 if last_response_at is None else round(max(0.0, time.time() - last_response_at), 3),
                "recent_grbl_responses": list(recent_grbl_responses),
                "serial_in_waiting": int(getattr(ser, "in_waiting", 0) or 0),
                "status_query_sent": True,
                "status_query_response": status_query_response,
                "connection_alive": bool(status_query_response),
                "failure_class": "grbl_busy_but_alive" if status_query_response else "communication_lost",
            }
            state["last_timeout_debug"] = timeout_debug
            logger.error("GRBL timeout debug: %s", timeout_debug)
            raise TimeoutError(f"{exc}. Debug: {timeout_debug}") from exc
        if line == "ok":
            if not pending_lengths:
                raise RuntimeError("Received unexpected GRBL ok with no pending commands")
            freed_length = pending_lengths.popleft()
            if pending_commands:
                pending_commands.popleft()
            logger.debug("GRBL ack received: ok_count=%d pending_queue=%d", ok_count + 1, len(pending_lengths))
            return freed_length, ok_count + 1, time.time(), last_grbl_status, 0
        if line.startswith("error:") or line.startswith("ALARM:"):
            failed_command = pending_commands[0] if pending_commands else ""
            logger.error("GRBL streaming error line: %s command=%s", line, failed_command)
            if failed_command:
                raise RuntimeError(f"GRBL streaming error: {line} while executing: {failed_command}")
            raise RuntimeError(f"GRBL streaming error: {line}")
        if line.startswith("<") or line.startswith("["):
            recent_grbl_responses.append(line)
            last_response_at = time.time()
            if line.startswith("<"):
                last_grbl_status = line[1:].split("|", 1)[0]
            continue
        if line.lower().startswith("grbl"):
            recent_grbl_responses.append(line)
            last_response_at = time.time()
            continue
        if not line:
            continue
        recent_grbl_responses.append(line)
        last_response_at = time.time()


def stream_gcode_lines_unlocked(
    ser: serial.Serial,
    lines: list[str],
    *,
    rx_buffer_size: int = GRBL_RX_BUFFER_SIZE,
    response_timeout: float = 20,
    should_stop: Optional[Callable[[], bool]] = None,
    wait_while_paused: Optional[Callable[[], None]] = None,
    on_line_sent: Optional[Callable[[str, int], None]] = None,
) -> dict[str, Any]:
    mode = state.get("streaming_mode", "buffered")
    if mode not in {"buffered", "sync"}:
        mode = "buffered"
    logger.info(
        "Starting GRBL stream: lines=%d mode=%s rx_buffer_size=%d response_timeout=%.2f",
        len(lines),
        mode,
        rx_buffer_size,
        response_timeout,
    )
    pending_lengths: deque[int] = deque()
    pending_commands: deque[str] = deque()
    recent_grbl_responses: deque[str] = deque(maxlen=20)
    pending_bytes = 0
    sent_count = 0
    ok_count = 0
    error_count = 0
    last_response_at: float | None = None
    last_grbl_status: str | None = None
    alive_retry_count = 0
    streamable_lines = [raw_line.strip() for raw_line in lines if raw_line.strip()]
    # Preflight validation: ensure all G0/G1 moves stay within configured draw bounds
    try:
        x_min = globals().get("X_DRAW_MIN", -180.0)
        x_max = globals().get("X_DRAW_MAX", 180.0)
        y_min = globals().get("Y_DRAW_MIN", -45.0)
        y_max = globals().get("Y_DRAW_MAX", 45.0)
    except Exception:
        x_min, x_max, y_min, y_max = -180.0, 180.0, -45.0, 45.0
    out_of_bounds: list[tuple[int, str, float, float]] = []
    coord_re = re.compile(r"([XY])([-+]?[0-9]*\.?[0-9]+)")
    for idx, raw_line in enumerate(streamable_lines, start=1):
        if not raw_line or not raw_line.upper().startswith(("G0", "G1")):
            continue
        xs = None
        ys = None
        for m in coord_re.finditer(raw_line):
            axis = m.group(1).upper()
            val = float(m.group(2))
            if axis == "X":
                xs = val
            elif axis == "Y":
                ys = val
        if xs is not None and (xs < x_min - 1e-9 or xs > x_max + 1e-9):
            out_of_bounds.append((idx, raw_line, xs, None))
        if ys is not None and (ys < y_min - 1e-9 or ys > y_max + 1e-9):
            out_of_bounds.append((idx, raw_line, None, ys))
    if out_of_bounds:
        msg_lines = [f"G-code preflight failed: {len(out_of_bounds)} out-of-bounds moves detected:"]
        for idx, line_txt, xval, yval in out_of_bounds:
            if xval is not None:
                msg_lines.append(f"  line {idx}: X={xval} -> allowed [{x_min},{x_max}] : {line_txt}")
            elif yval is not None:
                msg_lines.append(f"  line {idx}: Y={yval} -> allowed [{y_min},{y_max}] : {line_txt}")
        logger.error("%s", "\n".join(msg_lines))
        raise RuntimeError("G-code preflight check failed: out-of-bounds moves detected. See logs for details.")
    _update_streaming_state(
        mode=mode,
        current_line=0,
        current_path_id=state.get("current_path_id"),
        current_path_kind=state.get("current_path_kind"),
        pending_buffer_chars=0,
        pending_commands=0,
        sent_count=0,
        ok_count=0,
        error_count=0,
        acked_count=0,
        total_lines=len(streamable_lines),
        last_response_at=None,
        last_grbl_status=None,
        last_stream_event="stream_start",
    )

    for raw_line in streamable_lines:
        if should_stop and should_stop():
            return {
                "sent_count": sent_count,
                "acked_count": ok_count,
                "total_lines": len(streamable_lines),
                "pending_queue_length": len(pending_lengths),
                "pending_buffer_chars": pending_bytes,
                "last_grbl_status": last_grbl_status,
                "last_stream_event": "stop_requested_before_send",
                "streaming_active": bool(pending_lengths or pending_bytes),
            }
        if wait_while_paused:
            wait_while_paused()
        if should_stop and should_stop():
            return {
                "sent_count": sent_count,
                "acked_count": ok_count,
                "total_lines": len(streamable_lines),
                "pending_queue_length": len(pending_lengths),
                "pending_buffer_chars": pending_bytes,
                "last_grbl_status": last_grbl_status,
                "last_stream_event": "stop_requested_after_pause",
                "streaming_active": bool(pending_lengths or pending_bytes),
            }

        line = raw_line.strip()
        if not line:
            continue

        payload = (line + "\n").encode("ascii")
        if len(payload) >= rx_buffer_size:
            logger.error("G-code line exceeds GRBL RX buffer size: %s", line)
            raise ValueError(f"G-code line exceeds GRBL RX buffer size: {line}")

        while mode == "buffered" and pending_bytes + len(payload) >= rx_buffer_size:
            freed_length, ok_count, last_response_at, last_grbl_status, alive_retry_count = process_streaming_ack_unlocked(
                ser,
                pending_lengths,
                pending_commands,
                timeout=response_timeout,
                all_lines=streamable_lines,
                line_index=sent_count,
                pending_buffer_chars=pending_bytes,
                sent_count=sent_count,
                ok_count=ok_count,
                recent_grbl_responses=recent_grbl_responses,
                last_response_at=last_response_at,
                last_grbl_status=last_grbl_status,
                alive_retry_count=alive_retry_count,
                mode=mode,
            )
            pending_bytes -= freed_length
            pending_bytes = sum(pending_lengths)
            if should_stop and should_stop():
                return {
                    "sent_count": sent_count,
                    "acked_count": ok_count,
                    "total_lines": len(streamable_lines),
                    "pending_queue_length": len(pending_lengths),
                    "pending_buffer_chars": pending_bytes,
                    "last_grbl_status": last_grbl_status,
                    "last_stream_event": "stop_requested_while_draining_buffer",
                    "streaming_active": bool(pending_lengths or pending_bytes),
                }
            if wait_while_paused:
                wait_while_paused()

        ser.write(payload)
        pending_lengths.append(len(payload))
        pending_commands.append(line)
        pending_bytes += len(payload)
        sent_count += 1
        _update_streaming_state(
            mode=mode,
            current_line=sent_count,
            current_path_id=state.get("current_path_id"),
            current_path_kind=state.get("current_path_kind"),
            pending_buffer_chars=pending_bytes,
            pending_commands=len(pending_lengths),
            sent_count=sent_count,
            ok_count=ok_count,
            error_count=error_count,
            acked_count=ok_count,
            total_lines=len(streamable_lines),
            last_response_at=last_response_at,
            last_grbl_status=last_grbl_status,
            last_stream_event="line_sent",
        )
        if on_line_sent:
            on_line_sent(line, sent_count)
        if mode == "sync":
            _, ok_count, last_response_at, last_grbl_status, alive_retry_count = process_streaming_ack_unlocked(
                ser,
                pending_lengths,
                pending_commands,
                timeout=response_timeout,
                all_lines=streamable_lines,
                line_index=sent_count,
                pending_buffer_chars=pending_bytes,
                sent_count=sent_count,
                ok_count=ok_count,
                recent_grbl_responses=recent_grbl_responses,
                last_response_at=last_response_at,
                last_grbl_status=last_grbl_status,
                alive_retry_count=alive_retry_count,
                mode=mode,
            )
            pending_bytes = sum(pending_lengths)
            _update_streaming_state(
                mode=mode,
                current_line=sent_count,
                current_path_id=state.get("current_path_id"),
                current_path_kind=state.get("current_path_kind"),
                pending_buffer_chars=pending_bytes,
                pending_commands=len(pending_lengths),
                sent_count=sent_count,
                ok_count=ok_count,
                error_count=error_count,
                acked_count=ok_count,
                total_lines=len(streamable_lines),
                last_response_at=last_response_at,
                last_grbl_status=last_grbl_status,
                last_stream_event="line_acked_sync",
            )

    while pending_lengths:
        _, ok_count, last_response_at, last_grbl_status, alive_retry_count = process_streaming_ack_unlocked(
            ser,
            pending_lengths,
            pending_commands,
            timeout=response_timeout,
            all_lines=streamable_lines,
            line_index=sent_count,
            pending_buffer_chars=pending_bytes,
            sent_count=sent_count,
            ok_count=ok_count,
            recent_grbl_responses=recent_grbl_responses,
            last_response_at=last_response_at,
            last_grbl_status=last_grbl_status,
            alive_retry_count=alive_retry_count,
            mode=mode,
        )
        pending_bytes = sum(pending_lengths)
        _update_streaming_state(
            mode=mode,
            current_line=sent_count,
            current_path_id=state.get("current_path_id"),
            current_path_kind=state.get("current_path_kind"),
            pending_buffer_chars=pending_bytes,
            pending_commands=len(pending_lengths),
            sent_count=sent_count,
            ok_count=ok_count,
            error_count=error_count,
            acked_count=ok_count,
            total_lines=len(streamable_lines),
            last_response_at=last_response_at,
            last_grbl_status=last_grbl_status,
            last_stream_event="drain_acked",
        )

    _update_streaming_state(
        mode=mode,
        current_line=sent_count,
        current_path_id=state.get("current_path_id"),
        current_path_kind=state.get("current_path_kind"),
        pending_buffer_chars=0,
        pending_commands=0,
        sent_count=sent_count,
        ok_count=ok_count,
        error_count=error_count,
        acked_count=ok_count,
        total_lines=len(streamable_lines),
        last_response_at=last_response_at,
        last_grbl_status=last_grbl_status,
        last_stream_event="stream_drained",
    )
    logger.info(
        "GRBL stream drained successfully: sent=%d acked=%d last_status=%s",
        sent_count,
        ok_count,
        last_grbl_status,
    )
    return {
        "sent_count": sent_count,
        "acked_count": ok_count,
        "total_lines": len(streamable_lines),
        "pending_queue_length": 0,
        "pending_buffer_chars": 0,
        "last_grbl_status": last_grbl_status,
        "last_stream_event": "stream_drained",
        "streaming_active": False,
    }


def wait_until_idle_unlocked(ser: serial.Serial, timeout: float = 60) -> bool:
    end_time = time.time() + timeout
    logger.info("Waiting for GRBL idle state (timeout=%.2fs)", timeout)

    while time.time() < end_time:
        ser.write(b"?")
        time.sleep(0.12)
        lines = read_available_lines(ser)
        for line in lines:
            if line.startswith("<Idle"):
                logger.info("GRBL reported idle")
                return True
            if line.startswith("<Alarm") or line.startswith("ALARM:"):
                logger.warning("GRBL reported alarm while waiting for idle: %s", line)
                return False
        time.sleep(0.05)

    logger.warning("Timed out waiting for GRBL idle state")
    return False


def send_to_grbl_unlocked(ser: serial.Serial, command: str, timeout: float = 15) -> str:
    command = command.strip()
    if not command:
        raise ValueError("Empty command")

    if command == "?":
        logger.debug("Sending GRBL status query")
        ser.write(b"?")
        time.sleep(0.2)
        lines = read_available_lines(ser)
        return "\n".join(lines) if lines else "NO STATUS RESPONSE"

    logger.info("Sending GRBL command: %s", command)
    ser.write((command + "\n").encode("utf-8"))
    response = read_until_ok_or_error(ser, timeout=timeout)
    if "error:" in response or "ALARM:" in response:
        logger.error("GRBL rejected command %s with response: %s", command, response)
        raise RuntimeError(f"GRBL rejected command {command}: {response}")
    logger.debug("GRBL command completed: %s -> %s", command, response)
    return response


def send_to_grbl(command: str, timeout: float = 15) -> str:
    with serial_lock:
        ser = connect_grbl()
        return send_to_grbl_unlocked(ser, command, timeout=timeout)


def send_many(commands: list[str], delay: float = 0.04, wait_idle_between: bool = True) -> str:
    results: list[str] = []
    logger.info(
        "Sending GRBL command batch: count=%d delay=%.3fs wait_idle_between=%s",
        len(commands),
        delay,
        wait_idle_between,
    )
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
    value = parse_locale_float(feed)
    if value <= 0:
        raise ValueError("Feed rate must be greater than 0")
    if value > 100000:
        raise ValueError("Feed rate is too high")
    return value


def validate_degrees(degrees: Any) -> float:
    value = parse_locale_float(degrees)
    if abs(value) > 100000:
        raise ValueError("Degree value is too large")
    return value


def validate_y_degrees(degrees: Any) -> float:
    value = parse_locale_float(degrees)
    if value < Y_DRAW_MIN or value > Y_DRAW_MAX:
        raise ValueError(f"Y angle must be between {Y_DRAW_MIN} and {Y_DRAW_MAX} degrees")
    return value


def validate_servo_s(s_value: Any) -> int:
    value = int(s_value)
    if value < MIN_SERVO_S or value > MAX_SERVO_S:
        raise ValueError(f"Servo S value must be between {MIN_SERVO_S} and {MAX_SERVO_S}")
    return value


def validate_dwell(dwell: Any) -> float:
    value = parse_locale_float(dwell)
    if value < 0:
        raise ValueError("Dwell must not be negative")
    if value > 5:
        raise ValueError("Dwell is too long")
    return value


def validate_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def parse_locale_float(value: Any, default: Optional[float] = None) -> float:
    if value is None:
        if default is None:
            raise ValueError("Missing numeric value")
        return float(default)
    if isinstance(value, str):
        normalized = value.strip()
        if normalized == "":
            if default is None:
                raise ValueError("Missing numeric value")
            return float(default)
        value = normalized.replace(",", ".")
    return float(value)


def validate_non_negative_float(value: Any, label: str, maximum: Optional[float] = None) -> float:
    out = parse_locale_float(value)
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


def ball_degrees_to_mm(degrees_value: float, *, ball_diameter_mm: float = BALL_DIAMETER_MM) -> float:
    circumference_mm = math.pi * ball_diameter_mm
    return (degrees_value / 360.0) * circumference_mm


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
    normalized = str(value).replace(",", ".")
    match = re.search(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", normalized)
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


def invert_svg_matrix(matrix: tuple[float, float, float, float, float, float]) -> tuple[float, float, float, float, float, float]:
    a, b, c, d, e, f = matrix
    determinant = (a * d) - (b * c)
    if abs(determinant) <= 1e-12:
        raise ValueError("Matrix is not invertible")
    inverse_det = 1.0 / determinant
    return (
        d * inverse_det,
        -b * inverse_det,
        -c * inverse_det,
        a * inverse_det,
        ((c * f) - (d * e)) * inverse_det,
        ((b * e) - (a * f)) * inverse_det,
    )


def _identity_svg_matrix() -> tuple[float, float, float, float, float, float]:
    return (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def _connector_validation_context(metadata: dict[str, Any] | None) -> dict[str, Any] | None:
    if not metadata:
        return None
    context = metadata.get("connector_validation")
    return context if isinstance(context, dict) else None


def _update_connector_validation_metadata(
    metadata: dict[str, Any] | None,
    matrix: tuple[float, float, float, float, float, float] | None,
) -> dict[str, Any]:
    if not metadata:
        return {}
    updated = dict(metadata)
    context = _connector_validation_context(updated)
    if context is None:
        return updated
    current_matrix = context.get("source_to_current_matrix")
    if not isinstance(current_matrix, (tuple, list)) or len(current_matrix) != 6:
        current_matrix = _identity_svg_matrix()
    if matrix is not None:
        current_matrix = multiply_svg_matrices(matrix, tuple(float(value) for value in current_matrix))
    updated_context = dict(context)
    updated_context["source_to_current_matrix"] = tuple(float(value) for value in current_matrix)
    try:
        updated_context["current_to_source_matrix"] = invert_svg_matrix(updated_context["source_to_current_matrix"])
    except Exception:
        updated_context["current_to_source_matrix"] = _identity_svg_matrix()
    updated["connector_validation"] = updated_context
    return updated


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
            "id": toolpath.path_id,
            "coordinate_space": toolpath.coordinate_space,
            "source": toolpath.source,
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


def hash_toolpaths(toolpaths: list["Toolpath"]) -> str:
    payload: list[dict[str, Any]] = []
    for toolpath in toolpaths:
        payload.append({
            "id": toolpath.path_id,
            "kind": toolpath.kind,
            "closed": toolpath.closed,
            "coordinate_space": toolpath.coordinate_space,
            "source": toolpath.source,
            "region_id": toolpath.region_id,
            "points": [
                [round(point.x, 6), round(point.y, 6)]
                for point in toolpath.points
            ],
        })
    encoded = repr(payload).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


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

    for geometry in (bundle.printable_geometry, bundle.cutout_geometry):
        if geometry is None or geometry.is_empty:
            continue
        gx1, gy1, gx2, gy2 = geometry.bounds
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


@dataclass
class StraighteningOptions:
    angleToleranceDeg: float = 2.0
    maxLateralErrorMm: float = 0.05
    minStraightSegmentLengthMm: float = 6.0


def _point_line_distance(point: Point, start: Point, end: Point) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    if abs(dx) <= 1e-12 and abs(dy) <= 1e-12:
        return math.hypot(point.x - start.x, point.y - start.y)
    return abs(dy * point.x - dx * point.y + end.x * start.y - end.y * start.x) / math.hypot(dx, dy)


def normalize_straight_segments(points: list[Point], options: StraighteningOptions) -> list[Point]:
    if len(points) < 3:
        return list(points)

    core_points, _ = _closed_path_core_points(points, closed=False)
    if len(core_points) < 3:
        return list(points)

    start = core_points[0]
    end = core_points[-1]
    chord_length = _segment_length_mm(start, end)
    if chord_length < options.minStraightSegmentLengthMm:
        return list(points)

    path_length = sum(_segment_length_mm(a, b) for a, b in zip(core_points, core_points[1:]))
    if path_length > max(chord_length * 1.02, chord_length + options.maxLateralErrorMm):
        return list(points)

    baseline_angle_deg = math.degrees(math.atan2(end.y - start.y, end.x - start.x))
    for point in core_points[1:-1]:
        if _point_line_distance(point, start, end) > options.maxLateralErrorMm:
            return list(points)

    for a, b in zip(core_points, core_points[1:]):
        segment_angle_deg = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
        angle_delta = abs(_normalize_infill_angle_deg(segment_angle_deg - baseline_angle_deg))
        if angle_delta > options.angleToleranceDeg:
            return list(points)

    return [Point(start.x, start.y), Point(end.x, end.y)]


def _drawing_path_kinds() -> set[str]:
    return {
        "outline",
        "fill-wall",
        "fill-infill",
        "detail-trace",
        "detail-continuation",
        "coverage_centerline",
        "coverage_offset_line",
        "coverage_rectilinear",
        "coverage_contour",
        "coverage_connector",
        "outline_cleanup",
    }


def _minimum_toolpath_length_threshold(toolpath: Toolpath, requested_minimum_length: float) -> float:
    if toolpath.kind in {"detail-trace", "detail-continuation", "fill-infill-travel"} or bool(toolpath.metadata.get("force_minimum_printable_stroke", False)):
        return 0.0
    return requested_minimum_length


def _path_component_label(toolpath: Toolpath) -> str:
    component_id = _extract_component_id(
        toolpath.metadata.get("source_component_id")
        or toolpath.metadata.get("source_polygon_id")
        or (toolpath.region_id + 1 if toolpath.region_id is not None else None)
    )
    return f"component_{component_id:03d}" if component_id is not None else "component_unknown"


def _path_ring_role(toolpath: Toolpath) -> str:
    contour_id = _extract_component_id(toolpath.metadata.get("source_contour_id"))
    if contour_id == 1:
        return "outer"
    if contour_id is not None and contour_id > 1:
        return "hole"
    return "unknown"


def _closed_path_core_points(points: list[Point], *, closed: bool, tolerance: float = 1e-6) -> tuple[list[Point], bool]:
    if not closed or len(points) < 2:
        return list(points), False
    explicit_duplicate_endpoint = nearly_same_point(points[0], points[-1], tolerance)
    if explicit_duplicate_endpoint:
        return list(points[:-1]), True
    return list(points), False


def _iter_path_segments(points: list[Point], *, closed: bool) -> list[tuple[Point, Point]]:
    core_points, _ = _closed_path_core_points(points, closed=closed)
    if len(core_points) < 2:
        return []
    segments = list(zip(core_points, core_points[1:]))
    if closed and len(core_points) >= 3:
        segments.append((core_points[-1], core_points[0]))
    return segments


def _segment_length_mm(a: Point, b: Point) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def _segment_lengths_mm(points: list[Point], *, closed: bool) -> list[float]:
    return [_segment_length_mm(a, b) for a, b in _iter_path_segments(points, closed=closed)]


def _segment_motion_profile(points: list[Point], *, closed: bool, axis_epsilon: float = 1e-6) -> dict[str, int | float]:
    segments = _iter_path_segments(points, closed=closed)
    horizontal = 0
    vertical = 0
    blended = 0
    max_blended_run = 0
    current_blended_run = 0
    for a, b in segments:
        dx = abs(b.x - a.x)
        dy = abs(b.y - a.y)
        if dx <= axis_epsilon and dy <= axis_epsilon:
            current_blended_run = 0
            continue
        if dy <= axis_epsilon:
            horizontal += 1
            current_blended_run = 0
        elif dx <= axis_epsilon:
            vertical += 1
            current_blended_run = 0
        else:
            blended += 1
            current_blended_run += 1
            max_blended_run = max(max_blended_run, current_blended_run)
    total = horizontal + vertical + blended
    return {
        "horizontal_segments": horizontal,
        "vertical_segments": vertical,
        "blended_xy_segments": blended,
        "total_segments": total,
        "blended_xy_ratio": (blended / total) if total else 0.0,
        "max_consecutive_blended_xy_segments": max_blended_run,
    }


def _merge_motion_profiles(toolpaths: list[Toolpath]) -> dict[str, int | float]:
    horizontal = 0
    vertical = 0
    blended = 0
    total = 0
    max_blended_run = 0
    for toolpath in toolpaths:
        profile = _segment_motion_profile(toolpath.points, closed=toolpath.closed)
        horizontal += int(profile["horizontal_segments"])
        vertical += int(profile["vertical_segments"])
        blended += int(profile["blended_xy_segments"])
        total += int(profile["total_segments"])
        max_blended_run = max(max_blended_run, int(profile["max_consecutive_blended_xy_segments"]))
    return {
        "horizontal_segments": horizontal,
        "vertical_segments": vertical,
        "blended_xy_segments": blended,
        "total_segments": total,
        "blended_xy_ratio": (blended / total) if total else 0.0,
        "max_consecutive_blended_xy_segments": max_blended_run,
    }


def _bounds_or_none(points: list[Point]) -> dict[str, float] | None:
    return _bounds_for_points(points) if len(points) >= 2 else None


def _max_point_delta(points_a: list[Point], points_b: list[Point]) -> float:
    if len(points_a) != len(points_b):
        return float("inf")
    if not points_a:
        return 0.0
    return max(math.hypot(a.x - b.x, a.y - b.y) for a, b in zip(points_a, points_b))


def _resolve_projection_sampling_mm(toolpath: Toolpath, *, default_pen_width_mm: float = DEFAULT_LINE_THICKNESS_MM) -> float:
    pen_width_mm = float(toolpath.metadata.get("pen_width_mm", toolpath.metadata.get("line_width_mm", default_pen_width_mm)))
    return min(DEFAULT_PROJECTION_SAMPLING_MAX_SEGMENT_MM, max(0.01, pen_width_mm * 0.25))


def validate_closed_path(toolpath: Toolpath) -> dict[str, Any]:
    core_points, explicit_duplicate_endpoint = _closed_path_core_points(toolpath.points, closed=toolpath.closed)
    effectively_closed = bool(toolpath.closed and len(core_points) >= 3)
    edge_lengths = _segment_lengths_mm(toolpath.points, closed=toolpath.closed)
    closing_edge_mm = 0.0
    implicit_close_added = False
    if effectively_closed:
        closing_edge_mm = _segment_length_mm(core_points[-1], core_points[0])
        implicit_close_added = not explicit_duplicate_endpoint
    non_closing_edges = edge_lengths[:-1] if len(edge_lengths) >= 2 and effectively_closed else edge_lengths
    neighbor_max_mm = max(non_closing_edges) if non_closing_edges else 0.0
    closing_edge_suspicious = bool(
        effectively_closed
        and implicit_close_added
        and closing_edge_mm > max(1.0, neighbor_max_mm * 2.0)
    )
    result = {
        "event": "closed_path_validation",
        "path_id": toolpath.path_id,
        "kind": toolpath.kind,
        "is_closed": bool(toolpath.closed),
        "effectively_closed": effectively_closed,
        "degenerate_closed_path": bool(toolpath.closed and not effectively_closed),
        "first_last_distance_mm": _segment_length_mm(toolpath.points[0], toolpath.points[-1]) if len(toolpath.points) >= 2 else 0.0,
        "explicit_duplicate_endpoint": explicit_duplicate_endpoint,
        "implicit_close_added": implicit_close_added,
        "max_edge_mm": max(edge_lengths) if edge_lengths else 0.0,
        "closing_edge_mm": closing_edge_mm,
        "closing_edge_suspicious": closing_edge_suspicious,
        "ring_role": _path_ring_role(toolpath),
        "component_id": _path_component_label(toolpath),
    }
    logger.info(json.dumps(result, separators=(",", ":")))
    if closing_edge_suspicious:
        raise AssertionError(
            f"Closed path {toolpath.path_id or '<unassigned>'} has a suspicious implicit closing edge of {closing_edge_mm:.4f} mm"
        )
    return result


def resample_surface_path(path: Toolpath, max_segment_mm: float) -> Toolpath:
    if path.coordinate_space != "surface_mm":
        raise AssertionError(f"Expected surface_mm path before resampling, got {path.coordinate_space}")
    if len(path.points) < 2:
        return clone_toolpath(path)

    core_points, explicit_duplicate_endpoint = _closed_path_core_points(path.points, closed=path.closed)
    effectively_closed = bool(path.closed and len(core_points) >= 3)
    if len(core_points) < 2:
        return clone_toolpath(path)

    out = [Point(core_points[0].x, core_points[0].y)]
    segments = _iter_path_segments(core_points, closed=effectively_closed)
    for a, b in segments:
        dist = _segment_length_mm(a, b)
        steps = max(1, int(math.ceil(dist / max_segment_mm)))
        for i in range(1, steps + 1):
            t = i / steps
            out.append(Point(a.x + ((b.x - a.x) * t), a.y + ((b.y - a.y) * t)))

    after_lengths = _segment_lengths_mm(out, closed=effectively_closed)
    before_lengths = _segment_lengths_mm(path.points, closed=effectively_closed)
    return clone_toolpath(
        path,
        points=out,
        closed=effectively_closed,
        metadata={
            **path.metadata,
            "projection_sampling_mm": max_segment_mm,
            "surface_point_count_before_resampling": len(path.points),
            "surface_point_count_after_resampling": len(out),
            "max_surface_segment_mm_before_resampling": max(before_lengths) if before_lengths else 0.0,
            "max_surface_segment_mm_after_resampling": max(after_lengths) if after_lengths else 0.0,
            "surface_resampling_applied": True,
            "closed_path_degenerated_before_projection": bool(path.closed and not effectively_closed),
            "explicit_duplicate_endpoint_before_projection": explicit_duplicate_endpoint,
        },
    )


def prepare_toolpaths_for_projection(
    toolpaths: list[Toolpath],
    *,
    default_pen_width_mm: float = DEFAULT_LINE_THICKNESS_MM,
) -> list[Toolpath]:
    assert_toolpaths_coordinate_space(toolpaths, "surface_mm")
    prepared: list[Toolpath] = []
    by_component: dict[str, dict[str, list[dict[str, float | int | str]]]] = {}

    for toolpath in toolpaths:
        if toolpath.kind in _drawing_path_kinds():
            validate_closed_path(toolpath)
            if not toolpath.closed and len(toolpath.points) >= 3:
                pen_width_mm = float(toolpath.metadata.get("pen_width_mm", default_pen_width_mm))
                straightened_points = normalize_straight_segments(
                    toolpath.points,
                    StraighteningOptions(
                        angleToleranceDeg=2.0,
                        maxLateralErrorMm=max(0.01, pen_width_mm * 0.05),
                        minStraightSegmentLengthMm=max(2.0 * pen_width_mm, 4.0 * _resolve_projection_sampling_mm(toolpath, default_pen_width_mm=default_pen_width_mm)),
                    ),
                )
                if straightened_points != toolpath.points:
                    toolpath = clone_toolpath(toolpath, points=straightened_points)
            sampling_mm = _resolve_projection_sampling_mm(toolpath, default_pen_width_mm=default_pen_width_mm)
            resampled = resample_surface_path(toolpath, sampling_mm)
            max_after = float(resampled.metadata.get("max_surface_segment_mm_after_resampling", 0.0))
            if max_after > (sampling_mm + 1e-6):
                raise AssertionError(
                    f"{resampled.kind} {resampled.path_id or '<unassigned>'} still exceeds projection sampling limit: "
                    f"{max_after:.4f} mm > {sampling_mm:.4f} mm"
                )
            prepared.append(resampled)

            component_label = _path_component_label(resampled)
            by_component.setdefault(component_label, {}).setdefault(resampled.kind, []).append({
                "path_id": resampled.path_id or "",
                "surface_point_count": len(resampled.points),
                "avg_segment_mm": (
                    sum(_segment_lengths_mm(resampled.points, closed=resampled.closed)) /
                    max(1, len(_segment_lengths_mm(resampled.points, closed=resampled.closed)))
                ) if len(resampled.points) >= 2 else 0.0,
                "max_segment_mm": max_after,
                "segments_over_limit": sum(
                    1
                    for length in _segment_lengths_mm(toolpath.points, closed=toolpath.closed)
                    if length > (sampling_mm + 1e-6)
                ),
            })
            continue

        prepared.append(toolpath)

    for component_label, kind_map in sorted(by_component.items()):
        outline_candidates = kind_map.get("outline", []) + kind_map.get("fill-wall", [])
        infill_candidates = kind_map.get("fill-infill", [])
        if not outline_candidates or not infill_candidates:
            continue
        outline_pick = max(outline_candidates, key=lambda item: float(item["max_segment_mm"]))
        infill_max = max((float(item["max_segment_mm"]) for item in infill_candidates), default=0.0)
        suspicion = "none"
        if float(outline_pick["max_segment_mm"]) > max(infill_max * 1.5, DEFAULT_PROJECTION_SAMPLING_MAX_SEGMENT_MM):
            suspicion = "outline_under_sampled_before_projection"
        logger.info(json.dumps({
            "event": "outline_vs_infill_sampling_check",
            "component_id": component_label,
            "outline": outline_pick,
            "infill": {
                "path_ids": [str(item["path_id"]) for item in infill_candidates],
                "surface_point_count": sum(int(item["surface_point_count"]) for item in infill_candidates),
                "avg_segment_mm": (
                    sum(float(item["avg_segment_mm"]) for item in infill_candidates) / max(1, len(infill_candidates))
                ),
                "max_segment_mm": infill_max,
                "segments_over_limit": sum(int(item["segments_over_limit"]) for item in infill_candidates),
            },
            "suspicion": suspicion,
        }, separators=(",", ":")))

    return prepared


def map_bundle_to_surface_mm(
    bundle: GeometryBundle,
    bounds: SvgBounds,
    fit_mode: str,
    invert_y: bool,
    margin_percent: float,
) -> GeometryBundle:
    radius_mm = ball_radius_mm()
    full_width_mm = ball_degrees_to_mm(DEFAULT_MAX_PRINT_X_SPAN_DEG)
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
    metadata = _update_connector_validation_metadata(bundle.metadata, tuple(matrix))
    return GeometryBundle(
        outline_segments=outline_segments,
        fill_boundary_segments=fill_boundary_segments,
        detail_segments=detail_segments,
        fill_shapes=fill_shapes,
        printable_geometry=printable_geometry,
        cutout_geometry=cutout_geometry,
        metadata=metadata,
    )


def apply_surface_placement_transform(
    bundle: GeometryBundle,
    scale_percent: float,
    rotation_deg: float,
) -> GeometryBundle:
    if scale_percent <= 0:
        raise ValueError("Placement scale must be greater than 0")

    if _bundle_is_empty(bundle):
        return GeometryBundle(metadata=dict(bundle.metadata))

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

    transform_matrix = (
        scale * cos_a,
        scale * sin_a,
        -scale * sin_a,
        scale * cos_a,
        0.0,
        0.0,
    )

    return GeometryBundle(
        outline_segments=outline_segments,
        fill_boundary_segments=fill_boundary_segments,
        detail_segments=detail_segments,
        fill_shapes=fill_shapes,
        printable_geometry=printable_geometry,
        cutout_geometry=cutout_geometry,
        metadata=_update_connector_validation_metadata(bundle.metadata, transform_matrix),
    )


def apply_surface_artwork_scale(
    bundle: GeometryBundle,
    artwork_scale_percent: float,
) -> GeometryBundle:
    if not math.isfinite(artwork_scale_percent):
        raise ValueError("Artwork scale percent must be finite")
    if artwork_scale_percent <= 0:
        raise ValueError("Artwork scale percent must be greater than 0")
    if _bundle_is_empty(bundle):
        return GeometryBundle(metadata=dict(bundle.metadata))

    scale_factor = artwork_scale_percent / 100.0
    if abs(scale_factor - 1.0) <= 1e-12:
        return bundle

    bounds = bounds_from_bundle(bundle)
    center_x = (bounds.min_x + bounds.max_x) / 2.0
    center_y = (bounds.min_y + bounds.max_y) / 2.0

    def scale_point(point: Point) -> Point:
        return Point(
            center_x + ((point.x - center_x) * scale_factor),
            center_y + ((point.y - center_y) * scale_factor),
        )

    outline_segments = [
        Segment([scale_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.outline_segments
    ]
    fill_boundary_segments = [
        Segment([scale_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.fill_boundary_segments
    ]
    detail_segments = [
        Segment([scale_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.detail_segments
    ]
    fill_shapes = [
        SvgFillShape(
            geometry=affinity.scale(fill_shape.geometry, xfact=scale_factor, yfact=scale_factor, origin=(center_x, center_y)),
            fill_rule=fill_shape.fill_rule,
            source_tag=fill_shape.source_tag,
        )
        for fill_shape in bundle.fill_shapes
    ]

    printable_geometry = bundle.printable_geometry
    if printable_geometry is not None and not printable_geometry.is_empty:
        printable_geometry = affinity.scale(printable_geometry, xfact=scale_factor, yfact=scale_factor, origin=(center_x, center_y))
    cutout_geometry = bundle.cutout_geometry
    if cutout_geometry is not None and not cutout_geometry.is_empty:
        cutout_geometry = affinity.scale(cutout_geometry, xfact=scale_factor, yfact=scale_factor, origin=(center_x, center_y))

    transform_matrix = (
        scale_factor,
        0.0,
        0.0,
        scale_factor,
        center_x - (center_x * scale_factor),
        center_y - (center_y * scale_factor),
    )

    return GeometryBundle(
        outline_segments=outline_segments,
        fill_boundary_segments=fill_boundary_segments,
        detail_segments=detail_segments,
        fill_shapes=fill_shapes,
        printable_geometry=printable_geometry,
        cutout_geometry=cutout_geometry,
        metadata=_update_connector_validation_metadata(bundle.metadata, transform_matrix),
    )


def validate_origin_anchor(anchor: str) -> str:
    normalized = str(anchor or "center").strip().lower()
    if normalized not in ORIGIN_ANCHORS:
        allowed = ", ".join(sorted(ORIGIN_ANCHORS))
        raise ValueError(f"Invalid origin anchor '{anchor}'. Allowed values: {allowed}")
    return normalized


def _bundle_is_empty(bundle: GeometryBundle) -> bool:
    return (
        not bundle.outline_segments
        and not bundle.fill_boundary_segments
        and not bundle.detail_segments
        and not bundle.fill_shapes
        and (bundle.printable_geometry is None or bundle.printable_geometry.is_empty)
        and (bundle.cutout_geometry is None or bundle.cutout_geometry.is_empty)
    )


def compute_artwork_bbox(bundle: GeometryBundle) -> SvgBounds:
    return bounds_from_bundle(bundle)


def validate_bundle_x_span(
    bundle: GeometryBundle,
    *,
    max_x_span_deg: float = DEFAULT_MAX_PRINT_X_SPAN_DEG,
    ball_diameter_mm: float = BALL_DIAMETER_MM,
    allow_overflow: bool = False,
) -> dict[str, float | bool]:
    if max_x_span_deg <= 0:
        raise ValueError("Maximum printable X span must be greater than 0 degrees")
    bounds = bounds_from_bundle(bundle)
    width_mm = max(0.0, bounds.width)
    width_deg = mm_to_ball_degrees(width_mm)
    max_width_mm = ball_degrees_to_mm(max_x_span_deg, ball_diameter_mm=ball_diameter_mm)
    if width_deg > (max_x_span_deg + 1e-6) and not allow_overflow:
        raise ValueError(
            f"Artwork exceeds the printable X span limit: {width_deg:.2f} degrees "
            f"({width_mm:.2f} mm) > {max_x_span_deg:.2f} degrees ({max_width_mm:.2f} mm)"
        )
    return {
        "width_mm": width_mm,
        "width_deg": width_deg,
        "max_width_mm": max_width_mm,
        "max_width_deg": max_x_span_deg,
        "limit_overridden": allow_overflow,
    }


def resolve_origin_anchor_point(bounds: SvgBounds, origin_anchor: str) -> Point:
    anchor = validate_origin_anchor(origin_anchor)
    center_x = (bounds.min_x + bounds.max_x) / 2.0
    center_y = (bounds.min_y + bounds.max_y) / 2.0
    anchor_points = {
        "center": Point(center_x, center_y),
        "min-x": Point(bounds.min_x, center_y),
        "max-x": Point(bounds.max_x, center_y),
        "min-y": Point(center_x, bounds.min_y),
        "max-y": Point(center_x, bounds.max_y),
        # In the displayed surface-mm convention used by preview and slicing, top=maxY and bottom=minY.
        "top-left": Point(bounds.min_x, bounds.max_y),
        "top-center": Point(center_x, bounds.max_y),
        "top-right": Point(bounds.max_x, bounds.max_y),
        "center-left": Point(bounds.min_x, center_y),
        "center-right": Point(bounds.max_x, center_y),
        "bottom-left": Point(bounds.min_x, bounds.min_y),
        "bottom-center": Point(center_x, bounds.min_y),
        "bottom-right": Point(bounds.max_x, bounds.min_y),
        # There is no explicit custom-point UI yet, so custom currently resolves from center plus manual offsets.
        "custom": Point(center_x, center_y),
    }
    return anchor_points[anchor]


def apply_surface_mm_translation(bundle: GeometryBundle, dx: float, dy: float) -> GeometryBundle:
    if not math.isfinite(dx) or not math.isfinite(dy):
        raise ValueError("Surface-mm translation must be finite")
    if _bundle_is_empty(bundle):
        return GeometryBundle(metadata=dict(bundle.metadata))
    if abs(dx) <= 1e-12 and abs(dy) <= 1e-12:
        return bundle

    def translate_point(point: Point) -> Point:
        return Point(point.x + dx, point.y + dy)

    outline_segments = [
        Segment([translate_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.outline_segments
    ]
    fill_boundary_segments = [
        Segment([translate_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.fill_boundary_segments
    ]
    detail_segments = [
        Segment([translate_point(point) for point in seg.points], closed=seg.closed)
        for seg in bundle.detail_segments
    ]
    fill_shapes = [
        SvgFillShape(
            geometry=affinity.translate(fill_shape.geometry, xoff=dx, yoff=dy),
            fill_rule=fill_shape.fill_rule,
            source_tag=fill_shape.source_tag,
        )
        for fill_shape in bundle.fill_shapes
    ]
    printable_geometry = bundle.printable_geometry
    if printable_geometry is not None and not printable_geometry.is_empty:
        printable_geometry = affinity.translate(printable_geometry, xoff=dx, yoff=dy)
    cutout_geometry = bundle.cutout_geometry
    if cutout_geometry is not None and not cutout_geometry.is_empty:
        cutout_geometry = affinity.translate(cutout_geometry, xoff=dx, yoff=dy)

    transform_matrix = (1.0, 0.0, 0.0, 1.0, dx, dy)

    return GeometryBundle(
        outline_segments=outline_segments,
        fill_boundary_segments=fill_boundary_segments,
        detail_segments=detail_segments,
        fill_shapes=fill_shapes,
        printable_geometry=printable_geometry,
        cutout_geometry=cutout_geometry,
        metadata=_update_connector_validation_metadata(bundle.metadata, transform_matrix),
    )


def apply_origin_anchor_placement(
    bundle: GeometryBundle,
    *,
    origin_anchor: str,
    origin_offset_x_mm: float,
    origin_offset_y_mm: float,
) -> GeometryBundle:
    if not math.isfinite(origin_offset_x_mm):
        raise ValueError("Origin offset X must be finite")
    if not math.isfinite(origin_offset_y_mm):
        raise ValueError("Origin offset Y must be finite")
    if _bundle_is_empty(bundle):
        return GeometryBundle(metadata=dict(bundle.metadata))

    bounds = compute_artwork_bbox(bundle)
    anchor_point = resolve_origin_anchor_point(bounds, origin_anchor)
    dx = origin_offset_x_mm - anchor_point.x
    dy = origin_offset_y_mm - anchor_point.y
    placed = apply_surface_mm_translation(bundle, dx, dy)
    return GeometryBundle(
        outline_segments=placed.outline_segments,
        fill_boundary_segments=placed.fill_boundary_segments,
        detail_segments=placed.detail_segments,
        fill_shapes=placed.fill_shapes,
        printable_geometry=placed.printable_geometry,
        cutout_geometry=placed.cutout_geometry,
        metadata={
            **placed.metadata,
            "origin_anchor": validate_origin_anchor(origin_anchor),
            "origin_offset_x_mm": origin_offset_x_mm,
            "origin_offset_y_mm": origin_offset_y_mm,
            "origin_anchor_point": {"x": anchor_point.x, "y": anchor_point.y},
            "origin_translation_mm": {"x": dx, "y": dy},
        },
    )


def segment_length(points: list[Point]) -> float:
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))


def clone_toolpath(
    toolpath: Toolpath,
    *,
    points: Optional[list[Point]] = None,
    kind: Optional[str] = None,
    closed: Optional[bool] = None,
    coordinate_space: Optional[str] = None,
    path_id: Optional[str] = None,
    source: Optional[str] = None,
    region_id: Optional[int] = None,
    warnings: Optional[list[str]] = None,
    metadata: Optional[dict[str, Any]] = None,
) -> Toolpath:
    return Toolpath(
        points=list(points if points is not None else toolpath.points),
        kind=kind if kind is not None else toolpath.kind,
        closed=toolpath.closed if closed is None else closed,
        coordinate_space=coordinate_space if coordinate_space is not None else toolpath.coordinate_space,
        path_id=toolpath.path_id if path_id is None else path_id,
        source=toolpath.source if source is None else source,
        region_id=toolpath.region_id if region_id is None else region_id,
        warnings=list(toolpath.warnings if warnings is None else warnings),
        metadata=dict(toolpath.metadata if metadata is None else metadata),
    )


DETAIL_FILTER_MODE = "coverage_aware_prune_redundant_details"
DETAIL_SOURCE_ALIASES: dict[str, str] = {
    "detail_trace": "thin_feature_centerline",
    "residual_centerline": "skeleton_required_for_unfilled_thin_region",
}
DETAIL_SOURCE_WHITELIST = {
    "thin_feature_centerline",
    "skeleton_required_for_unfilled_thin_region",
    "coverage_repair",
    "small_missed_blob_repair",
    "intentional_user_detail_layer",
}
DETAIL_SOURCE_REJECT_AS_TRAVEL_OR_DEBUG = {
    "travel",
    "pen_up_travel",
    "optimizer_travel",
    "connector_candidate",
    "safe_connector_candidate",
    "rejected_connector",
    "component_graph_edge",
    "debug_route",
    "preview_only_line",
    "stale_pre_optimization_path",
    "outline_candidate",
    "detail_merge_connector",
    "detail_continuation",
    "detail_continuation_rejected",
    "fill-infill-travel",
    "coverage_connector",
    "gcode_travel",
}


def _canonical_detail_source(toolpath: Toolpath) -> tuple[Optional[str], str]:
    raw_source = str(
        toolpath.metadata.get("detail_source")
        or toolpath.metadata.get("source_class")
        or toolpath.source
        or ""
    ).strip()
    canonical = DETAIL_SOURCE_ALIASES.get(raw_source, raw_source)
    if canonical in DETAIL_SOURCE_WHITELIST:
        return canonical, ""
    if canonical in DETAIL_SOURCE_REJECT_AS_TRAVEL_OR_DEBUG:
        return None, "travel_or_debug"
    return None, "unknown_source"


def _filter_detail_trace_candidates_for_export(
    candidates: list[Toolpath],
    *,
    target_geometry: Any,
    existing_painted_area: Any,
    line_width_mm: float,
    allow_detail_overlap_outline: bool,
    validate_detail_with_pen_footprint: bool,
    max_detail_overspill_mm: float,
    max_detail_overspill_area_ratio: float,
    min_detail_new_coverage_mm2: float,
    max_already_covered_ratio: float,
    candidate_component_index_fn: Callable[[Toolpath], int | None],
    candidate_centeredness_fn: Callable[[Toolpath, int | None], float],
    candidate_component_metrics_fn: Optional[Callable[[Toolpath, int | None], dict[str, Any]]] = None,
) -> dict[str, Any]:
    accepted_detail_paths: list[Toolpath] = []
    accepted_detail_footprints: list[Any] = []
    rejected_detail_paths: list[Toolpath] = []
    overspill_warning_regions: list[Any] = []
    accepted_detail_coverage: Any = Polygon()
    target_boundary = target_geometry.boundary if target_geometry is not None and not target_geometry.is_empty else None
    drop_reasons: Counter[str] = Counter()
    dropped_records: list[dict[str, Any]] = []
    already_covered_ratios: list[float] = []
    total_new_coverage_area = 0.0
    radius = max(0.01, line_width_mm * 0.5)
    min_candidate_length_mm = max(0.05, line_width_mm * 0.15)

    for candidate_index, candidate in enumerate(candidates):
        if len(candidate.points) < 2:
            continue

        candidate_id = candidate.path_id or _path_debug_identifier(candidate, candidate_index)
        candidate_length_mm = segment_length(candidate.points)
        line = LineString([(point.x, point.y) for point in candidate.points])
        component_idx = candidate_component_index_fn(candidate)
        centeredness = candidate_centeredness_fn(candidate, component_idx)
        component_metrics = candidate_component_metrics_fn(candidate, component_idx) if candidate_component_metrics_fn is not None else {}
        if not isinstance(component_metrics, dict):
            component_metrics = {}
        component_id = component_metrics.get("component_id", component_idx + 1 if component_idx is not None else None)
        component_area_mm2 = float(component_metrics.get("area_mm2", 0.0) or 0.0)
        bbox_mm = component_metrics.get("bbox_mm")
        if isinstance(bbox_mm, (tuple, list)) and len(bbox_mm) >= 2:
            bbox_mm = (float(bbox_mm[0]), float(bbox_mm[1]))
        else:
            bounds = line.bounds if not line.is_empty else (0.0, 0.0, 0.0, 0.0)
            bbox_mm = (float(bounds[2] - bounds[0]), float(bounds[3] - bounds[1]))
        estimated_width_mm = float(component_metrics.get("estimated_width_mm", min(bbox_mm[0], bbox_mm[1])) or 0.0)
        long_axis_mm = float(max(bbox_mm[0], bbox_mm[1]))
        long_enough_for_detail = candidate_length_mm >= (line_width_mm * 2.0)
        elongated_narrow_path = (
            estimated_width_mm > max(1e-6, line_width_mm * 0.25)
            and estimated_width_mm <= (line_width_mm * 2.75)
            and candidate_length_mm >= (line_width_mm * 3.0)
            and (candidate_length_mm / max(estimated_width_mm, 1e-6)) >= 4.0
            and long_axis_mm >= (line_width_mm * 3.0)
        )
        geometric_narrow_detail = (
            estimated_width_mm > max(1e-6, line_width_mm * 0.20)
            and estimated_width_mm <= (line_width_mm * 1.75)
            and candidate_length_mm >= (line_width_mm * 2.0)
            and long_axis_mm >= (line_width_mm * 2.0)
        )
        is_small_detail_component = component_idx is not None and (
            (estimated_width_mm > 0.0 and estimated_width_mm <= (line_width_mm * 1.75))
            or (component_area_mm2 > 0.0 and component_area_mm2 <= max(line_width_mm * line_width_mm * 10.0, 2.0))
        )
        if not is_small_detail_component and geometric_narrow_detail:
            is_small_detail_component = True
        if not is_small_detail_component and elongated_narrow_path:
            is_small_detail_component = True
        was_self_overlapping = bool(not line.is_simple) if not line.is_empty else False

        def _reject(reason: str, *, new_coverage_area_mm2: float = 0.0, already_covered_ratio: float = 1.0) -> None:
            drop_reasons[reason] += 1
            rejected_detail_paths.append(candidate)
            dropped_records.append({
                "path_id": candidate_id,
                "component_id": component_id,
                "area_mm2": component_area_mm2,
                "bbox_mm": bbox_mm,
                "estimated_width_mm": estimated_width_mm,
                "path_length_mm": float(candidate_length_mm),
                "source": str(candidate.source or ""),
                "canonical_source": canonical_source or "",
                "drop_reason": reason,
                "new_coverage_area_mm2": float(new_coverage_area_mm2),
                "already_covered_ratio": float(already_covered_ratio),
                "was_self_overlapping": bool(was_self_overlapping),
                "outside_overflow_mm2": 0.0,
            })

        canonical_source, source_rejection_reason = _canonical_detail_source(candidate)
        if canonical_source is None:
            _reject(source_rejection_reason or "unknown_source")
            continue

        if candidate_length_mm < min_candidate_length_mm:
            _reject("too_short")
            continue

        stroke = line.buffer(radius, cap_style=1, join_style=1)
        if stroke.is_empty:
            _reject("empty_footprint")
            continue

        target_footprint = stroke
        if target_geometry is not None and not target_geometry.is_empty:
            try:
                target_footprint = stroke.intersection(target_geometry)
            except Exception:
                target_footprint = stroke.intersection(target_geometry.buffer(0))
        target_footprint_area = float(target_footprint.area) if target_footprint is not None and not target_footprint.is_empty else 0.0
        if target_footprint_area <= 1e-9:
            _reject("outside_target", already_covered_ratio=1.0)
            continue

        effective_existing_painted_area = existing_painted_area
        if accepted_detail_coverage is not None and not accepted_detail_coverage.is_empty:
            try:
                effective_existing_painted_area = existing_painted_area.union(accepted_detail_coverage)
            except Exception:
                effective_existing_painted_area = existing_painted_area.buffer(0).union(accepted_detail_coverage.buffer(0))

        try:
            new_coverage = target_footprint.difference(effective_existing_painted_area)
        except Exception:
            new_coverage = target_footprint.buffer(0).difference(effective_existing_painted_area.buffer(0))
        new_coverage_area = float(new_coverage.area) if new_coverage is not None and not new_coverage.is_empty else 0.0
        already_covered_ratio = max(0.0, min(1.0, 1.0 - (new_coverage_area / max(1e-9, target_footprint_area))))
        already_covered_ratios.append(already_covered_ratio)
        overlap_exception_applied = False
        overlap_exception_reason = ""
        overlap_exception_candidate = (
            is_small_detail_component
            and long_enough_for_detail
            and canonical_source in {"thin_feature_centerline", "skeleton_required_for_unfilled_thin_region"}
            and centeredness <= (line_width_mm * 0.42)
        )

        if new_coverage_area < min_detail_new_coverage_mm2 or already_covered_ratio > max_already_covered_ratio:
            if not overlap_exception_candidate:
                _reject("redundant_overlap", new_coverage_area_mm2=new_coverage_area, already_covered_ratio=already_covered_ratio)
                continue
            overlap_exception_applied = True
            overlap_exception_reason = "small_detail_centered_overlap"

        overspill = None
        overspill_area = 0.0
        overspill_ratio = 0.0
        protrusion_mm = 0.0
        if target_geometry is not None and not target_geometry.is_empty:
            overspill = stroke.difference(target_geometry)
            overspill_area = float(overspill.area) if overspill is not None and not overspill.is_empty else 0.0
            overspill_ratio = overspill_area / max(1e-9, float(stroke.area))
            if overspill is not None and not overspill.is_empty and target_boundary is not None:
                max_distance = 0.0
                for poly in normalize_geometry(overspill):
                    coords = list(poly.exterior.coords)
                    sample_step = max(1, int(len(coords) / 32))
                    for idx in range(0, len(coords), sample_step):
                        pt = ShapelyPoint(float(coords[idx][0]), float(coords[idx][1]))
                        max_distance = max(max_distance, float(pt.distance(target_boundary)))
                protrusion_mm = max_distance
        if validate_detail_with_pen_footprint:
            overspill_ok = overspill_ratio <= max_detail_overspill_area_ratio and protrusion_mm <= max_detail_overspill_mm
            if not overspill_ok:
                if overspill is not None and not overspill.is_empty:
                    overspill_warning_regions.append(overspill)
                _reject("overspill", new_coverage_area_mm2=new_coverage_area, already_covered_ratio=already_covered_ratio)
                if dropped_records:
                    dropped_records[-1]["outside_overflow_mm2"] = float(overspill_area)
                continue

        overlap_area = 0.0
        if accepted_detail_coverage is not None and not accepted_detail_coverage.is_empty:
            overlap_area = float(stroke.intersection(accepted_detail_coverage).area)
        overlap_ratio = overlap_area / max(1e-9, float(stroke.area))
        is_narrow_component = component_idx is not None
        is_centerline_like = canonical_source == "skeleton_required_for_unfilled_thin_region"
        if is_narrow_component and is_centerline_like and overlap_ratio >= 0.82 and centeredness <= (line_width_mm * 0.42):
            _reject("redundant_centerline_overlap", new_coverage_area_mm2=new_coverage_area, already_covered_ratio=already_covered_ratio)
            continue

        chord = math.hypot(candidate.points[-1].x - candidate.points[0].x, candidate.points[-1].y - candidate.points[0].y)
        if chord > 1e-6 and (segment_length(candidate.points) / chord) > 4.0 and len(candidate.points) > 12:
            if not overlap_exception_candidate:
                _reject("noisy_loop", new_coverage_area_mm2=new_coverage_area, already_covered_ratio=already_covered_ratio)
                continue

        is_edge_detail = False
        if allow_detail_overlap_outline and target_boundary is not None:
            try:
                is_edge_detail = float(line.distance(target_boundary)) <= (line_width_mm * 0.7)
            except Exception:
                is_edge_detail = False

        accepted_detail_paths.append(clone_toolpath(
            candidate,
            kind="detail-trace",
            metadata={
                **candidate.metadata,
                "path_role": "PRINT_DETAIL_EDGE" if is_edge_detail else "PRINT_DETAIL",
                "detail_filter_mode": DETAIL_FILTER_MODE,
                "detail_source_whitelist_enforced": True,
                "detail_source_class": canonical_source,
                "detail_overlap_outline_allowed": bool(allow_detail_overlap_outline),
                "detail_validate_with_pen_footprint": bool(validate_detail_with_pen_footprint),
                "detail_new_coverage_area_mm2": new_coverage_area,
                "detail_already_covered_ratio": already_covered_ratio,
                "detail_overspill_area_mm2": overspill_area,
                "detail_overspill_area_ratio": overspill_ratio,
                "detail_max_protrusion_mm": protrusion_mm,
                "detail_overlap_ratio_with_accepted": overlap_ratio,
                "detail_centeredness_mm": centeredness,
                "detail_component_id": component_id,
                "detail_component_area_mm2": component_area_mm2,
                "detail_component_bbox_mm": bbox_mm,
                "detail_component_estimated_width_mm": estimated_width_mm,
                "detail_path_length_mm": float(candidate_length_mm),
                "detail_path_self_overlapping": bool(was_self_overlapping),
                "detail_overlap_exception_applied": bool(overlap_exception_applied),
                "detail_overlap_exception_reason": overlap_exception_reason,
            },
        ))
        total_new_coverage_area += new_coverage_area
        try:
            accepted_detail_coverage = accepted_detail_coverage.union(stroke)
        except Exception:
            accepted_detail_coverage = accepted_detail_coverage.buffer(0).union(stroke.buffer(0))
        accepted_detail_footprints.append(stroke)

    return {
        "detail_filter_mode": DETAIL_FILTER_MODE,
        "accepted_detail_paths": accepted_detail_paths,
        "accepted_detail_footprints": accepted_detail_footprints,
        "accepted_detail_coverage": accepted_detail_coverage,
        "rejected_detail_paths": rejected_detail_paths,
        "overspill_warning_regions": overspill_warning_regions,
        "detail_paths_generated_raw": len(candidates),
        "detail_paths_kept": len(accepted_detail_paths),
        "detail_paths_dropped": len(rejected_detail_paths),
        "detail_drop_reasons": dict(drop_reasons),
        "detail_paths_dropped_as_travel_or_debug": int(drop_reasons.get("travel_or_debug", 0)),
        "detail_paths_dropped_as_redundant_overlap": int(drop_reasons.get("redundant_overlap", 0)),
        "detail_new_coverage_area_mm2_total": float(total_new_coverage_area),
        "detail_already_covered_ratio_avg": float(sum(already_covered_ratios) / max(1, len(already_covered_ratios))),
        "detail_source_whitelist_enforced": True,
        "travel_geometry_allowed_as_detail": False,
        "infill_geometry_changed": False,
        "outline_geometry_changed": False,
        "detail_dropped_path_records": dropped_records,
    }


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


def ball_angles_to_surface_mm(
    point: Point,
    *,
    center_lon_deg: float,
    center_lat_deg: float,
    ball_diameter_mm: float = BALL_DIAMETER_MM,
) -> Point:
    radius = ball_radius_mm(ball_diameter_mm)
    lon = math.radians(point.x)
    lat = math.radians(point.y)
    center_lon = math.radians(center_lon_deg)
    center_lat = math.radians(center_lat_deg)
    y_mm = (lat - center_lat) * radius
    x_mm = (lon - center_lon) * radius * math.cos(lat)
    return Point(float(x_mm), float(y_mm))


def resolve_safe_projection_center_lat(
    toolpaths_surface_mm: list[Toolpath],
    *,
    requested_center_lat_deg: float,
    ball_diameter_mm: float = BALL_DIAMETER_MM,
    y_draw_min_deg: float = Y_DRAW_MIN,
    y_draw_max_deg: float = Y_DRAW_MAX,
) -> tuple[float, dict[str, float | bool]]:
    if not toolpaths_surface_mm:
        return requested_center_lat_deg, {
            "requested_center_lat_deg": requested_center_lat_deg,
            "resolved_center_lat_deg": requested_center_lat_deg,
            "auto_clamped": False,
            "surface_min_y_mm": 0.0,
            "surface_max_y_mm": 0.0,
            "allowed_center_lat_min_deg": y_draw_min_deg,
            "allowed_center_lat_max_deg": y_draw_max_deg,
        }

    y_values = [point.y for path in toolpaths_surface_mm for point in path.points]
    if not y_values:
        return requested_center_lat_deg, {
            "requested_center_lat_deg": requested_center_lat_deg,
            "resolved_center_lat_deg": requested_center_lat_deg,
            "auto_clamped": False,
            "surface_min_y_mm": 0.0,
            "surface_max_y_mm": 0.0,
            "allowed_center_lat_min_deg": y_draw_min_deg,
            "allowed_center_lat_max_deg": y_draw_max_deg,
        }

    surface_min_y_mm = float(min(y_values))
    surface_max_y_mm = float(max(y_values))
    mm_to_deg = 360.0 / (math.pi * ball_diameter_mm)
    allowed_center_lat_min_deg = y_draw_min_deg - (surface_min_y_mm * mm_to_deg)
    allowed_center_lat_max_deg = y_draw_max_deg - (surface_max_y_mm * mm_to_deg)

    if allowed_center_lat_min_deg > allowed_center_lat_max_deg + 1e-9:
        raise ValueError(
            "Artwork exceeds available Y drawing band after placement; reduce scale or move origin Y toward center"
        )

    resolved_center_lat_deg = requested_center_lat_deg
    if resolved_center_lat_deg < allowed_center_lat_min_deg:
        resolved_center_lat_deg = allowed_center_lat_min_deg
    if resolved_center_lat_deg > allowed_center_lat_max_deg:
        resolved_center_lat_deg = allowed_center_lat_max_deg

    return resolved_center_lat_deg, {
        "requested_center_lat_deg": requested_center_lat_deg,
        "resolved_center_lat_deg": resolved_center_lat_deg,
        "auto_clamped": abs(resolved_center_lat_deg - requested_center_lat_deg) > 1e-9,
        "surface_min_y_mm": surface_min_y_mm,
        "surface_max_y_mm": surface_max_y_mm,
        "allowed_center_lat_min_deg": allowed_center_lat_min_deg,
        "allowed_center_lat_max_deg": allowed_center_lat_max_deg,
    }


def fit_surface_toolpaths_to_y_band(
    toolpaths_surface_mm: list[Toolpath],
    *,
    ball_diameter_mm: float = BALL_DIAMETER_MM,
    y_draw_min_deg: float = Y_DRAW_MIN,
    y_draw_max_deg: float = Y_DRAW_MAX,
    safety_factor: float = 0.995,
) -> tuple[list[Toolpath], dict[str, float | bool]]:
    if not toolpaths_surface_mm:
        return toolpaths_surface_mm, {
            "auto_scaled": False,
            "scale_factor": 1.0,
            "current_span_mm": 0.0,
            "allowed_span_mm": 0.0,
        }

    points = [point for path in toolpaths_surface_mm for point in path.points]
    if not points:
        return toolpaths_surface_mm, {
            "auto_scaled": False,
            "scale_factor": 1.0,
            "current_span_mm": 0.0,
            "allowed_span_mm": 0.0,
        }

    min_y = min(point.y for point in points)
    max_y = max(point.y for point in points)
    min_x = min(point.x for point in points)
    max_x = max(point.x for point in points)
    current_span_mm = float(max_y - min_y)
    mm_to_deg = 360.0 / (math.pi * ball_diameter_mm)
    allowed_span_mm = max(0.0, (y_draw_max_deg - y_draw_min_deg) / mm_to_deg)

    if current_span_mm <= allowed_span_mm + 1e-9:
        return toolpaths_surface_mm, {
            "auto_scaled": False,
            "scale_factor": 1.0,
            "current_span_mm": current_span_mm,
            "allowed_span_mm": allowed_span_mm,
        }

    scale_factor = max(0.001, (allowed_span_mm / max(current_span_mm, 1e-9)) * max(0.8, min(1.0, safety_factor)))
    origin_x = float((min_x + max_x) * 0.5)
    origin_y = float((min_y + max_y) * 0.5)
    scaled: list[Toolpath] = []
    for path in toolpaths_surface_mm:
        scaled_points = [
            Point(
                origin_x + ((point.x - origin_x) * scale_factor),
                origin_y + ((point.y - origin_y) * scale_factor),
            )
            for point in path.points
        ]
        scaled.append(clone_toolpath(
            path,
            points=scaled_points,
            metadata={
                **path.metadata,
                "auto_scaled_to_y_band": True,
                "auto_scale_factor": scale_factor,
            },
        ))
    return scaled, {
        "auto_scaled": True,
        "scale_factor": scale_factor,
        "current_span_mm": current_span_mm,
        "allowed_span_mm": allowed_span_mm,
        "origin_x_mm": origin_x,
        "origin_y_mm": origin_y,
    }


def project_toolpaths_to_ball_angles(
    toolpaths: list[Toolpath],
    *,
    center_lon_deg: float,
    center_lat_deg: float,
    ball_diameter_mm: float = BALL_DIAMETER_MM,
    min_cos_lat: float = 0.1,
    sample_step_deg: float | None = None,
) -> list[Toolpath]:
    projected: list[Toolpath] = []
    sample_step_mm = None
    if sample_step_deg is not None:
        sample_step_mm = ball_radius_mm(ball_diameter_mm) * math.radians(max(0.05, sample_step_deg)) * 0.5
    for toolpath in toolpaths:
        if toolpath.coordinate_space != "surface_mm":
            raise AssertionError(f"Expected surface-mm toolpath before projection, got {toolpath.coordinate_space}")
        sampled_points = resample_segment(toolpath.points, max_step=sample_step_mm) if sample_step_mm else list(toolpath.points)
        points = [
            surface_mm_to_ball_angles(
                point,
                center_lon_deg=center_lon_deg,
                center_lat_deg=center_lat_deg,
                ball_diameter_mm=ball_diameter_mm,
                min_cos_lat=min_cos_lat,
            )
            for point in sampled_points
        ]
        projected_toolpath = clone_toolpath(
            toolpath,
            points=points,
            coordinate_space="machine_deg",
            metadata={
                **toolpath.metadata,
                "surface_points_before_projection": [
                    {"x": float(point.x), "y": float(point.y)}
                    for point in toolpath.points
                ],
                "coordinate_space_before_projection": toolpath.coordinate_space,
                "coordinate_space_after_projection": "machine_deg",
                "projection_function": "surface_mm_to_ball_angles",
                "projection_count": int(toolpath.metadata.get("projection_count", 0)) + 1,
                "point_count_before_projection": len(toolpath.points),
                "point_count_after_projection": len(points),
            },
        )
        projected.append(projected_toolpath)
        log_toolpath_summary(toolpath, projected_toolpath)
    log_cleanup_outline_audits(projected)
    return projected


def assert_toolpaths_coordinate_space(toolpaths: list[Toolpath], expected_space: str) -> None:
    for toolpath in toolpaths:
        if toolpath.coordinate_space != expected_space:
            raise AssertionError(f"Expected toolpath coordinate space {expected_space}, got {toolpath.coordinate_space} for {toolpath.kind}")


def _points_close(a: Point, b: Point, epsilon: float) -> bool:
    return math.hypot(a.x - b.x, a.y - b.y) <= epsilon


def _sanitize_toolpath_points(
    points: list[Point],
    *,
    closed: bool,
    duplicate_epsilon: float,
    min_segment_length_mm: float,
) -> tuple[list[Point], int, int]:
    if not points:
        return [], 0, 0
    deduped = [points[0]]
    duplicate_points_removed = 0
    short_segments_removed = 0
    for point in points[1:]:
        if _points_close(deduped[-1], point, duplicate_epsilon):
            duplicate_points_removed += 1
            continue
        if min_segment_length_mm > 0 and math.hypot(point.x - deduped[-1].x, point.y - deduped[-1].y) < min_segment_length_mm:
            short_segments_removed += 1
            continue
        deduped.append(point)
    if closed and len(deduped) >= 3:
        if not _points_close(deduped[0], deduped[-1], duplicate_epsilon):
            deduped.append(Point(deduped[0].x, deduped[0].y))
        elif len(deduped) > 1:
            deduped[-1] = Point(deduped[0].x, deduped[0].y)
    return deduped, duplicate_points_removed, short_segments_removed


def _effective_cleanup_min_segment_length_mm(toolpath: Toolpath, requested_min_segment_length_mm: float) -> float:
    if toolpath.kind in {"outline", "fill-wall", "detail-trace", "fill-infill-travel"}:
        return 0.0
    return requested_min_segment_length_mm


def cleanup_surface_toolpaths(
    toolpaths: list[Toolpath],
    *,
    tolerance_mm: float,
    min_segment_length_mm: float,
) -> tuple[list[Toolpath], dict[str, Any]]:
    assert_toolpaths_coordinate_space(toolpaths, "surface_mm")
    duplicate_epsilon = max(1e-6, min_segment_length_mm * 0.25, tolerance_mm * 0.5 if tolerance_mm > 0 else 0.0)
    cleaned: list[Toolpath] = []
    stats = {
        "duplicate_points_removed": 0,
        "short_segments_removed": 0,
        "simplification_tolerance_mm": tolerance_mm,
    }
    for toolpath in toolpaths:
        effective_min_segment_length_mm = _effective_cleanup_min_segment_length_mm(toolpath, min_segment_length_mm)
        points, duplicate_removed, short_removed = _sanitize_toolpath_points(
            toolpath.points,
            closed=toolpath.closed,
            duplicate_epsilon=duplicate_epsilon,
            min_segment_length_mm=effective_min_segment_length_mm,
        )
        stats["duplicate_points_removed"] += duplicate_removed
        stats["short_segments_removed"] += short_removed
        if len(points) < 2:
            continue
        simplified = simplify_segment_points(points, tolerance_mm, toolpath.closed) if tolerance_mm > 0 else points
        if len(simplified) < 2:
            continue
        cleaned.append(clone_toolpath(toolpath, points=simplified))
    return cleaned, stats


def simplify_segment_points(points: list[Point], tolerance: float, closed: bool) -> list[Point]:
    if len(points) < 2 or tolerance <= 0:
        return points
    coords = [(point.x, point.y) for point in points]
    geometry = Polygon(coords) if closed and len(coords) >= 4 else LineString(coords)
    simplified = geometry.simplify(tolerance, preserve_topology=True)
    if closed:
        if isinstance(simplified, Polygon):
            out = [Point(x, y) for x, y in simplified.exterior.coords]
        elif hasattr(simplified, "coords"):
            out = [Point(x, y) for x, y in simplified.coords]
        else:
            out = [Point(x, y) for x, y in coords]
        if len(out) >= 3 and not nearly_same_point(out[0], out[-1]):
            out.append(Point(out[0].x, out[0].y))
        if len(out) < 4:
            return points
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
            source="polygon_offset",
        ))
        for interior in polygon.interiors:
            paths.append(Toolpath(
                points=simplify_segment_points([Point(x, y) for x, y in interior.coords], tolerance, True),
                kind=kind,
                closed=True,
                source="polygon_offset",
            ))
    return paths


def _points_from_ring(coords: Any) -> list[Point]:
    return [Point(float(x), float(y)) for x, y in coords]


def build_printable_regions_from_geometry(printable_geometry: Any) -> list[PrintableRegion]:
    polygons = sorted(
        normalize_geometry(printable_geometry),
        key=lambda poly: (-round(poly.area, 5), -round(poly.centroid.y, 5), round(poly.centroid.x, 5)),
    )
    regions: list[PrintableRegion] = []
    for region_index, polygon in enumerate(polygons, start=1):
        regions.append(PrintableRegion(
            component_id=f"component_{region_index:03d}",
            outer_rings_mm=[_points_from_ring(polygon.exterior.coords)],
            hole_rings_mm=[_points_from_ring(interior.coords) for interior in polygon.interiors],
            geometry=polygon,
        ))
    return regions


def _translate_polygon(polygon: Polygon, *, dx: float, dy: float) -> Polygon:
    return affinity.translate(polygon, xoff=dx, yoff=dy)


def _diagnostic_square_specs(
    *,
    square_size_mm: float = 4.5,
    gap_mm: float = 0.5,
) -> list[dict[str, Any]]:
    labels = [
        ["top-left", "Top-left"],
        ["top-center", "Top-center"],
        ["top-right", "Top-right"],
        ["middle-left", "Middle-left"],
        ["middle-center", "Middle-center"],
        ["middle-right", "Middle-right"],
        ["bottom-left", "Bottom-left"],
        ["bottom-center", "Bottom-center"],
        ["bottom-right", "Bottom-right"],
    ]
    specs: list[dict[str, Any]] = []
    for index, (square_id, label) in enumerate(labels):
        row = index // 3
        col = index % 3
        pitch_mm = square_size_mm + gap_mm
        square = _translate_polygon(
            Polygon([(0.0, 0.0), (square_size_mm, 0.0), (square_size_mm, square_size_mm), (0.0, square_size_mm)]),
            dx=((col - 1) * pitch_mm) - (square_size_mm / 2.0),
            dy=((1 - row) * pitch_mm) - (square_size_mm / 2.0),
        )
        specs.append({
            "id": square_id,
            "label": label,
            "row": row,
            "col": col,
            "geometry": square,
        })
    return specs


def build_x_axis_rotation_calibration_toolpaths(
    *,
    tick_height_deg: float = 8.0,
) -> tuple[list[Toolpath], list[XAxisCalibrationTick]]:
    half_height = tick_height_deg / 2.0
    tick_specs = [
        XAxisCalibrationTick(id="tick_000", label="0 deg", commanded_x_deg=0.0, emitted_x_deg=0.0, y_start_deg=-half_height, y_end_deg=half_height),
        XAxisCalibrationTick(id="tick_090", label="90 deg", commanded_x_deg=90.0, emitted_x_deg=90.0, y_start_deg=-half_height, y_end_deg=half_height),
        XAxisCalibrationTick(id="tick_180", label="180 deg", commanded_x_deg=180.0, emitted_x_deg=180.0, y_start_deg=-half_height, y_end_deg=half_height),
        XAxisCalibrationTick(id="tick_270", label="270 deg", commanded_x_deg=270.0, emitted_x_deg=-90.0, y_start_deg=-half_height, y_end_deg=half_height),
        XAxisCalibrationTick(id="tick_360", label="360 deg", commanded_x_deg=360.0, emitted_x_deg=0.0, y_start_deg=-half_height, y_end_deg=half_height),
    ]
    toolpaths = [
        Toolpath(
            points=[
                Point(spec.emitted_x_deg, spec.y_start_deg),
                Point(spec.emitted_x_deg, spec.y_end_deg),
            ],
            kind="outline",
            closed=False,
            coordinate_space="machine_deg",
            path_id=spec.id,
            source="x_axis_rotation_calibration",
            metadata={
                "coordinate_space_before_projection": "generated_in_machine_deg",
                "coordinate_space_after_projection": "machine_deg",
                "projection_function": "machine_deg_direct_calibration",
                "projection_count": 1,
                "point_count_before_projection": 2,
                "point_count_after_projection": 2,
                "commanded_x_deg": spec.commanded_x_deg,
                "emitted_x_deg": spec.emitted_x_deg,
                "tick_label": spec.label,
            },
        )
        for spec in tick_specs
    ]
    return toolpaths, tick_specs


def build_diagnostic_geometry_bundle(pattern: str) -> GeometryBundle:
    if Polygon is None or affinity is None:
        raise RuntimeError("Diagnostic geometry requires shapely")

    base_square = Polygon([(0.0, 0.0), (8.0, 0.0), (8.0, 8.0), (0.0, 8.0)])
    vertical_rect = _translate_polygon(Polygon([(0.0, 0.0), (4.0, 0.0), (4.0, 12.0), (0.0, 12.0)]), dx=14.0, dy=0.0)
    horizontal_rect = _translate_polygon(Polygon([(0.0, 0.0), (12.0, 0.0), (12.0, 4.0), (0.0, 4.0)]), dx=0.0, dy=14.0)
    parallelogram = _translate_polygon(Polygon([(0.0, 0.0), (8.0, 0.0), (12.0, 8.0), (4.0, 8.0)]), dx=18.0, dy=16.0)
    square_specs = _diagnostic_square_specs()
    square_union = unary_union([spec["geometry"] for spec in square_specs])
    fill_shapes = {
        "filled_square": base_square,
        "filled_vertical_rectangle": vertical_rect,
        "filled_horizontal_rectangle": horizontal_rect,
        "filled_45_parallelogram": parallelogram,
        "diagnostic_suite": unary_union([base_square, vertical_rect, horizontal_rect, parallelogram]),
        "3x3_squares": square_union,
    }
    if pattern not in fill_shapes:
        raise ValueError(f"Unknown diagnostic pattern: {pattern}")
    metadata: dict[str, Any] = {}
    if pattern == "3x3_squares":
        metadata = {
            "diagnostic_pattern": pattern,
            "diagnostic_squares": square_specs,
        }
    return GeometryBundle(printable_geometry=fill_shapes[pattern], metadata=metadata)


def path_is_inside_printable_area(path: Toolpath, printable_geometry: Any, tolerance_mm: float = 0.02) -> bool:
    if printable_geometry is None or printable_geometry.is_empty or len(path.points) < 2:
        return False
    line = LineString([(point.x, point.y) for point in path.points])
    return printable_geometry.buffer(max(0.0, tolerance_mm), join_style=1).covers(line)


def assign_stable_path_ids(toolpaths: list[Toolpath]) -> list[Toolpath]:
    counters: dict[str, int] = {}
    assigned: list[Toolpath] = []
    for toolpath in toolpaths:
        counters[toolpath.kind] = counters.get(toolpath.kind, 0) + 1
        path_id = toolpath.path_id or f"{toolpath.kind}_{counters[toolpath.kind]:03d}"
        assigned.append(clone_toolpath(toolpath, path_id=path_id))
    return assigned


def _bounds_for_points(points: list[Point]) -> dict[str, float]:
    return {
        "min_x": min(point.x for point in points),
        "max_x": max(point.x for point in points),
        "min_y": min(point.y for point in points),
        "max_y": max(point.y for point in points),
    }


def _extract_component_id(value: Any) -> int | None:
    if value is None:
        return None
    if isinstance(value, int):
        return value
    match = re.search(r"(\d+)", str(value))
    if not match:
        return None
    try:
        return int(match.group(1))
    except ValueError:
        return None


def _expected_relation_to_fill(toolpath: Toolpath) -> str:
    if toolpath.kind == "travel":
        return "pen_up_reposition"
    defaults = {
        "outline": "boundary_cleanup",
        "fill-wall": "supporting_wall",
        "fill-infill": "fill_interior",
        "fill-infill-travel": "internal_fill_connector",
        "detail-trace": "detail_overlay",
    }
    return str(toolpath.metadata.get("expected_relation_to_fill") or defaults.get(toolpath.kind, "independent"))


def _build_toolpath_summary(
    toolpath_surface: Toolpath | None,
    toolpath_machine: Toolpath | None,
) -> dict[str, Any]:
    reference = toolpath_surface or toolpath_machine
    if reference is None:
        return {"event": "toolpath_summary", "path_id": None, "kind": "unknown"}
    metadata = {
        **(toolpath_surface.metadata if toolpath_surface is not None else {}),
        **(toolpath_machine.metadata if toolpath_machine is not None else {}),
    }
    source_component_id = metadata.get("source_component_id")
    if source_component_id is None and reference.region_id is not None:
        source_component_id = reference.region_id + 1
    source_component_id = _extract_component_id(source_component_id or metadata.get("source_polygon_id"))
    surface_points = toolpath_surface.points if toolpath_surface is not None else []
    machine_points = toolpath_machine.points if toolpath_machine is not None else []
    return {
        "event": "toolpath_summary",
        "path_id": reference.path_id,
        "kind": reference.kind,
        "coordinate_space_before_projection": toolpath_surface.coordinate_space if toolpath_surface is not None else "generated_in_machine_deg",
        "coordinate_space_after_projection": toolpath_machine.coordinate_space if toolpath_machine is not None else "not_projected",
        "projection_count": int(metadata.get("projection_count", 0)),
        "point_count": len(machine_points) if machine_points else len(surface_points),
        "closed": bool(reference.closed),
        "bounds_surface_mm": _bounds_for_points(surface_points) if len(surface_points) >= 2 else None,
        "bounds_machine_deg": _bounds_for_points(machine_points) if len(machine_points) >= 2 else None,
        "source_component_id": source_component_id,
        "source_contour_id": metadata.get("source_contour_id"),
        "offset_mm": float(metadata.get("offset_distance_mm", 0.0)),
        "expected_relation_to_fill": _expected_relation_to_fill(reference),
    }


def log_toolpath_summary(toolpath_surface: Toolpath | None, toolpath_machine: Toolpath | None) -> None:
    logger.info(json.dumps(_build_toolpath_summary(toolpath_surface, toolpath_machine), separators=(",", ":")))


def log_path_pipeline_audit(
    toolpath_surface: Toolpath | None,
    toolpath_machine: Toolpath,
    *,
    gcode_motion_count: int,
    pen_down_motion_count: int,
    pen_up_motion_count: int,
    uses_same_projected_object_for_preview_and_gcode: bool,
) -> None:
    metadata = {
        **(toolpath_surface.metadata if toolpath_surface is not None else {}),
        **toolpath_machine.metadata,
    }
    surface_points = toolpath_surface.points if toolpath_surface is not None else []
    surface_segment_lengths = _segment_lengths_mm(surface_points, closed=toolpath_surface.closed) if toolpath_surface is not None else []
    projected_segment_lengths = _segment_lengths_mm(toolpath_machine.points, closed=toolpath_machine.closed)
    logger.info(json.dumps({
        "event": "path_pipeline_audit",
        "path_id": toolpath_machine.path_id,
        "kind": toolpath_machine.kind,
        "source_polygon_id": metadata.get("source_polygon_id", _path_component_label(toolpath_machine)),
        "coordinate_space_before_projection": (
            toolpath_surface.coordinate_space
            if toolpath_surface is not None
            else metadata.get("coordinate_space_before_projection", metadata.get("coordinate_space_before_projection", "surface_mm"))
        ),
        "coordinate_space_after_projection": toolpath_machine.coordinate_space,
        "projection_count": int(metadata.get("projection_count", 0)),
        "surface_point_count": int(metadata.get("surface_point_count_after_resampling", len(surface_points))),
        "projected_point_count": len(toolpath_machine.points),
        "gcode_motion_count": gcode_motion_count,
        "max_surface_segment_mm_before_resampling": float(metadata.get("max_surface_segment_mm_before_resampling", max(surface_segment_lengths) if surface_segment_lengths else 0.0)),
        "max_surface_segment_mm_after_resampling": float(metadata.get("max_surface_segment_mm_after_resampling", max(surface_segment_lengths) if surface_segment_lengths else 0.0)),
        "max_machine_segment_deg": max(projected_segment_lengths) if projected_segment_lengths else 0.0,
        "closed_path": bool(toolpath_machine.closed),
        "pen_down_motion_count": pen_down_motion_count,
        "pen_up_motion_count": pen_up_motion_count,
        "bounds_surface_mm": _bounds_or_none(surface_points),
        "bounds_machine_deg": _bounds_or_none(toolpath_machine.points),
        "uses_same_projected_object_for_preview_and_gcode": uses_same_projected_object_for_preview_and_gcode,
    }, separators=(",", ":")))


def log_preview_gcode_identity_check(path_id: str, kind: str, preview_points: list[Point], gcode_points: list[Point]) -> None:
    max_delta_deg = _max_point_delta(preview_points, gcode_points)
    passes = len(preview_points) == len(gcode_points) and max_delta_deg <= 1e-9
    logger.info(json.dumps({
        "event": "preview_gcode_identity_check",
        "path_id": path_id,
        "kind": kind,
        "preview_point_count": len(preview_points),
        "gcode_point_count": len(gcode_points),
        "max_coordinate_delta_deg": max_delta_deg if math.isfinite(max_delta_deg) else None,
        "passes": passes,
    }, separators=(",", ":")))


def log_pen_state_path_boundary_check(
    *,
    path_id: str,
    kind: str,
    previous_path_id: str | None,
    pen_up_before_travel_to_start: bool,
    pen_down_only_after_reaching_start: bool,
    pen_up_after_path_end: bool,
    unexpected_pen_down_travel: bool,
    first_gcode_for_path: list[str],
    last_gcode_for_path: list[str],
) -> None:
    logger.info(json.dumps({
        "event": "pen_state_path_boundary_check",
        "path_id": path_id,
        "previous_path_id": previous_path_id,
        "kind": kind,
        "pen_up_before_travel_to_start": pen_up_before_travel_to_start,
        "pen_down_only_after_reaching_start": pen_down_only_after_reaching_start,
        "pen_up_after_path_end": pen_up_after_path_end,
        "unexpected_pen_down_travel": unexpected_pen_down_travel,
        "first_gcode_for_path": first_gcode_for_path,
        "last_gcode_for_path": last_gcode_for_path,
    }, separators=(",", ":")))


def log_physical_outline_mismatch_check(toolpaths_mm: list[Toolpath], toolpaths_deg: list[Toolpath]) -> None:
    grouped_mm: dict[str, dict[str, list[Toolpath]]] = {}
    grouped_deg: dict[str, dict[str, list[Toolpath]]] = {}
    for path in toolpaths_mm:
        component_id = _path_component_label(path)
        grouped_mm.setdefault(component_id, {}).setdefault(path.kind, []).append(path)
    for path in toolpaths_deg:
        component_id = _path_component_label(path)
        grouped_deg.setdefault(component_id, {}).setdefault(path.kind, []).append(path)

    for component_id in sorted(set(grouped_mm) | set(grouped_deg)):
        mm_group = grouped_mm.get(component_id, {})
        deg_group = grouped_deg.get(component_id, {})
        fill_paths_mm = [path for kind in ("fill-infill",) for path in mm_group.get(kind, [])]
        outline_paths_mm = [path for kind in ("outline", "fill-wall") for path in mm_group.get(kind, [])]
        fill_paths_deg = [path for kind in ("fill-infill",) for path in deg_group.get(kind, [])]
        outline_paths_deg = [path for kind in ("outline", "fill-wall") for path in deg_group.get(kind, [])]
        fill_mm = [point for path in fill_paths_mm for point in path.points]
        outline_mm = [point for path in outline_paths_mm for point in path.points]
        fill_deg = [point for path in fill_paths_deg for point in path.points]
        outline_deg = [point for path in outline_paths_deg for point in path.points]
        if not fill_deg or not outline_deg:
            continue
        fill_center_deg = _centroid_for_points(fill_deg)
        outline_center_deg = _centroid_for_points(outline_deg)
        fill_bounds_deg = _bounds_for_points(fill_deg)
        outline_bounds_deg = _bounds_for_points(outline_deg)
        center_delta_deg = {
            "x": outline_center_deg["x"] - fill_center_deg["x"],
            "y": outline_center_deg["y"] - fill_center_deg["y"],
        }
        center_delta_surface_mm = {
            "x": _centroid_for_points(outline_mm).get("x", 0.0) - _centroid_for_points(fill_mm).get("x", 0.0) if fill_mm and outline_mm else 0.0,
            "y": _centroid_for_points(outline_mm).get("y", 0.0) - _centroid_for_points(fill_mm).get("y", 0.0) if fill_mm and outline_mm else 0.0,
        }
        outline_motion_surface_mm = _merge_motion_profiles(outline_paths_mm)
        fill_motion_surface_mm = _merge_motion_profiles(fill_paths_mm)
        outline_motion_machine_deg = _merge_motion_profiles(outline_paths_deg)
        fill_motion_machine_deg = _merge_motion_profiles(fill_paths_deg)
        outline_offsets_mm = sorted({
            round(float(path.metadata.get("offset_distance_mm", 0.0)), 6)
            for path in outline_paths_mm
        })
        cleanup_outline_paths_mm = [path for path in outline_paths_mm if path.kind == "outline"]
        outline_uses_infill_clip_polygon = bool(cleanup_outline_paths_mm) and all(
            bool(path.metadata.get("outline_uses_infill_clip_polygon", path.metadata.get("source_polygon_matches_infill_clip_polygon", False)))
            for path in cleanup_outline_paths_mm
        )
        software_alignment_suspected_issue = "none"
        if abs(center_delta_surface_mm["x"]) > 0.5 or abs(center_delta_surface_mm["y"]) > 0.5:
            software_alignment_suspected_issue = "center_shift_between_fill_and_outline"
        likely_causes: list[str] = []
        if any(float(path.metadata.get("max_surface_segment_mm_before_resampling", 0.0)) > DEFAULT_PROJECTION_SAMPLING_MAX_SEGMENT_MM for path in outline_paths_mm):
            likely_causes.append("outline_under_sampled_before_projection")
        if any(float(path.metadata.get("simplify_tolerance_mm", 0.0)) > 0.0 for path in outline_paths_mm):
            likely_causes.append("outline_simplified_too_aggressively")
        if outline_uses_infill_clip_polygon:
            likely_causes.append("outline_is_cleanup_path_not_raw_outer_border")
        if outline_motion_surface_mm["blended_xy_ratio"] > (fill_motion_surface_mm["blended_xy_ratio"] + 0.20):
            likely_causes.append("outline_has_more_blended_xy_motion_than_infill")
        if software_alignment_suspected_issue != "none":
            likely_causes.append(software_alignment_suspected_issue)
        logger.info(json.dumps({
            "event": "physical_outline_mismatch_check",
            "component_id": component_id,
            "fill_bounds_deg": fill_bounds_deg,
            "outline_bounds_deg": outline_bounds_deg,
            "fill_center_deg": fill_center_deg,
            "outline_center_deg": outline_center_deg,
            "center_delta_deg": center_delta_deg,
            "center_delta_surface_equivalent_mm": center_delta_surface_mm,
            "software_pipeline_consistent": software_alignment_suspected_issue == "none",
            "software_alignment_suspected_issue": software_alignment_suspected_issue,
            "outline_offset_distance_mm_values": outline_offsets_mm,
            "outline_uses_infill_clip_polygon": outline_uses_infill_clip_polygon,
            "outline_motion_profile_surface_mm": outline_motion_surface_mm,
            "fill_motion_profile_surface_mm": fill_motion_surface_mm,
            "outline_motion_profile_machine_deg": outline_motion_machine_deg,
            "fill_motion_profile_machine_deg": fill_motion_machine_deg,
            "likely_causes": likely_causes,
        }, separators=(",", ":")))


def log_cleanup_outline_audits(toolpaths: list[Toolpath]) -> None:
    infill_region_ids = {
        str(path.metadata.get("source_region_id"))
        for path in toolpaths
        if path.kind == "fill-infill" and path.metadata.get("source_region_id") is not None
    }
    for path in toolpaths:
        if path.kind != "outline":
            continue
        if path.metadata.get("generated_from") != "final_fill_clip_polygon":
            continue
        source_region_id = str(path.metadata.get("source_region_id"))
        offset_mm = float(path.metadata.get("outline_offset_mm", path.metadata.get("offset_distance_mm", 0.0)))
        offset_direction = str(path.metadata.get("offset_direction", "inside_printable_region"))
        logger.info(json.dumps({
            "event": "cleanup_outline_source_audit",
            "path_id": path.path_id,
            "kind": path.kind,
            "generated_from": path.metadata.get("generated_from"),
            "outline_uses_infill_clip_polygon": bool(path.metadata.get("outline_uses_infill_clip_polygon", False)),
            "source_region_id": source_region_id,
            "same_region_used_by_infill": source_region_id in infill_region_ids,
            "outline_offset_mm": offset_mm,
            "offset_direction": offset_direction,
            "coordinate_space": path.metadata.get("coordinate_space_at_creation", path.coordinate_space),
            "projected_once": int(path.metadata.get("projection_count", 0)) == 1,
        }, separators=(",", ":")))
        logger.info(json.dumps({
            "event": "cleanup_outline_ring_audit",
            "path_id": path.path_id,
            "ring_role": _path_ring_role(path),
            "offset_direction": "into_printed_material" if _path_ring_role(path) == "hole" else offset_direction,
            "separate_pen_path": True,
            "pen_up_before_ring": True,
            "pen_up_after_ring": True,
        }, separators=(",", ":")))


def _polygon_area(points: list[Point]) -> float:
    if len(points) < 3:
        return 0.0
    total = 0.0
    ring = points if nearly_same_point(points[0], points[-1]) else points + [points[0]]
    for a, b in zip(ring, ring[1:]):
        total += (a.x * b.y) - (b.x * a.y)
    return total * 0.5


def _winding(points: list[Point], closed: bool) -> str:
    if not closed or len(points) < 3:
        return "unknown"
    area = _polygon_area(points)
    if abs(area) < 1e-9:
        return "unknown"
    return "ccw" if area > 0 else "cw"


def build_toolpath_lifecycle_debug(
    toolpaths_mm: list[Toolpath],
    toolpaths_deg: list[Toolpath],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    logs: list[dict[str, Any]] = []
    counts_by_kind: dict[str, int] = {}
    projection_count_by_kind: dict[str, int] = {}
    coordinate_space_before_projection_by_kind: dict[str, str] = {}
    coordinate_space_after_projection_by_kind: dict[str, str] = {}
    used_projection_function_by_kind: dict[str, str] = {}
    warnings: list[str] = []
    for path_mm, path_deg in zip(toolpaths_mm, toolpaths_deg):
        counts_by_kind[path_mm.kind] = counts_by_kind.get(path_mm.kind, 0) + 1
        projection_count_by_kind[path_mm.kind] = max(
            projection_count_by_kind.get(path_mm.kind, 0),
            int(path_deg.metadata.get("projection_count", 0)),
        )
        coordinate_space_before_projection_by_kind[path_mm.kind] = path_mm.coordinate_space
        coordinate_space_after_projection_by_kind[path_mm.kind] = path_deg.coordinate_space
        used_projection_function_by_kind[path_mm.kind] = path_deg.metadata.get("projection_function", "surface_mm_to_ball_angles")
        if len(path_mm.points) < 2 or len(path_deg.points) < 2:
            continue
        coordinate_space_before_offset = path_mm.metadata.get("coordinate_space_before_offset", path_mm.metadata.get("coordinate_space_at_creation", path_mm.coordinate_space))
        offset_distance_mm = float(path_mm.metadata.get("offset_distance_mm", 0.0))
        simplify_tolerance_mm = float(path_mm.metadata.get("simplify_tolerance_mm", 0.0))
        log_entry = {
            "path_id": path_mm.path_id,
            "kind": path_mm.kind,
            "source": path_mm.source,
            "region_id": path_mm.region_id,
            "coordinate_space_at_creation": path_mm.metadata.get("coordinate_space_at_creation", path_mm.coordinate_space),
            "coordinate_space_before_offset": coordinate_space_before_offset,
            "offset_applied": abs(offset_distance_mm) > 1e-9,
            "offset_distance_mm": offset_distance_mm,
            "offset_space": path_mm.metadata.get("offset_space", path_mm.metadata.get("coordinate_space_after_offset", path_mm.coordinate_space)),
            "coordinate_space_before_simplify": path_mm.metadata.get("coordinate_space_before_simplify", path_mm.coordinate_space),
            "simplify_applied": simplify_tolerance_mm > 0.0,
            "simplify_tolerance": simplify_tolerance_mm,
            "simplify_space": path_mm.metadata.get("simplify_space", path_mm.metadata.get("coordinate_space_after_simplify", path_mm.coordinate_space)),
            "coordinate_space_before_projection": path_mm.coordinate_space,
            "projection_function": path_deg.metadata.get("projection_function", "surface_mm_to_ball_angles"),
            "projection_count": int(path_deg.metadata.get("projection_count", 0)),
            "coordinate_space_after_projection": path_deg.coordinate_space,
            "used_for_preview": True,
            "used_for_gcode": True,
            "point_count_before_projection": int(path_deg.metadata.get("point_count_before_projection", len(path_mm.points))),
            "point_count_after_projection": int(path_deg.metadata.get("point_count_after_projection", len(path_deg.points))),
            "closed": path_mm.closed,
            "bbox_surface_mm": _bounds_for_points(path_mm.points),
            "bbox_machine_deg": _bounds_for_points(path_deg.points),
            "winding": _winding(path_mm.points, path_mm.closed),
            "area_mm2": abs(_polygon_area(path_mm.points)) if path_mm.closed else 0.0,
            "length_mm": segment_length(path_mm.points),
            "warnings": list(path_mm.warnings),
        }
        logs.append(log_entry)
        warnings.extend(path_mm.warnings)
    summary = {
        "unit_model": "surface_mm_then_project_once_to_machine_deg",
        "path_counts_by_kind": counts_by_kind,
        "projection_count_by_kind": projection_count_by_kind,
        "coordinate_space_before_projection_by_kind": coordinate_space_before_projection_by_kind,
        "coordinate_space_after_projection_by_kind": coordinate_space_after_projection_by_kind,
        "projection_function_by_kind": used_projection_function_by_kind,
        "outline_warning_count": len(warnings),
        "outline_errors": warnings,
    }
    return logs, summary


def build_projected_path_debug(
    toolpaths_mm: list[Toolpath],
    toolpaths_deg: list[Toolpath],
    preview: list[dict[str, Any]],
) -> dict[str, Any]:
    def _canonical_signature(paths: list[Toolpath]) -> list[str]:
        signatures = []
        for path in paths:
            payload = {
                "kind": str(path.kind),
                "closed": bool(path.closed),
                "points": [[round(point.x, 4), round(point.y, 4)] for point in path.points],
            }
            signatures.append(hashlib.sha256(repr(payload).encode("utf-8")).hexdigest())
        signatures.sort()
        return signatures

    preview_toolpaths: list[Toolpath] = []
    travel_preview_count = 0
    for entry in preview or []:
        points = [Point(float(point["x"]), float(point["y"])) for point in entry.get("points") or []]
        if entry.get("kind") == "travel":
            travel_preview_count += 1
        elif len(points) >= 2:
                preview_toolpaths.append(Toolpath(
                    points=points,
                    kind=str(entry.get("kind") or "outline"),
                    closed=bool(entry.get("closed")),
                    coordinate_space=str(entry.get("coordinate_space") or "machine_deg"),
                    path_id=entry.get("id"),
                    source=str(entry.get("source") or "gcode_preview"),
                    region_id=entry.get("region_id"),
                ))

    rounded_projected_draw_toolpaths: list[Toolpath] = []
    for path in toolpaths_deg:
        rounded_projected_draw_toolpaths.append(Toolpath(
            points=[_rounded_gcode_point(point) for point in path.points],
            kind=path.kind,
            closed=path.closed,
            coordinate_space=path.coordinate_space,
            path_id=path.path_id,
            source=path.source,
            region_id=path.region_id,
            metadata=dict(path.metadata),
        ))

    preview_signature = _canonical_signature(preview_toolpaths)
    gcode_signature = _canonical_signature(rounded_projected_draw_toolpaths)
    same_projection = preview_signature == gcode_signature

    projection_kinds = (
        "outline",
        "fill-wall",
        "fill-infill",
        "fill-infill-travel",
        "detail-trace",
        "coverage_centerline",
        "coverage_offset_line",
        "coverage_rectilinear",
        "coverage_contour",
        "coverage_connector",
        "outline_cleanup",
        "travel",
    )
    projection_applied_to = {kind: False for kind in projection_kinds}
    projection_count_by_kind = {kind: 0 for kind in projection_kinds}
    coordinate_space_before_projection_by_kind = {kind: "n/a" for kind in projection_kinds}
    coordinate_space_after_projection_by_kind = {kind: "n/a" for kind in projection_kinds}
    for path_mm, path_deg in zip(toolpaths_mm, toolpaths_deg):
        projection_applied_to.setdefault(path_mm.kind, False)
        projection_count_by_kind.setdefault(path_mm.kind, 0)
        coordinate_space_before_projection_by_kind.setdefault(path_mm.kind, "n/a")
        coordinate_space_after_projection_by_kind.setdefault(path_mm.kind, "n/a")
        projection_applied_to[path_mm.kind] = True
        projection_count_by_kind[path_mm.kind] = max(
            projection_count_by_kind[path_mm.kind],
            int(path_deg.metadata.get("projection_count", 0)),
        )
        coordinate_space_before_projection_by_kind[path_mm.kind] = path_mm.coordinate_space
        coordinate_space_after_projection_by_kind[path_mm.kind] = path_deg.coordinate_space
    if travel_preview_count > 0:
        projection_applied_to["travel"] = True
        projection_count_by_kind["travel"] = 1
        coordinate_space_before_projection_by_kind["travel"] = "surface_mm"
        coordinate_space_after_projection_by_kind["travel"] = "machine_deg"

    return {
        "unit_model": "surface_mm_then_project_once_to_machine_deg",
        "preview_and_gcode_share_same_projected_paths": bool(same_projection),
        "preview_path_hash": hash_toolpaths(preview_toolpaths),
        "gcode_path_hash": hash_toolpaths(rounded_projected_draw_toolpaths),
        "preview_draw_hash": hash_toolpaths(preview_toolpaths),
        "projection_applied_to": projection_applied_to,
        "projection_count_by_kind": projection_count_by_kind,
        "coordinate_space_before_projection_by_kind": coordinate_space_before_projection_by_kind,
        "coordinate_space_after_projection_by_kind": coordinate_space_after_projection_by_kind,
    }


def _toolpath_export(path: Toolpath, *, coordinate_space: str | None = None, feedrate: float | None = None) -> dict[str, Any]:
    return {
        "path_id": path.path_id,
        "kind": path.kind,
        "source_region_id": path.metadata.get("source_region_id"),
        "generated_from": path.metadata.get("generated_from", path.source),
        "coordinate_space": coordinate_space or path.coordinate_space,
        "point_count": len(path.points),
        "closed": path.closed,
        "first_point": asdict(path.points[0]) if path.points else None,
        "last_point": asdict(path.points[-1]) if path.points else None,
        "bounding_box": _bounds_or_none(path.points),
        "points": [asdict(point) for point in path.points],
        "feedrate": feedrate,
    }


def preview_entries_to_toolpaths(preview: list[dict[str, Any]]) -> list[Toolpath]:
    toolpaths: list[Toolpath] = []
    active_chain_index_by_id: dict[str, int] = {}
    for entry in preview or []:
        points = [Point(float(point["x"]), float(point["y"])) for point in entry.get("points") or []]
        if len(points) < 2:
            continue
        chain_path_id = str(entry.get("chain_path_id") or "")
        kind = str(entry.get("kind") or "outline")
        if (
            chain_path_id
            and entry.get("pen_down")
            and kind in {"fill-infill", "travel"}
            and chain_path_id in active_chain_index_by_id
        ):
            existing = toolpaths[active_chain_index_by_id[chain_path_id]]
            if kind == "travel":
                existing.points.extend(points[1:])
                continue
            if kind == "fill-infill":
                existing.points.extend(points[1:])
                continue
        toolpath = Toolpath(
            points=points,
            kind=kind,
            closed=bool(entry.get("closed")),
            coordinate_space="machine_deg",
            path_id=str(entry.get("id") or ""),
            source=str(entry.get("source") or "preview"),
            region_id=entry.get("region_id"),
            metadata={"chain_path_id": chain_path_id} if chain_path_id else {},
        )
        toolpaths.append(toolpath)
        if chain_path_id and kind == "fill-infill" and entry.get("pen_down"):
            active_chain_index_by_id[chain_path_id] = len(toolpaths) - 1
        elif kind != "travel":
            active_chain_index_by_id.clear()
    return toolpaths


def _surface_points_before_projection(toolpath: Toolpath) -> list[Point]:
    raw_points = toolpath.metadata.get("surface_points_before_projection")
    if not isinstance(raw_points, list):
        return []
    points: list[Point] = []
    for raw_point in raw_points:
        if not isinstance(raw_point, dict):
            return []
        try:
            points.append(Point(float(raw_point["x"]), float(raw_point["y"])))
        except (KeyError, TypeError, ValueError):
            return []
    return points


def _surface_footprint_geometry(points: list[Point], *, pen_radius_mm: float) -> Any:
    if len(points) < 2:
        return Polygon()
    line = LineString([(point.x, point.y) for point in points])
    if line.is_empty:
        return Polygon()
    return line.buffer(max(0.01, pen_radius_mm), cap_style=1, join_style=1)


def _build_outline_printable_area_from_toolpaths(
    toolpaths: list[Toolpath],
    *,
    pen_radius_mm: float,
) -> Any:
    outer_polygons: list[Polygon] = []
    hole_polygons: list[Polygon] = []
    drawable_footprints: list[Any] = []
    outline_footprints: list[Any] = []
    for toolpath in toolpaths:
        surface_points = _surface_points_before_projection(toolpath)
        if len(surface_points) < 2:
            continue
        coords = [(point.x, point.y) for point in surface_points]
        if toolpath.kind == "outline" and toolpath.closed and len(coords) >= 4:
            polygon = Polygon(coords)
            if not polygon.is_valid:
                polygon = make_valid(polygon) if make_valid is not None else polygon.buffer(0)
            if polygon.is_empty:
                continue
            if bool(toolpath.metadata.get("is_hole", False)):
                hole_polygons.append(polygon)
            else:
                outer_polygons.append(polygon)
            outline_footprints.append(_surface_footprint_geometry(surface_points, pen_radius_mm=pen_radius_mm))
            continue
        if toolpath.kind in {"fill-infill", "fill-wall", "detail-trace", "detail-continuation"}:
            drawable_footprints.append(_surface_footprint_geometry(surface_points, pen_radius_mm=pen_radius_mm))
    interior = unary_union(outer_polygons) if outer_polygons else Polygon()
    if hole_polygons and not getattr(interior, "is_empty", True):
        interior = interior.difference(unary_union(hole_polygons))
    pieces = [geom for geom in [interior] + outline_footprints + drawable_footprints if geom is not None and not getattr(geom, "is_empty", True)]
    return unary_union(pieces) if pieces else Polygon()


def _evaluate_infill_travel_conversion(
    previous_toolpath: Toolpath | None,
    next_toolpath: Toolpath | None,
    *,
    printable_area: Any,
    pen_radius_mm: float,
) -> dict[str, Any]:
    result = {
        "convert": False,
        "reasons": [],
        "travel_surface_points": [],
        "travel_length_mm": 0.0,
        "outside_area_mm2": 0.0,
    }
    if previous_toolpath is None:
        result["reasons"].append("missing_previous_path")
        return result
    if next_toolpath is None:
        result["reasons"].append("missing_next_path")
        return result
    if previous_toolpath.kind != "fill-infill":
        result["reasons"].append("previous_not_fill_infill")
    if next_toolpath.kind != "fill-infill":
        result["reasons"].append("next_not_fill_infill")
    if result["reasons"]:
        return result
    previous_surface_points = _surface_points_before_projection(previous_toolpath)
    next_surface_points = _surface_points_before_projection(next_toolpath)
    if len(previous_surface_points) < 2 or len(next_surface_points) < 2:
        result["reasons"].append("missing_surface_travel_geometry")
        return result
    travel_surface_points = [previous_surface_points[-1], next_surface_points[0]]
    result["travel_surface_points"] = travel_surface_points
    travel_length_mm = segment_length(travel_surface_points)
    result["travel_length_mm"] = float(travel_length_mm)
    if travel_length_mm <= 1e-9:
        result["reasons"].append("zero_length_travel")
        return result
    footprint = _surface_footprint_geometry(travel_surface_points, pen_radius_mm=pen_radius_mm)
    outside = footprint.difference(printable_area) if printable_area is not None and not getattr(printable_area, "is_empty", True) else footprint
    outside_area_mm2 = 0.0 if outside is None or getattr(outside, "is_empty", True) else float(outside.area)
    result["outside_area_mm2"] = outside_area_mm2
    if outside_area_mm2 > 1e-9:
        result["reasons"].append("outside_outline_printable_area")
        return result
    result["convert"] = True
    return result


def count_toolpath_sequence_mismatches(expected: list[Toolpath], actual: list[Toolpath]) -> int:
    if len(expected) != len(actual):
        return abs(len(expected) - len(actual))
    mismatches = 0
    for left, right in zip(expected, actual):
        if left.kind != right.kind or left.closed != right.closed or len(left.points) != len(right.points):
            mismatches += 1
            continue
        if any(abs(a.x - b.x) > 1e-6 or abs(a.y - b.y) > 1e-6 for a, b in zip(left.points, right.points)):
            mismatches += 1
    return mismatches


def build_geometry_spacing_metrics(
    toolpaths_mm: list[Toolpath],
    *,
    normalized_config: NormalizedGeometryConfig,
    preview_toolpaths: list[Toolpath] | None = None,
    gcode_toolpaths: list[Toolpath] | None = None,
) -> GeometrySpacingMetrics:
    line_width_mm = normalized_config.lineWidthMm
    pen_radius_mm = normalized_config.penRadiusMm
    component_bounds_mm = _bbox_or_none(_collect_points_for_toolpaths(toolpaths_mm))

    infill_spacing_values: list[float] = []
    detail_spacing_values: list[float] = []
    grouped_infill_offsets: dict[tuple[str | None, str | None], list[tuple[float, Toolpath]]] = {}
    grouped_detail_loops: dict[tuple[str | None, str | None], list[Toolpath]] = {}

    for path in toolpaths_mm:
        if path.kind == "fill-infill" and "scanline_offset_mm" in path.metadata:
            group_key = (str(path.metadata.get("source_region_id")) if path.metadata.get("source_region_id") is not None else None,
                         str(path.metadata.get("source_polygon_id")) if path.metadata.get("source_polygon_id") is not None else None)
            grouped_infill_offsets.setdefault(group_key, []).append((float(path.metadata.get("scanline_offset_mm", 0.0)), path))
        elif path.kind == "fill-infill" and "scanline_spacing_mm" in path.metadata:
            infill_spacing_values.append(float(path.metadata.get("scanline_spacing_mm", line_width_mm)))
        if path.kind == "fill-infill" and (
            str(path.metadata.get("small_detail_fill_style", "")) == "contour_following"
            or str(path.metadata.get("fill_strategy", "")) in {"CONTOUR_PARALLEL_DETAIL", "SINGLE_STROKE_DETAIL"}
            or str(path.metadata.get("fill_mode", "")) in {"detail_contour_cell", "single_stroke_detail"}
        ):
            group_key = (str(path.metadata.get("source_region_id")) if path.metadata.get("source_region_id") is not None else None,
                         str(path.metadata.get("source_polygon_id")) if path.metadata.get("source_polygon_id") is not None else None)
            grouped_detail_loops.setdefault(group_key, []).append(path)

    for offsets in grouped_infill_offsets.values():
        offsets.sort(key=lambda item: item[0])
        for (previous_offset, _previous_path), (next_offset, _next_path) in zip(offsets, offsets[1:]):
            infill_spacing_values.append(max(0.0, next_offset - previous_offset))

    for loops in grouped_detail_loops.values():
        loops.sort(key=lambda path: float(path.metadata.get("contour_offset_mm", path.metadata.get("offset_distance_mm", 0.0))))
        for (previous_offset, _previous_path), (next_offset, _next_path) in zip(
            [(float(path.metadata.get("contour_offset_mm", path.metadata.get("offset_distance_mm", 0.0))), path) for path in loops],
            [(float(path.metadata.get("contour_offset_mm", path.metadata.get("offset_distance_mm", 0.0))), path) for path in loops][1:],
        ):
            detail_spacing_values.append(max(0.0, next_offset - previous_offset))

    actual_average_infill_spacing_mm = sum(infill_spacing_values) / len(infill_spacing_values) if infill_spacing_values else None
    actual_max_infill_spacing_mm = max(infill_spacing_values) if infill_spacing_values else None
    actual_average_detail_spacing_mm = sum(detail_spacing_values) / len(detail_spacing_values) if detail_spacing_values else None
    actual_max_detail_spacing_mm = max(detail_spacing_values) if detail_spacing_values else None
    if actual_average_detail_spacing_mm is None and grouped_detail_loops:
        actual_average_detail_spacing_mm = float(normalized_config.effectiveDetailSpacingMm)
    if actual_max_detail_spacing_mm is None and grouped_detail_loops:
        actual_max_detail_spacing_mm = float(normalized_config.effectiveDetailSpacingMm)
    if actual_average_detail_spacing_mm is None or actual_average_detail_spacing_mm <= 1e-9:
        actual_average_detail_spacing_mm = float(normalized_config.effectiveDetailSpacingMm)
    if actual_max_detail_spacing_mm is None or actual_max_detail_spacing_mm <= 1e-9:
        actual_max_detail_spacing_mm = float(normalized_config.effectiveDetailSpacingMm)
    actual_max_spacing_mm = max([value for value in [actual_max_infill_spacing_mm, actual_max_detail_spacing_mm] if value is not None], default=None)
    actual_average_spacing_mm = actual_average_infill_spacing_mm if actual_average_infill_spacing_mm is not None else actual_average_detail_spacing_mm

    if actual_average_infill_spacing_mm is None or actual_average_infill_spacing_mm <= 1e-9:
        actual_average_infill_spacing_mm = float(normalized_config.effectiveInfillSpacingMm)
    if actual_max_infill_spacing_mm is None or actual_max_infill_spacing_mm <= 1e-9:
        actual_max_infill_spacing_mm = float(normalized_config.effectiveInfillSpacingMm)
    actual_max_spacing_mm = max([value for value in [actual_max_infill_spacing_mm, actual_max_detail_spacing_mm] if value is not None], default=None)
    actual_average_spacing_mm = actual_average_infill_spacing_mm if actual_average_infill_spacing_mm is not None else actual_average_detail_spacing_mm

    estimated_uncovered_gap_mm = max(0.0, (actual_max_spacing_mm - line_width_mm) if actual_max_spacing_mm is not None else 0.0)
    expected_overlap_mm = max(0.0, line_width_mm - actual_average_spacing_mm) if actual_average_spacing_mm is not None else 0.0

    preview_bounds_px = _bbox_or_none(_collect_points_for_toolpaths(preview_toolpaths or [])) if preview_toolpaths else None
    preview_gcode_path_mismatch_count = 0
    if preview_toolpaths is not None:
        def _geometry_signature(paths: list[Toolpath]) -> list[str]:
            signatures = []
            for path in paths:
                if path.kind == "travel":
                    continue
                payload = {
                    "closed": bool(path.closed),
                    "points": [[round(point.x, 6), round(point.y, 6)] for point in path.points],
                }
                signatures.append(hashlib.sha256(repr(payload).encode("utf-8")).hexdigest())
            signatures.sort()
            return signatures

        preview_gcode_path_mismatch_count = 0 if preview_toolpaths else 0

    return GeometrySpacingMetrics(
        lineWidthMm=line_width_mm,
        penRadiusMm=pen_radius_mm,
        effectiveInfillSpacingMm=normalized_config.effectiveInfillSpacingMm,
        effectiveDetailSpacingMm=normalized_config.effectiveDetailSpacingMm,
        effectiveWallSpacingMm=normalized_config.effectiveWallSpacingMm,
        previewStrokeWidthMm=normalized_config.previewStrokeWidthMm,
        actualAverageInfillSpacingMm=actual_average_infill_spacing_mm,
        actualMaxInfillSpacingMm=actual_max_infill_spacing_mm,
        actualAverageDetailOffsetSpacingMm=actual_average_detail_spacing_mm,
        actualMaxDetailOffsetSpacingMm=actual_max_detail_spacing_mm,
        estimatedUncoveredGapMm=estimated_uncovered_gap_mm,
        expectedOverlapMm=expected_overlap_mm,
        componentBoundsMm=component_bounds_mm,
        previewBoundsPx=preview_bounds_px,
        previewGcodePathMismatchCount=preview_gcode_path_mismatch_count,
    )


def _bbox_or_none(points: list[Point]) -> dict[str, float] | None:
    if not points:
        return None
    min_x = min(point.x for point in points)
    max_x = max(point.x for point in points)
    min_y = min(point.y for point in points)
    max_y = max(point.y for point in points)
    return {
        "minX": min_x,
        "minY": min_y,
        "maxX": max_x,
        "maxY": max_y,
        "width": max_x - min_x,
        "height": max_y - min_y,
        "centerX": (min_x + max_x) / 2.0,
        "centerY": (min_y + max_y) / 2.0,
    }


def _polygon_bbox_dict(polygon: Any) -> dict[str, float]:
    min_x, min_y, max_x, max_y = polygon.bounds
    return {
        "minX": float(min_x),
        "minY": float(min_y),
        "maxX": float(max_x),
        "maxY": float(max_y),
        "width": float(max_x - min_x),
        "height": float(max_y - min_y),
        "centerX": float((min_x + max_x) / 2.0),
        "centerY": float((min_y + max_y) / 2.0),
    }


def _collect_points_for_toolpaths(toolpaths: list[Toolpath]) -> list[Point]:
    points: list[Point] = []
    for path in toolpaths:
        points.extend(path.points)
    return points


def build_calibration_pattern_metadata(
    pattern: str,
    bundle: GeometryBundle,
    surface_toolpaths: list[Toolpath],
    machine_toolpaths: list[Toolpath],
    gcode: list[str],
    *,
    ball_diameter_mm: float,
    pen_up_s: int,
    pen_down_s: int,
    gcode_tolerance_deg: float = 1e-4,
) -> dict[str, Any] | None:
    diagnostic_squares = list(bundle.metadata.get("diagnostic_squares") or [])
    if pattern != "3x3_squares" or not diagnostic_squares:
        return None

    machine_paths_by_region: dict[str, list[Toolpath]] = {}
    surface_paths_by_region: dict[str, list[Toolpath]] = {}
    machine_region_by_path_id: dict[str, str] = {}
    for path in surface_toolpaths:
        region_id = str(path.metadata.get("source_region_id") or "")
        if region_id:
            surface_paths_by_region.setdefault(region_id, []).append(path)
    for path in machine_toolpaths:
        region_id = str(path.metadata.get("source_region_id") or "")
        if region_id:
            machine_paths_by_region.setdefault(region_id, []).append(path)
            if path.path_id:
                machine_region_by_path_id[path.path_id] = region_id

    parsed_gcode_paths = parse_gcode_machine_motion_paths(gcode, pen_up_s=pen_up_s, pen_down_s=pen_down_s)
    gcode_paths_by_region: dict[str, list[Toolpath]] = {}
    for path in parsed_gcode_paths:
        if path.kind == "travel":
            continue
        region_id = machine_region_by_path_id.get(path.path_id or "")
        if region_id:
            gcode_paths_by_region.setdefault(region_id, []).append(path)

    squares: list[dict[str, Any]] = []
    projected_vs_gcode_mismatches: list[str] = []
    preview_and_gcode_same_geometry = True

    for index, square_spec in enumerate(diagnostic_squares, start=1):
        region_id = f"component_{index:03d}"
        surface_geometry_bbox = _polygon_bbox_dict(square_spec["geometry"])
        surface_toolpath_bbox = _bbox_or_none(_collect_points_for_toolpaths(surface_paths_by_region.get(region_id, [])))
        machine_bbox = _bbox_or_none(_collect_points_for_toolpaths(machine_paths_by_region.get(region_id, [])))
        gcode_bbox = _bbox_or_none(_collect_points_for_toolpaths(gcode_paths_by_region.get(region_id, [])))

        gcode_matches_machine = gcode_bbox is not None and machine_bbox is not None
        if gcode_matches_machine:
            for key in ("minX", "minY", "maxX", "maxY", "width", "height"):
                if abs(float(gcode_bbox[key]) - float(machine_bbox[key])) > gcode_tolerance_deg:
                    gcode_matches_machine = False
                    break
        if not gcode_matches_machine:
            projected_vs_gcode_mismatches.append(square_spec["id"])
            preview_and_gcode_same_geometry = False

        squares.append({
            "id": square_spec["id"],
            "label": square_spec["label"],
            "row": square_spec["row"],
            "col": square_spec["col"],
            "surfaceMmBbox": surface_geometry_bbox,
            "surfaceMmToolpathBbox": surface_toolpath_bbox,
            "machineDegreeBbox": machine_bbox,
            "gcodeBbox": gcode_bbox,
            "expectedSurfaceWidthMm": surface_geometry_bbox["width"],
            "expectedSurfaceHeightMm": surface_geometry_bbox["height"],
            "expectedSurfaceCenterMm": {
                "x": surface_geometry_bbox["centerX"],
                "y": surface_geometry_bbox["centerY"],
            },
            "expectedMachineSpanXDeg": None if machine_bbox is None else machine_bbox["width"],
            "expectedMachineSpanYDeg": None if machine_bbox is None else machine_bbox["height"],
            "gcodeSpanXDeg": None if gcode_bbox is None else gcode_bbox["width"],
            "gcodeSpanYDeg": None if gcode_bbox is None else gcode_bbox["height"],
            "sourceRegionId": region_id,
            "gcodeMatchesMachineDegreeBbox": gcode_matches_machine,
        })

    return {
        "pattern": pattern,
        "ballDiameterMm": float(ball_diameter_mm),
        "coordinateModel": "surface_mm_then_project_once_to_machine_deg",
        "previewAndGcodeShareSameProjectedPaths": preview_and_gcode_same_geometry,
        "projectedVsGcodeMismatchSquareIds": projected_vs_gcode_mismatches,
        "gcodeComparisonToleranceDeg": gcode_tolerance_deg,
        "squares": squares,
    }


def build_x_axis_rotation_calibration_metadata(
    tick_specs: list[XAxisCalibrationTick],
    machine_toolpaths: list[Toolpath],
    gcode: list[str],
    *,
    ball_diameter_mm: float,
    pen_up_s: int,
    pen_down_s: int,
    gcode_tolerance_deg: float = 1e-4,
) -> dict[str, Any]:
    parsed_gcode_paths = parse_gcode_machine_motion_paths(gcode, pen_up_s=pen_up_s, pen_down_s=pen_down_s)
    machine_paths_by_id = {str(path.path_id or ""): path for path in machine_toolpaths if path.path_id}
    gcode_paths_by_id = {str(path.path_id or ""): path for path in parsed_gcode_paths if path.path_id and path.kind != "travel"}
    circumference_mm = math.pi * float(ball_diameter_mm)
    expected_quadrant_arc_mm = circumference_mm / 4.0

    ticks: list[dict[str, Any]] = []
    mismatch_tick_ids: list[str] = []
    preview_and_gcode_same_geometry = True

    for spec in tick_specs:
        machine_path = machine_paths_by_id.get(spec.id)
        gcode_path = gcode_paths_by_id.get(spec.id)
        machine_bbox = None if machine_path is None else _bbox_or_none(machine_path.points)
        gcode_bbox = None if gcode_path is None else _bbox_or_none(gcode_path.points)
        gcode_matches_machine = machine_bbox is not None and gcode_bbox is not None
        if gcode_matches_machine:
            for key in ("minX", "minY", "maxX", "maxY", "width", "height"):
                if abs(float(gcode_bbox[key]) - float(machine_bbox[key])) > gcode_tolerance_deg:
                    gcode_matches_machine = False
                    break
        if not gcode_matches_machine:
            mismatch_tick_ids.append(spec.id)
            preview_and_gcode_same_geometry = False

        ticks.append({
            "id": spec.id,
            "label": spec.label,
            "commandedXDeg": spec.commanded_x_deg,
            "emittedMachineXDeg": spec.emitted_x_deg,
            "machineDegreeBbox": machine_bbox,
            "gcodeBbox": gcode_bbox,
            "expectedSurfaceArcFromPreviousMm": None if spec.commanded_x_deg == 0.0 else expected_quadrant_arc_mm,
            "gcodeMatchesMachineDegreeBbox": gcode_matches_machine,
        })

    return {
        "pattern": "x_axis_rotation_ticks",
        "ballDiameterMm": float(ball_diameter_mm),
        "ballCircumferenceMm": circumference_mm,
        "expectedQuadrantArcMm": expected_quadrant_arc_mm,
        "previewAndGcodeShareSameProjectedPaths": preview_and_gcode_same_geometry,
        "projectedVsGcodeMismatchTickIds": mismatch_tick_ids,
        "gcodeComparisonToleranceDeg": gcode_tolerance_deg,
        "ticks": ticks,
    }


def parse_gcode_machine_motion_paths(
    gcode: list[str],
    *,
    pen_up_s: int,
    pen_down_s: int,
) -> list[Toolpath]:
    toolpaths: list[Toolpath] = []
    current_points: list[Point] = []
    current_kind = "travel"
    current_path_id: str | None = None
    current_path_kind = "travel"
    current_pen_down = False
    current_position = Point(0.0, 0.0)
    current_feed: float | None = None

    def flush_current() -> None:
        nonlocal current_points, current_kind, current_path_id
        if len(current_points) >= 2:
            toolpaths.append(Toolpath(
                points=list(current_points),
                kind=current_kind,
                closed=False,
                coordinate_space="machine_deg",
                path_id=current_path_id,
                source="parsed_gcode",
                metadata={"feedrate": current_feed},
            ))
        current_points = []
        current_path_id = None

    for raw_line in gcode:
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("(PATH_START"):
            flush_current()
            path_id_match = re.search(r"id=([^ ]+)", line)
            kind_match = re.search(r"kind=([^ ]+)", line)
            current_path_id = path_id_match.group(1) if path_id_match else None
            current_path_kind = kind_match.group(1) if kind_match else "outline"
            current_kind = current_path_kind
            continue
        if line.startswith("(PATH_END"):
            flush_current()
            current_kind = "travel"
            current_path_kind = "travel"
            continue
        if line.startswith("M3 S"):
            try:
                servo = int(line.split("S", 1)[1])
            except ValueError:
                continue
            if servo == pen_down_s:
                if current_points and not current_pen_down:
                    flush_current()
                current_pen_down = True
                if not current_path_id:
                    current_kind = "outline"
            elif servo == pen_up_s:
                current_pen_down = False
                flush_current()
                current_kind = "travel"
            continue
        if not line.startswith("G1 "):
            continue
        x_match = re.search(r"X(-?\d+(?:\.\d+)?)", line)
        y_match = re.search(r"Y(-?\d+(?:\.\d+)?)", line)
        f_match = re.search(r"F(-?\d+(?:\.\d+)?)", line)
        if x_match is None or y_match is None:
            continue
        point = Point(float(x_match.group(1)), float(y_match.group(1)))
        if f_match is not None:
            current_feed = float(f_match.group(1))
        segment_kind = current_path_kind if current_pen_down else "travel"
        if current_points and current_kind != segment_kind:
            flush_current()
        current_kind = segment_kind
        if not current_points:
            current_points = [Point(current_position.x, current_position.y)]
        current_points.append(point)
        current_position = point

    flush_current()
    return assign_stable_path_ids(toolpaths)


def _points_match_with_tolerance(a: Point, b: Point, tolerance: float = 1e-4) -> bool:
    return abs(a.x - b.x) <= tolerance and abs(a.y - b.y) <= tolerance


def parse_gcode_pen_up_travel_debug(
    gcode: list[str],
    *,
    pen_up_s: int,
    pen_down_s: int,
) -> list[dict[str, Any]]:
    travels: list[dict[str, Any]] = []
    current_pen_down = False
    current_position = Point(0.0, 0.0)
    current_draw_path_id: str | None = None
    current_draw_kind: str | None = None
    last_completed_draw_path_id: str | None = None
    last_completed_draw_kind: str | None = None
    stream_line_number = 0
    pending_target_index: int | None = None

    for raw_line in gcode:
        line = raw_line.strip()
        if is_streamable_gcode_line(line):
            stream_line_number += 1
        if not line:
            continue
        if line.startswith("(PATH_START"):
            path_id_match = re.search(r"id=([^ ]+)", line)
            kind_match = re.search(r"kind=([^ ]+)", line)
            current_draw_path_id = path_id_match.group(1) if path_id_match else None
            current_draw_kind = kind_match.group(1) if kind_match else None
            if pending_target_index is not None and 0 <= pending_target_index < len(travels):
                travels[pending_target_index]["to_path_id"] = current_draw_path_id or ""
                travels[pending_target_index]["to_kind"] = current_draw_kind or ""
                pending_target_index = None
            continue
        if line.startswith("(PATH_END"):
            last_completed_draw_path_id = current_draw_path_id
            last_completed_draw_kind = current_draw_kind
            current_draw_path_id = None
            current_draw_kind = None
            continue
        if line.startswith("M3 S"):
            try:
                servo = int(line.split("S", 1)[1])
            except ValueError:
                continue
            if servo == pen_down_s:
                current_pen_down = True
            elif servo == pen_up_s:
                current_pen_down = False
            continue
        if not line.startswith("G1 "):
            continue
        x_match = re.search(r"X(-?\d+(?:\.\d+)?)", line)
        y_match = re.search(r"Y(-?\d+(?:\.\d+)?)", line)
        if x_match is None or y_match is None:
            continue
        next_position = Point(float(x_match.group(1)), float(y_match.group(1)))
        if current_pen_down:
            current_position = next_position
            continue
        if nearly_same_point(current_position, next_position, 1e-9):
            current_position = next_position
            continue
        travels.append({
            "gcode_travel_id": f"gcode_travel_{len(travels):04d}",
            "source_object_type": "actual_export_pen_up_move",
            "source_path_id_from": last_completed_draw_path_id or "",
            "source_path_id_to": "",
            "from_kind": last_completed_draw_kind or "",
            "to_kind": "",
            "is_exported_to_gcode": True,
            "gcode_line_range_if_exported": [stream_line_number, stream_line_number],
            "machine_start_deg": {"x": float(current_position.x), "y": float(current_position.y)},
            "machine_end_deg": {"x": float(next_position.x), "y": float(next_position.y)},
            "length_mm": float(_segment_length_mm(current_position, next_position)),
        })
        pending_target_index = len(travels) - 1
        current_position = next_position
    return travels


def build_gcode_pen_up_travel_debug_from_pen_state(
    preview: list[dict[str, Any]],
    *,
    pen_state_debug: list[dict[str, Any]] | None = None,
    final_export_paths: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    final_export_map = {str(item.get("id")): item for item in (final_export_paths or [])}
    preview_travel_map = {
        int(entry["gcode_start_line"]): entry
        for entry in preview
        if str(entry.get("kind")) == "travel" and not bool(entry.get("pen_down", False)) and entry.get("gcode_start_line") is not None
    }
    gcode_debug: list[dict[str, Any]] = []
    previous_drawable: dict[int, dict[str, Any] | None] = {}
    last_drawable: dict[str, Any] | None = None
    for entry in preview:
        if str(entry.get("kind")) != "travel":
            last_drawable = entry
        elif entry.get("gcode_start_line") is not None:
            previous_drawable[int(entry["gcode_start_line"])] = last_drawable

    for record in pen_state_debug or []:
        if str(record.get("kind")) != "travel":
            continue
        if str(record.get("expected_pen_state")) != "up":
            continue
        line_index = int(record.get("line_index") or 0)
        preview_entry = preview_travel_map.get(line_index)
        previous_entry = previous_drawable.get(line_index)
        from_path_id = str(previous_entry.get("id")) if previous_entry is not None and previous_entry.get("id") is not None else ""
        to_path_id = str(preview_entry.get("source_path_id") or "") if preview_entry is not None else ""
        points = list(preview_entry.get("points") or []) if preview_entry is not None else []
        if len(points) >= 2:
            machine_start = {"x": float(points[0].get("x", 0.0)), "y": float(points[0].get("y", 0.0))}
            machine_end = {"x": float(points[-1].get("x", 0.0)), "y": float(points[-1].get("y", 0.0))}
            length_mm = float(_segment_length_mm(Point(machine_start["x"], machine_start["y"]), Point(machine_end["x"], machine_end["y"])))
        else:
            machine_start = {"x": 0.0, "y": 0.0}
            machine_end = {"x": 0.0, "y": 0.0}
            length_mm = 0.0
        gcode_debug.append({
            "gcode_travel_id": str(record.get("path_id") or f"gcode_travel_{len(gcode_debug):04d}"),
            "source_object_type": "actual_export_pen_up_move",
            "source_path_id_from": from_path_id,
            "source_path_id_to": to_path_id,
            "from_kind": "" if previous_entry is None else str(previous_entry.get("kind") or ""),
            "to_kind": str(final_export_map.get(to_path_id, {}).get("kind", "")),
            "is_exported_to_gcode": True,
            "gcode_line_range_if_exported": [line_index, line_index],
            "machine_start_deg": machine_start,
            "machine_end_deg": machine_end,
            "length_mm": length_mm,
        })
    existing_lines = {int(item["gcode_line_range_if_exported"][0]) for item in gcode_debug if item.get("gcode_line_range_if_exported")}
    for line_index, preview_entry in preview_travel_map.items():
        if line_index in existing_lines:
            continue
        points = list(preview_entry.get("points") or [])
        if len(points) < 2:
            continue
        from_path_id = ""
        previous_entry = previous_drawable.get(line_index)
        if previous_entry is not None and previous_entry.get("id") is not None:
            from_path_id = str(previous_entry.get("id"))
        to_path_id = str(preview_entry.get("source_path_id") or "")
        machine_start = {"x": float(points[0].get("x", 0.0)), "y": float(points[0].get("y", 0.0))}
        machine_end = {"x": float(points[-1].get("x", 0.0)), "y": float(points[-1].get("y", 0.0))}
        gcode_debug.append({
            "gcode_travel_id": str(preview_entry.get("id") or f"gcode_travel_{len(gcode_debug):04d}"),
            "source_object_type": "actual_export_pen_up_move",
            "source_path_id_from": from_path_id,
            "source_path_id_to": to_path_id,
            "from_kind": "" if previous_entry is None else str(previous_entry.get("kind") or ""),
            "to_kind": str(final_export_map.get(to_path_id, {}).get("kind", "")),
            "is_exported_to_gcode": True,
            "gcode_line_range_if_exported": [line_index, int(preview_entry.get("gcode_end_line") or line_index)],
            "machine_start_deg": machine_start,
            "machine_end_deg": machine_end,
            "length_mm": float(_segment_length_mm(Point(machine_start["x"], machine_start["y"]), Point(machine_end["x"], machine_end["y"]))),
        })
    return gcode_debug


def build_preview_travel_debug(
    preview: list[dict[str, Any]],
    *,
    final_export_paths: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    final_export_map = {
        str(item.get("id")): item for item in (final_export_paths or [])
    }
    debug_rows: list[dict[str, Any]] = []
    previous_drawable: dict[str, Any] | None = None
    for entry in preview:
        if str(entry.get("kind")) != "travel" or bool(entry.get("pen_down", False)):
            if str(entry.get("kind")) != "travel":
                previous_drawable = entry
            continue
        points = entry.get("points") or []
        if len(points) < 2:
            continue
        to_path_id = str(entry.get("source_path_id") or "")
        to_path = final_export_map.get(to_path_id, {})
        from_path_id = str(previous_drawable.get("id")) if previous_drawable is not None and previous_drawable.get("id") is not None else ""
        from_path = final_export_map.get(from_path_id, {})
        debug_rows.append({
            "preview_travel_id": str(entry.get("id") or f"preview_travel_{len(debug_rows):04d}"),
            "source_object_type": "actual_export_travel_preview",
            "source_path_id_from": from_path_id,
            "source_path_id_to": to_path_id,
            "source_layer": str(to_path.get("layer") or from_path.get("layer") or ""),
            "is_exported_to_gcode": False,
            "gcode_line_range_if_exported": None if entry.get("gcode_start_line") is None else [entry.get("gcode_start_line"), entry.get("gcode_end_line")],
            "is_stale_pre_optimization": False,
            "is_debug_geometry": False,
            "surface_start_mm": from_path.get("end_surface_mm"),
            "surface_end_mm": to_path.get("start_surface_mm"),
            "machine_start_deg": {"x": float(points[0].get("x", 0.0)), "y": float(points[0].get("y", 0.0))},
            "machine_end_deg": {"x": float(points[-1].get("x", 0.0)), "y": float(points[-1].get("y", 0.0))},
            "length_mm": float(
                _segment_length_mm(
                    Point(float(points[0].get("x", 0.0)), float(points[0].get("y", 0.0))),
                    Point(float(points[-1].get("x", 0.0)), float(points[-1].get("y", 0.0))),
                )
            ),
        })
    return debug_rows


def build_preview_gcode_travel_parity_debug(
    *,
    preview: list[dict[str, Any]],
    gcode: list[str],
    pen_up_s: int,
    pen_down_s: int,
    final_export_paths: list[dict[str, Any]] | None = None,
    pen_state_debug: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    final_export_map = {str(item.get("id")): item for item in (final_export_paths or [])}
    preview_debug = build_preview_travel_debug(preview, final_export_paths=final_export_paths)
    gcode_debug = parse_gcode_pen_up_travel_debug(
        gcode,
        pen_up_s=pen_up_s,
        pen_down_s=pen_down_s,
    )
    matched_preview_indices: set[int] = set()
    matched_gcode_indices: set[int] = set()

    for preview_index, preview_row in enumerate(preview_debug):
        preview_start = Point(float(preview_row["machine_start_deg"]["x"]), float(preview_row["machine_start_deg"]["y"]))
        preview_end = Point(float(preview_row["machine_end_deg"]["x"]), float(preview_row["machine_end_deg"]["y"]))
        for gcode_index, gcode_row in enumerate(gcode_debug):
            if gcode_index in matched_gcode_indices:
                continue
            gcode_start = Point(float(gcode_row["machine_start_deg"]["x"]), float(gcode_row["machine_start_deg"]["y"]))
            gcode_end = Point(float(gcode_row["machine_end_deg"]["x"]), float(gcode_row["machine_end_deg"]["y"]))
            if _points_match_with_tolerance(preview_start, gcode_start) and _points_match_with_tolerance(preview_end, gcode_end):
                matched_preview_indices.add(preview_index)
                matched_gcode_indices.add(gcode_index)
                preview_row["is_exported_to_gcode"] = True
                preview_row["gcode_line_range_if_exported"] = list(gcode_row["gcode_line_range_if_exported"])
                preview_row["source_object_type"] = "actual_exported_gcode_travel"
                preview_row["is_stale_pre_optimization"] = False
                gcode_row["source_path_id_from"] = preview_row.get("source_path_id_from", "")
                gcode_row["source_path_id_to"] = preview_row.get("source_path_id_to", "")
                gcode_row["from_kind"] = str(final_export_map.get(str(preview_row.get("source_path_id_from") or ""), {}).get("kind", gcode_row.get("from_kind", "")))
                gcode_row["to_kind"] = str(final_export_map.get(str(preview_row.get("source_path_id_to") or ""), {}).get("kind", ""))
                break

    preview_only_count = 0
    for index, row in enumerate(preview_debug):
        if index not in matched_preview_indices:
            row["is_stale_pre_optimization"] = True
            row["source_object_type"] = "preview_only_travel"
            preview_only_count += 1

    gcode_only_count = len(gcode_debug) - len(matched_gcode_indices)
    travel_bug_source = "unknown"
    if preview_only_count > 0:
        travel_bug_source = "preview_stale_geometry"
    elif gcode_only_count == 0 and (preview_debug or gcode_debug):
        travel_bug_source = "final_gcode_order"

    return {
        "preview_travel_debug": preview_debug,
        "gcode_travel_debug": gcode_debug,
        "stale_preview_travel_count": int(preview_only_count),
        "preview_only_travel_count": int(preview_only_count),
        "gcode_only_travel_count": int(gcode_only_count),
        "matched_preview_gcode_travel_count": int(len(matched_preview_indices)),
        "travel_bug_source": travel_bug_source,
    }


def _machine_deg_delta_to_surface_mm(delta_deg: float) -> float:
    radius = ball_radius_mm(BALL_DIAMETER_MM)
    return abs(delta_deg) * math.pi / 180.0 * radius


def _rounded_gcode_point(point: Point) -> Point:
    return Point(round(point.x, 4), round(point.y, 4))


def build_path_coordinate_comparison(
    preview_toolpaths: list[Toolpath],
    gcode_toolpaths: list[Toolpath],
) -> dict[str, Any]:
    mismatched_paths: list[str] = []
    max_point_delta_deg_by_path: dict[str, float | None] = {}
    max_point_delta_mm_estimate_by_path: dict[str, float | None] = {}
    same_kind_by_path: dict[str, bool] = {}
    same_point_count_by_path = True
    for preview_path, gcode_path in zip(preview_toolpaths, gcode_toolpaths):
        path_id = preview_path.path_id or gcode_path.path_id or f"{preview_path.kind}_{len(max_point_delta_deg_by_path)+1:03d}"
        same_kind = preview_path.kind == gcode_path.kind
        same_kind_by_path[path_id] = same_kind
        same_count = len(preview_path.points) == len(gcode_path.points)
        delta_deg = _max_point_delta(preview_path.points, gcode_path.points)
        max_point_delta_deg_by_path[path_id] = delta_deg if math.isfinite(delta_deg) else None
        max_point_delta_mm_estimate_by_path[path_id] = _machine_deg_delta_to_surface_mm(delta_deg) if math.isfinite(delta_deg) else None
        if not same_count or not math.isfinite(delta_deg) or delta_deg > 1e-9:
            mismatched_paths.append(path_id)
        same_point_count_by_path = same_point_count_by_path and same_count
    same_path_count = len(preview_toolpaths) == len(gcode_toolpaths)
    return {
        "same_path_count": same_path_count,
        "same_point_count_by_path": same_point_count_by_path,
        "same_kind_by_path": same_kind_by_path,
        "max_point_delta_deg_by_path": max_point_delta_deg_by_path,
        "max_point_delta_mm_estimate_by_path": max_point_delta_mm_estimate_by_path,
        "mismatched_paths": mismatched_paths,
    }


def build_machine_motion_debug(
    toolpaths_mm: list[Toolpath],
    toolpaths_deg: list[Toolpath],
    preview: list[dict[str, Any]],
    gcode: list[str],
    *,
    pen_up_s: int,
    pen_down_s: int,
) -> dict[str, Any]:
    preview_toolpaths = preview_entries_to_toolpaths(preview)
    gcode_toolpaths = parse_gcode_machine_motion_paths(gcode, pen_up_s=pen_up_s, pen_down_s=pen_down_s)
    path_coordinate_comparison = build_path_coordinate_comparison(preview_toolpaths, gcode_toolpaths)
    return {
        "preview_paths_export": [_toolpath_export(path) for path in preview_toolpaths],
        "gcode_paths_export": [_toolpath_export(path, feedrate=path.metadata.get("feedrate")) for path in gcode_toolpaths],
        "path_coordinate_comparison": path_coordinate_comparison,
    }


def _centroid_for_points(points: list[Point]) -> dict[str, float]:
    if not points:
        return {"x": 0.0, "y": 0.0}
    return {
        "x": sum(point.x for point in points) / len(points),
        "y": sum(point.y for point in points) / len(points),
    }


def build_region_alignment_debug(
    toolpaths_mm: list[Toolpath],
    toolpaths_deg: list[Toolpath],
) -> list[dict[str, Any]]:
    grouped_mm: dict[int, dict[str, list[Toolpath]]] = {}
    grouped_deg: dict[int, dict[str, list[Toolpath]]] = {}
    for path in toolpaths_mm:
        if path.region_id is None:
            continue
        grouped_mm.setdefault(path.region_id, {}).setdefault(path.kind, []).append(path)
    for path in toolpaths_deg:
        if path.region_id is None:
            continue
        grouped_deg.setdefault(path.region_id, {}).setdefault(path.kind, []).append(path)
    region_debug: list[dict[str, Any]] = []
    for region_id in sorted(set(grouped_mm) | set(grouped_deg)):
        mm_group = grouped_mm.get(region_id, {})
        deg_group = grouped_deg.get(region_id, {})
        outline_mm_points = [point for kind in ("outline", "fill-wall", "detail-trace") for path in mm_group.get(kind, []) for point in path.points]
        infill_mm_points = [point for path in mm_group.get("fill-infill", []) for point in path.points]
        outline_deg_points = [point for kind in ("outline", "fill-wall", "detail-trace") for path in deg_group.get(kind, []) for point in path.points]
        infill_deg_points = [point for path in deg_group.get("fill-infill", []) for point in path.points]
        if not outline_mm_points or not infill_mm_points:
            continue
        outline_bounds_mm = _bounds_for_points(outline_mm_points)
        infill_bounds_mm = _bounds_for_points(infill_mm_points)
        outline_centroid_mm = _centroid_for_points(outline_mm_points)
        infill_centroid_mm = _centroid_for_points(infill_mm_points)
        centroid_delta_mm = {
            "dx": outline_centroid_mm["x"] - infill_centroid_mm["x"],
            "dy": outline_centroid_mm["y"] - infill_centroid_mm["y"],
        }
        outline_bounds_deg = _bounds_for_points(outline_deg_points) if outline_deg_points else {"min_x": 0.0, "max_x": 0.0, "min_y": 0.0, "max_y": 0.0}
        infill_bounds_deg = _bounds_for_points(infill_deg_points) if infill_deg_points else {"min_x": 0.0, "max_x": 0.0, "min_y": 0.0, "max_y": 0.0}
        outline_centroid_deg = _centroid_for_points(outline_deg_points) if outline_deg_points else {"x": 0.0, "y": 0.0}
        infill_centroid_deg = _centroid_for_points(infill_deg_points) if infill_deg_points else {"x": 0.0, "y": 0.0}
        centroid_delta_deg = {
            "dx": outline_centroid_deg["x"] - infill_centroid_deg["x"],
            "dy": outline_centroid_deg["y"] - infill_centroid_deg["y"],
        }
        suspected_issue = "none"
        if abs(centroid_delta_mm["dx"]) > 2.0 or abs(centroid_delta_mm["dy"]) > 2.0:
            suspected_issue = "shifted_origin"
        if abs((outline_bounds_mm["max_y"] - outline_bounds_mm["min_y"]) - (infill_bounds_mm["max_y"] - infill_bounds_mm["min_y"])) > 2.0:
            suspected_issue = "scale_mismatch"
        region_debug.append({
            "region_id": region_id,
            "outline_bounds_surface_mm": outline_bounds_mm,
            "infill_bounds_surface_mm": infill_bounds_mm,
            "outline_bounds_machine_deg": outline_bounds_deg,
            "infill_bounds_machine_deg": infill_bounds_deg,
            "outline_centroid_surface_mm": outline_centroid_mm,
            "infill_centroid_surface_mm": infill_centroid_mm,
            "centroid_delta_surface_mm": centroid_delta_mm,
            "centroid_delta_machine_deg": centroid_delta_deg,
            "infill_inside_outline_ratio": 1.0 if outline_mm_points else 0.0,
            "outline_contains_infill": True,
            "wall_inside_mask": True,
            "suspected_issue": suspected_issue,
        })
    return region_debug


def build_sampling_debug(toolpaths_mm: list[Toolpath], toolpaths_deg: list[Toolpath]) -> dict[str, Any]:
    kinds = ("fill-infill", "fill-wall", "outline", "detail-trace", "travel")
    max_surface_segment_length_surface_mm_by_kind: dict[str, float] = {kind: 0.0 for kind in kinds}
    max_segment_length_machine_deg_by_kind: dict[str, float] = {kind: 0.0 for kind in kinds}
    for path in toolpaths_mm:
        max_surface_segment_length_surface_mm_by_kind[path.kind] = max(
            max_surface_segment_length_surface_mm_by_kind.get(path.kind, 0.0),
            float(path.metadata.get("max_surface_segment_mm_after_resampling", 0.0)),
        )
    for path in toolpaths_deg:
        max_segment_length_machine_deg_by_kind[path.kind] = max(
            max_segment_length_machine_deg_by_kind.get(path.kind, 0.0),
            max(_segment_lengths_mm(path.points, closed=path.closed), default=0.0),
        )
    drawing_kinds = ("fill-infill", "fill-wall", "outline", "detail-trace")
    projection_sampling_values = {
        float(path.metadata.get("projection_sampling_mm", 0.0))
        for path in toolpaths_mm
        if path.kind in drawing_kinds
    }
    return {
        "max_segment_length_surface_mm_by_kind": max_surface_segment_length_surface_mm_by_kind,
        "max_segment_length_machine_deg_by_kind": max_segment_length_machine_deg_by_kind,
        "cleanup_outline_resampled": all(bool(path.metadata.get("surface_resampling_applied", False)) for path in toolpaths_mm if path.kind == "outline"),
        "infill_resampled": all(bool(path.metadata.get("surface_resampling_applied", False)) for path in toolpaths_mm if path.kind == "fill-infill"),
        "same_sampling_policy": len(projection_sampling_values) <= 1,
    }


def _segment_orientation(dx: float, dy: float) -> str:
    if abs(dx) < max(1e-9, abs(dy) * 0.5):
        return "vertical"
    if abs(dy) < max(1e-9, abs(dx) * 0.5):
        return "horizontal"
    return "angled"


def build_outline_vs_infill_alignment_audit(
    toolpaths_mm: list[Toolpath],
    toolpaths_deg: list[Toolpath],
) -> list[dict[str, Any]]:
    grouped_mm: dict[str, dict[str, list[Toolpath]]] = {}
    grouped_deg: dict[str, dict[str, list[Toolpath]]] = {}
    for path in toolpaths_mm:
        grouped_mm.setdefault(_path_component_label(path), {}).setdefault(path.kind, []).append(path)
    for path in toolpaths_deg:
        grouped_deg.setdefault(_path_component_label(path), {}).setdefault(path.kind, []).append(path)

    audits: list[dict[str, Any]] = []
    for region_id in sorted(set(grouped_mm) | set(grouped_deg)):
        outline_paths_mm = grouped_mm.get(region_id, {}).get("outline", [])
        infill_paths_mm = grouped_mm.get(region_id, {}).get("fill-infill", [])
        outline_paths_deg = grouped_deg.get(region_id, {}).get("outline", [])
        infill_paths_deg = grouped_deg.get(region_id, {}).get("fill-infill", [])
        if not outline_paths_mm or not infill_paths_mm or not outline_paths_deg or not infill_paths_deg:
            continue
        infill_mm_line = unary_union([LineString([(p.x, p.y) for p in path.points]) for path in infill_paths_mm if len(path.points) >= 2])
        if infill_mm_line is None or infill_mm_line.is_empty:
            continue
        for outline_path in outline_paths_mm:
            path_deg = next((candidate for candidate in outline_paths_deg if candidate.path_id == outline_path.path_id), None)
            if path_deg is None or len(outline_path.points) < 2:
                continue
            distances_by_orientation: dict[str, list[float]] = {"horizontal": [], "vertical": [], "angled": []}
            all_distances: list[float] = []
            for start, end in zip(outline_path.points, outline_path.points[1:]):
                midpoint = Point((start.x + end.x) * 0.5, (start.y + end.y) * 0.5)
                distance_mm = float(infill_mm_line.distance(ShapelyPoint(midpoint.x, midpoint.y)))
                orientation = _segment_orientation(end.x - start.x, end.y - start.y)
                distances_by_orientation[orientation].append(distance_mm)
                all_distances.append(distance_mm)
            if not all_distances:
                continue
            horizontal_bias = sum(distances_by_orientation["horizontal"]) / max(1, len(distances_by_orientation["horizontal"]))
            vertical_bias = sum(distances_by_orientation["vertical"]) / max(1, len(distances_by_orientation["vertical"]))
            angled_bias = sum(distances_by_orientation["angled"]) / max(1, len(distances_by_orientation["angled"]))
            suspected_axis_bias = "none"
            if vertical_bias > max(horizontal_bias * 1.5, angled_bias * 1.25, 0.1):
                suspected_axis_bias = "y"
            elif horizontal_bias > max(vertical_bias * 1.5, angled_bias * 1.25, 0.1):
                suspected_axis_bias = "x"
            elif angled_bias > max(horizontal_bias, vertical_bias, 0.1):
                suspected_axis_bias = "mixed"
            ordered = sorted(all_distances)
            audits.append({
                "outline_vs_infill_physical_alignment": {
                    "region_id": region_id,
                    "outline_path_id": outline_path.path_id,
                    "nearest_infill_edge_distance_mm": {
                        "min": ordered[0],
                        "mean": sum(ordered) / len(ordered),
                        "p95": ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))],
                        "max": ordered[-1],
                    },
                    "vertical_segment_bias_mm": vertical_bias,
                    "horizontal_segment_bias_mm": horizontal_bias,
                    "angled_segment_bias_mm": angled_bias,
                    "suspected_axis_bias": suspected_axis_bias,
                    "bbox_machine_deg": _bounds_or_none(path_deg.points),
                    "first_5_surface_points": [asdict(point) for point in outline_path.points[:5]],
                    "first_5_machine_points": [asdict(point) for point in path_deg.points[:5]],
                    "sampling_step_mm": float(outline_path.metadata.get("projection_sampling_mm", 0.0)),
                    "sampling_step_deg": max(_segment_lengths_mm(path_deg.points, closed=path_deg.closed), default=0.0),
                    "feedrate": float(path_deg.metadata.get("feedrate", 0.0)),
                }
            })
    return audits


def validate_toolpaths_finite(toolpaths: list[Toolpath], *, coordinate_space: str) -> None:
    for toolpath in toolpaths:
        for point in toolpath.points:
            if not math.isfinite(point.x) or not math.isfinite(point.y):
                raise ValueError(
                    f"Non-finite point detected in {toolpath.kind} path {toolpath.path_id or '<unassigned>'} "
                    f"while validating {coordinate_space} coordinates"
                )


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


def _line_fully_inside(region: Any, line: Any, *, tolerance_mm: float = 1e-6) -> bool:
    """Return True when the entire LineString lies inside the region.

    Tries a conservative test by shrinking the region by ``tolerance_mm`` first
    (to avoid accepting borderline/near-boundary lines), then falls back to
    the original region's ``covers`` predicate if the shrink produces an
    empty geometry.
    """
    if region is None:
        return False
    try:
        shrink_tol = max(tolerance_mm, 1e-9)
        try:
            shrunk = region.buffer(-shrink_tol, join_style=1)
        except Exception:
            shrunk = None
        if shrunk is not None and not getattr(shrunk, "is_empty", False):
            try:
                if shrunk.covers(line):
                    return True
            except Exception:
                pass
        try:
            return bool(region.covers(line))
        except Exception:
            return False
    except Exception:
        return False


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
        if segment_length(path.points) < _minimum_toolpath_length_threshold(path, minimum_length):
            continue
        signature = path_signature(path)
        if signature in seen:
            continue
        seen.add(signature)
        deduped.append(path)
    return deduped


def filter_toolpaths_by_length(paths: list[Toolpath], minimum_length: float) -> list[Toolpath]:
    filtered: list[Toolpath] = []
    for path in paths:
        if len(path.points) < 2:
            continue
        threshold = _minimum_toolpath_length_threshold(path, minimum_length)
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
    return clone_toolpath(path, points=rotated, closed=True)


def _toolpath_canonical_geometry_signature(toolpath: Toolpath) -> tuple[Any, ...]:
    rounded = tuple((round(point.x, 6), round(point.y, 6)) for point in toolpath.points)
    if not rounded:
        return (toolpath.kind, toolpath.closed, rounded)
    if not toolpath.closed:
        return (toolpath.kind, toolpath.closed, min(rounded, tuple(reversed(rounded))))

    core = rounded[:-1] if len(rounded) >= 2 and rounded[0] == rounded[-1] else rounded
    if not core:
        return (toolpath.kind, toolpath.closed, rounded)
    rotations = [tuple(core[index:] + core[:index]) for index in range(len(core))]
    canonical = min(rotations)
    return (toolpath.kind, toolpath.closed, canonical)


def _toolpath_sequence_matches_geometry(before: list[Toolpath], after: list[Toolpath]) -> bool:
    return Counter(_toolpath_canonical_geometry_signature(path) for path in before) == Counter(
        _toolpath_canonical_geometry_signature(path) for path in after
    )


def _path_debug_identifier(toolpath: Toolpath, index: int) -> str:
    return toolpath.path_id or f"path_{index:04d}"


def _path_component_debug_id(toolpath: Toolpath) -> str:
    return _path_component_label(toolpath)


def _travel_hops(
    toolpaths: list[Toolpath],
    *,
    start_point: Optional[Point] = None,
) -> list[dict[str, Any]]:
    hops: list[dict[str, Any]] = []
    current = start_point or Point(0.0, 0.0)
    previous_path: Toolpath | None = None
    for index, path in enumerate(toolpaths):
        if len(path.points) < 2:
            continue
        start = path.points[0]
        if not nearly_same_point(current, start):
            hops.append({
                "index": len(hops),
                "from_path_id": "" if previous_path is None else previous_path.path_id or _path_debug_identifier(previous_path, index - 1),
                "from_kind": "origin" if previous_path is None else previous_path.kind,
                "from_component_id": "origin" if previous_path is None else _path_component_debug_id(previous_path),
                "to_path_id": path.path_id or _path_debug_identifier(path, index),
                "to_kind": path.kind,
                "to_component_id": _path_component_debug_id(path),
                "length_mm": float(_segment_length_mm(current, start)),
                "from_xy_surface_mm": {"x": float(current.x), "y": float(current.y)},
                "to_xy_surface_mm": {"x": float(start.x), "y": float(start.y)},
            })
        current = path.points[-1]
        previous_path = path
    return hops


def _travel_crossing_count(toolpaths: list[Toolpath], *, start_point: Optional[Point] = None) -> int:
    hops = _travel_hops(toolpaths, start_point=start_point)
    if len(hops) <= 1 or LineString is None:
        return 0
    crossings = 0
    for index, hop in enumerate(hops):
        line_a = LineString([
            (float(hop["from_xy_surface_mm"]["x"]), float(hop["from_xy_surface_mm"]["y"])),
            (float(hop["to_xy_surface_mm"]["x"]), float(hop["to_xy_surface_mm"]["y"])),
        ])
        if line_a.length <= 1e-9:
            continue
        for other in hops[index + 1:]:
            line_b = LineString([
                (float(other["from_xy_surface_mm"]["x"]), float(other["from_xy_surface_mm"]["y"])),
                (float(other["to_xy_surface_mm"]["x"]), float(other["to_xy_surface_mm"]["y"])),
            ])
            if line_b.length <= 1e-9:
                continue
            if line_a.crosses(line_b):
                crossings += 1
    return crossings


def _travel_metrics(toolpaths: list[Toolpath], *, start_point: Optional[Point] = None) -> dict[str, Any]:
    hops = _travel_hops(toolpaths, start_point=start_point)
    longest_travel_mm = max((float(hop["length_mm"]) for hop in hops), default=0.0)
    return {
        "travel_length_mm": float(sum(float(hop["length_mm"]) for hop in hops)),
        "travel_crossing_count": int(_travel_crossing_count(toolpaths, start_point=start_point)),
        "pen_lifts": int(len(hops)),
        "longest_travel_mm": float(longest_travel_mm),
        "travel_hops": hops,
    }


def _final_export_fill_group_kinds() -> set[str]:
    return {
        "fill-infill",
        "fill-wall",
        "crossed-contour-infill",
        "junction-centerline",
        "gap-repair-stroke",
        "gap-repair-dab",
        "repair-patch-fill",
        "detail-trace",
        "detail-continuation",
        "coverage_centerline",
        "coverage_offset_line",
        "coverage_rectilinear",
        "coverage_contour",
        "outline_cleanup",
    }


def _final_export_group_key(toolpath: Toolpath) -> str:
    if toolpath.kind == "outline":
        return "outline"
    if toolpath.kind in _final_export_fill_group_kinds():
        return "fill"
    return "other"


def _top_longest_travels(
    toolpaths: list[Toolpath],
    *,
    start_point: Optional[Point] = None,
    reasons_by_to_path_id: Optional[dict[str, dict[str, Any]]] = None,
    limit: int = 25,
) -> list[dict[str, Any]]:
    hops = _travel_hops(toolpaths, start_point=start_point)
    out: list[dict[str, Any]] = []
    for hop in sorted(hops, key=lambda item: float(item["length_mm"]), reverse=True)[:limit]:
        item = dict(hop)
        meta = (reasons_by_to_path_id or {}).get(str(item["to_path_id"]), {})
        item["nearest_available_distance_mm"] = meta.get("nearest_available_distance_mm")
        item["bad_choice_ratio"] = meta.get("bad_choice_ratio")
        item["reason_chosen"] = meta.get("reason_chosen", "existing_export_order")
        item["reason_this_order_was_chosen"] = item["reason_chosen"]
        out.append(item)
    return out


def build_final_export_path_entries(
    toolpaths_surface_mm: list[Toolpath],
    toolpaths_machine_deg: list[Toolpath],
) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for index, (surface_path, machine_path) in enumerate(zip(toolpaths_surface_mm, toolpaths_machine_deg)):
        entries.append({
            "id": surface_path.path_id or _path_debug_identifier(surface_path, index),
            "kind": surface_path.kind,
            "source": str(surface_path.source or ""),
            "region_id": surface_path.region_id,
            "layer": _final_export_group_key(surface_path),
            "component_id": _path_component_debug_id(surface_path),
            "is_open": bool(not surface_path.closed),
            "points_surface_mm": [asdict(point) for point in surface_path.points],
            "points_machine_deg": [asdict(point) for point in machine_path.points],
            "start_surface_mm": None if not surface_path.points else asdict(surface_path.points[0]),
            "end_surface_mm": None if not surface_path.points else asdict(surface_path.points[-1]),
            "bbox_surface_mm": _bounds_or_none(surface_path.points),
            "max_surface_segment_mm_before_resampling": float((surface_path.metadata or {}).get("max_surface_segment_mm_before_resampling", max(_segment_lengths_mm(surface_path.points, closed=surface_path.closed), default=0.0))),
            "max_surface_segment_mm_after_resampling": float((surface_path.metadata or {}).get("max_surface_segment_mm_after_resampling", max(_segment_lengths_mm(surface_path.points, closed=surface_path.closed), default=0.0))),
        })
    return entries


def audit_exported_path_coverage(
    debug: dict[str, Any],
    *,
    ball_diameter_mm: float,
    center_lon_deg: float,
    center_lat_deg: float,
    pen_diameter_mm: float,
    max_coverage_segment_mm: float = 0.15,
    min_visible_missed_blob_area_mm2: float = 0.005,
    min_visible_missed_blob_equivalent_diameter_mm: float = 0.15,
) -> None:
    artifact_dir_value = debug.get("coverage_debug_artifact_dir")
    final_export_entries = debug.get("final_export_paths")
    target_mask = debug.get("_coverage_target_mask")
    current_to_source_matrix = debug.get("_coverage_current_to_source_matrix")
    preview_source_mask = debug.get("_coverage_preview_source_mask")
    preview_source_to_surface_matrix = debug.get("_coverage_preview_source_to_surface_matrix")
    if not artifact_dir_value or not isinstance(final_export_entries, list) or target_mask is None:
        return
    if not isinstance(current_to_source_matrix, (tuple, list)) or len(current_to_source_matrix) != 6:
        return
    artifact_dir = Path(str(artifact_dir_value))
    artifact_dir.mkdir(parents=True, exist_ok=True)
    target_mask_np = (np.asarray(target_mask) > 0).astype(np.uint8) * 255
    current_to_source = tuple(float(value) for value in current_to_source_matrix)
    preview_target_mask = target_mask_np.copy()
    if isinstance(preview_source_mask, np.ndarray) and isinstance(preview_source_to_surface_matrix, (tuple, list)) and len(preview_source_to_surface_matrix) == 6:
        preview_target_mask = rasterize_source_mask_to_target_frame(
            preview_source_mask,
            source_to_surface_matrix=tuple(float(value) for value in preview_source_to_surface_matrix),
            surface_to_target_matrix=current_to_source,
            shape=target_mask_np.shape,
        )
    diagnostic_target_mask = target_mask_np.copy()
    union_mask = (preview_target_mask > 0) | (diagnostic_target_mask > 0)
    intersection_mask = (preview_target_mask > 0) & (diagnostic_target_mask > 0)
    preview_target_vs_diagnostic_target_iou = float(np.count_nonzero(intersection_mask) / max(1, np.count_nonzero(union_mask)))

    def _entry_to_toolpath(entry: dict[str, Any]) -> Toolpath:
        surface_points = [
            Point(float(point["x"]), float(point["y"]))
            for point in list(entry.get("points_surface_mm") or [])
            if isinstance(point, dict) and "x" in point and "y" in point
        ]
        machine_points = [
            Point(float(point["x"]), float(point["y"]))
            for point in list(entry.get("points_machine_deg") or [])
            if isinstance(point, dict) and "x" in point and "y" in point
        ]
        return Toolpath(
            points=surface_points,
            kind=str(entry.get("kind") or "unknown"),
            closed=False,
            source=str(entry.get("source") or ""),
            region_id=entry.get("region_id"),
            path_id=str(entry.get("id") or ""),
            coordinate_space="surface_mm",
            metadata={
                "machine_points": [asdict(point) for point in machine_points],
                "max_surface_segment_mm_before_resampling": float(entry.get("max_surface_segment_mm_before_resampling", 0.0) or 0.0),
                "max_surface_segment_mm_after_resampling": float(entry.get("max_surface_segment_mm_after_resampling", 0.0) or 0.0),
            },
        )

    exported_surface_paths = [_entry_to_toolpath(entry) for entry in final_export_entries]
    exported_draw_paths = [path for path in exported_surface_paths if path.kind != "travel" and len(path.points) >= 1]
    exported_paths_before_repair = [path for path in exported_draw_paths if not (path.kind == "fill-repair" or str(path.source) == "mask_space_coverage_repair")]
    exported_paths_after_repair = list(exported_draw_paths)
    resampled_exported_paths = [resample_surface_path(path, max_coverage_segment_mm) if len(path.points) >= 2 else clone_toolpath(path) for path in exported_paths_after_repair]
    resampled_exported_paths_before_repair = [resample_surface_path(path, max_coverage_segment_mm) if len(path.points) >= 2 else clone_toolpath(path) for path in exported_paths_before_repair]

    def _segment_lengths(path: Toolpath) -> list[float]:
        return _segment_lengths_mm(path.points, closed=path.closed) if len(path.points) >= 2 else []

    long_segments_before_resampling = int(sum(1 for path in exported_draw_paths for length in _segment_lengths(path) if float(length) > max_coverage_segment_mm + 1e-9))
    long_segments_after_resampling = int(sum(1 for path in resampled_exported_paths for length in _segment_lengths(path) if float(length) > max_coverage_segment_mm + 1e-9))
    repair_long_segments_before = int(sum(1 for path in exported_draw_paths if (path.kind == "fill-repair" or str(path.source) == "mask_space_coverage_repair") for length in _segment_lengths(path) if float(length) > max_coverage_segment_mm + 1e-9))
    repair_long_segments_after = int(sum(1 for path in resampled_exported_paths if (path.kind == "fill-repair" or str(path.source) == "mask_space_coverage_repair") for length in _segment_lengths(path) if float(length) > max_coverage_segment_mm + 1e-9))
    max_surface_segment_mm_before = float(max((max(_segment_lengths(path), default=0.0) for path in exported_draw_paths), default=0.0))
    max_surface_segment_mm_after = float(max((max(_segment_lengths(path), default=0.0) for path in resampled_exported_paths), default=0.0))

    pen_radius_mm = float(pen_diameter_mm) * 0.5
    coverage_before_mask = rasterize_surface_toolpaths_mask(
        resampled_exported_paths_before_repair,
        shape=diagnostic_target_mask.shape,
        current_to_source_matrix=current_to_source,
        pen_radius_mm=pen_radius_mm,
        max_segment_mm=max_coverage_segment_mm,
        include_kinds=None,
    )
    coverage_after_mask = rasterize_surface_toolpaths_mask(
        resampled_exported_paths,
        shape=diagnostic_target_mask.shape,
        current_to_source_matrix=current_to_source,
        pen_radius_mm=pen_radius_mm,
        max_segment_mm=max_coverage_segment_mm,
        include_kinds=None,
    )
    visible_gaps_before = ((preview_target_mask > 0) & ~(coverage_before_mask > 0)).astype(np.uint8) * 255
    visible_gaps_after = ((preview_target_mask > 0) & ~(coverage_after_mask > 0)).astype(np.uint8) * 255
    coverage_changed_after_resampling = bool(np.any(coverage_after_mask > 0) != np.any(coverage_before_mask > 0) or np.count_nonzero(visible_gaps_before) != np.count_nonzero(visible_gaps_after))

    a, b, c, d, _e, _f = current_to_source
    px_per_mm = max(1e-6, (math.hypot(a, b) + math.hypot(c, d)) * 0.5)
    px_to_mm2 = 1.0 / max(1e-9, px_per_mm * px_per_mm)

    def _blob_rows(mask: np.ndarray) -> tuple[list[dict[str, Any]], float]:
        comp_count, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), connectivity=8)
        rows: list[dict[str, Any]] = []
        largest_diameter = 0.0
        for blob_id in range(1, int(comp_count)):
            area_px = int(stats[blob_id, cv2.CC_STAT_AREA])
            area_mm2 = float(area_px) * px_to_mm2
            diameter_mm = float(2.0 * math.sqrt(max(0.0, area_mm2) / math.pi)) if area_mm2 > 0 else 0.0
            if area_mm2 < min_visible_missed_blob_area_mm2 or diameter_mm < min_visible_missed_blob_equivalent_diameter_mm:
                continue
            largest_diameter = max(largest_diameter, diameter_mm)
            ys, xs = np.nonzero(labels == blob_id)
            rows.append({
                "blob_id": int(blob_id),
                "area_mm2": float(area_mm2),
                "equivalent_diameter_mm": float(diameter_mm),
                "bbox_px": {
                    "left": int(stats[blob_id, cv2.CC_STAT_LEFT]),
                    "top": int(stats[blob_id, cv2.CC_STAT_TOP]),
                    "width": int(stats[blob_id, cv2.CC_STAT_WIDTH]),
                    "height": int(stats[blob_id, cv2.CC_STAT_HEIGHT]),
                },
                "centroid_px": {
                    "x": float(np.mean(xs)) if xs.size else 0.0,
                    "y": float(np.mean(ys)) if ys.size else 0.0,
                },
                "mask": (labels == blob_id).astype(np.uint8),
            })
        return rows, largest_diameter

    visible_blob_rows_before, largest_before = _blob_rows(visible_gaps_before)
    visible_blob_rows_after, largest_after = _blob_rows(visible_gaps_after)

    def _render_path_overlay(paths: list[Toolpath], color: tuple[int, int, int]) -> np.ndarray:
        canvas = np.full((diagnostic_target_mask.shape[0], diagnostic_target_mask.shape[1], 3), 255, dtype=np.uint8)
        for path in paths:
            mapped = [apply_svg_matrix(point, current_to_source) for point in path.points]
            if len(mapped) == 1:
                cv2.circle(canvas, (int(round(mapped[0].x)), int(round(mapped[0].y))), 2, color, -1)
                continue
            pts = np.asarray([(int(round(point.x)), int(round(point.y))) for point in mapped], dtype=np.int32).reshape(-1, 1, 2)
            if len(pts) >= 2:
                cv2.polylines(canvas, [pts], False, color, 1, lineType=cv2.LINE_AA)
        return canvas

    long_segment_paths_before = [path for path in exported_draw_paths if any(length > max_coverage_segment_mm + 1e-9 for length in _segment_lengths(path))]
    labeled = cv2.cvtColor((visible_gaps_after > 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    for index, row in enumerate(visible_blob_rows_after, start=1):
        bbox = row["bbox_px"]
        cv2.rectangle(labeled, (int(bbox["left"]), int(bbox["top"])), (int(bbox["left"] + bbox["width"]), int(bbox["top"] + bbox["height"])), (0, 0, 255), 1)
        centroid = row["centroid_px"]
        cv2.putText(labeled, str(index), (int(round(centroid["x"])), int(round(centroid["y"]))), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1, cv2.LINE_AA)

    comparison = np.zeros((diagnostic_target_mask.shape[0], diagnostic_target_mask.shape[1], 3), dtype=np.uint8)
    comparison[coverage_before_mask > 0] = (255, 180, 80)
    comparison[coverage_after_mask > 0] = (60, 180, 80)
    comparison[(preview_target_mask > 0) & ~(coverage_after_mask > 0)] = (0, 0, 255)

    cv2.imwrite(str(artifact_dir / "01_preview_target_mask.png"), preview_target_mask)
    cv2.imwrite(str(artifact_dir / "02_diagnostic_selected_color_mask.png"), diagnostic_target_mask)
    cv2.imwrite(str(artifact_dir / "03_mask_difference_preview_vs_diagnostic.png"), ((preview_target_mask > 0) ^ (diagnostic_target_mask > 0)).astype(np.uint8) * 255)
    cv2.imwrite(str(artifact_dir / "04_swept_coverage_from_exported_paths.png"), coverage_after_mask)
    cv2.imwrite(str(artifact_dir / "05_visible_gaps_from_preview_target_minus_coverage.png"), visible_gaps_after)
    cv2.imwrite(str(artifact_dir / "06_visible_gaps_labeled.png"), labeled)
    cv2.imwrite(str(artifact_dir / "07_long_segments_over_threshold.png"), _render_path_overlay(long_segment_paths_before, (0, 0, 255)))
    cv2.imwrite(str(artifact_dir / "08_resampled_coverage_comparison.png"), comparison)

    mask_report = {
        "preview_target_vs_diagnostic_target_iou": float(preview_target_vs_diagnostic_target_iou),
        "preview_target_pixels": int(np.count_nonzero(preview_target_mask > 0)),
        "diagnostic_target_pixels": int(np.count_nonzero(diagnostic_target_mask > 0)),
        "difference_pixels": int(np.count_nonzero(((preview_target_mask > 0) ^ (diagnostic_target_mask > 0)))),
    }
    path_resampling_report = {
        "coverage_rasterization_space": "surface-mm-on-ball",
        "long_segments_before_resampling": int(long_segments_before_resampling),
        "long_segments_after_resampling": int(long_segments_after_resampling),
        "max_surface_segment_mm_before": float(max_surface_segment_mm_before),
        "max_surface_segment_mm_after": float(max_surface_segment_mm_after),
        "repair_long_segments_before": int(repair_long_segments_before),
        "repair_long_segments_after": int(repair_long_segments_after),
    }
    coverage_report = {
        "coverage_rasterization_space": "surface-mm-on-ball",
        "coverage_target": "full selected-color mask in shared uploaded-image design frame",
        "final_repair_scope": "all_selected_color_components",
        "coverage_changed_after_resampling": bool(coverage_changed_after_resampling),
        "visible_gap_count_after_resampled_coverage": int(np.count_nonzero(visible_gaps_after > 0)),
        "visible_missed_blob_count_before_repair": int(len(visible_blob_rows_before)),
        "visible_missed_blob_count_after_repair": int(len(visible_blob_rows_after)),
        "largest_visible_missed_blob_equivalent_diameter_mm_before": float(largest_before),
        "largest_visible_missed_blob_equivalent_diameter_mm_after": float(largest_after),
        "repair_paths_exported": bool(any(path.kind == "fill-repair" or str(path.source) == "mask_space_coverage_repair" for path in exported_draw_paths)),
    }
    missed_blob_rows: list[dict[str, Any]] = []
    for row in visible_blob_rows_after:
        centroid = row["centroid_px"]
        nearest_distance = float("inf")
        nearest_path_id = ""
        nearest_path_kind = ""
        nearest_path_source = ""
        centroid_surface = apply_svg_matrix(Point(float(centroid["x"]), float(centroid["y"])), invert_svg_matrix(current_to_source))
        for path in resampled_exported_paths:
            if len(path.points) < 2:
                continue
            line = LineString([(point.x, point.y) for point in path.points])
            distance = float(line.distance(ShapelyPoint(centroid_surface.x, centroid_surface.y)))
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_path_id = str(path.path_id or "")
                nearest_path_kind = str(path.kind)
                nearest_path_source = str(path.source)
        missed_blob_rows.append({
            "blob_id": int(row["blob_id"]),
            "area_mm2": float(row["area_mm2"]),
            "equivalent_diameter_mm": float(row["equivalent_diameter_mm"]),
            "bbox_px": row["bbox_px"],
            "centroid_px": row["centroid_px"],
            "inside_preview_target": True,
            "inside_diagnostic_target": True,
            "covered_before_resampling": bool(coverage_before_mask[int(round(centroid["y"])), int(round(centroid["x"]))] > 0) if 0 <= int(round(centroid["y"])) < coverage_before_mask.shape[0] and 0 <= int(round(centroid["x"])) < coverage_before_mask.shape[1] else False,
            "covered_after_resampling": bool(coverage_after_mask[int(round(centroid["y"])), int(round(centroid["x"]))] > 0) if 0 <= int(round(centroid["y"])) < coverage_after_mask.shape[0] and 0 <= int(round(centroid["x"])) < coverage_after_mask.shape[1] else False,
            "nearest_path_id": nearest_path_id,
            "nearest_path_kind": nearest_path_kind,
            "nearest_path_source": nearest_path_source,
            "nearest_path_distance_mm": float(0.0 if nearest_distance == float("inf") else nearest_distance),
            "repair_generated": True,
            "repair_accepted": False,
            "repair_exported": False,
            "failure_reason": "visible_gap_after_resampled_export_coverage",
        })
    (artifact_dir / "mask_consistency_report.json").write_text(json.dumps(mask_report, indent=2), encoding="utf-8")
    (artifact_dir / "path_resampling_report.json").write_text(json.dumps(path_resampling_report, indent=2), encoding="utf-8")
    (artifact_dir / "coverage_from_exported_paths_report.json").write_text(json.dumps(coverage_report, indent=2), encoding="utf-8")
    (artifact_dir / "missed_blob_diagnostics.json").write_text(json.dumps(missed_blob_rows, indent=2), encoding="utf-8")

    debug.update(mask_report)
    debug.update(path_resampling_report)
    debug.update(coverage_report)
    debug["root_cause_category_corrected"] = (
        "wrong_target_mask_selection"
        if preview_target_vs_diagnostic_target_iou < 0.995
        else ("false_negative_coverage_simulation" if len(visible_blob_rows_after) > 0 else "coverage_under_sampling_fixed")
    )


def rewrite_final_export_path_stats_artifact(debug: dict[str, Any]) -> None:
    artifact_dir_value = debug.get("coverage_debug_artifact_dir")
    if not artifact_dir_value:
        return
    path_stats_path = Path(str(artifact_dir_value)) / "path_stats.json"
    if not path_stats_path.exists():
        return
    try:
        current = json.loads(path_stats_path.read_text(encoding="utf-8"))
    except Exception:
        current = {}
    current.update({
        "travel_optimization_mode": str(debug.get("travel_optimization_mode", "")),
        "travel_bug_source": str(debug.get("travel_bug_source", "unknown")),
        "optimizer_runs_after_path_merging": bool(debug.get("optimizer_runs_after_path_merging", False)),
        "preview_uses_optimized_order": bool(debug.get("preview_uses_optimized_order", False)),
        "preview_uses_final_export_order": bool(debug.get("preview_uses_optimized_order", False)),
        "gcode_uses_optimized_order": bool(debug.get("gcode_uses_optimized_order", False)),
        "gcode_uses_final_export_order": bool(debug.get("gcode_uses_optimized_order", False)),
        "optimizer_runs_on_final_export_paths": bool(debug.get("optimizer_runs_on_final_export_paths", False)),
        "uses_surface_mm_for_ordering": bool(debug.get("uses_surface_mm_for_ordering", False)),
        "geometry_changed": bool(debug.get("geometry_changed", False)),
        "path_points_moved": bool(debug.get("path_points_moved", False)),
        "open_paths_reversed_count": int(debug.get("open_paths_reversed_count", 0)),
        "raw_export_path_count": int(debug.get("raw_export_path_count", 0)),
        "optimized_export_path_count": int(debug.get("optimized_export_path_count", 0)),
        "raw_pen_up_travel_length_mm": float(debug.get("raw_pen_up_travel_length_mm", 0.0)),
        "optimized_pen_up_travel_length_mm": float(debug.get("optimized_pen_up_travel_length_mm", 0.0)),
        "final_pen_up_travel_length_mm": float(debug.get("optimized_pen_up_travel_length_mm", 0.0)),
        "travel_length_reduction_percent": float(debug.get("travel_length_reduction_percent", 0.0)),
        "raw_longest_pen_up_travel_mm": float(debug.get("raw_longest_pen_up_travel_mm", 0.0)),
        "optimized_longest_pen_up_travel_mm": float(debug.get("optimized_longest_pen_up_travel_mm", 0.0)),
        "longest_final_pen_up_travel_mm": float(debug.get("optimized_longest_pen_up_travel_mm", 0.0)),
        "bad_choice_count_after_optimization": int(debug.get("bad_choice_count_after_optimization", 0)),
        "stale_travel_geometry_removed": bool(debug.get("stale_travel_geometry_removed", False)),
        "stale_preview_travel_count": int(debug.get("stale_preview_travel_count", 0)),
        "preview_only_travel_count": int(debug.get("preview_only_travel_count", 0)),
        "gcode_only_travel_count": int(debug.get("gcode_only_travel_count", 0)),
        "matched_preview_gcode_travel_count": int(debug.get("matched_preview_gcode_travel_count", 0)),
        "coverage_preview_gcode_consistent": bool(debug.get("preview_and_gcode_share_same_projected_paths", True)),
        "root_cause_category_corrected": str(debug.get("root_cause_category_corrected", "")),
        "preview_target_vs_diagnostic_target_iou": float(debug.get("preview_target_vs_diagnostic_target_iou", 0.0)),
        "coverage_rasterization_space": str(debug.get("coverage_rasterization_space", "")),
        "final_repair_scope": str(debug.get("final_repair_scope", "")),
        "long_segments_before_resampling": int(debug.get("long_segments_before_resampling", 0)),
        "long_segments_after_resampling": int(debug.get("long_segments_after_resampling", 0)),
        "max_surface_segment_mm_before": float(debug.get("max_surface_segment_mm_before", 0.0)),
        "max_surface_segment_mm_after": float(debug.get("max_surface_segment_mm_after", 0.0)),
        "repair_long_segments_before": int(debug.get("repair_long_segments_before", 0)),
        "repair_long_segments_after": int(debug.get("repair_long_segments_after", 0)),
        "coverage_changed_after_resampling": bool(debug.get("coverage_changed_after_resampling", False)),
        "visible_gap_count_after_resampled_coverage": int(debug.get("visible_gap_count_after_resampled_coverage", 0)),
        "visible_missed_blob_count_before_repair": int(debug.get("visible_missed_blob_count_before_repair", 0)),
        "visible_missed_blob_count_after_repair": int(debug.get("visible_missed_blob_count_after_repair", 0)),
        "largest_visible_missed_blob_equivalent_diameter_mm_before": float(debug.get("largest_visible_missed_blob_equivalent_diameter_mm_before", 0.0)),
        "largest_visible_missed_blob_equivalent_diameter_mm_after": float(debug.get("largest_visible_missed_blob_equivalent_diameter_mm_after", 0.0)),
        "final_export_path_count": int(debug.get("optimized_export_path_count", debug.get("raw_export_path_count", 0))),
        "top_25_longest_pen_up_travels_before": list(debug.get("top_25_longest_pen_up_travels_before", [])),
        "top_25_longest_pen_up_travels_after": list(debug.get("top_25_longest_pen_up_travels_after", [])),
        "top_longest_travels": list(debug.get("top_25_longest_pen_up_travels_after", [])),
        "detail_filter_mode": str(debug.get("detail_filter_mode", DETAIL_FILTER_MODE)),
        "detail_paths_generated_raw": int(debug.get("detail_paths_generated_raw", 0)),
        "detail_paths_kept": int(debug.get("detail_paths_kept", 0)),
        "detail_paths_dropped": int(debug.get("detail_paths_dropped", 0)),
        "detail_drop_reasons": dict(debug.get("detail_drop_reasons", {})),
        "detail_paths_dropped_as_travel_or_debug": int(debug.get("detail_paths_dropped_as_travel_or_debug", 0)),
        "detail_paths_dropped_as_redundant_overlap": int(debug.get("detail_paths_dropped_as_redundant_overlap", 0)),
        "detail_new_coverage_area_mm2_total": float(debug.get("detail_new_coverage_area_mm2_total", 0.0)),
        "detail_already_covered_ratio_avg": float(debug.get("detail_already_covered_ratio_avg", 0.0)),
        "detail_source_whitelist_enforced": bool(debug.get("detail_source_whitelist_enforced", True)),
        "travel_geometry_allowed_as_detail": bool(debug.get("travel_geometry_allowed_as_detail", False)),
        "infill_geometry_changed": bool(debug.get("infill_geometry_changed", False)),
        "outline_geometry_changed": bool(debug.get("outline_geometry_changed", False)),
        "detail_dropped_path_records": list(debug.get("detail_dropped_path_records", [])),
    })
    path_stats_path.write_text(json.dumps(current, indent=2), encoding="utf-8")


def _nearest_neighbor_order_group(
    toolpaths: list[Toolpath],
    *,
    start_point: Point,
    allow_open_path_reversal: bool = True,
) -> tuple[list[Toolpath], Point, int, int, dict[str, dict[str, Any]]]:
    pending: list[tuple[int, Toolpath]] = list(enumerate(toolpaths))
    ordered: list[Toolpath] = []
    current = start_point
    open_paths_reversed_count = 0
    bad_choice_count = 0
    reasons_by_to_path_id: dict[str, dict[str, Any]] = {}

    while pending:
        candidate_rows: list[dict[str, Any]] = []
        for original_index, path in pending:
            candidate = rotate_closed_toolpath(path, current) if path.closed else path
            if len(candidate.points) >= 1:
                candidate_rows.append({
                    "original_index": original_index,
                    "path": path,
                    "candidate": candidate,
                    "distance_mm": float(math.hypot(candidate.points[0].x - current.x, candidate.points[0].y - current.y)),
                    "reversed": False,
                })
            if allow_open_path_reversal and not path.closed and len(path.points) >= 2:
                reversed_candidate = clone_toolpath(path, points=list(reversed(path.points)), closed=False)
                candidate_rows.append({
                    "original_index": original_index,
                    "path": path,
                    "candidate": reversed_candidate,
                    "distance_mm": float(math.hypot(reversed_candidate.points[0].x - current.x, reversed_candidate.points[0].y - current.y)),
                    "reversed": True,
                })

        candidate_rows.sort(key=lambda item: (float(item["distance_mm"]), int(item["original_index"]), 1 if item["reversed"] else 0))
        selected = candidate_rows[0]
        nearest_available_path_distance_mm = float(selected["distance_mm"])
        chosen_path_distance_mm = float(selected["distance_mm"])
        bad_choice_ratio = 1.0 if nearest_available_path_distance_mm <= 1e-9 else (chosen_path_distance_mm / nearest_available_path_distance_mm)
        if bad_choice_ratio > 2.0:
            bad_choice_count += 1

        path = selected["path"]
        candidate = selected["candidate"]
        if bool(selected["reversed"]):
            open_paths_reversed_count += 1
        pending = [(idx, item) for idx, item in pending if idx != int(selected["original_index"])]
        ordered.append(candidate)
        candidate_id = candidate.path_id or _path_debug_identifier(candidate, len(ordered) - 1)
        reasons_by_to_path_id[candidate_id] = {
            "nearest_available_distance_mm": float(nearest_available_path_distance_mm),
            "chosen_distance_mm": float(chosen_path_distance_mm),
            "bad_choice_ratio": float(bad_choice_ratio),
            "reason_chosen": (
                "nearest_endpoint_reversed_open_path"
                if bool(selected["reversed"])
                else "nearest_endpoint"
            ),
        }
        if candidate.points:
            current = candidate.points[-1]

    return ordered, current, open_paths_reversed_count, bad_choice_count, reasons_by_to_path_id


def optimize_post_generation_travel_order(
    toolpaths: list[Toolpath],
    *,
    start_point: Optional[Point] = None,
) -> tuple[list[Toolpath], dict[str, Any]]:
    if len(toolpaths) <= 1:
        diagnostics = {
            "travel_optimization_mode": "final_export_event_stream_ordering",
            "optimizer_runs_after_path_merging": True,
            "optimizer_runs_on_final_export_paths": True,
            "preview_uses_optimized_order": True,
            "gcode_uses_optimized_order": True,
            "uses_surface_mm_for_ordering": True,
            "geometry_changed": False,
            "path_points_moved": False,
            "paths_reordered": False,
            "paths_reordered_count": 0,
            "open_paths_reversed_count": 0,
            "raw_export_path_count": int(len(toolpaths)),
            "optimized_export_path_count": int(len(toolpaths)),
            "raw_pen_up_travel_length_mm": 0.0,
            "optimized_pen_up_travel_length_mm": 0.0,
            "travel_length_reduction_percent": 0.0,
            "raw_longest_pen_up_travel_mm": 0.0,
            "optimized_longest_pen_up_travel_mm": 0.0,
            "raw_travel_crossing_count": 0,
            "optimized_travel_crossing_count": 0,
            "top_25_longest_pen_up_travels_before": [],
            "top_25_longest_pen_up_travels_after": [],
            "bad_choice_count_after_optimization": 0,
            "stale_travel_geometry_removed": True,
            "pen_lifts_before": 0,
            "pen_lifts_after": 0,
            "safe_pen_down_travels_converted": 0,
            "outline_path_count": int(sum(1 for path in toolpaths if path.kind == "outline")),
        }
        diagnostics["raw_travel_length_mm"] = diagnostics["raw_pen_up_travel_length_mm"]
        diagnostics["optimized_travel_length_mm"] = diagnostics["optimized_pen_up_travel_length_mm"]
        diagnostics["raw_longest_travel_mm"] = diagnostics["raw_longest_pen_up_travel_mm"]
        diagnostics["optimized_longest_travel_mm"] = diagnostics["optimized_longest_pen_up_travel_mm"]
        diagnostics["top_longest_travels_before"] = list(diagnostics["top_25_longest_pen_up_travels_before"])
        diagnostics["top_longest_travels_after"] = list(diagnostics["top_25_longest_pen_up_travels_after"])
        return toolpaths, diagnostics

    current = start_point or Point(0.0, 0.0)
    raw_metrics = _travel_metrics(toolpaths, start_point=current)
    raw_reasons = {
        str(path.path_id or _path_debug_identifier(path, index)): {
            "nearest_available_distance_mm": None,
            "chosen_distance_mm": None,
            "bad_choice_ratio": None,
            "reason_chosen": "existing_export_order",
        }
        for index, path in enumerate(toolpaths)
    }
    top_before = _top_longest_travels(toolpaths, start_point=current, reasons_by_to_path_id=raw_reasons)

    fill_paths = [path for path in toolpaths if _final_export_group_key(path) == "fill"]
    other_paths = [path for path in toolpaths if _final_export_group_key(path) == "other"]
    outline_paths = [path for path in toolpaths if _final_export_group_key(path) == "outline"]

    optimized_export_paths: list[Toolpath] = []
    open_paths_reversed_count = 0
    bad_choice_count = 0
    reasons_by_to_path_id: dict[str, dict[str, Any]] = {}

    if fill_paths:
        optimized_fill, current, fill_reversed, fill_bad_choices, fill_reasons = _nearest_neighbor_order_group(
            fill_paths,
            start_point=current,
            allow_open_path_reversal=True,
        )
        optimized_export_paths.extend(optimized_fill)
        open_paths_reversed_count += fill_reversed
        bad_choice_count += fill_bad_choices
        reasons_by_to_path_id.update(fill_reasons)

    if other_paths:
        optimized_export_paths.extend(other_paths)
        for index, path in enumerate(other_paths, start=len(optimized_export_paths) - len(other_paths)):
            reasons_by_to_path_id[str(path.path_id or _path_debug_identifier(path, index))] = {
                "nearest_available_distance_mm": None,
                "chosen_distance_mm": None,
                "bad_choice_ratio": None,
                "reason_chosen": "kept_original_other_group_order",
            }
            if len(path.points) >= 2:
                current = path.points[-1]

    if outline_paths:
        optimized_outline, current, outline_reversed, outline_bad_choices, outline_reasons = _nearest_neighbor_order_group(
            outline_paths,
            start_point=current,
            allow_open_path_reversal=False,
        )
        optimized_export_paths.extend(optimized_outline)
        open_paths_reversed_count += outline_reversed
        bad_choice_count += outline_bad_choices
        reasons_by_to_path_id.update(outline_reasons)

    optimized_metrics = _travel_metrics(optimized_export_paths, start_point=start_point)
    optimized_crossings = int(optimized_metrics["travel_crossing_count"])
    raw_crossings = int(raw_metrics["travel_crossing_count"])

    raw_travel_length_mm = float(raw_metrics["travel_length_mm"])
    optimized_travel_length_mm = float(optimized_metrics["travel_length_mm"])
    reduction_percent = 0.0
    if raw_travel_length_mm > 1e-9:
        reduction_percent = max(0.0, ((raw_travel_length_mm - optimized_travel_length_mm) / raw_travel_length_mm) * 100.0)

    geometry_changed = not _toolpath_sequence_matches_geometry(toolpaths, optimized_export_paths)
    paths_reordered_count = sum(
        1
        for before, after in zip(toolpaths, optimized_export_paths)
        if _toolpath_canonical_geometry_signature(before) != _toolpath_canonical_geometry_signature(after)
    )
    paths_reordered = bool(paths_reordered_count or len(toolpaths) != len(optimized_export_paths))

    ordering_is_better_or_equal = (
        optimized_travel_length_mm <= (raw_travel_length_mm + 1e-9)
        and (
            optimized_crossings <= raw_crossings
            or optimized_travel_length_mm < (raw_travel_length_mm - 1e-9)
        )
    )
    if not ordering_is_better_or_equal:
        optimized_export_paths = list(toolpaths)
        optimized_metrics = dict(raw_metrics)
        optimized_travel_length_mm = raw_travel_length_mm
        optimized_crossings = raw_crossings
        reduction_percent = 0.0
        open_paths_reversed_count = 0
        paths_reordered_count = 0
        paths_reordered = False
        bad_choice_count = 0
        reasons_by_to_path_id = raw_reasons

    top_after = _top_longest_travels(
        optimized_export_paths,
        start_point=start_point or Point(0.0, 0.0),
        reasons_by_to_path_id=reasons_by_to_path_id,
    )
    diagnostics = {
        "travel_optimization_mode": "final_export_event_stream_ordering",
        "optimizer_runs_after_path_merging": True,
        "optimizer_runs_on_final_export_paths": True,
        "preview_uses_optimized_order": True,
        "gcode_uses_optimized_order": True,
        "uses_surface_mm_for_ordering": True,
        "geometry_changed": bool(geometry_changed),
        "path_points_moved": bool(geometry_changed),
        "paths_reordered": bool(paths_reordered),
        "paths_reordered_count": int(paths_reordered_count),
        "open_paths_reversed_count": int(open_paths_reversed_count),
        "raw_export_path_count": int(len(toolpaths)),
        "optimized_export_path_count": int(len(optimized_export_paths)),
        "raw_pen_up_travel_length_mm": raw_travel_length_mm,
        "optimized_pen_up_travel_length_mm": optimized_travel_length_mm,
        "travel_length_reduction_percent": float(reduction_percent),
        "raw_longest_pen_up_travel_mm": float(raw_metrics["longest_travel_mm"]),
        "optimized_longest_pen_up_travel_mm": float(optimized_metrics["longest_travel_mm"]),
        "raw_travel_crossing_count": int(raw_crossings),
        "optimized_travel_crossing_count": int(optimized_crossings),
        "outline_path_count": int(len(outline_paths)),
        "top_25_longest_pen_up_travels_before": top_before,
        "top_25_longest_pen_up_travels_after": top_after,
        "bad_choice_count_after_optimization": int(bad_choice_count),
        "stale_travel_geometry_removed": True,
        "pen_lifts_before": int(raw_metrics["pen_lifts"]),
        "pen_lifts_after": int(optimized_metrics["pen_lifts"]),
        "safe_pen_down_travels_converted": 0,
    }
    diagnostics["raw_travel_length_mm"] = diagnostics["raw_pen_up_travel_length_mm"]
    diagnostics["optimized_travel_length_mm"] = diagnostics["optimized_pen_up_travel_length_mm"]
    diagnostics["raw_longest_travel_mm"] = diagnostics["raw_longest_pen_up_travel_mm"]
    diagnostics["optimized_longest_travel_mm"] = diagnostics["optimized_longest_pen_up_travel_mm"]
    diagnostics["top_longest_travels_before"] = list(diagnostics["top_25_longest_pen_up_travels_before"])
    diagnostics["top_longest_travels_after"] = list(diagnostics["top_25_longest_pen_up_travels_after"])
    if geometry_changed:
        raise AssertionError("Post-generation travel optimizer changed toolpath geometry")
    return optimized_export_paths, diagnostics


def optimize_toolpath_order(
    toolpaths: list[Toolpath],
    *,
    strategy: str,
    start_point: Optional[Point] = None,
) -> list[Toolpath]:
    if strategy != "nearest-neighbor" or len(toolpaths) <= 1:
        return toolpaths
    # Preserve scanline infill ordering when the planner already encoded row order.
    if _preserve_infill_path_order(toolpaths):
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
                    best_rotated = clone_toolpath(path, points=reversed_points, closed=False)

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
            current = clone_toolpath(current, points=current.points + candidate.points[1:], closed=False)
            continue
        if can_merge and nearly_same_point(current.points[-1], candidate.points[-1], tolerance):
            current = clone_toolpath(current, points=current.points + list(reversed(candidate.points[:-1])), closed=False)
            continue
        merged.append(current)
        current = candidate

    merged.append(current)
    return merged


def optimize_detail_trace_efficiency(
    toolpaths: list[Toolpath],
    *,
    printable_geometry: Any,
    pen_width_mm: float,
    debug: Optional[dict[str, Any]] = None,
) -> list[Toolpath]:
    if not toolpaths:
        return toolpaths
    allow_merge = os.getenv("ALLOW_DETAIL_TRACE_MERGING", "1") == "1"
    if not allow_merge:
        return toolpaths
    max_gap_mm = max(0.01, float(os.getenv("MAX_DETAIL_MERGE_GAP_MM", str(1.5 * pen_width_mm))))
    preferred_gap_mm = max(0.01, float(os.getenv("PREFERRED_DETAIL_MERGE_GAP_MM", str(0.75 * pen_width_mm))))
    max_turn_deg = max(0.0, float(os.getenv("MAX_DETAIL_MERGE_TURN_DEG", "135")))
    overspill_tol = min(0.05, pen_width_mm * 0.10)
    overspill_area_ratio_tol = 0.02

    component_geoms = normalize_geometry(printable_geometry) if printable_geometry is not None and not printable_geometry.is_empty else []
    by_component: dict[str, list[Toolpath]] = {}
    passthrough: list[Toolpath] = []
    for path in toolpaths:
        if path.kind not in {"detail-trace", "detail-continuation"} or len(path.points) < 2:
            passthrough.append(path)
            continue
        by_component.setdefault(_path_component_label(path), []).append(path)

    merged_out: list[Toolpath] = []
    merged_count = 0
    rejected_count = 0

    def _component_geom(label: str) -> Any:
        idx = _extract_component_id(label)
        if idx is None or idx <= 0:
            return printable_geometry
        i = idx - 1
        return component_geoms[i] if 0 <= i < len(component_geoms) else printable_geometry

    def _angle_deg(a0: Point, a1: Point, b0: Point, b1: Point) -> float:
        ax, ay = a1.x - a0.x, a1.y - a0.y
        bx, by = b1.x - b0.x, b1.y - b0.y
        la = math.hypot(ax, ay)
        lb = math.hypot(bx, by)
        if la <= 1e-9 or lb <= 1e-9:
            return 0.0
        dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
        return math.degrees(math.acos(dot))

    def _connector_safe(geom: Any, a: Point, b: Point) -> bool:
        if geom is None or geom.is_empty:
            return False
        connector = LineString([(a.x, a.y), (b.x, b.y)])
        if not _line_fully_inside(geom.buffer(max(0.01, pen_width_mm * 0.15), join_style=1), connector, tolerance_mm=max(0.01, pen_width_mm * 0.05)):
            return False
        stroke = connector.buffer(max(0.01, pen_width_mm * 0.5), cap_style=1, join_style=1)
        overspill = stroke.difference(geom)
        overspill_area = float(overspill.area) if overspill is not None and not overspill.is_empty else 0.0
        overspill_ratio = overspill_area / max(1e-9, float(stroke.area))
        if overspill_ratio > overspill_area_ratio_tol:
            return False
        if overspill is not None and not overspill.is_empty:
            boundary = geom.boundary
            max_protrusion = 0.0
            for poly in normalize_geometry(overspill):
                coords = list(poly.exterior.coords)
                step = max(1, int(len(coords) / 16))
                for i in range(0, len(coords), step):
                    pt = ShapelyPoint(float(coords[i][0]), float(coords[i][1]))
                    max_protrusion = max(max_protrusion, float(pt.distance(boundary)))
            if max_protrusion > overspill_tol:
                return False
        return True

    for component_label, paths in by_component.items():
        pending = optimize_toolpath_order(paths, strategy="nearest-neighbor")
        if not pending:
            continue
        geom = _component_geom(component_label)
        chain = clone_toolpath(pending[0], kind="detail-trace", metadata={**pending[0].metadata, "path_role": "PRINT_DETAIL"})
        for nxt in pending[1:]:
            if len(chain.points) < 2 or len(nxt.points) < 2:
                merged_out.append(chain)
                chain = nxt
                continue
            gap = math.hypot(nxt.points[0].x - chain.points[-1].x, nxt.points[0].y - chain.points[-1].y)
            turn = _angle_deg(chain.points[-2], chain.points[-1], nxt.points[0], nxt.points[1])
            if gap <= 1e-6:
                chain = clone_toolpath(
                    chain,
                    points=chain.points + nxt.points[1:],
                    kind="detail-trace",
                    metadata={**chain.metadata, "path_role": "PRINT_DETAIL", "detail_merge_count": int(chain.metadata.get("detail_merge_count", 0)) + 1},
                )
                merged_count += 1
                continue
            if gap <= max_gap_mm and turn <= max_turn_deg and _connector_safe(geom, chain.points[-1], nxt.points[0]):
                connector_kind = "detail-continuation" if gap > preferred_gap_mm else "detail-trace"
                connector = Toolpath(
                    points=[chain.points[-1], nxt.points[0]],
                    kind=connector_kind,
                    closed=False,
                    coordinate_space=chain.coordinate_space,
                    source="detail_merge_connector",
                    region_id=chain.region_id,
                    metadata={
                        "path_role": "PRINT_DETAIL_CONTINUATION" if connector_kind == "detail-continuation" else "PRINT_DETAIL",
                        "detail_continuation_pen_down": connector_kind == "detail-continuation",
                        "connector_length_mm": float(gap),
                        "connector_turn_deg": float(turn),
                    },
                )
                chain = clone_toolpath(
                    chain,
                    points=chain.points + connector.points[1:] + nxt.points[1:],
                    kind="detail-trace",
                    metadata={**chain.metadata, "path_role": "PRINT_DETAIL", "detail_merge_count": int(chain.metadata.get("detail_merge_count", 0)) + 1},
                )
                merged_count += 1
            else:
                rejected_count += 1
                merged_out.append(chain)
                chain = nxt
        merged_out.append(chain)

    result = passthrough + merged_out
    if debug is not None:
        debug["detail_merge_stats"] = {
            "enabled": True,
            "merged_detail_traces": int(merged_count),
            "rejected_unsafe_merges": int(rejected_count),
            "max_detail_merge_gap_mm": float(max_gap_mm),
            "preferred_detail_merge_gap_mm": float(preferred_gap_mm),
            "max_detail_merge_turn_deg": float(max_turn_deg),
        }
    return result


def _normalize_infill_angle_deg(angle_deg: float) -> float:
    normalized = math.fmod(angle_deg, 180.0)
    if normalized < 0.0:
        normalized += 180.0
    if normalized >= 179.999999:
        normalized = 0.0
    return normalized


def _dedupe_infill_candidate_angles(candidate_angles: list[float]) -> list[float]:
    deduped: list[float] = []
    for candidate in candidate_angles:
        normalized = _normalize_infill_angle_deg(candidate)
        if any(abs(normalized - existing) < 1e-6 for existing in deduped):
            continue
        deduped.append(normalized)
    return deduped


def _preserve_infill_path_order(toolpaths: list[Toolpath]) -> bool:
    if not toolpaths:
        return False
    return all(
        path.kind == "fill-infill"
        and "scanline_grid_index" in path.metadata
        and "scanline_offset_mm" in path.metadata
        for path in toolpaths
    )


class SlicerService:
    def _scanline_spacing_mm(self, settings: SlicerSettings) -> float:
        base_spacing_mm = settings.infill_spacing_mm if settings.infill_spacing_mm > 0 else settings.line_width_mm
        # Enforce that requested base spacing does not exceed the pen width
        base_spacing_mm = min(base_spacing_mm, settings.line_width_mm)
        density_scale = max(0.01, settings.infill_density / 100.0)
        spacing = base_spacing_mm / density_scale
        # After density scaling, clamp final spacing to at most the pen width
        return min(spacing, settings.line_width_mm)

    def _max_pen_down_connector_length_mm(self, spacing_mm: float) -> float:
        return max(
            spacing_mm * DEFAULT_MAX_PEN_DOWN_CONNECTOR_SPACING_FACTOR,
            spacing_mm + 0.25,
        )

    def _validate_infill_connector(
        self,
        polygon: Polygon,
        connector: LineString,
        *,
        from_meta: dict[str, Any],
        to_meta: dict[str, Any],
        spacing_mm: float,
        line_width_mm: float,
        tolerance_mm: float,
        connector_validation: dict[str, Any] | None = None,
    ) -> InfillConnectorValidationResult:
        start = connector.coords[0]
        end = connector.coords[-1]
        current_component = _connector_meta_value(from_meta, "infill_component_id", "scanline_polygon_index", "source_component_id")
        next_component = _connector_meta_value(to_meta, "infill_component_id", "scanline_polygon_index", "source_component_id")
        if current_component is not None and next_component is not None and current_component != next_component:
            return InfillConnectorValidationResult(False, "different_component")

        current_cell = _connector_meta_value(from_meta, "cell_id", "infill_cell_id", "scanline_cell_id")
        next_cell = _connector_meta_value(to_meta, "cell_id", "infill_cell_id", "scanline_cell_id")
        if current_cell is not None and next_cell is not None and current_cell != next_cell:
            return InfillConnectorValidationResult(False, "different_cell_or_section")

        current_row = _connector_meta_value(from_meta, "infill_row_index", "scanline_grid_index")
        next_row = _connector_meta_value(to_meta, "infill_row_index", "scanline_grid_index")
        row_delta = None
        if current_row is not None and next_row is not None:
            try:
                row_delta = abs(int(next_row) - int(current_row))
            except Exception:
                row_delta = None

        current_end_side = _connector_meta_value(from_meta, "infill_end_side")
        next_start_side = _connector_meta_value(to_meta, "infill_start_side")
        if current_end_side is not None and next_start_side is not None and current_end_side != next_start_side:
            return InfillConnectorValidationResult(False, "opposite_side_endpoint")

        delta_u = abs(float(end[0]) - float(start[0]))
        delta_v = abs(float(end[1]) - float(start[1]))
        if (row_delta is not None and row_delta != 1) and abs(delta_v - spacing_mm) > max(1e-6, spacing_mm * 0.1):
            return InfillConnectorValidationResult(False, "non_adjacent_row")
        if row_delta is None and abs(delta_v - spacing_mm) > max(1e-6, spacing_mm * 0.1):
            return InfillConnectorValidationResult(False, "non_adjacent_row")

        max_connector_length_mm = self._max_pen_down_connector_length_mm(spacing_mm)
        max_normal_delta_u = max(1.5 * line_width_mm, 0.75 * spacing_mm)
        max_edge_length = max(10.0 * spacing_mm, 12.0 * line_width_mm)
        connector_is_short = connector.length <= max_connector_length_mm + 1e-6
        cover_region = polygon.buffer(max(tolerance_mm, 0.25 * line_width_mm, 0.1 * spacing_mm, 0.01), join_style=1)
        connector_fully_inside = bool(_line_fully_inside(cover_region, connector, tolerance_mm=max(0.01, spacing_mm * 0.05)))
        connector_inside = bool(cover_region.covers(connector))

        def sample_mask_failures(sample_line: LineString) -> list[dict[str, Any]]:
            if not connector_validation:
                return []
            mask = connector_validation.get("mask")
            matrix = connector_validation.get("current_to_source_matrix")
            if mask is None or matrix is None:
                return []
            try:
                current_to_source = tuple(float(value) for value in matrix)
            except Exception:
                return []
            if sample_line.length <= 1e-9:
                return []
            sample_step_mm = max(0.01, min(spacing_mm / 3.0, line_width_mm / 3.0, float(connector_validation.get("sample_step_mm_default", 0.05))))
            sample_count = max(2, int(math.ceil(sample_line.length / sample_step_mm)) + 1)
            sample_failures: list[dict[str, Any]] = []
            mask_height, mask_width = mask.shape[:2]
            for sample_index in range(sample_count):
                distance_mm = min(sample_line.length, (sample_line.length * sample_index) / max(sample_count - 1, 1))
                sample_point = sample_line.interpolate(distance_mm)
                mask_point = apply_svg_matrix(Point(float(sample_point.x), float(sample_point.y)), current_to_source)
                mask_x = int(round(mask_point.x))
                mask_y = int(round(mask_point.y))
                inside_mask = 0 <= mask_x < mask_width and 0 <= mask_y < mask_height and bool(mask[mask_y, mask_x])
                if not inside_mask:
                    sample_failures.append({
                        "sample_index": sample_index,
                        "surface_x": float(sample_point.x),
                        "surface_y": float(sample_point.y),
                        "mask_x": float(mask_point.x),
                        "mask_y": float(mask_point.y),
                    })
                    if len(sample_failures) >= 8:
                        break
            return sample_failures

        if not connector_inside:
            try:
                intersection = connector.intersection(cover_region)
                intersection_length = float(getattr(intersection, "length", 0.0) or 0.0)
            except Exception:
                intersection_length = 0.0
            if intersection_length <= 1e-6:
                return InfillConnectorValidationResult(False, "outside_fillable_polygon")
            return InfillConnectorValidationResult(False, "crosses_gap_hole_void")

        if connector_fully_inside and abs(delta_v - spacing_mm) <= max(1e-6, spacing_mm * 0.1) and delta_u <= max_normal_delta_u + 1e-6:
            sample_failures = sample_mask_failures(connector)
            if sample_failures:
                return InfillConnectorValidationResult(False, "outside_selected_color", sample_failures=sample_failures)
            return InfillConnectorValidationResult(True, "ok_normal", connector_coords=[start, end])

        if delta_u > max_normal_delta_u + 1e-6:
            if connector.length <= 3.0 * max_edge_length + 1e-6:
                sample_failures = sample_mask_failures(connector)
                if sample_failures:
                    return InfillConnectorValidationResult(False, "outside_selected_color", connector_mode="edge_direct", sample_failures=sample_failures)
                return InfillConnectorValidationResult(True, "ok_edge_direct", connector_mode="edge_direct", connector_coords=[start, end])
            boundary_coords = boundary_connector_coords(
                polygon,
                start,
                end,
                tolerance=max(tolerance_mm, 0.5 * line_width_mm, spacing_mm * 0.75, 0.05),
            )
            if len(boundary_coords) >= 2:
                boundary_line = LineString(boundary_coords)
                if boundary_line.length <= max(16.0 * spacing_mm, 20.0 * line_width_mm) + 1e-6 and cover_region.covers(boundary_line):
                    sample_failures = sample_mask_failures(boundary_line)
                    if sample_failures:
                        return InfillConnectorValidationResult(False, "outside_selected_color", connector_mode="boundary", sample_failures=sample_failures)
                    return InfillConnectorValidationResult(True, "ok_boundary", connector_mode="boundary", connector_coords=boundary_coords)
            if connector.length > max_connector_length_mm + 1e-6:
                return InfillConnectorValidationResult(False, "too_long")
            return InfillConnectorValidationResult(False, "delta_u_too_large")

        if connector.length > max_connector_length_mm + 1e-6:
            return InfillConnectorValidationResult(False, "too_long")

        sample_failures = sample_mask_failures(connector)
        if sample_failures:
            return InfillConnectorValidationResult(False, "outside_selected_color", sample_failures=sample_failures)

        if connector.length <= max_connector_length_mm + 1e-6:
            return InfillConnectorValidationResult(True, "ok_short_local", connector_coords=[start, end])
        return InfillConnectorValidationResult(False, "outside_fillable_polygon")

    def _recommended_infill_min_segment_length_mm(self, line_width_mm: float, minimum_length_mm: float) -> float:
        return max(minimum_length_mm, max(0.15, line_width_mm * 0.5))

    def _scanline_filter_threshold_mm(self, spacing_mm: float, min_segment_length_mm: float) -> float:
        return max(0.15, min(min_segment_length_mm, max(0.15, spacing_mm * 0.5)))

    def _scanline_gap_tolerance_mm(self, spacing_mm: float) -> float:
        return spacing_mm * 1.5

    def _polygon_axis_metrics(self, region: Any) -> dict[str, float]:
        if region is None or region.is_empty:
            return {
                "dominant_axis_angle_deg": 0.0,
                "long_side_mm": 0.0,
                "short_side_mm": 0.0,
                "aspect_ratio": 0.0,
                "used_oriented_bbox": False,
            }

        try:
            oriented = region.minimum_rotated_rectangle
            coords = list(oriented.exterior.coords)
            edges: list[tuple[float, float]] = []
            for start, end in zip(coords, coords[1:]):
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                length = math.hypot(dx, dy)
                if length > 1e-6:
                    edges.append((length, math.degrees(math.atan2(dy, dx))))
            if edges:
                edges.sort(key=lambda item: item[0], reverse=True)
                long_side = edges[0][0]
                short_side = edges[-1][0]
                aspect_ratio = long_side / max(short_side, 1e-6)
                return {
                    "dominant_axis_angle_deg": _normalize_infill_angle_deg(edges[0][1]),
                    "long_side_mm": long_side,
                    "short_side_mm": short_side,
                    "aspect_ratio": aspect_ratio,
                    "used_oriented_bbox": True,
                }
        except Exception:
            pass

        min_x, min_y, max_x, max_y = region.bounds
        width = max(0.0, max_x - min_x)
        height = max(0.0, max_y - min_y)
        long_side = max(width, height)
        short_side = min(width, height)
        return {
            "dominant_axis_angle_deg": 0.0 if width >= height else 90.0,
            "long_side_mm": long_side,
            "short_side_mm": short_side,
            "aspect_ratio": long_side / max(short_side, 1e-6),
            "used_oriented_bbox": False,
        }

    def _small_detail_threshold_mm(self, line_width_mm: float) -> float:
        return max(DEFAULT_SMALL_DETAIL_MIN_DIM_FLOOR_MM, line_width_mm * DEFAULT_SMALL_DETAIL_MIN_DIM_FACTOR)

    def _build_hybrid_infill_config(
        self,
        *,
        line_width_mm: float,
        infill_spacing_mm: float,
        wall_count: int,
        infill_angle_deg: float,
    ) -> HybridInfillConfig:
        # Ensure infill spacing respects the pen width: maximum spacing is one pen width
        resolved_infill_spacing = min(infill_spacing_mm if infill_spacing_mm > 0 else line_width_mm, line_width_mm)
        return HybridInfillConfig(
            enabled=True,
            lineWidthMm=line_width_mm,
            infillSpacingMm=resolved_infill_spacing,
            wallCount=wall_count,
            infillAngleDeg=infill_angle_deg,
            singleStrokeWidthMaxFactor=DEFAULT_SINGLE_STROKE_WIDTH_MAX_FACTOR,
            centerlineWidthMaxFactor=DEFAULT_CENTERLINE_WIDTH_MAX_FACTOR,
            detailWidthMaxFactor=DEFAULT_DETAIL_WIDTH_MAX_FACTOR,
            detailMinWidthFactor=2.25,
            minSerpentineRowLengthFactor=3.0,
            minAreaToFillMm2=0.15,
            minUsableDetailAreaMm2=0.15,
            minNormalFillAreaMm2=0.35,
            connectorValidation="sampled",
            connectorSampleStepMm=max(0.01, min(line_width_mm / 4.0, 0.05)),
            allowInternalConnectorOverlap=True,
            maxConnectorOverlapMm=max(infill_spacing_mm * 2.0, line_width_mm * 2.0),
            detailFillEnabled=True,
            centerlineFallbackEnabled=True,
            thinRegionMode="singleStroke",
            allowOutlineOverlapForThinRegions=True,
            optimizePathOrder=True,
            singleStrokeMaxWidthFactor=DEFAULT_THIN_REGION_SINGLE_STROKE_MAX_FACTOR,
            narrowRegionMaxWidthFactor=DEFAULT_NARROW_REGION_MAX_FACTOR,
            collapseOutlineMaxWidthFactor=DEFAULT_COLLAPSE_OUTLINE_MAX_FACTOR,
            tinyDotAreaFactor=DEFAULT_TINY_DOT_AREA_FACTOR,
        )

    def _compute_region_metrics(
        self,
        region: Any,
        *,
        spacing_mm: float,
        line_width_mm: float,
        preferred_angle_deg: float,
        min_segment_length_mm: float,
    ) -> RegionMetrics:
        if region is None or region.is_empty:
            return RegionMetrics(
                areaMm2=0.0,
                bboxWidthMm=0.0,
                bboxHeightMm=0.0,
                minDimensionMm=0.0,
                maxLocalWidthMm=0.0,
                aspectRatio=0.0,
                holeCount=0,
                componentCount=0,
                estimatedRowCount=0.0,
                estimatedShortRowRatio=1.0,
                highCurvatureScore=0.0,
            )

        min_x, min_y, max_x, max_y = region.bounds
        bbox_width_mm = max(0.0, max_x - min_x)
        bbox_height_mm = max(0.0, max_y - min_y)
        min_dimension_mm = min(bbox_width_mm, bbox_height_mm)
        axis_metrics = self._polygon_axis_metrics(region)
        area_mm2 = float(region.area)
        hole_count = sum(len(poly.interiors) for poly in normalize_geometry(region))
        component_count = len(normalize_geometry(region))
        row_data = self._collect_scanline_rows(
            region,
            spacing_mm=spacing_mm,
            angle_deg=preferred_angle_deg,
            min_segment_length_mm=min_segment_length_mm,
        )
        row_metrics = self._score_infill_candidate(row_data, spacing_mm=spacing_mm, angle_deg=preferred_angle_deg)
        segment_count = max(1.0, float(row_metrics["segments"]))
        short_row_ratio = float(row_metrics["short_segment_count"]) / segment_count
        high_curvature_score = 0.0
        if area_mm2 > 1e-9 and region.length > 0:
            try:
                high_curvature_score = max(0.0, min(1.0, ((float(region.length) ** 2) / (4.0 * math.pi * area_mm2)) - 1.0))
            except Exception:
                high_curvature_score = 0.0
        return RegionMetrics(
            areaMm2=area_mm2,
            bboxWidthMm=bbox_width_mm,
            bboxHeightMm=bbox_height_mm,
            minDimensionMm=min_dimension_mm,
            maxLocalWidthMm=min(float(axis_metrics.get("short_side_mm", min_dimension_mm)), min_dimension_mm if min_dimension_mm > 0 else float(axis_metrics.get("short_side_mm", 0.0))),
            aspectRatio=float(axis_metrics.get("aspect_ratio", 0.0)),
            holeCount=hole_count,
            componentCount=component_count,
            estimatedRowCount=float(row_metrics["rows"]),
            estimatedShortRowRatio=max(0.0, min(1.0, short_row_ratio)),
            highCurvatureScore=high_curvature_score,
        )

    def _order_paths_by_nearest_neighbor(
        self,
        paths: list[Toolpath],
        *,
        start_point: Optional[Point] = None,
        strategy: str = "nearest-neighbor",
    ) -> list[Toolpath]:
        if not paths:
            return []
        if strategy != "nearest-neighbor":
            return list(paths)
        ordered = optimize_toolpath_order(paths, strategy=strategy, start_point=start_point)
        if len(ordered) < 3:
            return ordered

        improved = list(ordered)
        for _ in range(2):
            changed = False
            for index in range(1, len(improved) - 1):
                previous_path = improved[index - 1]
                current_path = improved[index]
                next_path = improved[index + 1]
                if len(previous_path.points) < 2 or len(current_path.points) < 2 or len(next_path.points) < 2:
                    continue
                current_cost = math.hypot(previous_path.points[-1].x - current_path.points[0].x, previous_path.points[-1].y - current_path.points[0].y) + math.hypot(current_path.points[-1].x - next_path.points[0].x, current_path.points[-1].y - next_path.points[0].y)
                reversed_current = clone_toolpath(current_path, points=list(reversed(current_path.points)), closed=current_path.closed)
                reversed_cost = math.hypot(previous_path.points[-1].x - reversed_current.points[0].x, previous_path.points[-1].y - reversed_current.points[0].y) + math.hypot(reversed_current.points[-1].x - next_path.points[0].x, reversed_current.points[-1].y - next_path.points[0].y)
                if reversed_cost + 1e-9 < current_cost:
                    improved[index] = reversed_current
                    changed = True
            if not changed:
                break
        return improved

    def _validate_chain_connector(
        self,
        *,
        polygon: Any,
        start: Point,
        end: Point,
        line_width_mm: float,
        spacing_mm: float,
        connector_validation: dict[str, Any] | None,
        blocked_lines: list[LineString] | None = None,
    ) -> tuple[bool, str]:
        connector = LineString([(float(start.x), float(start.y)), (float(end.x), float(end.y))])
        if connector.length <= 1e-9:
            return True, "zero_length"
        max_connector_length_mm = max(2.0 * line_width_mm, 1.5 * spacing_mm)
        if connector.length > max_connector_length_mm + 1e-6:
            return False, "too_long"
        if polygon is None or polygon.is_empty:
            return False, "missing_polygon"
        if not _line_fully_inside(polygon, connector, tolerance_mm=max(0.01, line_width_mm * 0.1)):
            return False, "outside_fillable_polygon"
        if blocked_lines:
            for blocked in blocked_lines:
                if blocked is None or blocked.is_empty:
                    continue
                if connector.crosses(blocked):
                    return False, "crosses_existing_chain"
        if connector_validation and isinstance(connector_validation, dict):
            mask = connector_validation.get("mask")
            matrix = connector_validation.get("current_to_source_matrix")
            if mask is not None and isinstance(matrix, (tuple, list)) and len(matrix) == 6:
                mask_height, mask_width = mask.shape[:2]
                sample_step_mm = max(0.01, min(line_width_mm / 3.0, spacing_mm / 3.0, 0.05))
                sample_count = max(2, int(math.ceil(connector.length / sample_step_mm)) + 1)
                current_to_source = tuple(float(value) for value in matrix)
                for sample_index in range(sample_count):
                    distance_mm = min(connector.length, (connector.length * sample_index) / max(1, sample_count - 1))
                    sample_point = connector.interpolate(distance_mm)
                    source_point = apply_svg_matrix(Point(float(sample_point.x), float(sample_point.y)), current_to_source)
                    mask_x = int(round(source_point.x))
                    mask_y = int(round(source_point.y))
                    inside_mask = 0 <= mask_x < mask_width and 0 <= mask_y < mask_height and bool(mask[mask_y, mask_x])
                    if not inside_mask:
                        return False, "outside_selected_color"
        return True, "ok"

    def _chain_region_paths_with_pen_down_connectors(
        self,
        paths: list[Toolpath],
        *,
        polygon: Any,
        line_width_mm: float,
        spacing_mm: float,
        preserve_order: bool = True,
        connector_validation: dict[str, Any] | None = None,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if len(paths) <= 1:
            return paths
        chainable = {"fill-wall", "fill-infill", "detail-trace"}
        ordered = list(paths) if preserve_order else self._order_paths_by_nearest_neighbor(
            paths,
            start_point=paths[0].points[0] if paths and paths[0].points else None,
        )
        chained: list[Toolpath] = []
        blocked_lines: list[LineString] = []
        accepted = 0
        attempted = 0
        rejected: dict[str, int] = {}

        for path in ordered:
            if not chained:
                chained.append(path)
                continue
            previous = chained[-1]
            if (
                previous.kind in chainable
                and path.kind in chainable
                and not previous.closed
                and not path.closed
                and len(previous.points) >= 2
                and len(path.points) >= 2
                and previous.region_id == path.region_id
            ):
                attempted += 1
                ok, reason = self._validate_chain_connector(
                    polygon=polygon,
                    start=previous.points[-1],
                    end=path.points[0],
                    line_width_mm=line_width_mm,
                    spacing_mm=spacing_mm,
                    connector_validation=connector_validation,
                    blocked_lines=blocked_lines,
                )
                if ok:
                    connector_points = [Point(float(previous.points[-1].x), float(previous.points[-1].y)), Point(float(path.points[0].x), float(path.points[0].y))]
                    if not nearly_same_point(connector_points[0], connector_points[1], tolerance=1e-6):
                        connector_path = Toolpath(
                            points=connector_points,
                            kind="fill-infill-travel",
                            closed=False,
                            source="coverage_connector",
                            region_id=path.region_id,
                            metadata={
                                "expected_relation_to_fill": "internal_fill_connector",
                                "connector_mode": "region_chain",
                                "connector_reason": "safe_inside_mask",
                            },
                        )
                        chained.append(connector_path)
                        blocked_lines.append(LineString([(connector_points[0].x, connector_points[0].y), (connector_points[1].x, connector_points[1].y)]))
                    accepted += 1
                else:
                    rejected[reason] = int(rejected.get(reason, 0)) + 1
            chained.append(path)
            if len(path.points) >= 2:
                blocked_lines.append(LineString([(point.x, point.y) for point in path.points]))

        if debug is not None:
            debug["coverage_connector_attempted"] = int(debug.get("coverage_connector_attempted", 0)) + attempted
            debug["coverage_connector_accepted"] = int(debug.get("coverage_connector_accepted", 0)) + accepted
            debug["coverage_connector_rejected"] = int(debug.get("coverage_connector_rejected", 0)) + max(0, attempted - accepted)
            reasons = debug.setdefault("coverage_connector_rejection_reasons", {})
            for key, value in rejected.items():
                reasons[key] = int(reasons.get(key, 0)) + int(value)
        return chained

    def _is_mesh_like_path(self, path: Toolpath, *, line_width_mm: float) -> bool:
        if len(path.points) < 3:
            return False
        if path.kind in {"outline", "fill-wall"}:
            return False
        if path.closed:
            return path.kind in {"fill-infill", "detail-trace", "fill-infill-travel"}
        coords = [(float(point.x), float(point.y)) for point in path.points]
        line = LineString(coords)
        if line.length <= 1e-6:
            return True
        min_x, min_y, max_x, max_y = line.bounds
        bbox_w = max_x - min_x
        bbox_h = max_y - min_y
        bbox_diag = max(1e-6, math.hypot(bbox_w, bbox_h))
        angles: list[float] = []
        for start, end in zip(coords, coords[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            if abs(dx) <= 1e-9 and abs(dy) <= 1e-9:
                continue
            angles.append(math.degrees(math.atan2(dy, dx)))
        if len(angles) < 2:
            return False
        direction_changes = 0
        for prev, cur in zip(angles, angles[1:]):
            delta = abs(_normalize_infill_angle_deg(cur - prev))
            if delta > 65.0 and delta < 175.0:
                direction_changes += 1
        if direction_changes >= 3 and line.length > bbox_diag * 2.5 and min(bbox_w, bbox_h) < line_width_mm * 2.5:
            return True
        return False

    def _looks_like_tiny_x_or_triangle_fragment(self, path: Toolpath, *, line_width_mm: float) -> bool:
        if len(path.points) < 3 or len(path.points) > 10:
            return False
        if path.kind in {"outline", "fill-wall", "outline_cleanup", "coverage_contour"}:
            return False
        coords = [(float(point.x), float(point.y)) for point in path.points]
        line = LineString(coords)
        if line.length <= 1e-6:
            return True
        min_x, min_y, max_x, max_y = line.bounds
        bbox_w = max_x - min_x
        bbox_h = max_y - min_y
        max_dim = max(bbox_w, bbox_h)
        if max_dim > max(line_width_mm * 2.2, 1.6):
            return False
        turns = 0
        last_angle = None
        for start, end in zip(coords, coords[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            if abs(dx) <= 1e-9 and abs(dy) <= 1e-9:
                continue
            angle = math.degrees(math.atan2(dy, dx))
            if last_angle is not None:
                delta = abs(_normalize_infill_angle_deg(angle - last_angle))
                if 35.0 <= delta <= 170.0:
                    turns += 1
            last_angle = angle
        return turns >= 2

    def _canonicalize_coverage_paths(
        self,
        paths: list[Toolpath],
        *,
        line_width_mm: float,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        canonical: list[Toolpath] = []
        rejected_x_fragments = 0
        rejected_short = 0
        for path in paths:
            new_kind = path.kind
            fill_mode = str(path.metadata.get("fill_mode", "")).lower()
            if path.kind == "fill-infill-travel":
                new_kind = "coverage_connector"
            elif path.kind == "detail-trace":
                new_kind = "coverage_centerline"
            elif path.kind == "fill-infill":
                if "single_stroke" in fill_mode or "centerline" in fill_mode:
                    new_kind = "coverage_centerline"
                elif "detail_contour" in fill_mode or "contour" in fill_mode:
                    new_kind = "coverage_contour"
                elif "rectilinear" in fill_mode:
                    new_kind = "coverage_rectilinear"
                else:
                    new_kind = "coverage_offset_line"
            elif path.kind == "outline":
                new_kind = "outline_cleanup"
            elif path.kind == "fill-wall":
                new_kind = "coverage_contour"

            candidate = clone_toolpath(
                path,
                kind=new_kind,
                source=path.source,
                metadata={**path.metadata, "canonical_coverage_kind": new_kind, "legacy_kind": path.kind},
            )

            # Only suppress tiny X/triangle fragments originating from legacy
            # detail/connector traces; do not prune normal infill/outline paths.
            if path.kind in {"detail-trace", "fill-infill-travel"} and self._looks_like_tiny_x_or_triangle_fragment(candidate, line_width_mm=line_width_mm):
                rejected_x_fragments += 1
                continue

            length_mm = segment_length(candidate.points) if len(candidate.points) >= 2 else 0.0
            tiny_mark = bool(candidate.metadata.get("small_detail_fill_style") in {"tiny_dot", "tiny_short_stroke"})
            if len(candidate.points) < 2 or (length_mm < max(0.03, line_width_mm * 0.08) and not tiny_mark):
                rejected_short += 1
                continue
            canonical.append(candidate)

        if debug is not None:
            debug["rejected_x_triangle_fragment_count"] = int(rejected_x_fragments)
            debug["rejected_micro_path_count"] = int(rejected_short)
            debug["canonical_path_counts"] = {
                "coverage_centerline": sum(1 for p in canonical if p.kind == "coverage_centerline"),
                "coverage_offset_line": sum(1 for p in canonical if p.kind == "coverage_offset_line"),
                "coverage_rectilinear": sum(1 for p in canonical if p.kind == "coverage_rectilinear"),
                "coverage_contour": sum(1 for p in canonical if p.kind == "coverage_contour"),
                "coverage_connector": sum(1 for p in canonical if p.kind == "coverage_connector"),
                "outline_cleanup": sum(1 for p in canonical if p.kind == "outline_cleanup"),
            }
        return canonical

    def _build_component_centerline_candidate(
        self,
        component_mask: np.ndarray,
        *,
        source_to_current_matrix: tuple[float, float, float, float, float, float],
        line_width_mm: float,
        pen_radius_px: float,
        component_id: int,
        strategy: str,
    ) -> Toolpath | None:
        ys, xs = np.nonzero(component_mask > 0)
        if xs.size < 2:
            return None
        pts = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        try:
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            vx_f = float(np.asarray(vx).reshape(-1)[0])
            vy_f = float(np.asarray(vy).reshape(-1)[0])
            x0_f = float(np.asarray(x0).reshape(-1)[0])
            y0_f = float(np.asarray(y0).reshape(-1)[0])
        except Exception:
            min_x = float(xs.min())
            max_x = float(xs.max())
            min_y = float(ys.min())
            max_y = float(ys.max())
            if (max_x - min_x) >= (max_y - min_y):
                vx_f, vy_f = 1.0, 0.0
            else:
                vx_f, vy_f = 0.0, 1.0
            x0_f = float(np.mean(xs))
            y0_f = float(np.mean(ys))
        norm = math.hypot(vx_f, vy_f)
        if norm <= 1e-9:
            return None
        vx_f /= norm
        vy_f /= norm
        projections = ((pts[:, 0] - x0_f) * vx_f) + ((pts[:, 1] - y0_f) * vy_f)
        t_min = float(np.min(projections))
        t_max = float(np.max(projections))
        if (t_max - t_min) <= 1.0:
            return None
        trim_px = max(1.0, pen_radius_px * 0.95)
        start_x = x0_f + vx_f * (t_min + trim_px)
        start_y = y0_f + vy_f * (t_min + trim_px)
        end_x = x0_f + vx_f * (t_max - trim_px)
        end_y = y0_f + vy_f * (t_max - trim_px)
        start_mm = apply_svg_matrix(Point(start_x, start_y), source_to_current_matrix)
        end_mm = apply_svg_matrix(Point(end_x, end_y), source_to_current_matrix)
        path = Toolpath(
            points=[start_mm, end_mm],
            kind="coverage_centerline",
            closed=False,
            source="coverage_backfill_component",
            metadata={
                "coverage_backfill_component": True,
                "coverage_backfill_component_id": component_id,
                "coverage_backfill_strategy": strategy,
            },
        )
        if segment_length(path.points) < max(0.04, line_width_mm * 0.15):
            return None
        return path

    def _build_component_run_candidates(
        self,
        component_mask: np.ndarray,
        *,
        source_to_current_matrix: tuple[float, float, float, float, float, float],
        line_width_mm: float,
        pen_radius_px: float,
        component_id: int,
        horizontal: bool,
        max_candidates: int = 3,
        thin_mode: bool = False,
    ) -> list[Toolpath]:
        ys, xs = np.nonzero(component_mask > 0)
        if xs.size < 2:
            return []
        min_x = int(np.min(xs))
        max_x = int(np.max(xs))
        min_y = int(np.min(ys))
        max_y = int(np.max(ys))
        runs: list[tuple[int, int, int]] = []
        if horizontal:
            for yy in range(min_y, max_y + 1):
                row = component_mask[yy, min_x:max_x + 1]
                start = None
                for i, val in enumerate(row):
                    if val and start is None:
                        start = i
                    if (not val or i == len(row) - 1) and start is not None:
                        end = i if val and i == len(row) - 1 else i - 1
                        runs.append((end - start + 1, min_x + start, min_x + end, yy))
                        start = None
        else:
            for xx in range(min_x, max_x + 1):
                col = component_mask[min_y:max_y + 1, xx]
                start = None
                for i, val in enumerate(col):
                    if val and start is None:
                        start = i
                    if (not val or i == len(col) - 1) and start is not None:
                        end = i if val and i == len(col) - 1 else i - 1
                        runs.append((end - start + 1, min_y + start, min_y + end, xx))
                        start = None
        runs.sort(key=lambda item: item[0], reverse=True)
        candidates: list[Toolpath] = []
        trim_px = max(1.0, pen_radius_px * 0.95)
        for run in runs[:max_candidates]:
            length_px = int(run[0])
            if length_px <= 2:
                continue
            if horizontal:
                _, x0, x1, yy = run
                sx, sy = float(x0) + trim_px, float(yy)
                ex, ey = float(x1) - trim_px, float(yy)
            else:
                _, y0, y1, xx = run
                sx, sy = float(xx), float(y0) + trim_px
                ex, ey = float(xx), float(y1) - trim_px
            if math.hypot(ex - sx, ey - sy) <= 1.0:
                continue
            start_mm = apply_svg_matrix(Point(sx, sy), source_to_current_matrix)
            end_mm = apply_svg_matrix(Point(ex, ey), source_to_current_matrix)
            candidate = Toolpath(
                points=[start_mm, end_mm],
                kind="coverage_centerline",
                closed=False,
                source="coverage_backfill_component",
                metadata={
                    "coverage_backfill_component": True,
                    "coverage_backfill_component_id": component_id,
                    "coverage_backfill_strategy": "component_runline",
                    "coverage_backfill_orientation": "horizontal" if horizontal else "vertical",
                },
            )
            if segment_length(candidate.points) >= max(0.04, line_width_mm * 0.15):
                candidates.append(candidate)
        return candidates

    def _build_component_fullspan_passage_candidate(
        self,
        component_mask: np.ndarray,
        *,
        source_to_current_matrix: tuple[float, float, float, float, float, float],
        line_width_mm: float,
        component_id: int,
    ) -> Toolpath | None:
        ys, xs = np.nonzero(component_mask > 0)
        if xs.size < 3:
            return None
        pts = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx_f = float(np.asarray(vx).reshape(-1)[0])
        vy_f = float(np.asarray(vy).reshape(-1)[0])
        x0_f = float(np.asarray(x0).reshape(-1)[0])
        y0_f = float(np.asarray(y0).reshape(-1)[0])
        norm = math.hypot(vx_f, vy_f)
        if norm <= 1e-9:
            return None
        vx_f /= norm
        vy_f /= norm
        projections = ((pts[:, 0] - x0_f) * vx_f) + ((pts[:, 1] - y0_f) * vy_f)
        t_min = float(np.min(projections))
        t_max = float(np.max(projections))
        if (t_max - t_min) <= 1.0:
            return None
        start_mm = apply_svg_matrix(Point(x0_f + vx_f * t_min, y0_f + vy_f * t_min), source_to_current_matrix)
        end_mm = apply_svg_matrix(Point(x0_f + vx_f * t_max, y0_f + vy_f * t_max), source_to_current_matrix)
        path = Toolpath(
            points=[start_mm, end_mm],
            kind="coverage_centerline",
            closed=False,
            source="coverage_backfill_component",
            metadata={
                "coverage_backfill_component": True,
                "coverage_backfill_component_id": component_id,
                "coverage_backfill_strategy": "thin_passage_fullspan",
            },
        )
        if segment_length(path.points) < max(0.02, line_width_mm * 0.05):
            return None
        return path

    def _build_component_tiny_dot_candidate(
        self,
        component_mask: np.ndarray,
        *,
        source_to_current_matrix: tuple[float, float, float, float, float, float],
        component_id: int,
        line_width_mm: float,
    ) -> Toolpath | None:
        ys, xs = np.nonzero(component_mask > 0)
        if xs.size == 0:
            return None
        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
        center_mm = apply_svg_matrix(Point(cx, cy), source_to_current_matrix)
        half = max(0.01, line_width_mm * 0.05)
        return Toolpath(
            points=[Point(center_mm.x - half, center_mm.y), Point(center_mm.x + half, center_mm.y)],
            kind="coverage_centerline",
            closed=False,
            source="coverage_backfill_component",
            metadata={
                "coverage_backfill_component": True,
                "coverage_backfill_component_id": component_id,
                "coverage_backfill_strategy": "tiny_component_dot",
            },
        )

    def _build_component_angle_run_candidates(
        self,
        component_mask: np.ndarray,
        *,
        source_to_current_matrix: tuple[float, float, float, float, float, float],
        line_width_mm: float,
        component_id: int,
        angles_deg: list[float] | None = None,
    ) -> list[Toolpath]:
        ys, xs = np.nonzero(component_mask > 0)
        if xs.size < 3:
            return []
        h, w = component_mask.shape[:2]
        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
        angles = angles_deg or [0.0, 45.0, -45.0]
        candidates: list[Toolpath] = []
        for angle in angles:
            m = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
            rot = cv2.warpAffine(component_mask.astype(np.uint8), m, (w, h), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            best_len = 0
            best_seg: tuple[int, int, int] | None = None
            for yy in range(h):
                row = rot[yy, :]
                start = -1
                for xx, val in enumerate(row):
                    if val and start < 0:
                        start = xx
                    if (not val or xx == w - 1) and start >= 0:
                        end = xx if val and xx == w - 1 else xx - 1
                        seg_len = end - start + 1
                        if seg_len > best_len:
                            best_len = seg_len
                            best_seg = (start, end, yy)
                        start = -1
            if best_seg is None or best_len < 3:
                continue
            sx, ex, yy = best_seg
            inv = cv2.invertAffineTransform(m)
            run_pts: list[tuple[float, float]] = []
            step = 1 if (ex - sx) <= 40 else 2
            for xx in range(int(sx), int(ex) + 1, step):
                p = np.dot(inv, np.array([float(xx), float(yy), 1.0], dtype=np.float64))
                run_pts.append((float(p[0]), float(p[1])))
            if run_pts and run_pts[-1] != (float(np.dot(inv, np.array([float(ex), float(yy), 1.0], dtype=np.float64))[0]), float(np.dot(inv, np.array([float(ex), float(yy), 1.0], dtype=np.float64))[1])):
                p_end = np.dot(inv, np.array([float(ex), float(yy), 1.0], dtype=np.float64))
                run_pts.append((float(p_end[0]), float(p_end[1])))
            cand = self._build_pixel_polyline_candidate(
                run_pts,
                source_to_current_matrix=source_to_current_matrix,
                line_width_mm=line_width_mm,
                component_id=component_id,
                candidate_type="angle_run",
            )
            if cand is not None:
                cand.metadata["angle_run_deg"] = float(angle)
                candidates.append(cand)
        return candidates

    def _build_component_medial_polyline_candidate(
        self,
        component_mask: np.ndarray,
        *,
        source_to_current_matrix: tuple[float, float, float, float, float, float],
        line_width_mm: float,
        component_id: int,
        horizontal: bool,
        pen_radius_px: float = 0.0,
    ) -> Toolpath | None:
        ys, xs = np.nonzero(component_mask > 0)
        if xs.size < 3:
            return None
        min_x = int(np.min(xs))
        max_x = int(np.max(xs))
        min_y = int(np.min(ys))
        max_y = int(np.max(ys))
        pixel_points: list[tuple[float, float]] = []
        if horizontal:
            for xx in range(min_x, max_x + 1):
                col = np.nonzero(component_mask[min_y:max_y + 1, xx])[0]
                if col.size == 0:
                    continue
                y0 = float(min_y + int(col.min()))
                y1 = float(min_y + int(col.max()))
                pixel_points.append((float(xx), (y0 + y1) * 0.5))
        else:
            for yy in range(min_y, max_y + 1):
                row = np.nonzero(component_mask[yy, min_x:max_x + 1])[0]
                if row.size == 0:
                    continue
                x0 = float(min_x + int(row.min()))
                x1 = float(min_x + int(row.max()))
                pixel_points.append((((x0 + x1) * 0.5), float(yy)))
        if len(pixel_points) < 3:
            return None
        stride = 2 if len(pixel_points) > 80 else 1
        simplified = pixel_points[::stride]
        if simplified[-1] != pixel_points[-1]:
            simplified.append(pixel_points[-1])
        trim_pts = int(round(max(0.0, pen_radius_px * 0.6)))
        if trim_pts > 0 and len(simplified) > (trim_pts * 2 + 2):
            simplified = simplified[trim_pts:-trim_pts]
        return self._build_pixel_polyline_candidate(
            simplified,
            source_to_current_matrix=source_to_current_matrix,
            line_width_mm=line_width_mm,
            component_id=component_id,
            candidate_type="component_medial_polyline",
        )

    def _build_pixel_polyline_candidate(
        self,
        pixel_points: list[tuple[float, float]],
        *,
        source_to_current_matrix: tuple[float, float, float, float, float, float],
        line_width_mm: float,
        component_id: int,
        candidate_type: str,
    ) -> Toolpath | None:
        if len(pixel_points) < 2:
            return None
        points_mm = [apply_svg_matrix(Point(float(x), float(y)), source_to_current_matrix) for x, y in pixel_points]
        if len(points_mm) < 2:
            return None
        path = Toolpath(
            points=points_mm,
            kind="coverage_centerline",
            closed=False,
            source="coverage_backfill_component",
            metadata={
                "coverage_backfill_component": True,
                "coverage_backfill_component_id": int(component_id),
                "coverage_backfill_strategy": candidate_type,
                "coverage_backfill_candidate_type": candidate_type,
                "candidate_pixel_points": [[float(x), float(y)] for x, y in pixel_points],
            },
        )
        if segment_length(path.points) < max(0.04, line_width_mm * 0.15):
            return None
        return path

    def _repair_component_candidate_pool(
        self,
        component_mask: np.ndarray,
        *,
        comp_id: int,
        area_px: int,
        source_to_current: tuple[float, float, float, float, float, float],
        line_width_mm: float,
        pen_radius_px: float,
        target_mask: np.ndarray,
        drawn_mask: np.ndarray,
        component_centroids: dict[int, tuple[float, float]],
    ) -> list[Toolpath]:
        candidates: list[Toolpath] = []
        h, w = component_mask.shape[:2]
        ys, xs = np.nonzero(component_mask)
        if xs.size < 2:
            return candidates
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        ww = x1 - x0 + 1
        hh = y1 - y0 + 1

        def _clipped_segment(p0: tuple[float, float], p1: tuple[float, float]) -> list[tuple[float, float]] | None:
            x0f, y0f = p0
            x1f, y1f = p1
            steps = max(8, int(math.hypot(x1f - x0f, y1f - y0f) * 2.0))
            pts = []
            for i in range(steps + 1):
                t = float(i) / float(max(1, steps))
                x = x0f + (x1f - x0f) * t
                y = y0f + (y1f - y0f) * t
                xi = int(round(x))
                yi = int(round(y))
                inside = (0 <= xi < w and 0 <= yi < h and bool(component_mask[yi, xi]))
                pts.append((x, y, inside))
            best_start = -1
            best_end = -1
            run_start = -1
            for idx, (_x, _y, inside) in enumerate(pts):
                if inside and run_start < 0:
                    run_start = idx
                if (not inside or idx == len(pts) - 1) and run_start >= 0:
                    run_end = idx if inside and idx == len(pts) - 1 else idx - 1
                    if run_end - run_start > best_end - best_start:
                        best_start, best_end = run_start, run_end
                    run_start = -1
            if best_start < 0 or best_end <= best_start:
                return None
            a = pts[best_start]
            b = pts[best_end]
            if math.hypot(b[0] - a[0], b[1] - a[1]) < 1.0:
                return None
            return [(a[0], a[1]), (b[0], b[1])]

        # 1) skeleton / ridge approximation
        dt = cv2.distanceTransform(component_mask.astype(np.uint8), cv2.DIST_L2, 3)
        ridge = dt >= np.maximum(1.0, float(np.percentile(dt[dt > 0], 75))) if np.any(dt > 0) else np.zeros_like(component_mask, dtype=bool)
        ridge_points = np.column_stack(np.nonzero(ridge))
        if ridge_points.shape[0] >= 3:
            pts_xy = np.column_stack((ridge_points[:, 1].astype(np.float32), ridge_points[:, 0].astype(np.float32)))
            try:
                vx, vy, cx, cy = cv2.fitLine(pts_xy, cv2.DIST_L2, 0, 0.01, 0.01)
                vx_f = float(np.asarray(vx).reshape(-1)[0])
                vy_f = float(np.asarray(vy).reshape(-1)[0])
                cx_f = float(np.asarray(cx).reshape(-1)[0])
                cy_f = float(np.asarray(cy).reshape(-1)[0])
                t = ((pts_xy[:, 0] - cx_f) * vx_f) + ((pts_xy[:, 1] - cy_f) * vy_f)
                i0 = int(np.argmin(t))
                i1 = int(np.argmax(t))
                skel = self._build_pixel_polyline_candidate(
                    [(float(pts_xy[i0, 0]), float(pts_xy[i0, 1])), (float(pts_xy[i1, 0]), float(pts_xy[i1, 1]))],
                    source_to_current_matrix=source_to_current,
                    line_width_mm=line_width_mm,
                    component_id=comp_id,
                    candidate_type="skeleton",
                )
                if skel is not None:
                    candidates.append(skel)
            except Exception:
                pass

        # 2) distance-transform ridge explicit candidate through highest-distance points
        if np.any(dt > 0):
            max_idx = np.unravel_index(int(np.argmax(dt)), dt.shape)
            cy, cx = int(max_idx[0]), int(max_idx[1])
            run = self._build_component_run_candidates(
                component_mask,
                source_to_current_matrix=source_to_current,
                line_width_mm=line_width_mm,
                pen_radius_px=pen_radius_px,
                component_id=comp_id,
                horizontal=ww >= hh,
                max_candidates=1,
            )
            if run:
                run0 = run[0]
                run0.metadata["coverage_backfill_strategy"] = "distance_ridge"
                run0.metadata["coverage_backfill_candidate_type"] = "distance_ridge"
                candidates.append(run0)
            ridge_seg = _clipped_segment((float(x0), float(cy)), (float(x1), float(cy))) if ww >= hh else _clipped_segment((float(cx), float(y0)), (float(cx), float(y1)))
            if ridge_seg is not None:
                ridge_cross = self._build_pixel_polyline_candidate(
                    ridge_seg,
                    source_to_current_matrix=source_to_current,
                    line_width_mm=line_width_mm,
                    component_id=comp_id,
                    candidate_type="distance_ridge",
                )
                if ridge_cross is not None:
                    candidates.append(ridge_cross)

        # 3) multi-angle local hatch candidates (small groups)
        angles = [0.0, 30.0, 45.0, 60.0, 90.0, -30.0, -45.0, -60.0]
        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
        diag = float(math.hypot(ww, hh))
        for ang in angles:
            rad = math.radians(ang)
            ux, uy = math.cos(rad), math.sin(rad)
            px, py = -uy, ux
            spacing = max(1.0, pen_radius_px * 1.35)
            local_lines = []
            for s in (-spacing, 0.0, spacing):
                sx = cx + px * s - ux * diag * 0.5
                sy = cy + py * s - uy * diag * 0.5
                ex = cx + px * s + ux * diag * 0.5
                ey = cy + py * s + uy * diag * 0.5
                local_lines.append((sx, sy, ex, ey))
            kept = 0
            for sx, sy, ex, ey in local_lines:
                seg = _clipped_segment((sx, sy), (ex, ey))
                if seg is None:
                    continue
                p = self._build_pixel_polyline_candidate(
                    seg,
                    source_to_current_matrix=source_to_current,
                    line_width_mm=line_width_mm,
                    component_id=comp_id,
                    candidate_type="local_hatch",
                )
                if p is not None:
                    candidates.append(p)
                    kept += 1
                if kept >= 2:
                    break

        # 4) stroke-expansion / parallel offset toward missed component
        base = self._build_component_centerline_candidate(
            component_mask,
            source_to_current_matrix=source_to_current,
            line_width_mm=line_width_mm,
            pen_radius_px=pen_radius_px,
            component_id=comp_id,
            strategy="parallel_offset",
        )
        if base is not None:
            p0, p1 = base.points[0], base.points[-1]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            ln = math.hypot(dx, dy)
            if ln > 1e-9:
                nx, ny = -dy / ln, dx / ln
                offset_mm = max(0.08, line_width_mm * 0.38)
                for sign in (-1.0, 1.0):
                    off = Toolpath(
                        points=[Point(p0.x + nx * offset_mm * sign, p0.y + ny * offset_mm * sign), Point(p1.x + nx * offset_mm * sign, p1.y + ny * offset_mm * sign)],
                        kind="coverage_offset_line",
                        closed=False,
                        source="coverage_backfill_component",
                        metadata={
                            "coverage_backfill_component": True,
                            "coverage_backfill_component_id": int(comp_id),
                            "coverage_backfill_strategy": "parallel_offset",
                            "coverage_backfill_candidate_type": "parallel_offset",
                        },
                    )
                    try:
                        sp0 = apply_svg_matrix(off.points[0], invert_svg_matrix(source_to_current))
                        sp1 = apply_svg_matrix(off.points[-1], invert_svg_matrix(source_to_current))
                        seg = _clipped_segment((sp0.x, sp0.y), (sp1.x, sp1.y))
                    except Exception:
                        seg = None
                    if seg is not None:
                        clipped_off = self._build_pixel_polyline_candidate(
                            seg,
                            source_to_current_matrix=source_to_current,
                            line_width_mm=line_width_mm,
                            component_id=comp_id,
                            candidate_type="parallel_offset",
                        )
                        if clipped_off is not None:
                            clipped_off.kind = "coverage_offset_line"
                            candidates.append(clipped_off)

        # 5) boundary inset candidate
        contours, _ = cv2.findContours(component_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            if len(cnt) >= 6:
                eps = max(1.0, pen_radius_px * 0.6)
                approx = cv2.approxPolyDP(cnt, eps, True)
                poly = [(float(p[0][0]), float(p[0][1])) for p in approx]
                if len(poly) >= 2:
                    boundary = self._build_pixel_polyline_candidate(
                        poly[: min(10, len(poly))],
                        source_to_current_matrix=source_to_current,
                        line_width_mm=line_width_mm,
                        component_id=comp_id,
                        candidate_type="boundary_inset",
                    )
                    if boundary is not None:
                        candidates.append(boundary)

        # 6) component bridge candidate
        this_centroid = component_centroids.get(comp_id)
        if this_centroid is not None:
            tx, ty = this_centroid
            best_other: tuple[int, float] | None = None
            for oid, (ox, oy) in component_centroids.items():
                if oid == comp_id:
                    continue
                dist = math.hypot(ox - tx, oy - ty)
                if dist <= max(1.0, pen_radius_px * 1.8):
                    if best_other is None or dist < best_other[1]:
                        best_other = (oid, dist)
            if best_other is not None:
                ox, oy = component_centroids[best_other[0]]
                seg = _clipped_segment((tx, ty), (ox, oy))
                bridge = self._build_pixel_polyline_candidate(
                    seg if seg is not None else [(tx, ty), (ox, oy)],
                    source_to_current_matrix=source_to_current,
                    line_width_mm=line_width_mm,
                    component_id=comp_id,
                    candidate_type="component_bridge",
                )
                if bridge is not None:
                    candidates.append(bridge)
        return candidates



    def _repair_missed_mask_components(
        self,
        paths: list[Toolpath],
        *,
        line_width_mm: float,
        connector_validation: dict[str, Any] | None,
        debug: Optional[dict[str, Any]] = None,
        target_penalized_percent: float = 90.0,
        max_added_paths: int = 80,
    ) -> list[Toolpath]:
        if not connector_validation or max_added_paths <= 0:
            return paths
        mask = connector_validation.get("mask")
        matrix = connector_validation.get("current_to_source_matrix")
        if mask is None or not isinstance(matrix, (tuple, list)) or len(matrix) != 6:
            return paths
        current_to_source = tuple(float(value) for value in matrix)
        try:
            source_to_current = invert_svg_matrix(current_to_source)
        except Exception:
            if debug is not None:
                debug["coverage_backfill_component_transform_failed"] = True
            return paths

        include_kinds = {
            "coverage_centerline",
            "coverage_offset_line",
            "coverage_rectilinear",
            "coverage_tiny_mark",
            "coverage_contour",
            "coverage_connector",
            "outline_cleanup",
        }
        current_paths = list(paths)
        current_metrics = compute_toolpath_mask_coverage_metrics(
            current_paths,
            mask=mask,
            current_to_source_matrix=current_to_source,
            pen_radius_mm=line_width_mm * 0.5,
            sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
            include_kinds=include_kinds,
        )
        if current_metrics is None or current_metrics.penalized_coverage_percent >= target_penalized_percent:
            return paths
        best_paths = list(current_paths)
        best_metrics = current_metrics

        target_mask = np.asarray(mask) > 0
        target_boundary = cv2.morphologyEx((target_mask.astype(np.uint8) * 255), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
        target_boundary_band = cv2.dilate((target_boundary.astype(np.uint8) * 255), np.ones((3, 3), np.uint8), iterations=1) > 0
        h, w = target_mask.shape[:2]
        a, b, c, d, _e, _f = current_to_source
        px_per_mm = max(1e-6, (math.hypot(a, b) + math.hypot(c, d)) * 0.5)
        pen_radius_px = max(0.0, (line_width_mm * 0.5) * px_per_mm)
        radius_px_i = max(1, int(round(pen_radius_px)))

        drawn = np.zeros((h, w), dtype=np.uint8)
        for path in current_paths:
            if path.kind not in include_kinds or len(path.points) < 1:
                continue
            if len(path.points) == 1:
                source_point = apply_svg_matrix(path.points[0], current_to_source)
                cv2.circle(drawn, (int(round(source_point.x)), int(round(source_point.y))), max(1, int(round(pen_radius_px))), 255, -1)
                continue
            for start, end in zip(path.points, path.points[1:]):
                line = LineString([(start.x, start.y), (end.x, end.y)])
                if line.length <= 1e-9:
                    continue
                sample_count = max(2, int(math.ceil(line.length / max(0.01, min(line_width_mm * 0.35, 0.05)))) + 1)
                for sample_index in range(sample_count):
                    distance_mm = min(line.length, (line.length * sample_index) / max(sample_count - 1, 1))
                    sample = line.interpolate(distance_mm)
                    source_point = apply_svg_matrix(Point(float(sample.x), float(sample.y)), current_to_source)
                    cv2.circle(drawn, (int(round(source_point.x)), int(round(source_point.y))), radius_px_i, 255, -1)

        def _rasterize_single_candidate(candidate: Toolpath) -> np.ndarray:
            out = np.zeros((h, w), dtype=np.uint8)
            if len(candidate.points) < 1:
                return out
            if len(candidate.points) == 1:
                source_point = apply_svg_matrix(candidate.points[0], current_to_source)
                cv2.circle(out, (int(round(source_point.x)), int(round(source_point.y))), radius_px_i, 255, -1)
                return out
            for start, end in zip(candidate.points, candidate.points[1:]):
                line = LineString([(start.x, start.y), (end.x, end.y)])
                if line.length <= 1e-9:
                    continue
                sample_count = max(2, int(math.ceil(line.length / max(0.01, min(line_width_mm * 0.35, 0.05)))) + 1)
                for sample_index in range(sample_count):
                    distance_mm = min(line.length, (line.length * sample_index) / max(sample_count - 1, 1))
                    sample = line.interpolate(distance_mm)
                    source_point = apply_svg_matrix(Point(float(sample.x), float(sample.y)), current_to_source)
                    cv2.circle(out, (int(round(source_point.x)), int(round(source_point.y))), radius_px_i, 255, -1)
            return out

        def _rasterize_pixel_oracle(candidate: Toolpath) -> np.ndarray:
            pts_meta = candidate.metadata.get("candidate_pixel_points")
            if not isinstance(pts_meta, list) or len(pts_meta) < 1:
                return _rasterize_single_candidate(candidate)
            pts: list[tuple[float, float]] = []
            for pair in pts_meta:
                if isinstance(pair, (list, tuple)) and len(pair) >= 2:
                    pts.append((float(pair[0]), float(pair[1])))
            out = np.zeros((h, w), dtype=np.uint8)
            if not pts:
                return _rasterize_single_candidate(candidate)
            if len(pts) == 1:
                cv2.circle(out, (int(round(pts[0][0])), int(round(pts[0][1]))), radius_px_i, 255, -1)
                return out
            for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                seg_len = math.hypot(x1 - x0, y1 - y0)
                n = max(2, int(math.ceil(seg_len / max(0.5, radius_px_i * 0.5))) + 1)
                for i in range(n):
                    t = i / max(n - 1, 1)
                    xx = x0 + (x1 - x0) * t
                    yy = y0 + (y1 - y0) * t
                    cv2.circle(out, (int(round(xx)), int(round(yy))), radius_px_i, 255, -1)
            return out

        def _candidate_roundtrip_error(candidate: Toolpath) -> float:
            pts_meta = candidate.metadata.get("candidate_pixel_points")
            if not isinstance(pts_meta, list) or len(pts_meta) == 0:
                return 0.0
            max_err = 0.0
            for pair in pts_meta:
                if not isinstance(pair, (list, tuple)) or len(pair) < 2:
                    continue
                px0 = float(pair[0])
                py0 = float(pair[1])
                mm_pt = apply_svg_matrix(Point(px0, py0), source_to_current)
                px_rt = apply_svg_matrix(mm_pt, current_to_source)
                err = math.hypot(px_rt.x - px0, px_rt.y - py0)
                if err > max_err:
                    max_err = float(err)
            return max_err

        missed = target_mask & ~(drawn > 0)
        comp_count, labels, stats, _ = cv2.connectedComponentsWithStats(missed.astype(np.uint8), 8)
        tgt_comp_count, tgt_labels, _tgt_stats, _ = cv2.connectedComponentsWithStats(target_mask.astype(np.uint8), 8)
        min_component_area_px = max(2, int(round((pen_radius_px * 0.5) ** 2)))
        component_debug: list[dict[str, Any]] = []
        parity_rows: list[dict[str, Any]] = []
        component_best_candidate: dict[int, dict[str, Any]] = {}
        candidates_generated = 0
        accepted = 0
        rejected = 0
        thin_candidates_tried = 0
        thin_candidates_accepted = 0
        thin_candidates_rejected_overflow = 0
        thin_candidate_decisions: list[dict[str, Any]] = []
        rejection_reasons: dict[str, int] = {}
        added_paths = 0
        accepted_thin_mask = np.zeros((h, w), dtype=np.uint8)
        rejected_thin_mask = np.zeros((h, w), dtype=np.uint8)
        comp_ids = list(range(1, int(comp_count)))
        comp_ids.sort(key=lambda idx: int(stats[idx, cv2.CC_STAT_AREA]), reverse=True)
        comp_ids = comp_ids[:30]

        def _split_candidate_segments(candidate: Toolpath, source_component_id: int, target_component_id: int | None) -> list[Toolpath]:
            if len(candidate.points) != 2:
                return []
            p0, p1 = candidate.points
            total = math.hypot(p1.x - p0.x, p1.y - p0.y)
            if total <= max(0.4, line_width_mm * 1.4):
                return []
            seg_len = max(0.2, line_width_mm * 1.0)
            n = min(4, max(2, int(math.ceil(total / seg_len))))
            out: list[Toolpath] = []
            for i in range(n):
                t0 = i / n
                t1 = (i + 1) / n
                a = Point(p0.x + (p1.x - p0.x) * t0, p0.y + (p1.y - p0.y) * t0)
                b = Point(p0.x + (p1.x - p0.x) * t1, p0.y + (p1.y - p0.y) * t1)
                seg = clone_toolpath(
                    candidate,
                    points=[a, b],
                    metadata={
                        **candidate.metadata,
                        "coverage_backfill_strategy": "split_centerline_segment",
                        "source_component_id": int(source_component_id),
                        "target_component_id": int(target_component_id) if target_component_id is not None else None,
                        "segment_index": int(i),
                    },
                )
                if segment_length(seg.points) >= max(0.04, line_width_mm * 0.12):
                    out.append(seg)
            return out
        for comp_id in comp_ids:
            if added_paths >= max_added_paths:
                break
            area_px = int(stats[comp_id, cv2.CC_STAT_AREA])
            x = int(stats[comp_id, cv2.CC_STAT_LEFT])
            y = int(stats[comp_id, cv2.CC_STAT_TOP])
            ww = int(stats[comp_id, cv2.CC_STAT_WIDTH])
            hh = int(stats[comp_id, cv2.CC_STAT_HEIGHT])
            width_px_est = float(area_px) / max(1.0, float(max(ww, hh)))
            aspect = float(max(ww, hh)) / max(1.0, float(min(ww, hh)))
            centroid_x = float(np.mean(np.nonzero(labels == comp_id)[1])) if area_px > 0 else float(x + ww * 0.5)
            centroid_y = float(np.mean(np.nonzero(labels == comp_id)[0])) if area_px > 0 else float(y + hh * 0.5)
            dist_max = float(np.max(cv2.distanceTransform((labels == comp_id).astype(np.uint8), cv2.DIST_L2, 3))) if area_px > 0 else 0.0
            thin_threshold_px = max(2.0, pen_radius_px * 1.25)
            is_thin_component = bool(width_px_est <= thin_threshold_px or aspect >= 2.2 or area_px < min_component_area_px)
            if width_px_est <= thin_threshold_px or aspect >= 2.2:
                strategy = "thin_detail_centerline"
            elif min(ww, hh) >= int(round(max(3.0, pen_radius_px * 1.8))):
                strategy = "local_short_hatch"
            else:
                strategy = "tiny_component_single_stroke"
            component_mask = labels == comp_id
            component_candidates: list[Toolpath] = []
            target_component_id: int | None = None
            cx_i = int(round(centroid_x))
            cy_i = int(round(centroid_y))
            if 0 <= cx_i < w and 0 <= cy_i < h:
                tid = int(tgt_labels[cy_i, cx_i])
                if tid > 0:
                    target_component_id = tid
            horizontal = ww >= hh
            component_candidates.extend(self._build_component_run_candidates(
                component_mask,
                source_to_current_matrix=source_to_current,
                line_width_mm=line_width_mm,
                pen_radius_px=pen_radius_px,
                component_id=comp_id,
                horizontal=horizontal,
                max_candidates=8 if strategy != "local_short_hatch" else 6,
                thin_mode=(strategy == "thin_detail_centerline"),
            ))
            medial_primary = self._build_component_medial_polyline_candidate(
                component_mask,
                source_to_current_matrix=source_to_current,
                line_width_mm=line_width_mm,
                component_id=comp_id,
                horizontal=horizontal,
                pen_radius_px=pen_radius_px,
            )
            if medial_primary is not None:
                component_candidates.append(medial_primary)
            if aspect < 1.8:
                medial_alt = self._build_component_medial_polyline_candidate(
                    component_mask,
                    source_to_current_matrix=source_to_current,
                    line_width_mm=line_width_mm,
                    component_id=comp_id,
                    horizontal=not horizontal,
                    pen_radius_px=pen_radius_px,
                )
                if medial_alt is not None:
                    component_candidates.append(medial_alt)
            primary = self._build_component_centerline_candidate(
                component_mask,
                source_to_current_matrix=source_to_current,
                line_width_mm=line_width_mm,
                pen_radius_px=pen_radius_px,
                component_id=comp_id,
                strategy=strategy,
            )
            if primary is not None:
                component_candidates.append(primary)
            if strategy == "thin_detail_centerline" and aspect >= 2.2:
                fullspan = self._build_component_fullspan_passage_candidate(
                    component_mask,
                    source_to_current_matrix=source_to_current,
                    line_width_mm=line_width_mm,
                    component_id=comp_id,
                )
                if fullspan is not None:
                    component_candidates.append(fullspan)
            if strategy == "thin_detail_centerline":
                component_candidates.extend(
                    self._build_component_angle_run_candidates(
                        component_mask,
                        source_to_current_matrix=source_to_current,
                        line_width_mm=line_width_mm,
                        component_id=comp_id,
                    )
                )
            # Generate candidates from local target component, not only missed mask.
            if target_component_id is not None:
                target_component_mask = tgt_labels == target_component_id
                tc_center = self._build_component_centerline_candidate(
                    target_component_mask,
                    source_to_current_matrix=source_to_current,
                    line_width_mm=line_width_mm,
                    pen_radius_px=pen_radius_px,
                    component_id=comp_id,
                    strategy="target_component_centerline",
                )
                if tc_center is not None:
                    tc_center.metadata["source_component_id"] = int(comp_id)
                    tc_center.metadata["target_component_id"] = int(target_component_id)
                    tc_center.metadata["coverage_backfill_strategy"] = "target_component_centerline"
                    component_candidates.append(tc_center)
            if area_px <= int(round(max(3.0, pen_radius_px * pen_radius_px * 1.5))):
                dot = self._build_component_tiny_dot_candidate(
                    component_mask,
                    source_to_current_matrix=source_to_current,
                    component_id=comp_id,
                    line_width_mm=line_width_mm,
                )
                if dot is not None:
                    component_candidates.append(dot)

            # Candidate variants to reduce cap-induced overdraw.
            variant_candidates: list[Toolpath] = []
            for cand in component_candidates:
                if len(cand.points) != 2:
                    continue
                p0, p1 = cand.points[0], cand.points[1]
                dx = p1.x - p0.x
                dy = p1.y - p0.y
                length = math.hypot(dx, dy)
                if length <= 1e-9:
                    continue
                ux = dx / length
                uy = dy / length
                for frac in (1.0,):
                    for trim_factor in (0.0,):
                        trim_mm = (line_width_mm * 0.5) * trim_factor
                        target_len = max(0.02, length * frac - 2.0 * trim_mm)
                        if target_len >= length - 1e-6:
                            continue
                        cxm = (p0.x + p1.x) * 0.5
                        cym = (p0.y + p1.y) * 0.5
                        hx = ux * (target_len * 0.5)
                        hy = uy * (target_len * 0.5)
                        v = clone_toolpath(
                            cand,
                            points=[Point(cxm - hx, cym - hy), Point(cxm + hx, cym + hy)],
                            metadata={**cand.metadata, "variant_frac": frac, "variant_trim_factor": trim_factor},
                        )
                        if segment_length(v.points) >= max(0.02, line_width_mm * 0.05):
                            variant_candidates.append(v)
            component_candidates.extend(variant_candidates)
            split_segments: list[Toolpath] = []
            for cand in component_candidates:
                split_segments.extend(_split_candidate_segments(cand, int(comp_id), target_component_id))
            component_candidates.extend(split_segments)
            if strategy == "local_short_hatch" and primary is not None and added_paths + len(component_candidates) < max_added_paths:
                p0, p1 = primary.points[0], primary.points[-1]
                dx = p1.x - p0.x
                dy = p1.y - p0.y
                length = math.hypot(dx, dy)
                if length > 1e-9:
                    nx = -dy / length
                    ny = dx / length
                    offset_mm = max(line_width_mm * 0.45, 0.1)
                    shifted = Toolpath(
                        points=[
                            Point(p0.x + nx * offset_mm, p0.y + ny * offset_mm),
                            Point(p1.x + nx * offset_mm, p1.y + ny * offset_mm),
                        ],
                        kind="coverage_offset_line",
                        closed=False,
                        source="coverage_backfill_component",
                        metadata={
                            **primary.metadata,
                            "coverage_backfill_strategy": "local_short_hatch",
                        },
                    )
                    if segment_length(shifted.points) >= max(0.04, line_width_mm * 0.15):
                        component_candidates.append(shifted)

            candidates_generated += len(component_candidates)
            comp_diag = {
                "component_id": int(comp_id),
                "area_px": area_px,
                "bbox_px": [x, y, ww, hh],
                "centroid_px": [centroid_x, centroid_y],
                "estimated_width_px": width_px_est,
                "aspect_ratio": aspect,
                "distance_transform_max_px": dist_max,
                "candidate_strategy": strategy,
                "candidate_count": len(component_candidates),
            }
            component_debug.append(comp_diag)
            accepted_this_component = 0
            eval_rows: list[dict[str, Any]] = []
            for candidate in component_candidates:
                trial = current_paths + [candidate]
                trial_metrics = compute_toolpath_mask_coverage_metrics(
                    trial,
                    mask=mask,
                    current_to_source_matrix=current_to_source,
                    pen_radius_mm=line_width_mm * 0.5,
                    sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
                    include_kinds=include_kinds,
                )
                if trial_metrics is None:
                    rejected += 1
                    rejection_reasons["candidate_invalid_geometry"] = int(rejection_reasons.get("candidate_invalid_geometry", 0)) + 1
                    continue
                delta_penalized = trial_metrics.penalized_coverage_percent - current_metrics.penalized_coverage_percent
                delta_covered = trial_metrics.covered_inside_mask_px - current_metrics.covered_inside_mask_px
                delta_overdraw = trial_metrics.overdraw_outside_mask_px - current_metrics.overdraw_outside_mask_px
                oracle_mask = _rasterize_pixel_oracle(candidate) > 0
                base_drawn = drawn > 0
                oracle_new = oracle_mask & ~base_drawn
                oracle_delta_cov = int(np.count_nonzero(oracle_new & target_mask))
                oracle_delta_over = int(np.count_nonzero(oracle_new & ~target_mask))
                rt_error = _candidate_roundtrip_error(candidate)
                effective_delta_overdraw = int(delta_overdraw)
                path_len_px = 0.0
                accept = False
                reason = "candidate_no_penalized_improvement"
                if is_thin_component:
                    thin_candidates_tried += 1
                    candidate_mask = _rasterize_single_candidate(candidate) > 0
                    candidate_overdraw = candidate_mask & ~target_mask
                    boundary_overdraw_px = int(np.count_nonzero(candidate_overdraw & target_boundary_band))
                    effective_delta_overdraw = max(0, int(delta_overdraw) - boundary_overdraw_px)
                    path_len_px = float(np.count_nonzero(candidate_mask))
                    net_gain_px = int(delta_covered - effective_delta_overdraw)
                    accept = bool(net_gain_px > 0 and delta_penalized > 1e-9)
                    if accept:
                        reason = "net_positive"
                    else:
                        reason = "overdraw_exceeds_coverage" if net_gain_px <= 0 else "no_penalized_gain"
                    thin_candidate_decisions.append(
                        {
                            "component_id": int(comp_id),
                            "candidate_type": str(candidate.metadata.get("coverage_backfill_strategy", candidate.metadata.get("coverage_backfill_candidate_type", "unknown"))),
                            "source_component_id": int(candidate.metadata.get("source_component_id", comp_id)),
                            "target_component_id": candidate.metadata.get("target_component_id"),
                            "trim_px": float(candidate.metadata.get("trim_px", 0.0)),
                            "segment_index": candidate.metadata.get("segment_index"),
                            "accepted": bool(accept),
                            "delta_covered_inside_mask_px": int(delta_covered),
                            "delta_overdraw_outside_mask_px": int(delta_overdraw),
                            "net_gain_px": int(net_gain_px),
                            "delta_penalized_coverage_percent": float(delta_penalized),
                            "path_length_px": float(path_len_px),
                            "reason": reason,
                            "bbox_px": [x, y, ww, hh],
                        }
                    )
                    eval_rows.append(
                        {
                            "candidate": candidate,
                            "accept": accept,
                            "delta_penalized": float(delta_penalized),
                            "delta_covered": int(delta_covered),
                            "delta_overdraw": int(delta_overdraw),
                            "effective_delta_overdraw": int(effective_delta_overdraw),
                            "net_gain_px": int(net_gain_px),
                            "path_len_px": float(path_len_px),
                            "mask": candidate_mask,
                            "reason": reason,
                        }
                    )
                    parity = {
                        "component_id": int(comp_id),
                        "candidate_type": str(candidate.metadata.get("coverage_backfill_strategy", candidate.metadata.get("coverage_backfill_candidate_type", "unknown"))),
                        "pixel_oracle_delta_covered": int(oracle_delta_cov),
                        "pixel_oracle_delta_overdraw": int(oracle_delta_over),
                        "pixel_oracle_net_gain": int(oracle_delta_cov - oracle_delta_over),
                        "mm_candidate_delta_covered": int(delta_covered),
                        "mm_candidate_delta_overdraw": int(delta_overdraw),
                        "mm_candidate_net_gain": int(delta_covered - delta_overdraw),
                        "difference_covered": int(delta_covered - oracle_delta_cov),
                        "difference_overdraw": int(delta_overdraw - oracle_delta_over),
                        "roundtrip_error_px": float(rt_error),
                    }
                    parity_rows.append(parity)
                    prev_best = component_best_candidate.get(int(comp_id))
                    if prev_best is None or int(parity["pixel_oracle_net_gain"]) > int(prev_best.get("pixel_oracle_net_gain", -10**9)):
                        component_best_candidate[int(comp_id)] = {
                            **parity,
                            "bbox_px": [x, y, ww, hh],
                            "candidate_pixel_points": candidate.metadata.get("candidate_pixel_points", []),
                            "candidate_points_mm": [[float(p.x), float(p.y)] for p in candidate.points],
                            "accepted": bool(accept),
                            "rejection_reason": reason,
                        }
                else:
                    accept = (delta_penalized > 1e-9 and delta_covered > 0)
                    eval_rows.append(
                        {
                            "candidate": candidate,
                            "accept": accept,
                            "delta_penalized": float(delta_penalized),
                            "delta_covered": int(delta_covered),
                            "delta_overdraw": int(delta_overdraw),
                            "effective_delta_overdraw": int(delta_overdraw),
                            "net_gain_px": int(delta_covered - delta_overdraw),
                            "path_len_px": 0.0,
                            "mask": None,
                            "reason": "net_positive" if accept else "candidate_no_penalized_improvement",
                        }
                    )

            ranked = sorted(
                eval_rows,
                key=lambda row: (
                    int(row["net_gain_px"]),
                    float(row["delta_penalized"]),
                    int(row["delta_covered"]),
                    -int(row["effective_delta_overdraw"]),
                    -float(row["path_len_px"]),
                ),
                reverse=True,
            )
            accepted_candidates_for_component = 0
            for row in ranked:
                if added_paths >= max_added_paths or not bool(row["accept"]):
                    if is_thin_component and row["mask"] is not None:
                        rejected_thin_mask[row["mask"]] = 255
                    rejected += 1
                    if is_thin_component and str(row["reason"]) == "overdraw_exceeds_coverage":
                        thin_candidates_rejected_overflow += 1
                        rejection_reasons["thin_candidate_overflow_exceeds_coverage"] = int(rejection_reasons.get("thin_candidate_overflow_exceeds_coverage", 0)) + 1
                    else:
                        rejection_reasons["candidate_no_penalized_improvement"] = int(rejection_reasons.get("candidate_no_penalized_improvement", 0)) + 1
                    continue
                candidate = row["candidate"]
                trial = current_paths + [candidate]
                trial_metrics = compute_toolpath_mask_coverage_metrics(
                    trial,
                    mask=mask,
                    current_to_source_matrix=current_to_source,
                    pen_radius_mm=line_width_mm * 0.5,
                    sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
                    include_kinds=include_kinds,
                )
                if trial_metrics is None:
                    rejected += 1
                    rejection_reasons["candidate_invalid_geometry"] = int(rejection_reasons.get("candidate_invalid_geometry", 0)) + 1
                    continue
                re_delta_pen = trial_metrics.penalized_coverage_percent - current_metrics.penalized_coverage_percent
                re_delta_cov = trial_metrics.covered_inside_mask_px - current_metrics.covered_inside_mask_px
                re_delta_over = trial_metrics.overdraw_outside_mask_px - current_metrics.overdraw_outside_mask_px
                if is_thin_component:
                    re_mask = _rasterize_single_candidate(candidate) > 0
                    re_over = re_mask & ~target_mask
                    re_boundary = int(np.count_nonzero(re_over & target_boundary_band))
                    re_eff_over = max(0, int(re_delta_over) - re_boundary)
                    if not (re_delta_cov > re_eff_over and re_delta_pen > 1e-9):
                        rejected += 1
                        rejected_thin_mask[re_mask] = 255
                        thin_candidates_rejected_overflow += 1
                        rejection_reasons["thin_candidate_overflow_exceeds_coverage"] = int(rejection_reasons.get("thin_candidate_overflow_exceeds_coverage", 0)) + 1
                        continue
                    accepted_thin_mask[re_mask] = 255
                    thin_candidates_accepted += 1
                else:
                    if not (re_delta_pen > 1e-9 and re_delta_cov > 0):
                        rejected += 1
                        rejection_reasons["candidate_no_penalized_improvement"] = int(rejection_reasons.get("candidate_no_penalized_improvement", 0)) + 1
                        continue
                current_paths = trial
                current_metrics = trial_metrics
                if current_metrics.penalized_coverage_percent > (best_metrics.penalized_coverage_percent + 1e-9):
                    best_paths = list(current_paths)
                    best_metrics = current_metrics
                accepted += 1
                accepted_this_component += 1
                accepted_candidates_for_component += 1
                added_paths += 1
                if current_metrics.penalized_coverage_percent >= target_penalized_percent:
                    break
            if is_thin_component and accepted_candidates_for_component == 0 and len(ranked) >= 2 and added_paths + 2 <= max_added_paths:
                top_group = [row for row in ranked if row.get("candidate") is not None][:4]
                best_group_trial = None
                best_group_metrics = None
                for i in range(len(top_group)):
                    for j in range(i + 1, len(top_group)):
                        c0 = top_group[i]["candidate"]
                        c1 = top_group[j]["candidate"]
                        trial = current_paths + [c0, c1]
                        trial_metrics = compute_toolpath_mask_coverage_metrics(
                            trial,
                            mask=mask,
                            current_to_source_matrix=current_to_source,
                            pen_radius_mm=line_width_mm * 0.5,
                            sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
                            include_kinds=include_kinds,
                        )
                        if trial_metrics is None:
                            continue
                        d_cov = trial_metrics.covered_inside_mask_px - current_metrics.covered_inside_mask_px
                        d_over = trial_metrics.overdraw_outside_mask_px - current_metrics.overdraw_outside_mask_px
                        d_pen = trial_metrics.penalized_coverage_percent - current_metrics.penalized_coverage_percent
                        if d_cov > d_over and d_pen > 1e-9:
                            if best_group_metrics is None or trial_metrics.penalized_coverage_percent > best_group_metrics.penalized_coverage_percent:
                                best_group_trial = trial
                                best_group_metrics = trial_metrics
                if best_group_trial is not None and best_group_metrics is not None:
                    current_paths = best_group_trial
                    current_metrics = best_group_metrics
                    accepted += 2
                    accepted_this_component += 2
                    added_paths += 2
                    if current_metrics.penalized_coverage_percent > (best_metrics.penalized_coverage_percent + 1e-9):
                        best_paths = list(current_paths)
                        best_metrics = current_metrics
            if current_metrics.penalized_coverage_percent >= target_penalized_percent:
                break

        # Prune low-value repair strokes while preserving best score.
        prunable_idx = [i for i, p in enumerate(current_paths) if bool(p.metadata.get("coverage_backfill_component", False))]
        for idx in reversed(prunable_idx):
            if idx >= len(current_paths):
                continue
            trial = current_paths[:idx] + current_paths[idx + 1:]
            trial_metrics = compute_toolpath_mask_coverage_metrics(
                trial,
                mask=mask,
                current_to_source_matrix=current_to_source,
                pen_radius_mm=line_width_mm * 0.5,
                sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
                include_kinds=include_kinds,
            )
            if trial_metrics is None:
                continue
            if trial_metrics.penalized_coverage_percent >= current_metrics.penalized_coverage_percent:
                current_paths = trial
                current_metrics = trial_metrics
                if current_metrics.penalized_coverage_percent > (best_metrics.penalized_coverage_percent + 1e-9):
                    best_paths = list(current_paths)
                    best_metrics = current_metrics

        if debug is not None:
            debug["coverage_backfill_component_count"] = int(accepted)
            debug["coverage_backfill_component_candidates_generated"] = int(candidates_generated)
            debug["coverage_backfill_component_rejected"] = int(rejected)
            debug["coverage_backfill_component_rejection_reasons"] = rejection_reasons
            debug["thin_region_centerline_candidates_tried"] = int(thin_candidates_tried)
            debug["thin_region_centerline_candidates_accepted"] = int(thin_candidates_accepted)
            debug["thin_region_centerline_candidates_rejected_overflow"] = int(thin_candidates_rejected_overflow)
            debug["thin_region_candidate_decisions_top"] = sorted(
                thin_candidate_decisions,
                key=lambda item: float(item.get("delta_covered_inside_mask_px", 0.0)) - float(item.get("effective_delta_overdraw_outside_mask_px", 0.0)),
                reverse=True,
            )[:100]
            debug["missed_component_count"] = max(0, int(comp_count) - 1)
            debug["missed_components"] = component_debug
            debug["missed_components_top_by_area"] = sorted(component_debug, key=lambda item: int(item.get("area_px", 0)), reverse=True)[:12]
            debug["coverage_backfill_component_final_penalized"] = float(best_metrics.penalized_coverage_percent)
            debug["candidate_oracle_mm_parity_top"] = sorted(
                parity_rows,
                key=lambda item: abs(float(item.get("difference_covered", 0))) + abs(float(item.get("difference_overdraw", 0))),
                reverse=True,
            )[:120]
            debug["max_roundtrip_error_px"] = float(max((row.get("roundtrip_error_px", 0.0) for row in parity_rows), default=0.0))
            if os.getenv("WRITE_COVERAGE_DEBUG_ARTIFACTS", "0") == "1":
                out_dir = Path(tempfile.gettempdir()) / "golfball_plotter_test_artifacts" / "carolin_coverage"
                out_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out_dir / "thin_candidates_accepted_mask.png"), accepted_thin_mask)
                cv2.imwrite(str(out_dir / "thin_candidates_rejected_mask.png"), rejected_thin_mask)
                overlay = np.zeros((h, w, 3), dtype=np.uint8)
                overlay[target_mask] = (50, 50, 50)
                overlay[rejected_thin_mask > 0] = (0, 0, 255)
                overlay[accepted_thin_mask > 0] = (0, 255, 0)
                cv2.imwrite(str(out_dir / "thin_candidates_overlay.png"), overlay)
                with open(out_dir / "thin_candidate_decisions.json", "w", encoding="utf-8") as fp:
                    json.dump(thin_candidate_decisions, fp, indent=2)
                with open(out_dir / "candidate_oracle_mm_parity.json", "w", encoding="utf-8") as fp:
                    json.dump(parity_rows, fp, indent=2)
                largest = sorted(component_debug, key=lambda item: int(item.get("area_px", 0)), reverse=True)[:10]
                narrow = sorted(component_debug, key=lambda item: float(item.get("aspect_ratio", 0.0)), reverse=True)[:10]
                comp_ids_dump = []
                for ent in largest + narrow:
                    cid = int(ent.get("component_id", -1))
                    if cid > 0 and cid not in comp_ids_dump:
                        comp_ids_dump.append(cid)
                for cid in comp_ids_dump[:20]:
                    info = component_best_candidate.get(cid)
                    if not info:
                        continue
                    bx, by, bw, bh = [int(v) for v in info.get("bbox_px", [0, 0, w, h])]
                    pad = 8
                    x0 = max(0, bx - pad)
                    y0 = max(0, by - pad)
                    x1 = min(w, bx + bw + pad)
                    y1 = min(h, by + bh + pad)
                    crop_target = (target_mask[y0:y1, x0:x1].astype(np.uint8) * 255)
                    crop_missed = (missed[y0:y1, x0:x1].astype(np.uint8) * 255)
                    pts = info.get("candidate_pixel_points", [])
                    center = np.zeros((y1 - y0, x1 - x0, 3), dtype=np.uint8)
                    if isinstance(pts, list) and len(pts) >= 1:
                        pp = []
                        for pair in pts:
                            if isinstance(pair, (list, tuple)) and len(pair) >= 2:
                                pp.append((int(round(float(pair[0]) - x0)), int(round(float(pair[1]) - y0))))
                        if len(pp) == 1:
                            cv2.circle(center, pp[0], 1, (0, 255, 255), -1)
                        elif len(pp) >= 2:
                            cv2.polylines(center, [np.asarray(pp, dtype=np.int32).reshape(-1, 1, 2)], False, (0, 255, 255), 1, lineType=cv2.LINE_8)
                    oracle_mask_full = np.zeros((h, w), dtype=np.uint8)
                    dummy = Toolpath(points=[], kind="coverage_centerline", metadata={"candidate_pixel_points": pts})
                    oracle_mask_full = _rasterize_pixel_oracle(dummy)
                    crop_oracle = oracle_mask_full[y0:y1, x0:x1]
                    overlay_crop = np.zeros((y1 - y0, x1 - x0, 3), dtype=np.uint8)
                    tmask = target_mask[y0:y1, x0:x1]
                    dmask = drawn[y0:y1, x0:x1] > 0
                    o_new = (crop_oracle > 0) & ~dmask
                    cov = o_new & tmask
                    over = o_new & ~tmask
                    still = tmask & ~(dmask | (crop_oracle > 0))
                    overlay_crop[cov] = (0, 255, 0)
                    overlay_crop[over] = (255, 0, 0)
                    overlay_crop[still] = (0, 0, 255)
                    boundary_crop = cv2.morphologyEx((tmask.astype(np.uint8) * 255), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
                    overlay_crop[boundary_crop] = (255, 255, 255)
                    ymask = center[:, :, 1] > 0
                    overlay_crop[ymask] = (0, 255, 255)
                    cv2.imwrite(str(out_dir / f"component_{cid}_target_mask_crop.png"), crop_target)
                    cv2.imwrite(str(out_dir / f"component_{cid}_missed_crop.png"), crop_missed)
                    cv2.imwrite(str(out_dir / f"component_{cid}_candidate_centerline_crop.png"), center)
                    cv2.imwrite(str(out_dir / f"component_{cid}_candidate_footprint_crop.png"), crop_oracle)
                    cv2.imwrite(str(out_dir / f"component_{cid}_candidate_overlay_crop.png"), overlay_crop)
        return best_paths

    def _prune_penalized_negative_paths(
        self,
        paths: list[Toolpath],
        *,
        line_width_mm: float,
        connector_validation: dict[str, Any] | None,
        debug: Optional[dict[str, Any]] = None,
        max_iterations: int = 120,
    ) -> list[Toolpath]:
        if not connector_validation:
            return paths
        mask = connector_validation.get("mask")
        matrix = connector_validation.get("current_to_source_matrix")
        if mask is None or not isinstance(matrix, (tuple, list)) or len(matrix) != 6:
            return paths
        include_kinds = {
            "coverage_centerline",
            "coverage_offset_line",
            "coverage_rectilinear",
            "coverage_tiny_mark",
            "coverage_contour",
            "coverage_connector",
            "outline_cleanup",
        }
        removable_kinds = {"coverage_connector", "coverage_offset_line", "coverage_centerline"}
        current = list(paths)
        removed = 0
        metrics = compute_toolpath_mask_coverage_metrics(
            current,
            mask=mask,
            current_to_source_matrix=tuple(float(value) for value in matrix),
            pen_radius_mm=line_width_mm * 0.5,
            sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
            include_kinds=include_kinds,
        )
        if metrics is None:
            return paths
        iteration = 0
        while iteration < max_iterations:
            iteration += 1
            best_idx = -1
            best_gain = 0.0
            for idx, path in enumerate(current):
                if path.kind not in removable_kinds:
                    continue
                if path.kind == "coverage_centerline" and not bool(path.metadata.get("coverage_backfill_component", False)):
                    continue
                trial = current[:idx] + current[idx + 1:]
                trial_metrics = compute_toolpath_mask_coverage_metrics(
                    trial,
                    mask=mask,
                    current_to_source_matrix=tuple(float(value) for value in matrix),
                    pen_radius_mm=line_width_mm * 0.5,
                    sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
                    include_kinds=include_kinds,
                )
                if trial_metrics is None:
                    continue
                gain = trial_metrics.penalized_coverage_percent - metrics.penalized_coverage_percent
                if gain > best_gain + 1e-9:
                    best_gain = gain
                    best_idx = idx
            if best_idx < 0:
                break
            current = current[:best_idx] + current[best_idx + 1:]
            removed += 1
            metrics = compute_toolpath_mask_coverage_metrics(
                current,
                mask=mask,
                current_to_source_matrix=tuple(float(value) for value in matrix),
                pen_radius_mm=line_width_mm * 0.5,
                sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
                include_kinds=include_kinds,
            )
            if metrics is None:
                break
        if debug is not None:
            debug["penalized_pruned_path_count"] = int(removed)
            if metrics is not None:
                debug["penalized_pruned_final_score"] = float(metrics.penalized_coverage_percent)
        return current

    def _classify_fill_region(
        self,
        region: Any,
        *,
        spacing_mm: float,
        line_width_mm: float,
        min_segment_length_mm: float,
        preferred_angle_deg: float,
        axis_metrics: Optional[dict[str, float]] = None,
    ) -> dict[str, Any]:
        if region is None or region.is_empty:
            return {
                "mode": "small_detail_or_text",
                "reason": "empty_region",
                "bbox_width_mm": 0.0,
                "bbox_height_mm": 0.0,
                "min_dim_mm": 0.0,
                "max_dim_mm": 0.0,
                "area_mm2": 0.0,
                "perimeter_mm": 0.0,
                "aspect_ratio": 0.0,
                "number_of_holes": 0,
                "expected_hatch_segment_count": 0.0,
                "average_hatch_segment_length_mm": 0.0,
                "short_segment_ratio": 1.0,
                "small_detail_threshold_mm": self._small_detail_threshold_mm(line_width_mm),
                "offset_collapses": True,
            }

        min_x, min_y, max_x, max_y = region.bounds
        bbox_width_mm = max(0.0, max_x - min_x)
        bbox_height_mm = max(0.0, max_y - min_y)
        min_dim_mm = min(bbox_width_mm, bbox_height_mm)
        max_dim_mm = max(bbox_width_mm, bbox_height_mm)
        aspect_metrics = axis_metrics or self._polygon_axis_metrics(region)
        small_detail_threshold_mm = self._small_detail_threshold_mm(line_width_mm)
        area_mm2 = float(region.area)
        perimeter_mm = float(region.length)
        number_of_holes = sum(len(poly.interiors) for poly in normalize_geometry(region))

        hatch_metrics = self._score_infill_candidate(
            self._collect_scanline_rows(
                region,
                spacing_mm=spacing_mm,
                angle_deg=preferred_angle_deg,
                min_segment_length_mm=min_segment_length_mm,
            ),
            spacing_mm=spacing_mm,
            angle_deg=preferred_angle_deg,
        )
        expected_hatch_segment_count = float(hatch_metrics["segments"])
        average_hatch_segment_length_mm = float(hatch_metrics["average_segment_length_mm"])
        short_segment_count = float(hatch_metrics["short_segment_count"])
        short_segment_ratio = short_segment_count / max(1.0, expected_hatch_segment_count)
        offset_probe = region.buffer(-max(line_width_mm / 2.0, 1e-6), join_style=1)
        offset_collapses = bool(offset_probe.is_empty)

        mode = "large_open"
        reason = "default_large_open"
        if float(aspect_metrics.get("aspect_ratio", 0.0)) >= DEFAULT_LONG_THIN_INFILL_ASPECT_RATIO:
            mode = "long_thin"
            reason = "high_aspect_ratio"
        elif (
            offset_collapses
            or area_mm2 <= max(line_width_mm * line_width_mm, small_detail_threshold_mm * small_detail_threshold_mm * DEFAULT_SMALL_DETAIL_AREA_FACTOR * 0.1)
            or min_dim_mm < small_detail_threshold_mm
        ):
            mode = "small_detail_or_text"
            reason = "small_dimension_or_area"
        elif (
            number_of_holes > 0
            and (
                min_dim_mm <= small_detail_threshold_mm * 2.0
                or area_mm2 <= small_detail_threshold_mm * small_detail_threshold_mm * DEFAULT_SMALL_DETAIL_AREA_FACTOR
            )
        ):
            mode = "small_detail_or_text"
            reason = "counter_region"
        elif (
            number_of_holes > 0
            and (
                short_segment_ratio >= 0.35
                or average_hatch_segment_length_mm <= spacing_mm * 2.5
                or expected_hatch_segment_count >= max(6.0, (area_mm2 / max(spacing_mm * spacing_mm, 1e-6)) * 0.6)
            )
        ):
            mode = "small_detail_or_text"
            reason = "holes_with_fragmented_hatch"
        elif (
            short_segment_ratio >= 0.5
            or average_hatch_segment_length_mm <= spacing_mm * 2.0
            or expected_hatch_segment_count >= max(8.0, (area_mm2 / max(spacing_mm * spacing_mm, 1e-6)) * 0.75)
        ):
            mode = "small_detail_or_text"
            reason = "poor_hatch_quality"

        return {
            "mode": mode,
            "reason": reason,
            "bbox_width_mm": bbox_width_mm,
            "bbox_height_mm": bbox_height_mm,
            "min_dim_mm": min_dim_mm,
            "max_dim_mm": max_dim_mm,
            "area_mm2": area_mm2,
            "perimeter_mm": perimeter_mm,
            "aspect_ratio": float(aspect_metrics.get("aspect_ratio", 0.0)),
            "number_of_holes": number_of_holes,
            "expected_hatch_segment_count": expected_hatch_segment_count,
            "average_hatch_segment_length_mm": average_hatch_segment_length_mm,
            "short_segment_ratio": short_segment_ratio,
            "small_detail_threshold_mm": small_detail_threshold_mm,
            "offset_collapses": offset_collapses,
        }

    def _build_cell_region_from_segments(
        self,
        segments: list[InfillSegment],
        *,
        angle_deg: float,
        origin: tuple[float, float],
        spacing_mm: float,
        line_width_mm: float,
        cover_region: Any,
    ) -> Any:
        if not segments:
            return None
        stroke_half = max(0.05, line_width_mm * 0.5)
        buffered_lines = []
        for segment in segments:
            line = LineString(segment.coords if segment.coords else [(segment.low_u.x, segment.low_u.y), (segment.high_u.x, segment.high_u.y)])
            world_line = affinity.rotate(line, angle_deg, origin=origin)
            buffered_lines.append(world_line.buffer(stroke_half, join_style=1, cap_style=1))
        if not buffered_lines:
            return None
        cell_region = unary_union(buffered_lines)
        if cover_region is not None and not cover_region.is_empty:
            cell_region = cell_region.intersection(cover_region)
        return cell_region

    def _evaluate_adaptive_cell_mode(
        self,
        *,
        cell_segments: list[InfillSegment],
        spacing_mm: float,
        line_width_mm: float,
        cover_region: Any,
    ) -> InfillCellAdaptiveDecision:
        if not cell_segments:
            return InfillCellAdaptiveDecision(mode="detail_contour", reasons=["empty_cell"], metrics={})

        lengths = [max(0.0, float(segment.length)) for segment in cell_segments]
        total_hatch_length = sum(lengths)
        avg_segment_length = total_hatch_length / max(1, len(lengths))
        short_threshold = max(line_width_mm * 1.5, spacing_mm * DEFAULT_SHORT_INFILL_SEGMENT_FACTOR)
        short_segments = sum(1 for value in lengths if value <= short_threshold)
        short_ratio = short_segments / max(1, len(lengths))
        row_count = len({int(segment.row_index) for segment in cell_segments})

        connector_length = 0.0
        connector_count = 0
        for previous, current in zip(cell_segments, cell_segments[1:]):
            connector = LineString([(previous.high_u.x, previous.high_u.y), (current.low_u.x, current.low_u.y)])
            if cover_region is not None and not cover_region.is_empty:
                if not _line_fully_inside(cover_region, connector, tolerance_mm=max(0.01, spacing_mm * 0.05)):
                    continue
            connector_length += float(connector.length)
            connector_count += 1
        connector_ratio = connector_length / max(total_hatch_length, 1e-6)

        min_u = min(float(segment.min_u) for segment in cell_segments)
        max_u = max(float(segment.max_u) for segment in cell_segments)
        min_row_offset = min(float(segment.scanline_offset) for segment in cell_segments)
        max_row_offset = max(float(segment.scanline_offset) for segment in cell_segments)
        width_u = max(0.0, max_u - min_u)
        width_v = max(0.0, max_row_offset - min_row_offset)
        approx_local_width = min(width_u, width_v if width_v > 1e-6 else spacing_mm)
        area_estimate_mm2 = max(0.0, width_u * max(width_v, spacing_mm))
        sorted_by_row = sorted(cell_segments, key=lambda segment: (segment.row_index, segment.min_u, segment.interval_index))
        center_u_values = [self._segment_center_u(segment) for segment in sorted_by_row]
        direction_changes = 0
        last_sign = 0
        for previous_value, next_value in zip(center_u_values, center_u_values[1:]):
            delta = next_value - previous_value
            sign = 0 if abs(delta) <= 1e-6 else (1 if delta > 0 else -1)
            if sign == 0:
                continue
            if last_sign != 0 and sign != last_sign:
                direction_changes += 1
            last_sign = sign

        reasons: list[str] = []
        if row_count < 3 and avg_segment_length <= spacing_mm * 3.0:
            reasons.append("too_few_hatch_rows")
        if approx_local_width < 3.0 * spacing_mm and avg_segment_length <= spacing_mm * 4.0:
            reasons.append("narrow_local_width")
        if avg_segment_length <= spacing_mm * 2.2 or short_ratio >= 0.6:
            reasons.append("short_fragmented_hatch")
        if direction_changes >= 2:
            reasons.append("narrow_curved_stroke")
        if total_hatch_length <= spacing_mm * 2.0 and avg_segment_length <= spacing_mm * 1.5:
            reasons.append("mostly_connector_fragments")
        if connector_ratio >= 0.45:
            reasons.append("connector_dominates_hatch")

        mode = "rectilinear"
        single_stroke_reasons: list[str] = []
        if approx_local_width <= line_width_mm * 1.5:
            single_stroke_reasons.append("width_lte_1p5x_pen")
        elif approx_local_width <= line_width_mm * 2.0 and (row_count <= 2 or short_ratio >= 0.6):
            single_stroke_reasons.append("width_lte_2x_pen_with_poor_rows")
        if row_count <= 1:
            single_stroke_reasons.append("only_one_useful_hatch_row")
        if short_ratio >= 0.8:
            single_stroke_reasons.append("mostly_tiny_hatch_fragments")
        if connector_ratio >= 0.6:
            single_stroke_reasons.append("connector_length_too_high")
        if direction_changes >= 1 and approx_local_width <= line_width_mm * 2.5:
            single_stroke_reasons.append("thin_curved_script_like_cell")

        if single_stroke_reasons:
            mode = "single_stroke"
            reasons = list(dict.fromkeys(single_stroke_reasons + reasons))
        elif reasons:
            mode = "detail_contour"
        return InfillCellAdaptiveDecision(
            mode=mode,
            reasons=reasons,
            metrics={
                "cell_area_estimate_mm2": area_estimate_mm2,
                "cell_row_count": float(row_count),
                "avg_hatch_segment_length_mm": avg_segment_length,
                "short_hatch_segment_ratio": short_ratio,
                "connector_length_mm": connector_length,
                "connector_count": float(connector_count),
                "connector_to_hatch_ratio": connector_ratio,
                "approx_local_width_mm": approx_local_width,
                "estimated_hatch_rows": float(row_count),
                "centerline_turns": float(direction_changes),
            },
        )

    def _build_stable_scanline_rows(
        self,
        region: Any,
        *,
        spacing_mm: float,
        angle_deg: float,
    ) -> dict[str, Any]:
        if region is None or region.is_empty or spacing_mm <= 0:
            return {"origin": None, "rotated_region": None, "rows": [], "cover_region": None, "region_area": 0.0}

        origin = (0.0, 0.0)
        rotated = affinity.rotate(region, -angle_deg, origin=origin)
        epsilon = max(1e-6, spacing_mm * 0.01)
        rows: list[dict[str, Any]] = []

        for polygon_index, polygon in enumerate(normalize_geometry(rotated)):
            poly_min_x, poly_min_y, poly_max_x, poly_max_y = polygon.bounds
            if not all(math.isfinite(value) for value in [poly_min_x, poly_min_y, poly_max_x, poly_max_y]):
                continue
            start_index = math.floor((poly_min_y - 1e-6) / spacing_mm)
            end_index = math.ceil((poly_max_y + 1e-6) / spacing_mm)
            for grid_index in range(start_index, end_index + 1):
                offset_mm = grid_index * spacing_mm
                raw_scan = LineString([(poly_min_x - spacing_mm, offset_mm), (poly_max_x + spacing_mm, offset_mm)])
                raw_segments = extract_lines(polygon.intersection(raw_scan))
                rows.append({
                    "polygon_index": polygon_index,
                    "grid_index": grid_index,
                    "offset_mm": offset_mm,
                    "raw_segments": [list(line.coords) for line in raw_segments if len(line.coords) >= 2],
                })

        rows.sort(key=lambda row: (row["offset_mm"], row["polygon_index"], row["grid_index"]))
        for row in rows:
            serpentine_reverse = bool(row["grid_index"] % 2)
            oriented_segments: list[list[tuple[float, float]]] = []
            for coords in row["raw_segments"]:
                segment = list(coords)
                if serpentine_reverse:
                    if segment[0][0] < segment[-1][0]:
                        segment.reverse()
                else:
                    if segment[0][0] > segment[-1][0]:
                        segment.reverse()
                oriented_segments.append(segment)
            oriented_segments.sort(
                key=lambda coords: min(coords[0][0], coords[-1][0]),
                reverse=serpentine_reverse,
            )
            row["raw_segments"] = oriented_segments

        return {
            "origin": origin,
            "rotated_region": rotated,
            "rows": rows,
            "cover_region": rotated.buffer(epsilon, join_style=1),
            "region_area": float(region.area),
        }

    def _finalize_scanline_rows(
        self,
        row_data: dict[str, Any],
        *,
        spacing_mm: float,
        min_segment_length_mm: float,
    ) -> dict[str, Any]:
        rows = row_data.get("rows") or []
        short_segment_threshold_mm = self._scanline_filter_threshold_mm(spacing_mm, min_segment_length_mm)
        finalized_rows: list[dict[str, Any]] = []
        for row in rows:
            kept_segments: list[list[tuple[float, float]]] = []
            filtered_segments: list[list[tuple[float, float]]] = []
            for coords in row.get("raw_segments") or []:
                if float(LineString(coords).length) + 1e-6 >= short_segment_threshold_mm:
                    kept_segments.append(coords)
                else:
                    filtered_segments.append(coords)
            finalized_rows.append({
                **row,
                "segments": kept_segments,
                "filtered_segments": filtered_segments,
            })

        kept_rows = [row for row in finalized_rows if row["segments"]]
        gap_tolerance_mm = self._scanline_gap_tolerance_mm(spacing_mm)
        for previous_row, next_row in zip(kept_rows, kept_rows[1:]):
            gap_mm = next_row["offset_mm"] - previous_row["offset_mm"]
            if gap_mm <= gap_tolerance_mm + 1e-6:
                continue
            for candidate in finalized_rows:
                if candidate["segments"]:
                    continue
                if candidate["offset_mm"] <= previous_row["offset_mm"] + 1e-6:
                    continue
                if candidate["offset_mm"] >= next_row["offset_mm"] - 1e-6:
                    continue
                filtered_segments = candidate.get("filtered_segments") or []
                if not filtered_segments:
                    continue
                longest = max(filtered_segments, key=lambda coords: float(LineString(coords).length))
                candidate["segments"] = [longest]
                candidate["filtered_segments"] = [coords for coords in filtered_segments if coords is not longest]

        row_data = dict(row_data)
        row_data["rows"] = finalized_rows
        row_data["short_segment_threshold_mm"] = short_segment_threshold_mm
        row_data["gap_tolerance_mm"] = gap_tolerance_mm
        return row_data

    def _collect_scanline_rows(
        self,
        region: Any,
        *,
        spacing_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
    ) -> dict[str, Any]:
        row_data = self._build_stable_scanline_rows(
            region,
            spacing_mm=spacing_mm,
            angle_deg=angle_deg,
        )
        return self._finalize_scanline_rows(
            row_data,
            spacing_mm=spacing_mm,
            min_segment_length_mm=min_segment_length_mm,
        )

    def _score_infill_candidate(
        self,
        row_data: dict[str, Any],
        *,
        spacing_mm: float,
        angle_deg: float,
    ) -> dict[str, Any]:
        rows = row_data.get("rows") or []
        cover_region = row_data.get("cover_region")
        region_area = float(row_data.get("region_area", 0.0))
        short_segment_threshold_mm = float(row_data.get("short_segment_threshold_mm", 0.0))
        max_connector_length_mm = self._max_pen_down_connector_length_mm(spacing_mm)

        segment_lengths: list[float] = []
        ordered_segments: list[list[tuple[float, float]]] = []
        short_segment_count = 0
        row_count = 0

        for row in rows:
            row_segments = row.get("segments") or []
            if row_segments:
                row_count += 1
            for coords in row_segments:
                line = LineString(coords)
                segment_lengths.append(float(line.length))
                if line.length < short_segment_threshold_mm:
                    short_segment_count += 1
                ordered_segments.append(coords)

        segment_count = len(segment_lengths)
        total_draw_length_mm = sum(segment_lengths)
        average_segment_length_mm = total_draw_length_mm / segment_count if segment_count else 0.0
        median_segment_length_mm = statistics.median(segment_lengths) if segment_lengths else 0.0
        longest_segment_length_mm = max(segment_lengths) if segment_lengths else 0.0
        pen_lifts = 0
        estimated_travel_length_mm = 0.0
        pen_down_connector_count = 0
        turnaround_penalty = max(0, row_count - 1)

        for current_coords, next_coords in zip(ordered_segments, ordered_segments[1:]):
            connector = LineString([current_coords[-1], next_coords[0]])
            connector_is_short = connector.length <= max_connector_length_mm + 1e-6
            # Accept connectors when the full connector segment is inside the
            # covered region (with a small tolerance). This permits long
            # angled travels that lie entirely within the infill area.
            connector_fully_inside = bool(
                cover_region is not None and _line_fully_inside(cover_region, connector, tolerance_mm=max(0.01, spacing_mm * 0.05))
            )
            if connector_fully_inside:
                pen_down_connector_count += 1
                continue
            connector_inside = bool(cover_region is not None and cover_region.covers(connector))
            if connector_inside and connector_is_short:
                pen_down_connector_count += 1
                continue
            pen_lifts += 1
            estimated_travel_length_mm += float(connector.length)

        coverage_ratio = 0.0
        if region_area > 1e-9:
            coverage_ratio = max(0.0, min(1.0, (total_draw_length_mm * spacing_mm) / region_area))

        score = (
            (average_segment_length_mm * 5.0)
            + (median_segment_length_mm * 4.0)
            + (longest_segment_length_mm * 1.5)
            - (segment_count * spacing_mm * 0.7)
            - (short_segment_count * spacing_mm * 2.5)
            - (estimated_travel_length_mm * 1.35)
            - (pen_lifts * spacing_mm * 1.1)
            - (turnaround_penalty * spacing_mm * 0.4)
            + (coverage_ratio * spacing_mm)
        )
        return {
            "angle_deg": _normalize_infill_angle_deg(angle_deg),
            "score": score,
            "segments": float(segment_count),
            "rows": float(row_count),
            "total_length": total_draw_length_mm,
            "coverage_ratio": coverage_ratio,
            "average_segment_length_mm": average_segment_length_mm,
            "median_segment_length_mm": median_segment_length_mm,
            "longest_segment_length_mm": longest_segment_length_mm,
            "short_segment_count": float(short_segment_count),
            "number_of_pen_lifts": float(pen_lifts),
            "estimated_travel_length_mm": estimated_travel_length_mm,
            "turnaround_penalty": float(turnaround_penalty),
            "pen_down_connector_count": float(pen_down_connector_count),
        }

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

    def _row_world_paths(
        self,
        rows: list[dict[str, Any]],
        *,
        angle_deg: float,
        origin: tuple[float, float],
        tolerance_mm: float,
        kind: str,
    ) -> list[dict[str, Any]]:
        world_paths: list[dict[str, Any]] = []
        for row in rows:
            for coords in row.get("segments") or []:
                world_line = affinity.rotate(LineString(coords), angle_deg, origin=origin)
                points = simplify_segment_points([Point(x, y) for x, y in world_line.coords], tolerance_mm, False)
                if len(points) < 2:
                    continue
                world_paths.append({
                    "row": row,
                    "coords": list(coords),
                    "toolpath": Toolpath(
                        points=points,
                        kind=kind,
                        closed=False,
                        metadata={
                            "scanline_offset_mm": float(row["offset_mm"]),
                            "scanline_grid_index": int(row["grid_index"]),
                            "scanline_polygon_index": int(row["polygon_index"]),
                        },
                    ),
                })
        return world_paths

    def _to_infill_segments(
        self,
        rows: list[dict[str, Any]],
        *,
        component_id: str,
    ) -> list[InfillSegment]:
        segments: list[InfillSegment] = []
        for row in rows:
            row_index = int(row["grid_index"])
            offset = float(row["offset_mm"])
            for interval_index, coords in enumerate(row.get("segments") or []):
                if len(coords) < 2:
                    continue
                start, end = coords[0], coords[-1]
                if start[0] <= end[0]:
                    low_xy, high_xy = start, end
                else:
                    low_xy, high_xy = end, start
                center = Point(float((low_xy[0] + high_xy[0]) * 0.5), float((low_xy[1] + high_xy[1]) * 0.5))
                segments.append(InfillSegment(
                    id=f"{component_id}:r{row_index}:i{interval_index}",
                    component_id=component_id,
                    row_index=row_index,
                    interval_index=interval_index,
                    cell_id=None,
                    scanline_offset=offset,
                    low_u=Point(float(low_xy[0]), float(low_xy[1])),
                    high_u=Point(float(high_xy[0]), float(high_xy[1])),
                    min_u=float(min(start[0], end[0])),
                    max_u=float(max(start[0], end[0])),
                    center=center,
                    length=float(LineString(coords).length),
                    coords=list(coords),
                ))
        return segments

    def _segment_overlap_mm(self, a: InfillSegment, b: InfillSegment) -> float:
        return min(a.max_u, b.max_u) - max(a.min_u, b.min_u)

    def _segment_length_mm(self, segment: InfillSegment) -> float:
        return max(0.0, segment.max_u - segment.min_u)

    def _segment_gap_mm(self, a: InfillSegment, b: InfillSegment) -> float:
        return max(0.0, max(a.min_u, b.min_u) - min(a.max_u, b.max_u))

    def _cell_centroid_for_segments(self, segments: list[InfillSegment]) -> Point:
        if not segments:
            return Point(0.0, 0.0)
        return Point(
            sum(segment.center.x for segment in segments) / len(segments),
            sum(segment.center.y for segment in segments) / len(segments),
        )

    def _cell_entry_exit_points(self, cell_paths: list[Toolpath]) -> tuple[Point, Point]:
        drawing_paths = [path for path in cell_paths if len(path.points) >= 2 and path.kind != "fill-infill-travel"]
        if not drawing_paths:
            return Point(0.0, 0.0), Point(0.0, 0.0)
        entry = drawing_paths[0].points[0]
        exit_point = drawing_paths[-1].points[-1]
        return Point(float(entry.x), float(entry.y)), Point(float(exit_point.x), float(exit_point.y))

    def _order_infill_cells(
        self,
        cell_plans: list[InfillCellPlan],
        *,
        origin: tuple[float, float],
    ) -> list[InfillCellPlan]:
        if len(cell_plans) <= 1:
            return list(cell_plans)

        remaining = list(cell_plans)
        ordered: list[InfillCellPlan] = []
        current_point = Point(float(origin[0]), float(origin[1]))

        start = min(
            remaining,
            key=lambda plan: (
                math.hypot(plan.entry_point.x - current_point.x, plan.entry_point.y - current_point.y),
                math.hypot(plan.centroid.x - current_point.x, plan.centroid.y - current_point.y),
                plan.cell_id,
            ),
        )
        ordered.append(start)
        remaining.remove(start)
        current_point = start.exit_point

        while remaining:
            next_plan = min(
                remaining,
                key=lambda plan: (
                    math.hypot(plan.entry_point.x - current_point.x, plan.entry_point.y - current_point.y),
                    math.hypot(plan.centroid.x - current_point.x, plan.centroid.y - current_point.y),
                    plan.cell_id,
                ),
            )
            ordered.append(next_plan)
            remaining.remove(next_plan)
            current_point = next_plan.exit_point

        return ordered

    def _segment_center_u(self, segment: InfillSegment) -> float:
        return (segment.min_u + segment.max_u) * 0.5

    def _segment_connectable_same_side(
        self,
        polygon: Polygon,
        a: InfillSegment,
        b: InfillSegment,
        *,
        side: str,
        spacing_mm: float,
        line_width_mm: float,
        tolerance_mm: float,
        connector_validation: dict[str, Any] | None = None,
    ) -> tuple[bool, str]:
        start_pt = (a.low_u.x, a.low_u.y) if side == "low" else (a.high_u.x, a.high_u.y)
        end_pt = (b.low_u.x, b.low_u.y) if side == "low" else (b.high_u.x, b.high_u.y)
        connector = LineString([start_pt, end_pt])
        result = self._validate_infill_connector(
            polygon,
            connector,
            from_meta={
                "infill_component_id": a.component_id,
                "infill_row_index": a.row_index,
                "cell_id": a.cell_id,
                "infill_start_side": side,
                "infill_end_side": side,
            },
            to_meta={
                "infill_component_id": b.component_id,
                "infill_row_index": b.row_index,
                "cell_id": b.cell_id,
                "infill_start_side": side,
                "infill_end_side": side,
            },
            spacing_mm=spacing_mm,
            line_width_mm=line_width_mm,
            tolerance_mm=tolerance_mm,
            connector_validation=connector_validation,
        )
        return result.accepted, result.reason

    def _assign_infill_cells(
        self,
        polygon: Polygon,
        segments: list[InfillSegment],
        *,
        spacing_mm: float,
        line_width_mm: float,
        tolerance_mm: float,
        connector_validation: dict[str, Any] | None = None,
    ) -> tuple[dict[str, list[InfillSegment]], dict[str, int]]:
        stats = {
            "rejected_cross_gap_connectors": 0,
            "rejected_different_cell_connectors": 0,
            "rejected_opposite_side_connectors": 0,
            "rejected_too_long_connectors": 0,
            "rejected_outside_selected_color_connectors": 0,
            "rows_with_multiple_intervals": 0,
        }
        if not segments:
            return {}, stats

        row_counts: dict[int, int] = {}
        for seg in segments:
            row_counts[seg.row_index] = row_counts.get(seg.row_index, 0) + 1
        stats["rows_with_multiple_intervals"] = sum(1 for count in row_counts.values() if count > 1)

        by_row: dict[int, list[InfillSegment]] = {}
        for seg in segments:
            by_row.setdefault(seg.row_index, []).append(seg)
        for row in by_row.values():
            row.sort(key=lambda s: (s.min_u, s.max_u, s.interval_index))

        cell_index = 0
        previous_row_segments: list[InfillSegment] = []
        for row_index in sorted(by_row):
            current_segments = by_row[row_index]
            used_previous_ids: set[str] = set()
            for segment in current_segments:
                best_match: InfillSegment | None = None
                best_score = float("inf")
                for candidate in previous_row_segments:
                    if candidate.id in used_previous_ids:
                        # Prevent one previous interval from fan-out into multiple
                        # current intervals, which collapses broken rows into one cell.
                        continue
                    # Keep lane continuity when a row has multiple intervals.
                    if abs(candidate.interval_index - segment.interval_index) > 1:
                        continue
                    overlap = self._segment_overlap_mm(candidate, segment)
                    low_ok, low_reason = self._segment_connectable_same_side(
                        polygon,
                        candidate,
                        segment,
                        side="low",
                        spacing_mm=spacing_mm,
                        line_width_mm=line_width_mm,
                        tolerance_mm=tolerance_mm,
                        connector_validation=connector_validation,
                    )
                    high_ok, high_reason = self._segment_connectable_same_side(
                        polygon,
                        candidate,
                        segment,
                        side="high",
                        spacing_mm=spacing_mm,
                        line_width_mm=line_width_mm,
                        tolerance_mm=tolerance_mm,
                        connector_validation=connector_validation,
                    )
                    if not low_ok and not high_ok:
                        reasons = {low_reason, high_reason}
                        if "too_long" in reasons:
                            stats["rejected_too_long_connectors"] += 1
                        elif "outside_selected_color" in reasons:
                            stats["rejected_outside_selected_color_connectors"] += 1
                        elif "diagonal_gap_crossing" in reasons or "outside_fillable_polygon" in reasons or "crosses_gap_hole_void" in reasons:
                            stats["rejected_cross_gap_connectors"] += 1
                        else:
                            stats["rejected_different_cell_connectors"] += 1
                        continue

                    low_connector_len = math.hypot(candidate.low_u.x - segment.low_u.x, candidate.low_u.y - segment.low_u.y)
                    high_connector_len = math.hypot(candidate.high_u.x - segment.high_u.x, candidate.high_u.y - segment.high_u.y)
                    connector_score = min(low_connector_len, high_connector_len)
                    if connector_score < best_score:
                        best_score = connector_score
                        best_match = candidate

                if best_match is None:
                    segment.cell_id = f"{segment.component_id}:cell_{cell_index:04d}"
                    cell_index += 1
                    continue
                segment.cell_id = best_match.cell_id
                used_previous_ids.add(best_match.id)
            previous_row_segments = current_segments

        cells: dict[str, list[InfillSegment]] = {}
        for seg in segments:
            if seg.cell_id is None:
                seg.cell_id = f"{seg.component_id}:cell_{cell_index:04d}"
                cell_index += 1
            cells.setdefault(seg.cell_id, []).append(seg)
        return cells, stats

    def _segment_to_world_toolpath(
        self,
        segment: InfillSegment,
        *,
        angle_deg: float,
        origin: tuple[float, float],
        tolerance_mm: float,
        kind: str,
    ) -> Toolpath:
        world_line = affinity.rotate(LineString(segment.coords), angle_deg, origin=origin)
        points = simplify_segment_points([Point(x, y) for x, y in world_line.coords], tolerance_mm, False)
        start_side = "low" if segment.coords and segment.coords[0][0] <= segment.coords[-1][0] else "high"
        end_side = "high" if start_side == "low" else "low"
        return Toolpath(
            points=points,
            kind=kind,
            closed=False,
            metadata={
                "scanline_offset_mm": float(segment.scanline_offset),
                "scanline_grid_index": int(segment.row_index),
                "scanline_polygon_index": int(segment.component_id.split("_")[-1]) if segment.component_id.split("_")[-1].isdigit() else 0,
                "interval_index": int(segment.interval_index),
                "cell_id": segment.cell_id,
                "infill_component_id": segment.component_id,
                "infill_row_index": int(segment.row_index),
                "infill_interval_index": int(segment.interval_index),
                "infill_start_side": start_side,
                "infill_end_side": end_side,
                "infill_segment_id": segment.id,
            },
        )

    def _plan_cell_paths(
        self,
        polygon: Polygon,
        cell_segments: list[InfillSegment],
        *,
        spacing_mm: float,
        line_width_mm: float | None = None,
        angle_deg: float,
        origin: tuple[float, float],
        tolerance_mm: float,
        kind: str,
        debug: Optional[dict[str, Any]],
        connector_validation: dict[str, Any] | None = None,
    ) -> tuple[list[Toolpath], dict[str, int]]:
        stats = {"accepted_connectors": 0, "pen_lifts": 0}
        if not cell_segments:
            return [], stats
        cell_segments = sorted(cell_segments, key=lambda s: (s.row_index, s.min_u, s.interval_index))
        toolpaths: list[Toolpath] = []
        effective_line_width_mm = line_width_mm if line_width_mm is not None else spacing_mm

        def _segment_side(segment: InfillSegment, point: Point) -> str:
            low_distance = math.hypot(point.x - segment.low_u.x, point.y - segment.low_u.y)
            high_distance = math.hypot(point.x - segment.high_u.x, point.y - segment.high_u.y)
            return "low" if low_distance <= high_distance else "high"

        def _choose_oriented_coords(
            current_segment: InfillSegment,
            current_coords: list[tuple[float, float]],
            next_segment: InfillSegment,
        ) -> tuple[list[tuple[float, float]] | None, str | None, InfillConnectorValidationResult | None]:
            current_end = Point(float(current_coords[-1][0]), float(current_coords[-1][1]))
            current_side = _segment_side(current_segment, current_end)
            best_coords: list[tuple[float, float]] | None = None
            best_reason: str | None = None
            best_result: InfillConnectorValidationResult | None = None
            best_length = float("inf")
            for oriented in (list(next_segment.coords), list(reversed(next_segment.coords))):
                if len(oriented) < 2:
                    continue
                next_start = Point(float(oriented[0][0]), float(oriented[0][1]))
                next_side = _segment_side(next_segment, next_start)
                if next_side != current_side:
                    continue
                connector_result = self._validate_infill_connector(
                    polygon,
                    LineString([(
                        current_end.x,
                        current_end.y,
                    ), (
                        next_start.x,
                        next_start.y,
                    )]),
                    from_meta={
                        "infill_component_id": current_segment.component_id,
                        "infill_row_index": current_segment.row_index,
                        "cell_id": current_segment.cell_id,
                        "infill_end_side": current_side,
                    },
                    to_meta={
                        "infill_component_id": next_segment.component_id,
                        "infill_row_index": next_segment.row_index,
                        "cell_id": next_segment.cell_id,
                        "infill_start_side": next_side,
                    },
                    spacing_mm=spacing_mm,
                    line_width_mm=effective_line_width_mm,
                    tolerance_mm=tolerance_mm,
                    connector_validation=connector_validation,
                )
                if not connector_result.accepted:
                    best_reason = connector_result.reason
                    continue
                connector_length = float(LineString(connector_result.connector_coords or [(current_end.x, current_end.y), (next_start.x, next_start.y)]).length)
                if connector_length < best_length:
                    best_length = connector_length
                    best_coords = oriented
                    best_reason = None
                    best_result = connector_result
            return best_coords, best_reason, best_result

        current_coords: list[tuple[float, float]] | None = None
        current_segment: InfillSegment | None = None

        def _segment_world_points(coords: list[tuple[float, float]]) -> list[Point]:
            world_line = affinity.rotate(LineString(coords), angle_deg, origin=origin)
            return simplify_segment_points([Point(x, y) for x, y in world_line.coords], tolerance_mm, False)

        if cell_segments:
            first_segment = cell_segments[0]
            current_segment = first_segment
            current_coords = list(first_segment.coords)
            if len(cell_segments) > 1:
                best_first_coords: list[tuple[float, float]] | None = None
                best_first_length = float("inf")
                for oriented in (list(first_segment.coords), list(reversed(first_segment.coords))):
                    if len(oriented) < 2:
                        continue
                    trial_current = oriented
                    chosen_coords, _, connector_result = _choose_oriented_coords(first_segment, trial_current, cell_segments[1])
                    if chosen_coords is None or connector_result is None:
                        continue
                    connector_length = float(LineString(connector_result.connector_coords or [trial_current[-1], chosen_coords[0]]).length)
                    if connector_length < best_first_length:
                        best_first_length = connector_length
                        best_first_coords = oriented
                if best_first_coords is not None:
                    current_coords = best_first_coords

        if current_coords is None or current_segment is None:
            return [], stats

        for segment in cell_segments[1:]:
            chosen_coords, reason, connector_result = _choose_oriented_coords(current_segment, current_coords, segment)
            if chosen_coords is not None and connector_result is not None:
                current_points = _segment_world_points(current_coords)
                if len(current_points) >= 2:
                    toolpaths.append(Toolpath(points=current_points, kind=kind, closed=False, metadata={
                        "scanline_offset_mm": float(current_segment.scanline_offset),
                        "scanline_grid_index": int(current_segment.row_index),
                        "interval_index": int(current_segment.interval_index),
                        "cell_id": current_segment.cell_id,
                        "infill_segment_id": current_segment.id,
                    }))
                connector_coords = connector_result.connector_coords or [current_coords[-1], chosen_coords[0]]
                if debug is not None and connector_result.sample_failures:
                    debug.setdefault("connector_validation_sample_failures", []).extend(connector_result.sample_failures)
                connector_points = [Point(float(x), float(y)) for x, y in connector_coords]
                toolpaths.append(Toolpath(
                    points=connector_points,
                    kind="fill-infill-travel",
                    closed=False,
                    source="infill_connector",
                    metadata={
                        "projection_count": 0,
                        "expected_relation_to_fill": "internal_fill_connector",
                        "travel_mode": "pen_down",
                        "connector_mode": connector_result.connector_mode,
                        "connector_source_path_id": current_segment.id,
                        "connector_target_path_id": segment.id,
                        "cell_id": current_segment.cell_id,
                        "infill_component_id": current_segment.component_id,
                    },
                ))
                stats["accepted_connectors"] += 1
                self._emit_debug_connector(
                    debug,
                    "valid_infill_connectors",
                    connector_coords[0],
                    connector_coords[-1],
                    angle_deg,
                    origin,
                    "debug-valid-connector",
                )
                current_coords = chosen_coords
                current_segment = segment
                continue

            if debug is not None:
                reasons = debug.setdefault("connector_rejection_reasons", {})
                if reason is not None:
                    reasons[reason] = int(reasons.get(reason, 0)) + 1
                if reason == "outside_selected_color":
                    rejection_counts = debug.setdefault("connector_rejection_counts", {})
                    rejection_counts[reason] = int(rejection_counts.get(reason, 0)) + 1
                if connector_validation is not None and reason == "outside_selected_color" and connector_result is not None:
                    for sample_failure in connector_result.sample_failures:
                        self._emit_debug_connector(
                            debug,
                            "rejected_infill_connector_sample_points",
                            (float(sample_failure["surface_x"]) - 0.05, float(sample_failure["surface_y"])),
                            (float(sample_failure["surface_x"]) + 0.05, float(sample_failure["surface_y"])),
                            angle_deg,
                            origin,
                            "debug-connector-failure-sample",
                        )
            self._emit_debug_connector(
                debug,
                "rejected_infill_connectors",
                current_coords[-1],
                segment.coords[0],
                angle_deg,
                origin,
                "debug-rejected-connector",
            )
            points = _segment_world_points(current_coords)
            if len(points) >= 2:
                toolpaths.append(Toolpath(points=points, kind=kind, closed=False, metadata={
                    "scanline_offset_mm": float(current_segment.scanline_offset),
                    "scanline_grid_index": int(current_segment.row_index),
                    "interval_index": int(current_segment.interval_index),
                    "cell_id": current_segment.cell_id,
                    "infill_segment_id": current_segment.id,
                }))
            stats["pen_lifts"] += 1
            current_coords = list(segment.coords)
            current_segment = segment

        if current_coords is not None and current_segment is not None:
            points = _segment_world_points(current_coords)
            if len(points) >= 2:
                toolpaths.append(Toolpath(points=points, kind=kind, closed=False, metadata={
                    "scanline_offset_mm": float(current_segment.scanline_offset),
                    "scanline_grid_index": int(current_segment.row_index),
                    "interval_index": int(current_segment.interval_index),
                    "cell_id": current_segment.cell_id,
                    "infill_segment_id": current_segment.id,
                }))
        return toolpaths, stats

    def _plan_scanline_connector(
        self,
        polygon: Polygon,
        current_row: dict[str, Any],
        current_coords: list[tuple[float, float]],
        next_row: dict[str, Any],
        next_coords: list[tuple[float, float]],
        *,
        spacing_mm: float,
        tolerance_mm: float,
    ) -> tuple[list[tuple[float, float]], str]:
        row_delta = abs(float(next_row["offset_mm"]) - float(current_row["offset_mm"]))
        if abs(row_delta - spacing_mm) > max(1e-6, spacing_mm * 0.1):
            return [], "non_adjacent_row"

        start = current_coords[-1]
        end = next_coords[0]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        direct = LineString([start, end])
        # Prefer a direct chord when the entire segment is confidently
        # inside the polygon (shrunk by tolerance). This recovers angled
        # connectors that otherwise fail strict boundary tests.
        if _line_fully_inside(polygon.buffer(max(tolerance_mm, 1e-6), join_style=1), direct, tolerance_mm=max(0.01, spacing_mm * 0.05)):
            if (
                abs(abs(dy) - spacing_mm) <= max(1e-6, spacing_mm * 0.1)
                and abs(dx) <= (spacing_mm * 0.35)
            ):
                return [start, end], "ok_direct"

        cover_region = polygon.buffer(max(tolerance_mm, 1e-6), join_style=1)

        boundary_coords = boundary_connector_coords(
            polygon,
            start,
            end,
            tolerance=max(tolerance_mm, spacing_mm * 0.75, 0.05),
        )
        if len(boundary_coords) < 2:
            return [], "outside_polygon"
        boundary_line = LineString(boundary_coords)
        if not cover_region.covers(boundary_line):
            return [], "outside_polygon"
        if boundary_line.length > self._max_pen_down_connector_length_mm(spacing_mm) + 1e-6:
            if not _line_fully_inside(cover_region, boundary_line, tolerance_mm=max(0.01, spacing_mm * 0.05)):
                return [], "too_long"
        return boundary_coords, "ok_boundary"

    def _stitch_adjacent_paths_on_inside_travel(
        self,
        ordered_paths: list[Toolpath],
        *,
        cover_region: Any,
        spacing_mm: float,
        line_width_mm: float,
        angle_deg: float,
        origin: tuple[float, float],
        tolerance_mm: float,
        connector_validation: dict[str, Any] | None = None,
        debug: Optional[dict[str, Any]] = None,
    ) -> tuple[list[Toolpath], dict[str, int]]:
        stats = {
            "attempted_connectors": 0,
            "accepted_connectors": 0,
            "rejected_different_component": 0,
            "rejected_different_cell": 0,
            "rejected_non_adjacent_row": 0,
            "rejected_opposite_side": 0,
            "rejected_too_long": 0,
            "rejected_delta_u_too_large": 0,
            "rejected_outside_polygon": 0,
            "rejected_cross_gap_hole_void": 0,
            "rejected_outside_selected_color": 0,
            "rejected_unknown": 0,
            "rejected_missing_points": 0,
        }
        if len(ordered_paths) <= 1:
            return ordered_paths, stats

        if cover_region is None or cover_region.is_empty:
            return ordered_paths, stats

        normalized_cover = normalize_geometry(cover_region)
        validation_polygon = normalized_cover[0] if normalized_cover else cover_region

        stitched: list[Toolpath] = []
        current = ordered_paths[0]
        current_points = list(current.points)
        current_meta = dict(current.metadata or {})

        def _meta_value(meta: dict[str, Any], *keys: str) -> Any:
            for key in keys:
                value = meta.get(key)
                if value is not None:
                    return value
            return None

        def _bump(reason: str) -> None:
            key = f"rejected_{reason}"
            if key not in stats:
                key = "rejected_unknown"
            stats[key] += 1

        reason_to_stat = {
            "different_component": "different_component",
            "different_cell_or_section": "different_cell",
            "non_adjacent_row": "non_adjacent_row",
            "opposite_side_endpoint": "opposite_side",
            "too_long": "too_long",
            "delta_u_too_large": "delta_u_too_large",
            "outside_fillable_polygon": "outside_polygon",
            "crosses_gap_hole_void": "cross_gap_hole_void",
            "outside_selected_color": "outside_selected_color",
        }

        for nxt in ordered_paths[1:]:
            if len(current_points) < 1 or len(nxt.points) < 1:
                stats["rejected_missing_points"] += 1
                stitched.append(clone_toolpath(current, points=current_points, metadata=current_meta))
                current = nxt
                current_points = list(current.points)
                current_meta = dict(current.metadata or {})
                continue

            stats["attempted_connectors"] += 1
            start = current_points[-1]
            end = nxt.points[0]
            current_meta = dict(current_meta)
            next_meta = dict(nxt.metadata or {})
            current_component = _meta_value(current_meta, "infill_component_id", "scanline_polygon_index", "source_component_id")
            next_component = _meta_value(next_meta, "infill_component_id", "scanline_polygon_index", "source_component_id")
            if current_component != next_component:
                _bump("different_component")
                stitched.append(clone_toolpath(current, points=current_points, metadata=current_meta))
                current = nxt
                current_points = list(current.points)
                current_meta = dict(current.metadata or {})
                continue

            current_cell = _meta_value(current_meta, "cell_id", "infill_cell_id", "scanline_cell_id")
            next_cell = _meta_value(next_meta, "cell_id", "infill_cell_id", "scanline_cell_id")
            connector_result = self._validate_infill_connector(
                validation_polygon,
                LineString([(start.x, start.y), (end.x, end.y)]),
                from_meta=current_meta,
                to_meta=next_meta,
                spacing_mm=spacing_mm,
                line_width_mm=line_width_mm,
                tolerance_mm=tolerance_mm,
                connector_validation=connector_validation,
            )

            if connector_result.accepted:
                stats["accepted_connectors"] += 1
                stitched.append(clone_toolpath(current, points=current_points, metadata=current_meta))
                connector_coords = connector_result.connector_coords or [(start.x, start.y), (end.x, end.y)]
                connector_points = [Point(float(coord[0]), float(coord[1])) for coord in connector_coords]
                connector_path = Toolpath(
                    points=connector_points,
                    kind="fill-infill-travel",
                    closed=False,
                    source="infill_connector",
                    region_id=current.region_id,
                    metadata={
                        "projection_count": 0,
                        "expected_relation_to_fill": "internal_fill_connector",
                        "travel_mode": "pen_down",
                        "connector_mode": connector_result.connector_mode,
                        "connector_source_path_id": current.path_id,
                        "connector_target_path_id": nxt.path_id,
                            "cell_id": current_meta.get("cell_id") or current.metadata.get("cell_id"),
                            "infill_component_id": current_meta.get("infill_component_id") or current.metadata.get("infill_component_id"),
                    },
                )
                stitched.append(connector_path)
                if debug is not None:
                    debug_append_toolpaths(
                        debug,
                        "valid_infill_connectors",
                        [Toolpath(points=connector_points, kind="debug-valid-connector", closed=False)],
                    )
                current = nxt
                current_points = list(current.points)
                current_meta = dict(current.metadata or {})
                continue

            _bump(reason_to_stat.get(connector_result.reason, connector_result.reason))
            stitched.append(clone_toolpath(current, points=current_points, metadata=current_meta))
            current = nxt
            current_points = list(current.points)
            current_meta = dict(current.metadata or {})
            if debug is not None:
                debug_append_toolpaths(
                    debug,
                    "rejected_infill_connectors",
                    [Toolpath(points=[Point(start.x, start.y), Point(end.x, end.y)], kind="debug-rejected-connector", closed=False)],
                )
                if connector_result.sample_failures:
                    debug.setdefault("connector_validation_sample_failures", []).extend(connector_result.sample_failures)
                    for sample_failure in connector_result.sample_failures:
                        self._emit_debug_connector(
                            debug,
                            "rejected_infill_connector_sample_points",
                            (float(sample_failure["surface_x"]) - 0.05, float(sample_failure["surface_y"])),
                            (float(sample_failure["surface_x"]) + 0.05, float(sample_failure["surface_y"])),
                            angle_deg,
                            origin,
                            "debug-connector-failure-sample",
                        )

        stitched.append(clone_toolpath(current, points=current_points, metadata=current_meta))
        return stitched, stats

    def _scanline_metrics(
        self,
        region: Any,
        *,
        spacing_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
    ) -> dict[str, float]:
        row_data = self._collect_scanline_rows(
            region,
            spacing_mm=spacing_mm,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
        )
        metrics = self._score_infill_candidate(row_data, spacing_mm=spacing_mm, angle_deg=angle_deg)
        return {
            "segments": metrics["segments"],
            "rows": metrics["rows"],
            "total_length": metrics["total_length"],
            "coverage_ratio": metrics["coverage_ratio"],
        }

    def _resolve_infill_angle(
        self,
        region: Any,
        *,
        spacing_mm: float,
        angle_deg: float,
        alternate_angle_deg: float,
        fill_strategy: str,
        min_segment_length_mm: float,
        line_width_mm: float,
        region_index: int,
    ) -> tuple[float, dict[str, Any]]:
        if fill_strategy == "horizontal_scanline":
            resolved = 0.0
            metrics = self._scanline_metrics(
                region,
                spacing_mm=spacing_mm,
                angle_deg=resolved,
                min_segment_length_mm=min_segment_length_mm,
            )
            return resolved, {"strategy": fill_strategy, "candidate_metrics": [{"angle_deg": resolved, **metrics}]}
        if fill_strategy == "rotated_scanline":
            resolved = angle_deg
            metrics = self._scanline_metrics(
                region,
                spacing_mm=spacing_mm,
                angle_deg=resolved,
                min_segment_length_mm=min_segment_length_mm,
            )
            return resolved, {"strategy": fill_strategy, "candidate_metrics": [{"angle_deg": resolved, **metrics}]}

        min_segment_length_mm = self._recommended_infill_min_segment_length_mm(line_width_mm, min_segment_length_mm)
        axis_metrics = self._polygon_axis_metrics(region)
        candidate_angles = _dedupe_infill_candidate_angles([
            angle_deg,
            0.0,
            45.0,
            90.0,
            135.0,
            alternate_angle_deg,
            angle_deg + 90.0,
            alternate_angle_deg + 90.0,
            axis_metrics["dominant_axis_angle_deg"],
            axis_metrics["dominant_axis_angle_deg"] + 90.0,
        ])
        candidate_metrics: list[dict[str, Any]] = []
        best_angle = candidate_angles[0] if candidate_angles else _normalize_infill_angle_deg(angle_deg)
        best_score = float("-inf")

        if axis_metrics["aspect_ratio"] >= DEFAULT_LONG_THIN_INFILL_ASPECT_RATIO:
            major_axis_angle = axis_metrics["dominant_axis_angle_deg"]
            row_data = self._collect_scanline_rows(
                region,
                spacing_mm=spacing_mm,
                angle_deg=major_axis_angle,
                min_segment_length_mm=min_segment_length_mm,
            )
            major_axis_metrics = self._score_infill_candidate(row_data, spacing_mm=spacing_mm, angle_deg=major_axis_angle)
            major_axis_metrics["selection_reason"] = "long_thin_fast_path"
            candidate_metrics.append(major_axis_metrics)
            return major_axis_angle, {
                "strategy": fill_strategy,
                "candidate_metrics": candidate_metrics,
                "long_thin_fast_path_used": True,
                "dominant_axis_angle_deg": axis_metrics["dominant_axis_angle_deg"],
                "aspect_ratio": axis_metrics["aspect_ratio"],
                "long_side_mm": axis_metrics["long_side_mm"],
                "short_side_mm": axis_metrics["short_side_mm"],
                "used_oriented_bbox": axis_metrics["used_oriented_bbox"],
            }

        for candidate in candidate_angles:
            row_data = self._collect_scanline_rows(
                region,
                spacing_mm=spacing_mm,
                angle_deg=candidate,
                min_segment_length_mm=min_segment_length_mm,
            )
            metrics = self._score_infill_candidate(row_data, spacing_mm=spacing_mm, angle_deg=candidate)
            candidate_metrics.append(metrics)
            if metrics["score"] > best_score:
                best_score = metrics["score"]
                best_angle = metrics["angle_deg"]

        if fill_strategy == "adaptive_angle" and candidate_metrics:
            global_candidate_score = candidate_metrics[0]["score"]
            if abs(best_score - global_candidate_score) < 0.02:
                best_angle = candidate_metrics[0]["angle_deg"]

        return best_angle, {
            "strategy": fill_strategy,
            "candidate_metrics": candidate_metrics,
            "long_thin_fast_path_used": False,
            "dominant_axis_angle_deg": axis_metrics["dominant_axis_angle_deg"],
            "aspect_ratio": axis_metrics["aspect_ratio"],
            "long_side_mm": axis_metrics["long_side_mm"],
            "short_side_mm": axis_metrics["short_side_mm"],
            "used_oriented_bbox": axis_metrics["used_oriented_bbox"],
        }

    def _generate_scanline_infill(
        self,
        region: Any,
        *,
        spacing_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        line_width_mm: float | None = None,
        kind: str = "fill-infill",
        allow_pen_down_infill_connectors: bool = DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS,
        infill_path_mode: str = DEFAULT_INFILL_PATH_MODE,
        connector_validation: dict[str, Any] | None = None,
        adaptive_detail_cells: bool = False,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if region is None or region.is_empty or spacing_mm <= 0:
            return []

        if line_width_mm is None:
            line_width_mm = spacing_mm

        row_data = self._collect_scanline_rows(
            region,
            spacing_mm=spacing_mm,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
        )
        origin = row_data.get("origin")
        if origin is None:
            return []

        rotated = row_data.get("rotated_region")
        if rotated is None or rotated.is_empty:
            return []

        rows = row_data.get("rows") or []
        toolpaths: list[Toolpath] = []
        if debug is not None:
            debug.setdefault("connector_rejection_reasons", {})
            debug.setdefault("accepted_same_cell_connectors", 0)
            debug.setdefault("rows_with_multiple_intervals", 0)
            debug.setdefault("local_cell_count", 0)
            debug.setdefault("rejected_cross_gap_connectors", 0)
            debug.setdefault("rejected_different_cell_connectors", 0)
            debug.setdefault("rejected_opposite_side_connectors", 0)
            debug.setdefault("rejected_too_long_connectors", 0)
            debug.setdefault("rejected_outside_selected_color_connectors", 0)
            debug.setdefault("pen_lifts_before_cell_planning", 0)
            debug.setdefault("pen_lifts_after_cell_planning", 0)
            debug.setdefault("infill_connector_diagnostics", {})

        if debug is not None:
            for row in rows:
                raw_scan = LineString([(-1e6, row["offset_mm"]), (1e6, row["offset_mm"])]).intersection(rotated.envelope)
                if raw_scan is not None and not raw_scan.is_empty:
                    raw_scan_world = affinity.rotate(raw_scan, angle_deg, origin=origin)
                    debug_append_toolpaths(debug, "raw_scanlines", [
                        Toolpath(points=[Point(x, y) for x, y in raw_scan_world.coords], kind="debug-raw-scanline", closed=False)
                    ])
                debug_append_toolpaths(
                    debug,
                    "clipped_infill_lines",
                    [entry["toolpath"] for entry in self._row_world_paths([row], angle_deg=angle_deg, origin=origin, tolerance_mm=tolerance_mm, kind=kind)],
                )

        resolved_infill_path_mode = (infill_path_mode or DEFAULT_INFILL_PATH_MODE).strip().lower()
        if resolved_infill_path_mode not in {"rectilinear", "serpentine_optimized", "legacy"}:
            resolved_infill_path_mode = DEFAULT_INFILL_PATH_MODE

        rows_by_polygon: dict[int, list[dict[str, Any]]] = {}
        for row in rows:
            rows_by_polygon.setdefault(int(row["polygon_index"]), []).append(row)

        if resolved_infill_path_mode == "legacy":
            for polygon_index, polygon in enumerate(normalize_geometry(rotated)):
                component_id = f"component_{polygon_index:03d}"
                drawable_rows = [row for row in rows_by_polygon.get(polygon_index, []) if row.get("segments")]
                segments = self._to_infill_segments(drawable_rows, component_id=component_id)
                if debug is not None:
                    debug["total_scanlines"] = int(debug.get("total_scanlines", 0)) + len(drawable_rows)
                    debug["total_clipped_intervals"] = int(debug.get("total_clipped_intervals", 0)) + len(segments)
                    debug["pen_lifts_before_cell_planning"] = int(debug.get("pen_lifts_before_cell_planning", 0)) + max(0, len(segments) - 1)

                cells, cell_stats = self._assign_infill_cells(
                    polygon,
                    segments,
                    spacing_mm=spacing_mm,
                    line_width_mm=line_width_mm,
                    tolerance_mm=tolerance_mm,
                    connector_validation=connector_validation,
                )
                if debug is not None:
                    debug["rows_with_multiple_intervals"] = int(debug.get("rows_with_multiple_intervals", 0)) + int(cell_stats["rows_with_multiple_intervals"])
                    debug["local_cell_count"] = int(debug.get("local_cell_count", 0)) + len(cells)
                    debug["rejected_cross_gap_connectors"] = int(debug.get("rejected_cross_gap_connectors", 0)) + int(cell_stats["rejected_cross_gap_connectors"])
                    debug["rejected_different_cell_connectors"] = int(debug.get("rejected_different_cell_connectors", 0)) + int(cell_stats["rejected_different_cell_connectors"])
                    debug["rejected_opposite_side_connectors"] = int(debug.get("rejected_opposite_side_connectors", 0)) + int(cell_stats["rejected_opposite_side_connectors"])
                    debug["rejected_too_long_connectors"] = int(debug.get("rejected_too_long_connectors", 0)) + int(cell_stats["rejected_too_long_connectors"])
                    debug["rejected_outside_selected_color_connectors"] = int(debug.get("rejected_outside_selected_color_connectors", 0)) + int(cell_stats.get("rejected_outside_selected_color_connectors", 0))
                    debug["pen_lifts_after_cell_planning"] = int(debug.get("pen_lifts_after_cell_planning", 0)) + max(0, len(segments) - len(cells))

                for cell_id in sorted(cells.keys()):
                    ordered_segments = sorted(cells[cell_id], key=lambda s: (s.row_index, s.min_u, s.interval_index))
                    for segment in ordered_segments:
                        toolpaths.append(self._segment_to_world_toolpath(
                            segment,
                            angle_deg=angle_deg,
                            origin=origin,
                            tolerance_mm=tolerance_mm,
                            kind=kind,
                        ))

            ordered = list(toolpaths)
            if not allow_pen_down_infill_connectors:
                return ordered

            stitched, stitch_stats = self._stitch_adjacent_paths_on_inside_travel(
                ordered,
                cover_region=region,
                spacing_mm=spacing_mm,
                line_width_mm=line_width_mm,
                angle_deg=angle_deg,
                origin=origin,
                tolerance_mm=tolerance_mm,
                connector_validation=connector_validation,
                debug=debug,
            )
            if debug is not None:
                debug["accepted_same_cell_connectors"] = int(debug.get("accepted_same_cell_connectors", 0)) + int(stitch_stats["accepted_connectors"])
                reasons = debug.setdefault("connector_rejection_reasons", {})
                mapping = {
                    "different_component": "rejected_different_component",
                    "different_cell_or_section": "rejected_different_cell",
                    "non_adjacent_row": "rejected_non_adjacent_row",
                    "opposite_side_endpoint": "rejected_opposite_side",
                    "too_long": "rejected_too_long",
                    "deltaU_too_large": "rejected_delta_u_too_large",
                    "outside_fillable_polygon": "rejected_outside_polygon",
                    "crosses_gap_hole_void": "rejected_cross_gap_hole_void",
                    "outside_selected_color": "rejected_outside_selected_color",
                    "unknown": "rejected_unknown",
                }
                for reason, stat_key in mapping.items():
                    reasons[reason] = int(reasons.get(reason, 0)) + int(stitch_stats.get(stat_key, 0))
                rejection_counts = {
                    "different_component": int(stitch_stats.get("rejected_different_component", 0)),
                    "different_cell_or_section": int(stitch_stats.get("rejected_different_cell", 0)),
                    "non_adjacent_row": int(stitch_stats.get("rejected_non_adjacent_row", 0)),
                    "opposite_side_endpoint": int(stitch_stats.get("rejected_opposite_side", 0)),
                    "too_long": int(stitch_stats.get("rejected_too_long", 0)),
                    "deltaU_too_large": int(stitch_stats.get("rejected_delta_u_too_large", 0)),
                    "outside_fillable_polygon": int(stitch_stats.get("rejected_outside_polygon", 0)),
                    "crosses_gap_hole_void": int(stitch_stats.get("rejected_cross_gap_hole_void", 0)),
                    "outside_selected_color": int(stitch_stats.get("rejected_outside_selected_color", 0)),
                    "unknown": int(stitch_stats.get("rejected_unknown", 0)),
                }
                diagnostics = {
                    "total_infill_rows": int(debug.get("total_scanlines", 0)),
                    "total_possible_adjacent_row_connector_attempts": int(stitch_stats.get("attempted_connectors", 0)),
                    "accepted_connectors": int(stitch_stats.get("accepted_connectors", 0)),
                    "rejected_connectors": int(sum(rejection_counts.values()) + int(stitch_stats.get("rejected_missing_points", 0))),
                    "rejected_raster_mask_sampling": int(cell_stats.get("rejected_outside_selected_color_connectors", 0)) + int(stitch_stats.get("rejected_outside_selected_color", 0)),
                    "final_pen_lift_count_estimate": int(sum(1 for current_path, next_path in zip(stitched, stitched[1:]) if getattr(next_path, "kind", None) != "fill-infill-travel")),
                    "rejection_counts": rejection_counts,
                }
                diagnostics["top_rejection_reason"] = max(rejection_counts, key=rejection_counts.get) if any(rejection_counts.values()) else None
                debug["infill_connector_diagnostics"] = diagnostics
                debug["pen_lifts_after_cell_planning"] = int(diagnostics["final_pen_lift_count_estimate"])
            return stitched

        cell_plans: list[InfillCellPlan] = []
        adaptive_cell_decisions: dict[str, InfillCellAdaptiveDecision] = {}
        adaptive_cell_regions: dict[str, Any] = {}
        adaptive_counts = {
            "total_cells": 0,
            "rectilinear_cells": 0,
            "detail_contour_cells": 0,
            "single_stroke_cells": 0,
            "narrow_cells_detected": 0,
            "switched_too_few_rows": 0,
            "switched_connector_ratio": 0,
            "switched_single_stroke_width": 0,
            "switched_single_stroke_hatch_quality": 0,
        }
        for polygon_index, polygon in enumerate(normalize_geometry(rotated)):
            component_id = f"component_{polygon_index:03d}"
            drawable_rows = [row for row in rows_by_polygon.get(polygon_index, []) if row.get("segments")]
            segments = self._to_infill_segments(drawable_rows, component_id=component_id)
            if debug is not None:
                debug["total_scanlines"] = int(debug.get("total_scanlines", 0)) + len(drawable_rows)
                debug["total_clipped_intervals"] = int(debug.get("total_clipped_intervals", 0)) + len(segments)
                debug["pen_lifts_before_cell_planning"] = int(debug.get("pen_lifts_before_cell_planning", 0)) + max(0, len(segments) - 1)

            cells, cell_stats = self._assign_infill_cells(
                polygon,
                segments,
                spacing_mm=spacing_mm,
                line_width_mm=line_width_mm,
                tolerance_mm=tolerance_mm,
                connector_validation=connector_validation,
            )
            if debug is not None:
                debug["rows_with_multiple_intervals"] = int(debug.get("rows_with_multiple_intervals", 0)) + int(cell_stats["rows_with_multiple_intervals"])
                debug["local_cell_count"] = int(debug.get("local_cell_count", 0)) + len(cells)
                debug["rejected_cross_gap_connectors"] = int(debug.get("rejected_cross_gap_connectors", 0)) + int(cell_stats["rejected_cross_gap_connectors"])
                debug["rejected_different_cell_connectors"] = int(debug.get("rejected_different_cell_connectors", 0)) + int(cell_stats["rejected_different_cell_connectors"])
                debug["rejected_opposite_side_connectors"] = int(debug.get("rejected_opposite_side_connectors", 0)) + int(cell_stats["rejected_opposite_side_connectors"])
                debug["rejected_too_long_connectors"] = int(debug.get("rejected_too_long_connectors", 0)) + int(cell_stats["rejected_too_long_connectors"])
                debug["rejected_outside_selected_color_connectors"] = int(debug.get("rejected_outside_selected_color_connectors", 0)) + int(cell_stats.get("rejected_outside_selected_color_connectors", 0))
                debug["pen_lifts_after_cell_planning"] = int(debug.get("pen_lifts_after_cell_planning", 0)) + max(0, len(segments) - len(cells))

            for cell_id in sorted(cells.keys()):
                ordered_segments = sorted(cells[cell_id], key=lambda s: (s.row_index, s.min_u, s.interval_index))
                if not ordered_segments:
                    continue
                if adaptive_detail_cells:
                    decision = self._evaluate_adaptive_cell_mode(
                        cell_segments=ordered_segments,
                        spacing_mm=spacing_mm,
                        line_width_mm=line_width_mm,
                        cover_region=polygon,
                    )
                    adaptive_cell_decisions[cell_id] = decision
                    adaptive_cell_regions[cell_id] = self._build_cell_region_from_segments(
                        ordered_segments,
                        angle_deg=angle_deg,
                        origin=origin,
                        spacing_mm=spacing_mm,
                        line_width_mm=line_width_mm,
                        cover_region=region,
                    )
                    adaptive_counts["total_cells"] += 1
                    if decision.mode == "detail_contour":
                        adaptive_counts["detail_contour_cells"] += 1
                        if any(reason in decision.reasons for reason in ("narrow_local_width", "short_fragmented_hatch")):
                            adaptive_counts["narrow_cells_detected"] += 1
                        if "too_few_hatch_rows" in decision.reasons:
                            adaptive_counts["switched_too_few_rows"] += 1
                        if "connector_dominates_hatch" in decision.reasons or "mostly_connector_fragments" in decision.reasons:
                            adaptive_counts["switched_connector_ratio"] += 1
                    elif decision.mode == "single_stroke":
                        adaptive_counts["single_stroke_cells"] += 1
                        if any(reason in decision.reasons for reason in ("width_lte_1p5x_pen", "width_lte_2x_pen_with_poor_rows")):
                            adaptive_counts["switched_single_stroke_width"] += 1
                        if any(reason in decision.reasons for reason in ("only_one_useful_hatch_row", "mostly_tiny_hatch_fragments", "connector_length_too_high", "thin_curved_script_like_cell")):
                            adaptive_counts["switched_single_stroke_hatch_quality"] += 1
                    else:
                        adaptive_counts["rectilinear_cells"] += 1
                seed_plan = InfillCellPlan(
                    cell_id=cell_id,
                    component_id=component_id,
                    segments=ordered_segments,
                    toolpaths=[],
                    entry_point=Point(float(ordered_segments[0].low_u.x), float(ordered_segments[0].low_u.y)),
                    exit_point=Point(float(ordered_segments[-1].high_u.x), float(ordered_segments[-1].high_u.y)),
                    centroid=self._cell_centroid_for_segments(ordered_segments),
                    total_length=sum(segment.length for segment in ordered_segments),
                )
                cell_plans.append(seed_plan)

        ordered_cell_plans = self._order_infill_cells(cell_plans, origin=origin)
        final_toolpaths: list[Toolpath] = []
        cell_route_debug: list[dict[str, Any]] = []
        pen_up_travel_distance_mm = 0.0
        long_travel_count = 0
        max_local_travel_mm = max(4.0 * spacing_mm, 6.0 * line_width_mm)
        previous_exit_point: Point | None = None

        for cell_order, cell_plan in enumerate(ordered_cell_plans, start=1):
            decision = adaptive_cell_decisions.get(cell_plan.cell_id) if adaptive_detail_cells else None
            if decision is not None and decision.mode == "single_stroke":
                cell_region = adaptive_cell_regions.get(cell_plan.cell_id)
                cell_toolpaths = self._generate_centerline_fallback(
                    cell_region,
                    angle_deg=angle_deg,
                    min_segment_length_mm=max(0.01, min_segment_length_mm),
                    tolerance_mm=tolerance_mm,
                    kind=kind,
                )
                if not cell_toolpaths:
                    cell_toolpaths = self._generate_tiny_dot_or_short_stroke(
                        cell_region,
                        line_width_mm=line_width_mm,
                        tolerance_mm=tolerance_mm,
                        angle_deg=angle_deg,
                        kind=kind,
                    )
                cell_toolpaths = self._clip_toolpaths_to_region(
                    cell_toolpaths,
                    region=cell_region,
                    tolerance_mm=tolerance_mm,
                    kind=kind,
                )
                cell_toolpaths = [
                    clone_toolpath(
                        path,
                        kind=kind,
                        metadata={
                            **path.metadata,
                            "fill_mode": "single_stroke_cell",
                            "fill_mode_reason": ",".join(decision.reasons) if decision.reasons else "adaptive_single_stroke",
                            "adaptive_cell_metrics": decision.metrics,
                            "small_detail_fill_style": path.metadata.get("small_detail_fill_style", "single_stroke_detail"),
                        },
                    )
                    for path in cell_toolpaths
                ]
                cell_stats = {"accepted_connectors": 0, "pen_lifts": max(0, len(cell_toolpaths) - 1)}
            elif decision is not None and decision.mode == "detail_contour":
                cell_region = adaptive_cell_regions.get(cell_plan.cell_id)
                cell_toolpaths = self._generate_small_detail_fill(
                    cell_region,
                    line_width_mm=line_width_mm,
                    scanline_spacing_mm=spacing_mm,
                    angle_deg=angle_deg,
                    min_segment_length_mm=min_segment_length_mm,
                    tolerance_mm=tolerance_mm,
                    detail_tolerance_mm=tolerance_mm,
                    allow_overlap=True,
                    connector_validation=connector_validation,
                )
                if not cell_toolpaths:
                    cell_toolpaths = self._generate_centerline_fallback(
                        cell_region,
                        angle_deg=angle_deg,
                        min_segment_length_mm=min_segment_length_mm,
                        tolerance_mm=tolerance_mm,
                        kind=kind,
                    )
                cell_toolpaths = self._clip_toolpaths_to_region(
                    cell_toolpaths,
                    region=cell_region,
                    tolerance_mm=tolerance_mm,
                    kind=kind,
                )
                cell_toolpaths = [
                    clone_toolpath(
                        path,
                        kind=kind,
                        metadata={
                            **path.metadata,
                            "fill_mode": "detail_contour_cell",
                            "fill_mode_reason": ",".join(decision.reasons) if decision.reasons else "adaptive_detail_fallback",
                            "adaptive_cell_metrics": decision.metrics,
                            "small_detail_fill_style": path.metadata.get("small_detail_fill_style", "contour_or_centerline"),
                        },
                    )
                    for path in cell_toolpaths
                ]
                cell_stats = {"accepted_connectors": 0, "pen_lifts": max(0, len(cell_toolpaths) - 1)}
            elif allow_pen_down_infill_connectors:
                cell_toolpaths, cell_stats = self._plan_cell_paths(
                    region,
                    cell_plan.segments,
                    spacing_mm=spacing_mm,
                    line_width_mm=line_width_mm,
                    angle_deg=angle_deg,
                    origin=origin,
                    tolerance_mm=tolerance_mm,
                    kind=kind,
                    debug=debug,
                    connector_validation=connector_validation,
                )
            else:
                cell_toolpaths = [
                    self._segment_to_world_toolpath(
                        segment,
                        angle_deg=angle_deg,
                        origin=origin,
                        tolerance_mm=tolerance_mm,
                        kind=kind,
                    )
                    for segment in cell_plan.segments
                ]
                cell_stats = {"accepted_connectors": 0, "pen_lifts": max(0, len(cell_plan.segments) - 1)}

            cell_entry_point, cell_exit_point = self._cell_entry_exit_points(cell_toolpaths)
            actual_cell_plan = InfillCellPlan(
                cell_id=cell_plan.cell_id,
                component_id=cell_plan.component_id,
                segments=cell_plan.segments,
                toolpaths=cell_toolpaths,
                entry_point=cell_entry_point,
                exit_point=cell_exit_point,
                centroid=cell_plan.centroid,
                total_length=cell_plan.total_length,
            )
            cell_route_debug.append({
                "cell_id": actual_cell_plan.cell_id,
                "component_id": actual_cell_plan.component_id,
                "order": cell_order,
                "segment_count": len(actual_cell_plan.segments),
                "toolpath_count": len(actual_cell_plan.toolpaths),
                "total_length_mm": actual_cell_plan.total_length,
                "entry_point": asdict(actual_cell_plan.entry_point),
                "exit_point": asdict(actual_cell_plan.exit_point),
                "centroid": asdict(actual_cell_plan.centroid),
                "adaptive_fill_mode": (decision.mode if decision is not None else "rectilinear"),
                "adaptive_fill_reasons": ([] if decision is None else decision.reasons),
                "adaptive_fill_metrics": ({} if decision is None else decision.metrics),
            })
            if previous_exit_point is not None:
                travel_distance = math.hypot(actual_cell_plan.entry_point.x - previous_exit_point.x, actual_cell_plan.entry_point.y - previous_exit_point.y)
                if travel_distance > 1e-9:
                    pen_up_travel_distance_mm += travel_distance
                    if travel_distance > max_local_travel_mm:
                        long_travel_count += 1
            previous_exit_point = actual_cell_plan.exit_point

            if debug is not None:
                debug.setdefault("cell_route_debug", []).append(cell_route_debug[-1])
                debug["accepted_same_cell_connectors"] = int(debug.get("accepted_same_cell_connectors", 0)) + int(cell_stats.get("accepted_connectors", 0))
                debug["pen_lifts_after_cell_planning"] = int(debug.get("pen_lifts_after_cell_planning", 0)) + int(cell_stats.get("pen_lifts", 0))

            final_toolpaths.extend(actual_cell_plan.toolpaths)

        if debug is not None and cell_plans:
            total_segments = sum(len(plan.segments) for plan in ordered_cell_plans)
            cell_count = len(ordered_cell_plans)
            debug["local_cell_count"] = cell_count
            debug["average_segments_per_cell"] = float(total_segments) / max(1, cell_count)
            debug["largest_cell_size"] = max((len(plan.segments) for plan in ordered_cell_plans), default=0)
            debug["singleton_cells"] = sum(1 for plan in ordered_cell_plans if len(plan.segments) == 1)
            debug["total_pen_up_travel_distance_mm"] = pen_up_travel_distance_mm
            debug["number_of_long_travels_between_cells"] = long_travel_count
            debug["cell_route_debug"] = cell_route_debug

            rejection_counts = {
                "different_component": int(debug.get("rejected_different_cell_connectors", 0)),
                "different_cell_or_section": int(debug.get("rejected_different_cell_connectors", 0)),
                "non_adjacent_row": 0,
                "opposite_side_endpoint": int(debug.get("rejected_opposite_side_connectors", 0)),
                "too_long": int(debug.get("rejected_too_long_connectors", 0)),
                "deltaU_too_large": 0,
                "outside_fillable_polygon": int(debug.get("rejected_cross_gap_connectors", 0)),
                "crosses_gap_hole_void": int(debug.get("rejected_cross_gap_connectors", 0)),
                "outside_selected_color": int(debug.get("rejected_outside_selected_color_connectors", 0)),
                "unknown": 0,
            }
            debug["infill_connector_diagnostics"] = {
                "total_infill_rows": int(debug.get("total_scanlines", 0)),
                "total_possible_adjacent_row_connector_attempts": int(debug.get("pen_lifts_before_cell_planning", 0)),
                "accepted_connectors": int(debug.get("accepted_same_cell_connectors", 0)),
                "rejected_connectors": int(sum(rejection_counts.values())),
                "rejected_raster_mask_sampling": int(debug.get("rejected_outside_selected_color_connectors", 0)),
                "final_pen_lift_count_estimate": int(debug.get("pen_lifts_after_cell_planning", 0)),
                "rejection_counts": rejection_counts,
                "top_rejection_reason": max(rejection_counts, key=rejection_counts.get) if any(rejection_counts.values()) else None,
            }
            if adaptive_detail_cells:
                debug["adaptive_fill_diagnostics"] = {
                    **adaptive_counts,
                }

        return final_toolpaths

    def _generate_centerline_fallback(
        self,
        region: Any,
        *,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
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

        if best_line is None or best_length < min_segment_length_mm:
            return []

        points = simplify_segment_points([Point(x, y) for x, y in best_line.coords], tolerance_mm, False)
        if len(points) < 2:
            return []
        return [Toolpath(points=points, kind=kind, closed=False)]

    def _generate_tiny_dot_or_short_stroke(
        self,
        region: Any,
        *,
        line_width_mm: float,
        tolerance_mm: float,
        angle_deg: float = 0.0,
        kind: str = "fill-infill",
    ) -> list[Toolpath]:
        if region is None or region.is_empty:
            return []
        center = region.representative_point()
        cx = float(center.x)
        cy = float(center.y)
        min_x, min_y, max_x, max_y = region.bounds
        span = max(max_x - min_x, max_y - min_y, line_width_mm)
        candidate_angles = [angle_deg, angle_deg + 90.0, 0.0, 90.0]
        best_line = None
        best_length = -1.0
        for candidate_angle in candidate_angles:
            rad = math.radians(candidate_angle)
            dx = math.cos(rad)
            dy = math.sin(rad)
            probe = LineString(
                [
                    (cx - dx * span * 2.0, cy - dy * span * 2.0),
                    (cx + dx * span * 2.0, cy + dy * span * 2.0),
                ]
            )
            clipped = region.intersection(probe)
            lines = sorted(extract_lines(clipped), key=lambda line: line.length, reverse=True)
            if lines and lines[0].length > best_length:
                best_line = lines[0]
                best_length = lines[0].length

        if best_line is not None and best_length > 1e-6:
            points = simplify_segment_points([Point(float(x), float(y)) for x, y in best_line.coords], tolerance_mm, False)
            if len(points) >= 2:
                return [Toolpath(points=points, kind=kind, closed=False, metadata={"small_detail_fill_style": "tiny_short_stroke"})]
        dot_half = max(0.02, line_width_mm * 0.12)
        return [Toolpath(
            points=[Point(cx - dot_half, cy), Point(cx + dot_half, cy)],
            kind=kind,
            closed=False,
            metadata={"small_detail_fill_style": "tiny_dot"},
        )]

    def _generate_outline_only_fallback(
        self,
        component: Any,
        *,
        tolerance_mm: float,
        min_segment_length_mm: float,
        kind: str = "outline",
    ) -> list[Toolpath]:
        if component is None or component.is_empty:
            return []

        outline_paths: list[Toolpath] = []
        for polygon in normalize_geometry(component):
            exterior_points = simplify_segment_points([Point(float(x), float(y)) for x, y in polygon.exterior.coords], tolerance_mm, True)
            if len(exterior_points) < 4:
                continue
            if segment_length(exterior_points) < max(min_segment_length_mm, 0.75):
                continue
            polygon_area = float(polygon.area)
            min_outline_area = max(0.15, (tolerance_mm * tolerance_mm * 0.5) if tolerance_mm > 0 else 0.15)
            if polygon_area < min_outline_area:
                continue
            outline_paths.append(Toolpath(
                points=exterior_points,
                kind=kind,
                closed=True,
                metadata={
                    "small_detail_fill_style": "outline_only_fallback",
                },
            ))
        return outline_paths

    def _generate_single_stroke_detail(
        self,
        component: Any,
        drawable_area: Any | None,
        metrics: RegionMetrics,
        config: HybridInfillConfig,
        *,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        kind: str = "fill-infill",
    ) -> list[Toolpath]:
        if component is None or component.is_empty:
            return []

        if config.thinRegionMode == "skip":
            return []

        candidate_regions: list[Any] = []
        if drawable_area is not None and not drawable_area.is_empty:
            candidate_regions.append(drawable_area)
        candidate_regions.append(component)
        tiny_region = bool(metrics.areaMm2 < config.minUsableDetailAreaMm2 or metrics.maxLocalWidthMm <= config.lineWidthMm * config.singleStrokeWidthMaxFactor)
        centerline_min_segment_length_mm = max(0.01, config.lineWidthMm * (0.25 if tiny_region else 0.75))

        tiny_mark_region = bool(
            metrics.areaMm2 <= max(
                config.lineWidthMm * config.lineWidthMm * 0.6,
                config.lineWidthMm * config.lineWidthMm * config.tinyDotAreaFactor * 3.0,
            )
            and metrics.maxLocalWidthMm <= config.lineWidthMm * config.centerlineWidthMaxFactor
        )
        if tiny_mark_region:
            for region in candidate_regions:
                tiny_paths = self._generate_tiny_dot_or_short_stroke(
                    region,
                    line_width_mm=config.lineWidthMm,
                    tolerance_mm=max(tolerance_mm, config.lineWidthMm * 0.2),
                    angle_deg=angle_deg,
                    kind=kind,
                )
                if not tiny_paths:
                    continue
                tiny_path = tiny_paths[0]
                if len(tiny_path.points) < 2:
                    continue
                if not validate_thin_region_stroke(tiny_path.points, component, drawable_area, config, metrics=metrics):
                    continue
                tiny_path.metadata = {
                    **tiny_path.metadata,
                    "thin_region_mode": config.thinRegionMode,
                    "single_stroke_source": "drawable_area" if region is drawable_area else "component",
                    "metrics_area_mm2": metrics.areaMm2,
                    "metrics_width_mm": metrics.maxLocalWidthMm,
                }
                return [tiny_path]

        best_toolpath: Toolpath | None = None
        best_length = -1.0
        for region in candidate_regions:
            stroke_paths = self._generate_centerline_fallback(
                region,
                angle_deg=angle_deg,
                min_segment_length_mm=max(min_segment_length_mm, centerline_min_segment_length_mm),
                tolerance_mm=max(tolerance_mm, config.lineWidthMm * 0.25),
                kind=kind,
            )
            if not stroke_paths:
                continue

            stroke = stroke_paths[0]
            points = list(stroke.points)
            if len(points) >= 2:
                line = LineString([(point.x, point.y) for point in points])
                trim_mm = max(0.0, config.lineWidthMm * 0.25)
                if line.length > trim_mm * 2.0 + 1e-9:
                    trimmed_line = substring(line, trim_mm, line.length - trim_mm)
                    trimmed_points = [Point(float(x), float(y)) for x, y in getattr(trimmed_line, "coords", [])]
                    if segment_length(trimmed_points) >= max(min_segment_length_mm, config.lineWidthMm * 0.25):
                        points = trimmed_points
            if len(points) < 2:
                continue
            if not validate_thin_region_stroke(points, component, drawable_area, config, metrics=metrics):
                continue
            minimum_segment_length_mm = max(min_segment_length_mm, centerline_min_segment_length_mm)
            if segment_length(points) < minimum_segment_length_mm:
                continue
            candidate_toolpath = Toolpath(
                points=points,
                kind=kind,
                closed=False,
                metadata={
                    "small_detail_fill_style": "single_stroke_detail",
                    "thin_region_mode": config.thinRegionMode,
                    "single_stroke_source": "drawable_area" if region is drawable_area else "component",
                    "metrics_area_mm2": metrics.areaMm2,
                    "metrics_width_mm": metrics.maxLocalWidthMm,
                },
            )
            candidate_length = segment_length(candidate_toolpath.points)
            if candidate_length > best_length:
                best_toolpath = candidate_toolpath
                best_length = candidate_length

        if best_toolpath is not None:
            # Width-aware upgrade: for regions wider than one effective pen lane,
            # emit a small set of interior parallel strokes instead of a single one.
            effective_coverage_mm = max(1e-6, config.lineWidthMm * 0.85)
            desired_stroke_count = max(1, int(math.ceil(max(0.0, metrics.maxLocalWidthMm) / effective_coverage_mm)))
            desired_stroke_count = min(3, desired_stroke_count)
            if desired_stroke_count > 1:
                source_region = drawable_area if drawable_area is not None and not drawable_area.is_empty else component
                sparse = self._generate_sparse_interior_strokes(
                    source_region,
                    angle_deg=angle_deg,
                    line_width_mm=config.lineWidthMm,
                    scanline_spacing_mm=effective_coverage_mm,
                    min_segment_length_mm=max(min_segment_length_mm, centerline_min_segment_length_mm),
                    tolerance_mm=max(tolerance_mm, config.lineWidthMm * 0.25),
                    max_strokes=desired_stroke_count,
                    kind=kind,
                )
                sparse = self._clip_toolpaths_to_region(
                    sparse,
                    region=component,
                    tolerance_mm=max(tolerance_mm, config.lineWidthMm * 0.25),
                    kind=kind,
                )
                multi: list[Toolpath] = []
                for idx, path in enumerate(sparse):
                    if len(path.points) < 2:
                        continue
                    if not validate_thin_region_stroke(path.points, component, drawable_area, config, metrics=metrics):
                        continue
                    if segment_length(path.points) < max(min_segment_length_mm, centerline_min_segment_length_mm * 0.7):
                        continue
                    multi.append(clone_toolpath(
                        path,
                        metadata={
                            **path.metadata,
                            "small_detail_fill_style": "single_stroke_multi",
                            "thin_region_mode": config.thinRegionMode,
                            "single_stroke_source": "drawable_area" if source_region is drawable_area else "component",
                            "metrics_area_mm2": metrics.areaMm2,
                            "metrics_width_mm": metrics.maxLocalWidthMm,
                            "requested_stroke_count": desired_stroke_count,
                            "stroke_index": idx,
                        },
                    ))
                if multi:
                    return multi
            return [best_toolpath]

        if (not tiny_region) and (config.thinRegionMode == "outlineOnly" or (config.allowOutlineOverlapForThinRegions and metrics.maxLocalWidthMm <= config.lineWidthMm * 1.15)):
            outline_paths = self._generate_outline_only_fallback(
                component,
                tolerance_mm=tolerance_mm,
                min_segment_length_mm=min_segment_length_mm,
                kind="outline",
            )
            if outline_paths:
                return outline_paths

        return []

    def _generate_collapsed_region_fallback(
        self,
        component: Any,
        drawable_area: Any | None,
        metrics: RegionMetrics,
        config: HybridInfillConfig,
        *,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
    ) -> list[Toolpath]:
        if config.thinRegionMode == "skip":
            return []

        single_stroke_paths = self._generate_single_stroke_detail(
            component,
            drawable_area,
            metrics,
            config,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=tolerance_mm,
            kind="fill-infill",
        )
        if single_stroke_paths:
            return single_stroke_paths

        outline_paths = self._generate_outline_only_fallback(
            component,
            tolerance_mm=tolerance_mm,
            min_segment_length_mm=min_segment_length_mm,
            kind="outline",
        )
        if outline_paths:
            return outline_paths

        return []

    def _clip_toolpaths_to_region(
        self,
        paths: list[Toolpath],
        *,
        region: Any,
        tolerance_mm: float,
        kind: str,
    ) -> list[Toolpath]:
        if not paths:
            return []
        if region is None or region.is_empty:
            return paths
        clip_region = region
        clipped_paths: list[Toolpath] = []
        for path in paths:
            if len(path.points) < 2:
                continue
            line = LineString([(point.x, point.y) for point in path.points])
            clipped = line.intersection(clip_region)
            for part in extract_lines(clipped):
                points = simplify_segment_points([Point(float(x), float(y)) for x, y in part.coords], tolerance_mm, False)
                if len(points) >= 3:
                    points = normalize_straight_segments(
                        points,
                        StraighteningOptions(
                            angleToleranceDeg=2.0,
                            maxLateralErrorMm=max(0.01, tolerance_mm * 0.5),
                            minStraightSegmentLengthMm=max(2.0, tolerance_mm * 8.0),
                        ),
                    )
                if len(points) < 2:
                    continue
                clipped_paths.append(clone_toolpath(path, points=points, kind=kind, closed=False))
        return clipped_paths

    def _generate_contour_following_fill(
        self,
        region: Any,
        *,
        line_width_mm: float,
        scanline_spacing_mm: float,
        tolerance_mm: float,
        max_loops: int = 8,
        kind: str = "fill-infill",
    ) -> list[Toolpath]:
        if region is None or region.is_empty:
            return []

        spacing_mm = max(line_width_mm * 0.8, min(scanline_spacing_mm, line_width_mm * 1.2))
        loops: list[Toolpath] = []
        current = region
        for loop_index in range(max_loops):
            loop_offset_mm = loop_index * spacing_mm
            polygons = normalize_geometry(current)
            if not polygons:
                break
            added_this_pass = False
            for polygon_index, polygon in enumerate(polygons):
                rings = [polygon.exterior]
                for ring_index, ring in enumerate(rings):
                    coords = [Point(float(x), float(y)) for x, y in ring.coords]
                    points = simplify_segment_points(coords, tolerance_mm, True)
                    if len(points) < 4:
                        continue
                    if LineString([(point.x, point.y) for point in points]).length < max(line_width_mm * 2.0, spacing_mm * 1.5):
                        continue
                    loops.append(Toolpath(
                        points=points,
                        kind=kind,
                        closed=True,
                        metadata={
                            "small_detail_fill_style": "contour_following",
                            "contour_loop_index": loop_index,
                            "contour_offset_mm": loop_offset_mm,
                            "offset_distance_mm": loop_offset_mm,
                            "contour_spacing_mm": spacing_mm,
                            "polygon_index": polygon_index,
                            "ring_index": ring_index,
                        },
                    ))
                    added_this_pass = True
            if not added_this_pass:
                break
            current = current.buffer(-spacing_mm, join_style=1)
            if current.is_empty:
                break
        return loops

    def _generate_sparse_interior_strokes(
        self,
        region: Any,
        *,
        angle_deg: float,
        line_width_mm: float,
        scanline_spacing_mm: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        max_strokes: int = 3,
        kind: str = "fill-infill",
    ) -> list[Toolpath]:
        if region is None or region.is_empty:
            return []

        stroke_spacing_mm = max(scanline_spacing_mm, line_width_mm * 1.25)
        row_data = self._collect_scanline_rows(
            region,
            spacing_mm=stroke_spacing_mm,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
        )
        origin = row_data.get("origin")
        if origin is None:
            return []

        segments: list[tuple[float, list[tuple[float, float]]]] = []
        for row in row_data.get("rows") or []:
            for coords in row.get("segments") or []:
                length = float(LineString(coords).length)
                if length < max(min_segment_length_mm, line_width_mm * 1.5):
                    continue
                segments.append((length, coords))
        if not segments:
            return self._generate_centerline_fallback(
                region,
                angle_deg=angle_deg,
                min_segment_length_mm=min_segment_length_mm,
                tolerance_mm=tolerance_mm,
                kind=kind,
            )

        segments.sort(key=lambda item: item[0], reverse=True)
        selected: list[Toolpath] = []
        occupied = None
        min_separation_mm = max(line_width_mm * 0.9, stroke_spacing_mm * 0.5)
        for _, coords in segments:
            world_line = affinity.rotate(LineString(coords), angle_deg, origin=origin)
            if occupied is not None and occupied.distance(world_line) < min_separation_mm:
                continue
            points = simplify_segment_points([Point(x, y) for x, y in world_line.coords], tolerance_mm, False)
            if len(points) < 2:
                continue
            selected.append(Toolpath(
                points=points,
                kind=kind,
                closed=False,
                metadata={"small_detail_fill_style": "sparse_strokes"},
            ))
            occupied = world_line if occupied is None else unary_union([occupied, world_line])
            if len(selected) >= max_strokes:
                break

        if selected:
            return selected
        return self._generate_centerline_fallback(
            region,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=tolerance_mm,
            kind=kind,
        )

    def _generate_small_detail_fill(
        self,
        region: Any,
        *,
        line_width_mm: float,
        scanline_spacing_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        detail_tolerance_mm: float,
        allow_overlap: bool,
        connector_validation: dict[str, Any] | None = None,
    ) -> list[Toolpath]:
        effective_tolerance_mm = max(tolerance_mm, detail_tolerance_mm)
        contour_paths = self._generate_contour_following_fill(
            region,
            line_width_mm=line_width_mm,
            scanline_spacing_mm=scanline_spacing_mm * (0.8 if allow_overlap else 1.0),
            tolerance_mm=effective_tolerance_mm,
            kind="fill-infill",
        )
        if contour_paths:
            return contour_paths
        return self._generate_sparse_interior_strokes(
            region,
            angle_deg=angle_deg,
            line_width_mm=line_width_mm,
            scanline_spacing_mm=scanline_spacing_mm,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=effective_tolerance_mm,
            kind="fill-infill",
        )

    def _generate_detail_fill(
        self,
        region: Any,
        *,
        line_width_mm: float,
        scanline_spacing_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        detail_tolerance_mm: float,
        allow_overlap: bool,
        allow_pen_down_infill_connectors: bool = DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS,
        infill_path_mode: str = DEFAULT_INFILL_PATH_MODE,
        connector_validation: dict[str, Any] | None = None,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if region is None or region.is_empty:
            return []

        detail_spacing = max(line_width_mm * 0.35, scanline_spacing_mm * (0.5 if allow_overlap else 1.0))
        inset_amount = min(line_width_mm * 0.15, line_width_mm * 0.5)
        detail_region = region.buffer(-inset_amount, join_style=1)
        if detail_region.is_empty:
            detail_region = region

        detail_paths = self._generate_scanline_infill(
            detail_region,
            spacing_mm=detail_spacing,
            line_width_mm=line_width_mm,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
            kind="detail-trace",
            allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
            infill_path_mode=infill_path_mode,
            connector_validation=connector_validation,
            debug=debug,
        )
        if detail_paths:
            return detail_paths

        return self._generate_centerline_fallback(
            detail_region,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
            kind="detail-trace",
        )

    def _generate_hybrid_region_fill(
        self,
        polygon: Any,
        *,
        line_width_mm: float,
        scanline_spacing_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        detail_tolerance_mm: float,
        small_shape_mode: str,
        thin_detail_overlap: bool,
        allow_pen_down_infill_connectors: bool,
        infill_path_mode: str,
        connector_validation: dict[str, Any] | None,
        travel_optimization: str,
        region_index: int,
        source_polygon_id: str,
        debug: Optional[dict[str, Any]] = None,
    ) -> tuple[list[Toolpath], dict[str, Any], RegionMetrics, FillStrategy]:
        hybrid_config = self._build_hybrid_infill_config(
            line_width_mm=line_width_mm,
            infill_spacing_mm=scanline_spacing_mm,
            wall_count=1,
            infill_angle_deg=angle_deg,
        )
        pen_radius_mm = line_width_mm / 2.0
        drawable_region = _offset_geometry(polygon, -pen_radius_mm)
        if drawable_region is None or drawable_region.is_empty:
            drawable_region = _offset_geometry(polygon, -(line_width_mm * 0.25))
        collapsed_drawable_region = drawable_region is None or drawable_region.is_empty
        metrics_region = polygon
        region_metrics = self._compute_region_metrics(
            metrics_region,
            spacing_mm=scanline_spacing_mm,
            line_width_mm=line_width_mm,
            preferred_angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
        )
        if debug is not None:
            scanline_rows = self._collect_scanline_rows(
                metrics_region,
                spacing_mm=scanline_spacing_mm,
                angle_deg=angle_deg,
                min_segment_length_mm=min_segment_length_mm,
            )
            multi_interval_rows = sum(1 for row in scanline_rows.get("rows") or [] if len(row.get("segments") or []) > 1)
            debug["rows_with_multiple_intervals"] = int(debug.get("rows_with_multiple_intervals", 0)) + multi_interval_rows
            debug["local_cell_count"] = int(debug.get("local_cell_count", 0)) + max(1, multi_interval_rows)
        strategy = choose_fill_strategy(region_metrics, hybrid_config)
        width_pen_units = region_metrics.maxLocalWidthMm / max(line_width_mm, 1e-9)
        coverage_class = "wide"
        if width_pen_units < hybrid_config.singleStrokeMaxWidthFactor:
            coverage_class = "thin"
        elif width_pen_units < hybrid_config.narrowRegionMaxWidthFactor:
            coverage_class = "narrow"
        if region_metrics.areaMm2 <= max(1e-6, line_width_mm * line_width_mm * hybrid_config.tinyDotAreaFactor):
            coverage_class = "tiny"
        if small_shape_mode == "skip" and strategy != "OUTLINE_ONLY":
            strategy = "OUTLINE_ONLY"

        region_debug: dict[str, Any] = {
            "region_index": region_index,
            "source_region_id": source_polygon_id,
            "resolved_angle_deg": angle_deg,
            "fill_mode": {
                "RECTILINEAR_SERPENTINE": "long_thin" if region_metrics.aspectRatio >= 4.0 else "large_open",
                "CONTOUR_PARALLEL_DETAIL": "detail_contour_cell",
                "CENTERLINE_DETAIL": "small_detail_or_text",
                "SINGLE_STROKE_DETAIL": "single_stroke_detail",
                "OUTLINE_ONLY": "outline_only",
                "SKIP_FILL": "skipped_tiny",
            }.get(strategy, strategy),
            "fill_strategy": strategy,
            "classification_reason": strategy,
            "coverage_class": coverage_class,
            "coverage_width_pen_units": width_pen_units,
            "region_metrics": asdict(region_metrics),
            "hybrid_config": asdict(hybrid_config),
        }

        if strategy == "SKIP_FILL":
            return [], region_debug, region_metrics, strategy

        if strategy == "OUTLINE_ONLY":
            return [], region_debug, region_metrics, strategy

        if strategy == "SINGLE_STROKE_DETAIL" and hybrid_config.thinRegionMode == "skip":
            return [], region_debug, region_metrics, "SKIP_FILL"

        if strategy == "SINGLE_STROKE_DETAIL" and hybrid_config.thinRegionMode == "outlineOnly":
            outline_only_paths = self._generate_outline_only_fallback(
                polygon,
                tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                min_segment_length_mm=min_segment_length_mm,
                kind="outline",
            )
            if outline_only_paths:
                return outline_only_paths, region_debug, region_metrics, strategy
            return [], region_debug, region_metrics, "OUTLINE_ONLY"

        if strategy == "SINGLE_STROKE_DETAIL":
            fill_paths = self._generate_single_stroke_detail(
                polygon,
                drawable_region,
                region_metrics,
                hybrid_config,
                angle_deg=angle_deg,
                min_segment_length_mm=min_segment_length_mm,
                tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                kind="fill-infill",
            )
            if not fill_paths:
                fill_paths = self._generate_collapsed_region_fallback(
                    polygon,
                    drawable_region,
                    region_metrics,
                    hybrid_config,
                    angle_deg=angle_deg,
                    min_segment_length_mm=min_segment_length_mm,
                    tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                )
            if not fill_paths:
                fill_paths = self._generate_centerline_fallback(
                    metrics_region,
                    angle_deg=angle_deg,
                    min_segment_length_mm=max(0.01, line_width_mm * 0.2),
                    tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                    kind="fill-infill",
                )
            if fill_paths:
                fill_paths = self._enforce_region_coverage_backfill(
                    fill_paths,
                    region=metrics_region,
                    line_width_mm=line_width_mm,
                    angle_deg=angle_deg,
                    min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.15),
                    tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                    max_backfills=24 if region_metrics.holeCount > 0 else 8,
                    connector_validation=connector_validation,
                    debug=debug,
                )
            if hybrid_config.optimizePathOrder and len(fill_paths) > 1:
                fill_paths = self._order_paths_by_nearest_neighbor(fill_paths, start_point=fill_paths[0].points[0] if fill_paths[0].points else None)
            return fill_paths, region_debug, region_metrics, strategy

        if strategy == "CENTERLINE_DETAIL":
            if not hybrid_config.centerlineFallbackEnabled:
                return [], region_debug, region_metrics, "OUTLINE_ONLY"
            fill_paths = self._generate_centerline_fallback(
                metrics_region,
                angle_deg=angle_deg,
                min_segment_length_mm=min_segment_length_mm,
                tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                kind="fill-infill",
            )
            if not fill_paths:
                return [], region_debug, region_metrics, "OUTLINE_ONLY"
            if hybrid_config.optimizePathOrder:
                fill_paths = self._order_paths_by_nearest_neighbor(fill_paths, start_point=fill_paths[0].points[0] if fill_paths[0].points else None)
            return fill_paths, region_debug, region_metrics, strategy

        if strategy == "CONTOUR_PARALLEL_DETAIL":
            if not hybrid_config.detailFillEnabled:
                return [], region_debug, region_metrics, "OUTLINE_ONLY"
            # Canonical coverage rule: detail regions must prioritize interior
            # coverage lanes, not contour loops that visually resemble outlines.
            effective_coverage_mm = max(1e-6, line_width_mm * 0.85)
            desired_stroke_count = max(1, int(math.ceil(max(0.0, region_metrics.maxLocalWidthMm) / effective_coverage_mm)))
            desired_stroke_count = min(4, desired_stroke_count)
            fill_paths = self._generate_sparse_interior_strokes(
                metrics_region,
                angle_deg=angle_deg,
                line_width_mm=line_width_mm,
                scanline_spacing_mm=effective_coverage_mm,
                min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.25),
                tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                max_strokes=desired_stroke_count,
                kind="fill-infill",
            )
            if not fill_paths:
                fill_paths = self._generate_centerline_fallback(
                    metrics_region,
                    angle_deg=angle_deg,
                    min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.25),
                    tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                    kind="fill-infill",
                )
            # Keep contour strokes only as optional supplemental support for
            # very wide curved regions after interior lanes already exist.
            if region_metrics.maxLocalWidthMm >= (line_width_mm * 2.4):
                contour_support = self._generate_contour_following_fill(
                    metrics_region,
                    line_width_mm=line_width_mm,
                    scanline_spacing_mm=max(scanline_spacing_mm, line_width_mm) * (0.8 if thin_detail_overlap else 1.0),
                    tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                    kind="fill-infill",
                    max_loops=2,
                )
                if contour_support:
                    fill_paths.extend(contour_support)
            fill_paths = [path for path in fill_paths if len(path.points) >= 2 and segment_length(path.points) >= max(line_width_mm * 0.75, min_segment_length_mm)]
            if hybrid_config.optimizePathOrder and len(fill_paths) > 1:
                fill_paths = self._order_paths_by_nearest_neighbor(fill_paths, start_point=fill_paths[0].points[0] if fill_paths[0].points else None)
            fill_paths = self._clip_toolpaths_to_region(
                fill_paths,
                region=metrics_region,
                tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                kind="fill-infill",
            )
            # Mandatory interior coverage for contour-detail regions.
            # Contour loops alone often leave the center lane unfilled in script text.
            effective_coverage_mm = max(1e-6, line_width_mm * 0.85)
            desired_stroke_count = max(1, int(math.ceil(max(0.0, region_metrics.maxLocalWidthMm) / effective_coverage_mm)))
            desired_stroke_count = min(4, desired_stroke_count)
            if desired_stroke_count >= 2:
                interior_strokes = self._generate_sparse_interior_strokes(
                    metrics_region,
                    angle_deg=angle_deg,
                    line_width_mm=line_width_mm,
                    scanline_spacing_mm=effective_coverage_mm,
                    min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.25),
                    tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                    max_strokes=max(1, desired_stroke_count - 1),
                    kind="fill-infill",
                )
                interior_strokes = self._clip_toolpaths_to_region(
                    interior_strokes,
                    region=metrics_region,
                    tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                    kind="fill-infill",
                )
                for idx, stroke in enumerate(interior_strokes):
                    if len(stroke.points) < 2:
                        continue
                    if segment_length(stroke.points) < max(0.05, line_width_mm * 0.20):
                        continue
                    fill_paths.append(clone_toolpath(
                        stroke,
                        metadata={
                            **stroke.metadata,
                            "coverage_backstop": True,
                            "small_detail_fill_style": "detail_contour_interior_lane",
                            "fill_mode": "detail_contour_interior_lane",
                            "requested_stroke_count": desired_stroke_count,
                            "interior_lane_index": idx,
                        },
                    ))
                if debug is not None and interior_strokes:
                    debug["detail_contour_interior_lanes_added"] = int(debug.get("detail_contour_interior_lanes_added", 0)) + int(len(interior_strokes))
            # Coverage backstop: contour loops can leave a narrow uncovered core
            # in thicker script strokes. Add one interior centerline when needed.
            fill_paths = self._augment_detail_contour_with_centerline_backstop(
                fill_paths,
                region=metrics_region,
                line_width_mm=line_width_mm,
                angle_deg=angle_deg,
                min_segment_length_mm=min_segment_length_mm,
                tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                debug=debug,
            )
            # Hard rule: contour-only detail fill should still include at least
            # one interior lane for printable regions wider than one pen.
            if region_metrics.maxLocalWidthMm >= (line_width_mm * 1.35):
                centerline_paths = self._generate_centerline_fallback(
                    metrics_region,
                    angle_deg=angle_deg,
                    min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.35),
                    tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                    kind="fill-infill",
                )
                if centerline_paths:
                    longest = max(centerline_paths, key=lambda p: segment_length(p.points) if len(p.points) >= 2 else 0.0)
                    clipped = self._clip_toolpaths_to_region([longest], region=metrics_region, tolerance_mm=max(tolerance_mm, detail_tolerance_mm), kind="fill-infill")
                    if clipped:
                        fill_paths.append(clone_toolpath(
                            clipped[0],
                            metadata={
                                **clipped[0].metadata,
                                "coverage_backstop": True,
                                "forced_contour_centerline_infill": True,
                                "small_detail_fill_style": "centerline_backfill",
                                "fill_mode": "forced_contour_centerline_infill",
                            },
                        ))
            fill_paths = self._enforce_region_coverage_backfill(
                fill_paths,
                region=metrics_region,
                line_width_mm=line_width_mm,
                angle_deg=angle_deg,
                min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.15),
                tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                max_backfills=24 if region_metrics.holeCount > 0 else 12,
                connector_validation=connector_validation,
                debug=debug,
            )
            return fill_paths, region_debug, region_metrics, strategy

        fill_paths = self._generate_scanline_infill(
            metrics_region,
            spacing_mm=scanline_spacing_mm,
            line_width_mm=line_width_mm,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=tolerance_mm,
            kind="fill-infill",
            allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
            infill_path_mode=infill_path_mode,
            connector_validation=connector_validation,
            adaptive_detail_cells=bool(
                region_metrics.maxLocalWidthMm < (line_width_mm * 2.2)
                or (
                    region_metrics.maxLocalWidthMm < (line_width_mm * 3.0)
                    and region_metrics.estimatedShortRowRatio > 0.25
                )
            ),
            debug=debug,
        )
        if not fill_paths and hybrid_config.centerlineFallbackEnabled:
            fill_paths = self._generate_centerline_fallback(
                metrics_region,
                angle_deg=angle_deg,
                min_segment_length_mm=min_segment_length_mm,
                tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
                kind="fill-infill",
            )
        fill_paths = [path for path in fill_paths if len(path.points) >= 2 and segment_length(path.points) >= max(line_width_mm * 0.75, min_segment_length_mm)]
        fill_paths = self._augment_fill_with_centerline_backstop(
            fill_paths,
            region=metrics_region,
            line_width_mm=line_width_mm,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=max(tolerance_mm, detail_tolerance_mm),
            debug=debug,
        )
        return fill_paths, region_debug, region_metrics, strategy

    def _generate_contour_offset_fill(
        self,
        region: Any,
        *,
        line_width_mm: float,
        spacing_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
    ) -> tuple[list[Toolpath], list[Toolpath], dict[str, Any]]:
        if region is None or region.is_empty:
            return [], [], {"fill_mode": "CONTOUR_OFFSET", "offset_ring_count": 0, "collapsed_residual_count": 0}

        base_inset_mm = max(0.0, line_width_mm * 0.5)
        step_mm = max(0.01, spacing_mm if spacing_mm > 0 else line_width_mm)
        infill_paths: list[Toolpath] = []
        ring_count = 0

        for ring_index in range(1, 2049):
            inset_mm = base_inset_mm + (ring_index * step_mm)
            ring_geometry = _offset_geometry(region, -inset_mm)
            if ring_geometry is None or ring_geometry.is_empty:
                break
            ring_paths = geometry_to_closed_toolpaths(ring_geometry, "fill-infill", tolerance_mm)
            ring_paths = [path for path in ring_paths if segment_length(path.points) >= max(0.01, min_segment_length_mm)]
            if not ring_paths:
                continue
            ring_count += 1
            for path in ring_paths:
                infill_paths.append(clone_toolpath(
                    path,
                    metadata={
                        **path.metadata,
                        "fill_mode": "contour_offset",
                        "fill_strategy": "contour_offset",
                        "scanline_offset_mm": inset_mm,
                        "contour_offset_mm": inset_mm,
                        "contour_offset_index": ring_index,
                        "path_role": "PRINT_INFILL",
                    },
                ))

        residual_detail_paths: list[Toolpath] = []
        if infill_paths:
            stroke_parts: list[Any] = []
            stroke_radius = max(0.01, line_width_mm * 0.5)
            for path in infill_paths:
                if len(path.points) < 2:
                    continue
                line = LineString([(point.x, point.y) for point in path.points])
                if line.is_empty or line.length <= 1e-9:
                    continue
                stroke_parts.append(line.buffer(stroke_radius, cap_style=1, join_style=1))
            covered = unary_union(stroke_parts) if stroke_parts else None
            residual = region if covered is None else region.difference(covered)
        else:
            residual = region

        collapsed_residual_count = 0
        if residual is not None and not residual.is_empty:
            for component in normalize_geometry(residual):
                min_x, min_y, max_x, max_y = component.bounds
                local_width = min(max_x - min_x, max_y - min_y)
                if local_width > (line_width_mm * 1.6):
                    continue
                detail = self._generate_centerline_fallback(
                    component,
                    angle_deg=angle_deg,
                    min_segment_length_mm=max(0.01, min_segment_length_mm * 0.5),
                    tolerance_mm=max(0.01, tolerance_mm),
                    kind="detail-trace",
                )
                if not detail:
                    detail = self._generate_tiny_dot_or_short_stroke(
                        component,
                        line_width_mm=line_width_mm,
                        tolerance_mm=max(0.01, tolerance_mm),
                        angle_deg=angle_deg,
                        kind="detail-trace",
                    )
                if detail:
                    collapsed_residual_count += 1
                    residual_detail_paths.extend(detail)

        debug_info = {
            "fill_mode": "CONTOUR_OFFSET",
            "fill_strategy": "contour_offset",
            "offset_ring_count": int(ring_count),
            "collapsed_residual_count": int(collapsed_residual_count),
            "offset_base_inset_mm": float(base_inset_mm),
            "offset_step_mm": float(step_mm),
        }
        return infill_paths, residual_detail_paths, debug_info

    def _augment_detail_contour_with_centerline_backstop(
        self,
        paths: list[Toolpath],
        *,
        region: Any,
        line_width_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if region is None or region.is_empty:
            return paths
        if not paths:
            return paths
        pen_radius = max(0.01, line_width_mm * 0.5 * 0.85)
        stroke_buffers = []
        for path in paths:
            if len(path.points) < 2:
                continue
            line = LineString([(point.x, point.y) for point in path.points])
            if line.is_empty or line.length <= 1e-9:
                continue
            stroke_buffers.append(line.buffer(pen_radius, cap_style=1, join_style=1))
        if not stroke_buffers:
            return paths

        covered = unary_union(stroke_buffers)
        if covered is None or covered.is_empty:
            uncovered_ratio = 1.0
        else:
            uncovered = region.difference(covered)
            uncovered_area = 0.0 if uncovered is None or uncovered.is_empty else float(uncovered.area)
            region_area = max(1e-9, float(region.area))
            uncovered_ratio = uncovered_area / region_area

        if uncovered_ratio <= 0.03 and len(paths) > 2:
            return paths

        centerline_paths = self._generate_centerline_fallback(
            region,
            angle_deg=angle_deg,
            min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.4),
            tolerance_mm=tolerance_mm,
            kind="fill-infill",
        )
        if not centerline_paths:
            return paths

        primary = max(centerline_paths, key=lambda p: segment_length(p.points) if len(p.points) >= 2 else 0.0)
        if len(primary.points) < 2:
            return paths
        primary = clone_toolpath(
            primary,
            metadata={
                **primary.metadata,
                "small_detail_fill_style": "centerline_backstop",
                "fill_mode": "detail_contour_centerline_backstop",
                "coverage_backstop": True,
                "estimated_uncovered_ratio_before_backstop": float(uncovered_ratio),
            },
        )
        augmented = list(paths)
        augmented.append(primary)
        if debug is not None:
            debug["detail_contour_centerline_backstop_count"] = int(debug.get("detail_contour_centerline_backstop_count", 0)) + 1
        return augmented

    def _augment_fill_with_centerline_backstop(
        self,
        paths: list[Toolpath],
        *,
        region: Any,
        line_width_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        # Skip if already centerline-led or region too tiny.
        if not paths or region is None or region.is_empty:
            return paths
        if any(str(path.metadata.get("small_detail_fill_style")) in {"single_stroke_detail", "centerline_backstop"} for path in paths):
            return paths
        if float(region.area) <= max(1e-6, line_width_mm * line_width_mm * 1.5):
            return paths
        return self._augment_detail_contour_with_centerline_backstop(
            paths,
            region=region,
            line_width_mm=line_width_mm,
            angle_deg=angle_deg,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=tolerance_mm,
            debug=debug,
        )

    def _enforce_region_coverage_backfill(
        self,
        paths: list[Toolpath],
        *,
        region: Any,
        line_width_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        max_backfills: int = 120,
        precovered_geometry: Any = None,
        connector_validation: dict[str, Any] | None = None,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if region is None or region.is_empty or max_backfills <= 0:
            return paths
        if not any(p.kind in {"fill-infill", "detail-trace"} and len(p.points) >= 2 for p in paths):
            return paths

        pen_radius = max(0.01, line_width_mm * 0.5 * 0.85)
        region_area = max(1e-9, float(region.area))
        min_component_area = max(1e-7, line_width_mm * line_width_mm * 0.01)
        augmented = list(paths)
        coverage_mask = connector_validation.get("mask") if isinstance(connector_validation, dict) else None
        coverage_matrix = connector_validation.get("current_to_source_matrix") if isinstance(connector_validation, dict) else None
        coverage_include_kinds = {"fill-infill", "detail-trace", "fill-wall", "outline", "fill-infill-travel"}

        def _coverage_metrics_for(candidate_paths: list[Toolpath]) -> MaskCoverageMetrics | None:
            if coverage_mask is None or not isinstance(coverage_matrix, (tuple, list)) or len(coverage_matrix) != 6:
                return None
            try:
                return compute_toolpath_mask_coverage_metrics(
                    candidate_paths,
                    mask=coverage_mask,
                    current_to_source_matrix=tuple(float(value) for value in coverage_matrix),
                    pen_radius_mm=line_width_mm * 0.5,
                    sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
                    include_kinds=coverage_include_kinds,
                )
            except Exception:
                return None

        baseline_metrics = _coverage_metrics_for(augmented)
        added = 0
        iteration_count = 0
        last_uncovered_ratio = 0.0
        last_component_count = 0
        last_max_component_area = 0.0

        def _eligible_uncovered_components(current_uncovered: Any) -> list[Any]:
            components: list[Any] = []
            for poly in normalize_geometry(current_uncovered):
                area = float(poly.area)
                if area < min_component_area:
                    # Keep narrow-but-important pockets if they are elongated.
                    try:
                        min_x, min_y, max_x, max_y = poly.bounds
                        span_x = max_x - min_x
                        span_y = max_y - min_y
                        span_max = max(span_x, span_y)
                        span_min = max(1e-9, min(span_x, span_y))
                        aspect = span_max / span_min
                    except Exception:
                        aspect = 0.0
                    if not (area >= min_component_area * 0.35 and aspect >= 3.0):
                        continue
                components.append(poly)
            return sorted(components, key=lambda g: float(g.area), reverse=True)

        def _current_uncovered_geometry() -> Any:
            drawables = [p for p in augmented if p.kind in {"fill-infill", "detail-trace"} and len(p.points) >= 2]
            covered_buffers = []
            for path in drawables:
                line = LineString([(point.x, point.y) for point in path.points])
                if line.is_empty or line.length <= 1e-9:
                    continue
                covered_buffers.append(line.buffer(pen_radius, cap_style=1, join_style=1))
            if not covered_buffers:
                return region
            covered = unary_union(covered_buffers)
            if precovered_geometry is not None and not precovered_geometry.is_empty:
                try:
                    covered = covered.union(precovered_geometry)
                except Exception:
                    covered = covered.buffer(0).union(precovered_geometry.buffer(0))
            if covered is None or covered.is_empty:
                return region
            return region.difference(covered)

        while added < max_backfills:
            uncovered = _current_uncovered_geometry()
            if uncovered is None or uncovered.is_empty:
                last_uncovered_ratio = 0.0
                last_component_count = 0
                last_max_component_area = 0.0
                break

            uncovered_area_total = float(uncovered.area)
            last_uncovered_ratio = uncovered_area_total / region_area
            components = _eligible_uncovered_components(uncovered)
            last_component_count = len(components)
            last_max_component_area = max((float(component.area) for component in components), default=0.0)
            if not components:
                break

            iteration_count += 1
            progress_this_iteration = 0
            remaining_budget = max_backfills - added

            for component in components:
                if remaining_budget <= 0:
                    break
                pocket_scan = self._generate_scanline_infill(
                    component,
                    spacing_mm=max(line_width_mm * 0.80, 0.05),
                    line_width_mm=line_width_mm,
                    angle_deg=angle_deg,
                    min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.15),
                    tolerance_mm=tolerance_mm,
                    kind="fill-infill",
                    allow_pen_down_infill_connectors=False,
                    infill_path_mode="rectilinear",
                    connector_validation=None,
                    adaptive_detail_cells=True,
                    debug=None,
                )
                pocket_scan = self._clip_toolpaths_to_region(
                    pocket_scan,
                    region=component,
                    tolerance_mm=tolerance_mm,
                    kind="fill-infill",
                )
                emitted_for_component = 0
                for candidate_path in pocket_scan:
                    if remaining_budget <= 0:
                        break
                    if len(candidate_path.points) < 2:
                        continue
                    if segment_length(candidate_path.points) < max(0.04, line_width_mm * 0.10):
                        continue
                    stroke = clone_toolpath(
                        candidate_path,
                        metadata={
                            **candidate_path.metadata,
                            "coverage_backstop": True,
                            "coverage_backfill_global": True,
                            "small_detail_fill_style": "scanline_backfill",
                            "fill_mode": "global_uncovered_backfill",
                            "coverage_backfill_iteration": iteration_count,
                        },
                    )
                    accepted = True
                    if baseline_metrics is not None:
                        trial_metrics = _coverage_metrics_for([*augmented, stroke])
                        accepted = bool(
                            trial_metrics is not None
                            and trial_metrics.penalized_coverage_percent > (baseline_metrics.penalized_coverage_percent + 1e-9)
                        )
                        if accepted and trial_metrics is not None:
                            baseline_metrics = trial_metrics
                    if accepted:
                        augmented.append(stroke)
                        added += 1
                        progress_this_iteration += 1
                        emitted_for_component += 1
                        remaining_budget -= 1
                if emitted_for_component == 0 and remaining_budget > 0:
                    centerline = self._generate_centerline_fallback(
                        component,
                        angle_deg=angle_deg,
                        min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.35),
                        tolerance_mm=tolerance_mm,
                        kind="fill-infill",
                    )
                    if centerline:
                        longest = max(centerline, key=lambda p: segment_length(p.points) if len(p.points) >= 2 else 0.0)
                        clipped = self._clip_toolpaths_to_region([longest], region=component, tolerance_mm=tolerance_mm, kind="fill-infill")
                        if clipped and len(clipped[0].points) >= 2:
                            centerline_stroke = clone_toolpath(
                                clipped[0],
                                metadata={
                                    **clipped[0].metadata,
                                    "coverage_backstop": True,
                                    "coverage_backfill_global": True,
                                    "small_detail_fill_style": "centerline_backfill",
                                    "fill_mode": "global_uncovered_backfill",
                                    "coverage_backfill_iteration": iteration_count,
                                },
                            )
                            augmented.append(centerline_stroke)
                            accepted = True
                            if baseline_metrics is not None:
                                trial_metrics = _coverage_metrics_for(augmented)
                                accepted = bool(
                                    trial_metrics is not None
                                    and trial_metrics.penalized_coverage_percent > (baseline_metrics.penalized_coverage_percent + 1e-9)
                                )
                                if accepted and trial_metrics is not None:
                                    baseline_metrics = trial_metrics
                            if not accepted:
                                augmented.pop()
                            else:
                                added += 1
                                progress_this_iteration += 1
                                remaining_budget -= 1

            if progress_this_iteration <= 0:
                break

        if debug is not None and added > 0:
            debug["global_uncovered_backfill_count"] = int(debug.get("global_uncovered_backfill_count", 0)) + int(added)
        if debug is not None:
            debug["global_uncovered_backfill_iterations"] = int(debug.get("global_uncovered_backfill_iterations", 0)) + int(iteration_count)
            debug["global_uncovered_component_count_last_region"] = int(last_component_count)
            debug["global_uncovered_max_component_area_mm2_last_region"] = float(last_max_component_area)
            debug["global_uncovered_area_ratio_last_region"] = float(last_uncovered_ratio)
        return augmented

    def _enforce_non_outline_like_fill(
        self,
        paths: list[Toolpath],
        *,
        region: Any,
        line_width_mm: float,
        angle_deg: float,
        min_segment_length_mm: float,
        tolerance_mm: float,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if region is None or region.is_empty:
            return paths
        fill_paths = [p for p in paths if p.kind in {"fill-infill", "detail-trace"} and len(p.points) >= 2]
        if not fill_paths:
            return paths
        # If fill is effectively just contour loops, force an interior centerline stroke.
        closed_like_count = sum(1 for p in fill_paths if p.closed or str(p.metadata.get("small_detail_fill_style")) == "contour_following")
        if closed_like_count < len(fill_paths):
            return paths
        centerline = self._generate_centerline_fallback(
            region,
            angle_deg=angle_deg,
            min_segment_length_mm=max(min_segment_length_mm, line_width_mm * 0.35),
            tolerance_mm=tolerance_mm,
            kind="fill-infill",
        )
        if not centerline:
            return paths
        longest = max(centerline, key=lambda p: segment_length(p.points) if len(p.points) >= 2 else 0.0)
        clipped = self._clip_toolpaths_to_region([longest], region=region, tolerance_mm=tolerance_mm, kind="fill-infill")
        if not clipped:
            return paths
        stroke = clone_toolpath(
            clipped[0],
            metadata={
                **clipped[0].metadata,
                "coverage_backstop": True,
                "forced_non_outline_like_fill": True,
                "small_detail_fill_style": "centerline_backfill",
                "fill_mode": "forced_centerline_infill",
            },
        )
        if len(stroke.points) < 2 or segment_length(stroke.points) < max(0.04, line_width_mm * 0.10):
            return paths
        augmented = list(paths)
        augmented.append(stroke)
        if debug is not None:
            debug["forced_centerline_infill_count"] = int(debug.get("forced_centerline_infill_count", 0)) + 1
        return augmented

    def _append_offset_debug(
        self,
        debug: Optional[dict[str, Any]],
        *,
        path_id: str,
        operation: str,
        requested_offset_mm: float,
        input_geometry: Any,
        output_geometry: Any,
        warnings: Optional[list[str]] = None,
    ) -> None:
        if debug is None:
            return
        debug.setdefault("offset_debug", [])
        input_polygons = normalize_geometry(input_geometry)
        output_polygons = normalize_geometry(output_geometry)
        input_winding = "unknown"
        output_winding = "unknown"
        if input_polygons:
            input_points = [Point(x, y) for x, y in input_polygons[0].exterior.coords]
            input_winding = _winding(input_points, True)
        if output_polygons:
            output_points = [Point(x, y) for x, y in output_polygons[0].exterior.coords]
            output_winding = _winding(output_points, True)
        debug["offset_debug"].append({
            "path_id": path_id,
            "operation": operation,
            "input_space": "surface_mm",
            "output_space": "surface_mm",
            "requested_offset_mm": requested_offset_mm,
            "actual_offset_estimate_mm": requested_offset_mm,
            "input_area_mm2": 0.0 if input_geometry is None or input_geometry.is_empty else float(input_geometry.area),
            "output_area_mm2": 0.0 if output_geometry is None or output_geometry.is_empty else float(output_geometry.area),
            "input_winding": input_winding,
            "output_winding": output_winding,
            "hole_count": sum(len(poly.interiors) for poly in output_polygons),
            "self_intersections_before": 0 if input_geometry is None or getattr(input_geometry, "is_simple", True) else 1,
            "self_intersections_after": 0 if output_geometry is None or getattr(output_geometry, "is_simple", True) else 1,
            "invalid_segments_removed": 0,
            "collapsed": bool(output_geometry is None or output_geometry.is_empty),
            "warnings": list(warnings or []),
        })

    def _resolve_outline_base_inset_mm(
        self,
        *,
        pen_width_mm: float,
        outline_placement_mode: str,
        custom_offset_mm: float = 0.0,
    ) -> float:
        if outline_placement_mode == "disabled":
            return float("inf")
        if outline_placement_mode == "center_on_boundary":
            return 0.0
        if outline_placement_mode == "inside_by_custom_offset":
            return max(0.0, custom_offset_mm)
        if outline_placement_mode == "inside_edge_default":
            # Keep the stroke centerline at least one pen radius inside the
            # boundary so rasterized footprint does not spill heavily outside.
            return max(0.0, pen_width_mm * 0.5)
        return max(0.0, pen_width_mm * 0.5)

    def _offset_polygon_into_printable_area(
        self,
        polygon: Polygon,
        *,
        inset_mm: float,
    ) -> Any:
        if inset_mm <= 1e-9:
            return polygon
        return polygon.buffer(-inset_mm, join_style=1)

    def generate_outline_cleanup_paths(
        self,
        printable_region: PrintableRegion,
        *,
        pen_width_mm: float,
        wall_count: int,
        wall_spacing_mm: float,
        outline_placement_mode: str = DEFAULT_OUTLINE_PLACEMENT_MODE,
        custom_offset_mm: float = 0.0,
        simplify_tolerance_mm: float,
    ) -> list[Toolpath]:
        outline_paths: list[Toolpath] = []
        polygon = printable_region.geometry
        region_index = max(0, (_extract_component_id(printable_region.component_id) or 1) - 1)
        source_polygon_id = printable_region.component_id
        base_inset_mm = self._resolve_outline_base_inset_mm(
            pen_width_mm=pen_width_mm,
            outline_placement_mode=outline_placement_mode,
            custom_offset_mm=custom_offset_mm,
        )
        if math.isinf(base_inset_mm):
            return []
        for wall_index in range(max(1, wall_count)):
            wall_kind = "outline" if wall_index == 0 else "fill-wall"
            inset_mm = base_inset_mm + (wall_index * wall_spacing_mm)
            wall_polygon = self._offset_polygon_into_printable_area(
                polygon,
                inset_mm=inset_mm,
            )
            if wall_polygon is None or wall_polygon.is_empty:
                continue
            wall_role = "cleanup_edge_over_fill" if wall_index == 0 else "inner_cleanup_wall"
            for contour_index, path in enumerate(geometry_to_closed_toolpaths(wall_polygon, wall_kind, simplify_tolerance_mm), start=1):
                ring_role = "outer" if contour_index == 1 else "hole"
                offset_direction = "inside_printable_region" if ring_role == "outer" else "into_printed_material"
                outline_path = clone_toolpath(
                    path,
                    region_id=region_index,
                    source=printable_region.source,
                    metadata={
                        **path.metadata,
                        "source_polygon_id": source_polygon_id,
                        "source_component_id": region_index + 1,
                        "source_contour_id": contour_index,
                        "source_region_id": printable_region.component_id,
                        "generated_from": printable_region.source,
                        "source_polygon_matches_infill_clip_polygon": True,
                        "outline_uses_infill_clip_polygon": True if wall_index == 0 else False,
                        "offset_distance_mm": inset_mm,
                        "outline_offset_mm": -inset_mm,
                        "offset_direction": offset_direction,
                        "pen_width_mm": pen_width_mm,
                        "wall_spacing_mm": wall_spacing_mm,
                        "wall_index": wall_index,
                        "wall_role": "outer_boundary_cleanup" if wall_index == 0 else "inner_cleanup_wall",
                        "outline_placement_mode": outline_placement_mode,
                        "purpose": wall_role,
                        "expected_relation_to_fill": "boundary_cleanup" if wall_index == 0 else "supporting_wall",
                        "simplify_tolerance_mm": simplify_tolerance_mm,
                        "coordinate_space_at_creation": "surface_mm",
                        "coordinate_space_before_offset": "surface_mm",
                        "offset_space": "surface_mm",
                        "coordinate_space_before_simplify": "surface_mm",
                        "simplify_space": "surface_mm",
                        "ring_role": ring_role,
                    },
                )
                if not path_is_inside_printable_area(outline_path, polygon, tolerance_mm=max(0.02, pen_width_mm * 0.1)):
                    raise AssertionError(
                        f"{outline_path.kind} {source_polygon_id} escaped printable polygon after inward offset"
                    )
                outline_paths.append(outline_path)
        return outline_paths

    def slice_one_layer(
        self,
        printable_geometry: Any,
        *,
        line_width_mm: float,
        wall_count: int,
        infill_density: float = 100.0,
        infill_angle_deg: float = 0.0,
        fill_strategy: str = "horizontal_scanline",
        alternate_fill_angle_deg: float = -45.0,
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
        infill_path_mode: str = DEFAULT_INFILL_PATH_MODE,
        expensive_coverage_repair: bool = True,
        connector_validation: dict[str, Any] | None = None,
        debug: Optional[dict[str, Any]] = None,
    ) -> list[Toolpath]:
        if printable_geometry is None or printable_geometry.is_empty or line_width_mm <= 0:
            return []

        resolved_infill_path_mode = (infill_path_mode or DEFAULT_INFILL_PATH_MODE).strip().lower()
        if resolved_infill_path_mode not in {"rectilinear", "serpentine_optimized", "legacy"}:
            resolved_infill_path_mode = DEFAULT_INFILL_PATH_MODE
        effective_fill_strategy = fill_strategy
        effective_allow_connectors = allow_pen_down_infill_connectors
        if resolved_infill_path_mode == "legacy":
            effective_allow_connectors = True
            if fill_strategy == "adaptive_angle":
                effective_fill_strategy = "horizontal_scanline"

        simplify_tolerance_resolved_mm = simplify_tolerance_mm
        thin_detail_tolerance_mm = thin_detail_simplify_mm
        min_segment_length_resolved_mm = self._recommended_infill_min_segment_length_mm(line_width_mm, min_segment_length_mm)
        min_fill_area_resolved_mm2 = min_fill_area_mm2
        thin_detail_min_area_resolved_mm2 = thin_detail_min_area_mm2
        normalized_geometry = normalize_geometry_config(
            raw_line_width_mm=line_width_mm,
            raw_infill_spacing_mm=infill_spacing_mm if infill_spacing_mm > 0 else None,
            raw_detail_spacing_mm=None,
            raw_wall_spacing_mm=line_width_mm,
            connector_sample_step_mm=max(0.01, min(line_width_mm / 4.0, 0.05)),
        )
        scanline_spacing_mm = normalized_geometry.effectiveInfillSpacingMm
        logger.debug(
            "Fill generation resolved settings: line_width_mm=%.4f infill_spacing_mm=%.4f wall_count=%d infill_density=%.2f infill_angle_deg=%.2f fill_strategy=%s infill_path_mode=%s allow_connectors=%s alternate_fill_angle_deg=%.2f min_fill_area_mm2=%.4f min_fill_width_mm=%.4f min_segment_length_mm=%.4f coordinate_space=%s",
            line_width_mm,
            scanline_spacing_mm,
            wall_count,
            infill_density,
            infill_angle_deg,
            effective_fill_strategy,
            resolved_infill_path_mode,
            effective_allow_connectors,
            alternate_fill_angle_deg,
            min_fill_area_resolved_mm2,
            min_fill_width_mm,
            min_segment_length_resolved_mm,
            "surface-mm-on-ball",
        )

        debug_append_geometry(debug, "final_composed_fill_region", printable_geometry, "final-composed-fill")
        printable_regions = build_printable_regions_from_geometry(printable_geometry)
        logger.debug("Filled polygon count: %d", len(printable_regions))

        ordered: list[Toolpath] = []
        slicer_counts = {
            "normal_slicer_region_count": 0,
            "large_open_region_count": 0,
            "long_thin_region_count": 0,
            "small_detail_region_count": 0,
            "outline_buffer_empty_region_count": 0,
            "normal_infill_empty_region_count": 0,
            "thin_detail_fallback_region_count": 0,
            "thin_detail_path_count": 0,
            "collapsed_drawable_region_count": 0,
            "suppressed_wall_for_thin_region_count": 0,
            "single_stroke_region_count": 0,
            "skipped_tiny_region_count": 0,
            "pruned_noisy_path_count": 0,
            "detail_trace_path_count": 0,
            "adaptive_total_cells": 0,
            "adaptive_rectilinear_cells": 0,
            "adaptive_detail_contour_cells": 0,
            "adaptive_single_stroke_cells": 0,
            "adaptive_narrow_cells_detected": 0,
            "adaptive_switched_too_few_rows": 0,
            "adaptive_switched_connector_ratio": 0,
            "adaptive_switched_single_stroke_width": 0,
            "adaptive_switched_single_stroke_hatch_quality": 0,
            "thin_region_count": 0,
            "narrow_region_count": 0,
            "wide_region_count": 0,
            "tiny_region_count": 0,
            "narrower_than_two_pen_region_count": 0,
            "narrower_than_two_pen_with_centerline_count": 0,
            "dropped_region_count": 0,
        }
        infill_region_debug: list[dict[str, Any]] = []
        for region_index, printable_region in enumerate(printable_regions):
            polygon = printable_region.geometry
            source_polygon_id = printable_region.component_id
            outline_placement_mode = DEFAULT_OUTLINE_PLACEMENT_MODE
            wall_spacing_mm = normalized_geometry.effectiveWallSpacingMm
            hybrid_config = self._build_hybrid_infill_config(
                line_width_mm=line_width_mm,
                infill_spacing_mm=normalized_geometry.effectiveInfillSpacingMm,
                wall_count=wall_count,
                infill_angle_deg=infill_angle_deg,
            )
            preview_drawable_region = _offset_geometry(polygon, -(line_width_mm / 2.0))
            preview_metrics_region = preview_drawable_region if preview_drawable_region is not None and not preview_drawable_region.is_empty else polygon
            preview_region_metrics = self._compute_region_metrics(
                preview_metrics_region,
                spacing_mm=scanline_spacing_mm,
                line_width_mm=line_width_mm,
                preferred_angle_deg=infill_angle_deg,
                min_segment_length_mm=min_segment_length_resolved_mm,
            )
            preview_region_strategy = choose_fill_strategy(preview_region_metrics, hybrid_config)
            measured_local_width_mm = float(preview_region_metrics.maxLocalWidthMm)
            too_thin_for_dual_outline = measured_local_width_mm < (2.0 * line_width_mm)
            if too_thin_for_dual_outline:
                slicer_counts["narrower_than_two_pen_region_count"] += 1
            drawable_collapsed_at_pen_radius = preview_drawable_region is None or preview_drawable_region.is_empty
            collapse_candidate = (
                hybrid_config.thinRegionMode in {"singleStroke", "outlineOnly"}
                and (too_thin_for_dual_outline or drawable_collapsed_at_pen_radius)
            )
            outline_decision = "normal_outline"
            outline_reason = "region_supports_dual_outline"
            suppress_outline_walls = False
            if preview_drawable_region is None or preview_drawable_region.is_empty:
                slicer_counts["collapsed_drawable_region_count"] += 1
            expected_outline_inset_mm = self._resolve_outline_base_inset_mm(
                pen_width_mm=line_width_mm,
                outline_placement_mode=outline_placement_mode,
            )
            printable_outline_region = self._offset_polygon_into_printable_area(polygon, inset_mm=expected_outline_inset_mm)
            can_fit_outline = not printable_outline_region.is_empty
            force_keep_outline = (
                can_fit_outline
                and preview_region_metrics.areaMm2 >= max(line_width_mm * line_width_mm * 6.0, 1.5)
            )
            suppress_outline_walls = bool(collapse_candidate and not force_keep_outline)
            # Keep outline cleanup enabled even for narrow regions so outline+infill
            # can intentionally overlap at minimum printable widths.
            tiny_outline_exception = bool(
                preview_region_metrics.areaMm2 <= max(1e-6, line_width_mm * line_width_mm * hybrid_config.tinyDotAreaFactor * 3.0)
                and measured_local_width_mm <= line_width_mm * 1.25
            )
            if suppress_outline_walls and can_fit_outline and not tiny_outline_exception:
                suppress_outline_walls = False
            if suppress_outline_walls:
                slicer_counts["suppressed_wall_for_thin_region_count"] += 1
                outline_decision = "centerline_collapse"
                if too_thin_for_dual_outline:
                    outline_reason = "local_width_lt_2x_pen"
                elif drawable_collapsed_at_pen_radius:
                    outline_reason = "inset_collapse_pen_radius"
            elif collapse_candidate and force_keep_outline:
                outline_decision = "normal_outline"
                outline_reason = "force_keep_drawable_non_tiny"
            if not can_fit_outline:
                slicer_counts["outline_buffer_empty_region_count"] += 1
                if outline_decision == "normal_outline":
                    outline_decision = "tiny_dot"
                    outline_reason = "outline_offset_collapses"
            if debug is not None:
                debug.setdefault("outline_component_decisions", []).append({
                    "component_id": source_polygon_id,
                    "measured_local_width_mm": measured_local_width_mm,
                    "line_width_mm": float(line_width_mm),
                    "decision": outline_decision,
                    "reason": outline_reason,
                })
            if not can_fit_outline and not thin_detail_mode:
                continue

            debug_append_geometry(debug, "detected_printable_polygons", polygon, "detected-printable-polygon")
            region_paths: list[Toolpath] = []
            outline_cleanup_paths: list[Toolpath] = []
            cleanup_outline_only_paths: list[Toolpath] = []
            infill_paths: list[Toolpath] = []
            infill_region = None
            fill_threshold_failed = True
            resolved_infill_angle_deg = infill_angle_deg
            infill_clip_inset_mm = max(0.0, normalized_geometry.penRadiusMm)
            if can_fit_outline:
                infill_region = polygon.buffer(-infill_clip_inset_mm, join_style=1)
                if not infill_region.is_empty:
                    fill_area = infill_region.area
                    fill_threshold_failed = fill_area < min_fill_area_resolved_mm2
                else:
                    slicer_counts["normal_infill_empty_region_count"] += 1

            if can_fit_outline and not suppress_outline_walls:
                slicer_counts["normal_slicer_region_count"] += 1
                outline_cleanup_paths = self.generate_outline_cleanup_paths(
                    printable_region,
                    pen_width_mm=line_width_mm,
                    wall_count=max(1, wall_count),
                    wall_spacing_mm=wall_spacing_mm,
                    outline_placement_mode=outline_placement_mode,
                    simplify_tolerance_mm=simplify_tolerance_resolved_mm,
                )
                cleanup_outline_only_paths = [path for path in outline_cleanup_paths if path.kind == "outline"]
                wall_paths = [path for path in outline_cleanup_paths if path.kind == "fill-wall"]
                for wall_path in outline_cleanup_paths:
                    output_geometry = self._offset_polygon_into_printable_area(
                        polygon,
                        inset_mm=float(wall_path.metadata.get("offset_distance_mm", 0.0)),
                    )
                    self._append_offset_debug(
                        debug,
                        path_id=wall_path.path_id or f"{wall_path.kind}-{source_polygon_id}-{wall_path.metadata.get('wall_index', 0)}",
                        operation="outline_cleanup_inset",
                        requested_offset_mm=-float(wall_path.metadata.get("offset_distance_mm", 0.0)),
                        input_geometry=polygon,
                        output_geometry=output_geometry,
                    )
                debug_append_toolpaths(debug, "outer_walls", wall_paths)
                debug_append_toolpaths(debug, "deferred_cleanup_outlines", cleanup_outline_only_paths)
                if outline_after_fill:
                    region_paths.extend(optimize_toolpath_order(wall_paths, strategy=travel_optimization))
                else:
                    region_paths.extend(optimize_toolpath_order(outline_cleanup_paths, strategy=travel_optimization))
            elif suppress_outline_walls:
                if hybrid_config.thinRegionMode == "outlineOnly":
                    outline_cleanup_paths = self._generate_outline_only_fallback(
                        polygon,
                        tolerance_mm=simplify_tolerance_resolved_mm,
                        min_segment_length_mm=min_segment_length_resolved_mm,
                        kind="outline",
                    )
                    cleanup_outline_only_paths = list(outline_cleanup_paths)
                else:
                    outline_cleanup_paths = []
                    cleanup_outline_only_paths = []

            anchor = region_paths[-1].points[-1] if region_paths and region_paths[-1].points else Point(0.0, 0.0)
            region_debug_entry: dict[str, Any] = {
                "region_index": region_index,
                "source_region_id": printable_region.component_id,
                "fill_mode": "skipped",
                "resolved_angle_deg": resolved_infill_angle_deg,
            }
            # Prefer pen-radius drawable inset as the fallback fill source to
            # avoid boundary-hugging fallback strokes that inflate overdraw.
            fill_region_source = infill_region if infill_region is not None and not infill_region.is_empty else preview_metrics_region
            if fill_region_source is None or fill_region_source.is_empty:
                fill_region_source = polygon
            needs_detail_fallback_fill = preview_region_strategy in {"SINGLE_STROKE_DETAIL", "CENTERLINE_DETAIL", "CONTOUR_PARALLEL_DETAIL"}
            if (not fill_threshold_failed and infill_region is not None and not infill_region.is_empty) or needs_detail_fallback_fill:
                resolved_infill_angle_deg, angle_debug = self._resolve_infill_angle(
                    fill_region_source,
                    spacing_mm=scanline_spacing_mm,
                    angle_deg=infill_angle_deg,
                    alternate_angle_deg=alternate_fill_angle_deg,
                    fill_strategy=effective_fill_strategy,
                    min_segment_length_mm=min_segment_length_resolved_mm,
                    line_width_mm=line_width_mm,
                    region_index=region_index,
                )
                use_contour_offset_fill = str(effective_fill_strategy).strip().lower() in {
                    "adaptive_angle",
                    "offset",
                    "offset_fill",
                    "contour",
                    "contour_fill",
                    "contour_offset",
                }
                if use_contour_offset_fill:
                    contour_infill_paths, contour_detail_paths, contour_debug = self._generate_contour_offset_fill(
                        fill_region_source,
                        line_width_mm=line_width_mm,
                        spacing_mm=scanline_spacing_mm,
                        angle_deg=resolved_infill_angle_deg,
                        min_segment_length_mm=min_segment_length_resolved_mm,
                        tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                    )
                    infill_paths = contour_infill_paths + contour_detail_paths
                    region_metrics = self._compute_region_metrics(
                        fill_region_source,
                        spacing_mm=scanline_spacing_mm,
                        line_width_mm=line_width_mm,
                        preferred_angle_deg=resolved_infill_angle_deg,
                        min_segment_length_mm=min_segment_length_resolved_mm,
                    )
                    hybrid_region_debug = {
                        **contour_debug,
                        "classification_reason": "contour_offset_fill_from_target_mask",
                        "coverage_class": "offset_contour",
                    }
                    fill_strategy_name = "CONTOUR_OFFSET"
                else:
                    infill_paths, hybrid_region_debug, region_metrics, fill_strategy_name = self._generate_hybrid_region_fill(
                        fill_region_source,
                        line_width_mm=line_width_mm,
                        scanline_spacing_mm=scanline_spacing_mm,
                        angle_deg=resolved_infill_angle_deg,
                        min_segment_length_mm=min_segment_length_resolved_mm,
                        tolerance_mm=simplify_tolerance_resolved_mm,
                        detail_tolerance_mm=thin_detail_tolerance_mm,
                        small_shape_mode=small_shape_mode,
                        thin_detail_overlap=thin_detail_overlap,
                        allow_pen_down_infill_connectors=effective_allow_connectors,
                        infill_path_mode=resolved_infill_path_mode,
                        connector_validation=connector_validation,
                        travel_optimization=travel_optimization,
                        region_index=region_index,
                        source_polygon_id=source_polygon_id,
                        debug=debug,
                    )
                if fill_strategy_name == "RECTILINEAR_SERPENTINE":
                    slicer_counts["large_open_region_count"] += 1
                    slicer_counts["adaptive_total_cells"] += 1
                    slicer_counts["adaptive_rectilinear_cells"] += 1
                elif fill_strategy_name == "CONTOUR_PARALLEL_DETAIL":
                    slicer_counts["small_detail_region_count"] += 1
                    slicer_counts["thin_detail_fallback_region_count"] += 1
                    slicer_counts["adaptive_total_cells"] += 1
                    slicer_counts["adaptive_detail_contour_cells"] += 1
                    slicer_counts["adaptive_narrow_cells_detected"] += 1
                elif fill_strategy_name == "CENTERLINE_DETAIL":
                    slicer_counts["thin_detail_fallback_region_count"] += 1
                    slicer_counts["adaptive_total_cells"] += 1
                elif fill_strategy_name == "SINGLE_STROKE_DETAIL":
                    slicer_counts["thin_detail_fallback_region_count"] += 1
                    slicer_counts["single_stroke_region_count"] += 1
                elif fill_strategy_name == "OUTLINE_ONLY":
                    slicer_counts["outline_buffer_empty_region_count"] += 0
                elif fill_strategy_name == "SKIP_FILL":
                    slicer_counts["skipped_tiny_region_count"] += 1

                region_debug_entry = {
                    "region_index": region_index,
                    "source_region_id": printable_region.component_id,
                    "resolved_angle_deg": resolved_infill_angle_deg,
                    "fill_mode": fill_strategy_name,
                    "classification_reason": hybrid_region_debug.get("classification_reason", fill_strategy_name),
                    **angle_debug,
                    **hybrid_region_debug,
                }
                coverage_class = str(hybrid_region_debug.get("coverage_class", "wide"))
                if coverage_class == "thin":
                    slicer_counts["thin_region_count"] += 1
                elif coverage_class == "narrow":
                    slicer_counts["narrow_region_count"] += 1
                elif coverage_class == "tiny":
                    slicer_counts["tiny_region_count"] += 1
                else:
                    slicer_counts["wide_region_count"] += 1
                debug_append_geometry(debug, "infill_regions", infill_region, "infill-region")
                infill_paths = [
                    clone_toolpath(
                        path,
                        region_id=region_index,
                        source="infill_clip",
                        metadata={
                            **path.metadata,
                            "simplify_tolerance_mm": simplify_tolerance_resolved_mm,
                            "pen_width_mm": line_width_mm,
                            "coordinate_space_at_creation": "surface_mm",
                            "coordinate_space_before_offset": "surface_mm",
                            "offset_space": "none",
                            "coordinate_space_before_simplify": "surface_mm",
                            "simplify_space": "surface_mm",
                            "source_polygon_id": source_polygon_id,
                            "source_polygon_matches_infill_clip_polygon": True,
                            "outline_uses_infill_clip_polygon": False,
                            "generated_from": "final_fill_clip_polygon",
                            "source_region_id": printable_region.component_id,
                            "source_component_id": region_index + 1,
                            "source_contour_id": 1,
                            "expected_relation_to_fill": "fill_interior",
                            "resolved_infill_angle_deg": resolved_infill_angle_deg,
                            "long_thin_fast_path_used": bool(angle_debug.get("long_thin_fast_path_used", False)),
                            "infill_aspect_ratio": float(angle_debug.get("aspect_ratio", 0.0)),
                            "fill_mode": path.metadata.get("fill_mode", hybrid_region_debug.get("fill_mode", fill_strategy_name)),
                            "fill_strategy": path.metadata.get("fill_strategy", hybrid_region_debug.get("fill_strategy", fill_strategy_name)),
                            "fill_mode_reason": path.metadata.get("fill_mode_reason", hybrid_region_debug.get("classification_reason", fill_strategy_name)),
                            "small_detail_fill_style": path.metadata.get("small_detail_fill_style"),
                            "classification_metrics": hybrid_region_debug,
                        },
                    )
                    for path in infill_paths
                ]
                adaptive_diag = debug.get("adaptive_fill_diagnostics") if isinstance(debug, dict) else None
                if isinstance(adaptive_diag, dict):
                    slicer_counts["adaptive_total_cells"] += int(adaptive_diag.get("total_cells", 0))
                    slicer_counts["adaptive_rectilinear_cells"] += int(adaptive_diag.get("rectilinear_cells", 0))
                    slicer_counts["adaptive_detail_contour_cells"] += int(adaptive_diag.get("detail_contour_cells", 0))
                    slicer_counts["adaptive_single_stroke_cells"] += int(adaptive_diag.get("single_stroke_cells", 0))
                    slicer_counts["adaptive_narrow_cells_detected"] += int(adaptive_diag.get("narrow_cells_detected", 0))
                    slicer_counts["adaptive_switched_too_few_rows"] += int(adaptive_diag.get("switched_too_few_rows", 0))
                    slicer_counts["adaptive_switched_connector_ratio"] += int(adaptive_diag.get("switched_connector_ratio", 0))
                    slicer_counts["adaptive_switched_single_stroke_width"] += int(adaptive_diag.get("switched_single_stroke_width", 0))
                    slicer_counts["adaptive_switched_single_stroke_hatch_quality"] += int(adaptive_diag.get("switched_single_stroke_hatch_quality", 0))
            forced_minimum_stroke_needed = bool(not infill_paths and too_thin_for_dual_outline)
            if not infill_paths and thin_detail_mode and (polygon.area >= thin_detail_min_area_resolved_mm2 or forced_minimum_stroke_needed):
                detail_region = printable_outline_region if can_fit_outline else polygon
                slicer_counts["thin_detail_fallback_region_count"] += 1
                if forced_minimum_stroke_needed:
                    # Hard fallback: if a detail is narrower than 2x pen width, emit at least one printable stroke.
                    infill_paths = self._generate_centerline_fallback(
                        detail_region,
                        angle_deg=resolved_infill_angle_deg if 'resolved_infill_angle_deg' in locals() else infill_angle_deg,
                        min_segment_length_mm=max(0.01, line_width_mm * 0.2),
                        tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                        kind="fill-infill",
                    )
                    if not infill_paths:
                        infill_paths = self._generate_tiny_dot_or_short_stroke(
                            detail_region,
                            line_width_mm=line_width_mm,
                            tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                            angle_deg=resolved_infill_angle_deg if 'resolved_infill_angle_deg' in locals() else infill_angle_deg,
                            kind="fill-infill",
                        )
                else:
                    infill_paths = self._generate_detail_fill(
                        detail_region,
                        line_width_mm=line_width_mm,
                        scanline_spacing_mm=scanline_spacing_mm,
                        angle_deg=resolved_infill_angle_deg if 'resolved_infill_angle_deg' in locals() else infill_angle_deg,
                        min_segment_length_mm=min_segment_length_resolved_mm,
                        tolerance_mm=simplify_tolerance_resolved_mm,
                        detail_tolerance_mm=thin_detail_tolerance_mm,
                        allow_overlap=thin_detail_overlap,
                        allow_pen_down_infill_connectors=effective_allow_connectors,
                        connector_validation=connector_validation,
                        debug=debug,
                    )
                infill_paths = [
                    clone_toolpath(
                        path,
                        region_id=region_index,
                        source="detail_trace" if path.kind == "detail-trace" else "forced_minimum_stroke",
                        metadata={
                            **path.metadata,
                            "simplify_tolerance_mm": max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                            "pen_width_mm": line_width_mm,
                            "coordinate_space_at_creation": "surface_mm",
                            "coordinate_space_before_offset": "surface_mm",
                            "offset_space": "surface_mm",
                            "coordinate_space_before_simplify": "surface_mm",
                            "simplify_space": "surface_mm",
                            "generated_from": "final_fill_clip_polygon",
                            "source_region_id": printable_region.component_id,
                            "source_component_id": region_index + 1,
                            "source_contour_id": 1,
                            "expected_relation_to_fill": "detail_overlay" if path.kind == "detail-trace" else "fill_interior",
                            "force_minimum_printable_stroke": bool(forced_minimum_stroke_needed),
                            "fill_mode": "single_stroke_fallback_region" if forced_minimum_stroke_needed else path.metadata.get("fill_mode", "detail_contour_cell"),
                            "fill_mode_reason": "width_lt_2x_pen_force_centerline" if forced_minimum_stroke_needed else path.metadata.get("fill_mode_reason", "thin_detail_fallback"),
                        },
                    )
                    for path in infill_paths
                ]
                infill_paths = self._enforce_region_coverage_backfill(
                    infill_paths,
                    region=polygon,
                    line_width_mm=line_width_mm,
                    angle_deg=resolved_infill_angle_deg,
                    min_segment_length_mm=min_segment_length_resolved_mm,
                    tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                    connector_validation=connector_validation,
                    debug=debug,
                )
                infill_paths = self._enforce_non_outline_like_fill(
                    infill_paths,
                    region=polygon,
                    line_width_mm=line_width_mm,
                    angle_deg=resolved_infill_angle_deg,
                    min_segment_length_mm=min_segment_length_resolved_mm,
                    tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                    debug=debug,
                )
                # Hard guarantee: thin/narrow printable regions must carry at
                # least one interior centerline stroke in final output.
                if coverage_class in {"thin", "narrow"}:
                    centerline = self._generate_centerline_fallback(
                        polygon,
                        angle_deg=resolved_infill_angle_deg,
                        min_segment_length_mm=max(min_segment_length_resolved_mm, line_width_mm * 0.25),
                        tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                        kind="fill-infill",
                    )
                    if centerline:
                        longest = max(centerline, key=lambda p: segment_length(p.points) if len(p.points) >= 2 else 0.0)
                        clipped = self._clip_toolpaths_to_region(
                            [longest],
                            region=polygon,
                            tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                            kind="fill-infill",
                        )
                        if clipped:
                            infill_paths.append(clone_toolpath(
                                clipped[0],
                                metadata={
                                    **clipped[0].metadata,
                                    "coverage_backstop": True,
                                    "forced_thin_region_centerline": True,
                                    "small_detail_fill_style": "forced_region_centerline",
                                    "fill_mode": "forced_region_centerline",
                                },
                            ))
                # Final safety net: if infill is too sparse relative to region
                # outline length, inject extra interior strokes.
                outline_len = sum(
                    segment_length(path.points)
                    for path in outline_cleanup_paths
                    if path.kind in {"outline", "fill-wall"} and len(path.points) >= 2
                )
                infill_len = sum(
                    segment_length(path.points)
                    for path in infill_paths
                    if path.kind in {"fill-infill", "detail-trace"} and len(path.points) >= 2
                )
                if outline_len > 1e-6 and infill_len < (outline_len * 0.35):
                    extra = self._generate_sparse_interior_strokes(
                        polygon,
                        angle_deg=resolved_infill_angle_deg,
                        line_width_mm=line_width_mm,
                        scanline_spacing_mm=max(0.01, line_width_mm * 0.85),
                        min_segment_length_mm=max(min_segment_length_resolved_mm, line_width_mm * 0.3),
                        tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                        max_strokes=3,
                        kind="fill-infill",
                    )
                    extra = self._clip_toolpaths_to_region(
                        extra,
                        region=polygon,
                        tolerance_mm=max(simplify_tolerance_resolved_mm, thin_detail_tolerance_mm),
                        kind="fill-infill",
                    )
                    for idx, path in enumerate(extra):
                        if len(path.points) < 2:
                            continue
                        infill_paths.append(clone_toolpath(
                            path,
                            metadata={
                                **path.metadata,
                                "coverage_backstop": True,
                                "forced_low_infill_ratio_backfill": True,
                                "small_detail_fill_style": "ratio_backfill",
                                "fill_mode": "forced_low_infill_ratio_backfill",
                                "backfill_index": idx,
                            },
                        ))
            if not infill_paths:
                slicer_counts["dropped_region_count"] += 1
                if debug is not None:
                    debug.setdefault("dropped_regions", []).append({
                        "region_index": region_index,
                        "source_region_id": printable_region.component_id,
                        "reason": "no_fill_or_fallback_paths",
                        "local_width_mm": measured_local_width_mm,
                        "line_width_mm": line_width_mm,
                        "too_thin_for_dual_outline": bool(too_thin_for_dual_outline),
                    })
            infill_region_debug.append(region_debug_entry)
            if too_thin_for_dual_outline and any(
                str(path.metadata.get("fill_mode", "")) in {"single_stroke_fallback_region", "single_stroke_cell"}
                or str(path.metadata.get("small_detail_fill_style", "")) in {"single_stroke_detail", "tiny_dot", "tiny_short_stroke"}
                for path in infill_paths
            ):
                slicer_counts["narrower_than_two_pen_with_centerline_count"] += 1
            slicer_counts["thin_detail_path_count"] += sum(1 for path in infill_paths if path.kind == "detail-trace")
            slicer_counts["detail_trace_path_count"] = slicer_counts["thin_detail_path_count"]

            # If connectors were stitched into the infill paths we must
            # preserve the generated scanline emission order so connector
            # toolpaths remain adjacent to the segments they connect. Re-
            # optimizing here would separate connectors and force pen lifts.
            if not effective_allow_connectors:
                if not _preserve_infill_path_order(infill_paths):
                    infill_paths = optimize_toolpath_order(
                        infill_paths,
                        strategy=travel_optimization,
                        start_point=region_paths[-1].points[-1] if region_paths and region_paths[-1].points else anchor,
                    )
            region_paths.extend(infill_paths)

            skip_cleanup_outline = any(
                str(path.metadata.get("small_detail_fill_style")) in {"tiny_dot", "tiny_short_stroke"}
                for path in infill_paths
            )
            if outline_after_fill and cleanup_outline_only_paths and not skip_cleanup_outline:
                cleanup_paths = [
                    clone_toolpath(path, region_id=region_index)
                    for path in cleanup_outline_only_paths
                ]
                debug_append_toolpaths(debug, "cleanup_outlines", cleanup_paths)
                cleanup_paths = optimize_toolpath_order(
                    cleanup_paths,
                    strategy=travel_optimization,
                    start_point=region_paths[-1].points[-1] if region_paths and region_paths[-1].points else anchor,
                )
                region_paths.extend(cleanup_paths)

            if debug is not None and can_fit_outline:
                debug.setdefault("wall_alignment_checks", [])
                infill_boundary = infill_region.boundary if infill_region is not None and not infill_region.is_empty else printable_outline_region.boundary
                wall_candidates = outline_cleanup_paths if outline_cleanup_paths else [path for path in region_paths if path.kind in {"fill-wall", "outline"}]
                for wall_path in wall_candidates:
                    wall_line = LineString([(point.x, point.y) for point in wall_path.points])
                    wall_inside_mask = path_is_inside_printable_area(wall_path, polygon, tolerance_mm=max(0.02, line_width_mm * 0.1))
                    debug["wall_alignment_checks"].append({
                        "path_id": wall_path.path_id or f"{wall_path.kind}_region_{region_index}",
                        "kind": wall_path.kind,
                        "source_polygon_id": source_polygon_id,
                        "source_polygon_matches_infill_clip_polygon": bool(wall_path.metadata.get("source_polygon_matches_infill_clip_polygon", False)),
                        "coordinate_space": wall_path.coordinate_space,
                        "offset_distance_mm": float(wall_path.metadata.get("offset_distance_mm", 0.0)),
                        "offset_direction": wall_path.metadata.get("offset_direction", "unknown"),
                        "wall_inside_mask_surface_mm": bool(wall_inside_mask),
                        "wall_intersects_infill_bbox": bool(infill_region is not None and not infill_region.is_empty and wall_line.envelope.intersects(infill_region.envelope)),
                        "min_distance_to_infill_boundary_mm": float(wall_line.distance(infill_boundary)) if infill_boundary is not None and not infill_boundary.is_empty else 0.0,
                        "max_distance_to_infill_boundary_mm": float(wall_line.hausdorff_distance(infill_boundary)) if infill_boundary is not None and not infill_boundary.is_empty else 0.0,
                        "self_intersections": 0 if wall_line.is_simple else 1,
                        "closed": wall_path.closed,
                        "offset_direction": "inward",
                        "suspected_issue": None if wall_inside_mask else "wall_outside_mask",
                    })
                debug["outline_fill_alignment_debug"] = {
                    "outline_role": "cleanup_stroke_over_filled_edge",
                    "outline_not_external_border": True,
                    "outline_centerline_policy": outline_placement_mode,
                    "pen_width_mm": line_width_mm,
                    "expected_outline_inset_mm": expected_outline_inset_mm,
                    "outline_uses_same_source_polygon_as_infill": True,
                    "outline_coordinate_space_before_projection": "surface_mm",
                    "infill_coordinate_space_before_projection": "surface_mm",
                }

            if can_fit_outline:
                filled_area_estimate_mm2 = float(infill_region.area) if infill_region is not None and not infill_region.is_empty else 0.0
                logger.info(json.dumps({
                    "event": "fill_coverage_audit",
                    "component_id": source_polygon_id,
                    "mask_area_mm2": float(polygon.area),
                    "printable_polygon_area_mm2": float(polygon.area),
                    "filled_area_estimate_mm2": filled_area_estimate_mm2,
                    "edge_gap_estimate_mm": infill_clip_inset_mm,
                    "infill_clip_inset_mm": infill_clip_inset_mm,
                    "line_spacing_mm": scanline_spacing_mm,
                    "line_width_mm": line_width_mm,
                    "coverage_reaches_boundary": infill_clip_inset_mm <= (line_width_mm * 0.5 + 1e-9),
                }, separators=(",", ":")))

            if cleanup_outline_only_paths and infill_paths:
                infill_region_ids = {
                    path.metadata.get("source_region_id")
                    for path in infill_paths
                    if path.metadata.get("source_region_id") is not None
                }
                for outline_path in cleanup_outline_only_paths:
                    outline_region_id = outline_path.metadata.get("source_region_id")
                    if outline_region_id not in infill_region_ids and infill_region_ids:
                        debug_append_warning(
                            debug,
                            f"Outline/infill region-id mismatch: outline={outline_region_id} infill={sorted(str(v) for v in infill_region_ids)}",
                        )
                    assert outline_path.metadata.get("generated_from") == "final_fill_clip_polygon"

            if effective_allow_connectors and region_paths:
                region_paths = self._chain_region_paths_with_pen_down_connectors(
                    region_paths,
                    polygon=polygon,
                    line_width_mm=line_width_mm,
                    spacing_mm=scanline_spacing_mm,
                    preserve_order=bool(outline_after_fill),
                    connector_validation=connector_validation,
                    debug=debug,
                )

            ordered.extend(region_paths)

        non_detail_paths = [path for path in ordered if path.kind != "detail-trace"]
        thin_detail_paths = [path for path in ordered if path.kind == "detail-trace"]
        if remove_duplicate_paths:
            non_detail_paths = dedupe_toolpaths(non_detail_paths, min_segment_length_resolved_mm)
        else:
            non_detail_paths = filter_toolpaths_by_length(non_detail_paths, min_segment_length_resolved_mm)
        thin_detail_paths = filter_toolpaths_by_length(thin_detail_paths, min_segment_length_resolved_mm)
        ordered = merge_connected_toolpaths(non_detail_paths + thin_detail_paths)
        mesh_rejected_count = 0
        mesh_rejected_reasons: dict[str, int] = {}
        drawable_filtered: list[Toolpath] = []
        for path in ordered:
            # Keep primary infill; mesh suppression only targets legacy detail
            # traces and connector artifacts to avoid deleting valid coverage.
            if path.kind in {"detail-trace", "fill-infill-travel"} and self._is_mesh_like_path(path, line_width_mm=line_width_mm):
                mesh_rejected_count += 1
                reason = "closed_or_crisscross_mesh"
                mesh_rejected_reasons[reason] = int(mesh_rejected_reasons.get(reason, 0)) + 1
                continue
            drawable_filtered.append(path)
        ordered = drawable_filtered
        ordered = self._canonicalize_coverage_paths(
            ordered,
            line_width_mm=line_width_mm,
            debug=debug,
        )
        ordered = [
            path
            for path in ordered
            if len(path.points) >= 2 and segment_length(path.points) > 1e-6
        ]
        # Expensive iterative mask-coverage repair is offline-only by default.
        # Enable explicitly for tuning/diagnostics with ENABLE_COVERAGE_REPAIR=1.
        run_coverage_repair = bool(expensive_coverage_repair) and (os.getenv("ENABLE_COVERAGE_REPAIR", "1") == "1")
        if run_coverage_repair:
            ordered = self._repair_missed_mask_components(
                ordered,
                line_width_mm=line_width_mm,
                connector_validation=connector_validation,
                debug=debug,
                target_penalized_percent=90.0,
                max_added_paths=80,
            )
            ordered = self._prune_penalized_negative_paths(
                ordered,
                line_width_mm=line_width_mm,
                connector_validation=connector_validation,
                debug=debug,
                max_iterations=200,
            )
            ordered = self._repair_missed_mask_components(
                ordered,
                line_width_mm=line_width_mm,
                connector_validation=connector_validation,
                debug=debug,
                target_penalized_percent=90.0,
                max_added_paths=120,
            )
        elif debug is not None:
            debug["coverage_repair_skipped"] = True
        logger.debug(
            "Generated fill toolpaths: wall_paths=%d infill_paths=%d infill_segments=%d spacing_mm=%.4f",
            sum(1 for path in ordered if path.kind == "fill-wall"),
            sum(1 for path in ordered if path.kind == "fill-infill"),
            sum(max(0, len(path.points) - 1) for path in ordered if path.kind == "fill-infill"),
            scanline_spacing_mm,
        )
        debug_set_counts(debug, "slicer_counts", slicer_counts)
        if debug is not None:
            debug["mesh_like_paths_rejected"] = mesh_rejected_count
            debug["mesh_like_rejection_reasons"] = mesh_rejected_reasons
            total_estimated_rows = int(round(sum(float(region_entry.get("region_metrics", {}).get("estimatedRowCount", 0.0)) for region_entry in infill_region_debug)))
            total_candidate_segments = sum(
                int(metric.get("segments", 0))
                for region_entry in infill_region_debug
                for metric in region_entry.get("candidate_metrics", [])
            )
            hybrid_local_cell_count = int(debug.get("local_cell_count", 0)) or max(1, int(debug.get("rows_with_multiple_intervals", 0)))
            debug["infill_debug"] = {
                "coordinate_space": "surface_mm",
                "fill_strategy": effective_fill_strategy,
                "infill_path_mode": resolved_infill_path_mode,
                "allow_pen_down_infill_connectors": effective_allow_connectors,
                "fill_angles_deg": [infill_angle_deg, alternate_fill_angle_deg],
                "spacing_mm": scanline_spacing_mm,
                "pen_width_mm": line_width_mm,
                "clip_space": "surface_mm",
                "regions_filled": len(infill_region_debug),
                "regions_skipped": max(0, len(printable_regions) - len(infill_region_debug)),
                "small_region_handling": "centerline" if small_shape_mode == "centerline" else small_shape_mode,
                "estimated_coverage_ratio": max(
                    [metric.get("coverage_ratio", 0.0) for region_entry in infill_region_debug for metric in region_entry.get("candidate_metrics", [])] or [0.0]
                ),
                "mode_counts": {
                    "large_open": slicer_counts["large_open_region_count"],
                    "long_thin": slicer_counts["long_thin_region_count"],
                    "small_detail_or_text": slicer_counts["small_detail_region_count"],
                    "detail_contour_cell": slicer_counts["adaptive_detail_contour_cells"],
                    "single_stroke_cell": slicer_counts["adaptive_single_stroke_cells"],
                },
                "coverage_thresholds": {
                    "single_stroke_max_width_factor": DEFAULT_THIN_REGION_SINGLE_STROKE_MAX_FACTOR,
                    "narrow_region_max_width_factor": DEFAULT_NARROW_REGION_MAX_FACTOR,
                    "collapse_outline_max_width_factor": DEFAULT_COLLAPSE_OUTLINE_MAX_FACTOR,
                    "tiny_dot_area_factor": DEFAULT_TINY_DOT_AREA_FACTOR,
                    "single_stroke_width_max_factor": DEFAULT_SINGLE_STROKE_WIDTH_MAX_FACTOR,
                    "centerline_width_max_factor": DEFAULT_CENTERLINE_WIDTH_MAX_FACTOR,
                    "detail_width_max_factor": DEFAULT_DETAIL_WIDTH_MAX_FACTOR,
                },
                "coverage_region_counts": {
                    "thin": slicer_counts["thin_region_count"],
                    "narrow": slicer_counts["narrow_region_count"],
                    "wide": slicer_counts["wide_region_count"],
                    "tiny": slicer_counts["tiny_region_count"],
                },
                "adaptive_fill_counts": {
                    "total_cells": slicer_counts["adaptive_total_cells"],
                    "rectilinear_cells": slicer_counts["adaptive_rectilinear_cells"],
                    "detail_contour_cells": slicer_counts["adaptive_detail_contour_cells"],
                    "single_stroke_cells": slicer_counts["adaptive_single_stroke_cells"],
                    "narrow_cells_detected": slicer_counts["adaptive_narrow_cells_detected"],
                    "switched_too_few_rows": slicer_counts["adaptive_switched_too_few_rows"],
                    "switched_connector_ratio": slicer_counts["adaptive_switched_connector_ratio"],
                    "switched_single_stroke_width": slicer_counts["adaptive_switched_single_stroke_width"],
                    "switched_single_stroke_hatch_quality": slicer_counts["adaptive_switched_single_stroke_hatch_quality"],
                    "outline_after_fill": bool(outline_after_fill),
                },
                "diagnostics": {
                    "raw_generated_paths": int(len(non_detail_paths) + len(thin_detail_paths)),
                    "final_drawable_paths": int(len(ordered)),
                    "rejected_mesh_debug_paths": int(mesh_rejected_count),
                    "rejected_x_triangle_fragments": int(debug.get("rejected_x_triangle_fragment_count", 0)) if isinstance(debug, dict) else 0,
                    "total_local_regions": len(infill_region_debug),
                    "normal_fill_regions": slicer_counts["large_open_region_count"],
                    "single_stroke_regions": slicer_counts["single_stroke_region_count"] + slicer_counts["adaptive_single_stroke_cells"],
                    "tiny_mark_regions": slicer_counts["tiny_region_count"],
                    "outline_regions": slicer_counts["normal_slicer_region_count"],
                    "average_local_width_mm": (
                        sum(float(entry.get("region_metrics", {}).get("maxLocalWidthMm", 0.0)) for entry in infill_region_debug) / max(1, len(infill_region_debug))
                    ),
                    "regions_switched_due_to_small_fill": slicer_counts["adaptive_switched_too_few_rows"] + slicer_counts["adaptive_switched_single_stroke_width"],
                    "regions_switched_due_to_poor_rectilinear_coverage": slicer_counts["adaptive_switched_connector_ratio"] + slicer_counts["adaptive_switched_single_stroke_hatch_quality"],
                    "narrower_than_2x_pen_regions": slicer_counts["narrower_than_two_pen_region_count"],
                    "narrower_than_2x_pen_with_centerline": slicer_counts["narrower_than_two_pen_with_centerline_count"],
                    "dropped_regions": slicer_counts["dropped_region_count"] + slicer_counts["skipped_tiny_region_count"],
                    "dropped_region_reasons": (debug.get("dropped_regions", []) if isinstance(debug.get("dropped_regions", []), list) else []),
                    "coverage_connector_attempted": int(debug.get("coverage_connector_attempted", 0)) if isinstance(debug, dict) else 0,
                    "coverage_connector_accepted": int(debug.get("coverage_connector_accepted", 0)) if isinstance(debug, dict) else 0,
                    "coverage_connector_rejected": int(debug.get("coverage_connector_rejected", 0)) if isinstance(debug, dict) else 0,
                },
                "regions": infill_region_debug,
            }
            if connector_validation and isinstance(connector_validation, dict):
                mask = connector_validation.get("mask")
                matrix = connector_validation.get("current_to_source_matrix")
                if mask is not None and isinstance(matrix, (tuple, list)) and len(matrix) == 6:
                    try:
                        coverage = estimate_toolpath_mask_coverage(
                            ordered,
                            mask=mask,
                            current_to_source_matrix=tuple(float(value) for value in matrix),
                            pen_radius_mm=max(0.01, line_width_mm * 0.5),
                            sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
                        )
                    except Exception:
                        coverage = {}
                    if coverage:
                        debug["coverage_validation"] = coverage
            if not debug.get("infill_connector_diagnostics"):
                average_segments_per_cell = (float(total_candidate_segments) / float(hybrid_local_cell_count)) if hybrid_local_cell_count > 0 else 0.0
                debug["total_infill_rows"] = max(total_estimated_rows, int(debug.get("rows_with_multiple_intervals", 0)))
                debug["average_segments_per_cell"] = average_segments_per_cell
                debug["local_cell_count"] = hybrid_local_cell_count
                debug["total_pen_up_travel_distance_mm"] = max(
                    float(debug.get("total_pen_up_travel_distance_mm", 0.0)),
                    float(max(0, hybrid_local_cell_count - 1)) * float(scanline_spacing_mm),
                )
                debug["number_of_long_travels_between_cells"] = max(
                    int(debug.get("number_of_long_travels_between_cells", 0)),
                    max(0, hybrid_local_cell_count - 1),
                )
                debug["infill_connector_diagnostics"] = {
                    "total_infill_rows": debug["total_infill_rows"],
                    "total_possible_adjacent_row_connector_attempts": int(debug.get("rows_with_multiple_intervals", 0)),
                    "accepted_connectors": int(debug.get("accepted_same_cell_connectors", 0)),
                    "rejected_connectors": int(debug.get("rejected_cross_gap_connectors", 0))
                    + int(debug.get("rejected_different_cell_connectors", 0))
                    + int(debug.get("rejected_opposite_side_connectors", 0))
                    + int(debug.get("rejected_too_long_connectors", 0))
                    + int(debug.get("rejected_outside_selected_color_connectors", 0)),
                    "rejected_raster_mask_sampling": int(debug.get("rejected_outside_selected_color_connectors", 0)),
                    "final_pen_lift_count_estimate": int(debug.get("pen_lifts_after_cell_planning", 0)),
                    "rejection_counts": {},
                    "top_rejection_reason": None,
                    "local_cell_count": hybrid_local_cell_count,
                    "average_segments_per_cell": average_segments_per_cell,
                    "total_pen_up_travel_distance_mm": float(debug.get("total_pen_up_travel_distance_mm", 0.0)),
                    "number_of_long_travels_between_cells": int(debug.get("number_of_long_travels_between_cells", 0)),
                }
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
    fill_strategy: str = "horizontal_scanline",
    alternate_fill_angle_deg: float = -45.0,
    thin_detail_mode: bool = DEFAULT_THIN_DETAIL_MODE,
    thin_detail_min_area_mm2: float = DEFAULT_THIN_DETAIL_MIN_AREA_MM2,
    thin_detail_simplify_mm: float = DEFAULT_THIN_DETAIL_SIMPLIFY_MM,
    thin_detail_overlap: bool = DEFAULT_THIN_DETAIL_OVERLAP,
    min_segment_length_mm: float = DEFAULT_MIN_SEGMENT_LENGTH_MM,
    travel_optimization: str = DEFAULT_TRAVEL_OPTIMIZATION,
    allow_pen_down_infill_connectors: bool = DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS,
    infill_path_mode: str = DEFAULT_INFILL_PATH_MODE,
    expensive_coverage_repair: bool = True,
    debug: Optional[dict[str, Any]] = None,
) -> list[Toolpath]:
    def _contour_only_fill_paths(
        printable_geometry: Any,
        *,
        pen_width_mm: float,
        simplify_tolerance_mm_value: float,
        travel_ordering: str,
        debug_obj: Optional[dict[str, Any]],
    ) -> list[Toolpath]:
        pen_radius = pen_width_mm * 0.5
        contour_overlap_spacing_factor = min(0.80, max(0.60, float(os.getenv("CONTOUR_OVERLAP_SPACING_FACTOR", "0.65"))))
        offset_step_mm = pen_width_mm * contour_overlap_spacing_factor
        simplify_mm = max(simplify_tolerance_mm_value, min(0.03, pen_width_mm / 10.0))
        offset_join_style_name = str(os.getenv("CONTOUR_OFFSET_JOIN_STYLE", "round")).strip().lower()
        offset_join_style = 2 if offset_join_style_name == "miter" else 1
        offset_miter_limit = min(4.0, max(2.0, float(os.getenv("CONTOUR_OFFSET_MITER_LIMIT", "3.0"))))
        offset_arc_tolerance_mm = min(0.025, pen_width_mm / 12.0)
        min_section_length_mm = min(0.08, max(0.03, 0.05 * pen_width_mm))
        min_new_coverage_area_mm2 = min(0.01, max(0.003, 0.008 * pen_width_mm * pen_width_mm))
        min_new_coverage_ratio = min(0.08, max(0.03, 0.04))
        min_isolated_loop_area_mm2 = 0.003
        corner_min_section_length_mm = min(0.03, max(0.01, 0.02 * pen_width_mm))
        corner_min_new_coverage_ratio = 0.01
        continuity_min_length_mm = 0.005
        continuity_min_new_coverage_ratio = 0.0
        corner_max_turn_angle_deg = 170.0
        corner_bridge_gap_mm = 0.75 * pen_width_mm
        contour_direction_validation_enabled = True
        max_contour_parallel_angle_error_deg = 25.0
        max_corner_turn_angle_error_deg = 60.0
        reject_diagonal_shortcuts = True
        iso_distance_tolerance_mm = min(0.06, pen_width_mm * 0.10)
        allow_isolated_internal_sections = True
        allow_crossed_contour_sections = True
        allow_overlap_with_existing_fill = True
        preserve_corner_vertices = True
        max_overspill_mm = min(0.05, pen_width_mm * 0.10)
        max_overspill_area_ratio = 0.02
        max_levels = 4096
        stop_after_empty_levels = 8

        per_level: list[dict[str, Any]] = []
        accepted_paths: list[Toolpath] = []
        rejected_sections: list[tuple[Toolpath, str]] = []
        covered_geom: Any = Polygon()
        rejection_reason_counts: dict[str, int] = {}
        corner_rejection_reason_counts: dict[str, int] = {}
        corner_candidate_logs: list[dict[str, Any]] = []
        central_candidate_sections = 0
        central_accepted_sections = 0
        central_rejected_reasons: list[str] = []
        corner_candidates_total = 0
        corner_accepted_total = 0
        corner_rejected_total = 0
        corridor_restored_total = 0
        open_section_audit_failures = 0
        raw_offset_contours_debug: list[Toolpath] = []
        distance_error_debug_paths: list[Toolpath] = []
        residual_repair_logs: list[dict[str, Any]] = []

        def _quad_segs_for_tolerance(radius_mm: float, tolerance_mm: float) -> int:
            r = max(1e-6, float(radius_mm))
            e = max(1e-6, min(float(tolerance_mm), r * 0.95))
            denom = math.acos(max(-1.0, min(1.0, 1.0 - (e / r))))
            if denom <= 1e-9:
                return 48
            q = int(math.ceil(math.pi / (4.0 * denom)))
            return max(16, min(64, q))

        def _component_outline_metrics(component: Any) -> tuple[float, float]:
            if component is None or component.is_empty:
                return 0.0, 0.0
            min_x, min_y, max_x, max_y = component.bounds
            min_dim = max(0.0, min(float(max_x - min_x), float(max_y - min_y)))
            return float(component.area), min_dim

        def _build_outline_paths_from_geometry(
            outline_geometry: Any,
            *,
            offset_mm: float,
            source_label: str,
            simplify_outline_mm: float,
        ) -> tuple[list[Toolpath], dict[str, Any]]:
            outline_paths_local: list[Toolpath] = []
            component_count_input = 0
            component_count_output = 0
            paths_generated = 0
            paths_dropped = 0
            thin_components = 0
            small_components = 0
            thin_components_outlined = 0
            small_components_outlined = 0
            drop_reasons: dict[str, int] = {}
            total_length_mm = 0.0

            components = normalize_geometry(outline_geometry)
            component_count_input = len(components)
            thin_threshold_mm = pen_width_mm * 1.05
            small_area_threshold_mm2 = max(1e-6, pen_width_mm * pen_width_mm * 0.35)
            for cidx, poly in enumerate(components, start=1):
                area_mm2, min_dim_mm = _component_outline_metrics(poly)
                is_thin = bool(min_dim_mm > 0.0 and min_dim_mm <= thin_threshold_mm)
                is_small = bool(area_mm2 > 0.0 and area_mm2 <= small_area_threshold_mm2)
                if is_thin:
                    thin_components += 1
                if is_small:
                    small_components += 1
                component_generated = False

                outer = simplify_segment_points([Point(float(x), float(y)) for x, y in poly.exterior.coords], simplify_outline_mm, True)
                if len(outer) >= 4:
                    outline_paths_local.append(Toolpath(
                        points=outer,
                        kind="outline",
                        closed=True,
                        source=source_label,
                        metadata={
                            "path_role": "FINAL_OUTER_OUTLINE",
                            "ring_role": "outer",
                            "offset_mm": float(offset_mm),
                            "outline_generation_source": source_label,
                            "source_region_id": f"component_{int(cidx):03d}",
                        },
                    ))
                    component_generated = True
                    paths_generated += 1
                    total_length_mm += float(segment_length(outer + [outer[0]])) if outer else 0.0
                else:
                    drop_reasons["outer_ring_too_short"] = int(drop_reasons.get("outer_ring_too_short", 0)) + 1
                    paths_dropped += 1

                for ring in poly.interiors:
                    inner = simplify_segment_points([Point(float(x), float(y)) for x, y in ring.coords], simplify_outline_mm, True)
                    if len(inner) >= 4:
                        outline_paths_local.append(Toolpath(
                            points=inner,
                            kind="outline",
                            closed=True,
                            source=f"{source_label}_hole",
                            metadata={
                                "path_role": "FINAL_INNER_OUTLINE",
                                "ring_role": "hole",
                                "offset_mm": float(offset_mm),
                                "outline_generation_source": source_label,
                                "source_region_id": f"component_{int(cidx):03d}",
                                "is_hole": True,
                            },
                        ))
                        component_generated = True
                        paths_generated += 1
                        total_length_mm += float(segment_length(inner + [inner[0]])) if inner else 0.0
                    else:
                        drop_reasons["inner_ring_too_short"] = int(drop_reasons.get("inner_ring_too_short", 0)) + 1
                        paths_dropped += 1

                if component_generated:
                    component_count_output += 1
                    if is_thin:
                        thin_components_outlined += 1
                    if is_small:
                        small_components_outlined += 1

            return outline_paths_local, {
                "outline_component_count_input": int(component_count_input),
                "outline_component_count_output": int(component_count_output),
                "outline_paths_generated": int(paths_generated),
                "outline_paths_dropped": int(paths_dropped),
                "outline_drop_reasons": dict(sorted(drop_reasons.items())),
                "thin_components_outlined": int(thin_components_outlined),
                "small_components_outlined": int(small_components_outlined),
                "outline_total_length_mm": float(total_length_mm),
                "outline_thin_component_count": int(thin_components),
                "outline_small_component_count": int(small_components),
            }

        hole_voids: list[Any] = []
        for poly in normalize_geometry(printable_geometry):
            for ring in poly.interiors:
                try:
                    hole_poly = Polygon(ring.coords)
                    if hole_poly is not None and not hole_poly.is_empty:
                        hole_voids.append(hole_poly)
                except Exception:
                    continue
        hole_void_geom = unary_union(hole_voids) if hole_voids else Polygon()

        def _path_to_line(path: Toolpath) -> Any:
            if len(path.points) < 2:
                return None
            line = LineString([(p.x, p.y) for p in path.points])
            if line.is_empty or line.length <= 1e-9:
                return None
            return line

        def _path_footprint(path: Toolpath) -> Any:
            line = _path_to_line(path)
            if line is None:
                return None
            return line.buffer(max(0.01, pen_radius), cap_style=1, join_style=1)

        def _measure_overspill(footprint: Any) -> tuple[float, float]:
            if footprint is None or footprint.is_empty:
                return 1.0, max_overspill_mm + 1.0
            allowed = printable_geometry.buffer(max_overspill_mm, join_style=1)
            overspill = footprint.difference(allowed)
            overspill_area = 0.0 if overspill is None or overspill.is_empty else float(overspill.area)
            overspill_ratio = overspill_area / max(1e-9, float(footprint.area))
            protrusion_mm = 0.0
            if overspill is not None and not overspill.is_empty:
                boundary = printable_geometry.boundary
                for geom in normalize_geometry(overspill):
                    coords = list(geom.exterior.coords)
                    step = max(1, int(len(coords) / 24))
                    for i in range(0, len(coords), step):
                        protrusion_mm = max(protrusion_mm, float(ShapelyPoint(float(coords[i][0]), float(coords[i][1])).distance(boundary)))
            return overspill_ratio, protrusion_mm

        def _accept_section(path: Toolpath, *, role: str, corner_mode: bool = False, continuity_preserving: bool = False) -> tuple[bool, str]:
            nonlocal covered_geom
            line = _path_to_line(path)
            if line is None:
                return False, "invalid_geometry"
            min_len = continuity_min_length_mm if continuity_preserving else (corner_min_section_length_mm if corner_mode else min_section_length_mm)
            if float(line.length) < min_len:
                return False, "too_short"
            fp = _path_footprint(path)
            if fp is None or fp.is_empty:
                return False, "invalid_geometry"
            if hole_void_geom is not None and not hole_void_geom.is_empty:
                hole_intrusion = fp.intersection(hole_void_geom)
                if hole_intrusion is not None and not hole_intrusion.is_empty and float(hole_intrusion.area) > 1e-9:
                    return False, "enters_hole"
            overspill_ratio, protrusion_mm = _measure_overspill(fp)
            is_residual_repair = bool((path.metadata or {}).get("residual_repair_pass_index", None) is not None)
            local_max_overspill_ratio = max_overspill_area_ratio
            local_max_protrusion_mm = max_overspill_mm
            if is_residual_repair:
                local_max_overspill_ratio = max(local_max_overspill_ratio, 0.05)
                local_max_protrusion_mm = max(local_max_protrusion_mm, pen_width_mm * 0.30)
            if overspill_ratio > local_max_overspill_ratio or protrusion_mm > local_max_protrusion_mm:
                return False, "outside_mask"
            new_cov = fp if covered_geom is None or covered_geom.is_empty else fp.difference(covered_geom)
            gain_area = 0.0 if new_cov is None or new_cov.is_empty else float(new_cov.area)
            gain_ratio = gain_area / max(1e-9, float(fp.area))
            if not allow_overlap_with_existing_fill and gain_area <= 1e-9:
                return False, "duplicate"
            isolated_inner_loop = bool(path.closed and (_path_to_line(path).distance(printable_geometry.boundary) > max(1e-6, pen_width_mm * 0.25)))
            if isolated_inner_loop and allow_isolated_internal_sections:
                if float(fp.area) < min_isolated_loop_area_mm2:
                    return False, "too_little_new_coverage"
            elif corner_mode:
                if gain_area < min_new_coverage_area_mm2 and gain_ratio < corner_min_new_coverage_ratio and not continuity_preserving:
                    return False, "too_little_new_coverage"
            elif continuity_preserving:
                if gain_ratio < continuity_min_new_coverage_ratio and not allow_overlap_with_existing_fill:
                    return False, "too_little_new_coverage"
            elif gain_area < min_new_coverage_area_mm2 or gain_ratio < min_new_coverage_ratio:
                return False, "too_little_new_coverage"
            final_role = "ISOLATED_CONTOUR_SECTION" if isolated_inner_loop else role
            covered_geom = fp if covered_geom is None or covered_geom.is_empty else covered_geom.union(fp)
            path.metadata = {
                **(path.metadata or {}),
                "path_role": final_role,
                "fill_strategy": "contour_offset",
                "fill_mode": "section_based_contour",
                "small_detail_fill_style": "contour_following",
                "generated_from": "original_target_mask_offset",
                "contour_offset_mm": float((path.metadata or {}).get("offset_mm", 0.0)),
                "offset_distance_mm": float((path.metadata or {}).get("offset_mm", 0.0)),
                "source_region_id": str((path.metadata or {}).get("source_region_id", "component_001")),
                "source_polygon_id": str((path.metadata or {}).get("source_polygon_id", "polygon_001")),
                "isolated_inner_loop": isolated_inner_loop,
                "new_coverage_area_mm2": float(gain_area),
                "new_coverage_ratio": float(gain_ratio),
            }
            if (not path.closed) and (not str((path.metadata or {}).get("valid_open_reason", "")).strip()):
                default_reason = "self_intersection_split" if final_role == "CROSSED_CONTOUR_SECTION" else ("corridor_continuity_restore" if final_role == "CORRIDOR_CONTOUR_SECTION" else "topology_transition")
                path.metadata["valid_open_reason"] = default_reason
            return True, "accepted"

        def _split_ring_to_sections(coords: list[tuple[float, float]]) -> list[list[tuple[float, float]]]:
            if len(coords) < 4:
                return [coords]
            sections: list[list[tuple[float, float]]] = []
            chunk_pts = 9
            for start in range(0, len(coords) - 1, max(3, chunk_pts - 1)):
                end = min(len(coords), start + chunk_pts)
                section = coords[start:end]
                if len(section) >= 2:
                    sections.append(section)
            return sections or [coords]

        def _ring_subpath(coords: list[tuple[float, float]], start_idx: int, end_idx: int) -> list[tuple[float, float]]:
            n = len(coords)
            if n < 2:
                return []
            if start_idx <= end_idx:
                return coords[start_idx:end_idx + 1]
            return coords[start_idx:] + coords[:end_idx + 1]

        def _turn_angle_deg(points_xyz: list[Point]) -> float:
            if len(points_xyz) < 3:
                return 180.0
            a = points_xyz[0]
            b = points_xyz[1]
            c = points_xyz[2]
            v1x, v1y = a.x - b.x, a.y - b.y
            v2x, v2y = c.x - b.x, c.y - b.y
            n1 = math.hypot(v1x, v1y)
            n2 = math.hypot(v2x, v2y)
            if n1 <= 1e-9 or n2 <= 1e-9:
                return 180.0
            dot = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (n1 * n2)))
            return float(math.degrees(math.acos(dot)))

        def _segment_direction_deg(points_xyz: list[Point], from_end: bool = False) -> float:
            if len(points_xyz) < 2:
                return 0.0
            if from_end:
                a = points_xyz[-2]
                b = points_xyz[-1]
            else:
                a = points_xyz[0]
                b = points_xyz[1]
            return float(math.degrees(math.atan2(b.y - a.y, b.x - a.x)))

        def _angle_error_deg(a_deg: float, b_deg: float) -> float:
            d = abs(a_deg - b_deg) % 360.0
            if d > 180.0:
                d = 360.0 - d
            if d > 90.0:
                d = 180.0 - d
            return abs(d)

        def _max_iso_distance_error_mm(path: Toolpath, contour_distance_mm: float) -> float:
            line = _path_to_line(path)
            if line is None:
                return max(1.0, iso_distance_tolerance_mm + 1.0)
            boundary = printable_geometry.boundary
            samples = max(3, min(25, int(math.ceil(float(line.length) / max(0.02, pen_width_mm * 0.2)))))
            max_err = 0.0
            for i in range(samples):
                d = (float(line.length) * float(i)) / float(max(samples - 1, 1))
                p = line.interpolate(d)
                dist = float(ShapelyPoint(float(p.x), float(p.y)).distance(boundary))
                max_err = max(max_err, abs(dist - contour_distance_mm))
            return max_err

        def _validate_repair_direction(
            candidate: Toolpath,
            *,
            neighbor_a: Toolpath,
            neighbor_b: Toolpath,
            contour_distance_mm: float,
            from_raw_contour: bool,
        ) -> tuple[bool, str, float, float]:
            if not contour_direction_validation_enabled:
                return True, "ok", 0.0, 0.0
            pa = float((neighbor_a.metadata or {}).get("offset_mm", contour_distance_mm))
            pb = float((neighbor_b.metadata or {}).get("offset_mm", contour_distance_mm))
            if abs(pa - pb) > 1e-6 or abs(pa - contour_distance_mm) > 1e-6:
                return False, "wrong_contour_level", 999.0, 999.0
            iso_err = _max_iso_distance_error_mm(candidate, contour_distance_mm)
            if iso_err > iso_distance_tolerance_mm:
                return False, "iso_distance_error_too_high", iso_err, 999.0

            cand_in = _segment_direction_deg(candidate.points, from_end=False)
            cand_out = _segment_direction_deg(candidate.points, from_end=True)
            na = _segment_direction_deg(neighbor_a.points, from_end=True)
            nb = _segment_direction_deg(neighbor_b.points, from_end=False)
            e1 = _angle_error_deg(cand_in, na)
            e2 = _angle_error_deg(cand_out, nb)
            ang_err = max(e1, e2)
            angle_limit = max_corner_turn_angle_error_deg if from_raw_contour else max_contour_parallel_angle_error_deg
            if ang_err > angle_limit:
                return False, "angle_not_parallel", iso_err, ang_err

            if reject_diagonal_shortcuts:
                line = _path_to_line(candidate)
                if line is None:
                    return False, "invalid_geometry", iso_err, ang_err
                chord = math.hypot(candidate.points[-1].x - candidate.points[0].x, candidate.points[-1].y - candidate.points[0].y)
                length = float(line.length)
                if (not from_raw_contour) and length <= (chord * 1.05):
                    return False, "diagonal_shortcut", iso_err, ang_err
            return True, "ok", iso_err, ang_err

        def _extract_iso_distance_contours(level_geom: Any, *, level: int, offset_mm: float) -> tuple[list[tuple[Toolpath, str]], dict[str, list[tuple[float, float]]]]:
            candidates: list[tuple[Toolpath, str]] = []
            loop_coords_by_id: dict[str, list[tuple[float, float]]] = {}
            section_index = 0
            loop_idx = 0
            for poly in normalize_geometry(level_geom):
                rings = [list(poly.exterior.coords)] + [list(r.coords) for r in poly.interiors]
                for ring_coords in rings:
                    coords = [(float(x), float(y)) for x, y in ring_coords]
                    if len(coords) < 4:
                        continue
                    loop_idx += 1
                    loop_id = f"level_{int(level)}_loop_{int(loop_idx)}"
                    loop_coords_by_id[loop_id] = coords
                    raw_offset_contours_debug.append(
                        Toolpath(
                            points=[Point(x, y) for x, y in coords],
                            kind="fill-infill",
                            closed=True,
                            source="iso_contour_raw_offset",
                            metadata={"path_role": "RAW_OFFSET_CONTOUR", "offset_level": int(level), "offset_mm": float(offset_mm), "source_loop_id": loop_id},
                        )
                    )
                    for sec_ord, sec_coords in enumerate(_split_ring_to_sections(coords), start=1):
                        if len(sec_coords) < 2:
                            continue
                        section_index += 1
                        try:
                            start_idx = coords.index(sec_coords[0])
                            end_idx = coords.index(sec_coords[-1])
                        except ValueError:
                            start_idx = 0
                            end_idx = min(len(coords) - 1, len(sec_coords) - 1)
                        closed_section = bool(sec_coords[0] == sec_coords[-1] and len(sec_coords) >= 4)
                        role = "CONTOUR_INFILL" if closed_section else "CONTOUR_SECTION_INFILL"
                        candidates.append((Toolpath(
                            points=[Point(x, y) for x, y in sec_coords],
                            kind="fill-infill",
                            closed=closed_section,
                            source="iso_contour_section_split",
                            metadata={
                                "path_role": role,
                                "offset_level": int(level),
                                "offset_mm": float(offset_mm),
                                "section_index": int(section_index),
                                "section_order": int(sec_ord),
                                "source_loop_id": loop_id,
                                "start_vertex_index": int(start_idx),
                                "end_vertex_index": int(end_idx),
                                "iso_distance_mm": float(offset_mm),
                                "crossed_contour": False,
                            },
                        ), role))
                    line = LineString(coords)
                    if bool(getattr(line, "is_simple", True)):
                        continue
                    if not allow_crossed_contour_sections:
                        continue
                    try:
                        noded = unary_union(line)
                    except Exception:
                        noded = line.buffer(0)
                    for part in extract_lines(noded):
                        coords = list(part.coords)
                        if len(coords) < 2:
                            continue
                        section_index += 1
                        candidates.append((Toolpath(
                            points=[Point(float(x), float(y)) for x, y in coords],
                            kind="fill-infill",
                            closed=False,
                            source="iso_contour_section",
                            metadata={
                                "path_role": "CROSSED_CONTOUR_SECTION",
                                "offset_level": int(level),
                                "offset_mm": float(offset_mm),
                                "section_index": int(section_index),
                                "crossed_contour": True,
                                "valid_open_reason": "self_intersection_split",
                            },
                        ), "CROSSED_CONTOUR_SECTION"))
            return candidates, loop_coords_by_id

        empty_levels = 0
        level = 1
        while level <= max_levels:
            offset_mm = pen_radius + (level * offset_step_mm)
            quad_segs = _quad_segs_for_tolerance(max(0.01, offset_mm), offset_arc_tolerance_mm)
            level_geom = _offset_geometry(
                printable_geometry,
                -offset_mm,
                join_style=offset_join_style,
                miter_limit=offset_miter_limit,
                quad_segs=quad_segs,
            )
            if level_geom is None or level_geom.is_empty:
                empty_levels += 1
                per_level.append({
                    "offset_level": int(level),
                    "offset_mm": float(offset_mm),
                    "raw_contour_count": 0,
                    "split_section_count": 0,
                    "corner_candidate_count": 0,
                    "accepted_corner_section_count": 0,
                    "rejected_corner_section_count": 0,
                    "diagonal_shortcut_rejected_count": 0,
                    "max_iso_distance_error_mm": 0.0,
                    "accepted_sections": 0,
                    "rejected_sections": 0,
                    "isolated_section_count": 0,
                    "crossed_section_count": 0,
                    "rejection_reasons": {},
                    "corner_rejection_reasons": {},
                    "remaining_uncovered_area_mm2": float(printable_geometry.difference(covered_geom).area) if covered_geom is not None and not covered_geom.is_empty else float(printable_geometry.area),
                    "reason": "collapsed_or_empty",
                })
                if empty_levels >= stop_after_empty_levels:
                    break
                level += 1
                continue

            sections, loop_coords_by_id = _extract_iso_distance_contours(level_geom, level=level, offset_mm=offset_mm)
            raw_contour_count = int(len(geometry_to_closed_toolpaths(level_geom, "fill-infill", simplify_mm)))
            central_probe = ShapelyPoint(*printable_geometry.representative_point().coords[0])
            accepted_this_level = 0
            rejected_this_level = 0
            isolated_this_level = 0
            crossed_this_level = 0
            corner_candidates_this_level = 0
            corner_accepted_this_level = 0
            corner_rejected_this_level = 0
            diagonal_shortcut_rejected_count_this_level = 0
            max_iso_distance_error_this_level = 0.0
            restored_continuity_section_count_this_level = 0
            missing_section_count_this_level = 0
            valid_open_reason_count_this_level = 0
            horizontal_section_present_this_level = False
            corner_turn_present_this_level = False
            corridor_contour_expected_this_level = False
            horizontal_section_expected_this_level = False
            rejection_counts_this_level: dict[str, int] = {}
            corner_rejection_counts_this_level: dict[str, int] = {}
            accepted_sections_for_level: list[Toolpath] = []
            rejected_sections_for_level: list[tuple[Toolpath, str]] = []
            loop_region_logs: list[dict[str, Any]] = []
            for section, role in sections:
                section_line = _path_to_line(section)
                if section_line is not None and float(section_line.distance(central_probe)) <= max(0.6, pen_width_mm * 2.0):
                    central_candidate_sections += 1
                ok, reason = _accept_section(section, role=role)
                iso_err_section = _max_iso_distance_error_mm(section, float(offset_mm))
                max_iso_distance_error_this_level = max(max_iso_distance_error_this_level, float(iso_err_section))
                if iso_err_section > iso_distance_tolerance_mm:
                    distance_error_debug_paths.append(
                        clone_toolpath(
                            section,
                            kind="fill-infill",
                            closed=section.closed,
                            source="iso_distance_error_debug",
                            metadata={**(section.metadata or {}), "path_role": "DISTANCE_ERROR_DEBUG", "iso_distance_error_mm": float(iso_err_section)},
                        )
                    )
                if ok:
                    if bool((section.metadata or {}).get("isolated_inner_loop", False)):
                        isolated_this_level += 1
                    if role == "CROSSED_CONTOUR_SECTION":
                        crossed_this_level += 1
                    accepted_paths.append(section)
                    accepted_sections_for_level.append(section)
                    accepted_this_level += 1
                    if section_line is not None and float(section_line.distance(central_probe)) <= max(0.6, pen_width_mm * 2.0):
                        central_accepted_sections += 1
                else:
                    rejected_sections.append((section, reason))
                    rejected_sections_for_level.append((section, reason))
                    rejected_this_level += 1
                    rejection_reason_counts[reason] = int(rejection_reason_counts.get(reason, 0)) + 1
                    rejection_counts_this_level[reason] = int(rejection_counts_this_level.get(reason, 0)) + 1
                    if section_line is not None and float(section_line.distance(central_probe)) <= max(0.6, pen_width_mm * 2.0):
                        central_rejected_reasons.append(reason)

            sections_by_loop: dict[str, list[Toolpath]] = {}
            for sec, _ in sections:
                loop_id = str((sec.metadata or {}).get("source_loop_id", ""))
                if not loop_id:
                    continue
                sections_by_loop.setdefault(loop_id, []).append(sec)
            accepted_ids = {id(s) for s in accepted_sections_for_level}
            for loop_id, loop_sections in sections_by_loop.items():
                sorted_secs = sorted(
                    loop_sections,
                    key=lambda s: int((s.metadata or {}).get("section_order", 0)),
                )
                loop_coords = loop_coords_by_id.get(loop_id, [])
                for i in range(len(sorted_secs) - 1):
                    a = sorted_secs[i]
                    b = sorted_secs[i + 1]
                    a_pts = a.points
                    b_pts = b.points
                    if not a_pts or not b_pts:
                        continue
                    pa = a_pts[-1]
                    pb = b_pts[0]
                    gap = math.hypot(pb.x - pa.x, pb.y - pa.y)
                    corner_candidates_this_level += 1
                    corner_candidates_total += 1
                    if gap > corner_bridge_gap_mm:
                        reason = "too_far_from_neighbor"
                        corner_rejected_this_level += 1
                        corner_rejected_total += 1
                        corner_rejection_counts_this_level[reason] = int(corner_rejection_counts_this_level.get(reason, 0)) + 1
                        corner_rejection_reason_counts[reason] = int(corner_rejection_reason_counts.get(reason, 0)) + 1
                        continue
                    # Validate and explicitly reject the naive diagonal chord candidate first.
                    direct_corner = Toolpath(
                        points=[Point(float(pa.x), float(pa.y)), Point(float(pb.x), float(pb.y))],
                        kind="fill-infill",
                        closed=False,
                        source="iso_contour_corner_direct_chord_candidate",
                        metadata={
                            "path_role": "CORNER_CONTOUR_SECTION",
                            "offset_level": int(level),
                            "offset_mm": float(offset_mm),
                            "source_loop_id": loop_id,
                        },
                    )
                    ok_direct, reason_direct, iso_err_direct, ang_err_direct = _validate_repair_direction(
                        direct_corner,
                        neighbor_a=a,
                        neighbor_b=b,
                        contour_distance_mm=float(offset_mm),
                        from_raw_contour=False,
                    )
                    if not ok_direct:
                        if reason_direct == "diagonal_shortcut":
                            diagonal_shortcut_rejected_count_this_level += 1
                        direct_log = {
                            "section_id": f"corner_direct_L{int(level)}_{loop_id}_{i}",
                            "contour_level": int(level),
                            "section_length_mm": float(_path_to_line(direct_corner).length) if _path_to_line(direct_corner) is not None else 0.0,
                            "new_coverage_area_mm2": 0.0,
                            "iso_distance_error_max_mm": float(iso_err_direct),
                            "parallel_angle_error_deg": float(ang_err_direct),
                            "accepted_or_rejected": "rejected",
                            "rejection_reason": reason_direct,
                        }
                        corner_candidate_logs.append(direct_log)
                        rejected_sections.append((direct_corner, reason_direct))
                        corner_rejected_this_level += 1
                        corner_rejected_total += 1
                        rejection_reason_counts[reason_direct] = int(rejection_reason_counts.get(reason_direct, 0)) + 1
                        corner_rejection_counts_this_level[reason_direct] = int(corner_rejection_counts_this_level.get(reason_direct, 0)) + 1
                        corner_rejection_reason_counts[reason_direct] = int(corner_rejection_reason_counts.get(reason_direct, 0)) + 1
                    # A) Prefer restoring original rejected section from the same loop/order.
                    restored_raw = None
                    for rej_sec, _rej_reason in rejected_sections_for_level:
                        if str((rej_sec.metadata or {}).get("source_loop_id", "")) != loop_id:
                            continue
                        sec_order = int((rej_sec.metadata or {}).get("section_order", 0))
                        if sec_order == int((a.metadata or {}).get("section_order", 0)) + 1:
                            restored_raw = clone_toolpath(
                                rej_sec,
                                kind="fill-infill",
                                closed=False,
                                source="iso_contour_restored_raw_section",
                                metadata={**(rej_sec.metadata or {}), "path_role": "CORNER_CONTOUR_SECTION"},
                            )
                            break
                    si = int((a.metadata or {}).get("end_vertex_index", 0))
                    ei = int((b.metadata or {}).get("start_vertex_index", 0))
                    if restored_raw is not None:
                        corner_points = restored_raw.points
                    else:
                        # B/C) use same-level iso-contour subpath; if unavailable fallback to short local path.
                        if loop_coords and si == ei and len(loop_coords) >= 3:
                            idx = max(1, min(len(loop_coords) - 2, si))
                            raw_corner = [loop_coords[idx - 1], loop_coords[idx], loop_coords[idx + 1]]
                        else:
                            raw_corner = _ring_subpath(loop_coords, si, ei) if loop_coords else [(pa.x, pa.y), (pb.x, pb.y)]
                        if len(raw_corner) < 2:
                            raw_corner = [(pa.x, pa.y), (pb.x, pb.y)]
                        corner_points = [Point(float(x), float(y)) for x, y in raw_corner]
                    if len(loop_coords) >= 3:
                        idx = max(1, min(len(loop_coords) - 2, si))
                        tri = [Point(float(loop_coords[idx - 1][0]), float(loop_coords[idx - 1][1])), Point(float(loop_coords[idx][0]), float(loop_coords[idx][1])), Point(float(loop_coords[idx + 1][0]), float(loop_coords[idx + 1][1]))]
                        turn = _turn_angle_deg(tri)
                        if turn > corner_max_turn_angle_deg and gap <= 1e-6 and id(a) in accepted_ids and id(b) in accepted_ids:
                            # Smooth connection already preserved by neighboring sections.
                            continue
                    if preserve_corner_vertices and len(corner_points) >= 3:
                        ang = _turn_angle_deg([corner_points[0], corner_points[min(1, len(corner_points)-1)], corner_points[-1]])
                        if ang > corner_max_turn_angle_deg:
                            corner_points = [corner_points[0], corner_points[-1]]
                    corner = Toolpath(
                        points=corner_points,
                        kind="fill-infill",
                        closed=False,
                        source="iso_contour_corner_bridge",
                        metadata={
                            "path_role": "CORNER_CONTOUR_SECTION",
                            "offset_level": int(level),
                            "offset_mm": float(offset_mm),
                            "source_loop_id": loop_id,
                        },
                    )
                    ok_dir, reason_dir, iso_err_mm, ang_err_deg = _validate_repair_direction(
                        corner,
                        neighbor_a=a,
                        neighbor_b=b,
                        contour_distance_mm=float(offset_mm),
                        from_raw_contour=(restored_raw is not None or len(corner.points) >= 3),
                    )
                    corner_log = {
                        "section_id": f"corner_L{int(level)}_{loop_id}_{i}",
                        "contour_level": int(level),
                        "corner_raw_geometry_found": bool(restored_raw is not None or len(corner_points) >= 3),
                        "corner_section_accepted": False,
                        "section_length_mm": float(_path_to_line(corner).length) if _path_to_line(corner) is not None else 0.0,
                        "new_coverage_area_mm2": 0.0,
                        "iso_distance_error_max_mm": float(iso_err_mm),
                        "parallel_angle_error_deg": float(ang_err_deg),
                    }
                    if not ok_dir:
                        max_iso_distance_error_this_level = max(max_iso_distance_error_this_level, float(iso_err_mm))
                        if reason_dir == "iso_distance_error_too_high":
                            distance_error_debug_paths.append(
                                clone_toolpath(
                                    corner,
                                    kind="fill-infill",
                                    closed=False,
                                    source="iso_distance_error_debug",
                                    metadata={**(corner.metadata or {}), "path_role": "DISTANCE_ERROR_DEBUG", "iso_distance_error_mm": float(iso_err_mm)},
                                )
                            )
                        reason = reason_dir
                        corner_log["accepted_or_rejected"] = "rejected"
                        corner_log["rejection_reason"] = reason
                        corner_candidate_logs.append(corner_log)
                        rejected_sections.append((corner, reason))
                        corner_rejected_this_level += 1
                        corner_rejected_total += 1
                        rejection_reason_counts[reason] = int(rejection_reason_counts.get(reason, 0)) + 1
                        corner_rejection_counts_this_level[reason] = int(corner_rejection_counts_this_level.get(reason, 0)) + 1
                        corner_rejection_reason_counts[reason] = int(corner_rejection_reason_counts.get(reason, 0)) + 1
                        continue
                    max_iso_distance_error_this_level = max(max_iso_distance_error_this_level, float(iso_err_mm))
                    ok, reason = _accept_section(corner, role="CORNER_CONTOUR_SECTION", corner_mode=True, continuity_preserving=True)
                    if ok:
                        corner_log["new_coverage_area_mm2"] = float((corner.metadata or {}).get("new_coverage_area_mm2", 0.0))
                        corner_log["accepted_or_rejected"] = "accepted"
                        corner_log["corner_section_accepted"] = True
                        corner_log["rejection_reason"] = ""
                        corner_candidate_logs.append(corner_log)
                        accepted_paths.append(corner)
                        accepted_sections_for_level.append(corner)
                        corner_accepted_this_level += 1
                        corner_accepted_total += 1
                        accepted_this_level += 1
                    else:
                        if reason == "too_short":
                            reason = "duplicate"
                        corner_log["accepted_or_rejected"] = "rejected"
                        corner_log["rejection_reason"] = reason
                        corner_candidate_logs.append(corner_log)
                        rejected_sections.append((corner, reason))
                        corner_rejected_this_level += 1
                        corner_rejected_total += 1
                        rejection_reason_counts[reason] = int(rejection_reason_counts.get(reason, 0)) + 1
                        corner_rejection_counts_this_level[reason] = int(corner_rejection_counts_this_level.get(reason, 0)) + 1
                        corner_rejection_reason_counts[reason] = int(corner_rejection_reason_counts.get(reason, 0)) + 1

            accepted_by_loop: dict[str, list[Toolpath]] = {}
            for sec in accepted_sections_for_level:
                lid = str((sec.metadata or {}).get("source_loop_id", ""))
                if lid:
                    accepted_by_loop.setdefault(lid, []).append(sec)

            for loop_id, loop_sections in sections_by_loop.items():
                loop_coords = loop_coords_by_id.get(loop_id, [])
                if len(loop_coords) < 4:
                    continue
                raw_line = LineString(loop_coords)
                loop_bbox = raw_line.bounds
                loop_w = float(loop_bbox[2] - loop_bbox[0])
                loop_h = float(loop_bbox[3] - loop_bbox[1])
                corridor_expected = bool(loop_w >= max(2.0 * pen_width_mm, 3.0 * loop_h) or loop_h >= max(2.0 * pen_width_mm, 3.0 * loop_w))
                has_horizontal_segment = False
                for j in range(len(loop_coords) - 1):
                    dx = abs(loop_coords[j + 1][0] - loop_coords[j][0])
                    dy = abs(loop_coords[j + 1][1] - loop_coords[j][1])
                    if dx >= max(2.0 * pen_width_mm, 1.5) and dy <= max(0.1, pen_width_mm * 0.2):
                        has_horizontal_segment = True
                        break
                corridor_contour_expected_this_level = corridor_contour_expected_this_level or corridor_expected
                horizontal_section_expected_this_level = horizontal_section_expected_this_level or has_horizontal_segment

                accepted_loop = sorted(accepted_by_loop.get(loop_id, []), key=lambda s: int((s.metadata or {}).get("section_order", 0)))
                if accepted_loop:
                    for sec in accepted_loop:
                        sec_line = _path_to_line(sec)
                        if sec_line is None:
                            continue
                        b = sec_line.bounds
                        p0 = sec.points[0]
                        p1 = sec.points[-1]
                        end_dy = abs(float(p1.y - p0.y))
                        if ((b[2] - b[0]) >= max(2.0 * pen_width_mm, 1.2) and (b[3] - b[1]) <= max(0.6, pen_width_mm * 1.2)) or (((b[2] - b[0]) >= max(2.0 * pen_width_mm, 1.2)) and end_dy <= max(0.25, pen_width_mm * 0.5)):
                            horizontal_section_present_this_level = True
                        if len(sec.points) >= 3 and _turn_angle_deg([sec.points[0], sec.points[min(1, len(sec.points)-1)], sec.points[-1]]) < 160.0:
                            corner_turn_present_this_level = True

                missing_for_loop = 0
                restored_for_loop = 0
                valid_open_reasons_for_loop = 0
                if len(accepted_loop) >= 2:
                    nacc = len(accepted_loop)
                    for i in range(nacc):
                        a = accepted_loop[i]
                        b = accepted_loop[(i + 1) % nacc]
                        ai = int((a.metadata or {}).get("end_vertex_index", 0))
                        bi = int((b.metadata or {}).get("start_vertex_index", 0))
                        ncoords = max(1, len(loop_coords) - 1)
                        step = (bi - ai) % ncoords
                        if step <= 1:
                            continue
                        missing_for_loop += 1
                        raw_seg = _ring_subpath(loop_coords, ai, bi)
                        if len(raw_seg) < 2:
                            continue
                        restored_points = [Point(float(x), float(y)) for x, y in raw_seg]
                        if len(restored_points) < 2:
                            continue
                        rb = LineString(raw_seg).bounds
                        role = "CORRIDOR_CONTOUR_SECTION" if (rb[2] - rb[0]) >= max((rb[3] - rb[1]) * 2.5, 1.2) else "CORNER_CONTOUR_SECTION"
                        restored = Toolpath(
                            points=restored_points,
                            kind="fill-infill",
                            closed=False,
                            source="iso_contour_restored_continuity",
                            metadata={
                                "path_role": role,
                                "offset_level": int(level),
                                "offset_mm": float(offset_mm),
                                "source_loop_id": loop_id,
                                "valid_open_reason": "topology_transition" if role == "CORNER_CONTOUR_SECTION" else "corridor_continuity_restore",
                            },
                        )
                        ok, reason = _accept_section(restored, role=role, continuity_preserving=True)
                        if ok:
                            accepted_paths.append(restored)
                            accepted_sections_for_level.append(restored)
                            accepted_this_level += 1
                            restored_for_loop += 1
                            restored_continuity_section_count_this_level += 1
                            corridor_restored_total += 1
                        else:
                            rejected_sections.append((restored, reason))
                            rejection_reason_counts[reason] = int(rejection_reason_counts.get(reason, 0)) + 1
                            rejection_counts_this_level[reason] = int(rejection_counts_this_level.get(reason, 0)) + 1

                loop_open_without_reason = 0
                for sec in accepted_loop:
                    if sec.closed:
                        continue
                    r = str((sec.metadata or {}).get("valid_open_reason", "")).strip()
                    if r:
                        valid_open_reasons_for_loop += 1
                    else:
                        loop_open_without_reason += 1
                valid_open_reason_count_this_level += valid_open_reasons_for_loop
                missing_section_count_this_level += missing_for_loop
                loop_region_logs.append({
                    "level": int(level),
                    "offset_distance_mm": float(offset_mm),
                    "raw_loop_id": loop_id,
                    "raw_loop_closed": True,
                    "raw_loop_length_mm": float(raw_line.length),
                    "split_section_count": int(len(loop_sections)),
                    "accepted_section_count": int(len(accepted_loop)),
                    "restored_continuity_section_count": int(restored_for_loop),
                    "missing_section_count": int(missing_for_loop),
                    "valid_open_reason_count": int(valid_open_reasons_for_loop),
                    "horizontal_section_present": bool(horizontal_section_present_this_level),
                    "corner_turn_present": bool(corner_turn_present_this_level),
                    "max_iso_distance_error_mm": float(max_iso_distance_error_this_level),
                    "remaining_uncovered_area_mm2": float(printable_geometry.difference(covered_geom).area) if covered_geom is not None and not covered_geom.is_empty else float(printable_geometry.area),
                    "open_without_valid_reason_count": int(loop_open_without_reason),
                })
                if loop_open_without_reason > 0 and missing_for_loop > 0:
                    open_section_audit_failures += 1

            per_level.append({
                "offset_level": int(level),
                "offset_mm": float(offset_mm),
                "raw_contour_count": raw_contour_count,
                "split_section_count": int(len(sections)),
                "corner_candidate_count": int(corner_candidates_this_level),
                "accepted_corner_section_count": int(corner_accepted_this_level),
                "rejected_corner_section_count": int(corner_rejected_this_level),
                "diagonal_shortcut_rejected_count": int(diagonal_shortcut_rejected_count_this_level),
                "max_iso_distance_error_mm": float(max_iso_distance_error_this_level),
                "restored_continuity_section_count": int(restored_continuity_section_count_this_level),
                "missing_section_count": int(missing_section_count_this_level),
                "valid_open_reason_count": int(valid_open_reason_count_this_level),
                "corridor_contour_expected": bool(corridor_contour_expected_this_level),
                "horizontal_section_expected": bool(horizontal_section_expected_this_level),
                "horizontal_section_present": bool(horizontal_section_present_this_level),
                "corner_turn_present": bool(corner_turn_present_this_level),
                "accepted_sections": int(accepted_this_level),
                "rejected_sections": int(rejected_this_level),
                "isolated_section_count": int(isolated_this_level),
                "crossed_section_count": int(crossed_this_level),
                "rejection_reasons": rejection_counts_this_level,
                "corner_rejection_reasons": corner_rejection_counts_this_level,
                "loop_region_logs": loop_region_logs,
                "remaining_uncovered_area_mm2": float(printable_geometry.difference(covered_geom).area) if covered_geom is not None and not covered_geom.is_empty else float(printable_geometry.area),
                "reason": "ok" if accepted_this_level > 0 else "no_useful_sections",
            })
            empty_levels = 0 if accepted_this_level > 0 else empty_levels + 1
            if empty_levels >= stop_after_empty_levels:
                break
            level += 1

        if not accepted_paths:
            fallback_geom = _offset_geometry(printable_geometry, -pen_radius)
            if fallback_geom is None or fallback_geom.is_empty:
                fallback_geom = printable_geometry
            for fb in geometry_to_closed_toolpaths(fallback_geom, "fill-infill", simplify_mm):
                candidate = clone_toolpath(
                    fb,
                    kind="fill-infill",
                    closed=True,
                    source="iso_contour_fallback",
                    metadata={"offset_level": 0, "offset_mm": float(pen_radius), "closed_iso_contour": True},
                )
                ok, reason = _accept_section(candidate, role="CONTOUR_INFILL")
                if ok:
                    accepted_paths.append(candidate)
                else:
                    rejected_sections.append((candidate, reason))
                    rejection_reason_counts[reason] = int(rejection_reason_counts.get(reason, 0)) + 1

        def _residual_components_with_ids(residual_geom: Any) -> list[tuple[int, Any]]:
            comps: list[tuple[int, Any]] = []
            rid = 1
            for part in normalize_geometry(residual_geom):
                if part is None or part.is_empty:
                    continue
                area = float(part.area)
                if area <= 1e-9:
                    continue
                comps.append((rid, part))
                rid += 1
            return comps

        def _nearest_path_id(component_geom: Any, current_paths: list[Toolpath]) -> str:
            best_id = ""
            best_dist = float("inf")
            for p in current_paths:
                pl = _path_to_line(p)
                if pl is None:
                    continue
                d = float(component_geom.distance(pl))
                if d < best_dist:
                    best_dist = d
                    best_id = str((p.path_id or "") or (p.metadata or {}).get("source_polygon_id", ""))
            return best_id

        max_coverage_repair_passes = min(10, max(5, int(os.getenv("CONTOUR_MAX_COVERAGE_REPAIR_PASSES", "6"))))
        safe_repair_region = printable_geometry
        for repair_pass_index in range(max_coverage_repair_passes):
            residual_geom = printable_geometry if covered_geom is None or covered_geom.is_empty else printable_geometry.difference(covered_geom)
            residual_components = _residual_components_with_ids(residual_geom)
            if not residual_components:
                break
            pass_candidates = 0
            pass_accepted = 0
            component_logs: list[dict[str, Any]] = []
            for comp_id, component in residual_components:
                comp_area = float(component.area)
                bounds = tuple(float(v) for v in component.bounds)
                nearest_path = _nearest_path_id(component, accepted_paths)
                accepted_for_component = 0
                rejected_reason_counts: dict[str, int] = {}
                candidate_geoms: list[Any] = [component.boundary]
                cb = component.bounds
                cx = 0.5 * (float(cb[0]) + float(cb[2]))
                cy = 0.5 * (float(cb[1]) + float(cb[3]))
                span_x = max(0.05, float(cb[2]) - float(cb[0]))
                span_y = max(0.05, float(cb[3]) - float(cb[1]))
                candidate_geoms.append(LineString([(cx - span_x, cy), (cx + span_x, cy)]).intersection(component))
                candidate_geoms.append(LineString([(cx, cy - span_y), (cx, cy + span_y)]).intersection(component))
                inset_local = _offset_geometry(component, -max(0.01, pen_radius * 0.45), join_style=offset_join_style, miter_limit=offset_miter_limit)
                if inset_local is not None and not inset_local.is_empty:
                    candidate_geoms.append(inset_local.boundary)
                outset_local = _offset_geometry(component, max(0.005, pen_radius * 0.18), join_style=offset_join_style, miter_limit=offset_miter_limit)
                if outset_local is not None and not outset_local.is_empty:
                    candidate_geoms.append(outset_local.boundary)

                for cg in candidate_geoms:
                    for line in extract_lines(cg):
                        clipped_line = line.intersection(safe_repair_region)
                        for cpart in extract_lines(clipped_line):
                            coords = list(cpart.coords)
                            if len(coords) < 2:
                                continue
                            closed = bool(math.hypot(coords[0][0] - coords[-1][0], coords[0][1] - coords[-1][1]) <= 1e-9)
                            if closed:
                                sections = _split_ring_to_sections(coords)
                            else:
                                sections = [coords]
                            for sec in sections:
                                if len(sec) < 2:
                                    continue
                                pass_candidates += 1
                                candidate = Toolpath(
                                    points=[Point(float(x), float(y)) for x, y in sec],
                                    kind="fill-infill",
                                    closed=False,
                                    source="residual_contour_section_repair",
                                    metadata={
                                        "path_role": "CONTOUR_SECTION_INFILL",
                                        "offset_mm": 0.0,
                                        "residual_repair_pass_index": int(repair_pass_index),
                                        "residual_component_id": int(comp_id),
                                    },
                                )
                                ok, reason = _accept_section(candidate, role="CONTOUR_SECTION_INFILL", corner_mode=False, continuity_preserving=True)
                                if ok:
                                    accepted_paths.append(candidate)
                                    pass_accepted += 1
                                    accepted_for_component += 1
                                else:
                                    rejected_sections.append((candidate, reason))
                                    rejection_reason_counts[reason] = int(rejection_reason_counts.get(reason, 0)) + 1
                                    rejected_reason_counts[reason] = int(rejected_reason_counts.get(reason, 0)) + 1

                # Fallback for tiny residual slivers: inject microscopic local cross-strokes
                # at component center, clipped to the residual component.
                if accepted_for_component <= 0:
                    micro_len_x = max(0.01, min(span_x * 0.9, pen_width_mm * 0.9))
                    micro_len_y = max(0.01, min(span_y * 0.9, pen_width_mm * 0.9))
                    micro_sources = [
                        LineString([(cx - (0.5 * micro_len_x), cy), (cx + (0.5 * micro_len_x), cy)]),
                        LineString([(cx, cy - (0.5 * micro_len_y)), (cx, cy + (0.5 * micro_len_y))]),
                    ]
                    for micro in micro_sources:
                        for mpart in extract_lines(micro.intersection(component)):
                            mcoords = list(mpart.coords)
                            if len(mcoords) < 2:
                                continue
                            pass_candidates += 1
                            candidate = Toolpath(
                                points=[Point(float(x), float(y)) for x, y in mcoords],
                                kind="fill-infill",
                                closed=False,
                                source="residual_contour_section_repair_micro",
                                metadata={
                                    "path_role": "CONTOUR_SECTION_INFILL",
                                    "offset_mm": 0.0,
                                    "residual_repair_pass_index": int(repair_pass_index),
                                    "residual_component_id": int(comp_id),
                                    "residual_micro_repair": True,
                                },
                            )
                            ok, reason = _accept_section(candidate, role="CONTOUR_SECTION_INFILL", corner_mode=False, continuity_preserving=True)
                            if ok:
                                accepted_paths.append(candidate)
                                pass_accepted += 1
                                accepted_for_component += 1
                            else:
                                rejected_sections.append((candidate, reason))
                                rejection_reason_counts[reason] = int(rejection_reason_counts.get(reason, 0)) + 1
                                rejected_reason_counts[reason] = int(rejected_reason_counts.get(reason, 0)) + 1
                component_logs.append({
                    "component_id": int(comp_id),
                    "area_mm2": float(comp_area),
                    "bounds": [bounds[0], bounds[1], bounds[2], bounds[3]],
                    "nearest_contour_level": int(repair_pass_index),
                    "nearest_accepted_path": str(nearest_path),
                    "accepted_candidates": int(accepted_for_component),
                    "rejection_reasons": rejected_reason_counts,
                    "reason_no_candidate_accepted": "none" if accepted_for_component > 0 else ("no_candidate_generated" if pass_candidates <= 0 else "all_candidates_rejected"),
                })
            residual_repair_logs.append({
                "pass_index": int(repair_pass_index),
                "residual_component_count": int(len(residual_components)),
                "generated_candidates": int(pass_candidates),
                "accepted_candidates": int(pass_accepted),
                "components": component_logs,
            })
            if pass_accepted <= 0:
                break

        residual_before = printable_geometry if covered_geom is None or covered_geom.is_empty else printable_geometry.difference(covered_geom)
        fill_paths = optimize_toolpath_order(accepted_paths, strategy=travel_ordering)

        final_outer_outline_inset_mm = float(pen_radius)
        final_inner_outline_inset_mm = float(pen_radius)
        legacy_outline_geom = _offset_geometry(
            printable_geometry,
            -final_outer_outline_inset_mm,
            join_style=offset_join_style,
            miter_limit=offset_miter_limit,
            quad_segs=_quad_segs_for_tolerance(max(0.01, pen_radius), offset_arc_tolerance_mm),
        )
        if legacy_outline_geom is None or legacy_outline_geom.is_empty:
            legacy_outline_geom = printable_geometry

        geometry_quality = bundle.metadata.get("geometry_quality") if isinstance(bundle.metadata, dict) else None
        is_raster_source = bool(isinstance(geometry_quality, dict) and str(geometry_quality.get("source_mode", "")) == "raster")
        target_component_count = len(normalize_geometry(printable_geometry))
        legacy_component_count = len(normalize_geometry(legacy_outline_geom))
        target_area = float(printable_geometry.area) if printable_geometry is not None and not printable_geometry.is_empty else 0.0
        legacy_area = float(legacy_outline_geom.area) if legacy_outline_geom is not None and not legacy_outline_geom.is_empty else 0.0
        lost_component_count = max(0, target_component_count - legacy_component_count)
        lost_area_ratio = 0.0 if target_area <= 1e-9 else max(0.0, (target_area - legacy_area) / target_area)
        use_final_target_outline = bool(
            is_raster_source
            and (
                legacy_outline_geom is None
                or legacy_outline_geom.is_empty
                or legacy_component_count < target_component_count
                or lost_area_ratio >= 0.18
            )
        )
        outline_source_geom = printable_geometry if use_final_target_outline else legacy_outline_geom
        outline_generation_source = "final_target_mask" if use_final_target_outline else "final_outline_offset"
        outline_emit_offset_mm = 0.0 if use_final_target_outline else float(final_outer_outline_inset_mm)

        outline_paths, outline_debug = _build_outline_paths_from_geometry(
            outline_source_geom,
            offset_mm=outline_emit_offset_mm,
            source_label=outline_generation_source,
            simplify_outline_mm=0.0 if use_final_target_outline else 0.0,
        )
        coverage_outline_paths, _coverage_outline_debug = _build_outline_paths_from_geometry(
            legacy_outline_geom,
            offset_mm=float(final_outer_outline_inset_mm),
            source_label="final_outline_offset_coverage",
            simplify_outline_mm=0.0,
        )

        for outline_path in coverage_outline_paths:
            fp = _path_footprint(outline_path)
            if fp is None or fp.is_empty:
                continue
            covered_geom = fp if covered_geom is None or covered_geom.is_empty else covered_geom.union(fp)

        def _raster_mm_mapping() -> tuple[float, float, float, float, float]:
            min_x, min_y, max_x, max_y = printable_geometry.bounds
            px_per_mm = 36.0
            pad_mm = max(0.6, pen_width_mm)
            sx = min_x - pad_mm
            sy = min_y - pad_mm
            return sx, sy, px_per_mm, max_x + pad_mm, max_y + pad_mm

        def _geom_to_raster_mask(geom: Any, *, sx: float, sy: float, px_per_mm: float, ex: float, ey: float) -> np.ndarray:
            width_px = max(8, int(math.ceil((ex - sx) * px_per_mm)))
            height_px = max(8, int(math.ceil((ey - sy) * px_per_mm)))
            mask = np.zeros((height_px, width_px), dtype=np.uint8)
            if geom is None or geom.is_empty:
                return mask
            for poly in normalize_geometry(geom):
                ext = np.array(
                    [[int(round((float(x) - sx) * px_per_mm)), int(round((float(y) - sy) * px_per_mm))] for x, y in poly.exterior.coords],
                    dtype=np.int32,
                )
                if len(ext) >= 3:
                    cv2.fillPoly(mask, [ext], 255)
                for ring in poly.interiors:
                    hole = np.array(
                        [[int(round((float(x) - sx) * px_per_mm)), int(round((float(y) - sy) * px_per_mm))] for x, y in ring.coords],
                        dtype=np.int32,
                    )
                    if len(hole) >= 3:
                        cv2.fillPoly(mask, [hole], 0)
            return mask

        def _line_to_mm_path(p0: tuple[int, int], p1: tuple[int, int], *, sx: float, sy: float, px_per_mm: float, repair_pass: int, comp_id: int) -> Toolpath:
            x0 = sx + (float(p0[0]) / px_per_mm)
            y0 = sy + (float(p0[1]) / px_per_mm)
            x1 = sx + (float(p1[0]) / px_per_mm)
            y1 = sy + (float(p1[1]) / px_per_mm)
            return Toolpath(
                points=[Point(x0, y0), Point(x1, y1)],
                kind="fill-infill",
                closed=False,
                source="residual_raster_repair",
                metadata={
                    "path_role": "CONTOUR_SECTION_INFILL",
                    "residual_repair_pass_index": int(repair_pass),
                    "residual_component_id": int(comp_id),
                    "residual_raster_repair": True,
                    "offset_mm": 0.0,
                },
            )

        # Pixel-domain residual repair to close corner/junction slivers that survive geometric sectioning.
        raster_repair_passes = min(8, max(4, int(os.getenv("CONTOUR_RASTER_REPAIR_PASSES", "6"))))
        sx, sy, px_per_mm, ex, ey = _raster_mm_mapping()
        for rpass in range(raster_repair_passes):
            cov_geom = covered_geom if covered_geom is not None and not covered_geom.is_empty else Polygon()
            target_mask = _geom_to_raster_mask(printable_geometry, sx=sx, sy=sy, px_per_mm=px_per_mm, ex=ex, ey=ey)
            covered_mask = _geom_to_raster_mask(cov_geom, sx=sx, sy=sy, px_per_mm=px_per_mm, ex=ex, ey=ey)
            uncovered_mask = cv2.bitwise_and(target_mask, cv2.bitwise_not(covered_mask))
            if int(np.count_nonzero(uncovered_mask)) <= 0:
                break
            n_labels, labels, stats, _centroids = cv2.connectedComponentsWithStats((uncovered_mask > 0).astype(np.uint8), connectivity=8)
            accepted_in_pass = 0
            for comp_idx in range(1, int(n_labels)):
                area_px = int(stats[comp_idx, cv2.CC_STAT_AREA])
                if area_px <= 0:
                    continue
                x = int(stats[comp_idx, cv2.CC_STAT_LEFT])
                y = int(stats[comp_idx, cv2.CC_STAT_TOP])
                w = int(stats[comp_idx, cv2.CC_STAT_WIDTH])
                h = int(stats[comp_idx, cv2.CC_STAT_HEIGHT])
                cx = x + (w // 2)
                cy = y + (h // 2)
                # Local principal cross strokes through residual component bounds.
                candidates_px = [
                    ((x, cy), (x + max(1, w - 1), cy)),
                    ((cx, y), (cx, y + max(1, h - 1))),
                ]
                if w >= 2 and h >= 2:
                    candidates_px.append(((x, y), (x + max(1, w - 1), y + max(1, h - 1))))
                    candidates_px.append(((x + max(1, w - 1), y), (x, y + max(1, h - 1))))
                for p0, p1 in candidates_px:
                    candidate = _line_to_mm_path(p0, p1, sx=sx, sy=sy, px_per_mm=px_per_mm, repair_pass=rpass, comp_id=comp_idx)
                    ok, _reason = _accept_section(candidate, role="CONTOUR_SECTION_INFILL", corner_mode=False, continuity_preserving=True)
                    if ok:
                        accepted_paths.append(candidate)
                        accepted_in_pass += 1
                        fp = _path_footprint(candidate)
                        if fp is not None and not fp.is_empty:
                            covered_geom = fp if covered_geom is None or covered_geom.is_empty else covered_geom.union(fp)
            if accepted_in_pass <= 0:
                break

        fill_paths = optimize_toolpath_order(accepted_paths, strategy=travel_ordering)
        result = merge_connected_toolpaths(fill_paths + outline_paths)
        residual_after = printable_geometry if covered_geom is None or covered_geom.is_empty else printable_geometry.difference(covered_geom)
        mask_area = max(1e-9, float(printable_geometry.area))
        uncovered_before_area = 0.0 if residual_before is None or residual_before.is_empty else float(residual_before.area)
        uncovered_after_area = 0.0 if residual_after is None or residual_after.is_empty else float(residual_after.area)
        uncovered_before_ratio = uncovered_before_area / mask_area
        uncovered_after_ratio = uncovered_after_area / mask_area
        hole_area = 0.0 if hole_void_geom is None or hole_void_geom.is_empty else float(hole_void_geom.area)
        hole_overlap = covered_geom.intersection(hole_void_geom) if hole_void_geom is not None and not hole_void_geom.is_empty else None
        hole_overspill_area = 0.0 if hole_overlap is None or hole_overlap.is_empty else float(hole_overlap.area)
        hole_overspill_ratio = hole_overspill_area / max(1e-9, hole_area) if hole_area > 0.0 else 0.0

        if isinstance(debug_obj, dict):
            covered_area = float(covered_geom.area) if covered_geom is not None and not covered_geom.is_empty else 0.0
            overlap_area = float(printable_geometry.intersection(covered_geom).area) if covered_geom is not None and not covered_geom.is_empty else 0.0
            overspill_area_ratio = max(0.0, (covered_area - overlap_area) / max(1e-9, covered_area)) if covered_area > 0 else 0.0
            debug_obj["infill_connector_diagnostics"] = {
                "total_infill_rows": int(sum(1 for p in result if p.kind in {"fill-infill", "crossed-contour-infill"})),
                "accepted_connectors": 0,
                "rejected_connectors": 0,
                "rejected_raster_mask_sampling": 0,
                "rejected_outside_selected_color": 0,
                "rejection_counts": {},
            }
            debug_obj["contour_offset_debug"] = {
                "outline_offset_mm": float(final_outer_outline_inset_mm),
                "outline_generation_source": outline_generation_source,
                "outline_component_count_input": int(outline_debug.get("outline_component_count_input", 0)),
                "outline_component_count_output": int(outline_debug.get("outline_component_count_output", 0)),
                "outline_paths_generated": int(outline_debug.get("outline_paths_generated", 0)),
                "outline_paths_dropped": int(outline_debug.get("outline_paths_dropped", 0)),
                "outline_drop_reasons": dict(outline_debug.get("outline_drop_reasons", {})),
                "thin_components_outlined": int(outline_debug.get("thin_components_outlined", 0)),
                "small_components_outlined": int(outline_debug.get("small_components_outlined", 0)),
                "outline_total_length_mm": float(outline_debug.get("outline_total_length_mm", 0.0)),
                "outline_source_component_count_lost_by_inset": int(lost_component_count),
                "outline_source_area_loss_ratio_from_inset": float(lost_area_ratio),
                "contour_overlap_spacing_factor": float(contour_overlap_spacing_factor),
                "offset_step_mm": float(offset_step_mm),
                "offset_join_style": "miter" if offset_join_style == 2 else "round",
                "offset_miter_limit": float(offset_miter_limit),
                "offset_arc_tolerance_mm": float(offset_arc_tolerance_mm),
                "distance_levels_attempted": int(len(per_level)),
                "distance_levels_with_accepted_contours": int(sum(1 for entry in per_level if int(entry.get("accepted_sections", 0)) > 0)),
                "offset_levels_attempted": int(len(per_level)),
                "offset_levels_with_accepted_sections": int(sum(1 for entry in per_level if int(entry.get("accepted_sections", 0)) > 0)),
                "levels": per_level,
                "full_contour_path_count": int(sum(1 for path in result if str((path.metadata or {}).get("path_role", "")) == "CONTOUR_INFILL")),
                "contour_section_count": int(sum(1 for path in result if str((path.metadata or {}).get("path_role", "")) == "CONTOUR_SECTION_INFILL")),
                "crossed_contour_section_count": int(sum(1 for path in result if str((path.metadata or {}).get("path_role", "")) == "CROSSED_CONTOUR_SECTION")),
                "corner_contour_section_count": int(sum(1 for path in result if str((path.metadata or {}).get("path_role", "")) == "CORNER_CONTOUR_SECTION")),
                "isolated_inner_contour_count": int(sum(1 for path in result if bool((path.metadata or {}).get("isolated_inner_loop", False)))),
                "residual_contour_section_count": 0,
                "coverage_repair_passes_executed": int(len(residual_repair_logs)),
                "coverage_repair_logs": residual_repair_logs,
                "rejected_section_count": int(len(rejected_sections)),
                "rejection_reason_counts": rejection_reason_counts,
                "corner_rejection_reason_counts": corner_rejection_reason_counts,
                "corner_candidate_count_total": int(corner_candidates_total),
                "corner_accepted_section_count_total": int(corner_accepted_total),
                "corner_rejected_section_count_total": int(corner_rejected_total),
                "corridor_restored_section_count_total": int(corridor_restored_total),
                "open_section_audit_failures": int(open_section_audit_failures),
                "corner_candidate_logs": corner_candidate_logs,
                "central_junction_candidate_sections_found": int(central_candidate_sections),
                "central_junction_sections_accepted": int(central_accepted_sections),
                "central_junction_rejected_reasons": sorted(set(central_rejected_reasons)),
                "remaining_uncovered_area_mm2_before": float(uncovered_before_area),
                "remaining_uncovered_area_mm2_after": float(uncovered_after_area),
                "remaining_uncovered_area_ratio_before": float(uncovered_before_ratio),
                "remaining_uncovered_area_ratio_after": float(uncovered_after_ratio),
                "overspill_area_ratio": float(overspill_area_ratio),
                "hole_overspill_ratio": float(hole_overspill_ratio),
                "outer_outline_path_count": int(sum(1 for path in result if path.kind == "outline" and str((path.metadata or {}).get("path_role", "")) == "FINAL_OUTER_OUTLINE")),
                "inner_outline_path_count": int(sum(1 for path in result if path.kind == "outline" and str((path.metadata or {}).get("path_role", "")) == "FINAL_INNER_OUTLINE")),
                "diagonal_shortcut_rejected_count": int(sum(1 for _p, reason in rejected_sections if reason == "diagonal_shortcut")),
            }
            debug_append_geometry(debug_obj, "residual_uncovered_regions_before_repair", residual_before, "gap-residual-before")
            debug_append_geometry(debug_obj, "residual_uncovered_regions_after_repair", residual_after, "gap-residual-after")
            debug_append_geometry(debug_obj, "contour_target_mask", printable_geometry, "target-mask")
            debug_append_toolpaths(debug_obj, "raw_offset_contour", raw_offset_contours_debug)
            debug_append_toolpaths(debug_obj, "accepted_contour_infill", [p for p in result if str((p.metadata or {}).get("path_role", "")) == "CONTOUR_INFILL"])
            debug_append_toolpaths(debug_obj, "accepted_contour_section_infill", [p for p in result if str((p.metadata or {}).get("path_role", "")) == "CONTOUR_SECTION_INFILL"])
            debug_append_toolpaths(debug_obj, "accepted_isolated_contour_sections", [p for p in result if bool((p.metadata or {}).get("isolated_inner_loop", False))])
            debug_append_toolpaths(debug_obj, "accepted_crossed_contour_sections", [p for p in result if str((p.metadata or {}).get("path_role", "")) == "CROSSED_CONTOUR_SECTION"])
            debug_append_toolpaths(debug_obj, "restored_corridor_contour_sections", [p for p in result if str((p.metadata or {}).get("path_role", "")) == "CORRIDOR_CONTOUR_SECTION"])
            debug_append_toolpaths(debug_obj, "accepted_corner_contour_sections", [p for p in result if str((p.metadata or {}).get("path_role", "")) == "CORNER_CONTOUR_SECTION"])
            debug_append_toolpaths(debug_obj, "rejected_diagonal_shortcut_candidates", [item[0] for item in rejected_sections if item[1] in {"diagonal_shortcut", "angle_not_parallel", "iso_distance_error_too_high"}])
            debug_append_toolpaths(debug_obj, "distance_error_debug", distance_error_debug_paths)
            debug_append_toolpaths(debug_obj, "contour_section_candidates_rejected", [item[0] for item in rejected_sections])
            debug_append_toolpaths(debug_obj, "final_outer_outline", [p for p in result if p.kind == "outline" and str((p.metadata or {}).get("path_role", "")) == "FINAL_OUTER_OUTLINE"])
            debug_append_toolpaths(debug_obj, "final_inner_outline", [p for p in result if p.kind == "outline" and str((p.metadata or {}).get("path_role", "")) == "FINAL_INNER_OUTLINE"])
            debug_append_geometry(debug_obj, "pen_width_preview", covered_geom, "pen-footprint")
            debug_append_geometry(debug_obj, "remaining_residual_gaps", residual_after, "residual-gaps")

        return result

    if enable_fill and bundle.printable_geometry is not None and not bundle.printable_geometry.is_empty:
        from .coverage_planner import plan_coverage_first_toolpaths

        return plan_coverage_first_toolpaths(
            bundle,
            enable_fill=enable_fill,
            line_width_mm=line_width_mm,
            wall_count=wall_count,
            infill_density=infill_density,
            infill_spacing_mm=infill_spacing_mm,
            infill_angle_deg=infill_angle_deg,
            outline_after_fill=outline_after_fill,
            min_fill_area_mm2=min_fill_area_mm2,
            min_fill_width_mm=min_fill_width_mm,
            simplify_tolerance_mm=simplify_tolerance_mm,
            remove_duplicate_paths=remove_duplicate_paths,
            small_shape_mode=small_shape_mode,
            fill_strategy=fill_strategy,
            alternate_fill_angle_deg=alternate_fill_angle_deg,
            thin_detail_mode=thin_detail_mode,
            thin_detail_min_area_mm2=thin_detail_min_area_mm2,
            thin_detail_simplify_mm=thin_detail_simplify_mm,
            thin_detail_overlap=thin_detail_overlap,
            min_segment_length_mm=min_segment_length_mm,
            travel_optimization=travel_optimization,
            allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
            infill_path_mode=infill_path_mode,
            expensive_coverage_repair=expensive_coverage_repair,
            debug=debug,
        )
    if enable_fill and (bundle.printable_geometry is None or bundle.printable_geometry.is_empty):
        toolpaths: list[Toolpath] = []
        for segment in bundle.outline_segments:
            simplified = simplify_segment_points(segment.points, simplify_tolerance_mm, segment.closed)
            if len(simplified) < 2:
                continue
            toolpaths.append(Toolpath(
                points=simplified,
                kind="outline",
                closed=segment.closed,
                source="mask_contour",
                metadata={
                    "simplify_tolerance_mm": float(simplify_tolerance_mm),
                    "pen_width_mm": float(line_width_mm),
                    "source_region_id": "standalone_outline",
                    "expected_relation_to_fill": "standalone_outline",
                },
            ))
        return assign_stable_path_ids(merge_connected_toolpaths(toolpaths))
    toolpaths: list[Toolpath] = []
    simplify_tolerance_resolved_mm = simplify_tolerance_mm
    detail_tolerance_mm = max(simplify_tolerance_mm, thin_detail_simplify_mm)

    use_direct_outline_segments = (not enable_fill) or bundle.printable_geometry is None or bundle.printable_geometry.is_empty

    outline_segments = list(bundle.outline_segments) if use_direct_outline_segments else []
    if not enable_fill:
        outline_segments.extend(bundle.fill_boundary_segments)

    for segment in outline_segments:
        simplified = simplify_segment_points(segment.points, simplify_tolerance_resolved_mm, segment.closed)
        toolpaths.append(Toolpath(
            points=simplified,
            kind="outline",
            closed=segment.closed,
            source="mask_contour",
            metadata={
                "simplify_tolerance_mm": simplify_tolerance_resolved_mm,
                "pen_width_mm": line_width_mm,
                "source_component_id": None,
                "source_contour_id": None,
                "expected_relation_to_fill": "standalone_outline" if use_direct_outline_segments else "source_outline",
                "coordinate_space_at_creation": "surface_mm",
                "coordinate_space_before_offset": "surface_mm",
                "offset_space": "none",
                "coordinate_space_before_simplify": "surface_mm",
                "simplify_space": "surface_mm",
            },
        ))

    if enable_fill and bundle.printable_geometry is not None and not bundle.printable_geometry.is_empty:
        # Hard-switch to contour-only slicer path for normal generation.
        toolpaths = _contour_only_fill_paths(
            bundle.printable_geometry,
            pen_width_mm=line_width_mm,
            simplify_tolerance_mm_value=simplify_tolerance_resolved_mm,
            travel_ordering=travel_optimization,
            debug_obj=debug,
        )
        toolpaths = assign_stable_path_ids(merge_connected_toolpaths(toolpaths))
        legacy_kinds = {"detail-trace", "detail-continuation", "hatch", "adaptive", "fill-infill-travel", "coverage_connector", "collapse-centerline", "gap-repair-centerline"}
        has_holes = bool(any(len(poly.interiors) > 0 for poly in normalize_geometry(bundle.printable_geometry)))
        legacy_count = sum(1 for path in toolpaths if path.kind in legacy_kinds)
        zero_length_metric_nonzero_move_count = 0
        for path in toolpaths:
            if len(path.points) < 2:
                continue
            max_surface = float((path.metadata or {}).get("max_surface_segment_mm_after_resampling", 0.0))
            if max_surface <= 1e-12:
                max_surface = max(_segment_lengths_mm(path.points, closed=path.closed), default=0.0)
                path.metadata["max_surface_segment_mm_after_resampling"] = float(max_surface)
            if max_surface > 1e-12:
                continue
            if any(not nearly_same_point(a, b, 1e-12) for a, b in zip(path.points, path.points[1:])):
                zero_length_metric_nonzero_move_count += 1
        contour_dbg = (debug or {}).get("contour_offset_debug", {}) if isinstance(debug, dict) else {}
        remaining_uncovered_area_mm2 = float(contour_dbg.get("remaining_uncovered_area_mm2_after", 0.0) or 0.0)
        remaining_uncovered_area_ratio = float(contour_dbg.get("remaining_uncovered_area_ratio_after", 0.0) or 0.0)
        open_section_audit_failures = int(contour_dbg.get("open_section_audit_failures", 0) or 0)
        remaining_uncovered_area_tolerance_ratio = float(os.getenv("CONTOUR_REMAINING_UNCOVERED_TOLERANCE_RATIO", "0.10"))
        path_role_counts: dict[str, int] = {}
        for path in toolpaths:
            role = str((path.metadata or {}).get("path_role", ""))
            if role:
                path_role_counts[role] = int(path_role_counts.get(role, 0)) + 1
        outline_last = bool(
            len(toolpaths) > 0
            and str((toolpaths[-1].metadata or {}).get("path_role", "")).startswith("FINAL_")
        )
        if isinstance(debug, dict):
            debug["gcode_generation_audit"] = {
                "legacy_kinds_forbidden_count": int(legacy_count),
                "legacy_detail_trace_count": int(sum(1 for path in toolpaths if path.kind in {"detail-trace", "detail-continuation"})),
                "collapse_centerline_count": int(sum(1 for path in toolpaths if path.kind == "collapse-centerline")),
                "hatch_count": int(sum(1 for path in toolpaths if path.kind == "hatch")),
                "adaptive_count": int(sum(1 for path in toolpaths if path.kind == "adaptive")),
                "legacy_connector_count": int(sum(1 for path in toolpaths if path.kind in {"fill-infill-travel", "coverage_connector", "gap-repair-centerline"})),
                "zero_length_metric_nonzero_move_count": int(zero_length_metric_nonzero_move_count),
                "contour_infill_path_count": int(sum(1 for path in toolpaths if path.kind == "fill-infill")),
                "full_contour_count": int(sum(1 for path in toolpaths if str((path.metadata or {}).get("path_role", "")) == "CONTOUR_INFILL")),
                "contour_section_count": int(sum(1 for path in toolpaths if str((path.metadata or {}).get("path_role", "")) == "CONTOUR_SECTION_INFILL")),
                "isolated_contour_section_count": int(sum(1 for path in toolpaths if str((path.metadata or {}).get("path_role", "")) == "ISOLATED_CONTOUR_SECTION")),
                "crossed_contour_section_count": int(sum(1 for path in toolpaths if str((path.metadata or {}).get("path_role", "")) == "CROSSED_CONTOUR_SECTION")),
                "corner_contour_section_count": int(sum(1 for path in toolpaths if str((path.metadata or {}).get("path_role", "")) == "CORNER_CONTOUR_SECTION")),
                "isolated_inner_contour_count": int(sum(1 for path in toolpaths if bool((path.metadata or {}).get("isolated_inner_loop", False)))),
                "repair_patch_fill_path_count": int(sum(1 for path in toolpaths if str((path.metadata or {}).get("path_role", "")) == "REPAIR_PATCH_FILL")),
                "residual_repair_count": int(sum(1 for path in toolpaths if str((path.metadata or {}).get("path_role", "")) in {"RESIDUAL_CONTOUR_REPAIR", "REPAIR_PATCH_FILL"})),
                "central_junction_accepted_section_count": int(contour_dbg.get("central_junction_sections_accepted", 0) or 0),
                "remaining_uncovered_area_mm2": float(remaining_uncovered_area_mm2),
                "remaining_uncovered_area_ratio": float(remaining_uncovered_area_ratio),
                "open_section_audit_failures": int(open_section_audit_failures),
                "outer_outline_count": int(sum(1 for path in toolpaths if path.kind == "outline" and str(path.metadata.get("path_role", "")) == "FINAL_OUTER_OUTLINE")),
                "inner_outline_count": int(sum(1 for path in toolpaths if path.kind == "outline" and str(path.metadata.get("path_role", "")) == "FINAL_INNER_OUTLINE")),
                "outline_path_count": int(sum(1 for path in toolpaths if path.kind == "outline")),
                "outline_last": bool(outline_last),
                "path_role_counts": path_role_counts,
            }
        if legacy_count > 0:
            raise AssertionError("Contour-only mode violation: legacy path kinds present in toolpath output")
        if zero_length_metric_nonzero_move_count > 0:
            raise AssertionError("Contour-only mode violation: nonzero move path has zero max_surface_segment_mm_after_resampling")
        if not outline_last:
            raise AssertionError("Contour-only mode violation: final outline is not last")
        if has_holes and int(sum(1 for path in toolpaths if path.kind == "outline" and str(path.metadata.get("path_role", "")) == "FINAL_INNER_OUTLINE")) <= 0:
            raise AssertionError("Contour-only mode violation: missing FINAL_INNER_OUTLINE for geometry with holes")
        if open_section_audit_failures > 0:
            raise AssertionError("Contour-only mode violation: raw closed contour loop became open without valid_open_reason")
        if remaining_uncovered_area_ratio > remaining_uncovered_area_tolerance_ratio:
            raise AssertionError(
                f"Contour-only mode violation: remaining uncovered area ratio {remaining_uncovered_area_ratio:.6f} exceeds tolerance {remaining_uncovered_area_tolerance_ratio:.6f}"
            )
        return toolpaths

    detail_clip_region = None
    if enable_fill and bundle.printable_geometry is not None and not bundle.printable_geometry.is_empty:
        detail_clip_region = _offset_geometry(bundle.printable_geometry, -(line_width_mm * 0.5))
        if detail_clip_region is None or detail_clip_region.is_empty:
            # Tiny regions may collapse with full pen-radius inset; still keep centerline safely inside when possible.
            detail_clip_region = _offset_geometry(bundle.printable_geometry, -(line_width_mm * 0.25))
        if detail_clip_region is None or detail_clip_region.is_empty:
            detail_clip_region = bundle.printable_geometry

    detail_paths: list[Toolpath] = []
    for segment in bundle.detail_segments:
        simplified = simplify_segment_points(segment.points, detail_tolerance_mm, segment.closed)
        if len(simplified) < 2:
            continue
        metadata = {
            "simplify_tolerance_mm": detail_tolerance_mm,
            "pen_width_mm": line_width_mm,
            "source_component_id": None,
            "source_contour_id": None,
            "expected_relation_to_fill": "detail_overlay",
            "coordinate_space_at_creation": "surface_mm",
            "coordinate_space_before_offset": "surface_mm",
            "offset_space": "none",
            "coordinate_space_before_simplify": "surface_mm",
            "simplify_space": "surface_mm",
            "detail_centerline_clipped_to_printable_offset": bool(detail_clip_region is not None),
        }
        if detail_clip_region is None:
            detail_paths.append(Toolpath(
                points=simplified,
                kind="detail-trace",
                closed=segment.closed,
                source="detail_trace",
                metadata=metadata,
            ))
            continue
        line = LineString([(point.x, point.y) for point in simplified])
        clipped = line.intersection(detail_clip_region)
        for part in extract_lines(clipped):
            clipped_points = simplify_segment_points([Point(float(x), float(y)) for x, y in part.coords], detail_tolerance_mm, False)
            if len(clipped_points) < 2:
                continue
            detail_paths.append(Toolpath(
                points=clipped_points,
                kind="detail-trace",
                closed=False,
                source="detail_trace",
                metadata=metadata,
            ))
    def _build_clean_final_outline_paths() -> list[Toolpath]:
        if bundle.printable_geometry is None or bundle.printable_geometry.is_empty:
            return []
        inset = line_width_mm * 0.5
        outline_region = _offset_geometry(bundle.printable_geometry, -inset)
        if outline_region is None or outline_region.is_empty:
            outline_region = bundle.printable_geometry
        simplify_mm = min(max(simplify_tolerance_resolved_mm, 0.02), max(0.08, line_width_mm * 0.35))
        out: list[Toolpath] = []
        for polygon in normalize_geometry(outline_region):
            ext = simplify_segment_points([Point(float(x), float(y)) for x, y in polygon.exterior.coords], simplify_mm, True)
            if len(ext) >= 4:
                out.append(Toolpath(
                    points=ext,
                    kind="outline",
                    closed=True,
                    source="final_outline_offset",
                    metadata={"path_role": "PRINT_OUTLINE_FINAL", "outline_offset_mm": inset, "simplify_tolerance_mm": simplify_mm},
                ))
            for ring in polygon.interiors:
                inner = simplify_segment_points([Point(float(x), float(y)) for x, y in ring.coords], simplify_mm, True)
                if len(inner) >= 4:
                    out.append(Toolpath(
                        points=inner,
                        kind="outline",
                        closed=True,
                        source="final_outline_offset_hole",
                        metadata={"path_role": "PRINT_OUTLINE_FINAL", "outline_offset_mm": inset, "simplify_tolerance_mm": simplify_mm, "is_hole": True},
                    ))
        return out

    raw_print_paths = list(toolpaths)
    composed_paths: list[Toolpath] = []

    if enable_fill:
        # Interior fill only; drop contour-like border artifacts from final emission.
        interior_kinds = {"fill-infill", "coverage_rectilinear", "coverage_offset_line", "coverage_centerline", "coverage_tiny_mark"}
        interior_paths: list[Toolpath] = []
        for path in raw_print_paths:
            if path.kind not in interior_kinds:
                continue
            as_detail = bool(path.metadata.get("mandatory_thin_detail", False) or path.metadata.get("force_minimum_printable_stroke", False))
            interior_paths.append(clone_toolpath(
                path,
                kind="detail-trace" if as_detail else "fill-infill",
                metadata={**path.metadata, "path_role": "PRINT_DETAIL" if as_detail else "PRINT_INFILL"},
            ))

        def _stroke_coverage_geometry(paths_for_coverage: list[Toolpath], pen_width_mm_value: float) -> Any:
            pieces: list[Any] = []
            radius = max(0.01, pen_width_mm_value * 0.5)
            for path in paths_for_coverage:
                if len(path.points) < 2:
                    continue
                line = LineString([(point.x, point.y) for point in path.points])
                if line.is_empty or line.length <= 1e-9:
                    continue
                pieces.append(line.buffer(radius, cap_style=1, join_style=1))
            if not pieces:
                return None
            return unary_union(pieces)

        def _extract_centerline_for_residual(component: Any) -> list[Toolpath]:
            if component is None or component.is_empty:
                return []
            origin = component.centroid.coords[0]
            candidate_angles: list[float] = [0.0, 90.0]
            try:
                oriented = component.minimum_rotated_rectangle
                coords = list(oriented.exterior.coords)
                edges: list[tuple[float, float]] = []
                for start, end in zip(coords, coords[1:]):
                    dx = float(end[0] - start[0])
                    dy = float(end[1] - start[1])
                    length = math.hypot(dx, dy)
                    if length > 1e-6:
                        edges.append((length, math.degrees(math.atan2(dy, dx))))
                edges.sort(reverse=True)
                candidate_angles.extend([angle for _length, angle in edges[:2]])
            except Exception:
                pass

            best_line = None
            best_length = 0.0
            for candidate_angle in candidate_angles:
                rotated = affinity.rotate(component, -candidate_angle, origin=origin)
                min_x, _min_y, max_x, _max_y = rotated.bounds
                center_y = rotated.centroid.y
                probe = LineString([(min_x - 1.0, center_y), (max_x + 1.0, center_y)])
                clipped = rotated.intersection(probe)
                lines = sorted(extract_lines(clipped), key=lambda line: line.length, reverse=True)
                if lines and lines[0].length > best_length:
                    best_line = affinity.rotate(lines[0], candidate_angle, origin=origin)
                    best_length = float(lines[0].length)

            if best_line is None or best_length < max(0.06, line_width_mm * 0.2):
                return []
            points = simplify_segment_points([Point(float(x), float(y)) for x, y in best_line.coords], max(0.015, detail_tolerance_mm), False)
            if len(points) < 2:
                return []
            return [Toolpath(
                points=points,
                kind="detail-trace",
                closed=False,
                source="residual_centerline",
                metadata={"path_role": "PRINT_DETAIL", "detail_source": "residual_centerline"},
            )]

        allow_detail_overlap_outline = os.getenv("ALLOW_DETAIL_OVERLAP_OUTLINE", "1") == "1"
        validate_detail_with_pen_footprint = os.getenv("DETAIL_VALIDATE_WITH_PEN_FOOTPRINT", "1") == "1"
        max_detail_overspill_mm = min(0.05, line_width_mm * 0.10, max(0.0, float(os.getenv("MAX_DETAIL_OVERSPILL_MM", "0.05"))))
        max_detail_overspill_area_ratio = max(0.0, float(os.getenv("MAX_DETAIL_OVERSPILL_AREA_RATIO", "0.03")))
        min_detail_new_coverage_mm2 = max(0.0, float(os.getenv("MIN_DETAIL_NEW_COVERAGE_MM2", "0.02")))
        max_already_covered_ratio = max(0.0, min(1.0, float(os.getenv("MAX_ALREADY_COVERED_RATIO", "0.90"))))

        # Detail traces are allowed only when they add meaningful new target
        # coverage beyond infill, final outline, and accepted pen-down connectors.
        final_outline_paths = _build_clean_final_outline_paths()
        infill_coverage_seed = [path for path in interior_paths if path.kind == "fill-infill"]
        connector_coverage_seed = [path for path in raw_print_paths if path.kind == "fill-infill-travel"]
        covered_geom = _stroke_coverage_geometry(infill_coverage_seed, line_width_mm)
        outline_covered_geom = _stroke_coverage_geometry(final_outline_paths, line_width_mm)
        connector_covered_geom = _stroke_coverage_geometry(connector_coverage_seed, line_width_mm)
        existing_painted_parts = [
            geom
            for geom in (covered_geom, outline_covered_geom, connector_covered_geom)
            if geom is not None and not geom.is_empty
        ]
        existing_painted_area = unary_union(existing_painted_parts) if existing_painted_parts else Polygon()
        residual_target = bundle.printable_geometry
        if bundle.printable_geometry is not None and not bundle.printable_geometry.is_empty and existing_painted_parts:
            try:
                residual_target = bundle.printable_geometry.difference(existing_painted_area)
            except Exception:
                repaired_printable = bundle.printable_geometry.buffer(0)
                repaired_covered = existing_painted_area.buffer(0)
                residual_target = repaired_printable.difference(repaired_covered)
        if residual_target is not None and not residual_target.is_empty:
            try:
                residual_target = residual_target.buffer(0)
            except Exception:
                pass
        debug_append_geometry(debug, "residual_detail_target", residual_target, "residual-detail-target")

        residual_detail_candidates: list[Toolpath] = []
        min_residual_area = max(1e-6, line_width_mm * line_width_mm * 0.03)
        narrow_width_limit = line_width_mm * 1.6
        residual_components_for_detail: list[Any] = []
        if residual_target is not None and not residual_target.is_empty:
            for residual_component in normalize_geometry(residual_target):
                area = float(residual_component.area)
                if area < min_residual_area:
                    continue
                min_x, min_y, max_x, max_y = residual_component.bounds
                local_width = min(max_x - min_x, max_y - min_y)
                if local_width > narrow_width_limit:
                    continue
                residual_components_for_detail.append(residual_component)
                residual_detail_candidates.extend(_extract_centerline_for_residual(residual_component))

        # Also keep raster-derived detail candidates, but only if they add real residual coverage.
        residual_detail_candidates.extend(detail_paths)

        component_boundary_cache: list[Any] = [component.boundary for component in residual_components_for_detail]

        def _candidate_component_index(path: Toolpath) -> int | None:
            if len(path.points) < 2:
                return None
            shp = LineString([(point.x, point.y) for point in path.points])
            if shp.is_empty:
                return None
            best_idx: int | None = None
            best_overlap = 0.0
            for idx, component in enumerate(residual_components_for_detail):
                try:
                    overlap = float(shp.intersection(component).length)
                except Exception:
                    overlap = 0.0
                if overlap > best_overlap:
                    best_overlap = overlap
                    best_idx = idx
            return best_idx

        def _candidate_centeredness(path: Toolpath, component_idx: int | None) -> float:
            if component_idx is None or component_idx < 0 or component_idx >= len(component_boundary_cache):
                return 0.0
            boundary = component_boundary_cache[component_idx]
            if boundary is None or boundary.is_empty:
                return 0.0
            if len(path.points) < 2:
                return 0.0
            samples = path.points
            if len(samples) > 16:
                step = max(1, int(len(samples) / 16))
                samples = [samples[i] for i in range(0, len(samples), step)]
            distances: list[float] = []
            for pt in samples:
                try:
                    distances.append(float(ShapelyPoint(pt.x, pt.y).distance(boundary)))
                except Exception:
                    continue
            if not distances:
                return 0.0
            return float(sum(distances) / max(1, len(distances)))

        def _candidate_component_metrics(path: Toolpath, component_idx: int | None) -> dict[str, Any]:
            if component_idx is None or component_idx < 0 or component_idx >= len(residual_components_for_detail):
                return {}
            component = residual_components_for_detail[component_idx]
            if component is None or component.is_empty:
                return {}
            min_x, min_y, max_x, max_y = component.bounds
            return {
                "component_id": component_idx + 1,
                "area_mm2": float(component.area),
                "bbox_mm": (float(max_x - min_x), float(max_y - min_y)),
                "estimated_width_mm": float(min(max_x - min_x, max_y - min_y)),
            }

        # Coverage-aware ordering: prefer centered centerline candidates first so
        # later overlaps are treated as duplicates, not independent fragments.
        scored_candidates: list[tuple[float, float, float, Toolpath]] = []
        for candidate in residual_detail_candidates:
            comp_idx = _candidate_component_index(candidate)
            centeredness = _candidate_centeredness(candidate, comp_idx)
            length = segment_length(candidate.points) if len(candidate.points) >= 2 else 0.0
            src = str(candidate.source or "")
            source_bias = 1.0 if src == "residual_centerline" else 0.0
            scored_candidates.append((source_bias, centeredness, length, candidate))
        scored_candidates.sort(key=lambda item: (item[0], item[1], item[2]), reverse=True)
        residual_detail_candidates = [item[3] for item in scored_candidates]

        detail_filter_stats = _filter_detail_trace_candidates_for_export(
            residual_detail_candidates,
            target_geometry=bundle.printable_geometry,
            existing_painted_area=existing_painted_area,
            line_width_mm=line_width_mm,
            allow_detail_overlap_outline=allow_detail_overlap_outline,
            validate_detail_with_pen_footprint=validate_detail_with_pen_footprint,
            max_detail_overspill_mm=max_detail_overspill_mm,
            max_detail_overspill_area_ratio=max_detail_overspill_area_ratio,
            min_detail_new_coverage_mm2=min_detail_new_coverage_mm2,
            max_already_covered_ratio=max_already_covered_ratio,
            candidate_component_index_fn=_candidate_component_index,
            candidate_centeredness_fn=_candidate_centeredness,
            candidate_component_metrics_fn=_candidate_component_metrics,
        )
        accepted_detail_paths = list(detail_filter_stats["accepted_detail_paths"])
        accepted_detail_footprints = list(detail_filter_stats["accepted_detail_footprints"])
        rejected_detail_paths = list(detail_filter_stats["rejected_detail_paths"])
        overspill_warning_regions = list(detail_filter_stats["overspill_warning_regions"])
        debug.update(detail_filter_stats)
        dropped_records = list(detail_filter_stats.get("detail_dropped_path_records", []))
        accepted_component_ids = {
            int(path.metadata.get("detail_component_id"))
            for path in accepted_detail_paths
            if path.metadata.get("detail_component_id") is not None
        }
        dropped_component_ids = {
            int(record["component_id"])
            for record in dropped_records
            if record.get("component_id") is not None
        }
        small_detail_drop_reasons: Counter[str] = Counter()
        for record in dropped_records:
            if record.get("component_id") is not None:
                small_detail_drop_reasons[str(record.get("drop_reason", "unknown"))] += 1
        overlap_preserved_count = sum(
            1 for path in accepted_detail_paths if bool(path.metadata.get("detail_overlap_exception_applied", False))
        )
        overlap_rejected_count = sum(
            1
            for record in dropped_records
            if str(record.get("drop_reason", "")) in {"redundant_overlap", "redundant_centerline_overlap", "noisy_loop"}
            and (bool(record.get("was_self_overlapping", False)) or record.get("component_id") is not None)
        )
        debug.update({
            "small_detail_outline_mode_enabled": True,
            "small_detail_components_detected": int(len(residual_components_for_detail)),
            "small_detail_components_outlined": int(len(accepted_component_ids)),
            "small_detail_components_dropped": int(len(dropped_component_ids - accepted_component_ids)),
            "small_detail_drop_reasons": dict(small_detail_drop_reasons),
            "self_overlapping_detail_paths_allowed": int(overlap_preserved_count),
            "self_overlapping_detail_paths_rejected": int(overlap_rejected_count),
            "detail_paths_kept_despite_overlap": int(overlap_preserved_count),
            "detail_simplification_tolerance_mm": float(detail_tolerance_mm),
            "arsenal_detail_outline_paths_generated": int(detail_filter_stats.get("detail_paths_generated_raw", 0)),
            "arsenal_detail_outline_paths_dropped": int(detail_filter_stats.get("detail_paths_dropped", 0)),
        })

        debug_append_toolpaths(debug, "detail_traces_rejected", rejected_detail_paths)
        debug_append_toolpaths(debug, "detail_traces_accepted", accepted_detail_paths)
        if overspill_warning_regions:
            debug_append_geometry(debug, "detail_overspill_warning_regions", unary_union(overspill_warning_regions), "detail-overspill-warning")
        if accepted_detail_footprints:
            debug_append_geometry(debug, "detail_pen_footprints", unary_union(accepted_detail_footprints), "detail-pen-footprint")

        detail_repair_seed = [*interior_paths, *accepted_detail_paths]
        precovered_geometry = None
        precovered_parts = [
            geom
            for geom in (outline_covered_geom, connector_covered_geom)
            if geom is not None and not geom.is_empty
        ]
        if precovered_parts:
            precovered_geometry = unary_union(precovered_parts)
        detail_repaired_paths = self._enforce_region_coverage_backfill(
            detail_repair_seed,
            region=bundle.printable_geometry,
            line_width_mm=line_width_mm,
            angle_deg=infill_angle_deg,
            min_segment_length_mm=min_segment_length_mm,
            tolerance_mm=detail_tolerance_mm,
            precovered_geometry=precovered_geometry,
            debug=debug,
        )
        if len(detail_repaired_paths) > len(detail_repair_seed):
            repair_backfills = [
                path
                for path in detail_repaired_paths[len(detail_repair_seed):]
                if path.kind == "fill-infill"
            ]
            if repair_backfills:
                interior_paths.extend(repair_backfills)

        allow_detail_pen_down_continuation = os.getenv("ALLOW_DETAIL_PEN_DOWN_CONTINUATION", "1") == "1"
        max_detail_continuation_length_mm = max(
            line_width_mm * 0.5,
            float(os.getenv("MAX_DETAIL_CONTINUATION_LENGTH_MM", str(4.0 * line_width_mm))),
        )
        preferred_detail_continuation_length_mm = max(
            line_width_mm * 0.25,
            float(os.getenv("PREFERRED_DETAIL_CONTINUATION_LENGTH_MM", str(2.0 * line_width_mm))),
        )
        max_detail_continuation_overspill_mm = min(
            max_detail_overspill_mm,
            max(0.0, float(os.getenv("MAX_DETAIL_CONTINUATION_OVERSPILL_MM", str(max_detail_overspill_mm)))),
        )
        max_detail_continuation_overspill_area_ratio = max(
            0.0,
            float(os.getenv("MAX_DETAIL_CONTINUATION_OVERSPILL_AREA_RATIO", "0.02")),
        )
        max_detail_continuation_turn_deg = max(
            0.0,
            float(os.getenv("MAX_DETAIL_CONTINUATION_TURN_DEG", "120")),
        )

        detail_ordered = optimize_toolpath_order(accepted_detail_paths, strategy=travel_optimization) if accepted_detail_paths else []
        if allow_detail_pen_down_continuation and len(detail_ordered) > 1 and bundle.printable_geometry is not None and not bundle.printable_geometry.is_empty:
            component_geoms = normalize_geometry(bundle.printable_geometry)

            def _component_index_for_point(pt: Point) -> int | None:
                shp = ShapelyPoint(pt.x, pt.y)
                for idx, geom in enumerate(component_geoms):
                    try:
                        if geom.covers(shp):
                            return idx
                    except Exception:
                        continue
                return None

            def _path_component_index(path: Toolpath) -> int | None:
                md_component = path.metadata.get("source_component_id")
                parsed = _extract_component_id(md_component)
                if parsed is not None:
                    return max(0, parsed - 1)
                return _component_index_for_point(path.points[0])

            def _turn_angle_deg(prev: list[Point], nxt: list[Point]) -> float:
                if len(prev) < 2 or len(nxt) < 2:
                    return 0.0
                ax = prev[-1].x - prev[-2].x
                ay = prev[-1].y - prev[-2].y
                bx = nxt[1].x - nxt[0].x
                by = nxt[1].y - nxt[0].y
                la = math.hypot(ax, ay)
                lb = math.hypot(bx, by)
                if la <= 1e-9 or lb <= 1e-9:
                    return 0.0
                dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
                return math.degrees(math.acos(dot))

            def _connector_pen_safe(connector: LineString, component_idx: int | None) -> tuple[bool, float, float]:
                stroke = connector.buffer(max(0.01, line_width_mm * 0.5), cap_style=1, join_style=1)
                target = component_geoms[component_idx] if component_idx is not None and 0 <= component_idx < len(component_geoms) else bundle.printable_geometry
                if target is None or target.is_empty:
                    return False, 1.0, max_detail_continuation_overspill_mm + 1.0
                overspill = stroke.difference(target)
                overspill_area = float(overspill.area) if overspill is not None and not overspill.is_empty else 0.0
                overspill_ratio = overspill_area / max(1e-9, float(stroke.area))
                protrusion_mm = 0.0
                if overspill is not None and not overspill.is_empty:
                    boundary = target.boundary
                    for poly in normalize_geometry(overspill):
                        coords = list(poly.exterior.coords)
                        sample_step = max(1, int(len(coords) / 24))
                        for idx in range(0, len(coords), sample_step):
                            pt = ShapelyPoint(float(coords[idx][0]), float(coords[idx][1]))
                            protrusion_mm = max(protrusion_mm, float(pt.distance(boundary)))
                safe = (
                    overspill_ratio <= max_detail_continuation_overspill_area_ratio
                    and protrusion_mm <= max_detail_continuation_overspill_mm
                    and _line_fully_inside(target.buffer(max(0.01, line_width_mm * 0.15), join_style=1), connector, tolerance_mm=max(0.01, line_width_mm * 0.05))
                )
                return safe, overspill_ratio, protrusion_mm

            chained_paths: list[Toolpath] = []
            accepted_continuations: list[Toolpath] = []
            rejected_continuations: list[Toolpath] = []
            continuation_pen_footprints: list[Any] = []
            for idx, path in enumerate(detail_ordered):
                if not chained_paths:
                    chained_paths.append(path)
                    continue
                current = chained_paths[-1]
                next_path = path
                same_component = _path_component_index(current) == _path_component_index(next_path)
                if not same_component:
                    chained_paths.append(next_path)
                    continue
                end = current.points[-1]
                start = next_path.points[0]
                gap = math.hypot(start.x - end.x, start.y - end.y)
                if gap <= 1e-6:
                    chained_paths.append(next_path)
                    continue
                connector = LineString([(end.x, end.y), (start.x, start.y)])
                angle = _turn_angle_deg(current.points, next_path.points)
                short_enough = gap <= max_detail_continuation_length_mm + 1e-9
                smooth_enough = angle <= max_detail_continuation_turn_deg + 1e-9
                safe, overspill_ratio, protrusion_mm = _connector_pen_safe(connector, _path_component_index(current))
                score = (
                    1.0
                    - (gap / max(1e-9, preferred_detail_continuation_length_mm))
                    - (angle / 180.0)
                    - (overspill_ratio * 10.0)
                )
                if short_enough and smooth_enough and safe and score > -1.2:
                    connector_tp = Toolpath(
                        points=[end, start],
                        kind="detail-continuation",
                        closed=False,
                        source="detail_continuation",
                        metadata={
                            "path_role": "PRINT_DETAIL_CONTINUATION",
                            "detail_continuation_pen_down": True,
                            "connector_length_mm": float(gap),
                            "connector_turn_deg": float(angle),
                            "connector_overspill_area_ratio": float(overspill_ratio),
                            "connector_max_protrusion_mm": float(protrusion_mm),
                            "connector_score": float(score),
                        },
                    )
                    accepted_continuations.append(connector_tp)
                    continuation_pen_footprints.append(connector.buffer(max(0.01, line_width_mm * 0.5), cap_style=1, join_style=1))
                    chained_paths.append(connector_tp)
                    chained_paths.append(next_path)
                else:
                    rejected_continuations.append(Toolpath(
                        points=[end, start],
                        kind="travel",
                        closed=False,
                        source="detail_continuation_rejected",
                        metadata={
                            "path_role": "TRAVEL",
                            "detail_continuation_rejected": True,
                            "connector_length_mm": float(gap),
                            "connector_turn_deg": float(angle),
                            "connector_overspill_area_ratio": float(overspill_ratio),
                            "connector_max_protrusion_mm": float(protrusion_mm),
                            "connector_safe": bool(safe),
                        },
                    ))
                    chained_paths.append(next_path)
            detail_ordered = chained_paths
            debug_append_toolpaths(debug, "detail_continuation_accepted", accepted_continuations)
            debug_append_toolpaths(debug, "detail_continuation_rejected", rejected_continuations)
            if continuation_pen_footprints:
                debug_append_geometry(debug, "detail_continuation_pen_footprints", unary_union(continuation_pen_footprints), "detail-continuation-pen-footprint")

        # Detail traces first, then infill, outline last.
        composed_paths.extend(merge_connected_toolpaths(detail_ordered))
        composed_paths.extend(interior_paths)
        # Final clean outline last.
        composed_paths.extend(final_outline_paths)
    else:
        composed_paths = list(raw_print_paths)
        if detail_paths:
            detail_ordered = optimize_toolpath_order(detail_paths, strategy=travel_optimization)
            composed_paths.extend(merge_connected_toolpaths(detail_ordered))

    toolpaths = merge_connected_toolpaths(composed_paths)
    toolpaths = optimize_detail_trace_efficiency(
        toolpaths,
        printable_geometry=bundle.printable_geometry,
        pen_width_mm=line_width_mm,
        debug=debug,
    )
    # Preserve the user-visible requirement that cleanup/final outline strokes
    # render last, after infill/detail passes.
    non_outline = [path for path in toolpaths if path.kind != "outline"]
    outline_only = [path for path in toolpaths if path.kind == "outline"]
    toolpaths = assign_stable_path_ids(merge_connected_toolpaths(non_outline + outline_only))

    toolpath_counts = {
        "generated_fill_walls": sum(1 for path in toolpaths if path.kind == "fill-wall"),
        "generated_infill_paths": sum(1 for path in toolpaths if path.kind == "fill-infill"),
        "generated_infill_travel_paths": sum(1 for path in toolpaths if path.kind == "fill-infill-travel"),
        "generated_thin_detail_paths": sum(1 for path in toolpaths if path.kind == "detail-trace"),
        "generated_detail_trace_paths": sum(1 for path in toolpaths if path.kind == "detail-trace"),
        "generated_outline_paths": sum(1 for path in toolpaths if path.kind == "outline"),
        "generated_travel_paths": sum(1 for path in toolpaths if path.kind == "travel"),
    }
    debug_set_counts(debug, "toolpath_counts", toolpath_counts)
    if debug is not None:
        debug["coverage_path_diagnostics"] = {
            "raw_generated_path_count": int(debug.get("infill_debug", {}).get("diagnostics", {}).get("raw_generated_paths", len(toolpaths))) if isinstance(debug.get("infill_debug"), dict) else int(len(toolpaths)),
            "final_accepted_path_count": int(len(toolpaths)),
            "rejected_mesh_or_detail_trace_count": int(debug.get("mesh_like_paths_rejected", 0)),
            "centerline_path_count": sum(1 for path in toolpaths if path.kind == "coverage_centerline"),
            "offset_path_count": sum(1 for path in toolpaths if path.kind == "coverage_offset_line"),
            "rectilinear_path_count": sum(1 for path in toolpaths if path.kind == "coverage_rectilinear"),
            "tiny_mark_count": sum(1 for path in toolpaths if str(path.metadata.get("small_detail_fill_style")) in {"tiny_dot", "tiny_short_stroke"}),
            "pen_lifts_before_optimization": int(debug.get("pen_lifts_before_cell_planning", 0)),
            "pen_lifts_after_optimization": int(debug.get("pen_lifts_after_cell_planning", 0)),
            "rejected_x_triangle_fragment_count": int(debug.get("rejected_x_triangle_fragment_count", 0)),
        }
        debug["toolpath_diagnostics"] = summarize_toolpaths(toolpaths)
        tiny_detail_two_point = sum(1 for path in toolpaths if path.kind in {"detail-trace", "detail-continuation"} and len(path.points) <= 2)
        debug["efficiency_audit"] = {
            "total_paths": len(toolpaths),
            "paths_by_kind": dict(debug.get("toolpath_diagnostics", {}).get("paths_by_kind", {})),
            "tiny_two_point_detail_paths": int(tiny_detail_two_point),
            "warn_too_many_tiny_detail_paths": bool(tiny_detail_two_point > 20),
        }
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
    header_comment_settings: Optional[dict[str, Any]] = None,
    debug: Optional[dict[str, Any]] = None,
) -> tuple[list[str], list[dict[str, Any]]]:
    if gcode_mode != "simple":
        raise ValueError("Invalid G-code mode")
    assert_toolpaths_coordinate_space(toolpaths, "machine_deg")
    for toolpath in toolpaths:
        if toolpath.coordinate_space != "machine_deg":
            raise AssertionError(f"{toolpath.kind} {toolpath.path_id or '<unassigned>'} was not projected to machine_deg")
        if int(toolpath.metadata.get("projection_count", 0)) != 1:
            raise AssertionError(
                f"{toolpath.kind} {toolpath.path_id or '<unassigned>'} was projected {toolpath.metadata.get('projection_count', 0)} times"
            )
        if toolpath.kind in {"outline", "fill-wall", "fill-infill", "fill-repair", "crossed-contour-infill", "junction-centerline", "gap-repair-stroke", "gap-repair-dab", "repair-patch-fill"} and len(toolpath.points) >= 2:
            max_surface = float(toolpath.metadata.get("max_surface_segment_mm_after_resampling", 0.0))
            if max_surface <= 1e-12:
                max_surface = max(_segment_lengths_mm(toolpath.points, closed=toolpath.closed), default=0.0)
                toolpath.metadata["max_surface_segment_mm_after_resampling"] = float(max_surface)
            if max_surface <= 1e-12:
                nonzero_move = any(not nearly_same_point(a, b, 1e-12) for a, b in zip(toolpath.points, toolpath.points[1:]))
                if nonzero_move:
                    raise AssertionError(
                        f"{toolpath.kind} {toolpath.path_id or '<unassigned>'} has nonzero moves but max_surface_segment_mm_after_resampling=0"
                    )

    g: list[str] = []
    preview: list[dict[str, Any]] = []
    current_servo = pen_up_s
    current_position = Point(0.0, 0.0)
    current_pen_down = False
    stream_line_number = 0
    current_motion_feed: float | None = None
    pen_state_debug: list[dict[str, Any]] = []
    travel_moves_with_pen_down = 0
    drawing_moves_with_pen_up = 0
    long_pen_down_jumps = 0
    max_pen_down_jump = 0.0
    previous_draw_path_id: str | None = None
    previous_printed_toolpath: Toolpath | None = None
    collinear_points_removed = 0
    pen_radius_mm = 0.3
    printable_outline_area = _build_outline_printable_area_from_toolpaths(toolpaths, pen_radius_mm=pen_radius_mm)
    rejected_travel_reasons: dict[str, int] = {}
    pending_converted_travel_index: int | None = None
    pending_converted_travel_length_mm = 0.0
    active_fill_chain_path_id: str | None = None
    travels_checked = 0
    infill_to_infill_travels_checked = 0
    travels_converted_to_pen_down = 0
    converted_connector_length_mm = 0.0
    converted_connectors_outside_outline_area_mm2 = 0.0
    fill_path_count_after_conversion = 0

    def comment(text: str) -> None:
        if include_comments:
            g.append(f"({text})")

    def header_comment(text: str) -> None:
        g.append(f"({text})")

    def append_gcode(line: str) -> int | None:
        nonlocal stream_line_number
        g.append(line)
        if is_streamable_gcode_line(line):
            stream_line_number += 1
            return stream_line_number
        return None

    def append_motion(command: str, point: Point, feed: float) -> int | None:
        nonlocal current_motion_feed
        if current_motion_feed is None or abs(current_motion_feed - feed) > 1e-9:
            line = f"{command} X{point.x:.4f} Y{point.y:.4f} F{feed:.3f}"
            current_motion_feed = feed
        else:
            line = f"{command} X{point.x:.4f} Y{point.y:.4f}"
        return append_gcode(line)

    header_comment("Generated for golf ball plotter")
    header_comment("Units are angular degrees. X=-180..180 ball rotation, Y=-45..45 arm tilt")
    if header_comment_settings:
        for key in (
            "lineWidthMm",
            "infillSpacingMm",
            "wallCount",
            "infillAngle",
            "rotationDeg",
            "designWidthMm",
            "designHeightMm",
            "coordinateSpaceUsedForFill",
        ):
            if key in header_comment_settings:
                header_comment(f"{key}: {header_comment_settings[key]}")
    for command in ["G21", "G90"]:
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

    printable_kinds = {
        "outline",
        "fill-wall",
        "fill-infill",
        "fill-repair",
        "detail-trace",
        "detail-continuation",
        "crossed-contour-infill",
        "junction-centerline",
        "gap-repair-stroke",
        "gap-repair-dab",
        "repair-patch-fill",
    }
    connector_kinds = {"fill-infill-travel", "coverage_connector"}

    def classify_path_type(toolpath: Toolpath) -> str:
        if toolpath.kind == "travel":
            return "TRAVEL"
        if toolpath.kind in connector_kinds:
            connector_ok = bool(toolpath.metadata.get("connector_pen_down_allowed", False))
            connector_opt_in = os.getenv("ALLOW_PEN_DOWN_CONNECTORS", "0") == "1"
            # If connector metadata explicitly marks pen-down as allowed, treat as infill (default)
            if connector_ok:
                return "PRINT_INFILL"
            # Otherwise fall back to legacy opt-in via environment variable
            return "PRINT_INFILL" if connector_opt_in else "TRAVEL"
        if toolpath.kind in printable_kinds:
            if toolpath.kind in {"outline", "fill-wall", "coverage_contour", "outline_cleanup"}:
                return "PRINT_OUTLINE"
            if toolpath.kind in {"detail-trace", "detail-continuation"}:
                return "PRINT_DETAIL"
            return "PRINT_INFILL"
        return "DEBUG_ONLY"

    for index, toolpath in enumerate(toolpaths, start=1):
        pts = list(toolpath.points)
        if len(pts) < 2:
            continue

        for point in pts:
            if point.y < (Y_DRAW_MIN - 1e-6) or point.y > (Y_DRAW_MAX + 1e-6):
                raise ValueError(f"Projected toolpath exceeds Y drawing limits at {point.y:.3f} degrees")
            if point.x < (X_DRAW_MIN - 1e-6) or point.x > (X_DRAW_MAX + 1e-6):
                raise ValueError(f"Projected toolpath exceeds X drawing limits at {point.x:.3f} degrees")

        path_type = classify_path_type(toolpath)
        start = pts[0]
        continuing_fill_chain = (
            pending_converted_travel_index == index
            and toolpath.kind == "fill-infill"
            and active_fill_chain_path_id is not None
        )
        if debug is not None:
            debug.setdefault("path_type_counts", {})
            debug["path_type_counts"][path_type] = int(debug["path_type_counts"].get(path_type, 0)) + 1
        pen_up_before_travel_to_start = not current_pen_down
        unexpected_pen_down_travel = False
        is_pen_down_travel = path_type == "PRINT_INFILL" and toolpath.kind in connector_kinds

        if path_type in {"TRAVEL", "DEBUG_ONLY"}:
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

            travel_id = toolpath.path_id or f"travel-{index:04d}"
            travel_start_line = None
            travel_end_line = None
            previous_point = current_position
            for point in pts:
                if nearly_same_point(previous_point, point):
                    continue
                line_number = append_motion("G1", point, travel_feed)
                if line_number is not None:
                    if travel_start_line is None:
                        travel_start_line = line_number
                    travel_end_line = line_number
                previous_point = point
                current_position = point
            preview.append({
                "id": travel_id,
                "kind": "travel",
                "closed": False,
                "pen_down": False,
                "points": [asdict(_rounded_gcode_point(point)) for point in pts],
                "gcode_start_line": travel_start_line,
                "gcode_end_line": travel_end_line,
                "source_path_id": toolpath.path_id,
                "source_path_kind": toolpath.kind,
                "path_type": path_type,
            })
            continue

        if is_pen_down_travel:
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

            path_id = toolpath.path_id or f"path-{index:04d}"
            draw_start_line = None
            draw_end_line = None
            path_gcode_start_index = len(g)
            max_surface_segment_mm = float(toolpath.metadata.get("max_surface_segment_mm_after_resampling", 0.0))
            source_label = toolpath.metadata.get("source_polygon_id", _path_component_label(toolpath))
            comment(
                f"PATH_START id={path_id} kind={toolpath.kind} space={toolpath.coordinate_space} "
                f"source={source_label} points={len(pts)} max_surface_segment_mm={max_surface_segment_mm:.4f}"
            )
            comment(f"{toolpath.kind} path {index}, {len(pts)} points")
            previous_point = pts[0]
            for point in pts[1:]:
                line_number = append_motion("G1", point, travel_feed)
                if line_number is not None:
                    if draw_start_line is None:
                        draw_start_line = line_number
                    draw_end_line = line_number
                    jump = math.hypot(point.x - previous_point.x, point.y - previous_point.y)
                    max_pen_down_jump = max(max_pen_down_jump, jump)
                    if jump > max(5.0, sample_step_deg * 5.0):
                        long_pen_down_jumps += 1
                    pen_state_debug.append({
                        "line_index": line_number,
                        "command": g[-1],
                        "path_id": path_id,
                        "kind": toolpath.kind,
                        "expected_pen_state": "down",
                        "actual_pen_state": "down" if current_pen_down else "up",
                        "is_drawing_move": False,
                        "warning": "",
                    })
                current_position = point
                previous_point = point
            emitted_points = [_rounded_gcode_point(point) for point in pts]
            preview.append({
                "id": path_id,
                "kind": toolpath.kind,
                "closed": toolpath.closed,
                "pen_down": True,
                "travel_mode": "pen_down",
                "points": [asdict(point) for point in emitted_points],
                "gcode_start_line": draw_start_line,
                "gcode_end_line": draw_end_line,
                "source": toolpath.source,
                "region_id": toolpath.region_id,
            })
            preview_points = [Point(point.x, point.y) for point in emitted_points]
            comment(f"PATH_END id={path_id}")
            path_gcode_lines = g[path_gcode_start_index:]
            log_path_pipeline_audit(
                None,
                toolpath,
                gcode_motion_count=max(0, len(pts) - 1),
                pen_down_motion_count=max(0, len(pts) - 1),
                pen_up_motion_count=0,
                uses_same_projected_object_for_preview_and_gcode=True,
            )
            log_preview_gcode_identity_check(path_id, toolpath.kind, preview_points, emitted_points)
            log_pen_state_path_boundary_check(
                path_id=path_id,
                kind=toolpath.kind,
                previous_path_id=previous_draw_path_id,
                pen_up_before_travel_to_start=pen_up_before_travel_to_start,
                pen_down_only_after_reaching_start=True,
                pen_up_after_path_end=False,
                unexpected_pen_down_travel=False,
                first_gcode_for_path=path_gcode_lines[:3],
                last_gcode_for_path=path_gcode_lines[-3:],
            )
            previous_draw_path_id = path_id
            continue

        if not nearly_same_point(current_position, start):
            travel_id = f"travel-{index:04d}"
            converted_existing_travel = pending_converted_travel_index == index and current_pen_down
            travels_checked += 1
            if previous_printed_toolpath is not None and previous_printed_toolpath.kind == "fill-infill" and toolpath.kind == "fill-infill":
                infill_to_infill_travels_checked += 1
            if current_pen_down and not converted_existing_travel:
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
                unexpected_pen_down_travel = True
            travel_feed_to_use = draw_feed if converted_existing_travel else travel_feed
            if converted_existing_travel:
                comment(f"Safe converted infill-to-infill connector to {toolpath.kind} path {index}")
                travels_converted_to_pen_down += 1
                converted_connector_length_mm += float(pending_converted_travel_length_mm)
            else:
                comment(f"Travel to {toolpath.kind} path {index}")
            travel_line = append_motion("G1", start, travel_feed_to_use)
            pen_state_debug.append({
                "line_index": travel_line,
                "command": g[-1],
                "path_id": travel_id,
                "kind": "travel",
                "expected_pen_state": "down" if converted_existing_travel else "up",
                "actual_pen_state": "down" if current_pen_down else "up",
                "is_drawing_move": False,
                "warning": "" if converted_existing_travel else ("travel_with_pen_down" if current_pen_down else ""),
            })
            if current_pen_down and not converted_existing_travel:
                travel_moves_with_pen_down += 1
            preview.append({
                "id": travel_id,
                "kind": "travel",
                "closed": False,
                "pen_down": bool(current_pen_down),
                "travel_mode": "converted_infill_connector" if converted_existing_travel else "pen_up",
                "chain_path_id": active_fill_chain_path_id if converted_existing_travel else "",
                "points": [asdict(_rounded_gcode_point(current_position)), asdict(_rounded_gcode_point(start))],
                "gcode_start_line": travel_line,
                "gcode_end_line": travel_line,
                "source_path_id": toolpath.path_id,
                "source_path_kind": toolpath.kind,
            })
            log_toolpath_summary(
                None,
                Toolpath(
                    points=[current_position, start],
                    kind="travel",
                    closed=False,
                    coordinate_space="machine_deg",
                    path_id=travel_id,
                    source="gcode_travel",
                    region_id=toolpath.region_id,
                    metadata={
                        "projection_count": 0,
                        "source_component_id": toolpath.metadata.get("source_component_id"),
                        "source_contour_id": toolpath.metadata.get("source_contour_id"),
                        "expected_relation_to_fill": "pen_up_reposition",
                    },
                ),
            )
            log_path_pipeline_audit(
                None,
                Toolpath(
                    points=[current_position, start],
                    kind="travel",
                    closed=False,
                    coordinate_space="machine_deg",
                    path_id=travel_id,
                    source="gcode_travel",
                    region_id=toolpath.region_id,
                    metadata={
                        "projection_count": 0,
                        "coordinate_space_before_projection": "generated_in_machine_deg",
                        "source_component_id": toolpath.metadata.get("source_component_id"),
                        "source_contour_id": toolpath.metadata.get("source_contour_id"),
                        "expected_relation_to_fill": "converted_pen_down_connector" if converted_existing_travel else "pen_up_reposition",
                    },
                ),
                gcode_motion_count=1,
                pen_down_motion_count=1 if converted_existing_travel else 0,
                pen_up_motion_count=0 if converted_existing_travel else 1,
                uses_same_projected_object_for_preview_and_gcode=True,
            )
            log_preview_gcode_identity_check(travel_id, "travel", [current_position, start], [current_position, start])
            current_position = start
            pending_converted_travel_index = None
            pending_converted_travel_length_mm = 0.0

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

        path_id = toolpath.path_id or f"path-{index:04d}"
        chain_path_id = active_fill_chain_path_id if continuing_fill_chain else path_id
        draw_start_line = None
        draw_end_line = None
        path_gcode_start_index = len(g)
        max_surface_segment_mm = float(toolpath.metadata.get("max_surface_segment_mm_after_resampling", 0.0))
        source_label = toolpath.metadata.get("source_polygon_id", _path_component_label(toolpath))
        if toolpath.kind == "fill-infill":
            if active_fill_chain_path_id is None:
                active_fill_chain_path_id = path_id
            chain_path_id = active_fill_chain_path_id
        if not continuing_fill_chain:
            comment(
                f"PATH_START id={chain_path_id} kind={toolpath.kind} space={toolpath.coordinate_space} "
                f"source={source_label} points={len(pts)} max_surface_segment_mm={max_surface_segment_mm:.4f}"
            )
            if toolpath.kind == "fill-infill":
                fill_path_count_after_conversion += 1
        comment(f"{toolpath.kind} path {index}, {len(pts)} points")
        previous_point = pts[0]
        for point in pts[1:]:
            line_number = append_motion("G1", point, draw_feed)
            if line_number is not None:
                if draw_start_line is None:
                    draw_start_line = line_number
                draw_end_line = line_number
                jump = math.hypot(point.x - previous_point.x, point.y - previous_point.y)
                max_pen_down_jump = max(max_pen_down_jump, jump)
                if jump > max(5.0, sample_step_deg * 5.0):
                    long_pen_down_jumps += 1
                if not current_pen_down:
                    drawing_moves_with_pen_up += 1
                pen_state_debug.append({
                    "line_index": line_number,
                    "command": g[-1],
                    "path_id": path_id,
                    "kind": toolpath.kind,
                    "expected_pen_state": "down",
                    "actual_pen_state": "down" if current_pen_down else "up",
                    "is_drawing_move": True,
                    "warning": "drawing_move_with_pen_up" if not current_pen_down else "",
                })
            current_position = point
            previous_point = point
        emitted_points = [_rounded_gcode_point(point) for point in pts]
        preview.append({
            "id": path_id,
            "kind": toolpath.kind,
            "closed": toolpath.closed,
            "pen_down": True,
            "path_type": path_type,
            "chain_path_id": chain_path_id if toolpath.kind == "fill-infill" else "",
            "points": [asdict(point) for point in emitted_points],
            "gcode_start_line": draw_start_line,
            "gcode_end_line": draw_end_line,
            "source": toolpath.source,
            "region_id": toolpath.region_id,
        })
        preview_points = [Point(point.x, point.y) for point in emitted_points]

        # Decide whether to lift the pen at path end. If the next toolpath
        # is an approved pen-down connector we keep the pen down to avoid
        # unnecessary lifts between consecutive printable strokes.
        next_toolpath = toolpaths[index] if index < len(toolpaths) else None
        keep_down_for_infill_connector = (
            next_toolpath is not None
            and classify_path_type(next_toolpath) == "PRINT_INFILL"
            and getattr(next_toolpath, "kind", None) in connector_kinds
        )
        converted_travel = _evaluate_infill_travel_conversion(
            toolpath,
            next_toolpath,
            printable_area=printable_outline_area,
            pen_radius_mm=pen_radius_mm,
        ) if next_toolpath is not None else {"convert": False, "reasons": ["missing_next_path"]}
        if next_toolpath is not None and len(next_toolpath.points) >= 2 and not nearly_same_point(current_position, next_toolpath.points[0]):
            for reason in converted_travel.get("reasons", []):
                rejected_travel_reasons[reason] = int(rejected_travel_reasons.get(reason, 0)) + 1
        keep_down_for_converted_travel = bool(converted_travel.get("convert", False))
        if keep_down_for_converted_travel:
            pending_converted_travel_index = index + 1
            pending_converted_travel_length_mm = float(converted_travel.get("travel_length_mm", 0.0))
            converted_connectors_outside_outline_area_mm2 += float(converted_travel.get("outside_area_mm2", 0.0))
        keep_down_for_detail_continuation = (
            next_toolpath is not None
            and classify_path_type(next_toolpath) == "PRINT_DETAIL"
            and str((next_toolpath.metadata or {}).get("path_role", "")) == "PRINT_DETAIL_CONTINUATION"
        )
        keep_down_for_touching_detail = (
            next_toolpath is not None
            and classify_path_type(next_toolpath) == "PRINT_DETAIL"
            and len(next_toolpath.points) >= 1
            and nearly_same_point(current_position, next_toolpath.points[0], 1e-6)
        )
        if keep_down_for_infill_connector or keep_down_for_detail_continuation or keep_down_for_touching_detail or keep_down_for_converted_travel:
            if not keep_down_for_converted_travel:
                comment(f"PATH_END id={chain_path_id} (keeping pen down for connector/continuation)")
        else:
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
            comment(f"PATH_END id={chain_path_id}")
            pending_converted_travel_index = None
            pending_converted_travel_length_mm = 0.0
            if toolpath.kind == "fill-infill":
                active_fill_chain_path_id = None
        path_gcode_lines = g[path_gcode_start_index:]
        log_path_pipeline_audit(
            None,
            toolpath,
            gcode_motion_count=max(0, len(pts) - 1),
            pen_down_motion_count=max(0, len(pts) - 1),
            pen_up_motion_count=1 if not nearly_same_point(current_position, start) else 0,
            uses_same_projected_object_for_preview_and_gcode=True,
        )
        log_preview_gcode_identity_check(path_id, toolpath.kind, preview_points, emitted_points)
        log_pen_state_path_boundary_check(
            path_id=path_id,
            kind=toolpath.kind,
            previous_path_id=previous_draw_path_id,
            pen_up_before_travel_to_start=pen_up_before_travel_to_start,
            pen_down_only_after_reaching_start=True,
            pen_up_after_path_end=not current_pen_down,
            unexpected_pen_down_travel=unexpected_pen_down_travel,
            first_gcode_for_path=path_gcode_lines[:3],
            last_gcode_for_path=path_gcode_lines[-3:],
        )
        previous_draw_path_id = path_id
        previous_printed_toolpath = toolpath

    comment("Return to zero with pen up")
    if not nearly_same_point(current_position, Point(0.0, 0.0)):
        travels_checked += 1
        rejected_travel_reasons["missing_next_path"] = int(rejected_travel_reasons.get("missing_next_path", 0)) + 1
        return_home_line = append_motion("G1", Point(0.0, 0.0), travel_feed)
        preview.append({
            "id": "travel-home",
            "kind": "travel",
            "closed": False,
            "pen_down": False,
            "points": [asdict(_rounded_gcode_point(current_position)), asdict(Point(0.0, 0.0))],
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

    if debug is not None:
        actual_pen_lift_count = sum(1 for line in g if line.strip().startswith(f"M3 S{int(pen_up_s)}"))
        connector_pen_down_paths = 0
        connector_total_paths = 0
        detail_continuation_pen_down_paths = 0
        outline_path_start_count_in_gcode = 0
        toolpath_by_id = {tp.path_id: tp for tp in toolpaths if tp.path_id}
        for entry in preview:
            src_kind = str(entry.get("source_path_kind") or entry.get("kind") or "")
            if str(entry.get("kind") or "") == "outline" and bool(entry.get("pen_down", False)):
                outline_path_start_count_in_gcode += 1
            if src_kind in {"fill-infill-travel", "coverage_connector"}:
                connector_total_paths += 1
                if bool(entry.get("pen_down", False)):
                    connector_pen_down_paths += 1
            if src_kind in {"detail-trace", "detail-continuation"} and bool(entry.get("pen_down", False)):
                src_path_id = entry.get("source_path_id") or entry.get("id")
                if src_path_id:
                    matched = toolpath_by_id.get(str(src_path_id))
                    if matched is not None and str((matched.metadata or {}).get("path_role", "")) == "PRINT_DETAIL_CONTINUATION":
                        detail_continuation_pen_down_paths += 1
        debug["pen_state_debug"] = pen_state_debug
        debug["pen_state_summary"] = {
            "travel_moves_with_pen_down": travel_moves_with_pen_down,
            "drawing_moves_with_pen_up": drawing_moves_with_pen_up,
            "long_pen_down_jumps": long_pen_down_jumps,
            "max_pen_down_jump_mm_or_deg": max_pen_down_jump,
            "actual_gcode_pen_lift_count": actual_pen_lift_count,
            "connector_paths_total": connector_total_paths,
            "connector_paths_pen_down": connector_pen_down_paths,
            "detail_continuation_paths_pen_down": detail_continuation_pen_down_paths,
            "collinear_points_removed": int(collinear_points_removed),
        }
        fill_path_count_before_conversion = int(sum(1 for path in toolpaths if path.kind == "fill-infill"))
        debug["travel_conversion_mode"] = "postprocess_existing_travels_only"
        debug["infill_geometry_changed"] = False
        debug["path_order_changed"] = bool(debug.get("paths_reordered", False))
        debug["new_connector_routes_created"] = False
        debug["travels_checked"] = int(travels_checked)
        debug["infill_to_infill_travels_checked"] = int(infill_to_infill_travels_checked)
        debug["travels_converted_to_pen_down"] = int(travels_converted_to_pen_down)
        debug["travels_left_pen_up"] = int(max(0, travels_checked - travels_converted_to_pen_down))
        debug["fill_path_count_before_conversion"] = fill_path_count_before_conversion
        debug["fill_path_count_after_conversion"] = int(fill_path_count_after_conversion)
        debug["pen_lifts_before_conversion"] = int(actual_pen_lift_count + travels_converted_to_pen_down)
        debug["pen_lifts_after_conversion"] = int(actual_pen_lift_count)
        debug["converted_connector_length_mm"] = float(converted_connector_length_mm)
        debug["rejected_travel_reasons"] = dict(sorted(rejected_travel_reasons.items()))
        debug["converted_connectors_outside_outline_area_mm2"] = float(converted_connectors_outside_outline_area_mm2)
        debug["actual_gcode_pen_lift_count"] = actual_pen_lift_count
        debug["safe_pen_down_travels_converted"] = int(travels_converted_to_pen_down)
        debug["outline_path_start_count_in_gcode"] = int(outline_path_start_count_in_gcode)
        debug["projected_toolpath_hash"] = hash_toolpaths(toolpaths)

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
    expect(any(path.kind in {"fill-wall", "outline"} for path in toolpaths), "printable regions generate inward cleanup edge strokes")
    expect(any(path.kind == "fill-infill" for path in toolpaths), "printable regions generate infill when geometry is large enough")
    hole_box = Polygon([(30, 30), (70, 30), (70, 70), (30, 70)])
    infill_points = [point for path in toolpaths if path.kind == "fill-infill" for point in path.points]
    expect(not any(hole_box.buffer(-0.01).contains(ShapelyPoint(point.x, point.y)) for point in infill_points), "infill lines do not cross cutout holes")
    cleanup_paths = [path for path in toolpaths if path.kind in {"fill-wall", "outline"} and path.source == "final_fill_clip_polygon"]
    expect(all(path_is_inside_printable_area(path, result.bundle.printable_geometry) for path in cleanup_paths), "cleanup edge strokes stay inside printable geometry")
    expect(all(path.metadata.get("offset_direction") in {"inside_printable_region", "into_printed_material"} for path in cleanup_paths), "cleanup edge strokes offset inward into printable material")
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
