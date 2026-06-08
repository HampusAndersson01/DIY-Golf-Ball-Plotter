from __future__ import annotations

import hashlib
import json
import math
from io import BytesIO
from pathlib import Path
from unittest.mock import patch

import pytest

from app import create_app
from app.models.machine_state import MachineState
from app.services.raster_analysis_service import RasterAnalysisService


ROOT = Path(__file__).resolve().parents[1]
ARSENAL_FIXTURE = ROOT / "tests" / "fixtures" / "images" / "black-arsenal-logo-png-1.png"


@pytest.fixture()
def client():
    app = create_app()
    app.config.update(TESTING=True)
    with app.test_client() as test_client:
        yield test_client


def _selected_black_color_id(app, image_bytes: bytes) -> str:
    raster = RasterAnalysisService(app.config, MachineState(default_pen_up_s=575))
    analysis = raster.analyze_image(image_bytes, max_colors=app.config["DEFAULT_RASTER_MAX_COLORS"])
    selected = next(
        (color.id for color in analysis.colors if color.hex == "#000000"),
        analysis.colors[0].id if analysis.colors else None,
    )
    assert selected is not None
    return selected


def _frontend_generate(client, *, disable_thin_source_pass: bool) -> dict[str, object]:
    fixture_bytes = ARSENAL_FIXTURE.read_bytes()
    selected = _selected_black_color_id(client.application, fixture_bytes)
    patcher = None
    if disable_thin_source_pass:
        patcher = patch(
            "app.services.coverage_planner._source_thin_region_centerline_pass",
            return_value=([], {
                "thin_source_region_detection_ran": True,
                "thin_source_region_count": 0,
                "thin_centerline_candidate_count": 0,
                "thin_centerline_accepted_count": 0,
                "thin_centerline_total_length_mm": 0.0,
                "thin_centerline_paths_exported": False,
            }, []),
        )
        patcher.start()
    try:
        response = client.post(
            "/generate-image-gcode",
            data={
                "image": (BytesIO(fixture_bytes), "arsenal.png"),
                "selected_colors": f"[\"{selected}\"]",
                "line_thickness_mm": "0.6",
                "rotation_deg": "90",
                "debug_pipeline": "1",
            },
            content_type="multipart/form-data",
        )
    finally:
        if patcher is not None:
            patcher.stop()
    assert response.status_code == 200, response.get_data(as_text=True)[:1000]
    payload = response.get_json()
    assert payload["ok"] is True
    return payload


def _gcode_hash(gcode: list[str]) -> str:
    return hashlib.sha256("\n".join(gcode).encode("utf-8")).hexdigest()


def _path_signature(entries: list[dict[str, object]]) -> str:
    compact_rows: list[dict[str, object]] = []
    for entry in entries:
        pts = list(entry.get("points_surface_mm") or [])
        start = pts[0] if pts else None
        end = pts[-1] if pts else None
        compact_rows.append({
            "id": entry.get("id"),
            "kind": entry.get("kind"),
            "source": entry.get("source"),
            "count": len(pts),
            "start": None if start is None else [round(float(start["x"]), 3), round(float(start["y"]), 3)],
            "end": None if end is None else [round(float(end["x"]), 3), round(float(end["y"]), 3)],
        })
    return hashlib.sha256(json.dumps(compact_rows, sort_keys=True).encode("utf-8")).hexdigest()


def _path_length_mm(points: list[dict[str, float]]) -> float:
    return sum(
        math.hypot(float(points[index]["x"]) - float(points[index - 1]["x"]), float(points[index]["y"]) - float(points[index - 1]["y"]))
        for index in range(1, len(points))
    )


def test_arsenal_frontend_generate_exports_thin_centerlines_and_gcode_changes(client):
    baseline = _frontend_generate(client, disable_thin_source_pass=True)
    current = _frontend_generate(client, disable_thin_source_pass=False)

    baseline_debug = baseline["debug"]
    current_debug = current["debug"]
    baseline_export_paths = list(baseline_debug.get("final_export_paths") or [])
    current_export_paths = list(current_debug.get("final_export_paths") or [])
    current_thin_paths = [path for path in current_export_paths if path.get("source") == "thin_source_region_centerline"]
    current_thin_lengths = [_path_length_mm(list(path.get("points_surface_mm") or [])) for path in current_thin_paths]

    assert current_debug["frontend_generate_request_received"] is True
    assert current_debug["thin_source_region_pass_enabled"] is True
    assert current_debug["thin_source_region_pass_ran"] is True
    assert current_debug["thin_source_region_count"] >= 1
    assert current_debug["thin_centerline_candidate_count"] >= current_debug["thin_centerline_accepted_count"] > 0
    assert current_debug["thin_centerline_exported_count"] > 0
    assert current_debug["thin_centerline_total_length_mm"] > 5.0
    assert current_debug["final_path_count_after_thin_centerlines"] > current_debug["final_path_count_before_thin_centerlines"]
    assert current_debug["gcode_contains_thin_centerline_paths"] is True
    assert current_debug["gcode_path_count_by_kind"]["detail-trace"] > baseline_debug["gcode_path_count_by_kind"].get("detail-trace", 0)

    assert baseline_debug["thin_centerline_exported_count"] == 0
    assert baseline_debug["gcode_contains_thin_centerline_paths"] is False

    assert _gcode_hash(current["gcode"]) != _gcode_hash(baseline["gcode"])
    assert _path_signature(current_export_paths) != _path_signature(baseline_export_paths)
    assert current["preview"] != baseline["preview"]

    assert current_thin_paths
    assert max(current_thin_lengths) > 2.0
    assert sum(current_thin_lengths) > 5.0

    assert any("kind=detail-trace" in line and "source=thin_source_region_centerline" in line for line in current["gcode"])
    assert not all(length < 1.0 for length in current_thin_lengths)
