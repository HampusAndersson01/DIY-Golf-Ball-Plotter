from io import BytesIO
from pathlib import Path

import pytest

from app import create_app
from app.models.machine_state import MachineState
from app.services.pipeline_core import Point, Toolpath
from app.services.raster_analysis_service import RasterAnalysisService


ROOT = Path(__file__).resolve().parents[1]
ARSENAL_FIXTURE = ROOT / "tests" / "fixtures" / "images" / "black-arsenal-logo-png-1.png"


@pytest.fixture()
def client():
    app = create_app()
    app.config.update(TESTING=True)
    with app.test_client() as test_client:
        yield test_client


def test_generate_image_route_uses_expensive_coverage_repair_by_default(client):
    fixture_bytes = ARSENAL_FIXTURE.read_bytes()
    app = client.application
    raster = RasterAnalysisService(app.config, MachineState(default_pen_up_s=575))
    analysis = raster.analyze_image(fixture_bytes, max_colors=app.config["DEFAULT_RASTER_MAX_COLORS"])
    selected = next(
        (color.id for color in analysis.colors if color.hex == "#000000"),
        analysis.colors[0].id if analysis.colors else None,
    )
    assert selected is not None

    captured: dict[str, object] = {}

    def fake_generate_from_regions(_regions, **kwargs):
        captured.update(kwargs)
        return [
            Toolpath(
                points=[Point(0.0, 0.0), Point(2.0, 0.0)],
                kind="fill-infill",
                closed=False,
                source="test-route",
                metadata={"path_role": "TEST_ROUTE"},
            )
        ]

    app.extensions["toolpath_service"].generate_from_regions = fake_generate_from_regions

    response = client.post(
        "/generate-image-gcode",
        data={
            "image": (BytesIO(fixture_bytes), "arsenal.png"),
            "selected_colors": f"[\"{selected}\"]",
            "line_thickness_mm": "0.6",
            "rotation_deg": "90",
        },
        content_type="multipart/form-data",
    )

    assert response.status_code == 200
    assert captured.get("expensive_coverage_repair", True) is False


def test_generate_diagnostic_route_returns_calibration_metadata_for_3x3_pattern(client):
    response = client.post(
        "/generate-diagnostic-gcode",
        data={
            "pattern": "3x3_squares",
            "line_thickness_mm": "0.75",
            "draw_feed": "1200",
            "travel_feed": "3000",
            "wall_count": "1",
        },
    )

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["ok"] is True
    calibration = payload["calibrationPattern"]
    assert calibration["pattern"] == "3x3_squares"
    assert len(calibration["squares"]) == 9
    assert all(square["surfaceMmBbox"]["width"] == pytest.approx(4.5, abs=1e-6) for square in calibration["squares"])
    assert all(square["surfaceMmBbox"]["height"] == pytest.approx(4.5, abs=1e-6) for square in calibration["squares"])
    assert all(square["machineDegreeBbox"] is not None for square in calibration["squares"])
    assert all(square["gcodeBbox"] is not None for square in calibration["squares"])
    assert calibration["previewAndGcodeShareSameProjectedPaths"] is True


def test_calibration_metadata_square_centers_follow_top_middle_bottom_labels(client):
    response = client.post("/generate-diagnostic-gcode", data={"pattern": "3x3_squares"})

    assert response.status_code == 200
    calibration = response.get_json()["calibrationPattern"]
    squares = {square["id"]: square for square in calibration["squares"]}

    assert squares["top-left"]["expectedSurfaceCenterMm"]["y"] > squares["middle-left"]["expectedSurfaceCenterMm"]["y"]
    assert squares["middle-left"]["expectedSurfaceCenterMm"]["y"] > squares["bottom-left"]["expectedSurfaceCenterMm"]["y"]
    assert squares["top-left"]["expectedSurfaceCenterMm"]["x"] < squares["top-center"]["expectedSurfaceCenterMm"]["x"]
    assert squares["top-center"]["expectedSurfaceCenterMm"]["x"] < squares["top-right"]["expectedSurfaceCenterMm"]["x"]


def test_generate_diagnostic_route_returns_x_axis_rotation_calibration_pattern(client):
    response = client.post("/generate-diagnostic-gcode", data={"pattern": "x_axis_rotation_ticks"})

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["ok"] is True
    pattern = payload["xAxisCalibrationPattern"]
    assert pattern["pattern"] == "x_axis_rotation_ticks"
    assert len(pattern["ticks"]) == 5
    assert pattern["ticks"][0]["commandedXDeg"] == pytest.approx(0.0)
    assert pattern["ticks"][1]["commandedXDeg"] == pytest.approx(90.0)
    assert pattern["ticks"][2]["commandedXDeg"] == pytest.approx(180.0)
    assert pattern["ticks"][3]["commandedXDeg"] == pytest.approx(270.0)
    assert pattern["ticks"][4]["commandedXDeg"] == pytest.approx(360.0)
    assert pattern["ticks"][0]["emittedMachineXDeg"] == pytest.approx(0.0)
    assert pattern["ticks"][4]["emittedMachineXDeg"] == pytest.approx(0.0)
    assert pattern["expectedQuadrantArcMm"] == pytest.approx(pattern["ballCircumferenceMm"] / 4.0, abs=1e-9)
    assert all(tick["gcodeMatchesMachineDegreeBbox"] is True for tick in pattern["ticks"])
