import pytest

from app import create_app


@pytest.fixture()
def client():
    app = create_app()
    app.config.update(TESTING=True)
    with app.test_client() as test_client:
        yield test_client


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
