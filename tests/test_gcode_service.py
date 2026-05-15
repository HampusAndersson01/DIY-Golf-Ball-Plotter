from app.models.geometry import Point, Toolpath
from app.services import pipeline_core
from app.services.gcode_service import GcodeService


def test_generate_gcode_from_simple_toolpath():
    service = GcodeService()
    toolpaths = [Toolpath(points=[Point(0.0, 0.0), Point(1.0, 1.0)], kind="outline", closed=False)]
    gcode, preview = service.generate_from_toolpaths(
        toolpaths=toolpaths,
        draw_feed=1200.0,
        travel_feed=3000.0,
        sample_step_deg=1.0,
        pen_up_s=575,
        pen_down_s=700,
        servo_ramp_enabled=True,
        servo_ramp_step=20,
        servo_ramp_delay_ms=10.0,
        pen_up_dwell_ms=30.0,
        pen_down_dwell_ms=60.0,
        gcode_mode="simple",
        include_comments=True,
    )
    assert any(line.startswith("G1 X1.0000 Y1.0000") for line in gcode)
    assert preview[0]["kind"] == "outline"


def test_merge_connected_toolpaths_collapses_touching_fragments():
    toolpaths = [
        Toolpath(points=[Point(0.0, 0.0), Point(1.0, 0.0)], kind="detail-trace", closed=False),
        Toolpath(points=[Point(1.0, 0.0), Point(2.0, 0.0)], kind="detail-trace", closed=False),
        Toolpath(points=[Point(2.0, 0.0), Point(3.0, 0.0)], kind="detail-trace", closed=False),
    ]

    merged = pipeline_core.merge_connected_toolpaths(toolpaths)
    assert len(merged) == 1
    assert [(point.x, point.y) for point in merged[0].points] == [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
    ]


def test_summarize_toolpaths_reports_required_diagnostics():
    toolpaths = [
        Toolpath(points=[Point(0.0, 0.0), Point(1.0, 0.0)], kind="detail-trace", closed=False),
        Toolpath(points=[Point(1.0, 0.0), Point(2.0, 0.0), Point(3.0, 0.0)], kind="fill-infill", closed=False),
    ]

    summary = pipeline_core.summarize_toolpaths(toolpaths)
    assert summary["total_toolpaths"] == 2
    assert summary["one_move_toolpaths"] == 1
    assert summary["paths_by_kind"] == {"detail-trace": 1, "fill-infill": 1}
    assert summary["points_by_kind"] == {"detail-trace": 2, "fill-infill": 3}


def test_projected_gcode_includes_resolved_fill_header_comments():
    service = GcodeService()
    toolpaths = [Toolpath(points=[Point(0.0, 0.0), Point(5.0, 0.0)], kind="fill-infill", closed=False)]

    gcode, _ = service.generate_from_toolpaths(
        toolpaths=toolpaths,
        draw_feed=1200.0,
        travel_feed=3000.0,
        sample_step_deg=1.0,
        placement_offset_x=0.0,
        placement_offset_y=0.0,
        pen_up_s=575,
        pen_down_s=700,
        servo_ramp_enabled=True,
        servo_ramp_step=20,
        servo_ramp_delay_ms=10.0,
        pen_up_dwell_ms=30.0,
        pen_down_dwell_ms=60.0,
        gcode_mode="simple",
        include_comments=False,
        header_comment_settings={
            "lineWidthMm": "0.1500",
            "infillSpacingMm": "0.1500",
            "wallCount": 1,
            "infillAngle": "0.0000",
            "rotationDeg": "0.0000",
            "designWidthMm": "10.0000",
            "designHeightMm": "5.0000",
            "coordinateSpaceUsedForFill": "surface-mm-on-ball",
        },
    )

    header_lines = [line for line in gcode if line.startswith("(")]
    assert any("lineWidthMm: 0.1500" in line for line in header_lines)
    assert any("infillSpacingMm: 0.1500" in line for line in header_lines)
    assert any("coordinateSpaceUsedForFill: surface-mm-on-ball" in line for line in header_lines)
