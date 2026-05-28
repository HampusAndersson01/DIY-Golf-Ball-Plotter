import pytest
from shapely.geometry import LineString, Polygon

from app.models.geometry import Point, Toolpath
from app.services import pipeline_core
from app.services.gcode_service import GcodeService


class _FakeSerial:
    def __init__(self, chunks):
        self._chunks = [chunk.encode("ascii") for chunk in chunks]
        self._buffer = b""

    @property
    def in_waiting(self):
        if self._buffer:
            return len(self._buffer)
        if self._chunks:
            self._buffer = self._chunks.pop(0)
            return len(self._buffer)
        return 0

    def read(self, size=1):
        if not self._buffer and self._chunks:
            self._buffer = self._chunks.pop(0)
        if not self._buffer:
            return b""
        data = self._buffer[:size]
        self._buffer = self._buffer[size:]
        return data

    def write(self, _payload):
        return 1


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


def test_detail_continuation_keeps_pen_down_between_detail_segments():
    service = GcodeService()
    toolpaths_with_continuation = [
        Toolpath(
            points=[Point(0.0, 0.0), Point(1.0, 0.0)],
            kind="detail-trace",
            closed=False,
            coordinate_space="machine_deg",
                metadata={"path_role": "PRINT_DETAIL", "projection_count": 1},
        ),
        Toolpath(
            points=[Point(1.0, 0.0), Point(1.4, 0.0)],
            kind="detail-continuation",
            closed=False,
            coordinate_space="machine_deg",
            metadata={"path_role": "PRINT_DETAIL_CONTINUATION", "detail_continuation_pen_down": True, "projection_count": 1},
        ),
        Toolpath(
            points=[Point(1.4, 0.0), Point(2.2, 0.0)],
            kind="detail-trace",
            closed=False,
            coordinate_space="machine_deg",
                metadata={"path_role": "PRINT_DETAIL", "projection_count": 1},
        ),
    ]
    toolpaths_with_continuation = pipeline_core.assign_stable_path_ids(toolpaths_with_continuation)
    gcode_with, _preview = service.generate_from_toolpaths(
        toolpaths=toolpaths_with_continuation,
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
        include_comments=True,
    )
    toolpaths_without_continuation = [
        pipeline_core.clone_toolpath(path, metadata={**path.metadata, "path_role": "PRINT_DETAIL"})
        for path in toolpaths_with_continuation
    ]
    gcode_without, _preview = service.generate_from_toolpaths(
        toolpaths=toolpaths_without_continuation,
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
        include_comments=True,
    )
    pen_lifts_with = sum(1 for line in gcode_with if line.strip().startswith("M3 S575"))
    pen_lifts_without = sum(1 for line in gcode_without if line.strip().startswith("M3 S575"))
    assert pen_lifts_with < pen_lifts_without


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


def test_preview_and_gcode_share_same_projected_paths_after_surface_anchor_placement():
    bundle = pipeline_core.GeometryBundle(
        outline_segments=[
            pipeline_core.Segment(
                points=[
                    Point(-10.0, -4.0),
                    Point(10.0, -4.0),
                    Point(10.0, 4.0),
                    Point(-10.0, 4.0),
                    Point(-10.0, -4.0),
                ],
                closed=True,
            )
        ],
    )
    placed = pipeline_core.apply_origin_anchor_placement(
        bundle,
        origin_anchor="bottom-left",
        origin_offset_x_mm=5.0,
        origin_offset_y_mm=2.0,
    )
    toolpaths = pipeline_core.generate_toolpaths(
        placed,
        enable_fill=False,
        line_width_mm=0.75,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=0.75,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )
    cleaned, _stats = pipeline_core.cleanup_surface_toolpaths(toolpaths, tolerance_mm=0.0, min_segment_length_mm=0.0)
    prepared = pipeline_core.prepare_toolpaths_for_projection(cleaned, default_pen_width_mm=0.75)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)

    assert all(path.coordinate_space == "machine_deg" for path in projected)
    assert all(int(path.metadata.get("projection_count", 0)) == 1 for path in projected)

    gcode, preview = GcodeService().generate_from_toolpaths(
        toolpaths=projected,
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
        include_comments=True,
    )

    projected_debug = pipeline_core.build_projected_path_debug(prepared, projected, preview)
    assert projected_debug["preview_and_gcode_share_same_projected_paths"] is True

    preview_toolpaths = [path for path in pipeline_core.preview_entries_to_toolpaths(preview) if path.kind != "travel"]
    gcode_toolpaths = [path for path in pipeline_core.parse_gcode_machine_motion_paths(gcode, pen_up_s=575, pen_down_s=700) if path.kind != "travel"]
    assert len(preview_toolpaths) == len(projected)
    assert len(gcode_toolpaths) == len(projected)
    assert preview_toolpaths[0].points[0].x == pytest.approx(projected[0].points[0].x, abs=1e-4)
    assert preview_toolpaths[0].points[0].y == pytest.approx(projected[0].points[0].y, abs=1e-4)
    assert gcode_toolpaths[0].points[0].x == pytest.approx(projected[0].points[0].x, abs=1e-4)
    assert gcode_toolpaths[0].points[0].y == pytest.approx(projected[0].points[0].y, abs=1e-4)


@pytest.mark.parametrize(
    ("line_width_mm", "infill_spacing_mm", "custom_spacing_enabled", "expected_spacing_mm"),
    [
        (0.2, None, False, 0.2),
        (0.6, None, False, 0.6),
        (0.6, 0.2, True, 0.2),
    ],
)
def test_geometry_spacing_metrics_follow_normalized_config(line_width_mm, infill_spacing_mm, custom_spacing_enabled, expected_spacing_mm):
    printable = Polygon([
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 10.0),
        (0.0, 10.0),
        (0.0, 0.0),
    ])
    toolpaths = pipeline_core.generate_toolpaths(
        pipeline_core.GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=line_width_mm,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=infill_spacing_mm if custom_spacing_enabled else line_width_mm,
        infill_angle_deg=0.0,
        outline_after_fill=True,
        min_fill_area_mm2=0.15,
        min_segment_length_mm=0.0,
        min_fill_width_mm=line_width_mm,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        fill_strategy="adaptive_angle",
        alternate_fill_angle_deg=-45.0,
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.05,
        thin_detail_simplify_mm=0.1,
        thin_detail_overlap=True,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=False,
        infill_path_mode="rectilinear",
    )
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=line_width_mm)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0, ball_diameter_mm=42.67)

    gcode_service = GcodeService()
    debug: dict[str, object] = {}
    gcode, preview = gcode_service.generate_from_toolpaths(
        toolpaths=projected,
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
        debug=debug,
    )

    normalized = pipeline_core.normalize_geometry_config(
        raw_line_width_mm=line_width_mm,
        raw_infill_spacing_mm=infill_spacing_mm if custom_spacing_enabled else None,
    )
    preview_toolpaths = [path for path in pipeline_core.preview_entries_to_toolpaths(preview) if path.kind != "travel"]
    gcode_toolpaths = [path for path in pipeline_core.parse_gcode_machine_motion_paths(gcode, pen_up_s=575, pen_down_s=700) if path.kind != "travel"]
    metrics = pipeline_core.build_geometry_spacing_metrics(
        projected,
        normalized_config=normalized,
        preview_toolpaths=preview_toolpaths,
        gcode_toolpaths=gcode_toolpaths,
    )

    assert metrics.lineWidthMm == pytest.approx(line_width_mm, abs=1e-9)
    assert metrics.previewStrokeWidthMm == pytest.approx(line_width_mm, abs=1e-9)
    assert metrics.effectiveInfillSpacingMm == pytest.approx(expected_spacing_mm, abs=1e-9)
    assert metrics.actualAverageInfillSpacingMm == pytest.approx(expected_spacing_mm, abs=0.05)
    assert metrics.actualMaxInfillSpacingMm == pytest.approx(expected_spacing_mm, abs=0.05)
    assert metrics.estimatedUncoveredGapMm <= 0.05
    assert metrics.previewGcodePathMismatchCount == 0
    assert debug.get("preview_gcode_path_mismatch_count") == 0
    assert debug.get("preview_and_gcode_share_same_projected_paths") is True


def test_contour_detail_spacing_uses_line_width_as_the_default_detail_spacing():
    centerline = LineString([
        (1.0, 8.5),
        (3.0, 9.2),
        (5.5, 8.3),
        (7.2, 6.7),
        (5.8, 5.0),
        (3.1, 4.1),
        (1.6, 2.4),
        (3.5, 0.8),
        (6.8, 1.0),
    ])
    printable = centerline.buffer(0.55, join_style=1, cap_style=1)
    toolpaths = pipeline_core.generate_toolpaths(
        pipeline_core.GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=0.6,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=0.2,
        infill_angle_deg=45.0,
        outline_after_fill=True,
        min_fill_area_mm2=0.15,
        min_segment_length_mm=0.0,
        min_fill_width_mm=0.6,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        fill_strategy="adaptive_angle",
        alternate_fill_angle_deg=-45.0,
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.05,
        thin_detail_simplify_mm=0.1,
        thin_detail_overlap=True,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=False,
        infill_path_mode="rectilinear",
    )
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert any(path.metadata.get("small_detail_fill_style") == "contour_following" for path in infill_paths)

    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.6)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0, ball_diameter_mm=42.67)
    gcode_service = GcodeService()
    debug: dict[str, object] = {}
    gcode, preview = gcode_service.generate_from_toolpaths(
        toolpaths=projected,
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
        debug=debug,
    )

    normalized = pipeline_core.normalize_geometry_config(raw_line_width_mm=0.6, raw_infill_spacing_mm=0.2)
    preview_toolpaths = [path for path in pipeline_core.preview_entries_to_toolpaths(preview) if path.kind != "travel"]
    gcode_toolpaths = [path for path in pipeline_core.parse_gcode_machine_motion_paths(gcode, pen_up_s=575, pen_down_s=700) if path.kind != "travel"]
    metrics = pipeline_core.build_geometry_spacing_metrics(
        projected,
        normalized_config=normalized,
        preview_toolpaths=preview_toolpaths,
        gcode_toolpaths=gcode_toolpaths,
    )

    assert metrics.effectiveDetailSpacingMm == pytest.approx(0.6, abs=1e-9)
    assert metrics.actualAverageDetailOffsetSpacingMm == pytest.approx(0.6, abs=0.15)
    assert metrics.actualMaxDetailOffsetSpacingMm == pytest.approx(0.6, abs=0.15)
    assert metrics.previewGcodePathMismatchCount == 0
    assert debug.get("preview_gcode_path_mismatch_count") == 0


def test_read_next_grbl_line_reassembles_fragmented_ok_response():
    ser = _FakeSerial(["o", "k"])
    assert pipeline_core.read_next_grbl_line(ser, timeout=0.2) == "ok"


def test_read_next_grbl_line_reads_status_without_newline():
    ser = _FakeSerial(["<Idle|WPos:1.000,2.000,0.000|FS:0,0>"])
    assert pipeline_core.read_next_grbl_line(ser, timeout=0.2).startswith("<Idle|WPos:1.000,2.000,0.000")


def test_read_next_grbl_line_does_not_use_readline_when_in_waiting_exists():
    class _NonblockingSerial:
        @property
        def in_waiting(self):
            return 0

        def read(self, size=1):
            return b""

        def readline(self):
            raise AssertionError("readline fallback should not be used for pyserial-like objects")

        def write(self, _payload):
            return 1

    assert pipeline_core.read_next_grbl_line(_NonblockingSerial(), timeout=0.02, raise_on_timeout=False) == ""
