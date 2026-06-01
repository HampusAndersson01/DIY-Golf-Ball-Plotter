import pytest

from app.services.validation_service import ValidationService


def make_config():
    return {
        "DEFAULT_DRAW_FEED": 1200.0,
        "DEFAULT_TRAVEL_FEED": 3000.0,
        "DEFAULT_SAMPLE_STEP_DEG": 1.0,
        "DEFAULT_MAX_PRINT_X_SPAN_DEG": 120.0,
        "DEFAULT_MARGIN_PERCENT": 4.0,
        "DEFAULT_ROTATION_DEG": 0.0,
        "DEFAULT_PARSER_MODE": "visible_geometry",
        "DEFAULT_COLOR_MAPPING_MODE": False,
        "DEFAULT_LINE_THICKNESS_MM": 0.75,
        "DEFAULT_ENABLE_FILL": True,
        "DEFAULT_TRACE_STROKE_ONLY_PATHS": True,
        "DEFAULT_FILL_ONLY_DARK_SVG_FILLS": True,
        "DEFAULT_FILL_MODE": "slicer",
        "DEFAULT_WALL_COUNT": 1,
        "DEFAULT_INFILL_PATTERN": "zigzag",
        "DEFAULT_INFILL_DENSITY": 100.0,
        "DEFAULT_INFILL_SPACING_MM": 0.75,
        "DEFAULT_INFILL_ANGLE_DEG": 0.0,
        "DEFAULT_OUTLINE_AFTER_FILL": False,
        "DEFAULT_MIN_FILL_AREA_MM2": 1.0,
        "DEFAULT_MIN_FILL_WIDTH_MM": 0.75,
        "DEFAULT_SIMPLIFY_TOLERANCE_MM": 0.05,
        "DEFAULT_REMOVE_DUPLICATE_PATHS": True,
        "DEFAULT_SMALL_SHAPE_MODE": "single-wall",
        "DEFAULT_THIN_DETAIL_MODE": True,
        "DEFAULT_THIN_DETAIL_MIN_AREA_MM2": 0.05,
        "DEFAULT_THIN_DETAIL_SIMPLIFY_MM": 0.1,
        "DEFAULT_THIN_DETAIL_OVERLAP": True,
        "DEFAULT_MIN_SEGMENT_LENGTH_MM": 0.5,
        "DEFAULT_TRAVEL_OPTIMIZATION": "nearest-neighbor",
        "DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS": True,
        "DEFAULT_INFILL_PATH_MODE": "rectilinear",
        "DEFAULT_RASTER_MAX_COLORS": 8,
        "DEFAULT_RASTER_COLOR_TOLERANCE": 24,
        "DEFAULT_RASTER_MIN_COMPONENT_AREA_PX": 8,
        "DEFAULT_RASTER_MASK_OPEN_RADIUS_PX": 0,
        "DEFAULT_RASTER_MASK_CLOSE_RADIUS_PX": 1,
        "DEFAULT_RASTER_MIN_REGION_AREA_PX": 16.0,
        "DEFAULT_RASTER_REGION_SIMPLIFY_PX": 1.0,
        "DEFAULT_PEN_UP_S": 575,
        "DEFAULT_PEN_DOWN_S": 700,
        "DEFAULT_SERVO_RAMP_ENABLED": True,
        "DEFAULT_SERVO_RAMP_STEP": 20,
        "DEFAULT_SERVO_RAMP_DELAY_MS": 10.0,
        "DEFAULT_PEN_UP_DWELL_MS": 30.0,
        "DEFAULT_PEN_DOWN_DWELL_MS": 60.0,
        "DEFAULT_GCODE_MODE": "simple",
    }


def test_invalid_margin_percent_raises():
    service = ValidationService()
    with pytest.raises(ValueError, match="Margin percent must be between 0 and 25"):
        service.parse_generate_gcode_form({"margin_percent": "30"}, make_config())


def test_invalid_servo_value_raises():
    service = ValidationService()
    with pytest.raises(ValueError):
        service.validate_servo_s(1001)


def test_generate_raster_form_requires_selected_colors():
    service = ValidationService()
    with pytest.raises(ValueError, match="Select at least one color to print"):
        service.parse_generate_raster_form({}, make_config())


def test_generate_raster_form_derives_fill_defaults_from_pen_thickness():
    service = ValidationService()
    options = service.parse_generate_raster_form(
        {
            "selected_colors": "[\"#000000\"]",
            "line_thickness_mm": "0.6",
            "infill_spacing_mm": "",
            "min_fill_width_mm": "",
            "min_fill_area_mm2": "",
        },
        make_config(),
    )

    assert options["infill_spacing_mm"] == pytest.approx(0.48)
    assert options["infill_overlap_percent"] == pytest.approx(20.0)
    assert options["min_fill_width_mm"] == pytest.approx(0.6)
    assert options["min_fill_area_mm2"] == pytest.approx(0.36)


def test_generate_gcode_form_accepts_printable_x_span_override_toggle():
    service = ValidationService()
    options = service.parse_generate_gcode_form(
        {
            "selected_colors": "[\"#000000\"]",
            "ignore_printable_x_span_limit": "1",
        },
        make_config(),
    )

    assert options["ignore_printable_x_span_limit"] is True


def test_generate_raster_form_accepts_printable_x_span_override_toggle():
    service = ValidationService()
    options = service.parse_generate_raster_form(
        {
            "selected_colors": "[\"#000000\"]",
            "ignore_printable_x_span_limit": "1",
        },
        make_config(),
    )

    assert options["ignore_printable_x_span_limit"] is True


def test_generate_raster_form_ignores_stale_infill_spacing_when_custom_spacing_is_disabled():
    service = ValidationService()
    options = service.parse_generate_raster_form(
        {
            "selected_colors": "[\"#000000\"]",
            "line_thickness_mm": "0.3",
            "infill_spacing_mm": "0.75",
            "custom_infill_spacing": "0",
        },
        make_config(),
    )

    assert options["custom_infill_spacing"] is False
    assert options["infill_spacing_mm"] == pytest.approx(0.24)
    assert options["effective_infill_spacing_mm"] == pytest.approx(0.24)


def test_generate_raster_form_supports_locale_decimals_and_custom_infill_spacing():
    service = ValidationService()
    options = service.parse_generate_raster_form(
        {
            "selected_colors": "[\"#000000\"]",
            "line_thickness_mm": "0,5",
            "infill_spacing_mm": "0,8",
            "custom_infill_spacing": "1",
        },
        make_config(),
    )

    assert options["custom_infill_spacing"] is True
    assert options["line_thickness_mm"] == pytest.approx(0.5)
    assert options["infill_spacing_mm"] == pytest.approx(0.8)
    assert options["effective_infill_spacing_mm"] == pytest.approx(0.8)


def test_generate_raster_form_clamps_artwork_scale_percent():
    service = ValidationService()
    options = service.parse_generate_raster_form(
        {
            "selected_colors": "[\"#000000\"]",
            "artwork_scale_percent": "250",
        },
        make_config(),
    )

    assert options["artwork_scale_percent"] == pytest.approx(200.0)
    assert options["origin_anchor"] == "center"
    assert options["origin_offset_x_mm"] == pytest.approx(0.0)
    assert options["origin_offset_y_mm"] == pytest.approx(0.0)


def test_generate_raster_form_rejects_non_positive_artwork_scale_percent():
    service = ValidationService()
    with pytest.raises(ValueError, match="Artwork scale percent must be greater than 0"):
        service.parse_generate_raster_form(
            {
                "selected_colors": "[\"#000000\"]",
                "artwork_scale_percent": "0",
            },
            make_config(),
        )


def test_generate_raster_form_accepts_supported_origin_anchor_and_offsets():
    service = ValidationService()
    options = service.parse_generate_raster_form(
        {
            "selected_colors": "[\"#000000\"]",
            "origin_anchor": "bottom-left",
            "origin_offset_x_mm": "5.25",
            "origin_offset_y_mm": "-2,5",
        },
        make_config(),
    )

    assert options["origin_anchor"] == "bottom-left"
    assert options["origin_offset_x_mm"] == pytest.approx(5.25)
    assert options["origin_offset_y_mm"] == pytest.approx(-2.5)


def test_generate_raster_form_rejects_invalid_origin_anchor():
    service = ValidationService()
    with pytest.raises(ValueError, match="Invalid origin anchor"):
        service.parse_generate_raster_form(
            {
                "selected_colors": "[\"#000000\"]",
                "origin_anchor": "left",
            },
            make_config(),
        )


def test_generate_raster_form_rejects_invalid_infill_path_mode():
    service = ValidationService()
    with pytest.raises(ValueError, match="Invalid infill path mode"):
        service.parse_generate_raster_form(
            {
                "selected_colors": "[\"#000000\"]",
                "infill_path_mode": "unsupported",
            },
            make_config(),
        )


@pytest.mark.parametrize("field_name", ["origin_offset_x_mm", "origin_offset_y_mm"])
def test_generate_raster_form_rejects_non_finite_origin_offsets(field_name: str):
    service = ValidationService()
    with pytest.raises(ValueError, match="must be a finite number"):
        service.parse_generate_raster_form(
            {
                "selected_colors": "[\"#000000\"]",
                field_name: "Infinity",
            },
            make_config(),
        )
