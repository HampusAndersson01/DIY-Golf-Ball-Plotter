from app.models.geometry import Point, Toolpath
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
        include_comments=True,
    )
    assert any(line.startswith("G1 X1.0000 Y1.0000") for line in gcode)
    assert preview[0]["kind"] == "outline"
