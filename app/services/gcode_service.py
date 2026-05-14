from __future__ import annotations

from ._legacy import legacy


class GcodeService:
    build_pen_position_commands = staticmethod(legacy.build_pen_position_commands)
    is_streamable_line = staticmethod(legacy.is_streamable_gcode_line)

    def generate_from_toolpaths(self, **kwargs):
        return legacy.generate_gcode_from_toolpaths(
            kwargs["toolpaths"],
            kwargs["draw_feed"],
            kwargs["travel_feed"],
            kwargs["sample_step_deg"],
            kwargs["pen_up_s"],
            kwargs["pen_down_s"],
            kwargs["servo_ramp_enabled"],
            kwargs["servo_ramp_step"],
            kwargs["servo_ramp_delay_ms"],
            kwargs["pen_up_dwell_ms"],
            kwargs["pen_down_dwell_ms"],
            kwargs["include_comments"],
        )
