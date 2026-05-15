from __future__ import annotations

from dataclasses import asdict

from . import pipeline_core


class GcodeService:
    build_pen_position_commands = staticmethod(pipeline_core.build_pen_position_commands)
    is_streamable_line = staticmethod(pipeline_core.is_streamable_gcode_line)

    def _generate_from_angle_toolpaths_legacy(self, **kwargs):
        toolpaths = kwargs["toolpaths"]
        draw_feed = kwargs["draw_feed"]
        travel_feed = kwargs["travel_feed"]
        pen_up_s = kwargs["pen_up_s"]
        pen_down_s = kwargs["pen_down_s"]
        servo_ramp_enabled = kwargs["servo_ramp_enabled"]
        servo_ramp_step = kwargs["servo_ramp_step"]
        servo_ramp_delay_ms = kwargs["servo_ramp_delay_ms"]
        pen_up_dwell_ms = kwargs["pen_up_dwell_ms"]
        pen_down_dwell_ms = kwargs["pen_down_dwell_ms"]
        include_comments = kwargs["include_comments"]

        gcode: list[str] = []
        preview: list[dict] = []
        current_servo = pen_up_s
        current_position = None
        current_pen_down = False

        def comment(text: str) -> None:
            if include_comments:
                gcode.append(f"({text})")

        comment("Generated for golf ball plotter")
        comment("Units are angular degrees. X=-180..180 ball rotation, Y=-45..45 arm tilt")
        gcode.extend(["$X", "G21", "G90"])
        gcode.extend(self.build_pen_position_commands(
            pen_up_s,
            pen_up_s,
            ramp_enabled=False,
            ramp_step=servo_ramp_step,
            ramp_delay_ms=servo_ramp_delay_ms,
            dwell_ms=pen_up_dwell_ms,
        ))

        for index, toolpath in enumerate(toolpaths, start=1):
            pts = list(toolpath.points)
            if len(pts) < 2:
                continue
            start = pts[0]
            if current_position is not None and not pipeline_core.nearly_same_point(current_position, start):
                preview.append({"kind": "travel", "closed": False, "points": [asdict(current_position), asdict(start)]})
                if current_pen_down:
                    gcode.extend(self.build_pen_position_commands(
                        current_servo,
                        pen_up_s,
                        ramp_enabled=servo_ramp_enabled,
                        ramp_step=servo_ramp_step,
                        ramp_delay_ms=servo_ramp_delay_ms,
                        dwell_ms=pen_up_dwell_ms,
                    ))
                    current_servo = pen_up_s
                    current_pen_down = False
                comment(f"Travel to {toolpath.kind} path {index}")
                gcode.append(f"G1 X{start.x:.4f} Y{start.y:.4f} F{travel_feed:.3f}")
            if not current_pen_down:
                gcode.extend(self.build_pen_position_commands(
                    current_servo,
                    pen_down_s,
                    ramp_enabled=servo_ramp_enabled,
                    ramp_step=servo_ramp_step,
                    ramp_delay_ms=servo_ramp_delay_ms,
                    dwell_ms=pen_down_dwell_ms,
                ))
                current_servo = pen_down_s
                current_pen_down = True
            preview.append({"kind": toolpath.kind, "closed": toolpath.closed, "points": [asdict(point) for point in pts]})
            comment(f"{toolpath.kind} path {index}, {len(pts)} points")
            for point in pts[1:]:
                gcode.append(f"G1 X{point.x:.4f} Y{point.y:.4f} F{draw_feed:.3f}")
                current_position = point
            gcode.extend(self.build_pen_position_commands(
                current_servo,
                pen_up_s,
                ramp_enabled=servo_ramp_enabled,
                ramp_step=servo_ramp_step,
                ramp_delay_ms=servo_ramp_delay_ms,
                dwell_ms=pen_up_dwell_ms,
            ))
            current_servo = pen_up_s
            current_pen_down = False

        if current_position is not None and not pipeline_core.nearly_same_point(current_position, pipeline_core.Point(0.0, 0.0)):
            comment("Return to zero with pen up")
            preview.append({"kind": "travel", "closed": False, "points": [asdict(current_position), asdict(pipeline_core.Point(0.0, 0.0))]})
            gcode.append(f"G1 X0.0000 Y0.0000 F{travel_feed:.3f}")
        gcode.extend(self.build_pen_position_commands(
            current_servo,
            pen_up_s,
            ramp_enabled=False,
            ramp_step=servo_ramp_step,
            ramp_delay_ms=servo_ramp_delay_ms,
            dwell_ms=pen_up_dwell_ms,
        ))
        return gcode, preview

    def generate_from_toolpaths(self, **kwargs):
        if "placement_offset_x" not in kwargs and "placement_offset_y" not in kwargs:
            return self._generate_from_angle_toolpaths_legacy(**kwargs)
        return pipeline_core.generate_gcode_from_toolpaths(
            kwargs["toolpaths"],
            kwargs["draw_feed"],
            kwargs["travel_feed"],
            kwargs["sample_step_deg"],
            kwargs.get("placement_offset_x", 0.0),
            kwargs.get("placement_offset_y", 0.0),
            kwargs["pen_up_s"],
            kwargs["pen_down_s"],
            kwargs["servo_ramp_enabled"],
            kwargs["servo_ramp_step"],
            kwargs["servo_ramp_delay_ms"],
            kwargs["pen_up_dwell_ms"],
            kwargs["pen_down_dwell_ms"],
            kwargs["gcode_mode"],
            kwargs["include_comments"],
        )
