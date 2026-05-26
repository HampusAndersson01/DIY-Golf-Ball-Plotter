from __future__ import annotations

from dataclasses import asdict
import logging

from . import pipeline_core


class GcodeService:
    logger = logging.getLogger(__name__)
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
            is_pen_down_travel = toolpath.kind == "fill-infill-travel"
            if is_pen_down_travel:
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
                preview.append({"kind": toolpath.kind, "closed": toolpath.closed, "pen_down": True, "travel_mode": "pen_down", "points": [asdict(point) for point in pts]})
                comment(f"Internal travel to {toolpath.kind} path {index}")
                for point in pts[1:]:
                    gcode.append(f"G1 X{point.x:.4f} Y{point.y:.4f} F{travel_feed:.3f}")
                    current_position = point
                continue
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
            next_toolpath = toolpaths[index] if index < len(toolpaths) else None
            if next_toolpath is not None and getattr(next_toolpath, "kind", None) == "fill-infill-travel":
                comment(f"PATH_END id={index} (keeping pen down for connector)")
            else:
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
        toolpaths = kwargs["toolpaths"]
        self.logger.info(
            "Generating G-code from %d toolpaths (mode=%s include_comments=%s placement=(%s,%s))",
            len(toolpaths),
            kwargs.get("gcode_mode"),
            kwargs.get("include_comments"),
            kwargs.get("placement_offset_x"),
            kwargs.get("placement_offset_y"),
        )
        if "placement_offset_x" in kwargs or "placement_offset_y" in kwargs:
            if toolpaths and getattr(toolpaths[0], "coordinate_space", "surface_mm") != "machine_deg":
                toolpaths = pipeline_core.prepare_toolpaths_for_projection(toolpaths)
                toolpaths = pipeline_core.project_toolpaths_to_ball_angles(
                    toolpaths,
                    center_lon_deg=kwargs.get("placement_offset_x", 0.0),
                    center_lat_deg=kwargs.get("placement_offset_y", 0.0),
                )
        elif "placement_offset_x" not in kwargs and "placement_offset_y" not in kwargs:
            gcode, preview = self._generate_from_angle_toolpaths_legacy(**kwargs)
            debug = kwargs.get("debug")
            if isinstance(debug, dict):
                try:
                    from .runtime_estimation_service import estimate_gcode_runtime

                    runtime_estimate = estimate_gcode_runtime(
                        gcode,
                        draw_feed=float(kwargs["draw_feed"]),
                        travel_feed=float(kwargs["travel_feed"]),
                        pen_up_s=int(kwargs["pen_up_s"]),
                        pen_down_s=int(kwargs["pen_down_s"]),
                    )
                    debug["estimated_runtime_seconds"] = runtime_estimate.estimated_runtime_seconds
                    debug["estimated_runtime_breakdown"] = runtime_estimate.as_dict()
                except Exception as exc:  # pragma: no cover - diagnostics only
                    self.logger.debug("Unable to build runtime estimate: %s", exc)
            self.logger.info("Generated legacy angle G-code lines=%d preview_paths=%d", len(gcode), len(preview))
            return gcode, preview
        gcode, preview = pipeline_core.generate_gcode_from_toolpaths(
            toolpaths,
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
            kwargs.get("header_comment_settings"),
            kwargs.get("debug"),
        )
        debug = kwargs.get("debug")
        if isinstance(debug, dict):
            try:
                from .runtime_estimation_service import estimate_gcode_runtime

                runtime_estimate = estimate_gcode_runtime(
                    gcode,
                    draw_feed=float(kwargs["draw_feed"]),
                    travel_feed=float(kwargs["travel_feed"]),
                    pen_up_s=int(kwargs["pen_up_s"]),
                    pen_down_s=int(kwargs["pen_down_s"]),
                )
                debug["estimated_runtime_seconds"] = runtime_estimate.estimated_runtime_seconds
                debug["estimated_runtime_breakdown"] = runtime_estimate.as_dict()
            except Exception as exc:  # pragma: no cover - diagnostics only
                self.logger.debug("Unable to build runtime estimate: %s", exc)
        self.logger.info("Generated projected G-code lines=%d preview_paths=%d", len(gcode), len(preview))
        return gcode, preview
