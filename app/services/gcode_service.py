from __future__ import annotations

from dataclasses import asdict
import logging
import json
import tempfile
from pathlib import Path
from collections import Counter

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
                rounded_points = [asdict(pipeline_core._rounded_gcode_point(point)) for point in pts]
                preview.append({
                    "id": toolpath.path_id,
                    "kind": toolpath.kind,
                    "closed": toolpath.closed,
                    "region_id": toolpath.region_id,
                    "source": toolpath.source,
                    "coordinate_space": toolpath.coordinate_space,
                    "pen_down": True,
                    "travel_mode": "pen_down",
                    "points": rounded_points,
                })
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
            rounded_points = [asdict(pipeline_core._rounded_gcode_point(point)) for point in pts]
            preview.append({
                "id": toolpath.path_id,
                "kind": toolpath.kind,
                "closed": toolpath.closed,
                "region_id": toolpath.region_id,
                "source": toolpath.source,
                "coordinate_space": toolpath.coordinate_space,
                "points": rounded_points,
            })
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
        debug = kwargs.get("debug")
        surface_toolpaths_before_export = None

        def _write_travel_debug_files(gcode: list[str], preview: list[dict[str, object]]) -> None:
            if not isinstance(debug, dict):
                return
            parity = pipeline_core.build_preview_gcode_travel_parity_debug(
                preview=preview,
                gcode=gcode,
                pen_up_s=int(kwargs["pen_up_s"]),
                pen_down_s=int(kwargs["pen_down_s"]),
                final_export_paths=list(debug.get("final_export_paths", [])) if isinstance(debug.get("final_export_paths"), list) else None,
                pen_state_debug=list(debug.get("pen_state_debug", [])) if isinstance(debug.get("pen_state_debug"), list) else None,
            )
            debug.update(parity)
            artifact_dir = Path(str(debug.get("coverage_debug_artifact_dir") or (Path(tempfile.gettempdir()) / "golfball_plotter_coverage_debug")))
            artifact_dir.mkdir(parents=True, exist_ok=True)
            (artifact_dir / "preview_travel_debug.json").write_text(json.dumps(parity["preview_travel_debug"], indent=2), encoding="utf-8")
            (artifact_dir / "gcode_travel_debug.json").write_text(json.dumps(parity["gcode_travel_debug"], indent=2), encoding="utf-8")

        def _update_path_debug(gcode: list[str], preview: list[dict[str, object]]) -> None:
            if not isinstance(debug, dict):
                return
            projected_debug = pipeline_core.build_projected_path_debug(toolpaths, toolpaths, preview)
            debug["preview_and_gcode_share_same_projected_paths"] = bool(projected_debug.get("preview_and_gcode_share_same_projected_paths", False))
            debug["preview_gcode_path_mismatch_count"] = 0 if debug["preview_and_gcode_share_same_projected_paths"] else 1
            _write_travel_debug_files(gcode, preview)

        def _update_gcode_export_summary(gcode: list[str]) -> None:
            if not isinstance(debug, dict):
                return
            path_count_by_kind: Counter[str] = Counter()
            thin_centerline_count = 0
            for line in gcode:
                if not line.startswith("(PATH_START"):
                    continue
                parts = line.strip("()").split()
                fields: dict[str, str] = {}
                for part in parts[1:]:
                    if "=" not in part:
                        continue
                    key, value = part.split("=", 1)
                    fields[key] = value
                kind = str(fields.get("kind", "unknown"))
                source = str(fields.get("source", ""))
                path_count_by_kind[kind] += 1
                if source == "thin_source_region_centerline":
                    thin_centerline_count += 1
            debug["gcode_path_count_by_kind"] = dict(path_count_by_kind)
            debug["gcode_contains_thin_centerline_paths"] = bool(thin_centerline_count > 0)
            debug["thin_centerline_exported_count"] = int(thin_centerline_count)

        self.logger.info(
            "Generating G-code from %d toolpaths (mode=%s include_comments=%s placement=(%s,%s))",
            len(toolpaths),
            kwargs.get("gcode_mode"),
            kwargs.get("include_comments"),
            kwargs.get("placement_offset_x"),
            kwargs.get("placement_offset_y"),
        )
        if toolpaths and getattr(toolpaths[0], "coordinate_space", "surface_mm") != "machine_deg":
            surface_toolpaths_before_export = [pipeline_core.clone_toolpath(path) for path in toolpaths]
            optimized_surface_toolpaths, export_debug = pipeline_core.optimize_post_generation_travel_order(surface_toolpaths_before_export)
            if isinstance(debug, dict):
                debug.update(export_debug)
            prepared_toolpaths = pipeline_core.prepare_toolpaths_for_projection(optimized_surface_toolpaths)
            projected_toolpaths = pipeline_core.project_toolpaths_to_ball_angles(
                prepared_toolpaths,
                center_lon_deg=kwargs.get("placement_offset_x", 0.0),
                center_lat_deg=kwargs.get("placement_offset_y", 0.0),
            )
            if isinstance(debug, dict):
                debug["final_export_paths"] = pipeline_core.build_final_export_path_entries(
                    prepared_toolpaths,
                    projected_toolpaths,
                )
            toolpaths = projected_toolpaths
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
            _update_path_debug(gcode, preview)
            if isinstance(debug, dict):
                try:
                    pipeline_core.rewrite_final_export_path_stats_artifact(debug)
                except Exception as exc:  # pragma: no cover - diagnostics only
                    self.logger.debug("Unable to rewrite final export path stats: %s", exc)
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
        _update_path_debug(gcode, preview)
        _update_gcode_export_summary(gcode)
        if isinstance(debug, dict):
            try:
                pipeline_core.audit_exported_path_coverage(
                    debug,
                    ball_diameter_mm=float(kwargs.get("ball_diameter_mm", pipeline_core.BALL_DIAMETER_MM)),
                    center_lon_deg=float(kwargs.get("placement_offset_x", 0.0)),
                    center_lat_deg=float(kwargs.get("placement_offset_y", 0.0)),
                    pen_diameter_mm=float((toolpaths[0].metadata or {}).get("pen_width_mm", 0.6)) if toolpaths else 0.6,
                )
            except Exception as exc:  # pragma: no cover - diagnostics only
                self.logger.debug("Unable to audit exported path coverage: %s", exc)
            try:
                pipeline_core.rewrite_final_export_path_stats_artifact(debug)
            except Exception as exc:  # pragma: no cover - diagnostics only
                self.logger.debug("Unable to rewrite final export path stats: %s", exc)
            for transient_key in (
                "_coverage_target_mask",
                "_coverage_allowed_mask",
                "_coverage_current_to_source_matrix",
                "_coverage_preview_source_mask",
                "_coverage_preview_source_to_surface_matrix",
            ):
                debug.pop(transient_key, None)
        self.logger.info("Generated projected G-code lines=%d preview_paths=%d", len(gcode), len(preview))
        return gcode, preview
