from __future__ import annotations

import math

from app.services import pipeline_core


def count_motion_lines(gcode: list[str]) -> int:
    current_motion_command: str | None = None
    motion_lines = 0
    for raw_line in gcode:
        line = raw_line.strip().upper()
        if not line or (line.startswith("(") and line.endswith(")")):
            continue
        if line.startswith("G0"):
            current_motion_command = "G0"
        elif line.startswith("G1"):
            current_motion_command = "G1"
        if current_motion_command in {"G0", "G1"} and ("X" in line or "Y" in line):
            motion_lines += 1
    return motion_lines


def build_generation_metrics(
    gcode: list[str],
    *,
    pen_up_s: int,
    pen_down_s: int,
    line_width_mm: float,
    center_lon_deg: float = 0.0,
    center_lat_deg: float = 0.0,
    ball_diameter_mm: float = pipeline_core.BALL_DIAMETER_MM,
    estimated_draw_time_seconds: float | None = None,
) -> dict[str, float | int]:
    motion_paths = pipeline_core.parse_gcode_machine_motion_paths(
        gcode,
        pen_up_s=pen_up_s,
        pen_down_s=pen_down_s,
    )
    segment_lengths_mm: list[float] = []
    travel_segment_count = 0
    drawing_segment_count = 0

    for path in motion_paths:
        surface_points = [
            pipeline_core.ball_angles_to_surface_mm(
                point,
                center_lon_deg=center_lon_deg,
                center_lat_deg=center_lat_deg,
                ball_diameter_mm=ball_diameter_mm,
            )
            for point in path.points
        ]
        for index in range(1, len(surface_points)):
            length_mm = math.hypot(
                surface_points[index].x - surface_points[index - 1].x,
                surface_points[index].y - surface_points[index - 1].y,
            )
            if length_mm <= 1e-9:
                continue
            segment_lengths_mm.append(float(length_mm))
            if path.kind == "travel":
                travel_segment_count += 1
            else:
                drawing_segment_count += 1

    minimum_segment_length_mm = min(segment_lengths_mm) if segment_lengths_mm else 0.0
    average_segment_length_mm = (sum(segment_lengths_mm) / len(segment_lengths_mm)) if segment_lengths_mm else 0.0
    segments_below_pen_width_count = sum(1 for length_mm in segment_lengths_mm if length_mm + 1e-9 < line_width_mm)
    segments_below_pen_width_percent = (
        (segments_below_pen_width_count / len(segment_lengths_mm)) * 100.0 if segment_lengths_mm else 0.0
    )

    return {
        "path_count": len(motion_paths),
        "motion_line_count": count_motion_lines(gcode),
        "travel_segment_count": travel_segment_count,
        "drawing_segment_count": drawing_segment_count,
        "average_segment_length_mm": float(average_segment_length_mm),
        "minimum_segment_length_mm": float(minimum_segment_length_mm),
        "segments_below_pen_width_count": int(segments_below_pen_width_count),
        "segments_below_pen_width_percent": float(segments_below_pen_width_percent),
        "m3_command_count": int(sum(1 for line in gcode if line.strip().upper().startswith("M3"))),
        "g4_command_count": int(sum(1 for line in gcode if line.strip().upper().startswith("G4"))),
        "estimated_draw_time_seconds": float(max(0.0, estimated_draw_time_seconds or 0.0)),
    }
