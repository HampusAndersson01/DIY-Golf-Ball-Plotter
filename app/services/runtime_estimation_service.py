from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass
from typing import Any

from .gcode_service import GcodeService


TERMINAL_JOB_STATES = {"completed", "stopped", "aborted", "error", "failed"}
_WORD_RE = re.compile(r"([A-Z])([-+]?\d*\.?\d+)")


@dataclass(slots=True)
class RuntimeEstimateBreakdown:
    estimated_runtime_seconds: float
    estimated_motion_seconds: float
    estimated_pen_seconds: float
    estimated_dwell_seconds: float
    estimated_streaming_overhead_seconds: float
    estimated_short_segment_overhead_seconds: float
    estimated_finalization_overhead_seconds: float
    raw_gcode_lines: int
    streamable_gcode_lines: int
    pen_lifts: int
    pen_lowers: int
    short_segment_count: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "estimatedRuntimeSeconds": self.estimated_runtime_seconds,
            "estimatedMotionSeconds": self.estimated_motion_seconds,
            "estimatedPenSeconds": self.estimated_pen_seconds,
            "estimatedDwellSeconds": self.estimated_dwell_seconds,
            "estimatedStreamingOverheadSeconds": self.estimated_streaming_overhead_seconds,
            "estimatedShortSegmentOverheadSeconds": self.estimated_short_segment_overhead_seconds,
            "estimatedFinalizationOverheadSeconds": self.estimated_finalization_overhead_seconds,
            "rawGcodeLines": self.raw_gcode_lines,
            "streamableGcodeLines": self.streamable_gcode_lines,
            "penLifts": self.pen_lifts,
            "penLowers": self.pen_lowers,
            "shortSegmentCount": self.short_segment_count,
        }


def _parse_words(line: str) -> dict[str, float]:
    values: dict[str, float] = {}
    for key, raw_value in _WORD_RE.findall(line.upper()):
        try:
            values[key] = float(raw_value)
        except ValueError:
            continue
    return values


def estimate_gcode_runtime(
    gcode: list[str],
    *,
    draw_feed: float,
    travel_feed: float,
    pen_up_s: int,
    pen_down_s: int,
    serial_ack_overhead_seconds_per_line: float = 0.022,
    short_segment_threshold: float = 0.75,
    short_segment_overhead_seconds: float = 0.03,
    finalization_overhead_seconds: float = 2.5,
) -> RuntimeEstimateBreakdown:
    gcode_service = GcodeService()
    raw_gcode_lines = len(gcode)
    streamable_gcode_lines = sum(1 for line in gcode if gcode_service.is_streamable_line(line))
    current_x = 0.0
    current_y = 0.0
    current_feed = max(float(draw_feed or 0.0), 1e-6)
    motion_seconds = 0.0
    pen_seconds = 0.0
    dwell_seconds = 0.0
    short_segment_count = 0
    pen_lifts = 0
    pen_lowers = 0
    pending_pen_servo = False

    for raw_line in gcode:
        line = raw_line.strip().upper()
        if not line or (line.startswith("(") and line.endswith(")")):
            continue

        words = _parse_words(line)
        if line.startswith("M3"):
            servo_target = int(words.get("S", -1))
            if servo_target == int(pen_up_s):
                pen_lifts += 1
                pending_pen_servo = True
            elif servo_target == int(pen_down_s):
                pen_lowers += 1
                pending_pen_servo = True
            continue

        if line.startswith("G4"):
            dwell = max(0.0, float(words.get("P", 0.0)))
            if pending_pen_servo:
                pen_seconds += dwell
                pending_pen_servo = False
            else:
                dwell_seconds += dwell
            continue

        pending_pen_servo = False

        if line.startswith("G0") or line.startswith("G1"):
            x = float(words.get("X", current_x))
            y = float(words.get("Y", current_y))
            current_feed = max(float(words.get("F", current_feed)), 1e-6)
            distance = math.hypot(x - current_x, y - current_y)
            feed = current_feed
            if line.startswith("G0") and "F" not in words:
                feed = max(float(travel_feed or 0.0), 1e-6)
            motion_seconds += (distance / max(feed, 1e-6)) * 60.0
            if 0.0 < distance <= short_segment_threshold:
                short_segment_count += 1
            current_x = x
            current_y = y

    streaming_seconds = streamable_gcode_lines * max(0.0, serial_ack_overhead_seconds_per_line)
    short_segment_seconds = short_segment_count * max(0.0, short_segment_overhead_seconds)
    total_seconds = (
        motion_seconds
        + pen_seconds
        + dwell_seconds
        + streaming_seconds
        + short_segment_seconds
        + max(0.0, finalization_overhead_seconds)
    )
    return RuntimeEstimateBreakdown(
        estimated_runtime_seconds=max(0.0, total_seconds),
        estimated_motion_seconds=max(0.0, motion_seconds),
        estimated_pen_seconds=max(0.0, pen_seconds),
        estimated_dwell_seconds=max(0.0, dwell_seconds),
        estimated_streaming_overhead_seconds=max(0.0, streaming_seconds),
        estimated_short_segment_overhead_seconds=max(0.0, short_segment_seconds),
        estimated_finalization_overhead_seconds=max(0.0, finalization_overhead_seconds),
        raw_gcode_lines=raw_gcode_lines,
        streamable_gcode_lines=streamable_gcode_lines,
        pen_lifts=pen_lifts,
        pen_lowers=pen_lowers,
        short_segment_count=short_segment_count,
    )


def compute_elapsed_seconds(snapshot: dict[str, Any], now_seconds: float | None = None) -> float:
    started_at = snapshot.get("job_started_at") or snapshot.get("run_started_at")
    if not started_at:
        return 0.0
    paused_duration_seconds = float(snapshot.get("paused_duration_seconds") or 0.0)
    finished_at = snapshot.get("job_finished_at") or snapshot.get("run_finished_at")
    if finished_at:
        return max(0.0, float(finished_at) - float(started_at) - paused_duration_seconds)
    if snapshot.get("paused") and snapshot.get("pause_started_at"):
        return max(0.0, float(snapshot["pause_started_at"]) - float(started_at) - paused_duration_seconds)
    now_seconds = time.time() if now_seconds is None else now_seconds
    return max(0.0, float(now_seconds) - float(started_at) - paused_duration_seconds)


def compute_progress_fraction(snapshot: dict[str, Any]) -> float:
    total = max(0, int(snapshot.get("progress_total") or 0))
    done = max(0, min(total, int(snapshot.get("progress_done") or 0)))
    if total <= 0:
        return 0.0
    return max(0.0, min(1.0, done / total))


def compute_remaining_seconds(snapshot: dict[str, Any], *, now_seconds: float | None = None) -> float:
    job_state = str(snapshot.get("job_state") or "idle")
    if job_state in TERMINAL_JOB_STATES:
        return 0.0

    elapsed_seconds = compute_elapsed_seconds(snapshot, now_seconds=now_seconds)
    static_total = max(0.0, float(snapshot.get("job_estimated_total_seconds") or 0.0))
    progress_fraction = compute_progress_fraction(snapshot)
    if progress_fraction >= 1.0:
        return 0.0

    estimated_total = static_total
    if progress_fraction >= 0.05 and elapsed_seconds > 0.0:
        observed_total = elapsed_seconds / progress_fraction
        if estimated_total > 0.0:
            blend_weight = min(1.0, max(0.0, (progress_fraction - 0.05) / 0.15))
            estimated_total = (estimated_total * (1.0 - blend_weight)) + (observed_total * blend_weight)
        else:
            estimated_total = observed_total

    if estimated_total <= 0.0:
        return 0.0
    return max(0.0, estimated_total - elapsed_seconds)


def build_runtime_snapshot(snapshot: dict[str, Any], *, now_seconds: float | None = None) -> dict[str, Any]:
    elapsed_seconds = compute_elapsed_seconds(snapshot, now_seconds=now_seconds)
    remaining_seconds = compute_remaining_seconds(snapshot, now_seconds=now_seconds)
    estimated_total_seconds = max(0.0, float(snapshot.get("job_estimated_total_seconds") or 0.0))
    runtime_estimate_multiplier = 1.0
    if estimated_total_seconds > 0.0 and elapsed_seconds > 0.0 and str(snapshot.get("job_state") or "") == "completed":
        runtime_estimate_multiplier = max(0.0, elapsed_seconds / estimated_total_seconds)
    return {
        "job_elapsed_seconds": elapsed_seconds,
        "job_estimated_total_seconds": estimated_total_seconds,
        "job_estimated_remaining_seconds": remaining_seconds,
        "runtime_estimate_multiplier": runtime_estimate_multiplier,
    }
