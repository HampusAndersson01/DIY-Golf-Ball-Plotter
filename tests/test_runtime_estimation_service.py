from __future__ import annotations

from app.services.runtime_estimation_service import (
    build_runtime_snapshot,
    compute_elapsed_seconds,
    compute_remaining_seconds,
    estimate_gcode_runtime,
)


def test_running_job_elapsed_increases_with_wall_clock():
    snapshot = {
        "job_started_at": 100.0,
        "paused_duration_seconds": 0.0,
        "job_estimated_total_seconds": 300.0,
        "job_state": "running",
        "progress_done": 10,
        "progress_total": 100,
    }

    assert compute_elapsed_seconds(snapshot, now_seconds=160.0) == 60.0
    assert compute_elapsed_seconds(snapshot, now_seconds=175.0) == 75.0


def test_completed_job_elapsed_freezes_and_remaining_is_zero():
    snapshot = {
        "job_started_at": 100.0,
        "job_finished_at": 180.0,
        "paused_duration_seconds": 5.0,
        "job_estimated_total_seconds": 300.0,
        "job_state": "completed",
        "progress_done": 100,
        "progress_total": 100,
    }

    assert compute_elapsed_seconds(snapshot, now_seconds=999.0) == 75.0
    assert compute_remaining_seconds(snapshot, now_seconds=999.0) == 0.0
    runtime = build_runtime_snapshot(snapshot, now_seconds=999.0)
    assert runtime["job_elapsed_seconds"] == 75.0
    assert runtime["job_estimated_remaining_seconds"] == 0.0


def test_stopped_job_elapsed_freezes_and_remaining_is_zero():
    snapshot = {
        "job_started_at": 100.0,
        "job_finished_at": 145.0,
        "paused_duration_seconds": 0.0,
        "job_estimated_total_seconds": 300.0,
        "job_state": "stopped",
        "progress_done": 30,
        "progress_total": 100,
    }

    assert compute_elapsed_seconds(snapshot, now_seconds=300.0) == 45.0
    assert compute_remaining_seconds(snapshot, now_seconds=300.0) == 0.0


def test_paused_job_elapsed_excludes_active_pause_time():
    snapshot = {
        "job_started_at": 100.0,
        "pause_started_at": 140.0,
        "paused_duration_seconds": 10.0,
        "job_estimated_total_seconds": 300.0,
        "job_state": "paused",
        "paused": True,
        "progress_done": 20,
        "progress_total": 100,
    }

    assert compute_elapsed_seconds(snapshot, now_seconds=200.0) == 30.0
    assert compute_remaining_seconds(snapshot, now_seconds=200.0) > 0.0


def test_dynamic_eta_blends_to_observed_progress_after_threshold():
    snapshot = {
        "job_started_at": 100.0,
        "paused_duration_seconds": 0.0,
        "job_estimated_total_seconds": 100.0,
        "job_state": "running",
        "progress_done": 50,
        "progress_total": 100,
    }

    remaining = compute_remaining_seconds(snapshot, now_seconds=200.0)
    assert remaining == 100.0


def test_dynamic_eta_prefers_weighted_progress_over_raw_line_fraction():
    snapshot = {
        "job_started_at": 100.0,
        "paused_duration_seconds": 0.0,
        "job_estimated_total_seconds": 564.0,
        "job_state": "running",
        "progress_done": 5463,
        "progress_total": 7680,
        "job_estimate_profile": {
            "cumulative_seconds_by_stream_line": [160.0] * 5463 + [564.0] * (7680 - 5463),
        },
    }

    remaining = compute_remaining_seconds(snapshot, now_seconds=261.0)
    assert remaining > 380.0
    assert remaining < 430.0


def test_dynamic_eta_does_not_divide_by_zero_at_start():
    snapshot = {
        "job_started_at": 100.0,
        "paused_duration_seconds": 0.0,
        "job_estimated_total_seconds": 120.0,
        "job_state": "running",
        "progress_done": 0,
        "progress_total": 100,
    }

    assert compute_remaining_seconds(snapshot, now_seconds=100.0) == 120.0


def test_remaining_is_clamped_to_zero_at_full_progress():
    snapshot = {
        "job_started_at": 100.0,
        "paused_duration_seconds": 0.0,
        "job_estimated_total_seconds": 120.0,
        "job_state": "running",
        "progress_done": 100,
        "progress_total": 100,
    }

    assert compute_remaining_seconds(snapshot, now_seconds=180.0) == 0.0


def test_estimate_gcode_runtime_includes_motion_dwell_pen_and_streaming_breakdown():
    gcode = [
        "(header comment)",
        "G21",
        "G90",
        "M3 S575",
        "G4 P0.030",
        "G1 X0.5000 Y0.0000 F1200.000",
        "M3 S700",
        "G4 P0.060",
        "G1 X1.0000 Y0.0000",
        "G4 P0.500",
        "G1 X10.0000 Y0.0000 F3000.000",
    ]

    estimate = estimate_gcode_runtime(
        gcode,
        draw_feed=1200.0,
        travel_feed=3000.0,
        pen_up_s=575,
        pen_down_s=700,
        serial_ack_overhead_seconds_per_line=0.02,
        short_segment_threshold=0.75,
        short_segment_overhead_seconds=0.03,
        finalization_overhead_seconds=2.5,
    ).as_dict()

    assert estimate["rawGcodeLines"] == len(gcode)
    assert estimate["streamableGcodeLines"] == 10
    assert estimate["estimatedMotionSeconds"] > 0.0
    assert estimate["estimatedPenSeconds"] == 0.09
    assert estimate["estimatedDwellSeconds"] == 0.5
    assert estimate["estimatedStreamingOverheadSeconds"] == 0.2
    assert estimate["estimatedShortSegmentOverheadSeconds"] == 0.06
    assert estimate["penLifts"] == 1
    assert estimate["penLowers"] == 1
    assert estimate["estimatedRuntimeSeconds"] > (
        estimate["estimatedMotionSeconds"]
        + estimate["estimatedPenSeconds"]
        + estimate["estimatedDwellSeconds"]
    )
