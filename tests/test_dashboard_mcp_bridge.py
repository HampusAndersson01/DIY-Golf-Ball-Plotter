from __future__ import annotations

import pytest

from app.services.dashboard_mcp_bridge import (
    DashboardMcpBridge,
    DashboardMcpBridgeError,
)


class FakeClock:
    def __init__(self) -> None:
        self.value = 100.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def dashboard_snapshot(**machine_overrides: object) -> dict:
    machine = {
        "connected": True,
        "calibrated": True,
        "running": False,
        "paused": False,
        "status": "Idle",
        "current_gcode_line": 0,
        "y_loop_test": {"enabled": False},
    }
    machine.update(machine_overrides)
    return {
        "machine": machine,
        "job": {
            "job_id": "job-123",
            "line_count": 20,
            "summary": {
                "image_size": "32x32",
                "gcode_line_count": 20,
                "estimated_runtime_seconds": 3.5,
                "wall_path_count": 2,
                "infill_path_count": 1,
                "detail_trace_path_count": 1,
            },
        },
        "logs": ["dashboard ready"],
        "serial_diagnostics": {
            "last_complete_parsed_line": "ok",
            "last_parsed_ack_line": "ok",
            "last_parsed_status_line": None,
            "last_ok_at": 100.0,
            "last_status_at": None,
            "port_state": {"portOpen": True, "readerActive": True, "writerActive": True},
            "unexpected_ok_count": 0,
            "unexpected_error_count": 0,
            "recent_grbl_lines": ["ok"],
        },
    }


def ready_bridge(clock: FakeClock | None = None, **options: object) -> DashboardMcpBridge:
    bridge = DashboardMcpBridge(clock=clock or FakeClock(), **options)
    bridge.publish_snapshot(dashboard_snapshot())
    return bridge


def test_motion_request_is_rejected_when_dashboard_heartbeat_is_stale() -> None:
    clock = FakeClock()
    bridge = ready_bridge(clock)
    clock.advance(5.1)

    with pytest.raises(DashboardMcpBridgeError, match="[Dd]ashboard.*available"):
        bridge.create_request("calibrate", {"user_confirmation": "It is centered."})


def test_run_request_is_rejected_when_generated_job_identity_changed() -> None:
    bridge = ready_bridge()

    with pytest.raises(DashboardMcpBridgeError, match="job.*changed"):
        bridge.create_request(
            "run",
            {"job_id": "old-job", "user_instruction": "Run this job now."},
        )


def test_queued_run_is_rejected_if_dashboard_job_changes_before_approval() -> None:
    bridge = ready_bridge()
    request = bridge.create_request(
        "run", {"job_id": "job-123", "user_instruction": "Run this job now."}
    )
    changed_snapshot = dashboard_snapshot()
    changed_snapshot["job"]["job_id"] = "job-456"
    bridge.publish_snapshot(changed_snapshot)

    assert bridge.get_next_dashboard_request() is None
    assert bridge.get_request(request["request_id"])["status"] == "rejected"


def test_second_motion_request_is_rejected_until_first_request_is_terminal() -> None:
    bridge = ready_bridge()
    first = bridge.create_request(
        "calibrate", {"user_confirmation": "The pen is centered at origin."}
    )

    with pytest.raises(DashboardMcpBridgeError, match="motion request.*pending"):
        bridge.create_request(
            "run", {"job_id": "job-123", "user_instruction": "Run this job now."}
        )

    assert bridge.get_request(first["request_id"])["status"] == "queued"


def test_unrecognized_action_cannot_be_queued() -> None:
    bridge = ready_bridge()

    with pytest.raises(DashboardMcpBridgeError, match="[Uu]nsupported dashboard action"):
        bridge.create_request("raw_gcode", {"command": "G1 X10"})


def test_non_string_action_is_rejected_without_type_error() -> None:
    bridge = ready_bridge()

    with pytest.raises(DashboardMcpBridgeError, match="Unsupported dashboard action"):
        bridge.create_request({"action": "run"}, {})


def test_malformed_dashboard_machine_snapshot_is_rejected() -> None:
    bridge = DashboardMcpBridge()
    snapshot = dashboard_snapshot()
    snapshot["machine"] = []

    with pytest.raises(DashboardMcpBridgeError, match="machine snapshot"):
        bridge.publish_snapshot(snapshot)


def test_calibration_request_requires_explicit_user_statement() -> None:
    bridge = ready_bridge()

    with pytest.raises(DashboardMcpBridgeError, match="user confirmation"):
        bridge.create_request("calibrate", {"user_confirmation": "   "})


def test_request_queue_is_bounded() -> None:
    bridge = ready_bridge(max_pending_requests=2)
    bridge.create_request("generate_diagnostic_job", {"pattern": "3x3_squares"})
    bridge.create_request("generate_diagnostic_job", {"pattern": "3x3_squares"})

    with pytest.raises(DashboardMcpBridgeError, match="queue is full"):
        bridge.create_request("generate_diagnostic_job", {"pattern": "3x3_squares"})


def test_expired_request_is_not_delivered_and_has_terminal_status() -> None:
    clock = FakeClock()
    bridge = DashboardMcpBridge(clock=clock, request_ttl_seconds=120)
    bridge.publish_snapshot(dashboard_snapshot(connected=False))
    request = bridge.create_request("connect", {
        "com_port": "COM10", "usb_vendor_id": 0x1A86, "usb_product_id": 0x7523,
    })
    clock.advance(121)

    assert bridge.get_next_dashboard_request() is None
    assert bridge.get_request(request["request_id"])["status"] == "expired"


def test_connect_request_requires_a_single_explicit_com_port_and_usb_identity() -> None:
    bridge = DashboardMcpBridge(clock=FakeClock())
    bridge.publish_snapshot(dashboard_snapshot(connected=False))
    request = bridge.create_request("connect", {
        "com_port": "COM10", "usb_vendor_id": 0x1A86, "usb_product_id": 0x7523,
    })
    assert request["arguments"] == {"com_port": "COM10", "usb_vendor_id": 0x1A86, "usb_product_id": 0x7523}
    with pytest.raises(DashboardMcpBridgeError, match="schema"):
        bridge.create_request("connect", {"com_port": "COM10", "usb_vendor_id": 0x1A86})
    with pytest.raises(DashboardMcpBridgeError, match="COM port"):
        bridge.create_request("connect", {"com_port": "COM0", "usb_vendor_id": 0x1A86, "usb_product_id": 0x7523})


def test_dashboard_receives_typed_request_and_reports_terminal_result() -> None:
    bridge = ready_bridge()
    request = bridge.create_request("calibrate", {
        "user_confirmation": "The plotter is centered and ready to calibrate.",
    })

    delivered = bridge.get_next_dashboard_request()
    assert delivered == request | {"status": "awaiting_operator"}

    completed = bridge.complete_request(
        request["request_id"], "completed", {"message": "Origin set."}
    )
    assert completed["status"] == "completed"
    assert bridge.get_request(request["request_id"])["result"] == {"message": "Origin set."}
    assert bridge.get_next_dashboard_request() is None


def test_approved_run_stays_executing_past_pending_expiry_then_fails_if_dashboard_disappears() -> None:
    clock = FakeClock()
    bridge = DashboardMcpBridge(clock=clock, request_ttl_seconds=3)
    bridge.publish_snapshot(dashboard_snapshot())
    request = bridge.create_request("run", {
        "job_id": "job-123", "user_instruction": "Run this job now."
    })
    assert bridge.get_next_dashboard_request()["status"] == "awaiting_operator"
    assert bridge.start_request(request["request_id"])["status"] == "executing"

    clock.advance(4)
    bridge.publish_snapshot(dashboard_snapshot())
    assert bridge.get_request(request["request_id"])["status"] == "executing"

    clock.advance(6)
    failed = bridge.get_request(request["request_id"])
    assert failed["status"] == "failed"
    assert "check the plotter" in failed["result"]["message"]


def test_snapshot_is_copied_and_stale_snapshot_is_not_exposed() -> None:
    clock = FakeClock()
    bridge = DashboardMcpBridge(clock=clock)
    snapshot = dashboard_snapshot()
    bridge.publish_snapshot(snapshot)
    snapshot["machine"]["connected"] = False

    assert bridge.get_snapshot()["machine"]["connected"] is True
    clock.advance(5.1)
    assert bridge.get_snapshot() is None
