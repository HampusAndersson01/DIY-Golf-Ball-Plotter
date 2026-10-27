from __future__ import annotations

import asyncio
import json
import pytest

from scripts import dashboard_mcp_server as mcp_server


class FakeDashboardClient:
    def __init__(self) -> None:
        self.snapshot = {
            "machine": {"connected": True, "calibrated": False, "running": False, "paused": False},
            "job": {"job_id": "123e4567-e89b-12d3-a456-426614174000", "line_count": 8, "summary": {"gcode_line_count": 8}},
            "logs": [f"log {index}" for index in range(30)],
            "serial_diagnostics": {"last_parsed_ack_line": "ok"},
        }
        self.requests: list[tuple[str, dict | None]] = []
        self.request_status = {"request_id": "request-1234", "status": "queued"}

    def get_snapshot(self):
        return self.snapshot

    def get_logs(self):
        return self.snapshot["logs"]

    def get_job(self):
        return self.snapshot["job"]

    def get_serial_diagnostics(self):
        return self.snapshot["serial_diagnostics"]

    def create_request(self, action, arguments):
        self.requests.append((action, arguments))
        return {"request_id": "request-1234", "status": "queued", "action": action}

    def get_request(self, request_id):
        self.requests.append(("status", {"request_id": request_id}))
        return self.request_status


@pytest.fixture
def fake_client(monkeypatch):
    client = FakeDashboardClient()
    monkeypatch.setattr(mcp_server, "dashboard_client", client)
    return client


def test_read_tools_return_only_the_dashboard_snapshot_sections(fake_client) -> None:
    assert mcp_server.get_machine_status() == fake_client.snapshot["machine"]
    assert mcp_server.get_current_job_summary() == fake_client.snapshot["job"]
    assert mcp_server.get_serial_diagnostics() == fake_client.snapshot["serial_diagnostics"]
    assert mcp_server.get_dashboard_logs(3) == ["log 27", "log 28", "log 29"]


def test_request_tools_use_only_the_fixed_action_allowlist(fake_client, monkeypatch) -> None:
    from types import SimpleNamespace

    monkeypatch.setattr(mcp_server, "list_serial_ports", lambda: [
        SimpleNamespace(device="COM10", vid=0x1A86, pid=0x7523),
    ])
    responses = [
        mcp_server.generate_diagnostic_job("3x3_squares"),
        mcp_server.request_connect("COM10"),
        mcp_server.request_calibration("The plotter is centered at origin and ready."),
        mcp_server.request_run("123e4567-e89b-12d3-a456-426614174000", "Run the current job now."),
    ]

    assert [action for action, _ in fake_client.requests] == [
        "generate_diagnostic_job", "connect", "calibrate", "run"
    ]
    assert all(response["request_id"] == "request-1234" for response in responses)
    assert all("current_state" in response and "gcode" not in response for response in responses)


def test_connect_requires_exact_existing_com_port_and_passes_only_usb_identity(fake_client, monkeypatch) -> None:
    from types import SimpleNamespace

    monkeypatch.setattr(mcp_server, "list_serial_ports", lambda: [
        SimpleNamespace(device="COM10", vid=0x1A86, pid=0x7523),
        SimpleNamespace(device="COM9", vid=0x2341, pid=0x0043),
    ])
    response = mcp_server.request_connect("COM10")
    assert response["action"] == "connect"
    assert fake_client.requests[-1] == ("connect", {
        "com_port": "COM10", "usb_vendor_id": 0x1A86, "usb_product_id": 0x7523,
    })
    with pytest.raises(ValueError, match="not currently connected"):
        mcp_server.request_connect("COM11")
    with pytest.raises(ValueError, match="COM port"):
        mcp_server.request_connect("COM10; rm")


def test_calibration_requires_centered_user_statement(fake_client) -> None:
    with pytest.raises(ValueError, match="center"):
        mcp_server.request_calibration("Calibrate the machine now.")
    with pytest.raises(ValueError, match="centered at origin"):
        mcp_server.request_calibration("The plotter is not centered at origin.")


def test_tool_inputs_are_bounded_and_status_ids_are_validated(fake_client) -> None:
    with pytest.raises(ValueError, match="50"):
        mcp_server.get_dashboard_logs(51)
    with pytest.raises(ValueError, match="300"):
        mcp_server.request_run("123e4567-e89b-12d3-a456-426614174000", "r" * 301)
    with pytest.raises(ValueError, match="request ID"):
        mcp_server.get_action_status("../../run-gcode")
    with pytest.raises(ValueError, match="direct recent user instruction"):
        mcp_server.request_run("123e4567-e89b-12d3-a456-426614174000", "Do the job")
    with pytest.raises(ValueError, match="direct recent user instruction"):
        mcp_server.request_run("123e4567-e89b-12d3-a456-426614174000", "Do not run this job")


def test_no_raw_gcode_or_approval_tool_is_registered() -> None:
    names = {tool.name for tool in asyncio.run(mcp_server.mcp.list_tools())}
    assert names == {
        "get_machine_status", "get_dashboard_logs", "get_current_job_summary", "get_serial_diagnostics",
        "generate_diagnostic_job", "request_connect", "request_calibration", "request_run", "get_action_status",
    }
    assert not names.intersection({"send_gcode", "raw_command", "approve_action", "jog", "set_servo"})


def test_server_reports_dashboard_unavailable_clearly(monkeypatch) -> None:
    class OfflineClient:
        def get_snapshot(self):
            return None

    monkeypatch.setattr(mcp_server, "dashboard_client", OfflineClient())
    with pytest.raises(mcp_server.DashboardUnavailable, match="open the plotter dashboard"):
        mcp_server.get_machine_status()
    for operation in (
        lambda: mcp_server.request_connect("COM10"),
        lambda: mcp_server.request_calibration("The plotter is centered at origin."),
        lambda: mcp_server.request_run("123e4567-e89b-12d3-a456-426614174000", "Run this job now."),
    ):
        with pytest.raises(mcp_server.DashboardUnavailable, match="open the plotter dashboard"):
            operation()


def test_read_tool_reports_disconnected_machine_state_without_serial_access(monkeypatch) -> None:
    class OfflineMachineClient:
        def get_snapshot(self):
            return {"machine": {"connected": False, "calibrated": False, "running": False, "paused": False}}

    monkeypatch.setattr(mcp_server, "dashboard_client", OfflineMachineClient())
    assert mcp_server.get_machine_status()["connected"] is False


def test_http_client_reads_only_token_and_uses_fixed_direct_loopback(monkeypatch, tmp_path) -> None:
    token_file = tmp_path / "dashboard_mcp_token"
    token_file.write_text("a" * 48, encoding="utf-8")
    observed = {}
    observed["handler_types"] = []

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, _limit):
            return json.dumps({"ok": True, "snapshot": {"machine": {"connected": False}}}).encode()

    class Opener:
        def open(self, request, timeout):
            observed["url"] = request.full_url
            observed["token"] = request.get_header("X-dashboard-mcp-token")
            observed["timeout"] = timeout
            return Response()

    monkeypatch.setattr(mcp_server, "build_opener", lambda *handlers: (observed["handler_types"].extend(type(handler).__name__ for handler in handlers) or Opener()))
    client = mcp_server.DashboardMcpClient(token_file)

    assert client.get_snapshot() == {"machine": {"connected": False}}
    assert observed["url"] == "http://127.0.0.1:5000/api/mcp/v1/snapshot"
    assert observed["token"] == "a" * 48
    assert observed["timeout"] == 2.5
    assert observed["handler_types"] == ["ProxyHandler", "_NoRedirectHandler"]


def test_http_client_fails_clearly_when_token_file_is_missing(tmp_path) -> None:
    client = mcp_server.DashboardMcpClient(tmp_path / "missing-token")

    with pytest.raises(mcp_server.DashboardUnavailable, match="MCP token is missing"):
        client.get_snapshot()


def test_tool_schema_has_only_closed_diagnostic_enum_and_recent_quote_safety_text() -> None:
    tools = {tool.name: tool for tool in asyncio.run(mcp_server.mcp.list_tools())}
    diagnostic_schema = tools["generate_diagnostic_job"].input_schema
    assert diagnostic_schema["properties"]["pattern"]["enum"] == ["3x3_squares", "x_axis_rotation_ticks"]
    assert tools["request_connect"].input_schema["required"] == ["com_port"]
    assert "already-authorized" in tools["request_connect"].description
    assert "exact, recent user quote" in tools["request_calibration"].description
    assert "separately requires confirmation" in tools["request_run"].description
