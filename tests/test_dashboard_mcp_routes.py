from __future__ import annotations

import json

import pytest

from app import _load_or_create_secret, create_app
from app.config import Config


MCP_TOKEN = "test-mcp-token-not-for-production"
FRONTEND_ORIGIN = "http://127.0.0.1:5173"


@pytest.fixture
def mcp_app(tmp_path):
    token_file = tmp_path / "dashboard_mcp_token"
    token_file.write_text(MCP_TOKEN, encoding="utf-8")
    config = type(
        "McpTestConfig",
        (Config,),
        {
            "TESTING": True,
            "DASHBOARD_MCP_TOKEN_FILE": str(token_file),
            "DASHBOARD_MCP_FRONTEND_ORIGINS": (FRONTEND_ORIGIN,),
        },
    )
    return create_app(config)


def browser_session(client):
    response = client.post("/api/mcp/dashboard/session", json={}, headers={"Origin": FRONTEND_ORIGIN})
    assert response.status_code == 200
    return response.get_json()["csrf_token"]


def test_mcp_snapshot_requires_bearer_token(mcp_app) -> None:
    response = mcp_app.test_client().get("/api/mcp/v1/snapshot")

    assert response.status_code == 401


def test_invalid_token_file_is_replaced_on_backend_startup(tmp_path) -> None:
    token_file = tmp_path / "dashboard_mcp_token"
    token_file.write_text("stale", encoding="utf-8")

    token = _load_or_create_secret(str(token_file))

    assert 32 <= len(token) <= 512
    assert token_file.read_text(encoding="utf-8").strip() == token


def test_browser_session_and_poll_require_post_origin_requests(mcp_app) -> None:
    client = mcp_app.test_client()
    session_get = client.get("/api/mcp/dashboard/session", headers={"Origin": FRONTEND_ORIGIN})
    assert session_get.status_code == 405
    csrf = browser_session(client)
    poll_get = client.get(
        "/api/mcp/dashboard/requests/next",
        headers={"Origin": FRONTEND_ORIGIN, "X-Dashboard-CSRF": csrf},
    )

    assert poll_get.status_code == 405


def test_browser_session_initialization_is_idempotent(mcp_app) -> None:
    client = mcp_app.test_client()
    first = browser_session(client)
    second = client.post(
        "/api/mcp/dashboard/session", json={}, headers={"Origin": FRONTEND_ORIGIN}
    )

    assert second.status_code == 200
    assert second.get_json()["csrf_token"] == first


def test_mcp_routes_reject_non_loopback_clients(mcp_app) -> None:
    response = mcp_app.test_client().get(
        "/api/mcp/v1/snapshot",
        headers={"X-Dashboard-MCP-Token": MCP_TOKEN},
        environ_overrides={"REMOTE_ADDR": "192.168.1.42"},
    )

    assert response.status_code == 403


def test_browser_snapshot_is_the_source_for_mcp_state(mcp_app) -> None:
    client = mcp_app.test_client()
    csrf = browser_session(client)
    snapshot = {
        "machine": {"connected": True, "calibrated": True, "running": False, "paused": False, "status": "Idle", "current_gcode_line": 0, "y_loop_test": {"enabled": False}},
        "job": {"job_id": "job-9", "line_count": 9, "summary": {"image_size": "10x10", "gcode_line_count": 9, "estimated_runtime_seconds": 1.0, "wall_path_count": 2, "infill_path_count": 0, "detail_trace_path_count": 0}},
        "logs": ["browser log"],
        "serial_diagnostics": {"last_complete_parsed_line": "ok", "last_parsed_ack_line": "ok", "last_parsed_status_line": None, "last_ok_at": 1.0, "last_status_at": None, "port_state": {"portOpen": True, "readerActive": True, "writerActive": True}, "unexpected_ok_count": 0, "unexpected_error_count": 0, "recent_grbl_lines": ["ok"]},
    }
    heartbeat = client.post(
        "/api/mcp/dashboard/snapshot",
        json=snapshot,
        headers={"Origin": FRONTEND_ORIGIN, "X-Dashboard-CSRF": csrf},
    )
    response = client.get(
        "/api/mcp/v1/snapshot", headers={"X-Dashboard-MCP-Token": MCP_TOKEN}
    )

    assert heartbeat.status_code == 200
    assert response.get_json()["snapshot"]["job"]["job_id"] == "job-9"
    assert response.get_json()["snapshot"]["machine"]["connected"] is True


def test_dashboard_snapshot_requires_session_csrf_and_exact_origin(mcp_app) -> None:
    client = mcp_app.test_client()
    browser_session(client)
    snapshot = {"machine": {}, "job": {}, "logs": [], "serial_diagnostics": {}}

    no_csrf = client.post(
        "/api/mcp/dashboard/snapshot", json=snapshot, headers={"Origin": FRONTEND_ORIGIN}
    )
    wrong_origin = client.post(
        "/api/mcp/dashboard/snapshot",
        json=snapshot,
        headers={"Origin": "https://attacker.example", "X-Dashboard-CSRF": "wrong"},
    )

    assert no_csrf.status_code == 403
    assert wrong_origin.status_code == 403


def test_mcp_cannot_complete_or_approve_a_dashboard_request(mcp_app) -> None:
    client = mcp_app.test_client()
    response = client.post(
        "/api/mcp/v1/requests/request-1/complete",
        json={"status": "completed", "result": {}},
        headers={"X-Dashboard-MCP-Token": MCP_TOKEN},
    )

    assert response.status_code == 404

    start_response = client.post(
        "/api/mcp/v1/requests/request-1/start",
        json={},
        headers={"X-Dashboard-MCP-Token": MCP_TOKEN},
    )
    assert start_response.status_code == 404


def test_only_dashboard_session_can_mark_approved_request_executing(mcp_app) -> None:
    client = mcp_app.test_client()
    csrf = browser_session(client)
    client.post(
        "/api/mcp/dashboard/snapshot",
        json={
            "machine": {"connected": True, "calibrated": True, "running": False, "paused": False, "status": "Idle", "current_gcode_line": 0, "y_loop_test": {"enabled": False}},
            "job": {"job_id": "job-123", "line_count": 8, "summary": {"image_size": "10x10", "gcode_line_count": 8, "estimated_runtime_seconds": 1.0, "wall_path_count": 1, "infill_path_count": 1, "detail_trace_path_count": 0}},
            "logs": [],
            "serial_diagnostics": {"last_complete_parsed_line": "ok", "last_parsed_ack_line": "ok", "last_parsed_status_line": None, "last_ok_at": 1.0, "last_status_at": None, "port_state": {"portOpen": True, "readerActive": True, "writerActive": True}, "unexpected_ok_count": 0, "unexpected_error_count": 0, "recent_grbl_lines": ["ok"]},
        },
        headers={"Origin": FRONTEND_ORIGIN, "X-Dashboard-CSRF": csrf},
    )
    created = client.post(
        "/api/mcp/v1/requests",
        json={"action": "run", "arguments": {"job_id": "job-123", "user_instruction": "Run job-123 now."}},
        headers={"X-Dashboard-MCP-Token": MCP_TOKEN},
    ).get_json()["request"]
    csrf = browser_session(client)
    pending = client.post(
        "/api/mcp/dashboard/requests/next",
        json={},
        headers={"Origin": FRONTEND_ORIGIN, "X-Dashboard-CSRF": csrf},
    )
    assert pending.get_json()["request"]["request_id"] == created["request_id"]
    response = client.post(
        f"/api/mcp/dashboard/requests/{created['request_id']}/start",
        json={},
        headers={"Origin": FRONTEND_ORIGIN, "X-Dashboard-CSRF": csrf},
    )

    assert response.status_code == 200
    assert response.get_json()["request"]["status"] == "executing"


def test_mcp_request_route_rejects_unlisted_action(mcp_app) -> None:
    client = mcp_app.test_client()
    browser_session(client)
    client.post(
        "/api/mcp/dashboard/snapshot",
        json={
            "machine": {"connected": True, "calibrated": True, "running": False, "paused": False, "status": "Idle", "current_gcode_line": 0, "y_loop_test": {"enabled": False}},
            "job": {"job_id": "job-1", "line_count": 1, "summary": {"image_size": "1x1", "gcode_line_count": 1, "estimated_runtime_seconds": 0.1, "wall_path_count": 1, "infill_path_count": 0, "detail_trace_path_count": 0}},
            "logs": [],
            "serial_diagnostics": {"last_complete_parsed_line": None, "last_parsed_ack_line": None, "last_parsed_status_line": None, "last_ok_at": None, "last_status_at": None, "port_state": {"portOpen": False, "readerActive": False, "writerActive": False}, "unexpected_ok_count": 0, "unexpected_error_count": 0, "recent_grbl_lines": []},
        },
        headers={"Origin": FRONTEND_ORIGIN, "X-Dashboard-CSRF": browser_session(client)},
    )
    response = client.post(
        "/api/mcp/v1/requests",
        json={"action": "raw_gcode", "arguments": {"command": "G1 X10"}},
        headers={"X-Dashboard-MCP-Token": MCP_TOKEN},
    )

    assert response.status_code == 400
    assert "Unsupported" in response.get_json()["error"]
