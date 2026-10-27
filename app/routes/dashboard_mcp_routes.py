from __future__ import annotations

import hmac
from ipaddress import ip_address
from pathlib import Path
import secrets

from flask import Blueprint, current_app, jsonify, request, session

from ..extensions import get_dashboard_mcp_bridge
from ..services.dashboard_mcp_bridge import DashboardMcpBridgeError


dashboard_mcp_bp = Blueprint("dashboard_mcp", __name__)
MAX_BODY_BYTES = 24 * 1024


def _loopback_only() -> bool:
    try:
        return ip_address(request.remote_addr or "").is_loopback
    except ValueError:
        return False


def _error(message: str, status: int, code: str = "request_rejected"):
    return jsonify({"ok": False, "error": message, "code": code}), status


def _mcp_auth_error():
    if not _loopback_only():
        return _error("Dashboard MCP routes are available only on this computer.", 403)
    token = _read_secret(current_app.config["DASHBOARD_MCP_TOKEN_FILE"])
    provided = request.headers.get("X-Dashboard-MCP-Token", "")
    authorization = request.headers.get("Authorization", "")
    if authorization.lower().startswith("bearer "):
        provided = authorization[7:].strip()
    if not token or not hmac.compare_digest(provided, token):
        return _error("A valid local MCP token is required.", 401, "unauthorized")
    return None


def _browser_auth_error(*, needs_session: bool = True):
    if not _loopback_only():
        return _error("Dashboard bridge routes are available only on this computer.", 403)
    origin = request.headers.get("Origin", "")
    if origin not in current_app.config["DASHBOARD_MCP_FRONTEND_ORIGINS"]:
        return _error("Dashboard origin was not allowed.", 403, "origin_rejected")
    if needs_session:
        if not session.get("dashboard_mcp_session"):
            return _error("Dashboard session is missing.", 403, "session_required")
        expected = session.get("dashboard_mcp_csrf", "")
        supplied = request.headers.get("X-Dashboard-CSRF", "")
        if not expected or not hmac.compare_digest(expected, supplied):
            return _error("Dashboard CSRF token is missing or invalid.", 403, "csrf_rejected")
    return None


def _read_secret(path_value: str) -> str:
    try:
        return Path(path_value).read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _json_body():
    if request.content_length is not None and request.content_length > MAX_BODY_BYTES:
        return None, _error("Request body exceeds the size limit.", 413, "body_too_large")
    value = request.get_json(silent=True)
    if not isinstance(value, dict):
        return None, _error("Request body must be a JSON object.", 400, "invalid_json")
    return value, None


def _bridge_error(error: DashboardMcpBridgeError):
    return _error(str(error), error.status_code, error.code)


@dashboard_mcp_bp.post("/api/mcp/dashboard/session")
def dashboard_session():
    rejected = _browser_auth_error(needs_session=False)
    if rejected:
        return rejected
    csrf = session.get("dashboard_mcp_csrf")
    if not isinstance(csrf, str) or not 32 <= len(csrf) <= 256:
        csrf = secrets.token_urlsafe(32)
        session["dashboard_mcp_csrf"] = csrf
    session["dashboard_mcp_session"] = True
    return jsonify({"ok": True, "csrf_token": csrf})


@dashboard_mcp_bp.post("/api/mcp/dashboard/snapshot")
def dashboard_snapshot():
    rejected = _browser_auth_error()
    if rejected:
        return rejected
    body, failure = _json_body()
    if failure:
        return failure
    try:
        get_dashboard_mcp_bridge().publish_snapshot(body)
    except DashboardMcpBridgeError as error:
        return _bridge_error(error)
    return jsonify({"ok": True})


@dashboard_mcp_bp.post("/api/mcp/dashboard/requests/next")
def dashboard_next_request():
    rejected = _browser_auth_error()
    if rejected:
        return rejected
    _, failure = _json_body()
    if failure:
        return failure
    return jsonify({"ok": True, "request": get_dashboard_mcp_bridge().get_next_dashboard_request()})


@dashboard_mcp_bp.post("/api/mcp/dashboard/requests/<request_id>/complete")
def dashboard_complete_request(request_id: str):
    rejected = _browser_auth_error()
    if rejected:
        return rejected
    body, failure = _json_body()
    if failure:
        return failure
    try:
        result = get_dashboard_mcp_bridge().complete_request(
            request_id, body.get("status"), body.get("result", {})
        )
    except DashboardMcpBridgeError as error:
        return _bridge_error(error)
    return jsonify({"ok": True, "request": result})


@dashboard_mcp_bp.post("/api/mcp/dashboard/requests/<request_id>/start")
def dashboard_start_request(request_id: str):
    rejected = _browser_auth_error()
    if rejected:
        return rejected
    try:
        result = get_dashboard_mcp_bridge().start_request(request_id)
    except DashboardMcpBridgeError as error:
        return _bridge_error(error)
    return jsonify({"ok": True, "request": result})


@dashboard_mcp_bp.get("/api/mcp/v1/snapshot")
def mcp_snapshot():
    rejected = _mcp_auth_error()
    if rejected:
        return rejected
    return jsonify({"ok": True, "snapshot": get_dashboard_mcp_bridge().get_snapshot()})


@dashboard_mcp_bp.get("/api/mcp/v1/logs")
def mcp_logs():
    rejected = _mcp_auth_error()
    if rejected:
        return rejected
    snapshot = get_dashboard_mcp_bridge().get_snapshot() or {}
    return jsonify({"ok": True, "logs": snapshot.get("logs", [])})


@dashboard_mcp_bp.get("/api/mcp/v1/job")
def mcp_job():
    rejected = _mcp_auth_error()
    if rejected:
        return rejected
    snapshot = get_dashboard_mcp_bridge().get_snapshot() or {}
    return jsonify({"ok": True, "job": snapshot.get("job")})


@dashboard_mcp_bp.get("/api/mcp/v1/serial-diagnostics")
def mcp_serial_diagnostics():
    rejected = _mcp_auth_error()
    if rejected:
        return rejected
    snapshot = get_dashboard_mcp_bridge().get_snapshot() or {}
    return jsonify({"ok": True, "serial_diagnostics": snapshot.get("serial_diagnostics", {})})


@dashboard_mcp_bp.post("/api/mcp/v1/requests")
def mcp_create_request():
    rejected = _mcp_auth_error()
    if rejected:
        return rejected
    body, failure = _json_body()
    if failure:
        return failure
    try:
        created = get_dashboard_mcp_bridge().create_request(
            body.get("action"), body.get("arguments", {})
        )
    except DashboardMcpBridgeError as error:
        return _bridge_error(error)
    return jsonify({"ok": True, "request": created}), 202


@dashboard_mcp_bp.get("/api/mcp/v1/requests/<request_id>")
def mcp_get_request(request_id: str):
    rejected = _mcp_auth_error()
    if rejected:
        return rejected
    try:
        result = get_dashboard_mcp_bridge().get_request(request_id)
    except DashboardMcpBridgeError as error:
        return _bridge_error(error)
    return jsonify({"ok": True, "request": result})
