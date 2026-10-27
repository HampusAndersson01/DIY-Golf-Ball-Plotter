from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Literal
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

from mcp.server import MCPServer
from serial.tools.list_ports import comports as list_serial_ports


PROJECT_ROOT = Path(__file__).resolve().parents[1]
TOKEN_PATH = PROJECT_ROOT / "instance" / "dashboard_mcp_token"
BASE_URL = "http://127.0.0.1:5000"
MAX_RESPONSE_BYTES = 64 * 1024
MAX_REQUEST_BYTES = 24 * 1024
REQUEST_ID_PATTERN = re.compile(r"^[a-f0-9-]{8,64}$", re.IGNORECASE)
ALLOWED_ACTIONS = frozenset({"connect", "generate_diagnostic_job", "calibrate", "run"})
class DashboardUnavailable(RuntimeError):
    """The local dashboard bridge is unavailable or not authenticated."""


class _NoRedirectHandler(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class DashboardMcpClient:
    """Fixed-route, loopback-only HTTP client for the Flask dashboard bridge."""

    def __init__(self, token_path: Path = TOKEN_PATH) -> None:
        self._token_path = Path(token_path)

    def get_snapshot(self) -> dict | None:
        payload = self._request("GET", "/api/mcp/v1/snapshot")
        snapshot = payload.get("snapshot")
        if snapshot is None:
            raise DashboardUnavailable("Dashboard is not running; open the plotter dashboard and try again.")
        if not isinstance(snapshot, dict):
            raise DashboardUnavailable("Dashboard returned an invalid status snapshot.")
        return snapshot

    def get_logs(self) -> list[str]:
        payload = self._request("GET", "/api/mcp/v1/logs")
        logs = payload.get("logs")
        if not isinstance(logs, list) or any(not isinstance(line, str) for line in logs):
            raise DashboardUnavailable("Dashboard returned invalid logs.")
        return logs[-50:]

    def get_job(self) -> dict | None:
        payload = self._request("GET", "/api/mcp/v1/job")
        job = payload.get("job")
        return job if isinstance(job, dict) else None

    def get_serial_diagnostics(self) -> dict:
        payload = self._request("GET", "/api/mcp/v1/serial-diagnostics")
        diagnostics = payload.get("serial_diagnostics")
        if not isinstance(diagnostics, dict):
            raise DashboardUnavailable("Dashboard returned invalid serial diagnostics.")
        return diagnostics

    def create_request(self, action: str, arguments: dict) -> dict:
        if action not in ALLOWED_ACTIONS:
            raise ValueError("Unsupported dashboard action.")
        payload = self._request("POST", "/api/mcp/v1/requests", {"action": action, "arguments": arguments})
        result = payload.get("request")
        if not isinstance(result, dict):
            raise DashboardUnavailable("Dashboard did not return a request ID.")
        return result

    def get_request(self, request_id: str) -> dict:
        if not _is_request_id(request_id):
            raise ValueError("Invalid request ID.")
        payload = self._request("GET", f"/api/mcp/v1/requests/{request_id}")
        result = payload.get("request")
        if not isinstance(result, dict):
            raise DashboardUnavailable("Dashboard did not return request status.")
        return result

    def _request(self, method: Literal["GET", "POST"], path: str, body: dict | None = None) -> dict:
        if not path.startswith("/api/mcp/v1/") or ".." in path or "?" in path:
            raise ValueError("Unsupported dashboard bridge route.")
        token = self._read_token()
        encoded = None if body is None else json.dumps(body, separators=(",", ":"), allow_nan=False).encode("utf-8")
        if encoded is not None and len(encoded) > MAX_REQUEST_BYTES:
            raise ValueError("Dashboard request exceeds the size limit.")
        request = Request(
            f"{BASE_URL}{path}",
            data=encoded,
            headers={
                "X-Dashboard-MCP-Token": token,
                "Accept": "application/json",
                **({"Content-Type": "application/json"} if encoded is not None else {}),
            },
            method=method,
        )
        try:
            opener = build_opener(ProxyHandler({}), _NoRedirectHandler())
            with opener.open(request, timeout=2.5) as response:
                raw = response.read(MAX_RESPONSE_BYTES + 1)
        except HTTPError as error:
            if error.code in {401, 403, 404, 409, 429, 503}:
                raise DashboardUnavailable(_safe_http_message(error.code)) from None
            raise DashboardUnavailable(f"Local dashboard bridge returned HTTP {error.code}.") from None
        except (URLError, TimeoutError, OSError) as error:
            raise DashboardUnavailable("Local dashboard bridge is unavailable; start the Flask app and open its dashboard.") from error
        if len(raw) > MAX_RESPONSE_BYTES:
            raise DashboardUnavailable("Local dashboard response exceeded the size limit.")
        try:
            parsed = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise DashboardUnavailable("Local dashboard returned invalid JSON.") from None
        if not isinstance(parsed, dict) or parsed.get("ok") is not True:
            raise DashboardUnavailable("Local dashboard rejected the request.")
        return parsed

    def _read_token(self) -> str:
        try:
            token = self._token_path.read_text(encoding="utf-8").strip()
        except OSError:
            raise DashboardUnavailable("MCP token is missing; start the Flask app once, then reconnect Codex.") from None
        if len(token) < 32 or len(token) > 512:
            raise DashboardUnavailable("MCP token is invalid; restart the Flask app to regenerate it.")
        return token


def _safe_http_message(status: int) -> str:
    if status == 401:
        return "Local dashboard rejected the MCP token; restart Flask and reconnect Codex."
    if status == 403:
        return "Local dashboard bridge permits requests from this computer only."
    if status == 404:
        return "Dashboard request was not found or the running backend is outdated."
    if status == 409:
        return "Dashboard state changed or conflicts with this request; inspect the dashboard and retry."
    if status == 429:
        return "Dashboard request queue is full; wait for the current request to finish."
    return "Dashboard bridge is unavailable; open the plotter dashboard and try again."


def _is_request_id(value: str) -> bool:
    return isinstance(value, str) and bool(REQUEST_ID_PATTERN.fullmatch(value))


dashboard_client = DashboardMcpClient()
mcp = MCPServer("DIY Golf Ball Plotter Dashboard")


def _require_live_dashboard() -> dict:
    snapshot = dashboard_client.get_snapshot()
    if snapshot is None:
        raise DashboardUnavailable("Dashboard is not running; open the plotter dashboard and try again.")
    return snapshot


def _submit_dashboard_request(action: str, arguments: dict) -> dict:
    snapshot = _require_live_dashboard()
    request = dashboard_client.create_request(action, arguments)
    return {
        "request_id": request.get("request_id"),
        "status": request.get("status"),
        "action": request.get("action", action),
        "current_state": {
            "machine": snapshot.get("machine", {}),
            "job": snapshot.get("job", {}),
        },
    }


@mcp.tool()
def get_machine_status() -> dict:
    """Read connection, calibration, run, and current-line status from the live dashboard."""
    return _require_live_dashboard().get("machine", {})


@mcp.tool()
def get_dashboard_logs(limit: int = 20) -> list[str]:
    """Read up to 50 recent dashboard logs; generated G-code is not included."""
    if type(limit) is not int or not 0 <= limit <= 50:
        raise ValueError("Log limit must be between 0 and 50.")
    _require_live_dashboard()
    return dashboard_client.get_logs()[-limit:] if limit else []


@mcp.tool()
def get_current_job_summary() -> dict | None:
    """Read the current job ID, line count, and summary without retrieving G-code."""
    _require_live_dashboard()
    return dashboard_client.get_job()


@mcp.tool()
def get_serial_diagnostics() -> dict:
    """Read bounded parsed serial status diagnostics from the live dashboard."""
    _require_live_dashboard()
    return dashboard_client.get_serial_diagnostics()


@mcp.tool()
def generate_diagnostic_job(pattern: Literal["3x3_squares", "x_axis_rotation_ticks"]) -> dict:
    """Request a fixed diagnostic preview only; this does not move the plotter."""
    if pattern not in {"3x3_squares", "x_axis_rotation_ticks"}:
        raise ValueError("Unsupported diagnostic pattern.")
    return _submit_dashboard_request("generate_diagnostic_job", {"pattern": pattern})


@mcp.tool()
def request_connect(com_port: str) -> dict:
    """Connect the exact user-approved Windows COM port through an already-authorized dashboard Web Serial port; refuses missing or ambiguous browser permissions."""
    if not isinstance(com_port, str) or not re.fullmatch(r"COM[1-9][0-9]{0,3}", com_port):
        raise ValueError("Use an exact Windows COM port name such as COM10.")
    _require_live_dashboard()
    port = next((port for port in list_serial_ports() if port.device.upper() == com_port), None)
    if port is None:
        raise ValueError(f"The approved COM port {com_port} is not currently connected.")
    if type(port.vid) is not int or type(port.pid) is not int:
        raise ValueError(f"The approved COM port {com_port} does not report a USB vendor/product identity.")
    allowed_usb_ids = {(0x2341, None), (0x1A86, None), (0x10C4, None), (0x0403, None)}
    if not any(port.vid == vendor for vendor, _ in allowed_usb_ids):
        raise ValueError(f"The approved COM port {com_port} is not a supported USB serial controller.")
    return _submit_dashboard_request("connect", {
        "com_port": com_port,
        "usb_vendor_id": port.vid,
        "usb_product_id": port.pid,
    })


@mcp.tool()
def request_calibration(user_confirmation: str) -> dict:
    """Request calibration using an exact, recent user quote that states the pen is centered at origin. A separate dashboard click is required."""
    statement = _bounded_user_statement(user_confirmation)
    if not re.search(r"\bcenter(?:ed|ing)?\b", statement, re.IGNORECASE) or re.search(
        r"\b(?:not|isn't|isnt|never|uncentered|off[- ]center)\b.{0,32}\bcenter(?:ed|ing)?\b",
        statement,
        re.IGNORECASE,
    ):
        raise ValueError("Calibration requires an exact recent user statement that the plotter is centered at origin.")
    return _submit_dashboard_request("calibrate", {"user_confirmation": statement})


@mcp.tool()
def request_run(job_id: str, user_instruction: str) -> dict:
    """Request the exact current job using an exact, recent user instruction. The dashboard separately requires confirmation of this job and a clear motion area."""
    if not _is_request_id(job_id):
        raise ValueError("Invalid job ID.")
    instruction = _bounded_user_statement(user_instruction)
    if not re.search(r"\b(?:run|start|print|plot|draw|execute)\b", instruction, re.IGNORECASE) or re.search(
        r"\b(?:do\s+not|don't|dont|never|stop|cancel|avoid|without|no\s+need\s+to)\b.{0,40}\b(?:run|start|print|plot|draw|execute)\b",
        instruction,
        re.IGNORECASE,
    ):
        raise ValueError("Run request must contain a direct recent user instruction to run or print the job.")
    return _submit_dashboard_request("run", {"job_id": job_id, "user_instruction": instruction})


@mcp.tool()
def get_action_status(request_id: str) -> dict:
    """Read the status of a typed dashboard request; this tool cannot approve it."""
    if not _is_request_id(request_id):
        raise ValueError("Invalid request ID.")
    return dashboard_client.get_request(request_id)


def _bounded_user_statement(value: str) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 300:
        raise ValueError("An exact recent user statement of at most 300 characters is required.")
    return value.strip()


if __name__ == "__main__":
    mcp.run()
