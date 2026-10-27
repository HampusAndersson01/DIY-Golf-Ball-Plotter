from __future__ import annotations

import copy
import json
import re
import time
from collections import OrderedDict
from threading import RLock
from typing import Callable
from uuid import uuid4


class DashboardMcpBridgeError(ValueError):
    def __init__(self, message: str, *, code: str = "invalid_request", status_code: int = 400) -> None:
        super().__init__(message)
        self.code = code
        self.status_code = status_code


class DashboardMcpBridge:
    ALLOWED_ACTIONS = frozenset({"connect", "generate_diagnostic_job", "calibrate", "run"})
    MOTION_ACTIONS = frozenset({"calibrate", "run"})
    TERMINAL_STATUSES = frozenset({"completed", "rejected", "expired", "failed"})
    PENDING_STATUSES = frozenset({"queued", "awaiting_operator", "executing"})
    ALLOWED_DIAGNOSTIC_PATTERNS = frozenset({"3x3_squares", "x_axis_rotation_ticks"})
    MAX_RESULT_BYTES = 16 * 1024

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        heartbeat_ttl_seconds: float = 5.0,
        request_ttl_seconds: float = 120.0,
        max_pending_requests: int = 10,
    ) -> None:
        self._clock = clock
        self._heartbeat_ttl_seconds = max(1.0, float(heartbeat_ttl_seconds))
        self._request_ttl_seconds = max(1.0, float(request_ttl_seconds))
        self._max_pending_requests = max(1, int(max_pending_requests))
        self._lock = RLock()
        self._snapshot: dict | None = None
        self._snapshot_at: float | None = None
        self._requests: OrderedDict[str, dict] = OrderedDict()

    def publish_snapshot(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            raise DashboardMcpBridgeError("Dashboard snapshot must be an object.")
        snapshot_copy = self._copy_json_value(snapshot, "Dashboard snapshot")
        self._validate_snapshot(snapshot_copy)
        with self._lock:
            self._expire_requests_locked()
            self._snapshot = snapshot_copy
            self._snapshot_at = self._clock()
            self._expire_requests_locked()

    def get_snapshot(self) -> dict | None:
        with self._lock:
            self._expire_requests_locked()
            if not self._dashboard_is_available_locked():
                return None
            return copy.deepcopy(self._snapshot)

    def create_request(self, action: str, arguments: dict) -> dict:
        with self._lock:
            self._expire_requests_locked()
            snapshot = self._live_snapshot_locked()
            if snapshot is None:
                raise DashboardMcpBridgeError(
                    "Dashboard is not available; open the plotter dashboard and try again.",
                    code="dashboard_unavailable",
                    status_code=503,
                )

            clean_arguments = self._validate_action(action, arguments, snapshot)
            pending = [
                request for request in self._requests.values()
                if request["status"] in self.PENDING_STATUSES
            ]
            if len(pending) >= self._max_pending_requests:
                raise DashboardMcpBridgeError(
                    "Dashboard request queue is full.", code="queue_full", status_code=429
                )
            if action in self.MOTION_ACTIONS and any(
                request["action"] in self.MOTION_ACTIONS for request in pending
            ):
                raise DashboardMcpBridgeError(
                    "A motion request is already pending.", code="motion_request_pending", status_code=409
                )

            now = self._clock()
            request_id = str(uuid4())
            request = {
                "request_id": request_id,
                "action": action,
                "arguments": clean_arguments,
                "status": "queued",
                "created_at": now,
                "expires_at": now + self._request_ttl_seconds,
                "result": None,
            }
            self._requests[request_id] = request
            return copy.deepcopy(request)

    def get_next_dashboard_request(self) -> dict | None:
        with self._lock:
            self._expire_requests_locked()
            if not self._dashboard_is_available_locked():
                return None
            for request in self._requests.values():
                if request["status"] in {"queued", "awaiting_operator"}:
                    if not self._request_is_compatible_locked(request):
                        self._reject_request_locked(request, "Dashboard state changed before approval.")
                        continue
                    request["status"] = "awaiting_operator"
                    return copy.deepcopy(request)
            return None

    def complete_request(self, request_id: str, status: str, result: dict) -> dict:
        if not isinstance(status, str) or status not in self.TERMINAL_STATUSES:
            raise DashboardMcpBridgeError("Request completion status is invalid.")
        if not isinstance(result, dict):
            raise DashboardMcpBridgeError("Request result must be an object.")
        clean_result = self._copy_json_value(result, "Request result")
        with self._lock:
            self._expire_requests_locked()
            request = self._get_request_locked(request_id)
            if request["status"] not in self.PENDING_STATUSES:
                raise DashboardMcpBridgeError(
                    "Request is no longer pending.", code="request_not_pending", status_code=409
                )
            request["status"] = status
            request["result"] = clean_result
            return copy.deepcopy(request)

    def start_request(self, request_id: str) -> dict:
        with self._lock:
            self._expire_requests_locked()
            request = self._get_request_locked(request_id)
            if request["status"] != "awaiting_operator":
                raise DashboardMcpBridgeError(
                    "Request is not awaiting operator approval.", code="request_not_awaiting_operator", status_code=409
                )
            if not self._dashboard_is_available_locked() or not self._request_is_compatible_locked(request):
                self._reject_request_locked(request, "Dashboard state changed before execution.")
                raise DashboardMcpBridgeError(
                    "Dashboard state changed before execution.", code="state_conflict", status_code=409
                )
            request["status"] = "executing"
            return copy.deepcopy(request)

    def get_request(self, request_id: str) -> dict:
        with self._lock:
            self._expire_requests_locked()
            return copy.deepcopy(self._get_request_locked(request_id))

    def _validate_action(self, action: str, arguments: dict, snapshot: dict) -> dict:
        if not isinstance(action, str) or action not in self.ALLOWED_ACTIONS:
            raise DashboardMcpBridgeError("Unsupported dashboard action.")
        if not isinstance(arguments, dict):
            raise DashboardMcpBridgeError("Action arguments must be an object.")

        expected_keys = {
            "connect": {"com_port", "usb_vendor_id", "usb_product_id"},
            "generate_diagnostic_job": {"pattern"},
            "calibrate": {"user_confirmation"},
            "run": {"job_id", "user_instruction"},
        }[action]
        if set(arguments) != expected_keys:
            raise DashboardMcpBridgeError("Action arguments do not match the supported schema.")

        clean = self._copy_json_value(arguments, "Action arguments")
        machine = snapshot.get("machine") or {}
        if action == "connect":
            if machine.get("connected"):
                raise DashboardMcpBridgeError(
                    "Dashboard is already connected.", code="state_conflict", status_code=409
                )
            if not isinstance(clean.get("com_port"), str) or not re.fullmatch(r"COM[1-9][0-9]{0,3}", clean["com_port"]):
                raise DashboardMcpBridgeError("A valid COM port such as COM10 is required.")
            for key in ("usb_vendor_id", "usb_product_id"):
                if type(clean.get(key)) is not int or not 0 <= clean[key] <= 0xFFFF:
                    raise DashboardMcpBridgeError("A valid USB vendor and product ID are required.")
        elif action == "generate_diagnostic_job":
            if not isinstance(clean.get("pattern"), str) or clean.get("pattern") not in self.ALLOWED_DIAGNOSTIC_PATTERNS:
                raise DashboardMcpBridgeError("Unsupported diagnostic pattern.")
        elif action == "calibrate":
            statement = self._bounded_text(clean.get("user_confirmation"), "user confirmation")
            clean["user_confirmation"] = statement
            self._require_idle_connected(machine)
        elif action == "run":
            instruction = self._bounded_text(clean.get("user_instruction"), "user instruction")
            clean["user_instruction"] = instruction
            self._require_idle_connected(machine)
            if not machine.get("calibrated"):
                raise DashboardMcpBridgeError(
                    "Machine must be calibrated before a run.", code="state_conflict", status_code=409
                )
            job = snapshot.get("job") or {}
            if not job.get("job_id") or clean.get("job_id") != job.get("job_id"):
                raise DashboardMcpBridgeError(
                    "Generated job changed; inspect the current job before requesting a run.",
                    code="job_changed",
                    status_code=409,
                )
            if int(job.get("line_count") or 0) <= 0:
                raise DashboardMcpBridgeError(
                    "There is no generated job to run.", code="state_conflict", status_code=409
                )
        return clean

    @staticmethod
    def _bounded_text(value: object, label: str, limit: int = 300) -> str:
        if not isinstance(value, str) or not value.strip() or len(value) > limit:
            raise DashboardMcpBridgeError(f"A non-empty {label} of at most {limit} characters is required.")
        return value.strip()

    @staticmethod
    def _require_idle_connected(machine: dict) -> None:
        if not machine.get("connected"):
            raise DashboardMcpBridgeError(
                "Machine must be connected first.", code="state_conflict", status_code=409
            )
        if machine.get("running") or machine.get("paused"):
            raise DashboardMcpBridgeError(
                "Machine must be idle before this request.", code="state_conflict", status_code=409
            )
        if (machine.get("y_loop_test") or {}).get("enabled"):
            raise DashboardMcpBridgeError(
                "Stop the Y-axis loop before requesting this action.",
                code="state_conflict",
                status_code=409,
            )

    def _request_is_compatible_locked(self, request: dict) -> bool:
        snapshot = self._live_snapshot_locked()
        if snapshot is None:
            return False
        machine = snapshot.get("machine") or {}
        action = request["action"]
        if action == "connect":
            return not machine.get("connected")
        if action in self.MOTION_ACTIONS:
            if not machine.get("connected") or machine.get("running") or machine.get("paused"):
                return False
            if (machine.get("y_loop_test") or {}).get("enabled"):
                return False
        if action == "calibrate":
            return True
        if action == "run":
            job = snapshot.get("job") or {}
            return bool(
                machine.get("calibrated")
                and job.get("job_id") == request["arguments"].get("job_id")
                and int(job.get("line_count") or 0) > 0
            )
        return True

    def _expire_requests_locked(self) -> None:
        now = self._clock()
        dashboard_available = self._dashboard_is_available_locked()
        for request in self._requests.values():
            if request["status"] == "executing":
                if not dashboard_available:
                    request["status"] = "failed"
                    request["result"] = {
                        "message": "Dashboard heartbeat was lost during execution; check the plotter before continuing."
                    }
                continue
            if request["status"] not in {"queued", "awaiting_operator"}:
                continue
            if now >= request["expires_at"]:
                request["status"] = "expired"
                request["result"] = {"message": "Request expired before completion."}
            elif not dashboard_available:
                request["status"] = "expired"
                request["result"] = {"message": "Dashboard heartbeat was lost."}
            elif not self._request_is_compatible_locked(request):
                self._reject_request_locked(request, "Dashboard state changed before approval.")

    def _reject_request_locked(self, request: dict, message: str) -> None:
        request["status"] = "rejected"
        request["result"] = {"message": message}

    def _dashboard_is_available_locked(self) -> bool:
        return (
            self._snapshot is not None
            and self._snapshot_at is not None
            and self._clock() - self._snapshot_at <= self._heartbeat_ttl_seconds
        )

    def _live_snapshot_locked(self) -> dict | None:
        return self._snapshot if self._dashboard_is_available_locked() else None

    def _get_request_locked(self, request_id: str) -> dict:
        if not isinstance(request_id, str) or request_id not in self._requests:
            raise DashboardMcpBridgeError(
                "Dashboard request was not found.", code="request_not_found", status_code=404
            )
        return self._requests[request_id]

    @classmethod
    def _copy_json_value(cls, value: dict, label: str) -> dict:
        try:
            encoded = json.dumps(value, ensure_ascii=False, allow_nan=False)
            if len(encoded.encode("utf-8")) > cls.MAX_RESULT_BYTES:
                raise DashboardMcpBridgeError(f"{label} exceeds the size limit.")
            return json.loads(encoded)
        except (TypeError, ValueError) as exc:
            if isinstance(exc, DashboardMcpBridgeError):
                raise
            raise DashboardMcpBridgeError(f"{label} must contain JSON-compatible values.") from exc

    @staticmethod
    def _validate_snapshot(snapshot: dict) -> None:
        expected_root_keys = {"machine", "job", "logs", "serial_diagnostics"}
        if set(snapshot) != expected_root_keys:
            raise DashboardMcpBridgeError("Dashboard snapshot has an invalid shape.")
        machine = snapshot["machine"]
        expected_machine_keys = {
            "connected", "calibrated", "running", "paused", "status", "current_gcode_line", "y_loop_test"
        }
        if not isinstance(machine, dict) or set(machine) != expected_machine_keys:
            raise DashboardMcpBridgeError("Dashboard machine snapshot has an invalid shape.")
        if any(type(machine.get(key)) is not bool for key in ("connected", "calibrated", "running", "paused")):
            raise DashboardMcpBridgeError("Dashboard machine snapshot has invalid state flags.")
        if not isinstance(machine.get("status"), str) or len(machine["status"]) > 80:
            raise DashboardMcpBridgeError("Dashboard machine snapshot has an invalid status.")
        if type(machine.get("current_gcode_line")) is not int or machine["current_gcode_line"] < 0:
            raise DashboardMcpBridgeError("Dashboard machine snapshot has an invalid line number.")
        y_loop = machine.get("y_loop_test")
        if y_loop is not None and (
            not isinstance(y_loop, dict) or set(y_loop) != {"enabled"} or type(y_loop.get("enabled")) is not bool
        ):
            raise DashboardMcpBridgeError("Dashboard machine snapshot has invalid Y-loop state.")

        job = snapshot["job"]
        if not isinstance(job, dict) or set(job) != {"job_id", "line_count", "summary"}:
            raise DashboardMcpBridgeError("Dashboard job snapshot has an invalid shape.")
        job_id = job.get("job_id")
        if job_id is not None and (not isinstance(job_id, str) or not 1 <= len(job_id) <= 64):
            raise DashboardMcpBridgeError("Dashboard job snapshot has an invalid identity.")
        if type(job.get("line_count")) is not int or job["line_count"] < 0:
            raise DashboardMcpBridgeError("Dashboard job snapshot has an invalid line count.")
        summary = job.get("summary")
        summary_keys = {
            "image_size", "gcode_line_count", "estimated_runtime_seconds", "wall_path_count", "infill_path_count", "detail_trace_path_count"
        }
        if summary is not None and (
            not isinstance(summary, dict) or set(summary) != summary_keys
            or not isinstance(summary.get("image_size"), str)
            or len(summary["image_size"]) > 80
            or any(type(summary.get(key)) not in (int, float) for key in summary_keys - {"image_size"})
        ):
            raise DashboardMcpBridgeError("Dashboard job summary has an invalid shape.")

        logs = snapshot["logs"]
        if not isinstance(logs, list) or len(logs) > 50 or any(
            not isinstance(line, str) or len(line) > 500 or line.lstrip().startswith(">") for line in logs
        ):
            raise DashboardMcpBridgeError("Dashboard log snapshot has an invalid shape.")

        diagnostics = snapshot["serial_diagnostics"]
        diagnostic_keys = {
            "last_complete_parsed_line", "last_parsed_ack_line", "last_parsed_status_line", "last_ok_at", "last_status_at",
            "port_state", "unexpected_ok_count", "unexpected_error_count", "recent_grbl_lines",
        }
        if not isinstance(diagnostics, dict) or set(diagnostics) != diagnostic_keys:
            raise DashboardMcpBridgeError("Dashboard serial diagnostics have an invalid shape.")
        for key in ("last_complete_parsed_line", "last_parsed_ack_line", "last_parsed_status_line"):
            value = diagnostics.get(key)
            if value is not None and (not isinstance(value, str) or len(value) > 160):
                raise DashboardMcpBridgeError("Dashboard serial diagnostics have an invalid parsed line.")
        for key in ("last_ok_at", "last_status_at"):
            value = diagnostics.get(key)
            if value is not None and type(value) not in (int, float):
                raise DashboardMcpBridgeError("Dashboard serial diagnostics have an invalid timestamp.")
        for key in ("unexpected_ok_count", "unexpected_error_count"):
            if type(diagnostics.get(key)) is not int or diagnostics[key] < 0:
                raise DashboardMcpBridgeError("Dashboard serial diagnostics have invalid counters.")
        port_state = diagnostics.get("port_state")
        if not isinstance(port_state, dict) or set(port_state) != {"portOpen", "readerActive", "writerActive"} or any(
            type(flag) is not bool for flag in port_state.values()
        ):
            raise DashboardMcpBridgeError("Dashboard serial diagnostics have invalid port state.")
        recent_lines = diagnostics.get("recent_grbl_lines")
        if not isinstance(recent_lines, list) or len(recent_lines) > 20 or any(
            not isinstance(line, str) or len(line) > 160 for line in recent_lines
        ):
            raise DashboardMcpBridgeError("Dashboard serial diagnostics have an invalid recent line list.")
