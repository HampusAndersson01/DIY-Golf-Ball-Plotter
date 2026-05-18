from __future__ import annotations

from flask import current_app, jsonify, request


def json_ok(**payload):
    return jsonify({"ok": True, **payload})


def json_error(message: str, status: int = 400, **payload):
    return jsonify({"ok": False, "error": message, **payload}), status


def log_exception(message: str, exc: Exception, *, level: str = "exception", **context) -> None:
    logger = current_app.logger
    request_context = {
        "method": request.method,
        "path": request.path,
    }
    request_id = request.headers.get("X-Request-ID")
    if request_id:
        request_context["request_id"] = request_id
    request_context.update(context)
    log_method = getattr(logger, level)
    log_method("%s | context=%s | error=%s", message, request_context, exc, exc_info=True)
