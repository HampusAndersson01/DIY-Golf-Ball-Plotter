from __future__ import annotations

from flask import jsonify


def json_ok(**payload):
    return jsonify({"ok": True, **payload})


def json_error(message: str, status: int = 400):
    return jsonify({"ok": False, "error": message}), status
