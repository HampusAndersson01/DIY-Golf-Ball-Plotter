import os
import secrets
import getpass
from pathlib import Path

from flask import Flask

from .config import Config
from .extensions import init_extensions
from .logging_setup import get_log_path, setup_logging
from .routes import register_blueprints


def create_app(config_object: type[Config] = Config) -> Flask:
    app = Flask(__name__, template_folder="templates", static_folder="static")
    app.config.from_object(config_object)
    _load_or_create_secret(app.config["DASHBOARD_MCP_TOKEN_FILE"])
    # Keep this key process-local: another local process running as the same
    # Windows user cannot be isolated from a readable secret file by ACLs.
    # Flask restart invalidates dashboard sessions and requires a fresh page.
    app.secret_key = secrets.token_urlsafe(48)
    app.config["SESSION_COOKIE_HTTPONLY"] = True
    app.config["SESSION_COOKIE_SAMESITE"] = "Strict"
    setup_logging(app.config["FLASK_DEBUG"])
    app.logger.handlers.clear()
    app.logger.propagate = True
    app.logger.setLevel("DEBUG" if app.config["FLASK_DEBUG"] else "INFO")
    app.logger.info(
        "Creating Flask app with host=%s port=%s debug=%s serial_port=%s baud_rate=%s log_file=%s",
        app.config["FLASK_HOST"],
        app.config["FLASK_PORT"],
        app.config["FLASK_DEBUG"],
        app.config["SERIAL_PORT"],
        app.config["BAUD_RATE"],
        get_log_path().resolve(),
    )

    @app.before_request
    def _log_request_start() -> None:
        from flask import request

        if request.path == "/state":
            return
        app.logger.info("Request started: %s %s", request.method, request.path)

    @app.after_request
    def _log_request_end(response):
        from flask import request

        if request.path == "/state":
            return response
        app.logger.info("Request completed: %s %s -> %s", request.method, request.path, response.status_code)
        return response

    init_extensions(app)
    register_blueprints(app)
    return app


def _load_or_create_secret(path_value: str) -> str:
    path = Path(path_value)
    if not path.is_absolute():
        path = Path.cwd() / path
    path.parent.mkdir(parents=True, exist_ok=True)
    secret = ""
    try:
        secret = path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        pass
    if len(secret) < 32 or len(secret) > 512:
        secret = secrets.token_urlsafe(48)
        temporary_path = path.with_name(f".{path.name}.{secrets.token_hex(8)}.tmp")
        try:
            with temporary_path.open("x", encoding="utf-8") as output:
                output.write(secret)
            os.replace(temporary_path, path)
        finally:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError:
                pass
    if os.name == "nt":
        # Restrict the bearer token to the account running the local server.
        # A process-local session key uses no file and needs no ACL.
        try:
            import subprocess

            subprocess.run(
                ["icacls", str(path), "/inheritance:r", "/grant:r", f"{getpass.getuser()}:(F)"],
                check=False,
                capture_output=True,
            )
        except OSError:
            pass
    else:
        try:
            path.chmod(0o600)
        except OSError:
            pass
    return secret
