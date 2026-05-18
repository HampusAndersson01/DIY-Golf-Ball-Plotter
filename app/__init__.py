from flask import Flask

from .config import Config
from .extensions import init_extensions
from .logging_setup import get_log_path, setup_logging
from .routes import register_blueprints


def create_app(config_object: type[Config] = Config) -> Flask:
    app = Flask(__name__, template_folder="templates", static_folder="static")
    app.config.from_object(config_object)
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
