from flask import Flask

from .config import Config
from .extensions import init_extensions
from .routes import register_blueprints


def create_app(config_object: type[Config] = Config) -> Flask:
    app = Flask(__name__, template_folder="templates", static_folder="static")
    app.config.from_object(config_object)
    init_extensions(app)
    register_blueprints(app)
    return app
