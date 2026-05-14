from flask import Flask

from .job_routes import job_bp
from .machine_routes import machine_bp
from .raster_routes import raster_bp
from .svg_routes import svg_bp
from .ui_routes import ui_bp


def register_blueprints(app: Flask) -> None:
    app.register_blueprint(ui_bp)
    app.register_blueprint(machine_bp)
    app.register_blueprint(raster_bp)
    app.register_blueprint(svg_bp)
    app.register_blueprint(job_bp)
