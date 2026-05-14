from __future__ import annotations

from . import pipeline_core


class ToolpathService:
    generate_toolpaths = staticmethod(pipeline_core.generate_toolpaths)
