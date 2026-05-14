from __future__ import annotations

from . import pipeline_core


class SvgParser:
    def __init__(self, config, state) -> None:
        self._config = config
        self._state = state
        pipeline_core.configure_runtime(config, state.raw, pipeline_core.serial_lock)

    def analyze_svg(self, svg_text: str, **kwargs):
        return pipeline_core.analyze_svg(svg_text, **kwargs)

    def extract_svg_bundle(self, svg_text: str, **kwargs):
        return pipeline_core.extract_svg_bundle(svg_text, **kwargs)
