from __future__ import annotations

from ._legacy import configure_runtime, legacy


class SvgParser:
    def __init__(self, config, state) -> None:
        self._config = config
        self._state = state
        configure_runtime(config, state.raw, legacy.serial_lock)

    def analyze_svg(self, svg_text: str, **kwargs):
        return legacy.analyze_svg(svg_text, **kwargs)

    def extract_svg_bundle(self, svg_text: str, **kwargs):
        return legacy.extract_svg_bundle(svg_text, **kwargs)
