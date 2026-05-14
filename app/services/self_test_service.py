from __future__ import annotations

from . import pipeline_core


class SelfTestService:
    def run(self):
        return pipeline_core.run_integrated_svg_pipeline_self_test()
