from __future__ import annotations

from ._legacy import legacy


class SelfTestService:
    def run(self):
        return legacy.run_integrated_svg_pipeline_self_test()
