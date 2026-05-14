from __future__ import annotations

from ._legacy import legacy


class ToolpathService:
    generate_toolpaths = staticmethod(legacy.generate_toolpaths)
