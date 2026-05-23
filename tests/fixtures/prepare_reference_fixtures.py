from __future__ import annotations

import shutil
from pathlib import Path

from app.services.reference_infill_service import extract_gcode_from_3mf


ROOT = Path(__file__).resolve().parents[2]
FIXTURE_ROOT = ROOT / "tests" / "fixtures"
IMAGE_FIXTURE_DIR = FIXTURE_ROOT / "images"
REF_3MF_DIR = FIXTURE_ROOT / "reference_3mf"
REF_GCODE_DIR = FIXTURE_ROOT / "reference_gcode"


def main() -> int:
    IMAGE_FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    REF_3MF_DIR.mkdir(parents=True, exist_ok=True)
    REF_GCODE_DIR.mkdir(parents=True, exist_ok=True)

    root_png = ROOT / "ha-compact-lightbg.png"
    root_3mf = ROOT / "HA.3mf"
    if root_png.exists():
        shutil.copy2(root_png, IMAGE_FIXTURE_DIR / root_png.name)
        print(f"Copied image fixture: {root_png} -> {IMAGE_FIXTURE_DIR / root_png.name}")
    else:
        print(f"Image fixture source not found in root: {root_png}")

    if root_3mf.exists():
        fixture_3mf = REF_3MF_DIR / root_3mf.name
        shutil.copy2(root_3mf, fixture_3mf)
        print(f"Copied 3MF fixture: {root_3mf} -> {fixture_3mf}")
        extracted, inventory = extract_gcode_from_3mf(fixture_3mf, REF_GCODE_DIR)
        print(f"3MF archive entries: {len(inventory)}")
        if extracted is None:
            print("No embedded G-code file found in HA.3mf. Kept archive inventory for inspection.")
        else:
            print(f"Extracted embedded G-code: {extracted}")
    else:
        print(f"3MF source not found in root: {root_3mf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
