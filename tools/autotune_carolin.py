from __future__ import annotations

import csv
import json
import math
import random
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from shapely.geometry import LineString

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
FIXTURE = ROOT / "tests" / "fixtures" / "images" / "Carolin Line.png"
OUT_ROOT = ROOT / "artifacts" / "carolin_autotune"

from app.models.machine_state import MachineState
from app.services import pipeline_core
from app.services.gcode_service import GcodeService
from app.services.geometry_service import GeometryService
from app.services.raster_analysis_service import RasterAnalysisService
from app.services.toolpath_service import ToolpathService
from tests.test_svg_parser import CONFIG


DRAW_KINDS = {
    "fill-infill",
    "fill-wall",
    "outline",
    "detail-trace",
    "coverage_centerline",
    "coverage_offset_line",
    "coverage_rectilinear",
    "coverage_tiny_mark",
    "coverage_contour",
    "coverage_connector",
    "outline_cleanup",
}


@dataclass
class Candidate:
    candidate_id: str
    pen_width_mm: float
    curve_flatten_tolerance_mm: float
    fill_strategy: str
    fill_spacing_mm: float
    outline_pass_enabled: bool
    min_feature_size_mm: float
    hatch_angle_deg: float
    simplify_tolerance_mm: float
    score: float = -1.0
    metrics: dict[str, float] | None = None
    error: str | None = None


def _load_baseline() -> tuple[bytes, np.ndarray, Any, tuple[float, float, float, float, float, float]]:
    raster = RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))
    geometry = GeometryService()
    image_bytes = FIXTURE.read_bytes()
    analysis = raster.analyze_image(image_bytes, max_colors=32)
    selected = next((c.id for c in analysis.colors if c.hex == "#000000"), analysis.colors[0].id)
    mask_result = raster.build_mask(
        image_bytes,
        [selected],
        tolerance=24,
        min_component_area_px=0,
        open_radius_px=0,
        close_radius_px=1,
    )
    regions = raster.extract_regions(mask_result, min_region_area_px=8, simplify_tolerance_px=0.35)
    mapped = geometry.map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)
    matrix = tuple(float(v) for v in mapped.metadata["connector_validation"]["current_to_source_matrix"])
    return image_bytes, mask_result.mask, mapped, matrix


def _rasterize_toolpaths(
    toolpaths: list[pipeline_core.Toolpath],
    matrix: tuple[float, float, float, float, float, float],
    shape: tuple[int, int],
    pen_width_mm: float,
    *,
    centerline_only: bool,
) -> np.ndarray:
    h, w = shape
    out = np.zeros((h, w), dtype=np.uint8)
    a, b, c, d, _e, _f = matrix
    px_per_mm = max(1e-6, (math.hypot(a, b) + math.hypot(c, d)) * 0.5)
    if centerline_only:
        pen_radius_px = 1.0
    else:
        pen_radius_px = max(0.75, (pen_width_mm * 0.5) * px_per_mm)
    radius_i = max(1, int(round(pen_radius_px)))
    sample_step_mm = max(0.01, min(pen_width_mm * 0.35, 0.05))

    for path in toolpaths:
        if path.kind not in DRAW_KINDS or len(path.points) < 1:
            continue
        if len(path.points) == 1:
            s = pipeline_core.apply_svg_matrix(path.points[0], matrix)
            cv2.circle(out, (int(round(s.x)), int(round(s.y))), radius_i, 255, -1)
            continue
        for p0, p1 in zip(path.points, path.points[1:]):
            line = LineString([(p0.x, p0.y), (p1.x, p1.y)])
            if line.length <= 1e-9:
                continue
            sample_count = max(2, int(math.ceil(line.length / sample_step_mm)) + 1)
            for i in range(sample_count):
                dmm = min(line.length, (line.length * i) / max(sample_count - 1, 1))
                p = line.interpolate(dmm)
                s = pipeline_core.apply_svg_matrix(pipeline_core.Point(float(p.x), float(p.y)), matrix)
                cv2.circle(out, (int(round(s.x)), int(round(s.y))), radius_i, 255, -1)
    return out


def _edge_alignment_score(target_mask: np.ndarray, drawn_mask: np.ndarray) -> float:
    target_edge = cv2.morphologyEx((target_mask.astype(np.uint8) * 255), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
    drawn_edge = cv2.morphologyEx((drawn_mask.astype(np.uint8) * 255), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
    if not target_edge.any() or not drawn_edge.any():
        return 0.0
    dt_target = cv2.distanceTransform((~target_edge).astype(np.uint8), cv2.DIST_L2, 3)
    dt_drawn = cv2.distanceTransform((~drawn_edge).astype(np.uint8), cv2.DIST_L2, 3)
    d1 = float(np.mean(dt_target[drawn_edge]))
    d2 = float(np.mean(dt_drawn[target_edge]))
    mean_dist = 0.5 * (d1 + d2)
    return float(math.exp(-mean_dist / 2.0))


def _path_cleanliness_score(toolpaths: list[pipeline_core.Toolpath], pen_width_mm: float) -> float:
    draw_paths = [p for p in toolpaths if p.kind in DRAW_KINDS and len(p.points) >= 2]
    if not draw_paths:
        return 0.0
    seg_lengths: list[float] = []
    sharp_turns = 0
    turn_total = 0
    tiny_fragments = 0
    for path in draw_paths:
        total_len = pipeline_core.segment_length(path.points)
        if total_len < max(0.15, pen_width_mm * 0.35):
            tiny_fragments += 1
        for p0, p1 in zip(path.points, path.points[1:]):
            seg_lengths.append(math.hypot(p1.x - p0.x, p1.y - p0.y))
        for p0, p1, p2 in zip(path.points, path.points[1:], path.points[2:]):
            v1 = (p1.x - p0.x, p1.y - p0.y)
            v2 = (p2.x - p1.x, p2.y - p1.y)
            n1 = math.hypot(*v1)
            n2 = math.hypot(*v2)
            if n1 <= 1e-9 or n2 <= 1e-9:
                continue
            dot = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
            turn = math.degrees(math.acos(dot))
            turn_total += 1
            if turn > 150.0:
                sharp_turns += 1
    tiny_ratio = tiny_fragments / max(1, len(draw_paths))
    sharp_ratio = sharp_turns / max(1, turn_total)
    short_seg_ratio = sum(1 for d in seg_lengths if d < max(0.03, pen_width_mm * 0.12)) / max(1, len(seg_lengths))
    penalty = min(1.0, 0.45 * tiny_ratio + 0.35 * sharp_ratio + 0.2 * short_seg_ratio)
    return float(max(0.0, 1.0 - penalty))


def _evaluate_candidate(
    mapped: Any,
    target_mask: np.ndarray,
    matrix: tuple[float, float, float, float, float, float],
    c: Candidate,
) -> Candidate:
    toolpath_service = ToolpathService()
    try:
        debug: dict[str, Any] = {}
        toolpaths = toolpath_service.generate_from_regions(
            mapped,
            pen_width_mm=c.pen_width_mm,
            wall_count=1,
            infill_pattern="hatch",
            infill_spacing_mm=c.fill_spacing_mm,
            infill_density=100.0,
            infill_angle_deg=c.hatch_angle_deg,
            fill_strategy=c.fill_strategy,
            alternate_fill_angle_deg=-45.0,
            outline_after_fill=c.outline_pass_enabled,
            min_region_area=0.0,
            min_fill_width_mm=c.min_feature_size_mm,
            simplify_tolerance_mm=c.simplify_tolerance_mm,
            remove_duplicate_paths=True,
            small_shape_mode="single-wall",
            thin_detail_mode=True,
            thin_detail_min_area_mm2=0.0,
            thin_detail_simplify_mm=c.curve_flatten_tolerance_mm,
            thin_detail_overlap=True,
            min_segment_length_mm=0.0,
            travel_optimization="nearest-neighbor",
            allow_pen_down_infill_connectors=True,
            infill_path_mode="rectilinear",
            debug=debug,
        )
        if not toolpaths:
            c.error = "no_toolpaths"
            return c

        metrics = pipeline_core.compute_toolpath_mask_coverage_metrics(
            toolpaths,
            mask=target_mask,
            current_to_source_matrix=matrix,
            pen_radius_mm=c.pen_width_mm * 0.5,
            sample_step_mm=max(0.01, min(c.pen_width_mm * 0.35, 0.05)),
            include_kinds=DRAW_KINDS,
        )
        if metrics is None:
            c.error = "no_metrics"
            return c

        target = target_mask > 0
        drawn = _rasterize_toolpaths(toolpaths, matrix, target_mask.shape, c.pen_width_mm, centerline_only=False) > 0
        inter = int(np.count_nonzero(target & drawn))
        union = int(np.count_nonzero(target | drawn))
        iou = inter / max(1, union)
        recall = inter / max(1, int(np.count_nonzero(target)))
        precision = inter / max(1, int(np.count_nonzero(drawn)))
        overspill_inverse = max(0.0, 1.0 - (metrics.outside_overdraw_percent / 100.0))
        edge_score = _edge_alignment_score(target, drawn)
        clean_score = _path_cleanliness_score(toolpaths, c.pen_width_mm)
        score = (
            0.35 * iou
            + 0.25 * recall
            + 0.20 * overspill_inverse
            + 0.10 * edge_score
            + 0.10 * clean_score
        )
        c.score = float(score)
        c.metrics = {
            "iou": float(iou),
            "recall": float(recall),
            "precision": float(precision),
            "outside_overdraw_percent": float(metrics.outside_overdraw_percent),
            "raw_coverage_percent": float(metrics.raw_coverage_percent),
            "penalized_coverage_percent": float(metrics.penalized_coverage_percent),
            "edge_alignment_score": float(edge_score),
            "path_cleanliness_score": float(clean_score),
        }
        return c
    except Exception as exc:
        c.error = str(exc)
        return c


def _candidate_to_row(c: Candidate) -> dict[str, Any]:
    row: dict[str, Any] = {
        "candidate_id": c.candidate_id,
        "pen_width_mm": c.pen_width_mm,
        "curve_flatten_tolerance_mm": c.curve_flatten_tolerance_mm,
        "fill_strategy": c.fill_strategy,
        "fill_spacing_mm": c.fill_spacing_mm,
        "outline_pass_enabled": c.outline_pass_enabled,
        "min_feature_size_mm": c.min_feature_size_mm,
        "hatch_angle_deg": c.hatch_angle_deg,
        "simplify_tolerance_mm": c.simplify_tolerance_mm,
        "score": c.score,
        "error": c.error or "",
    }
    if c.metrics:
        row.update(c.metrics)
    return row


def _build_coarse_candidates(base_pen: float) -> list[Candidate]:
    defs = [
        ("hybrid", 0.8, 0.75, False, 0.5, 0.0),
        ("hybrid", 1.2, 0.85, True, 0.25, 0.0),
        ("offset", 0.8, 0.75, False, 0.5, 0.0),
        ("offset", 1.2, 0.85, True, 0.25, 0.0),
        ("hatch", 0.8, 0.75, False, 0.5, 45.0),
        ("hatch", 1.2, 0.85, True, 0.25, 90.0),
    ]
    out: list[Candidate] = []
    for idx, (strategy, pf, sf, outline, mf, ang) in enumerate(defs, start=1):
        pen = round(base_pen * pf, 4)
        tol = round(min(0.05, max(0.01, pen / 8.0)), 4)
        out.append(
            Candidate(
                candidate_id=f"C{idx:04d}",
                pen_width_mm=pen,
                curve_flatten_tolerance_mm=tol,
                fill_strategy=strategy,
                fill_spacing_mm=round(max(0.01, pen * sf), 4),
                outline_pass_enabled=outline,
                min_feature_size_mm=round(max(0.0, pen * mf), 4),
                hatch_angle_deg=float(ang),
                simplify_tolerance_mm=tol,
            )
        )
    return out


def _build_fine_candidates(top: list[Candidate]) -> list[Candidate]:
    out: list[Candidate] = []
    idx = 1
    for seed in top[:2]:
        pen_vals = sorted(set([seed.pen_width_mm * 0.92, seed.pen_width_mm, seed.pen_width_mm * 1.08]))
        spacing_vals = sorted(set([seed.fill_spacing_mm * 0.92, seed.fill_spacing_mm, seed.fill_spacing_mm * 1.08]))
        tol_vals = sorted(set([
            max(0.005, seed.curve_flatten_tolerance_mm * 0.75),
            seed.curve_flatten_tolerance_mm,
            min(0.05, seed.curve_flatten_tolerance_mm * 1.25),
        ]))
        minf_vals = sorted(set([max(0.0, seed.min_feature_size_mm * 0.75), seed.min_feature_size_mm, seed.min_feature_size_mm * 1.25]))
        angles = [seed.hatch_angle_deg]
        if seed.fill_strategy == "hatch":
            angles = sorted(set([seed.hatch_angle_deg, (seed.hatch_angle_deg + 45.0) % 180.0]))
        for pen in pen_vals:
            for spacing in spacing_vals:
                for tol in tol_vals:
                    for minf in minf_vals:
                        for outline in [seed.outline_pass_enabled, not seed.outline_pass_enabled]:
                            for ang in angles:
                                out.append(
                                    Candidate(
                                        candidate_id=f"F{idx:04d}",
                                        pen_width_mm=round(pen, 4),
                                        curve_flatten_tolerance_mm=round(tol, 4),
                                        fill_strategy=seed.fill_strategy,
                                        fill_spacing_mm=round(max(0.01, spacing), 4),
                                        outline_pass_enabled=outline,
                                        min_feature_size_mm=round(max(0.0, minf), 4),
                                        hatch_angle_deg=float(ang),
                                        simplify_tolerance_mm=round(tol, 4),
                                    )
                                )
                                idx += 1
    uniq: dict[tuple[Any, ...], Candidate] = {}
    for c in out:
        k = (
            c.pen_width_mm,
            c.curve_flatten_tolerance_mm,
            c.fill_strategy,
            c.fill_spacing_mm,
            c.outline_pass_enabled,
            c.min_feature_size_mm,
            c.hatch_angle_deg,
            c.simplify_tolerance_mm,
        )
        uniq[k] = c
    candidates = list(uniq.values())
    random.Random(7).shuffle(candidates)
    return candidates[:4]


def _render_and_save_candidate_artifacts(
    out_dir: Path,
    name: str,
    target_mask: np.ndarray,
    matrix: tuple[float, float, float, float, float, float],
    toolpaths: list[pipeline_core.Toolpath],
    pen_width_mm: float,
) -> None:
    target = target_mask > 0
    pen = _rasterize_toolpaths(toolpaths, matrix, target_mask.shape, pen_width_mm, centerline_only=False) > 0
    trace = _rasterize_toolpaths(toolpaths, matrix, target_mask.shape, pen_width_mm, centerline_only=True) > 0
    overlay = np.full((target_mask.shape[0], target_mask.shape[1], 3), 255, dtype=np.uint8)
    overlay[target] = (225, 225, 225)
    overlay[pen] = (255, 140, 0)
    cv2.imwrite(str(out_dir / f"{name}_pen_preview.png"), overlay)
    trace_img = np.full((target_mask.shape[0], target_mask.shape[1], 3), 255, dtype=np.uint8)
    trace_img[target] = (235, 235, 235)
    trace_img[trace] = (255, 140, 0)
    cv2.imwrite(str(out_dir / f"{name}_trace_preview.png"), trace_img)


def main() -> None:
    if not FIXTURE.exists():
        raise SystemExit(f"Fixture not found: {FIXTURE}")
    run_dir = OUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    _img, target_mask, mapped, matrix = _load_baseline()
    base_pen = max(0.25, float(CONFIG.get("DEFAULT_LINE_THICKNESS_MM", 0.6)), 0.6)

    coarse = _build_coarse_candidates(base_pen)
    coarse_results: list[Candidate] = []
    t0 = time.time()
    for c in coarse:
        coarse_results.append(_evaluate_candidate(mapped, target_mask, matrix, c))
        if len(coarse_results) % 20 == 0:
            print(f"[coarse] {len(coarse_results)}/{len(coarse)} elapsed={time.time()-t0:.1f}s", flush=True)

    coarse_sorted = sorted(coarse_results, key=lambda c: c.score if c.error is None else -1.0, reverse=True)
    coarse_top = [c for c in coarse_sorted if c.error is None][:5]

    fine = _build_fine_candidates(coarse_top)
    fine_results: list[Candidate] = []
    t1 = time.time()
    for c in fine:
        fine_results.append(_evaluate_candidate(mapped, target_mask, matrix, c))
        if len(fine_results) % 20 == 0:
            print(f"[fine] {len(fine_results)}/{len(fine)} elapsed={time.time()-t1:.1f}s", flush=True)

    all_results = coarse_results + fine_results
    valid = [c for c in all_results if c.error is None]
    if not valid:
        err_path = run_dir / "errors.json"
        err_path.write_text(json.dumps([_candidate_to_row(c) for c in all_results], indent=2), encoding="utf-8")
        raise RuntimeError(f"No valid candidates; see {err_path}")
    best = sorted(valid, key=lambda c: c.score, reverse=True)[0]

    csv_path = run_dir / "tuning_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        rows = [_candidate_to_row(c) for c in sorted(all_results, key=lambda x: x.score if x.error is None else -1.0, reverse=True)]
        writer = csv.DictWriter(f, fieldnames=sorted({k for row in rows for k in row.keys()}))
        writer.writeheader()
        writer.writerows(rows)

    # Rebuild best toolpaths + final G-code
    toolpath_service = ToolpathService()
    gcode_service = GcodeService()
    debug: dict[str, Any] = {}
    best_toolpaths = toolpath_service.generate_from_regions(
        mapped,
        pen_width_mm=best.pen_width_mm,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=best.fill_spacing_mm,
        infill_density=100.0,
        infill_angle_deg=best.hatch_angle_deg,
        fill_strategy=best.fill_strategy,
        alternate_fill_angle_deg=-45.0,
        outline_after_fill=best.outline_pass_enabled,
        min_region_area=0.0,
        min_fill_width_mm=best.min_feature_size_mm,
        simplify_tolerance_mm=best.simplify_tolerance_mm,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.0,
        thin_detail_simplify_mm=best.curve_flatten_tolerance_mm,
        thin_detail_overlap=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=True,
        infill_path_mode="rectilinear",
        debug=debug,
    )
    gcode, preview = gcode_service.generate_from_toolpaths(
        toolpaths=best_toolpaths,
        draw_feed=1200.0,
        travel_feed=3000.0,
        sample_step_deg=1.0,
        pen_up_s=575,
        pen_down_s=700,
        servo_ramp_enabled=True,
        servo_ramp_step=20,
        servo_ramp_delay_ms=10.0,
        pen_up_dwell_ms=30.0,
        pen_down_dwell_ms=60.0,
        gcode_mode="simple",
        include_comments=True,
        debug=debug,
    )

    _render_and_save_candidate_artifacts(run_dir, "best", target_mask, matrix, best_toolpaths, best.pen_width_mm)
    (run_dir / "final.gcode").write_text("\n".join(gcode) + "\n", encoding="utf-8")
    (run_dir / "best_preview.json").write_text(json.dumps(preview, indent=2), encoding="utf-8")

    top10 = [c for c in sorted(valid, key=lambda c: c.score, reverse=True)[:10]]
    (run_dir / "top_candidates.json").write_text(json.dumps([_candidate_to_row(c) for c in top10], indent=2), encoding="utf-8")
    summary = {
        "run_dir": str(run_dir),
        "coarse_count": len(coarse_results),
        "fine_count": len(fine_results),
        "valid_count": len(valid),
        "best": _candidate_to_row(best),
        "results_csv": str(csv_path),
        "best_gcode": str(run_dir / "final.gcode"),
        "best_pen_preview": str(run_dir / "best_pen_preview.png"),
        "best_trace_preview": str(run_dir / "best_trace_preview.png"),
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
