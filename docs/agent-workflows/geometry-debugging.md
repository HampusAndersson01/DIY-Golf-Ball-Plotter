# Geometry Debugging

This repository’s geometry bugs are only fixed when the emitted preview, toolpath, or G-code changes. Metadata-only adjustments are not enough.

## Trace the active pipeline

Start from the source input and follow the active path:

- Artwork ingestion
- Raster or SVG analysis
- Geometry cleanup and projection
- Toolpath generation
- G-code emission
- Preview rendering

The active code path usually flows through `app/services/pipeline_core.py` and then into the more specialized services such as `geometry_service.py`, `toolpath_service.py`, `raster_analysis_service.py`, and `gcode_service.py`.

## Active vs unused code paths

- Prefer the code path that actually feeds preview and G-code.
- Do not fix helpers that are no longer called by the live pipeline.
- If a helper exists only for historical compatibility, confirm it still affects output before changing it.

## Regression test strategy

- Add or update a test that exercises the live pipeline.
- Assert visible geometry output, not just helper return values.
- Cover preview-to-G-code consistency when the bug is about output mismatch.
- Keep tests close to the failure mode so future regressions are obvious.

## Coverage verification

When a geometry fix is made, verify the output path that changed:

- Preview coverage
- Outline coverage
- Infill or detail coverage
- Hole preservation
- Narrow passage preservation
- G-code path continuity

## Overflow verification

Use a proof-focused check when geometry risks overflowing the intended band or shape:

- Verify that offsets stay within the drawable area.
- Check that the pen footprint does not silently clip a visible region.
- Confirm that a collapsed contour falls back to a centerline or detail path instead of disappearing.

## Narrow passage handling

For narrow bridges, thin stems, and tight corridors:

- Keep the visible feature if the pen footprint can still traverse it safely.
- Prefer a centerline or detail fallback when an offset contour becomes too fragile.
- Do not drop the segment just because it is small.

## Centerline fallback handling

If an offset shape collapses:

- Fall back to a centerline or other detail-preserving path.
- Make the fallback visible in preview and present in emitted G-code.
- Add a regression test that proves the fallback path is used.

## Thin outline handling

For thin outlines and borders:

- Remember that pen radius is derived from the physical pen width.
- Inset outline centerlines by pen radius unless the task explicitly proves a different behavior is required.
- Keep thin visible marks in the active pipeline.

## Holes and slits

- Verify that holes remain holes when they should remain open.
- Verify that slits do not get sealed off by over-aggressive cleanup.
- Check both the preview and the exported toolpath.

## Acute triangles

Acute corners can collapse during offsetting or simplification.

- Confirm the visible corner survives the pipeline.
- Check whether a reduced-detail fallback is needed.
- Prefer a visible, testable path over a silent disappearance.

## Proof requirements for visible fixes

A geometry fix is not complete until you can show:

1. The input artifact.
2. The active code path that was changed.
3. The preview or G-code before the fix.
4. The preview or G-code after the fix.
5. The regression test that protects the behavior.
