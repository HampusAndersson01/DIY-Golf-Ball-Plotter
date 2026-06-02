# Engineering Roadmap

This roadmap is for implementation planning, not calendar planning.

The order below is based on dependency and engineering risk. Actual execution depends on how much time is available and how long each item takes in practice.

## Executive Summary

The project is already a working prototype with real machine control, a custom image-to-toolpath pipeline, GRBL streaming, calibration tooling, and a functional local dashboard.

The next meaningful work should focus on three areas:

1. Harden G-code generation so complex artwork either produces reliable output or fails safely with clear diagnostics.
2. Turn manual multi-color printing into a repeatable guided workflow instead of a loose sequence of operator steps.
3. Clean up the frontend dashboard so machine state, job state, calibration state, and next actions are easier to understand during actual use.

The most important sequencing rule is:

- Stabilize geometry and generation first
- Then build the multi-pass workflow on top of that
- Then clean up the dashboard around the real workflow that remains

## Roadmap Overview

| Phase | Focus | Why it comes now | Main areas |
|---|---|---|---|
| 0 | Baseline and observability | Prevents guessing and gives repeatable failures | `tests/`, debug payloads, reference fixtures |
| 1 | G-code generation hardening | Highest technical risk and blocks everything above it | `app/services/pipeline_core.py`, geometry cleanup, path planning |
| 2 | Manual multi-color workflow | Depends on stable pass generation and job summaries | backend state model, pass reuse, operator guidance |
| 3 | Frontend dashboard cleanup | Should reflect the actual workflow, not a temporary one | `frontend/src/App.tsx`, store structure, machine/job UX |

## Recommended Execution Order

Follow this order whenever you have time to work:

1. Baseline the failure cases.
2. Harden geometry and G-code generation until output is predictable.
3. Add an explicit multi-pass workflow with pass reuse and swap guidance.
4. Refactor and simplify the dashboard around the updated workflow.

If time is limited, stop at natural checkpoints:

- Checkpoint 1: known geometry failures are reproducible from tests or fixtures
- Checkpoint 2: hard artwork either generates correctly or fails safely
- Checkpoint 3: multi-color passes are cached, ordered, and reusable
- Checkpoint 4: dashboard state and control flow are clear enough for repeatable use

## Phase 0: Baseline And Observability

### Goals

- Capture the real failing artwork cases before changing algorithms.
- Make preview/G-code mismatches easier to inspect.
- Expand the existing regression harness instead of inventing new tooling from scratch.

### Specific technical tasks

1. Add regression fixtures for:
   - holes and islands
   - thin strokes
   - disconnected regions
   - overlap-heavy art
   - self-intersecting or invalid geometry
2. Expand `tests/test_toolpath_generation.py` with named fixture scenarios instead of only synthetic shapes.
3. Standardize debug payload fields from `pipeline_core` for:
   - geometry cleanup actions
   - warnings
   - preview/G-code parity
   - path counts and rejection reasons
4. Add a small artifact export path for difficult jobs so failed generation runs can be reviewed later.

### Priority order

1. Fixture capture
2. Regression assertions
3. Debug payload cleanup
4. Optional artifact export helpers

### Dependencies

- None

### Risks and blockers

- Physical print defects may still look like slicer bugs.
- If failing art is not captured as fixtures, later fixes will be hard to trust.

### Testing and validation

- Add or extend pytest coverage in `tests/test_toolpath_generation.py` and `tests/test_gcode_service.py`
- Save representative fixture inputs and expected diagnostic behavior
- Manually review a small set of known troublesome artworks

### Definition of done

- Known geometry failures are reproducible locally.
- Debug output is detailed enough to explain why a path was accepted, degraded, or rejected.

### Scope

- Must-have: reproducible fixtures, parity diagnostics, regression assertions
- Should-have: artifact export helpers
- Nice-to-have: batch comparison script for fixture runs

## Phase 1: G-code Generation Hardening

### Goals

- Make toolpath generation reliable across shape complexity levels.
- Preserve preview/G-code consistency.
- Fail safely when artwork exceeds what the current planner can handle well.

### Specific technical tasks

1. Add a geometry normalization stage before fill/outline planning:
   - repair invalid polygons
   - resolve self-intersections
   - union overlaps where appropriate
   - preserve holes and islands
   - prune unusable fragments
2. Add explicit output states:
   - accepted
   - accepted with degradation/warnings
   - rejected as too complex or too invalid
3. Harden infill and outline generation for:
   - holes and islands
   - narrow channels
   - disconnected regions
   - multi-interval scanline rows
4. Add thin-detail fallback behavior:
   - single-stroke fallback
   - outline-only fallback
   - reject with warning if below minimum printable width
5. Add planner bounds and fail-safes:
   - max path count
   - max segment explosion thresholds
   - min feature width thresholds
   - timeout or iteration guards for pathological inputs
6. Strengthen parity checks so preview and generated G-code stay aligned after projection and cleanup.

### Priority order

1. Invalid geometry and self-intersection cleanup
2. Hole/island and disconnected-region reliability
3. Thin-detail fallback rules
4. Complexity guards and safe rejection
5. Path planning and efficiency improvements

### Dependencies

- Phase 0 fixture coverage

### Risks and blockers

- Geometry repair may silently alter intended artwork.
- Over-cleanup can remove detail the operator expected to keep.
- Planner changes may improve one shape family while regressing another.

### Testing and validation

- Expand `tests/test_toolpath_generation.py`
- Expand `tests/test_gcode_service.py`
- Add preview/G-code hash or parity assertions
- Add coverage assertions that infill stays out of holes
- Run manual comparison on a representative artwork pack

### Definition of done

- Complex artwork either generates valid bounded output or returns a clear safe failure.
- Preview and emitted G-code agree on the projected drawing paths.
- Regressions are covered by tests.

### Scope

- Must-have: geometry cleanup, parity checks, safe rejection, regression coverage
- Should-have: better travel/path planning under complex shapes
- Nice-to-have: smarter artwork complexity scoring before full planning

## Phase 2: Manual Multi-Color Workflow

### Goals

- Make multi-color printing repeatable without requiring a hardware pen changer.
- Preserve operator context across pen swaps.
- Reduce avoidable mistakes between passes.

### Specific technical tasks

1. Introduce a multi-pass print-plan model:
   - one image
   - ordered color passes
   - shared placement assumptions
   - per-pass G-code and summary cache
2. Persist pass metadata:
   - selected color
   - pass index
   - generated hash
   - status
   - calibration assumptions
3. Add pass reuse rules so unchanged passes do not need regeneration.
4. Add a guided pen-swap state transition with explicit instructions before the next pass starts.
5. Add pre-pass diagnostics:
   - machine connected
   - calibration still locked
   - expected pass selected
   - current pass generated and not stale
6. Add support for:
   - re-run current pass
   - skip to pass N
   - invalidate only changed passes
7. Shape the data model so future semi-automation is possible without redesigning everything later.

### Priority order

1. Print-plan/pass model
2. Pass caching and stale-pass invalidation
3. Guided pen-swap flow
4. Pre-pass diagnostics
5. Re-run/resume controls

### Dependencies

- Stable single-pass generation and summaries from Phase 1

### Risks and blockers

- Real pen swaps can still introduce mechanical drift.
- The UI must not imply that calibration is guaranteed just because the pass state is preserved.

### Testing and validation

- Add backend tests for pass state transitions
- Add manual workflow validation for 2-color and 3-color jobs
- Confirm unchanged passes are reused correctly
- Confirm settings changes invalidate only the affected passes

### Definition of done

- Multi-color jobs can be prepared and executed as an ordered set of passes.
- Passes can be reused without ambiguous regeneration steps.
- The operator gets explicit guidance before each pass.

### Scope

- Must-have: pass model, pass cache, pre-pass checks, swap guidance
- Should-have: selective invalidation and pass resume/re-run behavior
- Nice-to-have: per-pass operator notes and richer diagnostics

## Phase 3: Frontend Dashboard Cleanup

### Goals

- Make the dashboard clearer during real machine operation.
- Reduce unsafe or confusing actions.
- Clean up the component and state structure without rewriting the whole UI unnecessarily.

### Specific technical tasks

1. Split orchestration out of `frontend/src/App.tsx` into clearer workflow-level containers or hooks.
2. Reorganize `frontend/src/store/appStore.ts` around actual domains:
   - machine state
   - generation state
   - preview state
   - calibration state
   - multi-pass job state
3. Rework the UI flow so the operator can move clearly through:
   - image import and color selection
   - placement and slicer setup
   - calibration
   - pass selection
   - run and monitoring
4. Improve project/job state visibility:
   - generated vs stale
   - current pass
   - calibration lock state
   - machine readiness
   - warnings and blocking conditions
5. Make machine-control UI safer:
   - disable invalid actions
   - separate stop vs reset vs clear calibration more clearly
   - improve wording around risky actions
6. Improve preview interaction:
   - current pass highlighting
   - warning banners for degraded generation
   - better slicer settings grouping
7. Improve logs and errors so failures are actionable rather than just noisy.

### Priority order

1. State clarity and workflow structure
2. Safety and readiness visibility
3. Slicer/settings layout cleanup
4. Preview and logs cleanup
5. Visual polish

### Dependencies

- Phase 2 pass/job model
- Phase 1 warning and diagnostics model

### Risks and blockers

- Refactoring top-level orchestration may create UI regressions.
- Cosmetic cleanup without state cleanup will not solve the real usability issues.

### Testing and validation

- `npm run build`
- `npm run lint`
- Manual workflow test:
  - connect
  - calibrate
  - generate
  - run
  - pause/resume
  - swap pen
  - continue next pass

### Definition of done

- The dashboard makes current machine state, current job/pass state, and next safe action obvious.
- The component structure is easier to extend without centralizing everything in `App.tsx`.

### Scope

- Must-have: workflow clarity, state cleanup, safer controls
- Should-have: improved preview and settings layout
- Nice-to-have: broader visual redesign

## Testing Strategy

### Backend and pipeline

- Keep pytest as the main regression gate.
- Expand fixture-based geometry tests first.
- Add explicit assertions for preview/G-code parity and hole-safe infill.

### Machine and workflow safety

- Extend machine/job tests for multi-pass and calibration-preservation flows.
- Keep safety behavior conservative when state is uncertain.

### Frontend

- At minimum enforce `npm run build` and `npm run lint`.
- Add focused UI tests later only where workflow state is easy to break.

### Manual validation

Use a small standard artwork pack:

- simple solid logo
- logo with hole/island
- thin-line art
- disconnected multi-part art
- multi-color artwork with at least 3 passes

For physical tests, keep a repeatable checklist for:

- calibration lock behavior
- pen swap behavior
- pass alignment
- stop/pause/resume safety

## Suggested GitHub Milestones And Issues

### Milestone: Geometry Hardening

- `pipeline_core: add geometry normalization and invalid-shape handling`
- `toolpath tests: add holes/islands/self-intersection regression fixtures`
- `gcode diagnostics: expose accepted/degraded/rejected generation status`
- `planner fail-safe: reject or degrade pathological artwork safely`
- `preview parity: add regression checks for projected path consistency`

### Milestone: Manual Multi-Color Workflow

- `backend: introduce multi-pass print-plan model`
- `backend: add per-pass cache and stale-pass invalidation`
- `frontend: add ordered pass list and pass status UI`
- `frontend: add pre-pass diagnostics and guided pen-swap flow`
- `workflow: support rerun/resume of a selected color pass`

### Milestone: Dashboard Cleanup

- `frontend: extract workflow orchestration from App.tsx`
- `frontend: reorganize Zustand store around machine/generation/pass domains`
- `frontend: improve machine-control safety states and wording`
- `frontend: redesign slicer settings layout around operator tasks`
- `frontend: improve warnings, logs, and stale-job visibility`

## Risks And Mitigation

| Risk | Why it matters | Mitigation |
|---|---|---|
| Geometry bugs are confused with mechanical slip | Can waste time fixing the wrong layer | Capture failing fixtures and use calibration diagnostics before changing planner logic |
| Pen swaps still disturb alignment | Software cannot fully solve mechanical drift | Add clear operator guidance, preserve state carefully, keep calibration assumptions explicit |
| Frontend refactor causes workflow regressions | Current orchestration is centralized | Refactor in slices and validate the full operator flow after each step |
| Scope expands into long-term automation | Can delay useful prototype improvements | Keep current roadmap focused on single-operator repeatability, not pen changer hardware |

## Immediate Next Work

If picking up the project right now, start here:

1. Capture the hardest current artwork failures as fixtures.
2. Harden geometry cleanup and add safe rejection for pathological shapes.
3. Add a real pass model for manual multi-color jobs.
4. Clean up the dashboard to reflect machine state, pass state, and safe actions more clearly.
