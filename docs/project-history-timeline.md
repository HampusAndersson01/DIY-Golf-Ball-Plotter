# Golf Ball Plotter Project History

## 1. High-level summary

Based on the Git history, this project became a fairly complete local control stack for a GRBL-driven golf ball plotter rather than just a few machine scripts. The repository starts as a small Windows/Python toolset for serial control and SVG-to-G-code work, then rapidly evolves into a structured Flask backend, a React/Vite dashboard, a raster and SVG toolpath pipeline, machine safety controls, GRBL streaming logic, and calibration diagnostics for both general print geometry and X-axis rotary behavior.

The strongest Git-backed story is the software systematization of the machine: turning one large plotter script into a tested application that can analyze artwork, generate previewable toolpaths, stream jobs safely to GRBL, control a pen servo, preserve calibration state, and help diagnose print accuracy. The hardware build itself is mostly not recorded here in source form, so that part should be described using external project context rather than Git evidence alone.

## 2. Chronological timeline

### 2026-05-13 - First repository baseline for local machine scripts

**Evidence from Git:**
- `bad96b3` - `Refactor code structure for improved readability and maintainability`
- Files changed: `.gitignore`, `README.md`, `cnc_test.py`, `cnc_web_controller.py`, `golf_ball_plotter_svg_gcode_runner.py`

**What happened:**
This is the first commit visible in Git and the best evidence for the repository starting point. The README describes the project as a "small collection of Python scripts" for a CNC plotter adapted for golf-ball artwork. The initial codebase included a serial test script, a local web controller, and a large SVG/G-code runner.

**Why it mattered:**
It established the project as a real machine-control software stack rather than a one-off experiment. Even at the first commit, the repo already had three clear concerns: connectivity testing, a control UI, and artwork-to-machine execution.

**What project milestone it represents:**
Initial software baseline for the plotter.

### 2026-05-13 - Consolidation around the main plotting runner

**Evidence from Git:**
- `8180e74` - `Add requirements.txt with essential dependencies for the project`
- `eb85ca5` - `Refactor code structure for improved readability and maintainability`
- Files changed: `requirements.txt`, `README.md`, `golf_ball_plotter_svg_gcode_runner.py`, `Arsenal.svg`, `artifacts/arsenal_classified_preview.svg`

**What happened:**
Later the same day, the repo became more focused. Dependencies were formalized, the old `cnc_web_controller.py` was archived and then removed, `cnc_test.py` was deleted, and work concentrated into `golf_ball_plotter_svg_gcode_runner.py`. The addition of `Arsenal.svg` and a classified preview artifact suggests practical artwork-processing tests, not just infrastructure work.

**Why it mattered:**
This looks like the first meaningful simplification of the software architecture: fewer parallel scripts, more emphasis on the actual SVG-to-toolpath workflow, and evidence of testing with real graphics.

**What project milestone it represents:**
First consolidation into a focused plotting pipeline with sample artwork.

### 2026-05-14 - Infill and travel optimization became a serious focus

**Evidence from Git:**
- `8c75254` - `Add infill density and travel optimization settings to slicer configuration`
- Files changed: `golf_ball_plotter_svg_gcode_runner.py`

**What happened:**
The main plotting script gained slicer-style controls for infill density and travel optimization. This is the first strong Git evidence that the project was moving beyond simple tracing and into the harder problem of making filled drawings practical on a curved object.

**Why it mattered:**
This is a key shift from "can the machine draw?" to "can it draw efficiently and cleanly?" Infill strategy directly affects print time, pen travel, visual quality, and whether the machine feels usable for real artwork.

**What project milestone it represents:**
First substantial print-strategy optimization work.

### 2026-05-14 - Monolithic scripts were refactored into a structured Flask application

**Evidence from Git:**
- `9b2b66a` - `feat: Add utility modules and response handling for Flask application`
- `4ba644d` - `Refactor code structure for improved readability and maintainability`
- Files changed: `app/__init__.py`, `app/config.py`, `app/routes/*`, `app/services/*`, `app/models/*`, `app/utils/*`, `pyproject.toml`, `run.py`, `tests/*`

**What happened:**
The project was reorganized into a proper application with routes, services, models, utilities, package metadata, and tests. Legacy logic was preserved behind wrappers rather than being thrown away immediately.

**Why it mattered:**
This was the turning point where the project became maintainable. It enabled later work on validation, raster processing, diagnostics, job safety, and UI changes without everything living inside one huge file.

**What project milestone it represents:**
Major software architecture refactor from script-based tooling to app-based system.

### 2026-05-14 - Raster artwork support and image analysis were added

**Evidence from Git:**
- `7152e43` - `Add raster analysis and image processing features`
- `17ba97f` - `feat: Enhance raster analysis and toolpath generation with thin detail mode`
- `c7719f3` - `feat: Implement derived pen defaults and enhance form handling in validation service ...`
- Files changed: `app/routes/raster_routes.py`, `app/services/raster_analysis_service.py`, `app/services/pipeline_core.py`, `app/services/validation_service.py`, `app/static/js/app.js`, `app/templates/index.html`, `tests/test_raster_analysis_service.py`

**What happened:**
The software gained raster image analysis, color quantization, mask generation, raster-to-G-code endpoints, and later a "thin detail mode" with additional fill/detail settings. At the same time, validation and UI controls grew to support more generation parameters and safer form handling.

**Why it mattered:**
This broadened the project from SVG-only input toward a more flexible artwork workflow. It also shows the project becoming more like a real operator tool, with configurable behavior instead of fixed hardcoded assumptions.

**What project milestone it represents:**
Expansion from vector plotting toward a more capable image-to-toolpath pipeline.

### 2026-05-14 - Toolpath quality and preview/debugging improved rapidly

**Evidence from Git:**
- `74db638` - `feat: Enhance preview controls and implement zoom functionality for mask and flat toolpath previews`
- `5bbf9d2` - `feat: Add boundary connector logic and improve toolpath generation for trapezoidal shapes`
- `a43901b` - `feat: Add support for pen-down infill connectors in toolpath generation and UI`
- `ba7e8e2` - `feat: Implement detail segment suppression in raster area fill toolpaths`
- `108c490` - `feat: Update default outline behavior after fill to true`
- Files changed: `app/services/pipeline_core.py`, `app/services/toolpath_service.py`, `app/static/js/app.js`, `app/templates/index.html`, `tests/test_toolpath_generation.py`

**What happened:**
The project added zoomable previews, better connector logic, pen-down infill linking, suppression of tiny detail segments, and a default behavior to run outline after fill. The tests also grew around toolpath generation, which suggests these changes were driven by real edge cases rather than cosmetic cleanup.

**Why it mattered:**
This was a practical print-quality phase. The code history strongly suggests active work on reducing travel waste, making filled regions cleaner, and keeping outlines/fills visually coherent.

**What project milestone it represents:**
First major toolpath-quality and operator-preview phase.

### 2026-05-14 - GRBL streaming reliability work appeared

**Evidence from Git:**
- `1583508` - `feat: Implement buffered streaming for G-code lines and enhance GRBL communication`
- Files changed: `app/services/job_runner.py`, `app/services/pipeline_core.py`, `app/services/serial_service.py`, `tests/test_grbl_streaming.py`

**What happened:**
The application gained buffered GRBL line streaming and dedicated streaming tests.

**Why it mattered:**
This is one of the clearest signs that the software was being hardened against real machine behavior. Streaming reliability is a practical milestone because bad line handling turns a plotter from "interesting demo" into "untrustworthy machine."

**What project milestone it represents:**
First explicit GRBL streaming robustness milestone.

### 2026-05-15 - Surface mapping was refined and the UI was rebuilt as React/Vite

**Evidence from Git:**
- `d931e3d` - `feat: Refactor geometry mapping and placement functions for improved surface handling`
- `d2c75ff` - `feat: add ToolpathLegend component and preview math utilities`
- `2d6aa9f` - `Refactor machine control components and improve UI`
- Files changed: `app/services/geometry_service.py`, `app/services/pipeline_core.py`, `frontend/*`, `dev.py`, `README.md`

**What happened:**
Geometry mapping and placement were refactored for surface handling, then the old template-based UI was replaced by a full React/Vite frontend. The new dashboard included preview tooling, app state, machine controls, calibration UI, G-code display, logs, and 3D/2D preview components. Manual control components were added shortly after.

**Why it mattered:**
This changed the project from a backend-with-page into a more serious operator console. It also indicates growing complexity: once geometry, previews, and machine state become central, a richer frontend becomes worth the investment.

**What project milestone it represents:**
Frontend/dashboard modernization and stronger preview tooling.

### 2026-05-17 - Machine safety and calibration-state handling became explicit

**Evidence from Git:**
- `7880286` - `feat: enhance infill settings and validation`
- `6a61e66` - `feat: add stepper hold policy testing and Y loop functionality`
- Files changed: `app/services/machine_service.py`, `app/services/job_runner.py`, `app/routes/machine_routes.py`, `frontend/src/components/machine/ManualControlCard.tsx`, `tests/test_machine_job_safety.py`

**What happened:**
The project expanded generation settings further, then added explicit stepper hold policy behavior, Y-loop test functionality, new machine state fields, and tests for machine job safety.

**Why it mattered:**
This is strong evidence that the project had moved into real machine-operations concerns: preserving calibration, verifying safe preconditions, and adding maintenance/test routines instead of only generating prettier G-code.

**What project milestone it represents:**
Machine safety, calibration locking, and maintenance diagnostics phase.

### 2026-05-18 - Debugging, alignment checking, and job lifecycle reliability matured

**Evidence from Git:**
- `e5d7257` - `feat: enhance toolpath debugging and projection handling with new alignment checks`
- `79c4cd0` - `Enhance logging and job management in job_runner, machine_service, and serial_service`
- `aee933c` - `Add tests for GcodeService and enhance toolpath generation validation`
- `5eafef1` - `feat: enhance DashboardApp and PenSettingsCard with advanced settings and improved UI elements`
- Files changed: `app/services/pipeline_core.py`, `app/services/job_runner.py`, `app/services/serial_service.py`, `app/logging_setup.py`, `tests/test_gcode_service.py`, `tests/test_toolpath_generation.py`, `frontend/src/components/image/PenSettingsCard.tsx`

**What happened:**
The code gained alignment checks, stronger projection debugging, more detailed job lifecycle handling, explicit logging across machine and job services, and broader test coverage for G-code behavior and toolpath validity.

**Why it mattered:**
This is the clearest evidence of "debugging the real system" rather than simply adding features. The project was becoming observable and auditable, which is important when physical machine motion must match preview geometry and emitted G-code.

**What project milestone it represents:**
Reliability and diagnostics hardening.

### 2026-05-20 - The geometry/debug contract was documented and validated

**Evidence from Git:**
- `613751f` - `Add tests for GRBL line reading and toolpath generation adjustments`
- Files changed: `CHATGPT_PROJECT_CONTEXT.md`, `app/routes/raster_routes.py`, `app/routes/svg_routes.py`, `app/services/pipeline_core.py`, `tests/test_gcode_service.py`, `tests/test_toolpath_generation.py`

**What happened:**
The repo gained a substantial internal project context document, more GRBL line-reading tests, and stronger assertions around toolpath regions, projected cleanup outlines, debug geometry, and preview/G-code consistency.

**Why it mattered:**
This suggests the project had reached the point where the main challenge was avoiding subtle regressions. The software now had explicit documentation for future maintainers and stronger tests around the most failure-prone geometry and streaming paths.

**What project milestone it represents:**
Stabilization and internal documentation phase.

### 2026-05-21 - Calibration diagnostics, scaling controls, and smarter infill were added

**Evidence from Git:**
- `7279453` - `feat(calibration): add 3x3 square calibration pattern generation and analysis`
- `6fa3d13` - `feat: implement artwork scaling functionality and validation across services and UI`
- `258583d` - `feat: Enhance infill processing and add new features`
- Files changed: `app/routes/raster_routes.py`, `app/routes/svg_routes.py`, `app/services/pipeline_core.py`, `frontend/src/components/calibration/*`, `frontend/src/components/image/PenSettingsCard.tsx`, `tests/test_diagnostic_calibration_routes.py`, `tests/test_toolpath_generation.py`

**What happened:**
The project added diagnostic calibration pattern generation, analysis for 3x3 printed squares, a dedicated X rotary test, artwork scaling controls, and more sophisticated infill scoring and normalization. The frontend also surfaced these tools in a more operator-friendly way.

**Why it mattered:**
This is the strongest Git-backed sign that the project had moved from basic function toward measurement-driven refinement. It is also the closest thing in the repo to explicit support for investigating physical print error, including X rotary under/over-travel, slip, backlash, eccentricity, and preview-versus-physical mismatch.

**What project milestone it represents:**
Measurement-driven calibration and advanced toolpath refinement.

## 3. Major milestones

- Initial script-based plotter software.
  Evidence: `bad96b3`
- Consolidation into a more focused SVG/G-code runner with sample artwork.
  Evidence: `8180e74`, `eb85ca5`
- First serious infill and travel optimization work.
  Evidence: `8c75254`
- Migration from monolithic scripts into a structured Flask app with tests.
  Evidence: `9b2b66a`, `4ba644d`
- Raster artwork analysis and thin-detail handling.
  Evidence: `7152e43`, `17ba97f`
- Toolpath cleanup, connector logic, and preview improvements.
  Evidence: `74db638`, `5bbf9d2`, `a43901b`, `ba7e8e2`, `108c490`
- Buffered GRBL streaming and communication hardening.
  Evidence: `1583508`
- React/Vite dashboard with richer preview and machine-control UI.
  Evidence: `d2c75ff`, `2d6aa9f`
- Machine safety, calibration lock, motor-hold policy, and Y-loop maintenance tests.
  Evidence: `6a61e66`
- Alignment/debugging hardening and stronger job lifecycle logging.
  Evidence: `e5d7257`, `79c4cd0`, `aee933c`
- Calibration diagnostics, artwork scaling, and smarter infill heuristics.
  Evidence: `7279453`, `6fa3d13`, `258583d`
- First CAD or mechanical design.
  Not directly visible in Git.
- Firmware / GRBL setup on Arduino Uno + CNC Shield.
  Not directly visible in Git as source or config files; only the software's GRBL assumptions are visible.
- Servo pen lift implementation on physical hardware.
  Partly visible in software through servo routes, config, and G-code generation, but the hardware wiring/mod itself is not directly visible in Git.
- Ball slippage discovery and grip redesign.
  Not directly visible as a confirmed event in commit messages or CAD files. Only later calibration diagnostics mention slip/backlash as possible causes.
- First successful permanent ink print.
  Not directly visible in Git.

## 4. Technical evolution

### Hardware design

The hardware itself is mostly outside the repository. There are no Fusion 360 source files, no STL/STEP/3MF exports, and no explicit mechanical revision history in Git. The best repository evidence is indirect: the software assumes a GRBL-controlled rotary plotting machine with calibration state, pen servo control, and X/Y motion handling. Hardware details such as the Arduino Uno, CNC Shield, servo through Z+ endstop, and 3D-printed structure come from external context, not from tracked files.

### Firmware / GRBL setup

The repo clearly targets GRBL operation over serial from the beginning. Over time, it becomes more explicit and more robust:

- Early scripts already assume serial-connected CNC control.
- By `9b2b66a`, GRBL-oriented app structure exists with machine routes and serial service wrappers.
- By `1583508`, buffered G-code streaming and GRBL line parsing are tested explicitly.
- By `79c4cd0`, logging and finalization behavior around GRBL communication become much stronger.
- By `613751f`, GRBL line-reading edge cases and streaming assumptions are being regression-tested.

There is no tracked GRBL firmware source, no `.ino`, and no repo-visible GRBL config dump. The firmware setup is therefore supported operationally by the software, but not versioned here as firmware code.

### Servo pen lift

Servo control is visible in the software even though the hardware implementation is not:

- Current project context describes pen servo control as a core workflow.
- `app/config.py` exposes servo dwell, servo ramp enable, servo ramp step, and ramp delay settings.
- `app/services/gcode_service.py` generates pen-up and pen-down behavior with optional ramping.
- Frontend controls for pen positions and pen actions exist in the React dashboard.
- Validation and defaults around pen settings grew over time, especially around `c7719f3`, `aee933c`, and later UI commits.

This suggests pen-lift behavior evolved from simple control toward more controlled motion timing and configurable values.

### G-code generation

G-code generation is the central technical thread in the repo history:

- It begins in the monolithic `golf_ball_plotter_svg_gcode_runner.py`.
- The 2026-05-14 refactor breaks generation into services and a legacy core.
- Raster and SVG generation both become first-class flows.
- Preview, debug payloads, and validation become increasingly important.
- Later commits focus on invariants such as generating fill/outline in surface-mm space first and projecting once into machine-degree space.

The history suggests the project gradually moved from "emit plausible G-code" toward "emit G-code that matches preview geometry and survives regression testing."

### Calibration

Calibration grows from an operator state into a diagnostic workflow:

- The machine model and routes include calibration locking and state.
- `6a61e66` adds tests and behavior around motor hold policy before/after calibration.
- `7279453` adds 3x3 square calibration output and X-axis rotary calibration output.
- The frontend gains analysis helpers that interpret measurement error patterns and suggest likely classes of issues.

This is a major maturity step because it turns calibration from a manual ritual into something the software can actively support.

### Print quality improvements

Print quality work is one of the clearest themes in the history:

- `8c75254` adds infill density and travel optimization.
- `17ba97f` introduces thin-detail mode.
- `5bbf9d2` improves boundary connectors for difficult shapes.
- `a43901b` adds pen-down infill connectors.
- `ba7e8e2` suppresses detail segments in area fill.
- `108c490` changes outline-after-fill defaults.
- `e5d7257`, `aee933c`, `613751f`, and `258583d` keep refining alignment, shared printable regions, and infill candidate selection.

The repo strongly supports a story of repeated iteration on how filled artwork behaves, especially around connector strategy, region consistency, and deterministic path generation.

### Mechanical grip / slippage

A confirmed mechanical redesign is not visible in Git. What is visible is later diagnostic code that explicitly names slip, backlash, eccentricity, and X rotary calibration error as likely causes when measured prints disagree with expected spacing:

- `7279453` adds X-axis calibration math and diagnosis strings for under-travel, over-travel, non-uniform quadrant spacing, cumulative slip, and backlash.

That is good evidence that mechanical slip became an important concern by this point, but it is not direct evidence of the first discovery date or of a grip redesign being completed in CAD.

### UI and software tools

The UI evolves in two phases:

- First, a server-rendered local dashboard with HTML/CSS/JavaScript.
- Then, on 2026-05-15, a major React/Vite dashboard migration with app state, toolpath legend, 2D/3D preview, logs, calibration UI, machine control, and advanced settings.

Later UI work keeps expanding diagnostics, settings visibility, and calibration tooling rather than just changing styling.

## 5. Problems and solutions

| Problem | Evidence in Git | Likely cause | Solution / attempted solution | Result |
|---|---|---|---|---|
| Outline and infill not sharing the same printable region | `aee933c`, `613751f`; tests mention fill walls, cleanup outlines, and shared printable region checks | Geometry generation and cleanup paths drifting from each other, or projection/debug mismatches | Added stronger tests, adjusted line width logic, and added checks so fill and outline paths share the same printable region | Strong evidence of an active fix/investigation path; exact physical print result is not directly proven in Git |
| Toolpaths wasting motion or leaving awkward fill behavior | `8c75254`, `5bbf9d2`, `a43901b`, `258583d` | Naive infill ordering, disconnected segments, poor candidate selection for long thin regions | Added infill density controls, travel optimization, boundary connectors, pen-down connectors, infill angle normalization, and segment scoring heuristics | Clearly improved in software; multiple follow-up commits imply the problem needed iterative refinement |
| Tiny detail segments creating noisy or low-value motion | `17ba97f`, `ba7e8e2` | Raster extraction producing many small fragments | Added thin-detail mode and detail-segment suppression for area-fill toolpaths | Likely reduced clutter and unnecessary motion; supported by tests |
| Pen up/down control needing safer or smoother handling | Current config and code plus `c7719f3`, `aee933c` | Servo timing sensitivity and need for consistent defaults | Added servo dwell/ramp configuration, validation, frontend controls, and G-code handling for ramped pen transitions | Well supported in software; hardware tuning outcome is not directly logged in Git |
| GRBL line handling and job streaming reliability | `1583508`, `79c4cd0`, `613751f` | Serial fragmentation, buffering issues, ambiguous finalization states | Added buffered streaming, line-reading tests, richer logging, and more explicit job finalization logic | Strong evidence of improved robustness and observability |
| Calibration state being easy to lose or misuse during machine handling | `6a61e66` and `tests/test_machine_job_safety.py` | Motors releasing at the wrong time, unsafe transitions around calibration or active jobs | Added stepper-hold policy management, calibration-aware behavior, and safety tests | Strong software evidence that calibration handling became more reliable |
| X rotary scale/slip/backlash uncertainty | `7279453`; `xAxisCalibrationMath.ts` explicitly mentions under-travel, over-travel, slip, backlash, eccentricity | Mechanical slip, backlash, incorrect X calibration ratio, or eccentric rotation | Added dedicated X rotary calibration pattern generation and measurement analysis | Strong evidence of diagnostic support; not proof that the mechanical root cause was fully solved in this repo |
| Physical print differing from preview/G-code expectations | `e5d7257`, `613751f`, `7279453` | Projection mismatch, debug mismatch, or real mechanical error | Added alignment checks, preview-vs-G-code invariants, diagnostic geometry bundles, and calibration metadata | Project became much better at isolating whether the error is software or hardware |

## 6. Good story points for a blog post

- "The first meaningful software milestone was turning a few local machine scripts into one focused SVG-to-G-code workflow."
- "A major turning point was when the project stopped being a giant script and became a proper app with routes, services, tests, and a real operator dashboard."
- "The project became more complex when raster artwork support, thin-detail handling, and slicer-style infill decisions were added."
- "A major debugging moment was realizing that preview geometry, printable regions, and emitted G-code all had to obey the same projection contract."
- "One of the most portfolio-worthy parts is the transition from 'generate something drawable' to 'generate something measurable, testable, and safe to run on a physical machine.'"
- "Another strong portfolio point is the calibration workflow: the software eventually generated dedicated calibration patterns and even included analysis for likely causes such as X under-travel, slip, backlash, or eccentricity."
- "The biggest engineering lesson visible in Git is that physical-machine software needs observability: logging, invariants, safety gating, and diagnostic tests become just as important as the geometry code."
- "The hardware story is strong, but the Git history mainly captures the software systematization of the machine rather than the full mechanical iteration."

## 7. Evidence list

| Date | Commit | Message | Why it matters |
|---|---|---|---|
| 2026-05-13 | `bad96b3` | Refactor code structure for improved readability and maintainability | First visible repository baseline |
| 2026-05-13 | `8180e74` | Add requirements.txt with essential dependencies for the project | Formalized dependencies and concentrated work into the runner |
| 2026-05-13 | `eb85ca5` | Refactor code structure for improved readability and maintainability | Added sample artwork and removed older controller path |
| 2026-05-14 | `8c75254` | Add infill density and travel optimization settings to slicer configuration | First clear infill/travel optimization milestone |
| 2026-05-14 | `9b2b66a` | feat: Add utility modules and response handling for Flask application | Converted the repo into a structured application |
| 2026-05-14 | `7152e43` | Add raster analysis and image processing features | Added raster workflow support |
| 2026-05-14 | `17ba97f` | feat: Enhance raster analysis and toolpath generation with thin detail mode | Added thin-detail handling and more generation sophistication |
| 2026-05-14 | `a43901b` | feat: Add support for pen-down infill connectors in toolpath generation and UI | Print quality and efficiency improvement |
| 2026-05-14 | `1583508` | feat: Implement buffered streaming for G-code lines and enhance GRBL communication | Major GRBL streaming robustness step |
| 2026-05-15 | `d2c75ff` | feat: add ToolpathLegend component and preview math utilities | Introduced the React/Vite dashboard |
| 2026-05-17 | `6a61e66` | feat: add stepper hold policy testing and Y loop functionality | Added calibration-aware machine safety tooling |
| 2026-05-18 | `79c4cd0` | Enhance logging and job management in job_runner, machine_service, and serial_service | Strong observability and job lifecycle hardening |
| 2026-05-20 | `613751f` | Add tests for GRBL line reading and toolpath generation adjustments | Stronger regression coverage around geometry and streaming |
| 2026-05-21 | `7279453` | feat(calibration): add 3x3 square calibration pattern generation and analysis | Added measurement-driven calibration tools |
| 2026-05-21 | `258583d` | feat: Enhance infill processing and add new features | Advanced infill heuristics at current HEAD |

## 8. Unknowns and assumptions

- The hardware work reportedly started around 2026-05-09, but the earliest visible Git commit is 2026-05-13.
- The TikTok / online-video inspiration is not visible in Git history.
- The fact that the design was created in Fusion 360 is not visible in Git because no Fusion files or CAD exports are tracked here.
- The Arduino Uno, CNC Shield, GRBL 1.1h Config B, and servo-through-Z+ details are not visible as tracked firmware/config files in the repository.
- The repo shows software support for pen servo control, but not the physical servo modification process itself.
- A grip redesign or confirmed ball-slippage fix is not directly shown in commit messages or CAD files. The strongest evidence is later calibration diagnostics that mention slip/backlash as likely causes.
- The exact date of the first successful physical drawing or first permanent-ink print is not visible in Git.
- Mechanical prototype iterations may exist outside Git in Fusion version history, photos, notes, or slicer files that are not checked into this repository.
- The repo does not contain a full firmware source tree, Arduino sketch, or GRBL configuration export, so firmware evolution cannot be reconstructed from Git here.
