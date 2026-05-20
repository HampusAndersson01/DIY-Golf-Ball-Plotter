# Golfball Printer Project Context

## What This Project Is

This repository is a local control stack for a GRBL-driven golf ball plotter.

It does two related jobs:

1. Convert SVG or raster artwork into toolpaths and then into G-code that fits a golf ball surface.
2. Control the physical machine over serial, including calibration, jogging, pen servo control, job streaming, pause/resume/stop, and motor hold policy.

The app is intended for a trusted local workflow, not an internet-exposed deployment.

## Tech Stack

- Backend: Flask, Python 3.11+
- Frontend: React + TypeScript + Vite
- Geometry/path work: `shapely`, `svgpathtools`
- Raster analysis: `opencv-python-headless`, `Pillow`, `numpy`
- Machine connection: `pyserial`
- Tests: `pytest`

Key Python package metadata is in [pyproject.toml](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/pyproject.toml:1).

## High-Level Architecture

### Backend app structure

- [run.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/run.py:1): starts the Flask backend.
- [dev.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/dev.py:1): starts Flask and the Vite frontend together for local development.
- [app/__init__.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/__init__.py:1): Flask app factory, logging setup, and blueprint registration.
- [app/extensions.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/extensions.py:1): dependency wiring for state and services.
- [app/config.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/config.py:1): environment-driven configuration and defaults.

### Route groups

- [app/routes/ui_routes.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/routes/ui_routes.py:1): `/`, `/api/bootstrap`, `/state`
- [app/routes/machine_routes.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/routes/machine_routes.py:1): machine connection, config, jog, pen, calibration, stepper hold, Y-loop test
- [app/routes/job_routes.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/routes/job_routes.py:1): run, pause, resume, stop
- [app/routes/svg_routes.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/routes/svg_routes.py:1): SVG analysis and SVG-to-G-code generation
- [app/routes/raster_routes.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/routes/raster_routes.py:1): raster color analysis and raster-to-G-code generation

### Core services

- `ValidationService`: parses and validates request payloads.
- `SvgParser`: converts SVG into normalized printable geometry.
- `RasterAnalysisService`: quantizes colors, builds masks, extracts printable regions.
- `GeometryService`: maps source geometry into surface-mm coordinates and applies placement/rotation.
- `ToolpathService`: generates outline, wall, infill, detail-trace, and travel toolpaths.
- `GcodeService`: converts projected toolpaths into G-code and preview paths.
- `SerialService`: GRBL connection, commands, and streaming.
- `MachineService`: higher-level machine actions and safety policies.
- `JobRunner`: background print execution, progress tracking, and finalization.

### Important legacy/core module

[app/services/pipeline_core.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/services/pipeline_core.py:1) is the main extracted legacy core. It still contains a large amount of the real geometry, projection, parsing, streaming, and G-code logic. Many service methods are wrappers around this module.

If a future change affects geometry, projection, fill generation, pen motion, or GRBL line handling, assume `pipeline_core.py` is part of the critical path even if the route or service layer looks small.

## Frontend Model

The frontend is a single dashboard app under `frontend/`.

Primary files:

- [frontend/src/store/appStore.ts](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/frontend/src/store/appStore.ts:1): Zustand store for machine state, settings, image analysis, preview, G-code, logs, and UI state.
- [frontend/src/api/client.ts](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/frontend/src/api/client.ts:1): API client for backend endpoints.
- [frontend/src/api/types.ts](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/frontend/src/api/types.ts:1): shared frontend-side contract assumptions.

The frontend centers around:

- Loading defaults from `/api/bootstrap`
- Polling `/state`
- Uploading an image and selecting colors
- Generating preview + G-code
- Running and monitoring the job
- Manual machine controls, calibration, and maintenance/test operations

## Core Data Flow

### Raster workflow

1. Frontend uploads image to `/analyze-image-colors`
2. Backend returns image size and detected/simplified colors
3. User selects colors and generation settings
4. Frontend posts to `/generate-image-gcode`
5. Backend:
   - builds a printable mask from selected colors
   - extracts connected regions and thin detail traces
   - maps geometry into surface-mm space
   - applies placement scale and rotation
   - generates toolpaths
   - cleans/simplifies toolpaths
   - projects surface-mm paths once into machine-degree space
   - generates G-code plus preview paths
6. Backend stores `last_gcode`, `last_preview`, and `last_summary` in machine state
7. Frontend can then run the generated job

### SVG workflow

1. Frontend uploads SVG to `/generate-gcode` or `/analyze-svg`
2. Backend parses visible geometry from SVG
3. Dark fills become printable geometry; light/transparent fills can become cutouts depending on classification
4. Strokes can remain trace paths
5. Geometry is mapped to surface-mm, placed, projected to machine degrees, then converted to G-code

### Print execution workflow

1. Machine must be connected and calibrated
2. Generated G-code is stored in `MachineState.last_gcode`
3. `/run-gcode` starts `JobRunner` in a background thread
4. `JobRunner` streams only streamable G-code lines to GRBL
5. Progress is reflected into `/state`
6. On finish or interruption, finalization attempts:
   - pen up
   - return home when safe
   - preserve or reapply the motor hold policy

## Coordinate Model

This project uses an explicit two-step spatial model:

1. Source artwork is normalized into a flat surface coordinate system in millimeters.
2. Surface-mm geometry is projected once into machine coordinates measured in degrees on the ball mechanism.

Both raster and SVG generation routes build debug payloads that emphasize this contract. The intended invariant is: generate fills/outlines in surface-mm space, then project to machine degrees exactly once.

This matters because many regressions in this codebase would come from:

- projecting too early
- projecting twice
- mixing surface-mm and machine-degree toolpaths
- misaligning preview and emitted G-code

## Machine and Safety Model

The machine has two important states:

- connection state
- calibration lock state

Calibration means the operator has physically aligned the pen to the ball center and marked that as origin. Running a job is blocked until calibrated.

### Stepper hold policy

`MachineService` manages a GRBL `$1` step idle delay policy:

- before calibration: release motors for easy manual movement
- after calibration: hold motors to preserve the trusted origin

This behavior is encapsulated in `StepperHoldPolicyManager` inside [app/services/machine_service.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/services/machine_service.py:1).

The policy can be deferred if streaming is active, and it performs readback verification via `$$`.

### Job finalization rules

`JobRunner.finalize_job()` distinguishes between:

- successful completion
- user stop
- timeout / connection loss
- GRBL error
- unknown interruption

It does not blindly mark success just because the machine is idle. It uses a completion guard based on:

- total lines
- sent lines
- acked lines
- empty pending queue
- empty pending serial buffer
- GRBL idle state
- no abort flag
- no error

This is a critical safety/correctness feature.

### Extra maintenance/test behavior

There is also a Y-axis current test loop that:

- requires connected + calibrated + no active job
- oscillates around the current Y center
- keeps motor hold expectations explicit in state

## State Model

[app/models/machine_state.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/models/machine_state.py:1) is the backend source of truth.

Important fields:

- connection/calibration flags
- machine position trust flags
- current servo value
- current machine X/Y estimate
- last generated G-code and preview
- last job summary
- streaming counters and GRBL status
- job finalization details
- motor hold policy state
- Y-loop maintenance state

The frontend relies heavily on `/state` and treats it as the runtime truth for job progress and machine readiness.

## Important API Endpoints

### UI/bootstrap/state

- `GET /`
- `GET /api/bootstrap`
- `GET /state`

### Machine control

- `POST /connect`
- `POST /apply-config`
- `POST /command`
- `POST /reset`
- `POST /pen-up`
- `POST /pen-down`
- `POST /pen-test`
- `POST /servo-off`
- `POST /jog`
- `POST /zero-position`
- `POST /zero-and-mark-calibrated`
- `POST /go-home`
- `POST /mark-calibrated`
- `POST /clear-calibrated`
- `POST /stepper-hold/apply`
- `POST /y-loop/start`
- `POST /y-loop/stop`

### Job execution

- `POST /run-gcode`
- `POST /pause`
- `POST /resume`
- `POST /stop`

### Artwork processing

- `POST /analyze-image`
- `POST /analyze-image-colors`
- `POST /generate-image-gcode`
- `POST /generate-diagnostic-gcode`
- `POST /generate-gcode`
- `POST /analyze-svg`
- `POST /self-test-svg-pipeline`

## What Kinds of Toolpaths Exist

Common path kinds include:

- `outline`
- `fill-wall`
- `fill-infill`
- `detail-trace`
- `travel`

Preview payloads and many debug structures depend on these exact kinds.

## Known Project Assumptions

- Local/trusted operator workflow
- GRBL controller on serial, default port `COM12`
- Ball diameter default: `42.67 mm`
- Machine coordinates are expressed in degrees
- The drawing band is limited by config defaults:
  - X: `-180..180`
  - Y: `-45..45`
- SVG rendering in the browser is considered trusted-local only

## Key Configuration Areas

Configuration is environment-driven through [app/config.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/config.py:1).

High-impact settings include:

- serial port and baud
- machine feed/acceleration defaults
- servo positions and dwell timings
- ball diameter
- fill generation defaults
- toolpath cleanup tolerances
- raster analysis thresholds
- streaming mode
- stepper hold policy values

## Test Coverage Snapshot

There is meaningful coverage around the critical logic, especially:

- SVG parser behavior
- geometry mapping/projection
- raster analysis
- toolpath generation
- G-code generation
- GRBL line parsing/streaming edge cases
- machine/job safety and finalization behavior

Representative tests:

- [tests/test_machine_job_safety.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/tests/test_machine_job_safety.py:1)
- [tests/test_gcode_service.py](/C:/Users/hampe/Documents/GIT/Golfball%20Printer/tests/test_gcode_service.py:1)
- `tests/test_toolpath_generation.py`
- `tests/test_svg_parser.py`
- `tests/test_raster_analysis_service.py`
- `tests/test_grbl_streaming.py`
- `tests/test_geometry_service.py`
- `tests/test_validation_service.py`

## Where To Look First For Changes

### If the issue is about machine behavior

- `app/services/machine_service.py`
- `app/services/job_runner.py`
- `app/services/serial_service.py`
- `app/models/machine_state.py`

### If the issue is about artwork interpretation

- `app/services/svg_parser.py`
- `app/services/raster_analysis_service.py`
- `app/services/geometry_service.py`
- `app/services/pipeline_core.py`

### If the issue is about generated fills, paths, or projection

- `app/services/toolpath_service.py`
- `app/services/gcode_service.py`
- `app/services/pipeline_core.py`

### If the issue is about dashboard behavior

- `frontend/src/store/appStore.ts`
- `frontend/src/api/client.ts`
- `frontend/src/api/types.ts`
- relevant component files under `frontend/src/components/`

## Guidance For Future ChatGPT Sessions

When reasoning about this repo, treat these as the most important invariants:

1. Do not break the distinction between surface-mm geometry and projected machine-degree geometry.
2. Do not break calibration gating before print execution.
3. Do not break stepper hold policy transitions around calibration, pause/resume, and finalization.
4. Do not mark a job successful unless all lines were sent and acknowledged and the machine is truly idle.
5. Keep preview data and emitted G-code aligned to the same projected paths.
6. Be cautious about editing `pipeline_core.py`; much of the real behavior still lives there.

If proposing changes, prefer preserving these invariants over simplifying the architecture.
