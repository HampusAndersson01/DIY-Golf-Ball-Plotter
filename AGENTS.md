# AGENTS.md

This file is the primary source of truth for coding agents working in this repository.

## Project Overview

DIY Golf Ball Plotter is a local control stack for a GRBL-driven golf ball plotting machine. The software takes artwork, analyzes it, generates previewable toolpaths and G-code, and streams jobs to real hardware through a Flask backend and a React/Vite operator dashboard.

This repository controls physical motion. Treat preview, geometry, and emitted G-code as machine-critical behavior, not just UI output.

## Architecture Overview

The repository is split into three main layers:

- `app/` contains the Flask backend, routes, services, models, utilities, and static/template assets.
- `frontend/` contains the React + TypeScript + Vite dashboard.
- `tests/` contains backend and pipeline regression tests.

Important backend areas:

- `app/services/pipeline_core.py` for the main artwork-to-toolpath pipeline
- `app/services/geometry_service.py` for geometry handling and projection logic
- `app/services/toolpath_service.py` for toolpath generation and cleanup
- `app/services/raster_analysis_service.py` for image analysis and region extraction
- `app/services/gcode_service.py` for G-code emission
- `app/services/job_runner.py`, `app/services/serial_service.py`, and `app/services/machine_service.py` for streaming and machine control

Important frontend areas:

- `frontend/src/App.tsx` for the dashboard shell and orchestration
- `frontend/src/store/appStore.ts` for application state
- `frontend/src/components/*` for preview, machine, calibration, image, and job UI
- `frontend/src/api/*` for backend API access

The repository also contains project history and documentation in `docs/`, including `docs/project-history-timeline.md` and `docs/ci-branch-protection.md`.

## Setup

Use the repository’s real setup commands:

```bash
python -m pip install -e .[dev]
```

```bash
cd frontend
npm ci
```

Run the backend only:

```bash
python run.py
```

Run both backend and frontend for local development:

```bash
python dev.py
```

Run the frontend by itself:

```bash
cd frontend
npm run dev
```

## Agent Device Setup

Each developer machine or agent-running device should have:

- Python 3.11 or newer
- Node.js 22 or newer
- `pip` for Python package installation
- `npm` for frontend dependency installation
- USB/serial access to the GRBL controller if the task touches real machine workflows

Recommended local setup:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e .[dev]
cd frontend
npm ci
```

Recommended validation setup:

- Run `pytest -q` for backend and pipeline changes.
- Run `cd frontend && npm run lint` for frontend code changes.
- Run `cd frontend && npm run build` for frontend changes that affect compilation, routing, or shared API types.
- Run the full set when changes cross backend/frontend boundaries.

## Graphify Context

Graphify is the default local context layer for agents working on this repository.

Before broad repository inspection:

1. Read the task input.
2. Classify the task type.
3. Check Graphify availability with:
   - `python3 -m pip show graphifyy`
   - `graphify --help`
4. If Graphify is missing, install it in the active device environment and initialize it with:
   - `python3 -m pip install graphifyy`
   - `graphify install --project --platform codex`
5. Check whether `graphify-out/graph.json` exists.
6. Generate or refresh the repository graph with:
   - `graphify update .`
7. Use Graphify context to identify likely relevant files, active code paths, related docs, tests, and workflows before reading many files directly.
8. If Graphify setup fails, capture the exact command and exact error, report the failure, and continue only with targeted inspection if the task remains safe to complete.

Graphify output is local agent-device context. Keep these artifacts out of version control by default:

- `graphify-out/`
- `.graphify/`

If graph output becomes stale after changes, refresh it with `graphify update .` before relying on it again.

Context refresh:

- Re-read this file before starting a new task.
- Re-read `README.md`, the relevant files in `docs/`, and the affected backend/frontend/test files before editing.
- Use `git status` and focused file reads to understand the current branch state before making changes.

Keep machine-local files, secrets, caches, and generated analysis output out of commits unless the task explicitly requires them. Existing ignore rules already cover common examples such as `.venv/`, `.pytest_cache/`, `.env`, `artifacts/`, `docs/videos/`, and editor files.

## Context Maintenance

- Keep `AGENTS.md`, the workflow docs in `docs/agent-workflows/`, the issue templates in `.github/ISSUE_TEMPLATE/`, and `.github/copilot-instructions.md` aligned with each other.
- Update this file when repository setup, validation commands, branching guidance, or task-intake rules change.
- Update workflow docs when the agent process changes, when new task types are supported, or when handoff guidance needs to be more specific.
- Update issue templates when the request intake fields need to capture new evidence, reproduction details, or acceptance criteria.
- Update device setup docs when local environment requirements, runtime dependencies, or optional tooling expectations change.
- Prevent instruction drift by treating this file as the canonical source and keeping the shorter docs as summaries or task-specific references.
- Tasks that do not come from GitHub issues should be handled from the prompt itself with explicit assumptions, not by inventing a fake issue.

## Universal Agent Task Intake

Agents must support all of these inputs:

- GitHub issue URL
- GitHub issue number
- Plain-language prompt
- Bug report
- Feature request
- Geometry, artwork, or screenshot-driven prompt
- Test failure prompt
- Code review prompt
- Documentation update prompt
- CI or workflow prompt
- Dependency or tooling prompt
- Future structured or unstructured agent task formats

For every task input:

1. Identify the task type.
2. Extract the objective, constraints, acceptance criteria, and relevant files.
3. Determine whether a linked issue exists.
4. If no issue exists, proceed from the prompt itself and state assumptions clearly.
5. Inspect the repository before editing.
6. Find the active code path before implementing behavior changes.
7. Choose a branch name based on the task type.
8. Add or update tests when behavior changes.
9. Run the appropriate validation commands.
10. Produce a PR-ready summary.

## Development Workflow

The standard workflow is:

1. Read the task input in full.
2. Classify the task as code, docs, workflow, test, geometry, or tooling work.
3. Inspect the relevant files and the active code path.
4. Create a task-scoped branch.
5. Make the smallest valid change.
6. Add or update tests when behavior changes.
7. Run validation.
8. Summarize the result in PR-ready form.

This workflow must work whether the starting input is an issue URL or a normal prompt.

## Validation

Use the repository’s actual validation commands:

```bash
pytest -q
```

```bash
cd frontend
npm run lint
```

```bash
cd frontend
npm run build
```

Known CI checks from `.github/workflows/ci.yml`:

- `Backend Tests`
- `Frontend Lint`
- `Frontend Build`

Prefer the narrowest validation set that proves the change, but do not skip the repository’s normal checks when behavior or shared interfaces changed.

## Geometry Rules

- Pen width is physical.
- Pen radius = pen width / 2.
- Outline centerlines should normally be inset by pen radius.
- Thin visible features must never be silently dropped.
- If offset contours collapse, use centerline or detail fallback instead of removing geometry.
- Geometry fixes must affect emitted preview, toolpath, or G-code behavior.
- Metadata-only changes are not valid geometry fixes.
- Regression tests should target the active pipeline, not unused helpers.

## Branching Rules

Recommended branch names:

- `feature/issue-123-short-name`
- `fix/issue-123-short-name`
- `docs/short-name`
- `workflow/short-name`
- `test/short-name`
- `agent/short-name`
- `prompt/short-name`

When a task starts from a prompt without an issue, use a concise branch name that reflects the task:

- `docs/agent-workflow-bootstrap`
- `workflow/ci-validation-update`
- `fix/geometry-outline-collapse`
- `agent/device-bootstrap`

## Required Agent Workflow

For every task:

1. Read the full task input.
2. Classify the task type.
3. Inspect affected files.
4. Find the active code path when behavior is involved.
5. Add or update a regression test when behavior changes.
6. Implement the smallest valid change.
7. Avoid unrelated refactors.
8. Run validation.

## PR Output Expectations

When the task is complete, provide:

- A short summary of what changed
- The validation commands that were run
- Any known limitations or follow-up work
- A clear note if the task was documentation, workflow, or tooling only

## Final Response Format

For completed tasks, keep the final response short and practical:

1. State what changed.
2. List the validation commands that ran and whether they passed.
3. Mention any missing commands or failures with the exact command and exact error.
4. Note any follow-up items only if they matter for the user’s next step.
5. Include a Graphify Context section with:
   - whether Graphify was installed before the task
   - whether Graphify was installed during the task
   - the install command used
   - the initialization command used
   - the graph generation or refresh command used
   - the output location
   - whether Graphify context was used before broad inspection
   - any Graphify errors
   - whether `.gitignore` was updated for Graphify artifacts

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

When the user types `/graphify`, invoke the `skill` tool with `skill: "graphify"` before doing anything else.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- Dirty graphify-out/ files are expected after hooks or incremental updates; dirty graph files are not a reason to skip graphify. Only skip graphify if the task is about stale or incorrect graph output, or the user explicitly says not to use it.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
