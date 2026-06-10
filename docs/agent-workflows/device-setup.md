# Device Setup

This repository can be worked on by humans or by local agents, but every device should be prepared the same way.

## Required runtime dependencies

- Python 3.11+
- Node.js 22+
- `pip`
- `npm`
- Access to the repository checkout
- USB/serial access to the GRBL controller if the task touches live hardware

## Python setup

Recommended setup:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e .[dev]
```

The backend uses the package metadata in `pyproject.toml`, and the dev extra is what enables the local test workflow.

## Frontend setup

Install the frontend dependencies from the `frontend/` directory:

```bash
cd frontend
npm ci
```

## Recommended local validation

- `pytest -q`
- `cd frontend && npm run lint`
- `cd frontend && npm run build`

Use the smallest validation set that proves the change, but do not skip the repo’s normal checks when backend and frontend interfaces both changed.

## Optional AI-agent tooling

- Graphify package name: `graphifyy`
- Graphify CLI: `graphify`
- Recommended local install command:

```bash
python3 -m pip install graphifyy
```

- Project-scoped initialization command used by this repository:

```bash
graphify install --project --platform codex
```

- Graph refresh command:

```bash
graphify update .
```

- Local output location:
  - `graphify-out/`

Graphify usage rules:

- Check `python3 -m pip show graphifyy` and `graphify --help` first when setting up a new device.
- If Graphify is missing, install it before broad source inspection.
- Use `graphify query`, `graphify path`, and `graphify explain` to identify relevant files and active paths before reading large code areas directly.
- Refresh the graph after substantial edits so the local context stays current.
- Keep generated graph output and cache directories out of version control unless a specific task intentionally requires a reviewed artifact.

Troubleshooting:

- If `python3 -m pip install graphifyy` hits a system-package restriction, install into a virtual environment instead.
- If `graphify` is not on `PATH`, run it from the active environment or use the interpreter that installed it.
- If graph generation fails, record the exact command and exact error before continuing with targeted inspection.

## Refreshing local agent context

- Re-read `AGENTS.md`.
- Re-read the relevant workflow docs.
- Inspect the affected source files.
- Confirm the current branch state with `git status`.
- Re-run the relevant validation commands after editing.

## Avoid committing local artifacts

Do not commit:

- Virtual environments
- Dependency caches
- `.pytest_cache`
- `.env`
- Local logs
- Generated graph/context artifacts
- Machine-specific analysis output

If a local tool produces persistent output, add an ignore rule before using it broadly.
