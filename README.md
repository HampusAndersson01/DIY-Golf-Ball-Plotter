# Golfball Plotter

Refactored Flask application for converting SVG artwork into G-code for a GRBL-driven golf ball plotter.

## Run

```bash
pip install -e .[dev]
python dev.py
```

## React Dashboard

One-command local development:

```bash
python dev.py
```

That starts:
- Flask backend on `http://127.0.0.1:5000`
- React/Vite dashboard on `http://127.0.0.1:5173`

If you want to run them separately, use:

```bash
python run.py

cd frontend
npm install
npm run dev
```

Vite proxies API requests to Flask. For a production frontend build:

```bash
cd frontend
npm run build
```

## Notes

- The legacy SVG parsing, geometry, and G-code algorithms are preserved through service wrappers to avoid behavior drift during the refactor.
- The raw SVG preview in the browser is intended for a trusted local workflow only. If this UI is ever exposed remotely, client-side SVG rendering should be sanitized or replaced.
- Upload size is limited with `MAX_CONTENT_LENGTH`.

## Tests

```bash
pytest
```
