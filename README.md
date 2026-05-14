# Golfball Plotter

Refactored Flask application for converting SVG artwork into G-code for a GRBL-driven golf ball plotter.

## Run

```bash
pip install -e .[dev]
python run.py
```

## Notes

- The legacy SVG parsing, geometry, and G-code algorithms are preserved through service wrappers to avoid behavior drift during the refactor.
- The raw SVG preview in the browser is intended for a trusted local workflow only. If this UI is ever exposed remotely, client-side SVG rendering should be sanitized or replaced.
- Upload size is limited with `MAX_CONTENT_LENGTH`.

## Tests

```bash
pytest
```
