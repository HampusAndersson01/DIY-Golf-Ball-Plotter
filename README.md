# Golfball Printer

Small collection of Python scripts to run and test a CNC plotter adapted for golf-ball artwork.

Files
- `cnc_test.py` — simple tests for CNC/serial connectivity.
- `cnc_web_controller.py` — web-based controller (Flask or similar; run locally).
- `golf_ball_plotter_svg_gcode_runner.py` — load SVG/gcode and send to the plotter.

Requirements
- Python 3.8+ on Windows
- `pyserial` for serial communication

Quickstart (Windows)
1. Create and activate a virtual environment:

```powershell
python -m venv .venv
& .venv\Scripts\Activate.ps1
```

2. Install dependencies:

```powershell
pip install pyserial
```

3. Edit the scripts to set the correct serial/COM port for your device, then run a script:

```powershell
python golf_ball_plotter_svg_gcode_runner.py
python cnc_web_controller.py
python cnc_test.py
```

Notes
- Adjust serial settings (baud rate, COM port) inside the scripts before connecting to hardware.
- If you prefer, create a `requirements.txt` and install packages with `pip install -r requirements.txt`.

License
- No license specified. Add one if you intend to publish this project.
