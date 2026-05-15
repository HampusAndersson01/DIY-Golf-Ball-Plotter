from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
FRONTEND_DIR = ROOT / "frontend"
NPM_EXECUTABLE = "npm.cmd" if os.name == "nt" else "npm"


def start_process(command: list[str], cwd: Path, extra_env: dict[str, str] | None = None) -> subprocess.Popen:
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    return subprocess.Popen(command, cwd=str(cwd), env=env)


def terminate_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    process.kill()
    process.wait(timeout=5)


def main() -> int:
    if not FRONTEND_DIR.exists():
        print("Missing frontend/ directory.", file=sys.stderr)
        return 1

    backend = None
    frontend = None
    try:
        print("Starting Flask backend on http://127.0.0.1:5000")
        backend = start_process([sys.executable, "run.py"], ROOT)

        print("Starting Vite frontend on http://127.0.0.1:5173")
        frontend = start_process([NPM_EXECUTABLE, "run", "dev", "--", "--host", "127.0.0.1", "--port", "5173"], FRONTEND_DIR)

        print("Dashboard: http://127.0.0.1:5173")
        print("Backend API: http://127.0.0.1:5000")
        print("Press Ctrl+C to stop both processes.")

        while True:
            backend_code = backend.poll()
            frontend_code = frontend.poll()
            if backend_code is not None:
                print(f"Flask exited with code {backend_code}.", file=sys.stderr)
                return backend_code
            if frontend_code is not None:
                print(f"Vite exited with code {frontend_code}.", file=sys.stderr)
                return frontend_code
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopping development servers...")
        return 0
    finally:
        if frontend is not None:
            terminate_process(frontend)
        if backend is not None:
            terminate_process(backend)


if __name__ == "__main__":
    if os.name == "nt":
        signal.signal(signal.SIGINT, signal.default_int_handler)
    raise SystemExit(main())
