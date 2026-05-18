from __future__ import annotations

import atexit
import logging
import sys
import threading
from pathlib import Path

LOG_DIR = Path("logs")
LATEST_LOG = LOG_DIR / "latest.log"

_logging_initialized = False
_shutdown_registered = False


def _install_exception_hooks() -> None:
    def handle_exception(exc_type, exc_value, exc_traceback) -> None:
        if issubclass(exc_type, KeyboardInterrupt):
            logging.getLogger("golf_ball_plotter").info("KeyboardInterrupt received; shutting down")
            return
        logging.getLogger("golf_ball_plotter").critical(
            "Unhandled exception",
            exc_info=(exc_type, exc_value, exc_traceback),
        )

    def handle_thread_exception(args: threading.ExceptHookArgs) -> None:
        logging.getLogger("golf_ball_plotter").critical(
            "Unhandled thread exception in %s",
            getattr(args.thread, "name", "unknown"),
            exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
        )

    sys.excepthook = handle_exception
    threading.excepthook = handle_thread_exception


def _register_shutdown_logging() -> None:
    global _shutdown_registered
    if _shutdown_registered:
        return

    @atexit.register
    def _log_shutdown() -> None:
        logging.getLogger("golf_ball_plotter").info("Backend shutdown complete")

    _shutdown_registered = True


def setup_logging(debug: bool = False) -> logging.Logger:
    global _logging_initialized

    LOG_DIR.mkdir(parents=True, exist_ok=True)

    root = logging.getLogger()
    root.setLevel(logging.DEBUG if debug else logging.INFO)
    root.handlers.clear()

    formatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d %(levelname)-8s [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = logging.FileHandler(LATEST_LOG, mode="w", encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG if debug else logging.INFO)
    console_handler.setFormatter(formatter)

    root.addHandler(file_handler)
    root.addHandler(console_handler)

    logging.captureWarnings(True)
    _install_exception_hooks()
    _register_shutdown_logging()
    _logging_initialized = True

    logger = logging.getLogger("golf_ball_plotter")
    logger.info("Logging initialized")
    logger.info("latest.log path: %s", LATEST_LOG.resolve())

    return logger


def get_log_path() -> Path:
    return LATEST_LOG


def is_logging_initialized() -> bool:
    return _logging_initialized
