from __future__ import annotations

from types import SimpleNamespace

import pytest

from app.services import pipeline_core


def test_connect_grbl_requires_explicit_serial_port(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(pipeline_core, "grbl", None)
    monkeypatch.setattr(pipeline_core, "SERIAL_PORT", None)

    with pytest.raises(RuntimeError, match="explicit SERIAL_PORT"):
        pipeline_core.connect_grbl()


def test_connect_grbl_uses_explicit_serial_port_without_com12_default(monkeypatch: pytest.MonkeyPatch):
    serial_calls: list[tuple[str, int, int]] = []
    fake_serial = SimpleNamespace(
        is_open=True,
        in_waiting=0,
        write=lambda payload: None,
        readline=lambda: b"",
    )

    def fake_serial_ctor(port: str, baud_rate: int, timeout: int = 0):
        serial_calls.append((port, baud_rate, timeout))
        return fake_serial

    monkeypatch.setattr(pipeline_core, "grbl", None)
    monkeypatch.setattr(pipeline_core, "SERIAL_PORT", "COM77")
    monkeypatch.setattr(pipeline_core.serial, "Serial", fake_serial_ctor)
    monkeypatch.setattr(pipeline_core.time, "sleep", lambda _: None)

    result = pipeline_core.connect_grbl()

    assert result is fake_serial
    assert serial_calls == [("COM77", pipeline_core.BAUD_RATE, 3)]
