from __future__ import annotations

from collections import deque

import pytest

from app.services import pipeline_core


class FakeSerial:
    def __init__(self, rx_buffer_size: int) -> None:
        self.rx_buffer_size = rx_buffer_size
        self.pending_lengths: deque[int] = deque()
        self.responses: deque[bytes] = deque()
        self.writes: list[bytes] = []

    def write(self, payload: bytes) -> int:
        pending_bytes = sum(self.pending_lengths)
        if pending_bytes + len(payload) >= self.rx_buffer_size:
            raise AssertionError("Streamer overflowed the GRBL RX buffer")
        self.writes.append(payload)
        self.pending_lengths.append(len(payload))
        self.responses.append(b"ok\n")
        return len(payload)

    def readline(self) -> bytes:
        if self.responses:
            self.pending_lengths.popleft()
            return self.responses.popleft()
        return b""


def test_stream_gcode_lines_unlocked_uses_buffered_streaming():
    serial = FakeSerial(rx_buffer_size=32)
    lines = [
        "G1 X1 Y1 F1200",
        "G1 X2 Y2 F1200",
        "G1 X3 Y3 F1200",
        "G1 X4 Y4 F1200",
    ]

    result = pipeline_core.stream_gcode_lines_unlocked(
        serial,
        lines,
        rx_buffer_size=32,
        response_timeout=0.01,
    )

    assert result["sent_count"] == len(lines)
    assert result["acked_count"] == len(lines)
    assert result["pending_queue_length"] == 0
    assert result["pending_buffer_chars"] == 0
    assert len(serial.writes) == len(lines)


def test_stream_gcode_lines_unlocked_raises_on_grbl_error():
    class ErrorSerial(FakeSerial):
        def write(self, payload: bytes) -> int:
            self.writes.append(payload)
            self.pending_lengths.append(len(payload))
            self.responses.append(b"error:1\n")
            return len(payload)

    serial = ErrorSerial(rx_buffer_size=64)

    with pytest.raises(RuntimeError, match="GRBL streaming error"):
        pipeline_core.stream_gcode_lines_unlocked(
            serial,
            ["G1 X1 Y1 F1200"],
            rx_buffer_size=64,
            response_timeout=0.01,
        )
