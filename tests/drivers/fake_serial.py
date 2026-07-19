"""Ein handgeschriebener Fake für serial.Serial.

Bewusst kein unittest.mock.Mock — handgeschrieben ist hier lesbarer,
und macht explizit, was die Tests vom Maestro-Protokoll erwarten.
"""

from __future__ import annotations

from collections import deque


class FakeSerial:
    """Fake-Implementation von serial.Serial für Tests.

    - `written` sammelt alle gesendeten Bytes (als bytearray).
    - `queue_response(...)` legt Bytes in eine FIFO, die `read()` ausliefert.
    - `read(n)` gibt bis zu n Bytes aus der FIFO zurück.
    """

    def __init__(self) -> None:
        self.written = bytearray()
        self._response_queue: deque[int] = deque()
        self.closed = False
        self.flush_count = 0

    def write(self, data: bytes | bytearray) -> int:
        if self.closed:
            raise OSError("FakeSerial: write nach close()")
        self.written.extend(data)
        return len(data)

    def read(self, n: int = 1) -> bytes:
        if self.closed:
            raise OSError("FakeSerial: read nach close()")
        out = bytearray()
        for _ in range(n):
            if not self._response_queue:
                break
            out.append(self._response_queue.popleft())
        return bytes(out)

    def flush(self) -> None:
        self.flush_count += 1

    def close(self) -> None:
        self.closed = True

    # --- Test-Helpers ---

    def queue_response(self, *data: int) -> None:
        """Lege Bytes für nachfolgende read()-Aufrufe in die FIFO."""
        self._response_queue.extend(data)

    def clear_written(self) -> None:
        """Verwirf bisher gesendete Bytes (z.B. nach Setup-Phase)."""
        self.written.clear()
