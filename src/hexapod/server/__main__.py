"""Start des Telemetrie-Servers: `uv run python -m hexapod.server`."""

from __future__ import annotations

import os

import uvicorn


def main() -> None:
    host = os.environ.get("HEXAPOD_HOST", "0.0.0.0")
    port = int(os.environ.get("HEXAPOD_PORT", "8000"))
    # Kein reload/Workers: der Roboter darf nur EINMAL geoeffnet werden.
    uvicorn.run("hexapod.server.app:app", host=host, port=port, workers=1)


if __name__ == "__main__":
    main()
