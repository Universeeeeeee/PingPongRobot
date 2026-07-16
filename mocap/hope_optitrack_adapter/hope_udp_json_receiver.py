from __future__ import annotations

import json
import socket
import time
from typing import Any


class HopeUdpBallReceiver:
    def __init__(self, host: str = "127.0.0.1", port: int = 38999, timeout_s: float = 0.005) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((host, int(port)))
        self._socket.settimeout(timeout_s)
        self.last_received_monotonic_s: float | None = None

    def recv_frame(self) -> dict[str, Any] | None:
        try:
            payload, _address = self._socket.recvfrom(65535)
        except socket.timeout:
            return None
        self.last_received_monotonic_s = time.monotonic()
        return json.loads(payload.decode("utf-8"))

    def close(self) -> None:
        self._socket.close()
