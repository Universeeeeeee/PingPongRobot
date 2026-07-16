from __future__ import annotations

from typing import Any


class ZmqPublisher:
    def __init__(self, bind: str = "tcp://*:5556") -> None:
        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError("ZMQ publisher requires pyzmq") from exc
        self._zmq = zmq
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.bind(bind)

    def send(self, message: dict[str, Any]) -> None:
        self._socket.send_pyobj(message)

    def close(self) -> None:
        self._socket.close(linger=0)
        self._context.term()
