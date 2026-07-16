from __future__ import annotations

import json
import sys
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable, TextIO

from .types import BallState, RobotPose


class DiagnosticsLogger:
    def __init__(
        self,
        log_dir: str | Path = "logs",
        enabled: bool = True,
        print_interval_s: float = 1.0,
        monotonic_clock: Callable[[], float] = time.monotonic,
        output: TextIO | None = None,
    ) -> None:
        if print_interval_s <= 0:
            raise ValueError("print_interval_s must be positive")
        self.enabled = enabled
        self._print_interval_s = float(print_interval_s)
        self._monotonic_clock = monotonic_clock
        self._output = output or sys.stdout
        self._summary_started_at_s = self._monotonic_clock()
        self._reset_summary()
        self._path: Path | None = None
        self._handle = None
        if enabled:
            directory = Path(log_dir)
            directory.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self._path = directory / f"mocap_bridge_{stamp}.jsonl"
            self._handle = self._path.open("a", encoding="utf-8")

    @property
    def path(self) -> Path | None:
        return self._path

    def record(
        self,
        event: str,
        ball: BallState | None = None,
        robot: RobotPose | None = None,
        extra: dict[str, Any] | None = None,
        received_monotonic_s: float | None = None,
    ) -> None:
        now_monotonic_s = self._monotonic_clock()
        self._record_summary(event, extra, received_monotonic_s, now_monotonic_s)
        if self._handle is not None:
            payload: dict[str, Any] = {
                "event": event,
                "wall_time_s": time.time(),
            }
            if ball is not None:
                payload["ball"] = asdict(ball)
            if robot is not None:
                payload["robot"] = asdict(robot)
            if received_monotonic_s is not None:
                payload["receive_monotonic_s"] = received_monotonic_s
            if extra:
                payload.update(extra)
            self._handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
            self._handle.flush()
        self.tick(now_monotonic_s)

    def tick(self, now_monotonic_s: float | None = None, source_status: str | None = None) -> None:
        now_monotonic_s = self._monotonic_clock() if now_monotonic_s is None else now_monotonic_s
        elapsed_s = now_monotonic_s - self._summary_started_at_s
        if elapsed_s < self._print_interval_s:
            return

        fps = self._ball_frame_count / elapsed_s
        valid_rate = (
            f"{self._valid_ball_frame_count / self._ball_frame_count * 100.0:.1f}%"
            if self._ball_frame_count
            else "n/a"
        )
        latency_ms = (
            f"{sum(self._latency_samples_ms) / len(self._latency_samples_ms):.1f}"
            if self._latency_samples_ms
            else "n/a"
        )
        drop_reason = ""
        if self._drop_reasons:
            reason_parts = [f"{reason}:{count}" for reason, count in self._drop_reasons.most_common()]
            drop_reason = " drop_reason=" + ",".join(reason_parts)
        source_status_text = f" {source_status}" if source_status else ""
        print(
            f"mocap fps={fps:.1f} latency_ms={latency_ms} "
            f"valid_rate={valid_rate} drop={self._drop_count}{drop_reason}{source_status_text}",
            file=self._output,
            flush=True,
        )
        self._summary_started_at_s = now_monotonic_s
        self._reset_summary()

    def _record_summary(
        self,
        event: str,
        extra: dict[str, Any] | None,
        received_monotonic_s: float | None,
        now_monotonic_s: float,
    ) -> None:
        if event not in {"publish", "ball_drop"}:
            return
        self._ball_frame_count += 1
        if event == "publish":
            self._valid_ball_frame_count += 1
        else:
            self._drop_count += 1
            if extra and extra.get("reason") is not None:
                self._drop_reasons[str(extra["reason"])] += 1
        if received_monotonic_s is not None:
            self._latency_samples_ms.append(max(0.0, now_monotonic_s - received_monotonic_s) * 1000.0)

    def _reset_summary(self) -> None:
        self._ball_frame_count = 0
        self._valid_ball_frame_count = 0
        self._drop_count = 0
        self._drop_reasons: Counter[str] = Counter()
        self._latency_samples_ms: list[float] = []

    def close(self) -> None:
        self.tick()
        if self._handle is not None:
            self._handle.close()
            self._handle = None
