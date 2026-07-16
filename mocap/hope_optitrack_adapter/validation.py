from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .types import BallState, float_list


@dataclass
class BallFrameValidator:
    min_dt_s: float = 0.0005
    max_dt_s: float = 0.05
    max_jump_m: float = 0.30
    max_speed_m_s: float = 30.0
    max_frame_gap: int = 1

    def __post_init__(self) -> None:
        self.last_drop_reason: str | None = None
        self._reset_tracking_state()

    def _reset_tracking_state(self) -> None:
        self._previous_frame_id: int | None = None
        self._previous_timestamp_s: float | None = None
        self._previous_position_m: list[float] | None = None
        self._previous_source: str | None = None
        self._previous_selected: int | None = None

    def accept(self, frame: dict[str, Any]) -> BallState | None:
        self.last_drop_reason = None
        if frame.get("valid") is not True:
            return self._drop("invalid", reset_tracking=True)
        if frame.get("trajectory_break") is True:
            return self._drop("trajectory_break", reset_tracking=True)

        try:
            timestamp_s = float(frame["timestamp"])
            position_m = float_list(frame["position"], 3, "position")
            velocity_m_s = float_list(frame["velocity"], 3, "velocity")
            frame_id = int(frame["frame"]) if "frame" in frame else None
            selected = int(frame["selected"]) if frame.get("selected") is not None else None
            source = str(frame["source"]) if frame.get("source") is not None else None
        except (KeyError, TypeError, ValueError) as exc:
            return self._drop(f"malformed:{exc}", reset_tracking=True)

        if selected is not None and selected < 0:
            return self._drop("no_selected_candidate", reset_tracking=True)

        dt_s = self._dt_for_validation(frame, timestamp_s)
        if dt_s is not None:
            if dt_s <= self.min_dt_s:
                return self._drop("dt_too_small", reset_tracking=True)
            if dt_s > self.max_dt_s:
                return self._drop("dt_too_large", reset_tracking=True)

        if self._previous_frame_id is not None and frame_id is not None:
            frame_gap = frame_id - self._previous_frame_id
            if frame_gap <= 0:
                return self._drop("frame_not_increasing", reset_tracking=True)
            if frame_gap > self.max_frame_gap:
                return self._drop("frame_gap", reset_tracking=True)

        if self._previous_source is not None and source != self._previous_source:
            return self._drop("source_changed", reset_tracking=True)
        if self._previous_selected is not None and selected != self._previous_selected:
            return self._drop("selected_changed", reset_tracking=True)
        if self._previous_position_m is not None:
            jump_m = math.dist(self._previous_position_m, position_m)
            if jump_m > self.max_jump_m:
                return self._drop("position_jump", reset_tracking=True)

        speed_m_s = math.sqrt(sum(value * value for value in velocity_m_s))
        if speed_m_s > self.max_speed_m_s:
            return self._drop("speed_too_large", reset_tracking=True)

        self._previous_frame_id = frame_id
        self._previous_timestamp_s = timestamp_s
        self._previous_position_m = position_m
        self._previous_source = source
        self._previous_selected = selected
        return BallState(
            timestamp_s=timestamp_s,
            position_m=position_m,
            velocity_m_s=velocity_m_s,
            frame_id=frame_id,
            source=source,
            selected=selected,
            valid=True,
        )

    def _dt_for_validation(self, frame: dict[str, Any], timestamp_s: float) -> float | None:
        if self._previous_timestamp_s is not None:
            return timestamp_s - self._previous_timestamp_s
        if frame.get("dt") is not None:
            return float(frame["dt"])
        return None

    def _drop(self, reason: str, reset_tracking: bool = False) -> None:
        self.last_drop_reason = reason
        if reset_tracking:
            self._reset_tracking_state()
        return None
