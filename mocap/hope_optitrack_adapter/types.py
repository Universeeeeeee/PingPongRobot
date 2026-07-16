from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


Vector3 = list[float]
QuaternionXyzw = list[float]


def float_list(values: Iterable[float], expected_len: int, field_name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != expected_len:
        raise ValueError(f"{field_name} must contain {expected_len} values")
    return result


@dataclass(frozen=True)
class BallState:
    timestamp_s: float
    position_m: Vector3
    velocity_m_s: Vector3
    frame_id: int | None = None
    source: str | None = None
    selected: int | None = None
    valid: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_s", float(self.timestamp_s))
        object.__setattr__(self, "position_m", float_list(self.position_m, 3, "position_m"))
        object.__setattr__(self, "velocity_m_s", float_list(self.velocity_m_s, 3, "velocity_m_s"))
        if self.frame_id is not None:
            object.__setattr__(self, "frame_id", int(self.frame_id))
        if self.selected is not None:
            object.__setattr__(self, "selected", int(self.selected))


@dataclass(frozen=True)
class RobotPose:
    timestamp_s: float
    position_m: Vector3
    quaternion_xyzw: QuaternionXyzw
    name: str = "P1"
    valid: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_s", float(self.timestamp_s))
        object.__setattr__(self, "position_m", float_list(self.position_m, 3, "position_m"))
        object.__setattr__(self, "quaternion_xyzw", float_list(self.quaternion_xyzw, 4, "quaternion_xyzw"))
