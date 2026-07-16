from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .types import RobotPose, float_list


Matrix3 = list[list[float]]


@dataclass(frozen=True)
class CoordinateAdapter:
    r_planner_from_source: Matrix3
    t_planner_from_source_m: list[float]

    @classmethod
    def default_hope_to_planner(cls) -> "CoordinateAdapter":
        return cls(
            r_planner_from_source=[
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            t_planner_from_source_m=[-1.37, 0.7625, 0.02],
        )

    def __post_init__(self) -> None:
        if len(self.r_planner_from_source) != 3:
            raise ValueError("r_planner_from_source must be 3x3")
        matrix = []
        for row in self.r_planner_from_source:
            matrix.append(float_list(row, 3, "r_planner_from_source row"))
        object.__setattr__(self, "r_planner_from_source", matrix)
        object.__setattr__(
            self,
            "t_planner_from_source_m",
            float_list(self.t_planner_from_source_m, 3, "t_planner_from_source_m"),
        )

    def transform_position(self, position_m: Iterable[float]) -> list[float]:
        rotated = self.transform_vector(position_m)
        return [rotated[i] + self.t_planner_from_source_m[i] for i in range(3)]

    def transform_vector(self, vector: Iterable[float]) -> list[float]:
        x, y, z = float_list(vector, 3, "vector")
        matrix = self.r_planner_from_source
        return [
            matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
            matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
            matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
        ]

    def transform_quaternion_xyzw(self, quaternion_xyzw: Iterable[float]) -> list[float]:
        source_quat = _normalize_quat(float_list(quaternion_xyzw, 4, "quaternion_xyzw"))
        frame_quat = _quat_from_rotation_matrix(self.r_planner_from_source)
        return _normalize_quat(_quat_multiply(frame_quat, source_quat))


@dataclass(frozen=True)
class RobotStaticTransform:
    translation_m: list[float]
    quaternion_xyzw: list[float]

    @classmethod
    def identity(cls) -> "RobotStaticTransform":
        return cls(
            translation_m=[0.0, 0.0, 0.0],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
        )

    def __post_init__(self) -> None:
        object.__setattr__(self, "translation_m", float_list(self.translation_m, 3, "translation_m"))
        object.__setattr__(
            self,
            "quaternion_xyzw",
            _normalize_quat(float_list(self.quaternion_xyzw, 4, "quaternion_xyzw")),
        )

    def apply(self, mocap_pose: RobotPose) -> RobotPose:
        mocap_quat = _normalize_quat(mocap_pose.quaternion_xyzw)
        offset_in_source = _rotate_vector_by_quat(self.translation_m, mocap_quat)
        return RobotPose(
            timestamp_s=mocap_pose.timestamp_s,
            position_m=[mocap_pose.position_m[i] + offset_in_source[i] for i in range(3)],
            quaternion_xyzw=_normalize_quat(_quat_multiply(mocap_quat, self.quaternion_xyzw)),
            name=mocap_pose.name,
            valid=mocap_pose.valid,
        )


def _quat_multiply(left: list[float], right: list[float]) -> list[float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def _quat_conjugate(quaternion_xyzw: list[float]) -> list[float]:
    x, y, z, w = quaternion_xyzw
    return [-x, -y, -z, w]


def _rotate_vector_by_quat(vector: Iterable[float], quaternion_xyzw: list[float]) -> list[float]:
    vector_quat = [*float_list(vector, 3, "vector"), 0.0]
    rotated = _quat_multiply(
        _quat_multiply(_normalize_quat(quaternion_xyzw), vector_quat),
        _quat_conjugate(_normalize_quat(quaternion_xyzw)),
    )
    return rotated[:3]


def _normalize_quat(quaternion_xyzw: list[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in quaternion_xyzw))
    if norm < 1e-12:
        raise ValueError("quaternion_xyzw norm is zero")
    return [value / norm for value in quaternion_xyzw]


def _quat_from_rotation_matrix(matrix: Matrix3) -> list[float]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s]
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return [0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s]
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return [(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s]
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return [(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s]
