from __future__ import annotations

from .types import BallState, RobotPose


def ball_to_zmq(ball: BallState) -> dict:
    return {
        "type": "ball",
        "t": ball.timestamp_s,
        "pos": _scale(ball.position_m, 1000.0),
        "vel": _scale(ball.velocity_m_s, 1000.0),
        "valid": ball.valid,
    }


def robot_to_zmq(robot: RobotPose) -> dict:
    return {
        "type": "robot",
        "t": robot.timestamp_s,
        "pos": _scale(robot.position_m, 1000.0),
        "quat": list(robot.quaternion_xyzw),
        "valid": robot.valid,
    }


def _scale(values: list[float], factor: float) -> list[float]:
    return [float(value) * factor for value in values]
