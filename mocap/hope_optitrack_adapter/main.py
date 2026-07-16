from __future__ import annotations

import argparse
import signal

from .config import load_config
from .diagnostics import DiagnosticsLogger
from .hope_udp_json_receiver import HopeUdpBallReceiver
from .natnet_receiver import NatNetRigidBodyBallReceiver
from .ros_receiver import HopeRosNamedPoseReceiver
from .schema_adapter import ball_to_zmq, robot_to_zmq
from .types import BallState, RobotPose
from .validation import BallFrameValidator
from .zmq_publisher import ZmqPublisher


class HopeOptitrackAdapter:
    def __init__(self, config_path: str | None = None, enable_ros: bool = True) -> None:
        self.config = load_config(config_path)
        self.coordinate_adapter = self.config.coordinate_adapter
        self.validator = BallFrameValidator()
        self.ball_receiver = self._create_ball_receiver()
        self.robot_receiver = (
            HopeRosNamedPoseReceiver(self.config.robot_rigid_body_name, self.config.robot_topic)
            if enable_ros
            else None
        )
        self.publisher = ZmqPublisher(self.config.publish_zmq)
        self.diagnostics = DiagnosticsLogger(
            self.config.diagnostics_log_dir,
            self.config.diagnostics_enabled,
            self.config.diagnostics_print_interval_s,
        )
        self._last_published_robot_timestamp_s: float | None = None
        self._running = True

    def _create_ball_receiver(self):
        if self.config.ball_source == "natnet_rigid_body":
            return NatNetRigidBodyBallReceiver(
                server_ip=self.config.natnet_server_ip,
                local_ip=self.config.natnet_local_ip,
                command_port=self.config.natnet_command_port,
                data_port=self.config.natnet_data_port,
                connection_type=self.config.natnet_connection_type,
                bitstream_version=self.config.natnet_bitstream_version,
                ball_name=self.config.ball_rigid_body_name,
                table_name=self.config.table_rigid_body_name,
            )
        if self.config.ball_source == "hope_natnet_udp_json":
            if self.config.udp_host is None or self.config.udp_port is None:
                raise ValueError("hope_natnet_udp_json requires ball.host and ball.port")
            return HopeUdpBallReceiver(self.config.udp_host, self.config.udp_port)
        raise ValueError(f"unsupported ball source {self.config.ball_source!r}")

    def close(self) -> None:
        self.ball_receiver.close()
        if self.robot_receiver is not None:
            self.robot_receiver.close()
        self.publisher.close()
        self.diagnostics.close()

    def stop(self, *_args) -> None:
        self._running = False

    def run(self) -> None:
        while self._running:
            self.process_once()

    def process_once(self) -> None:
        if self.robot_receiver is not None:
            self.robot_receiver.spin_once(timeout_sec=0.0)
        robot = self._publish_latest_robot()

        raw_ball = self.ball_receiver.recv_frame()
        if raw_ball is None:
            source_status = None
            if hasattr(self.ball_receiver, "diagnostic_status"):
                source_status = self.ball_receiver.diagnostic_status()
            self.diagnostics.tick(source_status=source_status)
            return

        received_monotonic_s = self.ball_receiver.last_received_monotonic_s
        ball = self.validator.accept(raw_ball)
        if ball is None:
            self.diagnostics.record(
                "ball_drop",
                extra={"reason": self.validator.last_drop_reason},
                received_monotonic_s=received_monotonic_s,
            )
            return

        ball = self.transform_ball(ball)
        self.publisher.send(ball_to_zmq(ball))
        self.diagnostics.record(
            "publish",
            ball=ball,
            robot=robot,
            received_monotonic_s=received_monotonic_s,
        )

    def _publish_latest_robot(self) -> RobotPose | None:
        if self.robot_receiver is None:
            return None
        robot = self.robot_receiver.latest_robot_pose()
        if robot is None or not robot.valid:
            return None

        robot = self.transform_robot(robot)
        if robot.timestamp_s != self._last_published_robot_timestamp_s:
            self.publisher.send(robot_to_zmq(robot))
            self._last_published_robot_timestamp_s = robot.timestamp_s
        return robot

    def transform_ball(self, ball: BallState) -> BallState:
        return BallState(
            timestamp_s=ball.timestamp_s,
            position_m=self.coordinate_adapter.transform_position(ball.position_m),
            velocity_m_s=self.coordinate_adapter.transform_vector(ball.velocity_m_s),
            frame_id=ball.frame_id,
            source=ball.source,
            selected=ball.selected,
            valid=ball.valid,
        )

    def transform_robot(self, robot: RobotPose) -> RobotPose:
        robot = self.config.robot_static_transform.apply(robot)
        return RobotPose(
            timestamp_s=robot.timestamp_s,
            position_m=self.coordinate_adapter.transform_position(robot.position_m),
            quaternion_xyzw=self.coordinate_adapter.transform_quaternion_xyzw(robot.quaternion_xyzw),
            name=robot.name,
            valid=robot.valid,
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="HOPE OptiTrack to current ZMQ mocap adapter")
    parser.add_argument("--config", default=None, help="Path to JSON/YAML adapter config")
    parser.add_argument("--no-ros", action="store_true", help="Run ball-only adapter without ROS robot receiver")
    args = parser.parse_args(argv)

    adapter = HopeOptitrackAdapter(config_path=args.config, enable_ros=not args.no_ros)
    signal.signal(signal.SIGINT, adapter.stop)
    signal.signal(signal.SIGTERM, adapter.stop)
    try:
        adapter.run()
    finally:
        adapter.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
