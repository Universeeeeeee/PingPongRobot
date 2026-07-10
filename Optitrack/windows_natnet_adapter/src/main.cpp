#include <iostream>
#include <stdexcept>

#include "optitrack_adapter/config.hpp"
#include "optitrack_adapter/jsonl_writer.hpp"
#include "optitrack_adapter/mocap_types.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: windows_natnet_adapter <config.json>\n";
    return 2;
  }

  optitrack_adapter::AdapterConfig config;
  try {
    config = optitrack_adapter::LoadConfigFromFile(argv[1]);
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  optitrack_adapter::MocapFrame frame;
  frame.session_id = "manual_test";
  frame.frame_id = 1;
  frame.natnet_version = "unknown";
  frame.motive_version = "unknown";
  frame.source_timestamp_s = 0.0;
  frame.received_monotonic_s = 0.0;

  for (const auto& body_config : config.rigid_bodies) {
    optitrack_adapter::PoseTrack body;
    body.tracked = false;
    body.streaming_id = -1;
    body.configured_name = body_config.motive_name;
    body.optional = body_config.optional;
    frame.rigid_bodies.emplace(body_config.role, body);
  }

  optitrack_adapter::PointTrack ball;
  ball.tracked = false;
  ball.semantics = config.ball.source;
  ball.validated_as_center = config.ball.validated_as_center;
  frame.ball = ball;

  frame.diagnostics.message = "config_loaded";

  std::cerr << "NatNet adapter skeleton loaded config: " << argv[1] << "\n";
  std::cerr << "server=" << config.server_address << " local=" << config.local_address
            << " connection=" << config.connection_type << " command_port=" << config.command_port
            << " data_port=" << config.data_port << "\n";
  std::cout << optitrack_adapter::ToJsonLine(frame) << "\n";

#ifndef OPTITRACK_ADAPTER_WITH_NATNET
  std::cerr << "NatNet SDK integration is disabled in this build. "
               "Configure with -DOPTITRACK_ADAPTER_WITH_NATNET=ON on Windows.\n";
#endif

  return 0;
}
