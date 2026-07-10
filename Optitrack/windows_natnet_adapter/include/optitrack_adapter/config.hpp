#pragma once

#include <string>
#include <vector>

namespace optitrack_adapter {

struct RigidBodyConfig {
  std::string role;
  std::string motive_name;
  bool optional{false};
};

struct BallConfig {
  std::string source{"unlabeled_marker"};
  std::string selection{"nearest_to_previous"};
  double max_jump_m{0.8};
  bool validated_as_center{false};
};

struct OutputConfig {
  std::string jsonl_path{"logs/session.jsonl"};
  int udp_localhost_port{5511};
  bool enable_udp{false};
};

struct AdapterConfig {
  std::string server_address;
  std::string local_address;
  std::string connection_type{"multicast"};
  int command_port{1510};
  int data_port{1511};
  std::vector<RigidBodyConfig> rigid_bodies;
  BallConfig ball;
  OutputConfig output;
};

AdapterConfig LoadConfigFromFile(const std::string& path);

}  // namespace optitrack_adapter

