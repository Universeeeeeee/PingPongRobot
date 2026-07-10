#include "optitrack_adapter/config.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace optitrack_adapter {

namespace {

std::string ReadFile(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open config: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string ExtractString(const std::string& text, const std::string& key, const std::string& fallback = "") {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return match[1].str();
  }
  return fallback;
}

int ExtractInt(const std::string& text, const std::string& key, int fallback) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*([0-9]+)");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return std::stoi(match[1].str());
  }
  return fallback;
}

double ExtractDouble(const std::string& text, const std::string& key, double fallback) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*([0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return std::stod(match[1].str());
  }
  return fallback;
}

bool ExtractBool(const std::string& text, const std::string& key, bool fallback) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*(true|false)");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return match[1].str() == "true";
  }
  return fallback;
}

bool IsOptionalRole(const std::string& text, const std::string& role) {
  const std::regex pattern("\"optional_rigid_bodies\"\\s*:\\s*\\[[^\\]]*\"" + role + "\"[^\\]]*\\]");
  return std::regex_search(text, pattern);
}

RigidBodyConfig ExtractRigidBody(const std::string& text, const std::string& role) {
  RigidBodyConfig config;
  config.role = role;
  config.motive_name = ExtractString(text, role);
  config.optional = IsOptionalRole(text, role);
  return config;
}

}  // namespace

AdapterConfig LoadConfigFromFile(const std::string& path) {
  const std::string text = ReadFile(path);

  AdapterConfig config;
  config.server_address = ExtractString(text, "server_address");
  config.local_address = ExtractString(text, "local_address");
  config.connection_type = ExtractString(text, "connection_type", "multicast");
  config.command_port = ExtractInt(text, "command_port", 1510);
  config.data_port = ExtractInt(text, "data_port", 1511);

  for (const std::string& role : {"table", "robot_base", "ego_camera"}) {
    RigidBodyConfig body = ExtractRigidBody(text, role);
    if (!body.motive_name.empty()) {
      config.rigid_bodies.push_back(body);
    }
  }

  config.ball.source = ExtractString(text, "source", "unlabeled_marker");
  config.ball.selection = ExtractString(text, "selection", "nearest_to_previous");
  config.ball.max_jump_m = ExtractDouble(text, "max_jump_m", 0.8);
  config.ball.validated_as_center = ExtractBool(text, "validated_as_center", false);

  config.output.jsonl_path = ExtractString(text, "jsonl_path", "logs/session.jsonl");
  config.output.udp_localhost_port = ExtractInt(text, "udp_localhost_port", 5511);
  config.output.enable_udp = ExtractBool(text, "enable_udp", false);

  return config;
}

}  // namespace optitrack_adapter

