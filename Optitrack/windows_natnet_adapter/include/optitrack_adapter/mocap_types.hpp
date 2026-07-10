#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace optitrack_adapter {

using Vec3 = std::array<double, 3>;
using QuatXyzw = std::array<double, 4>;

struct PointTrack {
  Vec3 position_m{};
  bool tracked{false};
  std::string semantics{"unlabeled_marker"};
  bool validated_as_center{false};
};

struct PoseTrack {
  Vec3 position_m{};
  QuatXyzw quaternion_xyzw{0.0, 0.0, 0.0, 1.0};
  bool tracked{false};
  int streaming_id{-1};
  std::string configured_name;
  bool optional{false};
};

struct Diagnostics {
  int dropped_frame_estimate{0};
  int missing_ball_frames{0};
  std::string message;
};

struct MocapFrame {
  std::string source{"optitrack"};
  int schema_version{1};
  std::string session_id;
  std::int64_t frame_id{0};
  std::string natnet_version;
  std::string motive_version;
  double source_timestamp_s{0.0};
  double received_monotonic_s{0.0};
  std::string coordinate_frame{"motive_world"};
  std::optional<PointTrack> ball;
  std::map<std::string, PoseTrack> rigid_bodies;
  Diagnostics diagnostics;
};

}  // namespace optitrack_adapter
