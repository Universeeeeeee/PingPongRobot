#include "optitrack_adapter/jsonl_writer.hpp"

#include <sstream>

namespace optitrack_adapter {

namespace {

void WriteVec3(std::ostringstream& out, const Vec3& value) {
  out << "[" << value[0] << "," << value[1] << "," << value[2] << "]";
}

void WriteQuat(std::ostringstream& out, const QuatXyzw& value) {
  out << "[" << value[0] << "," << value[1] << "," << value[2] << "," << value[3] << "]";
}

}  // namespace

std::string JsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const char ch : value) {
    if (ch == '\\') {
      out << "\\\\";
    } else if (ch == '"') {
      out << "\\\"";
    } else if (ch == '\n') {
      out << "\\n";
    } else {
      out << ch;
    }
  }
  return out.str();
}

std::string ToJsonLine(const MocapFrame& frame) {
  std::ostringstream out;
  out << "{";
  out << "\"source\":\"" << JsonEscape(frame.source) << "\",";
  out << "\"schema_version\":" << frame.schema_version << ",";
  out << "\"session_id\":\"" << JsonEscape(frame.session_id) << "\",";
  out << "\"frame_id\":" << frame.frame_id << ",";
  out << "\"natnet_version\":\"" << JsonEscape(frame.natnet_version) << "\",";
  out << "\"motive_version\":\"" << JsonEscape(frame.motive_version) << "\",";
  out << "\"source_timestamp_s\":" << frame.source_timestamp_s << ",";
  out << "\"received_monotonic_s\":" << frame.received_monotonic_s << ",";
  out << "\"coordinate_frame\":\"" << JsonEscape(frame.coordinate_frame) << "\",";

  out << "\"ball\":";
  if (frame.ball.has_value()) {
    out << "{";
    out << "\"position_m\":";
    WriteVec3(out, frame.ball->position_m);
    out << ",\"tracked\":" << (frame.ball->tracked ? "true" : "false");
    out << ",\"semantics\":\"" << JsonEscape(frame.ball->semantics) << "\"";
    out << ",\"validated_as_center\":" << (frame.ball->validated_as_center ? "true" : "false");
    out << "}";
  } else {
    out << "null";
  }

  out << ",\"rigid_bodies\":{";
  bool first_body = true;
  for (const auto& [name, pose] : frame.rigid_bodies) {
    if (!first_body) {
      out << ",";
    }
    first_body = false;
    out << "\"" << JsonEscape(name) << "\":{";
    out << "\"position_m\":";
    WriteVec3(out, pose.position_m);
    out << ",\"quaternion_xyzw\":";
    WriteQuat(out, pose.quaternion_xyzw);
    out << ",\"tracked\":" << (pose.tracked ? "true" : "false");
    out << ",\"streaming_id\":" << pose.streaming_id;
    out << ",\"configured_name\":\"" << JsonEscape(pose.configured_name) << "\"";
    out << ",\"optional\":" << (pose.optional ? "true" : "false");
    out << "}";
  }
  out << "},";

  out << "\"diagnostics\":{";
  out << "\"dropped_frame_estimate\":" << frame.diagnostics.dropped_frame_estimate << ",";
  out << "\"missing_ball_frames\":" << frame.diagnostics.missing_ball_frames << ",";
  out << "\"message\":\"" << JsonEscape(frame.diagnostics.message) << "\"";
  out << "}";

  out << "}";
  return out.str();
}

}  // namespace optitrack_adapter
