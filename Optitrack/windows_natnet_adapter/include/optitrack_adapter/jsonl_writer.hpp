#pragma once

#include <string>

#include "optitrack_adapter/mocap_types.hpp"

namespace optitrack_adapter {

std::string JsonEscape(const std::string& value);
std::string ToJsonLine(const MocapFrame& frame);

}  // namespace optitrack_adapter

