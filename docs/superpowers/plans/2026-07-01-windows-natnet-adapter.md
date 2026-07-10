# Windows NatNet Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and validate a Windows C++ OptiTrack NatNet adapter that records normalized ball position and rigid body poses, with Python replay tools for inspection.

**Architecture:** The C++ adapter wraps the official NatNet SDK and converts each callback into a project-level `MocapFrame`. It writes JSONL for replay first; live UDP output is added only after the recorded contract is stable.

**Tech Stack:** Windows 10/11, Visual Studio 2022, CMake, OptiTrack NatNet SDK, C++17, Python 3.10+ standard library.

---

## File Structure

- Create: `Optitrack/windows_natnet_adapter/README.md` - setup and on-site runbook.
- Create: `Optitrack/windows_natnet_adapter/config/example_config.json` - explicit receiver and object mapping.
- Create: `Optitrack/windows_natnet_adapter/CMakeLists.txt` - CMake project for Visual Studio.
- Create: `Optitrack/windows_natnet_adapter/include/optitrack_adapter/mocap_types.hpp` - normalized data types.
- Create: `Optitrack/windows_natnet_adapter/include/optitrack_adapter/config.hpp` - small config model.
- Create: `Optitrack/windows_natnet_adapter/src/main.cpp` - executable entry point.
- Create: `Optitrack/windows_natnet_adapter/src/jsonl_writer.cpp` - JSONL writer.
- Create: `Optitrack/windows_natnet_adapter/tools/replay_jsonl.py` - offline validation tool.
- Create: `Optitrack/windows_natnet_adapter/tests/test_replay_jsonl.py` - Python replay tests.

This plan intentionally implements JSONL recording before live streaming. That creates replayable evidence from the first device test and keeps the adapter boundary small.

## Task 1: Add Project Skeleton And Configuration

**Files:**
- Create: `Optitrack/windows_natnet_adapter/README.md`
- Create: `Optitrack/windows_natnet_adapter/config/example_config.json`
- Create: `Optitrack/windows_natnet_adapter/CMakeLists.txt`

- [ ] **Step 1: Create the README**

Write `Optitrack/windows_natnet_adapter/README.md`:

```markdown
# Windows NatNet Adapter

This directory contains the Windows receiver path for OptiTrack NatNet data.

## First Test Order

1. Open Motive and enable NatNet streaming.
2. Run the official NatNet SDK sample unchanged.
3. Build this adapter with Visual Studio 2022 and CMake.
4. Run the adapter with `config/example_config.json`.
5. Inspect the JSONL recording with `python tools/replay_jsonl.py logs/session.jsonl`.

## Expected Data

- `table`: Motive rigid body pose.
- `robot_base`: Motive rigid body pose.
- `ego_camera`: optional Motive rigid body pose.
- `ball`: conservative point measurement. The first implementation does not claim ball-center semantics.

## Notes

- Keep NatNet SDK headers and libraries outside this repository.
- Set `NATNET_SDK_DIR` to the local SDK directory before configuring CMake.
- Record Motive version, NatNet version, local IP, server IP, command port, and data port in each session log.
```

- [ ] **Step 2: Create the example config**

Write `Optitrack/windows_natnet_adapter/config/example_config.json`:

```json
{
  "server_address": "192.168.1.10",
  "local_address": "192.168.1.20",
  "connection_type": "multicast",
  "command_port": 1510,
  "data_port": 1511,
  "rigid_body_names": {
    "table": "PPT",
    "robot_base": "P1_base",
    "ego_camera": "P1_head_camera"
  },
  "optional_rigid_bodies": ["ego_camera"],
  "ball": {
    "source": "unlabeled_marker",
    "selection": "nearest_to_previous",
    "max_jump_m": 0.8,
    "validated_as_center": false
  },
  "output": {
    "jsonl_path": "logs/session.jsonl",
    "udp_localhost_port": 5511,
    "enable_udp": false
  }
}
```

- [ ] **Step 3: Add CMake project**

Write `Optitrack/windows_natnet_adapter/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20)
project(windows_natnet_adapter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ENV{NATNET_SDK_DIR})
  message(FATAL_ERROR "Set NATNET_SDK_DIR to the OptiTrack NatNet SDK directory")
endif()

set(NATNET_SDK_DIR "$ENV{NATNET_SDK_DIR}")

add_executable(windows_natnet_adapter
  src/main.cpp
  src/jsonl_writer.cpp
)

target_include_directories(windows_natnet_adapter PRIVATE
  include
  "${NATNET_SDK_DIR}/include"
)

target_link_directories(windows_natnet_adapter PRIVATE
  "${NATNET_SDK_DIR}/lib/x64"
)

target_link_libraries(windows_natnet_adapter PRIVATE NatNetLib)
```

- [ ] **Step 4: Configure the project on Windows**

Run from `Optitrack/windows_natnet_adapter` in Developer PowerShell:

```powershell
$env:NATNET_SDK_DIR="C:\NatNetSDK"
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
```

Expected: CMake configures a Visual Studio build tree. If it cannot find `NatNetLib`, adjust `NATNET_SDK_DIR` to the actual SDK install directory.

## Task 2: Define The Normalized Frame Contract

**Files:**
- Create: `Optitrack/windows_natnet_adapter/include/optitrack_adapter/mocap_types.hpp`

- [ ] **Step 1: Create normalized types**

Write `Optitrack/windows_natnet_adapter/include/optitrack_adapter/mocap_types.hpp`:

```cpp
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
  int64_t frame_id{0};
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
```

- [ ] **Step 2: Compile headers through the executable target**

Run:

```powershell
cmake --build build --config Debug
```

Expected before Task 3: build fails because `src/main.cpp` and `src/jsonl_writer.cpp` do not exist. This confirms CMake is now checking the planned source paths.

## Task 3: Add Minimal Executable And JSONL Writer

**Files:**
- Create: `Optitrack/windows_natnet_adapter/src/main.cpp`
- Create: `Optitrack/windows_natnet_adapter/src/jsonl_writer.cpp`

- [ ] **Step 1: Add a compileable main**

Write `Optitrack/windows_natnet_adapter/src/main.cpp`:

```cpp
#include <iostream>

#include "optitrack_adapter/mocap_types.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: windows_natnet_adapter <config.json>\n";
    return 2;
  }

  optitrack_adapter::MocapFrame frame;
  frame.session_id = "manual_test";
  frame.frame_id = 1;
  frame.natnet_version = "unknown";
  frame.motive_version = "unknown";

  std::cout << "NatNet adapter skeleton loaded config: " << argv[1] << "\n";
  std::cout << "schema_version=" << frame.schema_version << "\n";
  return 0;
}
```

- [ ] **Step 2: Add JSONL serialization helper**

Write `Optitrack/windows_natnet_adapter/src/jsonl_writer.cpp`:

```cpp
#include "optitrack_adapter/mocap_types.hpp"

#include <sstream>
#include <string>

namespace optitrack_adapter {

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
  out << "\"coordinate_frame\":\"" << JsonEscape(frame.coordinate_frame) << "\"";
  out << "}";
  return out.str();
}

}  // namespace optitrack_adapter
```

- [ ] **Step 3: Build**

Run:

```powershell
cmake --build build --config Debug
```

Expected: build succeeds and produces `build\Debug\windows_natnet_adapter.exe`.

- [ ] **Step 4: Run skeleton**

Run:

```powershell
.\build\Debug\windows_natnet_adapter.exe .\config\example_config.json
```

Expected output contains:

```text
NatNet adapter skeleton loaded config: .\config\example_config.json
schema_version=1
```

## Task 4: Add Python JSONL Replay Tool

**Files:**
- Create: `Optitrack/windows_natnet_adapter/tools/replay_jsonl.py`
- Create: `Optitrack/windows_natnet_adapter/tests/test_replay_jsonl.py`

- [ ] **Step 1: Write replay tool**

Write `Optitrack/windows_natnet_adapter/tools/replay_jsonl.py`:

```python
import json
import sys
from pathlib import Path


def load_frames(path: Path) -> list[dict]:
    frames: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                frames.append(json.loads(stripped))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSONL: {exc}") from exc
    return frames


def summarize(frames: list[dict]) -> dict:
    frame_ids = [int(frame["frame_id"]) for frame in frames if "frame_id" in frame]
    missing = 0
    if len(frame_ids) >= 2:
        expected = frame_ids[-1] - frame_ids[0] + 1
        missing = max(0, expected - len(set(frame_ids)))
    return {
        "frames": len(frames),
        "first_frame_id": frame_ids[0] if frame_ids else None,
        "last_frame_id": frame_ids[-1] if frame_ids else None,
        "missing_frame_count": missing,
    }


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: replay_jsonl.py <session.jsonl>", file=sys.stderr)
        return 2
    frames = load_frames(Path(argv[1]))
    print(json.dumps(summarize(frames), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
```

- [ ] **Step 2: Write replay tests**

Write `Optitrack/windows_natnet_adapter/tests/test_replay_jsonl.py`:

```python
import json
import tempfile
import unittest
from pathlib import Path

from tools.replay_jsonl import load_frames, summarize


class ReplayJsonlTest(unittest.TestCase):
    def test_summarize_counts_missing_frames(self):
        frames = [
            {"frame_id": 10},
            {"frame_id": 11},
            {"frame_id": 13},
        ]
        summary = summarize(frames)
        self.assertEqual(summary["frames"], 3)
        self.assertEqual(summary["first_frame_id"], 10)
        self.assertEqual(summary["last_frame_id"], 13)
        self.assertEqual(summary["missing_frame_count"], 1)

    def test_load_frames_reads_jsonl(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "session.jsonl"
            path.write_text(
                json.dumps({"frame_id": 1}) + "\n" + json.dumps({"frame_id": 2}) + "\n",
                encoding="utf-8",
            )
            self.assertEqual(load_frames(path), [{"frame_id": 1}, {"frame_id": 2}])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run Python tests**

Run from `Optitrack/windows_natnet_adapter`:

```powershell
python -m unittest discover -s tests -v
```

Expected: two tests pass.

## Task 5: Integrate Official NatNet SDK Callbacks

**Files:**
- Modify: `Optitrack/windows_natnet_adapter/src/main.cpp`
- Modify: `Optitrack/windows_natnet_adapter/CMakeLists.txt`
- Create: `Optitrack/windows_natnet_adapter/include/optitrack_adapter/natnet_session.hpp`
- Create: `Optitrack/windows_natnet_adapter/src/natnet_session.cpp`

- [ ] **Step 1: Copy the SDK sample connection pattern**

Open the official NatNet SDK C++ `MinimalClient` or `SampleClient` shipped with the installed SDK. Copy the SDK's current connection setup into `natnet_session.cpp`, preserving the SDK's callback signatures for the installed version.

Use this project wrapper interface in `include/optitrack_adapter/natnet_session.hpp`:

```cpp
#pragma once

#include <functional>
#include <string>

#include "optitrack_adapter/mocap_types.hpp"

namespace optitrack_adapter {

struct NatNetSessionConfig {
  std::string server_address;
  std::string local_address;
  std::string connection_type;
  int command_port{1510};
  int data_port{1511};
};

class NatNetSession {
 public:
  using FrameCallback = std::function<void(const MocapFrame&)>;

  explicit NatNetSession(NatNetSessionConfig config);
  bool Start(FrameCallback callback);
  void Stop();

 private:
  NatNetSessionConfig config_;
  FrameCallback callback_;
};

}  // namespace optitrack_adapter
```

- [ ] **Step 2: Add source to CMake**

Modify `CMakeLists.txt` executable sources:

```cmake
add_executable(windows_natnet_adapter
  src/main.cpp
  src/jsonl_writer.cpp
  src/natnet_session.cpp
)
```

- [ ] **Step 3: Build after SDK integration**

Run:

```powershell
cmake --build build --config Debug
```

Expected: build succeeds against the installed SDK. If callback signatures differ from the sample, update only `natnet_session.cpp` to match the installed SDK and keep `mocap_types.hpp` unchanged.

## Task 6: On-Site P0/P1 Verification

**Files:**
- Modify: `Optitrack/windows_natnet_adapter/README.md`
- Create: `Optitrack/windows_natnet_adapter/logs/README.md`

- [ ] **Step 1: Run official SDK sample unchanged**

Run the official sample executable included with the SDK or built from the SDK sample project.

Expected observations:

```text
Server application name: Motive
NatNet version: printed by sample
Motive/server version: printed by sample
Frame callbacks: continuously increasing
Rigid bodies: table and robot base are listed
Markers: ball source is visible
```

- [ ] **Step 2: Run adapter metadata mode**

Run:

```powershell
.\build\Debug\windows_natnet_adapter.exe .\config\example_config.json
```

Expected observations:

```text
connected to Motive
local_address=<configured local address>
server_address=<configured server address>
rigid_body table=<streaming id>
rigid_body robot_base=<streaming id>
```

- [ ] **Step 3: Record a 60 second JSONL session**

Run the adapter with JSONL enabled for at least 60 seconds. Save the output to:

```text
Optitrack/windows_natnet_adapter/logs/first_device_session.jsonl
```

- [ ] **Step 4: Replay the recording**

Run:

```powershell
python tools\replay_jsonl.py logs\first_device_session.jsonl
```

Expected output includes nonzero `frames` and a finite `missing_frame_count`.

- [ ] **Step 5: Record field notes**

Write `Optitrack/windows_natnet_adapter/logs/README.md`:

```markdown
# OptiTrack Field Logs

## first_device_session.jsonl

- Motive version:
- NatNet version:
- Server IP:
- Local IP:
- Connection type:
- Command port:
- Data port:
- Table rigid body name:
- Robot base rigid body name:
- Ego camera rigid body name:
- Ball source:
- Ball physical setup:
- Coordinate frame observed:
- Notes:
```

Fill every line during the field session. If a value is unknown, record the exact UI or sample output that was visible instead of leaving the line blank.

## Self-Review Against Spec

- Spec coverage: the plan covers Windows receiver setup, official SDK use, C++ normalized contract, JSONL recording, Python replay, and P0/P1 device verification.
- Scope control: the plan does not implement AEKF, planner integration, spin, ROS 2, or table-frame calibration.
- Risk coverage: the plan captures SDK version, Motive version, network settings, named rigid bodies, conservative ball semantics, and replayable logs.
- Repository note: the current workspace is not a Git repository, so commit steps cannot be executed here. If this directory is later moved into a Git repository, commit after each task with `git add <task files>` and `git commit -m "<task summary>"`.
