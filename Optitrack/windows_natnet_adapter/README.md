# Windows NatNet Adapter

This directory contains the Windows receiver path for OptiTrack NatNet data.

## Current Status

This is the first executable skeleton. It includes:

- C++17 project structure for a Windows NatNet receiver.
- A normalized mocap frame contract.
- Config loading from `config/example_config.json`.
- JSONL serialization helpers.
- Python JSONL replay and config checks that run without OptiTrack hardware.

The official NatNet SDK callback implementation still needs to be completed on the Windows/Motive machine because this workspace does not contain the SDK headers or libraries.

At the current stage, the executable can read the configured server/local addresses, rigid body names, optional rigid body list, and ball measurement semantics. It emits one JSONL frame with `tracked=false` stub measurements so the downstream logging and replay path can be tested before the NatNet callback is wired in.

## First Device Test Order

1. Open Motive and enable NatNet streaming.
2. Run the official NatNet SDK sample unchanged.
3. Read `WINDOWS_SETUP.md`.
4. Copy `config/example_config.json` to `config/lab_config.json` and edit IPs and rigid body names.
5. Build this adapter with Visual Studio 2022 and CMake.
6. Run the adapter with `config/lab_config.json`.
7. Inspect the JSONL recording with `python tools/replay_jsonl.py logs/session.jsonl`.

For Windows handoff details, use `WINDOWS_SETUP.md`. For field-session metadata, fill in `logs/README.md`.

## Expected Data

- `table`: Motive rigid body pose.
- `robot_base`: Motive rigid body pose.
- `ego_camera`: optional Motive rigid body pose.
- `ball`: conservative point measurement. The first implementation does not claim ball-center semantics.

## Local Verification

From this directory:

```powershell
python -m unittest discover -s tests -v
```

On macOS/Linux during development:

```bash
python3 -m unittest discover -s tests -v
```

You can also compile the SDK-free skeleton directly:

```bash
clang++ -std=c++17 -I include src/main.cpp src/config.cpp src/jsonl_writer.cpp -o /tmp/windows_natnet_adapter_skeleton
/tmp/windows_natnet_adapter_skeleton config/example_config.json > /tmp/session.jsonl
python3 tools/replay_jsonl.py /tmp/session.jsonl
```

The status messages go to stderr; stdout is reserved for JSONL frames so shell redirection produces a replayable log file.

The replay summary reports:

- total frame count;
- first and last frame id;
- estimated missing frame count;
- how many frames include a ball measurement;
- which rigid body roles are present;
- how many frames each rigid body role was tracked in.

## Windows Build Notes

- Keep NatNet SDK headers and libraries outside this repository.
- Set `NATNET_SDK_DIR` to the local SDK directory before configuring CMake with SDK support.
- Record Motive version, NatNet version, local IP, server IP, command port, and data port in each session log.

Example:

```powershell
$env:NATNET_SDK_DIR="C:\NatNetSDK"
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DOPTITRACK_ADAPTER_WITH_NATNET=ON
cmake --build build --config Debug
```

Or use the helper scripts:

```powershell
.\scripts\build_windows.ps1 -NatNetSdkDir "C:\NatNetSDK"
.\scripts\run_smoke_test.ps1 -Config .\config\lab_config.json -Seconds 10
```
