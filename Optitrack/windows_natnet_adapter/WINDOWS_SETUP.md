# Windows Setup For NatNet Adapter

This document is for the Windows-side operator who will connect to Motive and verify real OptiTrack data.

## Roles

- Mac development machine: edit code, run Python tests, compile the SDK-free skeleton.
- Windows validation machine: run Motive, install the NatNet SDK, build the official-SDK adapter, and record live JSONL sessions.

## Required Environment

- Windows 10/11 x64.
- Visual Studio 2022 with the Desktop development with C++ workload.
- CMake 3.20 or newer.
- Python 3.10 or newer, available as `python` in PowerShell.
- OptiTrack Motive installed and licensed.
- NatNet SDK installed locally.
- PowerShell 5 or newer.

Do not copy the NatNet SDK into this repository. Keep the SDK outside the project and point the build to it with an environment variable:

```powershell
$env:NATNET_SDK_DIR="C:\NatNetSDK"
```

The expected SDK layout is:

```text
C:\NatNetSDK
  include\
  lib\x64\
```

If your SDK folder is different, set `NATNET_SDK_DIR` to the actual folder containing `include` and `lib\x64`.

## Motive / NatNet Settings

In Motive, enable NatNet streaming and record these values before every field test:

- Motive version.
- NatNet SDK version.
- Server IP.
- Local IP.
- Connection type: multicast or unicast.
- Command port: `1510`.
- Data port: `1511`.
- Up axis.
- Rigid body streaming: enabled.
- Unlabeled markers: enabled if the ball is an unlabeled marker.

The IP addresses and ports must match `config/lab_config.json`. Start by copying the template:

```powershell
Copy-Item .\config\example_config.json .\config\lab_config.json
notepad .\config\lab_config.json
```

Update `server_address`, `local_address`, `connection_type`, and Motive rigid body names before running the adapter.

## First Device Test Order

1. Start Motive.
2. Enable NatNet streaming in Motive.
3. Run the official NatNet SDK `MinimalClient` or `SampleClient` unchanged.
4. Confirm the official sample receives increasing frame numbers.
5. Confirm the official sample lists rigid bodies and marker data.
6. Build this adapter.
7. Run a short smoke recording.
8. Inspect the JSONL with `tools/replay_jsonl.py`.
9. Fill in `logs/README.md` for the recorded session.

Do not skip the official `MinimalClient` / `SampleClient` step. It separates SDK/network/Motive problems from adapter problems.

## Build

From this directory:

```powershell
.\scripts\build_windows.ps1 -NatNetSdkDir "C:\NatNetSDK"
```

Equivalent manual commands:

```powershell
$env:NATNET_SDK_DIR="C:\NatNetSDK"
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DOPTITRACK_ADAPTER_WITH_NATNET=ON
cmake --build build --config Debug
```

The executable should be:

```text
build\Debug\windows_natnet_adapter.exe
```

## Smoke Test

Use `config/lab_config.json` for real device tests:

```powershell
.\scripts\run_smoke_test.ps1 -Config .\config\lab_config.json -Seconds 10
```

This writes:

```text
logs\smoke_session.jsonl
```

Then it runs:

```powershell
python tools\replay_jsonl.py logs\smoke_session.jsonl
```

For the current SDK-free skeleton, all tracked counts are expected to be zero. After the NatNet callback is connected, a real session should show:

- nonzero `frames`;
- increasing `frame_id`;
- nonzero `rigid_body_tracked_counts` for visible rigid bodies;
- nonzero `ball_frame_count` when ball data is visible.

## Common Failures

### CMake cannot find NatNet

Check:

```powershell
echo $env:NATNET_SDK_DIR
Test-Path "$env:NATNET_SDK_DIR\include"
Test-Path "$env:NATNET_SDK_DIR\lib\x64"
```

### Adapter starts but receives no live data

Check:

- Motive NatNet streaming is enabled.
- Windows Firewall allows Motive and the adapter.
- `server_address` and `local_address` in `config/lab_config.json` are correct.
- Command port is `1510`.
- Data port is `1511`.
- Multicast/unicast mode matches Motive.
- The official `SampleClient` receives data on the same machine.

### Rigid bodies are missing

Check Motive rigid body names against:

```json
"rigid_body_names": {
  "table": "PPT",
  "robot_base": "P1_base",
  "ego_camera": "P1_head_camera"
}
```

Do not rely on array order. Use names and streaming IDs.

### Ball data is missing

Check the ball physical setup:

- unlabeled marker;
- labeled marker;
- fully reflective ball;
- rigid body pivot.

Record the choice in `logs/README.md`. Do not call the measurement a ball center until it has been validated.

