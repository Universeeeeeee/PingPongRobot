# Windows NatNet Adapter Design

> Scope: Windows Motive host, OptiTrack NatNet SDK, ball position, and rigid body poses for the table-tennis robot project.
>
> Iteration policy: this document records two design passes. Iteration 1 defines the smallest reliable link. Iteration 2 incorporates a self-review focused on on-site failure modes and downstream estimator needs.

## Inputs

- Existing project note: `Optitrack/optitrack_hope_natnet_migration_summary.md`
- Official OptiTrack NatNet SDK docs: `https://docs.optitrack.com/v3.1/developer-tools/natnet-sdk/natnet-4.0`
- Confirmed deployment direction from the user: Windows receiver, with Python and C++ both possible.

## Decision

Use C++ for the NatNet receiver and Python for analysis tools.

The C++ process owns the official NatNet SDK integration because the stable SDK path is the native library plus headers. Python consumes normalized output for recording, plotting, AEKF experiments, and quick evaluation. Python direct depacketization is kept as a temporary diagnostic option only, not the main receiver.

## Iteration 1: Minimal Working Chain

### Architecture

```text
Motive / OptiTrack
  -> NatNet UDP
  -> C++ NatNetAdapter
      -> official NatNetClient callback
      -> rigid body name/id mapping
      -> ball marker extraction
      -> normalized MocapFrame
      -> JSONL log + optional UDP localhost stream
  -> Python tools
      -> inspect / plot / estimator prototype
```

### Objects

The first implementation recognizes these project-level tracks:

| Track | NatNet source | Output |
|---|---|---|
| `table` | vendor rigid body | position, quaternion, tracked flag |
| `robot_base` | vendor rigid body | position, quaternion, tracked flag |
| `ego_camera` | vendor rigid body, optional | position, quaternion, tracked flag |
| `ball` | unlabeled marker, labeled marker, or ball rigid body | position and measurement semantics |

Rigid body identity is resolved through Motive data descriptions at startup. The adapter must not rely on fixed array index order.

### Minimal Data Contract

Each received frame is normalized into one JSON object per line:

```json
{
  "source": "optitrack",
  "schema_version": 1,
  "frame_id": 12345,
  "natnet_version": "4.1",
  "motive_version": "3.1",
  "source_timestamp_s": 12.345678,
  "received_monotonic_s": 9876.54321,
  "coordinate_frame": "motive_world",
  "ball": {
    "position_m": [0.12, -0.34, 0.78],
    "tracked": true,
    "semantics": "unlabeled_marker",
    "validated_as_center": false
  },
  "rigid_bodies": {
    "table": {
      "position_m": [0.0, 0.0, 0.0],
      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
      "tracked": true,
      "streaming_id": 1
    },
    "robot_base": {
      "position_m": [1.0, 0.0, 0.0],
      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
      "tracked": true,
      "streaming_id": 2
    }
  }
}
```

The first on-site milestone may publish Motive world coordinates only. The table-frame transform is part of the second milestone once table pivot and axis directions are verified.

### P0 Acceptance

- Official NatNet sample runs on the Windows receiver.
- Adapter or sample prints server IP, local IP, Motive version, NatNet version, command port, and data port.
- Continuous frame callbacks are observed for at least 60 seconds.
- Rigid body names and streaming IDs are visible.
- Marker data for the ball is visible and categorized as unlabeled marker, labeled marker, or rigid body.

### P1 Acceptance

- C++ adapter writes JSONL with one normalized object per received frame.
- `table` and `robot_base` are selected by configured names.
- `ego_camera` is optional and absent frames do not crash the process.
- Python reader can load the JSONL and print frame rate, missing frame count, and last known positions.

## Iteration 1 Self-Review

The minimal chain is enough to prove connectivity, but it misses several field risks:

1. It does not force capture of Motive/NatNet versions and network mode in the log header.
2. It treats ball extraction as a single step, but ball semantics are the main ambiguity.
3. It does not define behavior when Motive rigid body names change or data descriptions refresh.
4. It leaves coordinate transforms vague; downstream planning should not silently consume Motive world data as table-frame data.
5. It does not separate online latest-frame consumption from bounded recording.
6. It does not include a replay path, so estimator work would depend on live OptiTrack availability.

## Iteration 2: Refined Design

### Components

| Component | Responsibility |
|---|---|
| `NatNetSdkClient` | Wrap official `NatNetClient`, connection setup, callbacks, and shutdown. |
| `DataDescriptionRegistry` | Maintain `rigid_body_name -> streaming_id` and record version/config metadata. |
| `FrameNormalizer` | Convert NatNet frame data into project `MocapFrame` without leaking NatNet types downstream. |
| `BallSelector` | Select ball measurement from configured source and label semantics explicitly. |
| `TransformManager` | Initially labels frames as `motive_world`; later owns `T_motive_table`. |
| `LatestFrameBuffer` | Keep only the newest frame for real-time consumers. |
| `JsonlRecorder` | Write normalized frames and a session metadata record. |
| `PythonReplayTool` | Read JSONL, summarize data quality, and feed estimator prototypes offline. |

### Configuration

Configuration is explicit and stored with every recording session:

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
  "ball": {
    "source": "unlabeled_marker",
    "selection": "nearest_to_previous",
    "max_jump_m": 0.8,
    "validated_as_center": false
  },
  "output": {
    "jsonl_path": "logs/optitrack/session.jsonl",
    "udp_localhost_port": 5511
  }
}
```

### Frame Contract

Downstream code consumes only normalized fields:

- `source`
- `schema_version`
- `session_id`
- `frame_id`
- `source_timestamp_s`
- `received_monotonic_s`
- `coordinate_frame`
- `ball`
- `rigid_bodies`
- `diagnostics`

The adapter may include raw NatNet IDs in diagnostics, but estimator and planner code must not depend on NatNet SDK structs.

### Ball Semantics

The adapter never calls the initial ball measurement `ball_center` unless the experiment validates that semantics. It uses:

- `surface_marker` for one or more discrete surface markers.
- `reflective_blob_center` for a fully reflective ball that reconstructs as a single blob.
- `rigid_body_pivot` if Motive tracks the ball as a rigid body.
- `unlabeled_marker` when the exact physical semantics are not yet verified.

### Error Handling

- Missing rigid body name: log a startup error and keep running only if the track is configured as optional.
- Lost rigid body tracking: emit the object with `tracked: false` instead of dropping the whole frame.
- No ball candidate: emit `ball: null` and increment a diagnostic counter.
- Frame ID jump: record `dropped_frame_estimate` in diagnostics.
- Data description refresh: rebuild the name/id registry and write a metadata event to JSONL.
- Output failure: keep the latest-frame buffer alive; stop only if both recording and streaming outputs fail.

### On-Site Verification Order

1. Run official sample unchanged to confirm SDK, firewall, IP, and Motive streaming.
2. Run adapter in metadata-only mode to print server/client versions and rigid body names.
3. Enable JSONL recording for rigid bodies only.
4. Add ball logging with conservative semantics.
5. Move the table, base marker, and ball in known directions to verify axis signs and units.
6. Record a short session for replay.
7. Use Python replay to compute frame rate, missing frames, coordinate ranges, and tracking loss counts.

### Out Of Scope For The First Implementation

- Full AEKF integration.
- Strike planner integration.
- Spin estimation.
- Magnus-force modeling.
- Automatic calibration of `T_motive_table`.
- ROS 2 publishing on Windows.

Those are downstream tasks after the data contract is stable and field logs exist.

## Open Review Points

Before implementation, the team should confirm:

1. Exact Motive version installed on the Windows machine.
2. Whether the receiver runs on the same Windows machine as Motive or a second Windows host.
3. Ball setup: fully reflective ball, one marker, multiple markers, labeled marker, or rigid body.
4. Initial rigid body names in Motive.
5. Whether Python consumers need live UDP streaming immediately, or JSONL replay is enough for the first test day.

