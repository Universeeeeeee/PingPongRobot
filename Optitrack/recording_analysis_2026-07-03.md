# OptiTrack NatNet Recording Analysis - 2026-07-03

Source recording: `/Users/vae/Downloads/录屏.mp4`

## Scope

This analysis is based on the screen recording of the `HOPE NatNet Python Viewer`.
It is useful for spotting visible symptoms, but it is not a replacement for raw
NatNet logging from the C++ callback.

## Recording Metadata

- Duration: 20.35 s
- Video frames: 406
- Video frame rate: 20 fps
- Resolution: 1772 x 1218

Important implication: the screen recording is only 20 fps. It cannot directly
prove the real Motive/NatNet data rate. Use the viewer's displayed `frame`, `dt`,
and `age` fields instead.

## Main Finding

The recording does not support the simple conclusion that the real NatNet stream
is only 60-120 Hz.

Two readable samples from the viewer show:

- Around the first sample: `frame=3087542`
- Around the last sample, about 20 s later: `frame=3094741`

That is a delta of 7199 Motive frames over about 20 s:

```text
7199 / 20.0 = 359.95 Hz
```

So the Motive frame counter is advancing at essentially the expected 360 Hz over
the full recording window.

## Observed Timing Symptoms

Most sampled frames show:

- `dt=0.0028s`, equivalent to about 357 Hz
- `missed=0`
- `break=False`
- `bridge=running`

However, there are intermittent timing anomalies:

- `dt=0.0056s`, equivalent to about 179 Hz
- `dt=0.0083s`, equivalent to about 120 Hz
- `dt=0.0111s`, equivalent to about 90 Hz
- one extreme sample with `dt=0.0001s`

The `dt=0.0001s` sample produced impossible derivative values:

- `frame=3089481`
- `speed=67.63 m/s`
- `acc_norm=1241285.5 m/s^2`

This looks like a derivative/filtering problem caused by a tiny `dt`, duplicate
or near-duplicate timestamp, or candidate switch. It should not be treated as
physical ball motion.

## Observed Latency Symptoms

Most sampled `age` values are low, commonly around 1-20 ms. There are visible
latency spikes:

- about 46 ms
- about 56 ms
- about 63 ms
- one large spike: `age=0.217s`

The 217 ms spike is a strong sign of stale data reaching the Python viewer, UI
delay, bridge queue backlog, or network jitter. Because the overall Motive frame
counter still advances at about 360 Hz, this spike is more likely a latency or
queueing symptom than a global camera-rate drop.

## Candidate And Validity Symptoms

The viewer shows several candidate/selection anomalies:

- `source=other selected=-1 valid=False candidates=0`
- `confidence=0.0 residual=-1.0000`
- later samples with `candidates=2`
- final sample with `candidates=4 selected=3`

Representative invalid sample:

```text
frame=3088794 source=other selected=-1
valid=False candidates=0 table=True
confidence=0.0 residual=-1.0000
nearest_keypoint=0.000 m
```

The next readable recovery sample shows a large velocity/acceleration jump:

```text
frame=3088972 source=labeled selected=0
speed=8.68 m/s acc_norm=394.8 m/s^2
```

This suggests the ball estimator should reset velocity/acceleration when the
source changes, validity is lost, selected candidate changes, or the selected
marker identity is not continuous.

## Rigid Body Symptoms

Most samples show both rigid bodies tracked:

```text
id=1 name=PPT tracked=True
id=2 name=qiupai tracked=True
```

At the end of the recording there is a clear rigid-body tracking loss:

```text
frame=3094741 source=labeled selected=3
valid=True candidates=4 table=True
qiupai present=True tracked=False
id=2 name=qiupai tracked=False
nearest_keypoint=0.060 m
```

This is consistent with ambiguity near the racket rigid body. The ball selection
logic should explicitly reject markers that belong to known rigid bodies, and it
should treat candidate switches near `qiupai` as lower-confidence measurements.

## Network Interpretation

Using a hotspot/Wi-Fi can absolutely cause UDP jitter, packet loss, and bursty
latency. Wired Ethernet is still the right validation setup.

But in this recording, the strongest evidence is:

- Motive frame id advances at about 360 Hz over the full 20 s.
- Many samples show `dt=0.0028s`.
- Some samples show larger `dt`, high `age`, invalid source, or candidate
  ambiguity.

So the likely problem is not "Motive only sends 60-120 Hz" globally. A better
working hypothesis is:

1. Motive/camera is producing 360 Hz frames.
2. The selected ball measurement is intermittently invalid, duplicated, skipped,
   or switched between candidates.
3. The C++ bridge / Python viewer / network path occasionally introduces stale
   frames or queueing delay.
4. Derived velocity and acceleration are currently too sensitive to tiny `dt`
   and candidate discontinuities.

## Recommended Next Test

Run two Windows-side captures with raw JSONL logging enabled:

1. Wired Ethernet, 30 s idle, 30 s hitting.
2. Hotspot/Wi-Fi, same scene, same motions.

Log at the C++ NatNet callback before any Python UI throttling:

- receive monotonic timestamp
- NatNet frame id
- NatNet timestamp
- receive-to-receive delta
- source timestamp delta
- `age`
- all rigid body ids, names, poses, and tracked flags
- all labeled marker ids, positions, sizes, residuals, and params
- selected ball candidate id/index
- number of candidates
- selected position
- selected velocity/acceleration
- validity/filter reason
- frame gap or dropped-frame count

Then compute:

- frame-id delta histogram
- source timestamp delta histogram
- receive wall-time delta histogram
- `age` p50/p90/p99/max
- invalid-frame ratio
- candidate-count distribution
- selected-candidate switch count
- rigid-body tracking loss windows
- velocity/acceleration outlier count

## Code Recommendations

- Do not infer data rate from the Python GUI refresh rate.
- Keep the UI at 30-60 Hz, but log every C++ callback frame.
- Use a latest-frame ring buffer for UI display, not an unbounded queue.
- Reset derivative state when `valid=False`, `source` changes, frame id jumps,
  selected candidate changes, or `dt` is outside a sane range.
- Clamp derivative calculation when `dt` is too small, for example below about
  1 ms for a 360 Hz system.
- Report separate rates:
  - Motive frame-id rate
  - C++ callback receive rate
  - valid ball selection rate
  - Python/UI render rate
- Add a clear on-screen diagnostic panel showing p50/p99/max `age`, frame drops,
  invalid count, candidate switches, and rigid body tracking loss.

