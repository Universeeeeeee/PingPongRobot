# ChatGPT Context Summary

Date: 2026-06-24

Sources:
- https://chatgpt.com/share/6a3b4877-9010-83ea-89e1-60a53f5640c3 (`2.SMASH`)
- https://chatgpt.com/share/6a3b492c-e71c-83ea-9578-c2fd0dd0a607 (`1.proposal`)

This file is a compressed working memory from two shared ChatGPT conversations. Treat it as derived context, not as a primary source. For paper-accurate claims about SMASH, verify against the SMASH paper itself.

## Project Context

The user is working on a humanoid robot table-tennis project, especially the perception side for robot-owned vision. The relevant group appears to be responsible for vision/perception, state estimation, prediction, confidence, and interfaces to planning/control.

The recurring research direction is:

External multi-view teacher -> synchronized ego-exo dataset -> ego-view student perception -> ball state / future trajectory / hit target interface -> active perception -> real robot hit validation.

The user already has or discusses an external multi-view system that can estimate ball position and velocity accurately. The proposed route should not discard it; it should use it as a teacher / ground-truth reference to train and evaluate robot egocentric perception.

## Core Thesis To Preserve

Do not frame the work as "directly reproducing SMASH" or "immediately replacing all external perception with ego-only perception."

Frame it as:

Reproduce SMASH's capability boundary and perception-control interface, then build a verifiable ego-exo transfer path using the existing external multi-view system as teacher and the robot's onboard camera as student.

The strongest near-term paper/proposal main line is:

1. Standardize the external multi-view teacher.
2. Collect synchronized ego-exo data.
3. Align coordinate frames and timestamps.
4. Train an ego-view student to estimate ball state and future trajectory.
5. Output a unified hit interface, especially `p_hit`, `v_hit`, and `tau`.
6. Add active perception as a feedback loop to reduce uncertainty / field-of-view loss.
7. Validate on real robot hitting.

Opponent intent recognition, strategy scoring, weakness analysis, and sophisticated spin reasoning are useful extensions, but should not be the first paper's core burden.

## SMASH Understanding

SMASH refers to `Mastering Scalable Whole-Body Skills for Humanoid Ping-Pong with Egocentric Vision`.

Important interpretation:

- SMASH is relevant because it targets humanoid ping-pong with onboard egocentric vision, not only robot-arm ping-pong or external-camera table tennis.
- SMASH should be treated as a target-type baseline: use its system boundary and interface as inspiration, not as a fully reproducible engineering recipe.
- Public material may not expose every visual engineering detail, such as complete camera layout, detection network details, synchronization, calibration, latency decomposition, or full code.

SMASH perception is best understood as two replaceable sensing modes:

- MoCap mode: high-accuracy external sensing for validation and tuning.
- Ego-view camera mode: onboard deployment-oriented sensing.

Both modes output compatible state information, then share the same backend:

Perception source -> ball position and torso pose -> adaptive EKF / state estimation -> physics-based trajectory prediction -> racket strike planning -> motion matching plus whole-body policy -> whole-body action output.

Do not describe SMASH as using MoCap to train an ego-view visual model unless the paper explicitly says so. In the shared discussion, the conclusion was that SMASH does not present an ego-exo teacher-student training framework.

SMASH ego-view details from the shared discussion:

- Head-mounted ZED X: long-range ball tracking, stereo triangulation, about 60 Hz.
- Downward ZED X Mini: AprilTag-based table / robot relative pose estimation, about 120 Hz.
- Ball detection pipeline: YOLO coarse detection, HSV refinement inside the bounding box, stereo triangulation from left/right image centers.
- Robot self-localization: AprilTag corners -> PnP / RANSAC-PnP -> transform into the required control frame.
- Key reason for two cameras: looking at the ball and maintaining stable robot/table localization are different visual tasks.

SMASH training should be explained as motion/control training more than visual training:

- Collect about 400 human whole-body striking MoCap motions.
- Retarget human motions to Unitree G1.
- Train a conditional/autoregressive Motion-VAE to expand the strike motion library.
- Filter generated motions with a tracker-based physical executability check.
- Train whole-body control with task-oriented motion matching and PPO / asymmetric actor-critic.
- Policy inputs include task targets such as `p_hit`, `v_hit`, `tau`, matched motion, and proprioception.
- Deployment plugs perception-derived task targets into the trained control stack.

Critical distinction:

Training and deployment are not the same thing. The controller may be trained with motion libraries and privileged information, while deployment uses perception outputs to generate task targets.

## Recommended Proposal Direction

The proposal's current structure is broadly complete but too broad. It includes related work, research content, technical route, innovation points, timeline, milestones, and demo goals, but the main line should be sharpened.

Main missing bridge:

External multi-view -> ego-view perception.

The proposal needs to explicitly define:

- Ego-exo synchronized data collection.
- What labels the external teacher outputs.
- What state the onboard student learns.
- How teacher and student coordinate frames are aligned.
- How timestamp skew and latency are measured or compensated.
- Whether and how teacher fallback exists during development.
- How to quantify the ego-teacher gap.

Do not make "ego-only with no external sensing" the immediate near-term claim. Better:

- Training/evaluation stage: external multi-view teacher plus robot onboard camera.
- Deployment stage: gradually reduce teacher dependency toward ego-only inference.

## Proposed Interface

Prefer one unified perception interface used by both teacher and student.

A useful shape:

```text
PerceptionState:
  frame_id
  capture_timestamp
  publish_timestamp
  source: exo_teacher | ego_student | fused
  ball_position
  ball_velocity
  predicted_trajectory
  predicted_bounce
  p_hit
  v_hit
  tau / time_to_hit
  confidence
  covariance / uncertainty
  latency
  sync_skew
  robot_base_pose
  head_pose
  racket_pose
  failure_flag
```

This interface is important because it lets teacher and student be compared, swapped, fused, and connected to planner/controller without changing downstream modules.

## Active Perception Framing

Active perception should be drawn as a feedback loop, not a sequential middle block.

Better pipeline:

```text
Ego image + proprioception
  -> ball / opponent / racket perception
  -> state estimation + prediction
  -> hit target generation
  -> controller

Active perception:
  state uncertainty / FOV risk
  -> head gaze command
  -> affects next ego image
```

Near-term active perception can start with rule-based gaze control or uncertainty-driven heuristics. Learning-based active perception can be a later extension.

## Proposal Risks To Watch

- "Reproduce SMASH" is too strong; say "reproduce its capability boundary and interface."
- Jumping directly from external vision to onboard vision lacks a method bridge.
- Ego-only as a near-term demo is too aggressive.
- Timeline must not require ICRA integration before modules scheduled after the submission window.
- Fifth group scope should stay around vision -> state -> prediction -> confidence -> planner/control interface.
- Intent recognition and strategy scoring overlap with other groups and should be extensions.
- Spin should not be overpromised from ordinary onboard RGB. A staged route is better: trajectory residual first, high-speed/event-camera teacher later, spin-aware residual or spin prior last.
- Related work should be organized by research gap, not as a flat paper list.
- Evaluation metrics should diagnose the system, not only report final hit success.

## Suggested Milestones

Before submission / first system paper:

1. External teacher standardization.
2. Ego-exo synchronized dataset.
3. Ego ball-state baseline.
4. Hit point and time-to-hit output.
5. Active perception ablation.
6. Real robot hitting demo.

Post-submission or second-stage extensions:

- Opponent intent.
- Spin fusion.
- Strategy-aware return.
- Weakness analysis.
- Self-play / tactical policy integration.

## Writing Style Guidance

When helping the user write slides, proposal text, or a spoken script:

- Keep the main story narrow and verifiable.
- Lead with the ego-exo teacher/student bridge.
- Use SMASH as motivation and interface baseline, not as something to copy blindly.
- State that the existing external perception system is an asset, not a crutch.
- Separate "what we can validate now" from "long-term ego-only autonomy."
- Be concrete about interfaces, timestamps, coordinate frames, latency, uncertainty, and ablations.

