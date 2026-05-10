# Debug & Prompt Log — Ackermann Perception-to-Action

## Initial AI prompts used
- Used Claude (Anthropic) to scaffold the workspace, URDF plugin integration,
  vision pipeline, and Ackermann control logic.
- First prompt context: "I have a barebones URDF (visual+collision only),
  a stripped world SDF using Ignition Fortress plugin names, on ROS 2 Jazzy
  with Gazebo Harmonic. Goal: perception-to-action pipeline detecting a box
  among 3 shape decoys, with Ackermann kinematics."

## Issues encountered & fixes

### 1. SDF file used Ignition Fortress plugin naming, not Gazebo Harmonic
- Provided `shapes.sdf` had `libignition-gazebo-physics-system.so`,
  `ignition::gazebo::systems::Physics`, etc.
- These names worked in Ignition Edifice (2021) and Fortress (2022) but
  Gazebo Harmonic (2023+) uses `gz-sim-*` and `gz::sim::*`.
- The world file would have failed to load on Harmonic.
- Fix: replaced all three plugin entries.
- Also added the missing `gz-sim-sensors-system` plugin — without it,
  the camera in the URDF is silently ignored (no /camera/image topic).

### 2. URDF mimic joint not supported by Ackermann plugin
- The provided URDF has `<mimic>` on FR_STEERING_JOINT for symmetric steering.
- Gazebo Harmonic's Ackermann plugin commands left and right steering
  joints independently — mimic conflicts.
- Note: real Ackermann geometry actually requires asymmetric steering
  (inner wheel turns more) but mimic with multiplier=1 forces symmetric.
- Fix: removed `<mimic>` and let the plugin drive both joints.

### 3. (FILL IN AS WE GO)

## Systematic debugging approach
- Verified each layer in isolation: `gz topic -l`, `ros2 topic list`,
  `ros2 topic hz`, `gz topic -e -t /...` at every stage.
- Built incrementally: workspace → URDF parse → Gazebo spawn →
  topic publish → bridge → ROS subscriber → control logic.
- Used `xacro <file>` to validate URDF before any colcon build.
- (FILL IN MORE AS WE GO)
