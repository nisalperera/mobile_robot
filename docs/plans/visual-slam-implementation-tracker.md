# Mobile Robot Repository Repair Plan

## Active revision: 2026-11-25, 01:15 +0530 review session

Repository: nisalperera/mobile_robot
Branch: feature/localization
Pinned reviewed HEAD: bf57ba3bbac1db56005b89758f465e85cb26d2e9
Previous reviewed HEAD: 66294a1e432565262783ed640e249f677e3b1dbd
New commit: 'modulerized camera setup and use two cameras'
Commit timestamp: 2026-11-24 19:43:15 UTC, or 2026-11-25 01:13:15 +0530.

This is the current plan. It supersedes the previous generated review and the active instructions in the historical attachment preserved below. Archived statuses, five-camera requirements and next actions are historical, not current work orders.

## 1. Objective and evidence boundary

Repair the existing simulation stack through small, independently testable changes. Preserve the working isolated mecanum baseline, complete laptop simulation before Jetson or physical integration, and do not migrate Fortress or rewrite unrelated functionality during this repair.

Primary project reference: robotics-project-guide.md, consulted again for this revision. Latest explicit user changes and the current committed two-camera model supersede older five-camera repair targets.

Evidence classes:
- Source verified: current files and commit patch inspected through GitHub.
- Historical runtime: user observations and capture summaries retained in the original plan.
- Author reported: localization-working commit message, not independent test evidence.
- Recommended: work and validation proposed here, not executed.
- Unknown: any still-uncommitted local changes, current installed overlay, effective runtime parameters and new test results.

The newly published commit contains five changed files, 14 additions and 43 deletions. This review examines that complete patch and current key files, with unchanged architecture carried forward from the prior source review. It does not imply access to a local worktree or comprehensive execution/testing.

No repository edits, commits, pushes, ROS builds, simulator runs, or physical tests were performed here. Only this Markdown deliverable was generated. No successful new runtime audit has been supplied.

### Review inventory

New commit patch inspected for all five changed files:
- config/nav2_params.yaml
- description/xacro/front_camera.xacro
- description/xacro/left_cameras.xacro
- description/xacro/right_cameras.xacro
- rviz/maps.rviz

Current full-file reads additionally cover:
- description/xacro/camera.xacro
- description/xacro/robot_core.xacro
- config/ros_gz_bridge.yaml
- launch/navigation.launch.py
- config/nav2_params.yaml
- rviz/maps.rviz

Prior-review source evidence retained for unchanged launch/localization.launch.py, launch/gz.launch.py, launch/robot.launch.py, config/ekf.yaml, config/twist_mux.yaml, description/xacro/ros2_control.xacro, CMakeLists.txt and package.xml. navigation.launch.py was re-read and its blob SHA remains ebb63176137b9b0c147574afa0a262fc23e182ae.

## 2. What changed and what must be removed from the old plan

| Previous finding/target | New source evidence | Current disposition |
|---|---|---|
| Nested footprint YAML blocks expected to fail ROS parameter loading | Both footprints are now quoted strings | Source fix complete; remove serialization repair task; runtime loading still needs evidence |
| Five active camera definitions | Front file has no camera instance; left/right files each instantiate one | Current default target is two cameras, not five |
| Camera model names differ from old bridge names | Prefixes are left_camera and right_camera again | Name mismatch repaired in source; do not build five-camera bridges |
| Front camera entrypoint needed as active sensor | front_camera.xacro is an empty robot wrapper with stale comment | Harmless inactive include pending cleanup, not a missing-sensor failure |
| RViz maps profile shows static /map | Static /map display removed; remaining Map display is /global_costmap/costmap | Add clear static-map and costmap displays for localization validation |
| RViz scan reliability | LaserScan display now Best Effort | Record change, but it still displays /scan rather than canonical /scan_fixed |
| Optional TF callback fix absent | Fix was committed in the earlier integration commit | Do not reapply blindly; successful runtime audit still pending |

Do not report the two-camera name fix as successful sensor delivery. Image/CameraInfo timing, frames, calibration and rendering are separate gates.

## 3. Current milestone and progress

Completed historically: isolated low-speed four-wheel holonomic translation, repeated bidirectional yaw at two rates, combined motion and visual stopping checks under the original documented conditions.

Implemented in source: differential EKF raw/fused separation, simulation IMU yaw-rate override, controller TF disable and provisional covariance, optional audit metadata handling, two-camera modular model, and footprint string serialization.

Author reported: localization works and needs fine tuning at commit 8e08ea2884c86ada68570766e1d74701651ccf27. This report predates both footprint changes and the current camera revision. It is not current-HEAD acceptance evidence.

Current milestone: obtain a reproducible current differential localization/navigation baseline, repair command arbitration and the physical footprint contract, then integrate the separately validated holonomic drivetrain deliberately.

| Repair phase | Status at reviewed HEAD | Remaining exit gate |
|---|---|---|
| 1. Baseline | Historical local baseline complete; remote snapshot refreshed | Local SHA/diff, installed share and external-plugin provenance |
| 2. TF/odometry | Differential source wiring repaired; historical runtime evidence retained | Current effective inputs, per-edge ownership, covariance and continuity |
| 3. Model | Two cameras, plate and planar lidar committed; isolated holonomic subset passed historically | Current Xacro expansion, sensor counts, measured/placeholder separation, complete collision envelope |
| 4. Sensors | Two-camera names aligned in source; typed relays partial | Two matching image/calibration pairs and direct planar scan validation |
| 5. Launch ownership | Dedicated mapping/localization entrypoints exist | Deprecate overlapping legacy launcher; one owner per component |
| 6. Nav2/commands | Footprint serialization fixed; routing and motion-model mismatch remain | Correct footprint geometry, smoothed arbitration, consistent active drivetrain profile |
| 7. Visualization/dependencies | RViz edited; static map no longer shown | Explicit profiles, direct dependency declarations, reproducible external packages |
| 8. Acceptance | Historical isolated motion subset only | Current-stack stationary, navigation, failure and clean-rebuild evidence |

## 4. Architecture to preserve and distinguish

### Shared mapping/localization profile

```text
mecanum_ign_hardware/MecanumSystem
  -> left_wheels/right_wheels virtual interfaces
  -> diff_drive_controller
  -> /diff_drive_controller/odom

/diff_drive_controller/odom + /imu_fixed
  -> EKF -> /odom and intended odom -> base_footprint TF

Mapping: SLAM Toolbox supplies intended map -> odom
Localization: AMCL supplies intended map -> odom
Mux output: /diff_drive_controller/cmd_vel_unstamped
```

This is differential-style operation of a four-wheel model. Shared Xacro and launch paths have not become fully holonomic merely because the wheels are called mecanum.

### Isolated holonomic test profile

Preserve mecanum_bringup_test.launch.py and mecanum_bringup_test.yaml:
- mecanum_four_wheel_hardware/FourWheelSystem, four independent wheel commands/states.
- mecanum_drive_controller, unstamped Twist on /mecanum_controller/reference_unstamped.
- /mecanum_controller/odometry, child mecanum_test_center, controller TF disabled.
- Radius .024 m, geometric sum .275 m, zero configured controller frame offset.
- Validated collision override .001 m; launch default remains .025 m.
- Working friction reference/directions, namespace handling and frame injection stay intact.

Its launch bypasses the normal simulation sensor bridges/relays/spawners and creates its own description/control arrangement. Historical success does not validate ordinary localization/sensor startup. External FourWheelSystem source/build/version provenance remains to capture.

Full holonomic intent is established for the isolated simulation. The unresolved issue is integration into the shared stack and eventually hardware, not whether the completed isolated lateral tests occurred.

## 5. Remaining findings, ranked

### P0-A. Navigation smoother bypass and recovery arbitration

Files: launch/navigation.launch.py, config/twist_mux.yaml, launch/localization.launch.py.

Current source routing:

```text
controller_server -> cmd_vel_nav -> twist_mux -> drivetrain
                             \-> velocity_smoother -> cmd_vel
behavior_server -> cmd_vel (no explicit navigation-input remap)
```

The mux consumes the unsmoothed stream. The smoother output is not the configured navigation mux input, and recovery behavior commands do not have an explicit path through that same input. This finding remains after the new commit. Both composed and standalone launch branches have the same routing concern.

Recommended contract:

```text
controller_server + behavior_server -> cmd_vel_nav_raw
cmd_vel_nav_raw -> velocity_smoother -> cmd_vel_nav
cmd_vel_nav + cmd_vel_joy -> twist_mux -> active drivetrain endpoint
```

Before patching, capture actual publishers/subscribers and verify message types, velocity-smoother effective parameters and lifecycle state on installed Humble. No dedicated velocity_smoother parameter block appears in the inspected nav2_params.yaml; defaults alone must not be assumed appropriate.

Retain joystick priority 100 over navigation 10. Record smoother limits, mux selection/release, controller stale-command timeout and stopping responsibility separately. A mux timeout is not proof of a safe stop.

Exit gate: autonomous and recovery commands pass through smoothing and arbitration; only the intended mux drives the controller; joystick takeover/release works; command-loss stopping is independently measured. If composition is supported, an existing component container must be explicitly provided/documented because navigation.launch.py loads components into one rather than creating it.

### P0-B. Footprint format is fixed, geometry still under-covers the plate

File: config/nav2_params.yaml; geometry evidence: robot_core.xacro.

Both current footprints are valid-looking quoted coordinate strings. Do not reapply the serialization patch.

The polygon is still a centered .40 x .40 m square: x/y each [-.20,+.20]. The source plate is .350 x .300 m, centered x=.150 in chassis, with chassis x=-.100 relative to base_link and no base_footprint/base_link XY offset. Therefore plate center is x=.050 in base_footprint, and its X bounds are [-.125,+.225] m. The polygon front edge at +.200 m under-covers the modeled plate by .025 m before any desired margin. This is a source-derived planar bound, not an independently measured physical dimension.

Derive the complete footprint from expanded wheel, chassis, camera and other collision/protrusion bounds in the chosen base frame. The current two-camera locations do not justify assuming the square is adequate. Keep the .001 m isolated wheel contact proxy separate from navigation/physical width.

robot_radius=.22 remains alongside the polygon. Do not classify coexistence alone as a startup failure; choose/document one primary geometry contract and confirm effective behavior in installed Nav2. Global costmap resolution remains implicit since the earlier commit; capture actual resolution/map interaction before changing it.

Exit gate: both costmaps activate with the intended polygon, model overlay is contained with documented margin, turning/clearance tests use that same reference frame, and no artificial wheel-contact width determines the navigation envelope.

### P0-C. Differential drivetrain versus lateral navigation mismatch

Files: nav2_params.yaml, ekf.yaml, ros2_control.xacro and shared launch chain.

Current shared profile: differential controller, differential AMCL motion model, EKF wheel vx/yaw-rate but not vy. Current DWB: max_vel_y=.26, acc_lim_y=2.5 and vy_samples=5. The planner/controller configuration can consider an axis the active two-side drivetrain cannot execute.

Keep coherent profiles rather than changing only AMCL or only EKF:
1. Differential regression: retain current shared controller/estimator and disallow lateral DWB trajectories.
2. Holonomic integration: select four independent wheel interfaces and mecanum controller, solve center/base transform semantics, fuse valid lateral wheel velocity, verify the installed AMCL omni plugin, configure bilateral lateral limits and all routing/timeout owners together.

DWB min_vel_y=0 excludes negative lateral sampling and min_vel_x=0 excludes reverse sampling. Set these intentionally for the selected profile, not by mechanical copy from the isolated test.

Resolve the .05 m center offset and single-parent TF tree before feeding center odometry into the shared estimator. Renaming child_frame_id is not a pose/twist point transform. Keep controller TF disabled if EKF owns odom -> navigation base. Preserve SLAM/AMCL map authority separation.

Exit gate: available command axes match actual kinematics; raw odometry and estimator are reference-point consistent; integrated signed vx/vy/yaw tests pass under defined conditions. Do not rerun old isolated tests merely to delay integration design.

### P1-A. Two-camera delivery and calibration, not five-camera expansion

Files: camera.xacro, left_cameras.xacro, right_cameras.xacro, front_camera.xacro, gz.launch.py and bridge configuration.

Current model instances, relative to chassis:

| Prefix | XYZ in metres | Yaw | Optical frame |
|---|---|---|---|
| left_camera | (.280,+.115,.056) | 0 | left_camera_link_optical |
| right_camera | (.280,-.115,.056) | 0 | right_camera_link_optical |

Both cameras face forward. They are not the previous outward five-camera coverage arrangement. Their nominal housing-origin lateral separation is .230 m, but this alone does not establish a calibrated stereo system. Do not introduce stereo processing or a five-camera requirement without a new explicit scope decision.

Macro source specifies 1280x800 RGB, 10 Hz and a simplified wide pinhole model. These are configured values, not measured stream rates or calibrated physical optics. Candidate-model comments do not prove hardware ownership/purchase.

The existing bridge names now align. Remaining checks:
- Determine actual Ignition image and CameraInfo topic names.
- Verify two live image streams and two calibration streams.
- Ensure corrected Image and CameraInfo headers use the same corresponding optical frame and consistent timestamps/dimensions/calibration.
- Current inline FrameRelay lacks CameraInfo support. Add matched handling if runtime demonstrates the existing namespaced-frame problem for calibration; avoid unnecessary relabeling if already correct.
- Never fabricate samples/calibration. Preserve payload and stamps.
- Centralize relay implementation rather than maintaining multiple inline copies when making this patch.

Remove stale comments about three new cameras or all-two-left instances. The empty front_camera.xacro include can be removed as a separate cleanup after successful expansion. No missing front-camera fault should be raised for the current design.

Exit gate: two complete image/calibration pairs, valid TF, no obsolete topic subscriptions, measured rates/load, and warning behavior on missing input. Camera completion is a separate gate from lidar-only localization acceptance.

### P1-B. Planar lidar still uses legacy point-cloud conversion

The lidar is configured as one vertical ring and 500 azimuth samples, while gz.launch.py retains scan_2d/points -> pointcloud_to_laserscan -> /scan -> /scan_fixed. Inspect conversion settings and compare native bridged /scan_2d against the current output before replacing it.

Preferred final default: native LaserScan bridge -> one frame correction -> /scan_fixed. Validate ranges, frame, timing, signs and scan alignment before disabling converter. Preserve an explicit legacy 3D mode only if still required. No blind sensor/package rewrite.

Exit gate: correctly framed intended 500-range scans at measured nominal-rate behavior, no duplicate corrected publishers, mapping/localization consumers on the agreed corrected topic.

### P1-C. Declarative bridge is dormant and divergent

File: config/ros_gz_bridge.yaml, newly inspected in full.

It states it is not loaded by launch. The active gz.launch.py uses inline arguments. Differences include:
- Dormant YAML maps model odometry to /diff_drive_controller/odom, while the active launch does not apply that same remap.
- Dormant IMU entry uses imu_fixed and gz.msgs.IMU rather than the active Fortress /imu bridge and correction path.
- Both source comments and YAML retain obsolete 3D lidar descriptions.

Do not switch to this YAML merely to centralize bridges. First reconcile it to the selected active topics/types and explicit odometry ownership. Loading it unreviewed could change the wheel-odometry source or duplicate a controller publisher.

Exit gate: one authoritative bridge definition, explicit selected backend message types, no accidental second odometry authority, sensor contracts documented.

### P1-D. Legacy bringup duplicates ownership

Prior source review of unchanged robot.launch.py found overlapping immediate/delayed SLAM includes, nested localization that starts its own simulator/navigation/mux, and a second navigation include at parent level. RSP ownership comments are inconsistent with gz.launch.py. Do not use this as the canonical supported launcher.

Deprecate with a clear diagnostic or convert to a thin exclusive dispatcher after mapping/localization ownership is checked. Avoid unrelated YOLO refactoring.

Exit gate: one simulator/entity/controller stack, RSP, EKF, mux and navigation instance; exactly SLAM or AMCL for map -> odom, never both. Reject unsupported argument combinations.

### P1-E. Duplicate keys and tuning semantics

behavior_server repeats max_rotational_vel, min_rotational_vel and rotational_acc_lim. Consolidate each to a single deliberate value after checking intended limits, rather than accepting silent parser overwrite. Verify installed parameter names, including controller max_y_velocity_threshold and behavior plugin-list settings, before assuming they are effective.

DWB trans_stopped_velocity=.25 is close to configured max_speed_xy=.26 and greater than the historical low-speed motion tests. Review goal-stopping classification against measured speeds; do not tune unrelated planner critics blindly. Snapshot existing limits and change one concern per patch.

Simulation EKF launch overrides base YAML to use /imu_fixed yaw rate only, with gravity removal false; hardware inherits different settings via /imu/data. Update stale logs/comments to show effective precedence, not just base YAML defaults. Hardware operation remains unvalidated.

### P1-F. RViz localization profile no longer shows static map

Current maps.rviz:
- Fixed frame map.
- Only remaining Map display reads /global_costmap/costmap and is named Map.
- LaserScan reads /scan, now Best Effort, not canonical /scan_fixed.
- RobotModel robot_description subscription remains Volatile.
- No camera image displays in the inspected profile.

Restore separate, uniquely named Static Map (/map), Global Costmap and Local Costmap displays as appropriate; show /scan_fixed for end-to-end localization debugging. Use description/map subscription durability appropriate to actual publishers, and validate late RViz startup. A missing static-map display is not proof map_server failed. Best Effort scan subscription alone is not evidence scan frame/timing is valid.

Maintain distinct model(base), odometry(odom) and navigation(map) profiles. Add two-camera inspection displays where useful without making camera rendering a requirement for every navigation debug session.

Exit gate: static map versus inflated costs visually distinguishable, corrected scans align in map frame, model appears after late startup, no duplicate names, camera views explicitly scoped.

### P2. Dependencies and reproducibility

Prior unchanged manifest/CMake review found undeclared direct runtime dependencies for robot_localization, imu_complementary_filter, gz_ros2_control and inline rclpy/sensor_msgs usage. Review exact dependencies and external-package installation/build provenance. tools are not installed by the current resource-directory install rule; keep source-invoked development tools documented or install them intentionally.

Record FourWheelSystem and mecanum controller source/version/library provenance. Do not invent a Humble binary installation route. Confirm current image/package manifests because historical captures predate added tooling.

Preserve Fortress as the repair baseline. Harmonic migration, verified NVIDIA renderer selection, Jetson ARM64/container compatibility, hardware interface/firmware and motor/power safety remain separately reviewed tracks. Physical robot operation must not depend on Gazebo.

## 6. Revised patch sequence

Each patch must contain exact diff/files, dependencies, execution location, build/validation commands, expected outcomes, actual evidence and rollback. No remote write without explicit approval.

| Patch | Scope | Exit gate |
|---|---|---|
| R0 | Current local SHA/diff, active share, environment and external package inventory; stationary graph capture | Recoverable source baseline and actual runtime mode identified |
| R1 | Duplicate YAML keys and model-derived footprint geometry, preserving already-correct string format | ROS parameter loading, costmap activation and envelope overlay |
| R2 | Controller/recovery -> smoother -> mux routing in both launch modes; deliberate limits | Endpoint, lifecycle, joystick and command-loss evidence |
| R3 | Coherent differential Nav2 profile plus exclusive/deprecated legacy entrypoint | No unsupported lateral command; one owner per component |
| R4a | Direct planar scan path after native-versus-converted comparison | Corrected scans and localization alignment preserved |
| R4b | Two-camera calibration/frame validation and minimal relay repair | Two image/calibration pairs, no five-camera scope expansion |
| R4c | RViz profiles, authoritative bridge config and stale comments | Clear visualization and one documented sensor/odometry source contract |
| R5 | Explicit holonomic integration profile with correct TF/point semantics, lateral estimator and AMCL/controller settings | Integrated signed motion and localization checks, rollback preserved |
| R6 | Navigation, failure handling, clean rebuild and documentation | Evidence-backed completion before Jetson/hardware work |

Do not put the footprint serialization fix back on the work queue. Do not restore five cameras simply because the historical attachment requested them. Preserve the isolated contact/friction corrections and short-test records; do not tune wheel geometry to hide integration errors.

## 7. Proposed local evidence and validation commands

All commands below are proposed, not executed here. Execution location: laptop container. Working directory: /home/nisalperera/projects. No pull, reset or clean is required.

```bash
cd /home/nisalperera/projects
source /opt/ros/humble/setup.bash
source install/setup.bash

git -C mobile_robot status --short --branch
git -C mobile_robot rev-parse HEAD
git -C mobile_robot diff --stat
ros2 pkg prefix --share mobile_robot
ros2 node list
ros2 control list_controllers -c /controller_manager
ros2 control list_hardware_interfaces -c /controller_manager
ros2 topic info /cmd_vel_nav --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /diff_drive_controller/cmd_vel_unstamped --verbose
ros2 topic info /diff_drive_controller/odom --verbose
ros2 topic info /odom --verbose
ros2 param dump /ekf_filter_node
ros2 param dump /velocity_smoother
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
```

Run against the already active stack. Missing nodes may simply mean isolated/mapping mode. Record exact entrypoint/arguments, source versus installed equality and whether the robot is stationary. Do not start another simulator to satisfy diagnostics.

```bash
# Execution location: laptop container, same sourced workspace
/usr/bin/python3 -u mobile_robot/tools/mecanum_integration_audit.py \
  --seconds 20 \
  --mapping-launch mobile_robot/launch/mapping.launch.py
```

The mapping flag selects source for review, not a mapping launch. The optional-info callback is already fixed in committed source; check local parity instead of reapplying. Full A/B/C output and local JSON remain required. The audit cannot prove per-edge TF publisher ownership if message metadata is unavailable, excludes legacy plugin source from automatic discovery, can truncate source/parameter selections and does not validate image/CameraInfo streams. Supplement these gaps explicitly.

```bash
# Execution location: laptop container, same sourced workspace
# Read-only camera endpoint and calibration checks
ros2 topic info /left_camera/image --verbose
ros2 topic info /right_camera/image --verbose
ros2 topic info /left_camera/camera_info --verbose
ros2 topic info /right_camera/camera_info --verbose
ros2 topic echo /left_camera/camera_info --once --qos-reliability best_effort
ros2 topic echo /right_camera/camera_info --once --qos-reliability best_effort
ros2 param get /local_costmap/local_costmap footprint
ros2 param get /global_costmap/global_costmap footprint
```

Use actual discovered CameraInfo topics if they differ. Echo commands wait for messages; absence is not automatic proof of bad model. Validate image headers on both original and corrected outputs, not just endpoint discovery. For a healthy non-camera stack, missing camera data does not invalidate lidar-only localization.

## 8. Acceptance and rollback

| Gate | Required evidence |
|---|---|
| Static | Syntax, duplicate-key rejection, installed ROS parameter parser, Xacro sim/hardware expansions and connected unique tree |
| Model | Two camera instances, intended lidar/IMU, valid inertias/collisions, complete base-frame footprint bounds |
| Stationary | Clock progress, single map/odom TF owners, timestamped lookups, scan alignment and no pose switching |
| Commands | Smoothed autonomous/recovery routing, mux priorities, controller command type, timeout/stop behavior |
| Sensors | Planar scan plus two image/calibration pairs with correct frames/timing/QoS |
| Localization | Map/world/spawn/initial-pose provenance, initial acquisition and repeatable stationary/moving alignment |
| Holonomic integration | Correct four independent wheel interfaces, lateral odometry/EKF, reference-point consistency and signed motion |
| Navigation | Goals, obstacles, recoveries, goal stopping, repeated runs and arbitration |
| Failures | Stale commands, sensor loss, clock pause/reset, controller/process/container restart, network loss where relevant |
| Reproducibility | Clean build/install, pinned external dependencies, active-share/source equality and documented restore procedure |

Use measured baselines and explicit requirements to establish new tolerances. Historical isolated test limits are not general navigation or hardware accuracy requirements. Keep known-good differential and isolated launch/config variants before each patch. Revert only the reviewed patch against the captured local baseline; do not overwrite unrelated local edits.

## 9. Immediate next action

Capture R0 from the existing stationary runtime if available, then deliver R1 for duplicate keys and actual footprint envelope. The earlier first task of converting footprint YAML to strings is complete in source and must not be repeated. Camera scope is now two forward-facing devices; validate their calibration/frame pipeline instead of adding three more bridges.

Proceed to routing and coherent drivetrain integration rather than generic extra motion tests or simulator migration. No successful build/runtime or acceptance is claimed by this document.

## 10. Evidence anchors

- Project requirements: robotics-project-guide.md.
- Historical test and continuity evidence: original mobile_robot_repair_plan.md archived below.
- Current branch commit: bf57ba3bbac1db56005b89758f465e85cb26d2e9.
- Current nav2_params.yaml blob: a4723bf1a4c37d1425a85b27e483f404ab5744e5.
- Current maps.rviz blob: 75b61b888ff416657364c5b2ef7cf8b29dec47d5.
- Camera macro blob: 0da390f48e604b044ebac33be202cd96bc8cad4e.
- Robot core blob: f022764a26f79daa1db5fef2d4da4a7eaa8a01b3.
- Dormant bridge YAML blob: 53a6733d234487ee739d219a3029c4c864a5c75d.
- Rechecked navigation launch blob: ebb63176137b9b0c147574afa0a262fc23e182ae.

Installed-version parameter/plugin compatibility remains an explicit validation gate. No new ROS/Nav2/Gazebo version combination or hardware compatibility is asserted in this revision.

---

# Historical archive: original attachment, last updated 2026-11-22

The original attachment below is retained for measurements and continuity. All five-camera requirements, status labels and immediate-action instructions in this archive are historical and superseded by the active revision above. Historical test measurements were not rerun during this review.

# Mobile Robot Repository Repair Plan

## Objective

Repair the repository through small, independently testable patches rather than one repository-wide rewrite.

Implementation order:

Establish the local baseline → fix TF and odometry → reconcile the model → integrate sensors → repair launch orchestration → align Nav2 and command routing → validate and document.

Use simulation-first development and local verification. Do not modify GitHub until the local patch series has been reviewed and explicitly approved. This document distinguishes planned work from user-executed results reviewed in logs and summaries. Completion applies only to the explicitly recorded configuration and test conditions; the assistant has not executed the robot or repository changes.

## Progress Update

Updated: 2026-11-22, 22:53 +0530.

### Overall status

Isolated low-speed holonomic mecanum motion validation is complete for the tested flat-floor simulation configuration. Mapping-stack integration, full model/sensor reconciliation, navigation, failure handling, and physical deployment are not complete.

The active runtime is verified Fortress, not Harmonic. The immediate task is a read-only integration audit, not another motion test or a simulator migration. The latest audit failed because its TF callback required message metadata that the running Python API did not supply. A guarded optional-metadata fix was provided, but application and a successful rerun have not yet been confirmed.

### Repair-phase tracker

- [x] Step 1: Establish the authoritative local baseline. Completed for local repair planning on 2026-11-21; retain historical Git/environment anchors below.
- [ ] Step 2: Fix TF ownership and wheel-odometry delivery. IN PROGRESS. Earlier differential-drive wheel-to-EKF wiring and per-edge TF ownership were observed; the new mecanum center/frame contract has not been integrated into mapping/EKF.
- [ ] Step 3: Reconcile and validate the current robot model. PARTIAL. Isolated spawn, controller activation, translation, yaw, and combined motion passed. Shared model consolidation, full sensor geometry and physical properties remain open.
- [ ] Step 4: Replace the fragile sensor pipeline. NOT COMPLETE. Some typed relays and corrected scan/IMU delivery were observed previously; the intended complete five-camera pipeline is not validated.
- [ ] Step 5: Simplify launch ownership. NOT COMPLETE. Preserve the isolated validation entry point while auditing actual mapping/localization orchestration.
- [ ] Step 6: Align Nav2, SLAM, footprint, and command routing. NOT COMPLETE. No promotion of isolated mecanum corrections into mapping/navigation has been reviewed or validated.
- [ ] Step 7: Repair visualization, dependencies, and documentation. PARTIAL DOCUMENTATION ONLY. Capture/audit runbooks exist; current container dependency changes are not yet captured in a reproducible image baseline.
- [ ] Step 8: Validate the complete repair. PARTIAL. The isolated low-speed motion subset passed; this is not complete repaired-stack validation.

### Completed isolated checkpoints

- [x] Four independent wheel command/state interfaces and `mecanum_drive_controller` active in the isolated launch.
- [x] Forward, reverse, left, and right translation, including numerical ground-truth comparison and visual confirmation.
- [x] Positive/negative yaw directions, center stability, and settling.
- [x] Collision-width A/B experiment: 25 mm versus 1 mm collision cylinders, without changing controller radius or geometric sum.
- [x] Three CCW/CW pairs at +/-0.10 rad/s.
- [x] Three CCW/CW pairs at +/-0.05 rad/s and cross-rate review.
- [x] Four combined translation/rotation phases, including approximate inverse-path retracing and visual confirmation.
- [ ] Successful complete read-only integration audit after the optional TF-message-metadata callback correction.
- [ ] Minimal integration patch, reviewed rollback, and mapping/EKF/TF revalidation.

### Working configuration to preserve

| Item | Latest verified isolated configuration |
|---|---|
| Container | Ubuntu 22.04.5 LTS; system Python 3.10.12; ROS 2 Humble |
| Simulator | Ignition Gazebo 6.18.0 / Fortress, confirmed from loaded process libraries |
| Active simulator/bridge messages and transport | ignition-msgs8 8.7.1; ignition-transport11 11.4.2 |
| World / model | `house_world` / `mobile_robot`; entity ID discovered per run, not hard-coded |
| Launch | `mobile_robot/launch/mecanum_bringup_test.launch.py` |
| Explicit launch argument | `wheel_collision_width:=0.001`; default remains the original 0.025 m unless separately reviewed |
| Test controller YAML | `mobile_robot/config/mecanum_bringup_test.yaml` |
| Controller | `/mecanum_controller`, `mecanum_drive_controller/MecanumDriveController` |
| New isolated hardware plugin | `mecanum_four_wheel_hardware/FourWheelSystem` |
| Control arrangement | Four individual wheel velocity commands and position/velocity states; do not confuse with the legacy two-virtual-side interface |
| Command topic / message | `/mecanum_controller/reference_unstamped`, `geometry_msgs/msg/Twist` |
| Wheel odometry | `/mecanum_controller/odometry`; not claimed to be wired into the mapping EKF |
| Controller frames | `odom` and `mecanum_test_center` |
| Controller frame offset | x=0, y=0, theta=0 |
| Wheel radius / geometric sum | 0.024 m / 0.275 m |
| Reference timeout | 0.5 s configured; command-loss behavior has not yet been independently validated |
| Controller TF publication | `enable_odom_tf=false` |
| Pose/twist covariance diagonals | `[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]`, provisional and not calibrated |
| Ground-truth stream | Native Fortress `/world/house_world/pose/info` |
| Model-to-test-center translation | `(0.05, 0, 0.024)` m, rotated using the full model quaternion |

Preserve the test-only friction corrections: reference link `base_footprint`, front-left/rear-right direction `(0.70710678, -0.70710678, 0)`, front-right/rear-left direction `(0.70710678, 0.70710678, 0)`, and the working namespaced `expressed_in` attribute. Runtime URDF showed `mu=1.0` and `mu2=0.02`. Do not change these or controller geometry merely to fit the small remaining errors.

### Scope and current restrictions

- Keep the isolated low-speed validation baseline working. No additional motion tests are required before the integration audit is reviewed.
- Do not promote the collision/friction/frame edits into the shared Xacro or mapping launch yet.
- The 1 mm collision proxy is a provisional simulation contact approximation. Visual geometry, mass, and inertia were preserved. It is not the physical wheel width or navigation footprint.
- Full holonomic operation is established for the isolated simulation. The shared mapping/navigation stack and physical hardware interface are not thereby converted or validated.
- Earlier source hashes and Git commits are historical anchors. They do not describe all subsequent uncommitted files or prove a new image/workspace can reproduce the current result.
- Native `ign` tooling is the working backend. Do not substitute Harmonic clients, migrate Gazebo, or clean up dependencies during this audit.
- No GitHub write, commit, push, mapping promotion, or physical-hardware validation is claimed.

## 1. Establish the Authoritative Baseline [COMPLETED]

### Objective

Prevent fixes from overwriting newer local URDF/Xacro files with older GitHub versions.

### Work

1. Capture the current local working tree, including untracked files.
2. Record:
   - Git branch and commit.
   - Uncommitted changes.
   - Active ROS package-share location.
   - Installed ROS, Gazebo, bridge, controller, and custom-plugin versions.
   - Container image and GPU/runtime configuration.
3. Capture the `mecanum_ign_hardware` source separately.
4. Expand the robot description in simulation and hardware configurations.
5. Record which previously supplied patches are actually present.

The uploaded configuration shows that some earlier changes were applied differently from the supplied patchers. Do not assume patch application from conversation history alone.

### Deliverables

- Baseline source snapshot.
- Version and environment manifest.
- Expanded URDFs and expansion logs.
- Reconciliation list: local-only, GitHub-only, and conflicting changes.

### Exit Gate

The exact source being built is known, the latest geometry is preserved, and every proposed patch has a recoverable baseline.

### Completion Record: 2026-11-21

Historical baseline record. Values and unverified items in this subsection describe the capture on that date, not the latest isolated runtime. Later changes and test outcomes are recorded in Progress Update and the 2026-11-22 validation record.

Status: COMPLETE for the local repair baseline, with the evidence limitations below.

- Source snapshot: user confirmed completion for both packages, including local and untracked source. Archive contents/checksum and restore behavior were not independently tested by the assistant.
- Source versus installed: six key files match and resolve to their local source paths. The config directory contents match. Installed-only entries are recorded below.
- Xacro capture: simulation.urdf and hardware.urdf received. Empty expansion logs were user-reported. Model structure, simulator spawn, and backend loading remain unvalidated.
- Environment capture: container manifest, host manifest, custom-plugin source checksums, and installed-library checksum reviewed.
- Git reconciliation: branch, commit, working-tree changes, and cached upstream relationship reviewed for both packages. No live GitHub freshness or merge-conflict check was performed.

#### Authoritative Source and Git Anchors

The current local working trees, including uncommitted and untracked source files, are the authoritative repair inputs. HEAD alone does not contain the full baseline.

| Package | Branch | Local HEAD | Cached upstream relationship |
|---|---|---|---|
| mobile_robot | feature/localization | 810433f343180b52be0ba757518e18e40a76cf7c | 0 local-only, 2 upstream-only commits; cached upstream 673d2bd337e214d96d9952f4f0dabc6e1185d76c |
| mecanum_ign_hardware | main | 3d450e614e7d342a9f914bf06a6b05bea90b9ef4 | HEAD matches cached origin/main; src/mecanum_system.cpp has local modifications |

mobile_robot has eight tracked file changes, including deletion of description/four_zed2_camera_mount.xacro, plus untracked files including oak1w_five_cameras.xacro and robot_core_box.xacro. The committed difference between cached upstream and local HEAD is launch/robot.launch.py; its contents require review before integration. Do not pull, reset, clean, or overwrite the local model as part of baseline closeout.

#### Captured Environment

- Host: Fedora 43; Docker client/server 29.9.3; NVIDIA Container Toolkit 1.20.2.
- Container: ROS2-humble, Ubuntu 22.04.5 LTS, x86_64, ROS 2 Humble; host networking, non-privileged, runtime field runc, all-GPU device request.
- Image ID: sha256:e4ec16dd62ca19383caaaaa0102049d664b7a0439bb42209e3a7515b42eb1ba3. Image reference is recorded in the host manifest.
- Host workspace: /home/nisalperera/Projects/personal/ros2-docker/projects.
- Container workspace: /home/nisalperera/projects.
- Active mobile_robot share: /home/nisalperera/projects/install/mobile_robot/share/mobile_robot.
- Gazebo library: libignition-gazebo6 6.18.0; source launch explicitly selects version 6. This is the current repair baseline, not the Harmonic target in the project guide. Migration remains separate.
- ros_gz_bridge and ros_gz_sim: 0.244.27; gz_ros2_control: 0.7.21.
- ros2_control: 2.54.4; diff_drive_controller: 2.54.1; Nav2: 1.1.20; robot_localization: 3.5.5.
- nvcc: 12.8.93; NVIDIA driver reported by container nvidia-smi: 580.178.04. GPU visibility observed; actual Gazebo GPU rendering unverified.
- Active workspace overlay includes local RTAB-Map packages.
- Installed custom-plugin library resolves to the workspace build directory. Checksums do not establish that it was compiled from the current source.

#### Recorded Discrepancies and Existing Changes

- Installed-only: description/four_zed2_camera_mount.xacro, launch/gazebo_riviz_zed.launch.py, and launch/__pycache__/.
- Source-only directory: description/urdf/.
- Installed-only files are not assumed preserved by the source-package archive. Leave installed/build trees untouched until these entries are inspected or separately preserved.
- Local controllers.yaml already adds enable_odom_tf: false; effective running value remains unverified.
- Local ekf.yaml already sets imu0: /imu_fixed; effective launch configuration remains unverified.
- gz.launch.py already uses inline typed rclpy relays, while older comments remain. Its shown pipeline still covers only the two older cameras and retains the point-cloud converter.
- Expanded descriptions contain five camera definitions, lidar, and IMU. This does not establish live sensor delivery.
- Hardware expansion retains simulation sensor definitions and Gazebo integration entries; review later without changing the baseline now.

#### Evidence Artifacts Reviewed

- logs.txt
- source-vs-installed-n9ZLDZKv.txt
- simulation.urdf and hardware.urdf
- container-manifest-koy8R7E3.txt
- robot-host-manifest-Sfkhxwe7.txt
- git-baseline-4M6E1cJr.txt

Exit gate accepted for local repair planning, subject to user-confirmed backup and the recorded uncertainties. The exact current source inputs are identified and latest local geometry is retained. Build reproducibility and runtime correctness are not yet established.

## 2. Fix TF Ownership and Wheel-Odometry Delivery

### Objective

Eliminate competing pose transforms and ensure the EKF receives actual wheel feedback.

### Files

- `config/controllers.yaml`
- `description/xacro/ros2_control.xacro`
- `launch/mapping.launch.py`
- `launch/localization.launch.py`
- `config/ekf.yaml`

### Work

1. Verify the existing local `enable_odom_tf: false` setting is loaded by the controller; change it only if missing or ineffective.
2. Keep EKF `publish_tf: true`.
3. Verify the actual wheel-odometry publisher and topic.
4. Establish a common contract:

```text
Wheel controller → raw wheel odometry → EKF → /odom
IMU              → corrected IMU     → EKF
```

Preferred canonical topics:

```text
/odom/raw     Wheel odometry only
/imu_fixed    Simulation IMU after frame correction
/odom         Fused odometry only
```

Implement raw-odometry remapping where the controller is created, not on its spawner. If the installed integration does not support that remapping cleanly, explicitly configure the simulation EKF to use the controller's actual odometry topic and document the exception.

5. Remove contradictory mapping/localization remappings.
6. Explicitly enable simulation time on relevant simulation nodes.
7. Keep real-hardware IMU overrides separate from simulation settings.

### Status as of 2026-11-22

Historical differential-drive stack evidence goes beyond the original source-only baseline:

- `step2-after-patch-hxbuthP1.txt` showed controller `enable_odom_tf=false`, state publisher simulation time enabled, and wheel odometry from `/diff_drive_controller/odom` reaching the EKF subscription.
- `step2-mapping-check-HuhgOPs6.txt` showed effective EKF inputs `/diff_drive_controller/odom` and `/imu_fixed`, fused `/odom`, and mapping nodes running in that earlier configuration.
- `tf-ownership.txt` attributed `odom -> base_footprint` to `/ekf_filter_node`, `map -> odom` to SLAM Toolbox, and body/wheel transforms to `robot_state_publisher` during its observation interval. This is stronger than merely listing `/tf` publishers. Earlier `step2-tf-owners-kkdrttb5.txt` lacked resolved GIDs and must not be used as equivalent per-edge ownership proof.
- `step2-covariance-review-0l5DtdWN.txt` exposed all-zero loaded differential-controller odometry covariance and large fused uncertainty despite source intentions. Later isolated mecanum covariance values are provisional; they do not prove this earlier fused-estimator issue is repaired.

Do not mark the full Step 2 exit gate closed for the new mecanum configuration. The isolated controller now publishes `/mecanum_controller/odometry` at `mecanum_test_center`, with TF disabled and no demonstrated mapping-EKF integration. Historical differential-drive wiring is not proof of current mecanum wiring.

The `/odom/raw` naming above remains a preferred contract, not a claimed deployed alias. Record the actual publisher and effective EKF input before deciding remapping. Never feed fused `/odom` back as a wheel-odometry input.

### Exit Gate

- One owner for `odom → base_footprint`.
- Wheel messages reach the configured EKF input.
- Fused output never feeds back into a raw input.
- No repeated pose switching during a stationary test.
- Controlled movement produces continuous, correctly signed odometry.

## 3. Reconcile and Validate the Current Robot Model

### Objective

Make the current local Xacros the single source of truth for geometry and sensor placement.

### Files

- `description/robot.urdf.xacro`
- `description/xacro/robot_core.xacro`
- Wheel Xacros.
- Five-camera macro and camera includes.
- `lidar.xacro`.
- `imu.xacro`.
- Inertial macros.

### Work

1. Confirm the 350 × 300 mm chassis plate.
2. Separate measured properties from placeholders:
   - Plate thickness and mass.
   - Motor/bracket masses.
   - Camera mount coordinates and heights.
   - Lidar and IMU mounting transforms.
3. Validate:
   - Unique link and joint names.
   - One connected kinematic tree.
   - Four continuous wheel joints.
   - Correct axle directions and wheel signs.
   - Valid inertias and collision geometry.
   - Spawn clearance above the floor.
4. Confirm exactly five cameras, one intended lidar, and one intended IMU in simulation.
5. Check the five camera headings and optical transforms.
6. Confirm the lidar has one horizontal scan plane.
7. Compare wheel geometry with controller radius and separation.

Updated scope: the isolated simulation has progressed to full holonomic mecanum operation, including lateral and combined motion, and that motion subset has passed the recorded checks. Preserve the legacy/shared mapping setup until a minimal integration change is reviewed. Do not infer that the legacy two-side plugin, physical motor interface, or Nav2 parameters are already holonomic.

Model reconciliation remains partial. The test-center frame, friction directions/reference, and optional collision-width override currently belong to the isolated validation arrangement. Inspect the actual source injection points before consolidating them. The shared wheel definition, physical footprint, complete sensor geometry, hardware expansion and measured inertial properties still require their own review.

### Exit Gate

Both Xacro configurations expand successfully, the simulation model spawns correctly, and all geometry-dependent configuration can be derived from the verified local model.

## 4. Replace the Fragile Sensor Pipeline

### Objective

Support all new sensors without startup races, inconsistent frames, or obsolete lidar processing.

### Files

- `launch/gz.launch.py`
- A dedicated sensor bridge/relay launch.
- Typed sensor relay implementation.
- `config/ros_gz_bridge.yaml`
- `launch/pointcloud_to_laserscan.launch.py`

### Work

#### Lidar

- Bridge the single-plane LaserScan directly.
- Correct its frame once, if required.
- Standardize the corrected output as `/scan_fixed`.
- Remove the point-cloud converter from the default 2D path.
- Retain a clearly separated legacy 3D mode only if still needed.

#### Cameras

For all five devices:

- Bridge images and `CameraInfo`.
- Correct both headers consistently.
- Preserve calibration, timestamps, encoding, and image content.
- Use actual expanded sensor topics and optical-frame names.
- Verify Gazebo-side calibration topic names.

#### IMU

- Replace discovery-racy transformation startup with a typed subscriber that waits for input.
- Preserve current measurement/covariance behavior initially.
- Review acceleration and gravity treatment only after odometry wiring works.

#### Shared Behavior

- Warn on missing or stale inputs.
- Never fabricate samples.
- Configure compatible QoS.
- Keep one authoritative bridge definition instead of divergent YAML and inline argument lists.

Earlier source captures showed typed inline relays replacing parts of the older transformation pipeline, while historical comments and a point-cloud conversion path remained. The inspected bridge/relay arrangement still covered only the two older camera streams. Treat the latest source/install audit as authoritative for what remains; do not blindly reapply an obsolete relay patch.

Previously observed corrected scan/IMU rates and valid frame delivery do not establish the complete five-camera image/CameraInfo pipeline or final sensor architecture.

### Exit Gate

- Five image streams and five corresponding calibration streams.
- Scan and IMU messages arrive with valid frames.
- No frame-fixer traceback loops.
- Missing sensors produce warnings rather than repeated crashes.
- No relevant QoS incompatibility warnings.

## 5. Simplify Launch Ownership

### Objective

Ensure each launch starts exactly one coherent stack.

### Files

- `launch/robot.launch.py`
- `launch/mapping.launch.py`
- `launch/localization.launch.py`
- `launch/gz.launch.py`
- `launch/online.async.launch.py`
- `launch/rviz.launch.py`

### Target Ownership

| Component | Owner |
|---|---|
| Robot description/state publisher | Canonical parent bring-up |
| Gazebo, spawning, simulation controllers | Simulation launch |
| Sensor bridges and relays | Sensor launch |
| EKF | Shared state-estimation component |
| SLAM Toolbox | Mapping mode only |
| AMCL and Nav2 | Localization/navigation mode only |
| RViz | Optional visualization launch |

### Work

- Replace or deprecate legacy overlapping orchestration.
- Make mapping and localization mutually exclusive.
- Prevent duplicate Gazebo, navigation, joystick, and state-publisher instances.
- Replace guessed delays with service readiness or lifecycle sequencing where supported.
- Preserve optional features without letting them start duplicate stacks.
- Reject unsupported combinations clearly.

### Exit Gate

Each supported entrypoint produces the expected node graph, with exactly one `map → odom` authority: SLAM or AMCL, never both.

## 6. Align Nav2, SLAM, Footprint, and Command Routing

### Current status

Not integrated or validated for the new mecanum drivetrain. Inspect inherited differential-drive settings, lateral limits and controller motion models before selecting any change. The successful isolated Twist tests establish low-speed holonomic motion, not an autonomous command path, correct footprint or EKF fusion.

The next integration patch must explicitly resolve the 0.05 m offset between `base_link` and `mecanum_test_center`, the current static body-tree parent of the test center, and pose/twist reference-point semantics. Do not rename an odometry child frame or enable `odom -> mecanum_test_center` without resolving TF parentage and point transforms.

### Files

- `config/nav2_params.yaml`
- `config/mapper_params_online_async.yaml`
- `config/twist_mux.yaml`
- `launch/navigation.launch.py`

### Work

1. Standardize all intended scan consumers on `/scan_fixed`.
2. Standardize fused-odometry consumers on `/odom`.
3. Derive a footprint polygon from the verified local model:
   - Include wheel and collision-relevant protrusions.
   - Express it in the configured robot base frame.
   - Account for chassis offsets.
   - Add a documented margin.
4. Remove contradictory or duplicate YAML keys.
5. Keep existing planner tuning unless testing identifies a specific defect.
6. Choose one command path:

```text
Nav2 controller/recoveries
    → optional velocity smoother
    → navigation mux input
    → drivetrain controller
```

Ensure the smoother is not bypassed, recovery commands reach the same arbitration path, and composition/non-composition configurations behave consistently.

### Exit Gate

- Correct scan and odometry subscriptions.
- Footprint matches the robot in RViz.
- Autonomous and recovery commands reach the drivetrain.
- Joystick priority and command timeout behavior remain correct.
- Navigation no longer depends on accidental topic defaults.

## 7. Repair Visualization, Dependencies, and Documentation

### Work

Create three explicit RViz profiles:

| Profile | Fixed Frame | Purpose |
|---|---|---|
| Model inspection | `chassis` or `base_link` | Geometry and sensor alignment |
| Odometry debugging | `odom` | Local motion continuity |
| Navigation | `map` | Localization, planning, and goals |

Add uniquely named displays for all five cameras and appropriate description/map subscription durability.

Then:

- Declare direct runtime dependencies.
- Install any new relay executable and tests correctly.
- Update README installation and launch instructions.
- Document the final topic and TF contracts.
- Replace obsolete stereo/3D-lidar descriptions.
- Correct misleading historical comments.
- Mark provisional physical parameters explicitly.

### Exit Gate

A clean workspace can install dependencies, build, and launch using only the documented procedure.

## 8. Validate the Complete Repair

### Status boundary

The isolated motion subset is complete for the documented short low-speed flat-floor tests and visual observations. The complete repair is not validated. In particular, no new mapping-stack promotion, holonomic Nav2 run, full five-camera pipeline, long-distance drift test, command-loss test, controller/network restart, GPU performance assessment or physical deployment is claimed.

Use a validation ladder rather than immediately testing autonomous navigation.

| Stage | Test |
|---|---|
| Static | Python syntax, duplicate-key YAML checks, Xacro expansion, model structure |
| Spawn | Model creation, controller activation, sensor initialization |
| Stationary | Pose continuity, sensor delivery, no conflicting TF |
| Manual motion | Straight, reverse, turn, and arc tests |
| Mapping | Consistent walls and scan alignment |
| Localization | Initial-pose acquisition and stable stationary behavior |
| Navigation | Goals, obstacles, recoveries, and command arbitration |
| Failure handling | Sensor dropout, stale commands, pause/resume, restart, clock reset |
| Performance | Five-camera rendering, GPU selection, simulation rate, CPU/GPU load |

Record actual outcomes. Define numerical drift, latency, and accuracy thresholds after measuring the repaired baseline rather than inventing project requirements.

## Validation and Continuity Record: 2026-11-22

### Evidence interpretation

The measurements below come from the user's posted capture summaries and visual observations. The assistant reviewed those results but did not execute the simulator, build the workspace, apply source patches, or inspect the unuploaded raw CSV contents. Historical attached reports are identified separately where used. Proposed screening limits are not retroactively declared formal project requirements.

### 1. Legacy and isolated control arrangements are distinct

The earlier mapping/differential setup uses `mecanum_ign_hardware/MecanumSystem` with two virtual side commands (`left_wheels` and `right_wheels`) fanned out to four wheel joints. Earlier interface/source reports demonstrate that arrangement.

The new isolated setup uses `mecanum_four_wheel_hardware/FourWheelSystem` and four wheel command/state interfaces with `mecanum_drive_controller`. Its launch logs showed hardware initialization/configuration/activation and both `mecanum_controller` and `joint_state_broadcaster` becoming active.

Do not overwrite or merge these plugins merely because both mention mecanum wheels. The old two-side interface cannot provide arbitrary independent lateral wheel commands. Integration requires deliberately selecting the four-wheel command path, updating odometry consumers, and retaining a rollback to the working isolated validation setup.

### 2. Lateral mismatch and test-only corrections

Initial isolated lateral testing revealed a discrepancy between Gazebo chassis yaw and wheel-derived odometry. Working test-only corrections changed friction directions/reference/frame handling. The corrected reference is `base_footprint`, with front-left/rear-right directions `(0.70710678, -0.70710678, 0)` and front-right/rear-left `(0.70710678, 0.70710678, 0)`.

The reconstructed geometric test center accounts for the model-origin offset instead of treating model-origin translation during yaw as chassis-center drift. The active controller reference frame is `mecanum_test_center`; its configured base-frame offset remains zero.

These fixes have not been reviewed for promotion into the default mapping launch or shared Xacro files.

### 3. Collision-width A/B experiment

Controller radius and geometric sum remained 0.024 m and 0.275 m. Only the diagnostic wheel collision-cylinder width was changed; visual geometry, mass and inertia were preserved.

| Collision width | Gazebo yaw magnitude | Wheel-odom yaw magnitude | Absolute disagreement | Excess relative to wheel odometry |
|---|---:|---:|---:|---:|
| 0.025 m baseline | 14.404402 deg | 13.750987 deg | 0.653415 deg | about 4.7518% |
| 0.001 m diagnostic | 13.775882 deg | 13.750987 deg | 0.024895 deg | about 0.1810% |

Both signs showed equal magnitudes in their respective run. The disagreement decreased by about 96.19%. Results strongly support collision width as a major contributor in this simulation arrangement and closely match a simple inward-edge lever-arm hypothesis. Contact locations were not directly measured, so the exact edge-contact mechanism is not claimed proven.

Decision: retain the 1 mm proxy provisionally in the isolated flat-floor simulation. Do not compensate by changing wheel radius or the geometric sum. Do not infer physical wheel dimensions, footprint, obstacle/terrain behavior, or hardware accuracy from this contact approximation.

Local capture artifacts reported:

- `mecanum-bidirectional-yaw-djl3fhrc.csv`: original-width bidirectional yaw.
- `mecanum-bidirectional-yaw-ndlqu9we.csv`: narrow-width bidirectional yaw.

### 4. Four-direction translation regression at 0.03 m/s

All four phases contained 20 nonzero messages followed by 10 zeros. Numerical behavior and all visual direction/drift/stopping observations were confirmed.

| Direction | Gazebo primary displacement | Wheel odometry | Gazebo cross-axis displacement | Gazebo yaw change |
|---|---:|---:|---:|---:|
| Forward | +60.600000 mm | +60.600000 mm | about zero | about zero |
| Reverse | -60.600000 mm | -60.600000 mm | about zero | about zero |
| Left | +60.597453 mm | +60.600000 mm | -0.001668 mm forward | +0.000219 deg |
| Right | -60.597453 mm | -60.600000 mm | -0.001668 mm forward | -0.000219 deg |

Lateral magnitude difference was about -0.0042%. Every final one-second span was zero at printed precision. Maximum sample gaps were 0.018 s for Gazebo and 0.010 s for wheel odometry.

The 60.6 mm result is consistent with slightly more than two seconds of effective motion; it is not evidence of a radius error when both sources agree. Later checkpoint code used an absolute simulation-time publishing schedule and separately reported a receipt-time command integral.

Reported local artifact prefix: `mecanum-four-directions-m57r_qoc`, including pose CSV, command CSV and text summary.

### 5. Checkpoint A: yaw repeatability at +/-0.10 rad/s

Three CCW/CW pairs completed with exactly 20 intended nonzero and 10 zero commands per phase.

- Gazebo heading magnitude: 11.479898 deg for every reported pulse.
- Wheel-odometry magnitude: 11.459156 deg.
- Mean gains: CCW 1.00181011; CW 1.00181010.
- Magnitude excess: about +0.18101%.
- Gain CV: 0.00000% at printed precision, not a claim of zero uncertainty.
- Maximum reconstructed center excursion: 0.000510 mm.
- Largest reported trajectory yaw disagreement: 0.021114 deg.
- Zero final one-second translation/yaw spans at printed precision.
- User confirmed correct directions, no noticeable center drift, stops between all pulses, and no contact/tipping/unexpected behavior.

Summary screen: `WITHIN_PROPOSED_LIMITS`; engineering review accepted for these low-speed pulses.

Reported artifact prefix: `mecanum-yaw_repeat-20261122-212053-0be7e4`.

### 6. Checkpoint B: yaw at +/-0.05 rad/s

Three CCW/CW pairs completed with the same correct message counts and signs.

- Gazebo heading magnitude: 5.739894 deg.
- Wheel-odometry magnitude: 5.729578 deg.
- Mean gains: CCW 1.00180053; CW 1.00180052.
- Magnitude excess: about +0.18005%.
- Gain CV: 0.00000% at printed precision.
- Maximum center excursion: 0.000129 mm.
- Largest reported trajectory yaw disagreement: 0.010316 deg.
- All final one-second spans zero at printed precision.
- User confirmed correct direction, center stability and stopping, with no unexpected behavior.

Mean yaw-gain difference between the two tested rates was about 0.000958 percentage points. No meaningful rate dependence is apparent over these two rates and short pulse durations. This is not a general velocity/load/terrain characterization.

Summary screen: `WITHIN_PROPOSED_LIMITS`; cross-rate review accepted for the tested low-speed regime.

Reported artifact prefix: `mecanum-yaw_second-20261122-212303-a2f966`.

### 7. Checkpoint C: combined translation and rotation

Commands used 0.02 m/s on one translation axis and +/-0.05 rad/s yaw. Four phases completed with correct counts and stopping.

| Phase | Endpoint XY disagreement | Maximum sampled trajectory XY disagreement | Endpoint yaw disagreement magnitude |
|---|---:|---:|---:|
| Forward + CCW | about 0.008568 mm | 0.044977 mm | 0.010211 deg |
| Reverse + CW | about 0.008492 mm | 0.054107 mm | 0.010211 deg |
| Left + CCW | about 0.052102 mm | 0.090225 mm | 0.000218 deg |
| Right + CW | about 0.052102 mm | 0.086500 mm | 0.000218 deg |

Forward/reverse yaw excess was about +0.17821%; lateral combined-motion excess was about -0.00381%. This motion-dependent residual is another reason not to fit a universal geometry correction from pure yaw alone.

Every final one-second translation/yaw span was zero at printed precision. User confirmed all arcs looked correct, approximately retraced the inverse paths, stopped between phases and after final zero, and showed no unexpected sliding, contact or tipping.

Summary screen: `WITHIN_PROPOSED_LIMITS`; accepted for these short combined-motion segments.

Reported artifact prefix: `mecanum-combined-20261122-212457-1ebb1a`.

### 8. Measurement and acceptance limitations

- Ground-truth center and wheel odometry are aligned independently to their phase-start poses/headings. These comparisons do not prove global odometry-origin correctness, whole-run closure, or long-duration accumulated drift.
- Trajectory disagreements were sampled on a 50 Hz interpolation grid, not at every physics step.
- Commands are unstamped. Receipt-time SE(2) predictions use the observer's latest simulation clock and are diagnostic, not controller execution truth. Small differences between those integrals and wheel odometry are not, alone, controller scale failures.
- Reported zero drift/variation means zero at the printed precision. Small residuals in this numerical simulator do not predict physical accuracy.
- Proposed screens included yaw disagreement <=1%, pure-yaw center excursion <=1 mm, within-sign gain CV <=1%, combined endpoint XY disagreement <=1 mm, combined maximum trajectory XY disagreement <=2 mm, trajectory yaw disagreement <=0.1 deg, and final-one-second translational/yaw spans <=0.1 mm / 0.1 deg. These are low-speed simulation review targets, not newly invented overall project requirements.
- Higher speeds, longer trajectories, load variation, uneven surfaces, collision handling, timeout/command loss, restart, clock reset, distributed simulation and hardware behavior are not validated by these tests.

### 9. Tooling and environment corrections

The first new capture utility incorrectly assumed Harmonic and used `gz.transport13`, `gz.msgs10`, and the `gz` CLI. Initial errors included missing `gz` Python imports and missing CLI entry point. An OSRF Jammy repository was added and Python bindings/CLI became available. The package preview showed Transport 13.7.0 and Messages 10.4.0 candidates; a post-install complete environment manifest was not captured, so those candidate versions must not be treated as a complete verified installed-package snapshot.

A loaded-library inspection then proved the simulator and bridge were still Fortress. Harmonic-client scene inspection returned no usable robot model and transport warnings appeared. The capture was corrected to use native `ign` service/topic CLI with `ignition.msgs` types and a protobuf-text reader. That backend successfully captured the four-direction and subsequent checkpoint results.

Do not diagnose the exact transport-warning cause beyond the available evidence, uninstall packages, or change repositories during the current integration audit. Record dependency provenance and decide Docker/image cleanup or pinning as a separate reviewed change. The earlier image ID and package manifest predate this tooling work.

Current user-local utility paths include:

- `mobile_robot/tools/mecanum_pending_checkpoints.py`: successfully used for A/B/C; no need to rerun those motion tests solely because of the audit issue.
- `mobile_robot/tools/mecanum_integration_audit.py`: next read-only evidence collector.

### 10. Read-only integration audit: blocked pending corrected rerun

The audit is intended to print:

1. Part A: actual runtime nodes, command topics/endpoints, TF edges and ownership where available, effective controller/EKF parameters, runtime URDF, controller interfaces and loaded libraries.
2. Part B: actual source/configuration inventory, mapping-launch evidence, line-numbered excerpts, source/install comparison.
3. Part C: unresolved integration decisions and review gate.

Latest received result:

```text
AUDIT ERROR: TypeError: observe.<locals>.tf_callback.<locals>.callback() missing 1 required positional argument: 'info'
```

Cause established by that traceback: the supplied audit required a second callback argument, but this runtime invoked the callback with the message only. This is an observer compatibility issue, not proof of robot or TF failure.

A guarded patch was supplied to change the TF callback to `callback(msg, info=None)` and mark metadata as `MESSAGE_INFO_UNAVAILABLE` when absent. The patch was syntax/fixture tested by the assistant. No user confirmation that it was applied, no successful corrected audit run, and no complete audit Parts A/B/C have been received as of this update.

Metadata limitation: if message info is unavailable, topic-level publisher lists and received frame edges remain observable, but individual edge-to-publisher attribution is unresolved. A single placeholder GID is not proof of one actual owner. Do not confuse the older successful TF ownership evidence with a newly verified mecanum integration audit.

Audit coverage limitation: the current supplied source scanner targets `mobile_robot` and the new `mecanum_four_wheel_hardware`; the legacy `mecanum_ign_hardware` may require an explicit additional source read before changing its launch path. Any truncated source or unexpanded include remains unreviewed evidence, not permission to infer contents.

### 11. Required integration decisions after evidence capture

- Verify the exact intended mapping entry point; historical records include `launch/mapping.launch.py`, but its latest contents, launch chain and effective arguments still require audit.
- Choose the navigation physical reference point and transform both pose and twist correctly from the mecanum center where necessary.
- Preserve a valid single-parent body TF tree. The currently described chain is `base_footprint -> base_link -> chassis -> mecanum_test_center`; the center is 0.05 m ahead of `base_link`. Do not simply add a competing `odom -> mecanum_test_center` transform.
- Select one authority for odom-to-navigation-base TF. Keep mapping/localization map-to-odom authority separate.
- Decide the effective wheel-odometry topic/EKF input contract without a fused-feedback loop. Review EKF lateral constraints and covariance for holonomic motion rather than retaining incompatible differential assumptions.
- Confirm the command routing, stamped/unstamped type, mux/smoother behavior, timeout owner, and Nav2 lateral motion configuration.
- Identify the minimal place for test-only collision/friction/frame corrections. Keep physical and navigation footprint dimensions independent from the thin contact proxy.
- Inspect relevant source/install divergence and complete missing source ranges before producing the patch.
- Preserve the isolated entry point and test evidence as the rollback/regression baseline.

No integration design, TF reparenting, EKF tuning, mapping launch promotion or navigation acceptance is approved by this status update.

### 12. Next-result handoff

Provide the read-only audit's complete Part A, Part B and Part C text, plus whether the robot remained stationary and the intended mapping entry point if known. Continue using pasteable text summaries and complete Markdown runbooks because file upload is not required for this workflow.

Do not request a new generic motion test merely to reconfirm completed low-speed checks. Use targeted additional tests only after a concrete integration or evidence gap is identified and reviewed.

## Separate Follow-On Tracks

These issues remain in scope but should not be mixed into the immediate repair:

- Gazebo Harmonic migration: verify and pin a supported version combination, then rerun the same validation suite.
- GPU rendering: verify actual renderer/device selection and performance, not just environment variables.
- Hardware readiness: verify C1 driver settings, serial-device configuration, IMU-filter parameter scope, and hardware interfaces.
- Jetson deployment: begin only after laptop simulation passes.
- Holonomic integration and deployment: isolated holonomic simulation is implemented and its low-speed motion subset is validated. Shared mapping/Nav2 integration and physical four-motor control remain pending; do not treat them as an untouched motion-model decision or as already deployed.

## Patch Delivery and Rollback

Deliver one patch per phase, each containing:

1. Exact affected files.
2. Complete files or an exact diff.
3. Dependencies and execution location: laptop host, laptop container, Jetson host, or Jetson container.
4. Build and validation commands.
5. Observed test results and remaining limitations.
6. Backup or revert instructions.

## Current Milestone and Immediate Next Action

Completed milestone: isolated four-wheel holonomic simulation with reviewed translation, repeated bidirectional yaw at two rates, combined motion, and visual stopping checks.

Current milestone: obtain a successful read-only runtime/source integration audit, then design the smallest mapping/EKF/TF integration patch without losing the isolated baseline.

Immediate next action, in the laptop container:

1. Confirm or apply the supplied guarded optional-`info` TF callback patch to `mobile_robot/tools/mecanum_integration_audit.py`.
2. Keep the existing isolated simulation stationary and use the already sourced Humble/workspace environment.
3. Run:

```bash
# Execution location: laptop container, audit terminal
# Working directory: /home/nisalperera/projects
cd /home/nisalperera/projects
/usr/bin/python3 -u mobile_robot/tools/mecanum_integration_audit.py --seconds 20
```

4. Paste report Parts A (runtime), B (sources) and C (review gate), plus whether the robot remained stationary. No upload is required.
5. Review unresolved ownership, actual mapping entry point, source/install differences and any truncated source ranges before editing anything.

Do not start another motion test, mapping launch, simulator migration, package cleanup or integration patch to bypass a missing audit result.
