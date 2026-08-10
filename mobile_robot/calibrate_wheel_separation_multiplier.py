#!/usr/bin/env python3
"""
calibrate_wheel_separation_multiplier.py


Automated wheel_separation_multiplier calibration with two test modes:


  --mode spin  (default): pure in-place rotation.
      Useful sanity check, but CANNOT detect the virtual-axle bias
      described in this repo's own commit history (8fea9ceca), because
      a pure spin has zero linear velocity and doesn't exercise the
      front/rear wheel averaging error that only appears when the robot
      turns WHILE moving forward.


  --mode arc: combined linear + angular motion (an actual curve).
      This is the test that can actually expose the bias. It compares
      BOTH final position (x, y) and final yaw between ground truth
      (Ignition physics, via `ign topic -e`) and wheel odometry
      (diff_drive_controller, via `ros2 topic echo`), then derives a
      corrected wheel_separation_multiplier from the yaw ratio, same as
      the spin test, but on a trajectory that can actually reveal the bug.


Run this ON THE LAPTOP, in the same shell/environment where the sim
(mapping.launch.py or equivalent) is already running.


IMPORTANT (yaw measurement limitation): the multiplier is derived from
BEFORE/AFTER endpoint yaw only (via quaternions), not from an integrated
trajectory. normalize_angle() wraps any delta to (-pi, pi], so a true
rotation of pi (180 deg) or more is ALIASED -- e.g. an actual +200 deg
turn is indistinguishable from -160 deg once wrapped. This is valid for
--mode spin/arc as long as the total commanded rotation
(angular_z * duration) stays below pi; run_automated() rejects requests
that would exceed that. It is ALSO true for --manual: only pass endpoint
snapshots that you know correspond to a rotation under 180 deg, unless
you have collected and integrated intermediate yaw samples yourself
(this script does not do that for you).


Usage:
    # Pure spin test (original behavior)
    python3 calibrate_wheel_separation_multiplier.py --mode spin


    # Arc test (forward + turning simultaneously) — recommended
    python3 calibrate_wheel_separation_multiplier.py --mode arc \
        --linear-x 0.15 --angular-z 0.3 --duration 10


    python3 calibrate_wheel_separation_multiplier.py --mode arc --current-multiplier 1.0


    # Skip live commands, compute directly from numbers you already have.
    # For --mode spin: pass 16 values but x/y won't matter for the yaw calc.
    # For --mode arc: pass 16 values (x, y, quat_z, quat_w for each snapshot).
    # NOTE: valid only if the actual rotation between snapshots was < 180 deg
    # (see IMPORTANT note above) -- this script cannot detect a violation
    # from manually-supplied numbers after the fact.
    python3 calibrate_wheel_separation_multiplier.py --manual \
        <truth_before_x> <truth_before_y> <truth_before_z> <truth_before_w> \
        <truth_after_x>  <truth_after_y>  <truth_after_z>  <truth_after_w>  \
        <odom_before_x>  <odom_before_y>  <odom_before_z>  <odom_before_w>  \
        <odom_after_x>   <odom_after_y>   <odom_after_z>   <odom_after_w>
"""


import argparse
import math
import re
import subprocess
import sys
import time


GROUND_TRUTH_TOPIC = "/model/mobile_robot/odometry"
WHEEL_ODOM_TOPIC = "/diff_drive_controller/odom"
CONTROLLER_CMD_TOPIC = "/diff_drive_controller/cmd_vel_unstamped"


SNAPSHOT_TIMEOUT_S = 10.0
TYPICAL_RANGE = (0.95, 1.08)



# --------------------------------------------------------------------------
# Command execution helpers
# --------------------------------------------------------------------------


def run_capture(cmd: list, timeout: float = SNAPSHOT_TIMEOUT_S) -> str:
    """Run a command, capture stdout, enforce a timeout so a dead topic
    doesn't hang the script forever."""
    print(f"  $ {' '.join(cmd)}")
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError(
            f"Command timed out after {timeout}s: {' '.join(cmd)}\n"
            "The topic likely isn't publishing. Check `ros2 topic list` / "
            "`ign topic -l` before retrying."
        )
    if result.returncode != 0 and not result.stdout.strip():
        raise RuntimeError(
            f"Command failed (exit {result.returncode}): {' '.join(cmd)}\n"
            f"stderr:\n{result.stderr}"
        )
    return result.stdout



def _extract_field(text: str, anchor: str, field: str, window_size: int = 400):
    """Find `anchor { ... field: <val> ... }`-style block and pull out `field`."""
    idx = text.lower().find(anchor.lower())
    window = text[idx: idx + window_size] if idx != -1 else text
    m = re.search(rf'(?<![a-zA-Z_]){re.escape(field)}\s*:\s*(-?[0-9.eE+-]+)', window)
    if not m:
        raise ValueError(f"Could not parse field '{field}' near '{anchor}' from:\n{text}")
    return float(m.group(1))



def parse_orientation_z_w(text: str) -> tuple:
    """Extract orientation.z and orientation.w from either:
       - `ign topic -e` protobuf text output (has an 'orientation { ... }' block)
       - `ros2 topic echo --field pose.pose.orientation` YAML output (bare x/y/z/w)
    """
    z = _extract_field(text, "orientation", "z")
    w = _extract_field(text, "orientation", "w")
    return z, w



def parse_position_x_y(text: str) -> tuple:
    """Extract position.x and position.y from either:
       - `ign topic -e` protobuf text (a 'position { ... }' block, nested under 'pose')
       - `ros2 topic echo --field pose.pose.position` YAML output (bare x/y/z)
    """
    x = _extract_field(text, "position", "x")
    y = _extract_field(text, "position", "y")
    return x, y



def get_ground_truth_pose() -> dict:
    """Single `ign topic -e` call gives us both position and orientation
    since they're both under the same 'pose { ... }' message."""
    out = run_capture(["ign", "topic", "-e", "-t", GROUND_TRUTH_TOPIC, "-n", "1"])
    z, w = parse_orientation_z_w(out)
    x, y = parse_position_x_y(out)
    return {"x": x, "y": y, "z": z, "w": w}



def get_wheel_odom_pose() -> dict:
    """diff_drive_controller/odom doesn't support fetching two fields in one
    `ros2 topic echo --field` call, so issue two quick calls back-to-back."""
    orient_out = run_capture([
        "ros2", "topic", "echo", WHEEL_ODOM_TOPIC,
        "--field", "pose.pose.orientation", "--once",
    ])
    pos_out = run_capture([
        "ros2", "topic", "echo", WHEEL_ODOM_TOPIC,
        "--field", "pose.pose.position", "--once",
    ])
    z, w = parse_orientation_z_w(orient_out)
    x, y = parse_position_x_y(pos_out)
    return {"x": x, "y": y, "z": z, "w": w}



def perform_motion(linear_x: float, angular_z: float, duration: float):
    """Publish a constant (linear.x, angular.z) command for `duration` seconds
    directly to diff_drive_controller, then stop, bypassing twist_mux entirely.
    Setting linear_x=0.0 reproduces the original pure-spin test."""
    pub_cmd = [
        "ros2", "topic", "pub", CONTROLLER_CMD_TOPIC,
        "geometry_msgs/msg/Twist",
        f"{{linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}}}",
        "--rate", "10",
    ]
    print(f"  $ {' '.join(pub_cmd)}   (running for {duration}s)")
    proc = subprocess.Popen(pub_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(duration)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


    stop_cmd = [
        "ros2", "topic", "pub", CONTROLLER_CMD_TOPIC,
        "geometry_msgs/msg/Twist",
        "{linear: {x: 0.0}, angular: {z: 0.0}}", "--once",
    ]
    run_capture(stop_cmd, timeout=5.0)
    # give the robot a moment to settle before the AFTER snapshot
    time.sleep(1.0)



def publish_stop():
    """Publish a single zero-velocity Twist directly to the controller
    command topic. Used as the safety fallback whenever the calibration
    flow aborts before or during perform_motion(), so a rejected/invalid
    request can never leave a nonzero command latched on the topic."""
    stop_cmd = [
        "ros2", "topic", "pub", CONTROLLER_CMD_TOPIC,
        "geometry_msgs/msg/Twist",
        "{linear: {x: 0.0}, angular: {z: 0.0}}", "--once",
    ]
    try:
        run_capture(stop_cmd, timeout=5.0)
    except RuntimeError as e:
        print(f"  WARNING: failed to publish safety stop: {e}", file=sys.stderr)



# --------------------------------------------------------------------------
# Math
# --------------------------------------------------------------------------


def yaw_from_quat(z: float, w: float) -> float:
    """yaw = 2*atan2(z, w), valid for pure Z-axis rotation (x=y=0 in quat)."""
    return 2.0 * math.atan2(z, w)



def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle



def compute_multiplier(
    truth_before: dict, truth_after: dict, odom_before: dict, odom_after: dict,
    current_multiplier: float = 1.0,
) -> dict:
    yaw_truth_before = yaw_from_quat(truth_before["z"], truth_before["w"])
    yaw_truth_after = yaw_from_quat(truth_after["z"], truth_after["w"])
    yaw_odom_before = yaw_from_quat(odom_before["z"], odom_before["w"])
    yaw_odom_after = yaw_from_quat(odom_after["z"], odom_after["w"])


    delta_yaw_truth = normalize_angle(yaw_truth_after - yaw_truth_before)
    delta_yaw_odom = normalize_angle(yaw_odom_after - yaw_odom_before)


    if abs(delta_yaw_truth) < 1e-6:
        raise ValueError(
            "Ground-truth yaw delta is ~0 — the robot didn't actually "
            "rotate during the test window. Increase --duration or "
            "--angular-z and retry."
        )


    ratio = delta_yaw_odom / delta_yaw_truth


    dx_truth = truth_after["x"] - truth_before["x"]
    dy_truth = truth_after["y"] - truth_before["y"]
    dx_odom = odom_after["x"] - odom_before["x"]
    dy_odom = odom_after["y"] - odom_before["y"]
    net_displacement_truth = math.hypot(dx_truth, dy_truth)
    net_displacement_odom = math.hypot(dx_odom, dy_odom)
    final_position_error = math.hypot(dx_odom - dx_truth, dy_odom - dy_truth)


    return {
        "yaw_truth_before_deg": math.degrees(yaw_truth_before),
        "yaw_truth_after_deg": math.degrees(yaw_truth_after),
        "yaw_odom_before_deg": math.degrees(yaw_odom_before),
        "yaw_odom_after_deg": math.degrees(yaw_odom_after),
        "delta_yaw_truth_deg": math.degrees(delta_yaw_truth),
        "delta_yaw_odom_deg": math.degrees(delta_yaw_odom),
        "ratio_odom_over_truth": ratio,
        "current_multiplier": current_multiplier,
        "new_multiplier": current_multiplier * ratio,
        "net_displacement_truth_m": net_displacement_truth,
        "net_displacement_odom_m": net_displacement_odom,
        "final_position_error_m": final_position_error,
    }



def print_result(result: dict, mode: str):
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    print(f"  Ground-truth yaw:  before = {result['yaw_truth_before_deg']:8.3f} deg   "
          f"after = {result['yaw_truth_after_deg']:8.3f} deg")
    print(f"  Wheel odom yaw:    before = {result['yaw_odom_before_deg']:8.3f} deg   "
          f"after = {result['yaw_odom_after_deg']:8.3f} deg")
    print(f"\n  Delta yaw (truth): {result['delta_yaw_truth_deg']:8.3f} deg")
    print(f"  Delta yaw (odom):  {result['delta_yaw_odom_deg']:8.3f} deg")


    if mode == "arc":
        print(f"\n  Net displacement (truth): {result['net_displacement_truth_m']:.4f} m")
        print(f"  Net displacement (odom):  {result['net_displacement_odom_m']:.4f} m")
        print(f"  Final position error (odom vs truth): {result['final_position_error_m']:.4f} m")


    print(f"\n  ratio (odom / truth):        {result['ratio_odom_over_truth']:.6f}")
    print(f"  current wheel_separation_multiplier: {result['current_multiplier']:.6f}")
    print(f"  NEW wheel_separation_multiplier:      {result['new_multiplier']:.6f}")
    print("=" * 70)


    lo, hi = TYPICAL_RANGE
    if not (lo <= result["new_multiplier"] <= hi):
        print(
            f"\n  WARNING: {result['new_multiplier']:.4f} is outside the "
            f"typical range ({lo}-{hi}) noted in this repo's commit history "
            "for this wheelbase. Double-check:\n"
            "    - the robot was fully stationary at both snapshots\n"
            "    - both topics were sampled over the same time window\n"
            "  Consider re-running with a larger --duration for more signal."
        )


    if mode == "spin" and abs(result["ratio_odom_over_truth"] - 1.0) < 0.01:
        print(
            "\n  NOTE: ratio is ~1.0. A pure in-place spin cannot expose the "
            "front/rear virtual-axle bias described in this repo's commit "
            "history (8fea9ceca) — that bias only appears during COMBINED "
            "linear+angular motion. Re-run with --mode arc before trusting "
            "this result as final."
        )


    print(
        f"\nNext step: set wheel_separation_multiplier: {result['new_multiplier']:.4f} "
        "in config/controllers.yaml, relaunch, and re-run this script to confirm "
        "the odom/truth deltas now match within ~1-2%."
    )



# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------


def run_automated(mode: str, linear_x: float, angular_z: float, duration: float,
                   current_multiplier: float):
    print("=" * 70)
    print(f"Wheel separation multiplier calibration (automated, mode={mode})")
    print("=" * 70)


    # BUGFIX: validate `duration` before anything else -- before
    # expected_rotation_rad is computed and before perform_motion() can
    # publish any motion command. A non-finite (NaN/inf) or non-positive
    # duration would otherwise flow straight into
    # expected_rotation_rad = abs(angular_z) * duration
    # (silently producing NaN, or a non-positive value that always passes
    # the ">= math.pi" guard below) and then into perform_motion(), which
    # would either publish motion for an invalid/nonsensical time or skip
    # the stop command entirely. Reject here, but still issue the
    # zero-velocity safety stop before raising, since some prior state
    # (a previous run, a stuck publisher) may have left the robot moving.
    if not math.isfinite(duration) or duration <= 0:
        publish_stop()
        raise ValueError(
            f"Invalid --duration={duration!r}: must be a finite, positive "
            "number of seconds. No motion was commanded; a zero-velocity "
            "stop was published as a precaution."
        )


    # BUGFIX: reject a planned rotation of >= 180 deg BEFORE moving the
    # robot at all. normalize_angle() wraps any yaw delta to (-pi, pi],
    # so a true rotation of pi or more is aliased (e.g. +200 deg reads
    # back as -160 deg) and would silently corrupt the computed
    # multiplier with no indication anything went wrong. See the
    # IMPORTANT note in the module docstring.
    expected_rotation_rad = abs(angular_z) * duration
    if expected_rotation_rad >= math.pi:
        publish_stop()
        raise ValueError(
            f"Requested rotation ~{math.degrees(expected_rotation_rad):.1f} deg "
            f"(angular_z={angular_z} rad/s x duration={duration}s) is >= 180 deg.\n"
            "Comparing only BEFORE/AFTER endpoint yaw via quaternions cannot "
            "distinguish e.g. a +200 deg rotation from a -160 deg one -- "
            "normalize_angle() would silently alias it, corrupting the "
            "computed multiplier. Reduce --duration or --angular-z so the "
            "total commanded rotation stays below 180 deg, or extend this "
            "script to integrate intermediate yaw samples if you need a "
            "single test that covers a full rotation."
        )


    print("\n[1/4] Sanity-checking topics are alive...")
    truth_before = get_ground_truth_pose()
    odom_before = get_wheel_odom_pose()
    print(f"  ground truth (x,y,z,w) = {truth_before}")
    print(f"  wheel odom   (x,y,z,w) = {odom_before}")


    effective_linear_x = linear_x if mode == "arc" else 0.0
    print(f"\n[2/4] Commanding motion: linear.x={effective_linear_x}, "
          f"angular.z={angular_z} for {duration}s "
          f"(direct to {CONTROLLER_CMD_TOPIC}, bypassing twist_mux)...")
    perform_motion(effective_linear_x, angular_z, duration)


    print("\n[3/4] Capturing AFTER snapshot...")
    truth_after = get_ground_truth_pose()
    odom_after = get_wheel_odom_pose()
    print(f"  ground truth (x,y,z,w) = {truth_after}")
    print(f"  wheel odom   (x,y,z,w) = {odom_after}")


    print("\n[4/4] Computing multiplier...")
    result = compute_multiplier(truth_before, truth_after, odom_before, odom_after,
                                 current_multiplier)
    print_result(result, mode)



def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["spin", "arc"], default="spin",
                         help="'spin' = pure in-place rotation (cannot detect virtual-axle "
                              "bias). 'arc' = forward + turning simultaneously (recommended).")
    parser.add_argument("--linear-x", type=float, default=0.15,
                         help="Commanded linear velocity in m/s for --mode arc (default: 0.15). "
                              "Ignored in --mode spin (forced to 0.0).")
    parser.add_argument("--angular-z", type=float, default=0.3,
                         help="Commanded angular velocity in rad/s (default: 0.3)")
    parser.add_argument("--duration", type=float, default=10.0,
                         help="Motion duration in seconds (default: 10.0)")
    parser.add_argument("--current-multiplier", type=float, default=1.0,
                         help="Current wheel_separation_multiplier in controllers.yaml (default: 1.0)")
    parser.add_argument("--manual", nargs=16, type=float, metavar=(
        "TRUTH_BEFORE_X", "TRUTH_BEFORE_Y", "TRUTH_BEFORE_Z", "TRUTH_BEFORE_W",
        "TRUTH_AFTER_X", "TRUTH_AFTER_Y", "TRUTH_AFTER_Z", "TRUTH_AFTER_W",
        "ODOM_BEFORE_X", "ODOM_BEFORE_Y", "ODOM_BEFORE_Z", "ODOM_BEFORE_W",
        "ODOM_AFTER_X", "ODOM_AFTER_Y", "ODOM_AFTER_Z", "ODOM_AFTER_W"),
        help="Skip live commands; compute directly from 16 pose components "
             "(x, y, quat_z, quat_w for truth-before, truth-after, odom-before, odom-after). "
             "Valid only if the actual rotation between snapshots was < 180 deg.")
    args = parser.parse_args()


    if args.manual:
        v = args.manual
        truth_before = {"x": v[0], "y": v[1], "z": v[2], "w": v[3]}
        truth_after = {"x": v[4], "y": v[5], "z": v[6], "w": v[7]}
        odom_before = {"x": v[8], "y": v[9], "z": v[10], "w": v[11]}
        odom_after = {"x": v[12], "y": v[13], "z": v[14], "w": v[15]}
        result = compute_multiplier(truth_before, truth_after, odom_before, odom_after,
                                     args.current_multiplier)
        print_result(result, args.mode)
        return


    try:
        run_automated(args.mode, args.linear_x, args.angular_z, args.duration,
                      args.current_multiplier)
    except (RuntimeError, ValueError) as e:
        print(f"\nERROR: {e}", file=sys.stderr)
        sys.exit(1)



if __name__ == "__main__":
    main()
