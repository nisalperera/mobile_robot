#!/usr/bin/env python3
"""
diagnose_rotation_dropout.py

Diagnoses why a commanded (linear.x, angular.z) motion appears to "work for
a few seconds then decay into straight-line motion" in Gazebo, by logging
BOTH the raw command actually being published AND the robot's real achieved
twist/pose (from /diff_drive_controller/odom) at high frequency throughout
the whole motion window.

Unlike the calibration script, this uses rclpy directly (no `ros2 topic pub`
subprocess) so the command publisher itself can never be the confound —
it runs in-process on a ROS timer for the full duration, guaranteed.

SAFETY CHECK: before doing anything else, this script verifies there is
EXACTLY ONE publisher on /diff_drive_controller/odom, polling for up to
--publisher-check-timeout seconds to allow for DDS discovery delay. If a
previous test's ROS2/Ignition session (or a colliding topic remap) left
more than one publisher, or if 0 publishers are found (e.g. mismatched
ROS_DOMAIN_ID between shells), it refuses to proceed and explains why.

DROPOUT DETECTION: ignores the first --settle-time seconds of achieved
velocity samples before checking for a sustained drop below threshold.
This avoids false positives from the robot's normal, physically-expected
acceleration ramp-up at the start of motion (finite acceleration means
achieved velocity is near zero for a brief moment after the command is
first sent -- that is NOT a dropout).

It logs, at ~20 Hz:
  - the exact Twist message being published (what we intend)
  - the Odometry message's twist.linear.x / twist.angular.z (what the
    controller reports as ACHIEVED velocity)
  - the Odometry message's pose (x, y, yaw) over time

Then it:
  - writes everything to a CSV for offline plotting/inspection
  - automatically detects the timestamp at which achieved angular.z first
    sustainably drops below a threshold fraction of the commanded value
    AFTER the settle window, which is the real "it turned into a straight
    line at t=X seconds" moment, if one actually occurred.

Run this ON THE LAPTOP, with the sim already running.

Usage:
    python3 diagnose_rotation_dropout.py
    python3 diagnose_rotation_dropout.py --linear-x 0.15 --angular-z 0.3 --duration 15
    python3 diagnose_rotation_dropout.py --csv-out rotation_log.csv
    python3 diagnose_rotation_dropout.py --settle-time 1.5
    python3 diagnose_rotation_dropout.py --skip-publisher-check   # NOT recommended
"""

import argparse
import csv
import math
import sys
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
except ImportError:
    print(
        "ERROR: rclpy / geometry_msgs / nav_msgs not found.\n"
        "Run this on the Laptop with a sourced ROS2 Humble environment:\n"
        "    source /opt/ros/humble/setup.bash\n"
        "    source install/setup.bash   # (from your mobile_robot workspace)\n",
        file=sys.stderr,
    )
    sys.exit(1)

CONTROLLER_CMD_TOPIC = "/diff_drive_controller/cmd_vel_unstamped"
WHEEL_ODOM_TOPIC = "/diff_drive_controller/odom"


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Standard yaw extraction (robust even if x/y aren't exactly zero)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def find_dropout_time(samples: list, commanded_value: float, field: str,
                       threshold_fraction: float = 0.5, min_consecutive: int = 5,
                       settle_time_s: float = 1.0):
    """
    Returns the timestamp at which `field` first sustainably dropped below
    threshold_fraction * |commanded_value| for at least `min_consecutive`
    consecutive samples AFTER settle_time_s has elapsed, or None if no such
    dropout occurred. Samples before settle_time_s are ignored entirely,
    since a brief near-zero achieved velocity right after the command
    starts is normal acceleration ramp-up, not a real dropout.
    """
    threshold = threshold_fraction * abs(commanded_value)
    run = []
    for s in samples:
        if s["t"] < settle_time_s:
            continue
        if abs(s[field]) < threshold:
            run.append(s)
            if len(run) >= min_consecutive:
                return run[0]["t"]
        else:
            run = []
    return None


def check_single_publisher(node: "Node", topic: str, timeout_s: float = 10.0) -> int:
    """
    Poll for up to `timeout_s` seconds for exactly one publisher on `topic`,
    since DDS discovery can take a few seconds to populate the graph for a
    freshly created node. Aborts with a clear diagnosis if the final count
    (after the timeout) is anything other than exactly 1:
      - 0 publishers usually means a ROS_DOMAIN_ID mismatch between this
        shell and the shell that launched the sim.
      - >1 publishers usually means a leftover/zombie process from a prior
        test run, OR a colliding topic remap in a launch file bridging an
        unrelated Ignition topic onto this same ROS topic name.
    """
    deadline = time.time() + timeout_s
    last_count = -1
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.3)
        infos = node.get_publishers_info_by_topic(topic)
        count = len(infos)
        if count != last_count:
            names = ', '.join(i.node_name for i in infos) if infos else '(none yet)'
            print(f"  ...discovered {count} publisher(s) so far: {names}")
            last_count = count
        if count == 1:
            print(f"  OK: exactly 1 publisher on {topic} ({infos[0].node_name})")
            return count

    infos = node.get_publishers_info_by_topic(topic)
    names = ', '.join(i.node_name for i in infos) if infos else '(none)'
    print(
        f"\nERROR: Expected exactly 1 publisher on {topic}, found {len(infos)} "
        f"after {timeout_s}s of discovery.\nPublisher node(s): {names}\n\n"
        "If this is 0: check ROS_DOMAIN_ID matches between this shell and the "
        "shell that launched the sim:\n"
        "    echo $ROS_DOMAIN_ID   # run in both shells, compare\n\n"
        "If this is >1: check for a leftover/zombie process:\n"
        "    pkill -9 -f ign\n"
        "    pkill -9 -f ros2\n"
        "    ps aux | grep -E 'ign|ros2' | grep -v grep   # confirm empty\n"
        "...or a colliding topic remap in gz.launch.py bridging an unrelated "
        f"Ignition topic onto {topic}.\n"
        f"Verify with: ros2 topic info {topic} --verbose\n"
        "before relaunching the sim and re-running this script.",
        file=sys.stderr,
    )
    sys.exit(1)


class RotationDropoutDiagnostics(Node):
    def __init__(self, linear_x: float, angular_z: float, duration: float,
                 publish_rate_hz: float = 10.0, log_rate_hz: float = 20.0):
        super().__init__("rotation_dropout_diagnostics")
        self.linear_x = linear_x
        self.angular_z = angular_z
        self.duration = duration
        self.start_time = None

        self.cmd_log = []   # what we actually published
        self.odom_log = []  # what the controller reports as achieved

        self.cmd_pub = self.create_publisher(Twist, CONTROLLER_CMD_TOPIC, 10)
        self.create_subscription(Odometry, WHEEL_ODOM_TOPIC, self._odom_cb, 50)

        self._motion_timer = self.create_timer(1.0 / publish_rate_hz, self._publish_cmd)
        self._log_timer = self.create_timer(1.0 / log_rate_hz, self._log_cmd)
        self._stop_timer = None  # set once motion begins

    def _now_t(self) -> float:
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        return now - self.start_time

    def _publish_cmd(self):
        t = self._now_t()
        if t >= self.duration:
            # stop the robot and stop publishing further motion commands
            self.cmd_pub.publish(Twist())
            self._motion_timer.cancel()
            return
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.cmd_pub.publish(msg)

    def _log_cmd(self):
        t = self._now_t()
        if t > self.duration + 1.0:
            self._log_timer.cancel()
            return
        self.cmd_log.append({
            "t": round(t, 4),
            "lin_x_commanded": self.linear_x if t < self.duration else 0.0,
            "ang_z_commanded": self.angular_z if t < self.duration else 0.0,
        })

    def _odom_cb(self, msg: Odometry):
        t = self._now_t()
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_log.append({
            "t": round(t, 4),
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw_deg": math.degrees(yaw),
            "lin_x_achieved": msg.twist.twist.linear.x,
            "ang_z_achieved": msg.twist.twist.angular.z,
        })


def merge_logs(cmd_log: list, odom_log: list) -> list:
    """Merge cmd and odom logs into one time-sorted table for the CSV,
    carrying forward the most recent commanded value at each odom sample."""
    merged = []
    cmd_idx = 0
    last_cmd = {"lin_x_commanded": 0.0, "ang_z_commanded": 0.0}
    for o in odom_log:
        while cmd_idx < len(cmd_log) and cmd_log[cmd_idx]["t"] <= o["t"]:
            last_cmd = cmd_log[cmd_idx]
            cmd_idx += 1
        merged.append({**o, **{k: v for k, v in last_cmd.items() if k != "t"}})
    return merged


def print_analysis(merged: list, angular_z: float, linear_x: float, settle_time: float):
    print("\n" + "=" * 70)
    print("ANALYSIS")
    print("=" * 70)

    if not merged:
        print("  No odometry samples were logged — check that "
              f"{WHEEL_ODOM_TOPIC} is actually publishing.")
        return

    cmd_dropout_t = find_dropout_time(merged, angular_z, "ang_z_commanded",
                                       settle_time_s=settle_time)
    achieved_dropout_t = find_dropout_time(merged, angular_z, "ang_z_achieved",
                                            settle_time_s=settle_time)

    print(f"  Total samples logged: {len(merged)}")
    print(f"  Time range: {merged[0]['t']:.2f}s -> {merged[-1]['t']:.2f}s")
    print(f"  Ignoring first {settle_time:.2f}s as normal acceleration ramp-up "
          "before checking for dropout.")

    if cmd_dropout_t is not None:
        print(f"\n  Commanded angular.z dropped below 50% of {angular_z} rad/s "
              f"at t={cmd_dropout_t:.2f}s (after the settle window)")
        print("  -> This means OUR OWN PUBLISHER stopped sending the full "
              "command before --duration elapsed, or --duration was shorter "
              "than you expected. Check the script's timer/duration logic.")
    else:
        print(f"\n  Commanded angular.z stayed at {angular_z} rad/s for the "
              "full requested duration (as expected) — the publisher itself "
              "is NOT the problem.")

    if achieved_dropout_t is not None:
        print(f"\n  ACHIEVED angular.z (from {WHEEL_ODOM_TOPIC}) dropped below "
              f"50% of the commanded value at t={achieved_dropout_t:.2f}s "
              "(after the settle window), while the command was still being sent.")
        print(
            "  This is the actual 'turns into a straight line' moment you "
            "saw in Gazebo. Likely causes to check next:\n"
            "    - diff_drive_controller wheel velocity limits (per-joint "
            "<limit> tags in ros2_control.xacro) clamping the outer wheel's "
            "required speed once linear+angular combine, effectively "
            "reducing achievable angular.z\n"
            "    - controllers.yaml acceleration/deceleration limits "
            "(linear.x.max_acceleration / angular.z.max_acceleration) "
            "causing angular velocity to decay under a combined-motion load\n"
            "    - wheel slip re-appearing under combined load despite the "
            "mu=1.0/mu2=0.5 friction tuning (check real_time_factor via "
            "`ign topic -e -t /world/<world>/stats` during the test — if it "
            "drops sharply at the same timestamp, physics may be struggling, "
            "not the controller)"
        )
    else:
        print(f"\n  ACHIEVED angular.z tracked the commanded {angular_z} rad/s "
              "for the entire test (after the settle window) — no dropout "
              "detected. This looks like a healthy, correctly-behaving run.")

    print("=" * 70)


def write_csv(merged: list, path: str):
    if not merged:
        return
    fieldnames = list(merged[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(merged)
    print(f"\nFull time-series log written to: {path}")
    print("Columns: t, x, y, yaw_deg, lin_x_achieved, ang_z_achieved, "
          "lin_x_commanded, ang_z_commanded")
    print("Plot ang_z_commanded vs ang_z_achieved over t to see the dropout visually.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--linear-x", type=float, default=0.15,
                         help="Commanded linear velocity in m/s (default: 0.15)")
    parser.add_argument("--angular-z", type=float, default=0.3,
                         help="Commanded angular velocity in rad/s (default: 0.3)")
    parser.add_argument("--duration", type=float, default=15.0,
                         help="Motion duration in seconds (default: 15.0)")
    parser.add_argument("--settle-time", type=float, default=1.0,
                         help="Seconds of acceleration ramp-up to ignore before "
                              "checking for a real dropout (default: 1.0)")
    parser.add_argument("--csv-out", type=str, default="rotation_dropout_log.csv",
                         help="Output CSV path (default: rotation_dropout_log.csv)")
    parser.add_argument("--publisher-check-timeout", type=float, default=10.0,
                         help="Seconds to wait for DDS discovery of the odom "
                              "topic publisher before giving up (default: 10.0)")
    parser.add_argument("--skip-publisher-check", action="store_true",
                         help="Skip the single-publisher safety check (NOT recommended — "
                              "results will be meaningless if a zombie process or a "
                              "colliding topic remap is present)")
    args = parser.parse_args()

    print("=" * 70)
    print("Rotation dropout diagnostics")
    print("=" * 70)

    rclpy.init()
    node = RotationDropoutDiagnostics(args.linear_x, args.angular_z, args.duration)

    if not args.skip_publisher_check:
        print("\n[0/4] Checking for a single publisher on the odom topic "
              "(guards against leftover/zombie processes or colliding remaps)...")
        check_single_publisher(node, WHEEL_ODOM_TOPIC, timeout_s=args.publisher_check_timeout)

    print(f"\n  Commanding linear.x={args.linear_x}, angular.z={args.angular_z} "
          f"for {args.duration}s, direct to {CONTROLLER_CMD_TOPIC}")
    print(f"  Logging {WHEEL_ODOM_TOPIC} at high frequency throughout...\n")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait until motion + a settle buffer has fully elapsed
    time.sleep(args.duration + 2.0)

    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    merged = merge_logs(node.cmd_log, node.odom_log)
    write_csv(merged, args.csv_out)
    print_analysis(merged, args.angular_z, args.linear_x, args.settle_time)


if __name__ == "__main__":
    main()