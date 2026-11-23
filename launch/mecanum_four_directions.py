#!/usr/bin/env python3
"""Isolated Gazebo mecanum translation regression. Default operation is read-only."""
import argparse
import bisect
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

CMD = "/mecanum_controller/reference_unstamped"
ODOM = "/mecanum_controller/odometry"
STATUS = Path(tempfile.gettempdir()) / f"mecanum-four-directions-{os.getuid()}.json"
PHASES = [("FORWARD", .03, 0.), ("REVERSE", -.03, 0.),
          ("LEFT", 0., .03), ("RIGHT", 0., -.03)]
OFFSET = (.05, 0., .024)
WHEELS = [f"{end}_{side}_wheel" for end in ("front", "rear")
          for side in ("left", "right")]

def run(args):
    return subprocess.run(args, capture_output=True, text=True,
                          check=True, timeout=25).stdout

def parameters(node):
    import yaml
    values = yaml.safe_load(run(["ros2", "param", "dump", node]))
    if not isinstance(values, dict) or len(values) != 1:
        raise RuntimeError(f"Expected exactly one parameter node: {node}")
    return next(iter(values.values()))["ros__parameters"]

def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

def rotated(q, v):
    x, y, z, w = q
    a, b, c = v
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return (a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz,
            c+w*tz+x*ty-y*tx)

def clean(samples):
    by_time = {row[0]: row for row in samples}
    out = [list(by_time[t]) for t in sorted(by_time)]
    for i in range(1, len(out)):
        out[i][4] = out[i-1][4] + math.remainder(out[i][4]-out[i-1][4], 2*math.pi)
    return out

def interp(samples, t):
    i = bisect.bisect_left([row[0] for row in samples], t)
    if i == 0:
        return list(samples[0])
    if i == len(samples):
        return list(samples[-1])
    a, b = samples[i-1], samples[i]
    f = (t-a[0])/(b[0]-a[0])
    return [t] + [a[k]+f*(b[k]-a[k]) for k in range(1, 5)]

def metrics(samples, start, end):
    a, b = interp(samples, start), interp(samples, end)
    rows = [a] + [r for r in samples if start < r[0] < end] + [b]
    cs, sn = math.cos(a[4]), math.sin(a[4])
    def local(r):
        dx, dy = r[1]-a[1], r[2]-a[2]
        return cs*dx+sn*dy, -sn*dx+cs*dy
    forward, left = local(b)
    tail_start = max(start, end-1.)
    tail = [interp(samples, tail_start)] + [r for r in rows if r[0] > tail_start]
    span = [max(r[k] for r in tail)-min(r[k] for r in tail) for k in (1, 2, 4)]
    return dict(forward=forward, left=left, yaw=b[4]-a[4],
                excursion=max(math.hypot(*local(r)) for r in rows), span=span)

def split_commands(commands):
    groups = []
    active = False
    for t, values in commands:
        moving = any(abs(v) > 1e-8 for v in values)
        if moving:
            if not active:
                groups.append(dict(start=t, moves=[], zeros=[]))
            groups[-1]["moves"].append((t, values))
        elif groups:
            groups[-1]["zeros"].append(t)
        active = moving
    return groups

def summarize(data, commands, reason):
    sources = {name: clean(data[name]) for name in ("gazebo_center", "wheel_odom")}
    lines = ["=== FOUR-DIRECTION TRANSLATION SUMMARY ===", f"Capture end: {reason}",
             "Collision proxy: 0.001 m; wheel radius: 0.024 m; kinematic sum: 0.275 m",
             f"Gazebo model-to-center offset: {OFFSET} m",
             "Displacements use each source's heading at that phase's start."]
    if not all(sources.values()):
        return "\n".join(lines+["INCOMPLETE: missing pose source."])
    start = max(r[0][0] for r in sources.values())
    end = min(r[-1][0] for r in sources.values())
    lines.append(f"SHARED INTERVAL: {start:.3f} to {end:.3f} s")
    if end <= start:
        return "\n".join(lines+["INCOMPLETE: no shared interval."])
    for name, rows in sources.items():
        gaps = [b[0]-a[0] for a, b in zip(rows, rows[1:])]
        lines.append(f"{name}: samples={len(rows)}; maximum sample gap={max(gaps, default=0):.3f} s")
    groups = split_commands(commands)
    if groups:
        lines.append(f"PRE-COMMAND BASELINE: {groups[0]['start']-start:.3f} s")
    for i, (name, vx, vy) in enumerate(PHASES):
        lines.append(f"\n=== {name} ===")
        if i >= len(groups):
            lines.append("INCOMPLETE: phase not observed")
            continue
        group = groups[i]
        values = sorted(set(tuple(v) for _, v in group["moves"]))
        expected = (vx, vy, 0., 0., 0., 0.)
        valid = (len(group["moves"]) == 20 and len(group["zeros"]) == 10
                 and all(all(abs(a-b) < 1e-7 for a,b in zip(v, expected)) for v in values))
        a = max(start, group["start"]-.05)
        b = min(end, groups[i+1]["start"]-.1) if i+1 < len(groups) else end
        lines += [f"Commands={len(group['moves'])}; subsequent zero messages={len(group['zeros'])}",
                  f"Twist values [vx,vy,vz,wx,wy,wz]: {values}",
                  f"Sequence/count check: {'OK' if valid else 'CHECK REQUIRED'}",
                  f"Evaluation window: {a:.3f} to {b:.3f} s"]
        if b <= a or b < group["start"] or not group["zeros"]:
            lines.append("INCOMPLETE: missing evaluation window or stopping commands")
            continue
        lines.append(f"Observed settling coverage after first zero: {b-group['zeros'][0]:.3f} s")
        found = {}
        for source, rows in sources.items():
            m = metrics(rows, a, b)
            found[source] = m
            along = (m["forward"] if vx else m["left"]) * (1 if (vx or vy)>0 else -1)
            cross = m["left"] if vx else m["forward"]
            lines += [f"{source}:",
                      f"  Center forward={1000*m['forward']:.6f} mm; left={1000*m['left']:.6f} mm",
                      f"  Signed progress along commanded direction={1000*along:.6f} mm; cross-axis={1000*cross:.6f} mm",
                      f"  Yaw change={math.degrees(m['yaw']):.6f} deg",
                      f"  Maximum center excursion={1000*m['excursion']:.6f} mm",
                      f"  Final-1.00s span: world X={1000*m['span'][0]:.6f} mm; world Y={1000*m['span'][1]:.6f} mm; yaw={math.degrees(m['span'][2]):.6f} deg"]
        g, o = found["gazebo_center"], found["wheel_odom"]
        delta = (g["forward"]-o["forward"], g["left"]-o["left"])
        main_g, main_o = (g["forward"], o["forward"]) if vx else (g["left"], o["left"])
        lines.append(f"Gazebo minus odometry: forward={1000*delta[0]:.6f} mm; left={1000*delta[1]:.6f} mm; yaw={math.degrees(g['yaw']-o['yaw']):.6f} deg")
        if abs(main_o) > .001:
            lines.append(f"Along-axis magnitude difference relative to odometry={100*(abs(main_g)/abs(main_o)-1):.4f}%")
    if len(groups) != 4:
        lines.append(f"WARNING: observed {len(groups)} nonzero command groups, expected 4.")
    lines.append("\nNo automatic engineering pass/fail. Review all four phases and visual observations.")
    return "\n".join(lines)

def write_status(values):
    tmp = STATUS.with_suffix(".tmp")
    tmp.write_text(json.dumps(dict(values, heartbeat=time.time())))
    tmp.replace(STATUS)

def read_status():
    info = json.loads(STATUS.read_text())
    if time.time()-info["heartbeat"] > 3.:
        raise RuntimeError("Capture heartbeat stale. No further nonzero commands allowed.")
    return info


# Fortress observer backend: existing ign CLI, no gz Python bindings.
import ast as _proto_ast
import re as _proto_re
from types import SimpleNamespace as _Namespace


def _proto_text(text):
    tokens = _proto_re.findall(r'"(?:\\.|[^"\\])*"|[{}:]|[^\s{}:]+', text)
    index = 0

    def message(nested=False):
        nonlocal index
        result = {}
        while index < len(tokens):
            key = tokens[index]
            index += 1
            if key == "}":
                if not nested:
                    raise ValueError("Unexpected closing protobuf brace")
                return result
            if index < len(tokens) and tokens[index] == ":":
                index += 1
            if index >= len(tokens):
                raise ValueError("Incomplete protobuf field")
            value = tokens[index]
            index += 1
            if value == "{":
                value = message(True)
            elif value.startswith('"'):
                value = _proto_ast.literal_eval(value)
            result.setdefault(key, []).append(value)
        if nested:
            raise ValueError("Unclosed protobuf message")
        return result
    return message()


def _first(message, key, default=None):
    return message.get(key, [default])[0]


def _ign_scene(service):
    text = run(["ign", "service", "-s", service,
                "--reqtype", "ignition.msgs.Empty",
                "--reptype", "ignition.msgs.Scene",
                "--timeout", "5000", "--req", ""])
    parsed = _proto_text(text)
    models = []
    for model in parsed.get("model", []):
        links = [_Namespace(name=_first(link, "name", ""))
                 for link in model.get("link", [])]
        models.append(_Namespace(name=_first(model, "name", ""),
                                 id=int(_first(model, "id", 0)), link=links))
    print("Fortress scene models and direct link names:", flush=True)
    for model in models[:30]:
        print(f"  {model.name!r}, id={model.id}: {[link.name for link in model.link]}", flush=True)
    if not models:
        raise RuntimeError(f"Native ign scene response has no models. Response excerpt: {text[:1800]!r}")
    return _Namespace(model=models)


class _IgnPoseObserver:
    def __init__(self, topic, callback):
        self.error = None
        self._closed = False
        self._stderr = tempfile.TemporaryFile(mode="w+t")
        self.proc = subprocess.Popen(
            ["stdbuf", "-oL", "ign", "topic", "-e", "-t", topic],
            stdout=subprocess.PIPE, stderr=self._stderr,
            text=True, bufsize=1,
        )
        self._thread = threading.Thread(target=self._read, args=(callback,), daemon=True)
        self._thread.start()

    def _read(self, callback):
        stamp = None
        chunk = []
        depth = 0
        try:
            for line in self.proc.stdout:
                stripped = line.strip()
                if not chunk and not _proto_re.match(r"^(header|pose)\s*\{", stripped):
                    continue
                chunk.append(line)
                braces = _proto_re.sub(r'"(?:\\.|[^"\\])*"', '""', line)
                depth += braces.count("{") - braces.count("}")
                if depth < 0 or len(chunk) > 10000:
                    raise RuntimeError("Invalid or oversized ign pose text block")
                if depth:
                    continue
                parsed = _proto_text("".join(chunk))
                chunk = []
                if "header" in parsed:
                    header = _first(parsed, "header", {})
                    value = _first(header, "stamp", {})
                    stamp = _Namespace(sec=int(_first(value, "sec", 0)),
                                       nsec=int(_first(value, "nsec", 0)))
                for pose in parsed.get("pose", []):
                    if stamp is None:
                        continue
                    position = _first(pose, "position", {})
                    orientation = _first(pose, "orientation", {})
                    p = _Namespace(**{key: float(_first(position, key, 0)) for key in ("x", "y", "z")})
                    q = _Namespace(**{key: float(_first(orientation, key, 0)) for key in ("x", "y", "z", "w")})
                    entity = _Namespace(id=int(_first(pose, "id", 0)),
                                        name=_first(pose, "name", ""),
                                        position=p, orientation=q)
                    callback(_Namespace(header=_Namespace(stamp=stamp), pose=[entity]))
            if not self._closed:
                self.error = "ign topic echo ended unexpectedly"
        except Exception as exc:
            if not self._closed:
                self.error = str(exc)

    def check(self):
        if self.error or self.proc.poll() is not None:
            self._stderr.flush()
            self._stderr.seek(0)
            detail = self._stderr.read()[-1800:]
            raise RuntimeError(f"Fortress pose observer failed: {self.error}; {detail}")

    def close(self):
        self._closed = True
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3)
        self._thread.join(timeout=2)
        self.proc.stdout.close()
        self._stderr.close()


def capture():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    if STATUS.exists():
        old = json.loads(STATUS.read_text())
        if old.get("state") == "ready" and time.time()-old.get("heartbeat", 0) < 3:
            raise RuntimeError("Another capture appears active. Stop it first.")
    write_status(dict(state="initializing"))
    root = ET.fromstring(parameters("/robot_state_publisher")["robot_description"])
    for wheel in WHEELS:
        cylinders = root.findall(f"link[@name='{wheel}']/collision/geometry/cylinder")
        if len(cylinders) != 1 or abs(float(cylinders[0].get("length"))-.001)>1e-9 or abs(float(cylinders[0].get("radius"))-.024)>1e-9:
            raise RuntimeError(f"Expected 1 mm collision width and 24 mm radius: {wheel}")
    for name, xyz in [("base_footprint_joint", (0.,0.,.024)),
                      ("chassis_joint",(-.1,0.,0.)),
                      ("mecanum_test_center_joint",(.15,0.,0.))]:
        origin = root.find(f"joint[@name='{name}']/origin")
        if origin is None or any(abs(a-b)>1e-9 for a,b in zip(map(float,origin.get("xyz","0 0 0").split()),xyz)) or any(abs(v)>1e-9 for v in map(float,origin.get("rpy","0 0 0").split())):
            raise RuntimeError(f"Unexpected center transform at {name}")
    p = parameters("/mecanum_controller")
    k = p["kinematics"]
    if (p["base_frame_id"] != "mecanum_test_center" or p["use_stamped_vel"]
        or abs(k["wheels_radius"]-.024)>1e-9
        or abs(k["sum_of_robot_center_projection_on_X_Y_axis"]-.275)>1e-9
        or any(abs(k["base_frame_offset"][key])>1e-9 for key in ("x","y","theta"))):
        raise RuntimeError("Unexpected runtime controller parameters")
    topics = run(["ign","topic","-l"]).splitlines()
    choices = [t.strip() for t in topics if t.strip().startswith("/world/") and t.strip().endswith("/pose/info") and "/dynamic_pose/" not in t]
    if len(choices) != 1:
        raise RuntimeError(f"Expected one world pose topic, found {choices}")
    pose_topic = choices[0]
    world_prefix = pose_topic[:-len("/pose/info")]
    scene = _ign_scene(world_prefix + "/scene/info")
    models = [m for m in scene.model if set(WHEELS).issubset({link.name.rsplit("::", 1)[-1] for link in m.link})]
    if len(models) != 1:
        raise RuntimeError("Cannot identify one top-level robot model by its four wheel links. No commands published.")
    model = models[0]
    print(f"Gazebo topic: {pose_topic}; model: {model.name!r}; entity ID: {model.id}", flush=True)
    rclpy.init()
    node = rclpy.create_node("mecanum_four_direction_capture")
    lock = threading.Lock()
    data = {key: [] for key in ("gazebo_model","gazebo_center","wheel_odom")}
    commands, raw = [], []
    clock = [None, time.monotonic()]
    error = [None]
    def add(source, t, pos, q, velocity=(0.,0.,0.)):
        if data[source] and t < data[source][-1][0]-.05:
            error[0] = "Simulation timestamp moved backwards"
        data[source].append((t,*pos,yaw(q)))
        raw.append((source,t,*pos,*q,*velocity))
    def on_pose(msg):
        matches = [pose for pose in msg.pose if pose.id == model.id]
        if len(matches) != 1:
            return
        p = matches[0]
        t = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9
        pos = (p.position.x,p.position.y,p.position.z)
        q = (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
        off = rotated(q, OFFSET)
        with lock:
            add("gazebo_model",t,pos,q)
            add("gazebo_center",t,tuple(a+b for a,b in zip(pos,off)),q)
    def on_clock(msg):
        t = msg.clock.sec + msg.clock.nanosec*1e-9
        if clock[0] is not None and t < clock[0]-.05:
            error[0] = "Simulation clock reset"
        if t != clock[0]:
            clock[:] = [t,time.monotonic()]
    def on_odom(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        with lock:
            add("wheel_odom",t,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w),(v.linear.x,v.linear.y,v.angular.z))
    def on_cmd(msg):
        if clock[0] is None:
            return
        v = (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        commands.append((clock[0],v))
    node.create_subscription(Clock,"/clock",on_clock,qos_profile_sensor_data)
    node.create_subscription(Odometry,ODOM,on_odom,qos_profile_sensor_data)
    node.create_subscription(Twist,CMD,on_cmd,200)
    observer = _IgnPoseObserver(pose_topic, on_pose)
    fd, filename = tempfile.mkstemp(prefix="mecanum-four-directions-", suffix=".csv", dir=Path.cwd())
    os.close(fd)
    stem = Path(filename)
    ready = None
    beginning = time.monotonic()
    last_status = 0.
    reason = "capture time limit"
    print("Waiting for live /clock, Gazebo poses and wheel odometry...",flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            observer.check()
            now = time.monotonic()
            with lock:
                latest = {name: list(rows[bisect.bisect_left(rows, (rows[-1][0]-1.5,)):])
                          if rows else [] for name, rows in data.items()}
            if error[0]:
                raise RuntimeError(error[0])
            if ready is None:
                if now-beginning>25:
                    raise RuntimeError("Timed out waiting for live streams")
                if clock[0] is None or not all(latest.values()):
                    continue
                if any(abs(rows[-1][0]-clock[0])>.3 for rows in latest.values()):
                    continue
                if node.get_publishers_info_by_topic(CMD):
                    raise RuntimeError("Another command publisher exists. Stop teleop/other publishers first.")
                ready = clock[0]
                print("READY: observer only, no commands published. Capture budget: 80 simulation seconds.",flush=True)
            if now-clock[1]>5:
                raise RuntimeError("Simulation clock stalled for 5 wall seconds")
            if any(clock[0]-rows[-1][0]>.5 for rows in latest.values()):
                raise RuntimeError("Pose stream stale by more than 0.5 simulation seconds")
            settled = False
            rows = clean(latest["gazebo_center"])
            if rows[-1][0]-rows[0][0]>=1.:
                tail = [r for r in rows if r[0]>=rows[-1][0]-1.]
                settled = (math.hypot(max(r[1] for r in tail)-min(r[1] for r in tail),
                                      max(r[2] for r in tail)-min(r[2] for r in tail)) <= .001
                           and max(r[4] for r in tail)-min(r[4] for r in tail) <= math.radians(.5))
            if now-last_status>.2:
                write_status(dict(state="ready",sim_time=clock[0],deadline=ready+80.,
                                  ready_sim=ready,settled=settled,csv=str(stem),
                                  session=str(stem)))
                last_status=now
            groups = split_commands(commands)
            if len(groups)>=4 and len(groups[-1]["zeros"])>=10 and clock[0]-groups[-1]["zeros"][-1]>=4.:
                reason="four phases and final settling interval captured"
                break
            if clock[0]-ready>=80 or now-beginning>=600:
                break
    except KeyboardInterrupt:
        reason="observer interrupted; observer publishes no stopping commands"
    except Exception as exc:
        reason=f"ABORTED: {exc}"
    finally:
        write_status(dict(state="finished",reason=reason))
        observer.close()
        with lock:
            saved = {key:list(rows) for key,rows in data.items()}
            saved_raw = list(raw)
        with stem.open("w",newline="") as f:
            w=csv.writer(f)
            w.writerow(["source","sim_time","x","y","z","qx","qy","qz","qw","vx","vy","wz"])
            w.writerows(sorted(saved_raw,key=lambda row:row[1]))
        command_file=stem.with_name(stem.stem+"-commands.csv")
        with command_file.open("w",newline="") as f:
            w=csv.writer(f);w.writerow(["sim_time","vx","vy","vz","wx","wy","wz"])
            w.writerows((t,*v) for t,v in commands)
        report=summarize(saved,commands,reason)
        report_file=stem.with_suffix(".txt")
        report_file.write_text(f"POSE FILE: {stem}\nCOMMAND FILE: {command_file}\n"+report+"\n")
        print(f"\nPOSE FILE: {stem}\nCOMMAND FILE: {command_file}\nSUMMARY FILE: {report_file}\n{report}",flush=True)
        node.destroy_node();rclpy.shutdown()

def drive():
    import rclpy
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data
    initial=read_status()
    if initial.get("state")!="ready" or initial["deadline"]-initial["sim_time"]<35:
        raise RuntimeError("Capture not ready or fewer than 35 simulation seconds remain")
    if input("Confirm simulated area is clear and 1 mm diagnostic is active. Type RUN: ")!="RUN":
        print("Cancelled. No commands published.");return
    rclpy.init();node=rclpy.create_node("mecanum_four_direction_commands")
    clk=[None,time.monotonic()]
    def update(msg):
        t=msg.clock.sec+msg.clock.nanosec*1e-9
        if clk[0] is not None and t<clk[0]-.05:
            raise RuntimeError("Simulation clock reset")
        if t!=clk[0]:clk[:]=[t,time.monotonic()]
    node.create_subscription(Clock,"/clock",update,qos_profile_sensor_data)
    pub=None
    success=False
    session=initial["session"]
    def check():
        status=read_status()
        if status.get("state")!="ready" or status.get("session")!=session:
            raise RuntimeError("Capture stopped or session changed")
        if time.monotonic()-clk[1]>3:
            raise RuntimeError("Clock stalled")
        if len(node.get_publishers_info_by_topic(CMD))>1:
            raise RuntimeError("Another command publisher appeared")
        return status
    def wait_until(target):
        while clk[0] is None or clk[0]<target:
            rclpy.spin_once(node,timeout_sec=.01)
            check()
    try:
        limit=time.monotonic()+10
        while clk[0] is None or node.count_subscribers(CMD)<2:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()>limit:raise RuntimeError("Clock or command subscribers unavailable")
        if node.get_publishers_info_by_topic(CMD):
            raise RuntimeError("Existing command publisher; refusing to compete")
        pub=node.create_publisher(Twist,CMD,10)
        wait_until(clk[0]+1.)
        status=check()
        if status["deadline"]-clk[0]<35:raise RuntimeError("Insufficient capture budget")
        wait_until(max(clk[0],status["ready_sim"])+3.)
        if not check()["settled"]:raise RuntimeError("Robot not stationary before test")
        for name,vx,vy in PHASES:
            print(f"{name}: vx={vx:+.2f}, vy={vy:+.2f} m/s; 20 messages at 10 Hz",flush=True)
            msg=Twist();msg.linear.x=vx;msg.linear.y=vy
            for _ in range(20):
                check();pub.publish(msg);wait_until(clk[0]+.1)
            for _ in range(10):
                pub.publish(Twist());wait_until(clk[0]+.1)
            wait_until(clk[0]+3.)
            if not check()["settled"]:
                raise RuntimeError(f"{name}: failed settling gate (1 mm / 0.5 deg over last second)")
        success=True
        print("All four command phases finished. Wait for capture summary.",flush=True)
    finally:
        if pub is not None and not success:
            print("INTERRUPTED/ABORTED: sending zero commands. Do not resume sequence.",flush=True)
            for _ in range(10):
                pub.publish(Twist())
                time.sleep(.1)
        node.destroy_node();rclpy.shutdown()

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode",choices=["capture","drive"],nargs="?",default="capture")
    args=parser.parse_args()
    try:
        capture() if args.mode=="capture" else drive()
    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception as exc:
        if args.mode=="capture":
            write_status(dict(state="failed",reason=str(exc)))
        raise SystemExit(f"ERROR: {exc}")
