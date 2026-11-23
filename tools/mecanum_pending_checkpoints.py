#!/usr/bin/env python3
"""Read-only capture and separately armed commands for isolated Fortress validation."""
import argparse
import ast
import bisect
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import uuid
import xml.etree.ElementTree as ET

CMD = "/mecanum_controller/reference_unstamped"
ODOM = "/mecanum_controller/odometry"
WORLD = "house_world"
MODEL = "mobile_robot"
OFFSET = (.05, 0., .024)
WHEELS = {f"{end}_{side}_wheel" for end in ("front", "rear") for side in ("left", "right")}
STATUS = Path(tempfile.gettempdir()) / f"mecanum-pending-{os.getuid()}.json"
LOCK = Path(tempfile.gettempdir()) / f"mecanum-pending-{os.getuid()}"
POSE_GAP_LIMIT = .1
STOP_XY = .0001
STOP_YAW = math.radians(.1)
YAW_SCALE_PERCENT = 1.
YAW_CENTER_EXCURSION = .001
REPEAT_GAIN_CV_PERCENT = 1.
COMBINED_ENDPOINT_XY = .001
COMBINED_MAX_XY = .002
COMBINED_MAX_YAW = math.radians(.1)


def phases(checkpoint):
    if checkpoint in ("yaw_repeat", "yaw_second"):
        rate = .1 if checkpoint == "yaw_repeat" else .05
        return [(f"PAIR_{pair}_{sign}", 0., 0., multiplier*rate)
                for pair in range(1, 4)
                for sign, multiplier in (("CCW", 1), ("CW", -1))]
    return [("FORWARD_CCW", .02, 0., .05),
            ("REVERSE_CW", -.02, 0., -.05),
            ("LEFT_CCW", 0., .02, .05),
            ("RIGHT_CW", 0., -.02, -.05)]


def command_vector(phase):
    _, vx, vy, wz = phase
    return (vx, vy, 0., 0., 0., wz)


def run(args):
    result = subprocess.run(args, capture_output=True, text=True, timeout=25)
    if result.returncode:
        raise RuntimeError(f"Command failed: {args!r}\n{result.stdout}\n{result.stderr}")
    return result.stdout


def parameters(node):
    import yaml
    values = yaml.safe_load(run(["ros2", "param", "dump", node]))
    if not isinstance(values, dict) or len(values) != 1:
        raise RuntimeError(f"Expected exactly one parameter node: {node}")
    return next(iter(values.values()))["ros__parameters"]


def proto_text(text):
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|[{}:]|[^\s{}:]+', text)
    index = 0
    def message(nested=False):
        nonlocal index
        out = {}
        while index < len(tokens):
            key = tokens[index]; index += 1
            if key == "}":
                if not nested:
                    raise ValueError("Unexpected closing protobuf brace")
                return out
            if index < len(tokens) and tokens[index] == ":":
                index += 1
            if index >= len(tokens):
                raise ValueError("Incomplete protobuf field")
            value = tokens[index]; index += 1
            if value == "{":
                value = message(True)
            elif value.startswith('"'):
                value = ast.literal_eval(value)
            out.setdefault(key, []).append(value)
        if nested:
            raise ValueError("Unclosed protobuf block")
        return out
    return message()


def first(message, field, default=None):
    return message.get(field, [default])[0]


def identify_model():
    topic = f"/world/{WORLD}/pose/info"
    if topic not in run(["ign", "topic", "-l"]).splitlines():
        raise RuntimeError(f"Missing native Fortress topic: {topic}")
    text = run(["ign", "service", "-s", f"/world/{WORLD}/scene/info",
                "--reqtype", "ignition.msgs.Empty", "--reptype", "ignition.msgs.Scene",
                "--timeout", "5000", "--req", ""])
    models = proto_text(text).get("model", [])
    found = [m for m in models
             if first(m, "name", "") == MODEL
             and WHEELS.issubset({first(link, "name", "").rsplit("::", 1)[-1]
                                 for link in m.get("link", [])})]
    if len(found) != 1:
        details = [(first(m, "name", ""),
                    [first(link, "name", "") for link in m.get("link", [])]) for m in models]
        raise RuntimeError(f"Cannot uniquely identify {MODEL!r} with its four wheel links: {details}")
    return topic, int(first(found[0], "id", 0))


def verify_configuration():
    for executable in ("ign", "stdbuf", "ros2"):
        if not shutil.which(executable):
            raise RuntimeError(f"Missing existing command: {executable}. Do not install packages blindly.")
    root = ET.fromstring(parameters("/robot_state_publisher")["robot_description"])
    for wheel in sorted(WHEELS):
        cylinders = root.findall(f"link[@name='{wheel}']/collision/geometry/cylinder")
        if len(cylinders) != 1:
            raise RuntimeError(f"Expected one collision cylinder: {wheel}")
        if (abs(float(cylinders[0].get("length"))-.001)>1e-9
            or abs(float(cylinders[0].get("radius"))-.024)>1e-9):
            raise RuntimeError(f"Wrong collision width/radius for {wheel}; expected .001/.024 m")
    expected = [
        ("base_footprint_joint", "base_footprint", "base_link", (0.,0.,.024)),
        ("chassis_joint", "base_link", "chassis", (-.1,0.,0.)),
        ("mecanum_test_center_joint", "chassis", "mecanum_test_center", (.15,0.,0.)),
    ]
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = {link.get("name") for link in root.findall("link")}-children
    if roots != {"base_footprint"}:
        raise RuntimeError(f"Unexpected URDF root: {roots}")
    for name, parent, child, xyz in expected:
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.get("type") != "fixed":
            raise RuntimeError(f"Missing fixed transform: {name}")
        origin = joint.find("origin")
        actual_xyz = list(map(float, origin.get("xyz", "0 0 0").split())) if origin is not None else [0.]*3
        actual_rpy = list(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else [0.]*3
        if (joint.find("parent").get("link") != parent
            or joint.find("child").get("link") != child
            or len(actual_xyz) != 3 or len(actual_rpy) != 3
            or any(abs(a-b)>1e-9 for a,b in zip(actual_xyz,xyz))
            or any(abs(a)>1e-9 for a in actual_rpy)):
            raise RuntimeError(f"Unexpected center transform: {name}")
    p = parameters("/mecanum_controller")
    k = p["kinematics"]
    if (not p["use_sim_time"] or p["use_stamped_vel"] or p["enable_odom_tf"]
        or p["base_frame_id"] != "mecanum_test_center"
        or abs(p["reference_timeout"]-.5)>1e-9
        or abs(k["wheels_radius"]-.024)>1e-9
        or abs(k["sum_of_robot_center_projection_on_X_Y_axis"]-.275)>1e-9
        or any(abs(k["base_frame_offset"][key])>1e-9 for key in ("x","y","theta"))):
        raise RuntimeError("Runtime controller configuration differs from the validated baseline")


class PoseObserver:
    def __init__(self, topic, callback):
        self.error = None
        self.closed = False
        self.stderr = tempfile.TemporaryFile(mode="w+t")
        self.proc = subprocess.Popen(["stdbuf", "-oL", "ign", "topic", "-e", "-t", topic],
                                     stdout=subprocess.PIPE, stderr=self.stderr, text=True, bufsize=1)
        self.thread = threading.Thread(target=self.read, args=(callback,), daemon=True)
        self.thread.start()

    def read(self, callback):
        stamp = None; chunk = []; depth = 0
        try:
            for line in self.proc.stdout:
                if not chunk and not re.match(r"^(header|pose)\s*\{", line.strip()):
                    continue
                chunk.append(line)
                stripped = re.sub(r'"(?:\\.|[^"\\])*"', '""', line)
                depth += stripped.count("{")-stripped.count("}")
                if depth < 0 or len(chunk)>10000:
                    raise RuntimeError("Invalid ign pose text block")
                if depth:
                    continue
                value = proto_text("".join(chunk)); chunk = []
                if "header" in value:
                    header_stamp = first(first(value,"header",{}), "stamp", {})
                    stamp = int(first(header_stamp,"sec",0))+int(first(header_stamp,"nsec",0))*1e-9
                if stamp is not None:
                    for pose in value.get("pose", []):
                        pos = first(pose,"position",{}); ori = first(pose,"orientation",{})
                        callback(stamp, int(first(pose,"id",0)),
                                 tuple(float(first(pos,k,0)) for k in ("x","y","z")),
                                 tuple(float(first(ori,k,0)) for k in ("x","y","z","w")))
            if not self.closed:
                self.error = "ign topic echo exited"
        except Exception as exc:
            if not self.closed:
                self.error = str(exc)

    def check(self):
        if self.error or self.proc.poll() is not None:
            self.stderr.flush(); self.stderr.seek(0)
            raise RuntimeError(f"Native pose reader failed: {self.error}; {self.stderr.read()[-1500:]}")

    def close(self):
        self.closed = True
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill(); self.proc.wait(timeout=3)
        self.thread.join(timeout=2)
        self.proc.stdout.close(); self.stderr.close()


def angle(q):
    x,y,z,w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rotate(q, v):
    x,y,z,w = q; a,b,c = v
    tx,ty,tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx


class Series:
    def __init__(self, samples):
        unique = {row[0]: row for row in samples}
        self.rows = [list(unique[t]) for t in sorted(unique)]
        for i in range(1,len(self.rows)):
            self.rows[i][4] = self.rows[i-1][4]+math.remainder(self.rows[i][4]-self.rows[i-1][4],2*math.pi)
        self.times = [r[0] for r in self.rows]

    def at(self, t):
        if not self.rows or t < self.times[0]-1e-8 or t > self.times[-1]+1e-8:
            raise ValueError("Attempted interpolation outside measured coverage")
        i = bisect.bisect_left(self.times,t)
        if i == 0: return list(self.rows[0])
        if i == len(self.rows): return list(self.rows[-1])
        a,b = self.rows[i-1],self.rows[i]
        f = (t-a[0])/(b[0]-a[0])
        return [t]+[a[k]+f*(b[k]-a[k]) for k in range(1,5)]

    def relative(self, t, origin):
        row=self.at(t); dx,dy=row[1]-origin[1],row[2]-origin[2]
        c,s=math.cos(origin[4]),math.sin(origin[4])
        return c*dx+s*dy, -s*dx+c*dy, row[4]-origin[4]

    def gap(self):
        return max((b-a for a,b in zip(self.times,self.times[1:])),default=0.)


def windows(commands):
    out=[]; moving=False
    for t,v in commands:
        nonzero=any(abs(a)>1e-8 for a in v)
        if nonzero:
            if not moving: out.append(dict(start=t,moves=[],zeros=[]))
            out[-1]["moves"].append((t,v))
        elif out:
            out[-1]["zeros"].append(t)
        moving=nonzero
    return out


def metric(series, a, b):
    origin=series.at(a)
    times=[a]+[t for t in series.times if a<t<b]+[b]
    relative=[series.relative(t,origin) for t in times]
    tail_start=max(a,b-1.)
    tail=[series.at(tail_start)]+[series.at(t) for t in times if t>tail_start]
    spans=[max(r[k] for r in tail)-min(r[k] for r in tail) for k in (1,2,4)]
    return dict(end=relative[-1], excursion=max(math.hypot(r[0],r[1]) for r in relative),
                path=sum(math.hypot(q[0]-p[0],q[1]-p[1]) for p,q in zip(relative,relative[1:])),
                spans=spans, tail_duration=b-tail_start,
                zspan=max(series.at(t)[3] for t in times)-min(series.at(t)[3] for t in times))


def trajectory_error(g, o, a, b):
    go,oo=g.at(a),o.at(a)
    count=max(1,math.ceil((b-a)/.02))
    values=[]
    for i in range(count+1):
        t=a+(b-a)*i/count
        gr,orr=g.relative(t,go),o.relative(t,oo)
        values.append((math.hypot(gr[0]-orr[0],gr[1]-orr[1]),abs(gr[2]-orr[2])))
    return max(v[0] for v in values),max(v[1] for v in values)


def integrate_commands(commands, a, b):
    # Zero-order hold using callback receipt time, not controller execution time.
    active=(0.,)*6; previous=a; x=y=heading=0.
    def advance(end, v):
        nonlocal x,y,heading,previous
        dt=end-previous; vx,vy,wz=v[0],v[1],v[5]
        if abs(wz)<1e-12:
            dx,dy=vx*dt,vy*dt
        else:
            h=wz*dt
            dx=(vx*math.sin(h)+vy*(math.cos(h)-1))/wz
            dy=(vx*(1-math.cos(h))+vy*math.sin(h))/wz
        c,s=math.cos(heading),math.sin(heading)
        x+=c*dx-s*dy; y+=s*dx+c*dy; heading+=wz*dt; previous=end
    for t,v in commands:
        if t<=a: active=v; continue
        if t>b: break
        advance(t,active); active=v
    advance(b,active)
    return x,y,heading


def summarize(checkpoint, data, commands, reason):
    plan=phases(checkpoint)
    g,o=Series(data["gazebo_center"]),Series(data["wheel_odom"])
    lines=[f"=== CHECKPOINT SUMMARY: {checkpoint} ===",f"Capture end: {reason}",
           "Runtime backend: native Fortress ign CLI; simulation only",
           "Collision width=.001 m; radius=.024 m; geometry sum=.275 m",
           f"Model={MODEL}; model-to-center offset={OFFSET} m",
           "Each source aligned to its own phase-start position and heading.",
           "Trajectory-error sampling: 50 Hz; command prediction uses receipt timestamps."]
    flags=[]
    if reason!="completed": flags.append(f"capture ended: {reason}")
    if not g.rows or not o.rows:
        return "\n".join(lines+["INCOMPLETE: missing measured pose source.","OVERALL SCREEN: REVIEW_REQUIRED"])
    a0,b0=max(g.times[0],o.times[0]),min(g.times[-1],o.times[-1])
    if b0<=a0:
        return "\n".join(lines+["INCOMPLETE: no shared coverage.","OVERALL SCREEN: REVIEW_REQUIRED"])
    lines += [f"SHARED INTERVAL: {a0:.3f} to {b0:.3f} s"]
    for name,s in (("gazebo_center",g),("wheel_odom",o)):
        lines.append(f"{name}: samples={len(s.rows)}; maximum gap={s.gap():.3f} s")
        if s.gap()>POSE_GAP_LIMIT: flags.append(f"{name} sample gap > .1 s")
    groups=windows(commands)
    if len(groups)!=len(plan): flags.append(f"observed {len(groups)} phases, expected {len(plan)}")
    baseline=groups[0]["start"]-a0 if groups else 0.
    lines.append(f"PRE-COMMAND BASELINE: {baseline:.3f} s")
    if baseline<3.: flags.append("baseline < 3 s")
    gains={1:[],-1:[]}; angles={1:[],-1:[]}
    for i,phase in enumerate(plan):
        name,vx,vy,wz=phase
        lines.append(f"\n--- {i+1}. {name}: vx={vx:+.3f}, vy={vy:+.3f}, wz={wz:+.3f} ---")
        if i>=len(groups):
            lines.append("INCOMPLETE: phase missing"); flags.append(name+": missing");continue
        group=groups[i]; local_flags=[]
        expected=command_vector(phase)
        match=all(all(abs(x-y)<1e-7 for x,y in zip(v,expected)) for _,v in group["moves"])
        if len(group["moves"])!=20 or len(group["zeros"])!=10 or not match:
            local_flags.append("command count/value mismatch")
        a=max(a0,group["start"]-.05)
        b=min(b0,groups[i+1]["start"]-.1) if i+1<len(groups) else b0
        lines.append(f"Commands={len(group['moves'])}; zeros={len(group['zeros'])}; values={'OK' if match else 'MISMATCH'}; window={a:.3f}..{b:.3f} s")
        if b<=a or b<group["start"] or not group["zeros"]:
            lines.append("INCOMPLETE: stopping/pose coverage missing");flags.append(name+": incomplete");continue
        settle=b-group["zeros"][0]
        if settle<2.5: local_flags.append("settling coverage < 2.5 s")
        gm,om=metric(g,a,b),metric(o,a,b)
        ge,oe=gm["end"],om["end"]
        maxxy,maxyaw=trajectory_error(g,o,a,b)
        endxy=math.hypot(ge[0]-oe[0],ge[1]-oe[1]); endyaw=abs(ge[2]-oe[2])
        pred=integrate_commands(commands,a,b)
        for label,m in (("Gazebo",gm),("Odom",om)):
            e=m["end"];s=m["spans"]
            lines.append(f"{label}: forward={1000*e[0]:+.6f} mm; left={1000*e[1]:+.6f} mm; yaw={math.degrees(e[2]):+.6f} deg; excursion={1000*m['excursion']:.6f} mm")
            lines.append(f"  Final-1s span: world X={1000*s[0]:.6f} mm; Y={1000*s[1]:.6f} mm; yaw={math.degrees(s[2]):.6f} deg; phase Z span={1000*m['zspan']:.6f} mm")
            if m["tail_duration"]<.999 or math.hypot(s[0],s[1])>STOP_XY or s[2]>STOP_YAW:
                local_flags.append(label+" not settled within proposed limits")
        lines.append(f"G-minus-O: forward={1000*(ge[0]-oe[0]):+.6f} mm; left={1000*(ge[1]-oe[1]):+.6f} mm; yaw={math.degrees(ge[2]-oe[2]):+.6f} deg")
        lines.append(f"Trajectory max disagreement: XY={1000*maxxy:.6f} mm; yaw={math.degrees(maxyaw):.6f} deg; settling coverage={settle:.3f} s")
        lines.append(f"Receipt-time command SE(2) prediction: forward={1000*pred[0]:+.6f} mm; left={1000*pred[1]:+.6f} mm; yaw={math.degrees(pred[2]):+.6f} deg [diagnostic, not execution ground truth]")
        if ge[2]*wz<=0 or oe[2]*wz<=0: local_flags.append("wrong/zero yaw sign")
        if abs(oe[2])<math.radians(1):
            local_flags.append("odometry yaw response too small for reliable ratio")
            ratio=None
        else:
            ratio=abs(ge[2]/oe[2])
            lines.append(f"Gazebo/odom yaw gain={ratio:.8f}; magnitude excess={(ratio-1)*100:+.5f}%")
            if abs(ratio-1)*100>YAW_SCALE_PERCENT: local_flags.append("yaw disagreement > 1%")
        if checkpoint!="combined":
            if max(gm["excursion"],om["excursion"])>YAW_CENTER_EXCURSION:
                local_flags.append("pure-yaw center excursion > 1 mm")
            if ratio is not None:
                gains[1 if wz>0 else -1].append(ratio)
                angles[1 if wz>0 else -1].append(abs(math.degrees(ge[2])))
        else:
            if endxy>COMBINED_ENDPOINT_XY: local_flags.append("endpoint XY disagreement > 1 mm")
            if maxxy>COMBINED_MAX_XY: local_flags.append("trajectory XY disagreement > 2 mm")
            if maxyaw>COMBINED_MAX_YAW: local_flags.append("trajectory yaw disagreement > .1 deg")
            main_g=ge[0] if vx else ge[1]
            main_o=oe[0] if vx else oe[1]
            if main_g*(vx or vy)<=0 or main_o*(vx or vy)<=0:
                local_flags.append("wrong/zero primary translation sign")
        lines.append("PHASE SCREEN: "+("REVIEW_REQUIRED: "+"; ".join(local_flags) if local_flags else "WITHIN_PROPOSED_LIMITS"))
        flags += [name+": "+f for f in local_flags]
    if checkpoint!="combined":
        lines.append("\n--- REPEATABILITY BY YAW SIGN ---")
        for sign,label in ((1,"CCW"),(-1,"CW")):
            values=gains[sign]
            if len(values)!=3:
                flags.append(label+": fewer than 3 usable repetitions")
                lines.append(label+": INCOMPLETE");continue
            mean=statistics.mean(values);sd=statistics.stdev(values);cv=100*sd/abs(mean)
            av=angles[sign]
            lines.append(f"{label}: n=3; mean yaw gain={mean:.8f}; sample SD={sd:.8f}; gain CV={cv:.5f}%; Gazebo angle range={min(av):.6f}..{max(av):.6f} deg")
            if cv>REPEAT_GAIN_CV_PERCENT: flags.append(label+": gain CV > 1%")
        if len(gains[1])==3 and len(gains[-1])==3:
            difference=100*abs(statistics.mean(gains[1])-statistics.mean(gains[-1]))
            lines.append(f"CCW/CW mean-gain difference={difference:.5f} percentage points")
            if difference>1.: flags.append("CCW/CW mean-gain difference > 1 percentage point")
    lines.append("\nOVERALL SCREEN: "+("REVIEW_REQUIRED" if flags else "WITHIN_PROPOSED_LIMITS"))
    if flags: lines.append("Reasons: "+" | ".join(flags))
    lines += ["Screening limits are proposed engineering targets, not previously agreed project acceptance criteria.",
              "Visual confirmation still required: correct directions/arcs, no collision or tipping, stopped after every zero phase.",
              "Do not advance to the next checkpoint until this summary and visual behavior have been reviewed.",
              "=== END CHECKPOINT SUMMARY ==="]
    return "\n".join(lines)


def publish_status(values):
    temporary=STATUS.with_suffix(".tmp")
    temporary.write_text(json.dumps(dict(values,heartbeat=time.time())))
    temporary.replace(STATUS)


def get_status():
    info=json.loads(STATUS.read_text())
    if time.time()-info["heartbeat"]>2.:
        raise RuntimeError("Observer heartbeat stale")
    return info


def acquire_lock(suffix):
    file=open(str(LOCK)+suffix,"a")
    try: fcntl.flock(file,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        file.close();raise RuntimeError(f"Another {suffix} process is active")
    return file


def capture(checkpoint):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    lockfile=acquire_lock(".capture")
    publish_status(dict(state="initializing",checkpoint=checkpoint))
    node=None;observer=None
    data={name:[] for name in ("gazebo_model","gazebo_center","wheel_odom")}
    commands=[];raw=[];mutex=threading.Lock()
    clock=[None,time.monotonic()];error=[None]
    stamp=time.strftime("%Y%m%d-%H%M%S")+"-"+uuid.uuid4().hex[:6]
    stem=Path.cwd()/f"mecanum-{checkpoint}-{stamp}"
    reason="preflight failed";session=uuid.uuid4().hex
    try:
        verify_configuration()
        topic,entity_id=identify_model()
        print(f"Fortress model: {MODEL!r}; entity ID={entity_id}; pose topic={topic}",flush=True)
        rclpy.init();node=rclpy.create_node("mecanum_checkpoint_capture")
        def add(source,t,pos,q,v=(0.,0.,0.)):
            if not all(math.isfinite(a) for a in (t,*pos,*q,*v)):
                error[0]="Nonfinite pose/velocity";return
            norm=math.sqrt(sum(a*a for a in q))
            if abs(norm-1.)>.01:
                error[0]="Invalid pose quaternion";return
            if data[source] and t<data[source][-1][0]-.05:
                error[0]="Pose timestamp moved backwards"
            data[source].append((t,*pos,angle(q)))
            raw.append((source,t,*pos,*q,*v))
        def on_pose(t,entity,pos,q):
            if entity!=entity_id:return
            off=rotate(q,OFFSET)
            with mutex:
                add("gazebo_model",t,pos,q)
                add("gazebo_center",t,tuple(a+b for a,b in zip(pos,off)),q)
        def on_clock(msg):
            t=msg.clock.sec+msg.clock.nanosec*1e-9
            if clock[0] is not None and t<clock[0]-.05:error[0]="Simulation clock reset"
            if t!=clock[0]:clock[:]=[t,time.monotonic()]
        def on_odom(msg):
            if msg.child_frame_id!="mecanum_test_center":
                error[0]=f"Unexpected odometry child frame: {msg.child_frame_id!r}";return
            t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            p,q,v=msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist
            with mutex:
                add("wheel_odom",t,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w),
                    (v.linear.x,v.linear.y,v.angular.z))
        def on_command(msg):
            if clock[0] is None:return
            v=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
            if not all(math.isfinite(x) for x in v):error[0]="Nonfinite command"
            commands.append((clock[0],v))
        node.create_subscription(Clock,"/clock",on_clock,qos_profile_sensor_data)
        node.create_subscription(Odometry,ODOM,on_odom,qos_profile_sensor_data)
        node.create_subscription(Twist,CMD,on_command,200)
        observer=PoseObserver(topic,on_pose)
        ready=None;beginning=time.monotonic();last_status=0.
        print("Waiting for live clock, native model poses, and center-frame wheel odometry...",flush=True)
        reason="capture budget exhausted"
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02);observer.check()
            now=time.monotonic()
            if error[0]:raise RuntimeError(error[0])
            with mutex:
                latest={name:list(rows[bisect.bisect_left(rows,(rows[-1][0]-1.5,)):])
                        if rows else [] for name,rows in data.items()}
            if ready is None:
                if now-beginning>30:raise RuntimeError("Timed out waiting for live measured streams")
                if clock[0] is None or not all(latest.values()):continue
                if any(abs(rows[-1][0]-clock[0])>.3 for rows in latest.values()):continue
                if node.get_publishers_info_by_topic(CMD):
                    raise RuntimeError("Another command publisher exists; stop teleop/other drivers")
                ready=clock[0]
                print(f"READY: checkpoint={checkpoint}; observer only; 120 simulation seconds available.",flush=True)
            if now-clock[1]>5:raise RuntimeError("Clock stalled for 5 wall seconds")
            if any(abs(clock[0]-rows[-1][0])>.3 for rows in latest.values()):
                raise RuntimeError("Measured pose stream stale or ahead of /clock by > .3 s")
            rows=Series(latest["gazebo_center"]);settled=False
            if rows.times[-1]-rows.times[0]>=1.:
                m=metric(rows,rows.times[-1]-1.,rows.times[-1]);s=m["spans"]
                settled=math.hypot(s[0],s[1])<=STOP_XY and s[2]<=STOP_YAW
            if now-last_status>.2:
                publish_status(dict(state="ready",checkpoint=checkpoint,session=session,
                                    sim=clock[0],ready_sim=ready,deadline=ready+120.,settled=settled))
                last_status=now
            groups=windows(commands)
            if len(groups)>=len(phases(checkpoint)) and len(groups[-1]["zeros"])>=10 and clock[0]-groups[-1]["zeros"][-1]>=4.2:
                reason="completed";break
            if clock[0]-ready>=120 or now-beginning>=600:break
    except KeyboardInterrupt:
        reason="capture interrupted; observer publishes no commands"
    except Exception as exc:
        reason=f"ABORTED: {exc}"
    finally:
        publish_status(dict(state="finished",checkpoint=checkpoint,session=session,reason=reason))
        if observer is not None:observer.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        with mutex:
            saved={name:list(rows) for name,rows in data.items()};saved_raw=list(raw)
        pose_file=Path(str(stem)+"-poses.csv")
        with pose_file.open("w",newline="") as f:
            w=csv.writer(f);w.writerow(["source","sim_time","x","y","z","qx","qy","qz","qw","vx","vy","wz"])
            w.writerows(sorted(saved_raw,key=lambda row:row[1]))
        command_file=Path(str(stem)+"-commands.csv")
        with command_file.open("w",newline="") as f:
            w=csv.writer(f);w.writerow(["sim_time","vx","vy","vz","wx","wy","wz"])
            w.writerows((t,*v) for t,v in commands)
        try:report=summarize(checkpoint,saved,commands,reason)
        except Exception as exc:report=f"=== CHECKPOINT SUMMARY: {checkpoint} ===\nINCOMPLETE: summary error: {exc}\nCapture end: {reason}\nRaw data preserved."
        report_file=Path(str(stem)+"-summary.txt")
        report_file.write_text(report+"\n")
        print(f"\nPOSE FILE: {pose_file}\nCOMMAND FILE: {command_file}\nSUMMARY FILE: {report_file}\n\n{report}",flush=True)
        lockfile.close()


def send_stop():
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init();node=rclpy.create_node("mecanum_checkpoint_stop")
    pub=node.create_publisher(Twist,CMD,10)
    try:
        time.sleep(.5)
        for _ in range(10):pub.publish(Twist());time.sleep(.1)
        print("Ten zero commands sent. Confirm the robot is stationary.",flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


def drive(checkpoint):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    lockfile=acquire_lock(".drive")
    node=None;pub=None;success=False
    try:
        initial=get_status()
        budget=6*len(phases(checkpoint))+12
        if initial.get("state")!="ready" or initial.get("checkpoint")!=checkpoint:
            raise RuntimeError("No ready capture for this checkpoint")
        if initial["deadline"]-initial["sim"]<budget:
            raise RuntimeError("Not enough capture time remains; restart capture")
        print(f"Simulation only. Running {checkpoint}: {len(phases(checkpoint))} phases.")
        print("Watch directions/arcs. Ctrl+C here aborts and sends zeros.")
        if input("Area clear and previous checkpoint reviewed? Type RUN: ")!="RUN":
            print("Cancelled; no commands published.");return
        session=initial["session"]
        rclpy.init();node=rclpy.create_node("mecanum_checkpoint_drive")
        clock=[None,time.monotonic()]
        def on_clock(msg):
            t=msg.clock.sec+msg.clock.nanosec*1e-9
            if clock[0] is not None and t<clock[0]-.05:
                raise RuntimeError("Simulation clock reset")
            if t!=clock[0]:clock[:]=[t,time.monotonic()]
        node.create_subscription(Clock,"/clock",on_clock,qos_profile_sensor_data)
        def check():
            status=get_status()
            if status.get("state")!="ready" or status.get("session")!=session or status.get("checkpoint")!=checkpoint:
                raise RuntimeError("Capture unavailable or changed")
            if time.monotonic()-clock[1]>3:raise RuntimeError("Clock stalled for 3 wall seconds")
            if clock[0] is not None and abs(status["sim"]-clock[0])>.5:
                raise RuntimeError("Driver/observer clocks disagree")
            if len(node.get_publishers_info_by_topic(CMD))>1:
                raise RuntimeError("Another command publisher appeared")
            if clock[0] is not None and status["deadline"]-clock[0]<5:
                raise RuntimeError("Capture deadline too close")
            return status
        def wait(target):
            while clock[0] is None or clock[0]<target:
                rclpy.spin_once(node,timeout_sec=.01);check()
        deadline=time.monotonic()+10
        while clock[0] is None or not {"mecanum_controller","mecanum_checkpoint_capture"}.issubset(
                {info.node_name for info in node.get_subscriptions_info_by_topic(CMD)}):
            rclpy.spin_once(node,timeout_sec=.05)
            status=get_status()
            if status.get("session")!=session or status.get("state")!="ready":
                raise RuntimeError("Capture changed while waiting")
            if time.monotonic()>deadline:raise RuntimeError("Clock/controller/capture subscriptions unavailable")
        if node.get_publishers_info_by_topic(CMD):raise RuntimeError("Existing command publisher; refusing to compete")
        pub=node.create_publisher(Twist,CMD,10)
        wait(clock[0]+1.)
        status=check()
        if status["deadline"]-clock[0]<budget:raise RuntimeError("Insufficient time after arming")
        wait(max(clock[0],status["ready_sim"])+3.)
        if not check()["settled"]:raise RuntimeError("Robot not stationary before starting")
        for index,phase in enumerate(phases(checkpoint),1):
            name,vx,vy,wz=phase
            print(f"{index}: {name}, vx={vx:+.3f}, vy={vy:+.3f}, wz={wz:+.3f}",flush=True)
            msg=Twist();msg.linear.x=vx;msg.linear.y=vy;msg.angular.z=wz
            t0=clock[0]
            for j in range(30):
                target=t0+.1*j;wait(target);check()
                if clock[0]-target>.075:
                    raise RuntimeError("Missed 10 Hz schedule by > 75 simulation ms")
                pub.publish(msg if j<20 else Twist())
            wait(t0+6.)
            if not check()["settled"]:
                raise RuntimeError(f"{name}: failed final-one-second settling gate")
        success=True
        print("Sequence complete. Wait for the observer's checkpoint summary.",flush=True)
    finally:
        if pub is not None and not success:
            print("ABORT: sending ten zeros; do not resume this partial sequence.",flush=True)
            for _ in range(10):pub.publish(Twist());time.sleep(.1)
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        lockfile.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode",choices=("capture","drive","stop"))
    parser.add_argument("checkpoint",choices=("yaw_repeat","yaw_second","combined"),nargs="?")
    args=parser.parse_args()
    if args.mode!="stop" and args.checkpoint is None:parser.error("capture/drive requires a checkpoint")
    if args.mode=="stop":send_stop()
    elif args.mode=="capture":capture(args.checkpoint)
    else:drive(args.checkpoint)


if __name__=="__main__":
    try:main()
    except KeyboardInterrupt:print("Interrupted.")
    except Exception as exc:raise SystemExit(f"ERROR: {exc}")
