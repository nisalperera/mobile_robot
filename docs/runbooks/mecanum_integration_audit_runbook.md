# Read-only mecanum integration audit: code and run instructions

## Purpose

Collect evidence for the next integration step without changing the validated robot configuration. The utility prints three reports that you can paste into the conversation. No upload is required.

- Part A: runtime nodes, topic publishers/subscribers, TF-edge ownership, odometry/sensor frame IDs, effective parameters, runtime robot description, controller/interface state, and loaded simulator libraries.
- Part B: source inventory, intended mapping-launch candidates, source/installed-share comparisons, and line-numbered excerpts.
- Part C: unresolved integration decisions and observations requiring review.

This is evidence collection, not an integration patch or acceptance test. It does not start mapping, command motion, publish TF/odometry, set parameters, switch controllers, expand Xacro, or execute launch files.

## Preserve the working baseline

Your verified simulation is Fortress: ignition-gazebo6 6.18.0, ignition-transport11, ignition-msgs8. The original project target was Harmonic, but do not migrate it during this audit.

Preserve:

```text
Isolated launch:             mecanum_bringup_test.launch.py
Explicit collision proxy:    wheel_collision_width:=0.001
Wheel radius:                0.024 m
Controller geometry sum:     0.275 m
Controller base frame:       mecanum_test_center
Controller TF publication:   enable_odom_tf=false
Command topic:               /mecanum_controller/reference_unstamped
Wheel odometry:              /mecanum_controller/odometry
```

Keep the existing friction/frame corrections. Do not modify shared Xacro, mapping launch, footprint, package repositories, or installed Gazebo packages. No new dependencies are required.

## 1. Leave the isolated simulation running at rest

Keep exactly one working isolated simulation running. Stop previous validation drivers/captures, teleoperation, and other command publishers. Do not launch mapping for this audit. Do not run any motion sequence.

If the isolated launch is already running, do not restart it or run another copy. If it is not running, use a separately sourced laptop-container terminal:

```bash
# Execution location: laptop container, launch terminal
cd /home/nisalperera/projects
source /opt/ros/humble/setup.bash
source /home/nisalperera/projects/install/setup.bash
source /home/nisalperera/projects/install/mecanum_four_wheel_hardware/local_setup.bash
ros2 launch \
  /home/nisalperera/projects/mobile_robot/launch/mecanum_bringup_test.launch.py \
  wheel_collision_width:=0.001
```

## 2. Prepare the audit terminal

```bash
# Execution location: laptop container, audit terminal
# Working directory: /home/nisalperera/projects
cd /home/nisalperera/projects
source /opt/ros/humble/setup.bash
source /home/nisalperera/projects/install/setup.bash
source /home/nisalperera/projects/install/mecanum_four_wheel_hardware/local_setup.bash
```

Use the laptop container, not the Fedora host or Jetson. The utility uses system Python, existing ROS Humble Python messages, PyYAML, and existing `ros2` CLI commands. It does not invoke `gz` or require Gazebo Python bindings.

## 3. Create the complete utility

Copy the entire following block into the audit terminal. It creates only the new standalone audit utility and backs up an existing copy of that utility. The previous motion-test scripts are untouched.

```bash
# Execution location: laptop container
cd /home/nisalperera/projects
mkdir -p mobile_robot/tools
if [ -e mobile_robot/tools/mecanum_integration_audit.py ]; then
    cp -p mobile_robot/tools/mecanum_integration_audit.py \
      "mobile_robot/tools/mecanum_integration_audit.py.before-$(date +%Y%m%d-%H%M%S)"
fi
cat > mobile_robot/tools/mecanum_integration_audit.py <<'PY'
#!/usr/bin/env python3
"""Read-only ROS/source audit. No command, TF, odometry, or clock publishers."""
import argparse
from collections import defaultdict, deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

ROOT=Path('/home/nisalperera/projects')
SKIP={'.git','build','install','log','logs','__pycache__','.venv','venv','tools'}
MATCH=re.compile(r'cmd_vel|reference_unstamped|odom|base_frame|base_link|base_footprint|mecanum_test_center|enable_odom_tf|publish_tf|timeout|watchdog|friction|fdir1|expressed_in|collision_width|mu2|mecanum|diff.?drive|holonomic|motion_model|min_vel_y|max_vel_y|vy_samples|footprint|robot_radius|slam_toolbox|robot_localization|velocity_smoother|twist_mux|IncludeLaunchDescription|Node\(|get_package_share_directory|create_publisher|create_subscription',re.I)

def run(args,timeout=12):
    try:
        p=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
        return dict(code=p.returncode,stdout=p.stdout.strip(),stderr=p.stderr.strip())
    except Exception as e:return dict(code=-1,stdout='',stderr=str(e))

def flatten(value,prefix=''):
    out={}
    if isinstance(value,dict):
        for k,v in value.items():out.update(flatten(v,f'{prefix}.{k}' if prefix else str(k)))
    else:out[prefix]=value
    return out

def short(value,n=220):
    s=str(value).replace('\n',' ')
    return s if len(s)<=n else s[:n]+' [TRUNCATED]'

def fullname(info):return info.node_namespace.rstrip('/')+'/'+info.node_name

def gid(value):
    try:return bytes(value).hex()
    except Exception:return 'unavailable'

def path_between(edges,start,end):
    graph=defaultdict(set)
    for a,b in edges:graph[a].add(b);graph[b].add(a)
    queue=deque([(start,[start])]);seen={start}
    while queue:
        here,path=queue.popleft()
        if here==end:return path
        for other in sorted(graph[here]):
            if other not in seen:seen.add(other);queue.append((other,path+[other]))
    return None

def libraries():
    out=[]
    pattern=re.compile(r'lib(?:ignition-(?:gazebo|transport|msgs)|gz-(?:sim|transport|msgs))\d+\.so')
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if proc.stat().st_uid!=os.getuid():continue
            maps=(proc/'maps').read_text()
            libs=sorted({line.split()[-1] for line in maps.splitlines() if pattern.search(line)})
            cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            if any(re.search(r'lib(?:ignition-gazebo|gz-sim)\d+\.so',lib) for lib in libs) or ('parameter_bridge' in cmd and libs):
                out.append(dict(pid=int(proc.name),command=cmd,libraries=libs))
        except OSError:pass
    return out

def observe(seconds):
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan,Imu
    from geometry_msgs.msg import Twist,TwistStamped
    rclpy.init();node=rclpy.create_node('mecanum_integration_audit')
    edges={};samples={};clock=[];subs=[];selected=set();limits=[]
    def tf_callback(kind):
        def callback(msg,info):
            source=gid(getattr(info,'publisher_gid',[]))
            for t in msg.transforms:
                key=(kind,t.header.frame_id,t.child_frame_id)
                row=edges.setdefault(key,dict(kind=kind,parent=key[1],child=key[2],count=0,gids=set()))
                row['count']+=1;row['gids'].add(source)
                row['stamp']=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
                p,q=t.transform.translation,t.transform.rotation
                row['xyz']=[p.x,p.y,p.z];row['quaternion']=[q.x,q.y,q.z,q.w]
        return callback
    subs.append(node.create_subscription(TFMessage,'/tf',tf_callback('dynamic'),qos_profile_sensor_data))
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs.append(node.create_subscription(TFMessage,'/tf_static',tf_callback('static'),qos))
    def on_clock(msg):clock.append(msg.clock.sec+msg.clock.nanosec*1e-9)
    subs.append(node.create_subscription(Clock,'/clock',on_clock,qos_profile_sensor_data))
    types={'nav_msgs/msg/Odometry':('odom',Odometry),'sensor_msgs/msg/LaserScan':('scan',LaserScan),
           'sensor_msgs/msg/Imu':('imu',Imu),'geometry_msgs/msg/Twist':('command',Twist),
           'geometry_msgs/msg/TwistStamped':('command',TwistStamped)}
    def sample_callback(topic,kind):
        def callback(msg):
            row=samples.setdefault(topic,dict(kind=kind,count=0,frames=set(),children=set(),nonzero_commands=0))
            row['count']+=1
            if hasattr(msg,'header'):
                row['frames'].add(msg.header.frame_id)
                row['last_stamp']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if kind=='odom':row['children'].add(msg.child_frame_id)
            if kind=='command':
                v=msg.twist if hasattr(msg,'twist') else msg
                if any(abs(a)>1e-8 for a in (v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z)):
                    row['nonzero_commands']+=1
        return callback
    began=time.monotonic();last=-10
    try:
        while time.monotonic()-began<seconds:
            rclpy.spin_once(node,timeout_sec=.03)
            if time.monotonic()-last<1:continue
            last=time.monotonic()
            for topic,advertised in sorted(node.get_topic_names_and_types()):
                if topic in selected or len(advertised)!=1 or advertised[0] not in types:continue
                if len(selected)>=40:limits.append(topic);continue
                kind,cls=types[advertised[0]];selected.add(topic)
                subs.append(node.create_subscription(cls,topic,sample_callback(topic,kind),qos_profile_sensor_data))
        topics={}
        for topic,advertised in node.get_topic_names_and_types():
            if topic not in selected and topic not in ('/tf','/tf_static','/clock'):continue
            record={'types':advertised}
            for direction,query in [('publishers',node.get_publishers_info_by_topic),('subscribers',node.get_subscriptions_info_by_topic)]:
                record[direction]=[dict(node=fullname(e),type=e.topic_type,gid=gid(e.endpoint_gid),
                                       reliability=str(e.qos_profile.reliability),durability=str(e.qos_profile.durability)) for e in query(topic)]
            topics[topic]=record
        nodes=sorted(ns.rstrip('/')+'/'+name for name,ns in node.get_node_names_and_namespaces() if name!='mecanum_integration_audit')
        for row in edges.values():row['gids']=sorted(row['gids'])
        for row in samples.values():
            row['frames']=sorted(row['frames']);row['children']=sorted(row['children'])
        return dict(nodes=nodes,topics=topics,edges=list(edges.values()),samples=samples,
                    clock=clock,wall_seconds=time.monotonic()-began,observation_caps=sorted(set(limits)))
    finally:node.destroy_node();rclpy.shutdown()

def source_files():
    packages=[]
    for directory,dirs,files in os.walk(ROOT,followlinks=False):
        rel=Path(directory).relative_to(ROOT)
        dirs[:]=sorted(d for d in dirs if d not in SKIP and not d.startswith('.'))
        if len(rel.parts)>=4:dirs[:]=[]
        if 'package.xml' not in files:continue
        try:name=ET.parse(Path(directory)/'package.xml').getroot().findtext('name')
        except Exception:continue
        if name in ('mobile_robot','mecanum_four_wheel_hardware'):
            packages.append((name,Path(directory)));dirs[:]=[]
    entries=[];warnings=[]
    for name,root in packages:
        share=None
        try:
            from ament_index_python.packages import get_package_share_directory
            share=Path(get_package_share_directory(name))
        except Exception:pass
        count=0
        for directory,dirs,files in os.walk(root,followlinks=False):
            rel=Path(directory).relative_to(root)
            dirs[:]=sorted(d for d in dirs if d not in SKIP and not d.startswith('.'))
            if len(rel.parts)>=8:dirs[:]=[]
            for filename in sorted(files):
                p=Path(directory)/filename
                launch='launch' in rel.parts and p.suffix=='.py' and not any(s in filename for s in ('four_directions','pending_checkpoints','integration_audit'))
                if not (launch or p.suffix in ('.yaml','.yml','.urdf','.xacro','.sdf','.world','.rviz','.cpp','.cc','.hpp','.h','.xml') or filename=='CMakeLists.txt'):continue
                count+=1
                if count>160:continue
                if p.stat().st_size>1024*1024:warnings.append(f'File >1 MiB omitted: {p}');continue
                raw=p.read_bytes();text=raw.decode(errors='replace');lines=text.splitlines()
                matches=[i+1 for i,line in enumerate(lines) if MATCH.search(line)]
                if not launch and not matches:continue
                installed=share/p.relative_to(root) if share else None
                comparison='NO_INSTALLED_COUNTERPART'
                if installed and installed.is_file():comparison='MATCH' if installed.read_bytes()==raw else 'DIFFERENT'
                entries.append(dict(path=str(p),package=name,launch=launch,lines=len(lines),
                                    sha256=hashlib.sha256(raw).hexdigest(),comparison=comparison,
                                    installed=str(installed) if installed and installed.is_file() else None,text=text,matches=matches))
        if count>160:warnings.append(f'Candidate cap reached in {root}; eligible={count}, inspected first 160')
    if not packages:warnings.append('Target source packages not discovered within four levels')
    return entries,warnings

def robot_evidence(params):
    text=params.get('/robot_state_publisher',{}).get('values',{}).get('robot_description')
    if not text:return {'error':'Runtime robot_description unavailable'}
    try:
        root=ET.fromstring(text);joints=[];wheels=[];friction=[]
        for j in root.findall('joint'):
            parent,child=j.find('parent'),j.find('child')
            if parent is None or child is None:continue
            o=j.find('origin')
            joints.append(dict(name=j.get('name'),type=j.get('type'),parent=parent.get('link'),child=child.get('link'),
                               xyz=o.get('xyz','0 0 0') if o is not None else '0 0 0',rpy=o.get('rpy','0 0 0') if o is not None else '0 0 0'))
        for link in root.findall('link'):
            if 'wheel' in link.get('name',''):
                wheels.append(dict(name=link.get('name'),cylinders=[dict(c.attrib) for c in link.findall('collision/geometry/cylinder')]))
        for g in root.findall('gazebo'):
            if 'wheel' in g.get('reference',''):
                friction.append(dict(link=g.get('reference'),xml=ET.tostring(g,encoding='unicode')))
        return dict(roots=sorted({l.get('name') for l in root.findall('link')}-{j['child'] for j in joints}),
                    joints=joints,wheels=wheels,friction=friction,
                    hardware_plugins=[p.text for p in root.findall('ros2_control/hardware/plugin')],
                    gazebo_plugins=[dict(p.attrib) for p in root.findall('.//gazebo/plugin')])
    except Exception as e:return dict(error=str(e))

def reports(data,params,robot,processes,control,entries,source_warnings,mapping,max_source):
    a=['=== INTEGRATION AUDIT PART A: RUNTIME ===','Read-only observation; no mapping launch started.']
    for p in processes:
        a.append(f"PROCESS {p['pid']}: {short(p['command'])}");a.extend('  '+s for s in p['libraries'])
    if not processes:a.append('Loaded simulator libraries: not available; do not infer runtime from project target')
    a.append('NODES: '+repr(data['nodes']))
    a.append(f"CLOCK: n={len(data['clock'])}; range={data['clock'][0] if data['clock'] else None}..{data['clock'][-1] if data['clock'] else None}; observation={data['wall_seconds']:.2f} wall s")
    owners=defaultdict(set)
    for topic,row in sorted(data['topics'].items()):
        a.append(f"TOPIC {topic}: {row['types']}")
        for direction in ('publishers','subscribers'):
            if not row[direction]:a.append('  '+direction+': none observed')
            for e in row[direction]:
                label=' AUDITOR' if e['node']=='/mecanum_integration_audit' else ''
                a.append(f"  {direction[:-1]} {e['node']}{label}; type={e['type']}; GID={e['gid'][:20]}; QoS={e['reliability']}/{e['durability']}")
                if topic in ('/tf','/tf_static') and direction=='publishers':owners[e['gid']].add(e['node'])
    for topic,row in sorted(data['samples'].items()):a.append('SAMPLE '+topic+': '+json.dumps(row))
    for e in sorted(data['edges'],key=lambda r:(r['kind'],r['parent'],r['child'])):
        names=sorted({owner for g in e['gids'] for owner in owners.get(g,{'UNRESOLVED_GID:'+g[:20]})})
        a.append(f"TF {e['kind']} {e['parent']} -> {e['child']}: owners={names}; n={e['count']}; stamp={e['stamp']}; xyz={e['xyz']}; q={e['quaternion']}")
    for node,row in params.items():
        a.append('PARAMETERS '+node)
        if 'error' in row:a.append('  UNAVAILABLE: '+row['error']);continue
        selected=[(k,v) for k,v in flatten(row['values']).items() if k!='robot_description' and MATCH.search(k+' '+str(v)[:200])]
        for k,v in selected[:65]:a.append('  '+k+' = '+short(v))
        if len(selected)>65:a.append('  PARAMETER DISPLAY TRUNCATED; full values retained in local evidence JSON')
    if 'error' in robot:a.append('RUNTIME URDF UNAVAILABLE: '+robot['error'])
    else:
        a.append('RUNTIME URDF ROOTS: '+repr(robot['roots']))
        targets={'base_footprint','base_link','chassis','mecanum_test_center','laser_frame','imu_link'}
        for row in data['samples'].values():targets.update(row['frames'])
        for j in robot['joints']:
            if j['child'] in targets:a.append('BODY/SENSOR JOINT '+json.dumps(j))
        for w in robot['wheels']:a.append('WHEEL '+json.dumps(w))
        for f in robot['friction']:a.append('FRICTION '+json.dumps(f))
        a.append('HARDWARE PLUGINS: '+repr(robot['hardware_plugins']))
        a.append('GAZEBO PLUGINS: '+repr(robot['gazebo_plugins']))
    for label,row in control.items():
        a.append(f"CONTROL QUERY {label}: exit={row['code']}")
        a.extend('  '+s for s in (row['stdout'] or row['stderr']).splitlines()[:45])
    a.append('=== END INTEGRATION AUDIT PART A ===')
    b=['=== INTEGRATION AUDIT PART B: SOURCES ===','Static source only; no launch import, execution, or Xacro expansion.']
    candidates=[e['path'] for e in entries if e['launch'] and re.search(r'map|slam|nav|bringup',Path(e['path']).name,re.I) and 'test' not in Path(e['path']).name]
    b.append('MAPPING CANDIDATES: '+repr(candidates));b.append('INTENDED MAPPING LAUNCH: '+(str(mapping) if mapping else 'UNCONFIRMED'))
    b.extend('NOTICE: '+w for w in source_warnings)
    entries=sorted(entries,key=lambda e:(0 if mapping and Path(e['path']).resolve()==mapping else 1 if Path(e['path']).name=='mecanum_bringup_test.launch.py' else 2 if e['path'] in candidates else 3 if e['launch'] else 4,e['path']))
    for e in entries:
        b.append(f"FILE {e['path']}: lines={e['lines']}; sha256={e['sha256']}; installed={e['comparison']}")
        if e['comparison']=='DIFFERENT':b.append('  INSTALLED: '+str(e['installed']))
    remaining=max_source
    for e in entries:
        if remaining<=0:break
        lines=e['text'].splitlines();wanted=set()
        for i in e['matches']:wanted.update(range(max(1,i-2),min(len(lines),i+2)+1))
        if e['launch'] and len(lines)<=70:wanted.update(range(1,len(lines)+1))
        if not wanted:continue
        selected=sorted(wanted)[:min(100,remaining)];remaining-=len(selected)
        b.append('EXCERPT '+e['path']);previous=None
        for i in selected:
            if previous is not None and i>previous+1:b.append('  ...')
            b.append(f'L{i:04d}: {lines[i-1]}');previous=i
        if len(selected)<len(wanted):b.append('SOURCE_DETAIL_TRUNCATED: request exact --show ranges before editing')
    if remaining<=0:b.append('GLOBAL EXCERPT LIMIT REACHED: additional files/ranges remain uninspected in this report')
    b.append('Includes, launch arguments and YAML selections were NOT evaluated; source matches do not prove effective runtime configuration.')
    b.append('=== END INTEGRATION AUDIT PART B ===')
    c=['=== INTEGRATION AUDIT PART C: REVIEW GATE ===']
    edges={(e['parent'],e['child']) for e in data['edges']};parents=defaultdict(set)
    for p,ch in edges:parents[ch].add(p)
    for frame in ('mecanum_test_center','base_footprint','base_link'):
        c.append(f"Observed odom -> {frame} connectivity: {path_between(edges,'odom',frame) or 'NOT OBSERVED'}")
    c.append('Connectivity is topology only, not a timestamped tf2 lookup test.')
    for child,values in sorted(parents.items()):
        if len(values)>1:c.append(f'REVIEW: multiple observed parents for {child}: {sorted(values)}')
    for e in data['edges']:
        if len(e['gids'])>1:c.append(f"REVIEW: multiple publisher GIDs for {e['parent']} -> {e['child']}")
    p=params.get('/mecanum_controller',{}).get('values',{})
    c.append('Controller enable_odom_tf: '+repr(p.get('enable_odom_tf','UNAVAILABLE')))
    if p.get('enable_odom_tf') is False:c.append('Missing odom-to-base TF can be intentional in the isolated setup. Do not enable it to make this report look complete.')
    for j in robot.get('joints',[]):
        if j['child']=='mecanum_test_center':c.append('Existing URDF center parent: '+j['parent']+'; review TF parentage before adding odom -> center')
    if not data['clock'] or data['clock'][-1]<=data['clock'][0]:c.append('REVIEW: progressing simulation clock not established')
    if '/mecanum_controller/odometry' not in data['samples']:c.append('REVIEW: no mecanum wheel-odometry samples observed')
    for topic,row in data['samples'].items():
        if row['nonzero_commands']:c.append('REVIEW: nonzero commands observed on '+topic)
    if data['topics'].get('/mecanum_controller/reference_unstamped',{}).get('publishers'):c.append('REVIEW: active controller command publishers are present')
    if not mapping:c.append('REVIEW: intended mapping launch identity remains unconfirmed; select from source evidence')
    if data['observation_caps']:c.append('REVIEW: subscription cap excluded '+repr(data['observation_caps']))
    c.extend(['DECISION NEEDED: one odom-to-navigation-base TF authority, separate from map-to-odom authority.',
              'DECISION NEEDED: navigation base reference point and correct pose/twist transformation. Renaming child_frame_id is not a point transformation.',
              'DECISION NEEDED: actual command route, stamped/unstamped handling, timeout owner, and holonomic settings.',
              'DECISION NEEDED: minimal location for validated test-only corrections; preserve physical/navigation footprint.',
              'AUDIT STATUS: EVIDENCE_CAPTURED_FOR_REVIEW, NOT INTEGRATION_APPROVED',
              'No robot configuration changed, no commands published, no mapping launch started.',
              '=== END INTEGRATION AUDIT PART C ==='])
    return ['\n'.join(part) for part in (a,b,c)]

def main():
    global ROOT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=20.)
    parser.add_argument('--projects',type=Path,default=ROOT)
    parser.add_argument('--mapping-launch',type=Path)
    parser.add_argument('--source-lines',type=int,default=320)
    parser.add_argument('--show',type=Path)
    parser.add_argument('--start',type=int,default=1)
    parser.add_argument('--end',type=int,default=200)
    args=parser.parse_args();ROOT=args.projects.resolve()
    if args.show:
        p=(ROOT/args.show).resolve() if not args.show.is_absolute() else args.show.resolve()
        if not p.is_relative_to(ROOT) or p.stat().st_size>1024*1024:raise RuntimeError('Source must resolve inside projects and be <=1 MiB')
        if args.start<1 or args.end<args.start or args.end-args.start>=500:raise RuntimeError('Request 1..500 source lines')
        lines=p.read_text().splitlines()
        print(f'=== SOURCE RANGE {p}; sha256={hashlib.sha256(p.read_bytes()).hexdigest()} ===')
        for i in range(args.start,min(args.end,len(lines))+1):print(f'L{i:04d}: {lines[i-1]}')
        print('=== END SOURCE RANGE ===');return
    if not 5<=args.seconds<=60:parser.error('--seconds must be 5..60 wall seconds')
    if not 100<=args.source_lines<=1000:parser.error('--source-lines must be 100..1000')
    mapping=args.mapping_launch
    if mapping:
        mapping=(ROOT/mapping).resolve() if not mapping.is_absolute() else mapping.resolve()
        if not mapping.is_relative_to(ROOT) or not mapping.is_file() or mapping.stat().st_size>1024*1024:raise RuntimeError('Mapping launch must be a source file <=1 MiB inside projects')
    print(f'Read-only observation for {args.seconds:g} wall seconds; no motion or driver step.',flush=True)
    processes=libraries();data=observe(args.seconds)
    print('Reading effective parameters and controller state...',flush=True)
    import yaml
    names=['/mecanum_controller','/controller_manager','/robot_state_publisher']
    names+=sorted(n for n in set(data['nodes']) if re.search(r'slam|amcl|ekf|localization|velocity_smoother|twist_mux|controller_server|costmap',n))[:7]
    params={}
    for name in dict.fromkeys(names):
        if name not in data['nodes']:continue
        row=run(['ros2','param','dump',name])
        try:
            if row['code']:raise RuntimeError(row['stderr'] or row['stdout'])
            document=yaml.safe_load(row['stdout'])
            if not isinstance(document,dict) or len(document)!=1:raise ValueError('Expected one parameter node')
            params[name]={'values':next(iter(document.values()))['ros__parameters']}
        except Exception as e:params[name]={'error':str(e)}
    robot=robot_evidence(params)
    control={label:run(['ros2','control',verb,'-c','/controller_manager']) for label,verb in [('controllers','list_controllers'),('interfaces','list_hardware_interfaces')]}
    print('Inspecting source candidates without executing them...',flush=True)
    entries,warnings=source_files()
    if mapping and not any(Path(e['path']).resolve()==mapping for e in entries):
        raw=mapping.read_bytes();text=raw.decode(errors='replace')
        entries.append(dict(path=str(mapping),package='explicit',launch=True,lines=len(text.splitlines()),
                            sha256=hashlib.sha256(raw).hexdigest(),comparison='NOT_COMPARED',installed=None,
                            text=text,matches=[i+1 for i,line in enumerate(text.splitlines()) if MATCH.search(line)]))
    parts=reports(data,params,robot,processes,control,entries,warnings,mapping,args.source_lines)
    stem=ROOT/('mecanum-integration-audit-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    for suffix,part in zip(('A-runtime','B-sources','C-gate'),parts):
        p=Path(str(stem)+'-'+suffix+'.txt');p.write_text(part+'\n');print(f'\nREPORT FILE: {p}\n{part}',flush=True)
    p=Path(str(stem)+'-evidence.json')
    p.write_text(json.dumps(dict(runtime=data,parameters=params,robot=robot,processes=processes,control=control,sources=entries,source_warnings=warnings),indent=2))
    print(f'\nLocal evidence only: {p}\nPaste report text A/B/C. No upload required. Do not change mapping yet.',flush=True)

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:raise SystemExit('Audit interrupted. No motion commanded. Confirm the robot remains stationary.')
    except Exception as e:raise SystemExit(f'AUDIT ERROR: {type(e).__name__}: {e}')
PY
/usr/bin/python3 -m py_compile mobile_robot/tools/mecanum_integration_audit.py
```

A successful syntax check prints nothing. Do not paste the Markdown fence markers. If the shell remains at a `>` continuation prompt, finish the here-document with `PY` on its own line.

## 4. Run one read-only audit

```bash
# Execution location: laptop container, audit terminal
# Working directory: /home/nisalperera/projects
cd /home/nisalperera/projects
/usr/bin/python3 -u mobile_robot/tools/mecanum_integration_audit.py --seconds 20
```

There is no READY/drive handshake. Leave the robot stationary throughout.

The script observes for 20 wall seconds, then performs time-bounded parameter/control queries and source inspection. The whole audit can take longer than 20 seconds, especially if a service times out. Wait for Parts A, B, and C.

Do not edit files, restart nodes, or change controllers during the audit. The observation window and subsequent queries are not an atomic snapshot of a changing system.

The only controller commands used are read-only:

```text
ros2 control list_controllers -c /controller_manager
ros2 control list_hardware_interfaces -c /controller_manager
```

If a command or query fails, its error is reported. Do not install packages or activate a controller merely to remove an audit warning.

## 5. Paste these results

Paste each complete report, separately if necessary:

1. `=== INTEGRATION AUDIT PART A: RUNTIME ===` through its END marker.
2. `=== INTEGRATION AUDIT PART B: SOURCES ===` through its END marker.
3. `=== INTEGRATION AUDIT PART C: REVIEW GATE ===` through its END marker.

Add:

```text
Robot remained stationary throughout audit: yes/no
Intended mapping launch: <actual path, or "not sure; use candidate list">
```

No raw CSV/JSON or script upload is needed. The local evidence JSON is for troubleshooting only.

## How to interpret the reports

### Runtime versus source evidence

Part A describes the observed running system. Part B reads source text without importing or executing it. A keyword in a file is not proof that its configuration is currently active. Launch arguments, includes, substitutions, and selected YAML files are not evaluated.

The utility searches source packages named `mobile_robot` and `mecanum_four_wheel_hardware` within four directory levels of the projects directory. Build/install/log/git/virtual-environment/tools trees are excluded from source discovery. It separately compares same-relative-path installed-share files when the package index is available. A source/installed difference is an observation, not permission to rebuild or overwrite anything.

The source search is bounded: up to 160 eligible candidates per package, files up to 1 MiB, and a default 320 displayed excerpt lines. Caps and truncation are reported. Missing matches do not prove that a behavior does not exist elsewhere.

The script does not guess your intended mapping-launch filename. Review its candidate list and identify the real entry point before preparing integration changes.

### TF ownership

The utility records individual TF edges and publisher GIDs from received messages. When possible it resolves each GID to a node. An `UNRESOLVED_GID` means ownership was not established. Several nodes publishing `/tf` is not automatically a conflict; ownership concerns individual edges.

Multiple observed parents or publisher GIDs are review items, not automatically diagnosed root causes. Listed TF connectivity is graph topology only, not proof that a timestamped tf2 lookup succeeds or that every edge is fresh.

Normal ROS node discovery, rosout and parameter infrastructure may appear. The script creates no application publisher for commands, TF, odometry, or clock. Its own observation subscriptions are marked `AUDITOR`.

### Expected isolated-launch omissions

- `enable_odom_tf=false` is intentional. Missing observed `odom -> base` TF can be expected. Do not enable it solely to make this audit look complete.
- `map -> odom` can be absent because mapping/localization is not running. Do not add a static identity transform to hide that omission.
- ROS scan/IMU streams may be absent if the isolated launch does not bridge those sensors. A sensor defined in URDF is not proof of active ROS data.
- No command publisher while idle can be correct. Do not start a dummy command publisher.
- The 1 mm runtime collision proxy can differ from the 25 mm shared wheel definition. The audit should locate the test-only injection point, not force those files to match.

## Important integration decision: physical reference point and TF parentage

Your previously posted robot description has:

```text
base_footprint -> base_link -> chassis -> mecanum_test_center
```

The known fixed offsets imply a center 0.05 m ahead of `base_link`, and `(0.05, 0, 0.024)` m relative to `base_footprint`. The audit prints the actual joint origins so these can be verified again.

Do not simply add `odom -> mecanum_test_center` while the published body tree already gives that frame another parent. Resolve the base-frame and TF authority design first.

Changing `child_frame_id` alone does not transform an odometry pose or twist to a different point on a rotating robot. The future integration patch must preserve the physical meaning of pose and twist, select one odom-to-navigation-base authority, and keep map-to-odom authority separate.

Also confirm the actual command route, stamped/unstamped interface, timeout owner, lateral motion limits and navigation motion model. Do not copy the thin contact proxy into the physical robot footprint or costmap footprint.

## Additional source ranges, only when needed

Truncated excerpts are not sufficient evidence for arbitrary edits. The same utility can print exact numbered source ranges without ROS observation or execution.

For the known isolated launch:

```bash
# Execution location: laptop container
cd /home/nisalperera/projects
/usr/bin/python3 mobile_robot/tools/mecanum_integration_audit.py \
  --show mobile_robot/launch/mecanum_bringup_test.launch.py \
  --start 1 --end 200
```

Other ranges should use actual paths and line numbers from the report. Up to 500 lines can be requested at once. Paths must resolve inside the projects directory.

An optional `--mapping-launch` argument accepts an existing source path from the candidate list. It prioritizes that file but does not execute it. Do not invent a filename; the default audit is the first step.

## Saved reports and replay

The utility prints unique filenames and saves reports in the projects directory:

```text
mecanum-integration-audit-<timestamp>-<session>-A-runtime.txt
mecanum-integration-audit-<timestamp>-<session>-B-sources.txt
mecanum-integration-audit-<timestamp>-<session>-C-gate.txt
mecanum-integration-audit-<timestamp>-<session>-evidence.json
```

To print the latest Part A again:

```bash
# Execution location: laptop container
cd /home/nisalperera/projects
/usr/bin/python3 - <<'PY'
from pathlib import Path
files = list(Path('.').glob('mecanum-integration-audit-*-A-runtime.txt'))
if not files:
    raise SystemExit('No Part A report found.')
latest = max(files, key=lambda p: p.stat().st_mtime)
print(latest)
print(latest.read_text())
PY
```

For B or C, replace `A-runtime` in the glob with `B-sources` or `C-gate`.

## Stop, failures, and rollback

Ctrl+C stops the audit only. It never publishes a zero command or takes control. Run it only with the robot already stationary. If motion is unexpectedly present, use the existing validated driver's stopping procedure in the command terminal.

If interrupted before reports are generated, copy the error and rerun after confirming the robot is stationary. Do not interpret a partial transcript as a complete audit.

No launch, Xacro, YAML, ROS parameter, controller state, hardware source, simulator setting, or package is modified. Only the utility, optional backup, Python bytecode, and new report/evidence files are created. No build or robot-configuration rollback is required. Restore the timestamped helper backup only if you need to undo replacing an earlier copy of this audit script.

Do not start mapping or implement changes after seeing `EVIDENCE_CAPTURED_FOR_REVIEW`. That status explicitly does not approve integration. Review the reports first, request any missing exact source ranges, then prepare the smallest explicit patch with its own validation and rollback.

## Verification scope

Before generating this runbook, the supplied code passed:

- Python syntax checks.
- TF topology and parameter-flattening helper tests.
- Runtime URDF/body-frame extraction tests.
- Three-part report-rendering tests.
- Static checks for absence of application publisher creation, parameter-setting calls, and action-goal submission.

It has not been executed against your live ROS/Gazebo container. Treat any live error as evidence to inspect, not a reason to install more dependencies blindly.
