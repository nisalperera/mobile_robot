#!/usr/bin/env python3
"""Patch only the generated observer to use native Fortress CLI transport."""
from pathlib import Path
from datetime import datetime
import shutil

HELPERS = '\n# Fortress observer backend: existing ign CLI, no gz Python bindings.\nimport ast as _proto_ast\nimport re as _proto_re\nfrom types import SimpleNamespace as _Namespace\n\n\ndef _proto_text(text):\n    tokens = _proto_re.findall(r\'"(?:\\\\.|[^"\\\\])*"|[{}:]|[^\\s{}:]+\', text)\n    index = 0\n\n    def message(nested=False):\n        nonlocal index\n        result = {}\n        while index < len(tokens):\n            key = tokens[index]\n            index += 1\n            if key == "}":\n                if not nested:\n                    raise ValueError("Unexpected closing protobuf brace")\n                return result\n            if index < len(tokens) and tokens[index] == ":":\n                index += 1\n            if index >= len(tokens):\n                raise ValueError("Incomplete protobuf field")\n            value = tokens[index]\n            index += 1\n            if value == "{":\n                value = message(True)\n            elif value.startswith(\'"\'):\n                value = _proto_ast.literal_eval(value)\n            result.setdefault(key, []).append(value)\n        if nested:\n            raise ValueError("Unclosed protobuf message")\n        return result\n    return message()\n\n\ndef _first(message, key, default=None):\n    return message.get(key, [default])[0]\n\n\ndef _ign_scene(service):\n    text = run(["ign", "service", "-s", service,\n                "--reqtype", "ignition.msgs.Empty",\n                "--reptype", "ignition.msgs.Scene",\n                "--timeout", "5000", "--req", ""])\n    parsed = _proto_text(text)\n    models = []\n    for model in parsed.get("model", []):\n        links = [_Namespace(name=_first(link, "name", ""))\n                 for link in model.get("link", [])]\n        models.append(_Namespace(name=_first(model, "name", ""),\n                                 id=int(_first(model, "id", 0)), link=links))\n    print("Fortress scene models and direct link names:", flush=True)\n    for model in models[:30]:\n        print(f"  {model.name!r}, id={model.id}: {[link.name for link in model.link]}", flush=True)\n    if not models:\n        raise RuntimeError(f"Native ign scene response has no models. Response excerpt: {text[:1800]!r}")\n    return _Namespace(model=models)\n\n\nclass _IgnPoseObserver:\n    def __init__(self, topic, callback):\n        self.error = None\n        self._closed = False\n        self._stderr = tempfile.TemporaryFile(mode="w+t")\n        self.proc = subprocess.Popen(\n            ["stdbuf", "-oL", "ign", "topic", "-e", "-t", topic],\n            stdout=subprocess.PIPE, stderr=self._stderr,\n            text=True, bufsize=1,\n        )\n        self._thread = threading.Thread(target=self._read, args=(callback,), daemon=True)\n        self._thread.start()\n\n    def _read(self, callback):\n        stamp = None\n        chunk = []\n        depth = 0\n        try:\n            for line in self.proc.stdout:\n                stripped = line.strip()\n                if not chunk and not _proto_re.match(r"^(header|pose)\\s*\\{", stripped):\n                    continue\n                chunk.append(line)\n                braces = _proto_re.sub(r\'"(?:\\\\.|[^"\\\\])*"\', \'""\', line)\n                depth += braces.count("{") - braces.count("}")\n                if depth < 0 or len(chunk) > 10000:\n                    raise RuntimeError("Invalid or oversized ign pose text block")\n                if depth:\n                    continue\n                parsed = _proto_text("".join(chunk))\n                chunk = []\n                if "header" in parsed:\n                    header = _first(parsed, "header", {})\n                    value = _first(header, "stamp", {})\n                    stamp = _Namespace(sec=int(_first(value, "sec", 0)),\n                                       nsec=int(_first(value, "nsec", 0)))\n                for pose in parsed.get("pose", []):\n                    if stamp is None:\n                        continue\n                    position = _first(pose, "position", {})\n                    orientation = _first(pose, "orientation", {})\n                    p = _Namespace(**{key: float(_first(position, key, 0)) for key in ("x", "y", "z")})\n                    q = _Namespace(**{key: float(_first(orientation, key, 0)) for key in ("x", "y", "z", "w")})\n                    entity = _Namespace(id=int(_first(pose, "id", 0)),\n                                        name=_first(pose, "name", ""),\n                                        position=p, orientation=q)\n                    callback(_Namespace(header=_Namespace(stamp=stamp), pose=[entity]))\n            if not self._closed:\n                self.error = "ign topic echo ended unexpectedly"\n        except Exception as exc:\n            if not self._closed:\n                self.error = str(exc)\n\n    def check(self):\n        if self.error or self.proc.poll() is not None:\n            self._stderr.flush()\n            self._stderr.seek(0)\n            detail = self._stderr.read()[-1800:]\n            raise RuntimeError(f"Fortress pose observer failed: {self.error}; {detail}")\n\n    def close(self):\n        self._closed = True\n        if self.proc.poll() is None:\n            self.proc.terminate()\n            try:\n                self.proc.wait(timeout=3)\n            except subprocess.TimeoutExpired:\n                self.proc.kill()\n                self.proc.wait(timeout=3)\n        self._thread.join(timeout=2)\n        self.proc.stdout.close()\n        self._stderr.close()\n\n'


def patched(old):
    if "class _IgnPoseObserver:" in old:
        raise RuntimeError("Fortress observer patch is already present. No files changed.")
    new = old
    def replace_once(a, b):
        nonlocal new
        if new.count(a) != 1:
            raise RuntimeError(f"Expected exactly one patch anchor: {a!r}. No files changed.")
        new = new.replace(a, b, 1)
    for line in (
        "    from gz.transport13 import Node as GzNode\n",
        "    from gz.msgs10.pose_v_pb2 import Pose_V\n",
        "    from gz.msgs10.scene_pb2 import Scene\n",
        "    from google.protobuf import text_format\n",
    ):
        replace_once(line, "")
    replace_once('run(["gz","topic","-l"])', 'run(["ign","topic","-l"])')
    start = new.index("    scene = Scene()\n")
    end = new.index("    models = [m for m in scene.model", start)
    new = new[:start] + '    scene = _ign_scene(world_prefix + "/scene/info")\n' + new[end:]
    replace_once('set(WHEELS).issubset({link.name for link in m.link})',
                 'set(WHEELS).issubset({link.name.rsplit("::", 1)[-1] for link in m.link})')
    replace_once('    gz = GzNode()\n    if not gz.subscribe(Pose_V,pose_topic,on_pose):\n        raise RuntimeError("Gazebo subscription failed")\n',
                 '    observer = _IgnPoseObserver(pose_topic, on_pose)\n')
    replace_once('        gz.unsubscribe(pose_topic)\n', '        observer.close()\n')
    replace_once('            rclpy.spin_once(node,timeout_sec=.02)\n',
                 '            rclpy.spin_once(node,timeout_sec=.02)\n            observer.check()\n')
    replace_once('def capture():\n', HELPERS + '\ndef capture():\n')
    compile(new, "mecanum_four_directions.py", "exec")
    if 'from gz.' in new or 'run(["gz"' in new:
        raise RuntimeError("Unexpected remaining gz dependency. No files changed.")
    return new


def main():
    path = Path("mobile_robot/launch/mecanum_four_directions.py")
    old = path.read_text()
    new = patched(old)
    backup = path.with_name(path.name + ".before-fortress-" + datetime.now().strftime("%Y%m%d-%H%M%S"))
    shutil.copy2(path, backup)
    path.write_text(new)
    print(f"Updated observer: {path}")
    print(f"Backup: {backup}")
    print(f"Rollback command: cp -- {backup} {path}")
    print("Syntax check: PASS")
    print("Backend: ign CLI / ignition.msgs types; no gz Python bindings required")
    print("No launch, Xacro, controller YAML, or drive-sequence changes.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        raise SystemExit(f"ERROR: {exc}")
