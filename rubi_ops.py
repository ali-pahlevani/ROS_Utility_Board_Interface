#!/usr/bin/env python3
"""RUBI operations layer.

All ROS-side logic and helpers that are independent of the GUI live here so
they can be unit-tested headlessly. The Dear PyGui app (rubi.py) imports this.
"""

import os
import csv
import time
import fnmatch
import subprocess

import rclpy
from rclpy.qos import QoSReliabilityPolicy, QoSDurabilityPolicy
from rosidl_runtime_py import message_to_yaml, set_message_fields
from rosidl_runtime_py.utilities import get_service, get_message

try:
    import yaml
except ImportError:
    yaml = None

# ---- Shared color palette (legend and table cells use the SAME constants) ----
C_LOW = (255, 107, 107)    # rate < 1 Hz
C_MID = (210, 210, 210)    # 1 Hz <= rate <= 10 Hz
C_HIGH = (107, 222, 107)   # rate > 10 Hz
C_MUTED = (170, 170, 180)
C_ACCENT = (94, 169, 255)
C_WARN = (255, 184, 77)
C_TEXT = (228, 230, 235)
C_OK = (107, 222, 107)


# ======================================================================
# QoS helpers (mismatch detection)
# ======================================================================
# Map by the enum member *name* so it works across distros (humble/jazzy),
# including policies like BEST_AVAILABLE that only exist on newer releases.
_REL_CODES = {'RELIABLE': 'R', 'BEST_EFFORT': 'BE',
              'BEST_AVAILABLE': 'BA', 'SYSTEM_DEFAULT': 'SD', 'UNKNOWN': '?'}
_DUR_CODES = {'VOLATILE': 'V', 'TRANSIENT_LOCAL': 'TL',
              'BEST_AVAILABLE': 'BA', 'SYSTEM_DEFAULT': 'SD', 'UNKNOWN': '?'}


def rel_code(r):
    return _REL_CODES.get(getattr(r, 'name', str(r)), '?')


def dur_code(d):
    return _DUR_CODES.get(getattr(d, 'name', str(d)), '?')


def qos_summary(rel_set, dur_set):
    if not rel_set:
        return '—'
    rc = '/'.join(sorted({rel_code(r) for r in rel_set}))
    dc = '/'.join(sorted({dur_code(d) for d in dur_set}))
    return f"{rc}·{dc}"


def qos_mismatch(pub_rel, pub_dur, sub_rel, sub_dur):
    """Classic ROS 2 request/offered incompatibilities that silently drop data."""
    reasons = []
    if QoSReliabilityPolicy.BEST_EFFORT in pub_rel and QoSReliabilityPolicy.RELIABLE in sub_rel:
        reasons.append('reliability: BEST_EFFORT pub vs RELIABLE sub')
    if QoSDurabilityPolicy.VOLATILE in pub_dur and QoSDurabilityPolicy.TRANSIENT_LOCAL in sub_dur:
        reasons.append('durability: VOLATILE pub vs TRANSIENT_LOCAL sub')
    return reasons


# ======================================================================
# Bandwidth + health watchdog
# ======================================================================
def human_bytes(b, suffix='/s'):
    if b is None:
        return 'NaN'
    for unit in ('B', 'KB', 'MB', 'GB'):
        if b < 1024.0:
            return f"{b:.0f} {unit}{suffix}" if unit == 'B' else f"{b:.1f} {unit}{suffix}"
        b /= 1024.0
    return f"{b:.1f} TB{suffix}"


def load_rules(path):
    """Load YAML health rules: {topics: {pattern: {min_hz, max_hz, max_delay}}}."""
    if not path or yaml is None or not os.path.isfile(path):
        return {}
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get('topics', {}) or {}
    except Exception as e:
        print(f"[RUBI] Failed to load rules '{path}': {e}")
        return {}


def match_rule(rules, name):
    if name in rules:
        return rules[name]
    for pattern, rule in rules.items():
        if fnmatch.fnmatch(name, pattern):
            return rule
    return None


def topic_health(rules, name, rate, delay):
    """Return (label, color) for the Health column based on user rules."""
    rule = match_rule(rules, name)
    if not rule:
        return ('—', C_MUTED)
    fail = False
    try:
        r = float(rate)
        if 'min_hz' in rule and r < float(rule['min_hz']):
            fail = True
        if 'max_hz' in rule and r > float(rule['max_hz']):
            fail = True
    except (ValueError, TypeError):
        pass
    if 'max_delay' in rule:
        try:
            if float(delay) > float(rule['max_delay']):
                fail = True
        except (ValueError, TypeError):
            pass
    return ('✗ FAIL', C_LOW) if fail else ('✓ OK', C_OK)


# ======================================================================
# One-shot service / parameter / lifecycle calls
#
# Every helper call runs in its OWN rclpy context + executor. An isolated
# context never shares a wait set with the monitor node spinning in the GUI
# thread, which avoids rclpy's "wait set index too big" race entirely while
# still discovering everything on the same DDS domain.
# ======================================================================
from rclpy.executors import SingleThreadedExecutor

_seq = [0]


def _run(timeout, body):
    """Run body(node, ex) -> (ok, text) inside a private, isolated context."""
    _seq[0] += 1
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node(f'rubi_helper_{os.getpid()}_{_seq[0]}', context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    try:
        return body(node, ex)
    except Exception as e:
        return False, f"error: {e}"
    finally:
        try:
            ex.remove_node(node)
        except Exception:
            pass
        node.destroy_node()
        try:
            ctx.try_shutdown()
        except Exception:
            pass


def _await_service(ex, cli, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline and not cli.service_is_ready():
        ex.spin_once(timeout_sec=0.05)
    return cli.service_is_ready()


def _await_future(ex, future, timeout):
    ex.spin_until_future_complete(future, timeout_sec=timeout)
    return future.result() if future.done() else None


def request_skeleton(type_str, is_action):
    """Return an editable YAML skeleton (all fields with default values) for a
    service Request or an action Goal, to pre-fill the caller form."""
    try:
        if is_action:
            from rosidl_runtime_py.utilities import get_action
            obj = get_action(type_str).Goal()
        else:
            obj = get_service(type_str).Request()
        text = message_to_yaml(obj).strip()
        return '' if text in ('', '{}', 'null') else text
    except Exception:
        return ''


def call_service(type_str, name, request_yaml='', timeout=5.0):
    """Call any service. Returns (ok, text)."""
    try:
        srv = get_service(type_str)
    except Exception as e:
        return False, f"unknown service type '{type_str}': {e}"
    req = srv.Request()
    if request_yaml and request_yaml.strip():
        if yaml is None:
            return False, "pyyaml not installed"
        try:
            data = yaml.safe_load(request_yaml)
            if data:
                set_message_fields(req, data)
        except Exception as e:
            return False, f"bad request YAML: {e}"

    def body(node, ex):
        cli = node.create_client(srv, name)
        if not _await_service(ex, cli, timeout):
            return False, f"service '{name}' unavailable"
        res = _await_future(ex, cli.call_async(req), timeout)
        if res is None:
            return False, "timed out waiting for response"
        return True, message_to_yaml(res)

    return _run(timeout, body)


def send_action_goal(type_str, name, goal_yaml='', timeout=15.0):
    """Send an action goal and wait for the result. Returns (ok, text)."""
    from rclpy.action import ActionClient
    from rosidl_runtime_py.utilities import get_action
    try:
        action = get_action(type_str)
    except Exception as e:
        return False, f"unknown action type '{type_str}': {e}"
    goal = action.Goal()
    if goal_yaml and goal_yaml.strip():
        try:
            data = yaml.safe_load(goal_yaml)
            if data:
                set_message_fields(goal, data)
        except Exception as e:
            return False, f"bad goal YAML: {e}"

    def body(node, ex):
        client = ActionClient(node, action, name)
        deadline = time.time() + timeout
        while time.time() < deadline and not client.server_is_ready():
            ex.spin_once(timeout_sec=0.05)
        if not client.server_is_ready():
            return False, f"action server '{name}' unavailable"
        handle = _await_future(ex, client.send_goal_async(goal), timeout)
        if handle is None or not handle.accepted:
            return False, "goal rejected by server"
        result = _await_future(ex, handle.get_result_async(), timeout)
        if result is None:
            return False, "timed out waiting for result"
        return True, message_to_yaml(result.result)

    return _run(timeout, body)


# ---- Parameters ------------------------------------------------------
def _pv_to_py(v):
    from rcl_interfaces.msg import ParameterType as PT
    t = v.type
    if t == PT.PARAMETER_BOOL:
        return v.bool_value
    if t == PT.PARAMETER_INTEGER:
        return v.integer_value
    if t == PT.PARAMETER_DOUBLE:
        return v.double_value
    if t == PT.PARAMETER_STRING:
        return v.string_value
    if t == PT.PARAMETER_BYTE_ARRAY:
        return list(v.byte_array_value)
    if t == PT.PARAMETER_BOOL_ARRAY:
        return list(v.bool_array_value)
    if t == PT.PARAMETER_INTEGER_ARRAY:
        return list(v.integer_array_value)
    if t == PT.PARAMETER_DOUBLE_ARRAY:
        return list(v.double_array_value)
    if t == PT.PARAMETER_STRING_ARRAY:
        return list(v.string_array_value)
    return None


def _pv_type_name(v):
    from rcl_interfaces.msg import ParameterType as PT
    return {PT.PARAMETER_NOT_SET: 'not set', PT.PARAMETER_BOOL: 'bool',
            PT.PARAMETER_INTEGER: 'int', PT.PARAMETER_DOUBLE: 'double',
            PT.PARAMETER_STRING: 'string', PT.PARAMETER_BYTE_ARRAY: 'byte[]',
            PT.PARAMETER_BOOL_ARRAY: 'bool[]', PT.PARAMETER_INTEGER_ARRAY: 'int[]',
            PT.PARAMETER_DOUBLE_ARRAY: 'double[]',
            PT.PARAMETER_STRING_ARRAY: 'string[]'}.get(v.type, '?')


def list_node_parameters(node_fqn, timeout=4.0):
    """Return list of (name, type_name, value_repr) for a node. Empty on failure."""
    from rcl_interfaces.srv import ListParameters, GetParameters

    def body(node, ex):
        lc = node.create_client(ListParameters, f'{node_fqn}/list_parameters')
        if not _await_service(ex, lc, timeout):
            return True, []
        req = ListParameters.Request()
        req.depth = 0
        res = _await_future(ex, lc.call_async(req), timeout)
        if res is None:
            return True, []
        names = sorted(res.result.names)
        if not names:
            return True, []
        gc = node.create_client(GetParameters, f'{node_fqn}/get_parameters')
        if not _await_service(ex, gc, timeout):
            return True, [(n, '?', '') for n in names]
        greq = GetParameters.Request()
        greq.names = names
        gres = _await_future(ex, gc.call_async(greq), timeout)
        values = gres.values if gres else []
        return True, [(n, _pv_type_name(v), str(_pv_to_py(v)))
                      for n, v in zip(names, values)]

    ok, out = _run(timeout, body)
    return out if ok and isinstance(out, list) else []


def set_node_parameter(node_fqn, name, text_value, timeout=4.0):
    """Set a parameter, inferring type from its current value. Returns (ok, msg)."""
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType as PT

    def body(node, ex):
        gc = node.create_client(GetParameters, f'{node_fqn}/get_parameters')
        if not _await_service(ex, gc, timeout):
            return False, "node has no parameter services"
        greq = GetParameters.Request()
        greq.names = [name]
        gres = _await_future(ex, gc.call_async(greq), timeout)
        if not gres or not gres.values:
            return False, f"parameter '{name}' not found"
        cur = gres.values[0]
        pv = ParameterValue()
        pv.type = cur.type
        try:
            if cur.type == PT.PARAMETER_BOOL:
                pv.bool_value = text_value.strip().lower() in ('1', 'true', 'yes', 'on')
            elif cur.type == PT.PARAMETER_INTEGER:
                pv.integer_value = int(text_value)
            elif cur.type == PT.PARAMETER_DOUBLE:
                pv.double_value = float(text_value)
            elif cur.type == PT.PARAMETER_STRING:
                pv.string_value = text_value
            else:
                return False, "editing array parameters is not supported"
        except ValueError:
            return False, f"'{text_value}' is not a valid {_pv_type_name(cur)}"
        sc = node.create_client(SetParameters, f'{node_fqn}/set_parameters')
        if not _await_service(ex, sc, timeout):
            return False, "node has no set_parameters service"
        sreq = SetParameters.Request()
        sreq.parameters = [Parameter(name=name, value=pv)]
        sres = _await_future(ex, sc.call_async(sreq), timeout)
        if not sres or not sres.results:
            return False, "no response"
        r = sres.results[0]
        return (r.successful, r.reason or ("set OK" if r.successful else "rejected"))

    return _run(timeout, body)


# ---- Lifecycle -------------------------------------------------------
LIFECYCLE_TRANSITIONS = [
    ('configure', 1), ('activate', 3), ('deactivate', 4),
    ('cleanup', 2), ('shutdown', 5),
]


def get_lifecycle_state(node_fqn, timeout=2.0):
    from lifecycle_msgs.srv import GetState

    def body(node, ex):
        cli = node.create_client(GetState, f'{node_fqn}/get_state')
        if not _await_service(ex, cli, timeout):
            return True, None
        res = _await_future(ex, cli.call_async(GetState.Request()), timeout)
        return True, (res.current_state.label if res else None)

    ok, label = _run(timeout, body)
    return label if ok else None


def change_lifecycle_state(node_fqn, transition_id, timeout=4.0):
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.msg import Transition

    def body(node, ex):
        cli = node.create_client(ChangeState, f'{node_fqn}/change_state')
        if not _await_service(ex, cli, timeout):
            return False, "not a lifecycle node"
        req = ChangeState.Request()
        req.transition = Transition(id=transition_id)
        res = _await_future(ex, cli.call_async(req), timeout)
        if res is None:
            return False, "timed out"
        return res.success, ("transition OK" if res.success else "transition rejected")

    return _run(timeout, body)


# ======================================================================
# Process metrics (best-effort node -> PID matching via psutil)
# ======================================================================
class ProcessMonitor:
    """Best-effort mapping of ROS node names to OS processes (CPU%/RAM)."""

    def __init__(self):
        try:
            import psutil
            self.psutil = psutil
        except ImportError:
            self.psutil = None
        self._procs = {}   # pid -> psutil.Process (kept so cpu_percent() is delta-based)
        self.by_node = {}  # node base name -> (pid, cpu%, rss_bytes)

    @property
    def available(self):
        return self.psutil is not None

    def scan(self, node_names):
        if not self.psutil:
            return
        ps = self.psutil
        bases = {n.rsplit('/', 1)[-1]: n for n in node_names}
        seen = set()
        result = {}
        for proc in ps.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmd = ' '.join(proc.info.get('cmdline') or [])
                if not cmd:
                    continue
                for base, full in bases.items():
                    if not base:
                        continue
                    # match the node name as a whole token in the command line
                    if base in cmd.split() or f'__node:={base}' in cmd or f'/{base}' in cmd:
                        pid = proc.info['pid']
                        seen.add(pid)
                        p = self._procs.get(pid)
                        if p is None:
                            p = ps.Process(pid)
                            p.cpu_percent(None)  # prime
                            self._procs[pid] = p
                        try:
                            cpu = p.cpu_percent(None)
                            rss = p.memory_info().rss
                        except Exception:
                            cpu, rss = 0.0, 0
                        result[full] = (pid, cpu, rss)
                        break
            except (ps.NoSuchProcess, ps.AccessDenied):
                continue
        for pid in list(self._procs):
            if pid not in seen:
                del self._procs[pid]
        self.by_node = result


# ======================================================================
# Text sparkline (rendered as Unicode blocks; no plot widget needed)
# ======================================================================
_BLOCKS = '▁▂▃▄▅▆▇█'


def spark(values, width=14):
    import math
    vals = [float(v) for v in values
            if v is not None and not math.isnan(v) and not math.isinf(v)][-width:]
    if not vals:
        return ''
    lo, hi = min(vals), max(vals)
    if hi <= lo:
        return (_BLOCKS[0] if hi == 0 else _BLOCKS[3]) * len(vals)
    span = hi - lo
    n = len(_BLOCKS) - 1
    return ''.join(_BLOCKS[min(n, int((v - lo) / span * n))] for v in vals)


# ======================================================================
# Graph snapshot + diff
# ======================================================================
def diff_sets(old, new):
    """Return (added, removed) sorted lists."""
    old, new = set(old), set(new)
    return sorted(new - old), sorted(old - new)


# ======================================================================
# Exporters
# ======================================================================
def _ensure_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)


def export_topics_csv(path, rows):
    """rows: list of dicts with keys name,type,rate,bw,delay,qos,pubs,subs."""
    _ensure_parent(path)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Topic', 'Type', 'Rate(Hz)', 'Bandwidth', 'Delay(s)', 'QoS',
                    'Publishers', 'Subscribers'])
        for r in rows:
            w.writerow([r['name'], r['type'], r['rate'], r['bw'], r['delay'],
                        r['qos'], ';'.join(r['pubs']), ';'.join(r['subs'])])
    return path


def export_topics_markdown(path, rows):
    _ensure_parent(path)
    lines = ['| Topic | Type | Rate (Hz) | Bandwidth | Delay (s) | QoS | Publishers | Subscribers |',
             '|---|---|---|---|---|---|---|---|']
    for r in rows:
        lines.append('| {} | {} | {} | {} | {} | {} | {} | {} |'.format(
            r['name'], r['type'], r['rate'], r['bw'], r['delay'], r['qos'],
            '<br>'.join(r['pubs']), '<br>'.join(r['subs'])))
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return path


def export_graph_dot(path, rows):
    """Graphviz dot of the node<->topic graph (pub: node->topic, sub: topic->node)."""
    _ensure_parent(path)
    lines = ['digraph rubi {', '  rankdir=LR;',
             '  node [style=filled];']
    nodes, topics = set(), set()
    edges = []
    for r in rows:
        topics.add(r['name'])
        for p in r['pubs']:
            if p and p != 'None':
                nodes.add(p)
                edges.append((f'"{p}"', f'"{r["name"]}"'))
        for s in r['subs']:
            if s and s != 'None':
                nodes.add(s)
                edges.append((f'"{r["name"]}"', f'"{s}"'))
    for n in sorted(nodes):
        lines.append(f'  "{n}" [shape=ellipse, fillcolor="#bcd6ff"];')
    for t in sorted(topics):
        lines.append(f'  "{t}" [shape=box, fillcolor="#d7ffd9"];')
    for a, b in edges:
        lines.append(f'  {a} -> {b};')
    lines.append('}')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return path


# ======================================================================
# Bag recording / playback
# ======================================================================
BAG_DIR = 'Bag'


def list_bags(base=BAG_DIR):
    """Return paths of rosbag2 directories under the Bag folder."""
    if not os.path.isdir(base):
        return []
    out = []
    for entry in sorted(os.listdir(base)):
        p = os.path.join(base, entry)
        if os.path.isdir(p) and os.path.isfile(os.path.join(p, 'metadata.yaml')):
            out.append(p)
    return out


def bag_info(path):
    """Return the text of `ros2 bag info <path>`."""
    path = (path or '').strip()
    if not path:
        return "Provide a bag path."
    if not os.path.exists(path):
        return f"path not found: {path}"
    try:
        r = subprocess.run(['ros2', 'bag', 'info', path],
                           capture_output=True, text=True, timeout=20)
        return r.stdout or r.stderr or "(no output)"
    except FileNotFoundError:
        return "ros2 CLI not found on PATH"
    except Exception as e:
        return f"failed: {e}"


class BagRecorder:
    def __init__(self):
        self.proc = None
        self.output = None

    @property
    def recording(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, topics_text, out_dir=''):
        if self.recording:
            return False, "already recording"
        os.makedirs(BAG_DIR, exist_ok=True)
        cmd = ['ros2', 'bag', 'record']
        topics_text = (topics_text or '').strip()
        if not topics_text or topics_text in ('-a', 'all', '*'):
            cmd.append('-a')
        else:
            cmd += topics_text.split()
        name = out_dir.strip() or ('rosbag2_' + time.strftime('%Y_%m_%d-%H_%M_%S'))
        # keep all recordings inside the Bag/ folder unless an absolute path is given
        self.output = name if os.path.isabs(name) else os.path.join(BAG_DIR, name)
        cmd += ['-o', self.output]
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            return False, "ros2 CLI not found on PATH"
        return True, f"recording -> {self.output}"

    def stop(self):
        if not self.recording:
            return False, "not recording"
        import signal
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        out = self.output
        self.proc = None
        return True, f"saved {out}"


class BagPlayer:
    def __init__(self):
        self.proc = None
        self.path = None

    @property
    def playing(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, path, rate='1.0', loop=False):
        if self.playing:
            return False, "already playing"
        path = (path or '').strip()
        if not path or not os.path.exists(path):
            return False, f"bag not found: {path}"
        cmd = ['ros2', 'bag', 'play', path]
        try:
            if rate and float(rate) > 0:
                cmd += ['--rate', str(float(rate))]
        except (ValueError, TypeError):
            pass
        if loop:
            cmd.append('--loop')
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            return False, "ros2 CLI not found on PATH"
        self.path = path
        return True, f"playing {path}"

    def stop(self):
        if not self.playing:
            return False, "not playing"
        import signal
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.proc = None
        return True, "playback stopped"
