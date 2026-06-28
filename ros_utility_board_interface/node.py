"""RUBI ROS 2 introspection node: topic/service/action/node discovery,
rate/bandwidth/delay stats, QoS, /rosout, TF, lifecycle detection and the
live-plot time-series collection."""

import threading
import time
import importlib
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from rclpy.serialization import serialize_message
from rcl_interfaces.msg import Log
from tf2_msgs.msg import TFMessage

from . import ops


class SimpleMonitorNode(Node):
    def __init__(self):
        super().__init__('ros_utility_board_interface')
        self._self_fqn = f"{self.get_namespace().rstrip('/')}/{self.get_name()}"

        self.lock = threading.Lock()

        self.topic_stats = {}
        self.service_stats = {}
        self.action_stats = {}
        self.nodes = []
        self.lifecycle_nodes = set()

        self.logs = deque(maxlen=2000)
        self.tf_frames = {}      # child -> {'parent', 'ts': deque, 'static': bool}

        # live plotting: requested (topic -> set of field paths) and the
        # collected time series ((topic, field) -> deque of (t, value))
        self.plot_requests = {}
        self.plot_series = {}

        self._my_subscriptions = {}
        self.msg_classes = {}

        self.proc_mon = ops.ProcessMonitor()

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # /rosout, /tf, /tf_static
        rosout_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST, depth=100)
        self.create_subscription(Log, '/rosout', self._rosout_cb, rosout_qos)
        self.create_subscription(TFMessage, '/tf', lambda m: self._tf_cb(m, False), 50)
        tf_static_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=100)
        self.create_subscription(TFMessage, '/tf_static',
                                 lambda m: self._tf_cb(m, True), tf_static_qos)

        self.graph_timer = self.create_timer(1.5, self._update_graph)
        self.stats_timer = self.create_timer(0.4, self._compute_stats)
        self.proc_timer = self.create_timer(2.0, self._scan_procs)

    # ---- subscription callbacks ----
    def _topic_callback(self, msg, topic):
        now = time.time()
        try:
            nbytes = len(serialize_message(msg))
        except Exception:
            nbytes = 0
        with self.lock:
            stats = self.topic_stats.get(topic)
            if stats:
                stats['timestamps'].append(now)
                stats['sizes'].append(nbytes)
                stats['last_msg'] = msg
                reqs = self.plot_requests.get(topic)
                if reqs:
                    for fpath in reqs:
                        try:
                            v = ops.get_field_value(msg, fpath)
                        except Exception:
                            continue
                        dq = self.plot_series.get((topic, fpath))
                        if dq is not None:
                            dq.append((now, v))
                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    stamp = msg.header.stamp
                    stamp_sec = stamp.sec + stamp.nanosec * 1e-9
                    delay = now - stamp_sec
                    if delay >= 0:
                        stats['delays'].append(delay)

    def _rosout_cb(self, msg):
        with self.lock:
            self.logs.append((msg.level, msg.name, msg.msg,
                              msg.stamp.sec + msg.stamp.nanosec * 1e-9))

    def _tf_cb(self, msg, static):
        now = time.time()
        with self.lock:
            for tr in msg.transforms:
                child = tr.child_frame_id
                f = self.tf_frames.get(child)
                if f is None:
                    f = {'parent': tr.header.frame_id,
                         'ts': deque(maxlen=100), 'static': static}
                    self.tf_frames[child] = f
                f['parent'] = tr.header.frame_id
                f['static'] = static
                f['ts'].append(now)

    def _scan_procs(self):
        try:
            self.proc_mon.scan(list(self.nodes))
        except Exception:
            pass

    def add_plot_field(self, topic, fpath):
        with self.lock:
            self.plot_requests.setdefault(topic, set()).add(fpath)
            if (topic, fpath) not in self.plot_series:
                self.plot_series[(topic, fpath)] = deque(maxlen=6000)

    def remove_plot_field(self, topic, fpath):
        with self.lock:
            reqs = self.plot_requests.get(topic)
            if reqs:
                reqs.discard(fpath)
                if not reqs:
                    del self.plot_requests[topic]
            self.plot_series.pop((topic, fpath), None)

    # ---- periodic graph discovery ----
    def _update_graph(self):
        try:
            with self.lock:
                node_names = self.get_node_names_and_namespaces()
                self.nodes = sorted({f"{ns}{name}" for name, ns in node_names})

                topic_info = self.get_topic_names_and_types()
                current_topics = {name for name, _ in topic_info}

                for topic in list(self.topic_stats):
                    if topic not in current_topics:
                        if topic in self._my_subscriptions:
                            self.destroy_subscription(self._my_subscriptions[topic])
                            del self._my_subscriptions[topic]
                        if topic in self.topic_stats:
                            del self.topic_stats[topic]

                for name, types in topic_info:
                    if name not in self.topic_stats:
                        self.topic_stats[name] = {
                            'type': types[0] if types else 'Unknown',
                            'pubs': [], 'subs': [],
                            'timestamps': deque(maxlen=200),
                            'delays': deque(maxlen=200),
                            'sizes': deque(maxlen=200),
                            'rate_hist': deque(maxlen=40),
                            'last_msg': None,
                            'rate': 'NaN', 'delay': 'NaN', 'bw': 'NaN',
                            'qos': '—', 'qos_bad': False, 'qos_reasons': []
                        }

                    s = self.topic_stats[name]

                    if name.endswith(('/_action/feedback', '/_action/status',
                                      '/_action/result', '/_action/goal')):
                        pubs = self.get_publishers_info_by_topic(name)
                        s['pubs'] = sorted({f"{p.node_namespace}{p.node_name}" for p in pubs})
                        s['subs'] = []
                        s['rate'] = 'N/A'
                        s['delay'] = 'N/A'
                        s['bw'] = 'N/A'
                        s['qos'] = '—'
                        s['qos_bad'] = False
                        s['qos_reasons'] = []
                        if name in self._my_subscriptions:
                            self.destroy_subscription(self._my_subscriptions[name])
                            del self._my_subscriptions[name]
                        continue

                    pubs = self.get_publishers_info_by_topic(name)
                    s['pubs'] = sorted({f"{p.node_namespace}{p.node_name}" for p in pubs})

                    subs = [su for su in self.get_subscriptions_info_by_topic(name)
                            if f"{su.node_namespace}{su.node_name}" != self._self_fqn]
                    s['subs'] = sorted({f"{su.node_namespace}{su.node_name}" for su in subs})

                    pub_rel = {p.qos_profile.reliability for p in pubs}
                    pub_dur = {p.qos_profile.durability for p in pubs}
                    sub_rel = {su.qos_profile.reliability for su in subs}
                    sub_dur = {su.qos_profile.durability for su in subs}
                    reasons = ops.qos_mismatch(pub_rel, pub_dur, sub_rel, sub_dur)
                    s['qos_bad'] = bool(reasons)
                    s['qos_reasons'] = reasons
                    if pub_rel:
                        s['qos'] = ops.qos_summary(pub_rel, pub_dur)
                    elif sub_rel:
                        s['qos'] = ops.qos_summary(sub_rel, sub_dur)
                    else:
                        s['qos'] = '—'

                    if s['pubs']:
                        if name not in self._my_subscriptions:
                            try:
                                type_str = s['type']
                                pkg, _, msg_type = type_str.rpartition('/msg/')
                                msg_module = importlib.import_module(pkg + '.msg')
                                msg_class = getattr(msg_module, msg_type)
                                cb = lambda msg, t=name: self._topic_callback(msg, t)
                                sub = self.create_subscription(
                                    msg_class, name, cb, qos_profile=self.sensor_qos)
                                self._my_subscriptions[name] = sub
                                self.msg_classes[name] = msg_class
                            except Exception as e:
                                self.get_logger().warn(f"Failed to subscribe to {name}: {e}")
                    else:
                        s['rate'] = '0.0'
                        s['delay'] = 'N/A'
                        s['bw'] = '0 B/s'
                        if name in self._my_subscriptions:
                            self.destroy_subscription(self._my_subscriptions[name])
                            del self._my_subscriptions[name]

                self.action_stats.clear()
                for topic_name, stats in self.topic_stats.items():
                    if topic_name.endswith('/_action/status'):
                        action_name = topic_name[:-len('/_action/status')]
                        feedback_topic = action_name + '/_action/feedback'
                        inferred_type = 'Unknown'
                        if feedback_topic in self.topic_stats:
                            ft = self.topic_stats[feedback_topic]['type']
                            if '_FeedbackMessage' in ft:
                                inferred_type = ft.replace('_FeedbackMessage', '')
                            elif '/action/' in ft:
                                inferred_type = ft
                        server_nodes = stats['pubs'] if stats['pubs'] else ['Unknown']
                        self.action_stats[action_name] = {
                            'type': inferred_type, 'nodes': server_nodes}

                service_info = self.get_service_names_and_types()
                self.service_stats.clear()
                self.lifecycle_nodes = set()
                for name, types in service_info:
                    self.service_stats[name] = {
                        'type': types[0] if types else 'Unknown', 'nodes': []}
                    if name.endswith('/get_state') and \
                       any('lifecycle_msgs/srv/GetState' in t for t in types):
                        self.lifecycle_nodes.add(name[:-len('/get_state')])

                for node_name, ns in node_names:
                    full_node = f"{ns}{node_name}"
                    try:
                        node_services = self.get_service_names_and_types_by_node(node_name, ns)
                        for s_name, _ in node_services:
                            if s_name in self.service_stats:
                                self.service_stats[s_name]['nodes'].append(full_node)
                    except Exception:
                        pass
        except Exception as e:
            self.get_logger().error(f"Graph update error: {e}")

    def _compute_stats(self):
        now = time.time()
        with self.lock:
            for topic, s in self.topic_stats.items():
                ts = s['timestamps']
                if len(ts) < 2:
                    s['rate'] = '0.0' if (ts and now - ts[0] > 5.0) else 'NaN'
                else:
                    dt = ts[-1] - ts[0]
                    s['rate'] = f"{(len(ts) - 1) / dt:.2f}" if dt > 0 else '0.0'

                sz = s['sizes']
                if s['rate'] in ('NaN', 'N/A'):
                    s['bw'] = s['rate']
                elif len(ts) >= 2 and sz:
                    dt = ts[-1] - ts[0]
                    s['bw'] = ops.human_bytes(sum(sz) / dt) if dt > 0 else '0 B/s'
                else:
                    s['bw'] = '0 B/s'

                try:
                    rv = float(s['rate'])
                    if rv != rv or rv in (float('inf'), float('-inf')):  # NaN/inf
                        rv = 0.0
                except (ValueError, TypeError):
                    rv = 0.0
                s['rate_hist'].append(rv)

                ds = s['delays']
                s['delay'] = f"{sum(ds) / len(ds):.4f}" if ds else 'NaN'


# ======================================================================
# GUI application
# ======================================================================
