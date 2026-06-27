#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       QoSReliabilityPolicy, QoSDurabilityPolicy)
from rclpy.serialization import serialize_message
import dearpygui.dearpygui as dpg
import threading
import time
import importlib
import argparse
import fnmatch
import os
from collections import deque

try:
    import yaml
except ImportError:
    yaml = None

# ---- Shared color palette (legend and table cells use the SAME constants) ----
C_LOW = (255, 107, 107)   # rate < 1 Hz
C_MID = (210, 210, 210)   # 1 Hz <= rate <= 10 Hz
C_HIGH = (107, 222, 107)  # rate > 10 Hz
C_MUTED = (170, 170, 180)
C_ACCENT = (94, 169, 255)
C_WARN = (255, 184, 77)
C_TEXT = (228, 230, 235)  # default cell text (used to reset colored cells)


# ---- QoS helpers (v2: mismatch detection) --------------------------------
def _rel_code(r):
    return {QoSReliabilityPolicy.RELIABLE: 'R',
            QoSReliabilityPolicy.BEST_EFFORT: 'BE'}.get(r, '?')


def _dur_code(d):
    return {QoSDurabilityPolicy.VOLATILE: 'V',
            QoSDurabilityPolicy.TRANSIENT_LOCAL: 'TL'}.get(d, '?')


def _qos_summary(rel_set, dur_set):
    if not rel_set:
        return '—'
    rc = '/'.join(sorted({_rel_code(r) for r in rel_set}))
    dc = '/'.join(sorted({_dur_code(d) for d in dur_set}))
    return f"{rc}·{dc}"


def _qos_mismatch(pub_rel, pub_dur, sub_rel, sub_dur):
    """Classic ROS 2 request/offered incompatibilities that silently drop data."""
    reasons = []
    if QoSReliabilityPolicy.BEST_EFFORT in pub_rel and QoSReliabilityPolicy.RELIABLE in sub_rel:
        reasons.append('reliability: BEST_EFFORT pub vs RELIABLE sub')
    if QoSDurabilityPolicy.VOLATILE in pub_dur and QoSDurabilityPolicy.TRANSIENT_LOCAL in sub_dur:
        reasons.append('durability: VOLATILE pub vs TRANSIENT_LOCAL sub')
    return reasons


# ---- Bandwidth helper (v2) -----------------------------------------------
def _human_bw(b):
    if b is None:
        return 'NaN'
    for unit in ('B', 'KB', 'MB', 'GB'):
        if b < 1024.0:
            return f"{b:.0f} {unit}/s" if unit == 'B' else f"{b:.1f} {unit}/s"
        b /= 1024.0
    return f"{b:.1f} TB/s"


# ---- Health watchdog (v2) -------------------------------------------------
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


def _match_rule(rules, name):
    if name in rules:
        return rules[name]
    for pattern, rule in rules.items():
        if fnmatch.fnmatch(name, pattern):
            return rule
    return None


def topic_health(rules, name, rate, delay):
    """Return (label, color) for the Health column based on user rules."""
    rule = _match_rule(rules, name)
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
    return ('✗ FAIL', C_LOW) if fail else ('✓ OK', C_HIGH)


class SimpleMonitorNode(Node):
    def __init__(self):
        super().__init__('ros_utility_board_interface')

        # RUBI's own fully-qualified name, so it can hide its own monitoring
        # subscriptions from the pub/sub graph it reports.
        self._self_fqn = f"{self.get_namespace().rstrip('/')}/{self.get_name()}"

        self.lock = threading.Lock()

        self.topic_stats = {}
        self.service_stats = {}
        self.action_stats = {}
        self.nodes = []

        self._my_subscriptions = {}
        self.msg_classes = {}

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.graph_timer = self.create_timer(1.5, self._update_graph)
        self.stats_timer = self.create_timer(0.4, self._compute_stats)

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
                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    stamp = msg.header.stamp
                    stamp_sec = stamp.sec + stamp.nanosec * 1e-9
                    delay = now - stamp_sec
                    if delay >= 0:
                        stats['delays'].append(delay)

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
                            'pubs': [],
                            'subs': [],
                            'timestamps': deque(maxlen=200),
                            'delays': deque(maxlen=200),
                            'sizes': deque(maxlen=200),
                            'rate': 'NaN',
                            'delay': 'NaN',
                            'bw': 'NaN',
                            'qos': '—',
                            'qos_bad': False,
                            'qos_reasons': []
                        }

                    s = self.topic_stats[name]

                    if name.endswith('/_action/feedback') or \
                       name.endswith('/_action/status') or \
                       name.endswith('/_action/result') or \
                       name.endswith('/_action/goal'):
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

                    # ---- QoS introspection + mismatch detection (v2) ----
                    pub_rel = {p.qos_profile.reliability for p in pubs}
                    pub_dur = {p.qos_profile.durability for p in pubs}
                    sub_rel = {su.qos_profile.reliability for su in subs}
                    sub_dur = {su.qos_profile.durability for su in subs}
                    reasons = _qos_mismatch(pub_rel, pub_dur, sub_rel, sub_dur)
                    s['qos_bad'] = bool(reasons)
                    s['qos_reasons'] = reasons
                    if pub_rel:
                        s['qos'] = _qos_summary(pub_rel, pub_dur)
                    elif sub_rel:
                        s['qos'] = _qos_summary(sub_rel, sub_dur)
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
                                    msg_class, name, cb,
                                    qos_profile=self.sensor_qos
                                )
                                self._my_subscriptions[name] = sub
                                self.msg_classes[name] = msg_class
                            except Exception as e:
                                self.get_logger().warn(f"Failed to subscribe to {name}: {str(e)}")
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
                            'type': inferred_type,
                            'nodes': server_nodes
                        }

                service_info = self.get_service_names_and_types()
                self.service_stats.clear()
                for name, types in service_info:
                    self.service_stats[name] = {'type': types[0] if types else 'Unknown', 'nodes': []}

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
            self.get_logger().error(f"Graph update error: {str(e)}")

    def _compute_stats(self):
        now = time.time()
        with self.lock:
            for topic, s in self.topic_stats.items():
                ts = s['timestamps']
                if len(ts) < 2:
                    if ts and (now - ts[0] > 5.0):
                        s['rate'] = '0.0'
                    else:
                        s['rate'] = 'NaN'
                else:
                    dt = ts[-1] - ts[0]
                    if dt > 0:
                        rate = (len(ts) - 1) / dt
                        s['rate'] = f"{rate:.2f}"
                    else:
                        s['rate'] = '0.0'

                # bandwidth over the same sample window (v2)
                sz = s['sizes']
                if s['rate'] in ('NaN', 'N/A'):
                    s['bw'] = s['rate']
                elif len(ts) >= 2 and sz:
                    dt = ts[-1] - ts[0]
                    s['bw'] = _human_bw(sum(sz) / dt) if dt > 0 else '0 B/s'
                else:
                    s['bw'] = '0 B/s'

                ds = s['delays']
                if ds:
                    avg_delay = sum(ds) / len(ds)
                    s['delay'] = f"{avg_delay:.4f}"
                else:
                    s['delay'] = 'NaN'


def _rate_color(rate):
    try:
        r = float(rate)
    except (ValueError, TypeError):
        return C_MID
    if r < 1.0:
        return C_LOW
    if r > 10.0:
        return C_HIGH
    return C_MID


def main():
    parser = argparse.ArgumentParser(description='RUBI - ROS Utility Board Interface')
    parser.add_argument('--rules', default='rubi_rules.yaml',
                        help='YAML health-watchdog rules (default: ./rubi_rules.yaml if present)')
    args, _ = parser.parse_known_args()
    rules = load_rules(args.rules)
    if rules:
        print(f"[RUBI] Loaded {len(rules)} health rule(s) from {args.rules}")

    rclpy.init()
    node = SimpleMonitorNode()

    # ROS spinning lives in exactly ONE place (this daemon thread). The UI loop
    # never spins the node itself, so the executor is never entered concurrently.
    def spin_thread():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    threading.Thread(target=spin_thread, daemon=True).start()

    time.sleep(1.5)

    dpg.create_context()
    dpg.create_viewport(title='RUBI (ROS Utility Board Interface)',
                        width=1450, height=1050, min_width=900, min_height=600,
                        resizable=True)

    with dpg.font_registry():
        try:
            font = dpg.add_font("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
            dpg.bind_font(font)
        except Exception:
            pass

    # ---------------------------------------------------------------- theme ---
    with dpg.theme() as global_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (24, 26, 31))
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (31, 34, 41))
            dpg.add_theme_color(dpg.mvThemeCol_Border, (54, 58, 68))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (228, 230, 235))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (40, 44, 53))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (52, 57, 68))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (60, 66, 78))
            dpg.add_theme_color(dpg.mvThemeCol_Button, (45, 96, 160))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (58, 120, 196))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (38, 82, 140))
            dpg.add_theme_color(dpg.mvThemeCol_Tab, (38, 42, 51))
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered, (58, 120, 196))
            dpg.add_theme_color(dpg.mvThemeCol_TabActive, (45, 96, 160))
            dpg.add_theme_color(dpg.mvThemeCol_Header, (45, 96, 160))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (58, 120, 196))
            dpg.add_theme_color(dpg.mvThemeCol_TableHeaderBg, (44, 48, 58))
            dpg.add_theme_color(dpg.mvThemeCol_TableBorderStrong, (62, 67, 79))
            dpg.add_theme_color(dpg.mvThemeCol_TableBorderLight, (46, 50, 60))
            dpg.add_theme_color(dpg.mvThemeCol_TableRowBg, (33, 36, 44))
            dpg.add_theme_color(dpg.mvThemeCol_TableRowBgAlt, (38, 42, 51))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarBg, (24, 26, 31))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrab, (60, 66, 78))
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)
            dpg.add_theme_style(dpg.mvStyleVar_TabRounding, 5)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FrameBorderSize, 1)
            dpg.add_theme_style(dpg.mvStyleVar_CellPadding, 8, 4)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 6)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 14, 12)
    dpg.bind_theme(global_theme)

    # Distinct accent for the freeze button while frozen.
    with dpg.theme() as frozen_btn_theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (176, 64, 64))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (200, 84, 84))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (150, 52, 52))

    frozen = [False]

    def toggle_freeze():
        frozen[0] = not frozen[0]
        if frozen[0]:
            dpg.configure_item("freeze_button", label="Unfreeze")
            dpg.bind_item_theme("freeze_button", frozen_btn_theme)
            dpg.configure_item("live_text", default_value="❚❚ FROZEN", color=(255, 170, 80))
        else:
            dpg.configure_item("freeze_button", label="Freeze")
            dpg.bind_item_theme("freeze_button", 0)
            dpg.configure_item("live_text", default_value="● LIVE", color=C_HIGH)

    TABLE_DEFS = [
        ("topics_tbl",
         ["Topic", "Type", "Rate (Hz)", "Bandwidth", "Delay (s)", "QoS", "Health",
          "Publisher Nodes", "Subscriber Nodes"]),
        ("services_tbl", ["Service", "Type", "Node"]),
        ("actions_tbl", ["Action", "Type", "Server Nodes"]),
        ("nodes_tbl", ["Node"]),
    ]

    def make_table(tag, columns):
        with dpg.table(tag=tag, header_row=True, borders_outerH=True,
                       borders_outerV=True, borders_innerH=True, borders_innerV=True,
                       policy=dpg.mvTable_SizingStretchProp, resizable=True,
                       row_background=True, scrollY=True, height=-1, freeze_rows=1):
            for col in columns:
                dpg.add_table_column(label=col)

    with dpg.window(tag="main_window", label="ROS 2 Overview", no_close=True,
                    no_move=True, no_resize=True):

        # ---- Header bar -----------------------------------------------------
        with dpg.group(horizontal=True):
            dpg.add_text("RUBI", color=C_ACCENT)
            dpg.add_text("ROS Utility Board Interface", color=C_MUTED)
            dpg.add_spacer(width=24)
            dpg.add_text("● LIVE", tag="live_text", color=C_HIGH)

        dpg.add_spacer(height=10)

        with dpg.group(horizontal=True, horizontal_spacing=16):
            dpg.add_button(label="Freeze", tag="freeze_button", callback=toggle_freeze, width=120)
            dpg.add_input_text(tag="global_search", hint="Search (all tabs)...", width=-1)

        dpg.add_spacer(height=8)
        dpg.add_separator()
        dpg.add_spacer(height=8)

        # ---- Legend (colors match the table cells exactly) ------------------
        with dpg.child_window(height=46, border=True):
            with dpg.group(horizontal=True, horizontal_spacing=20):
                dpg.add_text("Legend:", color=(200, 200, 200))
                dpg.add_text("< 1 Hz", color=C_LOW)
                dpg.add_text("1 - 10 Hz", color=C_MID)
                dpg.add_text("> 10 Hz", color=C_HIGH)
                dpg.add_text("|", color=(90, 95, 105))
                dpg.add_text("⚠ QoS mismatch", color=C_LOW)
                dpg.add_text("✓ / ✗ = health", color=C_MUTED)
                dpg.add_text("|", color=(90, 95, 105))
                dpg.add_text("NaN = no data", color=C_MUTED)
                dpg.add_text("0.0 = no msgs", color=C_MUTED)
                dpg.add_text("None = no nodes", color=C_MUTED)

        dpg.add_spacer(height=6)
        dpg.add_text("", tag="status_text", color=C_MUTED)
        dpg.add_spacer(height=6)

        with dpg.tab_bar(tag="main_tabs"):
            with dpg.tab(label="Topics"):
                make_table(*TABLE_DEFS[0])
            with dpg.tab(label="Services"):
                make_table(*TABLE_DEFS[1])
            with dpg.tab(label="Actions"):
                make_table(*TABLE_DEFS[2])
            with dpg.tab(label="Nodes"):
                make_table(*TABLE_DEFS[3])

    dpg.set_primary_window("main_window", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    # ------------------------------------------------------------------------
    # Flicker-free table syncing.
    #
    # `row_index` caches the ordered list of row keys currently shown in each
    # table. While the set of rows is unchanged (the common case, even as rates
    # tick) we only overwrite the per-cell values in place -> no flicker, scroll
    # position preserved. A full rebuild happens only when rows appear/disappear
    # or the search filter changes (rare).
    # ------------------------------------------------------------------------
    row_index = {}

    def sync_table(tag, prefix, desired):
        keys = [k for k, _ in desired]
        if row_index.get(tag) == keys:
            for key, cells in desired:
                for ci, (text, color) in enumerate(cells):
                    cell_tag = f"{prefix}|{key}|{ci}"
                    dpg.set_value(cell_tag, text)
                    dpg.configure_item(cell_tag, color=color or C_TEXT)
            return

        y = dpg.get_y_scroll(tag) if dpg.does_item_exist(tag) else 0.0
        for child in (dpg.get_item_children(tag, 1) or []):
            dpg.delete_item(child)
        for key, cells in desired:
            with dpg.table_row(parent=tag):
                for ci, (text, color) in enumerate(cells):
                    dpg.add_text(text, tag=f"{prefix}|{key}|{ci}", color=color or C_TEXT)
        row_index[tag] = keys
        dpg.set_y_scroll(tag, y)

    REFRESH_PERIOD = 0.25  # seconds (UI data refresh ~4 Hz; rendering stays smooth)
    last_refresh = 0.0

    while dpg.is_dearpygui_running():
        now = time.time()
        if not frozen[0] and now - last_refresh >= REFRESH_PERIOD:
            last_refresh = now
            search = (dpg.get_value("global_search") or "").lower()

            # Take a quick snapshot under the lock, then build the UI lock-free
            # so ROS callbacks are never starved.
            with node.lock:
                snap_topics = {
                    n: {'type': s['type'], 'rate': s['rate'], 'delay': s['delay'],
                        'bw': s['bw'], 'qos': s['qos'], 'qos_bad': s['qos_bad'],
                        'pubs': list(s['pubs']), 'subs': list(s['subs'])}
                    for n, s in node.topic_stats.items()
                }
                snap_services = {
                    n: {'type': s['type'], 'nodes': sorted(set(s['nodes']))}
                    for n, s in node.service_stats.items()
                }
                snap_actions = {
                    n: {'type': s['type'], 'nodes': sorted(set(s['nodes']))}
                    for n, s in node.action_stats.items()
                }
                snap_nodes = list(node.nodes)

            topic_rows = []
            health_fail = 0
            qos_bad_count = 0
            for name in sorted(snap_topics):
                s = snap_topics[name]
                if s['rate'] == 'N/A' and s['delay'] == 'N/A':
                    continue
                if search and search not in name.lower():
                    continue
                pubs = "\n".join(s['pubs']) if s['pubs'] else "None"
                subs = "\n".join(s['subs']) if s['subs'] else "None"
                qos_text = (s['qos'] + ' ⚠') if s['qos_bad'] else s['qos']
                qos_color = C_LOW if s['qos_bad'] else C_MUTED
                if s['qos_bad']:
                    qos_bad_count += 1
                health_label, health_color = topic_health(rules, name, s['rate'], s['delay'])
                if health_label == '✗ FAIL':
                    health_fail += 1
                topic_rows.append((name, [
                    (name, C_LOW if s['qos_bad'] else None),
                    (s['type'], None),
                    (s['rate'], _rate_color(s['rate'])),
                    (s['bw'], None),
                    (s['delay'], None),
                    (qos_text, qos_color),
                    (health_label, health_color),
                    (pubs, None),
                    (subs, None),
                ]))

            service_rows = []
            for name in sorted(snap_services):
                s = snap_services[name]
                if search and search not in name.lower():
                    continue
                nodes_str = "\n".join(s['nodes']) if s['nodes'] else "None"
                service_rows.append((name, [
                    (name, None), (s['type'], None), (nodes_str, None),
                ]))

            action_rows = []
            for name in sorted(snap_actions):
                s = snap_actions[name]
                if search and search not in name.lower():
                    continue
                nodes_str = "\n".join(s['nodes']) if s['nodes'] else "None"
                action_rows.append((name, [
                    (name, None), (s['type'], None), (nodes_str, None),
                ]))

            node_rows = []
            for name in sorted(snap_nodes):
                if search and search not in name.lower():
                    continue
                node_rows.append((name, [(name, None)]))

            sync_table("topics_tbl", "tp", topic_rows)
            sync_table("services_tbl", "sv", service_rows)
            sync_table("actions_tbl", "ac", action_rows)
            sync_table("nodes_tbl", "nd", node_rows)

            alerts = []
            if qos_bad_count:
                alerts.append(f"⚠ {qos_bad_count} QoS mismatch(es)")
            if health_fail:
                alerts.append(f"✗ {health_fail} health fail(s)")
            alert_str = ("        " + "   ".join(alerts)) if alerts else ""
            dpg.set_value(
                "status_text",
                f"Topics {len(topic_rows)}   ·   Services {len(service_rows)}   ·   "
                f"Actions {len(action_rows)}   ·   Nodes {len(node_rows)}"
                f"        Updated {time.strftime('%H:%M:%S')}{alert_str}"
            )
            dpg.configure_item("status_text", color=C_WARN if alerts else C_MUTED)

        dpg.render_dearpygui_frame()
        time.sleep(0.016)  # ~60 FPS for smooth scrolling

    dpg.destroy_context()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
