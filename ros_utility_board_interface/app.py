#!/usr/bin/env python3
"""RUBI - ROS Utility Board Interface (v2).

A single-window, lightweight ROS 2 control board: live topic rates, bandwidth,
delays, QoS mismatch detection, health watchdog, message inspector, /rosout
logs, parameters, lifecycle control, TF health, service/action caller, bag
recording, graph snapshot/diff, exports, and domain switching.
"""

import os
import sys
import time
import threading
import argparse

import rclpy
import dearpygui.dearpygui as dpg

from . import ops
from .ops import C_LOW, C_MID, C_HIGH, C_MUTED, C_ACCENT, C_WARN, C_TEXT, C_OK
from .node import SimpleMonitorNode

LOG_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
LOG_COLORS = {10: C_MUTED, 20: C_TEXT, 30: C_WARN, 40: C_LOW, 50: (255, 80, 80)}


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


# ---- pure row builders (no GUI; unit-testable) ----------------------------
def build_topic_rows(snap_topics, search, rules):
    rows, health_fail, qos_bad = [], 0, 0
    for name in sorted(snap_topics):
        s = snap_topics[name]
        if s['rate'] == 'N/A' and s['delay'] == 'N/A':
            continue
        if search and search not in name.lower():
            continue
        pubs = "\n".join(s['pubs']) if s['pubs'] else "None"
        subs = "\n".join(s['subs']) if s['subs'] else "None"
        qos_text = (s['qos'] + ' ⚠') if s['qos_bad'] else s['qos']
        if s['qos_bad']:
            qos_bad += 1
        hlabel, hcolor = ops.topic_health(rules, name, s['rate'], s['delay'])
        if hlabel == '✗ FAIL':
            health_fail += 1
        rows.append((name, [
            (name, C_LOW if s['qos_bad'] else None),
            (s['type'], None),
            (s['rate'], _rate_color(s['rate'])),
            (s['spark'], C_ACCENT),
            (s['bw'], None),
            (s['delay'], None),
            (qos_text, C_LOW if s['qos_bad'] else C_MUTED),
            (hlabel, hcolor),
            (pubs, None),
            (subs, None)]))
    return rows, health_fail, qos_bad


def build_entity_rows(snap, search):
    rows = []
    for name in sorted(snap):
        s = snap[name]
        if search and search not in name.lower():
            continue
        nodes_str = "\n".join(s['nodes']) if s['nodes'] else "None"
        rows.append((name, [(name, None), (s['type'], None), (nodes_str, None)]))
    return rows


def build_node_rows(snap_nodes, snap_lifecycle, proc_by_node, search):
    rows = []
    for name in sorted(snap_nodes):
        if search and search not in name.lower():
            continue
        lc = "lifecycle" if name in snap_lifecycle else "—"
        pid, cpu, rss = proc_by_node.get(name, (None, None, None))
        rows.append((name, [
            (name, None),
            (lc, C_ACCENT if lc == "lifecycle" else C_MUTED),
            (str(pid) if pid else "—", None),
            (f"{cpu:.0f}" if cpu is not None else "—", None),
            (ops.human_bytes(rss, suffix='') if rss else "—", None)]))
    return rows


def build_tf_rows(tf_snapshot, search, now):
    rows = []
    for child in sorted(tf_snapshot):
        f = tf_snapshot[child]
        if search and search not in child.lower():
            continue
        tsl = f['ts']
        if f['static']:
            rate_s, status, scol = "static", "OK", C_OK
        else:
            if len(tsl) >= 2 and tsl[-1] - tsl[0] > 0:
                rate_s = f"{(len(tsl) - 1) / (tsl[-1] - tsl[0]):.1f}"
            else:
                rate_s = "0.0"
            stale = (not tsl) or (now - tsl[-1] > 1.0)
            status, scol = ("STALE", C_LOW) if stale else ("OK", C_OK)
        rows.append((child, [(child, None), (f['parent'], None),
                             (rate_s, None), (status, scol)]))
    return rows


# ======================================================================
# GUI application
# ======================================================================
def main():
    parser = argparse.ArgumentParser(description='RUBI - ROS Utility Board Interface')
    parser.add_argument('--rules', default='rubi_rules.yaml',
                        help='YAML health-watchdog rules (default: ./rubi_rules.yaml)')
    parser.add_argument('--domain', type=int, default=None,
                        help='ROS_DOMAIN_ID to launch on')
    args, _ = parser.parse_known_args()

    if args.domain is not None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rules = ops.load_rules(args.rules)
    if rules:
        print(f"[RUBI] Loaded {len(rules)} health rule(s) from {args.rules}")

    rclpy.init()
    node = SimpleMonitorNode()

    stop_event = threading.Event()

    def spin_thread():
        while not stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    spinner = threading.Thread(target=spin_thread, daemon=True)
    spinner.start()
    time.sleep(1.5)

    def shutdown():
        stop_event.set()
        spinner.join(timeout=1.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    dpg.create_context()

    # Load a Unicode-capable font AND the glyph ranges RUBI actually uses
    # (em dash, ✓/✗, ⚠, ● ❚, and the ▁▂▃ sparkline blocks). Without these
    # ranges Dear PyGui only loads basic Latin and renders everything else
    # as '?'.
    font_candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
    ]
    with dpg.font_registry():
        for path in font_candidates:
            if not os.path.isfile(path):
                continue
            try:
                with dpg.font(path, 20) as ft:
                    dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
                    dpg.add_font_range(0x2010, 0x2060)   # punctuation incl. em dash
                    dpg.add_font_range(0x2500, 0x27BF)   # blocks, shapes, symbols, dingbats
                dpg.bind_font(ft)
                break
            except Exception:
                pass

    _build_theme()
    state = {'snapshot': None, 'param_types': {}, 'services': {}, 'actions': {},
             'plot_lines': {}}

    # ---------------------------------------------------------------- helpers
    def run_async(fn):
        threading.Thread(target=fn, daemon=True).start()

    # ---- domain switch (re-exec the process on the new domain) ----
    def switch_domain():
        try:
            d = int(dpg.get_value("domain_input"))
        except (ValueError, TypeError):
            dpg.set_value("status_text", "Invalid domain id")
            return
        os.environ['ROS_DOMAIN_ID'] = str(d)
        argv = [a for a in sys.argv if not a.startswith('--domain')]
        try:
            dpg.stop_dearpygui()
        except Exception:
            pass
        os.execvpe(sys.executable, [sys.executable] + argv + [f'--domain={d}'], os.environ)

    # ---- exports ----
    def collect_topic_rows_for_export():
        rows = []
        with node.lock:
            for name in sorted(node.topic_stats):
                s = node.topic_stats[name]
                if s['rate'] == 'N/A' and s['delay'] == 'N/A':
                    continue
                rows.append({'name': name, 'type': s['type'], 'rate': s['rate'],
                             'bw': s['bw'], 'delay': s['delay'], 'qos': s['qos'],
                             'pubs': list(s['pubs']) or ['None'],
                             'subs': list(s['subs']) or ['None']})
        return rows

    def do_export(kind):
        rows = collect_topic_rows_for_export()
        stamp = time.strftime('%Y%m%d_%H%M%S')
        try:
            if kind == 'csv':
                p = ops.export_topics_csv(os.path.join('CSV', f'rubi_topics_{stamp}.csv'), rows)
            elif kind == 'md':
                p = ops.export_topics_markdown(os.path.join('MD', f'rubi_topics_{stamp}.md'), rows)
            else:
                p = ops.export_graph_dot(os.path.join('DOT', f'rubi_graph_{stamp}.dot'), rows)
            dpg.set_value("status_text", f"Exported {os.path.abspath(p)}")
            dpg.configure_item("status_text", color=C_OK)
        except Exception as e:
            dpg.set_value("status_text", f"Export failed: {e}")
            dpg.configure_item("status_text", color=C_LOW)

    # ---- snapshot / diff ----
    def capture_snapshot():
        with node.lock:
            state['snapshot'] = {
                'nodes': set(node.nodes),
                'topics': set(node.topic_stats),
                'services': set(node.service_stats),
                'actions': set(node.action_stats)}
        dpg.set_value("snap_output",
                      f"Snapshot captured at {time.strftime('%H:%M:%S')}\n"
                      f"  {len(state['snapshot']['nodes'])} nodes, "
                      f"{len(state['snapshot']['topics'])} topics, "
                      f"{len(state['snapshot']['services'])} services, "
                      f"{len(state['snapshot']['actions'])} actions")

    def diff_snapshot():
        if not state['snapshot']:
            dpg.set_value("snap_output", "No snapshot captured yet.")
            return
        with node.lock:
            cur = {'nodes': set(node.nodes), 'topics': set(node.topic_stats),
                   'services': set(node.service_stats), 'actions': set(node.action_stats)}
        lines = []
        for key in ('nodes', 'topics', 'services', 'actions'):
            added, removed = ops.diff_sets(state['snapshot'][key], cur[key])
            lines.append(f"== {key} ==")
            for a in added:
                lines.append(f"  + {a}")
            for r in removed:
                lines.append(f"  - {r}")
            if not added and not removed:
                lines.append("  (no change)")
        dpg.set_value("snap_output", "\n".join(lines))

    # ---- inspector ----
    def peek_message():
        topic = dpg.get_value("inspect_topic")
        if not topic:
            return
        with node.lock:
            s = node.topic_stats.get(topic)
            msg = s['last_msg'] if s else None
        if msg is None:
            dpg.set_value("inspect_output", "(no message received yet)")
            return
        try:
            from rosidl_runtime_py import message_to_yaml
            dpg.set_value("inspect_output", message_to_yaml(msg))
        except Exception as e:
            dpg.set_value("inspect_output", f"(error: {e})")

    # ---- parameters ----
    def on_param_select():
        name = dpg.get_value("param_name")
        t = state['param_types'].get(name, '')
        dpg.configure_item("param_value", hint=f"new value ...({t})" if t else "new value")

    def load_params():
        n = dpg.get_value("param_node")
        if not n:
            return
        dpg.set_value("param_status", f"Loading parameters of {n} ...")

        def work():
            params = ops.list_node_parameters(n)
            for child in (dpg.get_item_children("params_tbl", 1) or []):
                dpg.delete_item(child)
            for pname, ptype, pval in params:
                with dpg.table_row(parent="params_tbl"):
                    dpg.add_text(pname)
                    dpg.add_text(ptype, color=C_MUTED)
                    dpg.add_text(pval, color=C_ACCENT)
            names = [p[0] for p in params]
            state['param_types'] = {p[0]: p[1] for p in params}
            dpg.configure_item("param_name", items=names)
            dpg.set_value("param_name", names[0] if names else "")
            on_param_select()
            dpg.set_value("param_status",
                          f"{len(params)} parameter(s)" if params else "no parameters / unreachable")
        run_async(work)

    def set_param():
        n = dpg.get_value("param_node")
        name = (dpg.get_value("param_name") or "").strip()
        val = dpg.get_value("param_value")
        if not n or not name:
            dpg.set_value("param_status", "Pick a node and a parameter")
            return

        def work():
            ok, msg = ops.set_node_parameter(n, name, val)
            dpg.set_value("param_status", ("✓ " if ok else "✗ ") + msg)
            dpg.configure_item("param_status", color=C_OK if ok else C_LOW)
            if ok:
                load_params()
        run_async(work)

    # ---- lifecycle ----
    def refresh_lifecycle():
        with node.lock:
            lnodes = sorted(node.lifecycle_nodes)
        for child in (dpg.get_item_children("lifecycle_tbl", 1) or []):
            dpg.delete_item(child)
        if not lnodes:
            with dpg.table_row(parent="lifecycle_tbl"):
                dpg.add_text("(no lifecycle nodes detected)", color=C_MUTED)
            return
        dpg.set_value("lifecycle_status", "Querying states ...")

        def work():
            for ln in lnodes:
                state_label = ops.get_lifecycle_state(ln) or '?'
                with dpg.table_row(parent="lifecycle_tbl"):
                    dpg.add_text(ln)
                    color = C_OK if state_label == 'active' else (
                        C_WARN if state_label in ('inactive', 'unconfigured') else C_MUTED)
                    dpg.add_text(state_label, color=color)
                    with dpg.group(horizontal=True):
                        for label, tid in ops.LIFECYCLE_TRANSITIONS:
                            dpg.add_button(
                                label=label, small=True,
                                callback=lambda s, a, u: do_transition(*u),
                                user_data=(ln, tid))
            dpg.set_value("lifecycle_status", f"{len(lnodes)} lifecycle node(s)")
        run_async(work)

    def do_transition(node_fqn, tid):
        dpg.set_value("lifecycle_status", f"Transitioning {node_fqn} ...")

        def work():
            ok, msg = ops.change_lifecycle_state(node_fqn, tid)
            dpg.set_value("lifecycle_status", ("✓ " if ok else "✗ ") + f"{node_fqn}: {msg}")
            dpg.configure_item("lifecycle_status", color=C_OK if ok else C_LOW)
            time.sleep(0.3)
            refresh_lifecycle()
        run_async(work)

    # ---- service / action caller ----
    def on_call_mode():
        """Repopulate the name dropdown when Service/Action is toggled."""
        is_action = dpg.get_value("call_mode") == "Action"
        names = sorted(state['actions'] if is_action else state['services'])
        dpg.configure_item("call_name", items=names)
        dpg.set_value("call_name", "")
        dpg.set_value("call_type_display", "(select a name)")
        dpg.set_value("call_request", "")

    def on_call_name():
        is_action = dpg.get_value("call_mode") == "Action"
        name = dpg.get_value("call_name")
        registry = state['actions'] if is_action else state['services']
        type_str = registry.get(name, '')
        dpg.set_value("call_type_display", f"type: {type_str}" if type_str else "")
        dpg.set_value("call_request", ops.request_skeleton(type_str, is_action))

    def do_call():
        is_action = dpg.get_value("call_mode") == "Action"
        name = dpg.get_value("call_name")
        registry = state['actions'] if is_action else state['services']
        type_str = registry.get(name, '')
        req = dpg.get_value("call_request")
        if not name or not type_str:
            dpg.set_value("call_output", "Select a service/action from the list.")
            return
        dpg.set_value("call_output", "Calling ...")

        def work():
            if is_action:
                ok, txt = ops.send_action_goal(type_str, name, req)
            else:
                ok, txt = ops.call_service(type_str, name, req)
            dpg.set_value("call_output", ("✓ OK\n" if ok else "✗ FAILED\n") + txt)
            dpg.configure_item("call_output", color=C_OK if ok else C_LOW)
        run_async(work)

    # ---- rosbag record / play / info ----
    recorder = ops.BagRecorder()
    player = ops.BagPlayer()

    def toggle_record():
        if recorder.recording:
            ok, msg = recorder.stop()
            dpg.configure_item("record_btn", label="Start Recording")
            dpg.bind_item_theme("record_btn", 0)
            refresh_bag_list()
        else:
            ok, msg = recorder.start(dpg.get_value("rec_topics"), dpg.get_value("rec_outdir"))
            if ok:
                dpg.configure_item("record_btn", label="Stop Recording")
                dpg.bind_item_theme("record_btn", state['danger_theme'])
        dpg.set_value("record_status", ("✓ " if ok else "✗ ") + msg)
        dpg.configure_item("record_status", color=C_OK if ok else C_LOW)

    def toggle_play():
        if player.playing:
            ok, msg = player.stop()
            dpg.configure_item("play_btn", label="Play")
            dpg.bind_item_theme("play_btn", 0)
        else:
            ok, msg = player.start(dpg.get_value("play_path"),
                                   dpg.get_value("play_rate"), dpg.get_value("play_loop"))
            if ok:
                dpg.configure_item("play_btn", label="Stop")
                dpg.bind_item_theme("play_btn", state['danger_theme'])
        dpg.set_value("play_status", ("✓ " if ok else "✗ ") + msg)
        dpg.configure_item("play_status", color=C_OK if ok else C_LOW)

    def refresh_bag_list():
        bags = ops.list_bags()
        dpg.configure_item("bag_select", items=bags)
        dpg.set_value("bag_list_status", f"{len(bags)} bag(s) in {ops.BAG_DIR}/")

    def on_bag_select():
        p = dpg.get_value("bag_select")
        dpg.set_value("play_path", p)
        dpg.set_value("info_path", p)

    def show_bag_info():
        path = dpg.get_value("info_path")
        dpg.set_value("bag_info_output", "Loading ...")
        run_async(lambda: dpg.set_value("bag_info_output", ops.bag_info(path)))

    # ---- live plot ----
    def add_plot_line(key):
        topic, fpath = key
        tag = f"series|{topic}|{fpath}"
        if not dpg.does_item_exist(tag):
            dpg.add_line_series([], [], label=f"{topic} : {fpath}",
                                parent="plot_y", tag=tag)
        state['plot_lines'][key] = tag

    def remove_plot_line(key):
        tag = state['plot_lines'].pop(key, None)
        if tag and dpg.does_item_exist(tag):
            dpg.delete_item(tag)

    def on_field_toggle(sender, value, user_data):
        topic, fpath = user_data
        if value:
            node.add_plot_field(topic, fpath)
            add_plot_line((topic, fpath))
        else:
            node.remove_plot_field(topic, fpath)
            remove_plot_line((topic, fpath))

    def load_plot_fields(*_):
        topic = dpg.get_value("plot_topic")
        for child in (dpg.get_item_children("plot_fields", 1) or []):
            dpg.delete_item(child)
        if not topic:
            return
        with node.lock:
            s = node.topic_stats.get(topic)
            msg = s['last_msg'] if s else None
            cls = node.msg_classes.get(topic)
        if msg is None and cls is not None:
            try:
                msg = cls()
            except Exception:
                msg = None
        fields = ops.numeric_fields(msg) if msg is not None else []
        if not fields:
            dpg.add_text("(no numeric fields / no data yet)",
                         parent="plot_fields", color=C_MUTED)
            return
        for fpath in fields:
            key = (topic, fpath)
            dpg.add_checkbox(label=fpath, parent="plot_fields",
                             default_value=key in state['plot_lines'],
                             callback=on_field_toggle, user_data=key)

    def clear_plot():
        for key in list(state['plot_lines']):
            node.remove_plot_field(*key)
            remove_plot_line(key)
        load_plot_fields()

    def on_plot_follow(sender, value):
        if not value:
            dpg.set_axis_limits_auto("plot_x")

    def update_plot():
        if not state['plot_lines']:
            return
        now = time.time()
        try:
            window = max(1.0, float(dpg.get_value("plot_window")))
        except (ValueError, TypeError):
            window = 20.0
        with node.lock:
            data = {k: list(node.plot_series[k]) for k in state['plot_lines']
                    if k in node.plot_series}
        for key, tag in state['plot_lines'].items():
            xs, ys = [], []
            for t, v in data.get(key, []):
                dt = t - now
                if dt >= -window:
                    xs.append(dt)
                    ys.append(v)
            if dpg.does_item_exist(tag):
                dpg.set_value(tag, [xs, ys])
        if dpg.get_value("plot_follow"):
            dpg.set_axis_limits("plot_x", -window, 0.5)
        if dpg.get_value("plot_auto_y"):
            dpg.fit_axis_data("plot_y")

    # ---------------------------------------------------------------- layout
    state['danger_theme'] = _danger_theme()
    frozen = [False]

    def toggle_freeze():
        frozen[0] = not frozen[0]
        if frozen[0]:
            dpg.configure_item("freeze_button", label="Unfreeze")
            dpg.bind_item_theme("freeze_button", state['danger_theme'])
            dpg.configure_item("live_text", default_value="❚❚ FROZEN", color=C_WARN)
        else:
            dpg.configure_item("freeze_button", label="Freeze")
            dpg.bind_item_theme("freeze_button", 0)
            dpg.configure_item("live_text", default_value="● LIVE", color=C_HIGH)

    def make_table(tag, columns, **kw):
        with dpg.table(tag=tag, header_row=True, borders_outerH=True,
                       borders_outerV=True, borders_innerH=True, borders_innerV=True,
                       policy=dpg.mvTable_SizingStretchProp, resizable=True,
                       row_background=True, scrollY=True, height=-1, freeze_rows=1, **kw):
            for col in columns:
                dpg.add_table_column(label=col)

    with dpg.window(tag="main_window", no_close=True, no_move=True, no_resize=True):
        # Header
        with dpg.group(horizontal=True):
            dpg.add_text("RUBI", color=C_ACCENT)
            dpg.add_text("ROS Utility Board Interface  v2", color=C_MUTED)
            dpg.add_spacer(width=18)
            dpg.add_text("● LIVE", tag="live_text", color=C_HIGH)
            dpg.add_spacer(width=24)
            dpg.add_text(f"Domain {os.environ.get('ROS_DOMAIN_ID', '0')}", color=C_MUTED)
            dpg.add_input_int(tag="domain_input", width=110,
                              default_value=int(os.environ.get('ROS_DOMAIN_ID', '0') or 0))
            dpg.add_button(label="Switch", callback=switch_domain)
            dpg.add_spacer(width=18)
            dpg.add_button(label="Export CSV", callback=lambda: do_export('csv'))
            dpg.add_button(label="Export MD", callback=lambda: do_export('md'))
            dpg.add_button(label="Export DOT", callback=lambda: do_export('dot'))

        dpg.add_spacer(height=8)
        with dpg.group(horizontal=True, horizontal_spacing=16):
            dpg.add_button(label="Freeze", tag="freeze_button", callback=toggle_freeze, width=120)
            dpg.add_input_text(tag="global_search", hint="Search (all tabs)...", width=-1)

        dpg.add_spacer(height=8)
        dpg.add_separator()
        dpg.add_spacer(height=8)

        with dpg.child_window(height=80, border=True):
            with dpg.group(horizontal=True, horizontal_spacing=18):
                dpg.add_text("Legend:", color=(200, 200, 200))
                dpg.add_text("< 1 Hz", color=C_LOW)
                dpg.add_text("1 - 10 Hz", color=C_MID)
                dpg.add_text("> 10 Hz", color=C_HIGH)
                dpg.add_text("|", color=(90, 95, 105))
                dpg.add_text("⚠ QoS mismatch", color=C_LOW)
                dpg.add_text("✓/✗ health", color=C_MUTED)
                dpg.add_text("|", color=(90, 95, 105))
                dpg.add_text("NaN=no data", color=C_MUTED)
                dpg.add_text("0.0=no msgs", color=C_MUTED)
                dpg.add_text("None=no nodes", color=C_MUTED)
            with dpg.group(horizontal=True, horizontal_spacing=14):
                dpg.add_text("QoS:", color=(200, 200, 200))
                dpg.add_text("reliability·durability", color=C_MUTED)
                dpg.add_text("|", color=(90, 95, 105))
                dpg.add_text("R=Reliable", color=C_ACCENT)
                dpg.add_text("BE=Best-Effort", color=C_ACCENT)
                dpg.add_text("V=Volatile", color=C_ACCENT)
                dpg.add_text("TL=Transient-Local", color=C_ACCENT)
                dpg.add_text("BA=Best-Available", color=C_MUTED)
                dpg.add_text("SD=System-Default", color=C_MUTED)

        dpg.add_spacer(height=6)
        dpg.add_text("", tag="status_text", color=C_MUTED)
        dpg.add_spacer(height=6)

        with dpg.tab_bar(tag="main_tabs"):
            with dpg.tab(label="Topics"):
                make_table("topics_tbl",
                           ["Topic", "Type", "Rate (Hz)", "Trend", "Bandwidth",
                            "Delay (s)", "QoS", "Health", "Publishers", "Subscribers"])
            with dpg.tab(label="Services"):
                make_table("services_tbl", ["Service", "Type", "Node"])
            with dpg.tab(label="Actions"):
                make_table("actions_tbl", ["Action", "Type", "Server Nodes"])
            with dpg.tab(label="Nodes"):
                make_table("nodes_tbl", ["Node", "Lifecycle", "PID", "CPU %", "Memory"])

            with dpg.tab(label="Inspect"):
                with dpg.group(horizontal=True):
                    dpg.add_text("Topic:")
                    dpg.add_combo([], tag="inspect_topic", width=460)
                    dpg.add_button(label="Peek latest", callback=peek_message)
                    dpg.add_checkbox(label="live", tag="inspect_live")
                dpg.add_spacer(height=6)
                with dpg.child_window(height=-1, border=True):
                    dpg.add_text("(select a topic and click Peek)", tag="inspect_output", wrap=0)

            with dpg.tab(label="Logs"):
                with dpg.group(horizontal=True):
                    dpg.add_text("Min level:")
                    dpg.add_combo(list(LOG_LEVELS.values()), default_value="INFO",
                                  tag="log_level", width=140)
                    dpg.add_button(label="Clear", callback=lambda: node.logs.clear())
                dpg.add_spacer(height=6)
                dpg.add_child_window(tag="log_child", height=-1, border=True)

            with dpg.tab(label="Params"):
                with dpg.group(horizontal=True):
                    dpg.add_text("Node:")
                    dpg.add_combo([], tag="param_node", width=400, callback=load_params)
                    dpg.add_button(label="Reload", callback=load_params)
                    dpg.add_text("", tag="param_status", color=C_MUTED)
                dpg.add_spacer(height=4)
                with dpg.group(horizontal=True):
                    dpg.add_text("Set:")
                    dpg.add_combo([], tag="param_name", width=260, callback=on_param_select)
                    dpg.add_input_text(tag="param_value", hint="new value", width=300)
                    dpg.add_button(label="Apply", callback=set_param)
                dpg.add_spacer(height=6)
                with dpg.table(tag="params_tbl", header_row=True, borders_innerH=True,
                               borders_outerH=True, borders_outerV=True,
                               policy=dpg.mvTable_SizingStretchProp, resizable=True,
                               row_background=True, scrollY=True, height=-1, freeze_rows=1):
                    dpg.add_table_column(label="Parameter")
                    dpg.add_table_column(label="Type")
                    dpg.add_table_column(label="Value")

            with dpg.tab(label="Lifecycle"):
                with dpg.group(horizontal=True):
                    dpg.add_button(label="Refresh states", callback=refresh_lifecycle)
                    dpg.add_text("", tag="lifecycle_status", color=C_MUTED)
                dpg.add_spacer(height=6)
                with dpg.table(tag="lifecycle_tbl", header_row=True, borders_innerH=True,
                               borders_outerH=True, borders_outerV=True,
                               policy=dpg.mvTable_SizingStretchProp, resizable=True,
                               row_background=True, scrollY=True, height=-1, freeze_rows=1):
                    dpg.add_table_column(label="Lifecycle Node")
                    dpg.add_table_column(label="State")
                    dpg.add_table_column(label="Transitions")

            with dpg.tab(label="TF"):
                make_table("tf_tbl", ["Child Frame", "Parent Frame", "Rate (Hz)", "Status"])

            with dpg.tab(label="Call"):
                with dpg.group(horizontal=True):
                    dpg.add_radio_button(["Service", "Action"], tag="call_mode",
                                         horizontal=True, default_value="Service",
                                         callback=on_call_mode)
                with dpg.group(horizontal=True):
                    dpg.add_text("Name:")
                    dpg.add_combo([], tag="call_name", width=560, callback=on_call_name)
                dpg.add_text("(select a name)", tag="call_type_display", color=C_ACCENT)
                dpg.add_spacer(height=4)
                dpg.add_text("Request / Goal (YAML) — pre-filled from the type, edit as needed:",
                             color=C_MUTED)
                dpg.add_input_text(tag="call_request", multiline=True, width=-1, height=140)
                dpg.add_button(label="Call", callback=do_call)
                dpg.add_spacer(height=6)
                with dpg.child_window(height=-1, border=True):
                    dpg.add_text("(response appears here)", tag="call_output", wrap=0)

            with dpg.tab(label="Rosbag"):
                dpg.add_text("Record, play and inspect rosbag2 files. "
                             "All recordings are saved under the Bag/ folder.", color=C_MUTED)
                dpg.add_spacer(height=6)

                with dpg.collapsing_header(label="Record", default_open=True):
                    dpg.add_input_text(tag="rec_topics", width=-1, default_value="-a",
                                       hint="-a for all, or space-separated topic names")
                    dpg.add_input_text(tag="rec_outdir", width=-1,
                                       hint="bag name (optional, saved under Bag/)")
                    dpg.add_spacer(height=4)
                    dpg.add_button(label="Start Recording", tag="record_btn",
                                   callback=toggle_record, width=200)
                    dpg.add_text("", tag="record_status", color=C_MUTED)

                dpg.add_spacer(height=6)
                with dpg.group(horizontal=True):
                    dpg.add_text("Detected bags:")
                    dpg.add_combo([], tag="bag_select", width=420, callback=on_bag_select)
                    dpg.add_button(label="Refresh", callback=refresh_bag_list)
                    dpg.add_text("", tag="bag_list_status", color=C_MUTED)

                with dpg.collapsing_header(label="Play", default_open=True):
                    dpg.add_input_text(tag="play_path", width=-1,
                                       hint="bag path (pick from Detected bags or type a path)")
                    with dpg.group(horizontal=True):
                        dpg.add_text("Rate:")
                        dpg.add_input_text(tag="play_rate", width=80, default_value="1.0")
                        dpg.add_checkbox(label="loop", tag="play_loop")
                        dpg.add_button(label="Play", tag="play_btn", callback=toggle_play, width=120)
                    dpg.add_text("", tag="play_status", color=C_MUTED)

                with dpg.collapsing_header(label="Info", default_open=True):
                    with dpg.group(horizontal=True):
                        dpg.add_input_text(tag="info_path", width=-1,
                                           hint="bag path for `ros2 bag info`")
                        dpg.add_button(label="Get info", callback=show_bag_info)
                    with dpg.child_window(height=220, border=True):
                        dpg.add_text("(bag info appears here)", tag="bag_info_output", wrap=0)

            with dpg.tab(label="Snapshot"):
                dpg.add_text("Capture the current graph, then diff it later to catch "
                             "intermittent nodes/topics.", color=C_MUTED)
                dpg.add_spacer(height=6)
                with dpg.group(horizontal=True):
                    dpg.add_button(label="Capture snapshot", callback=capture_snapshot)
                    dpg.add_button(label="Diff vs now", callback=diff_snapshot)
                dpg.add_spacer(height=6)
                with dpg.child_window(height=-1, border=True):
                    dpg.add_text("(no snapshot yet)", tag="snap_output", wrap=0)

            with dpg.tab(label="Plot"):
                with dpg.group(horizontal=True):
                    dpg.add_text("Topic:")
                    dpg.add_combo([], tag="plot_topic", width=360, callback=load_plot_fields)
                    dpg.add_button(label="Reload", callback=load_plot_fields)
                    dpg.add_text("Window (s):")
                    dpg.add_input_text(tag="plot_window", width=70, default_value="20")
                    dpg.add_checkbox(label="follow", tag="plot_follow",
                                     default_value=True, callback=on_plot_follow)
                    dpg.add_checkbox(label="auto-Y", tag="plot_auto_y", default_value=True)
                    dpg.add_button(label="Clear all", callback=clear_plot)
                dpg.add_spacer(height=6)
                with dpg.group(horizontal=True):
                    with dpg.child_window(width=320, height=-1, border=True):
                        dpg.add_text("Numeric fields  (tick to plot)", color=C_MUTED)
                        dpg.add_separator()
                        dpg.add_group(tag="plot_fields")
                    with dpg.plot(label="Live plot", height=-1, width=-1, tag="live_plot"):
                        dpg.add_plot_legend()
                        dpg.add_plot_axis(dpg.mvXAxis, label="time (s, 0 = now)", tag="plot_x")
                        dpg.add_plot_axis(dpg.mvYAxis, label="value", tag="plot_y")

    if os.environ.get('RUBI_SELFTEST'):
        print('[selftest] full UI tree built without error')
        shutdown()
        dpg.destroy_context()
        return

    dpg.create_viewport(title='RUBI (ROS Utility Board Interface)',
                        width=1550, height=1050, min_width=960, min_height=600,
                        resizable=True)
    dpg.set_primary_window("main_window", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    refresh_bag_list()

    # ------------------------------------------------------------ live tables
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

    log_sig = [None]

    def refresh_logs(search):
        with node.lock:
            logs = list(node.logs)
        min_level = {v: k for k, v in LOG_LEVELS.items()}.get(dpg.get_value("log_level"), 20)
        shown = [(lvl, name, msg) for lvl, name, msg, _ in logs
                 if lvl >= min_level and (not search or search in name.lower()
                                          or search in msg.lower())][-300:]
        sig = (len(logs), min_level, search, len(shown))
        if sig == log_sig[0]:
            return
        log_sig[0] = sig
        for child in (dpg.get_item_children("log_child", 1) or []):
            dpg.delete_item(child)
        for lvl, name, msg in shown:
            dpg.add_text(f"[{LOG_LEVELS.get(lvl, '?'):5s}] [{name}] {msg}",
                         parent="log_child", color=LOG_COLORS.get(lvl, C_TEXT), wrap=0)
        dpg.set_y_scroll("log_child", -1.0)

    REFRESH_PERIOD = 0.25
    last_refresh = 0.0

    while dpg.is_dearpygui_running():
        now = time.time()
        if not frozen[0] and now - last_refresh >= REFRESH_PERIOD:
            last_refresh = now
            search = (dpg.get_value("global_search") or "").lower()

            with node.lock:
                snap_topics = {
                    n: {'type': s['type'], 'rate': s['rate'], 'delay': s['delay'],
                        'bw': s['bw'], 'qos': s['qos'], 'qos_bad': s['qos_bad'],
                        'spark': ops.spark(s['rate_hist']),
                        'pubs': list(s['pubs']), 'subs': list(s['subs'])}
                    for n, s in node.topic_stats.items()}
                snap_services = {n: {'type': s['type'], 'nodes': sorted(set(s['nodes']))}
                                 for n, s in node.service_stats.items()}
                snap_actions = {n: {'type': s['type'], 'nodes': sorted(set(s['nodes']))}
                                for n, s in node.action_stats.items()}
                snap_nodes = list(node.nodes)
                snap_lifecycle = set(node.lifecycle_nodes)
                proc_by_node = dict(node.proc_mon.by_node)
                tf_snapshot = {child: {'parent': f['parent'], 'static': f['static'],
                                       'ts': list(f['ts'])}
                               for child, f in node.tf_frames.items()}

            topic_rows, health_fail, qos_bad_count = build_topic_rows(snap_topics, search, rules)
            service_rows = build_entity_rows(snap_services, search)
            action_rows = build_entity_rows(snap_actions, search)
            node_rows = build_node_rows(snap_nodes, snap_lifecycle, proc_by_node, search)
            tf_rows = build_tf_rows(tf_snapshot, search, now)

            sync_table("topics_tbl", "tp", topic_rows)
            sync_table("services_tbl", "sv", service_rows)
            sync_table("actions_tbl", "ac", action_rows)
            sync_table("nodes_tbl", "nd", node_rows)
            sync_table("tf_tbl", "tf", tf_rows)
            refresh_logs(search)

            # combos / caller registries
            dpg.configure_item("inspect_topic", items=sorted(snap_topics))
            dpg.configure_item("plot_topic", items=sorted(snap_topics))
            dpg.configure_item("param_node", items=snap_nodes)
            state['services'] = {n: snap_services[n]['type'] for n in snap_services}
            state['actions'] = {n: snap_actions[n]['type'] for n in snap_actions}
            call_is_action = dpg.get_value("call_mode") == "Action"
            dpg.configure_item(
                "call_name",
                items=sorted(state['actions'] if call_is_action else state['services']))

            if dpg.get_value("inspect_live"):
                peek_message()

            alerts = []
            if qos_bad_count:
                alerts.append(f"⚠ {qos_bad_count} QoS mismatch(es)")
            if health_fail:
                alerts.append(f"✗ {health_fail} health fail(s)")
            alert_str = ("        " + "   ".join(alerts)) if alerts else ""
            dpg.set_value("status_text",
                          f"Topics {len(topic_rows)}   ·   Services {len(service_rows)}   ·   "
                          f"Actions {len(action_rows)}   ·   Nodes {len(node_rows)}   ·   "
                          f"TF {len(tf_rows)}        Updated {time.strftime('%H:%M:%S')}{alert_str}")
            dpg.configure_item("status_text", color=C_WARN if alerts else C_MUTED)

        # the live plot updates every frame (data is collected at message rate)
        if not frozen[0]:
            update_plot()

        dpg.render_dearpygui_frame()
        time.sleep(0.016)

    shutdown()
    dpg.destroy_context()


# ======================================================================
# Theme
# ======================================================================
def _build_theme():
    with dpg.theme() as global_theme:
        with dpg.theme_component(dpg.mvAll):
            c = dpg.add_theme_color
            c(dpg.mvThemeCol_WindowBg, (24, 26, 31))
            c(dpg.mvThemeCol_ChildBg, (31, 34, 41))
            c(dpg.mvThemeCol_Border, (54, 58, 68))
            c(dpg.mvThemeCol_Text, (228, 230, 235))
            c(dpg.mvThemeCol_FrameBg, (40, 44, 53))
            c(dpg.mvThemeCol_FrameBgHovered, (52, 57, 68))
            c(dpg.mvThemeCol_FrameBgActive, (60, 66, 78))
            c(dpg.mvThemeCol_Button, (45, 96, 160))
            c(dpg.mvThemeCol_ButtonHovered, (58, 120, 196))
            c(dpg.mvThemeCol_ButtonActive, (38, 82, 140))
            c(dpg.mvThemeCol_Tab, (38, 42, 51))
            c(dpg.mvThemeCol_TabHovered, (58, 120, 196))
            c(dpg.mvThemeCol_TabActive, (45, 96, 160))
            c(dpg.mvThemeCol_Header, (45, 96, 160))
            c(dpg.mvThemeCol_HeaderHovered, (58, 120, 196))
            c(dpg.mvThemeCol_TableHeaderBg, (44, 48, 58))
            c(dpg.mvThemeCol_TableBorderStrong, (62, 67, 79))
            c(dpg.mvThemeCol_TableBorderLight, (46, 50, 60))
            c(dpg.mvThemeCol_TableRowBg, (33, 36, 44))
            c(dpg.mvThemeCol_TableRowBgAlt, (38, 42, 51))
            c(dpg.mvThemeCol_ScrollbarBg, (24, 26, 31))
            c(dpg.mvThemeCol_ScrollbarGrab, (60, 66, 78))
            s = dpg.add_theme_style
            s(dpg.mvStyleVar_WindowRounding, 6)
            s(dpg.mvStyleVar_ChildRounding, 6)
            s(dpg.mvStyleVar_FrameRounding, 5)
            s(dpg.mvStyleVar_TabRounding, 5)
            s(dpg.mvStyleVar_ScrollbarRounding, 6)
            s(dpg.mvStyleVar_FrameBorderSize, 1)
            s(dpg.mvStyleVar_CellPadding, 8, 4)
            s(dpg.mvStyleVar_ItemSpacing, 8, 6)
            s(dpg.mvStyleVar_WindowPadding, 14, 12)
    dpg.bind_theme(global_theme)


def _danger_theme():
    with dpg.theme() as t:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (176, 64, 64))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (200, 84, 84))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (150, 52, 52))
    return t


if __name__ == '__main__':
    main()
