#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import dearpygui.dearpygui as dpg
import threading
import time
import importlib
from collections import deque
import os

class SimpleMonitorNode(Node):
    def __init__(self):
        super().__init__('ros_utility_board_interface')

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
        with self.lock:
            stats = self.topic_stats.get(topic)
            if stats:
                stats['timestamps'].append(now)
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
                self.nodes = sorted([f"{ns}{name}" for name, ns in node_names])

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
                            'rate': 'NaN',
                            'delay': 'NaN'
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
                        if name in self._my_subscriptions:
                            self.destroy_subscription(self._my_subscriptions[name])
                            del self._my_subscriptions[name]
                        continue

                    pubs = self.get_publishers_info_by_topic(name)
                    s['pubs'] = sorted({f"{p.node_namespace}{p.node_name}" for p in pubs})

                    subs = self.get_subscriptions_info_by_topic(name)
                    s['subs'] = sorted({f"{su.node_namespace}{su.node_name}" for su in subs})

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

                ds = s['delays']
                if ds:
                    avg_delay = sum(ds) / len(ds)
                    s['delay'] = f"{avg_delay:.4f}"
                else:
                    s['delay'] = 'NaN'


def main():
    rclpy.init()
    node = SimpleMonitorNode()

    def spin_thread():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.005)

    spin_thread = threading.Thread(target=spin_thread, daemon=True)
    spin_thread.start()

    time.sleep(1.5)

    dpg.create_context()
    dpg.create_viewport(title='RUBI (ROS Utility Board Interface)', width=1450, height=1050, resizable=True)

    with dpg.font_registry():
        try:
            font = dpg.add_font("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
            dpg.bind_font(font)
        except:
            pass

    frozen = [False]

    def toggle_freeze():
        frozen[0] = not frozen[0]
        new_label = "Unfreeze" if frozen[0] else "Freeze"
        dpg.configure_item("freeze_button", label=new_label)

    with dpg.window(tag="main_window", label="ROS 2 Overview", no_close=True, no_move=True, no_resize=True):
        dpg.add_spacer(height=16)

        with dpg.group(horizontal=True, horizontal_spacing=20):
            dpg.add_button(label="Freeze", tag="freeze_button", callback=toggle_freeze, width=120)
            dpg.add_input_text(tag="global_search", hint="Search (all tabs)...", width=-1)

        dpg.add_spacer(height=8)
        dpg.add_separator()
        dpg.add_spacer(height=8)

        # Nicer Legend Panel
        with dpg.child_window(height=50, border=True, horizontal_scrollbar=False):
            with dpg.group(horizontal=True, horizontal_spacing=30, pos=[20, 12]):
                dpg.add_text("Legends:", color=(220, 220, 220))

                dpg.add_text("  * < 1 Hz   ", color=(255, 100, 100))
                dpg.add_text("  * > 10 Hz  ", color=(100, 255, 100))
                dpg.add_text("  1 < * < 10 Hz  ", color=(200, 200, 200))

                dpg.add_text("   |   ", color=(160, 160, 160))

                dpg.add_text("NaN = no data yet   ", color=(180, 180, 180))
                dpg.add_text("0.0 = no messages   ", color=(180, 180, 180))
                dpg.add_text("None = no nodes", color=(180, 180, 180))

        dpg.add_spacer(height=12)

        with dpg.tab_bar(tag="main_tabs"):
            with dpg.tab(label="Topics"):
                with dpg.table(tag="topics_tbl", header_row=True, borders_outerH=True,
                               borders_outerV=True, borders_innerH=True, borders_innerV=True,
                               policy=dpg.mvTable_SizingStretchProp, resizable=True, height=-1,
                               freeze_rows=1):
                    dpg.add_table_column(label="Topic")
                    dpg.add_table_column(label="Type")
                    dpg.add_table_column(label="Rate (Hz)")
                    dpg.add_table_column(label="Delay (s)")
                    dpg.add_table_column(label="Publisher Nodes")
                    dpg.add_table_column(label="Subscriber Nodes")

            with dpg.tab(label="Services"):
                with dpg.table(tag="services_tbl", header_row=True, borders_outerH=True,
                               borders_outerV=True, borders_innerH=True, borders_innerV=True,
                               policy=dpg.mvTable_SizingStretchProp, resizable=True, height=-1,
                               freeze_rows=1):
                    dpg.add_table_column(label="Service")
                    dpg.add_table_column(label="Type")
                    dpg.add_table_column(label="Node")

            with dpg.tab(label="Actions"):
                with dpg.table(tag="actions_tbl", header_row=True, borders_outerH=True,
                               borders_outerV=True, borders_innerH=True, borders_innerV=True,
                               policy=dpg.mvTable_SizingStretchProp, resizable=True, height=-1,
                               freeze_rows=1):
                    dpg.add_table_column(label="Action")
                    dpg.add_table_column(label="Type")
                    dpg.add_table_column(label="Server Nodes")

            with dpg.tab(label="Nodes"):
                with dpg.table(tag="nodes_tbl", header_row=True, borders_outerH=True,
                               borders_outerV=True, borders_innerH=True, borders_innerV=True,
                               policy=dpg.mvTable_SizingStretchProp, resizable=True, height=-1,
                               freeze_rows=1):
                    dpg.add_table_column(label="Node")

    dpg.set_primary_window("main_window", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    last_scroll_y = {}

    while dpg.is_dearpygui_running():
        rclpy.spin_once(node, timeout_sec=0.005)

        if frozen[0]:
            dpg.render_dearpygui_frame()
            time.sleep(0.033)
            continue

        current_tab = dpg.get_value("main_tabs") if dpg.does_item_exist("main_tabs") else None

        if current_tab is not None:
            table_tag = {0: "topics_tbl", 1: "services_tbl", 2: "actions_tbl", 3: "nodes_tbl"}.get(current_tab)
            if table_tag and dpg.does_item_exist(table_tag):
                last_scroll_y[table_tag] = dpg.get_y_scroll(table_tag)

        search = (dpg.get_value("global_search") or "").lower()

        with node.lock:
            dpg.delete_item("topics_tbl", children_only=True)
            dpg.add_table_column(label="Topic", parent="topics_tbl")
            dpg.add_table_column(label="Type", parent="topics_tbl")
            dpg.add_table_column(label="Rate (Hz)", parent="topics_tbl")
            dpg.add_table_column(label="Delay (s)", parent="topics_tbl")
            dpg.add_table_column(label="Publisher Nodes", parent="topics_tbl")
            dpg.add_table_column(label="Subscriber Nodes", parent="topics_tbl")

            for name in sorted(node.topic_stats):
                s = node.topic_stats[name]
                if s['rate'] == 'N/A' and s['delay'] == 'N/A':
                    continue
                if search and search not in name.lower():
                    continue
                pubs = "\n".join(s['pubs']) if s['pubs'] else "None"
                subs = "\n".join(s['subs']) if s['subs'] else "None"
                with dpg.table_row(parent="topics_tbl"):
                    dpg.add_text(name)
                    dpg.add_text(s['type'])
                    try:
                        r = float(s['rate'])
                        color = (255, 100, 100) if r < 1.0 else (200, 255, 200) if r > 10 else (220, 220, 220)
                    except:
                        color = (220, 220, 220)
                    dpg.add_text(s['rate'], color=color)
                    dpg.add_text(s['delay'])
                    dpg.add_text(pubs)
                    dpg.add_text(subs)

            dpg.delete_item("services_tbl", children_only=True)
            dpg.add_table_column(label="Service", parent="services_tbl")
            dpg.add_table_column(label="Type", parent="services_tbl")
            dpg.add_table_column(label="Node", parent="services_tbl")

            for name in sorted(node.service_stats):
                s = node.service_stats[name]
                if search and search not in name.lower():
                    continue
                nodes_str = "\n".join(sorted(set(s['nodes']))) if s['nodes'] else "None"
                with dpg.table_row(parent="services_tbl"):
                    dpg.add_text(name)
                    dpg.add_text(s['type'])
                    dpg.add_text(nodes_str)

            dpg.delete_item("actions_tbl", children_only=True)
            dpg.add_table_column(label="Action", parent="actions_tbl")
            dpg.add_table_column(label="Type", parent="actions_tbl")
            dpg.add_table_column(label="Server Nodes", parent="actions_tbl")

            for name in sorted(node.action_stats):
                s = node.action_stats[name]
                if search and search not in name.lower():
                    continue
                nodes_str = "\n".join(sorted(set(s['nodes']))) if s['nodes'] else "None"
                with dpg.table_row(parent="actions_tbl"):
                    dpg.add_text(name)
                    dpg.add_text(s['type'])
                    dpg.add_text(nodes_str)

            dpg.delete_item("nodes_tbl", children_only=True)
            dpg.add_table_column(label="Node", parent="nodes_tbl")

            for name in sorted(node.nodes):
                if search and search not in name.lower():
                    continue
                with dpg.table_row(parent="nodes_tbl"):
                    dpg.add_text(name)

        if current_tab is not None:
            table_tag = {0: "topics_tbl", 1: "services_tbl", 2: "actions_tbl", 3: "nodes_tbl"}.get(current_tab)
            if table_tag and dpg.does_item_exist(table_tag) and table_tag in last_scroll_y:
                dpg.set_y_scroll(table_tag, last_scroll_y[table_tag])

        dpg.render_dearpygui_frame()
        time.sleep(0.033)

    dpg.destroy_context()
    rclpy.shutdown()


if __name__ == '__main__':
    main()