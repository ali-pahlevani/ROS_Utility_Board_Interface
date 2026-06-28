# RUBI (ROS_Utility_Board_Interface) (V2)
**RUBI v2**: A single-window ROS 2 *control board* — not just a monitor:
- **Live topic rates/bandwidth/delays**   |   **QoS mismatch detection**   |   **Health watchdog**
- **Message inspection**   |   **Live plotting**   |   **Logs**   |   **Parameters**   |   **TF health**   |   **Exports**
- **Lifecycle control**   |   **Service/Action caller**   |   **Rosbag record/play**   |   **Graph snapshot/diff** 

![RUBI Banner](https://github.com/user-attachments/assets/429bd575-41b3-4508-8a38-fa521a028bad)


### What is RUBI?

**RUBI** gives you a single, clean window to understand *and operate* your ROS 2 robot at a glance — no web server, no heavy stack. Runs fast even on embedded systems.

### Features

**Observe**
- **Realtime monitoring** — topic rates, delays, and a per-topic **rate sparkline** (trend) updated continuously
- **Live bandwidth** — per-topic throughput in human-readable B/s, like `ros2 topic bw` but for everything at once
- **QoS mismatch detector** — shows each topic's QoS and flags incompatible publisher/subscriber pairs (reliability & durability) right in the table — the #1 cause of silent "no data"
- **Health watchdog** — expected `min_hz` / `max_hz` / `max_delay` per topic (YAML + glob patterns) → instant ✓/✗ status and an alert banner
- **Message inspector** — peek the latest message of any topic as YAML (with a *live* toggle)
- **Live plotter** — select a topic, drill into its numeric fields (including array elements like `position[0]` for `/joint_states`), and plot them live vs time; overlay multiple signals with legend, pan/zoom, scrolling window and auto-Y
- **/rosout log pane** — severity-filtered, color-coded, searchable
- **Node process metrics** — best-effort PID / CPU% / memory per node (psutil)
- **TF health** — frame tree (child → parent), per-frame rate, and **stale-transform** detection
- **Lifecycle states** — detects lifecycle nodes and shows their current state

**Operate**
- **Parameter browser & live edit** — pick a node, choose a parameter from a dropdown, and set it (the value box shows the expected type)
- **Lifecycle control** — trigger configure / activate / deactivate / cleanup / shutdown
- **Service & Action caller** — pick any service/action from a dropdown; its request/goal form is pre-filled from the type, edit and call
- **Rosbag** — record, play (rate/loop), and inspect bags (`ros2 bag info`); recordings are saved under `Bag/`

**Analyze & share**
- **Graph snapshot & diff** — capture the graph and diff later to catch intermittent nodes/topics
- **Export** — topic table to CSV / Markdown, or the node graph to Graphviz `.dot` (saved under `CSV/`, `MD/`, `DOT/`)
- **Multi-domain** — switch `ROS_DOMAIN_ID` from the header (relaunches on the chosen domain)

**Polish**
- Flicker-free tables (in-place updates, preserved scroll), 60 FPS, clean dark theme, global search, freeze mode.

### Requirements

- **ROS 2 Humble or Jazzy**, installed and **sourced**. RUBI uses `rclpy` and
  ROS interface packages that come from your ROS install — they are *not* on
  PyPI, so ROS 2 must be present and sourced however you run RUBI.
- Python 3.8+ and three non-ROS Python packages: `dearpygui`, `pyyaml`, `psutil`.

```bash
source /opt/ros/humble/setup.bash        # or: source /opt/ros/jazzy/setup.bash
pip install dearpygui pyyaml psutil
```

### Installation

**Option A — run directly (quickest):**
```bash
git clone https://github.com/ali-pahlevani/ROS_Utility_Board_Interface.git
cd ROS_Utility_Board_Interface
python3 rubi.py                       # or: python3 rubi.py --rules rubi_rules.yaml --domain 0
```

**Option B — colcon (recommended for ROS users; enables `ros2 run`):**
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ali-pahlevani/ROS_Utility_Board_Interface.git
cd ~/ros2_ws && colcon build --packages-select ros_utility_board_interface
source install/setup.bash
ros2 run ros_utility_board_interface rubi
```

**Option C — pip (adds a `rubi` command):**
```bash
pip install ros-utility-board-interface   # from PyPI
# or, from a clone for development:
#   git clone https://github.com/ali-pahlevani/ROS_Utility_Board_Interface.git
#   cd ROS_Utility_Board_Interface && pip install .
rubi                                       # run inside a shell where ROS 2 is sourced
```

> However you install it (pip, PyPI, or colcon), ROS 2 must still be sourced at
> runtime — `rclpy` is provided by ROS, not pip.

### Outputs

RUBI writes into folders next to where it runs: exports under `CSV/`, `MD/`,
`DOT/`, and rosbag recordings under `Bag/`.

### Health watchdog rules

Create a `rubi_rules.yaml` (a sample ships with the repo):

```yaml
topics:
  "/scan":              { min_hz: 8, max_hz: 12 }
  "/odom":              { min_hz: 20, max_delay: 0.1 }
  "/camera/*/image_raw": { min_hz: 25 }   # glob patterns supported
```

Topics outside their bounds turn **✗ FAIL** (red) in the Health column, with a running count in the status bar. Topics without a rule simply show `—`.

I’d **love collaborations**! Contribute via pull requests on *GitHub* for bug fixes, new features, or documentation improvements. Reach out via *GitHub Issues* for questions, suggestions, or partnership ideas.

### Contributing

Contributions are welcome! To contribute:

1. Fork the repository.
2. Create a branch (`git checkout -b feature/your-feature`).
3. Commit changes (`git commit -m "Add your feature"`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a pull request.

---

+ Questions? Reach out: **a.pahlevani1998@gmail.com**
+ LinkedIn: **https://www.linkedin.com/in/ali-pahlevani/**
