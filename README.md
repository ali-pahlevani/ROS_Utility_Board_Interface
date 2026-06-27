# RUBI (ROS_Utility_Board_Interface)
**RUBI**: A clean ROS 2 utility board showing live topic rates/delays, pub/sub nodes, services, actions, and system overview.

![RUBI Banner](https://github.com/user-attachments/assets/b971837f-8fa6-4d51-ab8a-e1d1a6dbc03c)


### What is RUBI?

**RUBI** gives you a single, clean window to understand your ROS 2 robot at a glance:

- Live topic publication rates, **bandwidth (B/s)**, & delays
- **QoS introspection with automatic mismatch detection** (the #1 cause of silent "no data")
- **Health watchdog** — turn topics red when their rate/delay leaves your expected bounds
- Publisher & subscriber nodes per topic
- All services with their server nodes
- All actions with their server nodes
- Full node list
- Global search across everything
- Freeze / Unfreeze updates

No heavy dependencies. Runs fast even on embedded systems.

### Features

- **Realtime monitoring** — rates, bandwidth & delays updated continuously from actual subscriptions
- **QoS mismatch detector** 🆕 — shows each topic's QoS and flags incompatible publisher/subscriber pairs (reliability & durability) right in the table
- **Live bandwidth** 🆕 — per-topic throughput in human-readable B/s, like `ros2 topic bw` but for everything at once
- **Health watchdog** 🆕 — define expected `min_hz` / `max_hz` / `max_delay` per topic (YAML, glob patterns supported) and get instant ✓/✗ status plus an alert banner
- **Flicker-free UI** — tables update in place; smooth 60 FPS rendering, preserved scroll
- **Multi-tab interface** — Topics / Services / Actions / Nodes
- **Freeze mode** — stop updates to read long node lists
- **Beautiful & minimal** — Dear PyGui + clean dark theme

### Installation

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash   # or iron, jazzy, etc.

# 2. Install dependencies
pip install dearpygui pyyaml

# 3. Clone & run
git clone https://github.com/ali-pahlevani/ROS_Utility_Board_Interface.git
cd ROS_Utility_Board_Interface
python3 rubi.py

# Optional: enable the health watchdog with custom rules
python3 rubi.py --rules rubi_rules.yaml
```

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

+ If you have any questions, please let me know: **a.pahlevani1998@gmail.com**

+ Also, don't forget to check out our **website** at: **https://www.SLAMbotics.org**
