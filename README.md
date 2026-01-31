# ROS_Utility_Board_Interface
RUBI: A clean ROS 2 utility board showing live topic rates/delays, pub/sub nodes, services, actions, and system overview.
<div align="center">

<h1>RUBI – Realtime Unified Bot Inspector</h1>

<p>
  <strong>A beautiful, lightweight, real-time ROS 2 system overview & health monitor</strong>
</p>

<p>
  <a href="https://github.com/[your-username]/rubi-ros-monitor/stargazers">
    <img src="https://img.shields.io/github/stars/[your-username]/rubi-ros-monitor?style=social" alt="GitHub stars">
  </a>
  <a href="https://github.com/[your-username]/rubi-ros-monitor/issues">
    <img src="https://img.shields.io/github/issues/[your-username]/rubi-ros-monitor" alt="GitHub issues">
  </a>
  <img src="https://img.shields.io/badge/ROS%202-Humble%20|%20Iron%20|%20Jazzy-blue" alt="ROS 2 versions">
  <img src="https://img.shields.io/badge/Python-3.8%2B-green" alt="Python">
</p>

</div>

<br>

https://github.com/user-attachments/assets/xxxx-xxxx-xxxx-xxxx-xxxx  <!-- replace with a short demo GIF/video -->

### What is RUBI?

RUBI gives you a single, clean window to understand your ROS 2 robot at a glance:

- Live topic publication rates & delays (color-coded: red = slow, green = fast)
- Publisher & subscriber nodes per topic
- All services with their server nodes
- All actions with their server nodes
- Full node list
- Global search across everything
- Freeze / Unfreeze updates so you can comfortably read & copy long lists
- Select & copy any text (topic names, node lists, rates, etc.)

No heavy dependencies. Runs fast even on embedded systems.

### Features

- **Realtime monitoring** — rates & delays updated every ~0.4s from actual subscriptions
- **QoS friendly** — uses BEST_EFFORT by default to avoid common incompatibility warnings
- **Multi-tab interface** — Topics / Services / Actions / Nodes
- **Freeze mode** — stop updates to read long node lists or copy text without jumping
- **Copy-paste ready** — select any cell content and copy (Ctrl+C)
- **Beautiful & minimal** — Dear PyGui + clean layout

### Installation

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash   # or iron, jazzy, etc.

# 2. Install Dear PyGui (only dependency)
pip install dearpygui

# 3. Clone & run
git clone https://github.com/[your-username]/rubi-ros-monitor.git
cd rubi-ros-monitor
python3 health_app.py   # or rename to rubi.py if you prefer
