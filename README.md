# ROS_Utility_Board_Interface
RUBI: A clean ROS 2 utility board showing live topic rates/delays, pub/sub nodes, services, actions, and system overview.

![RUBI Banner](https://github.com/user-attachments/assets/b971837f-8fa6-4d51-ab8a-e1d1a6dbc03c)


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
