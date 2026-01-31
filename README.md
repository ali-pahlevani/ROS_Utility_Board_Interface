# RUBI (ROS_Utility_Board_Interface)
**RUBI**: A clean ROS 2 utility board showing live topic rates/delays, pub/sub nodes, services, actions, and system overview.

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
git clone https://github.com/ali-pahlevani/ROS_Utility_Board_Interface.git
cd ROS_Utility_Board_Interface
python3 rubi.py
```

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
