# Releasing RUBI

RUBI can be distributed two ways. Both are independent — you can do either or
both.

| Target | Users install with | Effort | Resolves ROS deps |
|--------|--------------------|--------|-------------------|
| **PyPI** (pip)  | `pip install <name>` | low | ❌ (ROS must already be sourced) |
| **ROS index** (apt) | `sudo apt install ros-<distro>-ros-utility-board-interface` | higher | ✅ via rosdep |

> Reminder: `rclpy` and the ROS message/service packages are **not** on PyPI.
> However RUBI is installed, ROS 2 (Humble or Jazzy) must be installed and
> sourced at runtime.

---

## 0. Before any release

1. **Bump the version in both files so they match** (single source of truth is
   manual today):
   - `setup.py` → `version='X.Y.Z'`
   - `package.xml` → `<version>X.Y.Z</version>`
2. Commit, then tag:
   ```bash
   git commit -am "Release X.Y.Z"
   git tag X.Y.Z
   git push && git push --tags
   ```
3. Sanity-check the build artifacts are clean (no stray `CSV/ MD/ DOT/ Bag/`
   folders — they are git-ignored).

---

## 1. PyPI (pip install)

Lets anyone run `pip install <name>` (they still need ROS 2 sourced to *run* it).

```bash
pip install build twine

# Build sdist + wheel into dist/
python3 -m build

# Validate metadata renders on PyPI
twine check dist/*

# (optional) dry-run on TestPyPI first
twine upload --repository testpypi dist/*

# Publish for real
twine upload dist/*
```

Notes:
- You need a **PyPI account** and an **API token** (store it in `~/.pypirc` or
  pass `-u __token__ -p pypi-...`).
- **Pick a free, valid name.** `setup.py` currently uses
  `ros_utility_board_interface` → normalized to
  `ros-utility-board-interface` on PyPI. Check availability at
  <https://pypi.org/project/ros-utility-board-interface/>. If taken, change
  `name=` in `setup.py` (e.g. `rubi-ros2`) and rebuild.
- A version can only be uploaded **once** — bump the version to re-upload.

---

## 2. ROS index / apt (via bloom)

Gets RUBI into the official ROS 2 index so the build farm produces
`ros-humble-...` / `ros-jazzy-...` Debian packages and `ros2 run` works out of
the box. This is the idiomatic path for a ROS tool, but it is reviewed by the
rosdistro maintainers and has stricter requirements.

### One-time prerequisites
- Public source repo with a **release tag** matching `package.xml` (step 0).
- A **CHANGELOG.rst** per package (bloom can scaffold it):
  ```bash
  pip install bloom            # or: sudo apt install python3-bloom
  catkin_generate_changelog --all   # creates/updates CHANGELOG.rst
  # edit CHANGELOG.rst, then commit
  ```
- **Every `<depend>` in `package.xml` must have a valid rosdep key.** RUBI's
  ROS deps (`rclpy`, `rcl_interfaces`, `lifecycle_msgs`, `tf2_msgs`,
  `rosidl_runtime_py`, `ros2bag`, `python3-yaml`, `python3-psutil`) already
  resolve. **`dearpygui` does not have a rosdep key** — for an official apt
  release you must either:
  1. add a rosdep key for it (PR to `ros/rosdistro` → `rosdep/python.yaml`), or
  2. keep it as a documented `pip install dearpygui` step and not list it as a
     `<depend>`.

### Release
```bash
# First release on a distro creates a "track" and a release repository.
bloom-release --new-track --rosdistro humble --track humble ros_utility_board_interface
# Subsequent releases:
bloom-release --rosdistro humble --track humble ros_utility_board_interface
# Repeat for jazzy:
bloom-release --rosdistro jazzy --track jazzy ros_utility_board_interface
```

`bloom-release` will:
1. ask for a **release repository** (e.g. a GitHub repo named
   `ros_utility_board_interface-release`),
2. generate the Debian/`rosdistro` metadata, and
3. open a **pull request to `ros/rosdistro`**.

After that PR is merged, the ROS build farm builds the binaries (can take a
day or two), and users can:
```bash
sudo apt update
sudo apt install ros-humble-ros-utility-board-interface
ros2 run ros_utility_board_interface rubi
```

### Don't want the official index?
You can host your **own apt repo** (build debs locally with
`bloom-generate rosdebian` + `fakeroot debian/rules binary`, then serve with
`reprepro`/`aptly`). More maintenance, full control.

---

## Recommended path

- **Today, zero extra infra:** colcon from source (`README` Option B) — already
  works.
- **Quick wide reach:** publish to **PyPI** (section 1).
- **Best long-term for a ROS tool:** **bloom → apt** (section 2), ideally after
  sorting the `dearpygui` rosdep key.
