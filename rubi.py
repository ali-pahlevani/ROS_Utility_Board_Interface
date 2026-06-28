#!/usr/bin/env python3
"""Convenience launcher so you can `python3 rubi.py` from a clone.

The implementation lives in the `ros_utility_board_interface` package
(app.py / node.py / ops.py). This thin shim is not installed by pip; the
installed `rubi` command points at ros_utility_board_interface.app:main.
"""

from ros_utility_board_interface.app import main

if __name__ == '__main__':
    main()
