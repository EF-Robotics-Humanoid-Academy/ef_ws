Dieses READ ME ist für ef_ws/g1/scripts

 
This folder contains example scripts and utilities for the Unitree G1 stack, organized by function.

**Structure**
- `arm_motion`: Arm control examples and SDK notes
- `basic`: Core locomotion and high‑level motion examples
- `dev`: Development tools and client utilities
- `navigation`: Navigation and SLAM related examples
- `obstacle_avoidance`: Obstacle‑avoidance examples and demos
- `obj_detection`: Object detection examples
- `safety`: Safety‑related scripts (if applicable)
- `sensors`: Sensor streaming and visualization tools
- `troubleshooting`: Diagnostics and troubleshooting helpers
- `usecases`: End‑to‑end task demos
- `punch.py`: Standalone demo script (see file header for usage)

**Quick Start**
- Find a script for your task area and open it to see its usage header or CLI flags.
- Run scripts with:
  ```bash
  python3 <script_name>.py
  ```
- If a script uses DDS or networked control, make sure your network interface is correct (see `ip link`).

**Notes**
- Some folders include `FAQ.txt` or `sdk_details.md` with extra setup info.
- If you see “library not found” errors, run `sudo ldconfig` or check your SDK install.