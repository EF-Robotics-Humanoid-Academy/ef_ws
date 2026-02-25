**Location**
This README applies to:
`ef_ws/g1/scripts/navigation/obstacle_avoidance/slam_sdk_example/example`

**Prerequisites**
- CMake (3.10 or newer recommended)
- A C++17 compiler (GCC 7+ or Clang 6+)
- `unitree_sdk2` installed on the system

**Install `unitree_sdk2`**
Follow the official installation guide:
`https://github.com/unitreerobotics/unitree_sdk2`

**Build**
```bash
cd ef_ws/g1/scripts/navigation/obstacle_avoidance/slam_sdk_example/example
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```

**Run**
```bash
./keyDemo eth0
```

`eth0` is the network interface used to communicate with the robot. The interface must be on the same subnet as the robot (for example, `192.168.123.x`).

**Expected Output**
- The program should start without library errors.
- If the interface is correct and the robot is reachable, you should see SLAM SDK messages/logs on the console.

**Tips**
- To list available interfaces:
  ```bash
  ip link
  ```
- To check your IP on the interface:
  ```bash
  ip -4 addr show eth0
  ```
- If you see a library load error (e.g., `libddscxx.so` not found), run:
  ```bash
  sudo ldconfig
  ```
 