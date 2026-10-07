# Your HW2 implementation

This directory contains only two setup examples. Complete the
[installation guide](../environment/README.md) before creating your own planner package.

## Run the examples

From the repository root, terminal A:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
ros2 launch hw02_sim simulation.launch.py layout:=wide
```

Keep A running. In terminal B, also from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
ros2 run hw02_sim readiness_demo
python3 assignments/hw02/my_solution/read_map.py
python3 assignments/hw02/my_solution/move_demo.py
```

`read_map.py` prints map metadata and saves the **raw**, uninflated map as NPZ and
PNG under ignored `build/hw02/demo_map/`. `move_demo.py` moves to `(1.5, 3)` and
returns to `(1, 3)`, checking measured positions. Both fail with a nonzero exit
code if prerequisites or movement fail. Neither example solves the assignment.

## Use one-destination movement

```python
from hw02_sim.robot_client import RobotClient

robot = RobotClient(your_ros_node)
if robot.ready():
    future = robot.go_to(1.5, 3.0)
```

Keep spinning your node. When `future.done()` is true, inspect
`future.result().success` and `.message`. Call `robot.cancel()` on interruption.
`ready()` checks action-server discovery; run readiness to check the complete setup.

For your own planned route, request one waypoint at a time and wait for a successful
result before requesting the next. Stop on failure. The helper turns and drives
directly; it does not avoid obstacles or execute an entire path for you.

A manual equivalent, after sourcing, is:

```bash
ros2 run hw02_sim go_to --x 1.5 --y 3.0
```

Use clear destinations for manual tests. Restart the simulation before assignment
demonstrations to restore the initial robot pose and clear latched contacts.

## Create your own package

From this directory, create a Python package with `ros2 pkg create --build-type
ament_python YOUR_PACKAGE`. Choose your own modules, node structure and executable
entry point. Declare `hw02_sim` and every other ROS/Python dependency you use.
Keep the two examples unchanged.

After sourcing the shared environment, build your package from the repository root:

```bash
colcon --log-base assignments/hw02/my_solution/log build \
  --symlink-install --base-paths assignments/hw02/my_solution \
  --build-base assignments/hw02/my_solution/build \
  --install-base assignments/hw02/my_solution/install
source assignments/hw02/my_solution/install/setup.bash
ros2 run YOUR_PACKAGE YOUR_EXECUTABLE
```

See [instructions.md](../instructions.md) for planning, visualization and report
requirements. Commit source, report and required figures, not caches/build products.
