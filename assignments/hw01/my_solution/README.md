# Your HW1 implementation

This folder contains a small installation example, not an assignment solution.
Read [the assignment](../instructions.md) and finish the
[environment setup](../environment/README.md) first.

## Run the example

Start the simulation in terminal A from the repository root after completing
[the environment setup](../environment/README.md#5-start-the-simulation):

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

Keep terminal A running. In terminal B, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
python3 assignments/hw01/my_solution/check_setup.py
```

The example prints the RGB image dimensions and encoding, then moves the arm to
`(0.20, -0.20, 0.14)` metres. Expect two PASS lines: fresh camera reception and
fixed-point arm movement. Failure returns a nonzero exit code. This does not check
or solve marker detection, depth reconstruction, or coordinate transforms.

## Use the helper in your own node

```python
from geometry_msgs.msg import PointStamped
from hw01_sim.arm_client import ArmClient

arm = ArmClient(your_ros_node)
point = PointStamped()
point.header.frame_id = 'base_link'
point.point.x, point.point.y, point.point.z = 0.20, -0.20, 0.14
if arm.ready():
    future = arm.move_to_point(point)
```

Keep spinning your ROS node while the motion runs. When `future.done()` is true,
`future.result().success` and `.message` describe the outcome. Do not block your
subscription callback waiting for the result. Call `arm.cancel()` to cancel a
pending motion. Only one motion may be active; coordinates use metres, and this
arm's fixed tip height is 0.14 m. The helper handles kinematics for you.
