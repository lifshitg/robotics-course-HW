# HW1 public checks

These checks use shared infrastructure and do not need instructor solution files.

From the repository root, run the supplied helper's math checks:

```bash
python3 -m pytest -q assignments/hw01/tests/test_helper.py
```

With the shared environment built and sourced, and Gazebo running:

```bash
ros2 run hw01_sim readiness_demo
python3 assignments/hw01/my_solution/check_setup.py
```

For your assignment implementation, start this observer before starting your node:

```bash
ros2 run hw01_sim check_reach --preset 1 --timeout 90
```

It checks the measured tip pose, a 2-second hold in simulation time, and live
Gazebo platform/marker contact data. Missing or stale contact streams cause failure.
Repeat after restarting the world for presets 2 and 3. A passing helper test does
not prove your perception or TF code works.

The helper's ROS action behavior checks use an explicit fake trajectory controller,
not Gazebo. After sourcing the built environment, run:

```bash
python3 -m pytest -q assignments/hw01/tests/test_action_contract.py
```

They verify invalid goals, concurrent requests, cancellation, and missing controller
behavior. They do not establish that camera rendering or physics works.
