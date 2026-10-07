# HW2 public checks

These checks require no TA or student planner implementation.

Raw-map/world consistency, target validation and direct-steering tests, from the
repository root:

```bash
python3 -m pytest -q assignments/hw02/tests/test_environment.py
```

The helper/client integration tests use fake localization in a **separate ROS
domain**. Stop your simulation and choose an unused domain for this command:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
ROS_DOMAIN_ID=84 python3 -m pytest -q assignments/hw02/tests/test_motion.py
```

Do not run these fake publishers in your simulation's domain. They cover invalid
coordinates, success/stopping, client and server concurrency, cancellation before
acceptance, lost localization, obstacle contact, a stalled clock and a wall-time deadline. They do not
establish Gazebo physics behavior.

For runtime readiness, start the simulation as described in
[environment/README.md](../environment/README.md), then run in another sourced terminal:

```bash
ros2 run hw02_sim readiness_demo
python3 assignments/hw02/my_solution/read_map.py
python3 assignments/hw02/my_solution/move_demo.py
```

For a student demonstration, restart the chosen layout and start this observer
before the implementation:

```bash
ros2 run hw02_sim check_navigation --layout mixed --timeout 300
```

Use the same layout name as the launch command. Repeat for `wide` and `blocked`.
It reads actual physics pose, velocity and latched contact streams; it never
commands movement. A solvable run must arrive within 0.10 m, stop for two simulated
seconds, and avoid wall/pillar contact. The blocked run must publish
`no_path_found` on `/hw02/planner_status` using reliable/transient-local QoS and stay
within 0.03 m of the start. Missing/stale streams fail the check.

These public checks do not prove your configuration-space, PRM or A* implementations
are correct. Explain them in your report and live demonstration, and include your
own tests of grid conversion, inflation, full-edge checking and graph search.
