# HW2 installation and simulation guide

**Draft. See [verification results](VERIFICATION.md) before distributing.**

Use Ubuntu 24.04 LTS, ROS 2 Jazzy and Gazebo Harmonic. HW2 uses local models and
Gazebo's standard differential-drive plugin; it needs no manufacturer packages,
MoveIt, Nav2, downloaded models, camera, or SLAM. The headless physics simulation
does not need camera/OpenGL rendering. GUI Gazebo and RViz still require graphics.

## Prepare Ubuntu and ROS

Native Ubuntu 24.04 is the primary platform. Windows/macOS users can use an Ubuntu
VM with guest tools and working 3D graphics for GUI use. Suggested starting resources
are four CPU cores, 8 GB RAM and 30 GB disk; these are not measured minimums.
Native Windows/macOS and Docker support are outside this assignment.

```bash
cat /etc/os-release
```

Expect `VERSION_ID="24.04"`. Use Bash and Ubuntu Python; deactivate Conda/venv before
building or running apt-installed ROS packages. Avoid mixing ROS distributions.

If ROS Jazzy is not installed, configure its official apt repository:

```bash
sudo apt update
sudo apt install locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
ROS_APT_VERSION=$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

See the [official Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
If HW1 already works, reuse the ROS installation and install the explicit HW2
packages below. You do not need to rebuild HW1.

## Install dependencies

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-gz-sim-vendor ros-jazzy-gz-plugin-vendor \
  ros-jazzy-nav-msgs ros-jazzy-tf2-ros ros-jazzy-visualization-msgs \
  ros-jazzy-rosidl-default-generators \
  python3-colcon-common-extensions python3-pytest \
  python3-numpy python3-scipy python3-matplotlib git build-essential
```

The ROS Gazebo vendor packages supply the matching Harmonic version. No separate
Gazebo apt repository is required. SciPy is available for numerical tools; using
it does not permit replacing your PRM/A* code with a complete library planner.

Check installation:

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
gz sim --versions
ros2 pkg prefix ros_gz_sim
```

Expect `jazzy`, Gazebo Sim `8.x`, and an installed package prefix.

## Check ROS communication

In terminal A:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

In terminal B:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

Expect `Publishing` and `I heard`. Stop both with Ctrl+C. Resolve installation,
networking or ROS domain issues before moving to Gazebo if they do not communicate.

## Build the environment

Follow the [fork/branch workflow](../../../README.md#working-on-an-assignment), using
`hw02` for this assignment. From your cloned repository's root:

```bash
source /opt/ros/jazzy/setup.bash
cd assignments/hw02/environment
colcon build --symlink-install --base-paths src --executor sequential
source install/setup.bash
cd ../../..
```

Expect three successful packages: `hw02_gz_checks`, `hw02_interfaces`, `hw02_sim`.
The observer plugin and custom action are built before the Python package. No TA
files or student planner are needed.

**In every new terminal**, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
```

You do not need to source HW1. Do not run HW1 and HW2 in the same ROS domain and
Gazebo partition: they share `/clock` and conventional frame names. On a shared
network, choose your own matching domain in every terminal, for example
`export ROS_DOMAIN_ID=42`. If sharing a machine, also set a matching unique
`GZ_PARTITION`, for example `export GZ_PARTITION=student42_hw02`.

## Start the simulation

Keep terminal A running after sourcing:

```bash
ros2 launch hw02_sim simulation.launch.py layout:=wide
```

Gazebo starts playing automatically. Expect a blue circular robot on the left,
a green goal on the right, room walls and red/brown pillars. The yellow mark on
the robot shows its forward direction.

For physics without a GUI:

```bash
ros2 launch hw02_sim simulation.launch.py layout:=wide gui:=false
```

To view maps and debug outputs in RViz, add `rviz:=true` to either launch command.
Stop the current launch before selecting `layout:=mixed` or `layout:=blocked`.
A new launch restores the robot to `(1,3)` and clears contact history.

## View the map and localization

In terminal B, after sourcing:

```bash
ros2 topic echo /map --once --field info \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /hw02/robot_pose --once
ros2 topic hz /hw02/robot_pose
ros2 run tf2_ros tf2_echo map base_link
```

Expect a 320×240 map at 0.025 m resolution, a pose near `(1,3)`, approximately
20 pose samples per simulated second, and a valid TF connection. Real CPU load
can reduce wall-clock rates. Stop continuous diagnostic commands with Ctrl+C.

RViz uses `map` as its fixed frame. It initially shows `/map` and a robot-footprint
marker. Enable the configuration-space display once your implementation publishes
`/hw02/cspace`; toggle the raw map off to avoid overlapping maps. Student roadmap
and planned-path displays are preconfigured. Their absence before your code runs
is expected.

The supplied map is a physical, uninflated grid. Occupied boundaries and any
unknown cells must be considered when you create configuration space. Map and
start/goal topics use reliable/transient-local QoS; ordinary volatile subscriptions
may miss a previously published static target.

## Run readiness and examples

Keep the simulation running in terminal A. In terminal B:

```bash
ros2 run hw02_sim readiness_demo
python3 assignments/hw02/my_solution/read_map.py
python3 assignments/hw02/my_solution/move_demo.py
```

Readiness prints separate PASS/FAIL results for advancing time, raw map, start/goal,
fresh pose, contact monitoring, TF and helper discovery. The map example prints
frame/dimensions/resolution/origin and saves NPZ/PNG under `build/hw02/demo_map/`.
The movement example drives 0.5 m forward and returns to the start using two
separate `go_to` requests, verifying actual positions. Any failure returns a
nonzero exit code. The readiness check alone does not test robot movement.

Checklist before working on planning:

- [ ] ROS talker/listener communicate and all three packages build.
- [ ] Simulation remains running; readiness reports all PASS.
- [ ] Raw map is received and the saved image matches the physical room.
- [ ] Movement example reaches its point and returns without contact.
- [ ] RViz coordinates agree with Gazebo and `map` → `base_link` exists.

Restart before assignment demonstrations, including after manual movement.
See [the assignment](../instructions.md) and [student README](../my_solution/README.md).

## Helper and contact behavior

The only movement API is `RobotClient.go_to(x,y)`; the corresponding action is
`/hw02/go_to`. `ready()` means the action server has been discovered, not that all
prerequisites passed. The helper turns and drives toward one point, without
obstacle avoidance. It returns failure on stale localization or monitoring,
contact, stalled time, or a 120-wall-second destination deadline. Client replies
have a 130-second deadline. Use clear manual targets:

```bash
ros2 run hw02_sim go_to --x 1.5 --y 3.0
```

Do not publish competing velocity commands while the helper is running. The
internal `/hw02/cmd_vel` topic is infrastructure, not your planner's output.
The Gazebo command watchdog stops commands after 0.5 wall seconds without refresh.

The physics observer reads the robot pose and checks all robot collision shapes
every physics step. It excludes normal floor contact and latches wall/pillar
contact until restart. It publishes `/hw02/contacts`, `/hw02/collision`, and
`/hw02/robot_velocity` at 20 simulated Hz, including empty contact samples.
Independent checks use these streams rather than the motion action result.

## Troubleshooting

| Symptom | Check or action |
| --- | --- |
| Package or action import missing | Build HW2 and source its overlay in this terminal. |
| CMake cannot find Gazebo | Install the explicit vendor packages; source Jazzy; do not mix Gazebo Classic or other versions. |
| Python imports fail | Deactivate Conda/venv; use Ubuntu Python and apt dependencies. |
| Apt download returns 404 | Run `sudo apt update`, then retry. |
| Map/target absent | Confirm map server and matching domain; use reliable/transient-local QoS. |
| Robot pose or contacts absent | Inspect the first Gazebo error for observer/plugin failures; do not bypass the check. |
| No motion or rejected goal | Wait for fresh pose/contact streams, run readiness, verify bounds and allow the previous request to finish. |
| Contact failure persists | Restart the world; contact history intentionally stays latched. |
| Motion times out | Check simulation is playing and pose/clock advance; reduce CPU load. |
| Figure is flipped or shifted | Check `origin='lower'`, row/column order, map origin and metres-per-cell conversion. |
| RViz map looks wrong | Fixed frame must be `map`; display raw map and configuration space separately. |
| Fake motion tests fail unpredictably | Stop the real simulation and use an unused ROS domain for tests. |
| Nodes see another student's robot | Set matching unique `ROS_DOMAIN_ID`; on shared hosts also set `GZ_PARTITION`. |
| GUI fails in VM | Check guest tools/3D acceleration, or use `gui:=false` and save figures without a display. |
| Bridge reports allocator/datawriter errors on exit | Try the optional Cyclone DDS setup below and restart all terminals. |

On the tested host, Fast DDS occasionally failed during bridge cleanup. The ROS
components shut down cleanly with Cyclone DDS. If you encounter this issue:

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Set the export in **every** simulation, example and planner terminal after sourcing.
Stop old processes before changing middleware. This is an optional workaround;
the recorded full navigation runs used the default middleware.

## Stop and restart

Cancel your node's pending movement before stopping it. Stop diagnostic tools,
then Ctrl+C the simulation launch and wait for its processes to exit. Restart for
each layout or random-seed demonstration. Do not reuse pose or TF from a previous
run. A killed helper is protected by the Gazebo watchdog, but graceful cancellation
is still required in your own code.

Automated headless runs sometimes required launch's SIGTERM escalation to stop
the Gazebo wrapper. Wait for shutdown to finish before restarting; do not launch
a second world in the same partition while the old one remains alive. Interactive
shutdown is still a clean-install release check.

## PDF tools for instructors

Students do not need LaTeX to run the assignment:

```bash
sudo apt install pandoc texlive-xetex texlive-fonts-recommended fonts-dejavu-core
python3 tools/build_hw02_pdf.py
```

Run from the repository root. The output is ignored `build/hw02/hw02.pdf`, containing
the assignment followed by this guide as an appendix.
