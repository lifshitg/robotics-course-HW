# HW1 installation and readiness guide

**Status: draft; see [verification results](VERIFICATION.md) before distributing.**

This guide targets **Ubuntu 24.04 LTS, ROS 2 Jazzy, and Gazebo Harmonic**.
The commands below are written for this combination. This environment uses
standard ROS/Gazebo control components and our small local arm model. You do not
need MoveIt, UR packages, physical robot hardware, or model downloads from Gazebo
Fuel. An internet connection is needed during package installation.

## 1. Prepare Ubuntu

Use native Ubuntu 24.04 when possible. If you use Windows, macOS, or another
operating system, you are responsible for arranging a suitable Ubuntu environment.
Options include:

- An Ubuntu 24.04 virtual machine (VM).
- Docker to isolate the Ubuntu dependencies instead of installing them directly
  on your host OS; you must configure graphics and ROS networking yourself.
- Ubuntu 24.04 installed alongside your current OS using dual boot.
- A separate computer, including an older machine capable of running the simulation,
  with Ubuntu 24.04 installed.

These are options to configure yourself, not tested setups supplied by this guide.
Whichever option you choose, camera rendering must work and all readiness checks
below must pass. See the [verification record](VERIFICATION.md) for testing limits.

A VM on the university servers may also be available. Access requires the
university VPN, and this option may not make setup easier. If you are interested,
contact **Gilad Lifshitz**.

For a local VM, use working accelerated graphics. As a starting allocation, use 4 CPU
cores, 8 GB RAM and 30 GB free disk; these are planning suggestions, not measured
minimum requirements. Install guest tools and enable 3D acceleration. A VM that
cannot render the camera is not ready even if the arm appears.

The commands below use Bash. Check your operating system:

```bash
cat /etc/os-release
```

Expect `VERSION_ID="24.04"`. Do not mix Humble/Fortress, Gazebo Classic, or another
ROS distribution with this environment. Leave Conda and Python virtual environments
inactive; use Ubuntu's `/usr/bin/python3` with the ROS apt packages.

## 2. Install ROS 2 Jazzy

If `/opt/ros/jazzy/setup.bash` already exists, skip repository configuration and go
to the dependency installation step. Otherwise, follow these commands, adapted
from the [official Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Enable a UTF-8 locale and Ubuntu Universe:

```bash
sudo apt update
sudo apt install locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

Configure the official ROS package repository:

```bash
ROS_APT_VERSION=$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

Install the required packages. `ros-jazzy-ros-gz` installs the matching Harmonic
vendor packages; a separate Gazebo package repository is unnecessary for this
supported combination. See [Gazebo's ROS installation guidance](https://gazebosim.org/docs/harmonic/ros_installation/).

```bash
sudo apt install \
  ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster \
  ros-jazzy-xacro ros-jazzy-tf2-geometry-msgs ros-jazzy-cv-bridge \
  ros-jazzy-message-filters ros-jazzy-rqt-image-view \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-gz-sim-vendor ros-jazzy-gz-plugin-vendor \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-numpy python3-opencv git build-essential
```

## 3. Check basic ROS communication

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

Expect `Publishing` in A and `I heard` in B. Stop both with Ctrl+C. If this fails,
resolve ROS installation or domain settings before trying Gazebo.

Check the installed distribution and Gazebo major version:

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
gz sim --versions
ros2 pkg prefix gz_ros2_control
```

Expect `jazzy`, Gazebo Sim `8.x` (Harmonic), and a valid package path.

## 4. Fork and build the shared environment

Follow the [repository forking instructions](../../../README.md#forking-and-initial-setup).
Clone your own fork, then run these commands from its root:

```bash
source /opt/ros/jazzy/setup.bash
cd assignments/hw01/environment
colcon build --symlink-install --base-paths src --executor sequential
source install/setup.bash
```

Expect a successful summary for `hw01_gz_checks`, `hw01_interfaces`, and
`hw01_sim`. The observer plugin and custom action are built before the Python package. Neither student code nor TA files are needed for this build.

Every new terminal must source both environments. The remaining examples assume
you are in the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
```

If multiple students share a network, choose a different `ROS_DOMAIN_ID` per student
or group, and set the same value in all of your terminals, for example
`export ROS_DOMAIN_ID=42`. If sharing a machine, also use a unique `GZ_PARTITION`.
Do not change these values between your simulation and student terminals.

## 5. Start the simulation

In terminal A, from the repository root after sourcing:

```bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

Expect a small blue/yellow arm, gray platform and red marker. The simulation starts
playing automatically. The log should show both `joint_state_broadcaster` and
`arm_controller` activating. Keep this terminal open.

Optional RViz:

```bash
ros2 launch hw01_sim simulation.launch.py preset:=1 rviz:=true
```

Headless server with camera rendering:

```bash
ros2 launch hw01_sim simulation.launch.py preset:=1 gui:=false
```

Headless mode still needs a functioning EGL/OpenGL renderer. It does not disable
the camera. To change presets, stop the current launch with Ctrl+C and relaunch
with `preset:=2` or `preset:=3`. Run only one instance per domain/partition.

## 6. Check the camera and controllers

Keep the simulation from [step 5](#5-start-the-simulation) running in terminal A.
In terminal B, after sourcing:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera/image_raw`. Expect an overhead RGB view containing the platform
and marker. The arm may cover part of the view when it moves. The marker must be
visible from the initial home position.

Useful terminal diagnostics:

```bash
ros2 topic list
ros2 topic echo /camera/camera_info --once --qos-reliability best_effort
ros2 topic hz /camera/image_raw --qos-reliability best_effort
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

The supplied Gazebo observer publishes `/hw01/measured_tip` directly from physics
in `base_link`, and `/hw01/platform_contacts` / `/hw01/marker_contacts` as
`ros_gz_interfaces/msg/Contacts` at 10 simulated Hz. Empty samples explicitly mean
no contact; missing samples mean the observer is unavailable. The public reach
check uses these streams independently of motion-action results.

Camera rate is configured to 10 Hz, but slower software rendering may reduce it.
Both controllers should be `active`. Stop continuous commands with Ctrl+C.

## 7. Run the readiness demo

Keep the simulation from [step 5](#5-start-the-simulation) running in terminal A.
From a sourced terminal B:

```bash
ros2 run hw01_sim readiness_demo
```

This waits for startup, checks camera/depth/calibration, clock, TF, joint feedback,
controllers and the helper, then moves the arm between known joint configurations
and returns it home. Expect individual PASS lines, including measured movement.
Any failed check returns exit status 1. Inspect the simulation log for the first
error rather than treating later missing topics as separate installation problems.

Then run the small student example:

```bash
python3 assignments/hw01/my_solution/check_setup.py
```

Expect `Image: 320 x 240, encoding=rgb8` (or `bgr8`), followed by PASS for fresh
camera reception and fixed-point motion. The arm moves to `(0.20, -0.20, 0.14)` m.
The example does not find the assignment marker. Restart the simulation to restore
the initial home position before beginning your assignment demonstration.

## 8. Ready to begin HW1

- [ ] ROS talker/listener communicate.
- [ ] Shared packages build successfully.
- [ ] Simulation stays running and both controllers are active.
- [ ] RGB, depth, and calibration are received; camera frame is correct.
- [ ] Readiness demo reports all PASS results and returns home.
- [ ] `check_setup.py` receives images and completes its fixed-point movement.

Passing these checks verifies these prerequisites, not every possible graphics or
student-code configuration. Build your own implementation using the interfaces
in [the assignment](../instructions.md) and the short
[student README](../my_solution/README.md).

## Troubleshooting

| Symptom | What to check or do |
| --- | --- |
| `ros2` or `gz` not found | Source `/opt/ros/jazzy/setup.bash`; verify installation. |
| Package/action module not found | Build all shared packages and source `environment/install/setup.bash` in this terminal. |
| Python import errors | Leave Conda/venv; check `which python3` and use Ubuntu Python. Do not replace apt ROS libraries with pip versions. |
| Apt download gives 404 | Run `sudo apt update`, then retry installation. |
| Controller stays inactive | Read the Gazebo log; confirm the control plugin loaded and simulation time advances. |
| Camera topic exists but no images | Check Gazebo is playing, Sensors plugin/renderer loaded, and subscriber QoS is best effort. |
| Simulation runs but TF is missing | Confirm joint states and static camera publishers; check names with `tf2_echo`. |
| Nodes cannot see each other | Check matching `ROS_DOMAIN_ID`, `GZ_PARTITION`, and firewall/VM networking. Restart ROS CLI daemon after changing domains. |
| Wrong point height rejected | The tip only reaches z=0.14 m. Transform the marker into `base_link`, then add 0.08 m. |
| Helper says unavailable/busy | Wait for startup; run readiness; allow the current motion to finish before sending another. |
| Motion times out | Check simulation is playing and rendering is not stalled. |
| Graphics crash/black image | Verify host/VM graphics acceleration and drivers. Try software rendering below. |

Try software rendering first:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch hw01_sim simulation.launch.py gui:=false
```

On a machine with no display, an alternative is Xvfb (optional package):

```bash
sudo apt install xvfb
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a \
  ros2 launch hw01_sim simulation.launch.py \
  gui:=false headless_rendering:=false
```

A virtual display does not repair a broken Mesa driver. If both methods crash,
use a machine/VM with functioning OpenGL support and share the first Gazebo error
and your package versions with the teaching staff. Readiness must actually pass;
do not bypass its camera checks.

## Stop and restart

Stop your own node and diagnostic viewers, then Ctrl+C the simulation launch.
Wait for its processes to exit before relaunching. Do not reuse stale camera data
or TF from a prior run. A new launch resets the world and arm to home.
