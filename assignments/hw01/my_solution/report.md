# HW01: Camera-guided reaching

Authors: Adi Megidish

## 1. Goal and code

The arm finds the red marker using the camera and moves its tip 8 cm above it.
[perception.py](hw01_reach/hw01_reach/perception.py) handles image processing and
camera math. [node.py](hw01_reach/hw01_reach/node.py) handles ROS messages, TF2,
and movement.

## 2. Finding the marker and its 3D position

The node receives RGB, aligned depth, and camera calibration with best-effort
sensor-data QoS. RGB/depth timestamps must match within 0.05 simulated seconds.
`CvBridge` reads the image encoding; OpenCV converts the color image to HSV.

Red pixels use hue ranges 0-10 and 170-179, saturation at least 120, and brightness
at least 70. The detector requires one red region of at least 30 pixels, at least
5 pixels wide and tall, and not touching the image edge. Small noise is ignored;
missing or ambiguous markers are rejected. A partly hidden marker can be used
if its visible part still meets these checks.

Depth is `32FC1`, measured in metres along the camera's forward Z axis. The red
mask is shrunk by one pixel using a 3-by-3 erosion to avoid background depth at
its edges. Invalid depths and values outside 0.05-5.0 m are removed. Depths more
than 0.01 m from the median are also removed. At least 10 pixels and 60% of the
remaining interior must agree. Their median gives depth `Z`; their average pixel
location gives `(u, v)`.

Camera calibration provides focal lengths `fx, fy` and image center `cx, cy`.
The marker's camera coordinates are:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

## 3. Transforming coordinates and moving

The node enables `use_sim_time`. It puts the camera position in a `PointStamped`
with the RGB image timestamp. TF2 transforms it **from `camera_optical_frame`
to `base_link` at that timestamp**, not using a hardcoded transform.

The marker positions are published on `/hw01/target_camera` and
`/hw01/target_base` before adding the hover offset. The node adds 0.08 m to the
base-frame Z coordinate and calls `ArmClient.move_to_point`. The supplied helper
handles inverse kinematics and the trajectory. One goal is sent per run; ROS
keeps processing messages while it moves, then reports success or failure.

## 4. Handling failures

- Missing markers, invalid depth/calibration, wrong encodings or frames,
  distorted images, and unavailable TF prevent motion and produce warnings.
- Zero, future, or more than 0.5-second-old simulation timestamps are rejected.
  Processing must finish within 2 real seconds of receiving the image pair.
  Missing images never trigger a goal; an old target is never reused.
- The node waits for the helper. If no safe goal is sent within 120 real seconds,
  it exits with failure. The helper also has a motion timeout.
- Ctrl+C or SIGTERM cancels pending motion before shutdown, waiting up to
  2 real seconds for confirmation and warning if it is not confirmed.

## 5. Build and run

From the repository root, build the shared environment and solution:

```bash
source /opt/ros/jazzy/setup.bash
cd assignments/hw01/environment
colcon build --symlink-install --base-paths src --executor sequential
cd ../../..
source assignments/hw01/environment/install/setup.bash
cd assignments/hw01/my_solution
colcon build --symlink-install --base-paths hw01_reach --executor sequential
cd ../../..
source assignments/hw01/my_solution/install/setup.bash
```

In each new terminal, first run:

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
source assignments/hw01/my_solution/install/setup.bash
```

Then use three terminals:

```bash
# Terminal A: simulation
ros2 launch hw01_sim simulation.launch.py preset:=1

# Terminal B: start the checker BEFORE the solution
ros2 run hw01_sim check_reach --preset 1 --timeout 90

# Terminal C: solution
ros2 run hw01_reach reach_marker
```

Restart the simulation for presets 2 and 3, changing `preset:=1` and `--preset 1`
to match. Use the same solution code for every preset.

## 6. Results and limitations

The build succeeded. On 2026-11-15, the same solution passed all three presets
in fresh headless Gazebo worlds (`gui:=false`) on Ubuntu 24.04 / WSL with ROS 2
Jazzy and Gazebo Harmonic. Readiness passed and the arm returned home before
each run. The public checker independently measured:

| Preset | Tip error | Hold time | Live contact streams | Result |
| --- | --- | --- | --- | --- |
| 1 | 3.3 mm | 2 simulated seconds | Clear | PASS |
| 2 | 2.2 mm | 2 simulated seconds | Clear | PASS |
| 3 | 2.3 mm | 2 simulated seconds | Clear | PASS |

All errors were below the required 20 mm limit.