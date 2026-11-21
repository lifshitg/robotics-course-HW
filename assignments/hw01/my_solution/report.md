# HW1 Report

**Zehava Rozental** - 326109501
**Ido Amar** - 204682769

Our solution is one ROS 2 node, `reach_marker` (package `hw01_reach`). For every synchronized RGB + depth
pair it runs: check freshness → find marker → 3D point in camera frame → TF2 to `base_link` → publish →
send goal (once). Each step returns `None` if something is wrong, and then the frame is skipped.
This way the arm only moves if all checks passed on the same image, and we don't need any extra state.

## Design decisions

**Matching RGB and depth.** We used `ApproximateTimeSynchronizer` with slop 0.05 s instead of just keeping
the latest message from each topic. If the two images are from different moments, the depth at the marker
pixel could belong to something else (for example the arm passing by).

**Red detection with a simple RGB threshold** (R > 100, G < 80, B < 80). In the simulation the marker is
pure red, the platform is gray and the arm is blue/yellow.

**Marker center = mean of the red pixels.** The threshold already removed every non-marker pixel, so all
remaining pixels are valid and their mean is the center of the disk (with sub-pixel precision).

**Depth = median, not mean.** We keep only finite depths between 0.05 and 3.0 m (the camera clip range),
which removes NaN, inf, zero and negative values. Pixels on the edge of the marker can get the depth of the
platform below it. The median ignores these outliers, the mean would be pulled by them. Here it is the
opposite of the center: in depth some values can be wrong, so the median is the right choice.

**Calibration from `CameraInfo`.** fx, fy, cx, cy are read from K (`k[0]`, `k[4]`, `k[2]`, `k[5]`) and not
hardcoded. Then X = (u − cx)·Z/fx and Y = (v − cy)·Z/fy (inverse pinhole model; depth is along the
optical Z axis).

**The point keeps the image header.** This gives it `camera_optical_frame` and the image timestamp, which is
exactly what TF2 needs. We call `tf_buffer.transform(point, 'base_link')` (source: optical frame, target:
`base_link`). In this world the camera is static so the time doesn't change the result, but with a moving
camera it would. The node uses `use_sim_time`, otherwise the image stamps (sim time) can't be compared to
the clock.

**If TF is not ready, skip and retry on the next frame** instead of waiting inside the callback. Waiting
would block the node. This case really happens on the first frame of every run.

**Send only one goal, from the first good observation.** We first thought about updating the goal while the
arm moves, but the arm itself covers the marker near the end of the motion. In our runs the measured position
moved by about 3 mm when the arm was above it. So the first clean measurement is the most reliable one.
`self.motion` is `None` until the goal is sent, and that is the only condition for sending.

**No waiting in callbacks.** `move_to_point` returns a future. The main loop calls `spin_once` and checks
`future.done()`, then prints SUCCESS/FAILED and exits. This keeps all callbacks (camera, TF, action) running.

**Stale data.** A frame older than 0.5 sim seconds is skipped (`abs()` also catches stamps from the future,
for example after a simulation restart). Goals are only sent from inside the camera callback, so if no
fresh data arrives nothing is sent, and the main loop warns after 2 seconds. Old targets are never stored.

## Warnings

Whenever the node skips a frame it prints a short warning with the reason (old image, no marker, bad depth,
TF not ready, etc.), so if the arm doesn't move we can see why. Images come 10 times a second, so we limit
each warning to once every 2 seconds, otherwise the terminal fills up with the same line.

Some examples:

- "Marker not found or occluded (13 red pixels)": fewer than 30 red pixels, the marker is missing or mostly hidden.
- "Not enough valid depth values on the marker": most depth values on the marker are NaN, zero or out of range.
- "Stale observation (0.73 s old), skipping": the image is older than 0.5 sim seconds.
- "Transform not available yet": TF2 doesn't know the path from the camera to `base_link` yet.
- "No camera calibration yet": no `CameraInfo` message arrived, so we don't have K.
- "No fresh camera data for 2 seconds": no synchronized RGB + depth images arrived for 2 seconds.

In our runs we saw three of them. "Transform not available yet" shows up at the start of every run, because
the first image comes before TF is ready. One frame later it works. "Marker not found or occluded" shows up
while the arm is moving over the marker, which is fine because the goal was already sent. "No fresh camera
data" shows up if we run the node without the simulation.

## Results

Docker (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic) on a Mac, simulation restarted for each preset.

- Preset 1: goal sent (0.352, −0.148, 0.140), check_reach PASS with error 0.0029 m.
- Preset 2: goal sent (0.401, 0.002, 0.140), check_reach PASS with error 0.0022 m.
- Preset 3: goal sent (0.352, 0.151, 0.140), check_reach PASS with error 0.0021 m.

No contact with the platform or the marker in any run.


## Build and Run Instructions

We used three terminals to run and test our solution.

### Build (first time only)

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash

cd assignments/hw01/environment
colcon build --symlink-install --base-paths src --executor sequential

cd ../my_solution
source ../environment/install/setup.bash
colcon build --symlink-install --base-paths hw01_solution
```

### Setup (in each terminal)

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
```

### Terminal 1 — Start the simulation

```bash
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a \
  ros2 launch hw01_sim simulation.launch.py \
  preset:=1 gui:=false headless_rendering:=false
```

We used software rendering because of graphics issues in our Ubuntu environment.

### Terminal 2 — Start the checker

```bash
ros2 run hw01_sim check_reach --preset 1
```

### Terminal 3 — Run our solution

```bash
python3 assignments/hw01/my_solution/hw01_reach/hw01_reach/reach_marker.py
```

We started the checker before running our code. After each test, we stopped the processes and restarted the simulation. We repeated the same steps for presets 2 and 3 by changing the preset number, without modifying our code. We verified the results using the checker's PASS output.
