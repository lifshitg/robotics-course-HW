# HW01: Camera-guided reaching

## Method

The `hw01_solution` node subscribes to RGB, aligned depth, and camera calibration
with sensor-data QoS. An approximate time synchronizer pairs RGB and depth within
0.05 simulated seconds. It rejects observations more than 0.5 simulated seconds
old or in the future, and stops using camera data when any camera stream has not
arrived for two wall-clock seconds. The calibration must have matching dimensions
and frame, finite intrinsics, and positive focal lengths.

The RGB image is decoded with `cv_bridge` and converted to HSV. Two hue intervals
cover red on either side of the hue wraparound. A morphological opening removes
small noise. Candidate contours must have sufficient area, a reasonable aspect
ratio and circularity, and must not touch the image boundary. The node rejects
missing, strongly occluded, and ambiguous detections; a partially visible target
is used only if its remaining region passes those checks. These thresholds are
heuristics and should be checked against the actual rendered images.

## Depth and reconstruction

Within the selected contour, the node retains red pixels and erodes the mask by
one pixel to reduce mixed pixels at the marker boundary. It discards nonfinite
depths and depths outside the camera's 0.05–3 m range. At least five pixels and
60% of the eroded region must have valid depths. The node calculates their median
and median absolute deviation (MAD), rejects a region with MAD above 0.02 m, and
removes samples farther than `max(0.01 m, 3 × MAD)` from the median. At least five
and 60% of the region must remain. It uses the median inlier depth as Z and the
mean inlier pixel coordinates `(u, v)` as the image location. With camera matrix
parameters `(fx, fy, cx, cy)`, the optical-frame point is:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = median inlier depth
```

Depth is optical-axis distance in metres, not radial distance. The estimated point
is published on `/hw01/target_camera` with the RGB timestamp. Color segmentation
and the centroid of the remaining visible region can be biased by partial
occlusion; the geometric checks limit but do not eliminate this error.

## TF, motion, and failures

The node uses TF2 to look up `base_link <- camera_optical_frame` at the RGB image
timestamp. If the transform is unavailable, it waits for a new observation
instead of substituting a fixed transform or an old target. It publishes the
unoffset transformed point on `/hw01/target_base`, then adds 0.08 m to base Z and
sends the stamped point to the provided `ArmClient`. It spins while the future is
pending and logs the helper's success or failure message. Only one goal can be
active; after failure it waits for a fresh observation before trying again, and
after success it sends no further goals in that run. On interruption it requests
cancellation and briefly spins to process the request.

Invalid color, depth, calibration, timestamps, or TF prevent a goal. The action
result reports controller completion; it does not establish the measured position
or absence of contact. Use the independent `check_reach` observer for those.

## Build and run

The following commands use Bash and are run from the HW01 directory
(`assignments/hw01`). First build the shared environment as described in
`environment/README.md`, start the simulation, and run its readiness demo and the
setup example. The setup example moves the arm; restart the simulation afterwards.

Build the student package in a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source environment/install/setup.bash
colcon --log-base my_solution/log build --base-paths my_solution/hw01_solution --build-base my_solution/build --install-base my_solution/install
```

Run each numbered block in a separate terminal from `assignments/hw01`. All
terminals must use the same ROS domain and Gazebo partition if those are set.
Replace `1` by `2` or `3` in both the launch and observer commands for the other
presets. Restart the simulation and node between presets.

Terminal A — simulation:

```bash
source /opt/ros/jazzy/setup.bash
source environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

Terminal B — start the independent observer before the solution:

```bash
source /opt/ros/jazzy/setup.bash
source environment/install/setup.bash
ros2 run hw01_sim check_reach --preset 1 --timeout 90
```

Terminal C — start the solution:

```bash
source /opt/ros/jazzy/setup.bash
source environment/install/setup.bash
source my_solution/install/setup.bash
ros2 run hw01_solution reach_marker
```

Prerequisite checks, with the simulation running, from `assignments/hw01`:

```bash
source /opt/ros/jazzy/setup.bash
source environment/install/setup.bash
ros2 run hw01_sim readiness_demo
python3 my_solution/check_setup.py
```

## Results

Record the action result and the independent observer's measured error, hold, and
contact result for each preset after running the demonstrations. These results
have not yet been measured.

| Preset | Action result | Observer result, error, and contacts |
| --- | --- | --- |
| 1 | Not yet run | Not yet measured |
| 2 | Not yet run | Not yet measured |
| 3 | Not yet run | Not yet measured |
