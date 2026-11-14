# HW1 Report — Camera-Guided Reaching

Yuval Bublil

## Overview

The node `hw01_reach/reach` detects the red marker in the overhead camera, reconstructs
its 3D position from the aligned depth image and the camera intrinsics, transforms that
position into `base_link` with TF2, adds the 0.08 m hover offset, and commands the
supplied motion helper (`hw01_sim.arm_client.ArmClient`) to move the tip above the marker.

## Detection

The RGB image is converted to `bgr8` through `cv_bridge`, which normalises the `rgb8`/`bgr8`
source encoding, and then to HSV. The marker is isolated with two red hue bands
(`0–10` and `170–180`) combined with high saturation and value thresholds, so lighting
on the gray platform and the blue/yellow arm do not match. A morphological open/close
with a 3×3 kernel removes speckle and fills the blob. The largest external contour is
kept; if no contour is found or its area is below 25 px the detection is treated as
missing or occluded and no command is issued. The marker pixel is the contour centroid
from image moments.

## Depth reconstruction

Depth is read as a `32FC1` image in metres. Around the marker pixel a 5×5 window is
sampled and reduced to the values that are finite, strictly positive, and inside
`[0.05, 5.0]` m; non-finite, zero, negative, and out-of-range samples are discarded.
If fewer than four valid samples remain the observation is rejected. The depth estimate
is the median of the remaining samples, which is robust to the marker edge and to
isolated invalid pixels while the marker top is flat. The 3D point in
`camera_optical_frame` is the pinhole back-projection

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = Z
```

using `fx, fy, cx, cy` from `/camera/camera_info`. It is published on
`/hw01/target_camera`.

## Coordinate transform (direction and timing)

The stamped camera-frame point is transformed into `base_link` with TF2
(`tf2_geometry_msgs.do_transform_point`) using the transform looked up at the RGB image
timestamp, so the point and the transform share the same time. The node runs with
`use_sim_time` enabled and the frame id comes from the image header
(`camera_optical_frame`); no fixed or hand-written transform is used. The base-frame
point is published on `/hw01/target_base`. The commanded goal copies this point, adds
0.08 m to its Z (the marker top near 0.06 m becomes the 0.14 m tip height), and keeps the
`base_link` frame.

## Failure handling

- RGB and depth are accepted as a pair only when their timestamps differ by at most
  0.05 simulated seconds.
- An observation is dropped if its image timestamp is more than 0.5 simulated seconds old
  or if no fresh RGB/depth has arrived for 2 wall-clock seconds; a missing observation
  never reuses an earlier target.
- Invalid depth (non-finite, zero, negative, out of range) and absent/occluded detections
  produce no command.
- If the transform is unavailable the observation is skipped and retried on the next
  frame.
- Only one motion is active at a time: a new goal is sent only when no motion future is
  pending, and after a successful reach the node stops commanding so the tip holds.
- On interruption the node cancels any pending motion before shutting down.

## Build and run

From the repository root, inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd assignments/hw01/environment && colcon build --symlink-install --base-paths src --executor sequential && cd -
source assignments/hw01/environment/install/setup.bash
cd assignments/hw01/my_solution && colcon build --symlink-install && cd -
source assignments/hw01/my_solution/install/setup.bash
```

Per preset (`N` = 1, 2, 3), in three shells sharing one `ROS_DOMAIN_ID`:

```bash
ros2 launch hw01_sim simulation.launch.py preset:=N gui:=false
ros2 run hw01_sim check_reach --preset N --timeout 90
ros2 run hw01_reach reach
```

## Results

Each preset was run headless with software rendering. The node issues a single
trajectory and holds, and `check_reach` confirms the hold in simulation time:

| Preset | `check_reach` result |
| --- | --- |
| 1 | PASS: tip error 0.0033 m, held 2 simulated seconds; live contact streams clear |
| 2 | PASS: tip error 0.0023 m, held 2 simulated seconds; live contact streams clear |
| 3 | PASS: tip error 0.0025 m, held 2 simulated seconds; live contact streams clear |

In every run the detected marker position in `base_link` matched the platform marker
and the commanded hover point reached the 0.14 m tip height, with no contact against the
platform or marker.
