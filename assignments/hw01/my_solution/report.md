# HW01 - Camera-Guided Reaching

**Student:** Ido Fraiberg

## Marker detection

The solution subscribes to RGB images, aligned depth images and
camera calibration using best-effort sensor-data QoS.

RGB and depth images are matched using their timestamps, with a
maximum difference of 0.05 seconds.

The RGB image is converted to HSV. Two hue ranges are used to
detect red pixels because red wraps around the HSV hue scale.
Connected component analysis is used to select the largest
red region.

Small regions and ambiguous detections are rejected.

## Depth reconstruction

Depth values are extracted from pixels belonging to the detected
marker. Nonfinite values and values outside the camera's
0.05-3.0 metre depth range are discarded.

The median depth is calculated, and measurements differing
from it by 0.02 metres or more are rejected as outliers.
The median coordinates of the remaining pixels determine
the estimated marker location.

Using the intrinsic parameters from CameraInfo, the 3D
coordinates are calculated as follows:

- X = (u - cx) * Z / fx
- Y = (v - cy) * Z / fy
- Z = median valid depth

The resulting point is expressed in camera_optical_frame.

## Coordinate transformation and movement

The estimated marker position is stored in a PointStamped
message using the RGB image timestamp.

TF2 transforms the point from camera_optical_frame to
base_link at that timestamp.

The estimated marker positions are published on:

- /hw01/target_camera
- /hw01/target_base

An offset of 0.08 metres is added to the base-frame Z
coordinate to obtain the desired hovering position.

The provided ArmClient handles inverse kinematics and
trajectory execution. The node continues processing ROS
callbacks while waiting for the motion result.

## Failure handling

The solution rejects missing or ambiguous detections,
invalid depth measurements, inconsistent image dimensions,
invalid calibration and unavailable transforms.

Observations older than 0.5 simulated seconds or received
more than 2 wall-clock seconds ago are rejected.

Only one motion can be active at a time. Failed motions
are reported, and subsequent attempts require fresh
observations.

Pending motion is canceled if the node is interrupted.
This was also tested by interrupting an active trajectory,
which returned a Motion canceled result.

## Results

The same implementation was tested in all three presets.

| Preset | Estimated marker (m) | Tip error (m) | Result |
| --- | --- | --- | --- |
| 1 | (0.351, -0.149, 0.060) | 0.0012 | PASS |
| 2 | (0.402, 0.002, 0.060) | 0.0024 | PASS |
| 3 | (0.351, 0.153, 0.060) | 0.0028 | PASS |

All three successful tests maintained the required position
for two simulated seconds with clear contact streams.

The implementation uses camera observations rather than
preset-specific marker coordinates.

The detection method assumes that the target is visually
distinguishable by its red color. Severe occlusion or other
large red objects may make the detection unreliable.

## Build commands

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd assignments/hw01/environment
colcon build --symlink-install --base-paths src --executor sequential
source install/setup.bash
cd ../my_solution
colcon build --symlink-install --base-paths hw01_solution
source install/setup.bash
```

## Run commands

Terminal A, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

Terminal B, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
source assignments/hw01/my_solution/install/setup.bash

ros2 run hw01_sim check_reach --preset 1 &
sleep 1
ros2 run hw01_solution reach_marker --ros-args -p use_sim_time:=true
```

Repeat with presets 2 and 3, restarting the simulation
between runs.

