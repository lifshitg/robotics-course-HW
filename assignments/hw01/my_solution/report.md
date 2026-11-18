# HW1 report: camera-guided reaching

Package: [`hw01_solution`](src/hw01_solution/). The node `marker_reacher`
([marker_reacher.py](src/hw01_solution/hw01_solution/marker_reacher.py)) extends
`CameraListener` ([camera_listener.py](src/hw01_solution/hw01_solution/camera_listener.py)),
which handles the camera input and marker detection.

## 1. Overview

For every synchronized RGB/depth pair the node runs these steps:

1. Find the red marker in the RGB image and take its pixel center `(u, v)`.
2. Take the median depth `Z` of the marker pixels.
3. Back-project `(u, v, Z)` into a point in `camera_optical_frame` and publish it on
   `/hw01/target_camera`.
4. Transform the point into `base_link` with TF2 and publish it on `/hw01/target_base`.
5. Add 0.08 m to Z and send the goal to the motion helper, if no motion is running
   and none has succeeded yet.

If any step fails, the frame is rejected, no goal is sent and the previous target is
discarded.

The camera topics use best-effort sensor-data QoS. RGB and depth are paired by
header timestamp with `ApproximateTimeSynchronizer` (slop 0.05 s). The node runs
with `use_sim_time=True`.

### Simulation time and wall-clock time

The node uses two clocks:

- **Simulation time** (from `/clock`) is used for anything compared with message
  timestamps: RGB/depth pairing, the 0.5 s staleness check, the TF lookup and the
  age of `/hw01/measured_tip`. Message stamps are in simulation time, and the
  simulation can run slower than real time, so wall-clock time would give wrong ages.
- **Wall-clock time** is used for the 2 s camera-silence check and the 2 s retry
  delay. If the simulation freezes, simulation time stops too, so a simulation-time
  timer would never fire. A wall-clock timer still notices that no camera data is
  arriving.

## 2. Marker detection

1. **Decode.** `cv_bridge` converts the image to `rgb8` using its declared encoding.
2. **Red mask.** A pixel is red if `R ≥ 150`, `G ≤ 100` and `B ≤ 100`.
3. **One region.** `cv2.connectedComponentsWithStats` groups red pixels into regions.
   Regions under 20 px are ignored as noise. Exactly one region must remain; if there
   are several, the frame is rejected rather than guessing.
4. **Shape checks (occlusion).** A partly hidden disk loses its round shape, so the
   region is rejected if:
   - it touches the image border (it may be cut off);
   - its bounding box is elongated (width/height ratio below 0.75);
   - it fills less than 55% or more than 90% of its bounding box (a disk fills about 79%);
   - its circularity `4πA/P²` is below 0.70 (1.0 is a perfect circle).
5. **Center.** The subpixel centroid of the region gives `(u, v)`.

## 3. Depth selection and 3D reconstruction

**Depth selection.** The depth image must be `32FC1` (metres) and have the same size
and frame as the RGB image. Depth is read only at the pixels of the accepted marker
region. A value is valid if it is finite and between 0.05 m and 3.0 m, which rejects
NaN, infinity, zero, negative and out-of-range values. The marker's depth is the
**median** of the valid values. The marker top is flat and faces the camera, so its
pixels have almost the same depth. The median ignores the few edge pixels that hit
the platform, which would pull a mean or a minimum off. If no value is valid, the
frame is rejected.

**Reconstruction.** The intrinsics `fx, fy, cx, cy` come from `CameraInfo.k`; nothing
is hardcoded. The calibration frame must match the image frame. Depth is distance
along the optical Z axis, so:

```text
Z = median marker depth
X = (u − cx) · Z / fx
Y = (v − cy) · Z / fy
```

The result is a `PointStamped` in `camera_optical_frame` that carries the RGB image's
timestamp.

## 4. TF direction and timing

- **Direction.** `lookup_transform('base_link', 'camera_optical_frame', stamp)` returns
  the transform that maps points from the optical frame into `base_link`.
  `do_transform_point` applies it. No transform is written by hand.
- **Timing.** The lookup uses the image capture timestamp, not the latest time, so
  the point and the transform describe the same moment. The timeout is zero, so the
  camera callback never blocks. If the transform is not available, that frame is
  rejected and the next frame tries again.
- **Check.** In preset 1 the optical point (0.148, −0.052, 0.940) became
  (0.352, −0.148, 0.060) in `base_link`. Z = 0.060 m matches the known height of the
  marker top.

## 5. Motion

The goal is the base-frame marker point plus 0.08 m in Z (about 0.14 m), sent with
`ArmClient.move_to_point`. A goal is sent only when:

- it comes from the current fresh frame;
- no motion is active (`arm.busy` is false), so at most one motion runs at a time;
- no motion has succeeded yet in this launch (each launch has one stationary marker).

The result arrives in a done-callback, so the node keeps spinning while the arm
moves. The node logs the helper's `success` and `message`. Because action success
does not prove the physical result, it also compares the goal with
`/hw01/measured_tip`; a tip more than 0.02 m away counts as a failure. A failed motion
is retried from a new fresh frame after 2 s, at most 3 times.

## 6. Failure handling

| Situation | What the node does |
|---|---|
| No red region, only tiny specks, or several red regions | Reject the frame |
| Region touches the border or is not round (likely occluded) | Reject the frame |
| No valid depth inside the marker | Reject the frame |
| Image cannot be decoded, wrong depth encoding, or RGB/depth size or frame mismatch | Log an error, reject the frame |
| No calibration yet, invalid calibration, or calibration frame mismatch | Reject the frame |
| Image timestamp more than 0.5 simulated s old, or no `/clock` yet | Reject as stale |
| No camera data for 2 wall-clock seconds | A wall-clock timer clears the stored target and logs a warning |
| TF lookup fails | Reject the frame; nothing is published on `/hw01/target_base` |
| Any rejected frame | Clear the stored target, so an old target is never reused |
| Helper rejects the goal or the motion fails | Log the message; retry after 2 s, at most 3 times |
| Helper reports success but the measured tip is more than 0.02 m away | Count it as a failure |
| Ctrl+C or SIGTERM | Cancel the pending motion, wait up to 2 s for the cancel to go through, then shut down |

ROS's default signal handler would stop the node before the cancel request could be
sent, so the node installs its own SIGINT/SIGTERM handler.

Once the arm hovers above the marker it partly hides it from the camera. The node
then logs occlusion rejections; this is expected, and no new goal is sent.

## 7. Results

Each preset was run on a fresh simulation with the same code, with `check_reach`
started before the node.

| Preset | Pixel (u, v) | Depth Z (m) | Camera point (m) | Base point (m) | Goal (m) | Helper | `check_reach` |
|---|---|---|---|---|---|---|---|
| 1 | (203.55, 104.73) | 0.940 | (0.148, −0.052, 0.940) | (0.352, −0.148, 0.060) | (0.352, −0.148, 0.140) | success | **PASS**, error 0.0029 m, no contact |
| 2 | (159.46, 90.05) | 0.940 | (−0.002, −0.102, 0.940) | (0.402, 0.002, 0.060) | (0.402, 0.002, 0.140) | success | **PASS**, error 0.0024 m, no contact |
| 3 | (115.36, 104.71) | 0.940 | (−0.151, −0.052, 0.940) | (0.352, 0.151, 0.060) | (0.352, 0.151, 0.140) | success | **PASS**, error 0.0023 m, no contact |

In all three runs the tip stayed within the 0.02 m tolerance for 2 simulated seconds.
The remaining 2–3 mm error comes from estimating the center from about 90 marker
pixels, where one pixel covers about 3.4 mm at this depth.

## 8. Build and run commands

Run all commands from the repository root. The shared environment is built once, as
described in the installation guide.

Build:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
cd assignments/hw01/my_solution
colcon build --symlink-install --base-paths src
cd ../../..
```

Terminal A, simulation (use `preset:=2` or `preset:=3` for the other runs):

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

Terminal B, reach checker, started before the node:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 run hw01_sim check_reach --preset 1
```

Terminal C, the solution:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
source assignments/hw01/my_solution/install/setup.bash
ros2 run hw01_solution marker_reacher
```

Stop all terminals with Ctrl+C and restart all three for each preset.
