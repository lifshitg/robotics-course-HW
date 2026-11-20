# HW1 Report — Camera-Guided Reaching with a Two-Joint Arm

**Authors:** Roi Prives, Tal Leonov

Our package `hw01_solution` (in `my_solution/hw01_solution/`) detects the red marker in
the overhead RGB-D camera, reconstructs it in 3D in `camera_optical_frame`, transforms it to
`base_link` with TF2 at the image timestamp, adds the 0.08 m hover offset and sends a single
goal to the provided `ArmClient`. It passes the public `check_reach` observer for all three
presets with tip errors of 2–3 mm.

## 1. Build and run commands

Tested on Windows 11 + WSL2, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, Mesa 25.2.8
through the WSLg D3D12 driver on Intel UHD Graphics (`export GALLIUM_DRIVER=d3d12`).

```bash
# Every terminal, from the repository root
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash

# Build the shared environment once
(cd assignments/hw01/environment && colcon build --symlink-install --base-paths src --executor sequential)

# Build our package
(cd assignments/hw01/my_solution && colcon build --symlink-install)
source assignments/hw01/my_solution/install/setup.bash

# Offline unit tests (no simulator): 20 tests
python3 -m pytest -q assignments/hw01/my_solution/hw01_solution/test

# Terminal A: simulation (restart between presets)
ros2 launch hw01_sim simulation.launch.py preset:=1
# Terminal B: public checker, started before our node
ros2 run hw01_sim check_reach --preset 1 --timeout 90
# Terminal C: our implementation (use_sim_time is enabled inside the node)
ros2 run hw01_solution reach_target
```

Debugging helpers: `ros2 topic echo /hw01/target_camera`, `ros2 topic echo /hw01/target_base`,
and `ros2 run hw01_solution capture_frame <dir>`, which saves one synchronized frame,
the calibration and the camera TF for offline analysis.

## 2. Design and message flow

```
/camera/image_raw ──┐ ApproximateTimeSynchronizer (slop 0.05 s, best-effort QoS)
/camera/depth/...  ─┴─► on_pair ─► checks: |Δstamp| ≤ 0.05 s, age ≤ 0.5 s sim, calibration, frames
/camera/camera_info ─► cached K
                         │
                         ├─ detect_marker (RGB)  ─► pixel centroid + blob mask
                         ├─ select_depth (depth) ─► robust Z
                         ├─ backproject (K)      ─► PointStamped in camera_optical_frame ─► /hw01/target_camera
                         ├─ tf2 Buffer.transform ─► PointStamped in base_link           ─► /hw01/target_base
                         └─ ConsistencyFilter (3 consecutive estimates within 1 cm)
                                    │
                         state machine: WAITING ─► MOVING ─► SUCCEEDED / FAILED
                                    │ z + 0.08 m
                                    ▼
                          ArmClient.move_to_point ─► /hw01/move_to_point action
/hw01/measured_tip ─► independent check of the final tip position after success
```

The code is split so that the ROS-independent parts can be tested offline:

- `perception.py` holds detection, depth selection, back-projection and depth-encoding conversion.
- `logic.py` holds the staleness test and the consistency filter.
- `reach_node.py` is the ROS node: subscriptions, TF2, the state machine and the motion helper.

## 3. Marker detection

1. The image is converted with `cv_bridge` to `bgr8` according to its `encoding` field.
   The simulator publishes `rgb8`. We never assume the raw byte layout.
2. The image is converted to HSV and thresholded for red. Red wraps around hue 0, so we use two
   hue bands, H ≤ 8 or H ≥ 170 (OpenCV scale 0–179), with S ≥ 120 and V ≥ 60. The upper bound
   of the low band stays below the yellow arm links (H ≈ 12). In the real frame, all marker
   pixels had H = 0 and S = 255, with V between 157 and 235, which leaves a wide margin.
3. A 3×3 morphological opening removes isolated noisy pixels. The marker is only about 10 px
   wide (88 px area at 0.94 m), so a larger kernel would erase it.
4. Connected components are computed and the largest one is kept. A detection is
   **rejected as unreliable** in any of these cases:
   - the blob is smaller than 12 px or larger than 3000 px;
   - a second red blob is larger than 30% of the first, so the target is ambiguous;
   - the blob touches the image border, so it is cut off and its centroid is biased;
   - its fill ratio (area / bounding-box area) is below 0.55, where a full disk gives π/4 ≈ 0.785;
   - its bounding-box aspect ratio is above 1.6.

   The last two tests catch a marker that the arm partly occludes, which would otherwise give
   a biased centroid.
5. The image location is the centroid of the blob pixels, in OpenCV pixel-index coordinates.

## 4. Depth selection and 3D reconstruction

The depth image uses `32FC1` in metres. Its encoding is checked, and `16UC1` in millimetres
is also supported. Depth is distance along the optical Z axis, so it is used directly as Z.
We do not treat it as a Euclidean range.

**Depth selection:**

1. The blob mask is eroded by 3×3. Edge pixels mix the marker top (0.940 m) with the platform
   (0.950 m); the real frame showed values 0.940, 0.946 and 0.950 in the full mask.
   If erosion leaves fewer than 5 pixels, the full mask is used instead.
2. Invalid values are discarded: non-finite, ≤ 0.05 m (near clip), negative, and ≥ 3.0 m
   (far clip).
3. The estimate is rejected if fewer than 5 pixels, or fewer than 50% of the pixels, remain.
4. Z is the **median** of the remaining values. The median is robust to the few mixed
   or occluded pixels that survive.
5. The estimate is rejected if the median absolute deviation exceeds 2 cm, which means the
   pixels do not lie on one surface.

**Back-projection** uses the intrinsic matrix from `CameraInfo.k`. Nothing is hard-coded;
the simulator gives fx = fy = 277.13, cx = 160 and cy = 120:

  X = (u − cx)·Z / fx,  Y = (v − cy)·Z / fy,  Z = Z

This gives a point in `camera_optical_frame`, with +X right, +Y down and +Z forward. The
distortion coefficients are zero in the simulator.

## 5. TF2 direction and timing

- The reconstructed point is a `PointStamped` whose header is copied from the RGB image.
  Its `frame_id` is therefore `camera_optical_frame` and its stamp is the image timestamp.
  We check that the image, depth and calibration headers all name the same frame.
- `tf2_ros.Buffer.transform(point, 'base_link')` looks up the transform
  **from `camera_optical_frame` to `base_link` at the image stamp**. That is the transform
  which maps coordinates expressed in the optical frame into the base frame:
  p_base = R·p_optical + t. The lookup is non-blocking. If the transform is not available
  for that stamp, the frame is rejected and the next one is tried.
- The chain is static: `base_link ← world` comes from the URDF fixed joint,
  `world → camera_link` and `camera_link → camera_optical_frame` come from static
  publishers. Using the image stamp is still the correct practice, and with sim time it keeps
  the lookup consistent with the observation.
- Sanity check: in the optical frame the camera looks along +Z, which maps to −Z in the base
  frame. The reconstructed base-frame Z was 0.060 m in all three runs, which equals the
  marker-top height given in the assignment.
- The node enables `use_sim_time` itself, through a parameter override in its constructor.

## 6. Synchronization, staleness and failure handling

| Situation | Behaviour |
| --- | --- |
| RGB/depth stamps differ > 0.05 s | Synchronizer slop is 0.05 s; we re-check explicitly and reject |
| No `/clock` yet, no calibration yet | Reject frame (seen at start-up in preset 1: "no camera calibration yet") |
| Observation > 0.5 s old in sim time, or stamped in the future | Reject as stale |
| No camera pair for 2 s of wall time | Wall-clock watchdog timer resets the target history and warns |
| Marker absent, too small, ambiguous, cut off, partly occluded | Reject detection (section 3) |
| Invalid, sparse or inconsistent depth | Reject (section 4) |
| Transform unavailable | Reject frame (seen at start-up in preset 2: TF trees not yet connected) |
| Hover point not within 0.01 m of z = 0.14 | Not sent; logged as a perception/TF error |
| Helper not ready | Wait; no goal sent |
| Motion rejected or failed | Logged; target history cleared; a fresh estimate is required for a retry, at most 3 attempts |
| Ctrl+C | Custom signal handling keeps the ROS context alive, so `ArmClient.cancel()` is sent and spun out before shutdown |

**No reuse of old targets.** Every rejected frame resets the consistency filter. A goal is
sent only after 3 consecutive fresh frames agree within 1 cm, and the goal uses their mean.
A missing observation can therefore never trigger a motion to an earlier position.

**One motion at a time.** The node sends one goal and switches to `MOVING`. While moving it
keeps publishing estimates for debugging but sends nothing. Near the end of the motion the
arm covers the marker. This is expected and is not treated as a failure.

**Physical check.** Action success is not proof of the final position. One simulated second
after success, the node compares `/hw01/measured_tip` with the commanded hover point and logs
the error.

## 7. Results

Each preset was run in a freshly launched simulation, with the checker started before our
node and the same code in every run.

| Preset | Estimated marker in `base_link` (m) | Hover goal (m) | Node's tip check | `check_reach` |
| --- | --- | --- | --- | --- |
| 1 | (0.352, −0.148, 0.060) | (0.352, −0.148, 0.140) | 0.0000 m | **PASS**, error 0.0033 m, 2 s hold, no contact |
| 2 | (0.401, 0.002, 0.060) | (0.401, 0.002, 0.140) | 0.0000 m | **PASS**, error 0.0022 m, 2 s hold, no contact |
| 3 | (0.352, 0.151, 0.060) | (0.352, 0.151, 0.140) | 0.0000 m | **PASS**, error 0.0024 m, 2 s hold, no contact |

Each run took about 1–2 s (wall time) from start-up to sending the goal; the motion completed about 10–12 s (wall time) later.

**Where the remaining 2–3 mm comes from.** The arm reaches the commanded point exactly; the
node's own check gives below 0.1 mm. The residual is therefore perception error. Analysis of a
captured frame shows a consistent shift of about half a pixel. Gazebo reports cx = W/2 = 160,
whereas under the pixel-center convention used for the centroid the optical axis would sit at
159.5. At Z = 0.94 m, half a pixel is about 1.7 mm. Sampling the marker at roughly 10 px
across adds discretization error. We deliberately do not correct for this offset, because the
correction would be tuned against the known answer rather than derived from the calibration.

## 8. Testing

- **20 offline unit tests** (`test/`) on synthetic scenes built with the real camera
  intrinsics. They cover:
  - centroid accuracy, several red shades, and the yellow arm not being taken for red;
  - rejection of occluded, ambiguous and cut-off markers;
  - depth filtering of NaN, inf, 0 and negative values, all-invalid and out-of-range depth;
  - back-projection and invalid intrinsics;
  - end-to-end reconstruction error under 3 mm;
  - depth encodings, staleness, and the consistency filter, including reset after a missing
    observation.
- **A captured real frame**, from `capture_frame`, was run through the same pipeline offline
  to check the thresholds, the depth statistics and the TF direction.
- **Live runs** against `check_reach` for all three presets (section 7).
