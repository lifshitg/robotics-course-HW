# HW01 Report: Camera-Guided Reaching with a Two-Joint Arm

**Group:** Yoav Levin, Ziv Zaltsman     
**Package:** `hw01_solution` (`assignments/hw01/my_solution/hw01_solution/`)

## Overview

Our solution is a ROS 2 Python package, `hw01_solution`, with two nodes that split perception from motion:

- **`point_detector`** subscribes to the camera's RGB, depth and calibration topics. For every synchronized RGB/depth pair it detects the red marker, reconstructs its 3D position in `camera_optical_frame` and transforms it into `base_link` with TF2. It publishes the marker position, before the hover offset, on `/hw01/target_camera` and `/hw01/target_base`. It never commands the arm.
- **`arm_motion`** subscribes to `/hw01/target_base`. When a fresh target arrives, it adds the 0.08 m hover offset to Z, which brings the goal to the arm's fixed tip height of 0.14 m (0.06 + 0.08). It then sends the goal to the provided motion helper through `ArmClient` (the `/hw01/move_to_point` action), reports whether the motion succeeded and exits. It sends exactly one goal per run.

We separated the two roles so that perception can be run and checked on its own, by watching the published points, without moving the arm. This also keeps the rule "only one motion at a time" in a single, simple place. Both nodes run with `use_sim_time`, so all timestamps and ages are measured in the simulation's clock.


## 1. Marker Detection

### Receiving the camera data

The `point_detector` node subscribes to three camera topics: `/camera/image_raw` (RGB), `/camera/depth/image_raw` (depth) and `/camera/camera_info` (calibration). All three subscriptions use `qos_profile_sensor_data`, which is best-effort. For camera data this is also the sensible choice: if a frame is lost, waiting for it to be resent is worse than simply using the next one.

The RGB and depth images arrive on separate topics, so a received depth frame is not necessarily from the same moment as the latest RGB frame. Since we later read the depth at the exact pixel where the marker was found, the two images must describe the same scene. We therefore pair them with `message_filters.ApproximateTimeSynchronizer`, which only calls our `on_images` callback with an RGB and a depth image whose timestamps differ by at most 0.05 simulated seconds (`sync_slop` parameter).

`camera_info` is handled separately, through its own callback (`on_info`) that just stores the most recent message. The calibration (the intrinsic matrix K) describes the camera's lens and sensor, which do not change during the run, so it does not need to be time-matched to each image. `on_images` skips the frame until at least one calibration message has been received.

### Decoding the image

A ROS `Image` message carries the pixels as a flat byte array, together with fields that describe how to read it. In `decode_rgb` we do not assume a fixed layout. We first check the `encoding` field and accept only `rgb8` or `bgr8`; any other encoding raises an error and the frame is skipped. We then use `step`, the actual number of bytes per row, to split the data into rows and drop any padding at the end of each row, and reshape the result into a height × width × 3 array. Finally, if the image is `rgb8`, we reverse the channel order to get BGR, which is the order OpenCV expects.

### Finding the marker

The detection is implemented in `detect_red`:

1. **Colour space.** The BGR image is converted to HSV with `cv2.cvtColor`. HSV separates the colour itself (Hue) from its strength (Saturation) and brightness (Value). Shadows and lighting changes therefore mostly affect V, while H stays "red". In BGR all three channels change together when the lighting changes.
2. **Thresholding.** In OpenCV, Hue is a circle from 0 to 179, and red sits at the point where it wraps around. One range cannot cover both ends, so we use two, [0, 10] and [170, 179], and combine the two masks with a logical OR. Both ranges also require S ≥ 100, which rejects washed-out, greyish pixels, and V ≥ 60, which rejects pixels too dark for their hue to be reliable.
3. **Noise removal.** A morphological opening with a 3×3 kernel (erosion followed by dilation) removes isolated red pixels and thin specks, while the marker itself keeps almost its original shape.
4. **Selecting the marker.** `cv2.connectedComponentsWithStats` splits the mask into separate blobs and returns the area and centroid of each one. We keep only the largest blob.
5. **Output.** The function returns the mask of that blob alone, which is used in Section 2 to read depth only from marker pixels, and its centroid (u, v) in pixels.

**Missing and unreliable detections.** If the mask contains no red component at all, the marker is treated as missing. If the largest blob is smaller than 25 pixels (`min_area_px`), it is treated as unreliable, since it is most likely noise or a marker that is almost completely hidden. In both cases `detect_red` returns `None`, `on_images` logs `No reliable red marker detected.` and skips the frame, so nothing is published and no earlier detection is reused. We observed this case in practice: once the arm hovers above the marker, it covers it in the overhead camera view (see Section 5).

## 2. Depth Lookup & 3D Reconstruction

A pixel location alone does not give a 3D position: the marker could be anywhere along the ray that passes from the camera through that pixel. The aligned depth image provides the missing information. Each depth pixel holds a `32FC1` value in metres, which is the distance along the camera's optical Z axis (not the straight-line distance to the point). This means the depth value can be used directly as the Z coordinate of the point.

In `decode_depth` we convert the raw bytes into a float array, using `step` (bytes per row) to correctly skip any row padding.

**Depth selection.** Instead of reading the depth at a single pixel, `pick_depth` uses all the pixels inside the marker's blob mask from Section 1. We first discard invalid values: non-finite readings (NaN or infinity), values at or below `depth_min` (0.05 m, which also removes zero and negative values) and values at or above `depth_max` (5.0 m). If fewer than 30% of the blob's pixels remain valid (`min_valid_depth_frac`), the depth is considered unreliable and the frame is skipped. Otherwise we take the median of the remaining values.

**Back-projection.** The intrinsic parameters fx, fy, cx and cy are read from the calibration matrix K in the latest `CameraInfo` message, not hardcoded. Using the pinhole camera model, the marker centroid (u, v) and its depth Z are converted into a 3D point in `camera_optical_frame`:

```
Z = depth
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

These formulas come from similar triangles: the offset of the pixel from the image centre, divided by the focal length, equals the offset of the point divided by its depth. The result is stored in a `PointStamped` whose `frame_id` and `stamp` are copied from the RGB image header, so the point keeps the frame and the moment of the observation.

## 3. TF Direction & Timing

The point from Section 2 is expressed in `camera_optical_frame`, but the motion helper expects goals in `base_link`, the frame attached to the arm's base. We do not write the camera pose into our code by hand. Instead, the node keeps a `tf2_ros.Buffer` that a `TransformListener` fills from `/tf` and `/tf_static`, and we ask TF2 for the transform. In this simulation the chain is `base_link` ← `world` ← `camera_link` ← `camera_optical_frame`, and all of its links are fixed.

**Direction.** We call `tf_buffer.transform(cam_pt, 'base_link')`. We convert a point that is known relative to the camera into the same physical point described relative to the arm's base.

**Timing.** Because the `PointStamped` carries the image's timestamp, TF2 returns the transform that was valid at the moment the image was captured, not the latest available one. In our setup the camera is fixed, so the two would be equal. However, if the camera or the robot were moving, combining an older observation with a newer transform would place the point in the wrong position. Using the image time keeps the measurement and the transform consistent. We use a timeout of zero so the callback never blocks. If the transform is not available for that time (`LookupException`, `ConnectivityException` or `ExtrapolationException`), we log a warning and skip the frame instead of guessing.

After a successful transform, both points are published before the hover offset is added: the camera-frame point on `/hw01/target_camera` and the base-frame point on `/hw01/target_base`. As a sanity check, the base-frame Z of the detected point is 0.060 m in our runs, which matches the known height of the marker's top surface.

**Freshness.** Both nodes run with `use_sim_time`, so their clocks follow the simulation's `/clock`. `point_detector` drops any image whose timestamp is more than 0.5 simulated seconds older than the current simulation time. `arm_motion` checks again before sending a goal: the stored target must be at most 0.5 s old in simulation time and must have arrived within the last 2 wall-clock seconds. We need both clocks because a check based only on simulation time cannot notice that the simulation has frozen: the current time and the stored timestamp stop together, so the data would always look fresh. The wall-clock check keeps running and catches this case. A target that fails either check is discarded and never reused.

## 4. Failure Handling

**Perception failures (`point_detector`).** These cases are handled in `on_images`, in the order they are checked:

| Failure | How it is detected | Response |
|---|---|---|
| No calibration yet | `self.camera_info is None` | skip frame |
| Stale image | image age > 0.5 s simulation time | skip frame |
| Unsupported RGB encoding | `decode_rgb` raises `ValueError` (only `rgb8` / `bgr8` accepted) | log error, skip frame |
| Marker missing | no red connected component in the mask | skip frame |
| Unreliable detection | largest blob smaller than 25 px (`min_area_px`) | skip frame |
| Invalid depth | fewer than 30% of blob pixels have finite depth in (0.05, 5.0) m | skip frame |
| Transform unavailable | `Lookup`/`Connectivity`/`ExtrapolationException` | skip frame |

**Motion failures (`arm_motion`).** These cases are handled in the main loop of `arm_motion`, before and after the goal is sent:

| Failure | How it is detected | Response |
|---|---|---|
| Stale target | target older than 0.5 s simulation time, or arrived more than 2 wall-clock seconds ago | discard target, wait for a new one |
| Reusing an earlier target | target is cleared when it is sent or discarded | an old target is never sent again |
| Second concurrent motion | a goal is only sent while `motion is None` (one-shot node); `ArmClient` also refuses a second goal | only one motion is ever active |
| Helper not ready | `arm.ready()` is false | keep the target, retry on the next loop |
| Corrupting the stored target | hover offset is added to a `deepcopy` | stored message stays unchanged |
| Motion fails or is rejected | `result().success` is false | log the helper's message, print `FAIL`, exit code 1 |
| No target at all | no successful motion within 100 wall-clock seconds | stop, print `FAIL`, exit code 1 |
| Interruption (Ctrl+C) or timeout | `KeyboardInterrupt` / loop ends | `finally`: `arm.cancel()`, brief spin to send the request, shut down |

On success the node prints `PASS` and exits with code 0.

We also rely on the public checker rather than on the action result alone, because a successful action does not prove that the tip physically reached the target (see Section 5).

## 5. Results

### Preset 1

With the simulation launched under `preset:=1` and `check_reach --preset 1` running
alongside `point_detector` and `arm_motion`, the pipeline produced the following:

- **Detected marker pixel:** (u=202.5, v=104.8)
- **Selected depth:** 0.940 m
- **3D position in `base_link`:** (X=0.352, Y=-0.144, Z=0.060) m
- **Commanded hover target (after +0.08 m offset):** z = 0.140 m
- **`arm_motion` result:** `PASS: arm motion to marker` (helper message: `Trajectory completed`)
- **`check_reach` result:** `PASS: tip error 0.0033 m, held 2 simulated seconds; live contact streams clear`

The tip settled 3.3 mm from the required hover point, well within the 0.02 m tolerance, with no platform or marker contact during the hold period.

### Preset 2

With the simulation restarted under `preset:=2` and `check_reach --preset 2`
running alongside `point_detector` and `arm_motion`:

- **Detected marker pixel:** (u=159.5, v=90.1)
- **Selected depth:** 0.940 m
- **3D position in `base_link`:** (X=0.401, Y=0.002, Z=0.060) m
- **Commanded hover target (after +0.08 m offset):** z = 0.140 m
- **`arm_motion` result:** `PASS: arm motion to marker` (helper message: `Trajectory completed`)
- **`check_reach` result:** `PASS: tip error 0.0022 m, held 2 simulated seconds; live contact streams clear`

**Observations from this run:**

1. *Partial occlusion during the motion.* About four seconds into the motion, the detected pixel moved from (159.5, 90.1) to (157.3, 89.3) and the base-frame estimate shifted from (0.401, 0.002) to (0.404, 0.009) m. The arm was passing above the marker and covering part of it, so the centroid of the visible red blob moved. Because `arm_motion` sends only one goal and ignores targets that arrive while a motion is active, this shifted estimate was never used as a goal.
2. *Full occlusion after arrival.* Once the tip was hovering above the marker, `point_detector` logged `No reliable red marker detected.` The arm now blocks the overhead camera's view of the marker. This is handled as a normal missing-detection case: the frame is skipped and nothing is published.

### Preset 3

With the simulation restarted under `preset:=3` and `check_reach --preset 3`
running alongside `point_detector` and `arm_motion`:

- **Detected marker pixel:** (u=115.5, v=104.6)
- **Selected depth:** 0.940 m
- **3D position in `base_link`:** (X=0.352, Y=0.151, Z=0.060) m
- **Commanded hover target (after +0.08 m offset):** z = 0.140 m
- **`arm_motion` result:** `PASS: arm motion to marker` (helper message: `Trajectory completed`)
- **`check_reach` result:** `PASS: tip error 0.0024 m, held 2 simulated seconds; live contact streams clear`

In this run the estimate stayed identical in every logged frame until the arm reached the marker. As in Preset 2, `point_detector` then reported `No reliable red marker detected.` because the hovering tip covers the marker in the overhead camera view.

### Summary

| Preset | Detected pixel (u,v) | base_link position (X,Y,Z) [m] | check_reach result |
|---|---|---|---|
| 1 | (202.5, 104.8) | (0.352, -0.144, 0.060) | PASS, tip error 0.0033 m |
| 2 | (159.5, 90.1) | (0.401, 0.002, 0.060) | PASS, tip error 0.0022 m |
| 3 | (115.5, 104.6) | (0.352, 0.151, 0.060) | PASS, tip error 0.0024 m |

All three presets passed with tip errors between 2.2 and 3.3 mm, well under the 0.02 m tolerance, and with no contact detected. In all three presets the marker's base-frame Z came out as 0.060 m, which matches the known top height of the marker. This confirms that the detection, depth reconstruction and TF steps work together, and that the target is derived purely from camera data.

## 6. Build & Run Commands

### a. Build the shared environment (once)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robotics-course-HW/assignments/hw01/environment
colcon build --symlink-install --base-paths src --executor sequential
```

### b. Build hw01_solution (once)

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
colcon build --symlink-install --base-paths assignments/hw01/my_solution/hw01_solution
```

### c. Run (repeated for preset 1, 2, and 3)

#### Terminal 1 - simulation:

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=<1|2|3>
```

#### Terminal 2 - public checker (started before our nodes, per preset):

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 run hw01_sim check_reach --preset <1|2|3>
```

#### Terminal 3 - point detector:

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
source install/setup.bash
ros2 run hw01_solution point_detector
```

#### Terminal 4 - arm motion:

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
source install/setup.bash
ros2 run hw01_solution arm_motion
```

`check_reach` gives up after 90 seconds by default, so `arm_motion` should be started soon after the checker. `arm_motion` exits on its own after reporting `PASS` or `FAIL`.

Between presets: Ctrl+C in Terminal 3, then Ctrl+C in Terminal 1, and wait for full exit. Before relaunching, check that no simulation server is left running:

```bash
pgrep -af "gz sim"   # should print nothing; otherwise: kill <pid>
```