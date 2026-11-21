# HW1 report: camera-guided reaching

The solution is a single ROS 2 Python node, `reach_node`, in the package
`my_solution/hw01_reach`. It finds the red marker in the camera image, reconstructs
its 3D position, transforms it into `base_link` with TF2 and sends the tip to a point
0.08 m above it through the provided `ArmClient`.

## Build and run

From the repository root, in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
```

Build once (and after changing `package.xml` or `setup.py`):

```bash
cd assignments/hw01/my_solution
colcon build --symlink-install
cd ../../..
source assignments/hw01/my_solution/install/setup.bash
```

Run, for preset `N` (1, 2 or 3), restarting the simulation between presets:

```bash
# terminal A
ros2 launch hw01_sim simulation.launch.py preset:=N
# terminal B (start before the node)
ros2 run hw01_sim check_reach --preset N
# terminal C (also source my_solution/install/setup.bash)
ros2 run hw01_reach reach_node
```

The node enables `use_sim_time` itself.

## Message flow

```
/camera/image_raw ───────┐ ApproximateTimeSynchronizer
/camera/depth/image_raw ─┘ (slop 0.05 s)  ──► detect ──► depth ──► pinhole ──► /hw01/target_camera
/camera/camera_info ──► latest K                                       │
/tf, /tf_static ──► tf2 Buffer ─────────────────────────────────► transform ──► /hw01/target_base
                                                                        │ z + 0.08
                                                                        ▼
                                                   ArmClient.move_to_point ──► /hw01/move_to_point
                                                                        │ on success
/hw01/measured_tip ─────────────────────────────────────────────────► tip check (done or retry)
```

All camera subscriptions use `qos_profile_sensor_data` (best effort), matching the
camera bridge. RGB and depth are paired by `message_filters.ApproximateTimeSynchronizer`
with a 0.05 s slop, so every processed pair is within 0.05 simulated seconds.

## Detection

The RGB message is converted with `cv_bridge` to `bgr8` (it reads `msg.encoding`, so
`rgb8` and `bgr8` both work) and then to HSV. Red is thresholded with two hue ranges,
because red wraps around hue 0: H 0–10 or 170–179, S ≥ 120, V ≥ 70. The saturation bound
matters because gray pixels get hue 0 in OpenCV. In the simulation the marker measures
H = 0, S = 255, V 156–235; the blue and yellow links are at H ≈ 108 and H ≈ 27 and the
gray surfaces have S ≈ 0, so the bounds leave a wide margin.

Connected components of the mask are filtered. A blob is a valid marker if:

- its area is 40–400 px (the full marker is about 91 px, an ~11 px disk), and
- it fills at least 60% of its bounding box (a full disk fills π/4 ≈ 78%).

The detection is accepted only if exactly one blob passes. Otherwise, with no marker,
noise or an ambiguous view, the frame is skipped. The marker pixel is the blob's
centroid, and the blob's mask is kept for the depth step.

## Depth reconstruction

The depth image (`32FC1`, metres; `16UC1` millimetres is also accepted, any other
encoding is rejected) is read with `cv_bridge` in passthrough mode. It must have the
same size as the RGB image (aligned).

**Depth selection:** take the depth values under the marker's mask, discard
non-finite, ≤ 0.1 m and ≥ 3 m values, require at least half of the marker's pixels to
be valid, and use the **median** of the valid values. The median ignores the marker's
edge pixels, which mix in the platform behind it (0.950 m versus 0.940 m on the
marker).

The point in `camera_optical_frame` comes from the pinhole model with K from
`camera_info` (fx = fy = 277.13, cx = 160, cy = 120 here):

```
X = (u − cx) · Z / fx,   Y = (v − cy) · Z / fy,   Z = depth
```

Depth is the distance along the optical Z axis, so it is used as Z directly. It is not
the Euclidean distance along the ray. The resulting `PointStamped` copies the image
header, so it carries `frame_id = camera_optical_frame` and the image timestamp.

## TF direction and timing

The point is transformed **from** `camera_optical_frame` (taken from its header)
**to** `base_link` with `tf2_ros.Buffer.transform`, at the **image timestamp** (also
from the header). The TF tree is
`camera_optical_frame → camera_link → world → base_link`, and tf2 combines and inverts
the transforms along that path. These links are static (`/tf_static`), so the camera
pose does not change during a run. Looking the transform up at the image time is
still what keeps the result correct for a moving camera.

The lookup has no timeout, so it returns immediately. If the transform is not
available yet (right after startup) it raises `TransformException` and the frame is
skipped. On every preset the marker comes out at z = 0.060 in `base_link`, its
surface height, which checks the depth and the transform together. The commanded tip
point is that point with z + 0.08 = 0.140.

## Failure handling

- **No goal from bad data.** The node skips a frame and sends nothing if there is:
  - no `/clock` yet, or the observation is more than 0.5 simulated seconds old,
  - no camera info yet,
  - no single valid marker blob,
  - too little valid depth,
  - no transform available.

  An old target is never reused: a goal is only built from the frame being processed.
- **Camera timeout (2 wall-clock seconds).** Goals are only built from the frame being
  processed, so a stalled camera stream can never produce a goal. A frame that arrives
  after a wall-clock gap but is less than 0.5 s old in sim time is treated as fresh,
  because sim time measures how current it is.
- **One motion at a time.** No new goal is sent while a motion future is pending, and
  goals are only sent when `arm.ready()`.
- **Result reporting.** A done-callback on the future logs `success` and `message`.
- **Physical check.** Action success alone does not prove that the tip arrived.
  After a successful motion, the node waits 0.5 simulated seconds and compares the
  latest `/hw01/measured_tip` (from Gazebo physics) with the commanded point. The job
  counts as done only if the tip is within 1 cm, which is stricter than the checker's
  2 cm.
- **Retries.** A failed motion, a tip that is off target, or a missing or stale tip
  measurement allows a retry from a new fresh observation, up to 3 attempts. After a
  verified success the node stops sending goals.
- **Ctrl+C.** rclpy's signal handler is disabled so that the ROS context survives
  Ctrl+C. The node then calls `arm.cancel()`, spins until the cancellation is confirmed
  (at most 2 s) and shuts down.

Known limitations:

- A marker cut by a straight edge (for example half hidden by the arm) still fills
  about π/4 of its bounding box, so the shape check does not catch it. The first
  target is taken before the arm moves, so this does not affect it. A retry, though,
  happens with the tip hovering over the marker. In a forced-retry test the partly
  hidden marker moved the estimate by about 3 mm. If the marker is hidden completely,
  no retry is possible, and the node waits instead of reusing the old target.
- If the simulation is restarted while the node keeps running, the node does not
  reset. Restart the node with the simulation.

## Results

Each preset was run with a fresh simulation, `check_reach` started first, then the
node. All passed:

| Preset | Marker in `base_link` (m) | Commanded tip (m) | `check_reach` |
| --- | --- | --- | --- |
| 1 | (0.352, −0.148, 0.060) | (0.352, −0.148, 0.140) | PASS, error 2.9 mm, no contact |
| 2 | (0.401, 0.002, 0.060) | (0.401, 0.002, 0.140) | PASS, error 2.2 mm, no contact |
| 3 | (0.352, 0.151, 0.060) | (0.352, 0.151, 0.140) | PASS, error 2.1 mm, no contact |

In every run the first goal succeeded ("Trajectory completed"). The node's own
`/hw01/measured_tip` check put the tip within 0.1 mm of the *commanded* point. The
remaining 2–3 mm in `check_reach`, which compares against the true marker position,
is therefore perception error in the marker estimate, not motion error.

Failure handling was also checked by hand:

| Scenario | Observed |
| --- | --- |
| Ctrl+C during the motion | Goal cancelled ("Motion canceled"); arm stopped at (0.268, −0.121), short of the target, and stayed there |
| Simulation paused before the node started | No goals sent; after resuming, normal reach |
| Node started with no simulation | No goals, no errors; clean exit on Ctrl+C |
| Tip check forced to fail (tolerance set below zero in a test copy) | "tip is … from the target; will retry"; a second goal was sent from a fresh observation |
