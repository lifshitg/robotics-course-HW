# HW1 - Camera-Guided Reaching

**Members:** Yuval Hadar (315943019), Moriel Turgeman (208322305)

Package: `assignments/hw01/my_solution/marker_reacher` (ament_python), executable `reacher_node`.

## Build and run

In every terminal, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
```

Build once:

```bash
cd assignments/hw01/my_solution
colcon build --symlink-install
cd ../../..
```

For each preset N (1, 2, 3), with a new simulation each time:

```bash
# terminal A: simulation
ros2 launch hw01_sim simulation.launch.py preset:=N

# terminal B: reach check (start it before our node)
ros2 run hw01_sim check_reach --preset N --timeout 90

# terminal C: our node
source assignments/hw01/my_solution/install/setup.bash
ros2 run marker_reacher reacher_node
```

The node sets `use_sim_time` to true by itself. We stop the node and the simulation with Ctrl+C before the
next preset.

## Message flow

RGB and depth are subscribed with best-effort QoS (`qos_profile_sensor_data`) and paired by timestamp with
`ApproximateTimeSynchronizer` (slop 0.05 s). The calibration comes from `/camera/camera_info`. For every pair:
detection -> depth -> 3D point in `camera_optical_frame` (published on `/hw01/target_camera`) -> TF2 to
`base_link` (published on `/hw01/target_base`) -> goal to the motion helper.

## Detection

We convert the image to `bgr8` with cv_bridge (the camera publishes `rgb8`) and then to HSV. In HSV the color (H)
is separate from the brightness (V), so shading barely changes H. Red is at both ends of the hue range, so we use
two ranges, H in [0, 10] and [170, 179] with S and V >= 100, and combine the two
masks with OR. The marker position (u, v) is the centroid of the mask from the image moments: u = m10/m00, v = m01/m00.

A detection is used only if the mask has at least 45 red pixels. The whole marker is about 90 pixels (we measured
88-91 in all presets), so 45 is about half. This rejects noise and a missing or mostly hidden marker. It also
matters after the arm arrives: the tip hides most of the marker (21 and 3 pixels left in presets 2 and 3), and
these frames are rejected.

## Depth reconstruction

The depth image is `32FC1` (meters) and aligned with the RGB image. We check the encoding before using it.

Depth selection: we take all the depth values under the marker mask, keep only finite values between 0.05 m and
3.0 m (the near and far clip planes of the simulated camera) and use their median. This removes NaN, inf, zero, negative
and out-of-range values, and if nothing is left we skip the frame. The median also ignores edge pixels that see
the platform behind the marker. In our runs the values were 0.940-0.950 m and the median was 0.940 m.

The depth value is Z, the distance along the optical axis (not the distance from the camera center). With the
intrinsics from `CameraInfo.k` (fx = fy = 277.13, cx = 160, cy = 120):

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

Example (preset 1): u = 203, v = 104, Z = 0.940 gives (0.146, -0.054, 0.940) m in `camera_optical_frame`. The
point is a `PointStamped` with the image stamp and frame id.

## TF direction and timing

`tf_buffer.transform(point_camera, "base_link")` uses the transform with target frame `base_link` and source frame
`camera_optical_frame`, so it converts the point from camera coordinates to base coordinates:
p_base = T_base_optical * p_optical. The lookup is at the image timestamp, in simulation time (`use_sim_time`).
The camera is fixed, so the transform does not change, but we still use the stamp of the image. Right after
startup the TF buffer can still be empty. Then we get one "TF transform failed" warning and the next pair works.

Check for preset 1: T_base_optical has translation (0.30, 0, 1.00) and gives x_base = 0.30 - y_opt,
y_base = -x_opt, z_base = 1.00 - z_opt. So (0.146, -0.054, 0.940) -> (0.354, -0.146, 0.060), and z = 0.06 m is
the top of the marker.

## Goal and motion

The goal is the base point + 0.08 m in z, so z = 0.140 m (the tip height). It is sent with
`ArmClient.move_to_point` only if `arm.ready()`. The result comes back in a done callback (so the node keeps
spinning), which logs whether the motion succeeded. Only one goal can be active (`goal_sent`). After a success no
more goals are sent, because the marker doesn't move and after arrival the arm partly hides it. After a failure
we try again from a new observation, at most 3 goals, so a systematic error (for example a wrong depth) can't
send goals forever.

## Failure handling

- Marker missing or fewer than 45 red pixels: no target, no goal, and a warning. Every new image overwrites the
  detection (or sets it to None), so an old target is never reused.
- Invalid depth (not finite, <= 0.05 m or >= 3 m): not used. If no valid value is left, the frame is skipped.
  Wrong depth encoding: frame skipped.
- RGB and depth more than 0.05 s apart: not paired by the synchronizer (and checked again in the depth callback).
- Stale data: a pair whose image stamp is more than 0.5 s from the current simulation time is ignored (too old, or
  from the future, for example before /clock starts). A timer on the steady clock warns if no fresh pair arrived
  for 2 wall-clock seconds, and no goal is sent without fresh data. The steady clock keeps running when the
  simulation is paused.
- Invalid calibration (fx or fy not a positive finite number): ignored, so no 3D point is computed until a valid
  CameraInfo arrives.
- TF not available (for example right after startup, before /tf_static arrives): `tf_buffer.transform` raises a
  `TransformException`. We catch it, print a warning and skip this pair, so no goal is sent. The next pair tries
  again.
- Helper not ready: no goal, we try again with the next pair.
- Motion failed or rejected: we log the message and retry from a new observation (max 3 goals).
- Ctrl+C: our own SIGINT handler only sets a flag (`stopping`), which blocks new goals. When the main loop ends
  (Ctrl+C or an error), a `finally` block cancels the goal with `arm.cancel()`, removes the two camera
  subscriptions and spins until the arm client is done (at most 3 s) before shutting down. rclpy's handler is off
  (`SignalHandlerOptions.NO`), so the node can still send the cancel. We avoid KeyboardInterrupt and remove the
  camera subscriptions because message_filters runs our callback while holding a lock: an exception there left it
  locked and hung the node in our tests.

## Results

Same code in all runs, a new simulation for every preset, `check_reach` started before the node:

| Preset | (u, v) | camera_optical_frame (m) | base_link (m) | Goal (m) | check_reach |
|---|---|---|---|---|---|
| 1 | (203, 104) | (0.146, -0.054, 0.940) | (0.354, -0.146, 0.060) | (0.354, -0.146, 0.140) | PASS, tip error 0.0060 m |
| 2 | (159, 90) | (-0.003, -0.102, 0.940) | (0.402, 0.003, 0.060) | (0.402, 0.003, 0.140) | PASS, tip error 0.0038 m |
| 3 | (115, 104) | (-0.153, -0.054, 0.940) | (0.354, 0.153, 0.060) | (0.354, 0.153, 0.140) | PASS, tip error 0.0050 m |

In all three runs the tip stayed within 0.02 m of the hover point for 2 simulated seconds with no contacts. Each
motion took about 5 s. The marker height came out 0.060 m every time, which is the marker top from the handout.

Failure tests:

- Ctrl+C during the motion (preset 1, about 1.6 s after the goal was sent): the log shows "Shutting down,
  canceling arm motion" and then "Arm movement failed: Motion canceled". The arm stopped at (0.193, -0.099),
  about 17 cm before the target, stayed there, and no new goal was sent. With a fake camera at 100 Hz we also
  pressed Ctrl+C 20 times at random moments: the node always exited in about 0.3 s.
- An error raised inside `synced_callback` right after the goal (with a small test script): the node canceled the
  goal ("Motion canceled"), exited by itself, and the arm stayed at its start position.
- Marker removed from the world before starting the node: no goal was sent, only "No reliable marker detected"
  warnings.
- Simulation paused after the motion: the watchdog printed "No fresh camera data for 2 seconds" every second until
  we resumed it.
