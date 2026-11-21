## Running the solution
- Use a similar setup to `./check_setup.py`, from the root of the repository running
  `ros2 launch hw01_sim simulation.launch.py preset:=1`.
- In another terminal, run `./solution_hw01.py` from `assignments/hw01/my_solution/`.

## Detecting the marker
We chose to use opencv for the marker detection.
Firstly we apply a color-mask, using `cv2.inRange`, and filter out any non-red colors.
We use `cv2.findContours`, and filter the ones that are not 'circular' enough, see [1].
Then, taking the max contour, we follow [2], using `cv2.moments` to find the center point.

## Transforming from 2d coordinates in image frame into 3d camera coordinates frame
We're given the intrinsic matrix[3].
K = [[f_x,   0, c_x]
     [  0, f_y, c_y]
     [  0,   0,   1]]
As explained in [3], given a vector a = [u v w]^T in the camera coordinate frame,
let K v = [x y w]^T, the pixel (x, y) represents the corresponding pixel to the vector.

Therefore, to go from a' = [x y 1]^T to b = [u v 1] we need to invert K, we get
K^-1 = [[1/f_x,     0, -c_x/f_x]
        [    0, 1/f_y, -c_y/f_y]
        [    0,     0,        1]]

Now, notice that b = K^-1 [x y 1] is normalized, therefore for it to represent the marker position,
we need to multiply it by the depth z. We get that
- u = z * (x - c_x) / f_x
- v = z * (y - c_y) / f_y
And that is the representation of the pixel (x, y) in the camera coordinate frame.

## Technical Commentary about the code

Perception & Main Loop:
Initializes the ROS 2 node with use_sim_time=True, subscribes to /camera/image_raw, /camera/depth/image_raw, and /camera/camera_info using qos_profile_sensor_data, detects the marker centroid, validates that the sampled depth is finite and positive, and constructs a PointStamped message in camera_optical_frame using the image header.

TF2 Transformation & Debug Topics:
Publishes the raw detected point to /hw01/target_camera, transforms it into base_link using tf2_ros.Buffer and TransformListener (with a 0.2 s timeout), and publishes the transformed point (before hover offset) to /hw01/target_base.

Hover Goal Construction:
Adds a 0.08 m offset along the $Z$-axis in base_link and validates that all coordinates are finite and that the resulting tip height is within 14 +/- 0.01 m before commanding the arm

Freshness (Staleness) Checks:
Rejects observations if no camera frame has arrived within 2.0 s wall-clock time (time.monotonic()) or if the observation timestamp is older than 0.5 s relative to the simulation clock.

Motion Lifecycle & Failure Handling:
Tracks a single active motion Future via ArmClient so no overlapping goals are sent, never reuses an earlier target when detection or TF2 fails, logs the final success and message outcome, and cancels any pending motion on KeyboardInterrupt / shutdown.
Additionaly, if we fail to detect the marker(see ## Detecting the marker), we exit with error code 1.

## Results
We passed all 3 given presets,
with each reporting that we held for '2 simulated seconds', and 'live contact streams clear'.
We get the following tip errors:
- For preset 1, a tip error of 0.0060 m.
- For preset 2, we get a tip error 0.0038 m.
- lastly, for preset 3, a tip error of 0.0050 m.

## Sources
- [1]: <https://stackoverflow.com/q/42203898>
- [2]: <https://stackoverflow.com/a/12608954>
- [3]: <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html>
