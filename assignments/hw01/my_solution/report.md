# HW1 | Camera-Guided Reaching with a Two-Joint Arm

**Overview.** Our ROS 2 node locates the red marker using RGB and aligned depth images, reconstructs its 3D position, converts it to the robot base frame, and commands the arm tip to hover above it.

## Marker detection and 3D reconstruction

**Synchronized input.** RGB and depth frames are paired within 0.05 simulated seconds using ApproximateTimeSynchronizer and sensor-data QoS. Camera intrinsics are read from /camera/camera_info.

**Detection.** A pixel is considered red when R > 150, R > 3G, and R > 3B. We select the largest connected red region containing at least five pixels, then use its centroid (u, v). If no suitable region exists, detection is rejected.

**Depth.** Depth values are taken from pixels in the selected region. Nonfinite values and depths outside (0, 10] m are removed; at least five valid samples are required. The median depth d reduces sensitivity to noise. With camera intrinsics (fx, fy, cx, cy), the optical-frame position is:

**X = (u − cx)d / fx     Y = (v − cy)d / fy     Z = d**

## TF2 and motion execution

The 3D marker point is published as a stamped PointStamped on /hw01/target_camera. TF2 transforms it from camera_optical_frame to base_link at the image timestamp (with use_sim_time enabled); the unmodified result is published on /hw01/target_base. We then add 0.08 m to base-frame Z (target tip height ≈ 0.14 m) and send it through ArmClient.move_to_point. The node logs the action result and allows only one outstanding motion.

## Failure handling

The node ignores missing/invalid calibration, mismatched image sizes or frames, unsupported depth encoding, unreliable detections, and insufficient valid depth. It skips commands when TF2 fails, the helper is unavailable/busy, an observation is older than 0.5 simulated seconds, or fresh camera data has been absent for 2 wall-clock seconds. Pending motion is canceled on shutdown.

## Experimental results

The public reach checker independently verifies the measured tip position and live contact streams. All three presets passed the required 2-simulated-second hold within 0.02 m of the hover point, with clear contact streams.

```console
idosg@Ido:~$ cd ~/HW/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
idosg@Ido:~/HW/robotics-course-HW$ ros2 run hw01_sim check_reach --preset 1
PASS: tip error 0.0012 m, held 2 simulated seconds; live contact streams clear
idosg@Ido:~/HW/robotics-course-HW$ ^C
idosg@Ido:~/HW/robotics-course-HW$ ros2 run hw01_sim check_reach --preset 2
PASS: tip error 0.0038 m, held 2 simulated seconds; live contact streams clear
idosg@Ido:~/HW/robotics-course-HW$ ^C
idosg@Ido:~/HW/robotics-course-HW$ ros2 run hw01_sim check_reach --preset 3
PASS: tip error 0.0028 m, held 2 simulated seconds; live contact streams clear
idosg@Ido:~/HW/robotics-course-HW$
```

## Build and run

We need 3 terminals to run the experiments: First, we’ll run the Gazebo env:

```bash
cd ~/HW/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1 # ← change this to 1, 2 or 3 in order to run each preset experiment
```

Then we’ll run the checker in a second terminal:

```bash
cd ~/HW/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 run hw01_sim check_reach --preset 1 # ← make sure preset is matching
```

Now to run our solution in the third:

```bash
cd ~/HW/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
python3 assignments/hw01/my_solution/reach_target.py
```

HW01 report • 1 page
