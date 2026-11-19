
Dor Katz and Gal Sasson


The ROS2 node detects te red marker and moves the robot arm to hover above it.


---- Setup: ----

* Created subscriptions: `/camera/image_raw`, `/camera/depth/image_raw` and
`/camera/camera_info`
* `cv_bridge` converts the RGB image for working with cv2.
* We synchronize the RGB and the Depth cameras to get an image with color and depth at each timestamp.


---- Detection of the marker: ----

1. Convert the image to HSV.
2. Two masks are combined:
   hue 0-10 and hue 170-179, with saturation >= 100 and value >= 80.
   Red pixels become 255 in the mask, everything else 0.
3. `cv2.findContours` finds the red regions.
4. Regions with less than 20 pixels are ignored, 
   the region with the largest area (`cv2.contourArea`) is kept. This ignores small
   noise regions.
5. The center of that region (`cv2.moments`) is the pixel `(u, v)`.
(If no region is found, no target exists and nothing is sent.)

---- Depth: ----
* We ue the info from the depth camera
* Depth is read in a 3x3 pixel window around `(u, v)`. 
* The median of the values is the depth `z`. (The median reduces the effect of noise in the depth camera) 
* We use the camera parameters (fx,fy,cx,cy) to calculate the locatoin of the pixel in the 3D space:
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = depth


---- Converting to Robot coordinates: ----
* We use tf2 to convert the coordinates from the camera system to the robot's base.
    base_point = self.tf_buffer.transform(camera_point, 'base_link')

---- Converting to Robot coordinates: ----
* We make a target point 0.08m above (on the Z axis) the red marker.
    self.hover_target = deepcopy(base_point)
    self.hover_target.point.z += 0.08
* Now we can move the arm using:
    self.arm.move_to_point(self.hover_target)
* A callback (movement_finished) prints if the motion succeded or failed.

---- Failure handling: ----

- No red region found: No target, nothing sent
- Depth encoding not `32FC1` or size mismatch: Frame skipped and warning 
- Motion failed: Error message printed
- Node interrupted: motion cancelled

---- Build and run commands: ----

source /opt/ros/jazzy/setup.bash
cd assignments/hw01/environment
colcon build --symlink-install --base-paths src --executor sequential
source install/setup.bash
cd ../../..

Build the package:

cd assignments/hw01/my_solution
source /opt/ros/jazzy/setup.bash
source ../environment/install/setup.bash
colcon build --symlink-install --base-paths hw01_solution
cd ../../..

* Terminal 1: run simulation
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1

* Terminal 2: reach checker test
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 run hw01_sim check_reach --preset 1 --timeout 90

* Terminal 3: our solution
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
source assignments/hw01/my_solution/install/setup.bash
ros2 run hw01_solution solution
