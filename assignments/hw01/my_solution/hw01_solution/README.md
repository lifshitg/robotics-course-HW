# hw01_solution

ROS 2 Jazzy package for HW1 (camera-guided reaching). Place this folder at
`assignments/hw01/my_solution/hw01_solution/`.

## Build (from the repository root)
```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
cd assignments/hw01/my_solution
colcon build --symlink-install
source install/setup.bash
cd ../../..
```

## Offline unit tests (no simulator)
```bash
python3 -m pytest -q assignments/hw01/my_solution/hw01_solution/test
```

## Run (simulation already running in terminal A)
```bash
# terminal B: start the checker first
ros2 run hw01_sim check_reach --preset 1 --timeout 90
# terminal C: start the solution
ros2 run hw01_solution reach_target
```

## Debug tools
```bash
ros2 run hw01_solution capture_frame hw01_frame   # saves rgb.png, depth.npy, camera_info.json, ...
ros2 topic echo /hw01/target_base                  # estimated marker position (before hover offset)
```

## Files
- `hw01_solution/perception.py` – red-marker detection, robust depth, pinhole back-projection (no ROS).
- `hw01_solution/logic.py` – staleness check and consecutive-estimate consistency filter (no ROS).
- `hw01_solution/reach_node.py` – the ROS node: sync, perception, TF2, single-goal state machine.
- `hw01_solution/capture_frame.py` – saves one real frame for calibration/offline tests.
