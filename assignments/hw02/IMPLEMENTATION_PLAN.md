# HW2 implementation plan

1. Generate Gazebo world and raw occupancy map from one definition of the wide,
   mixed and blocked pillar rooms; use a 0.25 m radius differential-drive robot.
2. Provide perfect measured localization, map/base TF, static start/goal topics,
   all-step contact latching and a Gazebo-side stale-command watchdog.
3. Provide only an asynchronous single-destination `go_to(x, y)` action/client
   and CLI. Students sequence their own waypoints; there is no path helper.
4. Provide map-read/save and short clear-space movement examples, with no planning
   scaffold. Write detailed setup, interfaces, assignment and submission instructions.
5. Keep inflation, supercover edge checking, PRM, A* and waypoint sequencing in the
   ignored TA reference. Verify it across three seeds in both solvable rooms and
   report bounded no-path behavior in the blocked room.
6. Add public infrastructure/action tests, independent navigation checks, room
   figures and PDF generation. Record actual evidence and limitations in
   [VERIFICATION.md](environment/VERIFICATION.md); retain draft status until release gates pass.
