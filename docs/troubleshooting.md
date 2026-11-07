# Troubleshooting

Check the logs first: `[CRUISE]`, `[SEARCH_DESCENT]`, `[APPROACH]` and `[FINAL_APPROACH]` lines from the nodes show where the mission stopped. Always debug on the bench with propellers **off**.

| Symptom | Likely cause | What to do |
| --- | --- | --- |
| Mission never starts | Flight mode is not GUIDED, or `mission_executive` is not running | Log says "Waiting for GUIDED mode..." until the mode switches. Check `ros2 node list` and `/mavros/state` |
| Arming rejected by FCU | Pre-arm check failing (GPS, compass, EKF, battery) | Read the message in the ground station and fix it. Do not disable pre-arm checks |
| Takeoff command rejected, then nothing happens | The executive logs the error and stops (no retry) | Fix the cause, switch out of GUIDED and back |
| `/mavros/state` shows `connected: false` | Wrong device, or the container cannot see the Cube | `ls /dev/ttyACM*` on the Pi, and check the device is passed into the container. Try another USB cable or port |
| Immediately switches to RTL | MAVROS heartbeat older than 5 s, or a goal was rejected | Check MAVROS is running and connected; check `ros2 action list` shows both actions |
| Navigation goal never completes | No GPS fix, or the target is far away | Check `/mavros/global_position/global`; the log prints "Waiting for GPS fix..." |
| Drone overshoots the waypoint | High cruise speed (5-6 m/s) with a 1.5 m arrival radius | Lower `WPNAV_SPEED`, increase the arrival radius, or give more room |
| No tag detected | `/detections` not publishing, wrong family or ID, tag too small for the altitude | `ros2 topic hz /detections`; check IDs in `params.yaml` and `apriltag_ros`; see [detection-range.md](detection-range.md) |
| Tag is detected at height but lost near the ground | Outer tag too large for close range | Use the nested tags and check the middle and inner tag sizes ([tags.md](tags.md)) |
| Drone holds altitude around 3 m, then RTL after 15 s | Tag not visible below 3 m: search only descends above 3 m | Check the tag is in the camera view and lit, and that the waypoint is above the tag |
| Drone moves away from the tag | Camera orientation or signs wrong | Check image top = nose, image left = drone left, or flip `sign_x` / `sign_y` ([camera-holder](hardware/camera-holder/README.md)) |
| Oscillation during landing | PID gains too high, or resolution changed | Lower `kp_*`, raise damping with `kd_*`, retune after changing resolution ([software/config.md](software/config.md)) |
| Drone follows a tag that is not yours | A tag ID is in the list | Only IDs in `tag_ids` are used, remove unused IDs |
| Detection flickers | Glare, blur, auto exposure changing | Matte print, steady lighting, fixed exposure if possible |
| Pi reboots, camera or USB drops in flight | Power supply too weak | Use a 5 V / 5 A buck converter; `vcgencmd get_throttled` on the host ([hardware](hardware/README.md)) |
| Timers frozen, nodes silent in simulation | `use_sim_time` mismatch | Same value for all nodes ([simulation.md](simulation.md)) |
| Parameters in `params.yaml` have no effect | Nodes still use hardcoded values | Apply the code changes in [software/config.md](software/config.md); verify with `ros2 param dump` |
| Drone sinks after switching back to Loiter | Throttle stick left low | See the throttle rule in [../SAFETY.md](../SAFETY.md) |

## Collecting information for a bug report

- Output of `ros2 topic echo /mavros/state --once`
- The mission log from all three nodes
- Your `params.yaml`
- The ArduPilot `.BIN` log from the flight controller
- Camera resolution, tag family, IDs and printed sizes
- Photos of the setup if hardware is involved
