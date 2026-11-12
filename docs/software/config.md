# Configuration

All tunable values are in [../../src/autonomous_flight_stack/config/params.yaml](../../src/autonomous_flight_stack/config/params.yaml), loaded by [../../src/autonomous_flight_stack/launch/uav_core.launch.py](../../src/autonomous_flight_stack/launch/uav_core.launch.py).

> **Status:** the original nodes hardcode these values. The YAML file and launch file are ready, but each node must declare and read its parameters. The code changes are listed at the end of this page.

## mission_executive

| Parameter | Default | Unit | Meaning |
| --- | --- | --- | --- |
| `target_lat` | 8.5703492 | deg | Waypoint latitude. **Change for your site** |
| `target_lon` | 76.8846539 | deg | Waypoint longitude. **Change for your site** |
| `takeoff_alt` | 6.0 | m | Takeoff and cruise altitude above home |
| `takeoff_wait_s` | 5.0 | s | Fixed wait after the takeoff command before navigating |
| `tag_ids` | [0, 1, 2] | | AprilTag IDs of the nested target (outer, middle, inner) |
| `tag_priorities` | [1.0, 0.8, 0.6] | | Priority per tag. When several are visible the highest value is tracked |
| `heartbeat_timeout_s` | 5.0 | s | No `/mavros/state` for this long triggers RTL |

## navigation_server

| Parameter | Default | Unit | Meaning |
| --- | --- | --- | --- |
| `arrival_radius_m` | 1.5 | m | Waypoint reached inside this distance |
| `setpoint_rate_hz` | 20.0 | Hz | Setpoint publish rate |
| `waypoint_speed_mps` | 5.0 | m/s | Cruise speed, recommended 5-6 m/s |

**Cruise speed.** `waypoint_speed_mps` is documentation for the value you set in ArduPilot: `WPNAV_SPEED` in cm/s equals `waypoint_speed_mps` x 100, so 500-600 for 5-6 m/s. The current code does not send it to the Cube; set `WPNAV_SPEED` in Mission Planner or QGroundControl. At these speeds the drone may overshoot the 1.5 m arrival radius, so keep the landing area clear.

## landing_server: camera

| Parameter | Default | Unit | Meaning |
| --- | --- | --- | --- |
| `image_width`, `image_height` | 640, 480 | px | Camera stream size. Image centre = half of each |
| `sign_x`, `sign_y` | 1.0, 1.0 | | Flip to -1.0 if your camera orientation differs |

The PID gains were tuned at 640x480, with errors in pixels. At 1280x720 the same pixel error has a different meaning and the image centre moves to (640, 360). Rescale the gains (roughly by the width ratio) and retune.

## landing_server: PID

| Parameter | Default | Meaning |
| --- | --- | --- |
| `kp_strafe`, `kp_fwd` | 0.0011 | Proportional gain, m/s per pixel |
| `kd_strafe`, `kd_fwd` | 0.0008 | Derivative gain |
| `ki_xy` | 0.0003 | Integral gain |
| `kp_yaw` | 0.005 | Yaw gain, rad/s per degree of tag rotation |
| `d_filter_alpha` | 0.2 | Derivative low-pass filter, 1.0 = no filter |
| `max_speed_mps` | 0.5 | Horizontal speed limit during landing |
| `final_gain_multiplier` | 0.6 | Gain scale below `final_approach_alt` |

## landing_server: altitudes and timing

| Parameter | Default | Unit | Meaning |
| --- | --- | --- | --- |
| `max_detection_alt` | 6.0 | m | Above this, descend faster while searching |
| `final_approach_alt` | 2.0 | m | Switch to FINAL_APPROACH below this |
| `forced_land_alt` | 1.5 | m | Stall watchdog active below this |
| `forced_land_timeout_s` | 3.0 | s | Time below `forced_land_alt` before LAND |
| `land_alt_trigger` | 0.40 | m | Engage LAND below this |
| `stall_check_alt` | 0.75 | m | Slope-based touchdown check below this |
| `stall_window_s` | 2.0 | s | Altitude history window |
| `stall_slope_thresh` | -0.025 | m/s | Touchdown if the descent rate is slower than this |
| `rtl_timeout_s` | 15.0 | s | No tag for this long triggers RTL |
| `probe_descent_rate` | -0.3 | m/s | Descent while searching |

## Code changes to read the parameters

Do this in each node. In `__init__`, replace each hardcoded value with a declared parameter:

```python
# before
self.TARGET_LAT = 8.5703492

# after
self.declare_parameter('target_lat', 8.5703492)
self.TARGET_LAT = self.get_parameter('target_lat').value
```

Mapping:

| Node | Parameter | Replaces |
| --- | --- | --- |
| `mission_executive` | `target_lat`, `target_lon`, `takeoff_alt` | `TARGET_LAT`, `TARGET_LON`, `TAKEOFF_ALT` |
| `mission_executive` | `tag_ids`, `tag_priorities` | `TARGET_TAG_IDS`, `TARGET_TAG_PRIORITIES` (priorities must be doubles) |
| `mission_executive` | `takeoff_wait_s`, `heartbeat_timeout_s` | `create_timer(5.0, ...)`, `heartbeat_timeout` |
| `navigation_server` | `arrival_radius_m`, `setpoint_rate_hz` | `dist < 1.5`, `create_rate(20.0)` |
| `landing_server` | `image_width`, `image_height` | `CX, CY = 320.0, 240.0` (use width / 2 and height / 2) |
| `landing_server` | the remaining table entries | the matching uppercase attributes in `__init__` |

Notes:

- Declare list parameters with a default of the right type, for example `self.declare_parameter('tag_ids', [0, 1, 2])` and `self.declare_parameter('tag_priorities', [1.0, 0.8, 0.6])`.
- Run `ros2 param dump /landing_server` on the running node to confirm the values it actually uses.
- Keep a local copy of `params.yaml` per vehicle and per site instead of editing the repository default.

---

<div align="center">

| [⬅️ Previous: **09 - ROS 2 Nodes**](nodes.md) | [📖 **Documentation Roadmap**](../../README.md#documentation-roadmap) | [➡️ Next: **11 - Landing Algorithm**](../landing-algorithm.md) |
| :--- | :---: | ---: |

</div>
