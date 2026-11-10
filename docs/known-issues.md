# Known Issues and Limitations

Found while documenting the code. The first three are worth checking before the next flight test.

## 1. Yaw bit in the position type mask

`navigation_server.py` sets `type_mask = 3512` and its comments say position and yaw are active. In binary, 3512 is 2048 + 1024 + 256 + 128 + 32 + 16 + 8. Bit 1024 is "ignore yaw", so yaw appears to be **ignored**, and the acceleration-x bit (64) is left clear, so it appears not to be ignored.

| Meaning | Mask |
| --- | --- |
| Position only | 3576 |
| Position and yaw | 2552 |
| Used in the code | 3512 |

3512 is 3576 minus 64. Check the mask on your firmware version. If you want the nose to face the target during cruise, use 2552. The yaw value itself is computed and sent already.

## 2. Tag priorities do not match the comment

The comment in `mission_executive.py` says the innermost tag gets the highest priority, but the values `[1.0, 0.8, 0.6]` for tags `[0, 1, 2]` give the highest priority to the **outer** tag 0. Because the highest priority visible tag is tracked, the outer tag is used whenever it is visible. Choose the behaviour you want and update the values or the comment, see [tags.md](tags.md).

## 3. Camera resolution and PID gains

The image centre (320, 240) and the PID gains are tuned for 640x480. At 1280x720 the centre is (640, 360) and the same gains act on larger pixel errors. See [software/config.md](software/config.md).

## 4. Other issues

| Issue | Effect |
| --- | --- |
| `NavigateToWaypoint` feedback `current_speed` is never filled in | Always 0 |
| Header comment in `mission_executive.py` says tags `[6, 7, 8]` | Code uses `[0, 1, 2]`, comment is outdated |
| Takeoff is followed by a fixed 5 s wait, not an altitude check | Navigation can start before the drone reaches `takeoff_alt` |
| A rejected takeoff command is only logged | The executive stops, no retry or failsafe |
| Below 3 m with no tag, SEARCH_DESCENT holds altitude | After 15 s the drone returns to launch |
| The stall watchdog usually triggers before the 0.40 m LAND trigger | LAND engages at about 0.8-0.9 m ([landing-algorithm.md](landing-algorithm.md)) |
| `landing_server.py` calls `rclpy.shutdown()` without checking `rclpy.ok()` | Possible error on Ctrl+C, the other two nodes already guard it |
| Parameters are hardcoded in the nodes | Apply the changes in [software/config.md](software/config.md) |
| `waypoint_speed_mps` is not applied by code | Set `WPNAV_SPEED` in ArduPilot |
| GUIDED mode starts the mission automatically | By design, see [../SAFETY.md](../SAFETY.md) |

## Hardware limitations

- The prototype's 5 V / 3 A buck converter is below the 5 V / 5 A the Pi 5 needs ([hardware/README.md](hardware/README.md)).
- IMX500 support requires Raspberry Pi OS on the host, which is why the stack runs in Docker.
- The 7-8 m (640x480) and 10-12 m (1280x720) detection ranges are for a 22.7 cm tag in your lighting conditions.

---

<div align="center">

| [⬅️ Previous: **11 - Troubleshooting**](troubleshooting.md) | [📖 **Documentation Roadmap**](../README.md#documentation-roadmap) | [➡️ Next: **Return to Project Home**](../README.md) |
| :--- | :---: | ---: |

</div>
