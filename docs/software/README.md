# Software: Setup and Running

How to run the stack. Parameter details are in [config.md](config.md) and node details in [nodes.md](nodes.md).

## Architecture

The Raspberry Pi 5 runs Raspberry Pi OS (host) with Docker. Inside Docker, an Ubuntu 22.04 image runs ROS 2 Humble, MAVROS, `apriltag_ros`, the camera node and this stack. The reason for Docker is that the IMX500 camera has no firmware support for the CSI port on Ubuntu 22.04.

| Component | Where | Purpose |
| --- | --- | --- |
| Camera node | Container | Publishes the IMX500 image |
| `apriltag_ros` | Container | Publishes AprilTag detections on `/detections` |
| MAVROS | Container | Bridges ROS 2 and the Cube over USB |
| `navigation_server` | Container | Flies to the GPS waypoint |
| `landing_server` | Container | Precision landing control |
| `mission_executive` | Container | Sequencing, takeoff, failsafe |

The Docker image, `Dockerfile` and `docker-compose.yml` are in a separate repository: TODO: add Docker repo link. This repository holds only the flight stack, configuration and documentation.

## Prerequisites

- Hardware built and checked: [../hardware/README.md](../hardware/README.md)
- Docker environment from the Docker repository running on the Pi
- Cube Orange+ connected by USB, GPS lock, RC mode switch set to GUIDED
- You have read [../../SAFETY.md](../../SAFETY.md)

## 1. Configure

Edit [../../config/params.yaml](../../config/params.yaml):

- `target_lat`, `target_lon`, `takeoff_alt` for your site
- `tag_ids` and `tag_priorities` to match your printed tags ([../tags.md](../tags.md))
- `image_width` and `image_height` to match the camera stream

## 2. Start the stack

Start the containers as described in the Docker repository (for example `docker compose up`). Services started there: TODO: list service names. The flight stack is started with:

```bash
ros2 launch autonomous_flight_stack uav_core.launch.py
# or with an explicit parameter file
ros2 launch autonomous_flight_stack uav_core.launch.py params_file:=/path/to/params.yaml
```

Add `use_sim_time:=true` only in simulation.

## 3. Verify before flight

Run these inside the container, with propellers **off**:

```bash
ros2 topic echo /mavros/state --once            # connected: true
ros2 topic hz /detections                       # publishes when a tag is in view
ros2 topic echo /mavros/global_position/global --once   # GPS fix
ros2 action list                                # /navigate_to_waypoint and /precision_land
```

Expected log lines from the three nodes:

```text
Navigation Action Server Online ...
Precision Landing Action Server Online ...
Mission Executive ready. Waiting for GUIDED mode...
```

## 4. Fly

1. Place the target tag at the landing site and check the waypoint coordinates.
2. Clear the area, arm checks complete, pilot ready with the transmitter.
3. Switch the Cube to **GUIDED**. **The mission starts immediately: arm, takeoff, flight to the waypoint, precision landing.**
4. Watch the logs (`[CRUISE]`, `[SEARCH_DESCENT]`, `[APPROACH]`, `[FINAL_APPROACH]`).
5. To abort at any time, change the flight mode with the transmitter. Remember the throttle rule in [../../SAFETY.md](../../SAFETY.md).

Leaving GUIDED mode makes the landing server abort its goal.

## Run order

1. Containers up (MAVROS connected, camera and detection running)
2. `uav_core.launch.py` running
3. Pilot switches to GUIDED

## Logs

ROS 2 logs are in `~/.ros/log` inside the container. Also keep the ArduPilot `.BIN` log from the Cube's SD card for each flight.
