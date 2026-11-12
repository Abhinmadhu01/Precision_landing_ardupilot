# Software Stack: Architecture & Deployment

This document describes how the software components interact, how to run the flight stack on physical hardware, and pre-flight verification steps. Parameter tuning is covered in [config.md](config.md) and node architectures in [nodes.md](nodes.md).

---

## Deployment Architecture

The system supports two companion computer deployment models depending on whether you run on Raspberry Pi 5 or Raspberry Pi 4:

```text
+-------------------------------------------------------------------------------------------------+
|                                COMPANION SOFTWARE ARCHITECTURE                                 |
+-------------------------------------------------+-----------------------------------------------+
|         Raspberry Pi 5 (Containerized)          |         Raspberry Pi 4 (Bare-Metal)           |
+-------------------------------------------------+-----------------------------------------------+
| Host OS: Raspberry Pi OS 64-bit                 | Host OS: Ubuntu 22.04 LTS (Jammy)             |
| Container: Ubuntu 22.04 LTS (Docker)            | Runtime: Native OS (No Docker required)       |
| Reason: IMX500 camera CSI firmware is handled   | Reason: Camera & CSI bus natively supported   |
|         by RPi OS host, while ROS 2 runs inside.|         directly in Ubuntu 22.04 kernel.      |
+-------------------------------------------------+-----------------------------------------------+
```

For package installation, dependencies, and MAVROS setup commands, see [installation-and-mavros.md](installation-and-mavros.md).

---

## Software Components Breakdown

| Component | Runtime Context | Responsibility |
| :--- | :--- | :--- |
| **Camera Driver** | Host / Container | Acquires raw video frames from the Sony IMX500 sensor at 30 fps (`/camera/image_raw`). |
| **`apriltag_ros`** | ROS 2 Node | Detects family 48h12 tags, computes image pixel coordinates, and publishes `/detections`. |
| **`mavros`** | ROS 2 Bridge | Bridges telemetry, mode commands, and velocity setpoints to Cube Orange+ via MAVLink. |
| **`navigation_server`**| ROS 2 Action Server | Publishes global GPS setpoints to navigate the drone from takeoff to the target waypoint. |
| **`landing_server`** | ROS 2 Action Server | Executes closed-loop PID centering on visible nested tags and manages descent state machine. |
| **`mission_executive`**| ROS 2 Coordinator | Manages mission lifecycle: arms on GUIDED mode, triggers takeoff, sequences navigation & landing. |

---

## Prerequisites

- **Hardware Built & Inspected**: Avionics and wiring verified per [docs/hardware/README.md](../hardware/README.md).
- **Packages & MAVROS Configured**: Verified per [docs/software/installation-and-mavros.md](installation-and-mavros.md).
- **Flight Controller Setup**: ArduCopter flashed, sensors calibrated, Cube connected via native USB port (`/dev/ttyACM0`), and RC mode switch assigned to **GUIDED**.
- **Safety Protocol Reviewed**: You have thoroughly reviewed [SAFETY.md](../../SAFETY.md).

---

## Step 1: Configure Mission Parameters

Edit [`src/autonomous_flight_stack/config/params.yaml`](../../src/autonomous_flight_stack/config/params.yaml):

1. **Target GPS Coordinates**:
   - `target_lat`: Destination latitude in degrees.
   - `target_lon`: Destination longitude in degrees.
   - `takeoff_alt`: Commanded cruise altitude in meters (default `6.0`).
2. **Nested Tag Configuration**:
   - `tag_ids`: `[0, 1, 2]` (outer, middle, inner).
   - `tag_priorities`: `[1.0, 0.8, 0.6]`.
3. **Camera Resolution**:
   - `image_width`: `640` (or `1280`).
   - `image_height`: `480` (or `720`).

---

## Step 2: Launch the Flight Stack

On the companion computer (inside the Docker container for Pi 5, or native terminal for Pi 4):

```bash
# Source workspace
source /opt/ros/humble/setup.bash
source ~/pr_land_ws/install/setup.bash

# Launch the three precision landing nodes
ros2 launch autonomous_flight_stack uav_core.launch.py
```

To pass an external parameters file:
```bash
ros2 launch autonomous_flight_stack uav_core.launch.py params_file:=/path/to/params.yaml
```

*Note: For physical hardware flight, do NOT pass `use_sim_time:=true`.*

---

## Step 3: Pre-Flight Hardware Verification

With **propellers REMOVED**, open a separate terminal and confirm critical telemetry:

```bash
# 1. Verify MAVROS connection to Cube Orange+
ros2 topic echo /mavros/state --once
# Output must indicate: connected: true

# 2. Verify Camera frame rate (must be ~30 Hz)
ros2 topic hz /camera/image_raw

# 3. Verify AprilTag detection topic
ros2 topic hz /detections

# 4. Verify GPS fix status
ros2 topic echo /mavros/global_position/global --once

# 5. Verify Action Servers are online
ros2 action list
# Must list: /navigate_to_waypoint and /precision_land
```

Expected node startup logs in the main launch terminal:
```text
[navigation_server]: Navigation Action Server Online ...
[landing_server]: Precision Landing Action Server Online ...
[mission_executive]: Mission Executive ready. Waiting for GUIDED mode...
```

---

## Step 4: Mission Flight Execution

1. Place the concentric nested AprilTag landing pad at the destination coordinates.
2. Verify GPS 3D/RTK lock and clear the designated flight area.
3. Power the drone and wait for companion computer boot.
4. Launch `uav_core.launch.py`.
5. With the pilot standing clear and holding the RC transmitter:
   - Flip the assigned RC transmitter switch to **GUIDED**.
   - **The vehicle arms automatically and takes off to `takeoff_alt`**.
   - The vehicle navigates to `target_lat`, `target_lon`.
   - The vehicle transitions to `SEARCH_DESCENT`, locks onto the nested AprilTag, and centers via PID.
   - At terminal altitude, `landing_server` switches the autopilot to `LAND` mode for touchdown and automatic disarm.
6. **Pilot Override**: To abort autonomous control at any point during flight, flip the RC flight mode switch out of GUIDED into **LOITER** or **STABILIZE**.

---

<div align="center">

| [⬅️ Previous: **07 - Installation & MAVROS**](installation-and-mavros.md) | [📖 **Documentation Roadmap**](../../README.md#documentation-roadmap) | [➡️ Next: **09 - ROS 2 Nodes & Interfaces**](nodes.md) |
| :--- | :---: | ---: |

</div>
