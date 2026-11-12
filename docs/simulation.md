# SITL Simulation Guide (Gazebo, RViz & QGroundControl)

This guide provides the complete procedure for simulating the autonomous precision landing mission using **ArduPilot SITL**, **Gazebo**, **RViz**, and **QGroundControl (QGC)** before testing on physical hardware.

<p align="center">
  <img src="images/simulation/sim_multiwindow_precision_landing.png" width="850" alt="SITL Gazebo, RViz, and QGroundControl Precision Landing Simulation" />
  <br>
  <em>Figure 1: Full multi-window simulation environment — QGroundControl with nadir camera gimbal (P: -90.0°), Gazebo Iris model on the runway with AprilTag pad, and RViz coordinate tracking.</em>
</p>

---

## Reference & Notes on Upstream ArduPilot ROS 2 Docs

This simulation environment follows the integration architecture described in the [ArduPilot ROS 2 Documentation](https://ardupilot.org/dev/docs/ros2.html). 

> [!NOTE]
> **Important Upstream Notice**: The upstream ArduPilot ROS 2 documentation contains certain generic ROS 2 repository installation commands that are outdated or cause package conflicts on Ubuntu 22.04 LTS (Jammy).  
> - For correct base ROS 2 and MAVROS package installations, strictly follow our verified guide: [docs/software/installation-and-mavros.md](software/installation-and-mavros.md).  
> - The precision landing simulation bringup commands and topic monitoring procedures below represent the verified working pipeline.

---

## Simulation System Architecture

The simulation pipeline runs entirely in software on an Ubuntu 22.04 host:

```text
+-----------------------+         +-------------------------+
| Gazebo Simulator      |         | ArduPilot SITL (Copter) |
| - Runway world        | <=====> | - Flight dynamics model |
| - Iris quadcopter     |  JSON/  | - EKF state estimation  |
| - AprilTag ground pad |  Lockstep| - MAVLink UDP (14550)   |
| - Mono camera sensor  |         +------------+------------+
+-----------+-----------+                      |
            | GZ Bridge                        | MAVROS
            v                                  v
+-----------+-----------+         +------------+------------+
| /camera/image_raw     |         | /mavros/state           |
| (Target: 30 Hz)       |         | /mavros/local_position  |
+-----------+-----------+         +------------+------------+
            |                                  |
            v                                  v
+-----------+-----------+         +------------+------------+
| apriltag_ros          |         | autonomous_flight_stack |
| -> /detections        | ======> | - mission_executive     |
+-----------------------+         | - navigation_server     |
                                  | - landing_server        |
                                  +-------------------------+
```

| Simulated Drone Model (Gazebo) | Ground AprilTag Landing Pad |
| :---: | :---: |
| ![S500 Quadcopter Gazebo Simulation Model](images/simulation/sim_gazebo_s500_drone_model.png) | ![Gazebo Runway AprilTag Target Marker](images/simulation/sim_gazebo_apriltag_target_marker.png) |
| *Iris/S500 quadcopter with downward camera sensor* | *Gazebo runway world with AprilTag landing pad* |

---

## Step 1: Launch Gazebo and RViz Simulation

In your terminal, source your ROS 2 simulation workspace and run the unified bringup launch file:

```bash
# Source ROS 2 and the ArduPilot Gazebo workspace
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash

# Launch Gazebo world, Iris model, ArduPilot SITL, and RViz
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### What this launch file does:
1. Boots **Gazebo** loading the runway world with the AprilTag ground marker model.
2. Spawns the **Iris quadcopter** equipped with an integrated nadir camera sensor.
3. Initializes the **ArduPilot SITL Copter** background process communicating with Gazebo via the JSON lockstep interface.
4. Opens **RViz** with the preconfigured robot model and transform frames.

---

## Step 2: Connect QGroundControl & Set Camera Angle Downward

1. Launch **QGroundControl** on your workstation:
   ```bash
   ./QGroundControl.AppImage
   ```
2. QGC will automatically connect to SITL via local UDP port `14550`.
3. **Make the camera face downward (Nadir)**:
   - On the top status bar or vehicle payload control panel in QGC, adjust the gimbal pitch.
   - Set **Pitch to -90.0°** (and Yaw to `0.0°` or `Yaw follow`).
   - Look at the top banner in Figure 1 above: it should confirm `P: -90.0  Y: 0.0`.
   - This ensures the simulated camera points straight down at the AprilTag landing pad on the runway.

---

## Step 3: Pre-Flight Health & Topic Verification (Mandatory)

> [!CRITICAL]
> **Do NOT launch the autonomous flight stack yet.**  
> Always verify that sensors, bridges, and detection topics are publishing at their nominal rates. If the camera or detection topics are silent or lagged, the landing state machine will abort into failsafe RTL.

Open a separate terminal and execute the following checks sequentially:

### 3.1 Verify Camera Publishing at 30 Hz
Verify that the Gazebo camera bridge is actively publishing video frames at the nominal 30 fps rate:
```bash
ros2 topic hz /camera/image_raw
```
*Expected output*: `average rate: 29.8 Hz - 30.2 Hz`.  
If the frame rate drops significantly below 20 Hz, reduce Gazebo rendering quality or adjust real-time update rate parameters.

### 3.2 Verify AprilTag Detection Pipeline
Ensure `apriltag_ros` is subscribed to the camera topic and publishing valid detections:
```bash
# Check publishing rate of detections
ros2 topic hz /detections

# Inspect detection message content
ros2 topic echo /detections --once
```
*Expected output*: A non-empty `AprilTagDetectionArray` containing the detected tag ID (e.g., ID 0) along with its image center coordinates $(c_x, c_y)$ and corner vectors.

### 3.3 Verify MAVROS Autopilot Telemetry
Check that MAVROS communicates cleanly with ArduPilot SITL:
```bash
# Confirm connection and flight mode
ros2 topic echo /mavros/state --once

# Confirm local position coordinates
ros2 topic echo /mavros/local_position/pose --once
```
*Expected output*: `connected: true` and `mode: STABILIZE` or `MANUAL`.

### 3.4 Inspect RViz Visual Overlays & Coordinates
In the RViz window:
- Verify that the TF coordinate frame tree (`odom` $\rightarrow$ `base_link` $\rightarrow$ `camera_link`) displays with zero latency.
- Confirm the image viewer panel displays the ground target with green detection bounding quads.

| Camera Target Detection View | Dynamic TF Coordinate Frames |
| :---: | :---: |
| ![Simulated camera feed](images/simulation/sim_gazebo_rviz_landing_interface.png) | ![RViz coordinate frames during landing](images/simulation/sim_rviz_coordinate_frames_landing.png) |
| *Real-time detection overlay on simulated video stream* | *Target translation vector relative to camera optical frame* |

---

## Step 4: Launch the Autonomous Flight Stack

Only after all verification checks in Step 3 have passed, launch the three autonomous precision landing nodes in simulation mode:

```bash
ros2 launch autonomous_flight_stack uav_core.launch.py use_sim_time:=true
```

> [!IMPORTANT]
> The parameter `use_sim_time:=true` ensures the action servers, PID loop, and timer callbacks synchronize directly with the Gazebo simulation clock (`/clock`).

You should observe the following startup logs:
```text
[navigation_server]: Navigation Action Server Online ...
[landing_server]: Precision Landing Action Server Online ...
[mission_executive]: Mission Executive ready. Waiting for GUIDED mode...
```

---

## Step 5: Execute Mission & Monitor Landing

To initiate the fully autonomous mission, command the flight controller into **GUIDED** mode via QGroundControl or MAVProxy:

### In MAVProxy terminal:
```text
mode guided
```
### Or in QGroundControl:
Select the flight mode dropdown in the top-left toolbar and choose **GUIDED**.

### Mission Execution Sequence:
1. **Automated Arming & Takeoff**: `mission_executive` detects GUIDED mode, arms the vehicle, and commands takeoff to `takeoff_alt` (default 6.0 m).
2. **Transit**: `navigation_server` publishes global setpoints to fly to the target GPS coordinates.
3. **Target Search**: Upon arrival within 1.5 m of the waypoint, `landing_server` engages, acquiring the outer AprilTag (ID 0).
4. **Centering & Descent**: The horizontal PID loop computes velocity setpoints (`cmd_vel_unstamped`) to nullify $(err_x, err_y)$ pixel errors while descending.
5. **Touchdown**: Below $0.4\text{ m}$, the stack commands `LAND` mode, and ArduCopter disarms on ground contact.

---

## Differences Between Simulation and Hardware

| Parameter / Subsystem | Gazebo SITL Simulation | Physical Drone Hardware |
| :--- | :--- | :--- |
| **`use_sim_time`** | `true` (synchronized to `/clock`) | `false` (wall-clock time) |
| **Camera Sensor** | Simulated Gazebo plugin camera | Sony IMX500 via CSI port (CAM0) |
| **Autopilot Interface** | UDP socket (`udp://:14550`) | Serial UART (`TELEM 2` $\rightarrow$ USB) |
| **Companion Deployment**| Host PC workstation | Raspberry Pi 5 (Docker) / Pi 4 (Native) |
| **Environmental Noise** | Ideal aerodynamic conditions | Real wind gusts, ground effect turbulence, sun glare |

---

<div align="center">

| [⬅️ Previous: **05 - Detection Range**](detection-range.md) | [📖 **Documentation Roadmap**](../README.md#documentation-roadmap) | [➡️ Next: **07 - Installation & MAVROS**](software/installation-and-mavros.md) |
| :--- | :---: | ---: |

</div>
