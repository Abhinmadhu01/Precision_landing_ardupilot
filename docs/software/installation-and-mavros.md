# Software Installation & MAVROS Setup Guide

This guide provides the complete, step-by-step instructions for installing ROS 2 Humble, MAVROS, GeographicLib datasets, and all required vision dependencies, as well as verifying communication with the Cube Orange+ autopilot.

---

## Architecture Selection: Raspberry Pi 4 vs. Raspberry Pi 5

Before proceeding with installation, determine your target hardware platform:

```text
+-----------------------------------------------------------------------------------+
|                            TARGET HARDWARE PLATFORM                               |
+-------------------------------------------------+---------------------------------+
|               Raspberry Pi 4 Model B            |         Raspberry Pi 5          |
+-------------------------------------------------+---------------------------------+
| OS: Ubuntu 22.04 LTS (Jammy) Bare-Metal        | OS: Raspberry Pi OS 64-bit      |
| Docker: NOT REQUIRED                            | Docker: REQUIRED (Containerized)|
| Reason: Native CSI camera kernel support exists | Reason: IMX500 CSI driver only  |
|         in Ubuntu 22.04 for Pi 4.               |         supported on RPi OS.    |
| Stack: Installed directly on the host system.   | Stack: Runs in Ubuntu container.|
+-------------------------------------------------+---------------------------------+
```

> [!TIP]
> **If using Raspberry Pi 4**: You do **not** need Docker. You can follow the bare-metal installation instructions below directly on Ubuntu 22.04 LTS.  
> **If using Raspberry Pi 5**: The commands below are executed **inside your Docker container** (or automated via Dockerfile).

---

## 1. ROS 2 Humble Installation (Ubuntu 22.04 LTS)

If you are setting up Ubuntu 22.04 bare-metal (on a PC or Raspberry Pi 4), install ROS 2 Humble using the official, verified Debian repositories:

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable Ubuntu Universe repository
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

# Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Base or Desktop
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
```

Source ROS 2 in your shell:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. MAVROS & GeographicLib Installation

MAVROS is the MAVLink communication bridge between ROS 2 and ArduPilot.

### 2.1 Install MAVROS Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-mavros-msgs
```

### 2.2 Install GeographicLib Datasets (Mandatory)
MAVROS relies on GeographicLib for geodetic coordinate transformations (GPS WGS-84 to local Cartesian ENU coordinates). **Without these datasets, MAVROS will crash or fail to publish `/mavros/global_position/` topics.**

Run the official installation script:
```bash
# Run the dataset download script with sudo permissions
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

---

## 3. Vision & AprilTag Dependencies

Install OpenCV, image transport, and the `apriltag` ROS 2 packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-apriltag-msgs \
  libapriltag-dev
```

If building `apriltag_ros` from source in your workspace:
```bash
cd ~/pr_land_ws/src
git clone https://github.com/christianrauch/apriltag_ros.git
cd ~/pr_land_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

---

## 4. Building the Precision Landing Workspace

Navigate to your workspace directory and compile `uav_interfaces` and `autonomous_flight_stack`:

```bash
cd ~/pr_land_ws

# Source ROS 2 underlay
source /opt/ros/humble/setup.bash

# Build all packages with symlink install
colcon build --symlink-install --packages-select uav_interfaces autonomous_flight_stack

# Source your workspace overlay
source install/setup.bash
echo "source ~/pr_land_ws/install/setup.bash" >> ~/.bashrc
```

---

## 5. Hardware Verification & Diagnostic Commands

Run these tests on the bench with **propellers removed** to verify hardware and communications:

### Step 5.1: Verify Serial Port Detection
Connect the Cube Orange+ TELEM 2 USB cable to the companion computer. Check the device node:
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```
Grant dialout user permissions if needed:
```bash
sudo usermod -a -G dialout $USER
```

### Step 5.2: Start and Test MAVROS Bridge
Launch MAVROS targeting the serial port and baud rate configured on ArduPilot (921600 baud):
```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600
# or if recognized as ACM:
# ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
```

In a second terminal, verify the heartbeat and telemetry streams:
```bash
# Check FCU connection status (must show connected: true)
ros2 topic echo /mavros/state --once

# Check GPS fix data
ros2 topic echo /mavros/global_position/global --once

# Check local altitude
ros2 topic echo /mavros/global_position/rel_alt --once
```

### Step 5.3: Verify Camera Publishing at 30 Hz
Start the camera publisher node (e.g. `gstream_pi_cam.py` or your camera driver node):
```bash
# Monitor image publishing frequency (target: 30 Hz)
ros2 topic hz /camera/image_raw
```

### Step 5.4: Verify AprilTag Detection
Point the camera towards your concentric nested AprilTag landing pad:
```bash
# Check detection publishing frequency
ros2 topic hz /detections

# Echo detected tag IDs and poses
ros2 topic echo /detections --once
```
Expected output includes `id: 0`, `id: 1`, or `id: 2` with pixel center coordinates $(c_x, c_y)$.

---

<div align="center">

| [⬅️ Previous: **06 - SITL Simulation**](../simulation.md) | [📖 **Documentation Roadmap**](../../README.md#documentation-roadmap) | [➡️ Next: **08 - Software Stack**](README.md) |
| :--- | :---: | ---: |

</div>

