
# MAVROS Installation Guide

This guide covers the installation of **MAVROS** (MAVLink extendable communication node for ROS 2). MAVROS is the critical bridge that allows your ROS 2 nodes (like the precision landing script) to communicate with the ArduPilot flight controller.

---

## Prerequisites

Before proceeding, ensure you have completed the following:
- ✅ Ubuntu 20.04 or 22.04 installed
- ✅ ROS 2 (Humble or Jazzy) installed and sourced
- ✅ Internet connection (for downloading datasets)

---

## Step 1: Install MAVROS Packages

We recommend installing from binary packages for stability.

### Run the installation command:
```bash
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y

```

**Breakdown of packages:**

* `ros-humble-mavros`: The core node that handles MAVLink communication.
* `ros-humble-mavros-extras`: Contains additional plugins (like `vision_pose_estimate` and `px4flow`) which are essential for precision landing projects.

---

## Step 2: Install GeographicLib Datasets (CRITICAL)

**⚠️ IMPORTANT:** MAVROS **will crash** or fail to initialize if these datasets are missing. They are required to convert GPS data (Latitude/Longitude) into local coordinates (X/Y meters).

### 1. Download the script

```bash
wget [https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh](https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh)

```

### 2. Make it executable

```bash
chmod +x install_geographiclib_datasets.sh

```

### 3. Run the script

```bash
sudo ./install_geographiclib_datasets.sh

```

**What this does:**

* Installs the **EGM96 Geoid** model (for accurate altitude conversion).
* Installs **Magnetic Field** models (for compass declination).
* *Note: This may take 2-5 minutes depending on your internet speed.*

---

## Step 3: Verify Installation

Once the installation is complete, verify that ROS 2 can find the MAVROS packages.

1. **Source your ROS environment:**
```bash
source /opt/ros/humble/setup.bash

```


2. **Check for MAVROS:**
```bash
ros2 pkg list | grep mavros

```



**Expected Output:**

```text
mavros
mavros_extras
mavros_msgs
...

```

✅ **Success:** If you see the list above, MAVROS is ready.

---

## Step 4: Understanding Connection URLs

When launching MAVROS, you need to specify a **fcu_url** to tell it where the Flight Controller is.

| Setup Type | Connection URL Example | Description |
| --- | --- | --- |
| **Real Hardware (USB)** | `/dev/ttyACM0:57600` | Standard for Pixhawk connected via USB. |
| **Real Hardware (Telemetry)** | `/dev/ttyUSB0:921600` | Common for distinct telemetry radios or onboard computers (Pi/Jetson) via UART. |
| **SITL (Simulation)** | `udp://:14550@` | Standard UDP port for local simulation. |
| **TCP** | `tcp://127.0.0.1:5760` | Less common, used for specific remote setups. |

---

## Troubleshooting common issues

### ❌ Error: "Failed to find GeographicLib datasets" or "Geoid not found"

* **Cause:** Step 2 was skipped or failed.
* **Fix:** Re-run `sudo ./install_geographiclib_datasets.sh`. Verify the files exist by running:
```bash
ls /usr/share/GeographicLib/geoids/
# You should see 'egm96-5.pgm'

```



### ❌ Error: "Package 'ros-humble-mavros' has no installation candidate"

* **Cause:** Your ROS 2 apt repository list is missing or outdated.
* **Fix:**
```bash
# Re-add ROS 2 GPG Key
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add Repo to Source List
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Update and Try Again
sudo apt update
sudo apt install ros-humble-mavros -y

```



### ❌ Error: "Permission denied" on /dev/ttyACM0

* **Cause:** Your user does not have permission to access serial ports.
* **Fix:** Add your user to the `dialout` group and **restart your computer**.
```bash
sudo usermod -a -G dialout $USER

```



---

## Additional Resources

* [Official MAVROS Documentation](https://github.com/mavlink/mavros)
* [MAVLink Protocol Guide](https://mavlink.io/)

```

```
