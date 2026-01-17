#!/bin/bash
# Script to start V4L2 Camera and AprilTag ROS 2 Nodes
# Author: Abhin M (Modified for Hardware Lock)

# ==============================================================================
# CONFIGURATION CONSTANTS
# ==============================================================================
# Workspace Path
WORKSPACE_DIR="/home/abhin/v3_cc"
CALIB_FILE="file://${WORKSPACE_DIR}/src/precision_landing/config/calibrationdata/ost.yaml"
TAG_CONFIG="${WORKSPACE_DIR}/src/apriltag_ros/cfg/tags_36h11.yaml"

# ==============================================================================
# CLEANUP FUNCTION
# ==============================================================================
cleanup() {
    echo ""
    echo "========================================="
    echo "Stopping all nodes..."
    kill $PID_CAM 2>/dev/null
    kill $PID_TAG 2>/dev/null
    echo "✓ All processes stopped."
    echo "========================================="
    exit
}

trap cleanup SIGINT

# ==============================================================================
# USER INPUTS
# ==============================================================================
echo "========================================="
echo "   Vision System Startup (Logitech C920) "
echo "========================================="
echo ""

# --- 1. Camera Configuration ---
read -p "Camera Index Number (e.g. 0, 2, 4) [2]: " CAM_INDEX
CAM_INDEX=${CAM_INDEX:-2}
CAM_DEV="/dev/video${CAM_INDEX}"

read -p "Image Width [640]: " IMG_WIDTH
IMG_WIDTH=${IMG_WIDTH:-640}

read -p "Image Height [480]: " IMG_HEIGHT
IMG_HEIGHT=${IMG_HEIGHT:-480}

# --- 2. Topic Configuration ---
read -p "Image Topic [/image_raw]: " TOPIC_IMG
TOPIC_IMG=${TOPIC_IMG:-/image_raw}

read -p "Camera Info Topic [/camera_info]: " TOPIC_INFO
TOPIC_INFO=${TOPIC_INFO:-/camera_info}

echo ""
echo "========================================="
echo "Settings Summary:"
echo "  Device: $CAM_DEV"
echo "  Format: MJPG @ ${IMG_WIDTH}x${IMG_HEIGHT}"
echo "========================================="
read -p "Press Enter to start..."

# ==============================================================================
# SOURCE WORKSPACE
# ==============================================================================
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# ==============================================================================
# HARDWARE PRE-TUNING (First Pass)
# ==============================================================================
echo ""
echo "[Hardware Fix] Pre-locking Settings via v4l2-ctl..."

# We force these now, but we ALSO send them to ROS below to prevent overwriting
v4l2-ctl -d $CAM_DEV -c focus_automatic_continuous=0 || true
v4l2-ctl -d $CAM_DEV -c focus_absolute=0 || true
v4l2-ctl -d $CAM_DEV -c auto_exposure=1 || true
v4l2-ctl -d $CAM_DEV -c exposure_time_absolute=150 || true
v4l2-ctl -d $CAM_DEV -c gain=30 || true

echo "✓ Settings Pre-Locked."

# ==============================================================================
# EXECUTION
# ==============================================================================

echo ""
echo "[1/2] Starting V4L2 Camera Node (Enforcing Params)..."

# CRITICAL CHANGE: Passing parameters (-p) tells ROS to keep these settings!
# exposure_dynamic_framerate=false prevents frame drops in low light
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=$CAM_DEV \
    -p image_width:=$IMG_WIDTH \
    -p image_height:=$IMG_HEIGHT \
    -p pixel_format:=yuyv \
    -p camera_name:=camera_logi \
    -p camera_info_url:=$CALIB_FILE \
    -p focus_automatic_continuous:=false \
    -p focus_absolute:=0 \
    -p auto_exposure:=1 \
    -p exposure_time_absolute:=150 \
    -p exposure_dynamic_framerate:=false \
    -p gain:=30 &

PID_CAM=$!
echo "✓ Camera Node PID: $PID_CAM"

echo "Waiting 3 seconds..."
sleep 3

echo ""
echo "[2/2] Starting AprilTag Node..."
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=$TOPIC_IMG \
    -r camera_info:=$TOPIC_INFO \
    --params-file $TAG_CONFIG &

PID_TAG=$!
echo "✓ AprilTag Node PID: $PID_TAG"

echo ""
echo "========================================="
echo "System Running. Press Ctrl+C to stop."
echo "========================================="

wait

# Wait indefinitely keeps the script running so the trap works
wait
