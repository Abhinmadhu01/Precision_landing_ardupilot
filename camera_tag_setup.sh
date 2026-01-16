#!/bin/bash
# Script to start V4L2 Camera and AprilTag ROS 2 Nodes
# Author: Abhin M (Modified by Gemini)

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

# Trap SIGINT (Ctrl+C)
trap cleanup SIGINT

# ==============================================================================
# USER INPUTS
# ==============================================================================
echo "========================================="
echo "   Vision System Startup (Cam + Tags)    "
echo "========================================="
echo ""

# --- 1. Camera Configuration ---
echo "--- Camera Configuration ---"
read -p "Camera Device [/dev/video2]: " CAM_DEV
CAM_DEV=${CAM_DEV:-/dev/video2}

read -p "Image Width [1920]: " IMG_WIDTH
IMG_WIDTH=${IMG_WIDTH:-1920}

read -p "Image Height [1080]: " IMG_HEIGHT
IMG_HEIGHT=${IMG_HEIGHT:-1080}

read -p "Framerate [30.0]: " IMG_FPS
IMG_FPS=${IMG_FPS:-30.0}

# --- 2. Topic Configuration ---
echo ""
echo "--- Topic Configuration ---"
read -p "Image Topic [/image_raw]: " TOPIC_IMG
TOPIC_IMG=${TOPIC_IMG:-/image_raw}

read -p "Camera Info Topic [/camera_info]: " TOPIC_INFO
TOPIC_INFO=${TOPIC_INFO:-/camera_info}

echo ""
echo "========================================="
echo "Settings Summary:"
echo "  Camera: $CAM_DEV @ ${IMG_WIDTH}x${IMG_HEIGHT} (${IMG_FPS}fps)"
echo "  Subscribing to: $TOPIC_IMG"
echo "========================================="
read -p "Press Enter to start..."

# ==============================================================================
# SOURCING WORKSPACES
# ==============================================================================
echo ""
echo "[Setting up Environment]"

# 1. Source System ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS 2 Humble"
else
    echo "❌ Error: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# 2. Source User Workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "✓ Sourced Workspace: $WORKSPACE_DIR"
else
    echo "❌ Error: Workspace setup file not found at $WORKSPACE_DIR/install/setup.bash"
    echo "   (Did you forget to run 'colcon build'?)"
    exit 1
fi

# ==============================================================================
# EXECUTION
# ==============================================================================

echo ""
echo "[1/2] Starting V4L2 Camera Node..."
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=$CAM_DEV \
    -p framerate:=$IMG_FPS \
    -p image_width:=$IMG_WIDTH \
    -p image_height:=$IMG_HEIGHT \
    -p pixel_format:=mjpeg2rgb \
    -p camera_name:=camera_logi \
    -p camera_info_url:=$CALIB_FILE &

PID_CAM=$!
echo "✓ Camera Node PID: $PID_CAM"

# Small delay to let camera node advertise topics
echo "Waiting 3 seconds for camera to initialize..."
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

# Wait indefinitely keeps the script running so the trap works
wait
