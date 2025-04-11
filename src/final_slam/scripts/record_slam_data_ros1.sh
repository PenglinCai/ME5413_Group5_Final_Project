#!/bin/bash

# ROS1 script to record Ground Truth and SLAM Pose
# For ROS Noetic, saves two rosbags for later trajectory evaluation using EVO

# === Set the relative save directory ===
SCRIPT_DIR=$(dirname "$(realpath "$0")")   # Get the directory where this script is located
EVAL_DIR="$SCRIPT_DIR/../eval"              # Relative path: up one level then into eval
mkdir -p "$EVAL_DIR"

echo "Data will be saved to: $EVAL_DIR"

# === Launch two terminals to record topics separately ===
echo "Starting rosbag recording for Ground Truth and SLAM Pose..."

# Record Ground Truth topic
gnome-terminal -- bash -c "rosbag record -O $EVAL_DIR/ground_truth.bag /gazebo/ground_truth/state; exec bash"

# Record SLAM Pose topic
gnome-terminal -- bash -c "rosbag record -O $EVAL_DIR/slam_traj.bag /tracked_pose; exec bash"

echo ""
echo "Recording... Please control the robot in RViz to perform the mapping task."
echo "After mapping is complete, return to this terminal and press [ENTER] to stop recording."
read

# === Stop recording (terminate all rosbag processes) ===
echo "Stopping rosbag recording..."
pkill -f "rosbag record"

# === Reindex .bag.active files ===
echo "Reindexing and renaming .bag.active files..."

if [ -f "$EVAL_DIR/ground_truth.bag.active" ]; then
    echo "Processing ground_truth.bag.active..."
    rosbag reindex "$EVAL_DIR/ground_truth.bag.active"
    mv "$EVAL_DIR/ground_truth.bag.active" "$EVAL_DIR/ground_truth.bag"
    echo "Finished ground_truth.bag!"
else
    echo "Warning: ground_truth.bag.active not found."
fi

if [ -f "$EVAL_DIR/slam_traj.bag.active" ]; then
    echo "Processing slam_traj.bag.active..."
    rosbag reindex "$EVAL_DIR/slam_traj.bag.active"
    mv "$EVAL_DIR/slam_traj.bag.active" "$EVAL_DIR/slam_traj.bag"
    echo "Finished slam_traj.bag!"
else
    echo "Warning: slam_traj.bag.active not found."
fi

echo ""
echo "Data recording and processing completed!"
echo "Save directory: $EVAL_DIR"
echo "Ground Truth bag:  ground_truth.bag"
echo "SLAM Pose bag:     slam_traj.bag"

