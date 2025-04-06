#!/bin/bash

# Step 3 & Step 5: ROS1 记录 Ground Truth 和 SLAM Pose 的脚本
# 适用于 ROS Noetic，保存两个 rosbag，后续可用于 EVO 轨迹评估

# === 设置保存目录 ===
EVAL_DIR=~/slam_eval
mkdir -p $EVAL_DIR

echo "✅ 数据将保存至：$EVAL_DIR"

# === 启动两个终端分别记录话题 ===
echo "📡 启动 rosbag 记录 Ground Truth 和 SLAM Pose..."

# Ground Truth 话题记录
gnome-terminal -- bash -c "rosbag record -O $EVAL_DIR/ground_truth.bag /gazebo/ground_truth/state; exec bash"

# SLAM Pose 话题记录
gnome-terminal -- bash -c "rosbag record -O $EVAL_DIR/slam_traj.bag /tracked_pose; exec bash"

echo ""
echo "🟢 正在记录中，请在 RViz 中控制机器人进行建图任务..."
echo "🔴 建图完成后，回到这个终端按 [ENTER] 停止记录..."
read

# === 停止记录（终止所有 rosbag） ===
echo "🛑 停止 rosbag 记录..."
pkill -f "rosbag record"

echo ""
echo "✅ 数据记录完成！"
echo "📁 保存路径：$EVAL_DIR"
echo "📄 Ground Truth:  ground_truth.bag"
echo "📄 SLAM Pose:     slam_traj.bag"

