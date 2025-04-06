#!/bin/bash

# ————————————————
# 完整修复版 EVO 分析脚本
# 确保输出 8 列 TUM 格式轨迹
# ————————————————

# 1. 路径配置
EVAL_DIR=/home/jv/slam_eval
GT_BAG=$EVAL_DIR/ground_truth.bag
SLAM_BAG=$EVAL_DIR/slam_traj.bag
GT_TXT=$EVAL_DIR/traj_gt.txt
SLAM_TXT=$EVAL_DIR/traj_slam.txt
PLOT_PDF=$EVAL_DIR/evo_ape_plot.pdf
RESULTS_ZIP=$EVAL_DIR/evo_ape_results.zip

# 2. 工具路径
EVO_TRAJ=~/.local/bin/evo_traj
EVO_APE=~/.local/bin/evo_ape

echo "📁 数据目录: $EVAL_DIR"

# 3. 检查输入
[ ! -f "$GT_BAG" ] && echo "❌ 找不到 $GT_BAG" && exit 1
[ ! -f "$SLAM_BAG" ] && echo "❌ 找不到 $SLAM_BAG" && exit 1

# 4. 清理旧文件
rm -f "$GT_TXT" "$SLAM_TXT" "$PLOT_PDF" "$RESULTS_ZIP"

# 5. 提取 Ground Truth 轨迹（TUM 格式）
echo "🔄 提取 Ground Truth 轨迹到 $GT_TXT ..."
$EVO_TRAJ bag "$GT_BAG" /gazebo/ground_truth/state --save_as_tum > "$GT_TXT"

# 6. 提取 SLAM 估计轨迹（TUM 格式）
echo "🔄 提取 SLAM 轨迹到 $SLAM_TXT ..."
$EVO_TRAJ bag "$SLAM_BAG" /tracked_pose --save_as_tum > "$SLAM_TXT"

# 7. 运行 APE 分析
echo "📊 正在运行 APE 分析..."
$EVO_APE tum "$GT_TXT" "$SLAM_TXT" -va --plot --plot_mode xyz \
  --save_results "$RESULTS_ZIP" --save_plot "$PLOT_PDF"

# 8. 完成提示
echo ""
echo "✅ 分析完成！"
echo "  • GT 轨迹:  $GT_TXT"
echo "  • SLAM 轨迹: $SLAM_TXT"
echo "  • 误差图:   $PLOT_PDF"
echo "  • 统计包:   $RESULTS_ZIP"

