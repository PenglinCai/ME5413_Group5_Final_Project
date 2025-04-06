#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from collections import defaultdict
import re

class CheatSelectTarget:
    def __init__(self):
        rospy.init_node('cheat_select_target_node')

        # 发布2D导航目标
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

        # 订阅模型状态
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        # 标志位，确保只处理一次
        self.processed = False

    def model_states_callback(self, msg):
        if self.processed:
            return

        model_names = msg.name
        pose_list = msg.pose

        number_count = defaultdict(int)
        model_pose_dict = {}

        # 统计带下划线的模型，记录数量
        for name, pose in zip(model_names, pose_list):
            match = re.match(r'number(\d+)_(\d+)', name)
            if match:
                num = match.group(1)
                number_count[num] += 1
            elif re.match(r'number\d+$', name):
                model_pose_dict[name] = pose

        rospy.loginfo("统计结果: %s", dict(number_count))

        if not number_count:
            rospy.logwarn("没有找到符合条件的模型！")
            return

        # 找出现最少的数字
        min_number = min(number_count, key=number_count.get)
        target_model_name = f"number{min_number}"

        rospy.loginfo("出现最少的数字是: %s (目标模型: %s)", min_number, target_model_name)

        if target_model_name not in model_pose_dict:
            rospy.logwarn("目标模型在model_states中找不到！")
            return

        # 发布目标位置（带Gazebo➔RViz坐标变换）
        pose = model_pose_dict[target_model_name]
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        # 坐标变换
        goal.pose.position.x = pose.position.y+1       # Gazebo的 y 变成 NavGoal的 x
        goal.pose.position.y = -pose.position.x      # Gazebo的 x 取负，作为NavGoal的 y
        goal.pose.position.z = 0.0                   # z固定为0

        # 姿态固定
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0

        self.goal_pub.publish(goal)
        rospy.loginfo("已发布2D Nav Goal到模型 [%s] 转换后的位置！", target_model_name)

        # 防止多次处理
        self.processed = True

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = CheatSelectTarget()
        node.run()
    except rospy.ROSInterruptException:
        pass

