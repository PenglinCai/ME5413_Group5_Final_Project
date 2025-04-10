#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool

class NavigationController:
    def __init__(self):
        rospy.init_node('jackal_navigation_controller')

        # 初始化move_base action客户端
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server to start...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # 订阅AMCL位置数据，用于初始导航时获取机器人的位置和朝向
        self.current_pose = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        # 初始化发布器
        self.bridge_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # 第一个导航点，添加25秒超时
        first_goal = MoveBaseGoal()
        first_goal.target_pose.header.frame_id = "map"
        first_goal.target_pose.header.stamp = rospy.Time.now()
        first_goal.target_pose.pose.position.x = 12
        first_goal.target_pose.pose.position.y = -22.0
        first_goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Starting first initial navigation to (12.0, -21.0)")
        self.move_base_client.send_goal(first_goal)
        self.wait_for_nav(first_goal, threshold=0.5, timeout=25)
        rospy.loginfo("First initial navigation complete or timeout reached.")

        # 第二个导航点，添加25秒超时
        second_goal = MoveBaseGoal()
        second_goal.target_pose.header.frame_id = "map"
        second_goal.target_pose.header.stamp = rospy.Time.now()
        second_goal.target_pose.pose.position.x = 12
        second_goal.target_pose.pose.position.y = -3.0
        second_goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Starting second initial navigation to (12.0, -3.0)")
        self.move_base_client.send_goal(second_goal)
        self.wait_for_nav(second_goal, threshold=0.5, timeout=25)
        rospy.loginfo("Second initial navigation complete or timeout reached.")

        # 订阅导航目标，仅执行一次
        self.navigated = False
        self.sub_goal = rospy.Subscriber("/navigation_goal", PoseStamped, self.goal_callback)

    def amcl_callback(self, msg):
        self.current_pose = msg

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wait_for_nav(self, goal, threshold=0.35, timeout=25):
        """
        检查是否达到导航目标，当位置和朝向的误差都在阈值之内时视为达到。
        新增timeout参数，在等待超过timeout秒后退出循环并取消目标，进入下一个导航点。
        """
        rate = rospy.Rate(10)  # 10Hz检测频率
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_pose:
                # 获取当前与目标位置信息
                current_x = self.current_pose.pose.pose.position.x
                current_y = self.current_pose.pose.pose.position.y
                goal_x = goal.target_pose.pose.position.x
                goal_y = goal.target_pose.pose.position.y
                distance = math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
                # 计算方向偏差
                current_yaw = self.quaternion_to_yaw(self.current_pose.pose.pose.orientation)
                target_yaw = self.quaternion_to_yaw(goal.target_pose.pose.orientation)
                yaw_error = abs(target_yaw - current_yaw)
                if yaw_error > math.pi:
                    yaw_error = 2 * math.pi - yaw_error

                rospy.loginfo("Current distance to goal: %.2f m, orientation error: %.2f rad", distance, yaw_error)
                # 如果位置和方向均在阈值内，则认为到达目标
                if distance < threshold and yaw_error < threshold:
                    rospy.loginfo("Navigation threshold reached: distance=%.2f, yaw_error=%.2f", distance, yaw_error)
                    break

            # 检查是否超过超时时间
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn("Timeout reached after %.2f seconds for goal at (%.2f, %.2f). Moving to the next goal.",
                              timeout, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                break

            rate.sleep()
        # 一旦跳出循环，取消当前导航目标
        self.move_base_client.cancel_goal()

    def goal_callback(self, msg):
        # 如果已经执行过导航，则忽略后续目标
        if self.navigated:
            rospy.loginfo("Navigation already executed once, ignoring subsequent goals.")
            return

        # 标记已执行导航并取消订阅
        self.navigated = True
        self.sub_goal.unregister()

        # 对接收到的目标应用一个偏移
        msg.pose.position.x += 2.5
        msg.pose.position.y -= 0.1
        rospy.loginfo("Received navigation goal: (%.2f, %.2f)", msg.pose.position.x, msg.pose.position.y)

        # 发送move_base导航目标
        goal = MoveBaseGoal()
        goal.target_pose = msg
        self.move_base_client.send_goal(goal)
        rospy.loginfo("Sent move_base goal, starting navigation...")

        # 使用AMCL判断是否到达目标，原始的阈值设置继续使用
        distance_threshold = 0.4      # 米
        orientation_threshold = 0.35   # 弧度
        rate = rospy.Rate(10)         # 10Hz检查频率

        while not rospy.is_shutdown():
            if self.current_pose:
                current_x = self.current_pose.pose.pose.position.x
                current_y = self.current_pose.pose.pose.position.y
                goal_x = msg.pose.position.x 
                goal_y = msg.pose.position.y
                distance = math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)

                current_yaw = self.quaternion_to_yaw(self.current_pose.pose.pose.orientation)
                target_yaw = self.quaternion_to_yaw(msg.pose.orientation)
                yaw_error = abs(target_yaw - current_yaw)
                if yaw_error > math.pi:
                    yaw_error = 2 * math.pi - yaw_error

                rospy.loginfo("Current distance to goal: %.2f m, orientation error: %.2f rad", distance, yaw_error)

                if distance < distance_threshold and yaw_error < orientation_threshold:
                    rospy.loginfo("Target position and orientation reached.")
                    break
            rate.sleep()

        self.move_base_client.cancel_goal()
        rospy.sleep(2.0)

        # 发布桥梁开启信号
        bool_msg = Bool(data=True)
        self.bridge_pub.publish(bool_msg)
        rospy.loginfo("Published /cmd_open_bridge message: True")

        # 直行5米
        speed = 1  # m/s
        distance_travel = 5  # m
        duration = distance_travel / speed
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0.0

        rospy.loginfo("Starting to drive straight for %.2f meters", distance_travel)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()

        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo("Straight driving complete.")

        rospy.loginfo("Navigation task complete, shutting down node.")
        rospy.signal_shutdown("Navigation task complete")
        sys.exit(0)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        nav_controller = NavigationController()
        nav_controller.run()
    except rospy.ROSInterruptException:
        pass
