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

        # Initialize move_base action client
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server to start...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Perform initial navigation: from [12, -22] to [12, -3]
        init_goal = MoveBaseGoal()
        init_goal.target_pose.header.frame_id = "map"
        init_goal.target_pose.header.stamp = rospy.Time.now()
        init_goal.target_pose.pose.position.x = 12.0
        init_goal.target_pose.pose.position.y = -21.0
        init_goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Starting initial navigation to (12.0, -22.0)")
        self.move_base_client.send_goal(init_goal)
        self.move_base_client.wait_for_result()
        rospy.loginfo("Initial navigation complete, arrived at (12.0, -2.0)")

        second_goal = MoveBaseGoal()
        second_goal.target_pose.header.frame_id = "map"
        second_goal.target_pose.header.stamp = rospy.Time.now()
        second_goal.target_pose.pose.position.x = 12.0
        second_goal.target_pose.pose.position.y = -3.0
        second_goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Starting initial navigation to (12.0, -3.0)")
        self.move_base_client.send_goal(second_goal)
        self.move_base_client.wait_for_result()
        rospy.loginfo("Initial navigation complete, arrived at (12.0, -30)")

        # Initialize publishers
        self.bridge_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # For storing current pose
        self.current_pose = None

        # Subscribe to AMCL pose data
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        
        # Subscribe to navigation goal messages, execute only once
        self.navigated = False
        self.sub_goal = rospy.Subscriber("/navigation_goal", PoseStamped, self.goal_callback)

    def amcl_callback(self, msg):
        self.current_pose = msg

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg):
        # If navigation already done, return immediately
        if self.navigated:
            rospy.loginfo("Navigation already executed once, ignoring subsequent goals.")
            return

        # Mark navigation executed and unsubscribe
        self.navigated = True
        self.sub_goal.unregister()

        # Apply offset adjustments to received goal
        msg.pose.position.x += 2.5
        msg.pose.position.y -= 0.18
        rospy.loginfo("Received navigation goal: (%.2f, %.2f)", msg.pose.position.x, msg.pose.position.y)

        # Send move_base goal
        goal = MoveBaseGoal()
        goal.target_pose = msg
        self.move_base_client.send_goal(goal)
        rospy.loginfo("Sent move_base goal, starting navigation...")

        # Use AMCL to determine arrival at goal
        distance_threshold = 0.4      # meters
        orientation_threshold = 0.3   # radians
        rate = rospy.Rate(10)         # 10Hz check frequency

        while not rospy.is_shutdown():
            if self.current_pose:
                # Current pose
                current_x = self.current_pose.pose.pose.position.x
                current_y = self.current_pose.pose.pose.position.y
                # Target pose (with offset)
                goal_x = msg.pose.position.x 
                goal_y = msg.pose.position.y
                # Euclidean distance
                distance = math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)

                # Orientation error
                current_orientation = self.current_pose.pose.pose.orientation
                target_orientation = msg.pose.orientation
                current_yaw = self.quaternion_to_yaw(current_orientation)
                target_yaw = self.quaternion_to_yaw(target_orientation)
                yaw_error = abs(target_yaw - current_yaw)
                if yaw_error > math.pi:
                    yaw_error = 2 * math.pi - yaw_error

                rospy.loginfo("Current distance to goal: %.2f m, orientation error: %.2f rad", distance, yaw_error)

                # Exit loop when both position and orientation thresholds are met
                if distance < distance_threshold and yaw_error < orientation_threshold:
                    rospy.loginfo("Target position and orientation reached.")
                    break
            rate.sleep()

        # Cancel move_base goal after arrival
        self.move_base_client.cancel_goal()
        rospy.sleep(2.0)

        # Publish /cmd_open_bridge message
        bool_msg = Bool(data=True)
        self.bridge_pub.publish(bool_msg)
        rospy.loginfo("Published /cmd_open_bridge message: True")

        # Maintain orientation and drive straight for 5 meters
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

        # Stop motion
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo("Straight driving complete.")

        # Shutdown node after navigation complete
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
