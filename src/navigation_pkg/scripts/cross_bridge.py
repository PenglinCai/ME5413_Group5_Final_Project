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

        # Subscribe to AMCL pose data
        self.current_pose = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        # Publishers
        self.bridge_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # First navigation point (no orientation requirement)
        first_goal = MoveBaseGoal()
        first_goal.target_pose.header.frame_id = "map"
        first_goal.target_pose.header.stamp = rospy.Time.now()
        first_goal.target_pose.pose.position.x = 18.0
        first_goal.target_pose.pose.position.y = -3.5
        first_goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Starting first initial navigation to (18.0, -3.5)")
        self.move_base_client.send_goal(first_goal)
        self.wait_for_nav(first_goal, threshold=0.4, timeout=25, check_yaw=False)
        rospy.loginfo("First initial navigation complete or timeout reached.")

        # Second navigation point (no orientation requirement)
        second_goal = MoveBaseGoal()
        second_goal.target_pose.header.frame_id = "map"
        second_goal.target_pose.header.stamp = rospy.Time.now()
        second_goal.target_pose.pose.position.x = 11.0
        second_goal.target_pose.pose.position.y = -3.5
        second_goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Starting second initial navigation to (11.0, -3.5)")
        self.move_base_client.send_goal(second_goal)
        self.wait_for_nav(second_goal, threshold=0.4, timeout=25, check_yaw=False)
        rospy.loginfo("Second initial navigation complete or timeout reached.")

        # Third navigation point (no orientation requirement)
        third_goal = MoveBaseGoal()
        third_goal.target_pose.header.frame_id = "map"
        third_goal.target_pose.header.stamp = rospy.Time.now()
        third_goal.target_pose.pose.position.x = 11.0
        third_goal.target_pose.pose.position.y = -21.0
        third_goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Starting third initial navigation to (11.0, -21.0)")
        self.move_base_client.send_goal(third_goal)
        self.wait_for_nav(third_goal, threshold=0.4, timeout=25, check_yaw=False)
        rospy.loginfo("Third initial navigation complete or timeout reached.")

        # Subscribe to navigation goal, executed only once
        self.navigated = False
        self.sub_goal = rospy.Subscriber("/navigation_goal", PoseStamped, self.goal_callback)

    def amcl_callback(self, msg):
        self.current_pose = msg

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wait_for_nav(self, goal, threshold=0.35, timeout=25, check_yaw=True):
        """
        Wait until the robot reaches the goal position within 'threshold' meters.
        If check_yaw is True, also require orientation error < threshold radians.
        Timeout after 'timeout' seconds.
        """
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_pose:
                # Compute position error
                cx = self.current_pose.pose.pose.position.x
                cy = self.current_pose.pose.pose.position.y
                gx = goal.target_pose.pose.position.x
                gy = goal.target_pose.pose.position.y
                distance = math.hypot(cx - gx, cy - gy)

                # Compute orientation error if required
                yaw_error = 0.0
                if check_yaw:
                    current_yaw = self.quaternion_to_yaw(self.current_pose.pose.pose.orientation)
                    target_yaw = self.quaternion_to_yaw(goal.target_pose.pose.orientation)
                    yaw_error = abs(target_yaw - current_yaw)
                    if yaw_error > math.pi:
                        yaw_error = 2 * math.pi - yaw_error

                rospy.loginfo("Distance to goal: %.2f m%s", distance,
                              (", yaw error: %.2f rad" % yaw_error) if check_yaw else "")

                # Check thresholds
                if distance < threshold and (not check_yaw or yaw_error < threshold):
                    rospy.loginfo("Navigation threshold reached.")
                    break

            # Check timeout
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Timeout of %.1f s reached for goal (%.2f, %.2f).", timeout,
                              goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                break

            rate.sleep()

        # Cancel goal when done or timed out
        self.move_base_client.cancel_goal()

    def goal_callback(self, msg):
        if self.navigated:
            rospy.loginfo("Navigation already executed once, ignoring subsequent goals.")
            return
        self.navigated = True
        self.sub_goal.unregister()

        # Offset and send the received goal
        msg.pose.position.x += 2.5
        msg.pose.position.y -= 0.1
        rospy.loginfo("Received navigation goal: (%.2f, %.2f)", msg.pose.position.x, msg.pose.position.y)

        goal = MoveBaseGoal()
        goal.target_pose = msg
        self.move_base_client.send_goal(goal)
        rospy.loginfo("Sent move_base goal, starting navigation...")

        # Original position & orientation checks
        dist_th = 0.4
        yaw_th = 0.35
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose:
                cx = self.current_pose.pose.pose.position.x
                cy = self.current_pose.pose.pose.position.y
                gx = msg.pose.position.x
                gy = msg.pose.position.y
                distance = math.hypot(cx - gx, cy - gy)

                current_yaw = self.quaternion_to_yaw(self.current_pose.pose.pose.orientation)
                target_yaw = self.quaternion_to_yaw(msg.pose.orientation)
                yaw_error = abs(target_yaw - current_yaw)
                if yaw_error > math.pi:
                    yaw_error = 2 * math.pi - yaw_error

                rospy.loginfo("Distance: %.2f m, Yaw error: %.2f rad", distance, yaw_error)
                if distance < dist_th and yaw_error < yaw_th:
                    rospy.loginfo("Target position and orientation reached.")
                    break
            rate.sleep()

        self.move_base_client.cancel_goal()
        rospy.sleep(2.0)

        # Open bridge
        self.bridge_pub.publish(Bool(data=True))
        rospy.loginfo("Published /cmd_open_bridge message: True")

        # Drive straight for 5 meters
        speed = 1.0
        distance_travel = 5.0
        duration = distance_travel / speed
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        rospy.loginfo("Starting to drive straight for %.2f meters", distance_travel)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Stop motion
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
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
