#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger
import os

class DigitRecognitionClient:
    def __init__(self):
        rospy.wait_for_service('/start_recognition')
        rospy.wait_for_service('/stop_recognition')
        rospy.loginfo("Digit Recognition Client Initialized")
        self.start_srv = rospy.ServiceProxy('/start_recognition', Trigger)
        self.stop_srv = rospy.ServiceProxy('/stop_recognition', Trigger)

    def start_recognition(self):
        try:
            self.start_srv()
            rospy.loginfo("Started digit recognition.")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to start recognition: %s", str(e))

    def stop_and_get_result(self):
        try:
            response = self.stop_srv()
            message = response.message
            rospy.loginfo("Recognition raw message: %s", message)
            digit = self.extract_digit(message)
            return digit
        except rospy.ServiceException as e:
            rospy.logerr("Failed to stop recognition: %s", str(e))
            return None

    def extract_digit(self, message):
        for c in message:
            if c.isdigit():
                return int(c)
        return None

class AfterBridge:
    def __init__(self):
        rospy.init_node('after_bridge_node')

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base!")

        self.digit_client = DigitRecognitionClient()

        self.target_digit = None

        # Define four target positions
        self.box_positions = [
            (0, -6),
            (0, -10),
            (0, -14),
            (0, -18)
        ]

        self.max_retry = 3
        rospy.loginfo("Initialization completed!")

        # Load digit sequence from file
        self.load_digit_sequence()

    def load_digit_sequence(self):
        try:
            filepath = os.path.join(os.getcwd(), "recognized_digits.txt")
            rospy.loginfo(f"Loading digit sequence from {filepath}")
            with open(filepath, "r") as f:
                content = f.read()
                # Parse numbers into a list
                digits = [int(num.strip()) for num in content.strip().split(",") if num.strip().isdigit()]
                rospy.loginfo("Loaded digits: %s", digits)

                # Count occurrences
                counts = {}
                for d in digits:
                    counts[d] = counts.get(d, 0) + 1
                rospy.loginfo("Digit occurrence counts: %s", counts)

                # Find the digit with the least occurrences
                self.target_digit = min(counts, key=counts.get)
                rospy.loginfo("Target digit (least frequent) is: %d", self.target_digit)

        except Exception as e:
            rospy.logerr("Failed to load or parse recognized_digits.txt: %s", str(e))
            rospy.signal_shutdown("Cannot load digits, shutting down.")

    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x + 2
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.z = 1
        goal.target_pose.pose.orientation.w = 0

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        rospy.loginfo("Reached goal at (%.2f, %.2f)", x, y)

    def recognize_digit(self):
        for attempt in range(self.max_retry):
            try:
                rospy.loginfo("Starting recognition attempt %d...", attempt + 1)
                self.digit_client.start_recognition()
                rospy.sleep(2.0)
                recognized_digit = self.digit_client.stop_and_get_result()
                if recognized_digit is not None:
                    rospy.loginfo("Recognized digit: %d", recognized_digit)
                    return recognized_digit
                else:
                    rospy.logwarn("No valid digit detected. Retrying...")
            except Exception as e:
                rospy.logwarn("Recognition attempt %d failed: %s", attempt + 1, str(e))
            rospy.sleep(1.0)
        rospy.logerr("Failed to recognize digit after %d attempts.", self.max_retry)
        return None

    def run(self):
        rospy.loginfo("Start navigating to boxes...")

        for idx, (x, y) in enumerate(self.box_positions, 1):
            rospy.loginfo("Navigating to Box %d at (%.2f, %.2f)", idx, x, y)
            self.send_goal(x, y)

            recognized_digit = self.recognize_digit()
            if recognized_digit is not None:
                rospy.loginfo("Box %d: Recognized Digit = %d", idx, recognized_digit)
                if recognized_digit == self.target_digit:
                    rospy.loginfo("Found the target box with digit %d! Shutting down.", recognized_digit)
                    rospy.signal_shutdown("Mission completed.")
                    break
            else:
                rospy.logwarn("Box %d: Recognition failed.", idx)

        rospy.loginfo("Finished visiting all boxes. Task ended.")

if __name__ == "__main__":
    try:
        node = AfterBridge()
        node.run()
    except rospy.ROSInterruptException:
        pass

