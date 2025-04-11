#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# This script implements a ROS node that navigates to several box positions
# and uses a digit recognition service to find a target box based on digit occurrence frequency.
# It uses actionlib to send navigation goals to move_base and calls two services for digit recognition.

import rospy                      # Import the core ROS Python functionality.
import actionlib                  # Import the ROS actionlib library to send/receive goals.
import numpy as np                # Import numpy for numerical operations (e.g., sqrt for distance).
import math                       # Import math library for math functions like sqrt and atan2.
import os                         # Import os module to work with file system paths.
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # Import messages to interact with move_base.
from std_srvs.srv import Trigger  # Import the Trigger service type used by the recognition services.
from geometry_msgs.msg import PoseWithCovarianceStamped  # Import message type for robot localization (AMCL).
from tf.transformations import euler_from_quaternion  # Import helper function to convert quaternion to Euler angles.

# Define a class for digit recognition, which wraps the start and stop service calls.
class DigitRecognitionClient:
    def __init__(self):
        # Wait until the '/start_recognition' service is available.
        rospy.wait_for_service('/start_recognition')
        # Wait until the '/stop_recognition' service is available.
        rospy.wait_for_service('/stop_recognition')
        # Log that the digit recognition client is being initialized.
        rospy.loginfo("Digit Recognition Client Initialized")
        # Create a service proxy to call the '/start_recognition' service with a Trigger request.
        self.start_srv = rospy.ServiceProxy('/start_recognition', Trigger)
        # Create a service proxy to call the '/stop_recognition' service with a Trigger request.
        self.stop_srv = rospy.ServiceProxy('/stop_recognition', Trigger)

    def start_recognition(self):
        # Attempt to call the start recognition service.
        try:
            self.start_srv()  # Call the service (no arguments needed for Trigger).
            rospy.loginfo("Started digit recognition.")
        except rospy.ServiceException as e:
            # Log an error if the service call fails.
            rospy.logerr("Failed to start recognition: %s", str(e))

    def stop_and_get_result(self):
        # Attempt to stop recognition and obtain the result.
        try:
            response = self.stop_srv()  # Call the stop recognition service.
            message = response.message  # Extract the message from the service response.
            rospy.loginfo("Recognition raw message: %s", message)
            # Call the helper function to extract a digit from the message.
            digit = self.extract_digit(message)
            return digit  # Return the extracted digit.
        except rospy.ServiceException as e:
            # If the service call fails, log an error and return None.
            rospy.logerr("Failed to stop recognition: %s", str(e))
            return None

    def extract_digit(self, message):
        # Go through each character in the message.
        for c in message:
            # Check if the character is a digit.
            if c.isdigit():
                # Return the digit as an integer when found.
                return int(c)
        # If no digit is found, return None.
        return None

# Define the main class for navigating after the bridge.
class AfterBridge:
    def __init__(self):
        # Initialize the ROS node with the name 'after_bridge_node'.
        rospy.init_node('after_bridge_node')
        
        # Create an action client for the move_base action server.
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # Block until the move_base server is available.
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base!")
        
        # Create an instance of the DigitRecognitionClient to handle digit recognition.
        self.digit_client = DigitRecognitionClient()
        
        # Initialize the target digit to None; will be set after loading digit sequence.
        self.target_digit = None

        # Define the positions for the boxes that need to be visited.
        # Each tuple represents a target position (x, y) in the map frame.
        self.box_positions = [
            (0, -6),
            (0, -10),
            (0, -14),
            (0, -18)
        ]

        # Set a maximum number of retry attempts for digit recognition.
        self.max_retry = 3
        rospy.loginfo("Initialization completed!")
        
        # Variable to hold the current robot pose as received from AMCL.
        self.current_pose = None
        # Subscribe to the "amcl_pose" topic to get the robot’s current estimated position.
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        
        # Load the digit sequence from a file to determine the target digit.
        self.load_digit_sequence()

    def amcl_pose_callback(self, msg):
        # Callback for the 'amcl_pose' topic.
        # Save the current robot pose from the incoming message.
        self.current_pose = msg.pose.pose

    def load_digit_sequence(self):
        # Attempt to load the recognized digits sequence from 'recognized_digits.txt'
        try:
            # Create the file path to the recognized digits file in the current working directory.
            filepath = os.path.join(os.getcwd(), "recognized_digits.txt")
            rospy.loginfo("Loading digit sequence from %s", filepath)
            with open(filepath, "r") as f:
                content = f.read()  # Read the entire content of the file.
                # Split the content by commas, strip whitespace, and parse integers.
                digits = [int(num.strip()) for num in content.strip().split(",") if num.strip().isdigit()]
                rospy.loginfo("Loaded digits: %s", digits)
                
                # Count how many times each digit appears.
                counts = {}
                for d in digits:
                    counts[d] = counts.get(d, 0) + 1
                rospy.loginfo("Digit occurrence counts: %s", counts)
                
                # Choose the digit with the fewest occurrences as the target digit.
                self.target_digit = min(counts, key=counts.get)
                rospy.loginfo("Target digit (least frequent) is: %d", self.target_digit)
        
        except Exception as e:
            # Log an error and shut down the node if the file cannot be read or parsed.
            rospy.logerr("Failed to load or parse recognized_digits.txt: %s", str(e))
            rospy.signal_shutdown("Cannot load digits, shutting down.")

    def send_goal(self, x, y):
        # Create a new MoveBaseGoal instance to navigate to a box.
        # An offset of 2 is applied to the x-coordinate during navigation.
        goal = MoveBaseGoal()
        # Set the header frame id to "map" so that the goal is relative to the map.
        goal.target_pose.header.frame_id = 'map'
        # Set the timestamp to the current time.
        goal.target_pose.header.stamp = rospy.Time.now()
        # Set the target x position by applying an offset of 2 to the given x value.
        goal.target_pose.pose.position.x = x + 2
        # Set the target y position.
        goal.target_pose.pose.position.y = y
        # Set the target z position to 0.0 (assuming flat ground).
        goal.target_pose.pose.position.z = 0.0
        # Set the desired orientation for the goal.
        # Here we define a fixed quaternion corresponding to a yaw of π (180 degrees).
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 1.0
        goal.target_pose.pose.orientation.w = 0.0

        # Log the target position (before offset) where the goal is being sent.
        rospy.loginfo("Sending goal to (%.2f, %.2f)...", x, y)
        # Send the goal to the move_base action server.
        self.move_base_client.send_goal(goal)

        # Set up a loop to monitor the robot's progress toward the goal.
        rate = rospy.Rate(2)  # Check the status at 2 Hz.
        pos_threshold = 0.36  # Threshold for positional error (in meters).
        ori_threshold = 0.28  # Threshold for orientation error (in radians).
        # Compute the target position after applying the x offset.
        target_x = x + 2
        target_y = y
        # Extract the target yaw (orientation) from the goal's quaternion.
        goal_quat = [
            goal.target_pose.pose.orientation.x,
            goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z,
            goal.target_pose.pose.orientation.w
        ]
        # Convert quaternion to Euler angles to obtain yaw.
        goal_yaw = euler_from_quaternion(goal_quat)[2]

        # Continue checking until the goal is reached or ROS shuts down.
        while not rospy.is_shutdown():
            # If we have not received the current robot pose yet, wait and continue.
            if self.current_pose is None:
                rate.sleep()
                continue

            # Get the current x and y positions from the received AMCL pose.
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y

            # Calculate the Euclidean distance error between current position and target position.
            distance_error = np.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2)

            # Get current orientation in quaternion form.
            current_quat = [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ]
            # Convert current quaternion to Euler angles and extract the yaw.
            current_yaw = euler_from_quaternion(current_quat)[2]
            # Compute the minimal angular difference between target yaw and current yaw.
            orientation_error = abs(math.atan2(math.sin(goal_yaw - current_yaw), math.cos(goal_yaw - current_yaw)))

            # Log the current position and orientation errors.
            rospy.loginfo("Distance error: %.3f, Orientation error: %.3f", distance_error, orientation_error)

            # Check if both the distance and orientation errors are below their thresholds.
            if distance_error < pos_threshold and orientation_error < ori_threshold:
                rospy.loginfo("Reached goal at (%.2f, %.2f) with errors: distance=%.3f, orientation=%.3f", x, y, distance_error, orientation_error)
                # Cancel the goal as it has been reached.
                self.move_base_client.cancel_goal()
                break

            # Wait for the next check cycle.
            rate.sleep()

    def recognize_digit(self):
        # Attempt to recognize a digit from the current location with retries.
        for attempt in range(self.max_retry):
            try:
                rospy.loginfo("Starting recognition attempt %d...", attempt + 1)
                # Start the digit recognition service.
                self.digit_client.start_recognition()
                # Allow time (2 seconds) for the recognition service to process.
                rospy.sleep(2.0)
                # Stop recognition and get the recognized digit.
                recognized_digit = self.digit_client.stop_and_get_result()
                # If a valid digit is recognized, return it.
                if recognized_digit is not None:
                    rospy.loginfo("Recognized digit: %d", recognized_digit)
                    return recognized_digit
                else:
                    rospy.logwarn("No valid digit detected. Retrying...")
            except Exception as e:
                # Log any exceptions and prepare to retry.
                rospy.logwarn("Recognition attempt %d failed: %s", attempt + 1, str(e))
            # Wait a moment before retrying.
            rospy.sleep(1.0)
        # After all attempts, if no valid digit was recognized, log an error.
        rospy.logerr("Failed to recognize digit after %d attempts.", self.max_retry)
        return None

    def run(self):
        # Main function to run the post-bridge navigation and digit recognition process.
        rospy.loginfo("Start navigating to boxes...")

        # Iterate through each box position defined in self.box_positions.
        for idx, (x, y) in enumerate(self.box_positions, 1):
            rospy.loginfo("Navigating to Box %d at (%.2f, %.2f)", idx, x, y)
            # Send a navigation goal to the current box position.
            self.send_goal(x, y)

            # After reaching the box, attempt to recognize the digit at that location.
            recognized_digit = self.recognize_digit()
            if recognized_digit is not None:
                rospy.loginfo("Box %d: Recognized Digit = %d", idx, recognized_digit)
                # Check if the recognized digit matches the target digit.
                if recognized_digit == self.target_digit:
                    rospy.loginfo("Found the target box with digit %d! Shutting down.", recognized_digit)
                    # If target is found, shut down the node.
                    rospy.signal_shutdown("Mission completed.")
                    break
            else:
                rospy.logwarn("Box %d: Recognition failed.", idx)

        rospy.loginfo("Finished visiting all boxes. Task ended.")

# The main entry point for the script.
if __name__ == "__main__":
    try:
        # Create an instance of the AfterBridge class.
        node = AfterBridge()
        # Run the main process.
        node.run()
    except rospy.ROSInterruptException:
        # Catch and silently handle any ROS interrupt exceptions.
        pass
