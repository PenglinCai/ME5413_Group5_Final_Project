#!/usr/bin/env python3
import rospy                        # Import ROS Python library for node handling
import actionlib                    # Import actionlib for using ROS action servers
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # Import required message types for move_base navigation
import tf                           # Import tf for coordinate transformations
import math                         # Import math library for mathematical functions
from geometry_msgs.msg import PoseWithCovarianceStamped  # Import pose message with covariance for AMCL localization

# Global variable to store the robot’s current position as estimated by AMCL
current_pose = None

def amcl_pose_callback(msg):
    # Callback function for the "amcl_pose" topic.
    # When a new AMCL pose is received, update the global current_pose variable with the new pose.
    global current_pose
    current_pose = msg.pose.pose

def send_goal(client, x, y, yaw, threshold=0.5, timeout_sec=None):
    """
    Create a navigation goal with the specified x, y coordinates and yaw angle,
    send it to the move_base action server, and monitor the robot's progress using AMCL updates.
    Cancel the goal when the robot reaches the target (within threshold) or when a timeout is reached.

    Parameters:
      client: The action client for move_base.
      x, y: Target position coordinates in the "map" frame.
      yaw: Target orientation (in radians) about the Z-axis.
      threshold: Distance threshold (in meters) to consider the goal as reached.
      timeout_sec: Maximum duration (in seconds) to wait for goal completion;
                   if None, the function will wait indefinitely.
    """
    # Create a new MoveBaseGoal instance for navigation.
    goal = MoveBaseGoal()
    # Set the frame in which the goal is defined to "map".
    goal.target_pose.header.frame_id = "map"
    # Set the timestamp for the goal to the current ROS time.
    goal.target_pose.header.stamp = rospy.Time.now()
    # Assign the target position values.
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Convert the given yaw angle (rotation about Z) to a quaternion for orientation.
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    # Set the quaternion components in the goal.
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    # Log the details of the goal being sent.
    rospy.loginfo("Sending goal: x=%f, y=%f, yaw=%f", x, y, yaw)
    # Send the goal to the move_base action server using the provided client.
    client.send_goal(goal)
    
    # Record the time when the goal was sent to manage timeouts.
    start_time = rospy.Time.now()
    # Set the loop rate to 10 Hz to check the robot's progress frequently.
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Check if current_pose has been updated by AMCL.
        if current_pose is not None:
            # Compute the differences in x and y between the current position and the target.
            dx = current_pose.position.x - x
            dy = current_pose.position.y - y
            # Calculate the Euclidean distance to the goal.
            distance = math.sqrt(dx * dx + dy * dy)
            rospy.loginfo("Current distance to goal: %.2f", distance)
            # If the robot is within the specified threshold, consider the goal reached.
            if distance <= threshold:
                rospy.loginfo("Reached goal based on AMCL: distance = %.2f", distance)
                # Cancel the current goal as it is reached.
                client.cancel_goal()
                break
        # If a timeout is specified and the elapsed time exceeds it, cancel the goal.
        if timeout_sec is not None and (rospy.Time.now() - start_time > rospy.Duration(timeout_sec)):
            rospy.logwarn("Timeout reached for goal: x=%.2f, y=%.2f. Cancelling goal.", x, y)
            client.cancel_goal()
            break
        # Sleep for the remainder of the loop cycle.
        rate.sleep()

def snake_pattern_horizontal(x_min, y_min, x_max, y_max, num_cols):
    """
    Generate a list of waypoints that form a horizontal "snake" pattern within a rectangular area.
    
    The area is defined by the bottom-left corner (x_min, y_min) and the top-right corner (x_max, y_max).
    The pattern is created by sampling a specified number of columns (num_cols) along the x-axis.
    For each column:
      - Even-indexed columns (starting from 0) traverse from bottom (y_min) to top (y_max).
      - Odd-indexed columns traverse from top (y_max) to bottom (y_min).

    Parameters:
      x_min: Minimum x-coordinate (left boundary).
      y_min: Minimum y-coordinate (bottom boundary).
      x_max: Maximum x-coordinate (right boundary).
      y_max: Maximum y-coordinate (top boundary).
      num_cols: Number of columns to generate along the x-axis.

    Returns:
      A list of (x, y) tuples representing the waypoints in a snake pattern.
    """
    waypoints = []  # Initialize an empty list to hold waypoints.
    # Compute the x-coordinates for waypoints, evenly spaced between x_max (right) and x_min (left).
    x_list = [x_max - i * (x_max - x_min) / float(num_cols - 1) for i in range(num_cols)]
    
    # Iterate through each x-coordinate and determine corresponding y-values based on column index.
    for idx, x in enumerate(x_list):
        # For even-indexed columns, traverse from bottom to top.
        if idx % 2 == 0:
            waypoints.append((x, y_min))
            waypoints.append((x, y_max))
        # For odd-indexed columns, traverse from top to bottom.
        else:
            waypoints.append((x, y_max))
            waypoints.append((x, y_min))
    # Return the complete list of generated waypoints.
    return waypoints

def main():
    # Initialize the ROS node with a specified node name.
    rospy.init_node("snake_pattern_nav_horizontal")
    
    # Subscribe to the "amcl_pose" topic to receive the robot's current position from AMCL.
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

    # Create a SimpleActionClient for communicating with the move_base action server.
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    # Block until the move_base server is available.
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # Define the initial target point where the robot should navigate first.
    initial_x = 22.3  # X-coordinate of the initial point
    initial_y = -21.5 # Y-coordinate of the initial point
    initial_yaw = 0.76  # Yaw orientation in radians for the initial point
    rospy.loginfo("Navigating to initial point: x=%f, y=%f", initial_x, initial_y)
    # Send the navigation goal to the initial point without a timeout.
    send_goal(client, initial_x, initial_y, initial_yaw)
    # Wait for a second to ensure the robot has time to stabilize at the initial point.
    rospy.sleep(1)

    # Define the rectangular area boundaries for the snake pattern navigation.
    x_min = 12    # Left boundary of the area
    y_min = -22.5 # Bottom boundary of the area
    x_max = 18.0  # Right boundary of the area
    y_max = -3.5  # Top boundary of the area

    # Specify the number of columns to generate in the snake pattern.
    num_cols = 2  # More columns yield a denser coverage pattern.
    # Generate the snake pattern waypoints within the defined area.
    waypoints = snake_pattern_horizontal(x_min, y_min, x_max, y_max, num_cols)
    rospy.loginfo("Generated waypoints: %s", waypoints)

    # Iterate over each generated waypoint and command the robot to navigate to it.
    for (x, y) in waypoints:
        yaw = 0.0  # Set the desired orientation (yaw) at each waypoint (adjustable as needed)
        # Send the navigation goal to each waypoint with a timeout of 40 seconds.
        send_goal(client, x, y, yaw, timeout_sec=40)
        # Pause for one second between waypoints to allow the robot to settle.
        rospy.sleep(1)

if __name__ == '__main__':
    # Execute the main function within a try-except block to catch ROS interrupt exceptions.
    try:
        main()
    except rospy.ROSInterruptException:
        pass
