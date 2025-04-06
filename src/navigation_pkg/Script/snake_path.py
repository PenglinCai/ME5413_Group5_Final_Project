#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

# Global variable to store the current estimated position from AMCL
current_pose = None

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def send_goal(client, x, y, yaw, threshold=0.5, timeout_sec=None):
    """
    Create a MoveBaseGoal based on the provided x, y coordinates and yaw angle,
    send it to the move_base action server, and use AMCL to determine when the goal is reached.
    If timeout_sec is not None, cancel the goal if it is not reached within timeout_sec seconds
    to prevent getting stuck if the target is inside an obstacle; if timeout_sec is None, wait indefinitely.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Convert yaw to quaternion
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo("Sending goal: x=%f, y=%f, yaw=%f", x, y, yaw)
    client.send_goal(goal)
    
    # Timeout handling to avoid getting stuck on unreachable goals
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # If current position is updated, calculate distance to goal
        if current_pose is not None:
            dx = current_pose.position.x - x
            dy = current_pose.position.y - y
            distance = math.sqrt(dx * dx + dy * dy)
            rospy.loginfo("Current distance to goal: %.2f", distance)
            if distance <= threshold:
                rospy.loginfo("Reached goal based on AMCL: distance = %.2f", distance)
                client.cancel_goal()
                break
        # Cancel goal if timeout is reached
        if timeout_sec is not None and (rospy.Time.now() - start_time > rospy.Duration(timeout_sec)):
            rospy.logwarn("Timeout reached for goal: x=%.2f, y=%.2f. Cancelling goal.", x, y)
            client.cancel_goal()
            break
        rate.sleep()

def snake_pattern_horizontal(x_min, y_min, x_max, y_max, num_cols):
    """
    Generate a horizontal snake pattern list of waypoints within the area defined by
    the bottom-left corner (x_min, y_min) and the top-right corner (x_max, y_max).
    
    Traversal rules:
      - Sample num_cols points evenly along the x-axis from right (x_max) to left (x_min);
      - For each column:
           * Even-indexed columns (starting from 0): traverse from bottom to top (y_min to y_max);
           * Odd-indexed columns: traverse from top to bottom (y_max to y_min).
           
    The start point is the bottom-right corner (x_max, y_min), and the end point is the top-left corner (x_min, y_max).
    """
    waypoints = []
    # Generate evenly spaced points along the x-axis from right to left
    x_list = [x_max - i * (x_max - x_min) / float(num_cols - 1) for i in range(num_cols)]
    
    for idx, x in enumerate(x_list):
        if idx % 2 == 0:
            waypoints.append((x, y_min))
            waypoints.append((x, y_max))
        else:
            waypoints.append((x, y_max))
            waypoints.append((x, y_min))
    return waypoints

def main():
    rospy.init_node("snake_pattern_nav_horizontal")
    
    # Subscribe to amcl_pose to get the robot's current position
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

    # Create an action client for move_base
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # First navigate to the initial point (21.8, -21.3) with no timeout
    initial_x = 21.8
    initial_y = -21.3
    initial_yaw = 0.0  # Adjust orientation as needed
    rospy.loginfo("Navigating to initial point: x=%f, y=%f", initial_x, initial_y)
    send_goal(client, initial_x, initial_y, initial_yaw)
    rospy.sleep(1)

    # Define the area using coordinates, e.g., bottom-left corner (12, -22.5) and top-right corner (18.4, -2.5)
    x_min = 12
    y_min = -22.5
    x_max = 18.4
    y_max = -2.5

    # Set the number of columns for horizontal traversal (more columns => denser path)
    num_cols = 3
    waypoints = snake_pattern_horizontal(x_min, y_min, x_max, y_max, num_cols)
    rospy.loginfo("Generated waypoints: %s", waypoints)

    # Send navigation goals sequentially, with a 40-second timeout
    for (x, y) in waypoints:
        yaw = 0.0  # Adjust robot orientation as needed
        send_goal(client, x, y, yaw, timeout_sec=40)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
