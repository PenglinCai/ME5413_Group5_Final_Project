#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import math
import re
import subprocess  # For calling external system commands if necessary
from std_srvs.srv import Trigger  # Service type used for starting/stopping visual recognition

# Topic for receiving recognized block information (as a matrix string)
MATRIX_TOPIC = "/found_blocks_info"

# Global variable to hold the most recent robot pose provided by AMCL
current_pose = None

def amcl_pose_callback(msg):
    """
    Callback function for the "amcl_pose" topic.
    Updates the global current_pose variable with the latest pose data
    obtained from AMCL (Adaptive Monte Carlo Localization).
    """
    global current_pose
    current_pose = msg.pose.pose

def send_goal(client, x, y, yaw, threshold=0.3, orientation_threshold=0.20, timeout_sec=40):
    """
    Sends a navigation goal to the move_base action server and monitors the robot's progress.
    
    This function creates a MoveBaseGoal with the specified target position (x, y)
    and target orientation (yaw). It converts yaw (in radians) to the corresponding
    quaternion, then sends the goal using the provided action client. The function
    continuously monitors the distance and angular difference between the current robot 
    pose and the target. If both the distance and orientation error fall below the 
    specified thresholds or a timeout is reached, the function cancels the goal.
    
    Parameters:
      client: The action client connected to move_base.
      x, y: Target position coordinates in the "map" frame.
      yaw: Target orientation (in radians) about the Z-axis.
      threshold: Maximum allowed positional error (in meters) to consider the goal reached.
      orientation_threshold: Maximum allowed angular error (in radians) for goal completion.
      timeout_sec: Maximum time to wait for reaching the goal before cancelling.
    """
    # Construct the goal message and assign header information
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Convert yaw angle to quaternion representation for orientation
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo("Sending goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)
    client.send_goal(goal)
    
    # Monitor the robot's progress towards the goal until thresholds or timeout occurs
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)  # Monitor at 10 Hz
    while not rospy.is_shutdown():
        if current_pose is not None:
            # Compute Euclidean distance between current position and the goal position
            dx = current_pose.position.x - x
            dy = current_pose.position.y - y
            distance = math.sqrt(dx * dx + dy * dy)
            
            # Convert current orientation from quaternion to euler (yaw angle)
            q = current_pose.orientation
            current_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            # Compute minimal angular difference between current_yaw and desired yaw
            angle_diff = abs(math.atan2(math.sin(current_yaw - yaw), math.cos(current_yaw - yaw)))
            
            rospy.loginfo("Distance to goal: %.2f, Orientation diff: %.2f", distance, angle_diff)
            
            # Check if both positional and orientation errors are within acceptable limits
            if distance <= threshold and angle_diff <= orientation_threshold:
                rospy.loginfo("Reached goal: distance=%.2f, angle diff=%.2f", distance, angle_diff)
                client.cancel_goal()
                break
        
        # Cancel the goal if waiting exceeds the timeout duration
        if timeout_sec is not None and (rospy.Time.now() - start_time > rospy.Duration(timeout_sec)):
            rospy.logwarn("Timeout reached for goal: x=%.2f, y=%.2f. Cancelling goal.", x, y)
            client.cancel_goal()
            break
        
        rate.sleep()

def compute_tsp_order(start, points):
    """
    Compute an approximately optimal TSP (Traveling Salesman Problem) order using the Held-Karp algorithm.
    
    Given the current robot position (start) and a list of target points (each represented as a tuple: (pid, x, y, info)),
    this function computes the order in which to visit all points such that the total travel distance is minimized.
    It returns a list of points arranged in the computed visiting order.
    
    Parameters:
      start: A tuple (x, y) representing the current position.
      points: A list of tuples (pid, x, y, info), where:
              - pid: An identifier for the point.
              - x, y: Coordinates of the point.
              - info: Additional information (e.g., digit recognized or other data).
    
    Returns:
      A list of points arranged in the optimal (or near-optimal) visiting order.
    """
    n = len(points)
    if n == 0:
        return []
        
    # dp dictionary to store minimum cost for a given bitmask and ending point
    dp = {}
    parent = {}
    
    # Initialize the dp table for paths starting from 'start' to each individual point
    for i in range(n):
        dp[(1 << i, i)] = math.sqrt((start[0] - points[i][1])**2 + (start[1] - points[i][2])**2)
    
    # Dynamic programming over all subsets of points
    for mask in range(1, 1 << n):
        for i in range(n):
            if mask & (1 << i):
                prev_mask = mask ^ (1 << i)
                if prev_mask == 0:
                    continue
                best_cost = float('inf')
                best_j = None
                for j in range(n):
                    if prev_mask & (1 << j):
                        cost = dp[(prev_mask, j)] + math.sqrt((points[j][1] - points[i][1])**2 + (points[j][2] - points[i][2])**2)
                        if cost < best_cost:
                            best_cost = cost
                            best_j = j
                dp[(mask, i)] = best_cost
                parent[(mask, i)] = best_j
    
    # Find the best ending point for the complete subset of points
    final_mask = (1 << n) - 1
    best_cost = float('inf')
    best_end = None
    for i in range(n):
        if dp[(final_mask, i)] < best_cost:
            best_cost = dp[(final_mask, i)]
            best_end = i
    
    # Reconstruct the path using the parent pointers
    mask = final_mask
    current = best_end
    path = [current]
    while mask:
        if (mask, current) in parent:
            prev = parent[(mask, current)]
        else:
            break
        path.append(prev)
        mask = mask ^ (1 << current)
        current = prev
    path.reverse()
    
    # Return the ordered list of points based on the computed TSP path
    ordered_points = [points[i] for i in path]
    return ordered_points

class PointNavigator:
    def __init__(self):
        """
        Initialize the PointNavigator node, set up subscribers, publishers, and the move_base action client.
        
        The node subscribes to AMCL pose updates and block information from MATRIX_TOPIC.
        It publishes recognized digit results on a latched topic so that the result is persistent.
        """
        rospy.init_node('matrix_subscriber')
        
        # Set up the action client for navigation to send move_base goals.
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")
        
        # Subscribe to the "amcl_pose" topic for current robot localization.
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
        # Subscribe to the block information topic which contains detected block positions.
        self.sub = rospy.Subscriber(MATRIX_TOPIC, String, self.callback)
        
        # Publisher for recognized digit array, latched to retain the latest message.
        self.digit_pub = rospy.Publisher('/recognized_digits', Int32MultiArray, queue_size=10, latch=True)
        self.recognized_digits = []  # List to store digits recognized at each navigation point.

    def callback(self, msg):
        """
        Callback function for processing incoming block matrix data.
        
        This function is executed once when a message arrives from MATRIX_TOPIC.
        It parses the string message using regular expressions to extract point data,
        reorders the navigation points using the Held-Karp algorithm, navigates to each point,
        triggers visual digit recognition, and publishes the recognized digits.
        """
        # Process the data only once; remove the unsubscribe if continuous processing is required.
        self.sub.unregister()

        data_str = msg.data
        rospy.loginfo("Received /found_blocks_info: %s", data_str)

        # Parse the incoming string data formatted as "(cube_order,x,y,number)"
        pattern = r'\(\s*(\d+)\s*,\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*,\s*(\d+)\s*\)'
        matches = re.findall(pattern, data_str)
        if not matches:
            rospy.logerr("No valid matrix entries found in /found_blocks_info")
            return

        points = []
        for match in matches:
            pid = int(match[0])
            x = float(match[1])
            y = float(match[2])
            info = int(match[3])
            rospy.loginfo("Parsed point %d: (%.2f, %.2f), info=%d", pid, x, y, info)
            points.append((pid, x, y, info))

        # Wait until the current robot pose is available from AMCL localization
        while current_pose is None and not rospy.is_shutdown():
            rospy.logwarn("Waiting for current robot position...")
            rospy.sleep(0.5)
        start = (current_pose.position.x, current_pose.position.y)
        rospy.loginfo("Current robot position: (%.2f, %.2f)", start[0], start[1])

        # Compute the optimal order for visiting each point using the Held-Karp algorithm
        ordered_points = compute_tsp_order(start, points)
        rospy.loginfo("Held-Karp ordered points: %s", ordered_points)

        # Iterate through each ordered navigation point
        for point in ordered_points:
            pid, x, y, info = point
            # Apply a fixed x-offset (e.g., to adjust the target relative to the detected position)
            x_target = x + 1.7
            rospy.loginfo("Navigating to point %d: (%.2f, %.2f), info=%d", pid, x_target, y, info)
            yaw = 3.1416  # Set a default target orientation in radians (adjustable as needed)
            send_goal(self.client, x_target, y, yaw, threshold=0.4, orientation_threshold=0.35, timeout_sec=50)
            
            # Allow some time for stabilization after reaching the target
            rospy.loginfo("Arrived at point %d. Waiting 0.5 seconds before digit recognition.", pid)
            rospy.sleep(0.5)
            
            # Start the visual recognition service to capture digits at the current location
            try:
                rospy.wait_for_service('start_recognition', timeout=3)
                start_srv = rospy.ServiceProxy('start_recognition', Trigger)
                start_resp = start_srv()
                rospy.loginfo("Digit recognition started: %s", start_resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call start_recognition: %s", e)
            
            # Allow the visual recognition system to run for 1 second
            rospy.sleep(1.0)

            # Stop the visual recognition service and extract the recognized digit from the response
            try:
                rospy.wait_for_service('stop_recognition', timeout=5)
                stop_srv = rospy.ServiceProxy('stop_recognition', Trigger)
                stop_resp = stop_srv()
                rospy.loginfo("Digit recognition stopped: %s", stop_resp.message)
                recognized_digit = -1  # Default value indicating no valid digit recognized
                # Expecting response format "最佳识别结果为：X" where X is the recognized digit
                match = re.search(r'最佳识别结果为：(\d)', stop_resp.message)
                if match:
                    recognized_digit = int(match.group(1))
                else:
                    rospy.logwarn("No valid digit recognized at point %d.", pid)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call stop_recognition: %s", e)
                recognized_digit = -1

            rospy.loginfo("Recognized digit at point %d: %d", pid, recognized_digit)
            self.recognized_digits.append(recognized_digit)
            
            # Publish the updated list of recognized digits to a persistent topic (latched publisher)
            array_msg = Int32MultiArray(data=self.recognized_digits)
            self.digit_pub.publish(array_msg)
            rospy.loginfo("Published recognized digits: %s", self.recognized_digits)
            
            # Short pause before navigating to the next point
            rospy.loginfo("Buffering 0.5 seconds before next target.")
            rospy.sleep(0.5)
        
        # After processing all points, write the recognized digits data to a file for record-keeping.
        output_file = "recognized_digits.txt"
        try:
            with open(output_file, "w") as f:
                f.write(",".join(map(str, self.recognized_digits)))
            rospy.loginfo("Recognized digits data written to %s", output_file)
        except Exception as e:
            rospy.logerr("Failed to write data to file: %s", e)
        
        # All points have been processed. Log completion and shut down the node.
        rospy.loginfo("All points visited. Shutting down.")
        rospy.signal_shutdown("Done")

if __name__ == '__main__':
    try:
        PointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
