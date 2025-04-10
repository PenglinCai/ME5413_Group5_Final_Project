#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import math
import re
import subprocess  # Used for calling system commands
from std_srvs.srv import Trigger  # Used for calling visual recognition service

# Define the topic name for subscribing to matrix messages.
MATRIX_TOPIC = "/found_blocks_info"

# Global variable to store the current position provided by AMCL.
current_pose = None

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def send_goal(client, x, y, yaw, threshold=0.4, orientation_threshold=0.35, timeout_sec=40):
    """
    Send a navigation goal and continuously monitor the difference between the robot's current position and the target.
    Cancel the goal when the preset thresholds are met or if a timeout occurs.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Convert yaw to quaternion.
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo("Sending goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)
    client.send_goal(goal)
    
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_pose is not None:
            dx = current_pose.position.x - x
            dy = current_pose.position.y - y
            distance = math.sqrt(dx * dx + dy * dy)
            q = current_pose.orientation
            current_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            angle_diff = abs(math.atan2(math.sin(current_yaw - yaw), math.cos(current_yaw - yaw)))
            rospy.loginfo("Distance to goal: %.2f, Orientation diff: %.2f", distance, angle_diff)
            if distance <= threshold and angle_diff <= orientation_threshold:
                rospy.loginfo("Reached goal: distance=%.2f, angle diff=%.2f", distance, angle_diff)
                client.cancel_goal()
                break
        if timeout_sec is not None and (rospy.Time.now() - start_time > rospy.Duration(timeout_sec)):
            rospy.logwarn("Timeout reached for goal: x=%.2f, y=%.2f. Cancelling goal.", x, y)
            client.cancel_goal()
            break
        rate.sleep()

def compute_tsp_order(start, points):
    """
    Calculate the optimal TSP path order using the Held-Karp algorithm:
      Starting from the initial position, compute the shortest path that visits all points.
    Parameters:
      start: Current robot position (x, y)
      points: Each point in the format (pid, x, y, info)
    Return:
      A list of points arranged in the optimal visiting order.
    """
    n = len(points)
    if n == 0:
        return []
    dp = {}
    parent = {}
    for i in range(n):
        dp[(1 << i, i)] = math.sqrt((start[0] - points[i][1])**2 + (start[1] - points[i][2])**2)
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
    final_mask = (1 << n) - 1
    best_cost = float('inf')
    best_end = None
    for i in range(n):
        if dp[(final_mask, i)] < best_cost:
            best_cost = dp[(final_mask, i)]
            best_end = i
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
    ordered_points = [points[i] for i in path]
    return ordered_points

class PointNavigator:
    def __init__(self):
        rospy.init_node('matrix_subscriber')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

        # Subscribe to amcl_pose to get the current robot position provided by AMCL.
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
        # Subscribe to the /found_blocks_info topic.
        self.sub = rospy.Subscriber(MATRIX_TOPIC, String, self.callback)

        # Publisher for the recognized digits array, using latch=True to ensure the message persists.
        self.digit_pub = rospy.Publisher('/recognized_digits', Int32MultiArray, queue_size=10, latch=True)
        self.recognized_digits = []  # Store the digit recognized at each navigation point.

    def callback(self, msg):
        # Process the data only once; if continuous processing is required, remove the unregister line below.
        self.sub.unregister()

        data_str = msg.data
        rospy.loginfo("Received /found_blocks_info: %s", data_str)

        # Parse the data (formatted as "(cube_order,x,y,number)")
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

        # Terminate the findcube.py process running from the command line.
        # rospy.loginfo("Ending findcube.py process using pkill...")
        # subprocess.call(["pkill", "-f", "findcube.py"])

        # Wait to obtain the current robot position.
        while current_pose is None and not rospy.is_shutdown():
            rospy.logwarn("Waiting for current robot position...")
            rospy.sleep(0.5)
        start = (current_pose.position.x, current_pose.position.y)
        rospy.loginfo("Current robot position: (%.2f, %.2f)", start[0], start[1])

        # Reorder the navigation points.
        ordered_points = compute_tsp_order(start, points)
        rospy.loginfo("Held-Karp ordered points: %s", ordered_points)

        for point in ordered_points:
            pid, x, y, info = point
            x_target = x + 1.7
            rospy.loginfo("Navigating to point %d: (%.2f, %.2f), info=%d", pid, x_target, y, info)
            yaw = 3.1416  # Adjust the target orientation as needed.
            send_goal(self.client, x_target, y, yaw, threshold=0.4, orientation_threshold=0.35, timeout_sec=50)
            
            rospy.loginfo("Arrived at point %d. Waiting 2 seconds before digit recognition.", pid)
            rospy.sleep(2)
            
            # Call the visual recognition function: first call start_recognition, then after 2 seconds call stop_recognition to obtain the digit.
            try:
                rospy.wait_for_service('start_recognition', timeout=3)
                start_srv = rospy.ServiceProxy('start_recognition', Trigger)
                start_resp = start_srv()
                rospy.loginfo("Digit recognition started: %s", start_resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call start_recognition: %s", e)
            
            rospy.sleep(2)  # Allow the visual recognition to run for a period of time.

            try:
                rospy.wait_for_service('stop_recognition', timeout=5)
                stop_srv = rospy.ServiceProxy('stop_recognition', Trigger)
                stop_resp = stop_srv()
                rospy.loginfo("Digit recognition stopped: %s", stop_resp.message)
                recognized_digit = -1
                # Extract the digit from the returned string using regex, expected format is "Best recognition result: X"
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
            
            # Publish the updated digit array; using latch ensures the message persists.
            array_msg = Int32MultiArray(data=self.recognized_digits)
            self.digit_pub.publish(array_msg)
            rospy.loginfo("Published recognized digits: %s", self.recognized_digits)
            
            rospy.sleep(1)
        
        # After completing all tasks, write the results to a file so that they are preserved after program exit.
        output_file = "recognized_digits.txt"
        try:
            with open(output_file, "w") as f:
                f.write(",".join(map(str, self.recognized_digits)))
            rospy.loginfo("Recognized digits data written to %s", output_file)
        except Exception as e:
            rospy.logerr("Failed to write data to file: %s", e)
        
        # Task completed, shutting down the node.
        rospy.loginfo("All points visited. Shutting down.")
        rospy.signal_shutdown("Done")

if __name__ == '__main__':
    try:
        PointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
