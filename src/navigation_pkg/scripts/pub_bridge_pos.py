#!/usr/bin/env python
import rospy
import re
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def bridge_info_callback(msg):
    """
    Callback function: Parses the message published by the findbridge node.
    Example message: "Best Bridge: Center=(6.85,-14.85) Start=(8.85,-14.85)"
    It uses the second number in the Center as the y-coordinate, with the x-coordinate fixed at 9,
    then constructs a navigation goal message and publishes it to the "/navigation_goal" topic.
    """
    # Use a regular expression to extract the coordinates from the "Center" field.
    match = re.search(r"Center=\(\s*([-+]?\d*\.\d+|\d+)\s*,\s*([-+]?\d*\.\d+|\d+)\s*\)", msg.data)
    if match:
        try:
            # Extract and convert to float; note that only the second number (y-coordinate) is needed.
            center_x = float(match.group(1))
            center_y = float(match.group(2))
            # Set the navigation goal: x is fixed as 9, y is the extracted value, and z is set to 0.
            nav_goal = PoseStamped()
            nav_goal.header.stamp = rospy.Time.now()
            nav_goal.header.frame_id = "map"  # Set according to the actual coordinate frame.
            nav_goal.pose.position.x = 7.0
            nav_goal.pose.position.y = center_y
            nav_goal.pose.position.z = 0.0
            # Set a unit quaternion (no rotation).
            nav_goal.pose.orientation.x = 0.0
            nav_goal.pose.orientation.y = 0.0
            nav_goal.pose.orientation.z = 1.0
            nav_goal.pose.orientation.w = 0.0

            # Publish the navigation goal message.
            navigation_pub.publish(nav_goal)
            rospy.loginfo("Published navigation goal: x=9, y=%.2f", center_y)
        except ValueError:
            rospy.logwarn("Error converting coordinates, original data: %s", msg.data)
    else:
        rospy.logwarn("Unable to parse coordinates from message: %s", msg.data)

def navigation_goal_listener():
    rospy.init_node('navigation_goal_publisher', anonymous=True)
    # Subscribe to the bridge information published by the findbridge node; assume the topic is "/bridge_info"
    rospy.Subscriber("/bridge_info", String, bridge_info_callback)
    # Publisher for navigation goal messages; message type is geometry_msgs/PoseStamped.
    global navigation_pub
    navigation_pub = rospy.Publisher("/navigation_goal", PoseStamped, queue_size=10)
    rospy.loginfo("Navigation goal publisher is running...")
    rospy.spin()

if __name__ == '__main__':
    navigation_goal_listener()
