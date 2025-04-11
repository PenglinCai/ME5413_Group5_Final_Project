#!/usr/bin/env python3
# Import required ROS Python packages and message/service types.
import rospy                                   # ROS Python API for node operations and logging
from std_srvs.srv import Trigger, TriggerResponse  # Import Trigger service and its response type
from digit_recognizer import DigitRecognizer     # Import our digit recognizer class
from visualization_msgs.msg import Marker         # Import Marker for RViz visualization

# Define a ROS node class that implements the digit recognition service functionality.
class DigitRecognitionServiceNode:
    def __init__(self):
        # Initialize this node with a unique name.
        rospy.init_node('digit_recognition_service_node')

        # Initialize variable to store the last recognized digit.
        # It is initially set to 0.
        self.last_digit = 0

        # Create an instance of the DigitRecognizer class.
        # The DigitRecognizer internally subscribes to the camera topic (/front/image_raw) and performs recognition.
        self.recognizer = DigitRecognizer()

        # Define and advertise the ROS services for starting and stopping digit recognition.
        # When the service 'start_recognition' is called, handle_start is triggered.
        self.start_srv = rospy.Service('start_recognition', Trigger, self.handle_start)
        # When the service 'stop_recognition' is called, handle_stop is triggered.
        self.stop_srv  = rospy.Service('stop_recognition', Trigger, self.handle_stop)

        # Create a ROS publisher for a Marker message to visualize the recognized digit.
        # The topic name is 'digit_recognition_marker' (without a leading slash).
        # The marker is latched so that new subscribers (e.g. in RViz) immediately get the latest message.
        self.marker_pub = rospy.Publisher(
            'digit_recognition_marker',
            Marker,
            queue_size=1,
            latch=True
        )

        # Before starting, publish a marker with the initial digit (0).
        rospy.sleep(0.5)  # Sleep a short moment to allow publishers to register
        self.publish_marker(self.last_digit)

        # Set up a timer that will repeatedly republish the current digit every second.
        # This ensures that RViz always displays the digit.
        rospy.Timer(rospy.Duration(1.0), self.timer_publish)

        # Log that the service node has started successfully.
        rospy.loginfo("DigitRecognitionServiceNode 启动完毕，当前显示 = 0，等待服务调用…")

    def handle_start(self, req):
        """
        Callback for the 'start_recognition' service.
        This function resets the displayed digit to 0 and starts the digit recognition process.
        """
        rospy.loginfo("[Service] start_recognition called, resetting display to 0")
        # Start the recognition thread in the DigitRecognizer.
        self.recognizer.start_recognition()
        # Reset the last recognized digit to 0.
        self.last_digit = 0
        # Publish a marker with the digit 0.
        self.publish_marker(self.last_digit)
        # Return a successful TriggerResponse with an informative message.
        return TriggerResponse(success=True, message="数字识别已启动，显示已重置为 0。")

    def handle_stop(self, req):
        """
        Callback for the 'stop_recognition' service.
        Stops the digit recognition process and retrieves the best recognized digit.
        """
        rospy.loginfo("[Service] stop_recognition called, stopping recognizer")
        # Stop the recognition thread and get the best digit from the recognizer.
        best = self.recognizer.stop_recognition()
        if best is None:
            # If no valid digit is recognized, keep the display as 0.
            rospy.loginfo("[Service] 无有效识别，保持 0 显示")
            self.last_digit = 0
            msg = "未检测到有效数字，显示为 0。"
        else:
            # Otherwise, update last_digit to the recognized best digit.
            self.last_digit = best
            msg = f"最佳识别结果为：{best}"
            rospy.loginfo(f"[Service] {msg}")
        # Publish the updated marker with the recognized digit.
        self.publish_marker(self.last_digit)
        # Return the TriggerResponse indicating a successful operation with an appropriate message.
        return TriggerResponse(success=True, message=msg)

    def timer_publish(self, event):
        """
        Timer callback function that republish the current recognized digit as a Marker.
        This function is called at a regular interval (every second) to ensure the display remains updated.
        """
        rospy.logdebug(f"[Timer] 重发当前显示: {self.last_digit}")
        self.publish_marker(self.last_digit)

    def publish_marker(self, digit):
        """
        Create and publish a Marker message to visualize the recognized digit in RViz.
        The marker is defined as text displayed in the 'map' frame.
        """
        m = Marker()  # Create a new Marker message instance.
        
        # Set the reference frame to "map" (should match the Fixed Frame in RViz).
        m.header.frame_id = 'map'
        # Timestamp the marker with the current ROS time.
        m.header.stamp = rospy.Time.now()
        # Set a namespace to group markers (useful for later identification in RViz).
        m.ns = 'digit_recognition'
        # Assign an ID to the marker (only one marker is used, so ID is 0).
        m.id = 0
        # Specify the marker type as TEXT_VIEW_FACING, which displays text that always faces the viewer.
        m.type = Marker.TEXT_VIEW_FACING
        # Specify that this marker is to be added (or updated).
        m.action = Marker.ADD

        # Define the marker's position in the map.
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 1.5  # Raise the marker 1.5 meters above the ground.
        # Set a neutral orientation (no rotation; text will always face the viewer).
        m.pose.orientation.w = 1.0

        # Define the scale (size) of the text.
        m.scale.x = 1.0  # Horizontal size scaling
        m.scale.y = 1.0  # Vertical size scaling
        m.scale.z = 1.5  # Font size (height) of the text
        
        # Set the marker color to white (RGBA values).
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0  # Fully opaque
        
        # Set the lifetime of the marker.
        m.lifetime = rospy.Duration(5.0)  # Marker will persist for 5 seconds

        # Set the text of the marker to the string representation of the digit.
        m.text = str(digit)

        # Publish the marker on the designated topic.
        self.marker_pub.publish(m)
        rospy.logdebug(f"[Publish] Marker text='{m.text}' frame='{m.header.frame_id}'")

# Main entry point of the node.
if __name__ == '__main__':
    try:
        # Create an instance of DigitRecognitionServiceNode.
        DigitRecognitionServiceNode()
        # Keep the node running until a shutdown signal is received.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
