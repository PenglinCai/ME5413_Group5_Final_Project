#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from digit_recognizer import DigitRecognizer

class DigitRecognitionServiceNode:
    def __init__(self):
        # Create an instance of the digit recognizer (it subscribes to /front/image_raw internally)
        self.recognizer = DigitRecognizer()
        # Define ROS service interfaces using the Trigger service type (simple request-response with no parameters)
        self.start_srv = rospy.Service('start_recognition', Trigger, self.handle_start)
        self.stop_srv  = rospy.Service('stop_recognition', Trigger, self.handle_stop)
        rospy.loginfo("DigitRecognitionServiceNode started, waiting for service calls...")

    def handle_start(self, req):
        rospy.loginfo("Received request to start recognition.")
        self.recognizer.start_recognition()
        return TriggerResponse(success=True, message="Digit recognition started.")

    def handle_stop(self, req):
        rospy.loginfo("Received request to stop recognition.")
        best_digit = self.recognizer.stop_recognition()
        if best_digit is None:
            message = "No valid digit detected."
        else:
            message = f"Best recognized digit: {best_digit}"
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)

if __name__ == '__main__':
    rospy.init_node('digit_recognition_service_node')
    node = DigitRecognitionServiceNode()
    rospy.spin()
