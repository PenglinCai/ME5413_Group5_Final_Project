#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
import threading
import time
import os
from sensor_msgs.msg import Image

class DigitRecognizer:
    def __init__(self, templates_dir=None):
        """
        Initialize the digit recognizer:
         - Load template images (grayscale), file names are 0.png to 9.png in the 'templates' folder
         - Initialize cv_bridge instance and subscribe to the /front/image_raw topic to receive images
         - Initialize internal variables: store latest frame, best digit and matching score
        """
        # Load digit templates
        self.templates = {}
        if templates_dir is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            templates_dir = os.path.join(script_dir, "templates")
        for digit in range(10):
            template_path = os.path.join(templates_dir, f"{digit}.png")
            template_img = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
            if template_img is not None:
                self.templates[str(digit)] = template_img
            else:
                rospy.logwarn("Template image not found: {}".format(template_path))
        if not self.templates:
            raise Exception("No templates loaded. Please check the template path!")

        # Create cv_bridge instance to convert ROS Image messages to OpenCV images
        self.bridge = cv_bridge.CvBridge()
        self.current_frame = None  # Store the latest received image
        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback)

        self.best_digit = None    # Current best matching digit
        self.best_score = -1      # Current best matching score
        self.stop_event = threading.Event()  # Thread stop flag
        self.thread = None

    def image_callback(self, img_msg):
        """
        Image callback function: Convert ROS Image message to OpenCV format using cv_bridge
        and store it in self.current_frame
        """
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("cv_bridge conversion error: %s", e)

    def recognition_loop(self):
        """
        Recognition loop (running in a separate thread):
         - Periodically check if a new image is available
         - If available, perform template matching via recognize_digit()
         - If the result is better than current best, update best result
         - Loop continues until stop signal is received
        """
        rate = rospy.Rate(10)  # Loop at 10Hz
        while not self.stop_event.is_set():
            if self.current_frame is None:
                rate.sleep()
                continue

            frame = self.current_frame.copy()  # Copy to avoid thread conflicts
            result = self.recognize_digit(frame)
            if result is not None:
                digit, score, pixel_offset = result
                if score > self.best_score:
                    self.best_score = score
                    self.best_digit = digit
                    rospy.loginfo("New best match: digit %s, score %.2f, offset %.2f", digit, score, pixel_offset)
            rate.sleep()

    def start_recognition(self):
        """
        Start the recognition thread:
         - Clear previous result and start a new thread for continuous digit recognition
        """
        self.stop_event.clear()
        self.best_digit = None
        self.best_score = -1
        self.thread = threading.Thread(target=self.recognition_loop)
        self.thread.start()
        rospy.loginfo("Digit recognition started...")

    def stop_recognition(self):
        """
        Stop the recognition thread and return the best recognized digit
        """
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
        rospy.loginfo("Digit recognition stopped.")
        return self.best_digit

    def recognize_digit(self, cv_image):
        """
        Perform template matching using OpenCV to recognize digits:
         1. Convert input BGR image to grayscale and apply histogram equalization
         2. Match each loaded template at multiple scales (0.4 to 1.6)
         3. Keep the best match with the highest score (if above threshold)
         4. Calculate horizontal offset between matched region center and image center
         Returns: (digit, best_score, pixel_offset); returns None if no valid match
        """
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        best_score = -1
        best_digit = None
        best_loc = None
        best_template_shape = None

        for digit, template in self.templates.items():
            template_gray = template.copy()
            template_gray = cv2.equalizeHist(template_gray)
            for scale in np.linspace(0.4, 1.6, 21):
                try:
                    resized_template = cv2.resize(template_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                except Exception:
                    continue
                tH, tW = resized_template.shape[:2]
                if gray.shape[0] < tH or gray.shape[1] < tW:
                    continue
                res = cv2.matchTemplate(gray, resized_template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(res)
                if max_val > best_score:
                    best_score = max_val
                    best_digit = digit
                    best_loc = max_loc
                    best_template_shape = resized_template.shape

        threshold = 0.7
        if best_score < threshold or best_digit is None:
            return None

        tH, tW = best_template_shape
        match_center_x = best_loc[0] + tW / 2
        image_center_x = gray.shape[1] / 2
        pixel_offset = match_center_x - image_center_x
        return best_digit, best_score, pixel_offset

# # For standalone testing
# if __name__ == "__main__":
#     rospy.init_node("digit_recognizer_test")
#     recognizer = DigitRecognizer()
#     recognizer.start_recognition()
#     input("Press Enter to stop recognition and display result...")
#     result = recognizer.stop_recognition()
#     rospy.loginfo("Final recognized digit: %s", result)
