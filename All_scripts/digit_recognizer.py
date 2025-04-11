#!/usr/bin/env python3                      # Specify the interpreter for the script as Python 3
import rospy                                # Import ROS Python API for node initialization and logging
import cv2                                  # Import OpenCV for image processing functions
import cv_bridge                            # Import cv_bridge to convert ROS Image messages to OpenCV images
import numpy as np                          # Import numpy for numerical operations (e.g., arrays, linspace)
import threading                            # Import threading to run recognition in a separate thread
import time                                 # Import time module for sleep and timing functions
import os                                   # Import os module for file path operations
from sensor_msgs.msg import Image           # Import Image message type from sensor_msgs package

class DigitRecognizer:
    def __init__(self, templates_dir=None):
        """
        Initialize the digit recognizer:
         - Load template images (in grayscale) with filenames 0.png to 9.png from a templates folder.
         - Initialize a cv_bridge instance and subscribe to the /front/image_raw topic to receive images.
         - Initialize internal variables to store the latest image, the best recognized digit, and its matching score.
        """
        # Initialize an empty dictionary to store template images keyed by digit (as string)
        self.templates = {}
        # If no template directory is provided, determine the default 'templates' directory relative to the script
        if templates_dir is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))  # Get directory of current script
            templates_dir = os.path.join(script_dir, "templates")       # Append "templates" folder to script directory
        # Loop through digits 0 to 9 to load each template image
        for digit in range(10):
            # Construct the file path for the template image (e.g., "templates/0.png")
            template_path = os.path.join(templates_dir, f"{digit}.png")
            # Read the template image in grayscale mode
            template_img = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
            # If the image is successfully loaded, store it in the templates dictionary using the digit as key
            if template_img is not None:
                self.templates[str(digit)] = template_img
            else:
                # Log a warning if the template image could not be found or loaded
                rospy.logwarn("模板图片未找到：{}".format(template_path))
        # If no templates were loaded, raise an exception to halt the node
        if not self.templates:
            raise Exception("未加载任何模板，请检查模板路径！")
        
        # Create a cv_bridge instance to convert between ROS Image messages and OpenCV images
        self.bridge = cv_bridge.CvBridge()
        # Variable to store the latest received image frame from the camera
        self.current_frame = None
        # Subscribe to the camera topic '/front/image_raw' to get new image messages and call image_callback when received
        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback)
        
        # Variable to store the best recognized digit (based on matching score)
        self.best_digit = None
        # Variable to store the highest matching score found (initialized to -1 to indicate no valid match yet)
        self.best_score = -1
        # Create a threading Event to control when the recognition thread should stop
        self.stop_event = threading.Event()
        # Initialize the thread variable; it will be used later to run the recognition loop
        self.thread = None

    def image_callback(self, img_msg):
        """
        Callback function for processing images:
         - Convert the incoming ROS Image message to an OpenCV BGR image using cv_bridge.
         - Save the converted image to self.current_frame for later processing.
        """
        try:
            # Convert the ROS Image message to an OpenCV image with "bgr8" encoding format
            self.current_frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            # Log an error if the image conversion fails
            rospy.logerr("cv_bridge 转换错误：%s", e)

    def recognition_loop(self):
        """
        Recognition thread loop:
         - Check the latest image (updated via the image callback) at regular intervals.
         - If an image is available, call recognize_digit() to perform template matching.
         - If a matching score higher than the current best is found, update the best match result.
         - Continue looping until a stop signal is received.
        """
        rate = rospy.Rate(10)  # Set loop rate to 10 Hz
        while not self.stop_event.is_set():  # Loop until stop_event flag is set
            if self.current_frame is None:
                # If no image has been received yet, sleep and continue checking later
                rate.sleep()
                continue

            # Make a copy of the current frame to avoid issues with concurrent access in the callback
            frame = self.current_frame.copy()
            # Perform digit recognition on the copied frame using template matching
            result = self.recognize_digit(frame)
            if result is not None:
                # Unpack the recognition result: digit, matching score, and horizontal offset
                digit, score, pixel_offset = result
                # If the current matching score is better than the previously stored best score, update the best result
                if score > self.best_score:
                    self.best_score = score
                    self.best_digit = digit
                    rospy.loginfo("更新最佳匹配：数字 %s，分数 %.2f，偏移 %.2f", digit, score, pixel_offset)
            # Sleep until the next iteration of the loop
            rate.sleep()

    def start_recognition(self):
        """
        Start the recognition thread:
         - Clear previous recognition results.
         - Start a new thread running the recognition_loop to continuously process incoming images.
        """
        self.stop_event.clear()       # Clear the event flag to allow the loop to run
        self.best_digit = None        # Reset the best recognized digit
        self.best_score = -1          # Reset the best matching score
        # Create a new thread targeting the recognition_loop function
        self.thread = threading.Thread(target=self.recognition_loop)
        self.thread.start()           # Start the recognition thread
        rospy.loginfo("数字识别启动……")  # Log that digit recognition has started

    def stop_recognition(self):
        """
        Stop the recognition thread:
         - Signal the recognition_loop to stop and wait for the thread to finish.
         - Return the best recognized digit found so far.
        """
        self.stop_event.set()         # Set the event flag, causing recognition_loop to exit
        if self.thread is not None:
            self.thread.join()        # Wait for the recognition thread to finish execution
        rospy.loginfo("数字识别停止。")    # Log that digit recognition has been stopped
        return self.best_digit        # Return the best recognized digit

    def recognize_digit(self, cv_image):
        """
        Recognize a digit using OpenCV template matching:
         1. Convert the input BGR image to a grayscale image and equalize its histogram.
         2. For each loaded template, perform matching over multiple scales.
         3. Select the template with the highest matching score (if it exceeds a specified threshold).
         4. Calculate the horizontal offset between the matching region's center and the image center.
         Return a tuple (digit, best_score, pixel_offset); if no match is found, return None.
        """
        # Convert the BGR image to grayscale for processing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Apply histogram equalization to improve contrast
        gray = cv2.equalizeHist(gray)

        best_score = -1              # Initialize the best score to a very low value
        best_digit = None            # Initialize best_digit as None
        best_loc = None              # Variable to store location of the best match
        best_template_shape = None   # Variable to store the shape of the best matching template

        # Iterate over each loaded template for digits 0-9; perform matching at multiple scales
        # The comment indicates scales between 1.2 and 2.4 (41 values) even though the docstring mentions 0.4~1.6; adjust as needed.
        for digit, template in self.templates.items():
            # Make a copy of the template image and equalize its histogram
            template_gray = template.copy()
            template_gray = cv2.equalizeHist(template_gray)
            # Iterate over a range of scales using np.linspace to generate 41 scale factors between 1.2 and 2.4
            for scale in np.linspace(1.2, 2.4, 41):
                try:
                    # Resize the template image according to the current scale factor using area interpolation
                    resized_template = cv2.resize(template_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                except Exception as e:
                    # If resizing fails (e.g., due to size issues), continue with the next scale factor
                    continue
                # Get the dimensions (height and width) of the resized template
                tH, tW = resized_template.shape[:2]
                # Skip this scale if the resized template is larger than the input image
                if gray.shape[0] < tH or gray.shape[1] < tW:
                    continue
                # Perform template matching using the normalized cross-correlation method
                res = cv2.matchTemplate(gray, resized_template, cv2.TM_CCOEFF_NORMED)
                # Retrieve the maximum matching score and its location within the result map
                _, max_val, _, max_loc = cv2.minMaxLoc(res)
                # If the obtained matching score is higher than the best recorded so far, update the best match variables
                if max_val > best_score:
                    best_score = max_val
                    best_digit = digit
                    best_loc = max_loc
                    best_template_shape = resized_template.shape
        # Define a threshold for matching; if the best score is lower than this, consider recognition unsuccessful
        threshold = 0.7
        if best_score < threshold or best_digit is None:
            return None

        # Calculate the center x-coordinate of the matching region based on the best location and template width
        tH, tW = best_template_shape
        match_center_x = best_loc[0] + tW / 2
        # Calculate the center x-coordinate of the entire image
        image_center_x = gray.shape[1] / 2
        # Determine the horizontal offset between the match center and image center
        pixel_offset = match_center_x - image_center_x
        # Return the recognized digit, the best matching score, and the computed horizontal offset
        return best_digit, best_score, pixel_offset

# The following code is commented out. It can be used to test the module independently.
# if __name__ == "__main__":
#     rospy.init_node("digit_recognizer_test")
#     recognizer = DigitRecognizer()
#     recognizer.start_recognition()
#     input("按 Enter 停止识别并输出结果……")
#     result = recognizer.stop_recognition()
#     rospy.loginfo("最终识别数字：%s", result)
