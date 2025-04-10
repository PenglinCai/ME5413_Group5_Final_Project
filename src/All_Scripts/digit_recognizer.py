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
        初始化数字识别器：
         - 加载模板图片（灰度图），模板文件名为 0.png ... 9.png，存放在 templates 文件夹中
         - 初始化 cv_bridge 实例以及订阅 /front/image_raw 话题获取图像
         - 初始化内部变量：存储最新图像、最佳识别数字与匹配分数等
        """
        # 加载模板
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
                rospy.logwarn("模板图片未找到：{}".format(template_path))
        if not self.templates:
            raise Exception("未加载任何模板，请检查模板路径！")
        
        # cv_bridge 实例：用于将 ROS Image 消息转换为 OpenCV 图像
        self.bridge = cv_bridge.CvBridge()
        # 存储最新接收到的图像
        self.current_frame = None
        # 订阅摄像头图像话题 /front/image_raw
        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback)
        
        self.best_digit = None    # 当前最佳匹配数字
        self.best_score = -1      # 当前最佳匹配分数
        self.stop_event = threading.Event()  # 用于控制识别线程的停止
        self.thread = None

    def image_callback(self, img_msg):
        """
        图像回调函数：利用 cv_bridge 将 ROS Image 消息转换为 OpenCV 格式，并保存到 self.current_frame
        """
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("cv_bridge 转换错误：%s", e)

    def recognition_loop(self):
        """
        识别线程循环：
         - 每隔一小段时间检查最新图像（由回调更新）
         - 如果有图像，则调用 recognize_digit() 进行模板匹配
         - 若匹配分数高于当前记录，则更新最佳识别结果
         - 循环直到收到停止信号
        """
        rate = rospy.Rate(10)  # 以 10Hz 循环
        while not self.stop_event.is_set():
            if self.current_frame is None:
                rate.sleep()
                continue

            # 为避免线程安全问题，对 current_frame 做拷贝
            frame = self.current_frame.copy()
            result = self.recognize_digit(frame)
            if result is not None:
                digit, score, pixel_offset = result
                if score > self.best_score:
                    self.best_score = score
                    self.best_digit = digit
                    rospy.loginfo("更新最佳匹配：数字 %s，分数 %.2f，偏移 %.2f", digit, score, pixel_offset)
            rate.sleep()

    def start_recognition(self):
        """
        启动识别线程：
         - 清除旧结果，启动新的识别线程，开始持续检测最新图像
        """
        self.stop_event.clear()
        self.best_digit = None
        self.best_score = -1
        self.thread = threading.Thread(target=self.recognition_loop)
        self.thread.start()
        rospy.loginfo("数字识别启动……")

    def stop_recognition(self):
        """
        停止识别线程，并返回当前最佳识别的数字
        """
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
        rospy.loginfo("数字识别停止。")
        return self.best_digit

    def recognize_digit(self, cv_image):
        """
        利用 OpenCV 模板匹配识别数字：
         1. 将输入的 BGR 图像转换为灰度并均衡化直方图
         2. 遍历加载的每个模板，并在多个尺度上进行匹配
         3. 选出匹配分数最高的模板（若超过阈值，则认为检测成功）
         4. 计算匹配区域的中心与图像中心的水平偏移
         返回：(digit, best_score, pixel_offset)；若未匹配到，则返回 None
        """
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        best_score = -1
        best_digit = None
        best_loc = None
        best_template_shape = None

        # 对每个模板在 0.4~1.6 的多个尺度上进行匹配（共 21 个尺度）
        for digit, template in self.templates.items():
            template_gray = template.copy()
            template_gray = cv2.equalizeHist(template_gray)
            for scale in np.linspace(1.2, 2.4, 41):
                try:
                    resized_template = cv2.resize(template_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                except Exception as e:
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
        # 设置匹配阈值
        threshold = 0.7
        if best_score < threshold or best_digit is None:
            return None

        tH, tW = best_template_shape
        match_center_x = best_loc[0] + tW / 2
        image_center_x = gray.shape[1] / 2
        pixel_offset = match_center_x - image_center_x
        return best_digit, best_score, pixel_offset

# # 仅用于单独测试模块
# if __name__ == "__main__":
#     rospy.init_node("digit_recognizer_test")
#     recognizer = DigitRecognizer()
#     recognizer.start_recognition()
#     input("按 Enter 停止识别并输出结果……")
#     result = recognizer.stop_recognition()
#     rospy.loginfo("最终识别数字：%s", result)
