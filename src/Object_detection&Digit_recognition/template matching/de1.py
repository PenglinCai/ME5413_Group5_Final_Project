#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import os

class TargetDetector:
    def __init__(self):
        """
        初始化 TargetDetector 节点：
          - 初始化 ROS 节点
          - 建立 cv_bridge 实例用于图像转换
          - 订阅摄像头和激光雷达话题（请根据实际情况修改话题名称）
          - 初始化 TF 缓冲及监听器，用于坐标转换
          - 加载数字模板（模板图片应放在当前脚本所在目录下的 templates 文件夹中，
            命名为 0.png, 1.png, ... 9.png）
          - 初始化内部变量，用于保存最新的图像、激光数据和目标检测结果
          - 设置定时器：一个用于定时处理检测，另一个用于定时统计数字出现频率
        """
        rospy.init_node('target_detector')

        # cv_bridge 用于将 ROS 图像消息转换为 OpenCV 格式
        self.bridge = cv_bridge.CvBridge()

        # 订阅图像和激光雷达话题（根据实际情况修改话题名称）
        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)

        # TF 相关：创建 TF Buffer 和 Listener 用于坐标转换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 存储最新接收到的图像和激光数据
        self.current_frame = None
        self.current_scan = None

        # 用于保存检测到的目标信息，每个目标记录包含全局坐标 (map_x, map_y) 和识别到的数字
        self.detected_objects = []
        # 重复检测的阈值（单位：米），如果新目标与已有目标的距离小于该值，则认为重复
        self.duplicate_threshold = 0.5

        # 加载数字模板（假设模板文件在当前脚本所在目录下的 templates 文件夹中）
        self.templates = {}
        script_dir = os.path.dirname(os.path.abspath(__file__))
        templates_dir = os.path.join(script_dir, "templates")
        for digit in range(10):
            template_path = os.path.join(templates_dir, "{}.png".format(digit))
            template_img = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
            if template_img is not None:
                self.templates[str(digit)] = template_img
            else:
                rospy.logwarn("模板图片未找到: {}".format(template_path))
        if not self.templates:
            rospy.logerr("未加载任何模板，请检查模板路径！")

        # 定时器：每 1秒调用 process() 进行目标检测处理
        rospy.Timer(rospy.Duration(1), self.process)
        # 定时器：每 10 秒统计一次检测到的数字，写入 ROS 参数服务器供后续任务调用
        rospy.Timer(rospy.Duration(10), self.report_least_frequent_digit)

        rospy.loginfo("TargetDetector 节点启动。")

    def image_callback(self, img_msg):
        """
        当接收到图像消息时调用
        输入：sensor_msgs/Image 消息
        输出：更新 self.current_frame 为 OpenCV 格式的图像
        """
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("cv_bridge 转换错误: %s", e)

    def scan_callback(self, scan_msg):
        """
        当接收到激光雷达扫描消息时调用
        输入：sensor_msgs/LaserScan 消息
        输出：更新 self.current_scan 为激光数据
        """
        self.current_scan = scan_msg

    def process(self, event=None):
        """
        定时处理函数（每 0.2 秒调用一次）
        1. 通过 recognize_digit() 从图像中检测目标数字及其水平像素偏移
        2. 计算目标相对于图像中心的角度 alpha
        3. 在激光数据中选取一个角度窗口内的点，计算质心（采用中值以提高鲁棒性）作为目标的估计中心
        4. 将目标中心从 base_link 坐标系转换到 map 坐标系
        5. 检查是否为重复目标，如不是则记录，并输出检测结果到命令窗口（并为数字打上编号）
        """
        if self.current_frame is None or self.current_scan is None:
            return

        # 1. 视觉检测数字：调用 recognize_digit() 得到 (digit, pixel_offset)
        result = self.recognize_digit(self.current_frame)
        if result is None:
            return
        digit, pixel_offset = result

        # 2. 计算目标相对于图像中心的角度
        image_width = self.current_frame.shape[1]
        fov_deg = 60.0  # 预设水平视场角，请根据实际相机调整
        angle_per_pixel = fov_deg / image_width
        alpha_deg = pixel_offset * angle_per_pixel  # 单位：度
        alpha = math.radians(alpha_deg)              # 转换为弧度

        # 3. 在激光数据中选取角度窗口（例如 alpha ±5°）内的所有有效点
        window_rad = math.radians(5.0)  # 5度窗口对应弧度
        scan = self.current_scan
        valid_indices = []
        for i, range_val in enumerate(scan.ranges):
            angle_i = scan.angle_min + i * scan.angle_increment
            if (angle_i >= alpha - window_rad) and (angle_i <= alpha + window_rad):
                if not math.isinf(range_val) and not math.isnan(range_val):
                    valid_indices.append(i)
        if len(valid_indices) == 0:
            rospy.logwarn("在角度 %.2f°窗口内未找到有效激光点", alpha_deg)
            return

        # 4. 将选取的激光点转换为 (x, y) 坐标，并计算质心（采用中值降低噪声影响）
        xs = []
        ys = []
        for i in valid_indices:
            r = scan.ranges[i]
            angle_i = scan.angle_min + i * scan.angle_increment
            xs.append(r * math.cos(angle_i))
            ys.append(r * math.sin(angle_i))
        centroid_x = np.median(xs)
        centroid_y = np.median(ys)

        # 5. 构造 base_link 坐标系下的目标位置，并利用 TF 转换到 map 坐标系
        pose_base = PoseStamped()
        pose_base.header.frame_id = "base_link"
        pose_base.header.stamp = rospy.Time(0)
        pose_base.pose.position.x = centroid_x
        pose_base.pose.position.y = centroid_y
        pose_base.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(0.5))
            pose_map = tf2_geometry_msgs.do_transform_pose(pose_base, transform)
        except Exception as e:
            rospy.logwarn("TF 坐标转换失败: %s", e)
            return

        map_x = pose_map.pose.position.x
        map_y = pose_map.pose.position.y

        # 6. 重复检测判断：检查新目标是否与已有目标距离过近
        if self.is_duplicate(map_x, map_y):
            rospy.loginfo("检测到重复目标 (%.2f, %.2f)，忽略。", map_x, map_y)
            return

        # 7. 新目标记录：保存目标全局坐标及检测到的数字
        self.detected_objects.append({
            "map_x": map_x,
            "map_y": map_y,
            "digit": digit
        })

        # 计算当前数字的出现次数（作为标号）
        count_same_digit = sum(1 for obj in self.detected_objects if obj["digit"] == digit)

        # 输出检测结果到命令窗口，并标号（例如“数字1的1号”）
        msg = "当前时刻检测到数字 {}（{}号）的全局坐标为: ({:.2f}, {:.2f})".format(digit, count_same_digit, map_x, map_y)
        rospy.loginfo("检测到目标：数字 %s，在全局坐标 (%.2f, %.2f)【标号：%d】", digit, map_x, map_y, count_same_digit)
        print(msg)

    def recognize_digit(self, cv_image):
        """
        利用 OpenCV 模板匹配实现数字识别：
          输入：
            - cv_image：BGR 格式的 OpenCV 图像
          处理：
            - 转换为灰度图，并做直方图均衡化
            - 遍历每个模板，并在多个尺度上进行匹配（尺度范围和步数可调整，此处为 0.9–1.1，共 5 个尺度）
            - 选取匹配分数最高的模板，如果分数超过预设阈值，则认为检测到对应数字
            - 计算匹配区域中心与图像中心在水平方向的像素偏移
          输出：
            - 返回 (digit, pixel_offset)
              例如 ("3", -20) 表示检测到数字 "3"，且目标在图像中心左侧 20 个像素
            - 如果未能检测到有效数字，则返回 None
        """
        if not self.templates:
            rospy.logerr("没有加载模板，无法进行数字识别！")
            return None

        # 转为灰度图，并进行直方图均衡化
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        best_score = -1
        best_digit = None
        best_loc = None
        best_template_shape = None

        # 对每个模板进行匹配
        for digit, template in self.templates.items():
            # 确保模板为灰度图，并做直方图均衡化
            if len(template.shape) == 3:
                template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            else:
                template_gray = template.copy()
            template_gray = cv2.equalizeHist(template_gray)

            # 在多个尺度上进行匹配，尺度范围由 0.4 到 1.6，共 21 个尺度
            for scale in np.linspace(0.4，1.6，21):
                resized_template = cv2.resize(template_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                tH, tW = resized_template.shape[:2]
                # 模板尺寸若大于图像尺寸，则跳过
                if gray.shape[0] < tH or gray.shape[1] < tW:
                    continue

                res = cv2.matchTemplate(gray, resized_template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                if max_val > best_score:
                    best_score = max_val
                    best_digit = digit
                    best_loc = max_loc
                    best_template_shape = resized_template.shape

        # 设置识别阈值
        threshold = 0.7
        if best_score < threshold or best_digit is None:
            rospy.loginfo("模板匹配未达到阈值，未检测到有效数字。最高分: {:.2f}".format(best_score))
            return None

        # 计算匹配区域的中心坐标
        tH, tW = best_template_shape
        match_center_x = best_loc[0] + tW / 2

        # 计算图像中心水平方向位置
        image_center_x = gray.shape[1] / 2
        pixel_offset = match_center_x - image_center_x

        rospy.loginfo("模板匹配成功：数字 %s，匹配分数: %.2f，像素偏移: %.2f", best_digit, best_score, pixel_offset)
        return best_digit, pixel_offset

    def is_duplicate(self, x, y):
        """
        判断新检测到的目标 (x, y)（全局 map 坐标下）是否与已有目标重复
        输入：
          - x, y：浮点数，新目标在 map 坐标系下的位置
        处理：
          - 遍历 self.detected_objects 中的目标，计算欧氏距离
          - 如果距离小于 duplicate_threshold，则认为为重复目标
        输出：
          - 返回布尔值 True（重复）或 False（不重复）
        """
        for obj in self.detected_objects:
            dist = math.hypot(obj["map_x"] - x, obj["map_y"] - y)
            if dist < self.duplicate_threshold:
                return True
        return False

    def report_least_frequent_digit(self, event):
        """
        定时统计检测到目标中数字出现的频率，并将出现次数最少的数字保存到 ROS 参数服务器，
        以供后续任务调用
        输入：
          - event：定时器传入的参数（可忽略）
        输出：
          - 无返回值，同时通过日志输出统计结果
        """
        if not self.detected_objects:
            rospy.loginfo("目前尚未检测到目标。")
            return

        freq = {}
        for obj in self.detected_objects:
            d = obj["digit"]
            freq[d] = freq.get(d, 0) + 1

        # 找出出现次数最少的数字
        least_freq_digit = min(freq, key=freq.get)
        count = freq[least_freq_digit]
        rospy.loginfo("出现次数最少的数字为：%s（出现 %d 次）", least_freq_digit, count)
        rospy.set_param("/least_frequent_digit", least_freq_digit)

if __name__ == '__main__':
    try:
        detector = TargetDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
