#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from digit_recognizer import DigitRecognizer

class DigitRecognitionServiceNode:
    def __init__(self):
        # 创建数字识别器实例（内部已订阅 /front/image_raw）
        self.recognizer = DigitRecognizer()
        # 定义 ROS 服务接口，用 Trigger 服务类型实现简单的无参请求应答
        self.start_srv = rospy.Service('start_recognition', Trigger, self.handle_start)
        self.stop_srv  = rospy.Service('stop_recognition', Trigger, self.handle_stop)
        rospy.loginfo("DigitRecognitionServiceNode 启动完毕，等待调用……")

    def handle_start(self, req):
        rospy.loginfo("收到启动识别请求。")
        self.recognizer.start_recognition()
        return TriggerResponse(success=True, message="数字识别已启动。")

    def handle_stop(self, req):
        rospy.loginfo("收到停止识别请求。")
        best_digit = self.recognizer.stop_recognition()
        if best_digit is None:
            message = "未检测到有效数字。"
        else:
            message = f"最佳识别结果为：{best_digit}"
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)

if __name__ == '__main__':
    rospy.init_node('digit_recognition_service_node')
    node = DigitRecognitionServiceNode()
    rospy.spin()
