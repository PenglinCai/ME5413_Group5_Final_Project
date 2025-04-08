#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger

def call_start_recognition():
    rospy.wait_for_service('start_recognition')
    try:
        start_recog = rospy.ServiceProxy('start_recognition', Trigger)
        response = start_recog()
        rospy.loginfo("启动识别服务返回: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("启动识别服务调用失败: %s", e)

def call_stop_recognition():
    rospy.wait_for_service('stop_recognition')
    try:
        stop_recog = rospy.ServiceProxy('stop_recognition', Trigger)
        response = stop_recog()
        rospy.loginfo("停止识别服务返回: %s", response.message)
        return response.message
    except rospy.ServiceException as e:
        rospy.logerr("停止识别服务调用失败: %s", e)
        return None

def main():
    rospy.init_node('navigator_client_interactive', anonymous=True)

    # 等待用户输入A以开启识别服务
    print("请在命令行输入 A 来启动数字识别服务:")
    user_input = input().strip()
    if user_input.upper() != 'A':
        print("未检测到 A，程序退出！")
        return

    # 调用启动服务
    call_start_recognition()

    # 提示用户按下 Enter 停止识别
    print("数字识别服务已启动，请按下 Enter 停止识别，并获取最终识别结果...")
    input()  # 等待用户按下 Enter 键

    # 调用停止服务并获取结果
    result_msg = call_stop_recognition()
    print("最终的数字识别返回结果：", result_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
