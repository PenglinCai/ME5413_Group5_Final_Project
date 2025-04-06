#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import subprocess

# 定义一个状态，用于执行指定的脚本
class RunScriptState(smach.State):
    def __init__(self, script_path, interpreter='python3'):
        smach.State.__init__(self, outcomes=['done'])
        self.script_path = script_path
        self.interpreter = interpreter

    def execute(self, userdata):
        cmd = "{} {}".format(self.interpreter, self.script_path)
        rospy.loginfo("执行脚本: %s", cmd)
        ret = subprocess.call(cmd, shell=True)
        if ret != 0:
            rospy.logerr("脚本 %s 退出码： %d", self.script_path, ret)
        return 'done'

def main():
    rospy.init_node("sequential_script_runner")
    # 构建状态机，依次执行 snake_path.py 和 Cross_Bridage.py
    sm = smach.StateMachine(outcomes=['finished'])
    with sm:
        # 先运行 snake_path.py (使用 python3)
        smach.StateMachine.add('RUN_SNAKE_PATH',
                               RunScriptState("./Script/snake_path.py", interpreter='python3'),
                               transitions={'done': 'RUN_CROSS_BRIDGE'})
        # 再运行 Cross_Bridage.py (使用 python)
        smach.StateMachine.add('RUN_CROSS_BRIDGE',
                               RunScriptState("./Script/Cross_Bridge.py", interpreter='python3'),
                               transitions={'done': 'finished'})
    outcome = sm.execute()

if __name__ == '__main__':
    main()
