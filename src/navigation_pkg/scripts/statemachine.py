import subprocess
import threading
import time
import sys

def run_continuous(script, stop_event):
    """
    持续运行指定的脚本。如果脚本退出，则自动重启，直到 stop_event 被设置。
    """
    while not stop_event.is_set():
        # 使用当前 Python 解释器
        process = subprocess.Popen([sys.executable, script])
        print(f"启动 {script} ...")
        while process.poll() is None:
            if stop_event.is_set():
                process.terminate()
                print(f"终止 {script} ...")
                break
            time.sleep(1)
        if not stop_event.is_set():
            print(f"{script} 已退出，1秒后重启...")
            time.sleep(1)

def run_once(script):
    """
    启动一次性运行的脚本，并等待其完成。
    """
    print(f"开始运行 {script} ...")
    process = subprocess.Popen([sys.executable, script])
    process.wait()
    print(f"{script} 执行完毕.")

def main():
    # 持续运行 digit_recognition_service_node.py
    stop_event_digit = threading.Event()
    thread_digit = threading.Thread(target=run_continuous, args=("digit_recognition_service_node.py", stop_event_digit), daemon=True)
    thread_digit.start()

    stop_event_findcube = threading.Event()
    stop_event_findbridge = threading.Event()

    # 状态1：持续运行 findcube.py
    thread_findcube = threading.Thread(target=run_continuous, args=("findcube.py", stop_event_findcube), daemon=True)
    thread_findcube.start()

    time.sleep(2)  # 等待 findcube.py 启动

    # 状态2：运行 snake_path.py
    run_once("snake_path.py")

    # 状态3：运行 subscribe_box_pos.py
    run_once("subscribe_box_pos.py")

    # 状态4：持续运行 findbridge.py
    thread_findbridge = threading.Thread(target=run_continuous, args=("findbridge.py", stop_event_findbridge), daemon=True)
    thread_findbridge.start()

    time.sleep(2)  # 等待 findbridge.py 启动

    # 新增：在运行 findbridge 后持续运行 pub_bridge_pos.py
    stop_event_pubbridge = threading.Event()
    thread_pubbridge = threading.Thread(target=run_continuous, args=("pub_bridge_pos.py", stop_event_pubbridge), daemon=True)
    thread_pubbridge.start()

    # 状态5：运行 cross_bridge.py
    run_once("cross_bridge.py")

    # 结束所有持续运行的进程
    print("所有状态执行完毕，正在结束持续运行的脚本...")
    stop_event_digit.set()
    stop_event_findcube.set()
    stop_event_findbridge.set()
    stop_event_pubbridge.set()

    thread_digit.join()
    thread_findcube.join()
    thread_findbridge.join()
    thread_pubbridge.join()

    print("状态机结束。")

if __name__ == "__main__":
    main()