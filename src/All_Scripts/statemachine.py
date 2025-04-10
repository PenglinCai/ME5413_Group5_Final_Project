import subprocess
import threading
import time
import sys

def run_continuous(script, stop_event):
    """
    Continuously runs the specified script.
    If the script exits, it will be automatically restarted until stop_event is set.
    """
    while not stop_event.is_set():
        # Use the current Python interpreter
        process = subprocess.Popen([sys.executable, script])
        print(f"Starting {script} ...")
        while process.poll() is None:
            if stop_event.is_set():
                process.terminate()
                print(f"Terminating {script} ...")
                break
            time.sleep(1)
        if not stop_event.is_set():
            print(f"{script} exited, restarting in 1 second...")
            time.sleep(1)

def run_once(script):
    """
    Runs a script once and waits for it to complete.
    """
    print(f"Starting {script} ...")
    process = subprocess.Popen([sys.executable, script])
    process.wait()
    print(f"{script} completed.")

def main():
    # Continuously run digit_recognition_service_node.py (this script will not be stopped)
    stop_event_digit = threading.Event()
    thread_digit = threading.Thread(target=run_continuous, args=("digit_recognition_service_node.py", stop_event_digit), daemon=True)
    thread_digit.start()

    stop_event_findcube = threading.Event()
    stop_event_findbridge = threading.Event()

    # State 1: Continuously run findcube.py
    thread_findcube = threading.Thread(target=run_continuous, args=("findcube.py", stop_event_findcube), daemon=True)
    thread_findcube.start()

    time.sleep(2)  # Wait for findcube.py to start

    # State 2: Run snake_path.py
    run_once("snake_path.py")

    # State 3: Run subscribe_box_pos.py
    run_once("subscribe_box_pos.py")

    # State 4: Continuously run findbridge.py
    thread_findbridge = threading.Thread(target=run_continuous, args=("findbridge.py", stop_event_findbridge), daemon=True)
    thread_findbridge.start()

    time.sleep(2)  # Wait for findbridge.py to start

    # Start continuously running pub_bridge_pos.py concurrently with findbridge.py
    stop_event_pubbridge = threading.Event()
    thread_pubbridge = threading.Thread(target=run_continuous, args=("pub_bridge_pos.py", stop_event_pubbridge), daemon=True)
    thread_pubbridge.start()

    # State 5: Run cross_bridge.py
    run_once("cross_bridge.py")
    
    # After cross_bridge.py completes, terminate selected continuously running scripts (excluding digit_recognition_service_node.py)
    print("Crossing bridge complete. Terminating selected continuously running scripts (excluding digit_recognition_service_node.py)...")
    stop_event_findcube.set()
    stop_event_findbridge.set()
    stop_event_pubbridge.set()

    thread_findcube.join()
    thread_findbridge.join()
    thread_pubbridge.join()

    # Then run after_bridge.py
    run_once("after_bridge.py")

    print("State machine completed.")

if __name__ == "__main__":
    main()

