#!/usr/bin/env python
# This script implements a state machine that manages the execution of multiple ROS/Python scripts.
# Some scripts are run continuously (and auto-restarted if they exit) whereas others are run once.
# It uses subprocesses to start the scripts and threads to manage their continuous execution.

import subprocess      # For launching external scripts as subprocesses
import threading       # For running functions concurrently in separate threads
import time            # For sleep functions and timing operations
import sys             # To access the current Python interpreter executable

def run_continuous(script, stop_event):
    """
    Continuously runs the specified script.
    If the script exits unexpectedly, it is automatically restarted until the stop_event is set.
    
    Parameters:
      script (str): The filename of the script to run (e.g., "findcube.py").
      stop_event (threading.Event): A threading event used to signal when to stop running the script.
    """
    # Loop until the stop_event has been triggered.
    while not stop_event.is_set():
        # Launch the script using the current Python interpreter.
        process = subprocess.Popen([sys.executable, script])
        print(f"Starting {script} ...")
        
        # Continuously check whether the process is still running.
        while process.poll() is None:
            # If a stop signal is received during execution...
            if stop_event.is_set():
                # Terminate the running process gracefully.
                process.terminate()
                print(f"Terminating {script} ...")
                break
            # Wait for 1 second before checking again.
            time.sleep(1)
        
        # If the process exited on its own and the stop_event isn't set, restart after a delay.
        if not stop_event.is_set():
            print(f"{script} exited, restarting in 1 second...")
            time.sleep(1)

def run_once(script):
    """
    Runs a script a single time and waits until it completes.
    
    Parameters:
      script (str): The filename of the script to execute.
    """
    print(f"Starting {script} ...")
    # Launch the script as a subprocess.
    process = subprocess.Popen([sys.executable, script])
    # Wait for the script to finish execution.
    process.wait()
    print(f"{script} completed.")

def main():
    """
    Main function that defines the various states of the overall process.
    It orchestrates the continuous and one-time execution of various scripts.
    """
    # State: Continuously run digit_recognition_service_node.py
    stop_event_digit = threading.Event()  # Create an event to signal when to stop the script
    thread_digit = threading.Thread(
        target=run_continuous,
        args=("digit_recognition_service_node.py", stop_event_digit),
        daemon=True   # Daemon thread to ensure it exits when the main program ends
    )
    thread_digit.start()  # Start the continuous execution thread for digit_recognition_service_node.py

    # State 1: Continuously run findcube.py
    stop_event_findcube = threading.Event()  # Event for findcube.py
    thread_findcube = threading.Thread(
        target=run_continuous,
        args=("findcube.py", stop_event_findcube),
        daemon=True
    )
    thread_findcube.start()  # Start findcube.py as a continuously running script

    time.sleep(1)  # Wait for 1 second to allow findcube.py to fully start

    # State 2: Run snake_path.py one time (only once)
    run_once("snake_path.py")

    # State 3: Run subscribe_box_pos.py one time (only once)
    run_once("subscribe_box_pos.py")

    # State 4: Stop findcube.py and then start continuously running findbridge.py
    print("State 4 begins: Stopping findcube...")
    stop_event_findcube.set()  # Signal findcube.py to stop
    thread_findcube.join()     # Wait until findcube.py's thread has finished

    # Start findbridge.py continuously using a new stop event and thread.
    stop_event_findbridge = threading.Event()
    thread_findbridge = threading.Thread(
        target=run_continuous,
        args=("findbridge.py", stop_event_findbridge),
        daemon=True
    )
    thread_findbridge.start()  # Begin continuous execution of findbridge.py

    time.sleep(1)  # Wait 1 second to allow findbridge.py to fully start

    # After findbridge.py is running, continuously run pub_bridge_pos.py.
    stop_event_pubbridge = threading.Event()
    thread_pubbridge = threading.Thread(
        target=run_continuous,
        args=("pub_bridge_pos.py", stop_event_pubbridge),
        daemon=True
    )
    thread_pubbridge.start()  # Start pub_bridge_pos.py as a continuously running script

    # State 5: Run cross_bridge.py one time.
    run_once("cross_bridge.py")

    # Immediately stop findbridge.py after cross_bridge.py completes.
    print("Cross bridge completed: Stopping findbridge...")
    stop_event_findbridge.set()  # Signal findbridge.py to stop
    thread_findbridge.join()     # Wait for findbridge.py thread to finish

    # State 6: Run after_bridge.py one time.
    run_once("after_bridge.py")

    # Terminate all continuously running scripts.
    print("All states completed. Terminating continuously running scripts...")
    stop_event_digit.set()       # Stop digit_recognition_service_node.py
    stop_event_pubbridge.set()   # Stop pub_bridge_pos.py

    # Wait for the threads to complete cleanly.
    thread_digit.join()
    thread_pubbridge.join()

    print("State machine completed.")

# Main entry point: run the state machine.
if __name__ == "__main__":
    main()
