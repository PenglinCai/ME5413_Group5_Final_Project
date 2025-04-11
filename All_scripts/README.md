# start_robot State Machine

This repository includes a state machine script (`strat_robot.py`) that orchestrates the startup, management, and shutdown of multiple ROS/Python nodes involved in robot operations. The state machine handles tasks such as digit recognition, object and bridge detection, path planning, and navigation by running different scripts either continuously or once, according to a defined sequence.

## Overview

The state machine uses subprocesses to launch external scripts and threads to manage scripts that must run continuously (with automatic restarts if they exit unexpectedly). The overall workflow is divided into several states:

1. **Continuous Execution:**  
   - **`digit_recognition_service_node.py`:**  
     Runs continuously to provide a digit recognition service.
   - **`findcube.py`:**  
     Runs continuously to detect cube-like objects.
  
2. **One-Time Execution:**  
   - **`snake_path.py`:**  
     Executes once to generate a snake-like navigation path.
   - **`subscribe_box_pos.py`:**  
     Executes once to process box position data.

3. **State Transition:**  
   - **State 4:**  
     Stops `findcube.py` and then continuously runs **`findbridge.py`** to detect bridge-like structures.
   - After `findbridge.py` starts, **`pub_bridge_pos.py`** is launched continuously to publish bridge position data.
  
4. **Bridge Crossing and Post-Operations:**  
   - **State 5:**  
     Runs **`cross_bridge.py`** once, handling the crossing operation.
   - Immediately after, `findbridge.py` is stopped.
   - **State 6:**  
     Runs **`after_bridge.py`** once to execute post-bridge operations.

5. **Cleanup:**  
   - Finally, the state machine stops all continuously running scripts and terminates gracefully.

## How to Run

To start the state machine, execute:

```bash
python3 start_robot.py
