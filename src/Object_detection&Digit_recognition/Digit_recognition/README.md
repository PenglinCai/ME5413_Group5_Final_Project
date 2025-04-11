
# Digit Recognition System (ROS + OpenCV)

## File Overview

| File                          | Description                                          |
|-------------------------------|------------------------------------------------------|
| `digit_recognition_service_node.py` | Launches the digit recognition service node         |
| `digit_recognizer.py`         | Implements the digit recognition algorithm using OpenCV |

> Note: Place the `templates/` folder (containing digit images `1.png` to `9.png`) in the same directory as `digit_recognizer.py`.

---

## System Usage Context

The digit recognition module is used in **two main stages** during the task execution:

1. **Before crossing the bridge**:  
   The robot navigates to each detected cube and performs digit recognition. The purpose is to determine the digit that appears least frequently.

2. **After crossing the bridge**:  
   The robot performs digit recognition again in front of the cubes. The purpose is to stop in front of the cube showing the target digit (i.e., the digit determined to be least frequent earlier).

