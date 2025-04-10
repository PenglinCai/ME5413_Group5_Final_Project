# ROS Navigation and Recognition System Overview

This document briefly describes a multi-module ROS system for navigation and digit recognition. Each module is responsible for a specific task.

1. snake_path.py  
   - Subscribes to AMCL for real-time pose updates.
   - Sends navigation goals with a timeout based on position threshold.
   - Generates waypoints in a snake pattern over a specified area.
   - Sequentially navigates through all waypoints.

2. subscribe_box_pos.py  
   - Listens to block detection data on the /found_blocks_info topic.
   - Sends navigation goals with checks for both position and orientation.
   - Optimizes the path using the Held-Karp algorithm (TSP route).
   - Integrates visual recognition for digits and saves the results to a file.

3. pub_bridge_pos.py  
   - Receives and parses bridge information messages.
   - Extracts coordinate data and publishes a corresponding navigation goal.

4. cross_bridge.py  
   - Navigates to two initial fixed points to set the starting position.
   - Listens for a bridge crossing command, adjusts the target goal, and sends it.
   - Publishes a command to open the bridge and drives straight for a set distance.
   - Shuts down after task completion.

5. after_bridge.py  
   - Sets up a digit recognition client.
   - Loads recognized digits from a file and identifies the least frequent digit.
   - Navigates to predefined box positions and performs additional digit recognition.
   - Stops the mission upon matching the target digit, with retry logic for recognition.
