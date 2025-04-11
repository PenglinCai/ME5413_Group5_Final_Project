This repository contains two ROS nodes, findcube.py and findbridge.py, designed for detecting specific structures using laser scan data and DBSCAN clustering.

Overview
Both nodes subscribe to laser scan data, perform a coordinate frame transformation from the sensor's frame (typically "front_laser") to a target map frame ("map"), apply DBSCAN clustering, and then visualize the detection results with markers published over ROS topics.

findcube.py
Purpose:
Detect and track cube-like or block objects from laser scan data.

Key Features:

Region of Interest (ROI): Only clusters with centers within a specified region are processed.

Clustering: Uses DBSCAN (with adjustable CLUSTER_EPS and CLUSTER_MIN_SAMPLES) for grouping laser scan points.

Fusion: Applies temporal fusion to update detected clusters and generate consistent block estimates.

Visualization: Publishes cluster centers and bounding rectangles on a ROS marker topic (/cluster_markers_map) and outputs found block information.

Configurable Parameters:

ROI boundaries: REGION_MIN_X, REGION_MAX_X, REGION_MIN_Y, REGION_MAX_Y

Clustering parameters: CLUSTER_EPS, CLUSTER_MIN_SAMPLES

Frames: INPUT_FRAME, OUTPUT_FRAME

Marker settings: MARKER_SCALE

Validation and Fusion thresholds: Valid rectangle size limits, merge distance, and fusion weights

findbridge.py
Purpose:
Detect bridge-like structures by analyzing laser scan data and merging clusters within a defined region.

Key Features:

Region of Interest (ROI): Only clusters whose centers lie within a specific area are considered.

Clustering: Employs DBSCAN clustering with configurable parameters to find clusters.

Bridge Detection: Merges clusters to form a candidate bridge based on size constraints (length and width) and computes the candidate's error relative to target dimensions.

Visualization: Uses markers to display both the current candidate and the best bridge detection. Additionally, publishes bridge information on /bridge_info.

Configurable Parameters:

ROI boundaries: REGION_MIN_X, REGION_MAX_X, REGION_MIN_Y, REGION_MAX_Y

Clustering parameters: CLUSTER_EPS, CLUSTER_MIN_SAMPLES

Bridge size limits: BRIDGE_LENGTH_MAX, BRIDGE_WIDTH_MAX

Target bridge dimensions: TARGET_BRIDGE_LENGTH, TARGET_BRIDGE_WIDTH

Coordinate offset: BRIDGE_X_OFFSET for adjusting the bridge's starting coordinate

Running the Nodes
Setup:
Ensure a proper ROS environment and required dependencies are installed (e.g., rospy, sensor_msgs, geometry_msgs, visualization_msgs, sklearn, tf2_ros, tf2_geometry_msgs).

Launch ROS Master:
Start the ROS master with:

bash
复制
roscore
Run Nodes:
Use rosrun (replace [your_package_name] with your actual package name) to run either node:

bash
复制
rosrun [your_package_name] findcube.py
or

bash
复制
rosrun [your_package_name] findbridge.py
Each node subscribes to /front/scan for laser scan data, publishes visualization markers (to /cluster_markers_map), and outputs detection-specific information to topics (/found_blocks_info for findcube.py and /bridge_info for findbridge.py).
