# ROS Object Detection Nodes Overview

Both Python scripts subscribe to laser scan data, perform a coordinate frame transformation from the sensor's frame (`front_scan`) to a target map frame (`map`), apply DBSCAN clustering, and then visualize the detection results with markers published over ROS topics.

---

## findcube.py

### Purpose
Detect and track cube-like or block objects from laser scan data.

### Key Features
- **Region of Interest (ROI):**  
  Only clusters with centers within a specified region are processed.
- **Clustering:**  
  Uses DBSCAN (with adjustable `CLUSTER_EPS` and `CLUSTER_MIN_SAMPLES`) for grouping laser scan points.
- **Fusion:**  
  Applies temporal fusion to update detected clusters and generate consistent block estimates.
- **Visualization:**  
  Publishes cluster centers and bounding rectangles on a ROS marker topic (`/cluster_markers_map`) and outputs found block information.

### Configurable Parameters
- **ROI boundaries:**  
  `REGION_MIN_X`, `REGION_MAX_X`, `REGION_MIN_Y`, `REGION_MAX_Y`
- **Clustering parameters:**  
  `CLUSTER_EPS`, `CLUSTER_MIN_SAMPLES`
- **Frames:**  
  `INPUT_FRAME`, `OUTPUT_FRAME`
- **Marker settings:**  
  `MARKER_SCALE`
- **Validation and Fusion thresholds:**  
  Valid rectangle size limits, merge distance, and fusion weights

---

## findbridge.py

### Purpose
Detect bridge-like structures by analyzing laser scan data and merging clusters within a defined region.

### Key Features
- **Region of Interest (ROI):**  
  Only clusters whose centers lie within a specific area are considered.
- **Clustering:**  
  Employs DBSCAN clustering with configurable parameters to find clusters.
- **Bridge Detection:**  
  Merges clusters to form a candidate bridge based on size constraints (length and width) and computes the candidate's error relative to target dimensions. It continuously updates the best candidate and publishes its start coordinate.
- **Visualization:**  
  Uses markers to display both the current candidate and the best bridge detection. Additionally, publishes bridge information on `/bridge_info`.

### Configurable Parameters
- **ROI boundaries:**  
  `REGION_MIN_X`, `REGION_MAX_X`, `REGION_MIN_Y`, `REGION_MAX_Y`
- **Clustering parameters:**  
  `CLUSTER_EPS`, `CLUSTER_MIN_SAMPLES`
- **Bridge size limits:**  
  `BRIDGE_LENGTH_MAX`, `BRIDGE_WIDTH_MAX`
- **Target bridge dimensions:**  
  `TARGET_BRIDGE_LENGTH`, `TARGET_BRIDGE_WIDTH`
- **Coordinate offset:**  
  `BRIDGE_X_OFFSET` for adjusting the bridge's starting coordinate
