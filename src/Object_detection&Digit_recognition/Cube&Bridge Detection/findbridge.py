#!/usr/bin/env python
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from sklearn.cluster import DBSCAN
import tf2_ros
import tf2_geometry_msgs

# =================== Configurable Parameters ===================
# Define the Region Of Interest (ROI): only clusters whose centers fall within this region are considered.
REGION_MIN_X = 5.0
REGION_MAX_X = 8.0
REGION_MIN_Y = -21.0
REGION_MAX_Y = -3.0

# Parameters for the DBSCAN clustering algorithm.
CLUSTER_EPS = 0.3             # Maximum distance between points to be considered as neighbors.
CLUSTER_MIN_SAMPLES = 10      # Minimum number of points required to form a valid cluster.

# Coordinate frame definitions:
# INPUT_FRAME: the frame in which the laser scan data is originally captured.
# OUTPUT_FRAME: the target frame in which clustering results will be transformed.
INPUT_FRAME = "front_laser"
OUTPUT_FRAME = "map"

# Visualization settings:
# MARKER_TOPIC: ROS topic for publishing cluster visualization markers.
# MARKER_SCALE: scale (size) for the markers.
MARKER_TOPIC = "/cluster_markers_map"
MARKER_SCALE = 0.4

# Size filtering parameters:
# BRIDGE_LENGTH_MAX: maximum allowed length (in X direction) for a candidate bridge (should be ≤ 4 meters).
# BRIDGE_WIDTH_MAX: maximum allowed width (in Y direction) for a candidate bridge (should be ≤ 2 meters).
BRIDGE_LENGTH_MAX = 4.0
BRIDGE_WIDTH_MAX  = 2.0

# Offset for adjusting the starting coordinate of the bridge.
BRIDGE_X_OFFSET = 2.0

# Desired target bridge dimensions.
TARGET_BRIDGE_LENGTH = 3.5
TARGET_BRIDGE_WIDTH  = 1.6
# ===============================================================

class BridgeDetector:
    def __init__(self):
        # Initialize the ROS node for bridge detection.
        rospy.init_node('bridge_detector')
        
        # Assign parameters to instance variables.
        self.region_min_x = REGION_MIN_X
        self.region_max_x = REGION_MAX_X
        self.region_min_y = REGION_MIN_Y
        self.region_max_y = REGION_MAX_Y
        self.cluster_eps = CLUSTER_EPS
        self.cluster_min_samples = CLUSTER_MIN_SAMPLES
        self.bridge_length_max = BRIDGE_LENGTH_MAX
        self.bridge_width_max  = BRIDGE_WIDTH_MAX
        self.bridge_x_offset   = BRIDGE_X_OFFSET
        self.target_length = TARGET_BRIDGE_LENGTH
        self.target_width  = TARGET_BRIDGE_WIDTH

        # Set up the TF buffer and listener for coordinate transformations.
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Set up subscribers and publishers.
        # Subscribe to the laser scan topic to receive sensor data.
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        # Publisher for visualization markers (current and best bridge candidates).
        self.marker_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1)
        # Publisher to output bridge detection information.
        self.bridge_info_pub = rospy.Publisher('/bridge_info', String, queue_size=1)

        # State variables:
        # best_bridge stores the currently best detected bridge candidate as a dictionary containing 'envelope', 'center', 'width', and 'height'.
        self.best_bridge = None
        # best_error keeps track of the minimal error compared to the target dimensions.
        self.best_error = float('inf')

        # Set up a timer to print the bridge status every 1 second.
        rospy.Timer(rospy.Duration(1.0), self.print_bridge_status)

        rospy.loginfo("BridgeDetector started")

    def scan_callback(self, msg):
        # Convert the laser scan ranges into Cartesian coordinates and perform clustering.
        angle = msg.angle_min
        pts = []
        for r in msg.ranges:
            # Only process finite range measurements.
            if np.isfinite(r):
                pts.append([r * math.cos(angle), r * math.sin(angle)])
            angle += msg.angle_increment
        pts = np.array(pts)
        # Exit if there are not enough points to form a cluster.
        if len(pts) < self.cluster_min_samples:
            return

        try:
            # Lookup the transformation from the input frame (laser) to the output frame (map).
            tf = self.tf_buffer.lookup_transform(OUTPUT_FRAME, INPUT_FRAME,
                                                   rospy.Time(0), rospy.Duration(1.0))
        except Exception:
            return

        # Apply DBSCAN clustering to the points.
        labels = DBSCAN(eps=self.cluster_eps,
                        min_samples=self.cluster_min_samples).fit(pts).labels_

        # Filter clusters: for each cluster, only retain it if its center is within the defined ROI.
        clusters = []
        for lab in set(labels):
            if lab < 0:
                continue  # Skip noise points (label -1).
            cpts = pts[labels == lab]
            mapped = []
            # Transform each point of the cluster into the output frame.
            for x, y in cpts:
                ps = PoseStamped()
                ps.header.frame_id = INPUT_FRAME
                ps.header.stamp = rospy.Time(0)
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, 0
                ps.pose.orientation.w = 1
                try:
                    pm = tf2_geometry_msgs.do_transform_pose(ps, tf)
                    mapped.append([pm.pose.position.x, pm.pose.position.y])
                except:
                    continue
            if not mapped:
                continue
            arr = np.array(mapped)
            # Compute the mean of the transformed points, which defines the cluster center.
            cx, cy = arr[:, 0].mean(), arr[:, 1].mean()
            # Only consider the cluster if its center lies within the ROI.
            if not (self.region_min_x <= cx <= self.region_max_x and
                    self.region_min_y <= cy <= self.region_max_y):
                continue
            clusters.append(arr)

        # Merge all clusters' points (if any) to compute the current candidate's bounding envelope.
        current = None
        if clusters:
            allpts = np.vstack(clusters)
            min_x, max_x = allpts[:, 0].min(), allpts[:, 0].max()
            min_y, max_y = allpts[:, 1].min(), allpts[:, 1].max()
            env = [[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]]
            w, h = max_x - min_x, max_y - min_y
            current = {'envelope': env, 'center': ((min_x + max_x) / 2, (min_y + max_y) / 2),
                       'width': w, 'height': h}

        # Compute the error between the current detection and the desired target bridge dimensions,
        # and update the best_bridge candidate if the current one is valid.
        if current and w <= self.bridge_length_max and h <= self.bridge_width_max:
            err = abs(w - self.target_length) + abs(h - self.target_width)
            if err < self.best_error:
                self.best_error = err
                self.best_bridge = current

        # Publish visualization markers and bridge information.
        self.publish_markers(current)
        self.publish_bridge_info()

    def publish_markers(self, current):
        ma = MarkerArray()
        # Clear old markers by sending DELETE actions for both 'current' and 'best' namespaces.
        for ns in ('current', 'best'):
            for i in range(2):
                m = Marker()
                m.header.frame_id = OUTPUT_FRAME
                m.header.stamp = rospy.Time.now()
                m.ns = ns
                m.id = i
                m.action = Marker.DELETE
                ma.markers.append(m)

        # Publish a marker for the current detection envelope in green.
        if current:
            m = Marker()
            m.header.frame_id = OUTPUT_FRAME
            m.header.stamp = rospy.Time.now()
            m.ns = "current"
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.1
            m.color.r = 0
            m.color.g = 1
            m.color.b = 0
            m.color.a = 1
            for x, y in current['envelope']:
                m.points.append(Point(x, y, 0))
            # Close the envelope loop by adding the first point again.
            m.points.append(Point(*current['envelope'][0], 0))
            ma.markers.append(m)

        # Publish a marker for the best detected bridge envelope in blue.
        if self.best_bridge:
            m = Marker()
            m.header.frame_id = OUTPUT_FRAME
            m.header.stamp = rospy.Time.now()
            m.ns = "best"
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.15
            m.color.r = 0
            m.color.g = 0
            m.color.b = 1
            m.color.a = 1
            for x, y in self.best_bridge['envelope']:
                m.points.append(Point(x, y, 0))
            m.points.append(Point(*self.best_bridge['envelope'][0], 0))
            ma.markers.append(m)

            # Publish a text marker showing the starting coordinate (adjusted by the bridge offset).
            bx, by = self.best_bridge['center']
            sx, sy = bx + self.bridge_x_offset, by
            t = Marker()
            t.header.frame_id = OUTPUT_FRAME
            t.header.stamp = rospy.Time.now()
            t.ns = "best"
            t.id = 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = sx
            t.pose.position.y = sy
            t.pose.position.z = 1.0
            t.scale.z = 0.5
            t.color.r = 1
            t.color.g = 1
            t.color.b = 0
            t.color.a = 1
            t.text = "Start:(%.2f,%.2f)" % (sx, sy)
            ma.markers.append(t)

        # Publish the complete array of markers.
        self.marker_pub.publish(ma)

    def publish_bridge_info(self):
        # Publish a text message containing the center and starting coordinates of the best detected bridge.
        if self.best_bridge:
            cx, cy = self.best_bridge['center']
            sx, sy = cx + self.bridge_x_offset, cy
            msg = String(data="Bridge:Center=(%.2f,%.2f) Start=(%.2f,%.2f)" % (cx, cy, sx, sy))
        else:
            msg = String(data="No bridge")
        self.bridge_info_pub.publish(msg)

    def print_bridge_status(self, event):
        # Log the status of the best detected bridge candidate to the console.
        if self.best_bridge:
            cx, cy = self.best_bridge['center']
            sx, sy = cx + self.bridge_x_offset, cy
            rospy.loginfo("Best Bridge: Center=(%.2f,%.2f) Start=(%.2f,%.2f)" % (cx, cy, sx, sy))
        else:
            rospy.loginfo("No bridge")

if __name__ == '__main__':
    rospy.init_node('bridge_detector')
    bd = BridgeDetector()
    rospy.spin()
