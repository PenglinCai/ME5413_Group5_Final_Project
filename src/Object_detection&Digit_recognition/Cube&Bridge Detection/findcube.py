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

# =================== Adjustable Parameters ===================
# Define the valid region boundaries in the output frame (map coordinates).
REGION_MIN_X = 10.0      
REGION_MAX_X = 19.5      
REGION_MIN_Y = -22.0     
REGION_MAX_Y = -2.5      

# DBSCAN clustering parameters: 
# 'CLUSTER_EPS' is the maximum distance between two samples for them to be considered neighbors.
# 'CLUSTER_MIN_SAMPLES' is the minimum number of points required to form a cluster.
CLUSTER_EPS = 0.2        
CLUSTER_MIN_SAMPLES = 10 

# Frame definitions:
# 'INPUT_FRAME' is the frame in which the laser scan data is captured.
# 'OUTPUT_FRAME' is the target frame to which the point cloud data will be transformed.
INPUT_FRAME = "front_laser"  
OUTPUT_FRAME = "map"         

# Visualization marker settings:
# 'MARKER_TOPIC' is the topic to which the cluster markers will be published.
# 'MARKER_SCALE' controls the size of the sphere markers representing cluster centers.
MARKER_TOPIC = "/cluster_markers_map"  
MARKER_SCALE = 0.4                     

# Valid bounding rectangle size range (in meters) for a cluster to be considered as a valid block.
VALID_RECT_MIN_SIZE = 0.6  
VALID_RECT_MAX_SIZE = 0.9  

# When fusing clusters with previously detected blocks, merge them if their centers are within this threshold.
MERGE_DISTANCE_THRESHOLD = 0.9
# Fusion weights for blending the previously found block with the new cluster data.
FUSION_OLD_WEIGHT = 0.4
FUSION_NEW_WEIGHT = 0.6
# =================================================================

class LocalClusterVisualizer:
    def __init__(self):
        # Initialize the ROS node.
        rospy.init_node('local_cluster_visualizer')
        self.last_scan = None
        self.last_clusters = []
        self.found_blocks = []

        # Initialize the TF buffer and listener to handle coordinate frame transformations.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to the laser scan topic and set up publishers for visualization markers and block information.
        rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1)
        self.info_pub = rospy.Publisher('/found_blocks_info', String, queue_size=10)

        # Set up a timer to process clustering, fusion, and publishing every 0.2 seconds.
        rospy.Timer(rospy.Duration(0.2), self.process_callback)
        rospy.loginfo("LocalClusterVisualizer started. Processing clustering and fusion every 0.2 seconds.")

    def scan_callback(self, msg):
        # Callback to update the latest laser scan data.
        self.last_scan = msg

    def process_callback(self, event):
        try:
            # Step 1: Prepare the point cloud from the laser scan data.
            scan = self.last_scan
            if scan is None:
                return
            pts = []
            ang = scan.angle_min
            for r in scan.ranges:
                # Only consider finite range values.
                if math.isfinite(r):
                    # Convert polar coordinates (range and angle) to Cartesian coordinates.
                    pts.append([r * math.cos(ang), r * math.sin(ang)])
                ang += scan.angle_increment
            pts = np.array(pts)
            if len(pts) < CLUSTER_MIN_SAMPLES:
                self.last_clusters = []
            else:
                # Step 2: Apply TF transformation.
                # Look up the latest transform from the input frame (laser) to the output frame (map).
                try:
                    trans = self.tf_buffer.lookup_transform(
                        OUTPUT_FRAME, INPUT_FRAME, rospy.Time(0), rospy.Duration(1.0))
                except Exception as e:
                    rospy.logwarn("TF lookup failed: {}".format(e))
                    return

                # Step 3: Perform DBSCAN clustering on the point cloud.
                labels = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES).fit_predict(pts)
                clusters = []
                # Process each detected cluster (ignore label -1 which indicates noise).
                for lbl in set(labels):
                    if lbl == -1:
                        continue
                    cpts = pts[labels == lbl]
                    mpts = []
                    # Transform each point in the cluster from the input frame to the output frame.
                    for x, y in cpts:
                        ps = PoseStamped()
                        ps.header.frame_id = INPUT_FRAME
                        ps.header.stamp = rospy.Time(0)
                        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, 0
                        ps.pose.orientation.w = 1.0
                        try:
                            pm = tf2_geometry_msgs.do_transform_pose(ps, trans)
                            mpts.append([pm.pose.position.x, pm.pose.position.y])
                        except:
                            continue
                    if not mpts:
                        continue
                    arr = np.array(mpts)
                    # Calculate the bounding rectangle of the transformed points.
                    min_x, max_x = arr[:, 0].min(), arr[:, 0].max()
                    min_y, max_y = arr[:, 1].min(), arr[:, 1].max()
                    # Compute the center of the bounding rectangle.
                    cx, cy = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
                    # Only consider clusters whose centers lie within the predefined valid region.
                    if not (REGION_MIN_X <= cx <= REGION_MAX_X and REGION_MIN_Y <= cy <= REGION_MAX_Y):
                        continue
                    clusters.append({
                        'center': (cx, cy),
                        'envelope': [[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]],
                        'width': max_x - min_x,
                        'height': max_y - min_y
                    })
                self.last_clusters = clusters

            # Step 4: Fusion update - merge new valid clusters with previously detected blocks.
            for cl in self.last_clusters:
                w, h = cl['width'], cl['height']
                # Consider only clusters with rectangle size within the valid range.
                if VALID_RECT_MIN_SIZE <= w <= VALID_RECT_MAX_SIZE and VALID_RECT_MIN_SIZE <= h <= VALID_RECT_MAX_SIZE:
                    nc = np.array(cl['center'])
                    merged = False
                    for fb in self.found_blocks:
                        fc = np.array(fb['center'])
                        if np.linalg.norm(nc - fc) < MERGE_DISTANCE_THRESHOLD:
                            # Fuse the new measurement with the existing block using weighted averaging.
                            uc = fc * FUSION_OLD_WEIGHT + nc * FUSION_NEW_WEIGHT
                            fb['center'] = (uc[0], uc[1])
                            fb['width']  = fb['width'] * FUSION_OLD_WEIGHT + w * FUSION_NEW_WEIGHT
                            fb['height'] = fb['height'] * FUSION_OLD_WEIGHT + h * FUSION_NEW_WEIGHT
                            # Update the bounding envelope based on the newly fused dimensions.
                            w2, h2 = fb['width'], fb['height']
                            x0, y0 = uc[0] - w2 / 2, uc[1] - h2 / 2
                            fb['envelope'] = [[x0, y0], [x0 + w2, y0], [x0 + w2, y0 + h2], [x0, y0 + h2]]
                            merged = True
                            break
                    if not merged:
                        # If no close block is found, append the new cluster as a new found block.
                        self.found_blocks.append(cl)

            # Step 5: Publish visualization markers.
            self.publish_markers()
            # Step 6: Publish information about the found blocks.
            self.publish_info()

        except Exception as e:
            rospy.logerr("Internal exception in process_callback: {}".format(e))

    def publish_markers(self):
        ma = MarkerArray()
        # Delete any old markers by sending delete actions for all namespaces.
        for ns in ["map_clusters", "map_cluster_rectangles", "found_blocks"]:
            for i in range(100):
                m = Marker()
                m.header.frame_id = OUTPUT_FRAME
                m.header.stamp = rospy.Time.now()
                m.ns = ns
                m.id = i
                m.action = Marker.DELETE
                ma.markers.append(m)

        # Add new markers: one for each detected cluster center and its bounding rectangle.
        for i, cl in enumerate(self.last_clusters):
            cx, cy = cl['center']
            env = cl['envelope']
            # Create a sphere marker for the cluster center.
            s = Marker()
            s.header.frame_id = OUTPUT_FRAME
            s.header.stamp = rospy.Time.now()
            s.ns = "map_clusters"
            s.id = i
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose.position.x, s.pose.position.y, s.pose.position.z = cx, cy, 0
            s.pose.orientation.w = 1
            s.scale.x = s.scale.y = s.scale.z = MARKER_SCALE
            s.color.r = 1
            s.color.a = 1
            ma.markers.append(s)

            # Create a line strip marker to represent the bounding rectangle of the cluster.
            r = Marker()
            r.header.frame_id = OUTPUT_FRAME
            r.header.stamp = rospy.Time.now()
            r.ns = "map_cluster_rectangles"
            r.id = i
            r.type = Marker.LINE_STRIP
            r.action = Marker.ADD
            r.scale.x = 0.1
            # Use green color if the rectangle size is valid; otherwise use an orange-like color.
            if VALID_RECT_MIN_SIZE <= cl['width'] <= VALID_RECT_MAX_SIZE and VALID_RECT_MIN_SIZE <= cl['height'] <= VALID_RECT_MAX_SIZE:
                r.color.g = 1
                r.color.a = 1
            else:
                r.color.r = 1
                r.color.g = 0.65
                r.color.a = 1
            # Append the rectangle vertices (closing the loop by adding the first point at the end).
            for x, y in env + [env[0]]:
                p = Point(x=x, y=y, z=0)
                r.points.append(p)
            ma.markers.append(r)

        # Add markers for the found blocks after fusion.
        for i, fb in enumerate(self.found_blocks):
            r = Marker()
            r.header.frame_id = OUTPUT_FRAME
            r.header.stamp = rospy.Time.now()
            r.ns = "found_blocks"
            r.id = i
            r.type = Marker.LINE_STRIP
            r.action = Marker.ADD
            r.scale.x = 0.1
            r.color.b = 1
            r.color.a = 1
            # Draw the envelope for the found block, closing the loop.
            for x, y in fb['envelope'] + [fb['envelope'][0]]:
                r.points.append(Point(x=x, y=y, z=0))
            ma.markers.append(r)

        # Publish the array of markers.
        self.marker_pub.publish(ma)

    def publish_info(self):
        # Build a string with information about each found block.
        rows = []
        for idx, fb in enumerate(self.found_blocks, 1):
            x, y = fb['center']
            rows.append(f"({idx},{x:.5f},{y:.5f},{fb.get('label',0)})")
        s = ", ".join(rows)
        rospy.loginfo("Found blocks: " + s)
        # Publish the found blocks' information as a string message.
        self.info_pub.publish(String(data=s))

if __name__ == '__main__':
    try:
        LocalClusterVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
