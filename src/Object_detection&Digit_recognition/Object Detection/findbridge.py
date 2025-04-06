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

# =================== 可调参数 ===================
# ROI：仅合并中心在此区域内的聚类
REGION_MIN_X = 5.0
REGION_MAX_X = 8.0
REGION_MIN_Y = -21.0
REGION_MAX_Y = -3.0

# DBSCAN 参数
CLUSTER_EPS = 0.3
CLUSTER_MIN_SAMPLES = 10

# 坐标系
INPUT_FRAME = "front_laser"
OUTPUT_FRAME = "map"

# 可视化
MARKER_TOPIC = "/cluster_markers_map"
MARKER_SCALE = 0.4

# 尺寸筛选
BRIDGE_LENGTH_MAX = 4.0  # X ≤4m
BRIDGE_WIDTH_MAX  = 2.0  # Y ≤2m

# 起始坐标偏移
BRIDGE_X_OFFSET = 2.0

# 目标桥尺寸
TARGET_BRIDGE_LENGTH = 3.5
TARGET_BRIDGE_WIDTH  = 1.6
# ================================================

class BridgeDetector:
    def __init__(self):
        rospy.init_node('bridge_detector')
        # 参数赋值
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

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # 发布/订阅
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1)
        self.bridge_info_pub = rospy.Publisher('/bridge_info', String, queue_size=1)

        # 状态
        self.best_bridge = None    # {'envelope':…, 'center':…, 'width':…, 'height':…}
        self.best_error = float('inf')

        # 定时打印
        rospy.Timer(rospy.Duration(1.0), self.print_bridge_status)

        rospy.loginfo("BridgeDetector started")

    def scan_callback(self, msg):
        # 转点并聚类
        angle = msg.angle_min
        pts = []
        for r in msg.ranges:
            if np.isfinite(r):
                pts.append([r*math.cos(angle), r*math.sin(angle)])
            angle += msg.angle_increment
        pts = np.array(pts)
        if len(pts) < self.cluster_min_samples:
            return

        try:
            tf = self.tf_buffer.lookup_transform(OUTPUT_FRAME, INPUT_FRAME,
                                                 rospy.Time(0), rospy.Duration(1.0))
        except Exception:
            return

        labels = DBSCAN(eps=self.cluster_eps,
                        min_samples=self.cluster_min_samples).fit(pts).labels_

        # 筛选出中心在 ROI 内的每个聚类，并合并
        clusters = []
        for lab in set(labels):
            if lab<0: continue
            cpts = pts[labels==lab]
            mapped = []
            for x,y in cpts:
                ps = PoseStamped(); ps.header.frame_id=INPUT_FRAME; ps.header.stamp=rospy.Time(0)
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x,y,0
                ps.pose.orientation.w=1
                try:
                    pm = tf2_geometry_msgs.do_transform_pose(ps, tf)
                    mapped.append([pm.pose.position.x, pm.pose.position.y])
                except:
                    continue
            if not mapped: continue
            arr = np.array(mapped)
            cx, cy = arr[:,0].mean(), arr[:,1].mean()
            if not (self.region_min_x<=cx<=self.region_max_x and
                    self.region_min_y<=cy<=self.region_max_y):
                continue
            clusters.append(arr)

        # 如果有聚类，合并所有点计算当前 envelope
        current = None
        if clusters:
            allpts = np.vstack(clusters)
            min_x, max_x = allpts[:,0].min(), allpts[:,0].max()
            min_y, max_y = allpts[:,1].min(), allpts[:,1].max()
            env = [[min_x,min_y],[max_x,min_y],[max_x,max_y],[min_x,max_y]]
            w,h = max_x-min_x, max_y-min_y
            current = {'envelope':env,'center':((min_x+max_x)/2,(min_y+max_y)/2),
                       'width':w,'height':h}

        # 计算误差并更新 best_bridge
        if current and w<=self.bridge_length_max and h<=self.bridge_width_max:
            err = abs(w-self.target_length)+abs(h-self.target_width)
            if err<self.best_error:
                self.best_error = err
                self.best_bridge = current

        self.publish_markers(current)
        self.publish_bridge_info()

    def publish_markers(self, current):
        ma = MarkerArray()
        # 清除旧 marker
        for ns in ('current','best'):
            for i in range(2):
                m=Marker(); m.header.frame_id=OUTPUT_FRAME; m.header.stamp=rospy.Time.now()
                m.ns = ns; m.id=i; m.action=Marker.DELETE
                ma.markers.append(m)

        # 当前 envelope: green
        if current:
            m=Marker(); m.header.frame_id=OUTPUT_FRAME; m.header.stamp=rospy.Time.now()
            m.ns="current"; m.id=0; m.type=Marker.LINE_STRIP; m.action=Marker.ADD
            m.scale.x=0.1; m.color.r=0; m.color.g=1; m.color.b=0; m.color.a=1
            for x,y in current['envelope']:
                m.points.append(Point(x,y,0))
            m.points.append(Point(*current['envelope'][0],0))
            ma.markers.append(m)

        # Best envelope: blue
        if self.best_bridge:
            m=Marker(); m.header.frame_id=OUTPUT_FRAME; m.header.stamp=rospy.Time.now()
            m.ns="best"; m.id=0; m.type=Marker.LINE_STRIP; m.action=Marker.ADD
            m.scale.x=0.15; m.color.r=0; m.color.g=0; m.color.b=1; m.color.a=1
            for x,y in self.best_bridge['envelope']:
                m.points.append(Point(x,y,0))
            m.points.append(Point(*self.best_bridge['envelope'][0],0))
            ma.markers.append(m)

            # 文本：起始坐标
            bx,by = self.best_bridge['center']
            sx,sy = bx+self.bridge_x_offset, by
            t=Marker(); t.header.frame_id=OUTPUT_FRAME; t.header.stamp=rospy.Time.now()
            t.ns="best"; t.id=1; t.type=Marker.TEXT_VIEW_FACING; t.action=Marker.ADD
            t.pose.position.x=sx; t.pose.position.y=sy; t.pose.position.z=1.0
            t.scale.z=0.5; t.color.r=1; t.color.g=1; t.color.b=0; t.color.a=1
            t.text="Start:(%.2f,%.2f)"%(sx,sy)
            ma.markers.append(t)

        self.marker_pub.publish(ma)

    def publish_bridge_info(self):
        if self.best_bridge:
            cx,cy=self.best_bridge['center']
            sx,sy=cx+self.bridge_x_offset,cy
            msg=String(data="Bridge:Center=(%.2f,%.2f) Start=(%.2f,%.2f)"%(cx,cy,sx,sy))
        else:
            msg=String(data="No bridge")
        self.bridge_info_pub.publish(msg)

    def print_bridge_status(self,event):
        if self.best_bridge:
            cx,cy=self.best_bridge['center']
            sx,sy=cx+self.bridge_x_offset,cy
            rospy.loginfo("Best Bridge: Center=(%.2f,%.2f) Start=(%.2f,%.2f)"%(cx,cy,sx,sy))
        else:
            rospy.loginfo("No bridge")

if __name__=='__main__':
    rospy.init_node('bridge_detector')
    bd=BridgeDetector()
    rospy.spin()
