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

# =================== 可调参数（Configurable Parameters） ===================
# 1. ROI 参数（map 坐标系）
REGION_MIN_X = 10.0      
REGION_MAX_X = 19.5      
REGION_MIN_Y = -22.0     
REGION_MAX_Y = -2.5      

# 2. 聚类参数
CLUSTER_EPS = 0.2        
CLUSTER_MIN_SAMPLES = 10 

# 3. 坐标系参数
INPUT_FRAME = "front_laser"  
OUTPUT_FRAME = "map"         

# 4. 可视化 Marker 参数
MARKER_TOPIC = "/cluster_markers_map"  
MARKER_SCALE = 0.4                     

# 5. 合格矩形判定参数
VALID_RECT_MIN_SIZE = 0.6  
VALID_RECT_MAX_SIZE = 0.9  

# 6. 新方块记录和融合参数
MERGE_DISTANCE_THRESHOLD = 1.0  

# 7. 融合权重参数
FUSION_OLD_WEIGHT = 0.4
FUSION_NEW_WEIGHT = 0.6
# ==========================================================================

class LocalClusterVisualizer:
    def __init__(self):
        rospy.init_node('local_cluster_visualizer')

        self.region_min_x = REGION_MIN_X
        self.region_max_x = REGION_MAX_X
        self.region_min_y = REGION_MIN_Y
        self.region_max_y = REGION_MAX_Y
        self.cluster_eps = CLUSTER_EPS
        self.cluster_min_samples = CLUSTER_MIN_SAMPLES
        self.input_frame = INPUT_FRAME
        self.output_frame = OUTPUT_FRAME
        self.marker_topic = MARKER_TOPIC
        self.marker_scale = MARKER_SCALE

        self.fusion_old_weight = FUSION_OLD_WEIGHT
        self.fusion_new_weight = FUSION_NEW_WEIGHT
        self.merge_distance_threshold = MERGE_DISTANCE_THRESHOLD

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.found_blocks_info_pub = rospy.Publisher('/found_blocks_info', String, queue_size=10)

        # 存储当前扫描得到的聚类信息，每个字典包含:
        # {'center': (x, y), 'envelope': [[x1,y1], [x2,y2], [x3,y3], [x4,y4]], 'width': w, 'height': h}
        self.last_clusters = []
        # 存储记录的激光检测的“找到的方块”
        self.found_blocks = []

        # 定时器：每秒打印一次调试信息（可选）
        self.print_timer = rospy.Timer(rospy.Duration(1.0), self.print_rectangles_info)
        # 定时器：每 0.2 秒触发一次融合更新
        self.fusion_timer = rospy.Timer(rospy.Duration(0.2), self.fusion_update_callback)

        rospy.loginfo("LocalClusterVisualizer 已启动，监听 /front/scan")

    def scan_callback(self, msg):
        # 将 LaserScan 数据转换为二维点（激光坐标系下）
        angle = msg.angle_min
        points = []
        for r in msg.ranges:
            if not math.isinf(r) and not math.isnan(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += msg.angle_increment
        points = np.array(points)
        if len(points) < self.cluster_min_samples:
            return

        # 获取 TF 变换，从激光坐标系到 map 坐标系
        try:
            tf = self.tf_buffer.lookup_transform(self.output_frame, self.input_frame,
                                                   rospy.Time(0), rospy.Duration(1.0))
        except Exception as e:
            return

        # 在激光坐标系下进行 DBSCAN 聚类
        clustering = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples).fit(points)
        labels = clustering.labels_

        clusters_info = []
        for label in set(labels):
            if label == -1:
                continue  # 忽略噪声
            cluster_pts = points[labels == label]
            points_map = []
            # 对聚类内的每个点转换到 map 坐标系
            for pt in cluster_pts:
                ps = PoseStamped()
                ps.header.frame_id = self.input_frame
                ps.header.stamp = rospy.Time(0)
                ps.pose.position.x = pt[0]
                ps.pose.position.y = pt[1]
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                try:
                    pt_map = tf2_geometry_msgs.do_transform_pose(ps, tf)
                    points_map.append([pt_map.pose.position.x, pt_map.pose.position.y])
                except Exception as e:
                    continue
            if len(points_map) == 0:
                continue

            points_map = np.array(points_map)
            min_x = np.min(points_map[:, 0])
            max_x = np.max(points_map[:, 0])
            min_y = np.min(points_map[:, 1])
            max_y = np.max(points_map[:, 1])
            envelope_map = [
                [min_x, min_y],
                [max_x, min_y],
                [max_x, max_y],
                [min_x, max_y]
            ]
            center_rect = [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0]
            width = max_x - min_x
            height = max_y - min_y

            # 检查是否在预设 ROI 内
            if not (self.region_min_x <= center_rect[0] <= self.region_max_x and
                    self.region_min_y <= center_rect[1] <= self.region_max_y):
                continue

            clusters_info.append({
                'center': (center_rect[0], center_rect[1]),
                'envelope': envelope_map,
                'width': width,
                'height': height
            })

        self.last_clusters = clusters_info
        # 融合更新由 fusion_timer 调用 fusion_update_callback 实现

    def fusion_update_callback(self, event):
        for cluster in self.last_clusters:
            width = cluster['width']
            height = cluster['height']
            if VALID_RECT_MIN_SIZE <= width <= VALID_RECT_MAX_SIZE and VALID_RECT_MIN_SIZE <= height <= VALID_RECT_MAX_SIZE:
                new_center = np.array(cluster['center'])
                new_width = width
                new_height = height
                merged = False
                for fb in self.found_blocks:
                    fb_center = np.array(fb['center'])
                    if np.linalg.norm(new_center - fb_center) < MERGE_DISTANCE_THRESHOLD:
                        updated_center = fb_center * self.fusion_old_weight + new_center * self.fusion_new_weight
                        fb['center'] = (updated_center[0], updated_center[1])
                        fb['width'] = fb['width'] * self.fusion_old_weight + new_width * self.fusion_new_weight
                        fb['height'] = fb['height'] * self.fusion_old_weight + new_height * self.fusion_new_weight
                        new_min_x = updated_center[0] - fb['width'] / 2.0
                        new_max_x = updated_center[0] + fb['width'] / 2.0
                        new_min_y = updated_center[1] - fb['height'] / 2.0
                        new_max_y = updated_center[1] + fb['height'] / 2.0
                        fb['envelope'] = [
                            [new_min_x, new_min_y],
                            [new_max_x, new_min_y],
                            [new_max_x, new_max_y],
                            [new_min_x, new_max_y]
                        ]
                        merged = True
                        break
                if not merged:
                    self.found_blocks.append(cluster)
        self.publish_markers()
        self.publish_found_blocks_info()

    def publish_markers(self):
        ma = MarkerArray()

        # 删除旧 Marker（两组）
        for i in range(100):
            m_del = Marker()
            m_del.header.frame_id = self.output_frame
            m_del.header.stamp = rospy.Time.now()
            m_del.ns = "map_clusters"
            m_del.id = i
            m_del.action = Marker.DELETE
            ma.markers.append(m_del)
        for i in range(100):
            m_del = Marker()
            m_del.header.frame_id = self.output_frame
            m_del.header.stamp = rospy.Time.now()
            m_del.ns = "map_cluster_rectangles"
            m_del.id = i
            m_del.action = Marker.DELETE
            ma.markers.append(m_del)

        for i, cluster in enumerate(self.last_clusters):
            center = cluster['center']
            envelope = cluster['envelope']
            width = cluster['width']
            height = cluster['height']

            # 红色 sphere 显示聚类中心
            m_sphere = Marker()
            m_sphere.header.frame_id = self.output_frame
            m_sphere.header.stamp = rospy.Time.now()
            m_sphere.ns = "map_clusters"
            m_sphere.id = i
            m_sphere.type = Marker.SPHERE
            m_sphere.action = Marker.ADD
            m_sphere.pose.position.x = center[0]
            m_sphere.pose.position.y = center[1]
            m_sphere.pose.position.z = 0.0
            m_sphere.pose.orientation = Quaternion(0, 0, 0, 1)
            m_sphere.scale.x = m_sphere.scale.y = m_sphere.scale.z = self.marker_scale
            m_sphere.color.r = 1.0
            m_sphere.color.g = 0.0
            m_sphere.color.b = 0.0
            m_sphere.color.a = 1.0
            ma.markers.append(m_sphere)

            # LINE_STRIP 显示包络矩形
            m_rect = Marker()
            m_rect.header.frame_id = self.output_frame
            m_rect.header.stamp = rospy.Time.now()
            m_rect.ns = "map_cluster_rectangles"
            m_rect.id = i
            m_rect.type = Marker.LINE_STRIP
            m_rect.action = Marker.ADD
            m_rect.scale.x = 0.1
            if VALID_RECT_MIN_SIZE <= width <= VALID_RECT_MAX_SIZE and VALID_RECT_MIN_SIZE <= height <= VALID_RECT_MAX_SIZE:
                m_rect.color.r = 0.0
                m_rect.color.g = 1.0
                m_rect.color.b = 0.0
                m_rect.color.a = 1.0
            else:
                m_rect.color.r = 1.0
                m_rect.color.g = 0.65
                m_rect.color.b = 0.0
                m_rect.color.a = 1.0
            for (x, y) in envelope:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.0
                m_rect.points.append(p)
            p0 = Point()
            p0.x, p0.y = envelope[0]
            p0.z = 0.0
            m_rect.points.append(p0)
            ma.markers.append(m_rect)

        for i, fb in enumerate(self.found_blocks):
            center = fb['center']
            envelope = fb['envelope']
            m_fb = Marker()
            m_fb.header.frame_id = self.output_frame
            m_fb.header.stamp = rospy.Time.now()
            m_fb.ns = "found_blocks"
            m_fb.id = i
            m_fb.type = Marker.LINE_STRIP
            m_fb.action = Marker.ADD
            m_fb.scale.x = 0.1
            m_fb.color.r = 0.0
            m_fb.color.g = 0.0
            m_fb.color.b = 1.0
            m_fb.color.a = 1.0
            for (x, y) in envelope:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.0
                m_fb.points.append(p)
            p0 = Point()
            p0.x, p0.y = envelope[0]
            p0.z = 0.0
            m_fb.points.append(p0)
            m_fb.lifetime = rospy.Duration(0)
            ma.markers.append(m_fb)

        self.marker_pub.publish(ma)

    def publish_found_blocks_info(self):
        """
        仅输出 4xn 数组，每行格式为 (cube_order, x, y, number)，
        cube_order 与 number 显示为整数，x 与 y 为浮点数。
        """
        blocks_list = []
        for i, fb in enumerate(self.found_blocks, start=1):
            digit = fb.get('label', 0)
            x_val, y_val = fb['center']
            blocks_list.append((i, x_val, y_val, digit))
        arr_str = ", ".join([f"({row[0]},{row[1]:.5f},{row[2]:.5f},{row[3]})" for row in blocks_list])
        # 直接发布 4xn 数组字符串
        rospy.loginfo(arr_str)
        self.found_blocks_info_pub.publish(String(data=arr_str))

    def print_rectangles_info(self, event):
        count = len(self.last_clusters)
        info_str = f"当前显示的矩形数量: {count}"
        if count > 0:
            sizes = []
            for cluster in self.last_clusters:
                width = cluster.get('width', 0)
                height = cluster.get('height', 0)
                sizes.append(f"({width:.2f}, {height:.2f})")
            info_str += "; 矩形尺寸（宽, 高）: " + ", ".join(sizes)
        rospy.loginfo(info_str)

if __name__ == '__main__':
    try:
        LocalClusterVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
