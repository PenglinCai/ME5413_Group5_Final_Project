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
# 1. ROI（Region Of Interest）参数：
#    定义在 map 坐标系下用于过滤聚类结果的有效区域。
REGION_MIN_X = 10.0      # map 坐标系中接受的最小 X 坐标
REGION_MAX_X = 19.5      # map 坐标系中接受的最大 X 坐标
REGION_MIN_Y = -22.0     # map 坐标系中接受的最小 Y 坐标
REGION_MAX_Y = -2.5      # map 坐标系中接受的最大 Y 坐标

# 2. 聚类（Clustering）参数：
#    用于 DBSCAN 算法的参数，决定聚类的密度和最小样本数量。
CLUSTER_EPS = 0.2        # 两个点被视为邻域内的最大距离
CLUSTER_MIN_SAMPLES = 10 # 成为核心点所需的最小邻域点数

# 3. 坐标系（Coordinate Frames）参数：
#    定义激光数据的原始坐标系及转换目标坐标系。
INPUT_FRAME = "front_laser"  # 激光雷达所在的坐标系
OUTPUT_FRAME = "map"         # 转换目标坐标系

# 4. 可视化 Marker 参数：
#    设置 Marker 发布话题以及 sphere marker 的尺寸。
MARKER_TOPIC = "/cluster_markers_map"  # Marker 发布的话题
MARKER_SCALE = 0.4                     # sphere marker 的尺寸

# 5. 合格矩形（Valid Rectangle）判定参数：
#    当矩形宽度和高度均在下面范围内时，认为该矩形合格
VALID_RECT_MIN_SIZE = 0.6   # 合格矩形的最小宽度和高度
VALID_RECT_MAX_SIZE = 0.9   # 合格矩形的最大宽度和高度

# 6. 新方块记录和融合参数：
#    如果检测到的合格绿色矩形与已记录蓝色矩形的中心距离小于 MERGE_DISTANCE_THRESHOLD，
#    则认为属于同一目标，并融合更新中心、宽度和高度。
MERGE_DISTANCE_THRESHOLD = 1.0  # 合并判断距离阈值（单位：米）

# 7. 融合权重参数（Fusion Weights）：
#    融合更新时，新旧值的权重系数。更新公式为：
#       new_value = old_value * FUSION_OLD_WEIGHT + new_value * FUSION_NEW_WEIGHT
FUSION_OLD_WEIGHT = 0.4
FUSION_NEW_WEIGHT = 0.6
# ==========================================================================

class LocalClusterVisualizer:
    def __init__(self):
        rospy.init_node('local_cluster_visualizer')

        # 将顶部参数赋值给类变量
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

        # 融合权重参数
        self.fusion_old_weight = FUSION_OLD_WEIGHT
        self.fusion_new_weight = FUSION_NEW_WEIGHT
        self.merge_distance_threshold = MERGE_DISTANCE_THRESHOLD

        # 初始化 TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅激光数据和发布 Marker
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        # 新增：发布已找到方块信息的 Publisher（发布 String 类型消息）
        self.found_blocks_info_pub = rospy.Publisher('/found_blocks_info', String, queue_size=10)

        # 存储当前扫描得到的聚类，每个元素为字典：
        # {'center': (cx, cy), 'envelope': [[x1,y1], [x2,y2], [x3,y3], [x4,y4]], 'width': w, 'height': h}
        self.last_clusters = []
        # 存储记录的“找到的方块”（合格矩形），持久显示为蓝色。
        self.found_blocks = []

        # 定时器：每秒打印一次当前矩形数量及尺寸信息（这里可选，保留简单调试）
        self.print_timer = rospy.Timer(rospy.Duration(1.0), self.print_rectangles_info)
        # 定时器：每 0.2 秒触发一次融合更新
        self.fusion_timer = rospy.Timer(rospy.Duration(0.2), self.fusion_update_callback)

        rospy.loginfo("LocalClusterVisualizer 已启动，监听 /front/scan")

    def scan_callback(self, msg):
        # 将激光数据转换为二维点（单位：激光坐标系）
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

        # 获取一次 TF 转换（从激光坐标系到 map 坐标系）
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
                continue  # 忽略噪声点
            cluster_pts = points[labels == label]
            points_map = []
            # 对聚类内的每个点进行 TF 转换到 map 坐标系
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
            # 在 map 坐标系下计算轴对齐的包络矩形
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
            # 计算矩形中心
            center_rect = [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0]
            width = max_x - min_x
            height = max_y - min_y

            # 判断矩形中心是否在设定的区域内
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
        # 这里 scan_callback 仅更新 self.last_clusters，
        # 融合更新由 fusion_timer 每 0.2 秒调用 fusion_update_callback 实现。

    def fusion_update_callback(self, event):
        """
        每 0.2 秒触发一次融合更新：
        遍历当前 self.last_clusters 中的合格矩形（绿色矩形），
        如果其中心与已有记录的蓝色矩形中心距离小于 merge_distance_threshold，则融合更新；
        否则作为新方块加入 found_blocks。
        融合公式：
            new_value = old_value * fusion_old_weight + new_value * fusion_new_weight
        同时根据更新后的中心、宽度和高度重新计算 envelope。
        """
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
                    dist = np.linalg.norm(new_center - fb_center)
                    if dist < self.merge_distance_threshold:
                        updated_center = fb_center * self.fusion_old_weight + new_center * self.fusion_new_weight
                        updated_width = fb['width'] * self.fusion_old_weight + new_width * self.fusion_new_weight
                        updated_height = fb['height'] * self.fusion_old_weight + new_height * self.fusion_new_weight
                        fb['center'] = (updated_center[0], updated_center[1])
                        fb['width'] = updated_width
                        fb['height'] = updated_height
                        new_min_x = updated_center[0] - updated_width / 2.0
                        new_max_x = updated_center[0] + updated_width / 2.0
                        new_min_y = updated_center[1] - updated_height / 2.0
                        new_max_y = updated_center[1] + updated_height / 2.0
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
        # 融合更新后发布 Marker 及已找到方块的信息
        self.publish_markers()
        self.publish_found_blocks_info()

    def publish_markers(self):
        ma = MarkerArray()

        # ===== 删除旧的当前聚类 Marker =====
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

        # ===== 添加新的当前聚类 Marker =====
        for i, cluster in enumerate(self.last_clusters):
            center = cluster['center']
            envelope = cluster['envelope']
            width = cluster['width']
            height = cluster['height']

            # 红色 sphere 显示矩形中心
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

            # LINE_STRIP 显示包络矩形（合格为绿色，不合格为橙色）
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

        # ===== 添加记录的“找到的方块” Marker（持久显示，蓝色）=====
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
        遍历 found_blocks，将每个方块的编号及中心信息组装成字符串，
        然后通过 found_blocks_info_pub 话题发布出去。
        """
        info_lines = []
        for i, fb in enumerate(self.found_blocks):
            info_lines.append(f"Block {i}: center = {fb['center']}")
        info_str = "\n".join(info_lines)
        msg = String(data=info_str)
        self.found_blocks_info_pub.publish(msg)

    def print_rectangles_info(self, event):
        # 可选的调试函数，可保留也可注释掉
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
