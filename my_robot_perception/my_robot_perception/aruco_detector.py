#!/usr/bin/env python3
"""ArUco 物体检测节点（M1）。

输入:
  /camera/image_raw    sensor_msgs/Image     （Gazebo 相机，RGB 1280x960）
  /camera/camera_info  sensor_msgs/CameraInfo（相机内参，Gazebo 无畸变）
输出:
  /object_detection    my_robot_interfaces/ObjectDetection
                       (class_id + PoseStamped(camera_link_optical 系) + confidence)

检测 DICT_4X4_50 标签，PnP(CV2) 求 6DoF 位姿。
id 映射（见 test_world.sdf 的红色方块）：0 -> "red_cube"。
"""

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from my_robot_interfaces.msg import ObjectDetection
from cv_bridge import CvBridge

ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE = 0.048  # 48mm，贴在方块 +X 侧面的 ArUco 标签（占满 50mm 侧面）
ID_CLASS_MAP = {0: "red_cube"}  # id=0 -> red_cube


def rotation_matrix_to_quaternion(R):
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w)"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    # 归一化
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / n, qy / n, qz / n, qw / n


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.K = None
        self.D = None

        # 图像 QoS 可通过参数切换（默认 Reliable，避免 BestEffort 订阅匹配不稳定）
        self.declare_parameter('image_qos_reliable', True)
        reliable = self.get_parameter('image_qos_reliable').value
        self.sensor_qos = QoSProfile(
            reliability=(ReliabilityPolicy.RELIABLE if reliable
                         else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.get_logger().info(
            f'图像订阅 QoS: {"RELIABLE" if reliable else "BEST_EFFORT"}')

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, self.sensor_qos)
        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.caminfo_cb, self.sensor_qos)
        self.det_pub = self.create_publisher(
            ObjectDetection, '/object_detection', 10)

        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # 周期统计（诊断：区分"收不到图像"/"检测不到"/"发布/M9 收不到"）
        self.frame_count = 0
        self.detect_count = 0
        self.no_frame_rounds = 0   # 连续无图像的统计轮数（用于订阅自愈）
        self.create_timer(5.0, self.stats_cb)
        self.get_logger().info("ArUco detector 启动")

    def stats_cb(self):
        self.get_logger().info(
            f'[统计] 近5s: 收到图像 {self.frame_count} 帧, '
            f'检测到标签 {self.detect_count} 次')
        if self.frame_count == 0:
            # 图像订阅失效（无图像 = 不检测 = 不发布），自动重建订阅自愈
            self.no_frame_rounds += 1
            if self.no_frame_rounds >= 2:   # 连续 ~10s 无图像
                self.get_logger().warn(
                    '连续 10s 未收到图像，重建图像/内参订阅以自愈')
                try:
                    self.destroy_subscription(self.image_sub)
                    self.destroy_subscription(self.caminfo_sub)
                except Exception as e:
                    self.get_logger().warn(f'销毁旧订阅失败: {e}')
                self.image_sub = self.create_subscription(
                    Image, '/camera/image_raw', self.image_cb, self.sensor_qos)
                self.caminfo_sub = self.create_subscription(
                    CameraInfo, '/camera/camera_info',
                    self.caminfo_cb, self.sensor_qos)
                self.no_frame_rounds = 0
        else:
            self.no_frame_rounds = 0
        self.frame_count = 0
        self.detect_count = 0

    def caminfo_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        if not hasattr(self, '_k_logged'):
            self._k_logged = True
            self.get_logger().info(f"相机内参已初始化 K={self.K.tolist()}")

    def image_cb(self, msg: Image):
        # 不捕获异常：让 rclpy 打印完整堆栈（便于定位问题），先不做保护
        self._process_image(msg)

    def _process_image(self, msg: Image):
        self.frame_count += 1
        if self.K is None:
            self.get_logger().warn(
                "收到图像但相机内参未初始化（检查 /camera/camera_info 是否有数据）",
                throttle_duration_sec=3.0)
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 转换失败: {e}")
            return
        if not hasattr(self, '_img_logged'):
            self._img_logged = True
            self.get_logger().info(
                f"收到图像 {img.shape[1]}x{img.shape[0]} frame={msg.header.frame_id}")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 检测参数放宽：允许更小的标签（默认 minMarkerPerimeterRate=0.03 → 边长≥34px，
        # 距离远时标签被直接过滤导致 rejected=0；降到 0.008 允许边长~9px）
        self.aruco_params.minMarkerPerimeterRate = 0.008

        # 多尺度检测（原图 1.0 / 放大 1.5 / 放大 2.0）：
        # 标签像素不足/阈值分割失败时，放大后检测成功率更高。
        # ArUco 检测器只容忍旋转、不容忍镜像；每级再试 3 种翻转（诊断镜像）。
        scales = [1.0, 1.5, 2.0]
        flips = [
            ("normal", None),
            ("hflip", 1),
            ("vflip", 0),
            ("hvflip", -1),
        ]
        corners = ids = None
        used_scale = 1.0
        used_label = "normal"
        for scale in scales:
            g = gray if scale == 1.0 else cv2.resize(
                gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            for label, fcode in flips:
                gg = g if fcode is None else cv2.flip(g, fcode)
                c, i, _ = cv2.aruco.detectMarkers(
                    gg, self.aruco_dict, parameters=self.aruco_params)
                if i is not None and len(i) > 0:
                    corners, ids, used_scale, used_label = c, i, scale, label
                    break
            if ids is not None and len(corners) > 0:
                break
        if ids is None or len(corners) == 0:
            self.get_logger().warn(
                f"图像 {img.shape[1]}x{img.shape[0]} 中未检测到 ArUco 标签"
                f"（标签可能太小/太远/角度差/镜像）", throttle_duration_sec=2.0)
            return
        # 放大图上检测到的角点坐标要还原到原图（PnP 用原图内参）
        corners = [corner / used_scale for corner in corners]
        if used_scale != 1.0 or used_label != "normal":
            self.get_logger().info(
                f"在 scale={used_scale} 方向={used_label} 下检测到标签")

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE, self.K, self.D)

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

            pose = PoseStamped()
            pose.header = msg.header  # frame_id = camera_link_optical
            pose.pose.position.x = float(tvec[0])
            pose.pose.position.y = float(tvec[1])
            pose.pose.position.z = float(tvec[2])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            det = ObjectDetection()
            det.class_id = ID_CLASS_MAP.get(marker_id, f"marker_{marker_id}")
            det.pose = pose
            det.confidence = 1.0
            det.bbox = [0, 0, 0, 0]
            self.detect_count += 1
            try:
                self.det_pub.publish(det)
            except Exception as e:
                self.get_logger().error(f"publish 异常: {e}")
            # 诊断：确认发布执行 + 订阅者发现情况
            try:
                sub_cnt = self.det_pub.get_subscription_count()
            except Exception:
                sub_cnt = -1
            self.get_logger().info(
                f"publish 调用完成，订阅者数={sub_cnt} | "
                f"检测到 marker {marker_id} ({det.class_id}) "
                f"位姿=({tvec[0]:.3f},{tvec[1]:.3f},{tvec[2]:.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
