import math
from typing import Dict, Optional, Tuple

import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from tf.transformations import euler_from_quaternion

from vlfm.utils.geometry_utils import get_fov, wrap_heading, xyz_yaw_to_tf_matrix


class ObservationBridge:
    """
    Converts ROS topics into the observation dictionary expected by RealityITMPolicyV2.
    """

    def __init__(self, params: Dict) -> None:
        self._params = params
        self._bridge = CvBridge()

        self.depth_width = int(params.get("depth_width", 224))
        self.depth_height = int(params.get("depth_height", 224))
        self.min_depth = float(params.get("min_depth", 0.2))
        self.max_depth = float(params.get("max_depth", 8.0))
        self.hfov = math.radians(float(params.get("hfov_deg", 90.0)))
        self.vfov = math.radians(float(params.get("vfov_deg", 67.0)))
        self.camera_translation = np.array(params.get("camera_translation", [0.0, 0.0, 0.0]), dtype=np.float32)
        self.camera_yaw_offset = math.radians(float(params.get("camera_yaw_deg", 0.0)))
        self.flip_y_axis = bool(params.get("flip_y_axis", False))

        self.fx = self.depth_width / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.depth_height / (2.0 * math.tan(self.vfov / 2.0))
        self.cx = self.depth_width / 2.0
        self.cy = self.depth_height / 2.0
        self.topdown_fov = get_fov(self.fx, self.depth_width)

        self._image: Optional[np.ndarray] = None
        self._point_cloud: Optional[np.ndarray] = None
        self._last_depth: Optional[np.ndarray] = None
        self._odom_msg: Optional[Odometry] = None
        self._start_xy: Optional[np.ndarray] = None
        self._start_yaw: Optional[float] = None

        self._tf_camera_to_map: Optional[np.ndarray] = None
        self._tf_map_to_camera: Optional[np.ndarray] = None

        self.image_sub = rospy.Subscriber(
            params.get("image_topic", "/camera/image"), Image, self._image_cb, queue_size=1
        )
        self.cloud_sub = rospy.Subscriber(
            params.get("pointcloud_topic", "/registered_scan"), PointCloud2, self._cloud_cb, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            params.get("odom_topic", "/state_estimation"), Odometry, self._odom_cb, queue_size=10
        )

    def ready(self) -> bool:
        return self._image is not None and self._point_cloud is not None and self._odom_msg is not None

    def _image_cb(self, msg: Image) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self._image = np.asarray(cv_image)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr_throttle(1.0, f"[uiap_ogn_ros] Failed to convert image: {exc}")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        try:
            points = np.asarray(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            self._point_cloud = points.astype(np.float32)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr_throttle(1.0, f"[uiap_ogn_ros] Failed to read point cloud: {exc}")

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_msg = msg
        quat = msg.pose.pose.orientation
        yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2] + self.camera_yaw_offset

        if self._start_xy is None:
            self._start_xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=np.float32)
            self._start_yaw = yaw

        xyz = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=np.float32
        )
        self._tf_camera_to_map, self._tf_map_to_camera = self._compute_tfs(xyz, yaw)

    def _compute_tfs(self, base_xyz: np.ndarray, yaw: float) -> Tuple[np.ndarray, np.ndarray]:
        translation_rotated = np.array(
            [
                math.cos(yaw) * self.camera_translation[0] - math.sin(yaw) * self.camera_translation[1],
                math.sin(yaw) * self.camera_translation[0] + math.cos(yaw) * self.camera_translation[1],
                self.camera_translation[2],
            ],
            dtype=np.float32,
        )
        camera_xyz = base_xyz + translation_rotated
        tf_camera_to_map = xyz_yaw_to_tf_matrix(camera_xyz, yaw)
        tf_map_to_camera = np.linalg.inv(tf_camera_to_map)
        return tf_camera_to_map, tf_map_to_camera

    def _robot_pose(self) -> Tuple[np.ndarray, float]:
        if self._odom_msg is None or self._start_xy is None or self._start_yaw is None:
            return np.zeros(2), 0.0
        pos = np.array([self._odom_msg.pose.pose.position.x, self._odom_msg.pose.pose.position.y], dtype=np.float32)
        delta = pos - self._start_xy
        c, s = math.cos(-self._start_yaw), math.sin(-self._start_yaw)
        rot = np.array([[c, -s], [s, c]], dtype=np.float32)
        local = rot @ delta
        if self.flip_y_axis:
            local[1] *= -1.0
        quat = self._odom_msg.pose.pose.orientation
        heading = wrap_heading(euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2] - self._start_yaw)
        if self.flip_y_axis:
            heading = wrap_heading(-heading)
        return local, heading

    def _project_depth(self) -> Optional[np.ndarray]:
        if self._point_cloud is None or self._tf_map_to_camera is None:
            return None
        points = self._point_cloud
        ones = np.ones((points.shape[0], 1), dtype=np.float32)
        points_h = np.hstack((points[:, :3], ones))
        cam_points = (self._tf_map_to_camera @ points_h.T).T[:, :3]

        valid_mask = cam_points[:, 0] > self.min_depth
        cam_points = cam_points[valid_mask]
        if cam_points.shape[0] == 0:
            return None

        z = cam_points[:, 0]
        u = (-cam_points[:, 1] * self.fx / z) + self.cx
        v = (-cam_points[:, 2] * self.fy / z) + self.cy
        in_view = (
            (u >= 0)
            & (u < self.depth_width)
            & (v >= 0)
            & (v < self.depth_height)
            & (z <= self.max_depth)
        )
        if not np.any(in_view):
            return None

        u_idx = u[in_view].astype(np.int32)
        v_idx = v[in_view].astype(np.int32)
        depth = np.full((self.depth_height, self.depth_width), self.max_depth, dtype=np.float32)
        np.minimum.at(depth, (v_idx, u_idx), z[in_view])
        self._last_depth = depth
        norm_depth = np.clip((depth - self.min_depth) / (self.max_depth - self.min_depth), 0.0, 1.0)
        return norm_depth

    def build_observations(self) -> Optional[Dict]:
        if not self.ready():
            return None

        depth = self._project_depth()
        if depth is None:
            return None

        robot_xy, robot_heading = self._robot_pose()
        tf_camera_to_map = self._tf_camera_to_map
        if tf_camera_to_map is None:
            return None

        obstacle_map_depths = [
            (depth, tf_camera_to_map, self.min_depth, self.max_depth, self.fx, self.fy, self.topdown_fov),
            (None, tf_camera_to_map, self.min_depth, self.max_depth, self.fx, self.fy, self.topdown_fov),
        ]

        observations = {
            "nav_depth": depth,
            "robot_xy": robot_xy,
            "robot_heading": robot_heading,
            "obstacle_map_depths": obstacle_map_depths,
            "value_map_rgbd": [
                (self._image, depth, tf_camera_to_map, self.min_depth, self.max_depth, self.hfov),
            ],
            "object_map_rgbd": [
                (self._image, depth, tf_camera_to_map, self.min_depth, self.max_depth, self.fx, self.fy),
            ],
        }
        return observations
