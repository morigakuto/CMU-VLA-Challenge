import math
import os
from pathlib import Path
from typing import Dict

import numpy as np
import rospkg
import rospy
import torch
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker

from uiap_ogn_ros.observation_bridge import ObservationBridge
from vlfm.policy.reality_policies import RealityITMPolicyV2


def _resolve_path(base: Path, maybe_path: str) -> Path:
    candidate = Path(maybe_path)
    if candidate.is_absolute():
        return candidate
    return (base / candidate).resolve()


class UiapPolicyNode:
    def __init__(self) -> None:
        rospy.init_node("uiap_policy_node")
        self.params = self._load_params()

        self.bridge = ObservationBridge(self.params)
        self.object_goal = self.params.get("default_object_goal", "chair")
        self.query_type = "find"
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.rate = rospy.Rate(float(self.params.get("publish_rate_hz", 2.0)))
        self.default_marker_height = float(self.params.get("default_marker_height", 0.6))
        self.use_traversable_guard = bool(self.params.get("use_traversable_guard", True))

        pkg_path = Path(rospkg.RosPack().get_path("uiap_ogn_ros"))
        repo_root = pkg_path.parents[3]
        uiap_root_param = self.params.get("uiap_root", "")
        self.uiap_root = Path(uiap_root_param).resolve() if uiap_root_param else (repo_root / "uiap-ogn").resolve()

        self._prepare_vlm_env()
        self.policy = self._init_policy()
        self.step_idx = 0

        self.waypoint_pub = rospy.Publisher("/way_point_with_heading", Pose2D, queue_size=2)
        self.marker_pub = rospy.Publisher("/selected_object_marker", Marker, queue_size=2)
        self.count_pub = rospy.Publisher("/numerical_response", Int32, queue_size=2)

        rospy.Subscriber("/uiap_ogn/object_goal", String, self._goal_cb, queue_size=1)
        rospy.Subscriber("/uiap_ogn/query_type", String, self._query_type_cb, queue_size=1)
        if self.params.get("question_topic"):
            rospy.Subscriber(self.params["question_topic"], String, self._question_cb, queue_size=1)

    def _load_params(self) -> Dict:
        defaults = {
            "depth_width": 224,
            "depth_height": 224,
            "hfov_deg": 90.0,
            "vfov_deg": 67.0,
            "min_depth": 0.2,
            "max_depth": 8.0,
            "camera_translation": [0.0, 0.0, 0.0],
            "camera_yaw_deg": 0.0,
            "flip_y_axis": False,
            "map_frame_id": "map",
            "odom_topic": "/state_estimation",
            "image_topic": "/camera/image",
            "pointcloud_topic": "/registered_scan",
            "traversable_topic": "/traversable_area",
            "object_markers_topic": "/object_markers",
            "uiap_root": "",
            "pointnav_weights": "data/pointnav_weights.pth",
            "text_prompt": "Seems like there is a target_object ahead.",
            "object_map_erosion": 3,
            "pointnav_stop_radius": 0.9,
            "policy_device": "auto",
            "publish_rate_hz": 2.0,
            "use_traversable_guard": True,
            "default_marker_height": 0.6,
            "question_topic": "/challenge_question",
            "default_object_goal": "chair",
        }
        params: Dict = {}
        for key, val in defaults.items():
            params[key] = rospy.get_param(f"~{key}", rospy.get_param(key, val))
        return params

    def _prepare_vlm_env(self) -> None:
        data_dir = self.uiap_root / "data"
        os.environ.setdefault("MOBILE_SAM_CHECKPOINT", str(data_dir / "mobile_sam.pt"))
        os.environ.setdefault(
            "GROUNDING_DINO_CONFIG",
            str(self.uiap_root / "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"),
        )
        os.environ.setdefault("GROUNDING_DINO_WEIGHTS", str(data_dir / "groundingdino_swint_ogc.pth"))
        os.environ.setdefault("CLASSES_PATH", str(self.uiap_root / "vlfm/vlm/classes.txt"))

    def _init_policy(self) -> RealityITMPolicyV2:
        pointnav_path = _resolve_path(self.uiap_root, self.params.get("pointnav_weights", "data/pointnav_weights.pth"))
        if not pointnav_path.exists():
            rospy.logwarn(f"[uiap_ogn_ros] pointnav weights not found at {pointnav_path}")
        device_str = "cuda" if torch.cuda.is_available() else "cpu"
        if self.params.get("policy_device", "auto") == "cpu":
            device_str = "cpu"
        elif device_str == "cpu":
            rospy.logwarn(
                "[uiap_ogn_ros] CUDA not available; RealityITMPolicyV2 expects a GPU and may fail on CPU."
            )
        torch_device = torch.device(device_str)
        rospy.loginfo(f"[uiap_ogn_ros] Using device {torch_device} for policy")
        if torch_device.type == "cuda" and torch.cuda.is_available():
            torch.cuda.set_device(torch_device.index or 0)

        policy = RealityITMPolicyV2(
            text_prompt=self.params.get("text_prompt", "Seems like there is a target_object ahead."),
            pointnav_policy_path=str(pointnav_path),
            depth_image_shape=(self.params.get("depth_height", 224), self.params.get("depth_width", 224)),
            pointnav_stop_radius=float(self.params.get("pointnav_stop_radius", 0.9)),
            object_map_erosion_size=int(self.params.get("object_map_erosion", 3)),
            visualize=False,
            compute_frontiers=True,
            min_obstacle_height=0.15,
            max_obstacle_height=0.88,
            obstacle_map_area_threshold=1.5,
            hole_area_thresh=100000,
            agent_radius=0.18,
            use_max_confidence=True,
        )
        return policy

    def _goal_cb(self, msg: String) -> None:
        self.object_goal = msg.data.strip()
        self.step_idx = 0
        rospy.loginfo(f"[uiap_ogn_ros] Set object goal to '{self.object_goal}'")

    def _query_type_cb(self, msg: String) -> None:
        self.query_type = msg.data.strip().lower()
        rospy.loginfo(f"[uiap_ogn_ros] Set query type to '{self.query_type}'")

    def _question_cb(self, msg: String) -> None:
        # Fallback parsing mirroring QuestionRouter
        text = msg.data.strip()
        lowered = text.lower()
        if lowered.startswith("how many") or "how many" in lowered:
            self.query_type = "count"
            self.object_goal = text.split("how many", 1)[-1].strip(" ?.")
        elif lowered.startswith("find") or "find the" in lowered:
            self.query_type = "find"
            self.object_goal = text.replace("find", "").strip(" ?.")
        else:
            self.query_type = "navigate"
            self.object_goal = text
        self.step_idx = 0
        rospy.loginfo(f"[uiap_ogn_ros] Direct question -> goal '{self.object_goal}' type '{self.query_type}'")

    def _publish_waypoint(self, nav_goal: np.ndarray, robot_xy: np.ndarray) -> None:
        if nav_goal is None or not np.any(nav_goal):
            return
        pose = Pose2D()
        pose.x = float(nav_goal[0])
        pose.y = float(nav_goal[1])
        heading = math.atan2(nav_goal[1] - robot_xy[1], nav_goal[0] - robot_xy[0])
        pose.theta = heading
        self.waypoint_pub.publish(pose)

    def _publish_marker(self, target_xy: np.ndarray) -> None:
        marker = Marker()
        marker.header.frame_id = self.params.get("map_frame_id", "map")
        marker.header.stamp = rospy.Time.now()
        marker.ns = "uiap_ogn"
        marker.id = 0
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.pose.position.x = float(target_xy[0])
        marker.pose.position.y = float(target_xy[1])
        marker.pose.position.z = self.default_marker_height
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.9
        self.marker_pub.publish(marker)

    def _publish_count(self, cloud: np.ndarray) -> None:
        if cloud.size == 0:
            count = 0
        else:
            # Heuristic: cluster ids are stored in the last column by ObjectPointCloudMap
            cluster_ids = np.unique(np.round(cloud[:, -1], decimals=2))
            count = int(len(cluster_ids))
        self.count_pub.publish(Int32(data=count))

    def spin(self) -> None:
        while not rospy.is_shutdown():
            observations = self.bridge.build_observations()
            if observations is None:
                self.rate.sleep()
                continue

            observations["objectgoal"] = self.object_goal
            mask_val = 0 if self.step_idx == 0 else 1
            masks = torch.tensor([[mask_val]], dtype=torch.bool, device=self.device)

            try:
                action_dict = self.policy.get_action(observations, masks, deterministic=True)
            except StopIteration:
                self.step_idx = 0
                continue
            except Exception as exc:  # noqa: BLE001
                rospy.logerr_throttle(1.0, f"[uiap_ogn_ros] Policy step failed: {exc}")
                self.rate.sleep()
                continue

            info = action_dict.get("info", {})
            nav_goal = np.array(info.get("nav_goal", np.zeros(2)), dtype=np.float32)
            self._publish_waypoint(nav_goal, observations["robot_xy"])

            if self.query_type == "find" and self.policy._object_map.has_object(self.object_goal):
                target_xy = self.policy._object_map.get_best_object(self.object_goal, observations["robot_xy"])
                self._publish_marker(target_xy)
            if self.query_type == "count" and self.policy._object_map.has_object(self.object_goal):
                cloud = self.policy._object_map.get_target_cloud(self.object_goal)
                self._publish_count(cloud)

            self.step_idx += 1
            self.rate.sleep()


def main() -> None:
    node = UiapPolicyNode()
    node.spin()


if __name__ == "__main__":
    main()
