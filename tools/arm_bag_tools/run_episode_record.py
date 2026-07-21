#!/usr/bin/env python3
"""
Target 抓取（robotic_arm_cmd type=0, frame_id=odom）+ 插孔（type=1, odom catalog）episode 自动执行并录制 rosbag2。

依赖（需先 source ROS2 + 工作空间，含 sealien_ctrlpilot_msgmanagement）:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source <ws>/install/setup.bash

用法:
  ./run_episode_record.sh --episode-id ep_target_001
  ./run_episode_record.sh --count 5 --prefix ep_target
  ./run_episode_record.sh --sample-type pick_fail --episode-id ep_target_pick_fail_001
  ./run_episode_record.sh --sample-type insert_fail --episode-id ep_target_insert_fail_001
  ./run_episode_record.sh --failure-set --prefix ep_target

全栈 launch 须已运行；HoloOcean + MTC 就绪且感知话题有数据。
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

SCRIPT_DIR = Path(__file__).resolve().parent
CATALOG_FILENAME = "manipulator_task_keypoints_odom.yaml"
CATALOG_REL_PATH = Path("config") / CATALOG_FILENAME


def resolve_keypoints_catalog_path(explicit: Optional[str] = None) -> Optional[Path]:
    """
    定位 odom keypoints catalog：
    1) --keypoints-catalog 显式路径
    2) 自脚本目录向上搜索 config/manipulator_task_keypoints_odom.yaml
    3) tools/arm_bag_tools/config/ 内置副本
    """
    candidates: List[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    for parent in SCRIPT_DIR.parents:
        candidates.append(parent / CATALOG_REL_PATH)
    candidates.append(SCRIPT_DIR / CATALOG_REL_PATH)
    seen: set[str] = set()
    for path in candidates:
        try:
            key = str(path.resolve())
        except OSError:
            key = str(path)
        if key in seen:
            continue
        seen.add(key)
        if path.is_file():
            return path
    return None


def default_keypoints_catalog_path() -> Path:
    found = resolve_keypoints_catalog_path(None)
    if found is not None:
        return found
    if len(SCRIPT_DIR.parents) >= 3:
        return SCRIPT_DIR.parents[2] / CATALOG_REL_PATH
    return SCRIPT_DIR / CATALOG_REL_PATH


DEFAULT_KEYPOINTS_CATALOG = default_keypoints_catalog_path()
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

try:
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
    from holoocean_interfaces.msg import TargetSensor
    from sealien_ctrlpilot_manipulator_orion_mtc_msgs.msg import TargetSet
    from sealien_ctrlpilot_manipulator_orion_mtc_msgs.srv import GetRobotState, ResetHeldObject
    from sealien_ctrlpilot_msgmanagement.action import RoboticArmCmd
    from sealien_ctrlpilot_msgmanagement.msg import RoboticArmRequest
    from std_srvs.srv import Trigger
    from tf2_geometry_msgs import do_transform_pose
    from tf2_ros import Buffer, TransformListener
    from manipulator_keypoints_odom import (
        DEFAULT_KEYPOINT_FRAME,
        compute_grasp_keypoint_odom_from_target_sensor,
        compute_grasp_reach_arm_base_link_m,
        get_insert_keypoint_odom,
        load_keypoints_catalog,
        transform_pose_arm_base_link_to_odom,
    )
except ImportError as exc:
    print(
        "导入失败: %s\n请 source ROS2 与工作空间（含 sealien_ctrlpilot_msgmanagement、"
        "sealien_ctrlpilot_manipulator_orion_mtc_msgs）。" % exc,
        file=sys.stderr,
    )
    sys.exit(1)


REQUEST_TYPE_GRASP = 0
REQUEST_TYPE_INSERT = 1

RESULT_SUCCESS = 0
RESULT_EXEC_FAILED = 1
RESULT_REJECTED_STATE = 2
RESULT_REJECTED_NO_HELD = 3

# 与 MTC workspace 硬上限一致（sealien_ctrlpilot_manipulator_orion_mtc_params / handlePick）
DEFAULT_MAX_GRASP_REACH_M = 1.9
DEFAULT_KEYPOINT_FRAME_ID = DEFAULT_KEYPOINT_FRAME
DEFAULT_TARGET_SENSOR_TOPIC = "/holoocean/rov0/TargetSensor"
DEFAULT_ROV_POSE_TOPIC = "/holoocean/rov0/PoseSensor"
DEFAULT_LEFT_ARM_BASE_IN_ROV = (1.55, 0.5653, -0.283628)
DEFAULT_TARGET_SET_TOPIC = "/manipulator/target_set"
DEFAULT_INSERT_HOLES_TOPIC = "/manipulator/target_insert_holes"
GRASP_KEYPOINT_SOURCE_TARGET_SET = "target_set"
GRASP_KEYPOINT_SOURCE_TARGET_SENSOR = "target_sensor"
INSERT_KEYPOINT_SOURCE_HOLES = "insert_holes"
INSERT_KEYPOINT_SOURCE_CATALOG = "catalog"
PLANNING_FRAME_ID = "arm_base_link"
TF_TARGET_FRAME = "sensor_left_roboticarm"

SAMPLE_TYPE_SUCCESS = "success"
SAMPLE_TYPE_PICK_FAIL = "pick_fail"
SAMPLE_TYPE_INSERT_FAIL = "insert_fail"

# arm_base_link 下故意偏置，用于合成失败样例（可按现场调大/调小）
DEFAULT_PICK_FAIL_OFFSET_XYZ = [0.12, 0.12, 0.05]
DEFAULT_INSERT_FAIL_OFFSET_XYZ = [0.10, 0.0, 0.0]

# 与 record_arm_chain.sh JOB_TOPICS 保持一致
JOB_BAG_TOPICS: List[str] = [
    "/joint_states",
    "/holoocean/rov0/ArmSensor",
    "/holoocean/command/agent/arm",
    "/arm_controller/follow_joint_trajectory/_action/feedback",
    "/arm_controller/follow_joint_trajectory/_action/status",
    "/hand_controller/follow_joint_trajectory/_action/feedback",
    "/hand_controller/follow_joint_trajectory/_action/status",
    "/manipulator/job_event",
    "/manipulator/task_stage",
    "/manipulator/runtime_status",
    "/manipulator/held_object_state",
    "/manipulator/left_arm_gripped",
    "/holoocean/rov0/TargetSensor",
    "/manipulator/target_set",
    "/manipulator/object_pose",
    "/manipulator/object_axis",
    "/manipulator/perception_state",
    "/manipulator/target_insert_holes",
    "/manipulator/insert_rel_state",
    "/holoocean/rov0/PoseSensor",
    "/holoocean/rov0/VelocitySensor",
    "/holoocean/rov0/IMUSensor",
    "/holoocean/rov0/DepthSensor",
    "/holoocean/rov0/DVLSensorVelocity",
    "/manipulator/tf",
    "/manipulator/tf_static",
]


def iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat()


def pose_reach_m(pose: Pose) -> float:
    x = float(pose.position.x)
    y = float(pose.position.y)
    z = float(pose.position.z)
    return math.sqrt(x * x + y * y + z * z)


def normalize_frame_id(frame_id: str) -> str:
    frame = (frame_id or "").strip()
    if frame.startswith("/"):
        frame = frame[1:]
    return frame or DEFAULT_KEYPOINT_FRAME_ID


def is_valid_grasp_pose_arm_base_link(pose: Pose, max_reach_m: float) -> bool:
    reach = pose_reach_m(pose)
    if not math.isfinite(reach):
        return False
    return reach <= max_reach_m


def is_valid_grasp_keypoint_odom(
    pose: Pose,
    frame_id: str,
    target_msg: TargetSensor,
    grasp_index: int,
    grasp_offset_m: float,
    rov_position: Sequence[float],
    rov_orientation_xyzw: Sequence[float],
    max_reach_m: float,
    t_arm_in_rov: Sequence[float],
) -> bool:
    frame = normalize_frame_id(frame_id)
    if frame != DEFAULT_KEYPOINT_FRAME_ID:
        return False
    if not all(math.isfinite(float(v)) for v in (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )):
        return False
    reach = compute_grasp_reach_arm_base_link_m(
        int(target_msg.num_targets),
        list(target_msg.positions),
        list(target_msg.directions),
        grasp_index,
        grasp_offset_m,
        rov_position,
        rov_orientation_xyzw,
        t_arm_in_rov,
    )
    if not math.isfinite(reach):
        return False
    return reach <= max_reach_m


def copy_pose(pose: Pose) -> Pose:
    out = Pose()
    out.position.x = float(pose.position.x)
    out.position.y = float(pose.position.y)
    out.position.z = float(pose.position.z)
    out.orientation.x = float(pose.orientation.x)
    out.orientation.y = float(pose.orientation.y)
    out.orientation.z = float(pose.orientation.z)
    out.orientation.w = float(pose.orientation.w)
    return out


def apply_position_offset(pose: Pose, offset_xyz: Sequence[float]) -> Pose:
    out = copy_pose(pose)
    out.position.x += float(offset_xyz[0])
    out.position.y += float(offset_xyz[1])
    out.position.z += float(offset_xyz[2])
    return out


def resolve_grasp_offset(args: argparse.Namespace) -> Optional[List[float]]:
    if args.grasp_offset_xyz is not None:
        return [float(x) for x in args.grasp_offset_xyz]
    if args.sample_type == SAMPLE_TYPE_PICK_FAIL:
        return DEFAULT_PICK_FAIL_OFFSET_XYZ.copy()
    return None


def resolve_insert_offset(args: argparse.Namespace) -> Optional[List[float]]:
    if args.insert_offset_xyz is not None:
        return [float(x) for x in args.insert_offset_xyz]
    if args.sample_type == SAMPLE_TYPE_INSERT_FAIL:
        return DEFAULT_INSERT_FAIL_OFFSET_XYZ.copy()
    return None


def effective_pick_only(args: argparse.Namespace) -> bool:
    if args.pick_only:
        return True
    return args.sample_type == SAMPLE_TYPE_PICK_FAIL


def build_failure_injection_meta(
    args: argparse.Namespace,
    grasp_offset: Optional[List[float]],
    insert_offset: Optional[List[float]],
    nominal_grasp: Optional[Pose],
    nominal_insert: Optional[Pose],
    sent_grasp: Pose,
    sent_insert: Optional[Pose],
    grasp_frame: str,
    insert_frame: str,
) -> Dict[str, Any]:
    meta: Dict[str, Any] = {"sample_type": args.sample_type}
    if grasp_offset is not None:
        meta["grasp_offset_xyz"] = grasp_offset
        if nominal_grasp is not None:
            meta["grasp_pose_nominal"] = pose_to_dict(nominal_grasp, grasp_frame)
        meta["grasp_pose_sent"] = pose_to_dict(sent_grasp, grasp_frame)
    if insert_offset is not None and sent_insert is not None:
        meta["insert_offset_xyz"] = insert_offset
        if nominal_insert is not None:
            meta["insert_pose_nominal"] = pose_to_dict(nominal_insert, insert_frame)
        meta["insert_pose_sent"] = pose_to_dict(sent_insert, insert_frame)
    return meta


def pose_to_dict(pose: Pose, frame: str) -> Dict[str, Any]:
    return {
        "frame": frame,
        "xyz": [pose.position.x, pose.position.y, pose.position.z],
        "quaternion_xyzw": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    }


def result_code_to_outcome(code: int) -> str:
    if code == RESULT_SUCCESS:
        return "SUCCEEDED"
    if code in (RESULT_REJECTED_STATE, RESULT_REJECTED_NO_HELD):
        return "REJECTED"
    return "FAILED"


def result_code_to_reason(code: int, phase: str) -> str:
    if code == RESULT_SUCCESS:
        if phase == "PICK":
            return "grip_confirmed"
        return "trajectory_completed"
    if code == RESULT_REJECTED_NO_HELD:
        return "no_held_object"
    if code == RESULT_REJECTED_STATE:
        return "rejected_state"
    return "exec_failed"


@dataclass
class JobRecord:
    episode_id: str
    episode_outcome: str
    bag_dir: str
    job_id: str
    task_name: str
    robotic_arm_cmd_type: int
    sequence_in_episode: int
    started_timestamp: str
    finished_timestamp: str
    outcome: str
    termination_reason: str
    grasp_source: Optional[str] = "TARGET_SENSOR"
    action_result_code: int = 0
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> str:
        row: Dict[str, Any] = {
            "episode_id": self.episode_id,
            "episode_outcome": self.episode_outcome,
            "bag_dir": self.bag_dir,
            "job_id": self.job_id,
            "task_name": self.task_name,
            "robotic_arm_cmd_type": self.robotic_arm_cmd_type,
            "grasp_source": self.grasp_source,
            "sequence_in_episode": self.sequence_in_episode,
            "started_timestamp": self.started_timestamp,
            "finished_timestamp": self.finished_timestamp,
            "outcome": self.outcome,
            "termination_reason": self.termination_reason,
            "action_result_code": self.action_result_code,
            "control_mode": "joint_position",
            "action_definition": {
                "action": "/manipulator/robotic_arm_cmd",
                "type": "sealien_ctrlpilot_msgmanagement/RoboticArmCmd",
                "units": "deg",
            },
        }
        row.update(self.extra)
        return json.dumps(row, ensure_ascii=False)


class EpisodeRecordNode(Node):
    """
    默认与网页 submit_job 同源：target_set / target_insert_holes（arm_base_link）
    经 TF/ROV 链变到 odom 后下发 robotic_arm_cmd。
    """

    def __init__(
        self,
        catalog: Dict[str, Any],
        target_set_topic: str,
        target_sensor_topic: str,
        rov_pose_topic: str,
        insert_holes_topic: str,
        grasp_index: int,
        max_grasp_reach_m: float,
        insert_index: int,
        action_name: str,
        grasp_keypoint_source: str,
        insert_keypoint_source: str,
        t_arm_in_rov: Sequence[float] = DEFAULT_LEFT_ARM_BASE_IN_ROV,
    ) -> None:
        super().__init__("episode_record_runner")
        self._catalog = catalog
        self._keypoint_frame = str(catalog.get("keypoint_frame", DEFAULT_KEYPOINT_FRAME_ID))
        self._grasp_offset_m = float(catalog.get("grasp_offset_along_direction_m", 0.122671))
        self._target_set_topic = target_set_topic
        self._target_sensor_topic = target_sensor_topic
        self._rov_pose_topic = rov_pose_topic
        self._insert_holes_topic = insert_holes_topic
        self._grasp_keypoint_source = grasp_keypoint_source
        self._insert_keypoint_source = insert_keypoint_source
        self._grasp_index = grasp_index
        self._max_grasp_reach_m = max_grasp_reach_m
        self._insert_index = insert_index
        self._t_arm_in_rov = tuple(float(x) for x in t_arm_in_rov)
        self._latest_target_set: Optional[TargetSet] = None
        self._latest_target_sensor: Optional[TargetSensor] = None
        self._latest_holes: Optional[PoseArray] = None
        self._rov_position: Optional[Tuple[float, float, float]] = None
        self._rov_orientation_xyzw: Optional[Tuple[float, float, float, float]] = None
        self._last_side_grasp_y: Optional[Any] = None

        self._tf_buffer = Buffer(self.get_clock())
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            TargetSet,
            target_set_topic,
            self._on_target_set,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TargetSensor,
            target_sensor_topic,
            self._on_target_sensor,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            insert_holes_topic,
            self._on_insert_holes,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            rov_pose_topic,
            self._on_rov_pose,
            qos_profile_sensor_data,
        )

        self._action_name = action_name
        self._action_client = ActionClient(self, RoboticArmCmd, action_name)
        self._get_state_cli = self.create_client(GetRobotState, "/manipulator/get_robot_state")
        self._reset_held_cli = self.create_client(ResetHeldObject, "/manipulator/reset_held_object")
        self._go_ready_cli = self.create_client(Trigger, "/manipulator/go_to_ready")

    def _on_target_set(self, msg: TargetSet) -> None:
        self._latest_target_set = msg

    def _on_target_sensor(self, msg: TargetSensor) -> None:
        self._latest_target_sensor = msg

    def _on_insert_holes(self, msg: PoseArray) -> None:
        self._latest_holes = msg

    def _on_rov_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._rov_position = (float(p.x), float(p.y), float(p.z))
        self._rov_orientation_xyzw = (float(o.x), float(o.y), float(o.z), float(o.w))

    def _transform_pose_to_odom(self, pose: Pose, source_frame: str) -> Optional[PoseStamped]:
        frame = normalize_frame_id(source_frame) or PLANNING_FRAME_ID
        stamped = PoseStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = frame
        stamped.pose = copy_pose(pose)
        lookup_frames = [frame, PLANNING_FRAME_ID, TF_TARGET_FRAME]
        seen: set[str] = set()
        for src in lookup_frames:
            if src in seen:
                continue
            seen.add(src)
            stamped.header.frame_id = src
            try:
                transform = self._tf_buffer.lookup_transform(
                    DEFAULT_KEYPOINT_FRAME_ID,
                    src,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                out = do_transform_pose(stamped, transform)
                out.header.frame_id = DEFAULT_KEYPOINT_FRAME_ID
                return out
            except Exception:
                continue
        if self._rov_position is None or self._rov_orientation_xyzw is None:
            return None
        if frame not in (PLANNING_FRAME_ID, TF_TARGET_FRAME, "base_link"):
            return None
        out_pose = transform_pose_arm_base_link_to_odom(
            pose,
            self._rov_position,
            self._rov_orientation_xyzw,
            self._t_arm_in_rov,
        )
        out = PoseStamped()
        out.header.stamp = stamped.header.stamp
        out.header.frame_id = DEFAULT_KEYPOINT_FRAME_ID
        out.pose = out_pose
        return out

    def _grasp_from_target_set(self) -> Optional[Tuple[PoseStamped, str, float]]:
        ts = self._latest_target_set
        if ts is None or len(ts.targets) <= self._grasp_index:
            return None
        candidate = ts.targets[self._grasp_index]
        pose_base = copy_pose(candidate.pose)
        if not is_valid_grasp_pose_arm_base_link(pose_base, self._max_grasp_reach_m):
            return None
        source_frame = normalize_frame_id(candidate.header.frame_id) or normalize_frame_id(ts.header.frame_id)
        odom_stamped = self._transform_pose_to_odom(pose_base, source_frame)
        if odom_stamped is None:
            return None
        object_id = "target_%d" % self._grasp_index
        if len(ts.object_ids) > self._grasp_index and ts.object_ids[self._grasp_index]:
            object_id = str(ts.object_ids[self._grasp_index])
        reach = pose_reach_m(pose_base)
        return odom_stamped, object_id, reach

    def _grasp_from_target_sensor(self) -> Optional[Tuple[PoseStamped, str, float]]:
        ts = self._latest_target_sensor
        if ts is None or self._rov_position is None or self._rov_orientation_xyzw is None:
            return None
        if ts.num_targets <= self._grasp_index:
            return None
        try:
            pose, frame, y_hint = compute_grasp_keypoint_odom_from_target_sensor(
                int(ts.num_targets),
                list(ts.positions),
                list(ts.directions),
                self._grasp_index,
                self._grasp_offset_m,
                self._rov_position,
                self._rov_orientation_xyzw,
                self._t_arm_in_rov,
                last_side_grasp_y=self._last_side_grasp_y,
            )
        except (IndexError, ValueError):
            return None
        self._last_side_grasp_y = y_hint
        if not is_valid_grasp_keypoint_odom(
            pose,
            frame,
            ts,
            self._grasp_index,
            self._grasp_offset_m,
            self._rov_position,
            self._rov_orientation_xyzw,
            self._max_grasp_reach_m,
            self._t_arm_in_rov,
        ):
            return None
        stamped = PoseStamped()
        stamped.header.frame_id = frame
        stamped.pose = pose
        object_id = "target_%d" % self._grasp_index
        reach = compute_grasp_reach_arm_base_link_m(
            int(ts.num_targets),
            list(ts.positions),
            list(ts.directions),
            self._grasp_index,
            self._grasp_offset_m,
            self._rov_position,
            self._rov_orientation_xyzw,
            self._t_arm_in_rov,
        )
        return stamped, object_id, reach

    def _try_grasp_keypoint(self) -> Optional[Tuple[PoseStamped, str, float]]:
        if self._grasp_keypoint_source == GRASP_KEYPOINT_SOURCE_TARGET_SET:
            grasp = self._grasp_from_target_set()
            if grasp is not None:
                return grasp
            return self._grasp_from_target_sensor()
        grasp = self._grasp_from_target_sensor()
        if grasp is not None:
            return grasp
        return self._grasp_from_target_set()

    def wait_for_server(self, timeout_sec: float) -> bool:
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def wait_for_grasp_pose(self, timeout_sec: float) -> Tuple[PoseStamped, str]:
        deadline = time.monotonic() + timeout_sec
        last_invalid_reach: Optional[float] = None
        while rclpy.ok() and time.monotonic() < deadline:
            grasp = self._try_grasp_keypoint()
            if grasp is not None:
                stamped, object_id, reach = grasp
                return stamped, object_id
            ts = self._latest_target_set
            if ts is not None and len(ts.targets) > self._grasp_index:
                last_invalid_reach = pose_reach_m(ts.targets[self._grasp_index].pose)
            rclpy.spin_once(self, timeout_sec=0.2)
        detail = (
            "grasp_source=%s topic_set=%s topic_sensor=%s index=%d max_reach=%.3fm"
            % (
                self._grasp_keypoint_source,
                self._target_set_topic,
                self._target_sensor_topic,
                self._grasp_index,
                self._max_grasp_reach_m,
            )
        )
        if last_invalid_reach is not None:
            detail += " last_reach=%.3fm" % last_invalid_reach
        if self._rov_position is None:
            detail += " rov_pose=missing(%s)" % self._rov_pose_topic
        raise TimeoutError("等待有效 odom 抓取 keypoint 超时: %s" % detail)

    def get_insert_keypoint_catalog(self) -> Tuple[Pose, str, str]:
        pose, frame, slot_id = get_insert_keypoint_odom(self._catalog, self._insert_index)
        return pose, frame, slot_id

    def _insert_from_holes(self) -> Optional[Tuple[Pose, str, str]]:
        holes = self._latest_holes
        if holes is None or len(holes.poses) <= self._insert_index:
            return None
        pose_base = copy_pose(holes.poses[self._insert_index])
        source_frame = normalize_frame_id(holes.header.frame_id) or PLANNING_FRAME_ID
        odom_stamped = self._transform_pose_to_odom(pose_base, source_frame)
        if odom_stamped is None:
            return None
        slot_id = "hole_slot_%d" % self._insert_index
        slots = self._catalog.get("insert_slots") or []
        if self._insert_index < len(slots):
            slot_id = str(slots[self._insert_index].get("id", slot_id))
        return odom_stamped.pose, DEFAULT_KEYPOINT_FRAME_ID, slot_id

    def wait_for_insert_keypoint(self, timeout_sec: float) -> Tuple[Pose, str, str]:
        if self._insert_keypoint_source == INSERT_KEYPOINT_SOURCE_CATALOG:
            return self.get_insert_keypoint_catalog()
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            insert = self._insert_from_holes()
            if insert is not None:
                return insert
            rclpy.spin_once(self, timeout_sec=0.2)
        raise TimeoutError(
            "等待插孔 keypoint 超时: source=%s topic=%s index=%d"
            % (self._insert_keypoint_source, self._insert_holes_topic, self._insert_index)
        )

    def get_insert_keypoint(self) -> Tuple[Pose, str, str]:
        if self._insert_keypoint_source == INSERT_KEYPOINT_SOURCE_CATALOG:
            return self.get_insert_keypoint_catalog()
        insert = self._insert_from_holes()
        if insert is not None:
            return insert
        return self.get_insert_keypoint_catalog()

    def get_task_id(self) -> str:
        if not self._get_state_cli.wait_for_service(timeout_sec=5.0):
            return ""
        req = GetRobotState.Request()
        future = self._get_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            return ""
        return future.result().task_id

    def reset_between_episodes(self) -> None:
        if self._reset_held_cli.wait_for_service(timeout_sec=3.0):
            fut = self._reset_held_cli.call_async(ResetHeldObject.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if self._go_ready_cli.wait_for_service(timeout_sec=3.0):
            fut = self._go_ready_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=120.0)

    def send_robotic_arm_cmd(
        self,
        request_type: int,
        keypoint: Pose,
        frame_id: str,
        timeout_sec: float,
    ) -> Tuple[bool, int]:
        if not self.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("robotic_arm_cmd Action 不可用: %s" % self._action_name)

        goal = RoboticArmCmd.Goal()
        goal.order = RoboticArmRequest()
        goal.order.type = int(request_type)
        goal.order.header.stamp = self.get_clock().now().to_msg()
        goal.order.header.frame_id = frame_id
        goal.order.keypoint = keypoint

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        if not send_future.done():
            raise TimeoutError("发送 Action goal 超时")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, RESULT_REJECTED_STATE

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            raise TimeoutError("Action 执行超时 (type=%d)" % request_type)
        wrapped = result_future.result()
        if wrapped is None:
            return False, RESULT_EXEC_FAILED
        return wrapped.result.result == RESULT_SUCCESS, int(wrapped.result.result)


class BagRecorder:
    """后台 ros2 bag record 子进程。"""

    def __init__(self, out_dir: Path) -> None:
        self._out_dir = out_dir
        self._proc: Optional[subprocess.Popen] = None

    def start(self) -> None:
        self._out_dir.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            "ros2",
            "bag",
            "record",
            "--include-hidden-topics",
            "-o",
            str(self._out_dir),
        ] + JOB_BAG_TOPICS
        self._proc = subprocess.Popen(cmd)
        time.sleep(1.0)

    def stop(self) -> int:
        if self._proc is None:
            return 0
        if self._proc.poll() is not None:
            return int(self._proc.returncode or 0)
        self._proc.send_signal(signal.SIGINT)
        try:
            return int(self._proc.wait(timeout=45))
        except subprocess.TimeoutExpired:
            self._proc.kill()
            return 1


def append_jobs_jsonl(path: Path, rows: List[JobRecord]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        for row in rows:
            f.write(row.to_json() + "\n")


def run_single_episode(
    node: EpisodeRecordNode,
    episode_id: str,
    bag_dir: Path,
    jobs_path: Path,
    args: argparse.Namespace,
) -> str:
    """执行一条 episode，返回 episode_outcome 字符串。"""
    node.get_logger().info("episode %s: 等待 TargetSensor odom keypoint..." % episode_id)
    grasp_stamped, grasp_object_id = node.wait_for_grasp_pose(args.wait_pose_sec)
    grasp_frame = grasp_stamped.header.frame_id or DEFAULT_KEYPOINT_FRAME_ID
    grasp_reach_m = 0.0
    if node._latest_target_sensor and node._rov_position and node._rov_orientation_xyzw:
        grasp_reach_m = compute_grasp_reach_arm_base_link_m(
            int(node._latest_target_sensor.num_targets),
            list(node._latest_target_sensor.positions),
            list(node._latest_target_sensor.directions),
            args.grasp_index,
            node._grasp_offset_m,
            node._rov_position,
            node._rov_orientation_xyzw,
            node._t_arm_in_rov,
        )
    node.get_logger().info(
        "episode %s: 抓取 %s reach=%.3fm frame=%s pos=(%.3f, %.3f, %.3f)"
        % (
            episode_id,
            grasp_object_id,
            grasp_reach_m,
            grasp_frame,
            grasp_stamped.pose.position.x,
            grasp_stamped.pose.position.y,
            grasp_stamped.pose.position.z,
        )
    )
    insert_pose: Optional[Pose] = None
    insert_frame = DEFAULT_KEYPOINT_FRAME_ID
    insert_slot_id = "hole_slot_%d" % args.insert_index
    pick_only = effective_pick_only(args)
    if not pick_only:
        insert_pose, insert_frame, insert_slot_id = node.wait_for_insert_keypoint(args.wait_pose_sec)

    grasp_offset = resolve_grasp_offset(args)
    insert_offset = resolve_insert_offset(args) if not pick_only else None
    if args.sample_type != SAMPLE_TYPE_SUCCESS and grasp_offset is None and insert_offset is None:
        node.get_logger().warn("episode %s: sample_type=%s 但未配置偏置" % (episode_id, args.sample_type))

    bag_rel = "bags/%s" % bag_dir.name
    records: List[JobRecord] = []

    recorder = BagRecorder(bag_dir)
    node.get_logger().info("episode %s: 开始录包 -> %s (sample=%s)" % (episode_id, bag_dir, args.sample_type))
    recorder.start()
    time.sleep(args.pre_buffer_sec)

    grasp_stamped, grasp_object_id = node.wait_for_grasp_pose(min(args.wait_pose_sec, 10.0))
    grasp_frame = grasp_stamped.header.frame_id or DEFAULT_KEYPOINT_FRAME_ID
    nominal_grasp_pose = copy_pose(grasp_stamped.pose)
    pick_keypoint = copy_pose(grasp_stamped.pose)
    if grasp_offset is not None:
        pick_keypoint = apply_position_offset(pick_keypoint, grasp_offset)
        node.get_logger().info(
            "episode %s: 抓取偏置 %s -> sent=(%.3f, %.3f, %.3f)"
            % (
                episode_id,
                grasp_offset,
                pick_keypoint.position.x,
                pick_keypoint.position.y,
                pick_keypoint.position.z,
            )
        )

    nominal_insert_pose: Optional[Pose] = None
    insert_keypoint: Optional[Pose] = None
    if insert_pose is not None:
        nominal_insert_pose = copy_pose(insert_pose)
        insert_keypoint = copy_pose(insert_pose)
        if insert_offset is not None:
            insert_keypoint = apply_position_offset(insert_keypoint, insert_offset)
            node.get_logger().info(
                "episode %s: 插孔偏置 %s -> sent=(%.3f, %.3f, %.3f)"
                % (
                    episode_id,
                    insert_offset,
                    insert_keypoint.position.x,
                    insert_keypoint.position.y,
                    insert_keypoint.position.z,
                )
            )

    failure_meta = build_failure_injection_meta(
        args,
        grasp_offset,
        insert_offset,
        nominal_grasp_pose,
        nominal_insert_pose,
        pick_keypoint,
        insert_keypoint,
        grasp_frame,
        insert_frame,
    )

    pick_started = iso_now()
    pick_task_id = node.get_task_id()
    pick_ok, pick_code = node.send_robotic_arm_cmd(
        REQUEST_TYPE_GRASP,
        pick_keypoint,
        grasp_frame,
        args.action_timeout_sec,
    )
    pick_finished = iso_now()
    pick_outcome = result_code_to_outcome(pick_code)
    records.append(
        JobRecord(
            episode_id=episode_id,
            episode_outcome="",
            bag_dir=bag_rel,
            job_id=pick_task_id or ("%s_pick" % episode_id),
            task_name="PICK",
            robotic_arm_cmd_type=REQUEST_TYPE_GRASP,
            sequence_in_episode=1,
            started_timestamp=pick_started,
            finished_timestamp=pick_finished,
            outcome=pick_outcome,
            termination_reason=result_code_to_reason(pick_code, "PICK"),
            action_result_code=pick_code,
            extra={
                "keypoint_frame": grasp_frame,
                "target_id": grasp_object_id,
                "grasp_target_index": args.grasp_index,
                "grasp_pose_source": "%s -> %s (source=%s)"
                % (args.target_set_topic, grasp_frame, args.grasp_keypoint_source),
                "insert_keypoint_source": args.insert_keypoint_source,
                "keypoints_catalog": args.keypoints_catalog,
                "object_initial_pose": pose_to_dict(nominal_grasp_pose, grasp_frame),
                "grasp_keypoint_sent": pose_to_dict(pick_keypoint, grasp_frame),
                "failure_injection": failure_meta,
                "grasp_success_gt": {
                    "signal": "/manipulator/left_arm_gripped",
                    "threshold": 0.5,
                },
            },
        )
    )
    node.get_logger().info(
        "episode %s: PICK %s (code=%d)" % (episode_id, pick_outcome, pick_code)
    )
    if args.sample_type == SAMPLE_TYPE_PICK_FAIL and pick_ok:
        node.get_logger().warn(
            "episode %s: pick_fail 样例但 PICK 成功，请增大 --grasp-offset-xyz 或换偏置方向"
            % episode_id
        )

    insert_ok = False
    insert_code = RESULT_EXEC_FAILED
    if pick_only or (not pick_ok and args.skip_insert_on_pick_fail):
        if args.sample_type == SAMPLE_TYPE_PICK_FAIL:
            episode_outcome = "PICK_FAILED" if not pick_ok else "PICK_FAILED_INJECTION_MISSED"
        elif not pick_ok:
            episode_outcome = "PICK_FAILED"
        else:
            episode_outcome = "PICK_ONLY"
    else:
        if insert_keypoint is None:
            raise RuntimeError("插孔位姿未就绪")
        time.sleep(args.between_jobs_sec)
        insert_started = iso_now()
        insert_task_id = node.get_task_id()
        insert_ok, insert_code = node.send_robotic_arm_cmd(
            REQUEST_TYPE_INSERT,
            insert_keypoint,
            insert_frame,
            args.action_timeout_sec,
        )
        insert_finished = iso_now()
        insert_outcome = result_code_to_outcome(insert_code)
        records.append(
            JobRecord(
                episode_id=episode_id,
                episode_outcome="",
                bag_dir=bag_rel,
                job_id=insert_task_id or ("%s_insert" % episode_id),
                task_name="TARGET_INSERT",
                robotic_arm_cmd_type=REQUEST_TYPE_INSERT,
                sequence_in_episode=2,
                started_timestamp=insert_started,
                finished_timestamp=insert_finished,
                outcome=insert_outcome,
                termination_reason=result_code_to_reason(insert_code, "INSERT"),
                grasp_source=None,
                action_result_code=insert_code,
                extra={
                    "keypoint_frame": insert_frame,
                    "target_id": insert_slot_id,
                    "keypoints_catalog": args.keypoints_catalog,
                    "target_pose": pose_to_dict(
                        nominal_insert_pose if nominal_insert_pose is not None else insert_keypoint,
                        insert_frame,
                    ),
                    "insert_keypoint_sent": pose_to_dict(insert_keypoint, insert_frame),
                    "failure_injection": failure_meta,
                    "insert_latch_gt": {
                        "signal": "/manipulator/target_set.latches",
                        "target_index": args.insert_index,
                        "threshold": 0.5,
                        "job_may_succeed_without_latch": True,
                    },
                },
            )
        )
        node.get_logger().info(
            "episode %s: INSERT %s (code=%d)" % (episode_id, insert_outcome, insert_code)
        )
        if args.sample_type == SAMPLE_TYPE_INSERT_FAIL and insert_ok:
            node.get_logger().warn(
                "episode %s: insert_fail 样例但 INSERT 成功，请增大 --insert-offset-xyz"
                % episode_id
            )
        if pick_ok and insert_ok:
            episode_outcome = "SUCCEEDED"
        elif pick_ok and not insert_ok:
            episode_outcome = "INSERT_FAILED"
        else:
            episode_outcome = "PICK_FAILED"

    time.sleep(args.post_buffer_sec)
    recorder.stop()
    node.get_logger().info("episode %s: 录包结束" % episode_id)

    for rec in records:
        rec.episode_outcome = episode_outcome
    append_jobs_jsonl(jobs_path, records)
    return episode_outcome


def build_arg_parser() -> argparse.ArgumentParser:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Target 抓+插 episode 自动执行并录制 rosbag2",
    )
    parser.add_argument("--episode-id", default="", help="episode 目录名；空则自动生成")
    parser.add_argument("--count", type=int, default=1, help="连续录制 episode 数")
    parser.add_argument("--prefix", default="ep_target", help="批量 episode_id 前缀")
    parser.add_argument(
        "--bags-root",
        default=str(script_dir / "bags"),
        help="bag 输出根目录",
    )
    parser.add_argument(
        "--jobs-jsonl",
        default=str(script_dir / "jobs.jsonl"),
        help="追加写入的元数据文件",
    )
    parser.add_argument(
        "--keypoints-catalog",
        default=str(default_keypoints_catalog_path()),
        help="odom 插孔 catalog YAML（默认自包根或 tools/arm_bag_tools/config/ 自动查找）",
    )
    parser.add_argument(
        "--target-set-topic",
        default=DEFAULT_TARGET_SET_TOPIC,
        help="target_set 话题（默认抓取 keypoint 来源，与网页 submit_job 一致）",
    )
    parser.add_argument(
        "--insert-holes-topic",
        default=DEFAULT_INSERT_HOLES_TOPIC,
        help="target_insert_holes 话题（默认插孔 keypoint 来源）",
    )
    parser.add_argument(
        "--grasp-keypoint-source",
        choices=[GRASP_KEYPOINT_SOURCE_TARGET_SET, GRASP_KEYPOINT_SOURCE_TARGET_SENSOR],
        default=GRASP_KEYPOINT_SOURCE_TARGET_SET,
        help="抓取 keypoint：target_set（默认，与网页一致）或 target_sensor 自算",
    )
    parser.add_argument(
        "--insert-keypoint-source",
        choices=[INSERT_KEYPOINT_SOURCE_HOLES, INSERT_KEYPOINT_SOURCE_CATALOG],
        default=INSERT_KEYPOINT_SOURCE_HOLES,
        help="插孔 keypoint：insert_holes（默认，与网页一致）或 catalog 静态 odom",
    )
    parser.add_argument(
        "--target-sensor-topic",
        default=DEFAULT_TARGET_SENSOR_TOPIC,
        help="TargetSensor 话题（grasp-keypoint-source=target_sensor 时使用）",
    )
    parser.add_argument(
        "--rov-pose-topic",
        default=DEFAULT_ROV_POSE_TOPIC,
        help="ROV PoseSensor（用于 arm_base_link reach 校验）",
    )
    parser.add_argument(
        "--grasp-index",
        type=int,
        default=0,
        help="TargetSensor 目标下标（默认 0）",
    )
    parser.add_argument(
        "--max-grasp-reach-m",
        type=float,
        default=DEFAULT_MAX_GRASP_REACH_M,
        help="抓取位姿相对 arm_base_link 最大允许距离 [m]（须 <= MTC workspace 硬上限）",
    )
    parser.add_argument("--insert-index", type=int, default=0, help="catalog insert_slots 下标")
    parser.add_argument(
        "--action-name",
        default="/manipulator/robotic_arm_cmd",
        help="RoboticArmCmd Action 名",
    )
    parser.add_argument("--wait-pose-sec", type=float, default=60.0, help="等待感知超时 [s]")
    parser.add_argument("--pre-buffer-sec", type=float, default=2.0, help="开录后缓冲 [s]")
    parser.add_argument("--post-buffer-sec", type=float, default=2.0, help="停录前缓冲 [s]")
    parser.add_argument("--between-jobs-sec", type=float, default=1.0, help="抓/插间隔 [s]")
    parser.add_argument(
        "--action-timeout-sec",
        type=float,
        default=900.0,
        help="单次 Action 最长等待 [s]",
    )
    parser.add_argument(
        "--sample-type",
        choices=[SAMPLE_TYPE_SUCCESS, SAMPLE_TYPE_PICK_FAIL, SAMPLE_TYPE_INSERT_FAIL],
        default=SAMPLE_TYPE_SUCCESS,
        help="样例类型：success / pick_fail（抓取偏置+仅 PICK）/ insert_fail（插孔偏置）",
    )
    parser.add_argument(
        "--failure-set",
        action="store_true",
        help="连续录 3 条：{prefix}_ok_001、{prefix}_pick_fail_001、{prefix}_insert_fail_001",
    )
    parser.add_argument(
        "--grasp-offset-xyz",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="抓取 keypoint 额外偏置 [m]；pick_fail 未指定时用默认 %s"
        % DEFAULT_PICK_FAIL_OFFSET_XYZ,
    )
    parser.add_argument(
        "--insert-offset-xyz",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="插孔 keypoint 额外偏置 [m]；insert_fail 未指定时用默认 %s"
        % DEFAULT_INSERT_FAIL_OFFSET_XYZ,
    )
    parser.add_argument(
        "--pick-only",
        action="store_true",
        help="仅抓取停录（pick_fail 样例会自动开启）",
    )
    parser.add_argument(
        "--skip-insert-on-pick-fail",
        action="store_true",
        default=True,
        help="抓取失败则跳过插孔（默认开启）",
    )
    parser.add_argument(
        "--no-skip-insert-on-pick-fail",
        action="store_false",
        dest="skip_insert_on_pick_fail",
        help="抓取失败仍尝试插孔",
    )
    parser.add_argument(
        "--reset-between",
        action="store_true",
        default=True,
        help="每条 episode 前 reset_held + go_ready",
    )
    parser.add_argument(
        "--no-reset-between",
        action="store_false",
        dest="reset_between",
        help="episode 间不复位",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只检查话题与 Action，不录包",
    )
    return parser


def configure_episode_args(base: argparse.Namespace, sample_type: str) -> argparse.Namespace:
    ep_args = deepcopy(base)
    ep_args.sample_type = sample_type
    if sample_type == SAMPLE_TYPE_PICK_FAIL:
        ep_args.pick_only = True
    elif sample_type == SAMPLE_TYPE_INSERT_FAIL:
        ep_args.pick_only = False
    return ep_args


def build_episode_runs(args: argparse.Namespace) -> List[Tuple[argparse.Namespace, str]]:
    if args.failure_set:
        plan = [
            (SAMPLE_TYPE_SUCCESS, "%s_ok_001" % args.prefix),
            (SAMPLE_TYPE_PICK_FAIL, "%s_pick_fail_001" % args.prefix),
            (SAMPLE_TYPE_INSERT_FAIL, "%s_insert_fail_001" % args.prefix),
        ]
        return [(configure_episode_args(args, st), eid) for st, eid in plan]
    runs: List[Tuple[argparse.Namespace, str]] = []
    for i in range(args.count):
        if args.episode_id and args.count == 1:
            ep_id = args.episode_id
        else:
            ep_id = "%s_%03d" % (args.prefix, i + 1)
        runs.append((args, ep_id))
    return runs


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    bags_root = Path(args.bags_root)
    jobs_path = Path(args.jobs_jsonl)
    catalog_path = resolve_keypoints_catalog_path(args.keypoints_catalog)
    if catalog_path is None:
        tried = [str(default_keypoints_catalog_path())]
        for parent in list(SCRIPT_DIR.parents)[:6]:
            tried.append(str(parent / CATALOG_REL_PATH))
        tried.append(str(SCRIPT_DIR / CATALOG_REL_PATH))
        print(
            "catalog 不存在。请确认已同步 config/manipulator_task_keypoints_odom.yaml，"
            "或通过 --keypoints-catalog 指定。\n已尝试:\n  - "
            + "\n  - ".join(tried),
            file=sys.stderr,
        )
        return 1
    try:
        catalog = load_keypoints_catalog(catalog_path)
    except (ValueError, OSError) as exc:
        print("加载 catalog 失败: %s" % exc, file=sys.stderr)
        return 1
    args.keypoints_catalog = str(catalog_path)

    rclpy.init()
    node = EpisodeRecordNode(
        catalog=catalog,
        target_set_topic=args.target_set_topic,
        target_sensor_topic=args.target_sensor_topic,
        rov_pose_topic=args.rov_pose_topic,
        insert_holes_topic=args.insert_holes_topic,
        grasp_index=args.grasp_index,
        max_grasp_reach_m=args.max_grasp_reach_m,
        insert_index=args.insert_index,
        action_name=args.action_name,
        grasp_keypoint_source=args.grasp_keypoint_source,
        insert_keypoint_source=args.insert_keypoint_source,
    )

    if args.dry_run:
        node.get_logger().info("dry-run: 等待 Action 与感知...")
        if not node.wait_for_server(timeout_sec=10.0):
            node.get_logger().error("Action 不可用")
            rclpy.shutdown()
            return 1
        try:
            grasp_stamped, grasp_object_id = node.wait_for_grasp_pose(min(args.wait_pose_sec, 15.0))
            reach_m = 0.0
            if node._latest_target_sensor and node._rov_position and node._rov_orientation_xyzw:
                reach_m = compute_grasp_reach_arm_base_link_m(
                    int(node._latest_target_sensor.num_targets),
                    list(node._latest_target_sensor.positions),
                    list(node._latest_target_sensor.directions),
                    args.grasp_index,
                    node._grasp_offset_m,
                    node._rov_position,
                    node._rov_orientation_xyzw,
                    node._t_arm_in_rov,
                )
            node.get_logger().info(
                "抓取 keypoint OK: %s reach=%.3fm frame=%s"
                % (
                    grasp_object_id,
                    reach_m,
                    grasp_stamped.header.frame_id,
                )
            )
        except TimeoutError as exc:
            node.get_logger().error(str(exc))
            rclpy.shutdown()
            return 1
        if not effective_pick_only(args):
            try:
                insert_pose, insert_frame, slot_id = node.wait_for_insert_keypoint(
                    min(args.wait_pose_sec, 15.0)
                )
                node.get_logger().info(
                    "插孔 keypoint OK: %s frame=%s pos=(%.3f, %.3f, %.3f)"
                    % (
                        slot_id,
                        insert_frame,
                        insert_pose.position.x,
                        insert_pose.position.y,
                        insert_pose.position.z,
                    )
                )
            except IndexError as exc:
                node.get_logger().error(str(exc))
                rclpy.shutdown()
                return 1
        if args.failure_set:
            node.get_logger().info(
                "failure-set 将录制: %s_ok_001, %s_pick_fail_001, %s_insert_fail_001"
                % (args.prefix, args.prefix, args.prefix)
            )
        node.get_logger().info("dry-run 通过")
        rclpy.shutdown()
        return 0

    exit_code = 0
    try:
        episode_runs = build_episode_runs(args)
        for run_idx, (ep_args, ep_id) in enumerate(episode_runs):
            bag_dir = bags_root / ep_id

            if ep_args.reset_between:
                node.get_logger().info("episode %s: 复位..." % ep_id)
                node.reset_between_episodes()
                time.sleep(2.0)

            try:
                node.wait_for_grasp_pose(ep_args.wait_pose_sec)
            except TimeoutError as exc:
                node.get_logger().error(str(exc))
                raise

            outcome = run_single_episode(node, ep_id, bag_dir, jobs_path, ep_args)
            node.get_logger().info("完成 %s -> %s" % (ep_id, outcome))
            if run_idx + 1 < len(episode_runs):
                time.sleep(ep_args.between_jobs_sec)
    except Exception as exc:
        node.get_logger().error("episode 失败: %s" % exc)
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
