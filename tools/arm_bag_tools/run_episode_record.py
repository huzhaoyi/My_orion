#!/usr/bin/env python3
"""
Target 抓取（robotic_arm_cmd type=0）+ 插孔（type=1）episode 自动执行并录制 rosbag2。

依赖（需先 source ROS2 + 工作空间，含 sealien_ctrlpilot_msgmanagement）:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source <ws>/install/setup.bash

用法:
  ./run_episode_record.sh --episode-id ep_target_001
  ./run_episode_record.sh --count 5 --prefix ep_target
  ./run_episode_record.sh --pick-only --episode-id ep_target_pick_fail_001

全栈 launch 须已运行；HoloOcean + MTC 就绪且感知话题有数据。
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

try:
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped
    from sealien_ctrlpilot_manipulator_orion_mtc_msgs.srv import GetRobotState, ResetHeldObject
    from sealien_ctrlpilot_msgmanagement.action import RoboticArmCmd
    from sealien_ctrlpilot_msgmanagement.msg import RoboticArmRequest
    from std_srvs.srv import Trigger
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
    """订阅感知位姿、调用 robotic_arm_cmd、复位 episode 间状态。"""

    def __init__(
        self,
        grasp_topic: str,
        grasp_fallback_topic: str,
        insert_holes_topic: str,
        action_name: str,
        insert_index: int,
    ) -> None:
        super().__init__("episode_record_runner")
        self._grasp_topic = grasp_topic
        self._grasp_fallback_topic = grasp_fallback_topic
        self._insert_holes_topic = insert_holes_topic
        self._insert_index = insert_index
        self._latest_grasp_primary: Optional[PoseStamped] = None
        self._latest_grasp_fallback: Optional[PoseStamped] = None
        self._latest_holes: Optional[PoseArray] = None

        self.create_subscription(
            PoseStamped,
            grasp_topic,
            self._on_grasp_primary,
            qos_profile_sensor_data,
        )
        if grasp_fallback_topic and grasp_fallback_topic != grasp_topic:
            self.create_subscription(
                PoseStamped,
                grasp_fallback_topic,
                self._on_grasp_fallback,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            PoseArray,
            insert_holes_topic,
            self._on_insert_holes,
            qos_profile_sensor_data,
        )

        self._action_name = action_name
        self._action_client = ActionClient(self, RoboticArmCmd, action_name)
        self._get_state_cli = self.create_client(GetRobotState, "/manipulator/get_robot_state")
        self._reset_held_cli = self.create_client(ResetHeldObject, "/manipulator/reset_held_object")
        self._go_ready_cli = self.create_client(Trigger, "/manipulator/go_to_ready")

    def _on_grasp_primary(self, msg: PoseStamped) -> None:
        self._latest_grasp_primary = msg

    def _on_grasp_fallback(self, msg: PoseStamped) -> None:
        self._latest_grasp_fallback = msg

    def _current_grasp_pose(self) -> Optional[PoseStamped]:
        if self._latest_grasp_primary is not None:
            return self._latest_grasp_primary
        return self._latest_grasp_fallback

    def _on_insert_holes(self, msg: PoseArray) -> None:
        self._latest_holes = msg

    def wait_for_server(self, timeout_sec: float) -> bool:
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def wait_for_grasp_pose(self, timeout_sec: float) -> PoseStamped:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            current = self._current_grasp_pose()
            if current is not None:
                return current
            rclpy.spin_once(self, timeout_sec=0.2)
        topics = self._grasp_topic
        if self._grasp_fallback_topic:
            topics = "%s / %s" % (self._grasp_topic, self._grasp_fallback_topic)
        raise TimeoutError("等待抓取位姿超时: %s" % topics)

    def wait_for_insert_hole(self, timeout_sec: float) -> Tuple[Pose, str]:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._latest_holes is not None and len(self._latest_holes.poses) > self._insert_index:
                frame = self._latest_holes.header.frame_id or "arm_base_link"
                return self._latest_holes.poses[self._insert_index], frame
            rclpy.spin_once(self, timeout_sec=0.2)
        raise TimeoutError(
            "等待插孔位姿超时: %s (index=%d)"
            % (self._insert_holes_topic, self._insert_index)
        )

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
    node.get_logger().info("episode %s: 等待感知..." % episode_id)
    grasp_stamped = node.wait_for_grasp_pose(args.wait_pose_sec)
    grasp_frame = grasp_stamped.header.frame_id or "arm_base_link"
    insert_pose: Optional[Pose] = None
    insert_frame = "arm_base_link"
    if not args.pick_only:
        insert_pose, insert_frame = node.wait_for_insert_hole(args.wait_pose_sec)

    bag_rel = "bags/%s" % bag_dir.name
    records: List[JobRecord] = []

    recorder = BagRecorder(bag_dir)
    node.get_logger().info("episode %s: 开始录包 -> %s" % (episode_id, bag_dir))
    recorder.start()
    time.sleep(args.pre_buffer_sec)

    pick_started = iso_now()
    pick_task_id = node.get_task_id()
    pick_ok, pick_code = node.send_robotic_arm_cmd(
        REQUEST_TYPE_GRASP,
        grasp_stamped.pose,
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
                "object_initial_pose": pose_to_dict(grasp_stamped.pose, grasp_frame),
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

    insert_ok = False
    insert_code = RESULT_EXEC_FAILED
    if args.pick_only or (not pick_ok and args.skip_insert_on_pick_fail):
        episode_outcome = "PICK_FAILED" if not pick_ok else "PICK_ONLY"
    else:
        if insert_pose is None:
            raise RuntimeError("插孔位姿未就绪")
        time.sleep(args.between_jobs_sec)
        insert_started = iso_now()
        insert_task_id = node.get_task_id()
        insert_ok, insert_code = node.send_robotic_arm_cmd(
            REQUEST_TYPE_INSERT,
            insert_pose,
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
                    "target_id": "hole_slot_%d" % args.insert_index,
                    "target_pose": pose_to_dict(insert_pose, insert_frame),
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
        "--grasp-topic",
        default="/manipulator/object_pose_targetsensor",
        help="Target 抓取 keypoint 来源",
    )
    parser.add_argument(
        "--grasp-topic-fallback",
        default="/manipulator/object_pose",
        help="抓取位姿回退话题",
    )
    parser.add_argument(
        "--insert-holes-topic",
        default="/manipulator/target_insert_holes",
        help="插孔 PoseArray 话题",
    )
    parser.add_argument("--insert-index", type=int, default=0, help="插孔孔位下标")
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
        "--pick-only",
        action="store_true",
        help="仅抓取（用于抓失败/抓单独样例）",
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


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    bags_root = Path(args.bags_root)
    jobs_path = Path(args.jobs_jsonl)

    rclpy.init()
    node = EpisodeRecordNode(
        grasp_topic=args.grasp_topic,
        grasp_fallback_topic=args.grasp_topic_fallback,
        insert_holes_topic=args.insert_holes_topic,
        action_name=args.action_name,
        insert_index=args.insert_index,
    )

    if args.dry_run:
        node.get_logger().info("dry-run: 等待 Action 与感知...")
        if not node.wait_for_server(timeout_sec=10.0):
            node.get_logger().error("Action 不可用")
            rclpy.shutdown()
            return 1
        try:
            node.wait_for_grasp_pose(min(args.wait_pose_sec, 15.0))
            node.get_logger().info("抓取位姿 OK")
        except TimeoutError as exc:
            node.get_logger().error(str(exc))
            rclpy.shutdown()
            return 1
        if not args.pick_only:
            try:
                node.wait_for_insert_hole(min(args.wait_pose_sec, 15.0))
                node.get_logger().info("插孔位姿 OK")
            except TimeoutError as exc:
                node.get_logger().error(str(exc))
                rclpy.shutdown()
                return 1
        node.get_logger().info("dry-run 通过")
        rclpy.shutdown()
        return 0

    exit_code = 0
    try:
        for i in range(args.count):
            if args.episode_id and args.count == 1:
                ep_id = args.episode_id
            else:
                ep_id = "%s_%03d" % (args.prefix, i + 1)
            bag_dir = bags_root / ep_id

            if args.reset_between:
                node.get_logger().info("episode %s: 复位..." % ep_id)
                node.reset_between_episodes()
                time.sleep(2.0)

            try:
                node.wait_for_grasp_pose(args.wait_pose_sec)
            except TimeoutError as exc:
                node.get_logger().error(str(exc))
                raise

            outcome = run_single_episode(node, ep_id, bag_dir, jobs_path, args)
            node.get_logger().info("完成 %s -> %s" % (ep_id, outcome))
            if args.count > 1 and i + 1 < args.count:
                time.sleep(args.between_jobs_sec)
    except Exception as exc:
        node.get_logger().error("episode 失败: %s" % exc)
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
