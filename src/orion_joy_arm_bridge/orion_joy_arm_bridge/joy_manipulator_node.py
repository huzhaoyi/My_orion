#!/usr/bin/env python3
"""
双路 sensor_msgs/Joy → Orion 机械臂：右手 buttons 急停/解刹（不分模式）；自动 pick_trigger、右手 axes[] 边沿 open/close_gripper（与手动同轴下标）、可选键位夹爪；
手动 6 轴速度积分 + 夹爪轨迹（FollowJointTrajectory → arm_controller / hand_controller）。
参数中关节角、角速度、限位均以度(°)为单位；订阅的 joint_states 与下发的轨迹位置仍为弧度（ROS 惯例），在节点内换算。
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, Empty, Float32
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_DEG_TO_RAD = math.pi / 180.0
_RAD_TO_DEG = 180.0 / math.pi


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _joy_button(msg: Optional[Joy], index: int) -> bool:
    if msg is None or index < 0:
        return False
    if index >= len(msg.buttons):
        return False
    return int(msg.buttons[index]) != 0


def _joy_axis(msg: Optional[Joy], index: int) -> float:
    if msg is None or index < 0:
        return 0.0
    if index >= len(msg.axes):
        return 0.0
    return float(msg.axes[index])


def _joy_axis_tri_sign(msg: Optional[Joy], index: int, threshold: float) -> int:
    """离散三态：axes 值 > threshold 为 +1，< -threshold 为 -1，否则 0（用于拨杆开/合爪）。"""
    if threshold <= 0.0:
        return 0
    v = _joy_axis(msg, index)
    if v > threshold:
        return 1
    if v < -threshold:
        return -1
    return 0


def _joy_snapshot(msg: Joy) -> Tuple[Tuple[int, ...], Tuple[float, ...]]:
    buttons = tuple(int(x) for x in msg.buttons)
    axes = tuple(float(x) for x in msg.axes)
    return (buttons, axes)


def _format_button_diff(old_b: Tuple[int, ...], new_b: Tuple[int, ...]) -> str:
    n = max(len(old_b), len(new_b))
    parts: List[str] = []
    for i in range(n):
        o = int(old_b[i]) if i < len(old_b) else None
        nv = int(new_b[i]) if i < len(new_b) else None
        if o != nv:
            parts.append("buttons[%d] %s→%s" % (i, o, nv))
    return "; ".join(parts)


def _format_axis_diff(old_a: Tuple[float, ...], new_a: Tuple[float, ...], eps: float) -> str:
    n = max(len(old_a), len(new_a))
    parts: List[str] = []
    for i in range(n):
        o = float(old_a[i]) if i < len(old_a) else None
        nv = float(new_a[i]) if i < len(new_a) else None
        if o is None or nv is None:
            parts.append("axes[%d] %s→%s" % (i, o, nv))
        elif abs(o - nv) > eps:
            parts.append("axes[%d] %.4f→%.4f" % (i, o, nv))
    return "; ".join(parts)


def _joy_snap_semantically_equal(
    prev: Tuple[Tuple[int, ...], Tuple[float, ...]],
    snap: Tuple[Tuple[int, ...], Tuple[float, ...]],
    axis_eps: float,
) -> bool:
    """与 log_joy_axis_epsilon 一致：按钮须完全相同；轴长度相同且逐维差值不超过 axis_eps 视为无变化（抑制摇杆噪声刷屏）。"""
    ob, oa = prev
    nb, na = snap
    if ob != nb:
        return False
    if len(oa) != len(na):
        return False
    for i in range(len(oa)):
        if abs(oa[i] - na[i]) > axis_eps:
            return False
    return True


class JoyManipulatorNode(Node):
    """手柄桥接节点。"""

    def __init__(self) -> None:
        super().__init__("joy_manipulator_node")

        self.declare_parameter("manipulator_ns", "/manipulator")
        self.declare_parameter("topic_left_joy", "left_joy")
        self.declare_parameter("topic_right_joy", "right_joy")
        self.declare_parameter("topic_joint_states", "/joint_states")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("axis_deadband", 0.1)
        self.declare_parameter("speed_axis_index", 2)
        self.declare_parameter("speed_axis_invert", False)
        # symmetric_abs：以 |axis| 为量，近 0 为 idle，±1 均为满速（旧行为）
        # throttle_full_span：[-1,1] 线性映射到 [0,1]，-1=0 +1=满速；方向仍由 ± 键决定
        self.declare_parameter("speed_axis_map_mode", "symmetric_abs")
        # 手动夹爪积分是否乘油门；false 时始终以 manual_gripper_max_speed 满速（与臂油门无关）
        self.declare_parameter("manual_gripper_use_throttle", True)
        # 夹爪油门读哪只手柄；-1 的 gripper_speed_axis_index 表示与 speed_axis_index 相同
        self.declare_parameter("gripper_speed_joy", "left")
        self.declare_parameter("gripper_speed_axis_index", -1)
        self.declare_parameter("manual_arm_max_speed_deg_s", 60.0)
        self.declare_parameter("manual_gripper_max_speed_deg_s", 25.0)
        # 仅手动夹爪积分：乘在 (max_speed×油门×dt) 上，与 6 臂无关；>1 加快开合
        self.declare_parameter("manual_gripper_speed_scale", 1.0)
        self.declare_parameter("trajectory_time_margin", 1.25)
        self.declare_parameter("mode_source", "button")
        self.declare_parameter("mode_button_index", 32)
        self.declare_parameter("mode_manual_when_pressed", True)
        self.declare_parameter("mode_axis_index", 0)
        self.declare_parameter("mode_manual_axis_positive_above", 0.2)
        self.declare_parameter("right_pick_button_index", 2)
        # 自动开闭夹爪键（右手）；须避开 right_emergency/clear 专用键；默认 -1 关闭
        self.declare_parameter("auto_gripper_open_button_index", -1)
        self.declare_parameter("auto_gripper_close_button_index", -1)
        self.declare_parameter("arm_button_plus_indices", [12, 14, 16, 18, 6, 8])
        self.declare_parameter("arm_button_minus_indices", [11, 13, 15, 17, 5, 7])
        self.declare_parameter("gripper_open_button_index", 9)
        self.declare_parameter("gripper_close_button_index", 10)
        # 手动：右手 axes[]：> 阈值开夹爪、< -阈值关（与左手 9/10 并存；-1 关闭）
        self.declare_parameter("manual_right_gripper_axis_index", 6)
        self.declare_parameter("manual_right_gripper_axis_threshold", 0.5)
        # 右手：任意模式下 buttons[] 上升沿 emergency_stop / 曾按下→松开 clear_estop（与 auto/manual 无关）；-1 关闭
        self.declare_parameter("right_emergency_stop_button_index", 0)
        self.declare_parameter("right_clear_estop_button_index", 5)
        self.declare_parameter("arm_joint_names", [
            "joint_base_link_Link1",
            "joint_Link1_Link2",
            "joint_Link2_Link3",
            "joint_LinkVirtual_Link4",
            "joint_Link4_Link5",
            "joint_Link5_Link6",
        ])
        self.declare_parameter("hand_joint_names", ["joint_Link6_Link7", "joint_Link6_Link8"])
        self.declare_parameter("arm_joint_lower_limits_deg", [-180.0] * 6)
        self.declare_parameter("arm_joint_upper_limits_deg", [180.0] * 6)
        # 与 SRDF 张开/闭合一致（与 0.4 / -0.4 rad 等价）
        self.declare_parameter("hand_q7_open_deg", float(math.degrees(0.4)))
        self.declare_parameter("hand_q8_open_deg", float(math.degrees(-0.4)))
        self.declare_parameter("hand_q7_close_deg", 0.0)
        self.declare_parameter("hand_q8_close_deg", 0.0)
        self.declare_parameter("arm_action_name", "arm_controller/follow_joint_trajectory")
        self.declare_parameter("hand_action_name", "hand_controller/follow_joint_trajectory")
        self.declare_parameter("on_manual_enter_call_emergency_stop", True)
        # 切回自动时：先 emergency_stop 再 clear_estop（取消残留轨迹后解闭锁）
        self.declare_parameter("on_auto_enter_call_clear_estop", True)
        self.declare_parameter("require_arm_action_server", True)
        self.declare_parameter("require_hand_action_server", True)
        self.declare_parameter("log_joint_targets_debug", False)
        self.declare_parameter("log_joy_on_change", True)
        # 与 log_joy 变化判定、diff 文案共用：轴差 ≤ 此值不记为变化、也不在 diff 中展开
        self.declare_parameter("log_joy_axis_epsilon", 0.01)
        # 与 joy_manipulator.yaml「自动模式：右手」并列：右手硬件 ready 键下标，Joy 变化日志「非0→0」附「（回 ready 位）」；-1 关闭
        self.declare_parameter("log_right_joy_ready_button_index", 4)
        # true：自动模式下该键由按下变为松开时调用 /manipulator/go_to_ready（与 Web「回 ready」同一服务）
        self.declare_parameter("on_auto_right_ready_release_call_go_to_ready", True)
        self.declare_parameter("publish_ui_status", True)
        self.declare_parameter("topic_ui_manual_mode", "/joy_manipulator/manual_mode")
        self.declare_parameter("topic_ui_throttle_percent", "/joy_manipulator/throttle_percent")

        self._manip_ns = self.get_parameter("manipulator_ns").get_parameter_value().string_value
        self._topic_left = self.get_parameter("topic_left_joy").get_parameter_value().string_value
        self._topic_right = self.get_parameter("topic_right_joy").get_parameter_value().string_value
        self._topic_js = self.get_parameter("topic_joint_states").get_parameter_value().string_value
        rate_hz = self.get_parameter("control_rate_hz").get_parameter_value().double_value
        self._dt = 1.0 / rate_hz if rate_hz > 1.0e-6 else 0.02
        self._axis_deadband = self.get_parameter("axis_deadband").get_parameter_value().double_value
        self._speed_axis_index = self.get_parameter("speed_axis_index").get_parameter_value().integer_value
        self._speed_axis_invert = self.get_parameter("speed_axis_invert").get_parameter_value().bool_value
        self._speed_axis_map_mode = (
            self.get_parameter("speed_axis_map_mode").get_parameter_value().string_value.strip().lower()
        )
        self._manual_gripper_use_throttle = (
            self.get_parameter("manual_gripper_use_throttle").get_parameter_value().bool_value
        )
        self._gripper_speed_joy = (
            self.get_parameter("gripper_speed_joy").get_parameter_value().string_value.strip().lower()
        )
        self._gripper_speed_axis_index = (
            self.get_parameter("gripper_speed_axis_index").get_parameter_value().integer_value
        )
        arm_deg_s = self.get_parameter("manual_arm_max_speed_deg_s").get_parameter_value().double_value
        grip_deg_s = self.get_parameter("manual_gripper_max_speed_deg_s").get_parameter_value().double_value
        self._arm_max_speed = arm_deg_s * _DEG_TO_RAD
        self._grip_max_speed = grip_deg_s * _DEG_TO_RAD
        g_scale = self.get_parameter("manual_gripper_speed_scale").get_parameter_value().double_value
        self._grip_speed_scale = float(max(0.0, g_scale))
        self._traj_margin = self.get_parameter("trajectory_time_margin").get_parameter_value().double_value

        self._mode_source = self.get_parameter("mode_source").get_parameter_value().string_value.strip().lower()
        self._mode_button_index = self.get_parameter("mode_button_index").get_parameter_value().integer_value
        self._mode_manual_when_pressed = self.get_parameter("mode_manual_when_pressed").get_parameter_value().bool_value
        self._mode_axis_index = self.get_parameter("mode_axis_index").get_parameter_value().integer_value
        self._mode_axis_thr = self.get_parameter("mode_manual_axis_positive_above").get_parameter_value().double_value

        self._pick_btn = self.get_parameter("right_pick_button_index").get_parameter_value().integer_value
        self._auto_g_open = self.get_parameter("auto_gripper_open_button_index").get_parameter_value().integer_value
        self._auto_g_close = self.get_parameter("auto_gripper_close_button_index").get_parameter_value().integer_value

        self._arm_plus = list(self.get_parameter("arm_button_plus_indices").get_parameter_value().integer_array_value)
        self._arm_minus = list(self.get_parameter("arm_button_minus_indices").get_parameter_value().integer_array_value)
        self._grip_open_i = self.get_parameter("gripper_open_button_index").get_parameter_value().integer_value
        self._grip_close_i = self.get_parameter("gripper_close_button_index").get_parameter_value().integer_value
        self._manual_r_grip_axis_i = (
            self.get_parameter("manual_right_gripper_axis_index").get_parameter_value().integer_value
        )
        self._manual_r_grip_axis_thr = (
            self.get_parameter("manual_right_gripper_axis_threshold").get_parameter_value().double_value
        )
        self._right_estop_btn_i = (
            self.get_parameter("right_emergency_stop_button_index").get_parameter_value().integer_value
        )
        self._right_clear_btn_i = (
            self.get_parameter("right_clear_estop_button_index").get_parameter_value().integer_value
        )

        self._arm_joint_names = list(self.get_parameter("arm_joint_names").get_parameter_value().string_array_value)
        self._hand_joint_names = list(self.get_parameter("hand_joint_names").get_parameter_value().string_array_value)
        low_deg = list(self.get_parameter("arm_joint_lower_limits_deg").get_parameter_value().double_array_value)
        high_deg = list(self.get_parameter("arm_joint_upper_limits_deg").get_parameter_value().double_array_value)
        self._arm_low = [float(v) * _DEG_TO_RAD for v in low_deg]
        self._arm_high = [float(v) * _DEG_TO_RAD for v in high_deg]
        self._hq7_open = self.get_parameter("hand_q7_open_deg").get_parameter_value().double_value * _DEG_TO_RAD
        self._hq8_open = self.get_parameter("hand_q8_open_deg").get_parameter_value().double_value * _DEG_TO_RAD
        self._hq7_close = self.get_parameter("hand_q7_close_deg").get_parameter_value().double_value * _DEG_TO_RAD
        self._hq8_close = self.get_parameter("hand_q8_close_deg").get_parameter_value().double_value * _DEG_TO_RAD

        arm_act = self.get_parameter("arm_action_name").get_parameter_value().string_value
        hand_act = self.get_parameter("hand_action_name").get_parameter_value().string_value
        self._estop_on_manual = self.get_parameter("on_manual_enter_call_emergency_stop").get_parameter_value().bool_value
        self._clear_estop_on_auto = self.get_parameter("on_auto_enter_call_clear_estop").get_parameter_value().bool_value
        self._req_arm = self.get_parameter("require_arm_action_server").get_parameter_value().bool_value
        self._req_hand = self.get_parameter("require_hand_action_server").get_parameter_value().bool_value
        self._log_targets = self.get_parameter("log_joint_targets_debug").get_parameter_value().bool_value
        self._log_joy_change = self.get_parameter("log_joy_on_change").get_parameter_value().bool_value
        self._log_joy_axis_eps = self.get_parameter("log_joy_axis_epsilon").get_parameter_value().double_value
        self._log_right_ready_btn = self.get_parameter("log_right_joy_ready_button_index").get_parameter_value().integer_value
        self._auto_ready_release_go = (
            self.get_parameter("on_auto_right_ready_release_call_go_to_ready").get_parameter_value().bool_value
        )
        self._publish_ui_status_flag = self.get_parameter("publish_ui_status").get_parameter_value().bool_value
        self._pub_ui_manual: Optional[object] = None
        self._pub_ui_throttle: Optional[object] = None

        self._snap_left_prev: Optional[Tuple[Tuple[int, ...], Tuple[float, ...]]] = None
        self._snap_right_prev: Optional[Tuple[Tuple[int, ...], Tuple[float, ...]]] = None

        if len(self._arm_plus) != len(self._arm_minus) or len(self._arm_plus) != len(self._arm_joint_names):
            self.get_logger().error(
                "参数错误：arm_button_plus/minus 数量须与 arm_joint_names 一致，请检查配置。"
            )
        if len(self._arm_low) < len(self._arm_joint_names):
            self.get_logger().warn("arm_joint_lower_limits_deg 长度不足，已用 -180° 补齐")
            while len(self._arm_low) < len(self._arm_joint_names):
                self._arm_low.append(-180.0 * _DEG_TO_RAD)
        if len(self._arm_high) < len(self._arm_joint_names):
            self.get_logger().warn("arm_joint_upper_limits_deg 长度不足，已用 180° 补齐")
            while len(self._arm_high) < len(self._arm_joint_names):
                self._arm_high.append(180.0 * _DEG_TO_RAD)

        qos_js = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Joy, self._topic_left, self._cb_left_joy, 10)
        self.create_subscription(Joy, self._topic_right, self._cb_right_joy, 10)
        self.create_subscription(JointState, self._topic_js, self._cb_joint_state, qos_js)

        self._pub_pick = self.create_publisher(Empty, self._manip_ns + "/pick_trigger", 10)

        self._srv_open = self.create_client(Trigger, self._manip_ns + "/open_gripper")
        self._srv_close = self.create_client(Trigger, self._manip_ns + "/close_gripper")
        self._srv_estop = self.create_client(Trigger, self._manip_ns + "/emergency_stop")
        self._srv_clear_estop = self.create_client(Trigger, self._manip_ns + "/clear_estop")
        self._srv_go_ready = self.create_client(Trigger, self._manip_ns + "/go_to_ready")

        self._arm_client = ActionClient(self, FollowJointTrajectory, arm_act)
        self._hand_client = ActionClient(self, FollowJointTrajectory, hand_act)

        if self._publish_ui_status_flag:
            t_man = self.get_parameter("topic_ui_manual_mode").get_parameter_value().string_value
            t_thr = self.get_parameter("topic_ui_throttle_percent").get_parameter_value().string_value
            self._pub_ui_manual = self.create_publisher(Bool, t_man, 10)
            self._pub_ui_throttle = self.create_publisher(Float32, t_thr, 10)
            self.get_logger().info(
                "上位机状态话题：%s（Bool 手动=true）%s（Float32 臂油门 0～100）" % (t_man, t_thr)
            )

        self._left_joy: Optional[Joy] = None
        self._right_joy: Optional[Joy] = None
        self._joint_pos: Dict[str, float] = {}
        self._js_received = False

        self._prev_pick = False
        self._prev_auto_open = False
        self._prev_auto_close = False
        self._prev_right_ready_pressed = False
        self._prev_right_estop_btn = False
        self._prev_right_clear_btn_pressed = False
        self._prev_auto_grip_axis_sign = 0
        self._prev_manual_mode = False

        self._arm_goal_handle = None
        self._hand_goal_handle = None

        self._arm_servers_logged = False

        self.create_timer(self._dt, self._control_timer)
        self.get_logger().info(
            "手柄桥接节点已启动：左手话题=%s 右手话题=%s 关节状态=%s 机械臂命名空间=%s"
            % (self._topic_left, self._topic_right, self._topic_js, self._manip_ns)
        )
        self.get_logger().info(
            "手动满油门角速度：臂 %.1f °/s，夹爪 %.1f °/s（内部 rad/s 与 joint_states 一致）"
            % (arm_deg_s, grip_deg_s)
        )

    def _publish_joy_ui_status(self, manual: bool, throttle_scale_arm: float) -> None:
        if self._pub_ui_manual is None or self._pub_ui_throttle is None:
            return
        msg_m = Bool()
        msg_m.data = bool(manual)
        self._pub_ui_manual.publish(msg_m)
        msg_t = Float32()
        msg_t.data = float(_clamp(throttle_scale_arm * 100.0, 0.0, 100.0))
        self._pub_ui_throttle.publish(msg_t)

    def _joy_mode_log_suffix(self, hand_label: str, left_ref: Optional[Joy]) -> str:
        """Joy 调试日志尾部：标明自动/手动，避免误以为按键变化即会驱关节。"""
        manual = self._compute_manual_mode(left_ref)
        if manual:
            return " [手动模式]"
        if hand_label == "左手柄":
            return " [自动模式：关节±键与手动夹爪轨迹不生效]"
        return " [自动模式：关节微动关闭；右手抓取/开闭夹爪服务仍有效]"

    def _annotate_right_joy_ready_button(self, hand_label: str, ob: Tuple[int, ...], nb: Tuple[int, ...], bline: str) -> str:
        if hand_label != "右手柄" or self._log_right_ready_btn < 0 or not bline:
            return bline
        idx = self._log_right_ready_btn
        if idx >= len(ob) or idx >= len(nb):
            return bline
        o_prev = int(ob[idx])
        n_next = int(nb[idx])
        if n_next != 0 or o_prev == 0 or o_prev == n_next:
            return bline
        token = "buttons[%d] %s→%s" % (idx, o_prev, n_next)
        if token in bline:
            return bline.replace(token, token + "（回 ready 位）")
        return bline

    def _log_joy_changed(self, hand_label: str, topic: str, prev_snap, msg: Joy) -> None:
        snap = _joy_snapshot(msg)
        eps = self._log_joy_axis_eps
        left_for_mode = msg if hand_label == "左手柄" else self._left_joy
        mode_suffix = self._joy_mode_log_suffix(hand_label, left_for_mode)
        if prev_snap is None:
            bt, ax = snap
            self.get_logger().info(
                "【%s】话题=%s 首帧：buttons(len=%d)=%s | axes(len=%d)=%s%s"
                % (
                    hand_label,
                    topic,
                    len(bt),
                    list(bt),
                    len(ax),
                    ["%.4f" % v for v in ax],
                    mode_suffix,
                )
            )
            return
        if _joy_snap_semantically_equal(prev_snap, snap, eps):
            return
        ob, oa = prev_snap
        nb, na = snap
        bline = self._annotate_right_joy_ready_button(
            hand_label, ob, nb, _format_button_diff(ob, nb)
        )
        aline = _format_axis_diff(oa, na, eps)
        detail_parts: List[str] = []
        if bline:
            detail_parts.append(bline)
        if aline:
            detail_parts.append(aline)
        if not detail_parts:
            detail = "buttons/axes 布局变化（详见首帧与传感器文档）"
        else:
            detail = " | ".join(detail_parts)
        self.get_logger().info(
            "【%s】话题=%s 变化：%s%s" % (hand_label, topic, detail, mode_suffix)
        )

    def _cb_left_joy(self, msg: Joy) -> None:
        if self._log_joy_change:
            self._log_joy_changed("左手柄", self._topic_left, self._snap_left_prev, msg)
            self._snap_left_prev = _joy_snapshot(msg)
        self._left_joy = msg

    def _cb_right_joy(self, msg: Joy) -> None:
        if self._log_joy_change:
            self._log_joy_changed("右手柄", self._topic_right, self._snap_right_prev, msg)
            self._snap_right_prev = _joy_snapshot(msg)
        self._right_joy = msg

    def _cb_joint_state(self, msg: JointState) -> None:
        self._js_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_pos[name] = float(msg.position[i])

    def _speed_scale_for(self, joy: Optional[Joy], axis_index: int) -> float:
        ax = _joy_axis(joy, axis_index)
        if self._speed_axis_invert:
            ax = -ax
        db = self._axis_deadband
        if self._speed_axis_map_mode in ("throttle_full_span", "throttle", "full_span"):
            # axis ∈ [-1,1] → 比例 [0,1]：-1 = 0%、+1 = 100%
            u = (ax + 1.0) * 0.5
            u = float(_clamp(u, 0.0, 1.0))
            if db <= 1.0e-9:
                return u
            if u <= db:
                return 0.0
            span = max(1.0e-6, 1.0 - db)
            return float(_clamp((u - db) / span, 0.0, 1.0))
        a = abs(ax)
        if a <= db:
            return 0.0
        span = max(1.0e-6, 1.0 - db)
        return float(_clamp((a - db) / span, 0.0, 1.0))

    def _compute_manual_mode(self, left: Optional[Joy]) -> bool:
        if self._mode_source == "axis":
            return _joy_axis(left, self._mode_axis_index) > self._mode_axis_thr
        pressed = _joy_button(left, self._mode_button_index)
        if self._mode_manual_when_pressed:
            return pressed
        return not pressed

    def _arm_pair_sign(self, msg: Optional[Joy], plus_i: int, minus_i: int) -> int:
        p = _joy_button(msg, plus_i)
        m = _joy_button(msg, minus_i)
        if p and m:
            return 0
        if p:
            return 1
        if m:
            return -1
        return 0

    def _send_traj(self, client: ActionClient, goal_handle_attr: str, joint_names: List[str], positions: List[float]) -> None:
        if len(joint_names) != len(positions):
            self.get_logger().error("关节名与位置数量不匹配")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        dur_sec = max(0.05, self._dt * self._traj_margin)
        sec = int(dur_sec)
        nanosec = int(round((dur_sec - sec) * 1.0e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec = 0
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal.trajectory.points.append(pt)

        gh_old = getattr(self, goal_handle_attr)
        if gh_old is not None:
            gh_old.cancel_goal_async()
            setattr(self, goal_handle_attr, None)

        def _resp_cb(fut, attr=goal_handle_attr):
            try:
                gh = fut.result()
            except Exception as ex:  # noqa: BLE001
                self.get_logger().warn("轨迹 Action 目标异步失败: %s" % ex)
                return
            if gh is None:
                return
            if not gh.accepted:
                self.get_logger().warn("FollowJointTrajectory 目标被拒绝")
                return
            setattr(self, attr, gh)

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(_resp_cb)

    def _maybe_call_trigger(self, client, log_name: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn("服务未就绪，跳过调用: %s" % log_name)
            return
        req = Trigger.Request()
        future = client.call_async(req)
        future.add_done_callback(
            lambda f, n=log_name: self._trigger_done_cb(f, n)
        )

    def _trigger_done_cb(self, future, name: str) -> None:
        try:
            res = future.result()
        except Exception as ex:  # noqa: BLE001
            self.get_logger().error("服务调用失败 [%s]: %s" % (name, ex))
            return
        if res is None:
            return
        if not res.success:
            self.get_logger().warn("服务返回失败 [%s]: %s" % (name, res.message))
        else:
            self.get_logger().info("服务调用成功 [%s]: %s" % (name, res.message))

    def _on_estop_done_then_clear_estop(self, future) -> None:
        """切回自动：急停回调后再 clear_estop（急停失败也尝试解除闭锁）。"""
        try:
            res = future.result()
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn("切回自动：emergency_stop 异常: %s，仍尝试 clear_estop" % ex)
            self._maybe_call_trigger(self._srv_clear_estop, "clear_estop")
            return
        if res is None:
            self.get_logger().warn("切回自动：emergency_stop 无响应，仍尝试 clear_estop")
            self._maybe_call_trigger(self._srv_clear_estop, "clear_estop")
            return
        if res.success:
            self.get_logger().info("切回自动：emergency_stop 已确认，接着调用 clear_estop")
        else:
            self.get_logger().warn(
                "切回自动：emergency_stop 返回失败 (%s)，仍尝试 clear_estop" % res.message
            )
        self._maybe_call_trigger(self._srv_clear_estop, "clear_estop")

    def _call_estop_then_clear_estop_for_auto_mode(self) -> None:
        """切回自动模式：先急停再解闭锁，避免残留轨迹与闭锁并存。"""
        if not self._srv_clear_estop.service_is_ready():
            self.get_logger().warn("切回自动：clear_estop 未就绪，跳过")
            return
        if not self._srv_estop.service_is_ready():
            self.get_logger().warn("切回自动：emergency_stop 未就绪，仅调用 clear_estop")
            self._maybe_call_trigger(self._srv_clear_estop, "clear_estop")
            return
        req = Trigger.Request()
        estop_future = self._srv_estop.call_async(req)
        estop_future.add_done_callback(self._on_estop_done_then_clear_estop)

    def _control_timer(self) -> None:
        left = self._left_joy
        right = self._right_joy
        manual = self._compute_manual_mode(left)
        scale_arm = self._speed_scale_for(left, self._speed_axis_index)
        self._publish_joy_ui_status(manual, scale_arm)

        if manual != self._prev_manual_mode:
            if manual:
                self.get_logger().info(
                    "【模式切换】自动 -> 手动：已启用手柄关节微动；通过 pick_trigger 抓取已关闭"
                )
            else:
                self.get_logger().info(
                    "【模式切换】手动 -> 自动：可通过 pick_trigger 执行抓取"
                )

        if manual and not self._prev_manual_mode and self._estop_on_manual:
            self.get_logger().warn("进入手动模式：正在调用急停服务（emergency_stop）")
            self._maybe_call_trigger(self._srv_estop, "急停")

        if not manual and self._prev_manual_mode and self._clear_estop_on_auto:
            self.get_logger().info("切回自动模式：先 emergency_stop 再 clear_estop")
            self._call_estop_then_clear_estop_for_auto_mode()

        pick_now = _joy_button(right, self._pick_btn)
        if manual:
            if pick_now and not self._prev_pick:
                self.get_logger().warn(
                    "手动模式：已忽略抓取键（未发送 pick_trigger）；请切到自动模式后再执行抓取"
                )
        else:
            if pick_now and not self._prev_pick:
                self._pub_pick.publish(Empty())
                self.get_logger().info("自动模式：已发布 pick_trigger")

        if right is not None:
            if self._right_estop_btn_i >= 0:
                pe = _joy_button(right, self._right_estop_btn_i)
                if pe and (not self._prev_right_estop_btn):
                    self.get_logger().warn(
                        "右手 buttons[%d] 上升沿 → emergency_stop" % self._right_estop_btn_i
                    )
                    self._maybe_call_trigger(self._srv_estop, "emergency_stop")
                self._prev_right_estop_btn = pe
            else:
                self._prev_right_estop_btn = False
            if self._right_clear_btn_i >= 0:
                pc = _joy_button(right, self._right_clear_btn_i)
                if (not pc) and self._prev_right_clear_btn_pressed:
                    self.get_logger().info(
                        "右手 buttons[%d] 松开 → clear_estop" % self._right_clear_btn_i
                    )
                    self._maybe_call_trigger(self._srv_clear_estop, "clear_estop")
                self._prev_right_clear_btn_pressed = pc
            else:
                self._prev_right_clear_btn_pressed = False
        else:
            self._prev_right_estop_btn = False
            self._prev_right_clear_btn_pressed = False

        if not manual:
            if self._prev_manual_mode and self._manual_r_grip_axis_i >= 0 and right is not None:
                self._prev_auto_grip_axis_sign = _joy_axis_tri_sign(
                    right,
                    self._manual_r_grip_axis_i,
                    float(self._manual_r_grip_axis_thr),
                )

            # 自动开闭夹爪：可选键位上升沿（下标须与急停/解刹错开）；或与手动共用右手 axes[] 边沿（+1→open_gripper，-1→close_gripper）
            auto_open = _joy_button(right, self._auto_g_open)
            auto_close = _joy_button(right, self._auto_g_close)
            if auto_open and not self._prev_auto_open:
                self._maybe_call_trigger(self._srv_open, "打开夹爪")
            if auto_close and not self._prev_auto_close:
                self._maybe_call_trigger(self._srv_close, "关闭夹爪")
            self._prev_auto_open = auto_open
            self._prev_auto_close = auto_close

            if self._manual_r_grip_axis_i >= 0 and right is not None:
                s_ax = _joy_axis_tri_sign(
                    right,
                    self._manual_r_grip_axis_i,
                    float(self._manual_r_grip_axis_thr),
                )
                if s_ax == 1 and self._prev_auto_grip_axis_sign != 1:
                    self.get_logger().info("自动模式：右手 axes[%d] → +1，调用 open_gripper" % self._manual_r_grip_axis_i)
                    self._maybe_call_trigger(self._srv_open, "open_gripper")
                if s_ax == -1 and self._prev_auto_grip_axis_sign != -1:
                    self.get_logger().info("自动模式：右手 axes[%d] → -1，调用 close_gripper" % self._manual_r_grip_axis_i)
                    self._maybe_call_trigger(self._srv_close, "close_gripper")
                self._prev_auto_grip_axis_sign = s_ax

            if self._log_right_ready_btn >= 0:
                ready_pressed = _joy_button(right, self._log_right_ready_btn)
                if (
                    self._auto_ready_release_go
                    and (not ready_pressed)
                    and self._prev_right_ready_pressed
                ):
                    self.get_logger().info(
                        "自动模式：右手 buttons[%d] 回 ready 位，调用 go_to_ready"
                        % self._log_right_ready_btn
                    )
                    self._maybe_call_trigger(self._srv_go_ready, "go_to_ready")
                self._prev_right_ready_pressed = ready_pressed
        else:
            self._prev_auto_open = False
            self._prev_auto_close = False
            if self._manual_r_grip_axis_i >= 0 and right is not None:
                self._prev_auto_grip_axis_sign = _joy_axis_tri_sign(
                    right,
                    self._manual_r_grip_axis_i,
                    float(self._manual_r_grip_axis_thr),
                )
            if self._log_right_ready_btn >= 0:
                self._prev_right_ready_pressed = _joy_button(right, self._log_right_ready_btn)
            else:
                self._prev_right_ready_pressed = False

        self._prev_pick = pick_now
        self._prev_manual_mode = manual

        if not manual:
            return

        if not self._js_received:
            self.get_logger().warn("尚未收到 joint_states，手动微动暂停", throttle_duration_sec=2.0)
            return

        if self._req_arm and not self._arm_client.server_is_ready():
            self.get_logger().warn("臂关节 FollowJointTrajectory 动作服务未就绪", throttle_duration_sec=3.0)
            return
        if self._req_hand and not self._hand_client.server_is_ready():
            self.get_logger().warn("手部 FollowJointTrajectory 动作服务未就绪", throttle_duration_sec=3.0)
            return
        if not self._arm_servers_logged:
            self.get_logger().info("手动微动所需轨迹 Action 服务已就绪")
            self._arm_servers_logged = True

        arm_q: List[float] = []
        for i, name in enumerate(self._arm_joint_names):
            if name not in self._joint_pos:
                self.get_logger().warn("joint_states 中缺少关节: %s" % name, throttle_duration_sec=3.0)
                return
            q0 = self._joint_pos[name]
            sg = self._arm_pair_sign(left, self._arm_plus[i], self._arm_minus[i])
            dq = float(sg) * self._arm_max_speed * scale_arm * self._dt
            lo = self._arm_low[i]
            hi = self._arm_high[i]
            arm_q.append(_clamp(q0 + dq, lo, hi))

        h7n = self._hand_joint_names[0]
        h8n = self._hand_joint_names[1]
        if h7n not in self._joint_pos or h8n not in self._joint_pos:
            self.get_logger().warn("joint_states 中缺少手部关节", throttle_duration_sec=3.0)
            return
        q7 = self._joint_pos[h7n]
        q8 = self._joint_pos[h8n]
        go = _joy_button(left, self._grip_open_i)
        gc = _joy_button(left, self._grip_close_i)
        ax_grip = 0
        if self._manual_r_grip_axis_i >= 0 and right is not None:
            ax_grip = _joy_axis_tri_sign(right, self._manual_r_grip_axis_i, float(self._manual_r_grip_axis_thr))
        open_req = bool(go or (ax_grip > 0))
        close_req = bool(gc or (ax_grip < 0))
        joy_g = right if self._gripper_speed_joy == "right" else left
        g_axis = (
            self._gripper_speed_axis_index
            if self._gripper_speed_axis_index >= 0
            else self._speed_axis_index
        )
        if self._manual_gripper_use_throttle:
            scale_grip = self._speed_scale_for(joy_g, g_axis)
        else:
            scale_grip = 1.0
        vg = self._grip_max_speed * scale_grip * self._dt * self._grip_speed_scale
        if open_req and close_req:
            pass
        elif open_req:
            q7 = _clamp(q7 + vg, min(self._hq7_close, self._hq7_open), max(self._hq7_close, self._hq7_open))
            q8 = _clamp(q8 - vg, min(self._hq8_close, self._hq8_open), max(self._hq8_close, self._hq8_open))
        elif close_req:
            q7 = _clamp(q7 - vg, min(self._hq7_close, self._hq7_open), max(self._hq7_close, self._hq7_open))
            q8 = _clamp(q8 + vg, min(self._hq8_close, self._hq8_open), max(self._hq8_close, self._hq8_open))

        hand_q = [q7, q8]

        if self._log_targets:
            arm_d = ["%.2f" % (q * _RAD_TO_DEG) for q in arm_q]
            hand_d = ["%.2f" % (q * _RAD_TO_DEG) for q in hand_q]
            self.get_logger().info(
                "手动目标 臂(°)=[%s] 手(°)=[%s] 臂油门=%.3f 夹爪油门=%.3f"
                % (", ".join(arm_d), ", ".join(hand_d), scale_arm, scale_grip)
            )

        self._send_traj(self._arm_client, "_arm_goal_handle", self._arm_joint_names, arm_q)
        self._send_traj(self._hand_client, "_hand_goal_handle", self._hand_joint_names, hand_q)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = JoyManipulatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
