#!/usr/bin/env python3
"""Run repeatable report-validation cases and record their ROS 2 data."""

import argparse
import csv
import math
import os
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from marine_acoustic_msgs.msg import Dvl
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState, FluidPressure, Imu
from std_msgs.msg import Bool, UInt8

from zemheri_interfaces.msg import (
    AuvState,
    GuidanceGoal,
    MissionStatus,
    NavigationStatus,
)
from zemheri_simulation_plugins.srv import SetCurrentMode, SetCurrentTarget
from zemheri_simulation_plugins.srv import (
    ClearCurrentSchedule,
    LoadBuiltInScenario,
    ResetCurrentEpisode,
    ScheduleCurrentEvent,
    SetCurrentPreset,
    TriggerGust,
)


TOPICS = [
    "/ground_truth/odometry",
    "/odometry/ukf",
    "/validation/resilience/raw_ukf",
    "/validation/resilience/protected_ukf",
    "/validation/resilience/oosm_ukf",
    "/validation/resilience/status",
    "/validation/resilience/disturbed_dvl",
    "/navigation/status",
    "/dvl/raw",
    "/dvl/quality_twist",
    "/imu/data",
    "/imu/data_raw",
    "/pressure/raw",
    "/pressure/depth_pose",
    "/auv/state",
    "/auv/mission/status",
    "/auv/failsafe/reset",
    "/auv/fire/permission_candidate",
    "/auv/fire/status",
    "/auv/fire/actuator_command",
    "/auv/fire/separation_confirmed",
    "/auv/fire/capacitive_surface_valid",
    "/auv/fire/capacitive_sensor_z",
    "/guidance/goal",
    "/guidance/setpoint",
    "/sara_uuv/cmd_vel",
    "/sara_uuv/propeller/cmd_angvel",
    "/sara_uuv/fin_1/cmd_pos",
    "/sara_uuv/fin_2/cmd_pos",
    "/sara_uuv/fin_3/cmd_pos",
    "/sara_uuv/fin_4/cmd_pos",
    "/ocean_current",
    "/ocean_current/status",
    "/battery/state",
    "/tf",
    "/tf_static",
    "/clock",
]

VALIDATION_PARAMETER_TARGETS = {
    "/guidance_node": [
        "use_sim_time",
        "publish_rate_hz",
        "goal_timeout_sec",
        "state_timeout_sec",
        "los_lookahead_m",
        "max_los_heading_correction_deg",
        "max_target_speed_mps",
        "degraded_navigation_speed_scale",
    ],
    "/resilience_dvl_delay_node": [
        "use_sim_time",
        "input_topic",
        "output_topic",
        "delay_seconds",
        "maximum_queue_size",
    ],
    "/resilience_dvl_dropout_node": [
        "use_sim_time",
        "input_topic",
        "output_topic",
        "pass_seconds",
        "dropout_seconds",
        "repeat",
        "start_on_motion",
        "motion_speed_threshold",
    ],
    "/validation/resilience/oosm/ukf_filter_node_odom": [
        "use_sim_time",
        "frequency",
        "sensor_timeout",
        "smooth_lagged_data",
        "history_length",
    ],
}

SENSOR_HEALTH_REQUIRED_TOPICS = {
    "/auv/state",
    "/imu/data",
    "/dvl/raw",
    "/dvl/quality_twist",
    "/pressure/raw",
    "/pressure/depth_pose",
    "/battery/state",
    "/navigation/status",
}


class ReportTestRunner(Node):
    """Publish the selected test reference while a rosbag is recorded."""

    def __init__(self, args):
        super().__init__("report_test_runner")
        self.args = args
        self.goal_pub = self.create_publisher(GuidanceGoal, "/guidance/goal", 10)
        self.start_pub = self.create_publisher(UInt8, "/auv/mission/start", 10)
        self.failsafe_reset_pub = self.create_publisher(
            Bool, "/auv/failsafe/reset", 10
        )
        self.current_mode_client = self.create_client(
            SetCurrentMode, "/ocean_current/set_mode"
        )
        self.current_target_client = self.create_client(
            SetCurrentTarget, "/ocean_current/set_target"
        )
        self.current_gust_client = self.create_client(
            TriggerGust, "/ocean_current/trigger_gust"
        )
        self.current_reset_client = self.create_client(
            ResetCurrentEpisode, "/ocean_current/reset_episode"
        )
        self.current_schedule_client = self.create_client(
            ScheduleCurrentEvent, "/ocean_current/schedule_event"
        )
        self.current_clear_schedule_client = self.create_client(
            ClearCurrentSchedule, "/ocean_current/clear_schedule"
        )
        self.current_preset_client = self.create_client(
            SetCurrentPreset, "/ocean_current/set_preset"
        )
        self.current_scenario_client = self.create_client(
            LoadBuiltInScenario, "/ocean_current/load_built_in_scenario"
        )
        self.create_subscription(
            MissionStatus, "/auv/mission/status", self.mission_status_cb, 10
        )
        self.create_subscription(AuvState, "/auv/state", self.state_cb, 20)
        self.sensor_health_topics_seen = set()
        if self.args.case == "sensor_health":
            self._create_sensor_health_subscriptions()
        self.started_at = None
        self.wall_created_at = time.monotonic()
        self.state = None
        self.origin = None
        self.initial_yaw = None
        self.waypoint_index = 0
        self.waypoints = []
        self.start_sent = False
        self.failsafe_reset_sent = False
        self.waiting_for_stack_logged = False
        self.waiting_for_failsafe_clear_logged = False
        self.warmup_logged = False
        self.done = False
        self.stop_reason = ""
        self.rl_phase = "WARMUP"
        self.rl_settle_since = None
        self.rl_episode_started_at = None
        self.ocean_current_service_results = []
        self.create_timer(0.1, self.tick)

    def is_rl_case(self):
        return self.args.case.startswith("rl_policy")

    def _create_sensor_health_subscriptions(self):
        """Require actual sensor flow before starting a sensor-health test."""
        subscriptions = [
            (Imu, "/imu/data"),
            (Dvl, "/dvl/raw"),
            (TwistWithCovarianceStamped, "/dvl/quality_twist"),
            (FluidPressure, "/pressure/raw"),
            (PoseWithCovarianceStamped, "/pressure/depth_pose"),
            (BatteryState, "/battery/state"),
            (NavigationStatus, "/navigation/status"),
        ]
        for message_type, topic in subscriptions:
            self.create_subscription(
                message_type,
                topic,
                lambda _msg, topic=topic: self._mark_sensor_health_topic(topic),
                10,
            )

    def _mark_sensor_health_topic(self, topic):
        self.sensor_health_topics_seen.add(topic)
        self._start_sensor_health_timer_if_ready()

    def _start_sensor_health_timer_if_ready(self):
        if (
            self.args.case == "sensor_health"
            and self.started_at is None
            and SENSOR_HEALTH_REQUIRED_TOPICS <= self.sensor_health_topics_seen
        ):
            self.started_at = self.get_clock().now()
            self.get_logger().info(
                "All required sensor topics received; validation timer started."
            )

    def elapsed(self):
        if self.started_at is None:
            return 0.0
        return (self.get_clock().now() - self.started_at).nanoseconds * 1e-9

    def rl_episode_elapsed(self):
        if self.rl_episode_started_at is None:
            return 0.0
        return (
            self.get_clock().now() - self.rl_episode_started_at
        ).nanoseconds * 1e-9

    def mission_status_cb(self, msg):
        if (
            self.start_sent
            and self.args.case in {"stage1_fsm", "stage2_bt"}
            and (msg.mission_complete or msg.failsafe_active)
        ):
            self.get_logger().info(
                f"Mission terminal state: {msg.stage_name}"
            )
            self.stop_reason = f"mission_terminal:{msg.stage_name}"
            self.done = True

    def state_cb(self, msg):
        self.state = msg
        if self.args.case == "sensor_health":
            self.sensor_health_topics_seen.add("/auv/state")
            self._start_sensor_health_timer_if_ready()
        elif self.started_at is None:
            self.started_at = self.get_clock().now()
            self.get_logger().info(
                "First /auv/state received; validation timer started."
            )
        if self.origin is None:
            self.origin = (float(msg.x), float(msg.y))
            self.initial_yaw = float(msg.yaw)
            distance = float(self.args.distance)
            cross_track = float(self.args.cross_track)
            self.waypoints = [
                (self.origin[0] + 0.30 * distance, self.origin[1]),
                (
                    self.origin[0] + 0.55 * distance,
                    self.origin[1] + cross_track,
                ),
                (
                    self.origin[0] + 0.80 * distance,
                    self.origin[1] + cross_track,
                ),
                (self.origin[0] + distance, self.origin[1]),
            ]

    def publish_goal(
        self,
        mode,
        target_x,
        target_y,
        target_yaw=0.0,
        target_speed=None,
        target_depth=None,
    ):
        msg = GuidanceGoal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.target_x = float(target_x)
        msg.target_y = float(target_y)
        msg.target_depth = float(
            self.args.depth if target_depth is None else target_depth
        )
        msg.target_yaw = float(target_yaw)
        msg.target_pitch = 0.0
        msg.target_speed = (
            0.0
            if mode == "STOP"
            else float(self.args.speed if target_speed is None else target_speed)
        )
        msg.guidance_mode = mode
        self.goal_pub.publish(msg)

    def tick(self):
        if self.state is None:
            if time.monotonic() - self.wall_created_at >= self.args.state_wait_timeout:
                self.stop_reason = "state_wait_timeout"
                self.get_logger().error(
                    "No /auv/state received before wall-time startup timeout."
                )
                self.done = True
                return
            if not self.waiting_for_stack_logged:
                self.get_logger().warn(
                    "Waiting for first /auv/state before starting validation timer."
                )
                self.waiting_for_stack_logged = True
            return
        if self.args.case == "sensor_health" and self.started_at is None:
            if time.monotonic() - self.wall_created_at >= self.args.state_wait_timeout:
                missing = sorted(
                    SENSOR_HEALTH_REQUIRED_TOPICS - self.sensor_health_topics_seen
                )
                self.stop_reason = "sensor_stack_wait_timeout"
                self.get_logger().error(
                    "Required sensor topics were not received before wall-time "
                    f"startup timeout: {missing}"
                )
                self.done = True
                return
            if not self.waiting_for_stack_logged:
                self.get_logger().warn(
                    "Waiting for all required sensor topics before starting "
                    "sensor-health validation."
                )
                self.waiting_for_stack_logged = True
            return

        duration_elapsed = (
            self.rl_episode_elapsed() if self.is_rl_case() else self.elapsed()
        )
        if (
            (not self.is_rl_case() or self.rl_episode_started_at is not None)
            and duration_elapsed >= self.args.duration
        ):
            self.publish_goal("STOP", 0.0, 0.0)
            self.stop_reason = "duration_limit_reached"
            self.get_logger().warn(
                "Test ROS-time duration limit reached; stopping recording "
                "without requesting mission abort."
            )
            self.done = True
            return

        if (
            (
                self.args.case in {
                "navigation_straight", "navigation_resilience",
                "guidance_los", "guidance_waypoint", "controller_tracking",
                "sensor_health", "ocean_current_response",
                "ocean_current_services",
                }
                or self.is_rl_case()
            )
            and self.elapsed() < self.args.warmup
        ):
            self.publish_goal("STOP", 0.0, 0.0)
            if not self.warmup_logged:
                self.get_logger().info(
                    f"Navigation filter warm-up: {self.args.warmup:.1f} s"
                )
                self.warmup_logged = True
        elif self.args.case in {"navigation_straight", "navigation_resilience"}:
            self.publish_goal("CRUISE", self.args.distance, 0.0)
        elif self.args.case == "guidance_los":
            if self.origin is None:
                return
            self.publish_goal(
                "LOS",
                self.origin[0] + self.args.distance,
                self.origin[1] + self.args.cross_track,
            )
        elif self.args.case == "guidance_waypoint":
            self._run_waypoint_route()
        elif self.args.case == "controller_tracking":
            if self.origin is None or self.initial_yaw is None:
                return
            yaw_step = (
                0.0 if self.elapsed() < 0.55 * self.args.duration
                else math.radians(20.0)
            )
            self.publish_goal(
                "HOLD",
                self.origin[0] + self.args.distance,
                self.origin[1],
                self.initial_yaw + yaw_step,
            )
        elif self.is_rl_case():
            self._run_rl_policy()
        elif self.args.case == "sensor_health":
            self.publish_goal("STOP", 0.0, 0.0)
        elif self.args.case == "ocean_current_response":
            if self.origin is None or self.initial_yaw is None:
                return
            self.publish_goal(
                "HOLD",
                self.origin[0],
                self.origin[1],
                target_yaw=self.initial_yaw,
                target_speed=0.0,
                target_depth=self.args.depth,
            )
        elif self.args.case == "ocean_current_services":
            if self.origin is None or self.initial_yaw is None:
                return
            self.publish_goal(
                "HOLD",
                self.origin[0],
                self.origin[1],
                target_yaw=self.initial_yaw,
                target_speed=0.0,
                target_depth=self.args.depth,
            )
        elif not self.start_sent:
            mission_ready = self.start_pub.get_subscription_count() > 0
            guidance_ready = self.goal_pub.get_subscription_count() > 0
            reset_ready = self.failsafe_reset_pub.get_subscription_count() > 0
            if not self.failsafe_reset_sent and reset_ready:
                self.failsafe_reset_pub.publish(Bool(data=True))
                self.failsafe_reset_sent = True
                self.get_logger().info(
                    "Failsafe reset requested before mission start."
                )
                return
            if (
                self.failsafe_reset_sent
                and self.state is not None
                and self.state.failsafe_active
            ):
                self.failsafe_reset_pub.publish(Bool(data=True))
                if not self.waiting_for_failsafe_clear_logged:
                    self.get_logger().warn(
                        "Waiting for /auv/state failsafe_active to clear..."
                    )
                    self.waiting_for_failsafe_clear_logged = True
                return
            state_ready = self.state is not None and not self.state.failsafe_active
            if mission_ready and guidance_ready and reset_ready and state_ready:
                msg = UInt8()
                msg.data = 1 if self.args.case == "stage1_fsm" else 2
                self.start_pub.publish(msg)
                self.start_sent = True
                self.get_logger().info("Mission and guidance ready; stage started.")
            elif not self.waiting_for_stack_logged:
                self.get_logger().warn(
                    "Waiting for mission, guidance and /auv/failsafe/reset "
                    "subscribers. Restart the simulation stack after rebuilding "
                    "if the reset subscriber is unavailable."
                )
                self.waiting_for_stack_logged = True

    def _run_waypoint_route(self):
        if self.state is None or not self.waypoints:
            return
        target_x, target_y = self.waypoints[self.waypoint_index]
        distance = ((target_x - self.state.x) ** 2
                    + (target_y - self.state.y) ** 2) ** 0.5
        start_x, start_y = (
            self.origin
            if self.waypoint_index == 0
            else self.waypoints[self.waypoint_index - 1]
        )
        segment_x = target_x - start_x
        segment_y = target_y - start_y
        segment_length = math.hypot(segment_x, segment_y)
        axis_x = segment_x / max(segment_length, 1e-9)
        axis_y = segment_y / max(segment_length, 1e-9)
        relative_x = self.state.x - start_x
        relative_y = self.state.y - start_y
        along_track = relative_x * axis_x + relative_y * axis_y
        cross_track = -relative_x * axis_y + relative_y * axis_x
        passed_waypoint = (
            along_track >= segment_length - self.args.waypoint_acceptance
            and abs(cross_track) <= 2.0 * self.args.waypoint_acceptance
        )
        if distance <= self.args.waypoint_acceptance or passed_waypoint:
            self.get_logger().info(
                f"Waypoint {self.waypoint_index + 1} accepted "
                f"at {distance:.2f} m; cross-track={cross_track:.2f} m"
            )
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.publish_goal("STOP", target_x, target_y)
                self.done = True
                return
            target_x, target_y = self.waypoints[self.waypoint_index]
        self.publish_goal("WAYPOINT", target_x, target_y)

    def _run_rl_policy(self):
        """Apply the selected policy candidate through the real ROS stack."""
        if self.state is None or self.origin is None or self.initial_yaw is None:
            return

        if self.rl_episode_started_at is None:
            self._run_rl_dive_and_settle()
            return

        measured_speed = math.sqrt(
            self.state.vx ** 2 + self.state.vy ** 2 + self.state.vz ** 2
        )
        speed = self.args.speed + 0.06
        if self.state.navigation_degraded:
            speed *= 0.55
        if measured_speed > 2.25:
            speed *= max(0.0, (2.50 - measured_speed) / 0.25)
        speed = max(0.0, min(1.50, speed))

        distance = float(self.args.distance)
        target_x = self.origin[0] + distance * math.cos(self.initial_yaw)
        target_y = self.origin[1] + distance * math.sin(self.initial_yaw)
        relative_x = self.state.x - self.origin[0]
        relative_y = self.state.y - self.origin[1]
        along_track = (
            relative_x * math.cos(self.initial_yaw)
            + relative_y * math.sin(self.initial_yaw)
        )
        cross_track = (
            -relative_x * math.sin(self.initial_yaw)
            + relative_y * math.cos(self.initial_yaw)
        )
        if (
            along_track >= distance - self.args.waypoint_acceptance
            and abs(cross_track) <= 2.0 * self.args.waypoint_acceptance
        ):
            self.publish_goal("STOP", target_x, target_y)
            self.stop_reason = "rl_target_reached"
            self.get_logger().info(
                "RL policy target reached; "
                f"along-track={along_track:.2f} m, "
                f"cross-track={cross_track:.2f} m"
            )
            self.done = True
            return
        self.publish_goal(
            "LOS",
            target_x,
            target_y,
            target_yaw=self.initial_yaw + 0.018,
            target_speed=speed,
            target_depth=max(0.1, self.args.depth - 0.03),
        )

    def _run_rl_dive_and_settle(self):
        """Reach operation depth before opening the RL evaluation window."""
        now = self.get_clock().now()
        depth_error = abs(float(self.args.depth) - float(self.state.depth))
        if depth_error <= self.args.rl_depth_tolerance:
            if self.rl_settle_since is None:
                self.rl_settle_since = now
                self.get_logger().info(
                    "RL depth tolerance reached; stability window started."
                )
            stable_for = (now - self.rl_settle_since).nanoseconds * 1e-9
        else:
            self.rl_settle_since = None
            stable_for = 0.0

        if stable_for >= self.args.rl_settle_duration:
            self.rl_phase = "EVALUATION"
            self.rl_episode_started_at = now
            self.origin = (float(self.state.x), float(self.state.y))
            self.initial_yaw = float(self.state.yaw)
            self.get_logger().info(
                "RL evaluation window started; "
                f"depth={self.state.depth:.2f} m, "
                f"settled_for={stable_for:.2f} s"
            )
            return

        if self.elapsed() >= self.args.rl_settle_timeout:
            self.publish_goal("STOP", self.state.x, self.state.y)
            self.stop_reason = "rl_depth_settle_timeout"
            self.get_logger().error(
                "RL depth precondition failed; "
                f"depth={self.state.depth:.2f} m, "
                f"target={self.args.depth:.2f} m"
            )
            self.done = True
            return

        self.rl_phase = "DIVE_AND_SETTLE"
        self.publish_goal(
            "DEPTH_HOLD",
            self.state.x,
            self.state.y,
            target_yaw=self.initial_yaw,
            target_speed=self.args.rl_dive_speed,
            target_depth=self.args.depth,
        )

    def stop(self):
        """Stop test commands without turning normal cleanup into a failsafe."""
        self.publish_goal("STOP", 0.0, 0.0)

    def configure_ocean_current(self):
        """Configure the deterministic current used by current validation."""
        if self.is_rl_case():
            self._call_ocean_current_service(
                self.current_mode_client,
                SetCurrentMode.Request(mode="constant"),
                "set_mode_constant",
            )
            self._call_ocean_current_service(
                self.current_target_client,
                SetCurrentTarget.Request(
                    x=float(self.args.current_x),
                    y=float(self.args.current_y),
                    z=float(self.args.current_z),
                ),
                "set_target",
            )
            return
        if self.args.case == "ocean_current_response":
            self._call_ocean_current_service(
                self.current_mode_client,
                SetCurrentMode.Request(mode="constant"),
                "set_mode_constant",
            )
            self._call_ocean_current_service(
                self.current_target_client,
                SetCurrentTarget.Request(
                    x=float(self.args.current_x),
                    y=float(self.args.current_y),
                    z=float(self.args.current_z),
                ),
                "set_target",
            )
            self.get_logger().info(
                "Ocean current target configured: "
                f"({self.args.current_x:.2f}, {self.args.current_y:.2f}, "
                f"{self.args.current_z:.2f}) m/s"
            )
            return
        if self.args.case != "ocean_current_services":
            return
        self._call_ocean_current_service(
            self.current_mode_client,
            SetCurrentMode.Request(mode="constant"),
            "set_mode_constant",
        )
        self._call_ocean_current_service(
            self.current_target_client,
            SetCurrentTarget.Request(x=0.20, y=0.05, z=0.0),
            "set_target",
        )
        self._call_ocean_current_service(
            self.current_gust_client,
            TriggerGust.Request(x=0.15, y=0.08, z=0.0, duration=2.0),
            "trigger_gust",
        )
        self._call_ocean_current_service(
            self.current_schedule_client,
            ScheduleCurrentEvent.Request(
                start_time=1.0,
                end_time=8.0,
                mode="constant",
                x=0.25,
                y=0.10,
                z=0.0,
                overwrite=True,
            ),
            "schedule_event",
        )
        self._call_ocean_current_service(
            self.current_clear_schedule_client,
            ClearCurrentSchedule.Request(clear_all=True),
            "clear_schedule",
        )
        self._call_ocean_current_service(
            self.current_preset_client,
            SetCurrentPreset.Request(preset_name="marmara_transition"),
            "set_preset",
        )
        self._call_ocean_current_service(
            self.current_scenario_client,
            LoadBuiltInScenario.Request(
                scenario_name="marmara_transition_case",
                overwrite=True,
            ),
            "load_built_in_scenario",
        )
        self._call_ocean_current_service(
            self.current_reset_client,
            ResetCurrentEpisode.Request(randomize=False),
            "reset_episode",
        )

    def _call_ocean_current_service(self, client, request, label):
        if not client.wait_for_service(timeout_sec=5.0):
            self.ocean_current_service_results.append({
                "service": label,
                "success": False,
                "message": "service unavailable",
            })
            raise RuntimeError(f"Ocean current {label} service unavailable.")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result() if future.done() else None
        success = bool(response is not None and response.success)
        message = "" if response is None else response.message
        self.ocean_current_service_results.append({
            "service": label,
            "success": success,
            "message": message,
        })
        if not success:
            raise RuntimeError(f"Ocean current {label} configuration failed.")


def unique_output(root, case):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = root / f"{case}_{stamp}"
    path.mkdir(parents=True)
    return path


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def git_state():
    repository = Path(__file__).resolve().parents[3]
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repository,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=repository,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip())
        return {"commit": commit, "dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": "unknown", "dirty": None}


def collect_remote_parameters(node):
    collected = {}
    for remote_node, names in VALIDATION_PARAMETER_TARGETS.items():
        client = node.create_client(
            GetParameters,
            f"{remote_node.rstrip('/')}/get_parameters",
        )
        available = any(
            client.wait_for_service(timeout_sec=1.0) for _ in range(3)
        )
        if not available:
            collected[remote_node] = {"available": False}
            node.destroy_client(client)
            continue
        request = GetParameters.Request()
        request.names = names
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if not future.done():
            collected[remote_node] = {"available": False}
            node.destroy_client(client)
            continue
        try:
            response = future.result()
        except Exception as error:
            collected[remote_node] = {
                "available": True,
                "parameters": {},
                "collection_error": str(error),
            }
            node.get_logger().warn(
                f"Parameter collection skipped for {remote_node}: {error}"
            )
            node.destroy_client(client)
            continue
        if response is None:
            collected[remote_node] = {"available": False}
            node.destroy_client(client)
            continue
        collected[remote_node] = {
            "available": True,
            "parameters": {
                name: parameter_value_to_python(value)
                for name, value in zip(names, response.values)
            },
        }
        node.destroy_client(client)
    return collected


def write_manifest(output, manifest):
    with (output / "test_manifest.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            manifest,
            stream,
            allow_unicode=True,
            sort_keys=False,
        )

def write_ocean_current_service_results(output, results):
    if not results:
        return
    metrics = output / "metrics"
    metrics.mkdir(parents=True, exist_ok=True)
    accepted = all(bool(row["success"]) for row in results)
    summary = {
        "Doğrulama kararı": (
            "KABUL - akıntı servisleri yanıt verdi"
            if accepted
            else "BAŞARISIZ - akıntı servis doğrulaması tamamlanmadı"
        ),
        "Başarılı servis sayısı": sum(1 for row in results if row["success"]),
        "Toplam servis sayısı": len(results),
    }
    with (metrics / "ocean_current_service_results.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=["service", "success", "message"])
        writer.writeheader()
        writer.writerows(results)
    with (metrics / "ocean_current_service_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary))
        writer.writeheader()
        writer.writerow(summary)
    lines = [
        "| Servis | Başarı | Mesaj |",
        "|---|---:|---|",
    ]
    for row in results:
        lines.append(
            f"| `{row['service']}` | {row['success']} | {row['message']} |"
        )
    (metrics / "ocean_current_service_results.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    with (metrics / "ocean_current_service_summary.md").open(
        "w", encoding="utf-8"
    ) as stream:
        stream.write("| Ölçüt | Değer |\n|---|---:|\n")
        for key, value in summary.items():
            stream.write(f"| {key} | {value} |\n")


def start_recorder(output):
    """Start the central CSV/log/resource/rosbag recorder."""
    command = [
        "ros2", "launch", "zemheri_bringup", "system_recording.launch.py",
        f"output_root:={output}",
        "session_name:=recording",
        "use_sim_time:=true",
    ]
    return subprocess.Popen(command, start_new_session=True)


def start_mission(case):
    stage = "1" if case == "stage1_fsm" else "2"
    command = [
        "ros2", "launch", "zemheri_mission", "mission.launch.py",
        f"stage:={stage}",
        "use_sim_time:=true",
        "autostart:=false",
    ]
    return subprocess.Popen(command, start_new_session=True)


def start_validation(case):
    """Start optional algorithm-validation nodes for a test case."""
    if case != "navigation_resilience":
        return None
    command = [
        "ros2", "launch", "zemheri_navigation",
        "navigation_resilience_validation.launch.py",
        "use_sim_time:=true",
    ]
    return subprocess.Popen(command, start_new_session=True)


def run_case_analysis(case, output, waypoint_acceptance):
    """Generate case-specific CSV metrics, decisions and figures."""
    bag = output / "recording" / "bag"
    if not (bag / "metadata.yaml").exists():
        return {"status": "skipped", "reason": "bag metadata missing"}
    if case in {"guidance_los", "guidance_waypoint"}:
        command = [
            "ros2", "run", "zemheri_simulation",
            "analyze_guidance_validation.py", str(bag),
            "--output", str(output),
            "--waypoint-acceptance", str(waypoint_acceptance),
        ]
    elif case.startswith("rl_policy"):
        command = [
            "ros2", "run", "zemheri_simulation",
            "rl_policy_validation.py", str(bag),
            "--output", str(output),
        ]
    elif case == "navigation_resilience":
        command = [
            "ros2", "run", "zemheri_simulation",
            "analyze_navigation_resilience.py", str(bag),
            "--output", str(output),
        ]
    elif case in {
        "sensor_health", "ocean_current_response", "ocean_current_services",
    }:
        command = [
            "ros2", "run", "zemheri_simulation",
            "analyze_environment_validation.py", str(bag),
            "--output", str(output),
            "--case", case,
            "--current-x", str(0.4),
            "--current-y", str(0.2),
            "--current-z", str(0.0),
        ]
    else:
        command = [
            "ros2", "run", "zemheri_simulation",
            "analyze_report_bag.py", str(bag),
            "--output", str(output),
        ]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    (output / "analysis.log").write_text(
        result.stdout + result.stderr, encoding="utf-8"
    )
    return {
        "status": "completed" if result.returncode == 0 else "failed",
        "returncode": result.returncode,
        "command": " ".join(command),
    }


def stop_process(process):
    if process is None:
        return

    # start_new_session=True makes the launcher PID the process-group ID.
    # Signal the whole group so ros2 launch / ros2 bag child processes do not
    # survive after their parent CLI process exits.
    process_group = process.pid
    for stop_signal, timeout in [
        (signal.SIGINT, 8),
        (signal.SIGTERM, 4),
        (signal.SIGKILL, 2),
    ]:
        try:
            os.killpg(process_group, stop_signal)
        except ProcessLookupError:
            return

        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue

        # The CLI parent may exit before its child nodes. Check the process
        # group again on the next iteration and escalate only when necessary.


def flush_stop_command(node):
    if node is None:
        return
    node.stop()
    for _ in range(3):
        rclpy.spin_once(node, timeout_sec=0.05)


def request_clean_shutdown(_signum, _frame):
    raise KeyboardInterrupt


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--case",
        required=True,
        choices=[
            "navigation_straight",
            "navigation_resilience",
            "guidance_los",
            "guidance_waypoint",
            "controller_tracking",
            "rl_policy",
            "rl_policy_following_current",
            "rl_policy_cross_current",
            "rl_policy_diagonal_current",
            "rl_policy_reverse_current",
            "rl_policy_hard_cross_current",
            "sensor_health",
            "ocean_current_response",
            "ocean_current_services",
            "stage1_fsm",
            "stage2_bt",
        ],
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=45.0,
        help=(
            "Maximum case duration in ROS/simulation seconds. Mission cases "
            "also stop when the mission reports a terminal state."
        ),
    )
    parser.add_argument("--speed", type=float, default=0.8)
    parser.add_argument("--depth", type=float, default=5.0)
    parser.add_argument("--distance", type=float, default=30.0)
    parser.add_argument("--cross-track", type=float, default=5.0)
    parser.add_argument("--waypoint-acceptance", type=float, default=1.5)
    parser.add_argument("--current-x", type=float, default=0.0)
    parser.add_argument("--current-y", type=float, default=0.0)
    parser.add_argument("--current-z", type=float, default=0.0)
    parser.add_argument("--rl-dive-speed", type=float, default=1.2)
    parser.add_argument("--rl-depth-tolerance", type=float, default=0.15)
    parser.add_argument("--rl-settle-duration", type=float, default=3.0)
    parser.add_argument("--rl-settle-timeout", type=float, default=75.0)
    parser.add_argument(
        "--warmup",
        type=float,
        default=5.0,
        help="Stationary filter initialization before navigation/guidance tests.",
    )
    parser.add_argument(
        "--state-wait-timeout",
        type=float,
        default=30.0,
        help="Wall-time limit for receiving the first /auv/state message.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=(
            Path.home()
            / "zemheri_ws/src/zemheri_ros2/analysis/report_validation"
        ),
    )
    parser.add_argument(
        "--external-mission",
        action="store_true",
        help="Do not launch mission stack; use an already-running external mission.",
    )
    return parser.parse_args(argv)


def main(args=None):
    signal.signal(signal.SIGTERM, request_clean_shutdown)
    signal.signal(signal.SIGHUP, request_clean_shutdown)
    cli_args = parse_args(remove_ros_args(args=args)[1:])
    output = unique_output(cli_args.output_root.expanduser().resolve(), cli_args.case)
    mission = None
    recorder = None
    validation = None
    node = None
    manifest = {
        "schema_version": 1,
        "case": cli_args.case,
        "status": "starting",
        "started_at_utc": utc_now(),
        "completed_at_utc": None,
        "actual_duration_seconds": None,
        "runner_arguments": {
            "duration_seconds": cli_args.duration,
            "speed_mps": cli_args.speed,
            "depth_m": cli_args.depth,
            "distance_m": cli_args.distance,
            "cross_track_m": cli_args.cross_track,
            "waypoint_acceptance_m": cli_args.waypoint_acceptance,
            "current_target_mps": [
                cli_args.current_x, cli_args.current_y, cli_args.current_z,
            ],
            "warmup_seconds": cli_args.warmup,
            "rl_dive_speed_mps": cli_args.rl_dive_speed,
            "rl_depth_tolerance_m": cli_args.rl_depth_tolerance,
            "rl_settle_duration_seconds": cli_args.rl_settle_duration,
            "rl_settle_timeout_seconds": cli_args.rl_settle_timeout,
            "state_wait_timeout_seconds": cli_args.state_wait_timeout,
            "external_mission": cli_args.external_mission,
        },
        "validation_nodes": {},
        "recorded_topics": TOPICS,
        "recording_directory": "recording",
        "analysis": {"status": "pending"},
        "git": git_state(),
    }
    write_manifest(output, manifest)
    wall_started_at = time.monotonic()
    ocean_current_service_results = []
    try:
        if (
            cli_args.case in {"stage1_fsm", "stage2_bt"}
            and not cli_args.external_mission
        ):
            mission = start_mission(cli_args.case)
            time.sleep(2.0)
            if mission.poll() is not None:
                raise RuntimeError("Mission launch başlatılamadı.")

        validation = start_validation(cli_args.case)
        if validation is not None:
            time.sleep(2.0)
            if validation.poll() is not None:
                raise RuntimeError("Validation launch başlatılamadı.")

        recorder = start_recorder(output)
        time.sleep(1.0)
        if recorder.poll() is not None:
            raise RuntimeError("Merkezi sistem kaydı başlatılamadı.")

        rclpy.init(args=args)
        node = ReportTestRunner(cli_args)
        node.configure_ocean_current()
        manifest["validation_nodes"] = collect_remote_parameters(node)
        manifest["status"] = "running"
        write_manifest(output, manifest)
        node.get_logger().info(f"Case={cli_args.case} | output={output}")
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        manifest["status"] = "interrupted"
    except Exception:
        manifest["status"] = "failed"
        raise
    finally:
        if node is not None:
            ocean_current_service_results = list(node.ocean_current_service_results)
            flush_stop_command(node)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_process(recorder)
        stop_process(validation)
        stop_process(mission)
        startup_failed = (
            node is not None
            and node.stop_reason in {
                "state_wait_timeout",
                "sensor_stack_wait_timeout",
            }
        )
        if startup_failed:
            manifest["status"] = "failed"
            manifest["analysis"] = {
                "status": "skipped",
                "reason": node.stop_reason,
            }
        else:
            manifest["analysis"] = run_case_analysis(
                cli_args.case,
                output,
                cli_args.waypoint_acceptance,
            )
            write_ocean_current_service_results(
                output, ocean_current_service_results
            )
        if manifest["status"] == "running":
            manifest["status"] = (
                "duration_limit_reached"
                if node is not None
                and node.stop_reason == "duration_limit_reached"
                else "completed"
            )
        if node is not None:
            manifest["stop_reason"] = node.stop_reason
        manifest["completed_at_utc"] = utc_now()
        manifest["actual_duration_seconds"] = round(
            time.monotonic() - wall_started_at, 3
        )
        write_manifest(output, manifest)
        print(f"[COMPLETE] {output}")


if __name__ == "__main__":
    main()
