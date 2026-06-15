#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from zemheri_interfaces.msg import AuvState, GuidanceGoal
from zemheri_simulation_plugins.srv import (
    ClearCurrentSchedule,
    LoadBuiltInScenario,
    SetCurrentMode,
    SetCurrentPreset,
    SetCurrentTarget,
)


DEFAULT_TOPICS = [
    "/ground_truth/odometry",
    "/odometry/ukf",
    "/sara_uuv/cmd_vel",
    "/guidance/goal",
    "/guidance/setpoint",
    "/auv/state",
    "/imu/data",
    "/pressure/raw",
    "/pressure/depth_pose",
    "/dvl/twist",
    "/dvl/raw",
    "/ocean_current",
    "/ocean_current/status",
]


def resolve_output_path(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    if path.is_absolute():
        return path.resolve()

    env_root = os.environ.get("ZEMHERI_ROS2_ROOT")
    candidates = []
    if env_root:
        candidates.append(Path(env_root).expanduser())

    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        ws_root = Path(prefix).expanduser().resolve().parent
        candidates.append(ws_root / "src" / "zemheri_ros2")

    candidates.extend([
        Path.cwd(),
        Path.home() / "zemheri_ws" / "src" / "zemheri_ros2",
    ])

    for candidate in candidates:
        if (candidate / "zemheri_simulation").exists():
            return (candidate / path).resolve()

    return (Path.cwd() / path).resolve()


def unique_bag_path(path: Path) -> Path:
    if not path.exists():
        return path

    for index in range(1, 1000):
        candidate = path.with_name(f"{path.name}_{index:02d}")
        if not candidate.exists():
            return candidate

    raise RuntimeError(f"Could not create a unique bag path for: {path}")


@dataclass(frozen=True)
class MissionPhase:
    name: str
    duration_sec: float
    target_depth_m: float
    target_yaw_rad: float
    target_speed_mps: float
    guidance_mode: str
    note: str
    max_duration_sec: float = 0.0


def stage_1_profile(speed_mps: float, turn_yaw_rad: float, turn_speed_mps: float) -> List[MissionPhase]:
    return [
        MissionPhase(
            "pre_cruise_10m",
            10.0,
            2.0,
            0.0,
            speed_mps,
            "CRUISE",
            "10 m düz ilerleme, z hedefi yaklaşık -2 m.",
        ),
        MissionPhase(
            "outbound_50m",
            50.0,
            2.0,
            0.0,
            speed_mps,
            "CRUISE",
            "10 m çizgiden sonra 50 m uzaklaşma; başlangıç referansından toplam 60 m.",
        ),
        MissionPhase(
            "turn_180deg",
            15.0,
            2.0,
            turn_yaw_rad,
            turn_speed_mps,
            "TURN_YAW",
            "180 derece dönüş; fin etkinliği için düşük ileri hız korunur.",
            45.0,
        ),
        MissionPhase(
            "stabilize_after_turn",
            3.0,
            2.0,
            turn_yaw_rad,
            0.0,
            "STABILIZE",
            "Dönüşten sonra kısa süre yaw/depth oturtma.",
        ),
        MissionPhase(
            "return_50m",
            50.0,
            2.0,
            turn_yaw_rad,
            speed_mps,
            "CRUISE",
            "Başlangıç yönünün tersine 50 m dönüş.",
        ),
        MissionPhase(
            "finish_line_10m",
            2.0,
            2.0,
            turn_yaw_rad,
            0.0,
            "FINISH_HOLD",
            "Başlangıç/bitiş çizgisinde kısa güvenli bekleme.",
        ),
    ]


def stage_2_profile(
    speed_mps: float,
    surface_command_depth_m: float,
    fire_hold_speed_mps: float,
    post_fire_speed_mps: float,
) -> List[MissionPhase]:
    return [
        MissionPhase(
            "fire_zone_30m",
            30.0,
            2.0,
            0.0,
            speed_mps,
            "CRUISE",
            "30 m ilerleme, D2 seyir derinliği yaklaşık z=-2 m.",
        ),
        MissionPhase(
            "pitch_to_surface",
            10.0,
            surface_command_depth_m,
            0.0,
            max(0.3, min(speed_mps, 0.6)),
            "PITCH_UP_SURFACE",
            "Yunuslama ile yüzeye yaklaşma; burun kısmının çıkması yeterli kabul edilir.",
        ),
        MissionPhase(
            "rocket_fire",
            3.0,
            surface_command_depth_m,
            0.0,
            fire_hold_speed_mps,
            "FIRE_PERMISSION",
            "Roket ateşleme zamanı: t=43 s.",
        ),
        MissionPhase(
            "rocket_surface_exit",
            7.0,
            surface_command_depth_m,
            0.0,
            post_fire_speed_mps,
            "POST_FIRE_SAFE_MODE",
            "Roketin yüzeye çıkışı için güvenli bekleme.",
        ),
    ]


def cumulative_times(phases: List[MissionPhase]) -> List[float]:
    total = 0.0
    out = []
    for phase in phases:
        total += phase.duration_sec
        out.append(total)
    return out


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class CompetitionMissionRunner(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("competition_mission_runner")
        self.args = args
        turn_yaw = -math.pi if args.turn_direction == "right" else math.pi
        self.phases = (
            stage_1_profile(args.speed, turn_yaw, args.turn_speed)
            if args.stage == 1
            else stage_2_profile(
                args.speed,
                args.stage2_surface_command_depth,
                args.stage2_fire_hold_speed,
                args.stage2_post_fire_speed,
            )
        )
        self.total_duration = sum(p.duration_sec for p in self.phases)
        self.phase_ends = cumulative_times(self.phases)
        self.phase_starts = [0.0] + self.phase_ends[:-1]
        self.stage_limit_sec = 180.0 if args.stage == 1 else 60.0
        self.current_phase_index = 0
        self.start_time = None
        self.phase_start_time = None
        self.phase_logged = False
        self.done = False
        self.has_state = False
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.along_track_m = 0.0
        self.cross_track_m = 0.0
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_z = 0.0
        self.stage2_nose15_z = float("-inf")
        self.stage2_max_nose15_z = float("-inf")
        self.current_speed_mps = 0.0
        self.max_observed_speed_mps = 0.0
        self.max_abs_roll_deg = 0.0
        self.bag_process: Optional[subprocess.Popen] = None
        self.bag_path: Optional[Path] = None
        self.current_config_result = {
            "enabled": False,
            "mode": "",
            "scenario": "",
            "preset": "",
            "vector_xyz_mps": None,
            "success": True,
            "message": "No ocean current requested.",
        }

        self.goal_pub = self.create_publisher(GuidanceGoal, args.goal_topic, 10)
        self.create_subscription(AuvState, args.state_topic, self.state_cb, 20)
        self.current_mode_cli = self.create_client(SetCurrentMode, "/ocean_current/set_mode")
        self.current_target_cli = self.create_client(SetCurrentTarget, "/ocean_current/set_target")
        self.current_preset_cli = self.create_client(SetCurrentPreset, "/ocean_current/set_preset")
        self.current_scenario_cli = self.create_client(
            LoadBuiltInScenario,
            "/ocean_current/load_built_in_scenario",
        )
        self.current_clear_cli = self.create_client(ClearCurrentSchedule, "/ocean_current/clear_schedule")

        self.configure_ocean_current()
        if (
            self.current_config_result["enabled"]
            and not self.current_config_result["success"]
            and args.require_current
        ):
            raise RuntimeError(
                "Ocean current was requested but could not be configured. "
                "Use --no-require-current to continue without failing."
            )

        if args.record_bag:
            self.start_bag_recording()

        self.timer = self.create_timer(1.0 / args.publish_rate, self.tick)
        self.write_metadata(started=True)
        self.get_logger().info(
            f"Competition mission runner started | stage={args.stage}, "
            f"duration={self.total_duration:.1f}s, speed={args.speed:.2f}m/s"
        )

    def call_service_blocking(self, client, request, label: str) -> bool:
        timeout = self.args.current_service_timeout
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"[CURRENT] {label} service unavailable after {timeout:.1f}s")
            return False

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().warn(f"[CURRENT] {label} service call timed out after {timeout:.1f}s")
            return False

        response = future.result()
        if response is None:
            self.get_logger().warn(f"[CURRENT] {label} service returned no response")
            return False

        if not getattr(response, "success", False):
            self.get_logger().warn(f"[CURRENT] {label} failed: {getattr(response, 'message', '')}")
            return False

        self.get_logger().info(f"[CURRENT] {label}: {getattr(response, 'message', 'ok')}")
        return True

    def configure_ocean_current(self) -> None:
        if self.args.current_scenario:
            req = LoadBuiltInScenario.Request()
            req.scenario_name = self.args.current_scenario
            req.overwrite = True
            ok = self.call_service_blocking(self.current_scenario_cli, req, "load scenario")
            self.current_config_result.update({
                "enabled": True,
                "scenario": self.args.current_scenario,
                "success": ok,
                "message": "Built-in scenario requested.",
            })
            return

        if self.args.current_preset:
            req = SetCurrentPreset.Request()
            req.preset_name = self.args.current_preset
            ok = self.call_service_blocking(self.current_preset_cli, req, "set preset")
            self.current_config_result.update({
                "enabled": True,
                "preset": self.args.current_preset,
                "success": ok,
                "message": "Preset requested.",
            })
            return

        if self.args.current_vector is None:
            return

        mode = self.args.current_mode or "constant"
        mode_req = SetCurrentMode.Request()
        mode_req.mode = mode
        mode_ok = self.call_service_blocking(self.current_mode_cli, mode_req, "set mode")

        x, y, z = self.args.current_vector
        target_req = SetCurrentTarget.Request()
        target_req.x = float(x)
        target_req.y = float(y)
        target_req.z = float(z)
        target_ok = self.call_service_blocking(self.current_target_cli, target_req, "set target")

        self.current_config_result.update({
            "enabled": True,
            "mode": mode,
            "vector_xyz_mps": [float(x), float(y), float(z)],
            "success": mode_ok and target_ok,
            "message": "Manual current vector requested.",
        })

    def clear_ocean_current(self) -> None:
        if not self.args.clear_current_on_finish:
            return

        clear_req = ClearCurrentSchedule.Request()
        clear_req.clear_all = True
        self.call_service_blocking(self.current_clear_cli, clear_req, "clear schedule")

        target_req = SetCurrentTarget.Request()
        target_req.x = 0.0
        target_req.y = 0.0
        target_req.z = 0.0
        self.call_service_blocking(self.current_target_cli, target_req, "zero target")

        mode_req = SetCurrentMode.Request()
        mode_req.mode = "constant"
        self.call_service_blocking(self.current_mode_cli, mode_req, "set constant mode")

    def now_sec(self) -> float:
        now = self.get_clock().now()
        return now.nanoseconds * 1e-9

    def elapsed_sec(self) -> float:
        now = self.now_sec()
        if self.start_time is None:
            self.start_time = now
        if self.phase_start_time is None:
            self.phase_start_time = now
        return now - self.start_time

    def phase_elapsed_sec(self) -> float:
        if self.phase_start_time is None:
            return 0.0
        return self.now_sec() - self.phase_start_time

    def state_cb(self, msg: AuvState) -> None:
        self.has_state = True
        if self.start_x is None:
            self.start_x = float(msg.x)
            self.start_y = float(msg.y)
            self.start_yaw = float(msg.yaw)
            self.get_logger().info(
                f"[REFERENCE] start=({self.start_x:.2f}, {self.start_y:.2f}), "
                f"yaw={math.degrees(self.start_yaw):.1f}deg"
            )

        self.current_yaw = float(msg.yaw)
        self.current_roll = float(msg.roll)
        self.current_pitch = float(msg.pitch)
        self.current_z = float(msg.z)
        self.stage2_nose15_z = self.current_z - (
            self.args.vehicle_body_length - self.args.stage2_nose_emergence_length
        ) * math.sin(self.current_pitch)
        self.stage2_max_nose15_z = max(self.stage2_max_nose15_z, self.stage2_nose15_z)
        self.current_speed_mps = math.sqrt(float(msg.vx) ** 2 + float(msg.vy) ** 2 + float(msg.vz) ** 2)
        self.max_observed_speed_mps = max(self.max_observed_speed_mps, self.current_speed_mps)
        self.max_abs_roll_deg = max(self.max_abs_roll_deg, abs(math.degrees(self.current_roll)))

        if self.start_x is not None and self.start_y is not None and self.start_yaw is not None:
            dx = float(msg.x) - self.start_x
            dy = float(msg.y) - self.start_y
            c = math.cos(self.start_yaw)
            s = math.sin(self.start_yaw)
            self.along_track_m = dx * c + dy * s
            self.cross_track_m = -dx * s + dy * c

    def yaw_error_deg(self, target_yaw: float) -> float:
        return abs(math.degrees(normalize_angle(target_yaw - self.current_yaw)))

    def reached_stage1_pre_cruise_line(self) -> bool:
        return self.has_state and self.along_track_m >= self.args.pre_cruise_distance - self.args.distance_tolerance

    def reached_stage1_turn_line(self) -> bool:
        return self.has_state and self.along_track_m >= self.args.outbound_distance - self.args.distance_tolerance

    def reached_stage1_finish_line(self) -> bool:
        return self.has_state and self.along_track_m <= self.args.finish_line_distance + self.args.distance_tolerance

    def planned_remaining_sec(self, phase_elapsed: float) -> float:
        if self.current_phase_index >= len(self.phases):
            return 0.0

        current = self.phases[self.current_phase_index]
        remaining = max(0.0, current.duration_sec - phase_elapsed)
        for phase in self.phases[self.current_phase_index + 1:]:
            remaining += phase.duration_sec
        return remaining

    def adaptive_speed(self, phase: MissionPhase, elapsed: float, phase_elapsed: float) -> float:
        speed = float(phase.target_speed_mps)

        if not self.args.adaptive_speed:
            return min(speed, self.args.dvl_speed_limit)

        if phase.name == "turn_180deg":
            yaw_error = self.yaw_error_deg(phase.target_yaw_rad) if self.has_state else 180.0
            overrun = max(0.0, phase_elapsed - phase.duration_sec)
            if yaw_error > self.args.yaw_tolerance_deg and overrun > 0.0:
                speed += self.args.turn_speed_ramp * overrun
            speed = min(speed, self.args.max_turn_speed)

            if (
                yaw_error < self.args.cross_track_control_yaw_threshold
                and abs(self.cross_track_m) > self.args.max_turn_cross_track
            ):
                excess = abs(self.cross_track_m) - self.args.max_turn_cross_track
                speed -= self.args.cross_track_speed_penalty * excess

            return min(max(self.args.min_turn_speed, speed), self.args.dvl_speed_limit)

        if speed <= 0.0 or phase.guidance_mode not in ["CRUISE", "PITCH_UP_SURFACE"]:
            return speed

        remaining_budget = max(0.0, self.stage_limit_sec - elapsed)
        planned_remaining = self.planned_remaining_sec(phase_elapsed)
        slack = remaining_budget - planned_remaining

        if slack >= self.args.speedup_slack_threshold:
            return speed

        pressure = (self.args.speedup_slack_threshold - slack) / max(self.args.speedup_slack_threshold, 1e-6)
        pressure = max(0.0, min(1.0, pressure))
        boosted_speed = speed + pressure * (self.args.max_cruise_speed - speed)

        return min(max(speed, min(boosted_speed, self.args.max_cruise_speed)), self.args.dvl_speed_limit)

    def should_advance_phase(self, phase: MissionPhase, phase_elapsed: float) -> bool:
        if self.args.stage == 1:
            if phase.name == "pre_cruise_10m":
                return self.reached_stage1_pre_cruise_line()

            if phase.name == "outbound_50m":
                return self.reached_stage1_turn_line()

            if phase.name == "return_50m":
                return self.reached_stage1_finish_line()

            if phase.name == "finish_line_10m":
                return phase_elapsed >= phase.duration_sec

        if phase.name == "turn_180deg":
            yaw_ok = self.has_state and self.yaw_error_deg(phase.target_yaw_rad) <= self.args.yaw_tolerance_deg
            if yaw_ok and phase_elapsed >= self.args.min_turn_time:
                return True

            if self.reached_stage1_finish_line():
                self.get_logger().warn(
                    "[PHASE] Araç dönüş sırasında başlangıç/bitiş çizgisine kadar yaklaştı; "
                    "geri dönüş mesafesi tamamlanmış kabul ediliyor.",
                    throttle_duration_sec=5.0,
                )
                return True

            max_duration = phase.max_duration_sec or phase.duration_sec
            if phase_elapsed >= max_duration:
                self.get_logger().warn(
                    f"[PHASE] turn_180deg max süreye ulaştı; "
                    f"yaw_error={self.yaw_error_deg(phase.target_yaw_rad):.1f}deg. "
                    "Dönüş tamamlanmadan dönüş fazından çıkılmayacak.",
                    throttle_duration_sec=5.0,
                )
                return False

            return False

        if phase.name == "pitch_to_surface":
            nose_emerged = self.has_state and self.stage2_nose15_z >= 0.0
            if nose_emerged and phase_elapsed >= self.args.stage2_min_pitch_time:
                self.get_logger().info(
                    f"[SURFACE] Nose emergence criterion reached: "
                    f"nose15_z={self.stage2_nose15_z:.3f}m, "
                    f"pitch={math.degrees(self.current_pitch):.1f}deg"
                )
                return True

            if phase_elapsed >= self.args.stage2_max_pitch_time:
                self.get_logger().warn(
                    f"[SURFACE] Nose emergence criterion not reached before timeout: "
                    f"nose15_z={self.stage2_nose15_z:.3f}m"
                )
                return True

            return False

        return phase_elapsed >= phase.duration_sec

    def advance_phase(self) -> None:
        self.current_phase_index += 1
        self.phase_start_time = self.now_sec()
        self.phase_logged = False

        if self.current_phase_index >= len(self.phases):
            self.publish_stop()
            self.finish()

    def tick(self) -> None:
        if self.done:
            return

        self.check_bag_process()

        elapsed = self.elapsed_sec()
        if elapsed >= self.stage_limit_sec:
            self.get_logger().warn(
                f"[TIMEOUT] Stage-{self.args.stage} süre limiti doldu "
                f"({self.stage_limit_sec:.0f}s). Güvenli stop."
            )
            self.publish_stop()
            self.finish()
            return

        if self.current_phase_index >= len(self.phases):
            self.publish_stop()
            self.finish()
            return

        phase = self.phases[self.current_phase_index]
        phase_elapsed = self.phase_elapsed_sec()
        cmd_speed = self.adaptive_speed(phase, elapsed, phase_elapsed)

        if not self.phase_logged:
            self.phase_logged = True
            self.get_logger().info(
                f"[PHASE] {phase.name} | t={elapsed:.1f}s, "
                f"depth={phase.target_depth_m:.2f}m, "
                f"yaw={math.degrees(phase.target_yaw_rad):.1f}deg, "
                f"forward_speed={cmd_speed:.2f}m/s"
            )

        if phase.name == "turn_180deg" and self.has_state:
            self.get_logger().info(
                f"[TURN] elapsed={phase_elapsed:.1f}s, "
                f"yaw_error={self.yaw_error_deg(phase.target_yaw_rad):.1f}deg, "
                f"forward_speed={cmd_speed:.2f}m/s, "
                f"along={self.along_track_m:.1f}m, cross={self.cross_track_m:.1f}m",
                throttle_duration_sec=2.0,
            )

        msg = GuidanceGoal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mission"
        msg.target_x = 0.0
        msg.target_y = 0.0
        msg.target_depth = float(phase.target_depth_m)
        msg.target_yaw = float(phase.target_yaw_rad)
        msg.target_speed = float(cmd_speed)
        msg.guidance_mode = phase.guidance_mode
        self.goal_pub.publish(msg)

        if self.should_advance_phase(phase, phase_elapsed):
            self.advance_phase()

    def publish_stop(self) -> None:
        msg = GuidanceGoal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mission"
        msg.target_depth = 0.0 if self.args.stage == 2 else 2.0
        msg.target_yaw = (-math.pi if self.args.turn_direction == "right" else math.pi) if self.args.stage == 1 else 0.0
        msg.target_speed = 0.0
        msg.guidance_mode = "STOP"
        for _ in range(10):
            self.goal_pub.publish(msg)

    def start_bag_recording(self) -> None:
        bag_root = resolve_output_path(self.args.bag_root)
        bag_root.mkdir(parents=True, exist_ok=True)

        if self.args.bag_name:
            bag_name = self.args.bag_name
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f"stage{self.args.stage}_mission_{stamp}"

        self.bag_path = unique_bag_path(bag_root / bag_name)
        cmd = ["ros2", "bag", "record", "-o", str(self.bag_path), *self.args.topics]
        self.get_logger().info(f"[BAG] Recording: {self.bag_path}")
        self.bag_process = subprocess.Popen(cmd)
        try:
            self.bag_process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            return

        if self.bag_process.returncode != 0:
            self.get_logger().error("[BAG] ros2 bag record failed; analysis will be skipped.")
            self.bag_process = None
            self.bag_path = None

    def check_bag_process(self) -> None:
        if self.bag_process is None:
            return

        return_code = self.bag_process.poll()
        if return_code is None:
            return

        if return_code != 0:
            self.get_logger().error(
                f"[BAG] ros2 bag record exited early with code {return_code}; "
                "analysis will be skipped."
            )
            self.bag_path = None
        self.bag_process = None

    def stop_bag_recording(self) -> None:
        if self.bag_process is None:
            return

        if self.bag_process.poll() is None:
            self.get_logger().info("[BAG] Stopping rosbag recorder...")
            self.bag_process.send_signal(signal.SIGINT)
            try:
                self.bag_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.bag_process.terminate()
                self.bag_process.wait(timeout=5.0)

    def write_metadata(self, started: bool) -> None:
        if self.args.metadata_dir:
            metadata_dir = resolve_output_path(self.args.metadata_dir)
        elif self.bag_path is not None:
            metadata_dir = self.bag_path
        else:
            metadata_dir = resolve_output_path(self.args.bag_root)

        metadata_dir.mkdir(parents=True, exist_ok=True)
        metadata = {
            "stage": self.args.stage,
            "speed_estimate_mps": self.args.speed,
            "stage_limit_sec": 180.0 if self.args.stage == 1 else 60.0,
            "planned_duration_sec": self.total_duration,
            "planned_event_times_sec": self.phase_ends,
            "adaptive_speed": self.args.adaptive_speed,
            "max_cruise_speed_mps": self.args.max_cruise_speed,
            "max_turn_speed_mps": self.args.max_turn_speed,
            "dvl_speed_limit_mps": self.args.dvl_speed_limit,
            "max_observed_speed_mps": self.max_observed_speed_mps,
            "dvl_speed_limit_exceeded": self.max_observed_speed_mps > self.args.dvl_speed_limit,
            "min_turn_speed_mps": self.args.min_turn_speed,
            "turn_speed_ramp_mps_per_sec": self.args.turn_speed_ramp,
            "max_turn_cross_track_m": self.args.max_turn_cross_track,
            "cross_track_speed_penalty": self.args.cross_track_speed_penalty,
            "speedup_slack_threshold_sec": self.args.speedup_slack_threshold,
            "yaw_tolerance_deg": self.args.yaw_tolerance_deg,
            "distance_tolerance_m": self.args.distance_tolerance,
            "start_reference": {
                "x": self.start_x,
                "y": self.start_y,
                "yaw_rad": self.start_yaw,
            },
            "max_abs_roll_deg": self.max_abs_roll_deg,
            "depth_sign_convention": "ROS/Gazebo z is negative below surface; GuidanceGoal target_depth is positive depth in meters.",
            "depth_marks_m": {
                "cruise_z": -2.0,
                "surface_z": 0.0,
                "stage2_d2_z": -2.0,
            },
            "stage2_surface_note": "Pitch-up phase accepts partial nose emergence; full vehicle surfacing is not required.",
            "stage2_surface_command_depth_m": self.args.stage2_surface_command_depth,
            "stage2_nose_emergence_length_m": self.args.stage2_nose_emergence_length,
            "stage2_max_nose15_z_m": (
                self.stage2_max_nose15_z if math.isfinite(self.stage2_max_nose15_z) else None
            ),
            "stage2_fire_hold_speed_mps": self.args.stage2_fire_hold_speed,
            "stage2_post_fire_speed_mps": self.args.stage2_post_fire_speed,
            "ocean_current": self.current_config_result,
            "phases": [asdict(p) for p in self.phases],
            "bag_path": str(self.bag_path) if self.bag_path is not None else None,
            "started": started,
            "completed": self.done,
        }

        with open(metadata_dir / "competition_mission_profile.json", "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)

    def run_analysis(self) -> None:
        if not self.args.analyze or self.bag_path is None:
            return

        if not (self.bag_path / "metadata.yaml").exists() or not list(self.bag_path.glob("*.db3")):
            self.get_logger().warn(f"[ANALYSIS] Bag eksik/boş görünüyor, analiz atlandı: {self.bag_path}")
            return

        script = Path(__file__).with_name("analyze_ekf_filters.py")
        cmd = [
            sys.executable,
            str(script),
            "--bag",
            str(self.bag_path),
            "--out-dir",
            str(resolve_output_path(self.args.analysis_out)),
            "--export-csv",
        ]
        self.get_logger().info("[ANALYSIS] Running UKF/GT analysis...")
        subprocess.run(cmd, check=False)

    def finish(self) -> None:
        if self.done:
            return
        self.done = True
        if self.max_observed_speed_mps > self.args.dvl_speed_limit:
            self.get_logger().warn(
                f"[DVL] max observed speed {self.max_observed_speed_mps:.2f}m/s "
                f"exceeded configured DVL limit {self.args.dvl_speed_limit:.2f}m/s"
            )
        else:
            self.get_logger().info(
                f"[DVL] max observed speed {self.max_observed_speed_mps:.2f}m/s "
                f"within configured DVL limit {self.args.dvl_speed_limit:.2f}m/s"
            )
        self.stop_bag_recording()
        self.clear_ocean_current()
        self.write_metadata(started=True)
        self.run_analysis()
        self.get_logger().info("[COMPLETE] Mission runner finished.")


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Zemheri competition mission bag runner")
    parser.add_argument("--stage", type=int, choices=[1, 2], required=True)
    parser.add_argument("--speed", type=float, default=1.0, help="V1/V2 estimated cruise speed [m/s]")
    parser.add_argument(
        "--stage2-surface-command-depth",
        type=float,
        default=0.15,
        help="Stage-2 pitch-up/fire fazlarında burun çıkışı için kullanılan kontrol derinliği [m]",
    )
    parser.add_argument(
        "--stage2-fire-hold-speed",
        type=float,
        default=0.25,
        help="Roket ateşleme fazında pitch tutmak için korunan düşük ileri hız [m/s]",
    )
    parser.add_argument(
        "--stage2-post-fire-speed",
        type=float,
        default=0.15,
        help="Ateşleme sonrası pitch/depth stabilitesi için korunan düşük ileri hız [m/s]",
    )
    parser.add_argument("--vehicle-body-length", type=float, default=0.983)
    parser.add_argument("--stage2-nose-emergence-length", type=float, default=0.15)
    parser.add_argument("--stage2-min-pitch-time", type=float, default=3.0)
    parser.add_argument("--stage2-max-pitch-time", type=float, default=12.0)
    parser.add_argument("--publish-rate", type=float, default=10.0)
    parser.add_argument("--goal-topic", default="/guidance/goal")
    parser.add_argument("--state-topic", default="/auv/state")
    parser.add_argument("--yaw-tolerance-deg", type=float, default=8.0)
    parser.add_argument("--distance-tolerance", type=float, default=2.0)
    parser.add_argument("--pre-cruise-distance", type=float, default=10.0)
    parser.add_argument("--outbound-distance", type=float, default=60.0)
    parser.add_argument("--finish-line-distance", type=float, default=10.0)
    parser.add_argument("--min-turn-time", type=float, default=5.0)
    parser.add_argument("--turn-direction", choices=["left", "right"], default="left")
    parser.add_argument("--turn-speed", type=float, default=0.45, help="Stage-1 180 derece dönüşte kullanılan düşük ileri hız [m/s]")
    parser.add_argument("--min-turn-speed", type=float, default=0.40)
    parser.add_argument("--adaptive-speed", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-cruise-speed", type=float, default=1.0)
    parser.add_argument("--max-turn-speed", type=float, default=0.65)
    parser.add_argument(
        "--dvl-speed-limit",
        type=float,
        default=2.57,
        help="Tracker 650 DVL 5 kts ölçüm sınırına göre komutlanan ileri hız üst limiti [m/s]",
    )
    parser.add_argument("--turn-speed-ramp", type=float, default=0.012)
    parser.add_argument("--max-turn-cross-track", type=float, default=18.0)
    parser.add_argument("--cross-track-speed-penalty", type=float, default=0.02)
    parser.add_argument("--cross-track-control-yaw-threshold", type=float, default=80.0)
    parser.add_argument("--speedup-slack-threshold", type=float, default=40.0)
    parser.add_argument(
        "--current-scenario",
        default="",
        help="Görev başında /ocean_current/load_built_in_scenario ile yüklenecek hazır senaryo",
    )
    parser.add_argument(
        "--current-preset",
        default="",
        help="Görev başında /ocean_current/set_preset ile uygulanacak tek akıntı preseti",
    )
    parser.add_argument(
        "--current-vector",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="World/odom frame manuel akıntı vektörü [m/s]; +X görev ileri, +Y sol, +Z yukarı",
    )
    parser.add_argument(
        "--current-mode",
        choices=["constant", "ou", "gust"],
        default="constant",
        help="--current-vector kullanıldığında uygulanacak akıntı modu",
    )
    parser.add_argument("--current-service-timeout", type=float, default=5.0)
    parser.add_argument("--require-current", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clear-current-on-finish", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--record-bag", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bag-root", default="analysis/zemheri_bags")
    parser.add_argument("--bag-name", default="")
    parser.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS)
    parser.add_argument("--metadata-dir", default="")
    parser.add_argument("--analyze", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--analysis-out", default="analysis/ukf_analiz")
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main(argv: Optional[List[str]] = None) -> None:
    argv = sys.argv if argv is None else argv
    args = parse_args(argv)

    rclpy.init(args=argv)
    node = CompetitionMissionRunner(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok() and not node.done:
            node.publish_stop()
            node.stop_bag_recording()
            node.write_metadata(started=True)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
