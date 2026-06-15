from __future__ import annotations

import importlib.util
import math
import random
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


def ensure_packages():
    required = [
        ("numpy", "numpy"),
        ("matplotlib", "matplotlib"),
        ("PIL", "pillow"),
        ("imageio", "imageio"),
        ("imageio_ffmpeg", "imageio-ffmpeg"),
    ]
    missing = [package for module, package in required if importlib.util.find_spec(module) is None]
    if missing:
        subprocess.check_call([sys.executable, "-m", "pip", "install", *missing])


ensure_packages()

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

try:
    import imageio_ffmpeg
    FFMPEG_PATH = imageio_ffmpeg.get_ffmpeg_exe()
except Exception:
    FFMPEG_PATH = ""


DT = 0.10
EPISODES = 16
MAX_STEPS = 1000
SEED = 42


def clip(value, low, high):
    return max(low, min(high, value))


def wrap_angle(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def body_to_ned_matrix(yaw, pitch):
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    return np.array(
        [
            [cy * cp, -sy, -cy * sp],
            [sy * cp, cy, -sy * sp],
            [sp, 0.0, cp],
        ],
        dtype=float,
    )


def rmse(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values * values))) if len(values) else 0.0


@dataclass(frozen=True)
class VehicleConfig:
    mass_kg: float = 15.8454
    length_m: float = 0.982
    tube_diameter_m: float = 0.200
    max_diameter_m: float = 0.26728
    cg_from_nozzle_m: float = 0.58379
    dvl_x_m: float = 0.05341
    dvl_y_m: float = 0.030
    dvl_z_m: float = 0.0
    target_speed_mps: float = 0.70
    target_depth_m: float = 2.0
    target_forward_m: float = 50.0
    battery_wh: float = 199.8
    nominal_voltage_v: float = 22.2
    nominal_motor_w: float = 1000.0
    throttle_max: float = 0.80
    fin_limit_deg: float = 12.0
    physical_fin_limit_deg: float = 15.0
    max_depth_m: float = 3.5
    max_pitch_deg: float = 18.0
    dvl_noise_mps: float = 0.02
    pressure_noise_m: float = 0.015
    imu_angle_noise_rad: float = math.radians(0.35)
    imu_rate_noise_radps: float = math.radians(0.08)


@dataclass
class TruthState:
    x: float = 0.0
    y: float = 0.0
    z: float = 2.0
    u: float = 0.0
    v: float = 0.0
    w: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    yaw_rate: float = 0.0
    pitch_rate: float = 0.0


@dataclass
class Command:
    throttle: float = 0.0
    pitch_fin: float = 0.0
    yaw_fin: float = 0.0


@dataclass(frozen=True)
class Action:
    speed_trim: float = 0.0
    depth_trim: float = 0.0
    heading_trim: float = 0.0


@dataclass
class StepResult:
    observation: np.ndarray
    reward: float
    done: bool
    truncated: bool
    info: dict


class PID:
    def __init__(self, kp, ki, kd, limit, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_error = 0.0
        self.first = True

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.first = True

    def step(self, error, dt):
        self.integral = clip(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        derivative = 0.0 if self.first else (error - self.previous_error) / dt
        self.first = False
        self.previous_error = error
        return clip(self.kp * error + self.ki * self.integral + self.kd * derivative, -self.limit, self.limit)


class CascadePID:
    def __init__(self, cfg):
        self.cfg = cfg
        fin_limit = math.radians(cfg.fin_limit_deg)
        self.speed = PID(0.82, 0.12, 0.06, 0.30, 2.0)
        self.depth = PID(0.28, 0.035, 0.08, math.radians(10.0), 1.2)
        self.pitch = PID(2.35, 0.10, 0.18, fin_limit, 0.6)
        self.heading = PID(1.95, 0.035, 0.16, fin_limit, 0.7)

    def reset(self):
        self.speed.reset()
        self.depth.reset()
        self.pitch.reset()
        self.heading.reset()

    def update(self, estimate, action, dt):
        speed_target = clip(self.cfg.target_speed_mps + action.speed_trim, 0.52, 0.84)
        depth_target = clip(self.cfg.target_depth_m + action.depth_trim, 1.65, 2.35)
        heading_bias = clip(action.heading_trim, -0.14, 0.14)
        speed_error = speed_target - estimate[3]
        base_throttle = 0.80 * speed_target / 1.18
        throttle = clip(base_throttle + self.speed.step(speed_error, dt), 0.0, self.cfg.throttle_max)
        depth_error = depth_target - estimate[2]
        pitch_ref = self.depth.step(depth_error, dt)
        pitch_fin = self.pitch.step(pitch_ref - estimate[7], dt)
        heading_ref = clip(heading_bias - 0.18 * estimate[1], -0.30, 0.30)
        heading_error = wrap_angle(heading_ref - estimate[6])
        yaw_fin = self.heading.step(heading_error, dt)
        return Command(throttle, pitch_fin, yaw_fin)


class UKF:
    def __init__(self):
        self.n = 8
        self.alpha = 0.35
        self.beta = 2.0
        self.kappa = 0.0
        self.lam = self.alpha * self.alpha * (self.n + self.kappa) - self.n
        self.scale = self.n + self.lam
        self.wm = np.full(2 * self.n + 1, 1.0 / (2.0 * self.scale))
        self.wc = np.full(2 * self.n + 1, 1.0 / (2.0 * self.scale))
        self.wm[0] = self.lam / self.scale
        self.wc[0] = self.wm[0] + 1.0 - self.alpha * self.alpha + self.beta
        self.x = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        self.p = np.diag([0.08, 0.08, 0.03, 0.05, 0.05, 0.05, 0.02, 0.02])
        self.q = np.diag([0.002, 0.002, 0.001, 0.012, 0.012, 0.012, 0.0008, 0.0008])

    def reset(self):
        self.__init__()

    def residual_state(self, a, b):
        d = np.array(a - b, dtype=float)
        d[6] = wrap_angle(d[6])
        return d

    def sigma_points(self):
        p = 0.5 * (self.p + self.p.T)
        jitter = 1e-9
        for _ in range(7):
            try:
                root = np.linalg.cholesky(self.scale * (p + np.eye(self.n) * jitter))
                break
            except np.linalg.LinAlgError:
                jitter *= 10.0
        else:
            eigval, eigvec = np.linalg.eigh(p)
            eigval = np.maximum(eigval, 1e-9)
            root = eigvec @ np.diag(np.sqrt(self.scale * eigval))
        pts = [self.x]
        for i in range(self.n):
            pts.append(self.x + root[:, i])
            pts.append(self.x - root[:, i])
        return np.array(pts)

    def mean_state(self, points):
        mean = np.sum(points * self.wm[:, None], axis=0)
        mean[6] = math.atan2(np.sum(self.wm * np.sin(points[:, 6])), np.sum(self.wm * np.cos(points[:, 6])))
        return mean

    def predict(self, dt, yaw_rate, pitch_rate):
        points = self.sigma_points()
        predicted = np.array([self.process_model(p, dt, yaw_rate, pitch_rate) for p in points])
        mean = self.mean_state(predicted)
        cov = np.array(self.q, dtype=float)
        for i in range(len(predicted)):
            d = self.residual_state(predicted[i], mean)
            cov += self.wc[i] * np.outer(d, d)
        self.x = mean
        self.x[6] = wrap_angle(self.x[6])
        self.p = 0.5 * (cov + cov.T)

    def process_model(self, state, dt, yaw_rate, pitch_rate):
        s = np.array(state, dtype=float)
        r = body_to_ned_matrix(s[6], s[7])
        ned_velocity = r @ s[3:6]
        s[0:3] += ned_velocity * dt
        s[6] = wrap_angle(s[6] + yaw_rate * dt)
        s[7] = clip(s[7] + pitch_rate * dt, math.radians(-25.0), math.radians(25.0))
        return s

    def correct(self, measurement, measurement_model, noise, angle_indices=()):
        z = np.asarray(measurement, dtype=float)
        r = np.asarray(noise, dtype=float)
        points = self.sigma_points()
        z_points = np.array([measurement_model(p) for p in points])
        z_mean = np.sum(z_points * self.wm[:, None], axis=0)
        for idx in angle_indices:
            z_mean[idx] = math.atan2(np.sum(self.wm * np.sin(z_points[:, idx])), np.sum(self.wm * np.cos(z_points[:, idx])))
        s_cov = np.array(r, dtype=float)
        pxz = np.zeros((self.n, len(z)))
        for i in range(len(points)):
            dx = self.residual_state(points[i], self.x)
            dz = np.array(z_points[i] - z_mean, dtype=float)
            for idx in angle_indices:
                dz[idx] = wrap_angle(dz[idx])
            s_cov += self.wc[i] * np.outer(dz, dz)
            pxz += self.wc[i] * np.outer(dx, dz)
        innovation = np.array(z - z_mean, dtype=float)
        for idx in angle_indices:
            innovation[idx] = wrap_angle(innovation[idx])
        gain = np.linalg.solve(s_cov.T, pxz.T).T
        self.x = self.x + gain @ innovation
        self.x[6] = wrap_angle(self.x[6])
        self.p = self.p - gain @ s_cov @ gain.T
        self.p = 0.5 * (self.p + self.p.T) + np.eye(self.n) * 1e-10


class CurrentProfile:
    def __init__(self, seed, duration_s):
        rng = random.Random(seed)
        self.segments = []
        t = 0.0
        while t < duration_s + 20.0:
            mode = rng.choices(["calm", "normal", "hard"], weights=[0.34, 0.46, 0.20], k=1)[0]
            length = rng.uniform(10.0, 24.0)
            if mode == "calm":
                north = rng.uniform(-0.03, 0.05)
                east = rng.uniform(-0.03, 0.03)
                down = rng.uniform(-0.006, 0.006)
            elif mode == "normal":
                north = rng.uniform(-0.08, 0.10)
                east = rng.uniform(-0.08, 0.08)
                down = rng.uniform(-0.012, 0.012)
            else:
                north = rng.uniform(-0.16, 0.18)
                east = rng.uniform(-0.16, 0.16)
                down = rng.uniform(-0.025, 0.025)
            phase = rng.uniform(0.0, 2.0 * math.pi)
            self.segments.append((t, t + length, north, east, down, phase, mode))
            t += length

    def at(self, t):
        for start, end, north, east, down, phase, mode in self.segments:
            if start <= t < end:
                g = math.sin(0.42 * t + phase)
                return np.array([north + 0.018 * g, east + 0.015 * g, down + 0.004 * g], dtype=float), mode
        start, end, north, east, down, phase, mode = self.segments[-1]
        g = math.sin(0.42 * t + phase)
        return np.array([north + 0.018 * g, east + 0.015 * g, down + 0.004 * g], dtype=float), mode


class Vehicle:
    def __init__(self, cfg):
        self.cfg = cfg
        self.state = TruthState(z=cfg.target_depth_m)
        self.energy_wh = 0.0

    def reset(self):
        self.state = TruthState(z=self.cfg.target_depth_m)
        self.energy_wh = 0.0

    def step(self, command, current_ned, dt):
        s = self.state
        fin_limit = math.radians(self.cfg.fin_limit_deg)
        throttle = clip(command.throttle, 0.0, self.cfg.throttle_max)
        pitch_fin = clip(command.pitch_fin, -fin_limit, fin_limit)
        yaw_fin = clip(command.yaw_fin, -fin_limit, fin_limit)
        target_u = 1.18 * throttle / self.cfg.throttle_max
        s.u += ((target_u - s.u) / 1.25 - 0.035 * s.u * abs(s.u)) * dt
        s.v += (-1.8 * s.v) * dt
        s.w += (-1.5 * s.w + 0.18 * s.pitch) * dt
        s.pitch_rate += (2.65 * pitch_fin - 1.35 * s.pitch_rate - 0.42 * s.pitch) * dt
        s.yaw_rate += (2.15 * yaw_fin - 1.18 * s.yaw_rate) * dt
        s.pitch = clip(s.pitch + s.pitch_rate * dt, math.radians(-22.0), math.radians(22.0))
        s.yaw = wrap_angle(s.yaw + s.yaw_rate * dt)
        r = body_to_ned_matrix(s.yaw, s.pitch)
        body_velocity = np.array([s.u, s.v, s.w], dtype=float)
        ned_velocity = r @ body_velocity + current_ned
        s.x += ned_velocity[0] * dt
        s.y += ned_velocity[1] * dt
        s.z = clip(s.z + ned_velocity[2] * dt, 0.05, 6.0)
        power_w = self.cfg.nominal_motor_w * (throttle / self.cfg.throttle_max) ** 2.35
        self.energy_wh += power_w * dt / 3600.0
        return ned_velocity


class SensorSuite:
    def __init__(self, cfg, seed):
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)

    def imu(self, state):
        yaw = wrap_angle(state.yaw + self.rng.normal(0.0, self.cfg.imu_angle_noise_rad))
        pitch = state.pitch + self.rng.normal(0.0, self.cfg.imu_angle_noise_rad)
        yaw_rate = state.yaw_rate + self.rng.normal(0.0, self.cfg.imu_rate_noise_radps)
        pitch_rate = state.pitch_rate + self.rng.normal(0.0, self.cfg.imu_rate_noise_radps)
        return yaw, pitch, yaw_rate, pitch_rate

    def dvl(self, state, ned_velocity):
        r = body_to_ned_matrix(state.yaw, state.pitch)
        body_ground = r.T @ ned_velocity
        omega = np.array([0.0, state.pitch_rate, state.yaw_rate], dtype=float)
        lever = np.array([self.cfg.dvl_x_m, self.cfg.dvl_y_m, self.cfg.dvl_z_m], dtype=float)
        lever_velocity = np.cross(omega, lever)
        noise = self.rng.normal(0.0, self.cfg.dvl_noise_mps, size=3)
        return body_ground + lever_velocity + noise

    def pressure(self, state):
        return np.array([state.z + self.rng.normal(0.0, self.cfg.pressure_noise_m)], dtype=float)


class MissionEnv:
    def __init__(self, cfg, seed):
        self.cfg = cfg
        self.seed = seed
        self.controller = CascadePID(cfg)
        self.vehicle = Vehicle(cfg)
        self.filter = UKF()
        self.sensors = SensorSuite(cfg, seed)
        self.current = CurrentProfile(seed + 1000, MAX_STEPS * DT)
        self.command = Command()
        self.t = 0.0
        self.step_index = 0
        self.history = []
        self.total_reward = 0.0

    def reset(self, seed=None):
        if seed is not None:
            self.seed = seed
        self.controller = CascadePID(self.cfg)
        self.vehicle = Vehicle(self.cfg)
        self.filter = UKF()
        self.sensors = SensorSuite(self.cfg, self.seed)
        self.current = CurrentProfile(self.seed + 1000, MAX_STEPS * DT)
        self.command = Command()
        self.t = 0.0
        self.step_index = 0
        self.history = []
        self.total_reward = 0.0
        return self.observation()

    def observation(self):
        x = self.filter.x
        return np.array(
            [
                clip(x[0] / self.cfg.target_forward_m, 0.0, 1.4),
                self.cfg.target_forward_m - x[0],
                self.cfg.target_depth_m - x[2],
                -x[1],
                x[3],
                x[5],
                x[6],
                x[7],
            ],
            dtype=float,
        )

    def step(self, action):
        previous_x = self.vehicle.state.x
        current_ned, current_mode = self.current.at(self.t)
        ned_velocity = self.vehicle.step(self.command, current_ned, DT)
        yaw_m, pitch_m, yaw_rate_m, pitch_rate_m = self.sensors.imu(self.vehicle.state)
        self.filter.predict(DT, yaw_rate_m, pitch_rate_m)
        dvl_m = self.sensors.dvl(self.vehicle.state, ned_velocity)
        pressure_m = self.sensors.pressure(self.vehicle.state)
        measurement = np.array([dvl_m[0], dvl_m[1], dvl_m[2], pressure_m[0], yaw_m, pitch_m], dtype=float)
        noise = np.diag(
            [
                self.cfg.dvl_noise_mps**2,
                self.cfg.dvl_noise_mps**2,
                self.cfg.dvl_noise_mps**2,
                self.cfg.pressure_noise_m**2,
                self.cfg.imu_angle_noise_rad**2,
                self.cfg.imu_angle_noise_rad**2,
            ]
        )
        self.filter.correct(
            measurement,
            lambda x: np.array([x[3], x[4], x[5], x[2], x[6], x[7]], dtype=float),
            noise,
            angle_indices=(4,),
        )
        self.command = self.controller.update(self.filter.x, action, DT)
        done = self.vehicle.state.x >= self.cfg.target_forward_m
        truncated = self.step_index + 1 >= MAX_STEPS and not done
        reward, reward_parts = self.reward(previous_x, done, truncated)
        self.total_reward += reward
        row = self.row(current_ned, current_mode, reward, reward_parts, done, truncated)
        self.history.append(row)
        self.t += DT
        self.step_index += 1
        return StepResult(self.observation(), reward, done, truncated, row)

    def reward(self, previous_x, done, truncated):
        s = self.vehicle.state
        progress = max(0.0, s.x - previous_x)
        depth_error = s.z - self.cfg.target_depth_m
        cross_track = s.y
        throttle_ratio = self.command.throttle / self.cfg.throttle_max
        fin_ratio = (abs(self.command.pitch_fin) + abs(self.command.yaw_fin)) / (2.0 * math.radians(self.cfg.fin_limit_deg))
        depth_penalty = -2.2 * abs(depth_error) - 0.9 * depth_error * depth_error
        cross_penalty = -0.85 * abs(cross_track)
        energy_penalty = -0.07 * throttle_ratio * throttle_ratio
        fin_penalty = -0.05 * fin_ratio
        time_penalty = -0.018
        progress_reward = 17.0 * progress
        safety_penalty = 0.0
        if s.z < 0.25 or s.z > self.cfg.max_depth_m:
            safety_penalty -= 12.0
        if abs(s.pitch) > math.radians(self.cfg.max_pitch_deg):
            safety_penalty -= 6.0
        terminal = 0.0
        if done:
            terminal = 220.0 - 62.0 * abs(depth_error) - 18.0 * abs(cross_track) - 0.32 * self.t
        if truncated:
            terminal = -120.0
        parts = {
            "progress": progress_reward,
            "depth": depth_penalty,
            "cross": cross_penalty,
            "energy": energy_penalty,
            "fin": fin_penalty,
            "time": time_penalty,
            "safety": safety_penalty,
            "terminal": terminal,
        }
        return float(sum(parts.values())), parts

    def row(self, current_ned, current_mode, reward, reward_parts, done, truncated):
        s = self.vehicle.state
        e = self.filter.x
        fin_max = math.radians(self.cfg.fin_limit_deg)
        esc_pwm = 1000.0 + 800.0 * self.command.throttle / self.cfg.throttle_max
        pitch_pwm = 1500.0 + 400.0 * clip(self.command.pitch_fin / fin_max, -1.0, 1.0)
        yaw_pwm = 1500.0 + 400.0 * clip(self.command.yaw_fin / fin_max, -1.0, 1.0)
        return {
            "t": self.t,
            "step": self.step_index,
            "x": s.x,
            "y": s.y,
            "z": s.z,
            "u": s.u,
            "yaw": s.yaw,
            "pitch": s.pitch,
            "ex": e[0],
            "ey": e[1],
            "ez": e[2],
            "eu": e[3],
            "throttle": self.command.throttle,
            "pitch_fin": self.command.pitch_fin,
            "yaw_fin": self.command.yaw_fin,
            "esc_pwm": esc_pwm,
            "pitch_pwm": pitch_pwm,
            "yaw_pwm": yaw_pwm,
            "current_n": current_ned[0],
            "current_e": current_ned[1],
            "current_d": current_ned[2],
            "current_mode": current_mode,
            "energy_wh": self.vehicle.energy_wh,
            "reward": reward,
            "done": done,
            "truncated": truncated,
            **{f"reward_{k}": v for k, v in reward_parts.items()},
        }

    def run(self, action):
        self.reset(self.seed)
        results = []
        for _ in range(MAX_STEPS):
            result = self.step(action)
            results.append(result)
            if result.done or result.truncated:
                break
        return results


def action_for_episode(index):
    speed_grid = [-0.04, -0.02, 0.0, 0.02, 0.04, 0.06]
    depth_grid = [-0.06, -0.03, 0.0, 0.03, 0.06, 0.09]
    heading_grid = [-0.035, -0.018, 0.0, 0.018, 0.035, 0.052]
    return Action(
        speed_trim=speed_grid[index % len(speed_grid)],
        depth_trim=depth_grid[(index // len(speed_grid)) % len(depth_grid)],
        heading_trim=heading_grid[(index * 5 + 2) % len(heading_grid)],
    )


def summarize(history, cfg, reward):
    x = np.array([r["x"] for r in history], dtype=float)
    y = np.array([r["y"] for r in history], dtype=float)
    z = np.array([r["z"] for r in history], dtype=float)
    t = np.array([r["t"] for r in history], dtype=float)
    success = len(history) > 0 and bool(history[-1]["done"]) and abs(z[-1] - cfg.target_depth_m) <= 0.35 and abs(y[-1]) <= 1.50
    return {
        "success": success,
        "steps": len(history),
        "time_s": float(t[-1]) if len(t) else 0.0,
        "reward": float(reward),
        "final_x_m": float(x[-1]) if len(x) else 0.0,
        "final_depth_m": float(z[-1]) if len(z) else 0.0,
        "final_cross_m": float(y[-1]) if len(y) else 0.0,
        "depth_rmse_m": rmse(z - cfg.target_depth_m),
        "max_cross_m": float(np.max(np.abs(y))) if len(y) else 0.0,
        "energy_wh": float(history[-1]["energy_wh"]) if len(history) else 0.0,
    }


def run_experiments(episodes=EPISODES, seed=SEED):
    cfg = VehicleConfig()
    summaries = []
    histories = []
    for ep in range(episodes):
        action = action_for_episode(ep)
        env = MissionEnv(cfg, seed + ep * 17)
        env.run(action)
        summary = summarize(env.history, cfg, env.total_reward)
        summary["episode"] = ep + 1
        summary["speed_trim"] = action.speed_trim
        summary["depth_trim"] = action.depth_trim
        summary["heading_trim"] = action.heading_trim
        summaries.append(summary)
        histories.append(env.history)
    best_index = max(range(len(summaries)), key=lambda i: (summaries[i]["success"], summaries[i]["reward"]))
    return cfg, summaries, histories, best_index


def print_table(summaries, best_index):
    ordered = sorted(summaries, key=lambda r: (r["success"], r["reward"]), reverse=True)
    print("ep ok steps time reward x depth cross drmse energy")
    for row in ordered[:10]:
        ok = "PASS" if row["success"] else "FAIL"
        print(
            f'{row["episode"]:02d} {ok:4s} {row["steps"]:4d} {row["time_s"]:6.1f} '
            f'{row["reward"]:8.1f} {row["final_x_m"]:6.2f} {row["final_depth_m"]:5.2f} '
            f'{row["final_cross_m"]:6.2f} {row["depth_rmse_m"]:5.3f} {row["energy_wh"]:5.2f}'
        )
    best = summaries[best_index]
    print()
    print(
        f'best_episode={best["episode"]} steps={best["steps"]} reward={best["reward"]:.1f} '
        f'time={best["time_s"]:.1f}s depth={best["final_depth_m"]:.2f}m cross={best["final_cross_m"]:.2f}m'
    )


def as_arrays(history):
    keys = [
        "t",
        "x",
        "y",
        "z",
        "ex",
        "ey",
        "ez",
        "throttle",
        "pitch_fin",
        "yaw_fin",
        "current_n",
        "current_e",
        "current_d",
        "reward",
        "energy_wh",
        "esc_pwm",
        "pitch_pwm",
        "yaw_pwm",
    ]
    return {k: np.array([r[k] for r in history], dtype=float) for k in keys}


def plot_results(cfg, summaries, histories, best_index):
    best_history = histories[best_index]
    data = as_arrays(best_history)
    ep = np.array([s["episode"] for s in summaries], dtype=int)
    rewards = np.array([s["reward"] for s in summaries], dtype=float)
    steps = np.array([s["steps"] for s in summaries], dtype=float)
    success = np.array([s["success"] for s in summaries], dtype=bool)
    fig1, ax = plt.subplots(1, 2, figsize=(13, 4))
    ax[0].bar(ep, rewards, color=np.where(success, "seagreen", "indianred"))
    ax[0].set_title("Episode reward")
    ax[0].set_xlabel("Episode")
    ax[0].set_ylabel("Reward")
    ax[1].bar(ep, steps, color=np.where(success, "steelblue", "gray"))
    ax[1].set_title("Step count")
    ax[1].set_xlabel("Episode")
    ax[1].set_ylabel("Step")
    fig1.tight_layout()
    fig2, ax = plt.subplots(3, 2, figsize=(14, 10))
    ax[0, 0].plot(data["t"], data["x"], color="steelblue")
    ax[0, 0].axhline(cfg.target_forward_m, color="black", linestyle=":")
    ax[0, 0].set_title("Forward")
    ax[0, 0].set_ylabel("m")
    ax[0, 1].plot(data["t"], data["z"], color="seagreen", label="truth")
    ax[0, 1].plot(data["t"], data["ez"], color="darkorange", alpha=0.8, label="ukf")
    ax[0, 1].axhline(cfg.target_depth_m, color="black", linestyle=":")
    ax[0, 1].set_title("Depth")
    ax[0, 1].set_ylabel("m")
    ax[0, 1].legend()
    ax[1, 0].plot(data["t"], data["y"], color="indianred", label="truth")
    ax[1, 0].plot(data["t"], data["ey"], color="purple", alpha=0.75, label="ukf")
    ax[1, 0].axhline(0.0, color="black", linestyle=":")
    ax[1, 0].set_title("Cross-track")
    ax[1, 0].set_ylabel("m")
    ax[1, 0].legend()
    ax[1, 1].plot(data["t"], data["throttle"], color="steelblue", label="throttle")
    ax[1, 1].plot(data["t"], np.degrees(data["pitch_fin"]), color="seagreen", label="pitch fin")
    ax[1, 1].plot(data["t"], np.degrees(data["yaw_fin"]), color="indianred", label="yaw fin")
    ax[1, 1].set_title("Control")
    ax[1, 1].legend()
    ax[2, 0].plot(data["t"], data["current_n"], color="steelblue", label="north")
    ax[2, 0].plot(data["t"], data["current_e"], color="indianred", label="east")
    ax[2, 0].plot(data["t"], data["current_d"], color="seagreen", label="down")
    ax[2, 0].set_title("Current")
    ax[2, 0].set_ylabel("m/s")
    ax[2, 0].set_xlabel("s")
    ax[2, 0].legend()
    ax[2, 1].plot(data["x"], data["z"], color="black")
    ax[2, 1].axhline(cfg.target_depth_m, color="seagreen", linestyle=":")
    ax[2, 1].axvline(cfg.target_forward_m, color="steelblue", linestyle=":")
    ax[2, 1].set_title("Mission path")
    ax[2, 1].set_xlabel("forward m")
    ax[2, 1].set_ylabel("depth m")
    ax[2, 1].invert_yaxis()
    fig2.tight_layout()
    fig1.savefig("sara_episode_summary.png", dpi=150)
    fig2.savefig("sara_best_episode.png", dpi=150)
    if "agg" not in plt.get_backend().lower():
        plt.show(block=False)
        plt.pause(0.1)
    else:
        plt.close(fig1)
        plt.close(fig2)
    return Path("sara_episode_summary.png"), Path("sara_best_episode.png")


def save_best_csv(history, filename="sara_best_episode.csv"):
    keys = list(history[0].keys())
    path = Path(filename)
    with path.open("w", encoding="utf-8") as f:
        f.write(",".join(keys) + "\n")
        for row in history:
            f.write(",".join(str(row[k]) for k in keys) + "\n")
    return path


def save_video(history, cfg, filename="sara_mission_video.mp4"):
    data = as_arrays(history)
    frame_count = min(120, len(history))
    indices = np.linspace(0, len(history) - 1, frame_count).astype(int)
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    fig.patch.set_facecolor("#f4f7fb")
    for a in ax:
        a.set_facecolor("#edf5fb")
        a.grid(True, alpha=0.24)
    ax[0].set_xlim(0.0, cfg.target_forward_m + 4.0)
    ax[0].set_ylim(3.2, 0.2)
    ax[0].axhline(cfg.target_depth_m, color="#2f855a", linewidth=1.4, linestyle="--")
    ax[0].axvline(cfg.target_forward_m, color="#2b6cb0", linewidth=1.4, linestyle="--")
    ax[0].set_title("Depth mission")
    ax[0].set_xlabel("forward m")
    ax[0].set_ylabel("depth m")
    ax[1].set_xlim(0.0, cfg.target_forward_m + 4.0)
    ax[1].set_ylim(-2.2, 2.2)
    ax[1].axhline(0.0, color="#2f855a", linewidth=1.4, linestyle="--")
    ax[1].axvline(cfg.target_forward_m, color="#2b6cb0", linewidth=1.4, linestyle="--")
    ax[1].set_title("Cross-track")
    ax[1].set_xlabel("forward m")
    ax[1].set_ylabel("cross m")
    depth_line, = ax[0].plot([], [], color="#1a365d", linewidth=2.2)
    depth_dot, = ax[0].plot([], [], marker="o", color="#d69e2e", markersize=9)
    cross_line, = ax[1].plot([], [], color="#742a2a", linewidth=2.2)
    cross_dot, = ax[1].plot([], [], marker="o", color="#d69e2e", markersize=9)
    text = fig.text(0.5, 0.02, "", ha="center", fontsize=11)

    def update(frame):
        i = indices[frame]
        depth_line.set_data(data["x"][: i + 1], data["z"][: i + 1])
        depth_dot.set_data([data["x"][i]], [data["z"][i]])
        cross_line.set_data(data["x"][: i + 1], data["y"][: i + 1])
        cross_dot.set_data([data["x"][i]], [data["y"][i]])
        text.set_text(
            f't={data["t"][i]:.1f}s  x={data["x"][i]:.2f}m  depth={data["z"][i]:.2f}m  '
            f'cross={data["y"][i]:.2f}m  throttle={data["throttle"][i]:.2f}'
        )
        return depth_line, depth_dot, cross_line, cross_dot, text

    anim = animation.FuncAnimation(fig, update, frames=frame_count, interval=50, blit=True)
    path = Path(filename)
    try:
        if not FFMPEG_PATH:
            raise RuntimeError("ffmpeg")
        plt.rcParams["animation.ffmpeg_path"] = FFMPEG_PATH
        writer = animation.FFMpegWriter(fps=20, bitrate=2200)
        anim.save(path, writer=writer, dpi=115)
    except Exception:
        path = Path("sara_mission_video.gif")
        writer = animation.PillowWriter(fps=20)
        anim.save(path, writer=writer, dpi=115)
    plt.close(fig)
    return path


def save_html(cfg, summaries, history, best_index, png_paths, video_path, csv_path, filename="sara_mission_report.html"):
    best = summaries[best_index]
    ordered = sorted(summaries, key=lambda r: (r["success"], r["reward"]), reverse=True)[:8]
    rows = "\n".join(
        f'<tr><td>{r["episode"]}</td><td>{"PASS" if r["success"] else "FAIL"}</td><td>{r["steps"]}</td>'
        f'<td>{r["time_s"]:.1f}</td><td>{r["reward"]:.1f}</td><td>{r["final_x_m"]:.2f}</td>'
        f'<td>{r["final_depth_m"]:.2f}</td><td>{r["final_cross_m"]:.2f}</td><td>{r["energy_wh"]:.2f}</td></tr>'
        for r in ordered
    )
    media_tag = (
        f'<video controls autoplay muted loop src="{video_path.name}"></video>'
        if video_path.suffix.lower() == ".mp4"
        else f'<img src="{video_path.name}" alt="mission animation">'
    )
    path = Path(filename)
    path.write_text(
        f"""<!doctype html>
<html lang="tr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SARA Mission Report</title>
<style>
body{{margin:0;font-family:Arial,Helvetica,sans-serif;background:#f4f7fb;color:#172033}}
main{{max-width:1180px;margin:0 auto;padding:28px}}
h1{{font-size:30px;margin:0 0 6px}}
h2{{font-size:20px;margin:28px 0 12px}}
.grid{{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:20px 0}}
.card{{background:white;border:1px solid #d9e2ec;border-radius:8px;padding:14px}}
.k{{font-size:12px;color:#5b6b7f;text-transform:uppercase;letter-spacing:.04em}}
.v{{font-size:25px;font-weight:700;margin-top:5px}}
img,video{{width:100%;border:1px solid #d9e2ec;border-radius:8px;background:white}}
table{{width:100%;border-collapse:collapse;background:white;border:1px solid #d9e2ec;border-radius:8px;overflow:hidden}}
th,td{{padding:10px;border-bottom:1px solid #e7edf3;text-align:right}}
th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){{text-align:left}}
th{{background:#edf2f7;color:#334155}}
.media{{display:grid;grid-template-columns:1fr;gap:16px}}
@media (max-width:800px){{.grid{{grid-template-columns:repeat(2,1fr)}}main{{padding:16px}}}}
</style>
</head>
<body>
<main>
<h1>SARA 50m / 2m Mission</h1>
<div class="grid">
<div class="card"><div class="k">Status</div><div class="v">{"PASS" if best["success"] else "FAIL"}</div></div>
<div class="card"><div class="k">Episode / Step</div><div class="v">{best["episode"]} / {best["steps"]}</div></div>
<div class="card"><div class="k">Forward</div><div class="v">{best["final_x_m"]:.2f} m</div></div>
<div class="card"><div class="k">Depth</div><div class="v">{best["final_depth_m"]:.2f} m</div></div>
<div class="card"><div class="k">Cross-track</div><div class="v">{best["final_cross_m"]:.2f} m</div></div>
<div class="card"><div class="k">Reward</div><div class="v">{best["reward"]:.1f}</div></div>
<div class="card"><div class="k">Energy</div><div class="v">{best["energy_wh"]:.2f} Wh</div></div>
<div class="card"><div class="k">Mass</div><div class="v">{cfg.mass_kg:.2f} kg</div></div>
</div>
<h2>Animation</h2>
<div class="media">{media_tag}</div>
<h2>Plots</h2>
<div class="media">
<img src="{png_paths[1].name}" alt="best episode plots">
<img src="{png_paths[0].name}" alt="episode summary">
</div>
<h2>Top Episodes</h2>
<table>
<thead><tr><th>Ep</th><th>OK</th><th>Step</th><th>Time</th><th>Reward</th><th>X</th><th>Depth</th><th>Cross</th><th>Wh</th></tr></thead>
<tbody>
{rows}
</tbody>
</table>
</main>
</body>
</html>
""",
        encoding="utf-8",
    )
    return path


cfg, summaries, histories, best_index = run_experiments()
print_table(summaries, best_index)
png_paths = plot_results(cfg, summaries, histories, best_index)
csv_path = save_best_csv(histories[best_index])
video_path = save_video(histories[best_index], cfg)
html_path = save_html(cfg, summaries, histories[best_index], best_index, png_paths, video_path, csv_path)
print(f"csv={csv_path.resolve()}")
print(f"plots={png_paths[0].resolve()} | {png_paths[1].resolve()}")
print(f"video={video_path.resolve()}")
print(f"html={html_path.resolve()}")
