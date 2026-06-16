#!/usr/bin/env python3
"""Run all algorithm validation scenarios and summarize their artifacts."""

import argparse
import csv
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import yaml


CASE_DEFAULTS = {
    "navigation_straight": ["--duration", "50", "--warmup", "5"],
    "navigation_resilience": ["--duration", "70", "--warmup", "5"],
    "guidance_los": [
        "--duration", "65", "--warmup", "5",
        "--distance", "40", "--cross-track", "5",
    ],
    "guidance_waypoint": [
        "--duration", "150", "--warmup", "5",
        "--distance", "60", "--cross-track", "3",
        "--waypoint-acceptance", "1.5",
    ],
    "controller_tracking": [
        "--duration", "55", "--warmup", "5",
        "--distance", "35", "--depth", "2", "--speed", "0.8",
    ],
    "rl_policy": [
        "--duration", "75", "--warmup", "5",
        "--distance", "50", "--depth", "2", "--speed", "0.8",
        "--rl-dive-speed", "1.2", "--rl-settle-timeout", "75",
        "--current-x", "0.0", "--current-y", "0.0", "--current-z", "0.0",
    ],
    "rl_policy_following_current": [
        "--duration", "75", "--warmup", "5",
        "--distance", "50", "--depth", "2", "--speed", "0.8",
        "--rl-dive-speed", "1.2", "--rl-settle-timeout", "75",
        "--current-x", "0.25", "--current-y", "0.0", "--current-z", "0.0",
    ],
    "rl_policy_cross_current": [
        "--duration", "75", "--warmup", "5",
        "--distance", "50", "--depth", "2", "--speed", "0.8",
        "--rl-dive-speed", "1.2", "--rl-settle-timeout", "75",
        "--current-x", "0.0", "--current-y", "0.25", "--current-z", "0.0",
    ],
    "rl_policy_diagonal_current": [
        "--duration", "85", "--warmup", "5",
        "--distance", "50", "--depth", "2", "--speed", "0.8",
        "--rl-dive-speed", "1.2", "--rl-settle-timeout", "75",
        "--current-x", "0.25", "--current-y", "0.2", "--current-z", "0.0",
    ],
    "rl_policy_reverse_current": [
        "--duration", "90", "--warmup", "5",
        "--distance", "50", "--depth", "2", "--speed", "0.8",
        "--rl-dive-speed", "1.2", "--rl-settle-timeout", "75",
        "--current-x", "-0.2", "--current-y", "0.0", "--current-z", "0.0",
    ],
    "rl_policy_hard_cross_current": [
        "--duration", "95", "--warmup", "5",
        "--distance", "50", "--depth", "2", "--speed", "0.8",
        "--rl-dive-speed", "1.2", "--rl-settle-timeout", "75",
        "--current-x", "0.0", "--current-y", "0.4", "--current-z", "0.0",
    ],
    "sensor_health": [
        "--duration", "35", "--warmup", "5", "--depth", "2",
    ],
    "ocean_current_response": [
        "--duration", "45", "--warmup", "5", "--depth", "2",
        "--current-x", "0.4", "--current-y", "0.2", "--current-z", "0.0",
    ],
    "ocean_current_services": [
        "--duration", "35", "--warmup", "5", "--depth", "2",
    ],
    "stage1_fsm": ["--duration", "240"],
    "stage2_bt": ["--duration", "90"],
}


def _latest_case_output(root, case):
    matches = []
    for candidate in root.glob(f"{case}_*"):
        manifest_path = candidate / "test_manifest.yaml"
        if not candidate.is_dir() or not manifest_path.exists():
            continue
        try:
            manifest = yaml.safe_load(
                manifest_path.read_text(encoding="utf-8")
            ) or {}
        except (OSError, yaml.YAMLError):
            continue
        if manifest.get("case") == case:
            matches.append(candidate)
    matches.sort()
    return matches[-1] if matches else None


def _write_summary(output, rows):
    fields = ["case", "status", "returncode", "output", "analysis_log"]
    with (output / "validation_suite_results.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    lines = [
        "# Algoritma Doğrulama Suite Sonuçları",
        "",
        "| Senaryo | Sonuç | Return code | Çıktı |",
        "|---|---|---:|---|",
    ]
    for row in rows:
        lines.append(
            f"| `{row['case']}` | {row['status']} | {row['returncode']} | "
            f"`{row['output']}` |"
        )
    (output / "validation_suite_results.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def _start_simulation(log_path):
    stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2", "launch", "zemheri_simulation", "simulation.launch.py",
            "control_backend:=ros",
            "enable_interface:=false",
            "enable_system_recording:=false",
            "use_rviz:=false",
        ],
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return process, stream


def _stop_process(process):
    if process is None:
        return
    for stop_signal, timeout in [
        (signal.SIGINT, 8),
        (signal.SIGTERM, 4),
        (signal.SIGKILL, 2),
    ]:
        try:
            os.killpg(process.pid, stop_signal)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cases",
        nargs="+",
        choices=sorted(CASE_DEFAULTS),
        default=list(CASE_DEFAULTS),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("analysis/report_validation"),
    )
    parser.add_argument(
        "--continue-on-failure",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--launch-simulation",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Launch a fresh ROS-controller Gazebo session for every case.",
    )
    parser.add_argument(
        "--simulation-warmup",
        type=float,
        default=14.0,
        help="Wall-time wait after launching each isolated simulation.",
    )
    args = parser.parse_args()
    root = args.output_root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    suite_output = root / f"suite_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    suite_output.mkdir()
    rows = []
    for case in args.cases:
        simulation = None
        simulation_stream = None
        command = [
            "ros2", "run", "zemheri_simulation", "report_test_runner.py",
            "--case", case,
            "--output-root", str(root),
            *CASE_DEFAULTS[case],
        ]
        log_path = suite_output / f"{case}.log"
        try:
            if args.launch_simulation:
                simulation, simulation_stream = _start_simulation(
                    suite_output / f"{case}_simulation.log"
                )
                time.sleep(args.simulation_warmup)
                if simulation.poll() is not None:
                    result = subprocess.CompletedProcess(
                        args=command,
                        returncode=simulation.returncode or 1,
                    )
                    log_path.write_text(
                        "Simulation exited before validation runner started.\n",
                        encoding="utf-8",
                    )
                else:
                    with log_path.open("w", encoding="utf-8") as stream:
                        result = subprocess.run(
                            command, check=False, stdout=stream,
                            stderr=subprocess.STDOUT,
                        )
            else:
                with log_path.open("w", encoding="utf-8") as stream:
                    result = subprocess.run(
                        command, check=False, stdout=stream,
                        stderr=subprocess.STDOUT,
                    )
        finally:
            _stop_process(simulation)
            if simulation_stream is not None:
                simulation_stream.close()
        case_output = _latest_case_output(root, case)
        analysis_log = case_output / "analysis.log" if case_output else None
        manifest_path = case_output / "test_manifest.yaml" if case_output else None
        analysis_status = None
        if manifest_path is not None and manifest_path.exists():
            try:
                manifest = yaml.safe_load(
                    manifest_path.read_text(encoding="utf-8")
                ) or {}
                analysis_status = manifest.get("analysis", {}).get("status")
            except (OSError, yaml.YAMLError):
                analysis_status = None
        status = (
            "TAMAMLANDI"
            if result.returncode == 0
            and analysis_status == "completed"
            else "BAŞARISIZ"
        )
        rows.append({
            "case": case,
            "status": status,
            "returncode": result.returncode,
            "output": str(case_output or ""),
            "analysis_log": str(analysis_log or ""),
        })
        _write_summary(suite_output, rows)
        if result.returncode != 0 and not args.continue_on_failure:
            break
    print(suite_output)


if __name__ == "__main__":
    main()
