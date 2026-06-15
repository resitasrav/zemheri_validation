#!/usr/bin/env python3
"""Run final ROS-controller validations and build one artifact index."""

import argparse
import csv
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import yaml


RL_POLICY_EPISODES = [
    ("no_current", [
        "--duration", "75", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
        "--current-x", "0.0", "--current-y", "0.0", "--current-z", "0.0",
    ]),
    ("following_current", [
        "--duration", "75", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
        "--current-x", "0.25", "--current-y", "0.0", "--current-z", "0.0",
    ]),
    ("cross_current", [
        "--duration", "75", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
        "--current-x", "0.0", "--current-y", "0.25", "--current-z", "0.0",
    ]),
    ("diagonal_current", [
        "--duration", "85", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
        "--current-x", "0.25", "--current-y", "0.20", "--current-z", "0.0",
    ]),
    ("reverse_current", [
        "--duration", "90", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
        "--current-x", "-0.20", "--current-y", "0.0", "--current-z", "0.0",
    ]),
    ("hard_cross_current", [
        "--duration", "95", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
        "--current-x", "0.0", "--current-y", "0.40", "--current-z", "0.0",
    ]),
]


CASE_ARGUMENTS = {
    "navigation_straight": ["--duration", "50", "--warmup", "5", "--depth", "2"],
    "navigation_resilience": ["--duration", "70", "--warmup", "5", "--depth", "2"],
    "guidance_los": [
        "--duration", "65", "--warmup", "5", "--distance", "40",
        "--cross-track", "5", "--depth", "2",
    ],
    "guidance_waypoint": [
        "--duration", "150", "--warmup", "5", "--distance", "60",
        "--cross-track", "3", "--waypoint-acceptance", "1.5", "--depth", "2",
    ],
    "controller_tracking": [
        "--duration", "55", "--warmup", "5", "--distance", "35",
        "--depth", "2", "--speed", "0.8",
    ],
    "rl_policy": [
        "--duration", "75", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
    ],
    "rl_policy_following_current": [
        "--duration", "75", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
        "--current-x", "0.25", "--current-y", "0.0", "--current-z", "0.0",
    ],
    "rl_policy_cross_current": [
        "--duration", "75", "--warmup", "5", "--distance", "50",
        "--depth", "2", "--speed", "0.8",
        "--current-x", "0.0", "--current-y", "0.25", "--current-z", "0.0",
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
    "stage2_bt": ["--duration", "100"],
}


def stop_process(process):
    """Stop a subprocess and all children started in its process group."""
    if process is None:
        return

    def group_alive():
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return False
        return True

    for stop_signal, timeout in [
        (signal.SIGINT, 12),
        (signal.SIGTERM, 5),
        (signal.SIGKILL, 2),
    ]:
        if not group_alive():
            return
        try:
            os.killpg(process.pid, stop_signal)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass

        deadline = time.monotonic() + timeout
        while group_alive() and time.monotonic() < deadline:
            time.sleep(0.2)


def run_logged(command, log_path, env=None):
    """Run one command while preserving its complete terminal output."""
    with log_path.open("w", encoding="utf-8") as stream:
        return subprocess.run(
            command,
            check=False,
            stdout=stream,
            stderr=subprocess.STDOUT,
            env=env,
        )


def newest_case(root, case, previous):
    """Return the newly produced case directory."""
    candidates = sorted(item for item in root.glob(f"{case}_*") if item.is_dir())
    new = [item for item in candidates if item not in previous]
    return new[-1] if new else None


def read_metric_files(case_path):
    """Collect compact key/value metrics from known analyzer outputs."""
    metrics = {}
    if case_path is None:
        return metrics

    for path in sorted(case_path.rglob("*.csv")):
        if path.name not in {
            "summary.csv",
            "guidance_summary.csv",
            "guidance_validation_summary.csv",
            "navigation_resilience_summary.csv",
            "rl_policy_summary.csv",
            "rl_episode_summary.csv",
            "sensor_health_summary.csv",
            "ocean_current_summary.csv",
            "ocean_current_service_summary.csv",
        }:
            continue

        try:
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))
        except OSError:
            continue

        if len(rows) == 2 and len(rows[0]) == len(rows[1]):
            metrics.update(dict(zip(rows[0], rows[1])))
        elif rows and len(rows[0]) == 2:
            metrics.update({row[0]: row[1] for row in rows[1:] if len(row) == 2})

    return metrics


def result_status(completed, metrics):
    """Separate successful execution from an explicit algorithm rejection."""
    decision_text = " ".join(
        str(value) for key, value in metrics.items()
        if "karar" in key.lower() or "decision" in key.lower()
    ).upper()

    if "BAŞARISIZ" in decision_text or "FAIL" in decision_text:
        return "BAŞARISIZ"
    if "KABUL" in decision_text or "PASS" in decision_text:
        return "KABUL"
    return "TAMAMLANDI" if completed else "ÇALIŞMADI"


def write_index(root, rows):
    """Write the single human-readable validation report index."""
    lines = [
        "# SARA Güncel Algoritma Doğrulama Testleri",
        "",
        f"Üretim zamanı: `{datetime.now().isoformat(timespec='seconds')}`",
        "",
        "Bu klasördeki Gazebo testleri `control_backend:=ros` ile yürütülür. "
        "ArduPilot henüz hız ve tutum kontrolü bakımından kalibre edilmediği "
        "için performans doğrulamasına dahil edilmemiştir.",
        "",
        "RL policy validation gerçek Gazebo modeli, UKF, sensör, guidance ve "
        "controller zinciri üzerinden seçilmiş politika adayını doğrular. "
        "Bu sonuç eğitilmiş bir SAC ajanı sonucu değildir.",
        "",
        "RL policy ana testi episode matrisi olarak çalıştırılır. Her episode "
        "için Gazebo sıfırdan başlatılır, rosbag kaydı alınır, analiz yapılır "
        "ve Gazebo kapatılır.",
        "",
        "Controller tracking çıktısı mevcut analiz aracı nedeniyle öncelikle "
        "ground-truth/UKF doğruluğunu ve aracın gerçekleşen hareketini raporlar. "
        "Hedef hız, derinlik ve yaw referanslarına göre ayrı kontrol-hata "
        "analizi sonraki geliştirme adımıdır.",
        "",
        "## Test Özeti",
        "",
        "| Test | Durum | Çıktı klasörü |",
        "|---|---|---|",
    ]

    for row in rows:
        output = Path(row["output"])
        relative = output.relative_to(root) if output.is_relative_to(root) else output
        lines.append(
            f"| `{row['case']}` | {row['status']} | [{relative}]({relative}) |"
        )

    lines.extend(["", "## Test Ayrıntıları", ""])

    for row in rows:
        output = Path(row["output"])
        relative = output.relative_to(root) if output.is_relative_to(root) else output

        lines.extend([
            f"### {row['case']}",
            "",
            f"- Durum: **{row['status']}**",
            f"- Çıktı: [{relative}]({relative})",
        ])

        if row.get("metrics"):
            for key, value in list(row["metrics"].items())[:14]:
                lines.append(f"- `{key}`: {value}")

        figures = sorted(output.rglob("*.png")) if output.exists() else []
        csv_files = sorted(output.rglob("*.csv")) if output.exists() else []
        bags = sorted(output.rglob("metadata.yaml")) if output.exists() else []

        lines.append(f"- PNG sayısı: {len(figures)}")
        lines.append(f"- CSV sayısı: {len(csv_files)}")
        lines.append(f"- Rosbag sayısı: {len(bags)}")

        for figure in figures[:4]:
            figure_relative = figure.relative_to(root)
            lines.append(f"- Grafik: [{figure.name}]({figure_relative})")

        lines.append("")

    completed_cases = {row["case"] for row in rows}
    pending_cases = []

    for case in CASE_ARGUMENTS:
        if case == "rl_policy":
            has_rl_matrix = any(row["case"].startswith("rl_policy_ep") for row in rows)
            if not has_rl_matrix:
                pending_cases.append(case)
        elif case not in completed_cases:
            pending_cases.append(case)

    if pending_cases:
        lines.extend([
            "## Henüz Çalıştırılmayan Testler",
            "",
            *[f"- `{case}`" for case in pending_cases],
            "",
        ])

    lines.extend([
        "## Tekrar Çalıştırma",
        "",
        "```bash",
        "source ~/zemheri_ws/install/setup.bash",
        "python3 analysis/final_validation/test_scripts/run_final_validation.py",
        "```",
        "",
        "Belirli testler için `--cases navigation_straight guidance_los` "
        "seçeneği kullanılabilir.",
        "",
        "RL episode matrisi için:",
        "",
        "```bash",
        "python3 analysis/final_validation/test_scripts/run_final_validation.py --cases rl_policy",
        "```",
        "",
    ])

    (root / "README.md").write_text("\n".join(lines), encoding="utf-8")


def existing_results(root):
    """Load the latest completed artifact set for each previously run case."""
    rows = []

    for case in CASE_ARGUMENTS:
        candidates = sorted(
            item for item in root.glob(f"{case}_*") if item.is_dir()
        )

        for output in reversed(candidates):
            manifest_path = output / "test_manifest.yaml"
            if manifest_path.exists():
                manifest = yaml.safe_load(
                    manifest_path.read_text(encoding="utf-8")
                ) or {}
                completed = (
                    manifest.get("analysis", {}).get("status") == "completed"
                )
                metrics = read_metric_files(output)
                rows.append({
                    "case": case,
                    "status": result_status(completed, metrics),
                    "output": str(output),
                    "metrics": metrics,
                })
                break

    return rows


def launch_simulation(root, case_name, environment):
    """Start Gazebo simulation for one isolated validation run."""
    simulation_log = root / f"{case_name}_simulation.log"
    simulation_stream = simulation_log.open("w", encoding="utf-8")

    simulation = subprocess.Popen(
        [
            "ros2", "launch", "zemheri_simulation", "simulation.launch.py",
            "control_backend:=ros",
            "enable_interface:=false",
            "enable_system_recording:=false",
            "use_rviz:=false",
        ],
        stdout=simulation_stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=environment,
    )

    return simulation, simulation_stream


def run_one_case(case, runner_case, runner_args, root, environment, simulation_warmup):
    """Run one validation case with a fresh Gazebo session."""
    before = set(root.glob(f"{runner_case}_*"))

    simulation, simulation_stream = launch_simulation(root, case, environment)

    try:
        time.sleep(simulation_warmup)

        if simulation.poll() is not None:
            result = subprocess.CompletedProcess(
                args=["simulation.launch.py"],
                returncode=simulation.returncode or 1,
            )
            (root / f"{case}_runner.log").write_text(
                "Simulation exited before validation runner started.\n",
                encoding="utf-8",
            )
        else:
            result = run_logged(
                [
                    "ros2", "run", "zemheri_simulation",
                    "report_test_runner.py",
                    "--case", runner_case,
                    "--output-root", str(root),
                    *runner_args,
                ],
                root / f"{case}_runner.log",
                environment,
            )
    finally:
        stop_process(simulation)
        simulation_stream.close()

    output = newest_case(root, runner_case, before)

    manifest = {}
    if output is not None and (output / "test_manifest.yaml").exists():
        manifest = yaml.safe_load(
            (output / "test_manifest.yaml").read_text(encoding="utf-8")
        ) or {}

    completed = (
        result.returncode == 0
        and manifest.get("analysis", {}).get("status") == "completed"
    )

    metrics = read_metric_files(output) if output else {}

    return {
        "case": case,
        "status": result_status(completed, metrics),
        "output": str(output or root),
        "metrics": metrics,
        "completed": completed,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cases",
        nargs="+",
        choices=list(CASE_ARGUMENTS),
        default=list(CASE_ARGUMENTS),
    )
    parser.add_argument("--simulation-warmup", type=float, default=14.0)
    parser.add_argument("--keep-going", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("analysis/final_validation/results"),
    )

    args = parser.parse_args()

    repository = Path(__file__).resolve().parents[3]
    root = (repository / args.output_root).resolve()
    root.mkdir(parents=True, exist_ok=True)

    environment = os.environ.copy()
    environment["ROS_LOG_DIR"] = str(root / "ros_logs")

    rows = existing_results(root)

    for case in args.cases:
        if case == "rl_policy":
            rows = [
                row for row in rows
                if row["case"] != "rl_policy"
                and not row["case"].startswith("rl_policy_ep")
            ]

            for episode_index, (episode_name, episode_args) in enumerate(
                RL_POLICY_EPISODES,
                start=1,
            ):
                episode_case = f"rl_policy_ep{episode_index:02d}_{episode_name}"

                row = run_one_case(
                    case=episode_case,
                    runner_case="rl_policy",
                    runner_args=episode_args,
                    root=root,
                    environment=environment,
                    simulation_warmup=args.simulation_warmup,
                )

                completed = row.pop("completed")
                rows.append(row)
                write_index(root.parent, rows)

                if not completed and not args.keep_going:
                    break

            continue

        rows = [row for row in rows if row["case"] != case]

        row = run_one_case(
            case=case,
            runner_case=case,
            runner_args=CASE_ARGUMENTS[case],
            root=root,
            environment=environment,
            simulation_warmup=args.simulation_warmup,
        )

        completed = row.pop("completed")
        rows.append(row)
        write_index(root.parent, rows)

        if not completed and not args.keep_going:
            break

    write_index(root.parent, rows)


if __name__ == "__main__":
    main()