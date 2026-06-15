#!/usr/bin/env python3
"""
SARA Validation Suite — artefact doğrulama scripti
==================================================

Repo teslim öncesi tutarlılık kontrolü yapar. Hiçbir metriği "varsayım" ile
kabul etmez; dosyalardan okur ve beklenenle karşılaştırır.

Kontroller:
  1. README ve wiki içindeki relative linklerin hedef dosyaları var mı?
  2. Önemli CSV'ler okunabiliyor mu? (episode + diagnosis özetleri)
  3. sara_best_episode.csv kolonları beklenen şema ile uyumlu mu?
  4. Episode bitiş koşulları: done=True, truncated=False, x>=50, derinlik ~2 m.
  5. RL corrected summary CSV ve span-check CSV okunabiliyor + tutarlı mı?
     (metrics UKF span ~0 iken raw telemetry span >> 0 olmalı = donmuş kolon kanıtı)
  6. Teşhis PNG'leri ve üretilen RL figürleri mevcut mu?
  7. HTML rapor okunabiliyor mu?
  8. Notebook geçerli JSON mu?
  9. Mimari CSV'leri tutarlı mı (bağlantı uçları düğüm listesinde var mı)?

Çıkış kodu 0 = tüm zorunlu kontroller geçti. Aksi halde 1.

Kullanım:
  python scripts/verify_validation_artifacts.py
"""
import csv
import json
import re
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

ROOT = Path(__file__).resolve().parents[1]

PASS, FAIL, WARN = "PASS", "FAIL", "WARN"
results = []  # (level, message)


def check(cond, ok_msg, fail_msg, level=FAIL):
    results.append((PASS if cond else level, ok_msg if cond else fail_msg))
    return cond


EXPECTED_EPISODE_COLS = [
    "t", "step", "x", "y", "z", "u", "yaw", "pitch", "ex", "ey", "ez", "eu",
    "throttle", "pitch_fin", "yaw_fin", "esc_pwm", "pitch_pwm", "yaw_pwm",
    "current_n", "current_e", "current_d", "current_mode", "energy_wh",
    "reward", "done", "truncated", "reward_progress", "reward_depth",
    "reward_cross", "reward_energy", "reward_fin", "reward_time",
    "reward_safety", "reward_terminal",
]


def import_pandas():
    try:
        import pandas as pd
        return pd
    except Exception:
        return None


def check_markdown_links():
    """README.md ve docs/wiki/*.md içindeki relative linkleri doğrula."""
    md_files = [ROOT / "README.md"] + sorted((ROOT / "docs" / "wiki").glob("*.md"))
    link_re = re.compile(r"\]\(([^)]+)\)")
    img_re = re.compile(r'<img\s+[^>]*src="([^"]+)"')
    broken = []
    checked = 0
    for md in md_files:
        if not md.exists():
            continue
        text = md.read_text(encoding="utf-8")
        targets = link_re.findall(text) + img_re.findall(text)
        for t in targets:
            t = t.split("#")[0].strip()
            if not t or t.startswith(("http://", "https://", "mailto:")):
                continue
            checked += 1
            if not (md.parent / t).resolve().exists():
                broken.append(f"{md.name} -> {t}")
    check(not broken, f"Markdown linkleri OK ({checked} hedef kontrol edildi)",
          f"Kırık link(ler): {broken}")


def check_episode_csv():
    pd = import_pandas()
    path = ROOT / "data" / "episodes" / "sara_best_episode.csv"
    if not check(path.exists(), "sara_best_episode.csv mevcut", "sara_best_episode.csv YOK"):
        return
    if pd is None:
        results.append((WARN, "pandas yok — episode metrik kontrolü atlandı"))
        return
    df = pd.read_csv(path)
    check(list(df.columns) == EXPECTED_EPISODE_COLS,
          "Episode kolonları beklenen şema ile birebir uyumlu",
          f"Episode kolonları farklı: {set(EXPECTED_EPISODE_COLS) ^ set(df.columns)}")
    last = df.iloc[-1]
    check(bool(last["done"]) is True, "Son satır done=True", f"done={last['done']} (True bekleniyordu)")
    check(bool(last["truncated"]) is False, "Son satır truncated=False", f"truncated={last['truncated']}")
    check(last["x"] >= 50.0, f"Final x={last['x']:.3f} >= 50 m", f"Final x={last['x']:.3f} < 50 m")
    check(abs(last["z"] - 2.0) < 0.2, f"Final derinlik z={last['z']:.3f} m (~2 m)",
          f"Final derinlik z={last['z']:.3f} hedeften uzak")
    check(abs(last["energy_wh"] - 7.27) < 0.2, f"Energy={last['energy_wh']:.3f} Wh (~7.27)",
          f"Energy={last['energy_wh']:.3f} beklenenden farklı", level=WARN)
    check(abs(df["reward"].sum() - 932.4) < 1.0,
          f"Toplam reward={df['reward'].sum():.2f} (~932.4)",
          f"Toplam reward={df['reward'].sum():.2f} beklenenden farklı", level=WARN)


def check_diagnosis():
    pd = import_pandas()
    diag = ROOT / "docs" / "diagnostics" / "rl_ukf"
    summary = diag / "corrected_rl_ukf_summary_from_raw_telemetry.csv"
    span = diag / "metrics_vs_raw_telemetry_ukf_span_check.csv"
    check(summary.exists(), "corrected RL summary CSV mevcut", "corrected RL summary CSV YOK")
    check(span.exists(), "UKF span-check CSV mevcut", "UKF span-check CSV YOK")
    for png in ["rl_ukf_raw_vs_aligned_rmse.png", "hard_cross_gt_ukf_alignment.png",
                "RL_UKF_GT_DIAGNOSIS.md",
                "recomputed_rl_ukf_from_telemetry_verification.csv",
                "rl_policy_validation_fixed.py",
                "legacy/rl_policy_validation_BUGGY.py",
                "legacy/legacy_rl_metrics_buggy_ukf_rmse.csv",
                "legacy/README.md"]:
        check((diag / png).exists(), f"diagnostics/{png} mevcut", f"diagnostics/{png} YOK")
    # düzeltme gerçekten uygulanmış mı?
    fixed = (diag / "rl_policy_validation_fixed.py")
    if fixed.exists():
        txt = fixed.read_text(encoding="utf-8")
        check('ukf["t"] -= start' in txt,
              "Düzeltilmiş exporter UKF zaman normalizasyonunu içeriyor",
              "Düzeltilmiş exporter'da beklenen fix satırı yok")
    if pd is None or not span.exists() or not summary.exists():
        return
    sp = pd.read_csv(span)
    frozen = (sp["metrics_x_ukf_span_m"].abs() < 1e-6).all()
    raw_moves = (sp["raw_telemetry_x_ukf_span_m"] > 10).all()
    check(frozen and raw_moves,
          "Donmuş-kolon kanıtı tutarlı: metrics UKF span≈0, raw telemetry span>>0",
          "Span-check beklenen donmuş-kolon desenini göstermiyor")
    su = pd.read_csv(summary)
    check(su["ukf_aligned_rmse_m"].between(0.0, 1.0).all(),
          f"Aligned UKF RMSE bandı OK: {su['ukf_aligned_rmse_m'].min():.2f}-{su['ukf_aligned_rmse_m'].max():.2f} m",
          "Aligned UKF RMSE beklenen 0-1 m bandında değil")


def check_rl_figures():
    figs = ROOT / "docs" / "figures" / "rl"
    for f in ["rl_episode_comparison_matrix.png", "rl_current_robustness.png",
              "rl_ukf_raw_vs_aligned_rmse.png", "rl_trajectory_overlay.png"]:
        check((figs / f).exists(), f"RL figürü {f} mevcut", f"RL figürü {f} YOK")


def check_html_report():
    html = ROOT / "reports" / "sara_mission_report.html"
    if not check(html.exists(), "sara_mission_report.html mevcut", "sara_mission_report.html YOK"):
        return
    txt = html.read_text(encoding="utf-8", errors="ignore")
    # local asset referansları (img src / href) repo içinde mi?
    assets = re.findall(r'(?:src|href)="([^"]+)"', txt)
    local = [a for a in assets if not a.startswith(("http", "#", "mailto"))]
    missing = [a for a in local if not (html.parent / a).exists()]
    check(not missing, f"HTML rapor local assetleri OK ({len(local)} ref)",
          f"HTML rapor eksik local asset: {missing}", level=WARN)


def check_notebook():
    nb = ROOT / "notebooks" / "sara_rl_validation.ipynb"
    if not check(nb.exists(), "sara_rl_validation.ipynb mevcut", "notebook YOK"):
        return
    try:
        data = json.loads(nb.read_text(encoding="utf-8"))
        check("cells" in data and isinstance(data["cells"], list),
              f"Notebook geçerli JSON ({len(data.get('cells', []))} hücre)",
              "Notebook JSON yapısı bozuk")
    except Exception as exc:
        check(False, "", f"Notebook JSON parse hatası: {exc}")


def check_architecture():
    arch = ROOT / "docs" / "architecture"
    nodes_csv = arch / "SARA_Sistem_Mimarisi.csv"
    conn_csv = arch / "SARA_Baglanti_Listesi.csv"
    for f in ["SARA_Sistem_Mimarisi_temiz.png", "SARA_Sistem_Mimarisi_temiz.pdf",
              "SARA_Sistem_Mimarisi_temiz.drawio", nodes_csv.name, conn_csv.name]:
        check((arch / f).exists(), f"architecture/{f} mevcut", f"architecture/{f} YOK")
    if not (nodes_csv.exists() and conn_csv.exists()):
        return
    nodes = set()
    for line in nodes_csv.read_text(encoding="utf-8").splitlines():
        if line.startswith("#") or line.startswith("id,") or not line.strip():
            continue
        nid = line.split(",")[0].strip()
        if nid:
            nodes.add(nid)
    missing = []
    with conn_csv.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            for end in (row.get("kaynak", ""), row.get("hedef", "")):
                if end and end not in nodes:
                    missing.append(end)
    check(not missing,
          f"Mimari CSV tutarlı: {len(nodes)} düğüm, tüm bağlantı uçları tanımlı",
          f"Mimari CSV tutarsız — düğüm listesinde olmayan uçlar: {sorted(set(missing))}")


def check_tracked_files_clean():
    """Git'e gitmemesi gereken ham/büyük/arşiv dosyaları staged/tracked mı?"""
    import subprocess
    try:
        out = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True,
                             text=True, encoding="utf-8")
    except Exception as exc:
        results.append((WARN, f"git ls-files çalıştırılamadı: {exc}"))
        return
    if out.returncode != 0:
        results.append((WARN, "git deposu değil ya da git yok — tracked kontrolü atlandı"))
        return
    tracked = [f for f in out.stdout.splitlines() if f.strip()]
    forbidden_ext = (".zip", ".rar", ".7z", ".tar", ".gz", ".bundle", ".db3",
                     ".bag", ".mcap", ".log", ".pyc", ".tmp", ".bak")
    bad = [f for f in tracked if f.lower().endswith(forbidden_ext)]
    bad += [f for f in tracked if "recording/" in f or f.startswith(("build/", "install/", "log/"))]
    bad += [f for f in tracked if f.lower().endswith(".mp4") and not f.startswith("reports/")]
    check(not bad, f"Yasaklı/ham dosya tracked DEĞİL ({len(tracked)} dosya izleniyor)",
          f"Repoya girmemesi gereken tracked dosyalar: {sorted(set(bad))}")
    # büyük dosya uyarısı (>5 MB)
    big = []
    for f in tracked:
        p = ROOT / f
        if p.exists() and p.stat().st_size > 5 * 1024 * 1024:
            big.append(f"{f} ({p.stat().st_size // (1024*1024)} MB)")
    if big:
        results.append((WARN, f"5 MB'tan büyük tracked dosya(lar): {big}"))
    else:
        results.append((PASS, "5 MB'tan büyük tracked dosya yok"))


def main():
    print("=" * 64)
    print("SARA Validation Suite — artefact doğrulama")
    print("=" * 64)
    check_markdown_links()
    check_episode_csv()
    check_diagnosis()
    check_rl_figures()
    check_html_report()
    check_notebook()
    check_architecture()
    check_tracked_files_clean()

    n_fail = sum(1 for lvl, _ in results if lvl == FAIL)
    n_warn = sum(1 for lvl, _ in results if lvl == WARN)
    for lvl, msg in results:
        print(f"  [{lvl}] {msg}")
    print("-" * 64)
    print(f"Toplam: {len(results)} kontrol | FAIL={n_fail} | WARN={n_warn}")
    if n_fail:
        print("SONUÇ: BAŞARISIZ — push öncesi düzeltilmeli.")
        sys.exit(1)
    print("SONUÇ: TÜM ZORUNLU KONTROLLER GEÇTİ.")
    sys.exit(0)


if __name__ == "__main__":
    main()
