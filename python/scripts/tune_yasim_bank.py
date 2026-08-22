#!/usr/bin/env python3
"""Iterative YASim bank-damping tuner (live SITL). Writes yasim_rascal.jsonc, races, scores."""

from __future__ import annotations

import csv
import json
import math
import pickle
import re
import subprocess
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_PYTHON = Path.home() / "anaconda/envs/pigeon/bin/python3"
_PLANT = _ROOT / "python/fw_sitl/plants/yasim_rascal.jsonc"
_RACE = _ROOT / "python/scripts/run_balloon_race.sh"
_KILL = _ROOT / "python/scripts/kill.sh"
_LOG = Path("/tmp/yasim_bank_tune_log.jsonl")

# Keys applied to both race_quat and pure_pursuit_quat (plus px4_inner).
OUTER_KEYS = (
    "pid_kp",
    "pid_ki",
    "pid_kd",
    "bank_kp_heading",
    "bank_max_roll_rad",
    "bank_kp_alt",
    "visual_lock_kp_alt",
    "climb_thrust_per_m",
    "approach_speed_mps",
)
INNER_KEYS = ("FW_R_TC", "FW_RR_P", "FW_RR_I", "FW_RR_FF")


def _strip_jsonc(text: str) -> str:
    """Minimal JSONC strip (reuse plant loader if available)."""
    import sys

    sys.path.insert(0, str(_ROOT / "python"))
    from fw_sitl.plant_loader import strip_jsonc

    return strip_jsonc(text)


def load_plant() -> dict:
    return json.loads(_strip_jsonc(_PLANT.read_text(encoding="utf-8")))


def write_plant(data: dict) -> None:
    """Rewrite plant JSONC with short comments preserved on key fields."""
    # Keep a compact JSONC the loader accepts; comments optional.
    text = json.dumps(data, indent=2)
    # Restore // style is optional; plain JSON is valid JSONC.
    _PLANT.write_text(text + "\n", encoding="utf-8")


def apply_gains(gains: dict) -> None:
    data = load_plant()
    inner = {k: v for k, v in data.get("px4_inner", [])}
    for k in INNER_KEYS:
        if k in gains:
            inner[k] = float(gains[k])
    data["px4_inner"] = [[k, inner[k]] for k in inner]
    for cname in ("race_quat", "pure_pursuit_quat"):
        block = data["controllers"][cname]
        for k in OUTER_KEYS:
            if k in gains:
                block[k] = float(gains[k])
    write_plant(data)


def current_gains() -> dict:
    data = load_plant()
    inner = dict(data["px4_inner"])
    rq = data["controllers"]["race_quat"]
    out = {k: float(rq[k]) for k in OUTER_KEYS if k in rq}
    out.update({k: float(inner[k]) for k in INNER_KEYS if k in inner})
    return out


def kill_race() -> None:
    subprocess.run(["bash", str(_KILL), "--all"], cwd=_ROOT, capture_output=True)
    subprocess.run(
        ["tmux", "kill-session", "-t", "balloon_race"],
        capture_output=True,
    )


def run_race(duration_s: float = 60.0) -> Path | None:
    kill_race()
    time.sleep(2.0)
    t0 = time.time()
    before = {p.resolve() for p in Path("/tmp").glob("balloon_race_*.csv")}
    cmd = [
        "bash",
        str(_RACE),
        "--yasim",
        "--duration",
        str(duration_s),
        "--detach",
        "--no-plot",
    ]
    env = dict(**{k: v for k, v in __import__("os").environ.items()})
    proc = subprocess.run(cmd, cwd=_ROOT, capture_output=True, text=True, env=env)
    if proc.returncode != 0:
        print(proc.stdout[-800:] if proc.stdout else "")
        print(proc.stderr[-800:] if proc.stderr else "")
        return None
    # Wait only for a NEW csv written after this launch (never reuse prior runs).
    deadline = time.time() + duration_s + 120.0
    seen: set[Path] = set()
    while time.time() < deadline:
        after = sorted(Path("/tmp").glob("balloon_race_*.csv"), key=lambda p: p.stat().st_mtime)
        cands = []
        for p in after:
            try:
                rp = p.resolve()
                mt = p.stat().st_mtime
            except OSError:
                continue
            if rp in before or mt < t0 - 0.5:
                continue
            cands.append(p)
        if cands:
            newest = cands[-1]
            if newest not in seen:
                seen.add(newest)
                print(f"  waiting on {newest.name} …", flush=True)
            try:
                text = newest.read_text(encoding="utf-8")
            except OSError:
                text = ""
            if ",end_" in text:
                pkl = newest.with_suffix(".pkl")
                for _ in range(45):
                    if pkl.is_file() and pkl.stat().st_size > 1000:
                        print(f"  got {newest.name}", flush=True)
                        return newest
                    time.sleep(1.0)
                print(f"  got {newest.name} (no/late pkl)", flush=True)
                return newest
        time.sleep(3.0)
    print("  race timed out waiting for new CSV", flush=True)
    return None


def analyze(csv_path: Path, gains: dict | None = None) -> dict:
    pkl = csv_path.with_suffix(".pkl")
    h = pickle.loads(pkl.read_bytes())
    t = __import__("numpy").asarray(h.t, float)
    import numpy as np

    g = gains if gains is not None else current_gains()
    m = t > 5.0
    rc = np.asarray(h.roll_cmd_deg, float)[m]
    rm = np.asarray(h.roll_deg, float)[m]
    tt = t[m]
    dt = np.diff(tt)
    drc = np.diff(rc)
    drm = np.diff(rm)
    rate_c = np.abs(drc) / np.maximum(dt, 1e-3)
    rate_m = np.abs(drm) / np.maximum(dt, 1e-3)
    rate_m_f = rate_m[rate_m < 120.0]
    sat_lim = math.degrees(abs(float(g.get("bank_max_roll_rad", 0.38)))) - 0.5
    sat = float(np.mean(np.abs(rc) > sat_lim)) if len(rc) else 1.0
    corr = (
        float(np.corrcoef(rc, rm)[0, 1])
        if len(rc) > 10 and np.std(rc) > 1e-6 and np.std(rm) > 1e-6
        else 0.0
    )
    zc = int(np.sum((rm[:-1] * rm[1:]) < 0)) if len(rm) > 2 else 0
    dur = float(tt[-1] - tt[0]) if len(tt) > 1 else 1.0

    passes = []
    with csv_path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if row["event"] != "pass":
                continue
            dn = float(row["pos_n"]) - float(row["tgt_n"])
            de = float(row["pos_e"]) - float(row["tgt_e"])
            dd = float(row["pos_d"]) - float(row["tgt_d"])
            passes.append(
                {
                    "miss3d": math.sqrt(dn * dn + de * de + dd * dd),
                    "xy": math.hypot(dn, de),
                    "dD": dd,
                }
            )
    miss3d = (
        float(sum(p["miss3d"] for p in passes) / len(passes)) if passes else 999.0
    )
    abs_dD = (
        float(sum(abs(p["dD"]) for p in passes) / len(passes)) if passes else 999.0
    )
    xy = float(sum(p["xy"] for p in passes) / len(passes)) if passes else 999.0

    # Closest approach (any time) for accuracy without requiring a pass row.
    x = np.asarray(h.x, float)
    y = np.asarray(h.y, float)
    z = np.asarray(h.z, float)
    tx = np.asarray(h.tgt_x, float)
    ty = np.asarray(h.tgt_y, float)
    tz = np.asarray(h.tgt_z, float)
    d3 = np.sqrt((x - tx) ** 2 + (y - ty) ** 2 + (z - tz) ** 2)
    dxy = np.hypot(x - tx, y - ty)
    i_xy = int(np.argmin(dxy))
    closest_xy = float(dxy[i_xy])
    closest_dD = float(z[i_xy] - tz[i_xy])
    closest_3d = float(d3[i_xy])

    shake = (
        40.0 * sat
        + 0.35 * float(np.percentile(rate_c, 90) if len(rate_c) else 50.0)
        + 0.25 * float(np.std(rm) if len(rm) else 50.0)
        + 8.0 * (zc / max(dur, 1.0))
        - 15.0 * max(corr, -0.5)
    )
    # Prefer small closest approach; pass miss if available.
    miss = 0.6 * closest_3d + 0.4 * (miss3d if passes else closest_3d)
    miss += 0.35 * abs(closest_dD)
    if passes:
        miss += 0.25 * abs_dD
    else:
        miss += 40.0  # no-pass penalty

    return {
        "csv": str(csv_path),
        "n_pass": len(passes),
        "sat": sat,
        "corr": corr,
        "cmd_rate_p90": float(np.percentile(rate_c, 90) if len(rate_c) else 99.0),
        "meas_std": float(np.std(rm) if len(rm) else 99.0),
        "meas_rate_p90": float(np.percentile(rate_m_f, 90) if len(rate_m_f) else 99.0),
        "zc_hz": zc / max(dur, 1.0),
        "miss3d_pass": miss3d,
        "abs_dD_pass": abs_dD,
        "xy_pass": xy,
        "closest_xy": closest_xy,
        "closest_dD": closest_dD,
        "closest_3d": closest_3d,
        "shake": shake,
        "miss": miss,
        "score": shake + 0.85 * miss,
    }


def main() -> int:
    # 10 improvement cycles: each mutates relative to the best-so-far.
    # Round-2 schedule: keep altitude authority while damping bank (from round-1 learnings).
    schedule = [
        {
            "note": "c1 alt+soft bank",
            "bank_kp_alt": 0.032,
            "visual_lock_kp_alt": 0.032,
            "climb_thrust_per_m": 0.028,
            "bank_kp_heading": 0.78,
            "bank_max_roll_rad": 0.36,
        },
        {
            "note": "c2 slower inner",
            "FW_R_TC": 0.80,
            "FW_RR_P": 0.095,
            "FW_RR_I": 0.10,
        },
        {
            "note": "c3 more D",
            "pid_kp": 0.42,
            "pid_kd": 0.12,
            "pid_ki": 0.07,
        },
        {"note": "c4 lower max roll", "bank_max_roll_rad": 0.34},
        {
            "note": "c5 bank 0.72",
            "bank_kp_heading": 0.72,
            "FW_R_TC": 0.82,
        },
        {
            "note": "c6 alt nudge",
            "bank_kp_alt": 0.034,
            "visual_lock_kp_alt": 0.034,
            "climb_thrust_per_m": 0.030,
            "approach_speed_mps": 23.0,
        },
        {
            "note": "c7 damp blend",
            "bank_kp_heading": 0.70,
            "bank_max_roll_rad": 0.34,
            "FW_RR_P": 0.09,
            "FW_R_TC": 0.85,
            "pid_kd": 0.13,
        },
        {
            "note": "c8 mid bank keep alt",
            "bank_kp_heading": 0.75,
            "bank_max_roll_rad": 0.35,
            "bank_kp_alt": 0.036,
            "visual_lock_kp_alt": 0.036,
            "climb_thrust_per_m": 0.030,
        },
        {
            "note": "c9 approach 24",
            "approach_speed_mps": 24.0,
            "climb_thrust_per_m": 0.032,
            "bank_kp_heading": 0.74,
        },
        {
            "note": "c10 polish",
            "bank_kp_heading": 0.73,
            "bank_max_roll_rad": 0.34,
            "pid_kp": 0.43,
            "pid_kd": 0.12,
            "FW_RR_P": 0.095,
            "FW_R_TC": 0.80,
            "bank_kp_alt": 0.034,
            "visual_lock_kp_alt": 0.034,
            "climb_thrust_per_m": 0.029,
            "approach_speed_mps": 23.0,
        },
    ]

    if _LOG.exists():
        _LOG.unlink()

    print("=== baseline ===", flush=True)
    base_g = current_gains()
    apply_gains(base_g)  # ensure plant matches measured baseline
    csv0 = run_race(60.0)
    if csv0 is None:
        print("baseline race failed")
        return 1
    base = analyze(csv0, base_g)
    base["gains"] = base_g
    base["cycle"] = 0
    base["note"] = "baseline"
    base["kept"] = True
    with _LOG.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(base) + "\n")
    print(
        f"baseline score={base['score']:.1f} shake={base['shake']:.1f} miss={base['miss']:.1f} "
        f"sat={100*base['sat']:.0f}% n_pass={base['n_pass']} closest3d={base['closest_3d']:.1f} "
        f"|dD|={abs(base['closest_dD']):.1f} csv={Path(base['csv']).name}",
        flush=True,
    )

    best = dict(base)
    best_gains = dict(base_g)
    # Miss budget stays anchored to baseline (do not tighten after a lucky miss win).
    miss3d_budget = float(base["closest_3d"]) + 8.0
    dD_budget = abs(float(base["closest_dD"])) + 8.0
    base_shake = float(base["shake"])

    for i, step in enumerate(schedule, start=1):
        note = step.pop("note")
        cand = dict(best_gains)
        cand.update(step)
        print(f"\n=== cycle {i}/10: {note} ===", flush=True)
        apply_gains(cand)
        csv_path = run_race(60.0)
        if csv_path is None:
            print("race failed; skip", flush=True)
            apply_gains(best_gains)
            continue
        m = analyze(csv_path, cand)
        m["gains"] = cand
        m["cycle"] = i
        m["note"] = note
        miss_ok = m["closest_3d"] <= miss3d_budget and abs(m["closest_dD"]) <= dD_budget
        pass_ok = m["n_pass"] >= min(1, int(base.get("n_pass", 0))) or m["closest_xy"] < 25.0
        # Primary goal: lower bank shake. Never accept shake worse than baseline+2.
        shake_better = m["shake"] < best["shake"] - 0.5
        shake_not_regress = m["shake"] <= base_shake + 2.0
        # Secondary: improve miss while not raising shake above baseline.
        miss_better = m["miss"] < best["miss"] - 2.0 and m["shake"] <= base_shake + 0.5
        kept = bool(miss_ok and pass_ok and shake_not_regress and (shake_better or miss_better))
        m["kept"] = kept
        with _LOG.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(m) + "\n")
        print(
            f"  score={m['score']:.1f} shake={m['shake']:.1f} miss={m['miss']:.1f} "
            f"sat={100*m['sat']:.0f}% cmd_p90={m['cmd_rate_p90']:.1f} "
            f"n_pass={m['n_pass']} closest3d={m['closest_3d']:.1f} |dD|={abs(m['closest_dD']):.1f} "
            f"csv={Path(m['csv']).name} kept={kept}",
            flush=True,
        )
        if kept:
            best = m
            best_gains = dict(cand)
            print(f"  NEW BEST shake={best['shake']:.1f} score={best['score']:.1f}", flush=True)
        else:
            apply_gains(best_gains)

    apply_gains(best_gains)
    kill_race()
    print("\n=== BEST ===")
    print(json.dumps({"score": best["score"], "gains": best_gains, "metrics": {
        k: best[k] for k in (
            "sat", "corr", "cmd_rate_p90", "meas_std", "n_pass",
            "closest_3d", "closest_dD", "closest_xy", "shake", "miss", "csv",
        )
    }}, indent=2))
    Path("/tmp/yasim_bank_tune_best.json").write_text(
        json.dumps({"gains": best_gains, "best": best}, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
