"""Map step-response stats to plant gain hints."""

from __future__ import annotations

LATENCY_BUDGET_MS = {
    "p": 150,
    "q": 150,
    "r": 150,
    "roll": 300,
    "pitch": 300,
    "yaw": 300,
    "az": 800,
    "w": 800,
}

KEYS: dict[tuple[str, str | None], tuple[str, ...]] = {
    ("p", None): (
        "px4_inner.FW_RR_P",
        "px4_inner.FW_RR_FF",
        "px4_inner.FW_R_TC",
    ),
    ("q", None): (
        "px4_inner.FW_PR_P",
        "px4_inner.FW_PR_FF",
    ),
    ("r", None): ("px4_inner.FW_YR_P",),
    ("roll", None): (
        "roll_tc",
        "pid_kp",
        "px4_inner.FW_R_TC",
    ),
    ("pitch", None): (
        "pitch_tc",
        "pid_kp",
        "bank_kp_alt",
    ),
    ("yaw", None): ("bank_kp_heading",),
    ("az", "pitch"): (
        "bank_kp_alt",
        "pitch_tc",
        "att_max_pitch_rad",
    ),
    ("w", "pitch"): (
        "bank_kp_alt",
        "pitch_tc",
        "att_max_pitch_rad",
    ),
    ("az", "thrust"): (
        "climb_thrust_per_m",
        "cruise_thrust",
        "min_thrust",
        "speed_thrust_per_mps",
    ),
    ("w", "thrust"): (
        "climb_thrust_per_m",
        "cruise_thrust",
        "min_thrust",
        "speed_thrust_per_mps",
    ),
}

_MAX_HINTS = 2


def _is_p_or_ff(key: str) -> bool:
    return "_P" in key or "_FF" in key or key == "pid_kp"


def verdict(stats: dict, channel: str) -> str:
    n = stats.get("n", 0)
    if n == 0:
        return "no_data"
    peak = float(stats.get("peak_mean", 0.0))
    if peak > 1.25:
        return "overshoot"
    if peak < 0.85:
        return "weak"
    budget = LATENCY_BUDGET_MS.get(channel)
    latency = float(stats.get("latency_mean_ms", 0.0))
    if budget is not None and latency > budget:
        return "slow"
    return "ok"


def _hint_items(channel: str, inject: str | None, stats: dict, kind: str) -> list[dict]:
    keys = KEYS.get((channel, inject), ())
    if not keys:
        return []
    peak = stats.get("peak_mean")
    if kind == "overshoot":
        reason = f"peak {peak} > 1.25"
        selected = [k for k in keys if _is_p_or_ff(k)][:_MAX_HINTS]
        if not selected:
            # yaw and both body-Z injects have no P/FF key at all; fall back
            # to the first entry with the sense inverted from weak/slow so
            # the loudest verdict still names something to change.
            selected = [keys[0]]
        return [
            {
                "key": k,
                "direction": "up" if "_tc" in k else "down",
                "reason": reason,
            }
            for k in selected
        ]
    # weak / slow: first KEYS entry
    key = keys[0]
    direction = "down" if "_tc" in key else "up"
    if kind == "weak":
        reason = f"peak {peak} < 0.85"
    else:
        reason = f"latency {stats.get('latency_mean_ms')} ms above budget"
    return [{"key": key, "direction": direction, "reason": reason}]


def hints_for_channel(channel: str, inject: str | None, stats: dict) -> dict:
    kind = verdict(stats, channel)
    if kind in ("no_data", "ok"):
        hints: list[dict] = []
    else:
        hints = _hint_items(channel, inject, stats, kind)
    return {
        "peak_mean": stats.get("peak_mean"),
        "peak_std": stats.get("peak_std", 0),
        "latency_mean_ms": stats.get("latency_mean_ms"),
        "latency_std_ms": stats.get("latency_std_ms", 0),
        "n": stats.get("n"),
        "verdict": kind,
        "hints": hints,
    }


def build_report(
    *,
    layer: str,
    inject: str | None,
    response: str,
    aborted: bool,
    channel_stats: dict[str, dict],
) -> dict:
    channels = {
        name: hints_for_channel(name, inject, st) for name, st in channel_stats.items()
    }
    return {
        "layer": layer,
        "inject": inject,
        "response": response,
        "aborted": aborted,
        "channels": channels,
    }
