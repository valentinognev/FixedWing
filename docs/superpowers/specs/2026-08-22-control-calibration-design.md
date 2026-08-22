# Control calibration (chirp SID) design

Date: 2026-08-22
Status: approved (layout, protocol, analysis/hints, tests)

## Goal

A host-side **workhorse** that (1) flies a straight-cruise overlay, (2) excites **one control channel at a time** with a log chirp then an inverse chirp, (3) logs command + simulator GT + PX4, (4) builds a Wiener **step response**, (5) prints **gain hints** an agent can apply to plant JSONC. The tool does **not** write plant files.

## Non-goals

- Auto-writing `platforms/*/…jsonc`
- Live SITL in the default unittest suite
- Chirping through `Px4FwAttCascade` on the rates/attitude layers (those layers hit PX4 inner loops directly)
- PIDToolBox as a runtime dependency
- Numeric Δ-gain values (direction + key only)

## Package

`python/controlCallibration/` (spelling locked by request). CLI:

```text
cd python && python3 -m controlCallibration run --layer rates|attitude|accel_z|vel_z [--inject pitch|thrust]
cd python && python3 -m controlCallibration analyze path/to/log.csv [--response gt|px4]
```

`--inject` required for `accel_z` / `vel_z`; ignored (one-line note) on rates/attitude.

Default plant/sim follows existing straight-flight flags (`--jsbsim` / `--yasim` / `--gz` / `--no-sim`). Hold uses **simulator GT** position/attitude (JSBSim FDM / FG telnet / Gazebo pose). PX4 ATTITUDE is logged always. Analysis defaults to GT.

## Modules

| File | Responsibility |
|------|----------------|
| `chirp.py` | Log-sine chirp + inverse samples; Welch `G(jω)` / coherence (PIDToolBox `estimate_freq_response`) |
| `stepresponse.py` | Wiener deconvolution → step stack + peak/latency stats (PIDToolBox `step_calc` / `step_stats`; **no** LOWESS / statsmodels) |
| `log_io.py` | CSV schema round-trip |
| `hints.py` | Verdict + JSON hints from stats (no JSONC writes) |
| `overlay.py` | Trim + one-axis command (rates, Euler, `az`/`w` via pitch or thrust) |
| `runner.py` | Engage, GT cruise, recapture sandwich, envelope abort, write CSV, optional analyze |
| `analyze.py` | Offline CSV → step PNG, Bode PNG, `hints.json` |
| `__main__.py` | `run` / `analyze` subcommands |

Reuse `fw_sitl` (MAVLink, plants, sim lifecycle, path hold helpers). Do **not** fold SID into `straight_flight_core`.

## Flight protocol

Between axes, **path-hold recapture** on GT cruise (OFFBOARD rate/attitude cannot also run TECS). Per axis:

1. Path hold until envelope quiet
2. Settle 3 s at trim
3. Log chirp `f0→f1` 20 s
4. Settle 2 s
5. Inverse chirp `f1→f0` 20 s
6. Settle 2 s
7. Path hold recapture

Waveform: `cmd = trim + A·sin(φ(t))` with logarithmic instantaneous frequency.

| Layer | Axes (sequenced) | A | f0…f1 | Send path |
|-------|------------------|---|-------|-----------|
| `rates` | `p,q,r` | 0.15 rad/s | 0.3–8 Hz | `send_attitude_rates`; other rates 0; thrust cruise |
| `attitude` | `roll,pitch,yaw` | 5° / 5° / 8° | 0.2–4 Hz | `send_attitude_target`; other Euler at GT trim |
| `accel_z` + `pitch` | `az` command | 1 m/s² | 0.2–3 Hz | pitch overlay; thrust cruise |
| `accel_z` + `thrust` | `az` **response** | Δthrust 0.08 | 0.2–3 Hz | `cmd` = thrust; attitude trim |
| `vel_z` + `pitch` | `w` command | 1 m/s | 0.1–2 Hz | pitch overlay |
| `vel_z` + `thrust` | `w` **response** | Δthrust 0.08 | 0.1–2 Hz | `cmd` = thrust; attitude trim |

Pitch map (v1): `Δθ = -az_cmd / g` (g=9.81); `Δθ = -w_cmd * 0.05` rad per (m/s) (~2.9° per m/s). Thrust map: `thrust = clip(cruise + chirp, min_thrust, max_thrust)`.

Control rate **50 Hz**. CSV one file per launch.

Abort if `|roll|>40°`, `|pitch|>25°`, `|Δalt|>30 m`, or airspeed below plant `fw_airspd_min`. Recapture, flush CSV, analyze finished channels, `hints.json` `aborted: true`.

## CSV columns

`t, channel, segment, cmd, gt, px4, thrust, roll_gt, pitch_gt, yaw_gt, p_gt, q_gt, r_gt, roll_px4, pitch_px4, yaw_px4, p_px4, q_px4, r_px4`

`segment` ∈ `hold|settle|chirp|inv_chirp`. `channel` is the axis under test (`p`/`q`/`r`/`roll`/`pitch`/`yaw`/`az`/`w`).

## Analysis

- Default `--response gt`. Use rows with `segment ∈ {chirp, inv_chirp}` for that `channel`. Concatenate forward + inverse.
- Wiener method as PIDToolBox, but:
  - `min_input` = 20% of that channel’s chirp amplitude (do **not** use the 0.25–20 Betaflight floor)
  - Step window: 0.5 s rates, 1.0 s attitude, 2.0 s body-Z
  - No LOWESS
- Bode optional from same pair (`estimate_freq_response`); hints from **step** only.
- Artifacts next to CSV: `*_step.png`, `*_bode.png`, `hints.json`

### hints.json

```json
{
  "layer": "rates",
  "inject": null,
  "response": "gt",
  "aborted": false,
  "channels": {
    "p": {
      "peak_mean": 1.32,
      "latency_mean_ms": 90,
      "n": 8,
      "verdict": "overshoot",
      "hints": [
        {"key": "px4_inner.FW_RR_P", "direction": "down", "reason": "peak 1.32 > 1.25"}
      ]
    }
  }
}
```

Verdicts: `ok` | `overshoot` (peak > 1.25) | `weak` (peak < 0.85) | `slow` (latency above budget) | `no_data` (`n=0`).

Latency budgets: rates 150 ms, attitude 300 ms, body-Z 800 ms.

Key map:

| Channel / inject | Keys |
|------------------|------|
| `p` | `px4_inner.FW_RR_P`, `px4_inner.FW_RR_FF`, `px4_inner.FW_R_TC` |
| `q` | `px4_inner.FW_PR_P`, `px4_inner.FW_PR_FF` |
| `r` | `px4_inner.FW_YR_P` |
| `roll` | `controllers.*.roll_tc`, `controllers.*.pid_kp`, `px4_inner.FW_R_TC` |
| `pitch` | `controllers.*.pitch_tc`, `controllers.*.pid_kp`, `controllers.*.bank_kp_alt` |
| `yaw` | `controllers.*.bank_kp_heading` |
| `az`/`w` + pitch | `controllers.*.bank_kp_alt`, `controllers.*.pitch_tc`, `controllers.*.att_max_pitch_rad` |
| `az`/`w` + thrust | `controllers.*.climb_thrust_per_m`, `controllers.*.cruise_thrust`, `controllers.*.min_thrust`, `controllers.*.speed_thrust_per_mps` |

Overshoot → `down` on P/FF. Weak/slow → `up` on P or `down` on `*_tc`. At most two hints per channel. `controllers.*` means emit the suffix (`roll_tc`, not every controller id).

## Errors

| Case | Behavior |
|------|----------|
| `--inject` missing on Z layers | Exit 2 |
| MAVLink / arm / sim fail | Stop sim, exit 1, no fake hints |
| Analyze missing column | Exit 2 |
| `n=0` | `verdict: no_data`, no keys |

## Tests

Host unittest only. Live SITL opt-in is out of v1. No PIDToolBox on `PYTHONPATH`.
