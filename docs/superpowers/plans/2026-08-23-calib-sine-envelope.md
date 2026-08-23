# Calibration sine waveform + envelope retry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Live GZ SID no longer dies on the first envelope trip; `--waveform sine` is an alternative to chirp (not a phase after it), with ≥60 s of sine per axis.

**Architecture:** Keep `python -m controlCallibration run|analyze` and `./run_control_calibration.sh`. Chirp vs sine is a CLI `--waveform` switch that selects `procedure.json` phase lists. Envelope abort prints the tripping limit, recaptures path-hold, and retries that axis (cap 3) instead of ending the run.

**Tech Stack:** Python 3, numpy, unittest.

**Spec:** `docs/superpowers/specs/2026-08-22-control-calibration-design.md` plus chat-approved 2026-08-23 design (sine is alternative scenario; envelope recapture-continue). Spec remains binding for CSV columns, hints keys, no JSONC writes, package spelling. Where this plan and that spec disagree on abort-ends-run or chirp-only phases, this plan wins.

## Global Constraints

- Package directory name is exactly `python/controlCallibration` (user spelling).
- No `pidbox` / PIDToolBox import; no statsmodels / LOWESS.
- Host tests: `cd python && MPLBACKEND=Agg python3 -m unittest tests.test_calibration_* tests.test_flight_history` — never `unittest discover` (race plots `plt.show(block=True)` hang).
- Do not modify `fw_sitl/straight_flight_core.py`.
- Do not write plant JSONC files.
- Tests live in `python/tests/test_calibration_*.py` with `_PYTHON_ROOT` sys.path insert.
- TDD: failing test first, then implementation. Watch RED before GREEN.
- Work from the `calib-sine-envelope` worktree. Commit after each task.
- `python3 -m unittest` (not pytest).
- Interactive `plt.show` must never run inside unittest (`--no-plot` / `show=False`).
- One `--layer` per invocation.
- Do not bump rates/attitude amplitudes (leave shipped A).
- Do not dispatch subagents from implementers or reviewers.
- Never use Composer 2.5 Fast.

## File map

| Path | Role |
|------|------|
| `python/controlCallibration/procedure.json` | Chirp `phases` unchanged; add `sine_phases`; per-layer `f_sine_hz` |
| `python/controlCallibration/procedure.py` | Load `sine_phases` + `LayerSpec.f_sine_hz` |
| `python/controlCallibration/chirp.py` | Add `tone(t, f_hz, amplitude)` |
| `python/controlCallibration/runner.py` | `--waveform`; `iter_schedule`/`chirp_value`; envelope reason + axis retry |
| `python/controlCallibration/log_io.py` | `sine` segment; excitation = sine XOR chirp+inv_chirp |
| `python/tests/test_calibration_*.py` | Coverage for the above |
| `README.md` / `UPDATES.md` | 0.50.0 |

## Rulings (bind implementers)

- `--waveform {chirp,sine}` default `chirp`. Sine is **instead of** chirp, not after it.
- Shipped `sine_phases`: settle 3.0, sine 60.0, settle 2.0. One axis excitation is 60 s (≥ 1 min).
- Shipped `f_sine_hz`: rates 0.5, attitude 0.3, accel_z 0.2, vel_z 0.2.
- `cmd = A * sin(2π f_sine t)` during `sine` (same overlay `axis_command` as chirp).
- `select_excitation` / `response_series`: if the channel has any `segment==sine` rows, use **only** those; else use `chirp`+`inv_chirp`. Never concatenate sine with chirp.
- Envelope abort: print which limit tripped with values; recapture path-hold (`_hold_ticks(3.0)` then `hold_until_quiet`); retry **that axis from its first phase**. Max **3** attempts per axis then skip to the next axis with a warning. `hints.json` `aborted: true` iff any axis was skipped after exhausting retries. A recovered retry → that axis is not aborted.
- `envelope_fail_reason` checks in the same order as `envelope_ok`: roll, pitch, Δalt, airspeed.

---

### Task 1: Sine as alternative waveform

**Files:**
- Modify: `python/controlCallibration/procedure.json`
- Modify: `python/controlCallibration/procedure.py`
- Modify: `python/controlCallibration/chirp.py`
- Modify: `python/controlCallibration/log_io.py`
- Modify: `python/controlCallibration/runner.py` (`parse_run_args`, `iter_schedule`, `chirp_value`, `run_offline_demo`; **not** the live envelope abort path — that is Task 2)
- Modify: `python/tests/test_calibration_procedure.py`
- Modify: `python/tests/test_calibration_chirp.py`
- Modify: `python/tests/test_calibration_log.py`
- Modify: `python/tests/test_calibration_runner.py`
- Modify: `python/tests/test_calibration_cli.py` if it snapshots `--help` / argv

**Interfaces:**
- `LayerSpec` gains `f_sine_hz: float`.
- `Procedure` gains `sine_phases: tuple[tuple[str, float], ...]`. Keep `phases` as the chirp list (shipped values unchanged so existing tests stay green).
- `_KNOWN_SEGMENTS` includes `sine`. `sine_phases` items must use `settle` or `sine` only (`chirp`/`inv_chirp` in `sine_phases` → `ValueError`).
- `load_procedure` requires every layer to have `f_sine_hz`.
- `chirp.tone(t, f_hz, amplitude) -> np.ndarray` = `amplitude * sin(2π f_hz t)`.
- `iter_schedule(layer: str, waveform: str = "chirp")` — `"chirp"` → today’s 15 tuples for rates; `"sine"` → 9 tuples `(ch, settle, 3), (ch, sine, 60), (ch, settle, 2)` per p,q,r; unknown waveform → `ValueError`.
- `chirp_value(..., f_sine: float = 0.0)` — if `phase == "sine"`: `float(tone(np.asarray([t_in_phase]), f_sine, amplitude)[0])`. Settle/hold still 0. Chirp/inv_chirp unchanged.
- `parse_run_args`: `--waveform` choices `chirp`/`sine`, default `chirp`.
- `run_offline_demo` uses `iter_schedule(layer, args.waveform)` and passes `f_sine=layer_sine_freq(layer)` into `chirp_value`. Add `layer_sine_freq(layer) -> float` reading `LayerSpec.f_sine_hz`.
- `log_io.SEGMENTS` includes `"sine"`. `_excitation_rows` as in Rulings.
- `runner.SINE_PHASES` module-level tuple loaded from procedure (so tests can patch it like `PHASES`).
- Do **not** change `run_sitl` envelope abort behavior in this task. Do wire `run_sitl` to honor `--waveform` the same way as dry-run (phases + `f_sine` in `chirp_value`) so Task 2 does not have to rediscover it. If `run_sitl` tests patch `PHASES` to short chirp, they stay valid for default waveform.

- [ ] **Step 1: Write the failing tests** (procedure shipped `sine_phases` / `f_sine_hz`; `tone` at t=0.25, f=0.5, A=0.15 equals `0.15*sin(2*pi*0.5*0.25)`; `iter_schedule("rates","sine")` length 9; `parse_run_args(["--layer","rates","--waveform","sine"]).waveform == "sine"`; `select_excitation` with only sine rows returns those, and mixed sine+chirp on one channel still returns **only sine**; dry-run `--waveform sine --no-plot` CSV segments are settle/sine/settle only).

- [ ] **Step 2: Run them and confirm RED** (`MPLBACKEND=Agg python3 -m unittest tests.test_calibration_procedure tests.test_calibration_chirp tests.test_calibration_log tests.test_calibration_runner tests.test_calibration_cli`). Failures must be missing feature, not import typos.

- [ ] **Step 3: Minimal implementation** to GREEN.

- [ ] **Step 4: Re-run focused tests plus full calibration suite.** All green.

- [ ] **Step 5: Commit**

```bash
git add python/controlCallibration/procedure.json python/controlCallibration/procedure.py python/controlCallibration/chirp.py python/controlCallibration/log_io.py python/controlCallibration/runner.py python/tests/test_calibration_procedure.py python/tests/test_calibration_chirp.py python/tests/test_calibration_log.py python/tests/test_calibration_runner.py python/tests/test_calibration_cli.py
git commit -m "$(cat <<'EOF'
feat: add --waveform sine as an alternative to chirp SID

Constant-frequency tone is a separate scenario (procedure sine_phases, 60 s per axis), not a phase after the log chirp.
EOF
)"
```

---

### Task 2: Envelope recapture-continue

**Files:**
- Modify: `python/controlCallibration/runner.py` (`envelope_ok` companion, `run_sitl` axis loop only)
- Modify: `python/tests/test_calibration_runner.py` (`envelope_fail_reason`)
- Modify: `python/tests/test_calibration_run_sitl.py` (replace abort-ends-run contract)

**Interfaces:**
- `envelope_fail_reason(roll, pitch, alt, alt0, airspeed, airspd_min) -> str | None` — `None` if `envelope_ok`; else a single-line reason with numbers, first failing check in `envelope_ok` order. Example: `pitch=31.2° > 25° (roll=1.0° Δalt=0.4m airspeed=16.2)`. Exact wording must include the limit name (`roll`/`pitch`/`Δalt` or `dalt`/`airspeed`) and the measured value. Tests should `assertIn` the axis name and the measured number, not require a golden string.
- `MAX_AXIS_RETRIES = 3`.
- `run_sitl` on envelope fail during overlay: print `Envelope abort during {channel}: {reason} — recapturing` to stderr; `_hold_ticks(3.0)`; `hold_until_quiet`; restart that channel’s phase loop from the beginning (do not keep rows from the failed attempt **or** do keep them — **Ruling: drop rows for that channel from the failed attempt** so Wiener is not poisoned by a truncated chirp; keep other channels’ finished rows). After 3 failed attempts, print a skip warning, set `aborted=True`, move to the next channel.
- Successful retry: `aborted` stays False unless another channel was skipped.
- Update `TestRunSitlEnvelopeAbort.test_out_of_envelope_sample_flushes_csv_and_marks_aborted`: trip envelope **once** (e.g. 15th send), then restore in-envelope attitude. Expect rc 0, CSV contains **all three** attitude channels (roll retried then pitch/yaw), `aborted` is **False**, and stderr/log includes the reason. Path-hold recapture still happens (`path_setpoints > 0`).
- New test: envelope stays out on every overlay tick for `roll` → after 3 attempts skip roll, still fly pitch/yaw; `aborted` True; `hints.json` aborted True; CSV channels are pitch and yaw only (no successful roll excitation, or only truncated rows that were dropped — **no roll chirp/inv_chirp rows**).

- [ ] **Step 1: Write the failing tests.**

- [ ] **Step 2: Confirm RED.**

- [ ] **Step 3: Minimal implementation.**

- [ ] **Step 4: Full calibration suite green.**

- [ ] **Step 5: Commit**

```bash
git add python/controlCallibration/runner.py python/tests/test_calibration_runner.py python/tests/test_calibration_run_sitl.py
git commit -m "$(cat <<'EOF'
fix: recapture and retry a SID axis after envelope abort

Print which limit tripped and retry the axis up to 3 times instead of ending the whole calibration run.
EOF
)"
```

---

### Task 3: Docs

**Files:**
- Modify: `README.md` (controlCallibration row, Run examples, Known limits)
- Modify: `UPDATES.md` (new top `## 0.50.0 - SID sine waveform and envelope retry`)

**Content:**
- README Run: add `./run_control_calibration.sh --layer rates --gz --waveform sine` and `--layer attitude --gz --waveform sine`. Note sine is instead of chirp; 60 s sine per axis from `procedure.json`.
- Known limits: envelope abort recaptures and retries (3×) rather than ending the run; still print the tripping limit. Rates layer commands body rates (use `--layer attitude` for visible Euler wobble). Live az/w still nan.
- UPDATES 0.50.0 bullets: `--waveform sine`; `sine_phases` 3/60/2; per-layer `f_sine_hz`; envelope recapture-continue + reason print; max 3 retries then skip axis; `aborted` only if an axis was skipped.

- [ ] **Step 1: Edit README and UPDATES.**

- [ ] **Step 2: Commit**

```bash
git add README.md UPDATES.md
git commit -m "$(cat <<'EOF'
docs: document sine waveform and envelope retry

Point operators at --waveform sine and the recapture-continue abort behavior.
EOF
)"
```
