# GZ advanced_plane race_quat + PN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Gazebo `advanced_plane` complete the production balloon course under shipped `race_quat` + `pn` with first-circuit 3D miss ≤5 m, matching the Cessna bar.

**Architecture:** Do not copy Cessna `FW_*` or 14 m/s energy onto AdvancedLiftDrag. Port the Cessna *outer structure* (elevator lead, 20° LOS cap, roll slew/LPF, pitch LPF, vz D, ±20° PX4 pitch limits) onto `gz_advanced_plane` `race_quat`/`race_euler`, keep this airframe's 20 m/s trim, SID the inner loop on this model, then live-tune thrust/speed until the gate. Shipped `flightSetup.json` stays `gz_model: rc_cessna`; this work is `--model advanced_plane`.

**Tech Stack:** Python 3 unittest, plant JSONC, PX4 v1.17 SITL, Gazebo (`./run_balloon_race.sh --gz --model advanced_plane`), host SID (`./run_control_calibration.sh --gz --model advanced_plane`).

**Spec:** In-chat design 2026-09-04 (approved). No separate spec file. Cessna live bar (UPDATES 0.70.0): first circuit B0 **2.91** / B1 **0.20** / B2 **4.62** m on the 500/200/10 m course.

## Global Constraints

- Shipped `python/flightSetup.json` `sim.gz_model` stays `rc_cessna`. Do not change `guidance.controller` (`race_quat`) or `homing_law` (`pn`).
- Do not modify `python/fw_sitl/platforms/gz/gz_rc_cessna.jsonc`.
- Do not copy Cessna `FW_PR_P` 0.50 / cruise 14 m/s onto advanced_plane. Inner loop comes from SID on this airframe. Cruise speed stays `fw_airspd_trim` **20** m/s unless a live race proves stall, then drop toward 16 not 14.
- Outer structure (both `controllers.race_quat` and `controllers.race_euler` unless noted): `kp_elev` **1.5**, `att_los_max_pitch_rad` **0.35**, `att_max_pitch_rad` **0.26**, `los_roll_slew_rad_s` **0.7853981633974483** (45°/s), `los_roll_lpf_tau_s` **0.10**, `los_pitch_lpf_tau_s` **0.50**. `pitch_vz_gain` **0.08** on `race_quat` only; `race_euler` omits it (loader default 0.03).
- `px4_inner` `FW_P_LIM_MIN` **-20**, `FW_P_LIM_MAX` **20** (replace +32/−15).
- Starting energy (Task 2, before live tune): keep `speed_mps` **20**, `approach_speed_mps` **14**, `slow_range_m` **160**, `cruise_thrust` **0.5**, `speed_thrust_per_mps` **0.04**; raise `min_thrust` **0.2 → 0.35** (stall floor). Task 6 may change energy keys only.
- `pure_pursuit_quat` block: unchanged except it still supplies PP-only aero (`v_stall_mps` 10, `mass_kg` 12).
- Success gate: production course (`flightSetup.json` balloons, `pass_radius_m` 10), 120 s, `race_quat` + `pn`, first three `event==pass` rows 3D miss ≤ **5.0** m (same helper as `assert_race_euler_csv_ok`).
- Host tests: `cd python && python3 -m unittest <modules>` — never `discover` (`plt.show` blocks).
- TDD for test+JSONC tasks. Plant JSONC is the production change; tests pin it.
- Work from this worktree. Commit after green. Do not push. Never dispatch nested subagents.
- Subagents and reviewers: model `cursor-grok-4.6-high` only (no Fast).
- Live SITL needs `DISPLAY` and image `px4-noble-sim-ros:latest`. Kill leftover GZ with `./kill.sh --all` before each SID/race.
- UPDATES newest entry `## 0.71.0 - GZ advanced_plane race_quat + pn`. README plants/known-limits mention the live numbers. Version bump is feature (`subver`).

## File map

- Modify: `python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc`
- Modify: `python/tests/test_plant_gains.py`
- Modify: `python/tests/test_race_quat_e2e.py` (setup round-trip for `gz_model=advanced_plane` only; no new live e2e script)
- Modify: `README.md`, `UPDATES.md`
- Do not modify: `python/flightSetup.json` `gz_model`, `gz_rc_cessna.jsonc`, controller Python, overlay SDF, Dockerfiles

---

### Task 1: RED tests for advanced race_quat outer structure

**Files:**
- Modify: `python/tests/test_plant_gains.py`
- Modify: `python/tests/test_race_quat_e2e.py`

**Interfaces:**
- Consumes: `load_plant_gains("gz_advanced_plane", controller="race_quat"|"race_euler")`, `write_race_quat_e2e_setup(..., gz_model="advanced_plane")`
- Produces: failing tests that pin the Task 2 JSONC values. `test_gz_advanced_plane_differs_from_cessna` still requires `speed_mps==20`, `fw_airspd_trim==20`, `bank_kp_heading` and `pid_kp` less than Cessna (default controller = `pure_pursuit_quat`).

- [ ] **Step 1: Write the failing tests**

In `python/tests/test_plant_gains.py`, **keep** `test_gz_advanced_plane_differs_from_cessna` and `test_px4_inner_gz_advanced_snapshot` (`FW_PR_P` 0.08 / `FW_RR_P` 0.03 / `FW_THR_TRIM` 0.25 still true until SID).

**Change** `test_px4_inner_gz_advanced_snapshot` to also assert pitch limits (this fails until Task 2):

```python
        self.assertAlmostEqual(inner["FW_P_LIM_MIN"], -20.0)
        self.assertAlmostEqual(inner["FW_P_LIM_MAX"], 20.0)
```

**Add** after `test_gz_advanced_plane_differs_from_cessna`:

```python
    def test_gz_advanced_plane_race_quat_center_through(self) -> None:
        """Outer structure matches Cessna chase set; energy stays 20 m/s trim."""
        import math
        from fw_sitl.attitude_pid import q_des_from_los
        from fw_sitl.quat import rpy_from_quat

        race = load_plant_gains("gz_advanced_plane", controller="race_quat")
        euler = load_plant_gains("gz_advanced_plane", controller="race_euler")
        self.assertAlmostEqual(race.kp_elev, 1.5)
        self.assertAlmostEqual(race.att_los_max_pitch_rad, 0.35)
        self.assertAlmostEqual(race.att_max_pitch_rad, 0.26)
        self.assertAlmostEqual(race.los_roll_slew_rad_s, math.radians(45.0))
        self.assertAlmostEqual(race.los_roll_lpf_tau_s, 0.10)
        self.assertAlmostEqual(race.los_pitch_lpf_tau_s, 0.50)
        self.assertAlmostEqual(race.pitch_vz_gain, 0.08)
        self.assertAlmostEqual(euler.pitch_vz_gain, 0.03)
        self.assertAlmostEqual(race.speed_mps, 20.0)
        self.assertAlmostEqual(race.approach_speed_mps, 14.0)
        self.assertAlmostEqual(race.slow_range_m, 160.0)
        self.assertAlmostEqual(race.cruise_thrust, 0.5)
        self.assertAlmostEqual(race.min_thrust, 0.35)
        self.assertAlmostEqual(race.speed_mps, euler.speed_mps)
        self.assertAlmostEqual(race.kp_elev, euler.kp_elev)
        self.assertAlmostEqual(race.att_los_max_pitch_rad, euler.att_los_max_pitch_rad)
        self.assertAlmostEqual(race.los_roll_slew_rad_s, euler.los_roll_slew_rad_s)
        self.assertAlmostEqual(race.los_roll_lpf_tau_s, euler.los_roll_lpf_tau_s)
        self.assertAlmostEqual(race.los_pitch_lpf_tau_s, euler.los_pitch_lpf_tau_s)
        self.assertAlmostEqual(race.cruise_thrust, euler.cruise_thrust)
        self.assertAlmostEqual(race.min_thrust, euler.min_thrust)
        lim = dict(race.px4_inner)
        self.assertAlmostEqual(lim["FW_P_LIM_MIN"], -20.0)
        self.assertAlmostEqual(lim["FW_P_LIM_MAX"], 20.0)
        steep = q_des_from_los(
            (0.2, 0.0, -1.0),
            yaw_rad=0.0,
            **race.los_kwargs(),
        )
        self.assertLessEqual(rpy_from_quat(steep)[1], 0.35 + 1e-6)
        self.assertGreater(rpy_from_quat(steep)[1], 0.26)
```

In `python/tests/test_race_quat_e2e.py` `TestRaceQuatE2ESetup`, add:

```python
    def test_write_setup_gz_advanced_plane(self) -> None:
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            path = write_race_quat_e2e_setup(
                Path(tmp) / "adv.json",
                platform="gz",
                duration_s=45.0,
                gz_model="advanced_plane",
            )
            setup = load_flight_setup(path)
            self.assertEqual(setup.sim.platform, "gz")
            self.assertEqual(setup.sim.gz_model, "advanced_plane")
            self.assertEqual(setup.guidance.controller, "race_quat")
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd python && python3 -m unittest tests.test_plant_gains.TestPlantGainsRegistry.test_gz_advanced_plane_race_quat_center_through tests.test_plant_gains.TestPlantGainsRegistry.test_px4_inner_gz_advanced_snapshot tests.test_race_quat_e2e.TestRaceQuatE2ESetup.test_write_setup_gz_advanced_plane -v
```

Expected: `test_write_setup_gz_advanced_plane` PASS (setup helper already accepts the model). `test_gz_advanced_plane_race_quat_center_through` FAIL (`kp_elev` 1.0 default, `att_los_max_pitch_rad` 0.7, `min_thrust` 0.2). `test_px4_inner_gz_advanced_snapshot` FAIL (`FW_P_LIM_MAX` 32 not 20).

- [ ] **Step 3: Commit tests only**

```bash
git add python/tests/test_plant_gains.py python/tests/test_race_quat_e2e.py
git commit -m "$(cat <<'EOF'
test: pin GZ advanced_plane race_quat outer structure

EOF
)"
```

---

### Task 2: GREEN — write outer keys into gz_advanced_plane.jsonc

**Files:**
- Modify: `python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc`
- Test: `python/tests/test_plant_gains.py` (from Task 1)

**Interfaces:**
- Consumes: Task 1 assertions
- Produces: `load_plant_gains("gz_advanced_plane", controller="race_quat")` matches Task 1. `px4_inner` still has `FW_PR_P` 0.08 (SID is Task 3).

- [ ] **Step 1: Confirm RED still fails on current JSONC**

Same unittest command as Task 1 Step 2. Center-through and P_LIM still FAIL.

- [ ] **Step 2: Minimal JSONC edits**

In `px4_inner` only change the pitch-limit rows:

```
["FW_P_LIM_MAX", 20.0],
["FW_P_LIM_MIN", -20.0],
```

Do not change `FW_PR_*` / `FW_RR_*` / `FW_YR_*` / `FW_THR_*` yet.

In **both** `controllers.race_quat` and `controllers.race_euler`:

- Set `att_max_pitch_rad` 0.35 → **0.26**
- Set `att_los_max_pitch_rad` 0.7 → **0.35**
- Add `kp_elev` **1.5**
- Add `los_roll_slew_rad_s` **0.7853981633974483**
- Add `los_roll_lpf_tau_s` **0.10**
- Add `los_pitch_lpf_tau_s` **0.50**
- Set `min_thrust` 0.2 → **0.35**
- Keep `cruise_thrust` 0.5, `speed_mps` 20, `approach_speed_mps` 14, `slow_range_m` 160, `speed_thrust_per_mps` 0.04, `bank_kp_heading` 1.3, `pid_kp` 0.7

In `controllers.race_quat` only, add:

```
"pitch_vz_gain": 0.08,
```

Do **not** add `pitch_vz_gain` on `race_euler` (default 0.03). Comment the new keys the same way Cessna does (one-line why, not an essay).

Do not touch `controllers.pure_pursuit_quat`.

- [ ] **Step 3: Run tests to verify they pass**

```bash
cd python && python3 -m unittest tests.test_plant_gains tests.test_race_quat_platforms tests.test_race_quat_e2e.TestRaceQuatE2ESetup -v
```

Expected: all PASS. `test_gz_advanced_plane_differs_from_cessna` still passes (PP `speed_mps` 20, `pid_kp` 0.7 < Cessna 1.0).

- [ ] **Step 4: Commit**

```bash
git add python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc
git commit -m "$(cat <<'EOF'
Give GZ advanced_plane the race_quat center-through outer set

EOF
)"
```

---

### Task 3: Live SID rates on advanced_plane → px4_inner

**Files:**
- Modify: `python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc` (`px4_inner` rate rows only)
- Modify: `python/tests/test_plant_gains.py` (`test_px4_inner_gz_advanced_snapshot`)

**Interfaces:**
- Consumes: Task 2 plant; `./run_control_calibration.sh --layer rates --gz --model advanced_plane --no-plot`
- Produces: `px4_inner` `FW_RR_P`/`FW_RR_FF`/`FW_PR_P`/`FW_PR_FF`/`FW_YR_P` (and `FW_R_TC` only if hints name it) from SID, not Cessna copies. Snapshot test matches the written numbers.

- [ ] **Step 1: Kill leftover sims**

```bash
./kill.sh --all
```

- [ ] **Step 2: Run rates SID**

From repo root (worktree):

```bash
./run_control_calibration.sh --layer rates --gz --model advanced_plane --no-plot
```

SID writes `/tmp/fw_calib_<stamp>/`. Find the latest `*_hints.json`. If the command fails to arm/start GZ, report **BLOCKED** with the log path — do not invent gains.

- [ ] **Step 3: Apply hints with these clamps**

Read every channel in `*_hints.json`. For each channel whose `verdict` is not `ok`/`no_data`, apply the first hint:

| direction | key contains `_P` or `_FF` | key contains `_TC` |
|-----------|-----------------------------|--------------------|
| up        | multiply current JSONC value by **1.25** | subtract **0.05** (floor 0.20) |
| down      | multiply by **0.80** | add **0.05** (ceil 0.80) |

Clamps after each apply: `FW_PR_P`/`FW_PR_FF` ∈ [0.05, 0.80]; `FW_RR_P`/`FW_RR_FF` ∈ [0.03, 0.70]; `FW_YR_P`/`FW_YR_FF` ∈ [0.10, 0.60]. Round to 2 decimal places.

If all channels `ok` or `no_data`, leave rate P/FF unchanged and record that in the commit message / report.

- [ ] **Step 4: Re-run SID at most twice more (3 total)**

After writing JSONC, `./kill.sh --all`, re-run rates SID. Stop when all of p/q/r are `ok` **or** this was the 3rd iteration. Keep the last applied numbers even if not all `ok`; put remaining verdicts in the report.

- [ ] **Step 5: Update snapshot test to the written values**

Replace the three `test_px4_inner_gz_advanced_snapshot` asserts for `FW_PR_P`, `FW_RR_P`, `FW_THR_TRIM` only as needed (`FW_THR_TRIM` stays 0.25 unless you had to change it — you must not). Add asserts for any rate key you changed (`FW_PR_FF`, `FW_YR_P`, …). Keep `FW_P_LIM_*` ±20.

- [ ] **Step 6: Host tests**

```bash
cd python && python3 -m unittest tests.test_plant_gains.TestPlantGainsRegistry.test_px4_inner_gz_advanced_snapshot tests.test_plant_gains.TestPlantGainsRegistry.test_gz_advanced_plane_race_quat_center_through -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc python/tests/test_plant_gains.py
git commit -m "$(cat <<'EOF'
SID GZ advanced_plane rate inner loop

EOF
)"
```

Report must include: calib dir path, per-channel verdicts (last run), keys written.

---

### Task 4: Live SID attitude → race_quat / race_euler pid and TCs

**Files:**
- Modify: `python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc` (`controllers.race_quat` and `controllers.race_euler` `pid_kp`/`roll_tc`/`pitch_tc` only)
- Modify: `python/tests/test_plant_gains.py` (`test_gz_advanced_plane_differs_from_cessna` only if `pid_kp` would break `assertLess` vs Cessna 1.0 — keep `pid_kp` ≤ 0.95)

**Interfaces:**
- Consumes: Task 3 inner; `./run_control_calibration.sh --layer attitude --gz --model advanced_plane --no-plot`
- Produces: same `pid_kp`, `roll_tc`, `pitch_tc` on `race_quat` and `race_euler`. Do not change PP block. Do not change `px4_inner` unless attitude hints name `FW_R_TC` (then write it and update the snapshot).

- [ ] **Step 1: Kill + attitude SID**

```bash
./kill.sh --all
./run_control_calibration.sh --layer attitude --gz --model advanced_plane --no-plot
```

- [ ] **Step 2: Apply hints**

Same multiply table as Task 3. `pid_kp` clamp [0.40, 0.95]. `roll_tc`/`pitch_tc` clamp [0.20, 0.80]. Apply identical values to `race_quat` and `race_euler`. Ignore hints that name `bank_kp_heading` / `bank_kp_alt` (those stay at JSONC 1.3 / 0.022 — outer structure already set). Max 3 iterations.

- [ ] **Step 3: If `pid_kp` is no longer < Cessna's 1.0**, change `test_gz_advanced_plane_differs_from_cessna` to load `controller="race_quat"` and still assert `adv.speed_mps == 20` and `fw_airspd_trim == 20` and `adv.pid_kp != cessna.pid_kp` (not necessarily less). Do not raise Cessna's pid.

- [ ] **Step 4: Host tests**

```bash
cd python && python3 -m unittest tests.test_plant_gains tests.test_race_quat_platforms -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc python/tests/test_plant_gains.py
git commit -m "$(cat <<'EOF'
SID GZ advanced_plane attitude cascade TCs

EOF
)"
```

---

### Task 5: Baseline 120 s live race (record, do not yet retune energy)

**Files:**
- None required. If the plane cannot arm or crashes before t=20 s, you may raise `min_thrust` by 0.05 (both race_* blocks) and re-run once — that is the only JSONC change allowed here.

**Interfaces:**
- Consumes: Tasks 2–4 plant
- Produces: CSV + pickle under `/tmp/balloon_race_adv_baseline_<stamp>.csv` (+ `.pkl`); report first-circuit misses. Does not claim the 5 m gate yet.

- [ ] **Step 1: Launch**

```bash
./kill.sh --all
export BALLOON_RACE_CSV=/tmp/balloon_race_adv_baseline.csv
./run_balloon_race.sh --gz --model advanced_plane --duration 120 --no-plot --detach --session adv_baseline
```

Wait until the CSV contains an `end_*` event (timeout 120+240 s). Then:

```bash
cd python && python3 - <<'PY'
from pathlib import Path
from fw_sitl.race_csv import load_pass_misses
from fw_sitl.race_e2e import assert_race_euler_csv_ok
p = Path("/tmp/balloon_race_adv_baseline.csv")
print("passes", load_pass_misses(p))
try:
    assert_race_euler_csv_ok(p, min_passes=3, max_miss_m=5.0)
    print("GATE PASS")
except Exception as e:
    print("GATE FAIL", e)
PY
```

- [ ] **Step 2: Report**

Write to the report file: CSV path, pickle path if present, first three (balloon_idx, miss_m), whether GATE PASS. If zero passes, describe stall vs fly-by (from CSV `pos_d` vs `tgt_d` and tmux control pane). Copy the CSV/pkl into the SDD workspace directory so later tasks can read them.

- [ ] **Step 3: Commit only if JSONC changed** (`min_thrust` bump). If no JSONC change, do not create an empty commit. Still report DONE.

If JSONC changed, update `test_gz_advanced_plane_race_quat_center_through` `min_thrust` assert to the new value, re-run that test, commit JSONC + test.

---

### Task 6: Energy iteration until first-circuit ≤5 m

**Files:**
- Modify: `python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc` (energy keys in `race_quat` **and** `race_euler` together: `cruise_thrust`, `min_thrust`, `speed_mps`, `approach_speed_mps`, `slow_range_m`, `speed_thrust_per_mps` — never `kp_elev` / pitch caps / LPF / `pitch_vz_gain` unless a race shows vz bob > 60 sign-flips / 120 s, then `pitch_vz_gain` on `race_quat` only, step ±0.02, clamp [0.03, 0.12])
- Modify: `python/tests/test_plant_gains.py` (center-through energy asserts)

**Interfaces:**
- Consumes: Task 5 baseline miss
- Produces: first circuit 3D ≤5 m on all three balloons, or a documented best-effort after **6** live races with the best set left in JSONC and `DONE_WITH_CONCERNS`.

- [ ] **Step 1: Decision table (apply before each race after the first)**

| Observation | Change (both race_* blocks) |
|-------------|------------------------------|
| Crash / GS < 12 m/s after t=20 | `min_thrust += 0.05` (ceil 0.55), `cruise_thrust += 0.05` (ceil 0.80) |
| 0 passes, flew past high (ΔD negative, |ΔD|>10) | `cruise_thrust -= 0.05` (floor 0.35) |
| 0 passes, flew under (ΔD positive >10) | `cruise_thrust += 0.05` |
| B0 miss >5, B1/B2 ok | leave energy; if high residual, `approach_speed_mps -= 2` (floor `fw_airspd_min` 10) |
| B2 miss >5, B0/B1 ≤5 | `slow_range_m` 160→220 then 280; `approach_speed_mps` 14→12 then 10 |
| All three >5 but all passed | `speed_thrust_per_mps` 0.04→0.06 |
| Stall on climb to B1 | `speed_mps` 20→18 (not below 16) |

Never set `speed_mps` to 14. Never copy Cessna `cruise_thrust` 0.65 as a first move — only if the table lands there.

- [ ] **Step 2: Run races**

Each attempt:

```bash
./kill.sh --all
export BALLOON_RACE_CSV=/tmp/balloon_race_adv_tuneN.csv
./run_balloon_race.sh --gz --model advanced_plane --duration 120 --no-plot --detach --session adv_tuneN
```

Score with `assert_race_euler_csv_ok(..., min_passes=3, max_miss_m=5.0)`. Stop at first GATE PASS or 6 attempts. Leave the **best** first-circuit (lowest max of the three misses, requiring 3 passes) in JSONC.

- [ ] **Step 3: Update tests** to the final energy numbers. Re-run:

```bash
cd python && python3 -m unittest tests.test_plant_gains tests.test_race_quat_platforms -v
```

- [ ] **Step 4: Commit**

```bash
git add python/fw_sitl/platforms/gz/gz_advanced_plane.jsonc python/tests/test_plant_gains.py
git commit -m "$(cat <<'EOF'
Tune GZ advanced_plane race_quat energy for center-through

EOF
)"
```

Report: table of attempts (CSV, three misses), winning keys, GATE PASS/FAIL.

---

### Task 7: README + UPDATES + live numbers

**Files:**
- Modify: `README.md`
- Modify: `UPDATES.md`
- Modify: `python/tests/test_plant_gains.py` only if Task 6 left a comment mismatch (no)

**Interfaces:**
- Consumes: Task 6 winning misses and plant keys
- Produces: `UPDATES.md` newest `## 0.71.0 - GZ advanced_plane race_quat + pn`. README plants row and known-limits / plant JSONC paragraph mention advanced_plane `race_quat` + `pn` and the live first-circuit numbers.

- [ ] **Step 1: UPDATES.md** — newest entry on top, after the `# Updates` heading. Feature bump 0.70.0 → **0.71.0**. Include: `--gz --model advanced_plane`, `race_quat`+`pn`, outer structure copied (20° / kp_elev 1.5 / vz D 0.08), SID inner (name the final `FW_PR_P` / `FW_RR_P`), energy keys, live first-circuit B0/B1/B2 m and CSV path. One line each. If Task 6 was `DONE_WITH_CONCERNS`, say the miss and that the gate is still open.

- [ ] **Step 2: README.md**

- Plants table `--gz` row: keep Cessna default; add that `advanced_plane` is race_quat+pn viable (or "tuning" if gate failed).
- Plant JSONC paragraph: after the Cessna `race_quat` sentence, one sentence that `gz_advanced_plane` uses the same outer structure at 20 m/s trim (name cruise_thrust / speed from JSONC).
- Known limits: one bullet with the live advanced_plane first-circuit numbers (or the miss if gate failed).
- Do not rewrite the Cessna 0.70.0 numbers.

- [ ] **Step 3: Confirm shipped setup still Cessna**

```bash
python3 - <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, "python")
from fw_sitl.flight_setup import load_flight_setup
s = load_flight_setup(Path("python/flightSetup.json"))
assert s.sim.gz_model == "rc_cessna", s.sim.gz_model
assert s.guidance.controller == "race_quat"
assert s.guidance.homing_law == "pn"
print("setup ok", s.sim.gz_model, s.guidance.controller, s.guidance.homing_law)
PY
```

- [ ] **Step 4: Host tests**

```bash
cd python && python3 -m unittest tests.test_plant_gains tests.test_flight_setup tests.test_race_quat_e2e.TestRaceQuatE2ESetup -v
```

- [ ] **Step 5: Commit**

```bash
git add README.md UPDATES.md
git commit -m "$(cat <<'EOF'
Document GZ advanced_plane race_quat + pn tune

EOF
)"
```

---

## Self-review

1. **Spec coverage:** outer port, SID inner, live energy, ≤5 m gate, Cessna plant untouched, shipped gz_model unchanged, docs — each has a task.
2. **Placeholders:** none. SID apply table and energy table are numeric.
3. **Types:** `load_plant_gains(..., controller="race_quat")`; `write_race_quat_e2e_setup(..., gz_model="advanced_plane")`; `assert_race_euler_csv_ok` reused for the 5 m 3D+|ΔD| gate.
