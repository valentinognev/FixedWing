# F-16 Aerodynamic Coefficients In JSON

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The F-16 JSON files carry the aerodynamic coefficients of the Morelli and Stevens models, and those numbers are loaded when the model runs. No aerodynamic coefficient remains a literal in `python/f16/`.

**Architecture:** `python/f16/aero_data.py` reads `data/planes/f16/<model>.json` and caches the `coefficients` object. `aero_morelli.py` and `aero_stevens.py` keep the polynomial and the table interpolation, and take every coefficient from that object. `write_trim_survey` copies an existing `coefficients` object into the file it rewrites.

**Tech Stack:** Python 3, numpy, unittest (`cd python && python3 -m unittest <module> -v`). Host-only. No Docker, MAVLink, ZMQ, PX4, network, or `fgfs`.

**Spec:** This plan is the spec. There is no separate design document. The approved design is the Approved design section below.

## Approved design

Each of `data/planes/f16/morelli.json` and `data/planes/f16/stevens.json` gains a `coefficients` object next to the existing trim grid.

Morelli `coefficients` is one array per polynomial group: `cx` (7), `cxq` (5), `cy` (3), `cyp` (4), `cyr` (4), `cz` (6), `czq` (5), `cl` (8), `clp` (4), `clr` (5), `clda` (7), `cldr` (7), `cm` (8), `cmq` (6), `cn` (7), `cnp` (5), `cnr` (3), `cnda` (10), `cndr` (6). Order is a0…, b0…, through s0… as written in `aero_morelli.py` today.

Stevens `coefficients` stores each lookup table in the shape the interpolation indexes (the array after `.T` for every table that transposes): `cx` (12×5), `cz` (12), `cl` (12×7), `cm` (12×5), `cn` (12×7), `dlda` (12×7), `dldr` (12×7), `dnda` (12×7), `dndr` (12×7), `damp` (12×9). Scalar gains: `cy_beta` -0.02, `cy_aileron` 0.021, `cy_rudder` 0.086, `aileron_norm_deg` 20, `rudder_norm_deg` 30, `beta_norm_deg` 57.3, `cz_elevator` -0.19, `elevator_norm_deg` 25.

A loader reads the file once per resolved path and caches it. `morelli_coefficients` and `stevens_coefficients` use that cache. A missing file, a missing `coefficients` object, or a missing group raises `ValueError` before any coefficient is returned. Polynomial expressions and Stevens index arithmetic stay in code. Thrust maps, inertia, and reference geometry stay in `model.py` and `units.py`.

`write_trim_survey` copies an existing `coefficients` object onto the payload it writes. A survey of a file that has no such object does not invent one.

## Global Constraints

- Aerodynamic coefficient numbers live only in `data/planes/f16/morelli.json` and `data/planes/f16/stevens.json` under `coefficients`. After Tasks 1 and 2, `python/f16/aero_morelli.py` contains none of the Morelli polynomial literals, and `python/f16/aero_stevens.py` contains none of the table entries or the scalar gains listed above.
- The polynomial structure in `morelli_coefficients` stays. The Stevens `_fix` / `_sign` interpolation stays, including the alpha index `0.2 * alpha`, the elevator column step `el / 12`, the `1.1` neighbor step, and the index clamps. `tvt = 0.5 / vt` and `phat = p * b / (2 * V)` stay. Those are the model structure, not stored coefficients.
- `cy_table` uses `cy_beta`, `cy_aileron`, `cy_rudder`, `aileron_norm_deg`, and `rudder_norm_deg`. `cz_table` uses `beta_norm_deg`, `cz_elevator`, and `elevator_norm_deg`. `dail` and `drdr` use `aileron_norm_deg` and `rudder_norm_deg`.
- Do not edit `python/f16/model.py` or `python/f16/units.py`. Do not move thrust tables or `c1`–`c9`.
- `load_aero_coefficients(model, root=None)` resolves `data/planes/f16/<model>.json` the same way `f16.trim_table.load_trim_table` resolves `TRIM_DIR` when `root` is omitted. `root` replaces that directory. Unknown model raises `ValueError` with `not implemented` in the message. Missing `coefficients`, or a missing group, raises `ValueError` that names the missing key.
- `morelli_coefficients` and `stevens_coefficients` gain a keyword-only `root: Path | None = None` after the existing positional arguments. Existing callers keep working.
- Cache key is the resolved file path. A second call with the same path does not re-read the file. Tests that need a different model use a different directory.
- Shipped Morelli `coefficients.cx[0]` is exactly the current `a0`, `-1.943367e-2`. Shipped Stevens `coefficients.cx[0][0]` is `-0.099`, `coefficients.cz[0]` is `0.770`, `coefficients.damp[0][0]` is `-0.267`, `coefficients.cy_beta` is `-0.02`.
- Host tests only: `cd python && python3 -m unittest <module> -v`. TDD: write the test, run it, see it fail because the loader or the JSON key is absent, then implement. Record RED and GREEN command output in the report.
- Commit each task on this branch. Do not push. Do not update git config. Do not use `--no-verify`. Commit message via HEREDOC.
- `UPDATES.md` top entry is `0.88.0`. README changes only the sentence named in Task 3.

## Parallelism

- Tasks 1–3 are sequential. Task 2 uses the loader from Task 1. Task 3 preserves the `coefficients` objects Tasks 1 and 2 wrote. Do not dispatch them together.

## File map

| File | Task | Responsibility |
|------|------|----------------|
| `python/f16/aero_data.py` | 1 | Load and cache `coefficients` |
| `python/f16/aero_morelli.py` | 1 | Polynomial uses loaded coefficients |
| `data/planes/f16/morelli.json` | 1 | Morelli `coefficients` plus the existing trim grid |
| `python/tests/test_f16_aero_coefficients.py` | 1–2 | JSON is what the evaluation reads |
| `python/f16/aero_stevens.py` | 2 | Tables and scalar gains come from JSON |
| `data/planes/f16/stevens.json` | 2 | Stevens `coefficients` plus the existing trim grid |
| `python/f16/trim_table.py` | 3 | Survey keeps an existing `coefficients` object |
| `python/tests/test_f16_trim_survey.py` | 3 | Preservation test |
| `README.md` | 3 | One sentence: coefficients load from the JSON files |
| `UPDATES.md` | 3 | `0.88.0` entry |

### Task 1: Morelli coefficients from JSON

**Files:**
- Create: `python/f16/aero_data.py`
- Create: `python/tests/test_f16_aero_coefficients.py`
- Modify: `python/f16/aero_morelli.py`
- Modify: `data/planes/f16/morelli.json`

**Interfaces:**
- Produces: `load_aero_coefficients(model: str, root: Path | None = None) -> dict` in `f16.aero_data`. The dict is the JSON `coefficients` object. `morelli_coefficients(..., *, root: Path | None = None)`.

- [ ] **Step 1: Write the failing tests**

Create `python/tests/test_f16_aero_coefficients.py` with these tests and no others:

1. `test_morelli_zero_state_follows_json_cx0` — Build a temp directory containing `morelli.json` whose `coefficients` has every Morelli group listed in the Approved design, each the right length, every value `0`, except `cx[0] == -0.5`. Call `morelli_coefficients(0, 0, 0, 0, 0, 0, 0, 0, 1.0, 1.0, 100.0, 0.35, 0.35, root=that_directory)`. Assert the six returned coefficients are `(-0.5, 0, 0, 0, 0, 0)` within `1e-12`.

2. `test_shipped_morelli_cx0` — `load_aero_coefficients("morelli")["cx"][0]` equals `-1.943367e-2` within `0` absolute tolerance (`assertEqual` on the float, or `assertAlmostEqual` with `delta=0`).

3. `test_missing_coefficients_raises` — A temp `morelli.json` of `{"model": "morelli"}` makes `load_aero_coefficients("morelli", root=that_directory)` raise `ValueError` and the message contains `coefficients`.

4. `test_morelli_source_has_no_a0_literal` — The text of `python/f16/aero_morelli.py` does not contain `1.943367e-2`.

- [ ] **Step 2: Run the tests and confirm RED**

```bash
cd python && python3 -m unittest tests.test_f16_aero_coefficients -v
```

Failure is `ImportError` or `ValueError` / missing key / assertion, because the loader or the JSON group does not exist yet. A pass is the wrong RED. Fix the test and re-run.

- [ ] **Step 3: Implement**

Add `python/f16/aero_data.py` with the loader and the cache described in Global Constraints. Copy the current Morelli literals into `data/planes/f16/morelli.json` under `coefficients` without changing the trim grid keys (`model`, `constraints`, `vt_mps`, `altitude_m`, `points`). Point `morelli_coefficients` at the loaded arrays, in the existing polynomial, and delete the literals.

- [ ] **Step 4: Run the tests and confirm GREEN**

```bash
cd python && python3 -m unittest tests.test_f16_aero_coefficients tests.test_f16_aero -v
```

Both modules pass. `test_f16_aero` is the proof the polynomial still matches AeroBench.

- [ ] **Step 5: Commit**

```bash
git add python/f16/aero_data.py python/f16/aero_morelli.py python/tests/test_f16_aero_coefficients.py data/planes/f16/morelli.json
git commit -m "$(cat <<'EOF'
Load Morelli aerodynamic coefficients from the F-16 JSON file.

EOF
)"
```

### Task 2: Stevens coefficients from JSON

**Files:**
- Modify: `python/f16/aero_stevens.py`
- Modify: `data/planes/f16/stevens.json`
- Modify: `python/tests/test_f16_aero_coefficients.py`

**Interfaces:**
- Consumes: `load_aero_coefficients` from Task 1.
- Produces: `stevens_coefficients(..., *, root: Path | None = None)`.

- [ ] **Step 1: Write the failing tests**

Append these tests to `python/tests/test_f16_aero_coefficients.py`:

1. `test_stevens_sideforce_follows_json_cy_beta` — Temp `stevens.json` whose `coefficients` has every Stevens table filled with zeros at the shapes in the Approved design, and the scalar gains from the Approved design except `cy_beta` is `-0.5`. Pick `beta` so `f16.units.aerobench_deg(beta) == 1.0`. Call `stevens_coefficients(0, beta, 0, 0, 0, 0, 0, 0, 1.0, 1.0, 150.0, 0.35, 0.35, root=that_directory)`. Assert `Cy == -0.5` within `1e-12`, and `Cx`, `Cz`, `Cl`, `Cm`, `Cn` are `0` within `1e-12`.

2. `test_shipped_stevens_table_corners` — From `load_aero_coefficients("stevens")`: `cx[0][0] == -0.099`, `cz[0] == 0.770`, `damp[0][0] == -0.267`, `cy_beta == -0.02`. Exact float equality.

3. `test_stevens_source_has_no_cy_beta_literal` — The text of `python/f16/aero_stevens.py` does not contain `-.02` and does not contain `-0.02`.

- [ ] **Step 2: Run the new tests and confirm RED**

```bash
cd python && python3 -m unittest tests.test_f16_aero_coefficients.TestStevensCoefficients -v
```

Put the new tests on a class `TestStevensCoefficients`. RED because the JSON group or the `root` argument is absent. Task 1 tests still pass.

- [ ] **Step 3: Implement**

Copy the current Stevens tables into `stevens.json` under `coefficients` in index order (after `.T` where the source transposes). Copy the scalar gains from the Approved design. Keep the trim grid. Read tables and scalars through `load_aero_coefficients("stevens", root=root)` inside each table function that needs them, or once in `stevens_coefficients` and pass the dict down. Delete the numeric table literals and the scalar gain literals from `aero_stevens.py`.

- [ ] **Step 4: Confirm GREEN**

```bash
cd python && python3 -m unittest tests.test_f16_aero_coefficients tests.test_f16_aero -v
```

- [ ] **Step 5: Commit**

```bash
git add python/f16/aero_stevens.py python/tests/test_f16_aero_coefficients.py data/planes/f16/stevens.json
git commit -m "$(cat <<'EOF'
Load Stevens aerodynamic coefficients from the F-16 JSON file.

EOF
)"
```

### Task 3: Trim survey keeps coefficients

**Files:**
- Modify: `python/f16/trim_table.py`
- Modify: `python/tests/test_f16_trim_survey.py`
- Modify: `README.md`
- Modify: `UPDATES.md`

**Interfaces:**
- Consumes: `coefficients` objects written by Tasks 1 and 2. `write_trim_survey(plane, model, root=None) -> Path`.

- [ ] **Step 1: Write the failing test**

In `python/tests/test_f16_trim_survey.py`, add `test_keeps_existing_coefficients`. Seed a temp directory with `morelli.json` whose body is `{"model": "morelli", "coefficients": {"cx": [-0.5]}}`. Patch `f16.trim_table._solve_point` to return `None`. Call `write_trim_survey("f16", "morelli", root=that_directory)`. The written file's `coefficients` equals `{"cx": [-0.5]}`, and `points` is `[]`.

- [ ] **Step 2: Confirm RED**

```bash
cd python && python3 -m unittest tests.test_f16_trim_survey.TestTrimSurvey.test_keeps_existing_coefficients -v
```

The test fails because the rewrite drops `coefficients`.

- [ ] **Step 3: Implement**

In `write_trim_survey`, if the destination file exists and its JSON has a `coefficients` object, set that object on the payload before writing. If the file is absent or has no `coefficients` key, omit the key. Do not change solved trim points.

- [ ] **Step 4: Confirm GREEN**

```bash
cd python && python3 -m unittest tests.test_f16_trim_survey tests.test_f16_trim_table -v
```

- [ ] **Step 5: Docs**

In `README.md`, in the `python/f16/` row of the package table, add this sentence immediately after the sentence that names `--aero` (default `morelli`): `Aerodynamic coefficients for both models load at runtime from data/planes/f16/{morelli,stevens}.json under coefficients.`

Add this top entry to `UPDATES.md`:

```markdown
## 0.88.0 - F-16 aerodynamic coefficients in JSON
- `data/planes/f16/{morelli,stevens}.json` carry each model's aerodynamic coefficients. `morelli_coefficients` and `stevens_coefficients` load that object at runtime.
- `write_trim_survey` keeps an existing `coefficients` object when it rewrites a trim file.
```

- [ ] **Step 6: Commit**

```bash
git add python/f16/trim_table.py python/tests/test_f16_trim_survey.py README.md UPDATES.md
git commit -m "$(cat <<'EOF'
Keep F-16 aerodynamic coefficients when a trim survey rewrites the JSON.

EOF
)"
```
