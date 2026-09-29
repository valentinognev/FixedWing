# Internal SI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every value this repository stores or integrates SI, leaving feet, feet per second, pounds-force, and knots only inside the adapters that talk to a program which still requires them.

**Architecture:** The F-16 state derivative already integrates metres, m/s, radians, and newtons. `_dampp` is the one live lookup that still receives degrees; it will take radians and keep AeroBench's 57.29578 index inside the function so the trim match does not move. JSBSim's initial-condition files are ours: `vt` becomes `M/S` and the attitude tags become `RAD`, at the same physical speed and the same angles. FlightGear packets, FlightGear `--vc`, balloon `elevation-ft`, and the AeroBench / `f16dynamics` parity calls stay imperial at that boundary. `python/f16/units.py` remains the only converter.

**Tech Stack:** Python 3, numpy, unittest (`cd python && python3 -m unittest <module> -v`). Host-only. No Docker, MAVLink, ZMQ, PX4, network, or `fgfs`.

**Spec:** This plan is the spec. Binding scope is the Global Constraints below. It follows the 2026-09-28 SI plant (`docs/superpowers/plans/2026-09-28-si-units.md`, UPDATES `0.79.0`) and does not reopen that conversion.

## Global Constraints

- Execute on current `main` (commit `6e82045` or later, UPDATES `0.79.0` already present). Do not edit `.worktrees/si-units` or `.worktrees/f16-cpp-parity`. Do not edit `../F16/AeroBenchVVPython` or `../F16/f16-flight-dynamics`.
- Host tests only: `cd python && python3 -m unittest <module> -v`. No Docker, MAVLink, ZMQ, PX4, network, or `fgfs`.
- `python/f16/units.py` is the only converter. Call sites do not multiply by `0.3048` or by `pi/180` themselves. `M_PER_FT` is `0.3048`. `AEROBENCH_RTOD` is `57.29578`.
- The integrated F-16 13-state stays SI, altitude positive up. Controls stay throttle 0–1 and surfaces in radians. Force inside the derivative stays newtons. `Nz` and `Ny` stay load factor in g. CSV stays NED metres. Time stays seconds.
- `_dampp(alpha_rad)` indexes the Stevens paper table with `s = 0.2 * aerobench_deg(alpha_rad)`. Do not replace `aerobench_deg` with `180/pi`. That factor is what the existing AeroBench trim match uses. Do not edit the 9×12 coefficient array.
- Do not rewrite thrust, inertia, or atmosphere paper literals in `model.py` / `units.py`. Those arrays are already scaled by `LBF_TO_N` and the SI constants at build time. Live values are newtons and SI; the literals stay.
- Do not remove `m_to_ft`, `ms_to_fts`, `rad_to_deg`, `state_si_to_imp`, `u_si_to_imp`, `aerobench_deg`, or `aerobench_poly_rad` from `units.py`. Replay, compare, and the damping index still call them.
- Parity stays a boundary conversion. `compare.py` still calls AeroBench and `f16dynamics` through `state_si_to_imp` / `u_si_to_imp`. Do not change `MIN_H_DIFF_M` (`50 * 0.3048`) or `FINAL_RMS_LIMIT` (`5.0`).
- FlightGear `FGNetFDM` in `python/f16/replay.py` keeps speeds in ft/s and `vcas` in knots (`_MPS_TO_KT`). Altitude in that packet stays metres. Euler angles stay radians. Do not edit `replay.py`.
- Balloon placement keeps `elevation-ft` in `python/fw_sitl/balloon_scene.py`. `FGModelMgr` ignores `elevation-m`. Do not edit `balloon_scene.py`.
- `python/assets/fg_spawn.env` keeps `--vc=58.3` (knots), `--units-meters`, and `--altitude=919.2`. `--heading` stays degrees. Do not edit that file.
- Latitude and longitude stay `unit="DEG"` in every JSBSim IC. Compass heading in `flightSetup.json` stays `heading_deg`. Gazebo pose stays ENU metres and yaw radians. Do not edit `flightSetup.json` or Gazebo pose code.
- JSBSim IC speed is the current physical value, not a rounded 30: `98.4 * 0.3048 = 29.99232` m/s. Unit token is exactly `M/S`. Attitude unit token is exactly `RAD`. Do not write `MPS`, `M/SEC`, or `FT/SEC`.
- `python/scripts/runSimJsbsimRascal.sh` mounts `python/assets/jsb_spawn.xml`. Changing only `jsb_spawn_xml()` does not change the launched sim. Both the generator and the shipped file change in the same task.
- Race gains, `python/fw_sitl/**` other than `spawn_ic.py`, and calibration are out of scope.
- Each task is one implementer, then one reviewer. Commit commands are in the tasks. Do not commit unless the user has asked for commits in that session; the controller may defer them.
- After the behavior change, UPDATES gets one new top entry `0.80.0`. README changes only the sentences named in Task 3.

## Parallelism

- Task 1 (`python/f16/model.py`, `python/tests/test_f16_model_si.py`) and Task 2 (`python/fw_sitl/spawn_ic.py`, `python/assets/jsb_spawn.xml`, `python/tests/test_spawn_ic.py`) are file-disjoint. Dispatch them in that order anyway: one implementer at a time.
- Task 3 (`README.md`, `UPDATES.md`) runs after Tasks 1 and 2. It consumes both results and does not change code.

## File map

| File | Task | Responsibility |
|---|---|---|
| `python/f16/model.py` | 1 | `_dampp` takes radians. Derivative no longer holds degree locals. |
| `python/tests/test_f16_model_si.py` | 1 | 10 AeroBench-degree row from a radian argument. Existing trim match stays. |
| `python/fw_sitl/spawn_ic.py` | 2 | Generated JSBSim IC uses `M/S` and `RAD`. FG env text unchanged. |
| `python/assets/jsb_spawn.xml` | 2 | File the JSBSim launcher mounts. Same units as the generator. |
| `python/tests/test_spawn_ic.py` | 2 | Generator and shipped file. FG `--vc=58.3` stays. |
| `README.md`, `UPDATES.md` | 3 | Record the boundary list at `0.80.0`. |

---

### Task 1: Damping lookup takes radians

**Files:**
- Modify: `python/f16/model.py` (`_dampp` signature and the four degree locals in `_subf16_morelli`)
- Test: `python/tests/test_f16_model_si.py`

**Interfaces:**
- Consumes: `aerobench_deg` from `f16.units` (`angle_rad * 57.29578`). `_dampp` coefficient array, unchanged. `subf16_derivative(x13, u4, model="morelli")` returning `(xd, Nz, Ny)`.
- Produces: `_dampp(alpha_rad: float) -> np.ndarray` shape `(9,)`. `subf16_derivative` signature unchanged. No `alpha_deg`, `beta_deg`, `dail`, or `drdr` names left in `model.py`.

- [ ] **Step 1: Write the failing test**

Add this method to `TestModelSi` in `python/tests/test_f16_model_si.py`. Import `AEROBENCH_RTOD` from `f16.units` next to the existing units import.

```python
def test_dampp_ten_aerobench_degrees_from_radians(self) -> None:
    from f16.model import _dampp
    from f16.units import AEROBENCH_RTOD

    alpha_rad = 10.0 / AEROBENCH_RTOD
    expected = np.array(
        [2.08, 0.962, 0.258, -31.2, 0.208, -0.383, -6.11, -0.370, -0.013],
        dtype=float,
    )
    np.testing.assert_allclose(_dampp(alpha_rad), expected, rtol=0, atol=1e-12)
```

That row is column index 4 of the untransposed Stevens damping table (exactly 10 degrees: `s = 2`, `da = 0`). Passing `10.0` (degrees) or `0.2 * alpha_rad` without `aerobench_deg` selects a different row and fails this test.

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_model_si.TestModelSi.test_dampp_ten_aerobench_degrees_from_radians -v`

Expected: FAIL. `_dampp` currently treats its argument as degrees, so `10/57.29578` indexes near zero instead of the 10-degree column.

- [ ] **Step 3: Write the minimal implementation**

In `python/f16/model.py`:

1. Drop `deg_to_rad` from the `f16.units` import. Keep `aerobench_deg` and `aerobench_poly_rad`.
2. Replace the opening of `_dampp` so the argument is radians and the index is unchanged numerically:

```python
def _dampp(alpha_rad):
    """Stevens damping table. Argument is radians.

    The paper grid is 5 degrees. ``0.2 * aerobench_deg`` is that grid with
    AeroBench's 57.29578, matching ``subf16_model``.
    """
    a = np.array([[-.267, -.110, .308, 1.34, 2.08, 2.91, 2.76, 2.05, 1.50, 1.49, 1.83, 1.21],
                  [.882, .852, .876, .958, .962, .974, .819, .483, .590, 1.21, -.493, -1.04],
                  [-.108, -.108, -.188, .110, .258, .226, .344, .362, .611, .529, .298, -2.27],
                  [-8.80, -25.8, -28.9, -31.4, -31.2, -30.7, -27.7, -28.2, -29.0, -29.8, -38.3, -35.3],
                  [-.126, -.026, .063, .113, .208, .230, .319, .437, .680, .100, .447, -.330],
                  [-.360, -.359, -.443, -.420, -.383, -.375, -.329, -.294, -.230, -.210, -.120, -.100],
                  [-7.21, -.540, -5.23, -5.26, -6.11, -6.64, -5.69, -6.00, -6.20, -6.40, -6.60, -6.00],
                  [-.380, -.363, -.378, -.386, -.370, -.453, -.550, -.582, -.595, -.637, -1.02, -.840],
                  [.061, .052, .052, -.012, -.013, -.024, .050, .150, .130, .158, .240, .150]], dtype=float).T
    s = 0.2 * aerobench_deg(alpha_rad)
```

Leave the rest of `_dampp` (`k = _fix(s)` through `return d`) as it is. The array literal above is the current one; do not retype it if a patch can change only `def _dampp(alpha):` and `s = .2 * alpha`.

3. In `_subf16_morelli`, delete these four lines and pass the radian state into `_dampp`:

```python
    alpha_deg = aerobench_deg(x[1])
    beta_deg = aerobench_deg(x[2])
```

```python
    dail = ail / deg_to_rad(20.0)
    drdr = rdr / deg_to_rad(30.0)
```

```python
    d = _dampp(x[1])
```

`aerobench_poly_rad(x[1])` and `aerobench_poly_rad(x[2])` stay on the `_morellif16` call. Those arguments are already radians.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd python && python3 -m unittest tests.test_f16_model_si tests.test_f16_model_llc -v`

Expected: PASS. `test_dampp_ten_aerobench_degrees_from_radians` passes, and the existing trim / off-trim AeroBench matches still pass (`rtol=1e-8`, `atol=1e-8`). If `f16dynamics` is importable, `test_cpp_trim_derivatives_and_u` passes too. A skip is allowed only when that import fails.

- [ ] **Step 5: Commit**

```bash
git add python/f16/model.py python/tests/test_f16_model_si.py
git commit -m "fix(f16): index the damping table from radians"
```

Defer this commit when the session has no commit permission.

---

### Task 2: JSBSim initial condition in SI

**Files:**
- Modify: `python/fw_sitl/spawn_ic.py` (`_JSB_VT_FT_S`, `jsb_spawn_xml`)
- Modify: `python/assets/jsb_spawn.xml`
- Test: `python/tests/test_spawn_ic.py`

**Interfaces:**
- Consumes: `SpawnSpec.heading_deg` (degrees, unchanged). `fg_spawn_env_text` (unchanged output).
- Produces: `jsb_spawn_xml(spawn) -> str` with `vt` unit `M/S` and value `29.99232`, `gamma` / `phi` / `theta` / `psi` unit `RAD`, latitude and longitude unit `DEG`. Shipped `python/assets/jsb_spawn.xml` uses the same tokens. `psi` text is `math.radians(heading_deg)` formatted with `.8f`.

- [ ] **Step 1: Write the failing test**

In `python/tests/test_spawn_ic.py`, replace the body of `test_jsb_xml_heading_and_offset_from_lszh` so it expects SI tags. Keep the latitude and longitude assertions.

```python
def test_jsb_xml_heading_and_offset_from_lszh(self) -> None:
    spawn = SpawnSpec(ned=(100.0, 0.0, 0.0), heading_deg=45.0)
    xml = jsb_spawn_xml(spawn)
    root = ET.fromstring(xml)
    lat = float(root.find("latitude").text)
    lon = float(root.find("longitude").text)
    vt = root.find("vt")
    psi = root.find("psi")
    self.assertGreater(lat, DEFAULT_ORIGIN_LAT_DEG)
    self.assertAlmostEqual(lon, DEFAULT_ORIGIN_LON_DEG, places=5)
    self.assertEqual(root.find("latitude").get("unit"), "DEG")
    self.assertEqual(root.find("longitude").get("unit"), "DEG")
    self.assertEqual(vt.get("unit"), "M/S")
    self.assertAlmostEqual(float(vt.text), 29.99232, places=5)
    self.assertEqual(psi.get("unit"), "RAD")
    self.assertAlmostEqual(float(psi.text), math.radians(45.0), places=8)
    for tag in ("gamma", "phi", "theta"):
        node = root.find(tag)
        self.assertEqual(node.get("unit"), "RAD")
        self.assertAlmostEqual(float(node.text), 0.0, places=8)
    self.assertNotIn("FT/SEC", xml)
```

Add this method on `TestJsbAndFgIc`:

```python
def test_shipped_jsb_spawn_matches_generator_units(self) -> None:
    text = (_PYTHON_ROOT / "assets" / "jsb_spawn.xml").read_text(encoding="utf-8")
    root = ET.fromstring(text)
    vt = root.find("vt")
    self.assertEqual(vt.get("unit"), "M/S")
    self.assertAlmostEqual(float(vt.text), 29.99232, places=5)
    for tag in ("gamma", "phi", "theta", "psi"):
        node = root.find(tag)
        self.assertEqual(node.get("unit"), "RAD")
        self.assertAlmostEqual(float(node.text), 0.0, places=8)
    self.assertEqual(root.find("altitude").get("unit"), "M")
    self.assertEqual(root.find("latitude").get("unit"), "DEG")
    self.assertNotIn("FT/SEC", text)
```

In `test_fg_env_has_lat_lon_heading`, add these assertions. Do not drop the existing `--heading=90` check.

```python
self.assertIn("--units-meters", text)
self.assertIn("--vc=58.3", text)
self.assertNotIn("--vc=30", text)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd python && python3 -m unittest tests.test_spawn_ic.TestJsbAndFgIc -v`

Expected: FAIL. `vt` is still `unit="FT/SEC"` with text `98.4`, and `psi` is still `45.0` degrees. The FG `--vc=58.3` assertion already passes; do not change `fg_spawn.env` to make it fail.

- [ ] **Step 3: Write the minimal implementation**

In `python/fw_sitl/spawn_ic.py`, replace the speed constant and its comment:

```python
# Same physical speed as the old 98.4 ft/s line and fg_spawn.env --vc=58.3 kn.
_JSB_VT_MPS = 98.4 * 0.3048
_FG_VC_KN = 58.3
```

Delete `_JSB_VT_FT_S`. `_FG_VC_KN` stays, and `fg_spawn_env_text` stays byte-for-byte apart from that rename of the constant it already uses.

Replace the return of `jsb_spawn_xml` with:

```python
def jsb_spawn_xml(spawn: SpawnSpec) -> str:
    lat, lon, alt = _geodetic(spawn)
    psi_rad = math.radians(spawn.heading_deg)
    return (
        '<?xml version="1.0"?>\n'
        "<!-- Generated from flightSetup.json spawn (LSZH origin + NED). -->\n"
        '<initialize name="Zurich-inair">\n'
        f'  <latitude type="geodetic" unit="DEG"> {lat:.8f} </latitude>\n'
        f'  <longitude unit="DEG"> {lon:.8f} </longitude>\n'
        f'  <altitude unit="M"> {alt:.3f} </altitude>\n'
        f'  <elevation unit="M"> {DEFAULT_GROUND_ALT_M} </elevation>\n'
        f'  <vt unit="M/S"> {_JSB_VT_MPS:.5f} </vt>\n'
        '  <gamma unit="RAD"> 0.0 </gamma>\n'
        '  <phi unit="RAD"> 0.0 </phi>\n'
        '  <theta unit="RAD"> 0.0 </theta>\n'
        f'  <psi unit="RAD"> {psi_rad:.8f} </psi>\n'
        "</initialize>\n"
    )
```

`29.99232` is five digits after the decimal; `.5f` of `98.4 * 0.3048` is that string.

Replace `python/assets/jsb_spawn.xml` with:

```xml
<?xml version="1.0"?>
<!-- JSBSim IC for PX4 headless Rascal (overrides scene/LSZH.xml via mount).
     In-air ~500 m AGL (MSL = elev 419.2 + 500) at 29.99232 m/s
     (the old 98.4 ft/s, matching FG fg_spawn.env --vc=58.3 kn).
     Attitudes are radians. Latitude and longitude stay degrees.
     Extra AGL gives race startup time before OFFBOARD. -->
<initialize name="Zurich-inair">
  <latitude type="geodetic" unit="DEG"> 47.458159 </latitude>
  <longitude unit="DEG"> 8.548004 </longitude>
  <altitude unit="M"> 919.2 </altitude>
  <elevation unit="M"> 419.2 </elevation>
  <vt unit="M/S"> 29.99232 </vt>
  <gamma unit="RAD"> 0.0 </gamma>
  <phi unit="RAD"> 0.0 </phi>
  <theta unit="RAD"> 0.0 </theta>
  <psi unit="RAD"> 0.0 </psi>
</initialize>
```

Do not edit `python/assets/fg_spawn.env`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd python && python3 -m unittest tests.test_spawn_ic -v`

Expected: PASS. Gazebo heading tests are unchanged and still pass.

- [ ] **Step 5: Commit**

```bash
git add python/fw_sitl/spawn_ic.py python/assets/jsb_spawn.xml python/tests/test_spawn_ic.py
git commit -m "feat(sitl): write the JSBSim IC in metres per second and radians"
```

Defer this commit when the session has no commit permission.

---

### Task 3: Document the SI interior and the imperial boundaries

**Files:**
- Modify: `README.md` (architecture cell for `python/f16/`, and the F-16 known-limits bullet)
- Modify: `UPDATES.md` (new top entry only)

**Interfaces:**
- Consumes: Task 1 (`_dampp` radians, `AEROBENCH_RTOD` index) and Task 2 (`M/S` `29.99232`, `RAD` attitudes, FG `--vc=58.3` unchanged).
- Produces: UPDATES line 3 starts with `## 0.80.0 - SI damping index and JSBSim IC`. README states the boundary list below. No code change.

- [ ] **Step 1: Write the failing check**

There is no unittest for the docs. The check is this script, run from the repo root. It must fail before the edit because line 3 of `UPDATES.md` is still `## 0.79.0`.

```bash
python3 - <<'PY'
from pathlib import Path
updates = Path("UPDATES.md").read_text(encoding="utf-8").splitlines()
readme = Path("README.md").read_text(encoding="utf-8")
assert updates[2].startswith("## 0.80.0 - SI damping index and JSBSim IC")
for needle in (
    "29.99232",
    "--vc=58.3",
    "elevation-ft",
    "FGNetFDM",
    "AEROBENCH_RTOD",
):
    assert needle in readme, needle
print("docs ok")
PY
```

Expected before the edit: `AssertionError` on the `0.80.0` heading.

- [ ] **Step 2: Update the docs**

Insert this block at the top of `UPDATES.md`, above the `0.79.0` entry. Do not edit `0.79.0` or anything below it.

```markdown
## 0.80.0 - SI damping index and JSBSim IC
- `_dampp` takes angle of attack in radians. The paper grid stays every 5 AeroBench degrees via `aerobench_deg` (`57.29578`) inside that function. Trim parity is unchanged. Unused `alpha_deg`, `beta_deg`, `dail`, and `drdr` are gone.
- `assets/jsb_spawn.xml` and `jsb_spawn_xml` use `vt` `M/S` `29.99232` (the old `98.4` ft/s) and radian attitudes. Latitude and longitude stay degrees. FG `--vc=58.3` knots, FGNetFDM speeds in ft/s, and balloon `elevation-ft` stay.

```

In the `python/f16/` architecture cell, append this sentence to the existing SI sentence (do not delete the AeroBench / `f16dynamics` / CSV sentences):

`The damping lookup takes radians and indexes with AEROBENCH_RTOD inside _dampp.`

In the F-16 known-limits bullet (the one that starts `F-16 GCAS is host-only`), append:

`JSBSim spawn vt is 29.99232 m/s. FG --vc stays 58.3 knots. FGNetFDM speeds stay ft/s. Balloon elevation-ft stays feet.`

- [ ] **Step 3: Run the check to verify it passes**

Run the Step 1 script again from the repo root.

Expected: `docs ok`.

Also run: `cd python && python3 -m unittest tests.test_f16_model_si tests.test_spawn_ic -v`

Expected: PASS. This confirms the docs task did not need a code change and the earlier tests are still green.

- [ ] **Step 4: Commit**

```bash
git add README.md UPDATES.md
git commit -m "docs: record SI damping index and JSBSim IC"
```

Defer this commit when the session has no commit permission.
