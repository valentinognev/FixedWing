# Updates

## 0.66.4 - flightSetup.speed_mps is GZ spawn IC, not chase V
- Live `091727` after 0.66.3 set that key to 14: spawn stalled, held z≈+35 m, CPA **12 m under** B0 (no pass). Sweep 08 and morning 083510 spawned at **30** with plant chase 14.
- Revert `guidance.speed_mps` to 30. `gz_spawn_velocity_enu` reads it; chase still uses plant `race_euler` 0.65/14.

## 0.66.3 - flightSetup.speed_mps matches plant 14
- Shipped `flightSetup.json` `guidance.speed_mps` 30→14. Race chase still reads `gz_rc_cessna` `race_euler` (`cruise_thrust` 0.65, `speed_mps` 14).

## 0.66.2 - GZ race_euler: 0.65 thrust + 14 m/s cruise
- 12× 120 s GZ Cessna `pn` grid on cruise thrust / speed P / `kp_elev` / V / pitch-vz D. Jitter = 20 Hz `vz` sign-flips; accuracy = first-circuit 3D ≤5 m.
- Winner `thr=0.65 V=14` (keep speed P 0.07, `kp_elev` 1.5, pitch-vz 0.03): B0 **1.47** / B1 **1.00** / B2 **3.10** m; flips **21** vs baseline 0.80/16 **67** (B2 9.04 m). Soft speed P or 0.55 thrust alone missed the circuit; V=14 with 0.80 thrust departed.

## 0.66.1 - Launcher reaps leftover host camera before bind
- Live: e2e `--no-display` camera kept `tcp://127.0.0.1:5557`; interactive `balloon_camera` died `Address already in use` and never opened a window. `kill.sh` only removes Docker.
- `run_balloon_race.sh` pkill leftover `run_balloon_{camera,control,image_source,gz_pose}.py` before starting panes.

## 0.66.0 - Pitch LPF is a plant gain; 0.80 s lagged B0
- `PlantGains.los_pitch_lpf_tau_s` (default 0.50). `race_quat` / `pure_pursuit_quat` read it.
- Live GZ `pitch_lpf_080_221408` at **0.80 s**: first B0 **6.13 m** (ΔD −5.5) vs **4.94 m** at 0.50; 20 Hz pitch HF 0.41→0.29 but **vz std 2.64→2.75**. Shipped `gz_rc_cessna` `race_euler` stays **0.50 s**.
- History time-series plots keep ≥20 ms spacing so LOCAL_POSITION bursts do not draw a fake velocity ribbon.

## 0.65.0 - Camera overlay shows PX4 arming denied
- Control binds the color ZMQ PUB before engage and forwards STATUSTEXT containing `Arming denied` onto `TargetColor.warn`.
- `balloon_camera` paints that line in red (also on the waiting-for-image placeholder). Cleared once armed.

## 0.64.1 - Off-blob flies next balloon z; steep dive not vz-damped
- Live NED PN `pn_ned_080513`: B0 **0.54 m** / B1 **0.93 m**; B2 overflew **21.7 m** high (XY 5.7 m). After B1, path-hold froze the B1 camera proxy (z≈−20) while tgt_d=+20; HSV only saw B2 ~60 m out. vz-damp then added ~+9° on the dive (`vz≈+5`, cmd −14° vs el −31°).
- Off-blob `z_target=tgt[2]`. Camera proxy reseeds path-hold only when `path_lock_token` matches (HSV drop, same balloon). New balloon discards the old proxy.
- `pitch_with_vz_damp` skips when `|los_el|>8°`.
- Live `pn_ned_000631` after that: B0 **1.06 m** / B1 **1.02 m**; B2 still **16.6 m** high. Path-hold dove to z≈−10 then HSV at el=+15° commanded +20° and stalled. `lookat_clears_alt_step`: while `|Δz|>10 m`, HSV look-at only if |el|≥12° with the needed sign.
- Live `pn_ned_001310` first circuit: B0 **5.23 m** (XY 0.77, ΔD −5.18) / B1 **0.60 m** / B2 **2.80 m**. All three passed; B2 is a hit. Second lap B0 **4.76 m** / B1 **1.77 m**.

## 0.64.0 - PN λ̇ is NED, not body/camera
- `pn`/`apn` rotate HSV LOS into NED with `q_act` and differentiate λ there. Body `ε̇+ω` invented λ̇ under pitch/roll, so inertial LOS rate never held and the plane missed.
- a = N V λ̇ in NED; look-lead is along inertial λ (λ̇=0 keeps look-at the balloon, not boresight). APN gravity is NED a_z − g. `pqr` unused. `apply_homing_law(..., q_act=)`.

## 0.63.1 - Shipped race uses seeker PN
- `python/flightSetup.json` `guidance.homing_law` is `pn` (was `bang` locally / `bias` in older check-ins). Comments match inertial PNG; `apn` is +g not +5°.

## 0.63.0 - Inertial seeker PN
- `pn`/`apn` are classical seeker PN: λ̇_az/el = ε̇_body + (r, q), a = N V λ̇ (APN adds +g on el), θ = (a/V) τ. LPF on λ̇ (τ=0.15 s), |a|≤2g. Lead **τ=0.25 s** after GZ sweep (was 0.15). Env: `FW_PN_N`, `FW_PN_TAU_S`, `FW_PN_LPF_TAU_S`, `FW_PN_A_MAX`.
- `apply_homing_law(..., speed_mps, pqr)`. Default V=30 m/s if omitted; pqr=None → body rates 0 (no IMU cancel until callers pass ATTITUDE). Dropped `_APN_GRAVITY_EL_RAD`.
- Wired `send_chase_setpoint(..., airspeed=history.last_airspeed, pqr=history.last_pqr)`. In-view `pn`/`apn` V is IAS else GS. Default `homing_law` unchanged.
- Race launcher prefixes `FW_PN_N`/`FW_PN_TAU_S`/`FW_PN_LPF_TAU_S`/`FW_PN_A_MAX` onto the control pane (tmux server env is sticky).
- GZ Cessna 10× 120 s `race_euler` sweep (`FW_HOMING_LAW=pn|apn`). Only **N=4, τ=0.25, LPF=0.15, 2g** passed balloon 0 (3D 4.61 m, 1/3 balloons). Other 9 configs: 0 passes. Still ignores boresight; `bang` remains the shipped default.

## 0.62.2 - Damp remaining high-alt look-at bob
- Live after 0.62.1 (`205431`): vz_rms **2.22** vs **2.30** before; 28 sign-flips unchanged. Throttle D was not the leftover driver.
- `bias` extra pitch now scales 0→12° over |el| 0→8° (was a ±12° relay at boresight). Look-at pitch `+= 0.03·vz` (NED +down). Pitch LPF τ 0.20→0.50 s.

## 0.62.1 - In-view thrust no longer tracks camera-el altitude
- In-view `race_*` throttle used `alt_err = 80·sin(el)` while pitch already tracked the same `el` → vz chatter (live `202709`: 27 vz sign-flips / 90 s). Thrust now holds speed with NED `vz` damping (`THRUST_VZ_GAIN=0.05`). Camera proxy still reseeds path-hold after HSV drop.
- `homing_law=fpa_thrust` still adds `sin(el)` thrust bias.

## 0.62.0 - Homing laws in flightSetup.json
- `guidance.homing_law` in `flightSetup.json` (JSONC comments for all 10 options). Parser default `lookat`; shipped value `bias`. `FW_HOMING_LAW` overrides JSON only when set (launcher no longer forces `bias`).
- Wired through `make_body_cmd_controller` / `build_controller` for `race_quat`/`race_euler` only.

## 0.61.0 - Ten camera homing principals; path z-freeze
- `fw_sitl/controllers/cam_homing.py`: 10 in-view HSV principals (`lookat` `pd_lead` `pn` `bias` `el_first` `bang` `area_slow` `fpa_thrust` `filter` `apn`). `FW_HOMING_LAW` (launcher default `bias`, 12° elev intercept + slow on steep el). Tracker `area_px` is wired. No balloon NED range while in view.
- Path-hold freezes `z_hold` at search start (GZ `003837` slid z 57→212 m before lock). HSV drop reseeds from last camera proxy, not `pos_z`.
- Live GZ `./run_balloon_race.sh --gz` 10-law sweep: B0/B1 **&lt;2 m** on `lookat`/`pd_lead`/`bias`/`bang`/`el_first`/`fpa_thrust`/`filter`; B3 **8.4 m** best (`bias` `005338`) / **9.3 m** (`bang`) / **11–18 m** others. `pn`/`apn` never passed balloon 0. 12° `bias` `013130`: **0.54 / 0.88 / 8.59 m**. 12°+area-slow `012817`: **5.92 / 5.39 / 5.88 m** (B3 best; B0/B1 sag). Still not 3× ≤5 m. Pitch stays ±20°.

## 0.60.0 - E2E gate 5 m 3D and |ΔD|
- `assert_race_euler_csv_ok`: after 3D, first `min_passes` must also have `|pos_d − tgt_d| ≤ max_miss_m`. Default `max_miss_m=5.0` (was 10) on the assert and on `run_race_euler_platform_e2e`.

## 0.59.0 - Camera-only in-view speed and altitude
- In-view `race_*` ignore NED `range_m` for z_hold and speed. `z_hold = pos_z − CAM_LOS_ALT_PROXY_M(80) × sin(los_el)` (`los_el` +up). Speed uses synthetic range `slow_range_m × (1 − min(1, |el|/20°))` then `chase_speed_mps`.
- Removed climb `el_frac *= 0.65`; climb and dive slow the same.
- GZ `race_euler` only: `approach_speed_mps` 8.0, `slow_range_m` 280, `kp_elev` 1.5. `race_quat` on that plant unchanged.
- Off-view path-hold still uses caller `range_m`.

## 0.58.0 - Through-center pass + camera-el inhibit
- `check_pass` no longer advances on first `dist ≤ pass_radius_m` (sphere surface). Enter the sphere, then recede `PASS_THROUGH_HYST_M` (2 m) from closest 3D range. `|cam_el| > 12°` (`PASS_CAM_EL_RAD`) keeps homing so a still-diving/climbing blob is not retargeted (GZ 223958 B2). `cam_el_rad=None` does not inhibit.
- `last_closest_ned` is the closest approach of the balloon just passed; CSV `pos_ned` uses that (else plane NED).
- Did not restore 4× fly-by / horizontal gate. 0.57.0 HSV `dir_cam` look-at unchanged.

## 0.57.0 - Camera-tracker homing; pass is 3D sphere only
- `chase_uses_lookat` is True only when HSV has `dir_cam`. Geometric NED is search (path-hold current z + bank onto bearing), not pitch homing. GZ `222435` after balloon 1 commanded +20° pitch with no blob.
- `check_pass` advances only when 3D range ≤ `pass_radius_m`. Dropped 4× closest-approach and horizontal-gate fly-bys (balloon 3 counted as a pass 20 m high).
- `run_balloon_control`: `z_target=pos[2]` off-blob; `dir_body` from camera only.
- Race length 60→120 s (`flightSetup.json` + `run_race_euler_e2e.sh`) so a missed first intercept can turn around.
- Live GZ `223958`: 3 passes at 75.0 / 88.0 / 100.7 s, 3D **9.83 / 9.61 / 9.71 m** (balloon 3 ΔD −7.5 m, HSV lock). Beat `222435` balloon 3 (20.06 m). Still outside the &lt;5 m center-through target. CSV `python/logs/e2e/race_euler_gz_20260824_223958.csv`.

## 0.56.0 - GZ race_euler live e2e (production course)
- `write_race_euler_e2e_setup` / `run_race_euler_platform_e2e`: same 500/200/10 m course as `flightSetup.json`; `cmd_mode=attitude` euler; ≥3 passes; each 3D miss ≤10 m. Opt-in `FW_SITL_E2E=1 ./python/scripts/run_race_euler_e2e.sh`.
- Live GZ Cessna `222435`: software passes at 36.6 / 50.4 / 64.6 s, 3D **9.32 / 9.52 / 20.06 m** (ΔD −2.4 / −0.6 / −20.0). Balloon 3 XY ~1 m, 20 m high. Gate failed on balloon 3. CSV `python/logs/e2e/race_euler_gz_20260824_222435.csv`. Did not beat `165855`.

## 0.55.0 - GZ race_euler center-through guidance
- `q_des_from_los` adds bank load-pitch only on level/climb LOS so a dive is not held high.
- Load-pitch gating is shared (`q_des_from_los`), not GZ-only.
- `PlantGains`: `kp_elev`, `los_roll_slew_rad_s`, `los_roll_lpf_tau_s` (defaults 1.0 / 30°/s / 0.20 s).
- `gz_rc_cessna` `race_euler`: `kp_elev` 1.2, `att_los_max_pitch` 0.35, roll slew 45°/s tau 0.10, approach 10 m/s, `slow_range` 220 m; `FW_P_LIM` ±20. `race_quat` pitch cap on this plant stays 0.26.

## 0.54.3 - Race contract tests after gz default setup
- `--model` without `--gz` is allowed when `flightSetup.json` `sim.platform` is already `gz`; `test_model_without_gz_exit_2` now rejects `--yasim --model` instead of launching a race.
- README chase-controller line names quaternion LOS again (doc contract).

## 0.54.2 - Attitude SID: narrower pitch/yaw chirps, unwrap gt at ±π
- `procedure.json` attitude chirp `f1_hz` pitch 3.0→1.5, yaw 2.0→1.2 (same reason as q/r: do not sweep past useful coherence). `f1_hz.roll` stays 4.0; all attitude `f0_hz` / amplitudes and `window_s` roll/yaw 1.0 / pitch 2.0 unchanged.
- Live attitude `gt`/`px4` are unwrapped onto `cmd`'s 2π branch. PX4 ATTITUDE Euler is (-π, π]; `cmd = trim + excitation` can sit just past π, and a wrapped `gt` was a 2π step that poisons Wiener.

## 0.54.1 - Rates SID: q/pitch 2 s window, narrower q/r chirps, one-click P
- `procedure.json` `window_s` q 0.5→2.0 and pitch 1.0→2.0 (Wiener already deconvolves 2 s segments; the q step plot was truncated at 500 ms). p/r stay 0.5; roll/yaw 1.0; az/w 2.0.
- Rates chirp `f1_hz` q 4.0→2.0, r 2.0→1.2 (energy past useful coherence). `f1_hz.p` stays 8.0; all `f0_hz` and amplitudes unchanged (q/r amp 0.08).
- GZ Cessna one-click: `FW_PR_P` 0.65→0.50, `FW_YR_P` 0.7→0.45. `FW_PR_FF` / `FW_YR_FF` / `FW_RR_*` / I-terms untouched.

## 0.54.0 - Rates SID: yaw wall was eating r, verdict used outlier peaks
- `limit_rates_by_angle(..., channel=)` skips the yaw Euler wall on channel `r`. Heading is not in the envelope abort; the restore rate replaced the r chirp with a one-sided square wave (live GZ). Roll/pitch walls unchanged; p/q overlays still restore r at the yaw cap.
- Overshoot/weak follow `curve_peak` (max of the mean Wiener step) not `peak_mean` (mean of per-window maxes). Live p was `peak=1.40` / `curve=1.03` → was `overshoot`, now `ok`. Printed metrics include `curve=`.
- History plot inserts NaNs across the chirp→inv_chirp settle hole so matplotlib does not draw a diagonal through the 2 s join.
- Live after the limiter fix (`/tmp/fw_calib_20260824_131307`): `p curve=1.026 ok`. q/r still `overshoot` on a climbing mean step (q first-cycle overshoot + high-f attenuation, `|G|≈0.6` at 1 Hz; r Dutch roll + climb bias). Lowering `FW_PR_P` 0.65→0.40 made q `curve` 1.28→1.41; reverted.

## 0.53.0 - GZ Cessna pitch/yaw rate retune from chirp plots
- Live `--layer rates --gz` chirp: 5° pitch/yaw walls turned q/r into limiter square-waves (one-sided cmd, heading walk). Stats said overshoot/slow; history showed the chirp never ran. Walls 10/12/40°, q/r amp 0.08, r `f0` 0.3→0.4 Hz.
- `gz_rc_cessna` pitch rate was high-P/low-FF (`FW_PR_P` 0.9 / `FW_PR_FF` 0.2): history |q|≈½|q_cmd|, |G|≈0.5 at 1 Hz. Now 0.65 / 0.45. Roll `FW_RR_*` unchanged (history already tracked; step mean `curve≈1.03`).
- Yaw: `FW_YR_P` 1.3→0.7, `FW_YR_FF` 0.3→0.5. More P/FF (0.85/0.7) raised the ~2.5 Hz Dutch-roll |G| peak 1.03→1.3; reverted. Residual +r bias still walks into the yaw wall late in the chirp — Cessna uncoordinated climb, not a stats-overshoot fix.
- Keeper plots: `/tmp/fw_calib_20260824_105528`. Do not tune off Wiener `peak_mean` when history is wall-clipped or the mean step `curve` sits near 1.

## 0.52.2 - Attitude overlay no longer aborts on Δalt
- Live `--layer attitude --gz` skipped roll/pitch/yaw on Δalt ≈ 80 m (wings-level, airspeed ~35). Attitude overlay drops TECS the same way rates does; the Cessna climbs to the cap, every axis is skipped, `run_sitl` returns 1 with no CSV → no plots.
- Overlay `check_dalt` is off for `attitude` as well as `rates`. Hold/recapture still uses the 80 m cap.

## 0.52.1 - Step-plot y-axis unit
- Step ylabel is `step (1)`: Wiener \(G\) is DC-normalized, so the curve is dimensionless (same unit as \(Y/U\) for rates/attitude).

## 0.52.0 - Per-channel chirp/sine frequencies for rates and attitude
- `procedure.json` `f0_hz` / `f1_hz` / `f_sine_hz` are per axis on `rates` (p 1–8 Hz / 0.5 Hz sine, q 0.5–4 / 0.3, r 0.3–2 / 0.2) and `attitude` (roll 0.2–4 / 0.3, pitch 0.15–3 / 0.2, yaw 0.1–2 / 0.15). `accel_z`/`vel_z` stay a scalar that expands to every amplitude key.
- `layer_freqs(layer, channel)` / `layer_sine_freq(layer, channel)` look up that axis. Live and dry-run apply the lookup inside the channel loop.

## 0.51.5 - Step plot showed mean curve, box showed mean-of-peaks
- `peak` in the step box / printed metrics is `mean(max(each Wiener window))`. The blue line is `mean(windows)`, so peaks that don't line up look ~1.0 while the box said 1.51. Plot faint per-window traces; box also shows `curve=` (max of the blue mean).

## 0.51.4 - Recover from dive once, then overlay all rate axes
- Live `--layer rates --gz` after 0.51.3: p and q plots were empty (`n=0` `no_data`). CSV had only `r` from t≈62 s. Each axis waited 15 s×2 for wings-level, skipped, and the GZ recapture from the unarmed drop (~62 s) only finished in time for r.
- `procedure.json` `hold_initial_timeout_s` 90. One path-hold until `|roll|/|pitch|≤5°` before the first axis (relock retry included). If that fails, skip the whole overlay. Per-axis recapture stays 15 s.
- Stuck-dive skip no longer burns p then q as warmup.

## 0.51.3 - Rates SID: tighter q/r walls, start from wings-level
- Live `--layer rates --gz` identified `q`/`r` as overshoot (peak 1.61 / 1.50) and the first `p` sample was still the post-fall −40° dive. Bounce-at-wall on a diving airframe distorts the chirp, so Wiener step ID is not usable.
- `procedure.json` `max_angle_deg` pitch/yaw 10°→5° (roll stays 10°). Rates amplitude `q`/`r` 0.15→0.08 so Euler stays inside the wall from trim (p stays 0.15). Chirp+inv_chirp still 20+20 s (≥4 s needed for 2 s Wiener segments).
- New `start_angle_deg` roll/pitch 5°. `_quiet_or_relock` waits for wings-level before overlay (recapture `_hold_ticks` stays envelope-only). Banked/diving attitude skips the axis instead of exciting it.

## 0.51.2 - Rates chirp 10° wall, f0 1 Hz
- `procedure.json` `max_angle_deg` 20°→10° (roll/pitch/yaw).
- Rates chirp `f0_hz` 0.3→1.0 (less dwell at slow rates that integrate into large Euler). `f1_hz` stays 8. Sine `f_sine_hz` unchanged.

## 0.51.1 - GZ rates still aborted; 20° wall + restore unused axis
- Live `--layer rates --gz`: `p` finished; `q`/`r` still skipped on Δalt 80 m. Uncommanded Euler (q=0 during p, etc.) walked because the limiter only negated a same-sign rate.
- `procedure.json` `max_angle_deg` 30°→20°. At the wall, a zero rate on that axis becomes a restoring rate of the excitation amplitude (opposite the Euler error).
- Live `--layer rates` overlay does not abort on Δalt (TECS is off; climb is expected). Roll/pitch/airspeed abort still apply. Hold/recapture still uses the 80 m Δalt cap.
- Envelope abort stays 40° / 80 m.

## 0.51.0 - Reverse rates SID at Euler wall
- `overlay.limit_rates_by_angle` flips `p`/`q`/`r` (does not zero) when Euler is at/beyond `MaxAngle` and the rate still drives into the wall; yaw error is `remainder(yaw−trim_yaw, 2π)`. New `AxisCommand`; `cmd` tracks the flipped axis when it matched.
- Live `--layer rates` applies that limiter after `axis_command` / `last_att_rad` and before `send_attitude_rates`. Caps from `procedure.json` `max_angle_deg` (10° as of 0.51.2). Logged/sent `cmd` is post-limit; `measured` stays the aircraft. Attitude layer does not call it. Envelope abort stays 40° / 80 m.

## 0.50.2 - GZ rates SID envelope was unflyable
- Live `--layer rates --gz` recaptured as designed but skipped p/q/r: first overlay tripped at pitch −25.1° (post-fall dive), then every retry hit Δalt ±30 m because `z_hold` stayed at the post-arm lock (~68 m) while rate overlay drops TECS and the Cessna climbs.
- Envelope pitch limit 25°→40° (same as roll). Δalt 30 m→80 m so a rates SID without TECS can finish an axis.
- After recapture, re-lock path origin + `z_hold` to the current pose. If the envelope is still not quiet, skip that axis instead of overlaying anyway.

## 0.50.1 - SID sine waveform review fixes
- New live-path regression test: `--waveform sine` on `run` (no `--dry-run`) is asserted to fly only `settle`/`sine` CSV segments (no chirp/inv_chirp leak-through) and to log `cmd` matching `A*sin(2π f_sine t)` sample-for-sample (rates layer, channel `p`: `A=0.15`, `f_sine=0.5` Hz).
- `run_offline_demo`/`run_sitl` read `args.waveform` directly; dropped the `getattr(args, "waveform", "chirp")` fallback now that every caller (CLI parser and hand-built test `Namespace`s) sets it.
- `run_sitl` docstring no longer says "Live chirp SID" / "abort flushes CSV and aborted=True": documents that `--waveform` picks `PHASES` vs `SINE_PHASES`, and that an envelope abort prints the tripping limit, recaptures, and retries the same axis up to `MAX_AXIS_RETRIES` (3) attempts total before skipping it (`aborted=True` only then).
- Envelope-abort integration tests now also assert the measured numeric value (e.g. `"60.0"`) is in the stderr message, not just the axis name, so a regression to `Envelope abort during roll: None` would fail.

## 0.50.0 - SID sine waveform and envelope retry
- `run`/`run --dry-run` take `--waveform chirp|sine` (default `chirp`); `sine` is an alternative scenario, not a phase tacked onto the chirp.
- `procedure.json` `sine_phases`: settle 3.0 s / sine 60.0 s / settle 2.0 s per axis (≥60 s of excitation, not concatenated with chirp/inv_chirp). Per-layer `f_sine_hz`: rates 0.5, attitude 0.3, accel_z/vel_z 0.2 Hz. `cmd = A * sin(2π f_sine t)` during the `sine` segment, same trim overlay as chirp.
- `analyze`/`log_io.select_excitation`: a channel with any `sine` rows uses only those for the FRF/step response; otherwise falls back to chirp+inv_chirp as before.
- Live `run`: an envelope trip during an axis no longer ends the whole calibration. It prints which limit tripped and its value, recaptures path-hold, and retries that axis from its first phase (dropping the failed attempt's rows) up to 3 times before skipping to the next axis with a warning.
- `hints.json` `aborted: true` only if an axis was skipped after exhausting its 3 retries; a recovered retry leaves `aborted: false`.

## 0.49.0 - In-view LOS speed/thrust
- Homing `v_cmd` slows on steep LOS elevation (dive more than climb); in-view thrust uses `range·sin(el)` as Δz, not balloon bookkeeping Z.
- GZ `race_euler` attitude/euler live `165855`: pass 3D 9.7 / 9.5 / 9.8 m (Z −2.1 / −2.2 / −9.6). Plant energy keys left at the `race_quat` copy (lower `min_thrust` did not beat that on later dirty-IC runs).

## 0.48.1 - Calibration launcher review fixes
- `runner.layer_amplitude` / `analyze._amplitude` now look up amplitude within the given `layer`'s `procedure.json` block instead of a flattened `amplitude_map` global (removed). A flattened map could silently collide on a shared key or mask a layer/channel mismatch (e.g. `layer_amplitude("rates", "p", "thrust")` used to return accel_z's `thrust` amplitude instead of erroring); both now raise `ValueError` when the layer has no matching amplitude key. Thrust `0.08` stays duplicated under `accel_z`/`vel_z` in `procedure.json` — same value, no collision.
- `run`'s `--out-dir` defaults to a fresh `runner.default_out_dir()` (`/tmp/fw_calib_<utcstamp>`), not `.` (which dropped CSV/PNGs under wherever `python/` happened to be the cwd). Dry-run and live both use it; tests that already pass `--out-dir` are unaffected.
- README: live run examples are `--layer rates` / `--layer attitude` (`--gz` etc. optional); `accel_z`/`vel_z` have no live GT wired (`gt`/`px4` are `nan`, hints read `verdict=no_data`) so they're `--dry-run`-only for now. `_fft.png` is written unless the log is too short for a Welch segment, not unconditionally.
- `tests.test_calibration_procedure` fixtures that mutate a copy of the shipped `procedure.json` now go through `strip_jsonc` (matching `load_procedure`) instead of raw `json.loads`, so `//` comments in the shipped file stay legal for those tests.
- New coverage: `tests.test_calibration_run_sitl.TestRunSitlStartsResolvedSim` pins `no_sim=False` calling `kill_docker(target=...)` / `start_sim(..., extra_args=...)` from the resolved `CalibrationSim` (e.g. `--gz --model advanced_plane`) — no live Docker, `start_sim`/`kill_docker` mocked.

## 0.48.0 - Calibration launcher (multi-plant SID)
- Root `./run_control_calibration.sh` (`chmod +x`) execs `python3 -m controlCallibration run` from `python/` (`set -euo pipefail`). One `--layer` per invocation.
- Chirp numbers (rate, phases, per-layer f0/f1/A, windows) live in `python/controlCallibration/procedure.json` (JSONC `//` allowed).
- Live plant + sim from `python/flightSetup.json` `sim.platform`/`sim.gz_model`; CLI `--gz`/`--yasim`/`--viz`/`--jsbsim`/`--model` override. No `--xplane`. Does not write plant JSONC.
- Analyze always writes `<stem>_<ch>_history.png`, `_fft.png`, `_step.png` plus `hints.json` and a printed metrics table. Interactive `plt.show(block=True)` unless `--no-plot`.
- Host unittest `tests.test_calibration_cli.TestRootShim`.

## 0.47.1 - Control calibration review fixes
- `stepresponse.step_calc` scale fix: relative Wiener regularizer (`|H|² + eps·mean|H|²`), each segment normalized by `max|sp_seg|`, and `resp / mean_steady` instead of the partial `resp * (2 - mean_steady)`. Reconstructed DC gain of a unity-DC plant went 0.79 → 1.00, so `overshoot` can fire at all: a ζ=0.15 ringer at the rates operating point reads `peak_mean` 0.97 → 1.29 (was verdict `weak` → "raise P" on a ringing airframe). Spec thresholds 0.85 / 1.25 kept. Regression: `tests.test_calibration_step.TestZetaSweepAtRatesOperatingPoint`.
- Peak estimation still *compresses* large overshoots (ζ=0.3 at 3 Hz reads ~1.17 against a true 1.37): a 0.5 s window plus 2 s Wiener segments cannot resolve the full first peak. Treat `peak_mean` as "does it ring", not as a calibrated overshoot percentage. Because `y_correction` normalizes DC to 1, `weak` (<0.85) now essentially cannot fire on a real log — a sluggish loop shows up as `slow` (latency), not `weak`.
- `runner.capture_trim`: live attitude/rates trim is measured (`FlightHistory.last_att_rad` / `last_pqr` + plant `cruise_thrust`), captured after engage and re-captured per axis. `run_sitl` no longer chirps around the hardcoded `DEFAULT_TRIM` roll=pitch=yaw=0 (a wings-level, due-north attitude step the instant OFFBOARD attitude took over). `--dry-run` keeps the zero trim.
- `overlay._attitude_command` adds to trim (`axis = trim.axis + value`, spec `cmd = trim + A·sin(φ)`) instead of replacing it; non-chirped Euler stay at trim (a roll chirp no longer commands yaw=0). Logged `cmd` is the sent channel scalar, so it carries the same trim offset as the `gt` Euler angle it is deconvolved against.
- `overlay.axis_command` clips thrust: `clip(cruise + chirp, min_thrust, max_thrust)`, defaults 0.22 / 1.0, overridable per plant (`run_sitl` passes the loaded plant's limits). `--dry-run` can no longer log an out-of-range thrust.
- `runner.hold_until_quiet`: inter-axis recapture holds until `envelope_ok` has been true for 1.0 s consecutive (timeout 15 s, warn and continue) instead of a fixed 2 s that could hand the next axis a still-rolling aircraft.
- `hints`: `overshoot` on a channel with no P/FF key (`yaw`, `az`/`w` + `pitch`, `az`/`w` + `thrust`) fell through to `hints: []`. Now falls back to the first key with the sense inverted from weak/slow (`down` on a gain, `up` on a `*_tc`).
- Live-path caveat: `gt` and `px4` are the **same** MAVLink ATTITUDE sample (plan v1 is JSBSim single-source), so `gt` is *not* FDM truth and `--response px4` cannot differ from `--response gt` on a live log. Only offline CSVs with a genuinely separate GT source can.
- `run_sitl` now has host tests (`tests.test_calibration_run_sitl`): trim/send wiring, recapture-per-axis, and envelope abort → CSV flush + `aborted: true`, all on fakes (no Docker, no MAVLink). Dry-run `accel_z`/`vel_z` are pinned too.

## 0.47.0 - Control calibration chirp SID
- `controlCallibration.stepresponse`: Wiener deconvolution `step_calc` / `step_stats` / `default_min_input` (20% of |A|; no Betaflight 0.25–20 clip; no LOWESS).
- `controlCallibration.analyze`: CSV → Wiener step PNG, Welch Bode PNG, `hints.json`; `main_analyze` CLI (`--layer`, `--inject`, `--response gt|px4`).
- `controlCallibration.__main__`: `python -m controlCallibration run|analyze`. `run` extends `parse_run_args` with `--dry-run`/`--out-dir`/`--no-sim`/`--udp`; missing `--inject` on `accel_z`/`vel_z` exits 2 (argparse, before any sim/MAVLink touch).
- `controlCallibration.runner.run_offline_demo` (`--dry-run`): 50 Hz synthetic schedule (`iter_schedule` + `axis_command`) → CSV → `analyze_log`. Never imports Docker/MAVLink — the agent-safe / test-gating path. Default trim zeros + thrust 0.62.
- `controlCallibration.runner.run_sitl` (live): connect/engage/path-hold copied from `fw_sitl.straight_flight_core.run_locked_line_hold` (JSBSim only, `--no-sim`/`--udp`); path-hold between axes, `send_attitude_rates`/`send_attitude_target` from `AxisCommand` during chirp/settle; envelope abort recaptures path hold, flushes CSV, analyzes with `aborted=True`. `--inject` on `rates`/`attitude` is dropped with a one-line note (`overlay.axis_command` rejects it there).
- Host unittest `tests.test_calibration_step`, `tests.test_calibration_analyze`, `tests.test_calibration_cli`.

## 0.44.0 - Log chirp and Welch FRF
- `python/controlCallibration/` (spelling locked): `log_chirp` / `inv_log_chirp` (log-sine phase) and Welch `estimate_freq_response` (PIDToolBox copy, no pidbox import).
- Host unittest `tests.test_calibration_chirp`.

## 0.43.0 - race_euler controller
- Selectable `guidance.controller=race_euler`: same LOS/path as `race_quat`, closes wrap(φ_des−φ)/wrap(θ_des−θ) every tick including in-view (no cascade reset).
- Plant JSONC `controllers.race_euler` copies `race_quat` outer gains; PP-only still filled from sibling `pure_pursuit_quat`.
- `cmd_mode=rates` is identical between `race_euler` and `race_quat`: `send_attitude_rates` dispatches `cascade_out.body_rates`, derived from `(q_des, q_act, roll_tc/pitch_tc, groundspeed)` — never `q_cmd`, never cascade I-state. The in-view behavior difference between the two controllers only shows up under `cmd_mode=attitude` (cascade `q_cmd`, I-state kept, vs `race_quat`'s open-loop `q_des` + `reset()`). Regression test: `tests.test_controllers_registry.TestRaceEulerLos.test_cmd_mode_rates_race_euler_matches_race_quat`.
- Host-tests-only / unflown: no live SITL race has exercised `race_euler` yet (same caveat as 0.42.0's cascade). Copied `race_quat` outer gains are a tuning starting point, not a verified-in-flight set.

## 0.42.0 - PX4 Euler attitude cascade
- Inner stepper is PX4-style Euler PID (`px4_att_cascade`); outer chase unchanged.
- `guidance.attitude_format` quat|euler.
- `cmd_mode=rates` sends body rates from Euler φ̇θ̇ψ̇.
- Host-tests-only / unflown: no live SITL race has exercised this cascade yet.
- Plant controller blocks now require `roll_tc`/`pitch_tc` (used by `Px4FwAttCascade` coordinated-turn rates).

## 0.41.6 - GZ Cessna look-at no longer stalls
- Cause (live `135412`): in-view `race_quat` capped pitch at 40° (`att_los_max_pitch=0.7`). Blob-high look-at bled GS 14→8 m/s by t=24; lock drop at t=25 spun west to z=+94. 12 s pre-arm health wait also put them ~200 m west (course 242°).
- `gz_rc_cessna`: look-at/path pitch 15° (`0.26`), `FW_P_LIM_MAX=15`, cruise thrust 0.62→0.80, `min_thrust` 0.22→0.50.
- Verify `--gz` 60 s `/tmp/balloon_race_20260822_140003.csv`: 1 pass (balloon 0, XY 0.9 m at t=50), mean GS 18 m/s, max 8 s sink 11 m (was stall to z=+94 on `135412`). Ended 43 m from balloon 1.

## 0.41.5 - GZ race_cam no longer stalls realtime
- Cause (`133939`): after 0.41.4 actually subscribed `race_cam`, gz-sim rendered 640×480 with default 4× MSAA + GUI frustum, 25 km GUI far clip, and blocking ZMQ copies on the gz.transport thread. 60 s wall covered ~160 m (~3 m/s vs 16 m/s cruise).
- `race_cam`: `visualize=false`, `anti_aliasing=0`, far 5 km→2 km. GUI follow far 25 km→4 km. Camera/pose bridges drop on `ZMQ NOBLOCK`; RGB passthrough skips a numpy copy; pose PUB capped at 20 Hz.
- Verify `--gz` 60 s `/tmp/balloon_race_20260822_134733.csv`: net 496 m, GS ~16 m/s after settle (was 163 m / crash z=133 on `133939`); end XY 83 m from balloon 0, still climbing. No pass in 60 s (spawned ~500 m SW, heading lock 247°).

## 0.41.4 - GZ camera/pose docker paths after platforms/ move
- `--gz` image pane died: `docker exec` still ran `/opt/fixedwing/python/fw_sitl/gz_camera.py` (and `gz_pose_bridge.py`); 0.40.0 moved them to `platforms/gz/`. Camera ZMQ never published → balloon_camera stayed on "waiting for image".

## 0.41.3 - JSBSim race_quat 3-cycle miss/speed tune
- Baseline `131011` (0.41.2): 1 pass / 60 s, XY 18.5 m, ΔD −26 m, ~15 m/s (kp=2, 45° bank, approach 12 @ 220 m crawled and over-banked).
- Cycle 1 `131644` (kept): kp 2.0→1.5, max_roll 0.79→0.70, approach 12→16, slow_range 220→140. 3 passes, mean XY 7.1 m, ~18 m/s (final 21 m/s). Balloon 2 still ~30 m high.
- Cycle 2 `132038` (reverted): look-at `kp_elev=1.4` → 0 passes, flew off NE. Cycle 3 `132342`: milder load-pitch 0.18, 3 passes but worse than cycle 1 (mean XY 9.1 m) → restore `LOS_LOAD_PITCH_RAD=0.35`.

## 0.41.2 - JSBSim look-at lead + wipe NED alt mix
- Cause (JSBSim `125350`): `kp_heading=1` made roll≈body az so ~9° residual never closed (balloon 0 XY 6.9 m, ΔD +7.2 m); balloon-1 intercept sat at 35° bank (XY 17.8 m). Inner `FW_PR_P=0.05` left pitch_cmd 25° vs meas ~7°.
- `jsbsim_rascal` / `_viz` `race_quat`: `bank_kp_heading` 1.0→2.0, `max_roll` 0.62→0.79 (~45°), approach 15→12 m/s, `slow_range` 180→220 m. Shared inner `FW_PR_P` 0.05→0.10, `FW_PR_FF` 0.4→0.55.
- Body LOS: vertical (horiz≈0) azimuth is 0, not NED yaw. Drop unused look-at `kp_alt`/`z_ned` mix, `los_kwargs.kp_alt` pop, and dead `visual_lock_kp_alt` plant field.

## 0.41.1 - Body seeker keeps residual azimuth
- Cause (JSBSim `124324`): 5° LOS heading deadband zeroed roll at cam_az≈−4° and subtracted 5° from larger errors (20° blob → 15° bank); XY miss 18.9 m.
- `q_des_from_los` default deadband 0 (optional db still zeros below threshold, no subtract).

## 0.41.0 - Body-frame seeker homing
- In-view chase for `race_quat` and `pure_pursuit_quat`: `dir_cam` → body via `camera.azimuth_deg`/`elevation_deg` (`CameraModel.with_mount` for a later gimbal); geometric LOS is NED→body by attitude.
- `q_des_from_los` banks/pitches vs body +X (not NED elevation). PP `û`/`v̂` in body, `a_des` rotated back to NED for `attitude_from_accel`.
- `flightSetup.json` mount comments; control passes `dir_body`.

## 0.40.1 - In-view race_quat ignores balloon Z
- Unassisted / in-view `race_quat`: attitude from LOS elevation only — no `z_target`, `kp_alt` / `visual_lock_kp_alt`, or ΔZ pitch-cap branch.
- Control passes `z_target=None` when `use_lookat`; thrust hold stays at current plane Z while in view.
- Path-hold and assisted geometric LOS unchanged (still use balloon NED).

## 0.40.0 - Platform folders under fw_sitl/platforms
- Move plant JSONC + backend modules into `platforms/{jsbsim,yasim,gz,xplane}/` (drop top-level `plants/`); `fg_camera` under `yasim/`.
- `load_plant_gains` resolves `platforms/<family>/{plant_id}.jsonc` via plant-id prefix; imports updated across runners/tests/scripts.

## 0.39.12 - YASim 10-cycle bank tune (miss-safe)
- Live tuner `python/scripts/tune_yasim_bank.py`: ≥10 YASim races; keep only if bank shake drops and miss stays within baseline budget.
- `yasim_rascal` (both controllers + inner): `bank_kp_heading` 0.85→0.78, `max_roll` 0.38→0.36, `kp_alt`/`visual` 0.022→0.032, `climb_thrust` 0.018→0.028; `FW_R_TC` 0.70→0.80, `FW_RR_P` 0.11→0.095, `FW_RR_I` 0.12→0.10.
- Round-2 best `111346` vs baseline `111019`: shake 28.5→6.5, sat 43%→10%, roll corr→0.50, closest3d 36.6→19.5 m, |dD| 35.4→19.1 m, passes 1→3.
- Confirm `113014`: shake 18.8, sat 22%, 3 passes, closest_xy 1.4 m (ΔD still run-to-run noisy ~36 m); shake still below baseline.

## 0.39.11 - Damp YASim bank oscillation
- `yasim_rascal`: `bank_kp_heading` 1.05→0.85, `max_roll` 0.45→0.38, `pid_kp` 0.55→0.45 / `ki` 0.10→0.08 / `kd` 0.08→0.10; inner `FW_R_TC` 0.55→0.70, `FW_RR_P` 0.14→0.11, `FW_RR_I` 0.18→0.12.
- Verify `102021` vs `101639`: roll sat 43%→28%, cmd rate p90 18.9→11.5 °/s, meas std 24→19°; 2 gate passes.

## 0.39.10 - FG GT altitude frame (phantom ~40 m ΔD)
- Cause: `_gt_pose_from_telnet_raw` used balloon-frame `z_hold_true` as the aircraft datum at settle → double-counted (ac−elev); CSV ΔD≈40 m on visual co-altitude hits. Separately, post–re-place `(ac_ft−elevation-ft)` often disagreed ~40 m with just-placed models → phantom climb, LOS off camera, YASim heading oscillation (`100130`/`100337`).
- Fix: GT Z datum is raw `z_hold` (`z_ref_at_settle`); after successful FG re-place set `z_hold_true=z_hold` (elev-ft only if re-place failed).
- Verify `100934`: pass ΔD **0.9 m** (was ~40); `yasim_rascal` bank_kp 1.05 / pid_kp 0.55 / `FW_RR_P` 0.14 (recover tracking after Z fix).

## 0.39.9 - Soften YASim race_quat shake
- Live baseline `093801`: roll cmd–meas corr 0.04, sat@22.5° 45%, pitch rate p90 24°/s, 1 pass miss 40 m ΔD.
- `yasim_rascal` (both controllers): `bank_kp_heading` 1.2→0.9, `max_roll` 0.40→0.45, `pid_kp` 0.65→0.50 / `kd` 0.06→0.08, alt/visual `kp_alt` 0.028→0.022, `climb_thrust` 0.025→0.018, approach 20→22 m/s; inner `FW_R_TC` 0.50→0.55, `FW_RR_P` 0.14→0.12.
- Verify `094707`: corr 0.20, sat 6%, pitch rate p90 14°/s, track err p50 7.5° (was 26°), 3 passes; 3D miss still ~39 m ΔD (XY 1–6 m) — later shown as bookkeeping bug (0.39.10).

## 0.39.8 - Image pane crash (waiting for image)
- Cause: `run_balloon_image_source` always imports `xp_camera`, which imported missing `request_data_streams` → image pane died on every plant; camera stayed on "waiting for image".
- `xp_camera`: use `request_local_position` + ATTITUDE interval (same as FG/synth).
- Launcher: `PLOT_TIMEOUT` truncates float `duration_s` (`60.0`) so bash arithmetic does not fail.

## 0.39.7 - Live race_quat e2e (opt-in)
- `fw_sitl/race_e2e.py` + `tests/test_race_quat_e2e.py`: per-platform live SITL with `guidance.controller=race_quat` (`jsbsim`|`viz`|`yasim`|`gz`); gated by `FW_SITL_E2E=1`; CSV must have `end_*` and ≥1 pass.
- Runner: `python/scripts/run_race_quat_e2e.sh` (`FW_SITL_E2E_PLATFORMS`, `FW_SITL_E2E_DURATION_S`).

## 0.39.6 - Per-platform race_quat tests
- `tests/test_race_quat_platforms.py`: for each `KNOWN_SIM_PLATFORMS` plant (plus gz `advanced_plane`), load `race_quat` gains, build controller, assert in-view `los` / off-screen `path`, and setup `sim.platform`+`controller=race_quat` E2E.

## 0.39.5 - Drop xplane from sim.platform menu
- `KNOWN_SIM_PLATFORMS` / `flightSetup` comments: `jsbsim`|`viz`|`yasim`|`gz` only; `sim.platform=xplane` and `./run_balloon_race.sh --xplane` exit with a clear error (plant code remains in-tree).

## 0.39.4 - Race duration under sim.duration_s
- `sim.duration_s` sits next to `platform` / `gz_model` (CLI `--duration` overrides); still mirrored onto `guidance.duration_s` for control. Legacy `guidance.duration_s`-only setups still load.

## 0.39.3 - gz_model availability errors
- `validate_gz_model_for_platform`: unknown model on `gz`, or any `--model`/explicit model on non-`gz`, raises a clear available-list message; launcher prints `Error: …` and exits 2.

## 0.39.2 - flightSetup sim.platform + CLI override
- Top-level `sim.platform` (`jsbsim`|`viz`|`yasim`|`gz`|`xplane`) and `sim.gz_model`; `guidance.duration_s` remains race length.
- `run_balloon_race.sh` defaults from setup; `--viz`/`--yasim`/`--gz`/`--xplane`/`--model`/`--duration` override via `resolve_race_sim`.

## 0.39.1 - flightSetup option comments (JSONC)
- `flightSetup.json` / `.e2e.json`: `//` comments on multi-option guidance fields (`laps`, `duration_s`, `cmd_mode`, `controller`).
- `load_flight_setup` strips JSONC via `strip_jsonc` (same scanner as plant files).

## 0.39.0 - Selectable chase controllers
- `guidance.controller` in `flightSetup.json` (`pure_pursuit_quat` default; unknown id hard error).
- `python/fw_sitl/controllers/`: registry + `race_quat` (LOS `q_des_from_los`) and `pure_pursuit_quat` (PP `a_des`→attitude→governor).
- Plant JSONC: nested `controllers.race_quat` / `controllers.pure_pursuit_quat`; `load_plant_gains(plant_id, controller=...)` merges selected block.

## 0.38.1 - X-Plane plugin builds; fetch px4xplane
- `fixedwing_balloons`: place via `XPLMWorldToLocal` + instance pose refresh; Makefile uses Laminar `XPSDK411.zip` and `--allow-shlib-undefined` (no link to libXPLM).
- `runSimXplaneCessna.sh` prefers `plugin/64/lin.xpl`, else builds with clean `/usr/bin/g++` (fail loud).
- `fetch_px4xplane.sh` → `assets/xplane/px4xplane/` (gitignored with `*.xpl`).

## 0.38.0 - X-Plane 12 demo Cessna balloon-race plant
- `./run_balloon_race.sh --xplane`: bind-mount host `$XP12_HOME` (`~/X-Plane 12`) into `px4-noble-xplane-cessna` on image `px4-noble-sim-ros`; LOWS spawn (~500 m AGL); plant `xplane_cessna172` (trim 40 / approach 28 m/s, `max_roll` 0.40).
- Runtime airframe `assets/xplane/5001_xplane_cessna172`; `fetch_px4xplane.sh` + `fixedwing_balloons` UDP plugin (49091); mss `--mode xp`; pose pane via plugin `pose_query`.
- Mutually exclusive with `--viz`/`--yasim`/`--gz`. `kill.sh --xplane` / `--all`. Spec/plan under `docs/superpowers/`.

## 0.37.1 - PP vertical ψ^c, stalled v̂, honest in-view tests
- Nearly-vertical LOS: `ψ^c` is velocity heading (`atan2(vy,vx)`) or `yaw_act`, not `atan2(0,0)`.
- `‖v‖ < 0.5`: reuse `_last_v_hat`; if never had velocity, skip PP and hold last commands, else yaw-heading `v̂` (no invented NED north). `normalize()` default unchanged.
- `visual_lock` unused on the PP branch. In-view tests renamed/doc'd for accel-driven pitch (not look-at / alt-loop climb).
- `load_plant_gains` raises `ValueError` if JSONC `plant_id` ≠ requested id.

## 0.37.0 - Pure-pursuit chase + JSONC plants
- Per-plant JSONC under `python/fw_sitl/plants/` (`jsbsim_rascal`, `jsbsim_rascal_viz`, `yasim_rascal`, `gz_rc_cessna`, `gz_advanced_plane`, `xplane_cessna172`). `plant_loader` (`strip_jsonc` scanner) → `PlantGains`; `load_plant_gains` reads `{id}.jsonc`; unknown id `KeyError`; missing file `FileNotFoundError` (chase fallback `jsbsim_rascal`).
- On-target chase: `pure_pursuit_accel` `a_des=k(u_hat-v_hat)` → `attitude_from_accel` (polar/geom) → `thrust_energy` (quadratic `D`, `T=(m a_par+D+mg sinγ)/cosα`, `SpeedGovernor` 0.8/1.1/1.2 with exclusive recover/reduce/ramp). `last_law` `pp_polar`/`pp_geom`.
- Path-hold unchanged (`path`). Sent yaw stays actual; roll/pitch LPF+slew kept. No `q_des_from_los` / `chase_speed_mps` on the PP branch.

## 0.35.27 - FG balloon collision actually off
- Live YASim `201819`: still crashed on a hit (roll 218°, GS p90 166). Root XML `<enable-hot>` and a post-`add-model` `solid=0` do not clear `SG_NODEMASK_TERRAIN_BIT`.
- Wiki form at end of each balloon XML: `<animation><object-name>balloon</object-name><enable-hot>false</enable-hot></animation>`. `add-model` request itself now has `enable-hot: 0` (load-time). Restart FG so `FG_ROOT` copy refreshes.

## 0.35.26 - FG balloons are visual-only
- `--viz`/`--yasim` `add-model` balloons were solid/hot: YASim ground cache rode the 10 m sphere (pass ~10 m under, pitch hunt). XML `enable-hot=false`; spawn sets `solid` and `enable-hot` false. Same fly-through as Gazebo. 3D plot balloon dots are not pickable.

## 0.35.25 - Vertical homing when low
- Live YASim `195358`: 3 passes 25 m *above* then a 22 m sag (pitch cmd ±20°, 54 zc). Headless `194912` grazed the 10 m sphere from 7 m under — HSV look-at had zeroed `kp_alt`.
- `q_des_from_los`: fade bookkeeping-Z mix with range (out by 100 m so close-in `los_el` is not double-counted); extra nose-up `LOS_LOAD_PITCH_RAD` in a bank so the intercept turn does not sag ~10 m. Headless `visual_lock_kp_alt` 0→0.020. Pitch LPF 0.20 s / 30°/s (same as bank).
- Energy on final: JSBSim/viz approach 12→15 m/s, `climb_thrust` 0.012→0.020; `--gz` approach 10→12, climb 0.012→0.018.

## 0.35.24 - Smooth noisy roll commands
- Live YASim `194313`: roll cmd sat ±23° 76% of the time; cmd–meas corr 0.18 (plane not tracking). `--viz` 148 cmd sign-flips vs 10 in measured roll — HSV LOS was banging the 60°/s slew cap. `--gz` heading kp 2.0 amplified az jitter.
- Chase bank: 5° heading deadband, 0.2 s LPF, 30°/s slew, state kept across LOS↔path (HSV drop no longer resets and jumps ±max). `gz_rc_cessna.bank_kp_heading` 2.0→1.4. YASim `FW_RR_P` 0.07→0.14, `FW_RR_FF` 0.35→0.48.
- Verify headless 60 s `/tmp/balloon_race_20260821_194912.csv`: 3 passes, miss 9.9 / 9.3 / 9.9 m. Roll cmd sign-flips 43→1; max cmd rate 60→30 °/s.

## 0.35.23 - Calibrate `--yasim` miss
- Live baseline 60 s `/tmp/balloon_race_20260821_192354.csv`: 3 passes, XY 5.8 / 6.0 / 0.4 m but 3D miss 27.1 / 31.0 / 29.0 m (ΔD 26–30 m, still climbing on final). Bank sat at 22.9°. GS p50 22 vs trim 28.
- `yasim_rascal`: `bank_kp_alt`/`visual_lock_kp_alt` 0.022→0.028, `climb_thrust_per_m` 0.012→0.025, `min_thrust` 0.28→0.18, `slow_range_m` 220→280, `speed_thrust_per_mps` 0.04→0.06. Geometric LOS inside 120 m with |ΔZ|>6 m uses `att_los_max_pitch` (HSV dropped on every YASim pass row).
- Approach 25 m/s lost the course (0 passes, `193520`). Harder altitude thrust (`climb` 0.040) cut to one pass (`193808`).
- Verify `/tmp/balloon_race_20260821_193224.csv`: 3 passes, miss 26.5 / 26.4 / 25.8 m (XY 5.4 / 6.8 / 0.4 m). Remaining 3D miss is the 28→20 m/s energy flare (~20 m Δh), not heading.

## 0.35.22 - Calibrate `--viz` and `--gz` miss
- `--viz` table `jsbsim_rascal_viz` (headless stays `jsbsim_rascal`). HSV look-at keeps `visual_lock_kp_alt` (=`bank_kp_alt`) so FG GT ΔD mixes into pitch; headless synth still zeros it. Chase thrust/heading use FG GT velocity, not EKF GS (live pickle p50 was a frozen 37.7 m/s).
- `--gz` `gz_rc_cessna`: approach 12→10 m/s, `slow_range` 150→180, `max_roll` 0.50→0.55, cruise thrust 0.55→0.62, `speed_thrust_per_mps` 0.04→0.07, `min_thrust` 0.28→0.22; `visual_lock_kp_alt=0.030`.
- Live `--viz` 60 s `/tmp/balloon_race_20260821_190857.csv`: 2 passes, miss 9.54 / 9.83 m (ΔD 3.5 m; prior `184447` 9.84 / 11.36 with ΔD 8–11 m). Balloon 2 16 m at t=60. GS p50 18.9.
- Live `--gz` 60 s `/tmp/balloon_race_20260821_190600.csv`: 2 passes, miss 9.43 / 9.45 m (prior `190029` one pass 9.57 m, GS p50 13.4 → 14.6). Balloon 2 started.

## 0.35.21 - Chase speed blend on all plants
- Shared `chase_speed_mps`: cruise beyond `slow_range_m`, blend to `approach_speed_mps` on final, extra cut when heading error is large. Thrust P (`speed_thrust_per_mps`) tracks the command (overspeed brake). Wired in attitude and velocity chase; race passes horizontal `range_m`.
- Tables: jsbsim 18→12 m/s @ 180 m (`min_thrust` 0.22, gain 0.05); yasim 28→20 @ 220 m; gz cessna 16→12 @ 150 m; advanced_plane 20→14 @ 160 m.
- Live JSBSim 60 s `/tmp/balloon_race_20260821_183220.csv`: 3 passes, miss 10.4 / 9.9 / 9.3 m (b1+b2 unassisted). GS p50 22.6 (was 27). Prior no-blend `180642`: 18.5 / 13.0 / 11.5 m.

## 0.35.20 - JSBSim drop fly-by; max_roll 0.62
- Live headless 90 s `/tmp/balloon_race_20260821_175115.csv`: one pass, miss 38.2 m (balloon 0, assisted); balloon 1 closest 76 m. Bank sat at 28.6°; GS p50 27 m/s vs trim 18. Fly-by used R(18, 0.50)≈60 m vs actual R(27, 0.50)≈136 m.
- GS-sized fly-by (`180246`, 60 s) was worse: 0 passes, balloon 0 closest 88 m — already east of balloon 0, so an earlier cut toward balloon 1 opened the miss.
- JSBSim `turn_radius_m` stays 0 (home until pass, same as GZ/YASim). `jsbsim_rascal.bank_max_roll_rad` 0.50 → 0.62. `flyby_radius_from_speed` remains in `race_guidance` for geometry tests.
- Verify 60 s `/tmp/balloon_race_20260821_180642.csv`: 3 passes, miss 18.5 / 13.0 / 11.5 m (balloons 0–2, all assisted at the pass row). Roll cmd 35.5°. Full lap + start of balloon 0 again.

## 0.35.19 - GZ look-at uses HSV blob; skip synth-only geom-gate
- Live `--gz` `/tmp/balloon_race_20260821_153119.csv`: climbed to tgt_d=0 but `assisted=1` until t=60.6. Pickle cam LOS NaN until then; first sample was az=24° el=10°. Plane flew geometric NED, not the camera error.
- Cause: 80 px `track_centroid_near_expected` gate (synth roofs) also ran on `--gz`. EKF pinhole vs `race_cam` is far off the blob — same reason `--viz` already skipped it.
- Skip that gate for `--viz`/`--yasim`/`--gz`; headless synth still requires centroid near the geometric projection.

## 0.35.18 - GZ chase Z is spawned-model altitude, not EKF z_hold
- Live `--gz` 90 s `/tmp/balloon_race_20260821_151653.csv`: plot/CSV passes at balloon 0 `(490.5, 0.3, 64.8)` vs tgt `(500, 0, 66.2)` (ΔD≈1 m) while Gazebo spheres stay at ENU z=500 (NED d=0). Mesh ~65 m under the balloon.
- Cause: `spawn_gz_from_setup(local_z=0)` then chase `rebase(..., z_hold)` after the unarmed fall (EKF z≈66). Same phantom-zero ΔD FG already corrected with balloon elevation.
- GZ race balloons stay at `local_z=0`; altitude hold uses that Z so the plane climbs back to the spheres.

## 0.35.17 - GZ race: PYTHONPATH for spawn_ic (false fan-out abort)
- `--gz` died with `mavlink-server fan-out failed` because `runSimGzPlane.sh` set `PYTHONPATH` as a shell variable and did not export it. tmux cwd is the repo root, so `python3 -m fw_sitl.spawn_ic` raised `ModuleNotFoundError` and exited before fan-out. `--viz` already prefixes `PYTHONPATH=` on the `python3` line.
- Race launcher dumps the sim pane on that abort so the real error is on stderr.

## 0.35.16 - Drop leftover debug NDJSON and session-id footnotes
- Removed leftover Cursor agent-log regions and `.cursor/debug-*.log`. Race CSV/pickle and `/tmp/balloon_race_plot.log` stay.
- History entries no longer cite Cursor session IDs or temporary probe names.

## 0.35.15 - Stop 4 s NED snaps: Nasal pose+velocity, no per-cycle model walk
- Post-0.35.14 pickle `143601`: red-balloon hit matched the error plot (model NED dxy=0, horiz 11 m at pass) but sim XY jumped 15–92 m every 4.0 s (21 snaps; 1641/2741 samples frozen). GT thread did 6 telnet gets + FOV/z gets + balloon-model Nasal every cycle.
- Pose is one Nasal dump (lat/lon/alt/att + speed-north/east/down-fps). Models/FOV/z are read once. Coast uses FG NED velocity, not a 4 s finite difference.
- Verified pickle `144653` (60 s `--viz`): 0 snaps >8 m vs 21 / 4.0 s max 92 m; red pass t=22.25 horiz ~10 m; 73 GT samples in 60 s with FG NED velocity always set. GT still ~800 ms/cycle (Nasal+`get /tmp/fw_pose`); leftover ~4 m kinks at that period, not 4 s plateaus.

## 0.35.14 - Chase/plot NED from FG aircraft + FG mesh, not EKF coast / settle offset
- Pickle `134011` overlay at the visual “hit” (`t=25.3` x=1060 y=306) was 154 m from chase tgt; telnet relock (true FG lat/lon) still 139 m. Overlay between locks was `gt + Δekf` (EKF 300–670 m from sim). Config tgt is settle+offset, never `/models/model` lat/lon.
- GT sample time is the end of the telnet burst (not the start — that projected ~60 m ahead). Chase XY locks once from FG balloon mesh geodetic (same `geodetic_to_ned` as the aircraft). `live_xy` uses the re-place origin, not a later aircraft lat.

## 0.35.13 - GT-only NED (no EKF coast sawtooth); hold 90° FOV
- Pickle `134011`: overlay `t=26.3 x=1076.7 y=337.5` looked like a balloon hit; error min 118 m at t=38. At t=26.2 plot Y=335 then t=26.4 Y=296 (40 m / 0.2 s) — each ~2.2 s telnet relock. EKF Y itself was smooth; `sim = gt + Δekf` injected the EKF−FG residual.
- Visual fill at 166 m for a 10 m sphere implies ~8–10° FOV, not 90°. GT telnet holds the socket ~2 s so `fg_camera.sync_camera_view` often cannot keep FOV; mouse-zoom/`goal-fov` also fights the set.
- History/chase/overlay NED = timestamped FG GT extrapolated with FG velocity (no EKF position coast). GT thread re-asserts `field-of-view` + `goal-field-of-view` 90. Error plot uses that sim pose.

## 0.35.12 - Plot sim pose + wall-clock time; error from FlightGear/Gazebo
- Live `--viz` 90 s: overlay/CSV wall 90 s vs plot `hist_t` 69 s (ratio 1.30); chase vs history XY split 20–190 m on a ~2 s period (slew vs snap) — the 3D sawtooth.
- Cause: `history.t` used PX4 `time_boot_ms` (JSBSim+FG slower than realtime); plots used slewed `gt−ekf` while chase snapped stale telnet GT.
- History time is wall (`time.time()-t0`). FG offset is `gt − EKF(at telnet start)` with no NED slew. `sim_*` is last FG/GZ pose plus EKF Δ; 3D/NED show dotted EKF and solid sim; ΔN/ΔE/ΔD and CSV miss use sim.

## 0.35.11 - FG clouds off
- `--disable-clouds` (already V6) still left 3D clouds / METAR overcast filling balloon_camera.
- JSBSim viz patch **V7**: `--disable-clouds3d` `--disable-real-weather-fetch` + `draw-mask/clouds` / `clouds3d-enable=false`. YASim `FG_run.py` extras get the same flags. `fg_camera.sync_camera_view` reasserts clouds off over telnet.

## 0.35.10 - Visual "hit" at 160 m was stock FG balloon scale
- Live `--viz` t≈17 s: camera filled with red + HSV lock, error plot Δhoriz≈160–250 m. GT aircraft→tgt matched the plot (~250 m); overlay NED matched GT.
- Cause: `assets/balloons/balloon_*.xml` referenced FG stock hot-air mesh (fills 90° FOV from >150 m), not the co-located ~10 m spheres — chase targets the model origin.
- XML paths → `balloon_R_G_B.ac` / `balloon_sphere.ac` (`object-name` balloon). Screen fill now ≈ close pass (~12 m for half of 90° FOV).

## 0.35.9 - Smooth FG NED plots; damp LOS roll oscillation
- Pickle `122330`: snapping `gt−ekf` every ~2 s jumped NED 10–97 m (EKF drift residual); HSV flicker flipped roll cmd ±46° in 50 ms; jsbsim `kp_heading=2` / `max_roll=0.8` saturated bank on straight homing.
- History NED offset slews at 80 m/s; chase still uses snapped `gt_ned_off_tgt`. `absorb_vel_jumps_from` on each poll burst.
- LOS: 3° heading deadband, 60°/s roll slew, jsbsim `bank_kp_heading=1.0` / `max_roll=0.50`. `RaceGuidance` holds last HSV LOS 0.35 s across brief blob drops.

## 0.35.8 - Error plot ~250 m while visually a few metres from the balloon
- Pickle/CSV `20260821_120853` t=32.95: overlay `x=1058 y=727` vs balloon `676,658` → horiz 389 m; HSV on red, geometric `on_screen=False`. GT offset N 375 m vs chase-target offset 21 m (354 m lag).
- Cause: NED `gt−ekf` slew at 12 m/s while the target offset jumped 50–90 m each ~2.4 s telnet. Chase/plot pos lagged FG; camera still tracked the visual balloon.
- Snap NED offset to latest `gt−ekf` (keep att slew + yaw absorb). Expect plot error ≈ visual miss at each GT sample.

## 0.35.7 - Plot velocity / yaw / LOS jumps (EKF glitches + mixed LOS frames)
- Pickle `20260821_090533`: NED was smooth (max Δxy 2.35 m) but vx/vy jumped 30–70 m/s and yaw 30–107° in one 50 ms sample (roll unchanged) — PX4 dead-reckoning EKF, not FG offset. LOS az jumped 85–140° when `los_deg_series` swapped camera-frame blob vs body +X on HSV flicker.
- Stamp LOCAL_POSITION with `time_boot_ms`; plot vx/vy/vz = ΔNED/Δt. `absorb_yaw_jumps_from` holds plotted yaw and shifts the locked att offset (and its slew target) so the glitch does not ramp back in.
- LOS panel is geometric body +X only; yaw/LOS az unwrapped on the figure.
- Live pickle stamp after the verification run.

## 0.35.6 - FG plot sawtooth / attitude jitter
- Pickle `20260821_083609`: NED jumps 12–725 m every 2.4 s (telnet `gt−ekf` relock) and yaw flipping ±125° (`add_rpy_offset_from` wrote FG Euler into `_last_att_deg`, so the next poll added the offset twice).
- Lock offset once from the first telnet pose **before** `t0` (`clear_series`); later updates **slew** (`slew_toward_ned` 12 m/s, `slew_toward_rpy` 50 °/s). Apply `last_ekf_pos + offset` every tick (do not relock from already-offset `last_pos`).
- `add_rpy_offset_from` leaves the poll att cache on EKF. Chase `q_act` is FG-frame; dashed cmds use `last_q_des` (same frame). Pickle is always written (`--no-plot` only skips PNGs).
- Live `20260821_090533`: pickle n=1898, unique XY 1897, max sample Δxy 2.35 m (was 725 m); sample Δxy max 1.32 m / Δyaw max 3.5° (no >8 m or >15° steps); yaw cmd−meas p95 1.5° (was 77°/162°). Plots: `/tmp/balloon_race_20260821_090533_{history,trajectory}.png`.

## 0.35.5 - Dense FG history (EKF+offset) and attitude commands on the angle plot
- Pickle `20260821_082257` had 1814 samples but only ~100 unique XY: every point was the same 2.4 s FG pose. Lock `gt − ekf` on each telnet publish; `add_ned_offset_from` / `add_rpy_offset_from` keep MAVLink-rate increments.
- Attitude panel overlays dashed `roll/pitch/yaw cmd` from `SET_ATTITUDE_TARGET`.
- Live `20260821_083609`: `n_xy_01m=1873` / 60 s (~1880 unique/min); pickle 1883 samples all with cmds; `savefig n=1883`.

## 0.35.4 - FG telnet `/>` was stalling history; plots still empty after 0.35.3
- Post-fix 0.35.3 run (`20260821_080848`): chase **did** reach balloon 0 (pass at 56.8 s, closest 22.5 m). Plots still `savefig n=16`. GT rebase stuck at ~796 ms/cycle; history appends 14 then 0.
- Cause: FG telnet answers slowly (often ~0.4 s/get with no newline). Synchronous `read_pose_deg` in the 20 Hz loop plus `poll_vehicle_state(HEARTBEAT)` discarding `LOCAL_POSITION_NED` left `savefig n=16`.
- Fix: reconnect after re-place; six gets on that socket; `command()` returns on `/>` or `\r`; race loop uses `history.last_armed`/`last_main_mode`; **telnet pose runs in `_gt_reader_loop`** so the chase tick is not blocked. Do not treat `=` as end-of-reply (truncated lat/lon → x≈−3.7e6 m). Live `20260821_081402` (sync): `savefig n=393`, passed balloons 0 and 1. Live `20260821_082257` (thread): `savefig n=1814`, yaw finite 1813/1814, closest 22 m.

## 0.35.3 - Unblock FG rebase so plots fill; send chase in PX4's EKF frame
- After 0.35.x ground-truth rebase, `_history.png` / pickle had ~13 samples over 60 s (CSV ~2.4 s/tick, not 20 Hz). Balloon re-place opened a second FG telnet and left `diag_tel` stale; six serial `get`s each waited the socket timeout. Reconnect after re-place; `FgTelnet.read_pose_deg` snapshots lat/lon/alt/att in one Nasal+get. Patch `overwrite_attitudes_from` so plots keep FG yaw, not NaN/EKF.
- Chase used FG NED LOS with EKF `q_act` and sent that Euler to PX4. With ~80–110° EKF yaw error the plane never turned toward the balloon. Guidance now runs on FG att; `q_exec` maps the body error onto the live EKF quaternion before `SET_ATTITUDE_TARGET`.

## 0.35.2 - FG balloons follow the live aircraft XY, not the LSZH IC origin
- Live `--viz --duration 60` (20260821_074003): 3/3 models placed, camera frames were real scenery (`frame_mean` 93–128), HSV locked at 1300–2500 px when the disk was in view. The aircraft was already 179 m from balloon 0 at spawn (NED 326 N, 127 E vs models at DEFAULT_ORIGIN) and 411 m away at race t=0 with heading error 134° — balloons sat outside the 90° FOV for 21/25 samples. Not a missing-mesh bug.
- `spawn_balloons_fg` now uses live `/position/latitude-deg` and `longitude-deg` as the NED XY origin (same pattern as live altitude-ft). Control re-places after altitude settle and `translate_balloons_ned` so chase NED matches the visual models (balloon 0 stays 200 m north of the settled aircraft).
- Post-fix run (20260821_074718): settle `horiz_live_to_b0=200`; race t=0 range 196 m (was 411 m); closest 82 m; camera `area_px>200` on 22 grabs.

## 0.35.1 - Disable --ekf-fix gps; record live evidence for a later retry
- `--viz`/`--yasim` keep ground-truth rebase only. `--ekf-fix gps` now exits 2 (`run_balloon_control.py`, `run_balloon_race.sh`). `ask` prompt removed. `prepare_sitl_arming(..., force_gps_aiding=True)` remains as a library hook (mag still forced off); do not wire it back to CLI without a new pre-arm sequence.
- FG bridge already sends usable aiding (`px4-noble-sim-ros` `flightgear_bridge/.../vehicle_state.cpp`): `HIL_GPS` `fix_type=3`, 10 sats, true lat/lon/alt/vel; `HIL_SENSOR` mag from the geomagnetic model. JSBSim/YASim ignore both via `SYS_HAS_MAG=0`, `EKF2_GPS_MODE=1` (dead-reckon). That is why EKF N/E/yaw drift from FG truth (up to ~1.5 km/min, ~110° yaw) — rebase overlays telnet lat/lon/alt/roll/pitch/heading onto guidance every 0.2 s.
- **Experiment A** (`--ekf-fix gps`, `SYS_HAS_MAG=1` + `EKF2_GPS_MODE=0` + forced reboot): live `--viz --duration 60`. PX4: `Compass needs calibration - Land now!` / `Preflight Fail: Compass 0 fault`. Armed unhealthy at 38 s (`z_ned=569`). Then a real dive: EKF pitch +25..+50° (thinks climbing) vs FG true pitch −35..−68°; at t=8.4 s EKF roll −29° vs true 137°; at t=16.8 s EKF roll 1° vs true −160° (inverted). Guidance `pitch_cmd` stayed +20° (`los` law). EKF z 595→1904 m in 55 s. FG telnet att went `None` after t=33.6 s. PX4: mag yaw aligned while unarmed/non-level (in-air spawn, ~40 s fall) → ~100° yaw error → `Ekf::isYawFailure()` 25° gate (`src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp`) within 1 s of takeoff → `tryYawEmergencyReset()` sets `mag_fault` permanently → `estimatorCheck.cpp` land-now. Do not enable `SYS_HAS_MAG` on this in-air spawn.
- **Experiment B** (same gps path, mag left off, no reboot — `EKF2_GPS_MODE=0` only): never armed. 60 s of `Arming denied: Resolve system health failures first` → `Engage failed`. Same-session `--ekf-fix rebase` armed at t=0. GPS fusion during the free-fall pre-arm window trips commander health even without mag.
- Future retry needs the aircraft level/GPS-healthy *before* EKF fuses GPS or mag (no unarmed JSBSim fall), not just flipping `EKF2_GPS_MODE`. Rebase is the working fix for chase/plots.

## 0.35.0 - User-selectable EKF drift fix for --viz/--yasim: GPS aiding vs ground-truth rebase
- Root cause behind the FOV-frustum mismatch (0.34.0) and earlier Z/homing bugs: with no GPS aiding, PX4's EKF dead-reckons on JSBSim/YASim and drifts hard from FG ground truth (observed up to ~1.5 km/min position, 110° yaw) — live MAVLink `ATTITUDE`/`LOCAL_POSITION_NED` vs FG telnet heading confirmed this.
- Investigated whether real GPS/mag simulation is even possible: inspected the FlightGear bridge (`Tools/simulation/flightgear/flightgear_bridge/src/vehicle_state.cpp` inside the `px4-noble-sim-ros` image). It already sends a valid `HIL_GPS` (`fix_type=3`, 10 sats, true lat/lon/alt/vel) and a geomagnetic-model `HIL_SENSOR` mag reading every frame — PX4 is just configured (`SYS_HAS_MAG=0`, `EKF2_GPS_MODE=1`) to ignore both on JSBSim/YASim.
- New `--ekf-fix {ask,gps,rebase}` (`run_balloon_control.py`, `--viz`/`--yasim` only):
  - `gps` — `prepare_sitl_arming(..., force_gps_aiding=True)` flips only `EKF2_GPS_MODE=0` (Automatic; not `reboot_required` per PX4's `params_gnss.yaml`, so no reboot needed). **`SYS_HAS_MAG` stays explicitly off.** First attempt also enabled `SYS_HAS_MAG` (gz-style) and forced a reboot to apply it — live-tested (`./scripts/run_balloon_race.sh --viz --duration 60 --ekf-fix gps`) and it crashed the aircraft. Root-caused from live EKF vs FG attitude plus PX4 source: mag-based yaw alignment happens while JSBSim is unarmed/non-level (falling ~40s through the forced reboot + arm-health-check delay) → ~100° initial yaw error → PX4's `Ekf::isYawFailure()` (hardcoded 25° gate, `gps_control.cpp`) trips within 1s of takeoff → `tryYawEmergencyReset()` resets yaw from GPS velocity and permanently sets `mag_fault=true` ("Compass needs calibration - Land now!", `estimatorCheck.cpp`) → attitude controller flies on a doubly-corrupted EKF attitude → real, unrecoverable dive. GPS position/velocity aiding alone was then tried (0.35.1): that path never armed.
  - `rebase` (default/safe) — every `GT_REBASE_PERIOD_S` (0.2 s), fetch FG telnet ground truth (lat/lon/alt-ft/roll/pitch/heading) on the existing diagnostic `FgTelnet` and overwrite the guidance `pos`/`att` (zero-order-held between fetches) instead of PX4's own drifting EKF state; `FlightHistory` is overwritten the same way so the 3D plot matches what was actually flown.
  - `ask` (default): interactive prompt at startup if stdin is a TTY, else falls back to `rebase` with a printed note (avoids blocking a non-interactive run). **Superseded in 0.35.1: gps/ask removed from CLI.**
- `z_hold_true`/diagnostic `FgTelnet` setup now applies to `--yasim` too (previously `--viz`-only), since YASim shares the same FG bridge/telnet.
- `run_balloon_race.sh`: `--ekf-fix gps|rebase` passthrough for `--viz`/`--yasim`; launcher always passes an explicit value (defaults to `rebase`) so the control tmux pane never blocks on the interactive prompt before the session is attached.
- Tests: `test_plant_gains.py::test_jsbsim_yasim_force_gps_aiding_keeps_mag_off`; updated `prepare_sitl_arming(...)`/`skip_reboot` substring contracts in `test_plant_gains.py`, `test_gz_race_contracts.py`, `test_yasim_race_contracts.py`.

## 0.34.0 - 3D history plot draws the camera FOV frustum
- User report: "3D history balloon positions don't match what I saw on screen" — saw the blue balloon (and others) on screen through the second half of a race, but the static 3D plot (path line + balloon dots, no FOV) made that look implausible by eye.
- Verified with the actual `offset_on_screen()`/`CameraModel` math against the recorded run (`run_balloon_control.py:path_sample`/pickle roll-pitch-yaw + positions): all 3 balloons were geometrically inside the 90°x70° FOV simultaneously from t≈39.5–57s of a 60s race — not a bug, just unverifiable by eye in a plain 3D scatter.
- `FlightHistory` now records `cam_hfov_deg`/`cam_vfov_deg`/`cam_mount_azimuth_deg`/`cam_mount_elevation_deg` (set from `flightSetup.json` camera block in `run_balloon_control.py`); the 3D trajectory figure draws translucent pyramid faces (`Poly3DCollection`) at ~5 sampled poses along the path, each reaching to the current target's range (capped to the scene span), colored by time. Old pickles load fine (dataclass class-level defaults back-fill missing fields).
- `fw_sitl/flight_history.py`: `frustum_corner_dirs_ned()`, `FlightHistory._plot_fov_frustums()`.

## 0.33.3 - Z-anchor fix, visual-lock pitch guard, correct assisted flag
- User reports: viz balloon looked much lower/higher than the plane while plot ΔZ≈0 (broken homing); plane later stalled and crashed climbing to a balloon; `assisted` stayed true while the balloon was off-screen (geometric-only, no real HSV lock).
- `run_balloon_control.py`: `z_hold_true` corrects PX4's settled EKF `z_hold` to the true FG balloon elevation at engage time (one-off `FgTelnet` diagnostic read of `/position/altitude-ft` and the balloon's `elevation-ft`); `race_balloons`/`world_balloons` rebase onto `z_hold_true` instead of the raw `z_hold` — fixes the phantom-zero ΔZ.
- `race.update_track(False, ...)` (not `True`) when a balloon is only geometrically on-screen (no real HSV blob) — `assisted` now correctly reflects "not visually tracking".
- `body_cmd_controllers.py` `AttitudeChaseController`: new `visual_lock` param on `send_chase_setpoint`. Real HSV lock → `kp_alt=0` (trust the camera-derived LOS elevation, don't let bookkeeping-Z fight it). No visual lock (geometric-only LOS) → cap pitch to `att_max_pitch_rad` (flyable sustained-climb ceiling) instead of the aggressive `att_los_max_pitch_rad`, which was demanding ~40° sustained climb and stalling the plane.
- `fg_camera.py`: removed a shared-socket FG telnet altitude probe that raced with the dedicated diagnostic connection (`ConnectionResetError`/`BrokenPipeError`).
- Tests: `test_body_cmd_controllers.py::test_visual_lock_ignores_alt_loop_uses_pure_los`, `::test_no_visual_lock_caps_steep_los_to_flyable_climb`; `test_gz_race_contracts.py` updated for the `update_track(False, ...)` geometric-only path.

## 0.33.2 - FG balloon origin follows live aircraft MSL
- `--viz` 60s: HSV yellow ring on camera, but control `tracker_in_view` stayed false all 60s (`geom_ok` never true, centroid 120–400 px from EKF UV). 0 passes; closest ~43 m. Plot ΔD≈0 (EKF rebase). Spawn-time FG `/position/altitude-ft`=3977 ft (1212 m) vs balloons at 3015.7 ft (919.2 m) → models ~293 m below the visual plane, so the blob sat low and the 80 px geom-gate rejected it.
- Spawn NED origin is live `/position/altitude-ft` (fallback 919.2 m); Nasal also writes `elevation-ft` on the placed node.
- After that origin match, `--viz` geom distance was still ≥165 px (lateral + chrome). Geom-gate now applies only to headless synth; `--viz` follows the HSV blob so look-at can start.

## 0.33.1 - FG balloons at sea level; camera still on the FG window
- FG `geo.put_model` writes `elevation-m`. `FGModelMgr::add_model` only reads `elevation-ft` (default 0). Live spawn logged `alt=919.2` m but models sat at 0 ft MSL (~500 m under the Rascal). Plot `z(D)` still matched EKF chase, so the plane never dove to the visible balloons.
- Spawn uses `fgcommand("add-model")` with `elevation-ft` from MSL metres (`919.2 m → 3015.7 ft`).
- `fit_window_outside_rect` shrinks `balloon_camera` into the leftover screen strip when FG is nearly fullscreen (640×480 could not sit beside it). Re-parks every 2 s.
- Tests: `test_fg_model_mgr_uses_elevation_ft_not_meters`; `test_nearly_fullscreen_fg_shrinks_camera_to_clear`.

## 0.33.0 - Balloons spawn before the airplane is used
- Race `--viz`/`--yasim`/`--gz`: `python -m fw_sitl.balloon_scene --setup … --fg|--gz` after sim/fan-out, **before** PX4 HEARTBEAT and control. Headless synth has no FG/GZ models.
- `spawn_fg_from_setup` places the cruise-MSL cluster (`local_z=0`), not EKF `pos_d`. GZ create retries until the world is up.
- Control `--no-sim` skips telnet/gz create (launcher already placed). Standalone control that owns the sim still spawns before MAVLink connect.
- Tests: `test_spawn_fg_from_setup_uses_cruise_msl`; launcher/control spawn-order contracts.

## 0.32.3 - balloon_camera sat on the FG window
- `--viz` `mss` grabs the FlightGear rectangle as composited on screen. OpenCV `balloon_camera` defaulted to (0,0) on top of FG → recursive window-in-window in the camera view.
- `place_outside_rect` puts the HighGUI window to the right (else below/left/above) of FG; parks top-right until FG geometry is known; retries every 2 s.
- Tests: `test_fg_camera.TestPlaceOutsideRect`; camera contract requires `moveWindow`.

## 0.32.2 - FG balloons used EKF local Z as MSL down
- `--viz` plot `z(D)` matched `tgt z` (same rebased PX4 local frame). FG `geo.put_model` took those z values as metres below cruise MSL 919.2. Live CSV `pos_d≈155` while JSBSim stayed ~919 m MSL → every balloon ~155 m under the plane.
- FG spawn is `rebase_balloons_to_local_z(..., local_z=0)`: plot-relative heights at cruise MSL. Chase/CSV still rebase onto settled EKF z.
- Tests: `test_fg_spawn_uses_config_ned_not_ekf_rebased_z`; `test_fg_cluster_at_cruise_msl_matches_plot_relative_z`.

## 0.32.1 - Smooth --viz balloon_camera
- Cause: FG image source called `sync_camera_view` (~26 telnet `set`s) and a full X11 window hunt every 20 Hz tick. `FgTelnet.set_prop` waited for a reply that `set` often never sends (2 s socket timeout) and FG handles props on the render thread — `balloon_camera` froze, then jumped several seconds of motion.
- `set_prop` is send-only. View sync + window geometry refresh every 2 s; reuse mss and cached geometry. xwininfo skips tiny class=fgfs children; first successful locate backend returns.
- Tests: `test_fg_camera` (nowait set, cached grab, xwininfo skip, publisher period contract).

## 0.32.0 - Aircraft spawn in flightSetup.json (all plants)
- `spawn.ned` / `heading_deg` (0=north, 90=east) in the same home NED frame as balloons. Default `[0,0,0]` heading 0 (200 m south of balloon 0, nose north).
- Race `--setup` writes JSBSim IC / FG `FG_ARGS_EX` / `PX4_GZ_MODEL_POSE` + GZ spawn velocity. JSBSim/YASim chase is balloon−spawn (PX4 home≈spawn); Gazebo keeps world NED. FG models stay on world XY.

## 0.31.4 - Synth camera used yaw=0 (looked north / "backward")
- `poll_mavlink` `recv_match(LOCAL_POSITION_NED)` discarded ATTITUDE, so synth `att` stayed (0,0,0). After passing balloon 0 southbound, balloon_camera kept looking north at the red disk (fills the FOV at ~7 m). Control plots were fine (they drain ATTITUDE with position).
- `poll_local_position_and_attitude`; sky/ground from world-down in camera frame (roll/pitch/yaw), not a fixed cy split.

## 0.31.3 - Fly-by only while closing on the current balloon
- JSBSim fly-by used inbound = LOS-to-current. Abeam of balloon 0 heading north that vs balloon 1 is ~180°, `d_turn`≈900 m, horiz≈26 m → chase skipped red immediately. Camera showed the balloon receding; plot `x(N)` ran ~200→480.
- `flyby_closing_ahead`: fly-by only if ground track is within 60° of LOS to the current balloon. Live start `/tmp/balloon_race_20260819_211846.csv` (settle origin (163,19) course 7.3°, race t=0 at (198,26)).

## 0.31.2 - balloon_camera black on conda base OpenCV 5
- Cause: `./run_balloon_race.sh` used Anaconda `base` python (OpenCV 5.0 Qt HighGUI, no bundled fonts). `namedWindow(WINDOW_NORMAL)` is also `WINDOW_GUI_EXPANDED` (flag 0) → black window. Frames still flowed (`seen_track=1`).
- Camera: `WINDOW_GUI_NORMAL` + `resizeWindow` + "waiting for image" placeholder; `waitKey` for the frame period. Launcher sets `DISPLAY` on `CAM_CMD` and falls back to conda env `pigeon` when OpenCV ≥5.

## 0.31.1 - Restore JSBSim path settle; fly-by at max_roll 0.80
- Reverted facing settle; JSBSim race uses `settle_path_altitude` like GZ/YASim. Fly-by `turn_radius_m` still JSBSim-only. `bank_max_roll_rad` 0.80 (not 0.90). Speed 18 m/s.
- Live #1 `/tmp/balloon_race_20260819_171109.csv`: unhealthy arm z_ned=237 settle 245.5; pass_count=0; miss list empty. Extra live #2 `/tmp/balloon_race_20260819_171310.csv`: armed 1.1s z_ned=57.1 settle 66.3; pass_count=2; miss_m 6.938 (balloon 0, assisted=0), 11.676 (balloon 1, assisted=1); max miss 11.676 m. Gate pass (≥2 passes, every miss_m≤14, ≥1 unassisted).

## 0.31.0 - JSBSim fly-by 90° corners; face balloon 0 at race start
- `coordinated_turn_radius_m` / `flyby_turn_distance_m`; JSBSim-only `RaceGuidance.turn_radius_m` aims the next balloon inside d_turn (pass still uses current). Gz/YASim stay `turn_radius_m=0`.
- JSBSim settle is `settle_altitude_facing_xy` (pursuit to balloon 0 XY, `|Δz|≤2 m` for 1.5 s, timeout 4 s, along_advance 40 m). `jsbsim_rascal.bank_max_roll_rad` 0.80 then allowed bump 0.90. Speed 18 m/s unchanged.
- Live 60 s headless JSBSim #1 (max_roll 0.80) `/tmp/balloon_race_20260819_165139.csv`: pass_count=1; miss_m 14.320 (balloon 0, assisted=1); max miss 14.320 m. Gate fail (≥2 passes, every miss_m≤14, ≥1 unassisted). t=0 south of balloon 0 (N=-119); facing settle + fly-by wired. Unhealthy arm z_ned≈235.
- Live #2 (max_roll 0.90) `/tmp/balloon_race_20260819_165531.csv`: pass_count=0; miss list empty; max miss n/a. Gate fail. Armed 18 s z_ned≈222. Stopped after the one allowed bump.

## 0.30.1 - Restore JSBSim Rascal cruise 18 m/s
- Task 5 16 m/s live stalled: unhealthy-armed, z_ned≈307 m, 0 passes. Restored `jsbsim_rascal.speed_mps=18.0` and `fw_airspd_trim=18.0` (min 10 / max 40). Heading 2.0, max_roll 0.70, look-at always-on unchanged.
- Live 60 s headless JSBSim `/tmp/balloon_race_20260819_153806.csv`: pass_count=1; miss_m 6.322 (balloon 0, assisted=0); max miss 6.322 m. Gate fail (≥2 passes). Plane armed healthy at z_ned≈58 m.

## 0.30.0 - JSBSim Rascal race cruise 16 m/s
- `jsbsim_rascal.speed_mps` and `fw_airspd_trim` 16.0 (`fw_airspd` min 10 / max 40). Heading 2.0, max_roll 0.70, inner `FW_RR_*`, pid/thrust unchanged. Fingerprint still unique vs `gz_rc_cessna` (`max_roll`).
- Live 60 s headless JSBSim `/tmp/balloon_race_20260819_152314.csv`: pass_count=0; miss list empty; max miss n/a. Gate fail (≥2 passes, every miss_m≤14.0, ≥1 unassisted). One run only; no second retune.

## 0.29.0 - Close balloon-race LOS off-screen
- `chase_uses_lookat` is always True (blob, on-screen geometric, and off-screen geometric). Frozen assisted path is unused for balloon-race attitude; tracker dir still preferred at the call site when in view.
- Live 60 s headless JSBSim `/tmp/balloon_race_20260819_151009.csv`: pass_count=2; miss_m 6.312 (balloon 0, assisted=0), 22.091 (balloon 1, assisted=1); max miss 22.091 m. Gate fail (every miss_m≤14.0).

## 0.28.1 - Race pass-miss helpers; live JSBSim gate
- `pass_miss_m(pos_ned, tgt_ned)` is 3D hypot; `load_pass_misses(path)` returns `(balloon_idx, miss_m, assisted)` for `event=="pass"` only.
- First live 60 s headless JSBSim (`/tmp/balloon_race_20260819_145504.csv`): 0 passes (spawned ~N of balloon 0, assisted, flew away). One allowed `jsbsim_rascal` retune: `speed_mps` 18, `bank_kp_heading` 2.0, `bank_max_roll_rad` 0.70. Gz heading comparison now equal at 2.0; tables still differ (`max_roll`).
- Second live 60 s (`/tmp/balloon_race_20260819_145839.csv`): pass_count=0, miss list empty, max miss n/a. Gate fail (≥2 passes, every miss_m≤14, ≥1 unassisted). Stopped after the one retune.

## 0.28.0 - JSBSim Rascal race gains
- `jsbsim_rascal` race/hold cruise is 20 m/s (`fw_airspd` 10/20/40); heading kp 1.9, max roll 0.60, alt kp 0.028 so LOS can close a 7 m gate on the 200 m triangle.
- Tighter PX4 roll inner: `FW_RR_FF=0.50`, `FW_RR_I=0.18`, `FW_RR_P=0.15`. Pitch/`FW_R_TC`/`FW_THR_TRIM` unchanged. YASim/Gazebo tables untouched.
- Host `test_load_shipped_flight_setup_json` now matches shipped balloon NED `[200,0,30]` / `[200,200,-30]` / `[0,200,0]` (JSON unchanged).

## 0.27.0 - Synth scene lock from control
- Color ZMQ payload may include `balloons: [[n,e,d], ...]`; `TargetColor.balloons_ned` is `None` on legacy messages.
- Control publishes rebased race NED on every color message. Headless synth subscribes that channel and freezes world balloons from it (template RGB/size kept), instead of rebasing onto the first falling MAVLink pose.

## 0.26.0 - Gz origin-bias lock; race NED = EKF − bias
- `--gz` locks a constant NED `origin_bias` from the first good EKF+mesh pair (`|h| >= 1 m`), then race/CSV/pass/plots use `pos = ekf − bias` (spawn/balloon frame). Stops per-tick mesh overwrite after lock. Pose tmux pane stays for the lock sample.
- 1 Hz `ekf_err_h` is still raw horizontal |EKF−mesh| (~50 m SITL origin offset). Do not treat raw EKF as balloon-frame NED.
- Verified live `--gz` 60 s: lock `|h|=48.6 m` (N=5.9,E=48.2); raw `ekf_err_h` stayed ~49–52 m; CSV passes at balloon 0 `(293.5,-1.7)` vs `(300,0)` and balloon 1 `(593.2,79.2)` vs `(600,80)` (within `pass_radius_m=7`).
- Verified live `--gz` 60 s: lock `|h|=48.6 m` (N=5.9,E=48.2); raw `ekf_err_h` stayed ~49–52 m; CSV passes at balloon 0 `(293.5,-1.7)` vs `(300,0)` and balloon 1 `(593.2,79.2)` vs `(600,80)` (within `pass_radius_m=7`).

## 0.25.0 - Gz mag default, GPS automatic, log |EKF−mesh|
- `prepare_sitl_arming` omits `SYS_HAS_MAG` on gz plants (leave airframe default; `--gz` still skips reboot) and sets `EKF2_GPS_MODE=0` (Automatic). JSBSim/YASim keep `SYS_HAS_MAG=0` and `EKF2_GPS_MODE=1`. `COM_ARM_MAG_STR=0` on all plants.
- `--gz` 1 Hz `t x y z` line appends `ekf_err_h=…m` (horizontal EKF vs Gazebo mesh; `nan` if mesh sample missing). Race NED, CSV, plots, and pass still overwrite from the ZMQ pose stream.

## 0.24.3 - Fix real jitter source: history.poll() burst vs last-only patch
- 0.24.2 fixed the *sampling* jitter (docker-exec staircase) but a second, independent bug remained and was still visible zoomed into the first few seconds of any race: `history.poll(master)` drains **all** queued MAVLink messages per call in a `while True` loop and appends one raw-EKF sample per `LOCAL_POSITION_NED` — measured ~2x `control_rate_hz`, so a *single* `poll()` call routinely appends 2 samples per control tick, not 1.
- Three call sites only patched the *last* appended sample of that burst (`history.x[-1] = world_ned`, `apply_target_to_last()`, `apply_cam_to_last()`), leaving every other sample in the burst at its raw/stale value: half the plotted NED-position points kept the drifted PX4 EKF position instead of the true Gazebo pose (huge error right after spawn while EKF is still converging, shrinking to invisible once it settles — hence "looks fine at full-race zoom, dense zigzag zoomed into 0–3 s"), and once camera tracking started, half the LOS points silently fell back to the geometric estimate (NaN cam-LOS pads in `los_deg_series`) instead of the tracked blob, alternating between two genuinely different LOS values every other sample.
- Fix: track `n_before_poll = len(history.x)` before calling `poll()`; new `FlightHistory.overwrite_positions_from(start_index, ned)` patches every sample appended since `start_index`, and `apply_target_to_last`/`apply_cam_to_last` take an optional `start_index` for the same reason (default `None` keeps old last-only behavior for existing callers/tests). `run_balloon_control.py` passes `n_before_poll` at all three call sites.
- Verified live (`--gz`, 60 s): rendered the interactive pickle at 0–3 s, 0–5 s, 0–20 s, and full-race zoom — NED position, velocity, attitude, LOS az/el, and Δ-distance panels are all smooth; the camera-acquisition instant is now a single clean step, not a zigzag.

## 0.24.2 - Stream Gazebo pose (ZMQ) instead of polling docker exec
- The 0.24.1 Gazebo-pose fix still jittered: each sample came from a fresh `docker exec gz model -m <name> --pose` (~0.4-0.5s subprocess latency), polled from a background thread. Between samples the position was frozen (staircase), and step widths varied with docker/host scheduling — that irregular staircase is what showed up as jitter in the NED-position and Δ-distance plots, not sensor noise.
- Real fix: a new `pose` tmux pane (`run_balloon_gz_pose.py` → `fw_sitl/gz_pose_bridge.py`, docker-exec'd once, long-lived) subscribes Gazebo's `/world/default/dynamic_pose/info` (`gz.transport`, `gz.msgs.Pose_V`) inside the container — measured ~40 Hz, the physics/render rate — and streams samples to control over a new ZMQ `pose` channel (`zmq_bus.PoseSample`/`PosePublisher`/`PoseSubscriber`, `flightSetup.json` `zmq.pose`). Control's per-tick `world_ned` now comes from `PoseSubscriber.latest()`, continuous and low-latency, same pattern as the existing `race_cam` image bridge (`gz_camera.py`). No more per-sample subprocess spawn.
- `fw_sitl/gz_pose.py`'s `fetch_gz_model_enu`/`gz_model_pose_argv`/`parse_gz_model_pose_enu` (the one-shot CLI path) stay as tested utilities but are no longer used in the live control loop.

## 0.24.1 - Gazebo race NED from world pose, not drifted EKF
- On `--gz`, CSV / 1 Hz print / pass detection / history plots use the plane's Gazebo ENU pose converted with the same origin as balloon spawn. PX4 `LOCAL_POSITION_NED` can be 150 m east of the mesh while `race_cam` is already inside the sphere (`SYS_HAS_MAG=0`, loose GPS).
- Real `gz model -m <name> --pose` output is `[x y z]`/`[roll pitch yaw]` (space-separated, brackets) — the first cut guessed `--pose --name` flags and a comma/pipe parser, which is not the real CLI; `fetch_gz_model_enu` silently returned `None` and every sample kept falling back to the EKF pos. Fixed CLI to `gz model -m <name> --pose` and the parser to a bracket-number regex; verified live against `docker exec px4-noble-gz-plane gz model -m rc_cessna_0 --pose` and a full race where `pass` events log `pos_ned` within the balloon radius.
- Real `gz model -m <name> --pose` output is `[x y z]`/`[roll pitch yaw]` (space-separated, brackets) — the first cut guessed `--pose --name` flags and a comma/pipe parser, which is not the real CLI; `fetch_gz_model_enu` silently returned `None` and every sample kept falling back to the EKF pos. Fixed CLI to `gz model -m <name> --pose` and the parser to a bracket-number regex; verified live against `docker exec px4-noble-gz-plane gz model -m rc_cessna_0 --pose` and a full race where `pass` events log `pos_ned` within the balloon radius.

## 0.24.0 - Interactive race plots; shared time axis
- Post-race viewer is a matplotlib window (toolbar zoom/pan), not an eog PNG. Time-series subplots share the x axis so a zoom/pan on one panel moves all of them.
- Control writes a `.pkl` of live history next to the CSV so the host waiter keeps yaw/camera LOS; CSV is the fallback.

## 0.23.11 - Race launcher attaches to tmux by default
- `./run_balloon_race.sh` attaches to session `balloon_race` (control pane) so the 1 Hz `t x y z` line is on screen. `--detach` keeps the old return-immediately behavior. No TTY → leftover detached.

## 0.23.10 - 1 Hz NED pose on stdout and balloon_camera
- Control prints `t=…s x=… y=… z=…` (PX4 local NED) each second. The same line is drawn on `balloon_camera`.

## 0.23.9 - Look-at and LOS plot follow the camera blob
- Geometric on-screen look-at zeroed PX4 yaw vs balloon bearing, so the LOS plot sat near 0° while `balloon_camera` showed the target ~25–45° right of center. Chase the HSV blob when it is visible; geometric body +X is only the fallback if the balloon still projects but the tracker missed.
- LOS panel uses that camera-frame az/el while a blob is logged (0=image center). `race_cam` pose is `relative_to` `base_link`.

## 0.23.8 - LOS vs body +X; look-at centers the nose
- LOS plot was azimuth minus *ground track* (0=on course). Track can sit on the balloon while yaw/camera boresight is ~10–20° off, so the panel showed 0 while the blob stayed off-center. Az is now bearing−yaw and el is elevation−pitch (0=in front of body +X); sticky-until-abeam unchanged.
- On-screen look-at banks vs yaw, not track, so the same error goes to zero and the balloon sits in front of the plane. Off-screen path law still uses coordinated ground track.

## 0.23.7 - Host eog waiter; savefig before docker kill
- Live races wrote CSV `end_duration` but never produced PNGs: control plots ran in detached tmux *after* `kill.sh --all`, and that teardown dropped the pane before savefig. Save PNGs before docker stop.
- `./run_balloon_race.sh` starts host `show_race_plots.py` (user shell, not tmux): wait for CSV `end_*`, write PNGs from the CSV if missing, open eog on `$DISPLAY` until closed. Log: `/tmp/balloon_race_plot.log`.

## 0.23.6 - Race plots block in eog until closed
- Tk/matplotlib windows from detached tmux never appeared, and `xdg-open` returned immediately so the process exited before anything showed. Figures are drawn with Agg, saved as PNGs, then `eog --new-instance` opens them on `$DISPLAY` and the control process waits until that window is closed.

## 0.23.6 - Race plots block in eog until closed
- Tk/matplotlib windows from detached tmux never appeared, and `xdg-open` returned immediately so the process exited before anything showed. Figures are drawn with Agg, saved as PNGs, then `eog --new-instance` opens them on `$DISPLAY` and the control process waits until that window is closed.

## 0.23.5 - Race plots open on the desktop after tmux teardown
- Control runs in a detached tmux pane, so matplotlib `plt.show()` never appeared on the host display. After a race, PNGs are written next to the CSV (`/tmp/balloon_race_<stamp>_history.png` and `_trajectory.png`) and opened with `xdg-open`. `--no-plot` still skips both.

## 0.23.4 - LOS until abeam; look-at holds altitude
- Race LOS plot was NED bearing to the *chase* balloon (`az=0` north). That stays ~15° on a constant-bearing intercept, and `pass_radius` retargets ~50 m *ahead*, so a miss never showed the ±90° abeam swing. Series is now az vs ground track to the balloon until abeam (0=on course).
- On-screen look-at pitch mixes the altitude loop (`kp_alt`); geometric `los_el` alone was only a few degrees far out, so the plane stayed high and elevation drifted to about −4°.

## 0.23.3 - Gazebo timed end no longer restarts
- `runSimGzPlane.sh` GPU→CPU fallback only on an immediate launch failure. `docker rm -f` after a live race (exit 137/143 or ≥30 s) stops the script instead of starting a second Gazebo.

## 0.23.3 - Gazebo timed end no longer restarts
- `runSimGzPlane.sh` GPU→CPU fallback only on an immediate launch failure. `docker rm -f` after a live race (exit 137/143 or ≥30 s) stops the script instead of starting a second Gazebo.

## 0.23.2 - NED plot dashed current target
- Figure 1 NED position overlays dashed tgt x/y/z (current balloon at each time, same colors as the plane).

## 0.23.2 - NED plot dashed current target
- Figure 1 NED position overlays dashed tgt x/y/z (current balloon at each time, same colors as the plane).

## 0.23.1 - 3D history marks balloons
- Figure 2 (3D trajectory) scatters each race balloon at rebased NED, colored RGB, and the equal-aspect box includes them.

## 0.23.0 - Timed race teardown, target log, LOS plots
- `./run_balloon_race.sh` runs `kill.sh --all` before starting a new plant (not with `--no-sim`).
- Default `guidance.duration_s` is 60 s; `0` / `--duration 0` has no time limit. `--duration SEC` on the launcher (else `BALLOON_RACE_DURATION`).
- Race-owned runs pass `--stop-sim-on-exit`: after duration/laps/Ctrl+C, remove SITL docker stacks, then plot vs time (`--no-plot` to skip).
- Race CSV adds `tgt_n/e/d` beside plane `pos_n/e/d`. History plots geometric LOS az/el to the current balloon and plane−target ΔN/ΔE/ΔD (JSBSim / `--viz` / YASim / Gazebo). 3D trajectory marks each balloon in its RGB.

## 0.22.0 - Per plant+airframe controller constants
- Host outer loop (PID, bank, thrust, speed, lookahead, `FW_AIRSPD_*`) and PX4 inner `FW_*` overlays live in `fw_sitl/plant_gains.py` keyed by `jsbsim_rascal` / `yasim_rascal` / `gz_rc_cessna` / `gz_advanced_plane`.
- JSBSim headless and `--viz` share `jsbsim_rascal` (former shared numbers). YASim and Gazebo tables are distinct; `--model advanced_plane` switches the Gazebo table.
- `prepare_sitl_arming` writes the selected plant overlay. Race/hold use that table; `flightSetup.json` speed is spawn/scenario only. CLI `--speed`/`--lookahead` still override.
- Gazebo velocity-mode "trim 16 until GS recovers" special case removed: Cessna trim is 16 in the plant table from arm.

## 0.21.10 - JSBSim race matches Gazebo balloon scenario
- Green/blue at +250/+150 m NED made look-at dive ~35° with heading error ~0, so the Rascal flew north and never turned. Same `flightSetup` XY as Gazebo, all z=0 at cruise.
- Synth painted balloons on the aircraft every frame; GZ models are world-fixed. Freeze synth NED on first pose.
- Chase bank used yaw when |track−yaw|>30° (roll_des=0 while velocity missed the balloon). On-screen look-at uses track up to 90° crab. Geometric LOS while the balloon is in the camera (same 3D point GZ flies at).

## 0.21.9 - On-screen chase uses Gazebo FW look-at
- Body-azimuth roll + commanded yaw did not close the pixel loop: when rolled, body horiz was ~4° while heading error was ~20° (weak bank); yaw≠actual violates the PX4 FW attitude contract; after the blob hit center a ~21° crab still flew past.
- Match Gazebo: bank onto LOS azimuth with coordinated heading, pitch to LOS elevation (40° cap), yaw stays actual. Look-at sends that roll/pitch on the wire (no quaternion-PID mix). Off-screen path law unchanged.

## 0.21.8 - On-screen pursuit yaws to hold the blob at center
- Look-at kept `yaw_des = yaw_act`, so only roll could turn. Body azimuth stayed tens of degrees and the balloon walked off-center until it left the frame.
- While on screen, yaw setpoint is actual yaw + body azimuth (clipped to ±45°, half HFOV) together with roll/pitch look-at. Off-screen path law is unchanged.

## 0.21.7 - Look-at while balloon is on screen; assisted only off-screen
- Tracker `in_view` was false while the current balloon still projected into the camera, so assisted path ran with the balloon visible (overlay also treated any miss as assisted).
- Look-at if the tracker has a blob **or** the active balloon projects into the image. Assisted path only when it is off-screen. Overlay follows the assisted flag only. After a pass, drop the old blob and publish the next color immediately.

## 0.21.6 - Race look-at only in-view; assisted is bank-to-turn
- Body look-at with `yaw_des = yaw + horiz_err` closed on red once, then assisted ~100° error commanded 40° pitch-up and a heading snap; the Rascal tumbled and never passed, so later balloons were never selected.
- In-view: body-frame roll/pitch look-at, yaw stays actual. Out of view: freeze origin+course on target change and use the straight-hold path law.

## 0.21.5 - Race look-at uses body-frame camera error
- NED Euler look-at (`los_az − yaw`, `pitch = clip(los_el, 20°)`, `yaw_des = yaw_act`) left the blob off-center: heading error on the wire was ~0 and pitch sat at the 20° cap while `los_el` was ~−50°.
- `q_des_from_los` rotates LOS into body: roll/yaw from body azimuth, pitch from current pitch + body elevation. Chase pitch cap ~40° (path hold still 20°).

## 0.21.4 - Balloon-race attitude chase is pure pursuit
- Assisted used the straight-hold intercept (frozen origin/course, pitch from altitude). Camera boresight followed the line, not the balloon, so the blob sat off-center.
- Attitude chase always looks along live LOS (`q_des_from_los`): camera blob in-view, geometric balloon LOS when assisted. Straight-flight path hold is unchanged.

## 0.21.3 - Balloon race assisted chase uses straight-flight bank law
- Assisted (out-of-view) attitude chase froze origin at the aircraft every tick, so intercept xt was always 0 and the plane crabbed off the line to the balloon.
- Lock origin+course on target change (same intercept + coordinated heading as straight hold). In-view LOS look-at is unchanged.
- After a pass, recompute chase LOS before locking: using the pre-pass dir froze a line along the old heading (wrap to balloon 0 flew 23° instead of ~200° back).
- After a pass, recompute chase LOS before locking: using the pre-pass dir froze a line along the old heading (wrap to balloon 0 flew 23° instead of ~200° back).

## 0.21.2 - Attitude hold ignores uncoordinated ground track
- Bank used `atan2(vy, vx)` whenever gs ≥ 5 m/s. After late in-air arm, track was ~160° from yaw; heading error ~116° saturated +26° roll and the path S-curved (xt rms ~570 m).
- Use ground track only when |track−yaw| ≤ 30° (keeps the 0.19.2 crab fix). Otherwise bank vs body yaw.

## 0.21.1 - JSBSim/YASim straight flight in-air attach (skip reboot, CBRK_SUPPLY_CHK, FG GPU)
- PX4 v1.17: MAVLink force-arm (21196) still runs health checks. Wrong name `CBRK_SUPPLYCHK` never set `CBRK_SUPPLY_CHK`.
- Skip autopilot reboot; JSBSim uses 60s arm / accept_unhealthy / no sim-reset so EKF local-pos can pass the 1s validity probation while falling.
- YASim `runSimYasimRascal.sh --gpus all`: without NVIDIA in the container FG has no visual and HIL sensors never start.

## 0.21.0 - Rascal balloon race (JSBSim + YASim) with Euler+thrust commands
- Attitude mode: quaternion PID internally; SET_ATTITUDE_TARGET is roll/pitch/yaw + thrust on JSBSim, YASim, and Gazebo.
- JSBSim/YASim straight flight default `--cmd-mode attitude` (was ignored / velocity).
- `./run_balloon_race.sh --yasim` — YASim FG Rascal, FG camera, Nasal balloons; exclusive with `--viz`/`--gz`.
- Race attach skips autopilot reboot (`--no-sim`); YASim sim gains mavlink-server fan-out + balloon models; `kill.sh --fg` drops the sidecar.
- `runSimYasimRascal.sh`: `--mavlink-server` / `MAVLINK_FANOUT` default 0, fail loud; balloons bind-mount `/opt/fixedwing/balloons` and copy into FG_ROOT `Models/FixedWing/`.
- `patch_px4_flightgear_sitl.sh`: idempotent `--telnet=5501` + `--allow-nasal-from-sockets` on FG_run.py (no `--fdm=null` / hide-aircraft).
- `kill.sh --fg` / `--all`: `kill_fg_stack` drops FG container, `${FG_NAME}-mavlink` sidecar, and host mavlink-server.

## 0.20.0 - Cycle balloons until duration (laps=0)
- After blue, wrap already chose red (`tgt=0`) but `laps=1` ended the race 50 ms later (`end_laps` at t=60.2 s).
- `guidance.laps=0` keeps cycling red→green→blue until `duration_s` or Ctrl+C. `laps>0` still ends after that many circuits.

## 0.19.6 - Retarget on fly-by, not after orbiting into 50 m
- Gate used camera LOS as approach, so gate_dot was identically −range and never fired. Pass waited for 50 m radius (GZ race: first fly-by 147 m east at t≈25 s, software pass t≈115 s).
- Closest-approach after a lock (range rising ≥10 m, miss ≤4× pass_radius) plus ground-track gate. Clear in-view so chase switches to the new balloon immediately.

## 0.19.5 - Gazebo balloons are visual-only (no collision)
- Removed the 5 m `<collision>` sphere. Static visuals stay in the camera; the plane flies through without a physics hit.

## 0.19.4 - In-view chase looks at the blob, not the 500 m aim point
- Camera is body-fixed: in-view bank uses yaw vs LOS azimuth (ground-track bank left the blob far left/right).
- Pitch follows LOS elevation; balloon Z is thrust-only. Do not scale Z by 500 m lookahead (that dove while the blob was still high).
- Out of view: keep track+intercept path law with `z_target` = balloon Z.

## 0.19.3 - Race chase uses ground-track bank (same as straight hold)
- Attitude chase banks from ground track vs balloon course, not yaw-only.

## 0.19.3 - Race chase uses ground-track bank (same as straight hold)
- Attitude chase banks from ground track vs balloon course, not yaw-only.

## 0.19.2 - Attitude hold crabbed ~80 m off the line
- Yaw P cancelled xt P at ~0° roll (peak xt −83 m, then slow return). Bank now uses ground track vs course+lookahead intercept.
- Missing ATTITUDE used identity (north) vs a west lock → 90° fake yaw error; hold until real attitude.

## 0.19.2 - Attitude hold crabbed ~80 m off the line
- Yaw P cancelled xt P at ~0° roll (peak xt −83 m, then slow return). Bank now uses ground track vs course+lookahead intercept.
- Missing ATTITUDE used identity (north) vs a west lock → 90° fake yaw error; hold until real attitude.

## 0.19.1 - Attitude hold was banking the wrong way on cross-track
- Right-of-line commanded more right bank (`+kp*xt`) → spiral (GZ run: xt −866 m rms 410 m; height held).
- Sign is now toward the line; thrust scales by `1/cos(roll)` so banked flight keeps altitude.

## 0.19.1 - Attitude hold was banking the wrong way on cross-track
- Right-of-line commanded more right bank (`+kp*xt`) → spiral (GZ run: xt −866 m rms 410 m; height held).
- Sign is now toward the line; thrust scales by `1/cos(roll)` so banked flight keeps altitude.

## 0.19.0 - Quaternion attitude PID (straight flight + race)
- `AttitudeChaseController` streams `SET_ATTITUDE_TARGET`: SO(3) error PID (Hamilton quat), Euler only for display / 1-D path loops.
- Thrust rises when below hold altitude. Gazebo straight flight defaults `--cmd-mode attitude`. Race `flightSetup.json` `cmd_mode` is `attitude`.
- Path `velocity` mode unchanged (TECS). `rates` still a stub.

## 0.18.13 - In-air TECS dove; chase Z followed the sink
- `z_cmd` was `pos_z−40` every tick. Clamp vs last command; do not raise `z_hold` toward a descending `pos_z`.
- Growing Z error still did not climb until t≈60 s (`gs≈32`). Spawn TAS ~14 vs trim 30 → TECS underspeed pitch-down. GZ race sets `FW_AIRSPD_TRIM`+airspeed SP to 16 m/s until level, then cruise 30. `FW_USE_AIRSPD=0` idled throttle — not used.

## 0.18.12 - In-view chase was ratcheting a sink
- Alt-preserve used current `pos_z` every tick during a turn, so an in-air descent became the altitude command and the balloon left the top of the image.
- Freeze last commanded Z on large heading error; when `in_view`, command 3D LOS altitude (skip alt-preserve) so the plane can climb to the blob.

## 0.18.12 - In-view chase was ratcheting a sink
- Alt-preserve used current `pos_z` every tick during a turn, so an in-air descent became the altitude command and the balloon left the top of the image.
- Freeze last commanded Z on large heading error; when `in_view`, command 3D LOS altitude (skip alt-preserve) so the plane can climb to the blob.

## 0.18.11 - Camera LOS Euler had inverted yaw/pitch
- Duplicate `ned_to_body_rotation` used Rz(-yaw)/wrong pitch: yaw +90° mapped body X west, nose-up to +z down.
- In-view balloon ~150 px right only commanded ~2° heading change (3D LOS XY ≈ geometric bearing), so the plane never centered the target.
- Single PX4 Tait-Bryan 321 DCM: yaw+ → East, pitch+ → nose up.

## 0.18.10 - Race chase used yaw=0 (never homed balloons)
- `history.poll()` already consumes `ATTITUDE`; control then `recv_match(ATTITUDE)` got nothing and left `att=(0,0,0)`.
- Camera LOS mapped as heading-north; `alt_preserve` froze Z whenever course was off-north → plane sank under balloons and orbited.
- Chase now uses `history.last_att_rad` after poll.

## 0.18.9 - Gazebo balloons were black (invalid SDF Color)
- `model.sdf` used nested `<r>/<g>/<b>` under `<diffuse>`; Gazebo SDF 1.9 Color is `r g b a`. Those children are dropped → default black.
- Vector `<ambient>/<diffuse>/<emissive>` matching filename RGB (red/green/blue). Re-spawn (re-run race) to pick up.

## 0.18.8 - Gazebo GUI follow uses rc_cessna_0
- Race GUI stayed at spawn/origin: follow locked `rc_cessna` because that string is a substring of listed `rc_cessna_0`, then CameraTracking `NodeByName` failed.
- Exact `gz model --list` names (prefer `model_0`); bake `<follow_target>rc_cessna_0</follow_target>` + 10 m / 3 m offset into `GZ_GUI_CONFIG`.

## 0.18.7 - Gazebo race panes: keep conda libs, unbuffered logs
- Camera pane aborted `Assertion failed: !_more (src/fq.cpp:80)`: ZMQ CONFLATE is incompatible with multipart image/color/track. Default `conflate=False`; `poll_and_update` already keeps the latest. Also keep conda `LD_LIBRARY_PATH` (0.18.5 unset mixed libzmq). tmux libtinfo lines are cosmetic.
- Image pane looked stuck on `Starting gz camera bridge` because `docker exec -i` block-buffers Python stdout. `PYTHONUNBUFFERED=1` / `python -u` on race panes + gz docker exec.
- Race launcher uses `${PYTHON:-python3}` so `(pigeon)` wins when that env is active.

## 0.18.7 - Gazebo race panes: keep conda libs, unbuffered logs
- Camera pane aborted `Assertion failed: !_more (src/fq.cpp:80)`: ZMQ CONFLATE is incompatible with multipart image/color/track. Default `conflate=False`; `poll_and_update` already keeps the latest. Also keep conda `LD_LIBRARY_PATH` (0.18.5 unset mixed libzmq). tmux libtinfo lines are cosmetic.
- Image pane looked stuck on `Starting gz camera bridge` because `docker exec -i` block-buffers Python stdout. `PYTHONUNBUFFERED=1` / `python -u` on race panes + gz docker exec.
- Race launcher uses `${PYTHON:-python3}` so `(pigeon)` wins when that env is active.

## 0.18.6 - Gazebo race: engage despite stalled spawn
- PX4 `Attitude failure (pitch)` / `No connection to the GCS` are in-air SITL preflight noise (same as straight flight; `accept_unhealthy` still arms).
- Real abort was `wait_min_airspeed` after tmux heartbeat wait: spawn velocity is one-shot, unarmed Cessna stalls (~1.4 m/s), gate blocked engage.
- Race control no longer waits for 15 m/s before arm; engage ASAP like `run_straight_flight_gz.py`.

## 0.18.5 - Gazebo race: NaN airspeed + missing cv2
- `./run_balloon_race.sh --gz` looked dead (`libtinfo` spam + truncated `t`) but tmux did start; camera/image died `No module named 'cv2'`, control died `last=nan m/s`.
- `wait_min_airspeed`: NaN/inf airspeed is missing → use `VFR_HUD.groundspeed` (Gazebo often publishes NaN IAS).
- Race launcher unsets Anaconda `LD_LIBRARY_PATH` (tmux/bash libtinfo warnings) and preflights `cv2`/numpy/zmq/pymavlink with a `requirements.txt` hint.
- `requirements.txt`: `opencv-python>=4.8,<5` (GUI camera pane; OpenCV 5 pulls numpy 2).

## 0.18.4 - Gazebo chase camera closer
- Follow offset 10 m behind / 3 m above (was 45 / 12) so the RC Cessna fills the GUI.

## 0.18.3 - Gazebo GUI chase-follows the plane
- Default gz camera looks at origin (`-6 0 6`) while the plane spawns at 500 m AGL — empty grey view.
- `gz_gui_follow`: `GZ_GUI_CONFIG` camera at chase pose + `/gui/track` FOLLOW on `rc_cessna`/`advanced_plane` (behind/above, far clip 25 km).

## 0.18.2 - Gazebo launch: local image required, GPU fallback
- `runSimGzPlane.sh` refuses missing `px4-noble-sim-ros` (no Docker Hub pull) and prints `Dockerfiles/PX4_noble_sim_build.sh`.
- `--gpus all` still default; on CDI/toolkit failure retry without GPU (`PX4_GZ_DOCKER_GPUS=none` skips NVIDIA).

## 0.18.1 - Fail loud when Gazebo/JSBSim sim runner dies at start
- `start_sim` always logs to `/tmp/fw_sim_runner_<pid>.log` (viz still `/tmp/jsbsim_viz_runner_<pid>.log`) and raises if the runner exits within 2 s (missing image, `docker --gpus all`, etc.) instead of waiting 180 s for a MAVLink heartbeat then `SystemExit: 1`.
- `run_straight_flight_gz.py` appends noble-image + nvidia-container-toolkit hint on that failure.

## 0.18.0 - Gazebo PX4 plane plant (balloon race --gz)
- Additive plant: `runSimGzPlane.sh` / `run_straight_flight_gz.py` (Cessna default, `--model advanced_plane`).
- Race: `./run_balloon_race.sh --gz` — onboard `race_cam` → ZMQ, balloons in gz world, GUI for operator.
- In-air spawn pose `0,0,500,0,0,1.570796` + spawn velocity; `kill.sh --gz`.
- Jetty `PythonSystemLoader`: `RaceSpawnVelocity` via `Link.set_linear_velocity` (no `gz.sim.components`); `get_system()` always defined.
- JSBSim fan-out ignores leftover gz sidecar unless `--gz`; `--gz` passes `--container` / `--gz-container`.
- `spawn_balloons_gz` clears all world `balloon_*` then requested names; camera 30 s clock from start; `docker run` fail hints `nvidia-container-toolkit`.
- `.gitignore`: `python/logs/`, `python/bin/`, `*.xwd` (e2e dumps / fetched mavlink-server).

## 0.17.6 - Race tmux "no server" was missing mavlink-server
- Symptom: `./run_balloon_race.sh` printed `no server running on /tmp/tmux-1000/default` and stopped.
- Cause: gitignored `python/bin/mavlink-server` missing → fan-out failed → sim pane exited → tmux server died → next `tmux` call looked like a tmux bug.
- Fix: auto-`fetch_mavlink_server.sh` when no usable host binary; race sets `remain-on-exit` and checks session after sim settle with a clear fan-out hint.
- Re-fetch now if needed: `python/scripts/fetch_mavlink_server.sh`

## 0.17.5 - Idempotent FG balloon spawn (no red pile)
- Root cause of “many reds + 2 other colors” on top of 0.17.4: `geo.put_model` only adds; re-runs/probes left stale `/models` (and old `/ai/models` stubs / stock `balloon4`) while new colored FixedWing XMLs also appeared.
- Fix: before place, remove `Models/FixedWing/balloon_*`, stock `balloon4` / `Aircraft/balloon/` models, and balloon-tagged `/ai/models` stubs; log per-index color+path; report live path list/count.
- Colors: each config RGB still loads distinct `Models/FixedWing/balloon_R_G_B.xml` (hull material override on stock mesh).
- Evidence: double spawn → clear 3 then place 3; live paths exactly `[balloon_255_0_0, balloon_0_255_0, balloon_0_0_255]` (no accumulation). Unit tests for clear contract + distinct paths/diffuse.

## 0.17.4 - FG camera grab + Nasal balloon spawn (empty-view root cause)
- Root cause of blank FG race camera: `fg_window_pattern` `FlightGear|fgfs` matched 3×3 `Qt Selection Owner for fgfs` before the real OSG window → mss resized a solid stub (no balloons ever). Secondary: `/ai/models/model[i]` props do not draw; need `geo.put_model` with `--allow-nasal-from-sockets`.
- Fix: skip tiny/selection-owner windows; prefer title FlightGear / largest; FG viz patch **V6** `--allow-nasal-from-sockets`; `spawn_balloons_fg` Nasal `geo.put_model`; `runSim --viz` copies balloons into FG_ROOT `Models/FixedWing/`; control spawns rebased race NED.
- Circling: failsafe after control end; during race assisted-only + bank keeps balloon off boresight; freefall NaN ~125 s kills pose. InvalidCRC/18570 unrelated (0.16.2/0.16.3).
- Verify: `python/scripts/verify_balloon_visibility.py` — synth 100 m `logs/e2e/synth_balloon_100m.png` in_view area_px≈752; FG `--fg` needs live viz.
- Tests: `test_skips_tiny_qt_fgfs_selection_owner`.

## 0.17.3 - Clear stale FG balloons before spawn
- Root cause of “many red balloons + 2 colored”: `geo.put_model` only adds; `spawn_balloons_fg` never removed prior `/models/model[*]`, so each race/probe accumulated leftovers (mostly older red defaults) while new R/G/B instances also appeared.
- Fix: clear `Models/FixedWing/balloon_*` nodes before place; log per-index color+path; report live path list/count after spawn (warn on count mismatch).
- Tests: distinct per-color model paths; clear-before-spawn contract; XML diffuse RGB still distinct.

## 0.17.2 - FG balloon XML loads colored .ac
- Root cause after 0.17.1: spawn prefers `balloon_R_G_B.xml`, but all wrappers still pointed at stock `balloon4.ac` with material override on object `hull`. Our spheres are object `balloon`, so overrides no-op → every FG balloon stayed stock red.
- Fix: each XML `<path>` → `/opt/fixedwing/balloons/balloon_R_G_B.ac` (sphere → `balloon_sphere.ac`); material animation `object-name` → `balloon` with matching diffuse/ambient.
- Test: `test_xml_wrappers_point_at_matching_ac` (path ends with matching `.ac`, not `balloon4`; object-name `balloon`).

## 0.17.1 - Distinct balloon RGB (FG materials)
- Root cause: AC3D `MATERIAL` blue channel was `129` (amb `51.6`) in all `assets/balloons/*.ac` — out of 0..1 range; FG effectively rendered every balloon near-pure blue.
- Fix: `balloon_{255_0_0,0_255_0,0_0_255,sphere}.ac` materials use normalized RGB (red/green/blue/white) with matching ambient.
- Synth already drew `BalloonSpec.color`; add centroid pixel RGB parity test + AC material 0..1/filename contract test. Color map: config RGB → `balloon_R_G_B.xml|.ac` → FG; synth disks use the same RGB.

## 0.17.0 - Altitude-preserving turn trajectory
- `BodyCmdBridge.chase_geometry`: when yaw is known and `|course−yaw|` is large, hold current altitude instead of chasing 3D aim Z (avoids spiral/altitude loss on steep LOS turns); near-aligned heading blends toward clamped aim Z (`max_alt_step_m` still applies).
- Threshold: `guidance.alt_preserve_heading_err_deg` (default 20°) → `BodyCmdBridge.alt_preserve_heading_err_rad`; control passes ATTITUDE yaw into chase setpoints.
- Tests: lateral LOS → `z_hold≈pos_z`; forward LOS → clamped aim Z; mid-error blend; setup default parse.

## 0.16.3 - mavlink-server 0.10.1
- Pin host fetch + Docker bake to bluerobotics [mavlink-server 0.10.1](https://github.com/bluerobotics/mavlink-server/releases/tag/0.10.1) (`python/scripts/fetch_mavlink_server.sh`, `Dockerfiles/PX4NobleSimNvidia.dockerfile` `MAVLINK_SERVER_VERSION`).
- Re-fetch host binary: `python/scripts/fetch_mavlink_server.sh` → `python/bin/mavlink-server` (`--version` → 0.10.1). Image bake needs rebuild for sidecar.
- Fetch downloads to temp then `mv` (avoids curl 23 / ETXTBSY when an old binary is still running).
- Compatible with existing `--mavlink-heartbeat-frequency 0` (0.10.1 #223: do not send heartbeat if frequency ≤ 0).
- Dialect/InvalidCRC on newer PX4 msgs (ESC_INFO etc.): unconfirmed whether 0.10.1 improves vs 0.9.0 — keep `RUST_LOG=off` + log redirect.
- Contract: `test_mavlink_server_version_pinned_to_0_10_1`.

## 0.16.2 - Silence mavlink-server InvalidCRC spam (18570)
- Root cause: PX4 SITL GCS binds local UDP **18570** (`udp_gcs_port_local`) and sends to remote **14550**; that stream includes newer common.xml msgs (`ESC_INFO`/`ESC_STATUS`/`OPEN_DRONE_ID_LOCATION`) whose CRC_EXTRA mavlink-server 0.9.0 rejects → `InvalidCRC origin=127.0.0.1:18570`. Not junk traffic.
- Fix: host fan-out redirects stdout/stderr to `/tmp/mavlink-server-fanout.log` and sets `RUST_LOG=off` (override via `MAVLINK_SERVER_RUST_LOG` / `MAVLINK_SERVER_LOG`); docker sidecar gets `RUST_LOG=off` too. Document 18570 + QGC-must-be-client in `start_mavlink_server.sh` / README.
- Common HEARTBEAT/pose still fan out; unknown-dialect msgs still dropped by mavlink-server (acceptable for race).

## 0.16.1 - Balloon-race tmux uses panes
- `run_balloon_race.sh`: one session / one `race` window; sim + control + image + camera as tiled split panes (targets `session:0.N`); control still starts before image/camera after heartbeat.
- Tear-down unchanged: root `kill.sh` kills by session name (no window-name dependency).
- Contract test asserts pane splits (not `new-window`).

## 0.16.0 - FG race camera fuselage-free (draw-mask + forward eye)
- Problem: FG Cockpit View (z≈+0.9 aft) filled the grab with canopy/struts — unusable for balloon track.
- Fix: FG viz patch **V5** launches with `--prop:/sim/rendering/draw-mask/aircraft=false` and `--prop:/sim/view[0]/config/z-offset-m=-5` (+ FOV 90). `fg_camera.sync_camera_view` reasserts mask=`0`, live+`goal-*` xyz offsets (`fg_eye_forward_m` default 5 m, FG −Z forward), body-relative az/el/HFOV. Window geometry via `xwininfo -id` absolute coords. FG balloon spawn deferred until after engage (background thread).
- Evidence: lit mss grab `python/logs/e2e/fg_verify_clear*.png` — structure_frac≈0 (no mid-grey canopy); mask false, z=-5. Headless e2e still 2 passes (`headless_verify.csv`). FG race still arms late (~55s, deep z) → 0 passes; `compare_balloon_runs.py` **FAIL** (pass 2 vs 0; path RMS ≫30 m). Guidance path: assisted geometric (`seen_track=0`).
- Unit tests: 95 OK.

## 0.15.0 - Headless balloon-race live e2e (altitude frame + chase fixes)
- Root cause of headless crash/no-pass: config balloon Z was ground-relative (≈-80) while PX4 LOCAL_NED home≈aircraft (z≈0), so assisted LOS commanded huge climbs; plus no-track chase kept `balloon_ned(0)` after a pass (orbit old balloon).
- Fix: home-relative balloon NED in `flightSetup*.json` (z≈0/+15/-15); rebase to settled local Z at race start; FG NED origin uses aircraft MSL; spawn AGL ~500 m (`jsb_spawn.xml` / `fg_spawn.env`); start control before image/camera; no-track chase uses active target; gate pass requires XY near balloon; clamp chase `|Δz|` (40 m); CSV `sample` rows + status diagnostics.
- Headless e2e verified: Armed+OFFBOARD held, ≥1 balloon pass (typically 0 and 1), CSV `python/logs/e2e/headless_verify.csv`. FG `--viz` launches but engage is unhealthy / plane already deep — no passes; parity compare vs headless FAILS (expected). `seen_track=0` (camera track not yet live) — assisted geometric only.
- Unit tests: 91 OK.

## 0.14.5 - Usable host mavlink-server for race fan-out
- Prefer `python/bin/mavlink-server`; reject wrong-arch PATH binaries (`--version` probe).
- Add `python/scripts/fetch_mavlink_server.sh`; ignore downloaded binary in `.gitignore`.
- Clearer errors when Noble image lacks baked mavlink-server.

## 0.14.4 - Root kill.sh for balloon race
- Add repo-root `kill.sh`: tear down tmux `balloon_race` + JSBSim (default); forward `--fg`/`--all` to `python/scripts/kill.sh`.
- `python/scripts/kill.sh --jsbsim|--all`: also remove `${JSB_NAME}-mavlink` sidecar and host `mavlink-server`.

## 0.14.3 - Root balloon-race launcher shim
- Add repo-root `run_balloon_race.sh` → `python/scripts/run_balloon_race.sh` (headless default; `--viz` unchanged).

## 0.14.2 - Balloon race sim ownership + color slow-joiner
- `run_balloon_race.sh`: always pass `--no-sim` to control (race owns sim or user `--no-sim`); control must not kill/restart.
- `run_balloon_control.py`: `kill_docker`/`start_sim` only when control owns sim (`not --no-sim`); republish color+assisted every 1 s for late camera SUB; drop unused `poll_mavlink` import.
- Contract: `test_race_passes_no_sim_to_control_when_race_owns_sim`.

## 0.14.1 - Fail empty/unmatched pixel dumps
- `race_compare`: when `--pixels-a/--pixels-b` are provided but yield zero time-matched pairs (empty dumps or unmatched timestamps), pixel check **FAIL**s (exit 1) instead of SKIP/PASS.
- Test: `test_empty_or_unmatched_pixel_dumps_fail`.

## 0.14.0 - Offline race parity compare (delta close)
- `fw_sitl/race_compare.py` + `scripts/compare_balloon_runs.py`: gate two race CSVs vs `verification.*` (pass-time tol, path RMS, optional pixel RMS / dumps); exit 0/1; skip pixels if absent.
- Tests: `test_race_compare.py` (synthetic fixtures). Closes balloon-race delta (0.12–0.13: FG/fan-out/tmux/CSV/end).

## 0.13.0 - tmux heartbeat gate + race CSV / end
- `run_balloon_race.sh`: after sim/fan-out, wait for MAVLink HEARTBEAT on control UDP 14540 (`HEARTBEAT_TIMEOUT_S=120`, fail if none) before image/camera/control.
- Control always writes `/tmp/balloon_race_<timestamp>.csv` (pass + `end_*` rows: idx/color/assisted/pos); `--csv` override.
- End first-wins: `guidance.laps` | `guidance.duration_s` | Ctrl+C (`race_end_reason` + lap count on wrap); plot still optional.
- Tests: `test_race_csv_end.py`; fan-out contracts assert heartbeat wait.

## 0.12.1 - Fan-out fail-loud + defaults
- `runSimJsbsimRascal.sh`: default `MAVLINK_FANOUT=0`; `--mavlink-server` enables; fail non-zero (no soft-skip / stderr swallow) when fan-out requested.
- `run_balloon_race.sh`: enables fan-out by default; aborts image/camera/control if mavlink-server is not up; passes `--udp` 14541/14540.
- `requirements.txt`: add `mss`; synthetic publisher default UDP 14541.

## 0.12.0 - FG Docker ops (balloons mount + mavlink-server)
- `runSimJsbsimRascal.sh --viz`: bind-mount `python/assets/balloons` → `/opt/fixedwing/balloons`; `balloon_scene` rewrites FG model-path to that container path.
- `fg_camera.capture_fg_frame`: window-only capture via title/class regex (`camera.fg_window_pattern`); xdotool → wmctrl → xwininfo; root grab is fallback only.
- Bake bluerobotics `mavlink-server` into Noble image; sim start fans GCS 14550 → control 14540 + image-source 14541 (`--no-mavlink-server` to skip).
- Confirmed viz patch V4: AI models on, `--telnet=5501` (no `--disable-ai-models`).

## 0.11.1 - Fix shadowed dir_cam_to_ned
- Remove duplicate `dir_cam_to_ned` in `camera_model.py` that broke control’s `(dir_cam, CameraModel, roll, pitch, yaw)` call at runtime.

## 0.11.0 - Balloon race (full stack)
- Fix `run_balloon_camera.py` ZMQ/camera_model/balloon_tracker APIs (`CameraModel`, `TrackMessage`, RGB frames).
- Refactor `synthetic_camera` to share `camera_model` projection; add `dir_cam_to_ned` / NED rotation helpers.
- `run_balloon_control.py`: OFFBOARD chase (`RaceGuidance` + `BodyCmdBridge`), color PUB, track SUB with held `dir_cam`, FG balloon spawn with `--viz`.
- FG viz patch V4: AI models on, `--telnet=5501`; `assets/balloons/*.ac`; `test_synthetic_parity` pixel parity.

## 0.10.0 - Balloon-race foundation (Phase 1)
- Add `python/flightSetup.json` example + `fw_sitl.flight_setup` loader/defaults.
- Add `zmq_bus` (image/color/track PUB/SUB), `camera_model` (FOV/mount, pixel↔cam LOS), `balloon_tracker` (color blob → cam unit LOS).
- Add `python/requirements.txt` (numpy, pyzmq, opencv-python-headless); unit tests for LOS roundtrip + synthetic disk track.

## 0.9.0 - Python folder layout (`fw_sitl` package)
- Move utilities into `python/fw_sitl/`; shells → `python/scripts/`; spawn files → `python/assets/`; tests → `python/tests/`.
- Entrypoints stay at `python/run_straight_flight_*.py` with `fw_sitl` imports + path bootstrap; old shell paths are thin shims.
- `sim_lifecycle` exposes `PYTHON_ROOT` / `SCRIPTS_DIR`; shell scripts resolve assets under `../assets` and repo `Dockerfiles/`.

## 0.8.1 - Wrap engage retry reconnect as EngageError
- `engage_offboard_with_retries`: convert reconnect/reboot/restart failures to `EngageError` (runners keep plant hints).
- Drop unused JSBSim imports (`Path`, `DEFAULT_LOOKAHEAD_M`); remove unused `sys` from `mavlink_io`.

## 0.8.0 - Split straight-flight Python modules
- Extract `path_geometry`, `mavlink_io`, `sim_lifecycle`, `cli_common`, `straight_flight_core` from the JSBSim runner.
- Thin `run_straight_flight_{jsbsim,yasim}.py` entrypoints (CLI + plant engage policy); YASim uses shared hold loop.
- Tests import path helpers from `path_geometry`; shims unchanged.

## 0.7.6 - Fix --viz fall / early exit / missing plot
- Cause: FG delays EKF health → arm denied ~10–20 s (plane falls); late lock marked unhealthy → reboot retries → `Engage failed` → exit 1 with no plot.
- `--viz`: `accept_unhealthy=True`, single attempt (no reboot loop), softer healthy thresholds; extra arm bypasses (`COM_ARM_SDCARD/HFLT_CHK/MAG_STR=0`).
- Clearer stderr when engage fails before any hold/plot.

## 0.7.5 - Stop ~50 m NED Z jumps (GPS alt vs baro)
- Cause: EKF GPS altitude fusion vs baro height reference snapped local Z mid-hold (~50 m cliff) while XY stayed smooth.
- `EKF2_HGT_REF=0` (baro), `EKF2_GPS_CTRL=5` (lon/lat+vel, no GPS alt); softer baro gate/noise.
- After arm: `settle_path_altitude` waits for stable Z before locking `z_hold` / starting history; re-lock on |Δz|>15 m during hold.
- Path summary reports max|Δz|/|Δxy|.

## 0.7.4 - Stop mid-hold NED jumps (OFFBOARD←local-pos invalid)
- Cause: position OFFBOARD is marked lost when `local_position_invalid` flickers (PX4 `offboardCheck`), even with live setpoints → failsafe cascade to ALTCTL + EKF/NED cliffs on plots.
- Soften EKF GNSS / dead-reckon (`EKF2_GPS_*` noise/gates, `EKF2_GPS_MODE=1`, `EKF2_NOAID_TOUT` max); `COM_OF_LOSS_T=60`, `COM_OBL_RC_ACT=Hold`, fix invalid `NAV_RCL_ACT=0`.
- Hold loop: restore OFFBOARD immediately on mode leave (no spam while in 6); on >40 m NED step, re-lock path origin so setpoints do not yank after an EKF snap.

## 0.7.3 - Make JSBSim --viz flight match headless
- Root cause: `--viz` inherited FG/docker stdout into the Python process (100s of MB), stalling OFFBOARD setpoints → mode flips / EKF sawtooth on plots; `accept_unhealthy` also locked drifted EKF.
- `start_sim --viz`: log to `/tmp/jsbsim_viz_runner_<pid>.log` (never inherit); headless still DEVNULL.
- `--viz` engage: same healthy path-lock as headless (`accept_unhealthy=False`), 60 s arm wait, still no full container restart.
- FG viz patch V3: `nice -n 15`, 15 Hz FPS cap, disable clouds/sound — keep JSBSim lockstep closer to headless.

## 0.7.2 - Fix --viz black screen / 10s restart loop
- Cause: late arm (>3.5 s) marked unhealthy → `full_sim_restart` killed JSBSim+FG every ~12 s; FG never finished loading (black view).
- `--viz`: `accept_unhealthy`, 45 s arm timeout, `full_sim_restart=False` (autopilot reboot only).
- FG viz patch V2: `--timeofday=noon` (less black/night default).

## 0.7.1 - Final-review viz I/O + bridge hint
- `start_sim`: with `--viz`, inherit stdout/stderr (headless still DEVNULL).
- Viz path `jsbsim_bridge` missing check now prints same rebuild hint as headless.

## 0.7.0 - JSBSim FG viz + YASim rename
- Rename headless → `run_straight_flight_jsbsim.py` / keep `runSimJsbsimRascal.sh`; add `--viz` (FG `--fdm=null` on same JSBSim plant).
- Rename FG YASim path → `run_straight_flight_yasim.py` + `runSimYasimRascal.sh`; old names shim.
- `Dockerfiles/patch_px4_jsbsim_fg_viz.sh`: TerraSync off + FG logs for JSBSim viz launch.

## 0.6.4 - FG no longer restarts on engage
- Cause: engage treated FG as failed (`have_pos` required `|z|>5`, 12 s arm timeout), then killed the FG container and respawned.
- Fix: any `LOCAL_POSITION_NED` counts as pose; FG uses 45 s arm timeout, `accept_unhealthy`, and autopilot reboot only (`full_sim_restart=False`). Headless JSBSim still full-sim restarts.
- Engage wait logs STATUSTEXT arm/failsafe lines + periodic armed/have_pos.

## 0.6.3 - FG history plot parity + post-engage stream refresh
- `run_straight_flight.py`: same `FlightHistory` recording/plot as headless (NED pos/vel, attitude, along/cross-track, 3D); recreate + `request_streams` after engage so retries do not leave an empty plot.
- Headless: same post-engage history refresh.

## 0.6.2 - FG straight flight matches headless locked-line hold
- `run_straight_flight.py`: drop carrot-from-current-XY; reuse headless ASAP arm, path lock at arm, FW position-only closest-on-line hold, SITL param/reboot prep, engage retries, and `flight_history` along/cross plot.

## 0.6.1 - Straight-flight plot clarity + FW position-only SP
- Explain/fix plot confusion: raw `x(N)` is North, not along-track; sawtooth ↔ brief OFFBOARD→ALTCTL turns + EKF jumps.
- `flight_history`: along-track / cross-track panel + path summary; log mode flips during hold.
- FW OFFBOARD type_mask is position-only (PX4 ignores velocity on fixed-wing); `EKF2_GPS_CHECK=0` for SITL.

## 0.6.0 - Fix headless straight-flight drift
- Root causes: (1) carrot setpoint from current XY never corrected cross-track; (2) late arm / path lock after EKF wander; (3) OFFBOARD exit on position/offboard-loss failsafes.
- `run_straight_flight_headless.py`: ASAP OFFBOARD force-arm with ahead-on-yaw bridge; lock origin/course/Z at arm (course←yaw unless `--course-deg`); stream closest-on-line path + tangent; on unhealthy arm (slow / drifted EKF) restart the JSBSim container and retry; INT32-safe params + reboot; softened `COM_POS_FS_*` / `COM_OF_LOSS_T`; `jsb_spawn`/default speed ~30 m/s.
- Verified 3/3 × ~40 s holds after healthy engage: cross-track RMS ~11–27 m (was ~800 m), along-track ~825–860 m, mode 6 stable.
- `flight_history.last_att_rad`; unit tests for path projection (+ bank helpers retained).

## 0.5.1 - 3D trajectory figure
- `flight_history.plot`: second matplotlib window with 3D North-East-Up path (start/end markers).

## 0.5.0 - Post-flight history plot (headless)
- `python/flight_history.py`: buffer LOCAL_POSITION_NED + ATTITUDE; matplotlib 3-panel plot (pos/vel/attitude).
- `run_straight_flight_headless.py`: record during hold; show plot after finish (Ctrl+C or `--duration`); `--no-plot` to skip. SIGINT/SIGTERM break the loop (plot then stop sim) instead of exiting immediately.

## 0.4.7 - kill.sh at flight script start
- `run_straight_flight.py` / `run_straight_flight_headless.py`: call `kill.sh` (`--fg` / `--jsbsim`) at start of `main()` before connect/sim start (also with `--no-sim`); keep runner `--kill` in `start_sim` and atexit cleanup.

## 0.4.6 - kill.sh for sim containers
- `python/kill.sh`: remove FG (`--fg`, default), JSBSim (`--jsbsim`), or both (`--all`).

## 0.4.5 - Detach Docker from debug console TTY
- Python sim start uses `stdin=DEVNULL` so runners do not `docker run -it` on the IDE/debug TTY (rerun was typing into the live PX4 shell).
- Shell runners honor `PX4_SITL_NO_DOCKER_TTY` to force non-TTY docker.

## 0.4.4 - Kill leftover Docker before sim restart
- `run_straight_flight.py` / `run_straight_flight_headless.py`: call sim `--kill` before starting so debugger stop/rerun does not leave the previous container running.
- Register `atexit` cleanup (idempotent with SIGINT/SIGTERM handlers).

## 0.4.3 - FG straight flight uses ahead waypoint + Z hold
- `run_straight_flight.py`: drop BODY_NED velocity-only (crab + no alt hold); same LOCAL_NED ahead-waypoint + locked Z + course velocity as headless.
- Defaults: 30 m/s (matches `fg_spawn.env`), `--lookahead` 300 m, `--rate` 20 Hz, force-arm for in-air SITL.

## 0.4.2 - OFFBOARD ahead waypoint + locked altitude
- Headless: velocity-only OFFBOARD left FW in MODE_OTHER (no TECS); now stream LOCAL_NED position ahead on course at engage `z` + course velocity (`--lookahead`, default 300 m).
- Enables PX4 FW `AUTO_PATH` altitude/course hold; force-arm immediately for in-air SITL; lock `z` on first real local sample if engage was early.

## 0.4.1 - FW OFFBOARD uses fixed LOCAL_NED course
- Headless runner: stop using BODY_NED velocity (causes FW crabbing); lock course from heading at engage (or `--course-deg`).
- `MAV_CMD_DO_CHANGE_SPEED` sets airspeed target to the same `|v|`.

## 0.4.0 - Headless engage ASAP at 1.5×Vstall
- `run_straight_flight_headless.py`: no warmup by default; stream setpoints → OFFBOARD → arm (force-arm fallback) immediately after heartbeat.
- Default speed `1.5 * Rascal Vstall` (10 kt → ~7.72 m/s); `--rate` default 20 Hz.
- `jsb_spawn.xml` IC airspeed aligned to 1.5×Vstall (~25.3 ft/s).

## 0.3.1 - Prebuild jsbsim_bridge in image
- Dockerfiles 1.5.0: bake `jsbsim_bridge` into `px4-noble-sim-ros` (runtime `--rm` was discarding compile).
- `runSimJsbsimRascal.sh`: no runtime apt/build; requires rebuilt image.

## 0.3.0 - Headless JSBSim straight flight
- `python/runSimJsbsimRascal.sh` — Docker headless Rascal via `HEADLESS=1 make px4_sitl jsbsim_rascal`.
- `python/jsb_spawn.xml` — in-air IC (~500 m AGL, ~30 m/s) mounted over default LSZH scene.
- `python/run_straight_flight_headless.py` — OFFBOARD straight hold using the JSBSim runner.
- Smoke: heartbeat on UDP 14540; VFR_HUD showed ~32 m/s airspeed and airborne altitude (no FG window).

## 0.2.1 - Git repo
- Initialized root git repository (`main`); `.gitignore` present. `Dockerfiles/` remains its own nested git checkout.

## 0.2.0 - Fix airspeed; Rascal RPM path
- Bridge patch: `HIL_SENSOR.id=0` + zero-init; runner re-applies patch and rebuilds `flightgear_bridge`.
- Rascal RPM via `/engines/engine/rpm` → `RAW_RPM` on GCS.
- Verified under OFFBOARD: `VFR_HUD.airspeed` finite, `VFR_HUD.throttle` / `SERVO_OUTPUT_RAW` / `ACTUATOR_OUTPUT_STATUS` move with thrust, `RAW_RPM.frequency` non-zero. Prefer `--udp 14540` so QGC keeps 14550.

## 0.1.0 - Project bootstrap
- Host helpers under `python/` for FlightGear Rascal SITL + OFFBOARD straight flight.
