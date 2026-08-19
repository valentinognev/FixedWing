# Updates

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

## 0.23.5 - Race plots open on the desktop after tmux teardown
- Control runs in a detached tmux pane, so matplotlib `plt.show()` never appeared on the host display. After a race, PNGs are written next to the CSV (`/tmp/balloon_race_<stamp>_history.png` and `_trajectory.png`) and opened with `xdg-open`. `--no-plot` still skips both.

## 0.23.4 - LOS until abeam; look-at holds altitude
- Race LOS plot was NED bearing to the *chase* balloon (`az=0` north). That stays ~15° on a constant-bearing intercept, and `pass_radius` retargets ~50 m *ahead*, so a miss never showed the ±90° abeam swing. Series is now az vs ground track to the balloon until abeam (0=on course).
- On-screen look-at pitch mixes the altitude loop (`kp_alt`); geometric `los_el` alone was only a few degrees far out, so the plane stayed high and elevation drifted to about −4°.

## 0.23.3 - Gazebo timed end no longer restarts
- `runSimGzPlane.sh` GPU→CPU fallback only on an immediate launch failure. `docker rm -f` after a live race (exit 137/143 or ≥30 s) stops the script instead of starting a second Gazebo.

## 0.23.2 - NED plot dashed current target
- Figure 1 NED position overlays dashed tgt x/y/z (current balloon at each time, same colors as the plane).

## 0.23.1 - 3D history marks balloons
- Figure 2 (3D trajectory) scatters each race balloon at rebased NED, colored RGB, and the equal-aspect box includes them.

## 0.23.0 - Timed race teardown, target log, LOS plots
- `./run_balloon_race.sh` runs `kill.sh --all` before starting a new plant (not with `--no-sim`).
- Default `guidance.duration_s` is 60 s; `0` / `--duration 0` has no time limit. `--duration SEC` on the launcher (else `BALLOON_RACE_DURATION`).
- Race-owned runs pass `--stop-sim-on-exit`: after duration/laps/Ctrl+C, remove SITL docker stacks, then plot vs time (`--no-plot` to skip).
- Race CSV adds `tgt_n/e/d` beside plane `pos_n/e/d`. History plots geometric LOS az/el to the current balloon and plane−target ΔN/ΔE/ΔD (JSBSim / `--viz` / YASim / Gazebo).

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

## 0.19.2 - Attitude hold crabbed ~80 m off the line
- Yaw P cancelled xt P at ~0° roll (peak xt −83 m, then slow return). Bank now uses ground track vs course+lookahead intercept.
- Missing ATTITUDE used identity (north) vs a west lock → 90° fake yaw error; hold until real attitude.

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
