"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from x31.types import CHANNEL_GAINS, CHANNEL_NDI, CHANNEL_PLANT

_ROOT = Path(__file__).resolve().parents[2]
_SIM_DIR = _ROOT / "sim_result"
_REF_DIR = _ROOT / "python" / "reference"
# Version-controlled: the floor the overlay contract is measured against.
HORIZONS = _REF_DIR / "horizons.json"

_VCHI_GROUPS = (
    "ndi__Group_5",
    "gains__bOrrOl_roll",
    "gains__berrol_roll",
)
_ATTRS = ("model", "group", "schedule_matched")
# Regenerated logs carry this suffix so the stored logs stay untouched.
_RERUN_SUFFIX = "_v031"
# Same relative tolerance the trajectory overlay uses, applied per column
# because the command vector mixes m/s and radians.
_SCHEDULE_TOL = 1e-3
# The trajectory tolerance from the plan, unchanged. `pass_horizon` applies it
# per channel and reports where it first breaks; nothing here relaxes it.
_PASS_TOL = 1e-3


def _command_name() -> str:
    for catalog in (CHANNEL_PLANT, CHANNEL_NDI, CHANNEL_GAINS):
        for name in catalog:
            if name == "x31control_cmdVelAng":
                return name
    raise KeyError("x31control_cmdVelAng")


_CMD = _command_name()


def stems() -> list[str]:
    return sorted(path.stem for path in _SIM_DIR.glob("*.mat"))


def load(stem: str) -> dict:
    """Read one case. A regenerated `_v031` log wins over the stored one.

    ``source`` says which: ``"rerun_v0.3.1"`` or ``"stored"``. The stored log
    stays on disk for the plots and for the revision-gap record.
    """
    path = load_path(_REF_DIR, stem)
    rerun = path == rerun_path(_REF_DIR, stem)
    with np.load(path) as archive:
        data = {
            key: _channel_array(archive[key])
            for key in archive.files
            if key not in _ATTRS
        }
        data["model"] = _text(archive["model"])
        data["group"] = _text(archive["group"])
        data["schedule_matched"] = _flag(archive["schedule_matched"])
    if stem.startswith("x31_03_"):
        # Pairing is derived from the logged command, so recompute it against
        # the signal_groups.npz that ships with the port. A flag written by an
        # older export must not outvote the numbers.
        groups = _groups()
        _model, group, matched, closest = _schedule(stem, data, groups)
        for key in [name for name in data if _is_group_key(name)]:
            del data[key]
        data["group"] = group
        data["schedule_matched"] = matched
        # The plan keeps an unmatched case runnable on its closest waveform, so
        # the best candidate travels separately from the verified answer.
        data["closest_group"] = group or closest
        if data["closest_group"]:
            data[data["closest_group"]] = np.array(
                groups[data["closest_group"]], copy=True
            )
    data["source"] = "rerun_v0.3.1" if rerun else "stored"
    return data


_GROUPS_CACHE: dict = {}


def _groups() -> dict[str, np.ndarray]:
    """The exported Signal Builder waveforms, read once per directory."""
    key = str(_REF_DIR)
    if key not in _GROUPS_CACHE:
        with np.load(_REF_DIR / "signal_groups.npz") as archive:
            _GROUPS_CACHE[key] = {name: archive[name] for name in archive.files}
    return _GROUPS_CACHE[key]


def _is_group_key(name) -> bool:
    return name.startswith(("ndi__Group_", "gains__"))


def rerun_path(directory: Path, stem: str) -> Path:
    """Where a regenerated log for `stem` belongs."""
    return Path(directory) / f"{stem}{_RERUN_SUFFIX}.npz"


def load_path(directory: Path, stem: str) -> Path:
    """The regenerated log when it exists, otherwise the stored one."""
    fresh = rerun_path(directory, stem)
    if fresh.is_file():
        return fresh
    return Path(directory) / f"{stem}.npz"


def convert_exported_mats() -> None:
    """Turn MATLAB plain mats into npz logs. Called by export_sim_results.m."""
    from scipy.io import loadmat

    _REF_DIR.mkdir(parents=True, exist_ok=True)
    groups = _load_plain(loadmat, _REF_DIR / "signal_groups_plain.mat")
    np.savez(_REF_DIR / "signal_groups.npz", **groups)
    _GROUPS_CACHE.pop(str(_REF_DIR), None)
    (_REF_DIR / "signal_groups_plain.mat").unlink()

    for plain in sorted(_REF_DIR.glob("*_plain.mat")):
        stem = plain.name[: -len("_plain.mat")]
        arrays = _load_plain(loadmat, plain)
        model, group, matched, closest = _schedule(stem, arrays, groups)
        payload = dict(arrays)
        payload["model"] = np.asarray(model)
        payload["group"] = np.asarray(group or closest)
        payload["schedule_matched"] = np.asarray(bool(matched))
        if group or closest:
            payload[group or closest] = groups[group or closest]
        np.savez(_REF_DIR / f"{stem}.npz", **payload)
        plain.unlink()
        print(f"{stem} model={model} group={group or '-'} schedule_matched={matched}")


def write_horizons(measured: dict | None = None) -> Path:
    """Write the MATLAB self-consistency measurement to the versioned file."""
    if measured is None:
        measured = {stem: self_consistency(stem) for stem in stems() if has_self_consistency(stem)}
    with HORIZONS.open("w", encoding="utf-8") as handle:
        json.dump(measured, handle, indent=1, sort_keys=True)
    return HORIZONS


def has_self_consistency(stem: str) -> bool:
    return (_REF_DIR / f"{stem}_selfB.npz").is_file()


def read_horizons() -> dict:
    """The committed MATLAB self-consistency measurement."""
    if not HORIZONS.is_file():
        return {}
    with HORIZONS.open(encoding="utf-8") as handle:
        return json.load(handle)


def convert_self_consistency_mats() -> dict:
    """Convert the `<stem>_selfB_plain.mat` staging files and measure horizons.

    Returns {stem: {channel: seconds}} - the time up to which two MATLAB runs of
    the same model, differing only in solver tolerance, still agree under the
    trajectory tolerance. That is the floor the port is held to.
    """
    from scipy.io import loadmat

    _REF_DIR.mkdir(parents=True, exist_ok=True)
    measured = {}
    for plain_path in sorted(_REF_DIR.glob("*_selfB_plain.mat")):
        stem = plain_path.name[: -len("_selfB_plain.mat")]
        arrays = _load_plain(loadmat, plain_path)
        np.savez(_REF_DIR / f"{stem}_selfB.npz", **arrays)
        plain_path.unlink()
        measured[stem] = self_consistency(stem)
        print(f"{stem} self-consistency horizons measured")
    write_horizons(measured)
    return measured


def self_consistency(stem: str) -> dict[str, float]:
    """Per-channel horizon between the stored rerun and its tighter twin.

    The two runs land on different time grids - they are variable-step runs and
    the tighter one takes different steps - so the second is interpolated onto
    the first's grid before anything is compared. Comparing sample by sample
    across two grids measures the grids, not the model.
    """
    first = load(stem)
    second = _load_npz(_REF_DIR / f"{stem}_selfB.npz")
    grid = np.asarray(first["x31sim_time"], dtype=float).reshape(-1)
    out = {}
    for key, y_b in second.items():
        name = str(key)
        if not name.startswith("x31") or name.endswith("_Time"):
            continue
        if name in ("x31sim_time", "x31control_time"):
            continue
        if name not in first:
            continue
        b_grid, b_values = _channel_on_grid(second, name)
        overlap = float(grid[-1]) if b_grid[-1] < grid[-1] else float(b_grid[-1])
        keep = grid <= overlap
        if not keep.any():
            continue
        b_on_a = _interp_rows(b_grid, b_values, grid[keep])
        out[name] = pass_horizon(
            np.asarray(first[name], dtype=float)[keep],
            b_on_a,
            grid[keep],
            quaternion_columns="stateR" in name,
        )
    return out


def _channel_on_grid(data: dict[str, np.ndarray], name: str):
    """A channel and the time vector it was sampled at.

    The two runs log every channel on the same grid, so this currently returns
    `x31sim_time` for all of them; the per-channel `_Time` vector is preferred
    so a log that does vary is still handled.
    """
    values = np.asarray(data[name], dtype=float)
    if values.ndim == 1:
        values = values.reshape(1, -1)
    time_key = f"{name}_Time"
    if time_key in data:
        grid = np.asarray(data[time_key], dtype=float).reshape(-1)
    else:
        grid = np.asarray(data["x31sim_time"], dtype=float).reshape(-1)
    count = min(grid.size, values.shape[0])
    return grid[:count], values[:count]


def _interp_rows(grid, values, query) -> np.ndarray:
    if values.shape[1] == 1:
        return np.interp(query, grid, values[:, 0]).reshape(-1, 1)
    return np.column_stack(
        [np.interp(query, grid, values[:, column]) for column in range(values.shape[1])]
    )


def pass_horizon(y_a, y_b, time, quaternion_columns: bool = False) -> float:
    """Largest time in `time` up to which the two series hold `_pass`.

    `_pass` is unchanged: the series must agree within 1e-3 * max(1, max|y_b|).
    The horizon is the first time the cumulative error crosses that bound, so a
    later excursion cannot hide an earlier one. `y_b` sets the scale, so the
    reference defines the tolerance.

    A To Workspace log holds one entry per major step, so a variable-step run
    that lands several steps on the same instant records that instant more than
    once - at the 5 ms transport-delay boundary it records two different values
    for what is t = 0.005. The timestamps are not bit-equal: they differ by
    about 1e-16, so the collapse has to be tolerant or it never fires.

    Repeated instants are collapsed to their last sample. The other two
    time-vector readers in this package (`maneuver.command`, `_sample_group`)
    keep the first via `np.unique(..., return_index=True)`; that difference is
    deliberate and is noted in _SHORTFALLS, which is where it shows up.
    """
    a = np.asarray(y_a, dtype=float)
    b = np.asarray(y_b, dtype=float)
    grid = np.asarray(time, dtype=float).reshape(-1)
    if a.ndim < 2 or b.ndim < 2 or a.shape[1:] != b.shape[1:]:
        return 0.0
    if not np.all(np.isfinite(b)):
        # A non-finite reference has no meaningful scale and no meaningful
        # tolerance, so nothing can be said about agreement. Score it as no
        # agreement rather than as a pass.
        return 0.0
    # The plan's scale is max(1, max|y_ref|) over the whole reference, so it is
    # taken before any truncation by the port's own stop time.
    scale = max(1.0, float(np.max(np.abs(b))))
    count = min(a.shape[0], b.shape[0], grid.size)
    a = a[:count]
    b = b[:count]
    grid = grid[:count]
    keep = _last_per_timestamp(grid)
    a = a[keep]
    b = b[keep]
    grid = grid[keep]
    if grid.size == 0:
        return 0.0
    if quaternion_columns and a.shape[1] == 7:
        # Rates on 0:3 with no sign freedom. The quaternion on 3:7 with ONE
        # global sign, per the plan's rule min(||q-q_ref||, ||q+q_ref||):
        # the max over components is taken inside each of the two alternatives,
        # not per component. Choosing a sign per component would forgive the
        # conjugate - a different rotation entirely - at zero error.
        quat = np.minimum(
            np.abs(a[:, 3:7] - b[:, 3:7]).max(axis=1),
            np.abs(a[:, 3:7] + b[:, 3:7]).max(axis=1),
        )
        per_sample = np.maximum(quat, np.abs(a[:, :3] - b[:, :3]).max(axis=1))
    else:
        per_sample = np.abs(a - b).max(axis=1)
    # A NaN would latch through np.maximum.accumulate and then compare False,
    # scoring a diverged channel as a full-length pass. Treat non-finite
    # samples as infinite error instead.
    per_sample = np.where(np.isfinite(per_sample), per_sample, np.inf)
    running = np.maximum.accumulate(per_sample)
    bad = np.nonzero(running > _PASS_TOL * scale)[0]
    return float(grid[bad[0]]) if bad.size else float(grid[-1])


# Two samples belong to the same instant if they are within this of each other.
# Major steps that land on the same time differ by a few ulp, not by zero.
_INSTANT_TOL = 1e-9


def _last_per_timestamp(grid) -> np.ndarray:
    """Indices of the last sample at each distinct instant, in order."""
    if grid.size < 2:
        return np.arange(grid.size)
    later = np.nonzero(np.diff(grid) > _INSTANT_TOL)[0]
    return np.append(later, grid.size - 1)


def _load_npz(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as archive:
        return {key: _channel_array(archive[key]) for key in archive.files}


def convert_rerun_mats() -> None:
    """Turn the rerun staging mats into `<stem>_v031.npz`. Called by rerun_x31_03.m."""
    from scipy.io import loadmat

    _REF_DIR.mkdir(parents=True, exist_ok=True)
    for plain_path in sorted(_REF_DIR.glob(f"*{_RERUN_SUFFIX}_plain.mat")):
        stem = plain_path.name[: -len(f"{_RERUN_SUFFIX}_plain.mat")]
        arrays = _load_plain(loadmat, plain_path)
        model, group, matched, _closest_name = _schedule(stem, arrays, _groups())
        payload = dict(arrays)
        payload["model"] = np.asarray(model)
        payload["group"] = np.asarray(group)
        payload["schedule_matched"] = np.asarray(bool(matched))
        np.savez(rerun_path(_REF_DIR, stem), **payload)
        plain_path.unlink()
        print(f"{stem} rerun model={model} group={group or '-'} schedule_matched={matched}")


def _load_plain(loadmat, path: Path) -> dict[str, np.ndarray]:
    raw = loadmat(path, squeeze_me=False, struct_as_record=False)
    arrays = {}
    for key, value in raw.items():
        if key.startswith("_"):
            continue
        arrays[key] = _channel_array(value)
    return arrays


def _channel_array(value) -> np.ndarray:
    """MATLAB column-major vectors as 1-D.

    A row vector is left alone: a single-sample command such as
    `x31control_cmdVelAng` is a (1, 3) array and callers rely on that width.
    """
    arr = np.asarray(value)
    if arr.ndim == 2 and arr.shape[1] == 1 and arr.shape[0] > 1:
        return np.ascontiguousarray(arr[:, 0])
    return np.ascontiguousarray(arr)


def _text(value) -> str:
    item = np.asarray(value).item()
    if isinstance(item, bytes):
        return item.decode()
    return str(item)


def _flag(value) -> bool:
    return bool(np.asarray(value).item())


def _schedule(stem: str, arrays: dict[str, np.ndarray], groups: dict[str, np.ndarray]):
    """Model name, verified group, matched flag, and the closest candidate.

    Model name and the Signal Builder group that actually flew this log.

    Pairing is numeric: the candidate whose V/Chi/Gamma waveform reproduces
    the logged `x31control_cmdVelAng` within tolerance wins. The filename
    token is not evidence - the GainS barrel-roll logs reproduce the NDI
    model's `Group 5` table to 1.4e-14 - so no token branch exists.

    `group` is empty unless the residual is inside tolerance. A case whose
    command cannot be placed on any waveform reports no group rather than the
    closest candidate.
    """
    model = _model_name(stem)
    if not stem.startswith("x31_03_"):
        # x31_01 and x31_02 are plotted, never used as an equality target.
        return model, "", False, ""
    # Only the (time, V, Chi, Gamma) tables are candidates. ndi__Group_1..4 and
    # gains_____1 are not: their third column holds 80000, a constant 60, or a
    # thrust in newtons, so converting it from degrees is meaningless and the
    # residual could never be small.
    keys = [key for key in _VCHI_GROUPS if key in groups]
    ranked = _rank(arrays, groups, keys)
    group, matched = _closest(ranked)
    closest = ranked[0][1] if ranked else ""
    return model, group, matched, closest


def _model_name(stem: str) -> str:
    if "NDI" in stem:
        return "ndi"
    if "GainS" in stem:
        return "gain_schedule"
    return "plant"


def _rank(arrays, groups, keys: list[str]):
    """Candidates scored against the logged command, best first."""
    if _CMD not in arrays or not keys:
        return []
    t_mat, y_mat = _command_series(arrays)
    if t_mat is None:
        return []
    scored = []
    for key in keys:
        y_py = _sample_group(groups[key], t_mat)
        if y_py.shape != y_mat.shape:
            continue
        # Per column: V is m/s and the angles are radians, so one max-abs scale
        # over the whole vector is always the airspeed and would leave ~6 deg of
        # slack on the angles at V = 100.
        score = 0.0
        for column in range(y_mat.shape[1]):
            column_err = float(np.max(np.abs(y_py[:, column] - y_mat[:, column])))
            column_scale = max(1.0, float(np.max(np.abs(y_mat[:, column]))))
            score = max(score, column_err / column_scale)
        scored.append((score, key))
    scored.sort(key=lambda item: (item[0], item[1]))
    return scored


def _closest(ranked):
    """The verified group from a ranking, or ("", False) when none can be shown.

    `group` is published only when the residual is inside tolerance on every
    column, so a caller never receives a waveform that was merely tried. Use
    `closest_group` for the best candidate when the flag is False.
    """
    if not ranked or ranked[0][0] > _SCHEDULE_TOL:
        return "", False
    return ranked[0][1], True


def _command_series(arrays: dict[str, np.ndarray]):
    """The logged command and the time it was sampled at, or (None, None).

    A command with no time vector, or one whose time vector disagrees with its
    own length, cannot be placed on a waveform. Both cases return None rather
    than inventing an axis.
    """
    y_mat = np.asarray(arrays[_CMD], dtype=float)
    if y_mat.ndim == 1:
        y_mat = y_mat.reshape(1, -1)
    t_key = _CMD + "_Time"
    if t_key not in arrays:
        return None, None
    t_mat = np.asarray(arrays[t_key], dtype=float).reshape(-1)
    if t_mat.size != y_mat.shape[0]:
        return None, None
    return t_mat, y_mat


def _sample_group(group: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    """A V/Chi/Gamma group sampled at t_query, with the angles in radians.

    A group is [time, V, Chi, Gamma] and the Signal Builder holds the angles in
    degrees, so only columns 1: of the sampled result are converted.
    """
    time = np.asarray(group[:, 0], dtype=float)
    values = np.asarray(group[:, 1:], dtype=float)
    order = np.argsort(time)
    time = time[order]
    values = values[order]
    time, unique_idx = np.unique(time, return_index=True)
    values = values[unique_idx]
    sampled = np.column_stack([np.interp(t_query, time, values[:, col]) for col in range(values.shape[1])])
    if sampled.shape[1] == 3:
        sampled[:, 1:] = np.deg2rad(sampled[:, 1:])
    return sampled
