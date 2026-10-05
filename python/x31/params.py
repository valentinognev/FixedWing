"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

from pathlib import Path

import numpy as np

_REFERENCE_DIR = Path(__file__).resolve().parents[1] / "reference"
# The plant copy first, then the bare-init copy. `x31_01_sim_trim180s` has no
# `X31_init*` keys at all, while the 0.3.1 logs carry the plant copy only.
_TABLE_PREFIXES = ("X31_initP_", "initP_")

# Figure 2.2 from X31dynamics_init_poly. Axis 1 is a then b.
# Columns are powers 1..6, constant, odd=1 / even=0.
_FIG22 = np.array(
    [
        [  # 1 Cl / Cd
            [0.068166435, 0.000838565, -7.53902e-05, 1.05377e-06, -4.55181e-09, -1.73074e-13, 0.0, 1],
            [0.009653169, -0.000177299, 2.91271e-05, -4.79288e-07, 2.16121e-09, 0.0, 0.003815982, 0],
        ],
        [  # 2 Cl delta canard / canard lift offset
            [-2.98973e-05, 2.87863e-06, -1.12387e-07, 1.83108e-09, -1.36183e-11, 3.85119e-14, 0.002310071, 0],
            [-0.161987094, -0.061576775, 0.001116867, -4.89738e-06, -1.82458e-08, 1.83146e-10, -0.015086919, 1],
        ],
        [  # 3 CY / CY with rudder
            [0, 0, 0, 0, 0, 0, -1, 0],
            [0.000129449, -1.68066e-05, 4.58837e-07, -4.98703e-09, 1.931e-11, 0, 0.003236518, 0],
        ],
        [  # 4 Cl with beta / Cn with beta
            [0.010568349, -0.001180732, 3.75585e-05, -4.51658e-07, 1.81067e-09, 4.91796e-13, -0.083588788, 0],
            [-0.009013965, -0.000415504, 1.97183e-05, -2.69626e-07, 1.27567e-09, -6.5435e-13, 0.175202791, 0],
        ],
        [  # 5 Cl with p / Cn with p
            [0.009368111, -0.001403253, 5.02484e-05, -6.593e-07, 2.89722e-09, 2.46726e-13, -0.078285489, 0],
            [0.001564168, -0.001140853, 5.19084e-05, -7.95876e-07, 3.84628e-09, 1.11711e-12, -0.000286817, 1],
        ],
        [  # 6 Cl with r / Cn with r
            [-0.022525193, 0.003573161, -0.000125695, 1.67113e-06, -7.98271e-09, 4.30185e-12, -0.0758901, 1],
            [0.017397535, -0.001987539, 6.01596e-05, -6.69396e-07, 2.49322e-09, 3.48365e-13, -0.32966209, 0],
        ],
        [  # 7 Cl with aileron / Cn with aileron
            [5.11295e-05, -6.16355e-06, 1.57143e-07, -1.7579e-09, 8.54994e-12, -1.20465e-14, 0.002346565, 0],
            [2.49532e-05, -2.72807e-06, 7.90021e-08, -1.01864e-09, 5.8109e-12, -1.07828e-14, 0.000760346, 0],
        ],
        [  # 8 Cl with rudder / Cn with rudder
            [1.61374e-05, -1.74829e-06, 4.94371e-08, -6.15656e-10, 3.3101e-12, -5.31398e-15, 0.000507964, 0],
            [-9.46234e-05, 1.16858e-05, -3.46362e-07, 4.56883e-09, -2.82200e-11, 6.61620e-14, -1.91018e-03, 0],
        ],
        [  # 9 Cm0 / Cm with q
            [0.002907947, 8.80921e-05, -6.66764e-06, 1.44272e-07, -1.34607e-09, 4.6273e-12, 0, 1],
            [0, 0, 0, 0, 0, 0, -3, 0],
        ],
        [  # 10 Cm with canard / canard pitching-moment offset
            [-4.46671e-05, 3.84916e-06, -1.32161e-07, 1.54809e-09, -5.0072e-12, -9.9798e-15, 0.005052863, 0],
            [-0.137580555, -0.070398444, 0.001549639, -1.2966e-05, 4.63561e-08, 0, -0.099241141, 1],
        ],
    ],
    dtype=float,
)


def physical():
    return {
        "mass": 10617,
        "Ixx": 22682,
        "Iyy": 77095,
        "Izz": 95561,
        "Ixy": 0,
        "Ixz": 1125,
        "Iyz": 0,
        "Sref": 57.7,
        "b": 13.11,
        "Cbar": 4.4,
        "Xp": 7,
        "Xc": 9.68,
        "Xr": 7.91,
        "Xt": 8.5,
        "size": {
            "fuselage": {"length": 20, "radius": 3, "offset_x": 0},
            "mainwing": {
                "span": 13.11,
                "center_chord": 8.8,
                "offset_x": -3,
                "offset_z": 1.5,
            },
            "canard": {
                "span": 6,
                "center_chord": 4,
                "offset_x": 9.68,
                "offset_z": 0,
            },
            "fin": {
                "height": 3,
                "base_width": 4,
                "tip_width": 2,
                "offset_x": -7.91,
                "offset_z": -1.2,
            },
        },
    }


def fig22():
    return _FIG22.copy()


def stored_table(stem: str, data: dict | None = None) -> np.ndarray:
    """The Figure 2.2 table one stored log ran with, as `(10, 2, 8)`.

    A v0.1/v0.2 log carries a three-row-old table; replaying it means reading
    those rows back rather than forking the model. `data` is an already-loaded
    npz mapping, so a caller holding one does not pay for a second read.

    Reads `python/reference/<stem>.npz` directly, deliberately not through
    `reference.load_path`: a `_v031` archive is a rerun and carries no init-table
    keys, so the overlay would hide the table the stored log actually ran with.

    A log carrying neither prefix, and a log carrying only some of the 20 rows,
    are both data problems rather than an empty table, so both raise instead of
    silently falling back to `fig22()` - and they raise differently, because a
    truncated export is a different repair from a log that never stored one.
    """
    if data is None:
        with np.load(_REFERENCE_DIR / f"{stem}.npz") as archive:
            # A log holds 140+ arrays and tens of MB; only these 20 are read.
            return _table(stem, set(archive.files), archive.__getitem__)
    return _table(stem, set(data), data.__getitem__)


def _table(stem: str, present: set, read) -> np.ndarray:
    """`present` is the key set; `read` fetches one key by name."""
    truncated = {}
    for prefix in _TABLE_PREFIXES:
        rows = [
            f"{prefix}fig2_2_{index}_{ab}"
            for index in range(1, 11)
            for ab in ("a", "b")
        ]
        missing = [key for key in rows if key not in present]
        if not missing:
            return np.array(
                [np.asarray(read(key), dtype=float).reshape(8) for key in rows],
                dtype=float,
            ).reshape(10, 2, 8)
        if any(key.startswith(f"{prefix}fig2_2_") for key in present):
            truncated[prefix] = missing
    if truncated:
        raise KeyError(
            f"{stem} carries an incomplete Figure 2.2 table: "
            + "; ".join(
                f"{prefix} has {20 - len(missing)} of 20 rows, missing "
                f"{', '.join(missing)}"
                for prefix, missing in truncated.items()
            )
        )
    raise KeyError(
        f"{stem} carries no Figure 2.2 table: none of "
        f"{', '.join(repr(prefix) for prefix in _TABLE_PREFIXES)}fig2_2_<n>_<ab> present"
    )
