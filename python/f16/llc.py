"""F-16 LQR inner loop (paper gains transformed to SI)."""
from __future__ import annotations

import numpy as np

from f16.units import DEG_TO_RAD, deg_to_rad, state_imp_to_si, u_imp_to_si

_K_LONG_DEG = np.array(
    [[-156.8801506723475, -31.037008068526642, -38.72983346216317]],
    dtype=float,
)
_K_LAT_DEG = np.array(
    [[37.84483, -25.40956, -6.82876, -332.88343, -17.15997],
     [-23.91233, 5.69968, -21.63431, 64.49490, -88.36203]],
    dtype=float,
)
_X_IMP = np.array(
    [502.0, 0.0389, 0.0, 0.0, 0.0389, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 9.0567],
    dtype=float,
)
_U_IMP = np.array([0.1395, -0.7496, 0.0, 0.0], dtype=float)


class CtrlLimits:
    def __init__(self) -> None:
        self.ThrottleMax = 1
        self.ThrottleMin = 0
        self.ElevatorMaxRad = deg_to_rad(25)
        self.ElevatorMinRad = deg_to_rad(-25)
        self.AileronMaxRad = deg_to_rad(21.5)
        self.AileronMinRad = deg_to_rad(-21.5)
        self.RudderMaxRad = deg_to_rad(30)
        self.RudderMinRad = deg_to_rad(-30)
        self.NzMax = 6
        self.NzMin = -1


class F16Llc:
    def __init__(self) -> None:
        self.K_lqr = np.zeros((3, 8))
        self.K_lqr[:1, :3] = _K_LONG_DEG * DEG_TO_RAD
        self.K_lqr[1:, 3:] = _K_LAT_DEG * DEG_TO_RAD
        self.xequil = state_imp_to_si(_X_IMP)
        self.uequil = u_imp_to_si(_U_IMP)
        self.ctrlLimits = CtrlLimits()
        self.model_str = "morelli"

    def get_u(self, u_ref4, x_f16):
        u_ref4 = np.asarray(u_ref4, dtype=float)
        x_f16 = np.asarray(x_f16, dtype=float)
        x_delta = x_f16.copy()
        x_delta[: len(self.xequil)] -= self.xequil
        x_ctrl = np.array([x_delta[i] for i in [1, 7, 13, 2, 6, 8, 14, 15]], dtype=float)
        u = np.zeros((4,))
        u[1:4] = np.dot(-self.K_lqr, x_ctrl)
        u[0] = u_ref4[3]
        u[0:4] += self.uequil
        lim = self.ctrlLimits
        u[0] = max(min(u[0], lim.ThrottleMax), lim.ThrottleMin)
        u[1] = max(min(u[1], lim.ElevatorMaxRad), lim.ElevatorMinRad)
        u[2] = max(min(u[2], lim.AileronMaxRad), lim.AileronMinRad)
        u[3] = max(min(u[3], lim.RudderMaxRad), lim.RudderMinRad)
        return x_ctrl, u

    def get_num_integrators(self) -> int:
        return 3

    def get_integrator_derivatives(self, t, x_f16, u_ref4, Nz, ps, Ny_r):
        return [Nz - u_ref4[0], ps - u_ref4[1], Ny_r - u_ref4[2]]
