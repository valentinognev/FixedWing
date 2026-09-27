"""F-16 LQR inner loop (frozen paper gains)."""
from __future__ import annotations

import numpy as np


class CtrlLimits:
    def __init__(self) -> None:
        self.ThrottleMax = 1
        self.ThrottleMin = 0
        self.ElevatorMaxDeg = 25
        self.ElevatorMinDeg = -25
        self.AileronMaxDeg = 21.5
        self.AileronMinDeg = -21.5
        self.RudderMaxDeg = 30
        self.RudderMinDeg = -30
        self.NzMax = 6
        self.NzMin = -1


class F16Llc:
    old_k_long = np.array([[-156.8801506723475, -31.037008068526642, -38.72983346216317]], dtype=float)
    old_k_lat = np.array([[37.84483, -25.40956, -6.82876, -332.88343, -17.15997],
                          [-23.91233, 5.69968, -21.63431, 64.49490, -88.36203]], dtype=float)
    old_xequil = np.array([502.0, 0.0389, 0.0, 0.0, 0.0389, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 1000.0, 9.0567], dtype=float)
    old_uequil = np.array([0.1395, -0.7496, 0.0, 0.0], dtype=float)

    def __init__(self) -> None:
        self.K_lqr = np.zeros((3, 8))
        self.K_lqr[:1, :3] = F16Llc.old_k_long
        self.K_lqr[1:, 3:] = F16Llc.old_k_lat
        self.xequil = F16Llc.old_xequil.copy()
        self.uequil = F16Llc.old_uequil.copy()
        self.ctrlLimits = CtrlLimits()
        self.model_str = "morelli"

    def get_u_deg(self, u_ref4, x_f16):
        u_ref4 = np.asarray(u_ref4, dtype=float)
        x_f16 = np.asarray(x_f16, dtype=float)
        x_delta = x_f16.copy()
        x_delta[: len(self.xequil)] -= self.xequil
        x_ctrl = np.array([x_delta[i] for i in [1, 7, 13, 2, 6, 8, 14, 15]], dtype=float)
        u_deg = np.zeros((4,))
        u_deg[1:4] = np.dot(-self.K_lqr, x_ctrl)
        u_deg[0] = u_ref4[3]
        u_deg[0:4] += self.uequil
        lim = self.ctrlLimits
        u_deg[0] = max(min(u_deg[0], lim.ThrottleMax), lim.ThrottleMin)
        u_deg[1] = max(min(u_deg[1], lim.ElevatorMaxDeg), lim.ElevatorMinDeg)
        u_deg[2] = max(min(u_deg[2], lim.AileronMaxDeg), lim.AileronMinDeg)
        u_deg[3] = max(min(u_deg[3], lim.RudderMaxDeg), lim.RudderMinDeg)
        return x_ctrl, u_deg

    def get_num_integrators(self) -> int:
        return 3

    def get_integrator_derivatives(self, t, x_f16, u_ref4, Nz, ps, Ny_r):
        return [Nz - u_ref4[0], ps - u_ref4[1], Ny_r - u_ref4[2]]
