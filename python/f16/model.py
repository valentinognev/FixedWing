"""Morelli F-16 state derivative (imperial inside)."""
from __future__ import annotations

from math import ceil, cos, floor, pi, sin, sqrt

import numpy as np


def subf16_derivative(x13, u_deg4):
    x13 = np.asarray(x13, dtype=float)
    u_deg4 = np.asarray(u_deg4, dtype=float)
    if x13.shape != (13,) or u_deg4.shape != (4,):
        raise ValueError(f"expected x13 (13,) and u (4,), got {x13.shape} {u_deg4.shape}")
    xd, Nz, Ny, _, _ = _subf16_morelli(x13, u_deg4)
    return xd, float(Nz), float(Ny)


def _fix(ele):
    """Round towards zero."""
    assert isinstance(ele, float)
    if ele > 0:
        return int(floor(ele))
    return int(ceil(ele))


def _sign(ele):
    if ele < 0:
        return -1
    if ele == 0:
        return 0
    return 1


def _adc(vt, alt):
    ro = 2.377e-3
    tfac = 1 - .703e-5 * alt
    if alt >= 35000:
        t = 390
    else:
        t = 519 * tfac
    rho = ro * tfac**4.14
    a = sqrt(1.4 * 1716.3 * t)
    amach = vt / a
    qbar = .5 * rho * vt * vt
    return amach, qbar


def _tgear(thtl):
    if thtl <= .77:
        tg = 64.94 * thtl
    else:
        tg = 217.38 * thtl - 117.38
    return tg


def _rtau(dp):
    if dp <= 25:
        rt = 1.0
    elif dp >= 50:
        rt = .1
    else:
        rt = 1.9 - .036 * dp
    return rt


def _pdot(p3, p1):
    if p1 >= 50:
        if p3 >= 50:
            t = 5
            p2 = p1
        else:
            p2 = 60
            t = _rtau(p2 - p3)
    else:
        if p3 >= 50:
            t = 5
            p2 = 40
        else:
            p2 = p1
            t = _rtau(p2 - p3)
    return t * (p2 - p3)


def _thrust(power, alt, rmach):
    a = np.array([[1060, 670, 880, 1140, 1500, 1860],
                  [635, 425, 690, 1010, 1330, 1700],
                  [60, 25, 345, 755, 1130, 1525],
                  [-1020, -170, -300, 350, 910, 1360],
                  [-2700, -1900, -1300, -247, 600, 1100],
                  [-3600, -1400, -595, -342, -200, 700]], dtype=float).T
    b = np.array([[12680, 9150, 6200, 3950, 2450, 1400],
                  [12680, 9150, 6313, 4040, 2470, 1400],
                  [12610, 9312, 6610, 4290, 2600, 1560],
                  [12640, 9839, 7090, 4660, 2840, 1660],
                  [12390, 10176, 7750, 5320, 3250, 1930],
                  [11680, 9848, 8050, 6100, 3800, 2310]], dtype=float).T
    c = np.array([[20000, 15000, 10800, 7000, 4000, 2500],
                  [21420, 15700, 11225, 7323, 4435, 2600],
                  [22700, 16860, 12250, 8154, 5000, 2835],
                  [24240, 18910, 13760, 9285, 5700, 3215],
                  [26070, 21075, 15975, 11115, 6860, 3950],
                  [28886, 23319, 18300, 13484, 8642, 5057]], dtype=float).T
    if alt < 0:
        alt = 0.01
    h = .0001 * alt
    i = _fix(h)
    if i >= 5:
        i = 4
    dh = h - i
    rm = 5 * rmach
    m = _fix(rm)
    if m >= 5:
        m = 4
    elif m <= 0:
        m = 0
    dm = rm - m
    cdh = 1 - dh
    s = b[i, m] * cdh + b[i + 1, m] * dh
    t = b[i, m + 1] * cdh + b[i + 1, m + 1] * dh
    tmil = s + (t - s) * dm
    if power < 50:
        s = a[i, m] * cdh + a[i + 1, m] * dh
        t = a[i, m + 1] * cdh + a[i + 1, m + 1] * dh
        tidl = s + (t - s) * dm
        thrst = tidl + (tmil - tidl) * power * .02
    else:
        s = c[i, m] * cdh + c[i + 1, m] * dh
        t = c[i, m + 1] * cdh + c[i + 1, m + 1] * dh
        tmax = s + (t - s) * dm
        thrst = tmil + (tmax - tmil) * (power - 50) * .02
    return thrst


def _dampp(alpha):
    a = np.array([[-.267, -.110, .308, 1.34, 2.08, 2.91, 2.76, 2.05, 1.50, 1.49, 1.83, 1.21],
                  [.882, .852, .876, .958, .962, .974, .819, .483, .590, 1.21, -.493, -1.04],
                  [-.108, -.108, -.188, .110, .258, .226, .344, .362, .611, .529, .298, -2.27],
                  [-8.80, -25.8, -28.9, -31.4, -31.2, -30.7, -27.7, -28.2, -29.0, -29.8, -38.3, -35.3],
                  [-.126, -.026, .063, .113, .208, .230, .319, .437, .680, .100, .447, -.330],
                  [-.360, -.359, -.443, -.420, -.383, -.375, -.329, -.294, -.230, -.210, -.120, -.100],
                  [-7.21, -.540, -5.23, -5.26, -6.11, -6.64, -5.69, -6.00, -6.20, -6.40, -6.60, -6.00],
                  [-.380, -.363, -.378, -.386, -.370, -.453, -.550, -.582, -.595, -.637, -1.02, -.840],
                  [.061, .052, .052, -.012, -.013, -.024, .050, .150, .130, .158, .240, .150]], dtype=float).T
    s = .2 * alpha
    k = _fix(s)
    if k <= -2:
        k = -1
    if k >= 9:
        k = 8
    da = s - k
    l = k + _fix(1.1 * _sign(da))
    k = k + 3
    l = l + 3
    d = np.zeros((9,))
    for i in range(9):
        d[i] = a[k - 1, i] + abs(da) * (a[l - 1, i] - a[k - 1, i])
    return d


def _morellif16(alpha, beta, de, da, dr, p, q, r, cbar, b, V, xcg, xcgref):
    phat = p * b / (2 * V)
    qhat = q * cbar / (2 * V)
    rhat = r * b / (2 * V)

    a0 = -1.943367e-2
    a1 = 2.136104e-1
    a2 = -2.903457e-1
    a3 = -3.348641e-3
    a4 = -2.060504e-1
    a5 = 6.988016e-1
    a6 = -9.035381e-1

    b0 = 4.833383e-1
    b1 = 8.644627
    b2 = 1.131098e1
    b3 = -7.422961e1
    b4 = 6.075776e1

    c0 = -1.145916
    c1 = 6.016057e-2
    c2 = 1.642479e-1

    d0 = -1.006733e-1
    d1 = 8.679799e-1
    d2 = 4.260586
    d3 = -6.923267

    e0 = 8.071648e-1
    e1 = 1.189633e-1
    e2 = 4.177702
    e3 = -9.162236

    f0 = -1.378278e-1
    f1 = -4.211369
    f2 = 4.775187
    f3 = -1.026225e1
    f4 = 8.399763
    f5 = -4.354000e-1

    g0 = -3.054956e1
    g1 = -4.132305e1
    g2 = 3.292788e2
    g3 = -6.848038e2
    g4 = 4.080244e2

    h0 = -1.05853e-1
    h1 = -5.776677e-1
    h2 = -1.672435e-2
    h3 = 1.357256e-1
    h4 = 2.172952e-1
    h5 = 3.464156
    h6 = -2.835451
    h7 = -1.098104

    i0 = -4.126806e-1
    i1 = -1.189974e-1
    i2 = 1.247721
    i3 = -7.391132e-1

    j0 = 6.250437e-2
    j1 = 6.067723e-1
    j2 = -1.101964
    j3 = 9.100087
    j4 = -1.192672e1

    k0 = -1.463144e-1
    k1 = -4.07391e-2
    k2 = 3.253159e-2
    k3 = 4.851209e-1
    k4 = 2.978850e-1
    k5 = -3.746393e-1
    k6 = -3.213068e-1

    l0 = 2.635729e-2
    l1 = -2.192910e-2
    l2 = -3.152901e-3
    l3 = -5.817803e-2
    l4 = 4.516159e-1
    l5 = -4.928702e-1
    l6 = -1.579864e-2

    m0 = -2.029370e-2
    m1 = 4.660702e-2
    m2 = -6.012308e-1
    m3 = -8.062977e-2
    m4 = 8.320429e-2
    m5 = 5.018538e-1
    m6 = 6.378864e-1
    m7 = 4.226356e-1

    n0 = -5.19153
    n1 = -3.554716
    n2 = -3.598636e1
    n3 = 2.247355e2
    n4 = -4.120991e2
    n5 = 2.411750e2

    o0 = 2.993363e-1
    o1 = 6.594004e-2
    o2 = -2.003125e-1
    o3 = -6.233977e-2
    o4 = -2.107885
    o5 = 2.141420
    o6 = 8.476901e-1

    p0 = 2.677652e-2
    p1 = -3.298246e-1
    p2 = 1.926178e-1
    p3 = 4.013325
    p4 = -4.404302

    q0 = -3.698756e-1
    q1 = -1.167551e-1
    q2 = -7.641297e-1

    r0 = -3.348717e-2
    r1 = 4.276655e-2
    r2 = 6.573646e-3
    r3 = 3.535831e-1
    r4 = -1.373308
    r5 = 1.237582
    r6 = 2.302543e-1
    r7 = -2.512876e-1
    r8 = 1.588105e-1
    r9 = -5.199526e-1

    s0 = -8.115894e-2
    s1 = -1.156580e-2
    s2 = 2.514167e-2
    s3 = 2.038748e-1
    s4 = -3.337476e-1
    s5 = 1.004297e-1

    Cx0 = a0 + a1 * alpha + a2 * de**2 + a3 * de + a4 * alpha * de + a5 * alpha**2 + a6 * alpha**3
    Cxq = b0 + b1 * alpha + b2 * alpha**2 + b3 * alpha**3 + b4 * alpha**4
    Cy0 = c0 * beta + c1 * da + c2 * dr
    Cyp = d0 + d1 * alpha + d2 * alpha**2 + d3 * alpha**3
    Cyr = e0 + e1 * alpha + e2 * alpha**2 + e3 * alpha**3
    Cz0 = (f0 + f1 * alpha + f2 * alpha**2 + f3 * alpha**3 + f4 * alpha**4) * (1 - beta**2) + f5 * de
    Czq = g0 + g1 * alpha + g2 * alpha**2 + g3 * alpha**3 + g4 * alpha**4
    Cl0 = h0 * beta + h1 * alpha * beta + h2 * alpha**2 * beta + h3 * beta**2 + h4 * alpha * beta**2 + h5 * \
        alpha**3 * beta + h6 * alpha**4 * beta + h7 * alpha**2 * beta**2
    Clp = i0 + i1 * alpha + i2 * alpha**2 + i3 * alpha**3
    Clr = j0 + j1 * alpha + j2 * alpha**2 + j3 * alpha**3 + j4 * alpha**4
    Clda = k0 + k1 * alpha + k2 * beta + k3 * alpha**2 + k4 * alpha * beta + k5 * alpha**2 * beta + k6 * alpha**3
    Cldr = l0 + l1 * alpha + l2 * beta + l3 * alpha * beta + l4 * alpha**2 * beta + l5 * alpha**3 * beta + l6 * beta**2
    Cm0 = m0 + m1 * alpha + m2 * de + m3 * alpha * de + m4 * de**2 + m5 * alpha**2 * de + m6 * de**3 + m7 * \
        alpha * de**2
    Cmq = n0 + n1 * alpha + n2 * alpha**2 + n3 * alpha**3 + n4 * alpha**4 + n5 * alpha**5
    Cn0 = o0 * beta + o1 * alpha * beta + o2 * beta**2 + o3 * alpha * beta**2 + o4 * alpha**2 * beta + o5 * \
        alpha**2 * beta**2 + o6 * alpha**3 * beta
    Cnp = p0 + p1 * alpha + p2 * alpha**2 + p3 * alpha**3 + p4 * alpha**4
    Cnr = q0 + q1 * alpha + q2 * alpha**2
    Cnda = r0 + r1 * alpha + r2 * beta + r3 * alpha * beta + r4 * alpha**2 * beta + r5 * alpha**3 * beta + r6 * \
        alpha**2 + r7 * alpha**3 + r8 * beta**3 + r9 * alpha * beta**3
    Cndr = s0 + s1 * alpha + s2 * beta + s3 * alpha * beta + s4 * alpha**2 * beta + s5 * alpha**2

    Cx = Cx0 + Cxq * qhat
    Cy = Cy0 + Cyp * phat + Cyr * rhat
    Cz = Cz0 + Czq * qhat
    Cl = Cl0 + Clp * phat + Clr * rhat + Clda * da + Cldr * dr
    Cm = Cm0 + Cmq * qhat + Cz * (xcgref - xcg)
    Cn = Cn0 + Cnp * phat + Cnr * rhat + Cnda * da + Cndr * dr - Cy * (xcgref - xcg) * (cbar / b)
    return Cx, Cy, Cz, Cl, Cm, Cn


def _subf16_morelli(x, u):
    """Line-for-line Morelli path of aerobench subf16_model (adjust_cy=True)."""
    xcg = 0.35

    thtlc, el, ail, rdr = u

    s = 300
    b = 30
    cbar = 11.32
    rm = 1.57e-3
    xcgr = .35
    he = 160.0
    c1 = -.770
    c2 = .02755
    c3 = 1.055e-4
    c4 = 1.642e-6
    c5 = .9604
    c6 = 1.759e-2
    c7 = 1.792e-5
    c8 = -.7336
    c9 = 1.587e-5
    rtod = 57.29578
    g = 32.17

    xd = x.copy()
    vt = x[0]
    alpha = x[1] * rtod
    beta = x[2] * rtod
    phi = x[3]
    theta = x[4]
    psi = x[5]
    p = x[6]
    q = x[7]
    r = x[8]
    alt = x[11]
    power = x[12]

    amach, qbar = _adc(vt, alt)
    cpow = _tgear(thtlc)
    xd[12] = _pdot(power, cpow)
    t = _thrust(power, alt, amach)
    dail = ail / 20
    drdr = rdr / 30

    cxt, cyt, czt, clt, cmt, cnt = _morellif16(
        alpha * pi / 180, beta * pi / 180, el * pi / 180, ail * pi / 180, rdr * pi / 180,
        p, q, r, cbar, b, vt, xcg, xcgr)

    tvt = .5 / vt
    b2v = b * tvt
    cq = cbar * q * tvt

    d = _dampp(alpha)
    cxt = cxt + cq * d[0]
    cyt = cyt + b2v * (d[1] * r + d[2] * p)
    czt = czt + cq * d[3]
    clt = clt + b2v * (d[4] * r + d[5] * p)
    cmt = cmt + cq * d[6] + czt * (xcgr - xcg)
    cnt = cnt + b2v * (d[7] * r + d[8] * p) - cyt * (xcgr - xcg) * cbar / b
    cbta = cos(x[2])
    u = vt * cos(x[1]) * cbta
    v = vt * sin(x[2])
    w = vt * sin(x[1]) * cbta
    sth = sin(theta)
    cth = cos(theta)
    sph = sin(phi)
    cph = cos(phi)
    spsi = sin(psi)
    cpsi = cos(psi)
    qs = qbar * s
    qsb = qs * b
    rmqs = rm * qs
    gcth = g * cth
    qsph = q * sph
    ay = rmqs * cyt
    az = rmqs * czt

    udot = r * v - q * w - g * sth + rm * (qs * cxt + t)
    vdot = p * w - r * u + gcth * sph + ay
    wdot = q * u - p * v + gcth * cph + az
    dum = (u * u + w * w)

    xd[0] = (u * udot + v * vdot + w * wdot) / vt
    xd[1] = (u * wdot - w * udot) / dum
    xd[2] = (vt * vdot - v * xd[0]) * cbta / dum

    xd[3] = p + (sth / cth) * (qsph + r * cph)
    xd[4] = q * cph - r * sph
    xd[5] = (qsph + r * cph) / cth

    xd[6] = (c2 * p + c1 * r + c4 * he) * q + qsb * (c3 * clt + c4 * cnt)
    xd[7] = (c5 * p - c7 * he) * r + c6 * (r * r - p * p) + qs * cbar * c7 * cmt
    xd[8] = (c8 * p - c2 * r + c9 * he) * q + qsb * (c4 * clt + c9 * cnt)

    t1 = sph * cpsi
    t2 = cph * sth
    t3 = sph * spsi
    s1 = cth * cpsi
    s2 = cth * spsi
    s3 = t1 * sth - cph * spsi
    s4 = t3 * sth + cph * cpsi
    s5 = sph * cth
    s6 = t2 * cpsi + t3
    s7 = t2 * spsi - t1
    s8 = cph * cth
    xd[9] = u * s1 + v * s3 + w * s6
    xd[10] = u * s2 + v * s4 + w * s7
    xd[11] = u * sth - v * s5 - w * s8

    xa = 15.0
    az = az - xa * xd[7]
    ay = ay + xa * xd[8]

    Nz = (-az / g) - 1
    Ny = ay / g
    return xd, Nz, Ny, az, ay
