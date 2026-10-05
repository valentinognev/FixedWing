"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

import numpy as np


def conj(q):
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def multiply(p, q):
    p = np.asarray(p, dtype=float)
    q = np.asarray(q, dtype=float)
    p0, p1, p2, p3 = p
    q0, q1, q2, q3 = q
    return np.array(
        [
            p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
            p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
            p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
            p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0,
        ]
    )


def _pure(v):
    v = np.asarray(v, dtype=float)
    return np.array([0.0, v[0], v[1], v[2]])


def rotate_body_to_earth(q, v):
    q = np.asarray(q, dtype=float)
    return multiply(multiply(q, _pure(v)), conj(q))[1:]


def rotate_earth_to_body(q, v):
    q = np.asarray(q, dtype=float)
    return multiply(multiply(conj(q), _pure(v)), q)[1:]


def body_321_to_q(roll, pitch, yaw):
    t1, t2, t3 = roll, pitch, yaw
    q0 = np.sin(t1 / 2) * np.sin(t2 / 2) * np.sin(t3 / 2) + np.cos(t1 / 2) * np.cos(
        t2 / 2
    ) * np.cos(t3 / 2)
    q1 = -np.cos(t1 / 2) * np.sin(t2 / 2) * np.sin(t3 / 2) + np.sin(t1 / 2) * np.cos(
        t2 / 2
    ) * np.cos(t3 / 2)
    q2 = np.sin(t1 / 2) * np.cos(t2 / 2) * np.sin(t3 / 2) + np.cos(t1 / 2) * np.sin(
        t2 / 2
    ) * np.cos(t3 / 2)
    q3 = -np.sin(t1 / 2) * np.sin(t2 / 2) * np.cos(t3 / 2) + np.cos(t1 / 2) * np.cos(
        t2 / 2
    ) * np.sin(t3 / 2)
    return np.array([q0, q1, q2, q3])


def body_231_to_q(alpha, beta, mu):
    sa = np.sin(alpha / 2)
    ca = np.cos(alpha / 2)
    sb = np.sin(beta / 2)
    cb = np.cos(beta / 2)
    sm = np.sin(mu / 2)
    cm = np.cos(mu / 2)
    q0 = ca * cb * cm - sa * sb * sm
    q1 = sa * sb * cm + ca * cb * sm
    q2 = sa * cb * cm + ca * sb * sm
    q3 = ca * sb * cm - sa * cb * sm
    return np.array([q0, q1, q2, q3])


def q_to_body_321(q):
    q = np.asarray(q, dtype=float)
    q0, q1, q2, q3 = q
    roll = np.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))
    pitch = np.asin(2 * (q0 * q2 - q3 * q1))
    yaw = np.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
    return (roll, pitch, yaw)
