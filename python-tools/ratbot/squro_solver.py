"""Kinematics for the biomimetic (SQuRo) leg.

Geometry measured from the Fusion model ``Q8bot_rev2_5_SQuRo_Leg`` by reading
the pivot-hole axes of SQ3_ShoulderCrank_P, SQ3_ElbowCrank_P, SQ3_Humerus_P,
SQ3_HipPushrod_P, SQ3_KneePushrod_P and SQ3_UlnaFoot_P.

Model frame, matching kinematics_solver.k_solver: x to the right, y positive
*downwards* (away from the body), origin at the outer motor axis, so the two
motor axes sit at (0, 0) and (d, 0).

    A  = (0, 0)      shoulder-crank axis      (outer motor)
    B  = (d, 0)      elbow-crank axis, and the humerus pivot (inner motor)
    P1 = shoulder crank pin      P2 = hip pushrod -> humerus
    Q1 = elbow crank pin         Q2 = knee pushrod -> ulna
    E  = humerus -> ulna pivot (the elbow)
    F  = foot centre

Unlike the Q8 five-bar, this linkage is a *double parallelogram*:

    |A-P1| = |B-P2| = 15.920      |P1-P2| = |A-B| = 19.500
    |B-Q1| = |E-Q2| = 15.390      |Q1-Q2| = |B-E| = 34.870

and B, P2, E are collinear (measured 129.019 deg vs 129.021 deg, a 0.002 deg
modelling residual). Both relations hold to four decimals, which means the
mechanism is kinematically a plain 2R serial arm with both actuators left on
the body:

    the humerus angle about B equals the shoulder-crank angle, and
    the ulna angle about E equals the elbow-crank angle.

So the foot is reached by

    E = B + L1 * u(theta1)
    F = E + L2 * u(theta2 + FOOT_OFFSET)

with u(a) the unit vector at angle a. That gives a closed-form IK, where the
five-bar needed a numerical solve.
"""

import math

import numpy as np

# Pivot coordinates as measured, in the model frame (mm).
MEASURED_PIVOTS = {
    'A': (0.000, 0.000), 'B': (19.500, 0.000),
    'P1': (-10.023, 12.369), 'P2': (9.477, 12.369),
    'Q1': (28.835, 12.236), 'Q2': (6.881, 39.327),
    'E': (-2.454, 27.091), 'F': (33.935, 45.113),
}


class squro_solver:
    """Biomimetic leg kinematics, interface-compatible with ``k_solver``.

    ``ik_solve``/``fk_solve`` use the same joint ordering as the five-bar
    solver: ``q1`` drives the motor at (d, 0) and ``q2`` the motor at (0, 0),
    which is the order ``q8_espnow.move_all`` puts on the wire.
    """

    # Humerus B->E and ulna E->F. The cranks and pushrods only transmit these.
    L1 = 34.870
    L2 = 40.607
    # Angle from the elbow-crank direction to the ulna's line to the foot.
    FOOT_OFFSET = -26.312
    # Link lengths kept for the assembly drawing and the parts list.
    SHOULDER_CRANK = 15.920
    ELBOW_CRANK = 15.390
    HIP_PUSHROD = 19.500
    KNEE_PUSHROD = 34.870
    # The pose the CAD is modelled in; the leg-attachment reference.
    MOUNT_POSE = (52.660, 129.021)

    def __init__(self, d=19.5, l1=L1, l2=L2, foot_offset=FOOT_OFFSET):
        self.d = d
        self.l1 = l1
        self.l2 = l2
        self.foot_offset = foot_offset
        self.prev_ik = list(self.MOUNT_POSE)

    def ik_check(self, x, y):
        """True when the foot lies inside the annulus the 2R arm can reach."""
        reach = math.hypot(x - self.d, y)
        return abs(self.l1 - self.l2) <= reach <= self.l1 + self.l2 and reach > 0

    def ik_solve(self, x, y, deg=True, rounding=3):
        """Closed-form 2R inverse kinematics. Returns (q1, q2, success)."""
        vx, vy = x - self.d, y
        reach = math.hypot(vx, vy)
        if not self.ik_check(x, y):
            return self.prev_ik[0], self.prev_ik[1], False
        cos_beta = (reach**2 + self.l1**2 - self.l2**2) / (2 * reach * self.l1)
        # Guard the arccos against reach values sitting exactly on a limit.
        beta = math.acos(max(-1.0, min(1.0, cos_beta)))
        # Positive branch: the elbow folds the way the CAD build does. The
        # negative branch is the mirror assembly and would fight the hard stop.
        theta1 = math.atan2(vy, vx) + beta
        ex = self.d + self.l1 * math.cos(theta1)
        ey = self.l1 * math.sin(theta1)
        theta2 = math.atan2(y - ey, x - ex) - math.radians(self.foot_offset)
        q1, q2 = math.degrees(theta2), math.degrees(theta1)
        if not deg:
            q1, q2 = math.radians(q1), math.radians(q2)
        self.prev_ik = [q1, q2]
        return np.round(q1, rounding), np.round(q2, rounding), True

    def fk_check(self):
        return True

    def fk_solve(self, q1, q2, deg=True, rounding=3):
        """Foot position for the commanded joint pair. Returns (x, y)."""
        theta2, theta1 = (q1, q2) if deg else (math.degrees(q1), math.degrees(q2))
        x, y = self.pivots(theta2, theta1)['F']
        return round(x, rounding), round(y, rounding)

    def pivots(self, q1, q2):
        """Every linkage pivot at the given joint pair, in degrees.

        Used by the assembly drawing, so the picture and the commanded pose
        cannot drift apart.
        """
        theta2, theta1 = math.radians(q1), math.radians(q2)
        foot_dir = theta2 + math.radians(self.foot_offset)
        a = (0.0, 0.0)
        b = (self.d, 0.0)
        e = (b[0] + self.l1 * math.cos(theta1), b[1] + self.l1 * math.sin(theta1))
        p1 = (a[0] + self.SHOULDER_CRANK * math.cos(theta1),
              a[1] + self.SHOULDER_CRANK * math.sin(theta1))
        p2 = (p1[0] + self.HIP_PUSHROD, p1[1])
        q1p = (b[0] + self.ELBOW_CRANK * math.cos(theta2),
               b[1] + self.ELBOW_CRANK * math.sin(theta2))
        q2p = (e[0] + self.ELBOW_CRANK * math.cos(theta2),
               e[1] + self.ELBOW_CRANK * math.sin(theta2))
        f = (e[0] + self.l2 * math.cos(foot_dir), e[1] + self.l2 * math.sin(foot_dir))
        return {'A': a, 'B': b, 'P1': p1, 'P2': p2, 'Q1': q1p, 'Q2': q2p, 'E': e, 'F': f}
