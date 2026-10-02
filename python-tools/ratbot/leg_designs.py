"""The leg designs the robot can be built with, and their mounting references.

Each entry carries the solver for that linkage and the joint pair the servos
are driven to while the linkages are attached. The mounting pose is what makes
a design swappable: it defines the keying between a servo horn and its crank,
so every later commanded angle means the same thing.

Joint ordering matches ``q8_espnow.move_all``: ``q1`` is the motor at (d, 0)
and ``q2`` the motor at (0, 0), repeated for FL, FR, BL, BR.
"""

from gait_manager import GAITS as Q8_GAITS
from kinematics_solver import k_solver
from routine_generator import G1, G2, G3
from squro_solver import squro_solver

# Dynamixel ticks per degree; the robot firmware uses 0 deg = 4096 ticks.
TICKS_PER_DEG = 4096.0 / 360.0
TICK_ZERO = 4096


def deg_to_ticks(deg):
    return int(deg * TICKS_PER_DEG + 0.5) + TICK_ZERO


# --- motion profiles ------------------------------------------------------
# A gait table is [stacktype, x0, y0, xrange, yrange, yrange2, s1, s2], where
# x0/y0 are the stance centre in foot space and xrange/yrange the stride and
# lift. Stride, lift and step counts describe locomotion, so they carry across
# designs unchanged; the stance centre does not, because it is where that
# particular linkage naturally hangs.
#
# The biomimetic table is the Q8 table with:
#   x0 -> the design's natural stance x (33.935, from the CAD pose), and
#   y0 -> y0 * (45.113 / 43.36), the ratio of the two natural stance heights,
# then stride and lift trimmed 5% at a time only where the joints would
# otherwise swing more than 40 deg from the mounting pose. Driving this leg
# with the Q8 stance centre instead reaches 219 deg on q2, about 90 deg past
# its as-designed pose, which is why the re-centring is needed at all.
BIOMIMETIC_GAITS = {
    'TROT':      ['trot', 33.935, 45.11, 40.0, 20.0, 0.0, 15, 30],
    'TROT_HIGH': ['trot', 33.935, 62.43, 20.0, 10.0, 0.0, 15, 30],
    'TROT_LOW':  ['trot', 33.935, 26.01, 19.0, 9.5, 0.0, 15, 30],
    'TROT_FAST': ['trot', 33.935, 45.11, 50.0, 20.0, 0.0, 12, 24],
    'WALK':      ['walk', 33.935, 45.11, 30.0, 20.0, 0.0, 20, 140],
    'CRAWL':     ['crawl', 33.935, 41.62, 10.0, 20.0, 0.0, 50, 25],
    'BOUND':     ['bound', 33.935, 34.71, 38.0, 0.0, 19.0, 50, 10],
    'PRONK':     ['pronk', 33.935, 34.71, 38.0, 0.0, 19.0, 60, 10],
}

# Greeting/rearing waypoints are stored as joint angles, not foot positions, so
# they are remapped per design: Q8 joints -> foot xy through the five-bar FK ->
# the new design's joints through its IK. That preserves the foot path, which
# is the part that means anything. Computed once from G1/G2/G3.
BIOMIMETIC_G1 = [92.0, 138.75] * 4
BIOMIMETIC_G2 = [34.24, 104.06] * 4
BIOMIMETIC_G3 = [-0.23, 86.98] * 2 + [73.25, 124.83] * 2


class LegDesign:
    """One buildable leg design."""

    def __init__(self, key, title, subtitle, solver_factory, mount_pose,
                 available=True, source=None, steps=(), notes=(),
                 natural_stance=None, gaits=None, sitting=None, rearing=None):
        self.key = key
        self.title = title
        self.subtitle = subtitle
        self._solver_factory = solver_factory
        self.mount_pose = mount_pose
        self.available = available
        self.source = source
        self.steps = tuple(steps)
        self.notes = tuple(notes)
        self.natural_stance = natural_stance
        self.gaits = gaits
        # Foot targets for the static sitting pose: (foot_x, front, rear).
        self.sitting = sitting
        # (G1, G2, G3, rearward_offset) for the greeting and rearing poses.
        self.rearing = rearing

    @property
    def asset_dir(self):
        """Folder holding this design's gait and pose pictures."""
        return self.key.lower()

    def solver(self):
        if not self.available or self._solver_factory is None:
            raise ValueError(f'{self.title} has no kinematic model yet')
        return self._solver_factory()

    def mount_positions(self):
        """The mounting pose expanded to all four legs, for move_all."""
        return [float(self.mount_pose[0]), float(self.mount_pose[1])] * 4

    def mount_ticks(self):
        return tuple(deg_to_ticks(a) for a in self.mount_pose)


# The original five-bar leg. Its mounting pose is the one the motor-config
# firmware already drives to at the end of setup (4618 / 5622 ticks).
Q8 = LegDesign(
    key='Q8',
    title='Q8',
    subtitle='Original five-bar parallel leg.',
    solver_factory=k_solver,
    mount_pose=(45.879, 134.121),
    source='Q8bot_rev2_5_FDM',
    natural_stance=(9.75, 43.36),
    gaits=Q8_GAITS,
    sitting=(9.75, 60.0, 20.0),
    rearing=(list(G1), list(G2), list(G3), 13.0),
    steps=(
        'Detach all four leg linkages before starting.',
        'Both joints of every leg are driven to the mounting pose and held.',
        'Fit each linkage so the upper links point along the held crank angle.',
        'Check that all four legs are mirrored the same way before releasing.',
    ),
)

# The MiNIQ leg. Declared so it appears in the picker, but it has no measured
# geometry yet, so it cannot be selected.
FULL_RANGE = LegDesign(
    key='FULL_RANGE',
    title='Full range',
    subtitle='MiNIQ leg. Geometry not measured yet.',
    solver_factory=None,
    mount_pose=None,
    available=False,
    source='Q8bot_MiNIQ_Leg',
    notes=('Pivot geometry still has to be read out of the MiNIQ Fusion model.',),
)

# The biomimetic SQuRo leg, measured from Q8bot_rev2_5_SQuRo_Leg.
BIOMIMETIC = LegDesign(
    key='BIOMIMETIC',
    title='Biomimetic',
    subtitle='SQuRo rat leg. Crank and pushrod, 2R equivalent.',
    solver_factory=squro_solver,
    mount_pose=squro_solver.MOUNT_POSE,
    source='Q8bot_rev2_5_SQuRo_Leg',
    natural_stance=(33.935, 45.113),
    gaits=BIOMIMETIC_GAITS,
    sitting=(33.935, 62.43, 20.81),
    rearing=(BIOMIMETIC_G1, BIOMIMETIC_G2, BIOMIMETIC_G3, 13.0),
    steps=(
        'Detach all four leg linkages. Support the body; the feet leave the ground.',
        'Both cranks of every leg are driven to the mounting pose and held.',
        'Fit each shoulder crank pointing along the humerus line, then the elbow crank.',
        'Fit the hip and knee pushrods last; they set the two parallelograms.',
    ),
    notes=(
        'Front and hind legs use different feet: SQ3_UlnaFoot and SQ4_UlnaFootHind.',
        'Verify on the bench which servo of each pair drives the shoulder crank.',
        'Gait stride and height are geometry-checked only; tune them on the bench.',
    ),
)

LEG_DESIGNS = {design.key: design for design in (Q8, FULL_RANGE, BIOMIMETIC)}
DEFAULT_DESIGN = 'Q8'


def design_for(key):
    return LEG_DESIGNS[key]


def selectable():
    return [design for design in LEG_DESIGNS.values() if design.available]
