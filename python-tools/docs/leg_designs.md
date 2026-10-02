# Leg designs and attachment calibration

The robot can be built with different leg linkages. **L** in the Pygame window
opens the leg picker; choosing a design starts the attachment calibration for
it. Up/Down move the cursor, Enter advances, Esc cancels at any point without
changing the fitted design. Left/Right keep turning the head throughout.

| Design | Linkage | Source model | State |
|---|---|---|---|
| Q8 | Five-bar parallel | `Q8bot_rev2_5_FDM` | Available |
| Full range | MiNIQ | `Q8bot_MiNIQ_Leg` | Declared, geometry not measured |
| Biomimetic | SQuRo crank and pushrod | `Q8bot_rev2_5_SQuRo_Leg` | Available |

## What calibration does

Calibration drives every joint to the selected design's **mounting pose** and
holds it there with torque on while the linkages are fitted. That pose is what
fixes the keying between each servo horn and its crank, so every later
commanded angle means the same thing. It is the host-side equivalent of the
one-shot move at the end of `firmware/ratbot_motor_config`, so a leg swap no
longer needs the setup firmware reflashed.

| Design | Mounting pose (q1 / q2) | Ticks |
|---|---|---|
| Q8 | 45.879° / 134.121° | 4618 / 5622 |
| Biomimetic | 52.660° / 129.021° | 4695 / 5564 |

The Q8 values are the ones the motor-config firmware already uses; a test pins
them to 4618/5622 so the two cannot drift apart.

Joint order is the one on the wire: `q1` is the motor at (d, 0), `q2` the motor
at (0, 0), repeated for FL, FR, BL, BR.

Support the robot before starting. The feet leave the ground, and the sequence
holds torque until you press Enter or Esc.

## Biomimetic (SQuRo) kinematics

Measured from the pivot-hole axes in `Q8bot_rev2_5_SQuRo_Leg`. The linkage is a
**double parallelogram**:

```
|A-P1| = |B-P2| = 15.920     |P1-P2| = |A-B| = 19.500
|B-Q1| = |E-Q2| = 15.390     |Q1-Q2| = |B-E| = 34.870
```

and B, P2, E are collinear (129.019° vs 129.021°, a 0.002° modelling residual).
Both relations hold to four decimals, which means the mechanism is
kinematically a plain **2R serial arm** with both actuators left on the body:

- the humerus angle about B equals the shoulder-crank angle, and
- the ulna angle about E equals the elbow-crank angle.

So the foot follows

```
E = B + L1 * u(theta1)
F = E + L2 * u(theta2 + FOOT_OFFSET)
```

with `L1 = 34.870 mm`, `L2 = 40.607 mm`, `FOOT_OFFSET = -26.312°` and motors at
`A = (0, 0)`, `B = (19.5, 0)`. That gives a closed-form IK where the five-bar
needs a numerical solve. Reach is an annulus of 5.737–75.477 mm about B.

`squro_solver.pivots()` returns every pivot, and the calibration diagram is
drawn from it, so the picture and the commanded pose cannot drift apart. The
test suite reproduces all eight measured pivots to under 5 µm and re-checks
both parallelogram loops across the workspace.

This models the **front** leg. Front and hind differ by more than the foot
part; see *Front and hind legs are not the same leg* below.

## Head servo

The head servo (ID 19) is not part of any leg design. It sits on the front
centreline in the CAD, at (−78.0, 7.1, 0.0), about 35 mm from either front
outer leg servo, and is daisy chained from ID 11 or ID 13. The motor-config
firmware sets it up, either during the first setup or on its own with
`config_head`; see the
[motor-config README](../../firmware/ratbot_motor_config/README.md).
The Left and Right arrow keys turn it the same way whichever legs are fitted.

## Per-design gaits and pictures

Fitting a design swaps four things together: the solver, the gait table, the
two static poses and the ten illustrations. Pictures live in
`docs/poses/<design>/` and are regenerated for every available design with
`venv/bin/python ratbot/pose_preview.py`. Limb shape in both the illustrations
and the calibration diagram comes from `leg_geometry`, the same description the
solvers command, so a picture cannot show a linkage the robot will not make.

A gait row is `[stacktype, x0, y0, xrange, yrange, yrange2, s1, s2]`. Stride,
lift and step counts describe locomotion, so they carry across designs
unchanged. The stance centre does not: it is where that particular linkage
naturally hangs. The biomimetic table is the Q8 table with

- `x0` set to the design's natural stance x (33.935 mm, from the CAD pose), and
- `y0` scaled by `45.113 / 43.36`, the ratio of the two natural stance heights,

then stride and lift trimmed 5% at a time only where the joints would otherwise
swing more than 40° from the mounting pose. Three gaits needed one 5% trim.

The re-centring is the whole point. Driving the biomimetic leg with the Q8
stance centre reaches **219° on q2, about 90° past its as-designed pose**.
Re-centred, the worst swing is 39.7°, which is better centred than the five-bar
is on its own gaits (74.2° at TROT_LOW). Tests assert both: no design exceeds
the 75° five-bar ceiling, and the biomimetic table holds its tighter 40° band.

`_generate_base_trajectories` previously validated inverse kinematics by
measuring the *string length* of the rounded angle and ignored the solver's own
reachability flag, so an unreachable target silently produced the previous
pose. It now honours the flag. Neither leg currently trips it, so nothing
changed today, but a new design would have hit it.

## Front and hind legs are not the same leg

Read from the CAD without activating the document: the front leg's parts all
share one occurrence transform, so the measured pose is its true assembled
pose. The hind leg's parts each carry a different transform, and:

| | Shoulder-crank axis A | Elbow/humerus axis B | Foot part |
|---|---|---|---|
| Front | world x −30.5 (inner) | −50.0 (outer) | `SQ3_UlnaFoot`, round pad r ≈ 19.1 mm |
| Hind | +50.0 (outer) | +30.5 (inner) | `SQ4_UlnaFootHind`, pointed toe |

Two consequences:

1. **A and B swap inner/outer between front and hind.** A commanded `(q1, q2)`
   pair therefore means different things on a front and a hind leg, depending
   on how the servo IDs map to the inner and outer positions. That cannot be
   settled from CAD; confirm it on the bench at the first calibration.
2. **The CAD stance is splayed.** The front foot sits 24.2 mm forward of its
   mount midpoint and the hind foot 20.1 mm rearward — opposite world
   directions. The gait pipeline applies one stance offset to all four legs, so
   a single `x0` cannot express that splay.

The humerus is the same length on both (34.870 mm), and the effective distance
from elbow to ground contact very nearly matches: 59.71 mm at the front (pad
centre 40.607 plus the 19.1 mm pad radius) against 59.42 mm to the hind toe.
So the two legs are close in reach and differ mainly in contact shape and pose.

Note that `y0` for the biomimetic leg is the **pad centre**, not the ground
contact; the robot stands about 19 mm higher than the number suggests.

## Known limits

- **Biomimetic gaits are geometry-checked, not bench-tuned.** Every trajectory
  is reachable and stays inside the joint band, but no stride length or stance
  height has been tried on the robot. The GUI warns whenever a non-default
  design is fitted.
- **All four legs share one model.** The solver, gait table and pictures
  describe the *front* leg. The hind leg's different foot and swapped servo
  positions are documented above but not modelled; reproducing the splayed CAD
  stance needs per-leg stance offsets, which the gait table format and all five
  trajectory generators would have to carry.
- **Rearing and the greeting** are stored as joint angles, so each design gets
  them remapped: Q8 joints to foot xy through the five-bar forward kinematics,
  then back through the new design's inverse kinematics. That preserves the
  foot path, but the support stance reaches 53° from the biomimetic mounting
  pose, outside the 40° band the gaits hold to, and no balance check is
  implemented for any design.
- **Which servo of each pair drives the shoulder crank** is taken from the CAD
  frame and should be confirmed on the bench during the first calibration. If
  the two are swapped, the crank points the wrong way and it is visible
  immediately at the mounting pose.
- **Full range (MiNIQ)** appears in the picker but cannot be selected until its
  pivot geometry is read out of the MiNIQ Fusion model the same way.
