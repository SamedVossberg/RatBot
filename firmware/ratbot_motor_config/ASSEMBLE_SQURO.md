# RatBot assembly: servos, head and biomimetic legs

The `assemble_squro` firmware and the guided console `tools/assembly_console.py`
take a fresh build to four fitted SQuRo legs: a new PCB, new or reused servos,
and the body as in the RatBot STL files, **batteries facing down**. The guide
explains every step on screen and waits for you; the firmware makes every
safety check and never moves anything on its own.

## Quick start

1. Assemble frame, PCB and batteries with the batteries facing down. Put the
   head servo in the neck clamp. **No cranks or leg parts on any servo yet.**
2. Battery off, robot USB connected, then upload:
   `pio run -e assemble_squro -t upload --upload-port <robot USB port>`
3. Battery on, close any other serial monitor, and start the guide with
   PlatformIO's Python (it has pyserial):
   `~/.platformio/penv/bin/python tools/assembly_console.py --port <robot USB port>`
4. Follow the guide. Answer `q` at any question to stop; the console releases
   every servo when it closes and tells you which step to resume with
   `--start` (`prepare`, `servos`, `head`, `FL`, `FR`, `BL`, `BR`, `check`).
   Everything is logged to `results/<date>/assembly.log`.

Directions are the robot's own: **nose** is the head end, **left** and
**right** as seen from behind with the head pointing away from you. `_N`
parts go on the left, `_P` parts on the right.

## What the guide does

| Step | What happens |
|---|---|
| Servo setup | Every ID 11–19 gets its settings (`SETUP`). A missing ID is a factory-new servo, plugged in on its own; a reused servo is checked and corrected. Then, for each place, you **turn that horn by hand** about a quarter turn, top toward the nose, with torque off. The position readings show which servo it is and that it counts the right way. Reused servos keep the ID of the place they had in the old build, so after turning the body over many sit at another ID's place. The guide **renumbers** those over the bus (`MOVEID`, via a temporary ID 21–29): nothing is unplugged. |
| Head | Centres servo 19 (`CENTERHEAD`) and holds it while the head goes on, looking straight ahead. |
| Each leg (FL, FR, BL, BR) | Lists the parts, drives the pair to its mounting pose and holds it while you fit **the two cranks only**. It then releases the servos and lists the rest of the leg: humerus or femur, ulna, and the pushrods last. |
| Final check | With the legs hanging, it compares every left joint with its right partner. A crank one screw hole off shows up as about 90°. |

If a bare horn sits too far round for the firmware to move it, the guide shows
which way to turn it by hand and counts down the degrees until it is in range.

## The body the other way up

No servo setting or mounting target changes with the batteries facing down.
Each ID is defined by its **place on the robot**, not by a PCB slot:

| ID | Place | Drive mode | Homing offset | Carries | Mounting pose |
|---|---|:---:|---:|---|---|
| 11 | front-left, nearest the nose | 4 | +2048 | elbow crank | 4695 ticks, 52.660° |
| 12 | front-left, nearer the middle | 4 | +4096 | shoulder crank | 5564 ticks, 129.021° |
| 13 | front-right, nearest the nose | 5 | −2048 | elbow crank | 4695 ticks, 52.660° |
| 14 | front-right, nearer the middle | 5 | −4096 | shoulder crank | 5564 ticks, 129.021° |
| 15 | hind-left, nearer the middle | 4 | +2048 | elbow crank, femur pivot | 3627 ticks, −41.286° |
| 16 | hind-left, nearest the tail | 4 | +4096 | shoulder crank | 4745 ticks, +57.000° |
| 17 | hind-right, nearer the middle | 5 | −2048 | elbow crank, femur pivot | 3627 ticks, −41.286° |
| 18 | hind-right, nearest the tail | 5 | −4096 | shoulder crank | 4745 ticks, +57.000° |
| 19 | head, neck clamp | 4 | +2048 | head | 4096 ticks, centre |

Angles are from the nose direction, positive downward, as the robot firmware
and the GUI use them. Every horn faces outward, so left servos count normally
and right servos reversed, and on both sides a positive angle turns a crank
from the nose toward the floor. Turning the body over changes which PCB slot
ends up nearest the nose on the left. The settings for that place stay the
same, because the old table was mirror-symmetric left/right and the servo
slots point-symmetric front/back. Fitting each crank at the commanded pose
also absorbs any constant offset. What the flip *can* break is a servo
plugged into the slot the old Q8bot instructions named ("front left
vertical"), or a reused servo left in its old slot: that slot is no longer the
same place. Locating every servo by hand catches that, and a reversed
direction, before any crank goes on, and renumbering fixes it.

Where the cranks point at the mounting pose, seen from the side:

| Leg | Elbow crank | Shoulder crank |
|---|---|---|
| Front (ID11/13, ID12/14) | forward and down, 53° below horizontal | backward and down, 51° below horizontal |
| Hind (ID15/17, ID16/18) | forward and up, 41° above horizontal | forward and down, 57° below horizontal |

The front foot is the round-pad `SQ3_UlnaFoot`, the hind foot the pointed
`SQ4_UlnaFootHind`. All four legs use the same five links otherwise. The
offsets put each servo's power-up reading in a window, [2048, 6143] for ±2048
and [4096, 8191] for ±4096, and every mounting target lies inside its window.
The CAD angles and pivot positions are in
[cad-crank-references.json](results/2026-10-01/cad-crank-references.json).

## Commands

The guide sends these. `--manual` gives the raw console, which accepts only
these commands.

| Command | Effect |
|---|---|
| `SCAN` | Read-only. One line: each ID 11–19 `ok` (configured for its place), `bad` (answers, wrong settings) or `--`, plus `new=` the number of factory-new servos (ID 1) and `temp=` any servo left on a temporary ID 21–29. |
| `POS` | Read-only. Present positions of IDs 11–19 in ticks. |
| `SETUP <11-19>` | Configures the one factory-new servo on the bus (ID 1, 57600 or 1 Mbps) as that ID. If that ID already answers, it checks the servo and corrects drive mode, operating mode and offset. Writes to no other servo and never enables torque. Parks the goal at the present position. |
| `MOVEID <from> <to>` | Renames one servo, IDs 11–19 or 21–29, only onto a free ID. Writes torque-off (if needed) and the ID, nothing else; `SETUP` then gives it the settings of its new place. |
| `WIGGLE <11-19>` | One configured servo: +256 ticks (22.5°) and back at Goal PWM 133, then released and restored. A failure prints what stopped it. Refused while an alignment is active. Manual use only. |
| `CENTERHEAD` | ID19 to 4096 ticks over five seconds, then hold. |
| `ALIGNFL`, `ALIGNFR`, `ALIGNBL`, `ALIGNBR` | That leg pair to its mounting pose over five seconds, then hold. |
| `STATUS`, `LEGS` | Read-only head and leg inventories. |
| `KEEP` | Heartbeat. The console sends it every second. |
| `RELEASE`, `!` | Torque off, then profiles and output restored. |

Only `KEEP`, `RELEASE` and `!` are accepted while a pose is moving or held.

## Limits and verification

`CENTERHEAD` and `ALIGNxx` need all nine servos configured for their place,
torque off, watchdog 0, no hardware errors, 3.5–5.5 V and below 45 °C. A move is
refused when a bare horn starts more than 256 ticks (head), 1536 (front) or
2048 (hind) from its target; the guide then has it turned by hand. Head Goal
PWM is capped at 133 (about 15 % duty), legs at 266 (about 30 %), or any lower
existing Goal PWM / PWM Limit. These are output caps, not current limits.
Sampled current above ±150 mA for the head or ±250 mA for a leg servo causes
release, as do failed telemetry reads, unexpected position, lost torque,
temperature or voltage faults, and excess PWM. The same current limits apply
to `WIGGLE`. These thresholds are conservative bench choices, not manufacturer
operating limits.

Targets must settle within 23 ticks (2°) for 200 ms after the move time. At
its 15 % cap the head servo's own controller stalls about 9 ticks short
against the neck's friction; a degree does not matter, since horns take the
cranks and the head in 90° steps. During
the hold, more than 128 ticks (11°) of disturbance causes release. The earlier
32-tick guard tripped while fitting every leg, because screwing a crank down
nudges the horn by up to about 4°. Loss of heartbeat for 10 seconds, failure to
settle within eight seconds, or the five-minute hold limit also releases.
Profiles and Goal PWM are restored only after every selected servo is
confirmed torque-off. A failed release or restoration blocks every further
movement, `SETUP` and `MOVEID` until `RELEASE` succeeds; an unconfirmed torque-off needs
the battery switched off. MCU startup releases any responding ID 11–19.

`SETUP` reads back every value it writes, including at the new ID and speed.
Repeating it recovers an interrupted ID or speed change. The routine parks
each goal at its measured position before torque-on and does not restore old
goal positions.

Register interpretation: [ROBOTIS XL330-M077 manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/).

Host checks, from this folder:

```bash
for t in test_servo_setup test_squro_assembly test_head test_replacement test_alignment; do
  c++ -std=c++17 -Wall -Wextra tests/$t.cpp -o /tmp/$t && /tmp/$t
done
python3 -m unittest discover -s tools -p 'test_*.py'
```

Firmware build: `pio run -e assemble_squro`.

## Earlier sessions

The first robot was assembled with the earlier body orientation and the
bench-only version of this tool: the head and front pairs on 2026-10-01, the
hind pairs on 2026-10-12. Those sessions established the mapping that the
table above keeps: the elbow crank on the servo of each pair nearer the nose.
Their notes and serial logs are in [`results/`](results).
