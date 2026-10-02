# RatBot motor configuration

Setup tools for the nine XL330-M077-T servos: IDs 11 to 18 drive the legs and
ID 19 turns the head. Every environment here replaces the robot application,
so flash `firmware/ratbot_robot` (`robot_permanent`) again before operating.

| Environment | Use |
|---|---|
| `seeed_xiao_esp32c3` (default) | First setup of brand new servos: the eight leg servos, then the head |
| `config_head` | Add a head servo to a robot whose leg servos are already configured |
| `assemble_squro` | Bench alignment of the head and of one SQuRo leg pair at a time |
| `replace_id18`, `align_id18` | Replace a single leg servo, see [REPLACE_ID18.md](REPLACE_ID18.md) |

## First setup

Follow steps 10 to 20 of [Robot Assembly](../../docs/instructions/robot_assembly.md).
The program configures one new servo at a time, in ID order, so connect them in
that order: ID 11 to ID 18 on the PCB, then the head servo last. The head sits on
the front centreline, about 35 mm from either front outer leg servo. Daisy chain
it from the free connector of ID 11 or ID 13, whichever side is convenient; the
order on the bus does not matter.

Every servo gets extended position mode, 1 Mbps and its drive mode and homing
offset. The head gets drive mode 4 and the +2048 offset of the other first-of-pair
servos, so its mechanical centre reads 0° (4096 ticks) in the robot firmware.
Once all nine answer, the program turns torque on, drives the legs to the Q8
mounting pose (4618 / 5622 ticks) and the head to its centre, and holds them.
Fit the head facing straight ahead while it is held. The program waits for the
head after ID 18 and only finishes once all nine servos are configured.

In operation, `ratbot_robot` parks the head's goal at its present position
before every torque-on and then turns it as the GUI's Left and Right keys ask,
up to 90° either side and at most 90° per second.

## Adding the head to a configured robot

`config_head` configures only the head. It never enables torque, never commands
a movement and never writes to IDs 11 to 18. Use it instead of the default
environment on a robot with legs fitted: that program rewrites the EEPROM of all
eight leg servos and ends by driving every joint to the Q8 mounting pose, which
is the wrong reference for the SQuRo legs.

1. With battery power off, daisy chain the new head servo as described above.
2. Upload: `pio run -e config_head -t upload --upload-port <robot USB port>`.
3. Switch battery power on and open the serial monitor at 115200 baud.
4. Send `STATUS`. Expect one candidate (a new servo answers at ID 1, 57600 baud)
   and `Leg motors answering at 1 Mbps: 8 of 8`. More than one candidate means
   another unconfigured servo is on the bus.
5. Send `CONFIGHEAD` and wait for `CONFIGHEAD SUCCESS`. Every setting is read
   back, including at the new ID and baud rate. Repeating the command skips
   settings that are already correct and recovers an interrupted ID or baud change.

`CONFIGHEAD` refuses to run unless IDs 11 to 18 answer at 1 Mbps with torque off
and exactly one healthy head candidate is present. It prints how far the head
sits from centre and parks the goal at the present position, so the next
torque-on does not move it. `LEGS` prints a read-only inventory of IDs 11 to 18.

## SQuRo bench alignment

`assemble_squro` moves one selected pair to its CAD mounting reference and holds
it while the linkage is fitted. Nothing moves until a command is sent.

| Command | Servos | Target ticks | Refused if the start is further than |
|---|---|---|---|
| `CENTERHEAD` | 19 | 4096 | 256 ticks (22.5°) |
| `ALIGNFL`, `ALIGNFR` | 11/12, 13/14 | 4695 / 5564 (elbow / shoulder crank) | 1536 ticks (135°) |
| `ALIGNBL`, `ALIGNBR` | 15/16, 17/18 | 3627 / 4745 | 2048 ticks (180°) |

ID 11 is the front-left servo nearest the nose. Check the other pairs' mapping
before aligning them. The rear references are measured from the CAD and have not
been used on the robot yet.

1. Support the robot and the head and detach the selected pair's linkage.
2. Upload: `pio run -e assemble_squro -t upload --upload-port <robot USB port>`.
3. Start the console, which sends the `KEEP` heartbeat once a second but never
   starts a movement:
   `python tools/assembly_console.py --port <robot USB port> --log assembly.log`
   (it needs pyserial, which the PlatformIO Python already has).
4. Switch battery power on and check `STATUS` and `LEGS`.
5. Send one command from the table, wait for `ASSEMBLY READY`, fit the linkage
   (pushrods last) and send `RELEASE` or `!`.

Before moving, all nine servos must report the expected configuration, torque
off, no hardware errors, 3.5 to 5.5 V and less than 45 °C. Output is capped at a
Goal PWM of 133 for the head and 266 for the legs. The pair is released when the
heartbeat stops for 10 seconds, the target is not reached within 8 seconds, the
hold exceeds 5 minutes, the measured current exceeds 150 mA (head) or 250 mA
(legs), or the position is disturbed by more than 32 ticks. Only `KEEP`,
`RELEASE` and `!` are accepted while a pair is moving or held.

## Host tests

The setup, configuration and alignment programs run against a simulated servo
bus. `test_setup` runs `src/main.cpp` itself while attaching nine new servos in
ID order:

```bash
for t in test_setup test_head test_squro_assembly test_replacement test_alignment; do
  c++ -std=c++17 -Wall -Wextra -Itests/stubs tests/$t.cpp -o /tmp/$t && /tmp/$t
done
```
