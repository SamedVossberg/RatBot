#!/usr/bin/env python3
"""Guided RatBot assembly with the assemble_squro firmware.

Run it with PlatformIO's Python, which has pyserial:

    ~/.platformio/penv/bin/python tools/assembly_console.py --port /dev/cu.usbmodemXXXX

It walks through a fresh build with the biomimetic legs and the body as in the
RatBot STL files, batteries facing down: every servo is configured, located by
turning its horn by hand (and renumbered if it carries another place's ID), the
head is centred, and each leg pair is held at its mounting pose while its two
cranks go on. The firmware makes every safety check; this script sends
commands and explains each step.

--start resumes at a step, --manual gives the raw command console instead.
"""
import argparse
import datetime
from dataclasses import dataclass
from pathlib import Path
import queue
import re
import select
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
TICKS_PER_DEG = 4096 / 360
# These mirror src/squro_assembly.h; test_assembly_console checks they agree.
FRONT_TARGETS = (4695, 5564)      # elbow crank, shoulder crank
REAR_TARGETS = (3627, 4745)
HEAD_TARGET = 4096
FRONT_TRAVEL, REAR_TRAVEL, HEAD_TRAVEL = 1536, 2048, 256
MARGIN = 64                       # ticks kept inside the travel limit when turning by hand
# These mirror src/servo_setup.h: left servos and the head count normally (4),
# right servos reversed (5); 21-29 are temporary IDs while renumbering.
DRIVES = {11: 4, 12: 4, 13: 5, 14: 5, 15: 4, 16: 4, 17: 5, 18: 5, 19: 4}
OFFSETS = {11: 2048, 12: 4096, 13: -2048, 14: -4096, 15: 2048, 16: 4096, 17: -2048, 18: -4096, 19: 2048}
# After power-up a servo reads within one turn from |offset|; hand turning
# while powered can wind the reading on by whole turns.
WINDOWS = {servo: (abs(offset), abs(offset) + 4095) for servo, offset in OFFSETS.items()}
TEMP_SHIFT = 10
LOCATE_TICKS = round(30 * TICKS_PER_DEG)   # a horn turned this far by hand, at least three
                                           # times as far as any other not yet found, is found
HOLD_TOLERANCE = 32               # ticks a held crank may sit off its target after fitting
KEYING_LIMIT = 45.0               # deg; a crank one screw hole off is 90 deg off its partner


@dataclass(frozen=True)
class Place:
    id: int
    name: str          # short, e.g. 'front-left, nose end'
    where: str         # how to find it on the robot
    target: int        # mounting pose, ticks
    travel: int        # largest move the firmware makes from a bare horn


PLACES = {p.id: p for p in (
    Place(11, 'front-left, nose end', 'front-left leg pair, the servo nearest the nose',
          FRONT_TARGETS[0], FRONT_TRAVEL),
    Place(12, 'front-left, middle', 'front-left leg pair, the servo nearer the middle of the body',
          FRONT_TARGETS[1], FRONT_TRAVEL),
    Place(13, 'front-right, nose end', 'front-right leg pair, the servo nearest the nose',
          FRONT_TARGETS[0], FRONT_TRAVEL),
    Place(14, 'front-right, middle', 'front-right leg pair, the servo nearer the middle of the body',
          FRONT_TARGETS[1], FRONT_TRAVEL),
    Place(15, 'hind-left, middle', 'hind-left leg pair, the servo nearer the middle of the body',
          REAR_TARGETS[0], REAR_TRAVEL),
    Place(16, 'hind-left, tail end', 'hind-left leg pair, the servo nearest the tail',
          REAR_TARGETS[1], REAR_TRAVEL),
    Place(17, 'hind-right, middle', 'hind-right leg pair, the servo nearer the middle of the body',
          REAR_TARGETS[0], REAR_TRAVEL),
    Place(18, 'hind-right, tail end', 'hind-right leg pair, the servo nearest the tail',
          REAR_TARGETS[1], REAR_TRAVEL),
    Place(19, 'head', 'head (in the neck clamp, cabled to the free connector of servo 11 or 13)',
          HEAD_TARGET, HEAD_TRAVEL),
)}


@dataclass(frozen=True)
class Leg:
    key: str
    title: str
    ids: tuple         # (elbow-crank servo, shoulder-crank servo)
    command: str
    suffix: str        # _N parts on the left, _P on the right
    front: bool


LEGS = (
    Leg('FL', 'front-left leg', (11, 12), 'ALIGNFL', '_N', True),
    Leg('FR', 'front-right leg', (13, 14), 'ALIGNFR', '_P', True),
    Leg('BL', 'hind-left leg', (15, 16), 'ALIGNBL', '_N', False),
    Leg('BR', 'hind-right leg', (17, 18), 'ALIGNBR', '_P', False),
)
STEPS = ('prepare', 'servos', 'head') + tuple(leg.key for leg in LEGS) + ('check',)
TITLES = {'prepare': 'Before you start', 'servos': 'Servo setup', 'head': 'Head', 'check': 'Final check',
          **{leg.key: leg.title.capitalize() for leg in LEGS}}


def degrees(ticks):
    """Robot-firmware angle: from pointing at the nose, positive downward."""
    return (ticks - 4096) / TICKS_PER_DEG


def pointing(ticks):
    """Where a crank at this angle points, seen from the side."""
    deg = degrees(ticks)
    forward = -90 < deg < 90
    below = deg if forward else 180 - deg
    return (f"{'FORWARD' if forward else 'BACKWARD'} (toward the {'nose' if forward else 'tail'}) "
            f"and {'DOWN' if below > 0 else 'UP'}, {abs(below):.0f}° "
            f"{'below' if below > 0 else 'above'} horizontal")


def parts(leg):
    s = leg.suffix
    ulna = f'SQ3_UlnaFoot{s} (round pad)' if leg.front else f'SQ4_UlnaFootHind{s} (pointed toe)'
    return [f'SQ3_ElbowCrank{s}', f'SQ3_ShoulderCrank{s}', f'SQ3_Humerus{s}',
            f'SQ3_HipPushrod{s}', f'SQ3_KneePushrod{s}', ulna]


def crank_steps(leg):
    elbow, shoulder = leg.ids
    return [f'Elbow crank SQ3_ElbowCrank{leg.suffix} on servo {elbow} ({PLACES[elbow].name}): '
            f'pointing {pointing(PLACES[elbow].target)}.',
            f'Shoulder crank SQ3_ShoulderCrank{leg.suffix} on servo {shoulder} '
            f'({PLACES[shoulder].name}): pointing {pointing(PLACES[shoulder].target)}.']


def linkage_steps(leg):
    s = leg.suffix
    elbow, _ = leg.ids
    upper = 'humerus' if leg.front else 'femur'
    joint = 'elbow' if leg.front else 'knee'
    ulna = parts(leg)[-1]
    return [f'{upper.capitalize()} SQ3_Humerus{s}: pivots on the axis of servo {elbow}, beside the elbow crank.',
            f'Ulna {ulna}: on the far end of the {upper}, the {joint}.',
            f'Hip pushrod SQ3_HipPushrod{s}: links the shoulder crank to the {upper}. This closes the '
            f'shoulder parallelogram, so the {upper} stays parallel to the shoulder crank.',
            f'Knee pushrod SQ3_KneePushrod{s}: links the elbow crank to the ulna. This closes the '
            'elbow parallelogram, so the ulna stays parallel to the elbow crank.',
            'Pushrods last. Both parallelograms must close without forcing a part.']


def parse_fields(line, prefix):
    """'SCAN 11=ok 12=-- new=1' -> {11: 'ok', 12: '--', 'new': '1'}."""
    fields = {}
    for key, value in re.findall(r'(\w+)=(\S+)', line[len(prefix):]):
        fields[int(key) if key.isdigit() else key] = value
    return fields


class Quit(Exception):
    """The person chose to stop."""


class Guide:
    """The assembly steps. link talks to the firmware, ui to the person."""

    def __init__(self, link, ui):
        self.link = link
        self.ui = ui

    # --- firmware ----------------------------------------------------------
    def scan(self):
        line = self.link.command('SCAN', ('SCAN ',), 8)
        if not line:
            raise RuntimeError('no SCAN reply')
        fields = parse_fields(line, 'SCAN')
        fields['new'] = int(fields.get('new', 0))
        temp = fields.get('temp', '-')
        fields['temp'] = [] if temp == '-' else [int(t) for t in temp.split(',')]
        return fields

    def setup(self, servo):
        line = self.link.command(f'SETUP {servo}', ('SETUP OK', 'SETUP FAIL', 'SETUP REFUSED'), 10)
        if line and line.startswith('SETUP OK'):
            return True
        if line is None and self.scan().get(servo) == 'ok':
            return True                  # The reply was lost; the servo reads back right.
        self.ui.say(f'Setting up ID {servo} did not complete: {line or "no reply"}')
        return False

    def move(self, old, new):
        line = self.link.command(f'MOVEID {old} {new}', ('MOVEID OK', 'MOVEID FAIL', 'MOVEID REFUSED'), 5)
        if line and line.startswith('MOVEID OK'):
            return
        if line is None:                 # The reply was lost: did the ID change?
            scan = self.scan()

            def present(servo):
                return servo in scan['temp'] if servo > 20 else scan.get(servo) in ('ok', 'bad')

            if present(new) and not present(old):
                return
        self.ui.say(f'Renumbering ID {old} to {new} did not complete: {line or "no reply"}')
        raise Quit

    def positions(self):
        for _ in range(3):               # A late reply is retried, not fatal.
            line = self.link.command('POS', ('POS ',), 3)
            if line:
                return {key: (None if value == '--' else int(value))
                        for key, value in parse_fields(line, 'POS').items()}
        raise RuntimeError('no POS reply')

    def release(self):
        line = self.link.command('RELEASE', ('ASSEMBLY RELEASED', 'ASSEMBLY ID'), 6)
        if line and 'restored' in line:
            return True
        self.ui.say('Torque-off or restoring the servo settings was not confirmed. Switch the battery '
                    'off before touching the legs, then on again, and resume this step.')
        raise Quit

    def hold(self, command):
        """Drive to a mounting pose and wait until it is held."""
        self.link.stopped()
        self.ui.say('Moving (5 seconds)...')
        line = self.link.command(command, ('ASSEMBLY READY', 'ASSEMBLY REFUSED', 'ASSEMBLY ABORT',
                                           'ASSEMBLY STOP'), 15)
        if line and line.startswith('ASSEMBLY READY'):
            return True
        self.ui.say(f'The servos did not reach the pose: {line or "no reply"}')
        return False

    # --- steps ---------------------------------------------------------------
    def run(self, start='prepare'):
        first = STEPS.index(start)
        step = start
        try:
            for number, step in enumerate(STEPS[first:], first + 1):
                self.ui.header(number, len(STEPS), TITLES[step])
                leg = next((leg for leg in LEGS if leg.key == step), None)
                if leg:
                    self.do_leg(leg)
                else:
                    getattr(self, f'do_{step}')()
        except (Quit, RuntimeError) as stop:
            if isinstance(stop, RuntimeError):
                self.ui.say(f'\nThe robot stopped answering ({stop}). Check the USB cable and the battery.')
            self.ui.say('\nStopped. Every servo is released when this console closes.')
            self.ui.say(f'Resume with:  --start {step}')
            return False
        self.ui.say('\nAssembly complete.')
        return True

    def do_prepare(self):
        self.ui.say('This guide sets up every servo, then fits the head and the four legs.')
        self.ui.bullets([
            'Body as in the RatBot STL files: batteries facing DOWN, toward the belly.',
            'Directions are the robot\'s own. NOSE is the head end. LEFT and RIGHT are as seen '
            'from behind, with the head pointing away from you.',
            '_N parts go on the left, _P parts on the right.',
            'No cranks or leg parts on any servo yet. Support the robot so every horn turns freely.',
            'Battery on, robot USB connected, no other serial monitor open.',
            'Answer q at any question to stop; the servos are released.'])
        self.ui.ask('Press Enter when ready.')
        try:
            self.scan()
        except RuntimeError:
            self.ui.say('The robot does not answer. Upload the assembly firmware first:\n'
                        '  pio run -e assemble_squro -t upload --upload-port <robot USB port>')
            raise Quit

    def do_servos(self):
        self.ui.say('First every servo gets the settings for its ID. Then you turn each horn by hand '
                    'once, so the guide learns which servo sits where. Reused servos keep the ID of '
                    'the place they had before; with the body turned over that is often another '
                    'place. Those are renumbered over the bus: nothing needs unplugging.')
        scan = self.wait_for_power()
        for temp in scan['temp']:
            servo = temp - TEMP_SHIFT
            self.ui.say(f'A servo was left on temporary ID {temp} by an interrupted renumbering: '
                        f'it becomes ID {servo}.')
            self.move(temp, servo)
            if not self.setup(servo):
                raise Quit
        for servo in PLACES:
            while not self.configure(PLACES[servo]):
                pass
        self.locate_all()
        self.unwind()

    def unwind(self):
        """Have the servos power-cycled if hand turning wound their readings up."""
        def wound(positions):
            return [servo for servo in PLACES if positions.get(servo) is None or
                    not WINDOWS[servo][0] <= positions[servo] <= WINDOWS[servo][1]]

        far = wound(self.positions())
        if not far:
            return
        self.ui.say('Turning the horns by hand wound some readings on by whole turns (ID '
                    + ', '.join(map(str, far)) + '). Switch the battery off, wait two seconds and '
                    'switch it on again: the servos then count from within one turn.')

        def fresh():
            positions = self.positions()
            if not wound(positions):
                return 'ok'
            self.ui.status('Waiting for every servo to answer...' if any(
                positions.get(servo) is None for servo in PLACES) else 'Still wound up: battery off, then on.')
            return None

        self.ui.ask('This continues by itself once every servo is back.', (), poll=fresh, every=1.0)
        self.ui.say('Readings reset.')

    @staticmethod
    def answering(scan):
        return scan['new'] > 0 or scan['temp'] or any(scan.get(i) in ('ok', 'bad') for i in PLACES)

    def wait_for_power(self):
        scan = self.scan()
        if self.answering(scan):
            return scan
        self.ui.say('No servo answers. Switch the battery on: USB powers only the board, not the '
                    'servos. With factory-new servos, plug in only the first one, for the '
                    f'{PLACES[11].where}.')

        def powered():
            scan = self.scan()
            return scan if self.answering(scan) else None

        return self.ui.ask('This continues by itself once a servo answers.', (), poll=powered, every=2.0)

    def configure(self, place):
        """Give ID place.id its settings, plugging in a new servo if none has it."""
        state = self.scan().get(place.id)
        if state == 'ok':
            self.ui.say(f'ID {place.id}: answers, settings correct.')
            return True
        if state == '--':
            self.wait_for_new(place)
        if not self.setup(place.id):
            self.ui.ask('Press Enter to try again.')
            return False
        self.ui.say(f'ID {place.id}: ' + ('settings corrected.' if state == 'bad' else 'new servo set up.'))
        return True

    def wait_for_new(self, place):
        self.ui.say(f'No servo has ID {place.id}. Plug in the servo for the {place.where}, and only '
                    'this one: every new servo answers as ID 1 until it is set up.')

        def plugged():
            scan = self.scan()
            if scan['new'] == 1 or scan.get(place.id) in ('ok', 'bad'):
                return 'found'
            self.ui.status('Waiting for it...' if scan['new'] == 0 else
                           f'{scan["new"]} new servos answer: unplug all but one.')
            return None

        self.ui.ask('This continues by itself once it answers.', (), poll=plugged, every=2.0)

    def locate_all(self):
        """Find which ID sits at each place, by hand; renumber the ones that differ."""
        self.ui.say('\nNow find each servo. Torque is off: turn each horn by hand when asked, about a '
                    'quarter turn, and leave the others alone.')
        found = {}
        for place in PLACES.values():
            found[place.id] = self.locate(place, found)
        wrong = {place: servo for place, servo in found.items() if place != servo}
        if not wrong:
            self.ui.say('Every servo sits where its ID belongs.')
            return
        self.ui.say('These servos carry the ID of another place and get renumbered:')
        self.ui.bullets([f'{PLACES[place].name}: ID {servo} becomes ID {place}'
                         for place, servo in wrong.items()])
        self.ui.ask('Press Enter to renumber them. Nothing moves.')
        for place, servo in wrong.items():
            self.move(servo, place + TEMP_SHIFT)
        for place in wrong:
            self.move(place + TEMP_SHIFT, place)
            if not self.setup(place):
                raise Quit
        scan = self.scan()
        if any(scan.get(servo) != 'ok' for servo in PLACES):
            self.ui.say('Not every ID reads back correctly after renumbering: '
                        + ' '.join(f'{servo}={scan.get(servo)}' for servo in PLACES))
            raise Quit
        self.ui.say('Renumbered: every servo now has the ID and settings of its place.')

    def locate(self, place, found):
        """The ID of the servo whose horn is turned by hand at place."""
        if place.id == 19:
            how = 'Turn the head servo\'s horn by hand, about a quarter turn, either way.'
        else:
            how = (f'Turn the horn of the {place.where} by hand, about a quarter turn, so the TOP of '
                   'the horn moves toward the NOSE.')
        while True:
            self.ui.say(how)
            baseline = self.positions()
            turned = {}

            def watch():
                now = self.positions()
                moves = {servo: now[servo] - baseline[servo] for servo in PLACES
                         if now.get(servo) is not None and baseline.get(servo) is not None}
                # Horns already found may still be settling from their turn:
                # they neither count nor block.
                open_moves = sorted(((abs(d), s) for s, d in moves.items() if s not in found.values()),
                                    reverse=True) or [(0, None)]
                (lead, servo), runner_up = open_moves[0], (open_moves[1][0] if len(open_moves) > 1 else 0)
                if lead >= LOCATE_TICKS and runner_up * 3 <= lead:
                    turned['delta'] = moves[servo]
                    return servo
                done = max(((abs(d), s) for s, d in moves.items() if s in found.values()), default=(0, None))
                if done[0] >= LOCATE_TICKS and done[0] > lead:
                    other = next(p for p, s in found.items() if s == done[1])
                    self.ui.status(f'That is the {PLACES[other].name} horn from before; '
                                   f'turn the one at the {place.where}.')
                elif lead >= LOCATE_TICKS:
                    self.ui.status('Two horns turned: keep turning only this one.')
                else:
                    self.ui.status(f'Turned {lead / TICKS_PER_DEG:.0f}° so far...')
                return None

            servo = self.ui.ask(f'Turn the {place.name} horn now. This continues by itself.',
                                (), poll=watch, every=0.4)
            # The reading counts the way its ID's drive mode says, so a servo
            # carrying an ID from the other side reads reversed.
            reversed_reading = turned['delta'] * (1 if DRIVES[servo] == DRIVES[place.id] else -1) < 0
            if place.id != 19 and reversed_reading:
                self.ui.say('That read as the TOP of the horn moving toward the TAIL.')
                if self.ui.ask('[r] turn it again, TOP toward the NOSE / [s] I am sure it went toward '
                               'the nose', ('r', 's')) == 's':
                    self.ui.say('Then this servo turns the wrong way for its place. Every horn must face '
                                'outward, away from the body: check that it sits the right way round in '
                                'its slot, like the other servos on this side.')
                    self.ui.ask('Press Enter once it is fixed.')
                continue
            self.ui.say(f'Found the {place.name} servo' +
                        ('.' if servo == place.id else f': it has ID {servo}.'))
            return servo

    def bring_into_range(self, ids):
        """Have bare horns turned by hand until the firmware will move them."""
        def remaining(servo, positions):
            place = PLACES[servo]
            error = positions[servo] - place.target
            return max(0, abs(error) - (place.travel - MARGIN)) / TICKS_PER_DEG, error

        positions = self.positions()
        far = [servo for servo in ids if remaining(servo, positions)[0] > 0]
        if not far:
            return
        for servo in far:
            degrees_left, error = remaining(servo, positions)
            if servo == 19:
                how = 'turn its bare horn by hand, either way; the number shows how far is left'
            else:
                how = (f'turn its bare horn by hand so the TOP of the horn moves toward the '
                       f'{"NOSE" if error < 0 else "TAIL"}')
            self.ui.say(f'Servo {servo} ({PLACES[servo].name}) is {degrees_left:.0f}° too far round to '
                        f'start from: {how}.')

        def check():
            positions = self.positions()
            left = {servo: remaining(servo, positions)[0] for servo in ids}
            self.ui.status('   '.join(f'servo {servo}: {"OK" if d == 0 else f"{d:.0f}° to go"}'
                                      for servo, d in left.items()))
            return 'ok' if not any(left.values()) else None

        self.ui.ask('Torque is off. This continues by itself once every horn is in range.',
                    (), poll=check, every=0.4)

    def fit_under_hold(self, command, ids, instructions):
        """Hold a pose while something is fitted; returns once it is fitted and released."""
        while True:
            self.bring_into_range(ids)
            if not self.hold(command):
                self.ui.ask('Press Enter to try again.')
                continue
            self.ui.say('Holding. Now, with the servos holding (up to 5 minutes):')
            self.ui.bullets(instructions)

            def watch():
                line = self.link.stopped()
                return 'stopped' if line else None

            answer = self.ui.ask('Press Enter once everything is screwed down (r = drive to the pose again).',
                                 ('', 'r'), poll=watch, every=0.5)
            if answer == 'stopped':
                self.release()           # Clears a release the firmware could not confirm.
                self.ui.say('The servos let go before you finished (pushed too far, or the 5-minute '
                            'limit). Loosen the screws you fitted, then drive to the pose again.')
                self.ui.ask('Press Enter when the screws are loose.')
                continue
            if answer == 'r':
                self.release()
                continue
            held = self.link.held()
            off = {servo: (held[servo] - PLACES[servo].target) / TICKS_PER_DEG
                   for servo in ids if servo in held and
                   abs(held[servo] - PLACES[servo].target) > HOLD_TOLERANCE}
            self.release()
            for servo, deg in off.items():
                self.ui.say(f'Note: servo {servo} was held {deg:+.0f}° off its target while you fitted '
                            'it. If a part was forced, redo this step.')
            return

    def do_head(self):
        self.ui.say('The head servo (19) sits in the neck clamp on the frame\'s front tongue. It is '
                    'centred and held while the head goes on.')
        if self.ui.ask('Press Enter to centre it (s = skip the head).', ('', 's')) == 's':
            return
        self.fit_under_hold('CENTERHEAD', (19,), [
            'Fit the head RatBot_RatHead_Cap_v2 on servo 19, looking straight ahead along the body.'])
        self.ui.say('Head done.')

    def do_leg(self, leg):
        self.ui.say('Parts: ' + ', '.join(parts(leg)) + '.')
        self.ui.say('The two cranks go on while the servos hold the mounting pose; they fix the '
                    'reference for every later angle. The rest of the leg goes on afterwards.')
        if self.ui.ask(f'Horns of servos {leg.ids[0]} and {leg.ids[1]} bare? Press Enter to start '
                       '(s = skip this leg).', ('', 's')) == 's':
            return
        self.fit_under_hold(leg.command, leg.ids, crank_steps(leg) + [
            'The cranks go on the horn screw holes in steps of 90°: pick the position closest to '
            'the direction above and never force a crank round.'])
        self.ui.say('Servos released. Now fit the rest of the leg:')
        self.ui.bullets(linkage_steps(leg))
        self.ui.ask('Press Enter when the leg is complete.')

    def do_check(self):
        self.ui.say('Hold the robot up so all four legs hang freely. Torque is off.')
        self.ui.ask('Press Enter to read the joints.')
        positions = self.positions()
        suspicious = []
        for left, right in ((11, 13), (12, 14), (15, 17), (16, 18)):
            if positions.get(left) is None or positions.get(right) is None:
                self.ui.say(f'Servo {left} or {right} did not answer.')
                suspicious.append((left, right))
                continue
            a, b = degrees(positions[left]), degrees(positions[right])
            self.ui.say(f'  {PLACES[left].name:<22} {a:7.1f}°   {PLACES[right].name:<23} {b:7.1f}°   '
                        f'difference {abs(a - b):5.1f}°')
            if abs(a - b) > KEYING_LIMIT:
                suspicious.append((left, right))
        if suspicious:
            self.ui.say('A left/right pair differs by more than 45°: one crank is probably a screw hole '
                        'off. Redo that leg with  --start <leg>.')
        else:
            self.ui.say('Left and right legs agree: the cranks are keyed alike.')
        self.ui.say('\nNext:')
        self.ui.bullets([
            'Flash the robot firmware:  cd firmware/ratbot_robot && pio run -e robot_permanent -t upload',
            'Start the GUI with the biomimetic legs:  cd python-tools/ratbot && python operate.py --legs BIOMIMETIC',
            'Set the robot on its feet and press T to stand up.'])


class Link:
    """Serial link to assemble_squro: a reader thread, KEEP every second, a log."""

    STOPS = ('ASSEMBLY ABORT', 'ASSEMBLY STOP')

    def __init__(self, port_name, log, echo=False):
        import serial
        self.serial = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=2)
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.port = port_name
        self.serial.open()
        self.log, self.echo = log, echo
        self.lines = queue.Queue()
        self.write_lock = threading.Lock()
        self._held, self._stop = {}, None
        self.alive = self.keeping = True
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._keep, daemon=True).start()

    def _read(self):
        pending = ''
        while self.alive:
            try:
                data = self.serial.read(4096)
            except Exception:  # noqa: BLE001 - port gone; the firmware releases on its own
                break
            if not data:
                continue
            pending += data.decode(errors='replace')
            *complete, pending = pending.split('\n')
            for line in complete:
                line = line.rstrip('\r')
                self.log.write(line + '\n')
                if self.echo:
                    print(f'  | {line}')
                match = re.match(r'ASSEMBLY holding ID(\d+) pos=(-?\d+)', line)
                if match:
                    self._held[int(match.group(1))] = int(match.group(2))
                if line.startswith(self.STOPS):
                    self._stop = line
                self.lines.put(line)

    def _keep(self):
        while self.keeping:
            time.sleep(1)
            if self.keeping:
                self.send('KEEP')

    def send(self, command):
        if command != 'KEEP':
            stamp = datetime.datetime.now().isoformat(timespec='seconds')
            self.log.write(f'[HOST {stamp}] Sending {command}\n')
        with self.write_lock:
            self.serial.write((command + '\n').encode())

    def command(self, command, until, timeout):
        """Send a command; the first reply line starting with one of until, or None."""
        while not self.lines.empty():
            self.lines.get_nowait()
        if command.startswith(('ALIGN', 'CENTERHEAD')):
            self._held = {}
        self.send(command)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            if line.startswith(until):
                return line
        return None

    def stopped(self):
        """The abort/stop line since the last call, if the firmware let go."""
        line, self._stop = self._stop, None
        return line

    def held(self):
        return dict(self._held)

    def close(self):
        self.keeping = False             # '!' is the last thing the firmware hears.
        try:
            self.send('!')
            time.sleep(1.5)
        finally:
            self.alive = False
            time.sleep(0.2)
            self.serial.close()


class Terminal:
    """The person at the keyboard. Every prompt and answer goes to the log too."""

    def __init__(self, log):
        self.log = log
        self._status = False

    def _end_status(self):
        if self._status:
            print()
            self._status = False

    def _print(self, text):
        self._end_status()
        print(text, flush=True)
        self.log.write(f'[GUIDE] {text}\n')

    def header(self, number, total, title):
        self._print(f'\n=== Step {number}/{total}: {title} ===')

    def say(self, text=''):
        self._print(text)

    def bullets(self, items):
        for item in items:
            self._print(f'  - {item}')

    def status(self, text):
        print(f'\r  {text:<100}', end='', flush=True)
        self._status = True

    def ask(self, prompt, choices=('',), poll=None, every=0.5):
        self._print(prompt)
        next_poll = 0.0
        while True:
            if poll and time.monotonic() >= next_poll:
                result = poll()
                next_poll = time.monotonic() + every
                if result is not None:
                    self._end_status()
                    return result
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            line = sys.stdin.readline()
            answer = line.strip().lower()
            if not line or answer == 'q':
                raise Quit
            self.log.write(f'[ANSWER] {answer}\n')
            if choices is None or answer in choices:
                return answer
            if poll and answer == '':
                next_poll = 0.0          # Enter: check now
                continue
            shown = '/'.join(c or 'Enter' for c in choices) or 'Enter to check now'
            self._print(f'Please answer {shown} (q stops).')


MANUAL_COMMANDS = re.compile(r'^(SCAN|POS|STATUS|LEGS|CENTERHEAD|ALIGN(FL|FR|BL|BR)|RELEASE|!|'
                             r'(SETUP|WIGGLE) 1[1-9]|MOVEID [12][1-9] [12][1-9])$')


def manual(link):
    """The raw console: firmware lines are echoed, commands checked before sending."""
    print('Commands: SCAN, POS, SETUP <11-19>, WIGGLE <11-19>, MOVEID <from> <to>, STATUS, LEGS, CENTERHEAD, '
          'ALIGNFL, ALIGNFR, ALIGNBL, ALIGNBR, RELEASE, !, EXIT')
    while True:
        line = sys.stdin.readline()
        if not line or line.strip() == 'EXIT':
            return
        command = line.strip()
        if MANUAL_COMMANDS.match(command):
            link.send(command)
        else:
            print('[HOST] Command rejected. Nothing sent.')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True, help='robot USB serial port')
    parser.add_argument('--log', type=Path,
                        default=HERE.parent / 'results' / datetime.date.today().isoformat() / 'assembly.log')
    parser.add_argument('--start', choices=STEPS, default='prepare', help='resume at this step')
    parser.add_argument('--manual', action='store_true', help='raw command console instead of the guide')
    parser.add_argument('--verbose', action='store_true', help='show every firmware line')
    args = parser.parse_args()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open('a', buffering=1) as log:
        log.write(f'\n[HOST] Connected to {args.port}. Automatic KEEP only.\n')
        link = Link(args.port, log, echo=args.verbose or args.manual)
        try:
            if args.manual:
                manual(link)
            else:
                Guide(link, Terminal(log)).run(args.start)
        except KeyboardInterrupt:
            print('\nInterrupted.')
        finally:
            link.close()
            print(f'Console closed; every servo was told to release. Log: {args.log}')


if __name__ == '__main__':
    main()
