"""Checks of the guided assembly console against a simulated assemble_squro.

python3 -m unittest discover -s tools -p 'test_*.py'   (from firmware/ratbot_motor_config)
"""
from pathlib import Path
import re
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import assembly_console as ac  # noqa: E402

SRC = Path(__file__).resolve().parent.parent / 'src'
IDENTITY = {place: place for place in ac.PLACES}
# Reused servos in their old PCB slots, body turned over about its long axis:
# left and right swap. End over end: front and hind swap, left stays left.
ROLLED = {11: 13, 12: 14, 13: 11, 14: 12, 15: 17, 16: 18, 17: 15, 18: 16, 19: 19}
PITCHED = {11: 16, 12: 15, 13: 18, 14: 17, 15: 12, 16: 11, 17: 14, 18: 13, 19: 19}


class FakeRobot:
    """assemble_squro as the guide sees it, over physical servos at places."""

    def __init__(self, layout=IDENTITY, positions=None, powered=True, answers=True):
        # place -> the servo sitting there: the ID it carries and its reading.
        self.servos = {place: {'id': servo, 'state': 'ok',
                               'pos': (positions or {}).get(place, ac.PLACES[place].target)}
                       for place, servo in layout.items() if servo is not None}
        self.plugged = None              # place of a factory-new servo (ID 1) on the bus
        self.powered, self.answers = powered, answers
        self.sent = []
        self.holding = ()
        self.stops = 0                   # holds that will be lost while fitting
        self._stop = None
        self.wrong_way = set()           # places whose horn is first turned toward the tail
        self.settle = 0                  # ticks the last horn turned keeps moving afterwards
        self.last_turned = None
        self.lost_pos = 0                # POS replies that arrive too late
        self.lost = set()                # SETUP/MOVEID commands done, but whose reply is lost

    def by_id(self, servo):
        return next((s for s in self.servos.values() if s['id'] == servo), None) if self.powered else None

    # What the person does.
    def power_on(self, prompt=''):
        self.powered = True

    def power_cycle(self, prompt=''):
        """Battery off and on: every reading starts again within its window."""
        for servo in self.servos.values():
            low = ac.WINDOWS[servo['id']][0]
            servo['pos'] = low + (servo['pos'] - low) % 4096

    def plug_next(self, prompt=''):
        empty = [place for place in ac.PLACES if place not in self.servos]
        if self.powered and self.plugged is None and empty:
            self.plugged = empty[0]

    def turn_named(self, prompt):
        """Turn the horn at the place the prompt names, top toward the nose."""
        place = next(p for p in ac.PLACES.values() if f'the {p.name} horn' in prompt)
        if self.last_turned not in (None, place.id):
            # The person finishes the previous quarter turn while reading on:
            # as on the bench, 11° more after that horn was found.
            self.servos[self.last_turned]['pos'] += self.settle
        self.last_turned = place.id
        toward_nose = place.id not in self.wrong_way
        self.wrong_way.discard(place.id)
        servo = self.servos[place.id]
        # Physics: the encoder counts the way the ID's drive mode says; the
        # place decides which way that is in the world.
        sense = 1 if ac.DRIVES[servo['id']] == ac.DRIVES[place.id] else -1
        servo['pos'] += 1024 * sense * (1 if toward_nose else -1)

    def turn_into_range(self, prompt=''):
        for servo in self.servos.values():
            place = ac.PLACES[servo['id']]
            error = servo['pos'] - place.target
            if abs(error) > place.travel - ac.MARGIN:
                servo['pos'] -= 300 if error > 0 else -300

    # The link.
    def command(self, command, until, timeout):
        self.sent.append(command)
        reply = self.reply(command) if self.answers else None
        if command in self.lost:
            self.lost.discard(command)
            return None
        return reply if reply and reply.startswith(until) else None

    def reply(self, command):
        if command == 'SCAN':
            states = ' '.join(f'{i}={(self.by_id(i) or {}).get("state", "--")}' for i in ac.PLACES)
            temps = sorted(s['id'] for s in self.servos.values() if s['id'] > 20) if self.powered else []
            return (f'SCAN {states} new={1 if self.plugged else 0} '
                    f'temp={",".join(map(str, temps)) or "-"}')
        if command == 'POS':
            if self.lost_pos:
                self.lost_pos -= 1
                return None
            return 'POS ' + ' '.join(f'{i}={(self.by_id(i) or {}).get("pos", "--")}' for i in ac.PLACES)
        if command.startswith('SETUP '):
            servo = int(command[6:])
            if self.by_id(servo):
                self.by_id(servo)['state'] = 'ok'
                return f'SETUP OK ID{servo}: existing servo checked'
            if self.plugged is None:
                return f'SETUP FAIL ID{servo}: no new servo (ID1) found.'
            self.servos[self.plugged] = {'id': servo, 'state': 'ok', 'pos': ac.PLACES[self.plugged].target}
            self.plugged = None
            return f'SETUP OK ID{servo}: new servo configured'
        if command.startswith('MOVEID '):
            old, new = map(int, command.split()[1:])
            servo = self.by_id(old)
            if not servo or self.by_id(new):
                return f'MOVEID FAIL {old}->{new}: taken or missing'
            servo['id'], servo['state'] = new, 'bad'   # Settings of the old place until SETUP.
            return f'MOVEID OK {old}->{new}'
        if command.startswith(('ALIGN', 'CENTERHEAD')):
            ids = {'CENTERHEAD': (19,), 'ALIGNFL': (11, 12), 'ALIGNFR': (13, 14),
                   'ALIGNBL': (15, 16), 'ALIGNBR': (17, 18)}[command]
            if any((self.by_id(i) or {}).get('state') != 'ok' for i in ac.PLACES):
                return 'ASSEMBLY REFUSED: ID11 configuration, torque, communication or health check failed.'
            for i in ids:
                if abs(self.by_id(i)['pos'] - ac.PLACES[i].target) > ac.PLACES[i].travel:
                    return f'ASSEMBLY REFUSED: ID{i} position or RAM settings outside checked range.'
            for i in ids:
                self.by_id(i)['pos'] = ac.PLACES[i].target
            self.holding = ids
            if self.stops:
                self.stops -= 1
                self._stop = f'ASSEMBLY ABORT: ID{ids[0]} telemetry/read check failed.'
            return 'ASSEMBLY READY: measured targets reached; holding up to 5 minutes.'
        if command == 'RELEASE':
            self.holding = ()
            return 'ASSEMBLY RELEASED: torque OFF; profiles/output restored.'
        raise AssertionError(command)

    def stopped(self):
        line, self._stop = self._stop, None
        return line

    def held(self):
        return {i: self.by_id(i)['pos'] for i in self.holding}

    def moves(self):
        return [c for c in self.sent if c not in ('SCAN', 'POS')]

    def ids_by_place(self):
        return {place: servo['id'] for place, servo in self.servos.items()}


class FakeUI:
    """A person: scripted answers by prompt text, Enter or 'y' otherwise.
    Hooks are what they do while a prompt is open, given the prompt."""

    def __init__(self, answers=None, hooks=None):
        self.answers = {key: list(values) for key, values in (answers or {}).items()}
        self.hooks = hooks or {}
        self.text = []

    def header(self, number, total, title):
        self.text.append(f'Step {number}/{total}: {title}')

    def say(self, text=''):
        self.text.append(text)

    def bullets(self, items):
        self.text.extend(items)

    def status(self, text):
        self.text.append(text)

    def ask(self, prompt, choices=('',), poll=None, every=0.5):
        self.text.append(prompt)
        hooks = [hook for key, hook in self.hooks.items() if key in prompt]
        for key, queued in self.answers.items():
            if key in prompt and queued:
                answer = queued.pop(0)
                if answer == 'q':
                    raise ac.Quit
                if poll is None or answer != 'poll':
                    assert answer in choices, (answer, prompt)
                    return answer
        if poll:
            for _ in range(1 if choices else 60):
                for hook in hooks:
                    hook(prompt)
                result = poll()
                if result is not None:
                    return result
            assert choices, f'never finished: {prompt}'
        for hook in hooks:
            hook(prompt)
        return 'y' if 'y' in choices else ''

    def all(self):
        return '\n'.join(self.text)


def guide(robot, answers=None, **hooks):
    """A guide and a person who powers up, plugs in and turns horns when asked."""
    actions = {'once a servo answers': lambda p: (robot.power_on(), robot.plug_next()),
               'once it answers': robot.plug_next, 'horn now': robot.turn_named,
               'every horn is in range': robot.turn_into_range,
               'every servo is back': robot.power_cycle, **hooks}
    person = FakeUI(answers=answers, hooks=actions)
    return ac.Guide(robot, person), person


class ConstantTests(unittest.TestCase):
    def test_targets_travel_and_drive_modes_match_the_firmware(self):
        assembly = (SRC / 'squro_assembly.h').read_text()
        setup = (SRC / 'servo_setup.h').read_text()

        def array(source, name):
            return tuple(int(v) for v in re.search(rf'{name}\[\] = \{{([^}}]*)\}}', source).group(1).split(','))

        def value(source, name):
            return int(re.search(rf'\b{name} = (-?\d+)', source).group(1))

        self.assertEqual(array(assembly, 'FRONT_TARGETS'), ac.FRONT_TARGETS)
        self.assertEqual(array(assembly, 'REAR_TARGETS'), ac.REAR_TARGETS)
        self.assertEqual(value(assembly, 'HEAD_TARGET'), ac.HEAD_TARGET)
        self.assertEqual((value(assembly, 'FRONT_TRAVEL'), value(assembly, 'REAR_TRAVEL'),
                          value(assembly, 'HEAD_TRAVEL')), (ac.FRONT_TRAVEL, ac.REAR_TRAVEL, ac.HEAD_TRAVEL))
        self.assertEqual(array(setup, 'DRIVES'), tuple(ac.DRIVES[i] for i in range(11, 20)))
        self.assertEqual(array(setup, 'OFFSETS'), tuple(ac.OFFSETS[i] for i in range(11, 20)))
        for servo, place in ac.PLACES.items():          # Every target in its power-up window.
            self.assertTrue(ac.WINDOWS[servo][0] <= place.target <= ac.WINDOWS[servo][1], servo)
        self.assertEqual(value(setup, 'TEMP_SHIFT'), ac.TEMP_SHIFT)

    def test_cranks_are_described_as_seen_from_the_side(self):
        self.assertEqual(ac.pointing(4695), 'FORWARD (toward the nose) and DOWN, 53° below horizontal')
        self.assertEqual(ac.pointing(5564), 'BACKWARD (toward the tail) and DOWN, 51° below horizontal')
        self.assertEqual(ac.pointing(3627), 'FORWARD (toward the nose) and UP, 41° above horizontal')
        self.assertEqual(ac.pointing(4745), 'FORWARD (toward the nose) and DOWN, 57° below horizontal')

    def test_left_legs_take_n_parts_and_hind_legs_the_pointed_foot(self):
        legs = {leg.key: leg for leg in ac.LEGS}
        self.assertIn('SQ3_ElbowCrank_N on servo 11', ac.crank_steps(legs['FL'])[0])
        self.assertIn('SQ3_ShoulderCrank_P on servo 14', ac.crank_steps(legs['FR'])[1])
        self.assertIn('SQ3_UlnaFoot_N (round pad)', ac.parts(legs['FL']))
        self.assertIn('SQ4_UlnaFootHind_P (pointed toe)', ac.parts(legs['BR']))
        self.assertIn('Femur SQ3_Humerus_N', ac.linkage_steps(legs['BL'])[0])

    def test_firmware_lines_are_parsed(self):
        self.assertEqual(ac.parse_fields('SCAN 11=ok 12=-- 13=bad new=1 temp=23,25', 'SCAN'),
                         {11: 'ok', 12: '--', 13: 'bad', 'new': '1', 'temp': '23,25'})
        self.assertEqual(ac.parse_fields('POS 11=4695 19=--', 'POS'), {11: '4695', 19: '--'})

    def test_manual_console_only_sends_known_commands(self):
        for good in ('SCAN', 'SETUP 11', 'WIGGLE 19', 'MOVEID 13 21', 'ALIGNBR', 'RELEASE', '!'):
            self.assertTrue(ac.MANUAL_COMMANDS.match(good), good)
        for bad in ('SETUP 10', 'SETUP 20', 'WIGGLE', 'MOVEID 13 20', 'MOVEID 13', 'ALIGNALL',
                    'setup 11', 'SCAN; RELEASE'):
            self.assertFalse(ac.MANUAL_COMMANDS.match(bad), bad)


class ServoTests(unittest.TestCase):
    def test_a_fresh_build_from_factory_servos_to_four_legs(self):
        robot = FakeRobot(layout={}, powered=False)
        g, person = guide(robot)
        self.assertTrue(g.run())
        expected = [f'SETUP {servo}' for servo in range(11, 20)] + ['CENTERHEAD', 'RELEASE']
        for command in ('ALIGNFL', 'ALIGNFR', 'ALIGNBL', 'ALIGNBR'):
            expected += [command, 'RELEASE']
        self.assertEqual(robot.moves(), expected)
        self.assertEqual(robot.ids_by_place(), IDENTITY)
        text = person.all()
        self.assertIn('Switch the battery on', text)
        self.assertIn('Plug in the servo for the front-left leg pair, the servo nearer the middle', text)
        self.assertIn('so the TOP of the horn moves toward the NOSE', text)
        self.assertIn('Every servo sits where its ID belongs', text)
        self.assertIn('Step 8/8: Final check', text)
        self.assertIn('Left and right legs agree', text)
        self.assertIn('Assembly complete', text)

    def test_reused_servos_swapped_left_and_right_are_renumbered(self):
        robot = FakeRobot(layout=ROLLED)
        g, person = guide(robot)
        self.assertTrue(g.run('servos'))
        self.assertEqual(robot.ids_by_place(), IDENTITY)
        moves = robot.moves()
        # Away to the temporary ID of the destination, then home, then settings.
        self.assertEqual(moves[:2], ['MOVEID 13 21', 'MOVEID 14 22'])
        self.assertIn('MOVEID 21 11', moves)
        self.assertLess(moves.index('MOVEID 21 11'), moves.index('SETUP 11'))
        self.assertNotIn('SETUP 19', moves)                     # The head was already right.
        text = person.all()
        self.assertIn('front-left, nose end: ID 13 becomes ID 11', text)
        # A left servo carrying a right ID reads reversed; that is expected, not a fault.
        self.assertNotIn('read as the TOP of the horn moving toward the TAIL', text)
        self.assertNotIn('Plug in', text)

    def test_reused_servos_swapped_front_and_back_are_renumbered(self):
        robot = FakeRobot(layout=PITCHED)
        g, person = guide(robot)
        self.assertTrue(g.run('servos'))
        self.assertEqual(robot.ids_by_place(), IDENTITY)
        self.assertIn('front-left, nose end: ID 16 becomes ID 11', person.all())

    def test_servos_already_in_place_are_only_checked(self):
        robot = FakeRobot()
        g, person = guide(robot)
        g.do_servos()
        self.assertEqual(robot.moves(), [])
        self.assertIn('Every servo sits where its ID belongs', person.all())

    def test_stale_settings_are_corrected(self):
        robot = FakeRobot()
        robot.servos[13]['state'] = 'bad'
        g, _ = guide(robot)
        g.do_servos()
        self.assertEqual(robot.moves(), ['SETUP 13'])

    def test_an_interrupted_renumbering_is_finished_first(self):
        layout = {**IDENTITY, 11: 21}          # Stopped between the two renumbering passes.
        robot = FakeRobot(layout=layout)
        g, _ = guide(robot)
        g.do_servos()
        self.assertEqual(robot.moves()[:2], ['MOVEID 21 11', 'SETUP 11'])
        self.assertEqual(robot.ids_by_place(), IDENTITY)

    def test_a_horn_turned_toward_the_tail_is_turned_again(self):
        robot = FakeRobot()
        robot.wrong_way = {14}
        g, person = guide(robot)
        g.do_servos()
        self.assertIn('read as the TOP of the horn moving toward the TAIL', person.all())
        self.assertEqual(robot.ids_by_place(), IDENTITY)

    def test_a_found_horn_still_moving_does_not_block_the_next(self):
        # The bench layout of 2026-11-11: ID 11 in place, the rest mixed up,
        # each horn turned on by 125 ticks (11°) after it was found.
        layout = {11: 11, 12: 15, 13: 18, 14: 17, 15: 12, 16: 16, 17: 13, 18: 14, 19: 19}
        robot = FakeRobot(layout=layout)
        robot.settle = 125
        g, person = guide(robot)
        g.do_servos()
        self.assertEqual(robot.ids_by_place(), IDENTITY)
        self.assertIn('front-left, middle: ID 15 becomes ID 12', person.all())

    def test_another_horn_bumped_on_the_way_still_resolves(self):
        robot = FakeRobot()
        bumped = {'done': False}

        def turn_and_bump(prompt):
            if 'front-left, middle horn' in prompt and not bumped['done']:
                robot.servos[13]['pos'] += 150          # A neighbour nudged by 13°...
                bumped['done'] = True
            robot.turn_named(prompt)                    # ...while this one keeps turning.

        g, person = guide(robot, **{'horn now': turn_and_bump})
        g.do_servos()
        self.assertIn('Found the front-left, middle servo.', person.all())
        self.assertEqual(robot.ids_by_place(), IDENTITY)

    def test_lost_replies_while_renumbering_are_checked_not_fatal(self):
        # The bench on 2026-11-11: five servos parked on temporary IDs, and the
        # reply to SETUP 13 held back by the USB link.
        layout = {11: 11, 12: 12, 13: 23, 14: 24, 15: 25, 16: 16, 17: 27, 18: 28, 19: 19}
        robot = FakeRobot(layout=layout)
        robot.lost = {'SETUP 13', 'MOVEID 24 14'}
        g, person = guide(robot)
        g.do_servos()
        self.assertEqual(robot.ids_by_place(), IDENTITY)
        self.assertNotIn('did not complete', person.all())

    def test_a_move_that_really_failed_still_stops(self):
        robot = FakeRobot(layout={**IDENTITY, 13: 23})
        robot.servos[14]['id'] = 13          # ID 13 taken: the move cannot happen.
        robot.lost = {'MOVEID 23 13'}
        g, person = guide(robot)
        with self.assertRaises(ac.Quit):
            g.do_servos()
        self.assertIn('Renumbering ID 23 to 13 did not complete', person.all())

    def test_readings_wound_up_by_hand_are_reset_by_a_power_cycle(self):
        # The bench after the locating runs: ID 11 at 14104, ID 12 at 16143.
        robot = FakeRobot(positions={11: 14104, 12: 16143})
        g, person = guide(robot)
        g.do_servos()
        self.assertIn('wound some readings on by whole turns (ID 11, 12)', person.all())
        self.assertIn('Readings reset.', person.all())
        for servo in robot.servos.values():
            self.assertTrue(ac.WINDOWS[servo['id']][0] <= servo['pos'] <= ac.WINDOWS[servo['id']][1])

    def test_unwound_readings_need_no_power_cycle(self):
        robot = FakeRobot()
        g, person = guide(robot)
        g.do_servos()
        self.assertNotIn('battery off', person.all())

    def test_a_late_position_reply_is_retried(self):
        robot = FakeRobot()
        robot.lost_pos = 2
        g, person = guide(robot)
        g.do_servos()
        self.assertIn('Every servo sits where its ID belongs', person.all())

    def test_without_the_firmware_nothing_moves(self):
        robot = FakeRobot(answers=False)
        g, person = guide(robot)
        self.assertFalse(g.run())
        self.assertIn('pio run -e assemble_squro -t upload', person.all())
        self.assertEqual(robot.moves(), [])


class LegTests(unittest.TestCase):
    def test_a_horn_too_far_round_is_turned_by_hand_first(self):
        robot = FakeRobot(positions={11: 2100, 12: 7900})
        g, person = guide(robot)
        self.assertTrue(g.run('FL'))
        text = person.all()
        self.assertRegex(text, r'Servo 11 \(front-left, nose end\).*TOP of the horn moves toward the NOSE')
        self.assertRegex(text, r'Servo 12 .*TOP of the horn moves toward the TAIL')
        self.assertEqual(robot.moves()[:2], ['ALIGNFL', 'RELEASE'])  # Never refused.
        self.assertIn('SQ3_ElbowCrank_N on servo 11', text)
        self.assertIn('FORWARD (toward the nose) and DOWN, 53° below horizontal', text)

    def test_a_hold_lost_while_fitting_is_redone(self):
        robot = FakeRobot()
        robot.stops = 1
        g, person = guide(robot, answers={'screwed down': ['poll', '']})
        g.run('BL')
        self.assertEqual(robot.moves()[:4], ['ALIGNBL', 'RELEASE', 'ALIGNBL', 'RELEASE'])
        self.assertIn('The servos let go before you finished', person.all())

    def test_stopping_names_the_step_to_resume(self):
        robot = FakeRobot()
        g, person = guide(robot, answers={'Horns of servos 15': ['q']})
        self.assertFalse(g.run('BL'))
        self.assertIn('--start BL', person.all())
        self.assertNotIn('ALIGNBL', robot.sent)

    def test_the_final_check_flags_a_crank_a_screw_hole_off(self):
        robot = FakeRobot(positions={13: 4695 + 1024})
        g, person = guide(robot)
        g.run('check')
        self.assertIn('probably a screw hole off', person.all())


if __name__ == '__main__':
    unittest.main()
