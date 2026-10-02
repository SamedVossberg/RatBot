"""Hardware-free checks for turning the head with the arrow keys."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import runpy
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'ratbot'))
import pygame
from control_panel import ControlPanel, WINDOW_SIZE
from espnow import HEAD_LIMIT_DEG, MAX_COMMAND_LENGTH, q8_espnow
from head_control import HeadControl
from input_handler import InputHandler
from kinematics_solver import k_solver
from leg_calibration import PICK
from leg_designs import selectable
from rearing_pose import RearingPose
from sitting_pose import SittingPose


class HeadControlTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.sent = []
        self.robot = Mock()
        self.robot.move_head.side_effect = lambda deg: self.sent.append((self.now, deg)) or True
        self.head = HeadControl(self.robot, lambda: self.now)

    def hold(self, direction, seconds, frame=0.005):
        end = self.now + seconds
        while self.now < end:
            self.head.update(direction)
            self.now += frame

    def test_limit_is_90_deg_either_side(self):
        self.assertEqual(HeadControl.LIMIT_DEG, 90.0)
        self.assertEqual(HEAD_LIMIT_DEG, 90.0)

    def test_holding_left_turns_at_90_deg_per_second_up_to_the_limit(self):
        self.hold(1, 0.5)
        self.assertGreaterEqual(self.head.angle, 43.2)
        self.assertLessEqual(self.head.angle, 46.8)
        self.hold(1, 0.7)
        self.assertEqual(self.head.angle, 90.0)
        calls = len(self.sent)
        self.hold(1, 1.0)
        self.assertEqual(self.head.angle, 90.0)
        self.assertEqual(len(self.sent), calls, 'nothing is sent while held at the limit')
        self.assertIn('limit', self.head.readout())

    def test_right_mirrors_left(self):
        self.hold(-1, 1.2)
        self.assertEqual(self.head.angle, -90.0)
        self.assertEqual(self.sent[-1][1], -90.0)
        self.assertEqual(self.head.readout(), '90 deg right, limit')

    def test_release_holds_and_the_next_press_steps_at_once(self):
        self.hold(1, 0.2)
        angle = self.head.angle
        calls = len(self.sent)
        self.hold(0, 1.0)
        self.assertEqual(self.head.angle, angle)
        self.assertEqual(len(self.sent), calls)
        self.head.update(-1)
        self.assertEqual(self.head.angle, round(angle - 3.6, 1))

    def test_at_most_one_command_per_period_however_fast_the_loop_runs(self):
        self.hold(1, 0.5, frame=0.001)
        times = [t for t, _ in self.sent]
        for a, b in zip(times, times[1:]):
            self.assertGreaterEqual(b - a, HeadControl.PERIOD_S - 1e-9)
        # Angles stay on a 0.1 deg grid, so repeated steps do not drift.
        for _, deg in self.sent:
            self.assertEqual(deg, round(deg, 1))

    def test_a_stalled_loop_does_not_catch_up(self):
        self.head.update(1)
        self.now += 2.0  # e.g. a jump blocks the GUI loop
        self.head.update(1)
        self.head.update(1)
        self.assertEqual(self.head.angle, 7.2)

    def test_failed_send_keeps_the_angle(self):
        self.robot.move_head.side_effect = None
        self.robot.move_head.return_value = False
        self.head.update(1)
        self.assertEqual(self.head.angle, 0.0)
        self.assertEqual(self.head.readout(), 'centre')


class HeadPacketTests(unittest.TestCase):
    """What q8_espnow writes to the controller dongle."""

    def setUp(self):
        patcher = patch('serial.Serial')
        self.port = patcher.start().return_value
        self.addCleanup(patcher.stop)
        self.robot = q8_espnow('mock-port')

    def written(self):
        return [call.args[0].decode() for call in self.port.write.call_args_list]

    def fields(self):
        packet = self.written()[-1]
        self.assertTrue(packet.endswith(';'))
        return packet[:-1].split(',')

    def test_torque_on_and_every_joint_command_carry_the_head_angle(self):
        self.robot.enable_torque()
        self.assertEqual(self.written()[-1], '0,0,0,0,0,0,0,0,0,0,1,0.0;')
        self.robot.move_all([0, 90] * 4, 500, False)
        self.assertEqual(self.fields()[8:], ['0', '500', '1', '0.0'])

    def test_head_command_repeats_the_last_joint_targets_and_timing(self):
        self.robot.enable_torque()
        positions = [45.879, 134.121] * 4
        self.robot.move_all(positions, 1000, True)
        self.assertTrue(self.robot.move_head(-45))
        fields = self.fields()
        self.assertEqual([float(f) for f in fields[:8]], positions)
        # Not recorded again, same profile, torque still on, head angle last.
        self.assertEqual(fields[8:], ['0', '1000', '1', '-45.0'])

    def test_head_angle_is_clamped_to_90_deg(self):
        self.robot.move_all([0, 90] * 4, 0, False)
        self.robot.move_head(135)
        self.assertEqual(self.fields()[11], '90.0')
        self.robot.move_head(-200)
        self.assertEqual(self.fields()[11], '-90.0')
        self.assertEqual(self.robot.head_deg, -HEAD_LIMIT_DEG)

    def test_before_any_joint_command_the_angle_waits_for_the_first_one(self):
        self.assertTrue(self.robot.move_head(20))
        self.port.write.assert_not_called()
        self.robot.move_all([0, 90] * 4, 0, False)
        self.assertEqual(self.fields()[11], '20.0')

    def test_overlong_commands_are_refused_and_the_head_angle_kept(self):
        self.assertFalse(self.robot.move_all([123.456789012] * 8, 0, False))
        self.port.write.assert_not_called()
        self.robot.move_all([0, 90] * 4, 0, False)
        length = len(self.written()[-1]) - 1
        with patch('espnow.MAX_COMMAND_LENGTH', length):
            self.assertFalse(self.robot.move_head(-90))
        self.assertEqual(self.robot.head_deg, 0.0)

    def test_every_leg_design_keeps_its_pose_and_fits_the_relay(self):
        for design in selectable():
            leg = design.solver()
            sitting = SittingPose(leg, Mock(), targets=design.sitting)
            poses = [design.mount_positions(),
                     RearingPose.target_positions(design.rearing),
                     sitting._pair(design.sitting[0], design.sitting[1]) * 4,
                     list(design.rearing[0]), list(design.rearing[1])]
            for positions in poses:
                self.robot.move_all(positions, 2500, True)
                self.assertTrue(self.robot.move_head(-90), (design.key, positions))
                packet = self.written()[-1]
                self.assertLessEqual(len(packet) - 1, MAX_COMMAND_LENGTH)
                self.assertEqual([float(f) for f in self.fields()[:8]], list(map(float, positions)))


class Keys:
    def __init__(self, *held):
        self.held = set(held)

    def __getitem__(self, key):
        return key in self.held


class HeadInputTests(unittest.TestCase):
    def test_arrow_keys_in_keyboard_and_joystick_modes(self):
        for handler in (InputHandler(), InputHandler(True, Mock(), {'actions': {}})):
            for held, direction in (((), 0), ((pygame.K_LEFT,), 1), ((pygame.K_RIGHT,), -1),
                                    ((pygame.K_LEFT, pygame.K_RIGHT), 0)):
                with patch('pygame.key.get_pressed', return_value=Keys(*held)):
                    self.assertEqual(handler.head_direction(), direction, held)

    def test_wasd_does_not_turn_the_head(self):
        with patch('pygame.key.get_pressed', return_value=Keys(pygame.K_a, pygame.K_d)):
            self.assertEqual(InputHandler().head_direction(), 0)


class HeadPanelTests(unittest.TestCase):
    def setUp(self):
        pygame.init()

    def test_turn_head_row_shows_the_live_angle_in_both_layouts(self):
        pose = SittingPose(k_solver(), Mock())
        for joystick in (False, True):
            panel = ControlPanel(ROOT / 'docs' / 'poses', joystick, {'actions': {'battery': 3}})
            self.assertIsNotNone(panel._head_row_y)
            row = pygame.Rect(20, 158 + panel._head_row_y - 4, 398, 27)
            head = HeadControl(Mock())
            shots = []
            for angle in (0.0, 36.0, -90.0):
                head.angle = angle
                window = pygame.Surface(WINDOW_SIZE)
                panel.draw(window, 'TROT', pose, None, head)
                shots.append(pygame.image.tobytes(window.subsurface(row), 'RGB'))
            self.assertEqual(len(set(shots)), 3)


class OperateHeadTests(unittest.TestCase):
    """The arrow keys through operate.py itself, including with the leg picker open."""

    def setUp(self):
        pygame.init()

    def tearDown(self):
        pygame.quit()

    def test_arrows_turn_the_head_in_every_state_and_leave_the_picker_to_up_down(self):
        robot = Mock()
        robot.move_all.return_value = True
        robot.move_head.return_value = True
        robot.serialHandler.in_waiting = 0
        frame = [0]
        cursors = {}
        original_draw = ControlPanel.draw

        def draw(panel, window, gait, pose, calibration=None, head=None):
            original_draw(panel, window, gait, pose, calibration, head)
            cursors[frame[0]] = (calibration.stage, calibration.cursor)

        def key(code):
            return [pygame.event.Event(pygame.KEYDOWN, key=code)]

        def events():
            frame[0] += 1
            return {4: key(pygame.K_l), 5: [pygame.event.Event(pygame.KEYUP, key=pygame.K_l)],
                    7: key(pygame.K_RIGHT), 8: key(pygame.K_DOWN),
                    10: key(pygame.K_ESCAPE), 12: key(pygame.K_ESCAPE)}.get(frame[0], [])

        def direction(_handler):
            return {2: 1, 7: -1}.get(frame[0], 0)

        with patch('espnow.q8_espnow', return_value=robot), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch.object(ControlPanel, 'draw', draw), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch.object(InputHandler, 'head_direction', direction), \
             patch.object(InputHandler, 'is_movement_input', return_value=False), \
             patch.object(InputHandler, 'is_action_pressed', return_value=False), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port']):
            runpy.run_path(str(ROOT / 'ratbot' / 'operate.py'), run_name='__main__')

        turns = [call.args[0] for call in robot.move_head.call_args_list]
        # Left in normal operation, then Right while the picker is open.
        self.assertEqual(turns, [3.6, 0.0])
        self.assertEqual(cursors[6], (PICK, 0))
        self.assertEqual(cursors[7], (PICK, 0), 'Right must not move the picker cursor')
        self.assertEqual(cursors[8], (PICK, 1), 'Down still moves it')
        robot.disable_torque.assert_called_once()


if __name__ == '__main__':
    unittest.main()
