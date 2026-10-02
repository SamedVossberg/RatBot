"""Render GUI screenshots headless, with no ESP32 attached.

Draws the same frames operate.py's main loop draws, via a dummy SDL video
driver, and writes them next to this file.

    venv/bin/python docs/gui/render_screenshots.py
"""
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT / 'ratbot'))

import pygame
import eth_theme as eth
from control_panel import ControlPanel, WINDOW_SIZE
from head_control import HeadControl
from helpers import Q8Logger
from leg_calibration import CONFIRM, HOLD, LegCalibration
from rearing_pose import RearingPose
from sitting_pose import SittingPose

pygame.init()
window = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('RatBot - Gaits and poses')
Q8Logger.set_pygame_surface(window)
panel = ControlPanel(ROOT / 'docs' / 'poses')


class _Bench:
    """Stands in for the robot so every screen renders offline."""
    def move_all(self, *_args, **_kwargs):
        return True

    def move_head(self, _deg):
        return True


head = HeadControl(_Bench())


def frame(gait, pose, messages, name, calibration=None):
    Q8Logger._message_log = list(messages)
    window.fill(eth.BACKGROUND)
    panel.draw(window, gait, pose, calibration, head)
    Q8Logger.render_pygame_messages()
    pygame.display.flip()
    pygame.image.save(window, str(HERE / name))
    print(f'wrote {name} ({panel.active_preview})')


stamp = time.strftime('%H:%M:%S')
ready = (stamp, 'INFO', 'Controls ready. G: switch gait. P: sit / stand. U: rear / lower.')
idle = SittingPose(None, None)
sitting = SittingPose(None, None)
sitting.active = True
rearing = RearingPose(None, None)
rearing.active = True

frame('TROT', idle, [ready], 'gui_startup.png')
head.angle = 36.0
frame('WALK', idle, [ready, (stamp, 'INFO', 'Switched to WALK'),
                     (stamp, 'INFO', 'Battery: 87%')], 'gui_walk.png')
head.angle = 0.0
frame('TROT', sitting, [(stamp, 'INFO', 'Sitting pose')], 'gui_sitting.png')
frame('TROT', rearing, [(stamp, 'INFO', 'Rearing pose')], 'gui_rearing.png')


cal = LegCalibration(_Bench())
cal.open_picker()
cal.cursor = 2
frame('TROT', idle, [(stamp, 'INFO', 'Change legs')], 'gui_legs_picker.png', cal)
cal.choose()
frame('TROT', idle, [(stamp, 'INFO', 'Calibrating Biomimetic legs.')],
      'gui_legs_confirm.png', cal)
cal.confirm()
cal.stage = HOLD
frame('TROT', idle, [(stamp, 'INFO', 'Driving to the mounting pose. Keep hands clear.')],
      'gui_legs_hold.png', cal)
pygame.quit()
