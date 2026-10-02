"""Turns the head with the Left and Right arrow keys.

The head servo (ID 19) is not part of any leg design, so this works the same
whichever legs are fitted, and in every state of the GUI. Positive angles turn
the head to the left. Holding a key turns it at SPEED_DEG_S up to LIMIT_DEG
either side of centre; releasing it holds the head where it is.

The angle moves in steps of PERIOD_S, so a held key sends at most 25 head
commands a second however fast the GUI loop runs. The robot firmware gives each
step a move time that matches its size, which blends the steps into one turn.
"""

import time

from espnow import HEAD_LIMIT_DEG


class HeadControl:
    LIMIT_DEG = HEAD_LIMIT_DEG
    SPEED_DEG_S = 90.0  # The robot firmware limits head moves to the same rate.
    PERIOD_S = 0.04

    def __init__(self, robot, clock=time.monotonic):
        self.robot = robot
        self.clock = clock
        self.angle = 0.0
        self._next_step = None

    def update(self, direction):
        """Call every frame. direction: 1 turns left, -1 right, 0 holds.

        Returns True when a new angle was sent.
        """
        if not direction:
            self._next_step = None  # The next press steps at once.
            return False
        now = self.clock()
        if self._next_step is not None and now < self._next_step:
            return False
        # Keep a steady cadence, but do not catch up after a stalled frame.
        if self._next_step is None or now - self._next_step >= self.PERIOD_S:
            self._next_step = now
        self._next_step += self.PERIOD_S
        step = direction * self.SPEED_DEG_S * self.PERIOD_S
        target = round(max(-self.LIMIT_DEG, min(self.LIMIT_DEG, self.angle + step)), 1)
        if target == self.angle or not self.robot.move_head(target):
            return False
        self.angle = target
        return True

    def readout(self):
        """Short text for the GUI, such as 'centre' or '36 deg left'."""
        if abs(self.angle) < 0.5:
            return 'centre'
        side = 'left' if self.angle > 0 else 'right'
        limit = ', limit' if abs(self.angle) >= self.LIMIT_DEG else ''
        return f'{abs(self.angle):.0f} deg {side}{limit}'
