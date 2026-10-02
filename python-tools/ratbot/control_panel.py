"""Keyboard/joystick reference and the selected gait's illustration."""
from pathlib import Path
import pygame
import eth_theme as eth
from calibration_panel import CalibrationPanel
from control_config import KEYBOARD_MAPPING
from leg_designs import DEFAULT_DESIGN, design_for
from pose_preview import DESCRIPTIONS, PREVIEW_SIZE, make_preview, preview_key

WINDOW_SIZE = (1280, 800)


class ControlPanel:
    def __init__(self, assets_dir, use_joystick=False, joystick_mapping=None,
                 design_key=DEFAULT_DESIGN):
        self.fonts = {size: pygame.font.Font(None, size) for size in (18, 20, 23, 26, 32)}
        self.assets_root = Path(assets_dir)
        self._sets = {}
        self.design_key = design_key
        self.previews = self._load(design_key)
        self.calibration_view = CalibrationPanel()
        self.controls = self._make_controls(use_joystick, joystick_mapping)
        self.active_preview = None

    def _load(self, design_key):
        """Pictures for one leg design, cached so a swap back is instant."""
        if design_key in self._sets:
            return self._sets[design_key]
        design = design_for(design_key)
        # Designs keep their pictures in their own folder. Older checkouts put
        # the five-bar set straight in docs/poses, so fall back to that.
        folders = [self.assets_root / design.asset_dir, self.assets_root]
        pictures = {}
        for name in DESCRIPTIONS:
            picture = None
            for folder in folders:
                try:
                    picture = pygame.image.load(str(folder / f'{name.lower()}.png'))
                    break
                except (OSError, pygame.error):
                    continue
            if picture is None:
                # Source checkouts and packaged apps both remain usable if an
                # asset is missing; drawing it costs a moment instead.
                picture = make_preview(name, design)
            pictures[name] = pygame.transform.smoothscale(picture, PREVIEW_SIZE)
        self._sets[design_key] = pictures
        return pictures

    def set_design(self, design_key):
        """Show the fitted design's pictures from now on."""
        self.design_key = design_key
        self.previews = self._load(design_key)

    def _text(self, surface, text, size, color, xy):
        surface.blit(self.fonts[size].render(text, True, color), xy)

    def _make_controls(self, joystick, mapping):
        panel = pygame.Surface((398, 564))
        panel.fill(eth.PANEL)
        self._text(panel, 'CONTROLS', 20, eth.ACCENT, (24, 24))
        self._text(panel, 'Joystick + keyboard' if joystick else 'Keyboard', 32, eth.TEXT_PRIMARY, (24, 53))
        if joystick:
            rows = [('Stick', 'Move / turn')]
        else:
            move = KEYBOARD_MAPPING['movement']
            keys = lambda names: '/'.join(pygame.key.name(move[n]).upper() for n in names)
            rows = [(keys(('forward', 'left', 'backward', 'right')), 'Move / turn'),
                    (keys(('forward_left', 'forward_right')), 'Forward with turn')]
        actions = [
            ('sit', 'Sit down / stand up'), ('rear', 'Rear up / lower'),
            ('switch_gait', 'Switch gait'), ('change_legs', 'Change legs'),
            ('reset', 'Reset gait'), ('greet', 'Greet'), ('jump', 'Jump'),
            ('battery', 'Battery level'), ('show_range', 'Show range'),
            ('record', 'Record movement'), ('exit', 'Exit / release torque'),
        ]
        for action, description in actions:
            if joystick and action not in ('sit', 'rear', 'exit'):
                button = (mapping or {}).get('actions', {}).get(action)
                if button is None:
                    continue
                key = f'Btn {button}'
            else:
                key = pygame.key.name(KEYBOARD_MAPPING['actions'][action]).upper()
                if key == 'ESCAPE':
                    key = 'ESC'
            rows.append((key, description))
        for index, (key, description) in enumerate(rows):
            y = 109 + index * 32
            pose_row = description in ('Sit down / stand up', 'Rear up / lower')
            color = eth.POSE_ACCENT if pose_row else eth.TEXT_SECONDARY
            pygame.draw.rect(panel, eth.CHIP, (20, y - 4, 115, 29), border_radius=5)
            self._text(panel, key, 20, color, (29, y + 2))
            self._text(panel, description, 23, color, (147, y + 1))
        self._text(panel, 'Click this window to use keyboard controls.', 18, eth.TEXT_MUTED, (24, 537))
        return panel

    def draw(self, window, gait, pose, calibration=None):
        if calibration is not None and calibration.active:
            return self._draw_calibration(window, calibration)
        name = preview_key(gait, pose.active, pose.POSE_NAME)
        self.active_preview = name
        window.blit(self.controls, (20, 158))
        window.blit(self.previews[name], (440, 158))
        label = f'Return gait: {DESCRIPTIONS[gait][0]}' if pose.active else f'Selected gait: {DESCRIPTIONS[gait][0]}'
        self._text(window, label, 23, eth.TEXT_SECONDARY, (444, 652))
        # Selection chips are indicators, matching the P and G keyboard controls.
        for index, key in enumerate(DESCRIPTIONS):
            x, y = 440 + (index % 5) * 161, 680 + (index // 5) * 32
            selected = key == name
            # Solid ETH Blue (bronze for poses) carries white text when selected;
            # the ramp colour stays visible as an edge bar in both states.
            pygame.draw.rect(window, eth.CHIP_ACTIVE[key] if selected else eth.CHIP_DIM,
                             (x, y, 153, 26), border_radius=5)
            pygame.draw.rect(window, eth.GAIT_COLORS[key], (x, y, 4, 26),
                             border_top_left_radius=5, border_bottom_left_radius=5)
            self._text(window, DESCRIPTIONS[key][0], 20,
                       eth.TEXT_ON_ACCENT if selected else eth.TEXT_MUTED, (x + 14, y + 6))
        self._status(window, pose.label())

    def _draw_calibration(self, window, calibration):
        self.active_preview = calibration.POSE_NAME
        window.blit(self.controls, (20, 158))
        window.blit(self.calibration_view.render(calibration), (440, 158))
        self._status(window, calibration.label())

    def _status(self, window, text):
        pygame.draw.rect(window, eth.STATUS_BAR, (0, 754, 1280, 46))
        self._text(window, text, 26, eth.TEXT_ON_ACCENT, (24, 767))
