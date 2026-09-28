"""Generate gait illustrations from Q8bot kinematics, with no hardware access."""
import math
from pathlib import Path

import pygame
import eth_theme as eth
from gait_manager import GAITS, GaitManager
from kinematics_solver import k_solver
from sitting_pose import SittingPose
from rearing_pose import RearingPose

PREVIEW_SIZE = (800, 480)
DESCRIPTIONS = {
    'TROT': ('Trot', 'Diagonal leg pairs move together.'),
    'TROT_HIGH': ('High trot', 'A taller stance with a shorter stride.'),
    'TROT_LOW': ('Low trot', 'A crouched stance with a shorter stride.'),
    'TROT_FAST': ('Fast trot', 'A longer stride and a shorter gait cycle.'),
    'WALK': ('Walk', 'Four evenly spaced leg phases.'),
    'CRAWL': ('Crawl', 'Legs lift in sequence with short steps.'),
    'BOUND': ('Bound', 'Front and rear pairs move in turn.'),
    'PRONK': ('Pronk', 'All four legs move together.'),
    'SITTING': ('Sitting', 'Front extended. Rear crouched. Hold this pose.'),
    'REARING': ('Rearing', 'Rear support stance. Forelegs held still.'),
}
COLORS = eth.GAIT_COLORS


def preview_key(gait, active, pose_name='SITTING'):
    return pose_name if active else gait


def make_preview(name):
    """A static illustration of a representative gait phase or seated posture.

    Limb geometry uses the real solver; chassis spacing and perspective are
    illustrative. This is a preview of commanded positions, not live telemetry.
    """
    if not pygame.font.get_init():
        pygame.font.init()
    leg = k_solver()
    if name == 'SITTING':
        heights = [SittingPose.FRONT_HEIGHT] * 2 + [SittingPose.REAR_HEIGHT] * 2
        positions = []
        for height in heights:
            q1, q2, valid = leg.ik_solve(SittingPose.FOOT_X, height)
            if not valid:
                raise ValueError('Sitting preview is unreachable')
            positions.extend((q1, q2))
    elif name == 'REARING':
        positions = RearingPose.target_positions()
        heights = [0] * 4
    else:
        manager = GaitManager(leg)
        if not manager.load_gait(name):
            raise ValueError(f'Cannot generate preview for {name}')
        trajectory = manager.current_trajectories[name]['f']
        # Sample halfway through the first swing segment, away from an endpoint.
        positions = trajectory[max(1, GAITS[name][6] // 2) % len(trajectory)]
        heights = [GAITS[name][2]] * 4

    # Render at 2x then downsample for smooth diagonal limbs and text.
    scale = 2
    surface = pygame.Surface((PREVIEW_SIZE[0] * scale, PREVIEW_SIZE[1] * scale))
    surface.fill(eth.PANEL)
    accent = pygame.Color(COLORS[name])
    far_accent = eth.mix(COLORS[name], eth.PANEL, .45)
    far_shank = eth.mix(eth.SHANK, eth.PANEL, .38)
    def point(p):
        return tuple(round(v * scale) for v in p)
    def text(value, size, color, xy):
        surface.blit(pygame.font.Font(None, size * scale).render(value, True, color), point(xy))
    def line(color, a, b, width=2):
        pygame.draw.line(surface, color, point(a), point(b), width * scale)
    def circle(color, p, radius):
        pygame.draw.circle(surface, color, point(p), radius * scale)
    def poly(color, points):
        pygame.draw.polygon(surface, color, [point(p) for p in points])
    def project(x, side, height):
        if name == 'REARING':
            return 400 + x * 1.65 + side * 1.05, 340 + side * .5 - height * 1.45
        return 400 + x * 2.6 + side * 1.05, 340 + side * .5 - height * 2.75

    # A raised chassis angle illustrates rearing; it is not a predicted balance
    # angle. Actual body tilt depends on load, contacts and joint compliance.
    pitch = math.radians(62)
    if name == 'REARING':
        rear_x, rear_y = leg.fk_solve(*positions[4:6])
        root_height = rear_y * math.cos(pitch) - (rear_x - leg.d / 2) * math.sin(pitch)
    def body_project(x, side, height):
        if name != 'REARING':
            return project(x, side, height)
        relative_x = x + 53
        return project(relative_x * math.cos(pitch) - height * math.sin(pitch) - 40,
                       side, relative_x * math.sin(pitch) + height * math.cos(pitch) + root_height)

    title, description = DESCRIPTIONS[name]
    text('STATIC POSE' if name in ('SITTING', 'REARING') else 'GAIT PREVIEW', 18, accent, (28, 23))
    text(title, 44, eth.TEXT_PRIMARY, (28, 47))
    text(description, 23, eth.TEXT_SECONDARY, (28, 95))
    # A ground grid supplies context for leg clearance and body tilt.
    for x in range(-100, 101, 25):
        line(eth.GRID, project(x, -58, 0), project(x, 58, 0), 1)
    for side in range(-50, 51, 25):
        line(eth.GRID, project(-105, side, 0), project(105, side, 0), 1)

    # FL, FR, BL, BR; far side is drawn first, chassis next, near side last.
    anchors = [(53, 27), (53, -27), (-53, 27), (-53, -27)]
    def draw_leg(index, far=False):
        body_x, side = anchors[index]
        height = heights[index]
        q1, q2 = [math.radians(float(q)) for q in positions[index * 2:index * 2 + 2]]
        foot_x, foot_y = leg.fk_solve(math.degrees(q1), math.degrees(q2))
        def local(x, y):
            return body_project(body_x + x - leg.d / 2, side, height - y)
        hips = [local(leg.d, 0), local(0, 0)]
        knees = [local(leg.d + leg.l1 * math.cos(q1), leg.l1 * math.sin(q1)),
                 local(leg.l1p * math.cos(q2), leg.l1p * math.sin(q2))]
        foot = local(foot_x, foot_y)
        rod = far_accent if far else accent
        for hip, knee in zip(hips, knees):
            line(eth.OUTLINE, hip, knee, 11)
            line(rod, hip, knee, 7)
            line(eth.OUTLINE, knee, foot, 9)
            line(far_shank if far else eth.SHANK, knee, foot, 5)
            circle(eth.OUTLINE, knee, 5)
            circle(rod, knee, 3)
            circle(eth.CHASSIS_FILL, hip, 9)
            circle(eth.JOINT_CORE, hip, 4)
        circle(eth.OUTLINE, foot, 7)
        circle(rod, foot, 4)
        if not far:
            label_xy = (foot[0] - 23, 387)
            if name == 'REARING':
                label_xy = (foot[0] + 10, foot[1] - 4) if index == 0 else (foot[0] - 23, foot[1] + 17)
            text('FRONT' if index == 0 else 'REAR', 17, eth.TEXT_MUTED, label_xy)
    draw_leg(3, True)
    draw_leg(1, True)
    corners = [body_project(-69, -24, heights[2]), body_project(69, -24, heights[0]),
               body_project(69, 24, heights[0]), body_project(-69, 24, heights[2])]
    poly(eth.CHASSIS_SHADOW, [(x, y + 13) for x, y in corners])
    poly(eth.CHASSIS_FILL, corners)
    pygame.draw.lines(surface, eth.CHASSIS_EDGE, True, [point(p) for p in corners], 2 * scale)
    # Two battery shapes on the chassis provide a recognizable robot silhouette.
    for side in (-11, 10):
        line(eth.BATTERY, body_project(-36, side, heights[2] * .75 + heights[0] * .25 + 3),
             body_project(36, side, heights[2] * .25 + heights[0] * .75 + 3), 10)
    draw_leg(2)
    draw_leg(0)
    line(accent, (686, 176), (725, 176), 2)
    poly(accent, [(725, 176), (716, 171), (716, 181)])
    text('FRONT', 17, accent, (681, 151))
    line(eth.PANEL_RULE, (28, 420), (772, 420), 1)
    if name == 'SITTING':
        detail = f'Front {SittingPose.FRONT_HEIGHT:g} mm  /  Rear {SittingPose.REAR_HEIGHT:g} mm'
    elif name == 'REARING':
        detail = f'Rear joint offset {RearingPose.REARWARD_ANGLE_OFFSET:g} deg  /  No waving'
    else:
        params = GAITS[name]
        detail = f'Rest height {params[2]:g} mm  /  Stride span {params[3]:g} mm'
    text(detail, 22, eth.TEXT_SECONDARY, (28, 437))
    text('Kinematic illustration', 18, eth.TEXT_MUTED, (615, 439))
    return pygame.transform.smoothscale(surface, PREVIEW_SIZE)


def generate_previews(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for name in DESCRIPTIONS:
        pygame.image.save(make_preview(name), str(directory / f'{name.lower()}.png'))


if __name__ == '__main__':
    generate_previews(Path(__file__).resolve().parent.parent / 'docs' / 'poses')
    print(f'Generated {len(DESCRIPTIONS)} gait/pose pictures.')
