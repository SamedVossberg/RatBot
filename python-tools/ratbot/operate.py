'''
Written by yufeng.wu0902@gmail.com

This is the latest control script for Q8bot (using ESPNow). Run this script 
in your laptop with an ESP32C3 connected and control the robot via keyboard.
'''

import time
import pygame
import sys
import argparse
import os
from kinematics_solver import k_solver
from espnow import q8_espnow
from helpers import XiaoPortFinder, Q8Logger
from gait_manager import GaitManager, GAITS
from routine_generator import show_range, greet
from input_handler import InputHandler, detect_and_init_joystick
from sitting_pose import SittingPose
from rearing_pose import RearingPose
from control_panel import ControlPanel, WINDOW_SIZE
import eth_theme as eth
from leg_calibration import LegCalibration, CONFIRM, FAILED, HOLD, PICK
from leg_designs import DEFAULT_DESIGN, design_for
from head_control import HeadControl

# Q8bot leg configuration
CENTER_DIST = 19.5  # Distance between two actuators
L1 = 25             # Upper leg length
L2 = 40             # Lower leg length

# Pygame config
SPEED = 200
res = 0.2

# Helper Functions
def move_xy(x, y, dur = 0, deg = True):
    """Move robot legs to specific x,y position."""
    q1, q2, success = leg.ik_solve(x, y, deg, 1)
    q8.move_mirror([q1, q2], dur)
    return success

# Parse command-line arguments
parser = argparse.ArgumentParser(description='RatBot control script')
parser.add_argument('com_port', nargs='?', help='COM port for ESP32C3 (optional, auto-detect if not provided)')
parser.add_argument('--debug', action='store_true', help='Enable debug logging')
args = parser.parse_args()

# Initialize logger
log = Q8Logger(debug=args.debug)

# Flags for main loop
movement = False
exit = False
record = False
request = "none"

# Find a serial port and connect
if args.com_port:
    # User provided a COM port
    com_port = args.com_port
    if not XiaoPortFinder.validate(com_port):
        log.error(f"{com_port} is not an ESP32C3 or does not exist.")
        sys.exit(1)
else:
    # Auto-detect COM port
    com_port = XiaoPortFinder.find()
    if com_port is None:
        log.error("No ESP32C3 controller device found.")
        sys.exit(1)

# Start pygame instance
pygame.init()
window = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('RatBot - Gaits and poses')
clock = pygame.time.Clock()

# Set up pygame surface for logger
Q8Logger.set_pygame_surface(window)

# Detect and initialize input device (joystick or keyboard)
use_joystick, joystick, joystick_mapping = detect_and_init_joystick()
input_handler = InputHandler(use_joystick, joystick, joystick_mapping)

# Resolve images for both source runs and PyInstaller executables.
def get_resource_path(relative_path):
    """Get absolute path to resource, works for dev and for PyInstaller"""
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        # Running in development mode
        base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

control_panel = ControlPanel(get_resource_path(os.path.join('docs', 'poses')),
                             use_joystick, joystick_mapping)
log.info('Controls ready. G: switch gait. P: sit / stand. U: rear / lower.')

# Initialize kinematics solver and Q8bot ESPNow instance. Every motion
# parameter below comes from the fitted leg design, so a swap changes the
# solver, the gait table and the two static poses together.
fitted_design = design_for(DEFAULT_DESIGN)
leg = fitted_design.solver()
q8 = q8_espnow(com_port)
sitting_pose = SittingPose(leg, q8, targets=fitted_design.sitting)
rearing_pose = RearingPose(leg, q8, waypoints=fitted_design.rearing)
q8.enable_torque()

# Initialize GaitManager
gait_names = list(fitted_design.gaits.keys())
gait_manager = GaitManager(leg, fitted_design.gaits)

# Starting location of leg end effector in x and y
first_gait_params = fitted_design.gaits[gait_names[0]]
pos_x = first_gait_params[1]
pos_y = first_gait_params[2]
move_xy(pos_x, pos_y, 1000)

# Pre-calculate trajectories for default gait
if not gait_manager.load_gait(gait_names[0]):
    log.error(f"Failed to load default gait: {gait_names[0]}")
    sys.exit(1)

# Leg-design picker and the leg-attachment calibration sequence.
calibration = LegCalibration(q8)
# Left/Right turn the head in every state, whichever legs are fitted.
head = HeadControl(q8)

def apply_leg_design(design):
    """Swap in a newly fitted design: solver, gait table, poses and pictures."""
    global fitted_design, leg, sitting_pose, rearing_pose, gait_manager, pos_x, pos_y
    leg = design.solver()
    sitting_pose = SittingPose(leg, q8, targets=design.sitting)
    rearing_pose = RearingPose(leg, q8, waypoints=design.rearing)
    gait_manager = GaitManager(leg, design.gaits)
    if not gait_manager.load_gait(gait_names[0]):
        log.error(f"{design.title}: could not generate gait {gait_names[0]}.")
        return
    fitted_design = design
    control_panel.set_design(design.key)
    pos_x, pos_y = design.gaits[gait_names[0]][1], design.gaits[gait_names[0]][2]
    move_xy(pos_x, pos_y, 1000)
    log.info(f"{design.title} legs fitted.")
    if design.key != DEFAULT_DESIGN:
        # Stride and stance are geometry-checked against this leg's workspace,
        # but they have not been tuned on the physical robot.
        log.warning(f"{design.title} gait stride and height are untuned.")

def advance_calibration():
    """Enter steps through the picker and then the calibration stages."""
    stage = calibration.stage
    if stage == PICK:
        if calibration.choose():
            log.info(f"Calibrating {calibration.chosen().title} legs.")
        else:
            log.info(calibration.error)
    elif stage in (CONFIRM, FAILED):
        if calibration.confirm():
            log.info("Driving to the mounting pose. Keep hands clear.")
        else:
            log.error(calibration.error)
    elif stage == HOLD:
        design = calibration.chosen()
        calibration.finish()
        apply_leg_design(design)

def calibration_keys(events):
    """Calibration owns Up/Down and Enter while it is open; Left/Right stay the head's."""
    for event in events:
        if event.type != pygame.KEYDOWN:
            continue
        if event.key == pygame.K_UP:
            calibration.move_cursor(-1)
        elif event.key == pygame.K_DOWN:
            calibration.move_cursor(1)
        elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            advance_calibration()

time.sleep(2)

while True:
    clock.tick(SPEED)
    events = pygame.event.get()
    escape_pressed = any(event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
                         for event in events)
    # While calibration is open Esc cancels it instead of quitting, so a leg
    # swap can be abandoned without dropping torque mid-sequence.
    if calibration.active and escape_pressed:
        calibration.cancel()
        log.info("Leg calibration cancelled.")
        escape_pressed = False
    if (any(event.type == pygame.QUIT for event in events) or escape_pressed
            or (not calibration.active and input_handler.is_action_pressed('exit'))):
        break

    calibration.update()
    if calibration.active:
        calibration_keys(events)
    elif input_handler.keyboard_action_pressed_once('change_legs', events):
        if sitting_pose.blocks_movement() or rearing_pose.blocks_movement():
            log.info("Return from the static pose before changing legs.")
        else:
            gait_manager.stop()
            movement = False
            calibration.open_picker()

    transition_error = rearing_pose.update()
    if transition_error:
        log.error(transition_error)

    head.update(input_handler.head_direction())

    # Consume both keys every frame so holding either never repeats a toggle.
    sit_pressed = input_handler.keyboard_action_pressed_once('sit', events)
    rear_pressed = input_handler.keyboard_action_pressed_once('rear', events)
    if calibration.active:
        sit_pressed = rear_pressed = False
    if rear_pressed:
        if sitting_pose.blocks_movement():
            log.info('Press P to stand before rearing.')
        else:
            try:
                toggle = rearing_pose.retry_lowering if rearing_pose.failed else rearing_pose.toggle
                if toggle(pos_x, pos_y):
                    if record:
                        q8.finish_recording()
                    record = False
                    gait_manager.stop()
                    movement = False
                    log.info('Rearing pose' if rearing_pose.active else 'Lowering to gait position')
                else:
                    log.error('Could not send rearing/lowering pose')
            except ValueError as error:
                log.error(str(error))
    elif sit_pressed and rearing_pose.blocks_movement():
        log.info('Press U to lower before sitting.')
    elif sit_pressed:
        try:
            if sitting_pose.toggle(pos_x, pos_y):
                if record:
                    q8.finish_recording()
                record = False
                gait_manager.stop()
                movement = False
                log.info("Sitting pose" if sitting_pose.active else "Returning to gait position")
            else:
                log.error("Could not send sitting/standing pose")
        except ValueError as error:
            log.error(str(error))

    # Clear screen and render logger messages
    window.fill(eth.BACKGROUND)
    display_pose = rearing_pose if rearing_pose.blocks_movement() else sitting_pose
    control_panel.draw(window, gait_manager.current_gait, display_pose, calibration, head)
    Q8Logger.render_pygame_messages()  # Draw logger on top
    pygame.display.flip()

    if calibration.blocks_movement():
        # Torque holds the mounting pose; nothing else may command the joints.
        pass
    elif sitting_pose.blocks_movement() or rearing_pose.blocks_movement():
        # Keep the static pose; battery checks and exit remain available.
        if input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
    elif movement:
        # Get requested direction from input handler
        requested_direction = input_handler.get_movement_direction()

        if requested_direction:
            # Start or switch movement direction
            if gait_manager.start_movement(requested_direction):
                # Execute current trajectory
                pos = gait_manager.tick()
                if pos:
                    q8.move_all(pos, 0, record)
            else:
                # Failed to start movement
                movement = False
        else:
            # No movement input - transition to idle
            move_xy(pos_x, pos_y, 0)
            q8.finish_recording()
            record = False
            gait_manager.stop()
            movement = False
    else:
        # Check for movement input
        if input_handler.is_movement_input():
            movement = True
        # Check action inputs using generalized interface
        elif input_handler.is_action_pressed('reset'):
            log.info("Gait Reset")
            move_xy(pos_x, pos_y, 500)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('recover'):
            log.info("Recovering servos: clearing faults and re-enabling torque")
            q8.send_recover()
            time.sleep(2)
            move_xy(pos_x, pos_y, 1000)
        elif input_handler.is_action_pressed('jump'):
            log.info("Jump")
            q8.send_jump()
            time.sleep(5)
            move_xy(pos_x, pos_y, 500)
        elif input_handler.is_action_pressed('switch_gait'):
            # Cycle to next gait
            gait_names.append(gait_names.pop(0))
            new_gait = gait_names[0]

            # Load new gait
            if gait_manager.load_gait(new_gait):
                # Update position to match new gait
                pos_x, pos_y = fitted_design.gaits[new_gait][1], fitted_design.gaits[new_gait][2]
                move_xy(pos_x, pos_y, 500)
                log.info(f"Switched to {new_gait}")
            else:
                log.error(f"Failed to load gait: {new_gait}")
                gait_names.insert(0, gait_names.pop())  # Revert gait change
            time.sleep(0.2)
        elif input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('record'):
            log.debug("Record next movement")
            record = True
            request = "data"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('show_range'):
            log.info("Show Range")
            show_range(q8)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('greet'):
            log.info("Greet")
            greet(q8)
            move_xy(pos_x, pos_y, 1000)  # Return to rest position
            time.sleep(0.2)
        elif input_handler.is_action_pressed('exit'):
            break

    # Drain replies while sitting as well as while operating a gait.
    try:
        while q8.serialHandler.in_waiting > 0:
            raw_data = q8.serialHandler.readline().decode('utf-8').strip().split()
            if request == "battery" and raw_data and raw_data[0].isdigit():
                log.info(f"Battery: {int(raw_data[0])}%")
                request = "none"
    except (ValueError, UnicodeError):
        log.debug("Data reading failed. Continuing...")

q8.disable_torque()
if joystick:
    joystick.quit()
pygame.quit()
