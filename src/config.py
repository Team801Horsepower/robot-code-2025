from typing import List, Tuple

from wpimath import units
from wpimath.geometry import Translation2d, Transform2d
from wpilib import DriverStation
from os import path
from math import pi

from utils import format_table

# -- General --

g = 9.81
field_width = 8.0518
field_length = 17.548


def is_red() -> bool:
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed


def flip_red(pos: Translation2d) -> Translation2d:
    return Translation2d(field_length, field_width) - pos if is_red() else pos


code_path = path.dirname(__file__) + "/"

robot_dimensions = Translation2d(units.inchesToMeters(20.5), units.inchesToMeters(20.5))

robot_frame_dimensions = Translation2d(
    units.inchesToMeters(28), units.inchesToMeters(30)
)

# -- Drive --

drive_gear_ratio = 4.4
turn_gear_ratio = 41.25

wheel_diameter = units.inchesToMeters(3)

drive_speed = 6
turn_speed = 6
auto_drive_speed = 6
auto_turn_speed = 6

drive_acc_lim: List[Tuple[float, ...]] = format_table(
    [
        (0, 20),
        (0.5, 16),
        (1, 13),
        (1.5, 10),
        (2, 7),
    ]
)

# drive id, turn id, cancoder id
swerves = [
    (10, 11, 12),
    (15, 16, 17),
    (20, 21, 22),
    (25, 26, 27),
]

# front left, back left, front right, back right
swerve_ids = [3, 1, 2, 0]

drive_pid = (0.15, 0, 0)
turn_pid = (0.5, 0, 0)

# -- Vision --

# -- Periscope --

# -- Claw --

claw_motor_id = 51
claw_beam_break_id = 0

# -- Climber --

# -- Arm --

# TODO: Set claw to wrist length and angle to correct value
claw_to_wrist_lengths = {
    "coral": units.inchesToMeters(17.5),  # TODO: this is approximate
    "algae": 0.5,
}
coral_algae_pickup_angle = 1  # TODO: get real

claw_up_down_lengths = (units.inchesToMeters(5), units.inchesToMeters(22))
ik_boundary_distance = units.feetToMeters(1.5)
ik_floor = 0.2

ik_neutral_x = 0.06846
ik_neutral_y = 0.97244
ik_neutral_wrist = units.degreesToRadians(3.31974)

# -- Pivot --

# TODO: Get real (y)
pivot_offset = Translation2d(-0.2284, 0.2177)
pivot_range = (pi / 4, pi / 2)  # TODO: Insert correct angle values

pivot_motor_ids = [30, 32, 31, 33]
# pivot_pid_constants = (0.65, 0, 0)
pivot_pid_constraint_constants = (100, 15)

pivot_gear_ratio = 80.801

pivot_encoder_id = 0
pivot_angle_offset = 0.407 * 2 * pi
pivot_com_offset_for_feedforward = 0.1742036732  # for claw at zero
pivot_epsilon_pos = 0.08
pivot_epsilon_v = 0.15

middle_finger_angle = units.degreesToRadians(90)

# fmt: off
# (extension, kP, kI, kD)
pivot_pid_constants: List[Tuple[float, ...]] = format_table([
    # (0.7366, 0.65, 0, 0),
    # (0.9165968599, 0.55, 0, 0),
    # (1.276590580, 0.30, 0, 0),
    # (1.636584300, 0.25, 0, 0),
    # (1.906579590, 0.2, 0, 0),
    (0.7366, 0.65, 0, 0),
    (0.9165968599, 0.6, 0, 0),
    (1.276590580, 0.55, 0, 0),
    (1.636584300, 0.5, 0, 0),
    (1.906579590, 0.45, 0, 0),
])

# (extension, acc)
pivot_acc_lim: List[Tuple[float, ...]] = format_table([
    (0.7366, 10),
    (0.9165968599, 8),
    (1.276590580, 3),
    (1.636584300, 1.5),
    (1.906579590, 0),
])
# fmt: on

# -- Elevator --

elevator_mass = 1  # kg

# fmt: off
elevator_dynamics_table: List[Tuple[float, ...]] = format_table([
    (0.7366, 0.00263556269000882, 0),
    (0.9165968599, 0.00299495760228274, 0),
    (1.096593720, 0.00347415081864798, 0),
    (1.276590580, 0.00407314233910454, 0),
    (1.456587440, 0.00443253725137846, 0),
    (1.91, 0.0065, 0),
])
# fmt: on
extension_motor_ids = [40, 41]
extension_pid_constants = (4, 0, 0)
extension_pid_constraint_constants = (1, 10)
elevator_ff_power = 0.02

extension_range = (units.inchesToMeters(29), units.inchesToMeters(75.375))  # m
extension_ratio = units.inchesToMeters(0.22557 * pi)  # m / revolution

# -- Wrist --
wrist_pid_constants = (0.8, 0, 0)
wrist_motor_id = 50  # TODO: change to actual motor id
wrist_limits = (
    units.degreesToRadians(50),
    units.degreesToRadians(252),
)  # TODO: change to the range of angles attainable by the wrist
# Starting angle, with claw against back beam
wrist_neutral_angle = units.degreesToRadians(88.31974)
# Lowest elevator extension at which the claw is guaranteed to be
# able to clear the back beam and rotate below the neutral angle
wrist_passthrough_min_extension = 1.8

# wrist_gear_ratio = 62.5 * 2 * pi
# Weird measured thing; TODO: find out why
wrist_gear_ratio = 11.45

# -- Setpoints --

reef_setpoints = [
    # Transform2d(0.4, 0.87237, units.degreesToRadians(-7)),
    (units.degreesToRadians(28), extension_range[0], units.degreesToRadians(155)),
    # Transform2d(0.45, 0.90237, units.degreesToRadians(-7)),
    # Transform2d(0.5, 0.87, units.degreesToRadians(-10)),
    Transform2d(0.6, 0.87, units.degreesToRadians(-10)),
    # Transform2d(0.47, 1.31416, units.degreesToRadians(-7)),
    Transform2d(0.57, 1.31416, units.degreesToRadians(-5)),
    # Transform2d(0.32, 1.988, units.degreesToRadians(-15)),
    # Transform2d(0.58, 1.988, units.degreesToRadians(-26)),
    # Transform2d(0.5, 1.988, units.degreesToRadians(-21)),
    # Transform2d(0.6, 1.988, units.degreesToRadians(-17)),
    Transform2d(0.52, 1.988, units.degreesToRadians(-17)),
]
barge_setpoint = Transform2d(-0.375, 2.4, units.degreesToRadians(140))
# source_setpoint = Transform2d(-0.50, 0.87, units.degreesToRadians(160))
source_setpoint = Transform2d(-0.5, 0.82, units.degreesToRadians(160))

ground_pickup_setpoint = (
    units.degreesToRadians(35),
    extension_range[0],
    units.degreesToRadians(118),
)
algae_reef_setpoints = [
    Transform2d(0.5, 1.1, units.degreesToRadians(35)),
    Transform2d(0.5, 1.6, units.degreesToRadians(35)),
]

transit_setpoint = (
    units.degreesToRadians(68.7),
    extension_range[0],
    units.degreesToRadians(114.6),
)
processor_setpoint = (
    # units.degreesToRadians(35),
    units.degreesToRadians(32),
    extension_range[0],
    units.degreesToRadians(166),
)
