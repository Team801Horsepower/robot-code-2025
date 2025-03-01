from typing import List, Tuple

from wpimath import units
from wpimath.geometry import Translation2d
from wpilib import DriverStation
from os import path
from math import pi

from utils import format_table

# -- General --

g = 9.81


def is_red() -> bool:
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed


code_path = path.dirname(__file__) + "/"

robot_dimensions = Translation2d(units.inchesToMeters(20.5), units.inchesToMeters(20.5))

# -- Drive --

drive_gear_ratio = 4.4
turn_gear_ratio = 41.25

wheel_diameter = units.inchesToMeters(3)

bumper_distance = 0.5

drive_speed = 2
turn_speed = 4
auto_drive_speed = 0.5
auto_turn_speed = 0.5


# drive id, turn id, cancoder id
swerves = [
    (10, 11, 12),
    (20, 21, 22),
    (15, 16, 17),
    (25, 26, 27),
]

# front left, back left, front right, back right
swerve_ids = [1, 2, 0, 3]  # TODO: this is probably wrong now

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
    "coral": 0.5,
    "algae": 0.5,
}
coral_algae_pickup_angle = 1

claw_up_down_lengths = (1, 0.5)
# TODO: Get real :()
ik_boundary_distance = units.feetToMeters(1.5)
# TODO: SET THIS; IK will not be able to put the claw below this point
ik_floor = 1

# -- Pivot --

# TODO: Get real (y)
pivot_offset = Translation2d(-0.19671, 0)
pivot_range = (0.295, 0.100)  # TODO: Insert correct angle values

pivot_motor_ids = [30, 32, 31, 33]
# pivot_pid_constants = (0.65, 0, 0)
pivot_pid_constraint_constants = (100, 15)

pivot_encoder_id = 0
pivot_angle_offset = 0.407 * 2 * pi
pivot_com_offset_for_feedforward = 0.1742036732  # for claw at zero
pivot_epsilon_pos = 0.05
pivot_epsilon_v = 0.05

# fmt: off
# (extension, kP, kI, kD)
pivot_pid_constants: List[Tuple[float, ...]] = format_table([
    (0.7366, 0.65, 0, 0),
    (0.9165968599, 0.55, 0, 0),
    (1.276590580, 0.30, 0, 0),
    (1.636584300, 0.15, 0, 0),
    (1.906579590, 0.10, 0, 0),
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
])
# fmt: on
extension_motor_ids = [40, 41]
extension_pid_constants = (5.555652474, 0, 0)
extension_pid_constraint_constants = (1, 10)
elevator_ff_power = 0.02

extension_range = (units.inchesToMeters(29), units.inchesToMeters(75.375))  # m
extension_ratio = units.inchesToMeters(0.22557 * pi)  # m / revolution

# -- Wrist --
wrist_pid_constants = (0.1, 0, 0)
wrist_motor_id = 50  # TODO: change to actual motor id
wrist_limits = (
    -20,
    33.2,
)  # TODO: change to the range of angles attainable by the wrist
