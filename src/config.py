from typing import List, Tuple

from wpimath import units
from wpimath.geometry import Translation2d
from wpilib import DriverStation
from os import path

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
    (30, 31, 32),
    (40, 41, 42),
]

# front left, back left, front right, back right
swerve_ids = [1, 2, 0, 3]

drive_pid = (0.15, 0, 0)
turn_pid = (0.5, 0, 0)

# -- Vision --

# -- Periscope --

# -- Claw --

# TODO: actual values
claw_motor_id = 0
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
# TODO: Get real
pivot_offset = Translation2d(1, 1)

pivot_motor_ids = [8, 9, 10, 11]  # TODO: set correct IDs
pivot_pid_constants = (1, 1, 1)
pivot_pid_constraint_constants = (1, 1)

pivot_encoder_id = 0
pivot_angle_offset = 0.5
pivot_epsilon_pos = 0.05
pivot_epsilon_v = 0.05
# -- Elevator --

elevator_mass = 1 # kg

# fmt: off
elevator_dynamics_table = [
    (0, 0, 0),
    (0, 0, 0),
    (0, 0, 0),
    (0, 0, 0),
    (0, 0, 0),
]
elevator_dynamics_table: List[Tuple[float, ...]] = format_table(elevator_dynamics_table)
# fmt: on

extension_motor_ids = [20, 21]  # TODO: set correct IDs
extension_pid_constants = (1, 1, 1)
extension_pid_constraint_constants = (1, 1)

extension_zero_length = 1 # m
extension_ratio = 1 # m / revolution
# -- Wrist --

wrist_motor_id = 0 # TODO: change to actual motor id
wrist_limits = (0, 1) # TODO: change to the range of angles attainable by the wrist
