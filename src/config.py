from wpimath import units
from wpimath.geometry import Translation2d
from wpilib import DriverStation

from os import path


def is_red() -> bool:
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed


code_path = path.dirname(__file__) + "/"

robot_dimensions = Translation2d(units.inchesToMeters(20.5), units.inchesToMeters(20.5))

drive_gear_ratio = 4.4
turn_gear_ratio = 41.25

wheel_diameter = units.inchesToMeters(3)

drive_speed = 2
turn_speed = 4
auto_drive_speed = 2
auto_turn_speed = 2

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
