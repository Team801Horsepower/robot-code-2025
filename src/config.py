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
auto_drive_speed = 0.5
auto_turn_speed = 0.5

swerves = [
    (10, 11, 0),
    (20, 21, 0),
    (30, 31, 0),
    (40, 41, 0),
]

# front left, back left, front right, back right
swerve_ids = [1, 2, 0, 3]
