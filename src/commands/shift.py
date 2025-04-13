from wpimath.geometry import Transform2d
from math import inf

from subsystems.drive import Drive
from commands.drive_to_pose import DriveToPose
import config


class Shift(DriveToPose):
    def __init__(
        self,
        delta: Transform2d,
        drive: Drive,
        speed: float = config.auto_drive_speed,
        turn_speed: float = config.auto_turn_speed,
        passthrough: float = 0,
        heading_pt: float = inf,
    ):
        self.delta = config.flip_red_transform(delta)
        target = drive.odometry.pose() + delta
        super().__init__(target, drive, speed, turn_speed, passthrough, heading_pt)

    def initialize(self):
        self.target = self.drive.odometry.pose() + self.delta
        super().initialize()
