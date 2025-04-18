from wpimath.geometry import Transform2d, Pose2d
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
        self.drive = drive
        target = self.make_target()
        super().__init__(target, drive, speed, turn_speed, passthrough, heading_pt)

    def initialize(self):
        self.target = self.make_target()
        super().initialize()

    def make_target(self) -> Pose2d:
        cur_pose = self.drive.odometry.pose()
        return Pose2d(
            cur_pose.translation() + self.delta.translation(),
            cur_pose.rotation() + self.delta.rotation(),
        )
