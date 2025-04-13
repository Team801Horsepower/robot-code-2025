from commands2 import Command
from wpimath.geometry import Transform2d, Translation2d, Rotation2d
from functools import reduce
import time

from subsystems.drive import Drive
from subsystems.claw import Claw
from commands.drive_to_pose import DriveToPose


class GrabAlgae(Command):
    def __init__(self, drive: Drive, claw: Claw):
        self.drive = drive
        self.claw = claw

        # self.dtps = None

        self.return_pose = None
        self.dtp = None
        self.has_algae_time = None
        self.min_has_algae_time = 0.05

    # def initialize(self):
    #     cur_pose = self.drive.odometry.pose()
    #     pose_diff = Transform2d(
    #         Translation2d(0.38, 0).rotateBy(cur_pose.rotation()), Rotation2d()
    #     )
    #     target_pose = cur_pose + pose_diff
    #     self.dtps = reduce(
    #         Command.andThen,
    #         [DriveToPose(pose, self.drive, 0.5) for pose in [target_pose, cur_pose]],
    #     )

    #     self.dtps.initialize()

    # def execute(self):
    #     if self.claw.has_algae():
    #         self.claw.set(0)
    #     else:
    #         self.claw.set(1)

    #     if self.dtps is None:
    #         return
    #     self.dtps.execute()

    # def isFinished(self) -> bool:
    #     # return (self.dtps or False) and self.dtps.isFinished()
    #     return self.dtps is not None and self.dtps.isFinished()

    # def end(self, interrupted: bool):
    #     if self.dtps is not None:
    #         self.dtps.end(interrupted)

    def initialize(self):
        self.return_pose = self.drive.odometry.pose() + Transform2d(
            Translation2d(-0.3, 0).rotateBy(self.drive.odometry.rotation()),
            Rotation2d(),
        )

    def execute(self):
        if self.claw.has_algae():
            if self.has_algae_time is None:
                self.has_algae_time = time.time()
        else:
            self.has_algae_time = None

        really_has_algae = (
            self.has_algae_time is not None
            and time.time() - self.has_algae_time > self.min_has_algae_time
        )

        if really_has_algae:
            self.claw.set(0)
            if self.return_pose is not None:
                if self.dtp is None:
                    self.dtp = DriveToPose(
                        self.return_pose, self.drive, speed=2.5, passthrough=0.3
                    )
                    self.dtp.initialize()
                self.dtp.execute()
        else:
            self.claw.set(1)
            self.drive.drive(Transform2d(1.6, 0, 0))

    def isFinished(self) -> bool:
        return self.dtp is not None and self.dtp.isFinished()
