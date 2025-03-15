from commands2 import Command
from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from typing import Optional, List
from math import atan2, isfinite

from commands.drive_to_pose import DriveToPose

# from commands import chase_note
from subsystems.drive import Drive

# from subsystems.note_vision import NoteVision
from utils.graph import Graph
import config


class GraphPathfind(Command):
    def __init__(
        self,
        target: Translation2d,
        graph: Graph,
        drive: Drive,
        # note_vision: NoteVision,
        # chase_note: bool = False,
        target_rot_override: Optional[Rotation2d] = None,
        final_passthrough: float = 0.1,
    ):
        self.target = target
        self.graph = graph
        self.drive = drive
        # self.note_vision = note_vision
        self.path: Optional[List[Translation2d]] = None
        self.dtp: Optional[DriveToPose] = None
        self.target_rot: Rotation2d = Rotation2d()
        # self.chase_note = chase_note
        self.target_rot_override = target_rot_override
        self.final_passthrough = final_passthrough

    def initialize(self):
        self.path = self.graph.create_path(
            self.drive.odometry.pose().translation(), self.target
        )
        final_vec = self.path[-1] - self.path[-2]
        final_vec_angle = atan2(final_vec.y, final_vec.x)
        if not isfinite(final_vec_angle):
            final_vec_angle = 0
        # self.target_rot = self.target_rot_override or (
        #     (self.path[-1] - self.path[-2]).angle() + Rotation2d.fromDegrees(180)
        # )
        self.target_rot = self.target_rot_override or (
            Rotation2d(final_vec_angle) + Rotation2d.fromDegrees(180)
        )

    def execute(self):
        # print("path:", self.path)
        if not self.path:
            return
        # cur_pos = self.drive.odometry.pose().translation()
        if self.dtp is None:
            passthrough = self.final_passthrough if len(self.path) == 1 else 0.6
            target_pose = Pose2d(self.path[0], self.target_rot)
            # dtp = DriveToPose(target_pose, self.drive, passthrough=passthrough)
            dtp = DriveToPose(
                target_pose,
                self.drive,
                speed=config.auto_drive_speed,
                turn_speed=config.auto_turn_speed,
                passthrough=passthrough,
            )
            # if self.chase_note and len(self.path) == 1:
            #     dtp = chase_note.from_dtp(dtp, self.note_vision)
            self.dtp = dtp
            self.dtp.initialize()
        self.dtp.execute()

        if self.dtp.isFinished():
            self.path.pop(0)
            self.dtp = None

    def isFinished(self):
        # return self.path is not None and len(self.path) <= (1 if self.end_early else 0)
        return self.path is not None and len(self.path) == 0

    def end(self, interrupted: bool):
        self.drive.drive(Transform2d(), True)
