from commands2 import Command
from wpimath.geometry import Rotation2d, Translation2d
from wpimath import units
from wpilib import SmartDashboard

from math import pi

from subsystems.drive import Drive
from subsystems.vision import Vision
from subsystems.arm import Arm

from commands.graph_pathfind import GraphPathfind
from commands.target_reef import TargetReef

from utils.graph import Graph
import config


class ApproachReef(Command):
    def __init__(
        self,
        drive: Drive,
        vision: Vision,
        arm: Arm,
        graph: Graph,
        stalk_i: int,
        target_level: int,
        algae: bool,
    ):
        self.drive = drive
        self.vision = vision
        self.arm = arm

        self.stalk_i = stalk_i
        self.target_level = target_level

        self.algae = algae

        self.tr_cmd = TargetReef(self.drive, self.vision, self.stalk_i, algae)

        reef_center_pos = Translation2d(4.5, 4)
        # pathfind_pos = reef_center_pos + Translation2d(2.2, 0).rotateBy(
        pathfind_pos = reef_center_pos + Translation2d(1.7, 0).rotateBy(
            Rotation2d(self.tr_cmd.face_i * pi / 3)
        )
        pathfind_pos = config.flip_red(pathfind_pos)

        self.gpf_cmd = GraphPathfind(
            pathfind_pos,
            graph,
            self.drive,
            Rotation2d(self.tr_cmd.target_angle),
        )

        self.current_cmd = self.gpf_cmd

    def initialize(self):
        self.current_cmd.initialize()

    def execute(self):
        near_reef = self.drive.odometry.near_reef_large()

        ang_dist = abs(
            (self.tr_cmd.target_angle - self.drive.odometry.rotation().radians() + pi)
            % (2 * pi)
            - pi
        )
        SmartDashboard.putNumber(
            "dist from target angle",
            units.radiansToDegrees(ang_dist),
        )

        tag_seen = (
            self.tr_cmd.get_left_param() is not None
            and self.tr_cmd.get_right_param() is not None
            and ang_dist < units.degreesToRadians(15)
        )

        if isinstance(self.current_cmd, GraphPathfind):
            if self.current_cmd.isFinished() or (tag_seen and near_reef):
                self.current_cmd.end(False)
                self.current_cmd = self.tr_cmd
                self.current_cmd.initialize()
        elif isinstance(self.current_cmd, TargetReef):
            if not tag_seen or not near_reef:
                self.current_cmd = self.gpf_cmd

        if near_reef:
            if not self.algae:
                self.arm.target = config.reef_setpoints[self.target_level]
            else:
                height = int(self.stalk_i / 2) % 2
                self.arm.target = config.algae_reef_setpoints[height]

        self.current_cmd.execute()

    def isFinished(self) -> bool:
        return (
            isinstance(self.current_cmd, TargetReef) and self.current_cmd.isFinished()
        )

    def end(self, interrupted: bool):
        self.current_cmd.end(interrupted)
