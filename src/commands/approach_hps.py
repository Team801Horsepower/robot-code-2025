from commands2 import Command
from wpimath.geometry import Rotation2d, Translation2d

from subsystems.drive import Drive
from subsystems.vision import Vision
from subsystems.periscope import Periscope

from commands.graph_pathfind import GraphPathfind
from commands.target_hps import TargetHPS

from utils.graph import Graph
import config


class ApproachHPS(Command):
    def __init__(
        self,
        drive: Drive,
        vision: Vision,
        periscope: Periscope,
        graph: Graph,
        left_hps: bool,
    ):
        self.drive = drive
        self.vision = vision
        self.periscope = periscope

        self.left_hps = left_hps

        self.th_cmd = TargetHPS(self.drive, self.vision, left_hps)

        # face_i = int(stalk_i / 2)
        # target_angle = 0.0 if config.is_red() else pi
        # target_angle += pi / 3 * face_i

        # reef_center_pos = Translation2d(4.5, 4)
        # pathfind_pos = reef_center_pos + Translation2d(2.2, 0).rotateBy(
        #     Rotation2d(self.th_cmd.face_i * pi / 3)
        # )
        # pathfind_pos = Translation2d(1.2, 7 if left_hps else config.field_width - 7)
        pathfind_pos = Translation2d(
            1.74, 7.34 if left_hps else config.field_width - 7.34
        )
        pathfind_pos = config.flip_red(pathfind_pos)

        self.gpf_cmd = GraphPathfind(
            pathfind_pos,
            graph,
            self.drive,
            Rotation2d(self.th_cmd.target_angle),
            # 0.4,
        )

        self.current_cmd = self.gpf_cmd

    def initialize(self):
        self.current_cmd.initialize()

    def execute(self):
        near_source = self.drive.odometry.near_source_large()

        if self.gpf_cmd.dtp is not None:
            if self.drive.odometry.near_reef():
                self.gpf_cmd.dtp.turn_speed = 0
            else:
                self.gpf_cmd.dtp.turn_speed = config.auto_turn_speed

        tag_seen = (
            self.th_cmd.get_left_param() is not None
            and self.th_cmd.get_right_param() is not None
            # and abs(
            #     self.th_cmd.target_angle - self.drive.odometry.rotation().radians()
            # )
            # < units.degreesToRadians(10)
        )

        if isinstance(self.current_cmd, GraphPathfind):
            if self.drive.odometry.near_reef():
                self.periscope.claw.set(1)
            else:
                self.periscope.claw.set(0)
            if self.current_cmd.isFinished() or (tag_seen and near_source):
                self.current_cmd.end(False)
                self.current_cmd = self.th_cmd
                self.current_cmd.initialize()
        elif (
            isinstance(self.current_cmd, TargetHPS) and not tag_seen or not near_source
        ):
            self.current_cmd = self.th_cmd

        if near_source:
            self.periscope.arm.target = config.source_setpoint
            self.periscope.claw.set(-1)

        self.current_cmd.execute()

    def isFinished(self) -> bool:
        return self.periscope.claw.has_coral()

    def end(self, interrupted: bool):
        self.current_cmd.end(interrupted)
