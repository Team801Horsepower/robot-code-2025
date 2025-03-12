from commands2 import Command
from wpimath.geometry import Rotation2d, Translation2d

from subsystems.drive import Drive
from subsystems.vision import Vision
from subsystems.arm import Arm

from commands.graph_pathfind import GraphPathfind
from commands.target_hps import TargetHPS

from utils.graph import Graph
import config


class ApproachHPS(Command):
    def __init__(
        self,
        drive: Drive,
        vision: Vision,
        arm: Arm,
        graph: Graph,
        left_hps: bool,
    ):
        self.drive = drive
        self.vision = vision
        self.arm = arm

        self.left_hps = left_hps

        self.th_cmd = TargetHPS(self.drive, self.vision, left_hps)

        # face_i = int(stalk_i / 2)
        # target_angle = 0.0 if config.is_red() else pi
        # target_angle += pi / 3 * face_i

        # reef_center_pos = Translation2d(4.5, 4)
        # pathfind_pos = reef_center_pos + Translation2d(2.2, 0).rotateBy(
        #     Rotation2d(self.th_cmd.face_i * pi / 3)
        # )
        pathfind_pos = Translation2d(1.2, 7 if left_hps else config.field_width - 7)
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
        if isinstance(self.current_cmd, GraphPathfind):
            tag_seen = (
                self.th_cmd.get_left_param() is not None
                and self.th_cmd.get_right_param() is not None
            )
            if self.current_cmd.isFinished() or tag_seen:
                self.current_cmd.end(False)
                self.current_cmd = self.th_cmd
                self.current_cmd.initialize()
        elif isinstance(self.current_cmd, TargetHPS):
            # TODO: change to condition of near reef
            if True:
                self.arm.target = config.source_setpoint

            # if self.current_cmd.isFinished():
            #     self.current_cmd.end(False)
            #     self.current_cmd = PlaceCoral(self.periscope)

        self.current_cmd.execute()

    def isFinished(self) -> bool:
        return (
            # isinstance(self.current_cmd, PlaceCoral) and self.current_cmd.isFinished()
            isinstance(self.current_cmd, TargetHPS)
            and self.current_cmd.isFinished()
        )

    def end(self, interrupted: bool):
        self.current_cmd.end(interrupted)
