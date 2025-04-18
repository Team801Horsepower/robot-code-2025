from commands2 import Command, InstantCommand, WaitCommand
from wpimath.geometry import Pose2d, Transform2d
from typing import Callable, Tuple
from wpimath import units
from functools import reduce
from math import pi, inf

from commands.drive_to_pose import DriveToPose
from subsystems.drive import Drive
from subsystems.vision import Vision
from subsystems.periscope import Periscope
from commands.approach_hps import ApproachHPS
from commands.approach_reef import ApproachReef
from commands.place_coral import PlaceCoral
from commands.grab_algae import GrabAlgae
from commands.shift import Shift
from utils.graph import Graph
import config

# class Collect:
#     def __init__(self, left_hps: bool):
#         self.left_hps = left_hps

#         self.command = None


# class Score:
#     def __init__(self, stalk: int, branch: int):
#         self.stalk = stalk
#         self.branch = branch

#         self.command = None


def make_auto_methods(
    drive: Drive, vision: Vision, periscope: Periscope, graph: Graph
) -> Tuple[
    Callable[[bool], Command],
    Callable[[int, int], Command],
    Callable[[int], Command],
    Callable[..., Command],
    Callable[..., Command],
    Callable[[Transform2d | Tuple[float, float, float]], Command],
    Callable[[Pose2d], Command],
]:
    def g(left_hps: bool):
        return ApproachHPS(drive, vision, periscope, graph, left_hps).andThen(
            InstantCommand(lambda: periscope.arm.set_target(config.transit_setpoint))
        )

    def s(stalk_i: int, level_i: int):
        return ApproachReef(
            drive, vision, periscope.arm, graph, stalk_i, level_i, False
        ).andThen(PlaceCoral(periscope))

    def a(face_i: int):
        return ApproachReef(
            drive, vision, periscope.arm, graph, face_i * 2, 0, True
        ).andThen(GrabAlgae(drive, periscope.claw))

    def sh(
        delta: Transform2d,
        speed=config.auto_drive_speed,
        turn_speed=config.auto_turn_speed,
        passthrough=0,
        heading_pt=inf,
    ):
        return Shift(delta, drive, speed, turn_speed, passthrough, heading_pt)

    def dtp(
        pose: Pose2d,
        speed=config.auto_drive_speed,
        turn_speed=config.auto_turn_speed,
        passthrough=0,
        heading_pt=inf,
    ):
        return DriveToPose(pose, drive, speed, turn_speed, passthrough, heading_pt)

    def arm(setpoint: Transform2d | Tuple[float, float, float]):
        return InstantCommand(lambda: periscope.arm.set_target(setpoint))

    def start_at(pose: Pose2d):
        return InstantCommand(lambda: drive.odometry.reset(pose))

    return (g, s, a, sh, dtp, arm, start_at)


class Autos:
    def __init__(
        self, drive: Drive, vision: Vision, periscope: Periscope, graph: Graph
    ):
        g, s, a, sh, dtp, arm, start_at = make_auto_methods(
            drive, vision, periscope, graph
        )

        def auto(*cmds: Command) -> Command:
            return reduce(Command.andThen, cmds)

        right_y = 0.5
        left_y = config.field_width - right_y
        center_y = config.field_width / 2

        color = "red" if config.is_red() else "blue"

        self.left = auto(
            InstantCommand(lambda: print(f"running {color} left auto")),
            start_at(config.flip_red_pose(Pose2d(7.17, left_y, 0))),
            s(3, 3),
            g(True),
            s(4, 3),
            g(True),
            s(5, 3),
        )

        self.flipped_left = auto(
            InstantCommand(lambda: print(f"running {color} flipped left auto")),
            start_at(config.flip_red_pose(Pose2d(7.17, left_y, pi))),
            s(3, 3),
            g(True),
            s(4, 3),
            g(True),
            s(5, 3),
        )

        self.right = auto(
            InstantCommand(lambda: print(f"running {color} right auto")),
            start_at(config.flip_red_pose(Pose2d(7.17, right_y, 0))),
            s(10, 3),
            g(False),
            s(9, 3),
            g(False),
            s(8, 3),
        )

        self.flipped_right = auto(
            InstantCommand(lambda: print(f"running {color} flipped right auto")),
            start_at(config.flip_red_pose(Pose2d(7.17, right_y, pi))),
            s(10, 3),
            g(False),
            s(9, 3),
            g(False),
            s(8, 3),
        )

        self.center = auto(
            InstantCommand(lambda: print(f"running {color} center auto")),
            start_at(config.flip_red_pose(Pose2d(7.17, center_y, pi))),
            s(0, 3),
            sh(Transform2d(0.65, 0, 0), passthrough=0.4),
            a(0),
            arm(config.processor_setpoint),
            # dtp(
            #     config.flip_red_pose(Pose2d(5.77, 0.85, 3 * pi / 2)),
            #     passthrough=0.1,
            #     heading_pt=units.degreesToRadians(10),
            # ),
            dtp(
                config.flip_red_pose(
                    Pose2d(5.44, 0.85, 3 * pi / 2 + units.degreesToRadians(8.01))
                ),
                passthrough=0.1,
                heading_pt=units.degreesToRadians(10),
            ),
            InstantCommand(lambda: drive.drive(Transform2d())),
            InstantCommand(lambda: periscope.claw.set(-1))
            .andThen(WaitCommand(0.4))
            .andThen(InstantCommand(lambda: periscope.claw.set(0))),
            arm(config.transit_setpoint),
            # sh(Transform2d(0.5, 0, 0), passthrough=0.3),
            sh(Transform2d(0, 0.5, 0), passthrough=0.3),
            a(5),
            arm(config.processor_setpoint),
            # dtp(
            #     config.flip_red_pose(Pose2d(5.77, 0.85, 3 * pi / 2)),
            #     passthrough=0.1,
            #     heading_pt=units.degreesToRadians(10),
            # ),
            dtp(
                config.flip_red_pose(
                    Pose2d(5.44, 0.85, 3 * pi / 2 + units.degreesToRadians(8.01))
                ),
                passthrough=0.1,
                heading_pt=units.degreesToRadians(10),
            ),
            InstantCommand(lambda: drive.drive(Transform2d())),
            InstantCommand(lambda: periscope.claw.set(-1))
            .andThen(WaitCommand(1))
            .andThen(InstantCommand(lambda: periscope.claw.set(0))),
        )
