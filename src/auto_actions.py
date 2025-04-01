from commands2 import Command, InstantCommand
from typing import Callable, Tuple

from subsystems.drive import Drive
from subsystems.vision import Vision
from subsystems.periscope import Periscope
from commands.approach_hps import ApproachHPS
from commands.approach_reef import ApproachReef
from commands.place_coral import PlaceCoral
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
) -> Tuple[Callable[[bool], Command], Callable[[int, int], Command]]:
    def transit():
        periscope.arm.set_target(config.transit_setpoint)

    def g(left_hps: bool):
        return ApproachHPS(drive, vision, periscope, graph, left_hps).andThen(
            InstantCommand(transit)
        )

    def s(stalk_i: int, level_i: int):
        return ApproachReef(
            drive, vision, periscope.arm, graph, stalk_i, level_i, False
        ).andThen(PlaceCoral(periscope))

    return (g, s)
