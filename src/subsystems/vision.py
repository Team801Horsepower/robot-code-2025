from commands2 import CommandScheduler, Subsystem
from wpimath.geometry import Translation2d
from typing import Tuple, List, Optional


class Vision(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

    def periodic(self):
        pass

    # (tag pitch, tag yaw)
    def cur_atag(
        self, cam_i: int, red_id: int, blue_id: int
    ) -> Optional[Tuple[float, float]]:
        return None

    # (ntags, (position, confidence, deviation))
    def pos_report(
        self, robot_angle: float
    ) -> Tuple[int, Optional[Tuple[Translation2d, float, float]]]:
        return (0, None)

    # returns corrected heading based on april tags
    def heading_correction(self, robot_angle: float) -> Optional[float]:
        return None
