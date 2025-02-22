from commands2 import CommandScheduler, Subsystem
from wpimath.geometry import Transform2d
from typing import Optional
from math import atan2

from subsystems.pivot import Pivot
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


class Arm(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.pivot = Pivot(scheduler)
        self.elevator = Elevator(scheduler)
        self.wrist = Wrist(scheduler)

        self.target: Optional[Transform2d] = None
        self.should_extend = False

    def periodic(self):
        if self.target is not None:
            self.pivot.target_angle = self.target.translation().angle()
            # TODO: Software limits on wrist
            self.wrist.target_angle = self.target.rotation()
            if self.should_extend:
                self.elevator.target_extension = self.target.translation().norm()
            else:
                # TODO: Insert retracted elevator length
                self.elevator.target_extension = 0

    def at_target(self) -> bool:
        return (
            self.pivot.at_angle()
            and self.elevator.at_extension()
            and self.wrist.at_angle()
        )
