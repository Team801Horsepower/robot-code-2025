from commands2 import CommandScheduler, Subsystem
from wpimath.geometry import Transform2d
from typing import Optional

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
        pass

    def at_target(self) -> bool:
        return (
            self.pivot.at_angle()
            and self.elevator.at_extension()
            and self.wrist.at_angle()
        )
