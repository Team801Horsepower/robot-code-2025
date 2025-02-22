from commands2 import CommandScheduler, Subsystem

from subsystems.arm import Arm
from subsystems.claw import Claw
from subsystems.climb import Climb


class Periscope(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.arm = Arm(scheduler)
        self.claw = Claw(scheduler)
        self.climb = Climb(scheduler)

    def periodic(self):
        pass
