from commands2 import CommandScheduler, Subsystem

from subsystems.arm import Arm
from subsystems.claw import Claw
from subsystems.climber import Climber


class Periscope(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.arm = Arm(scheduler)
        self.claw = Claw(scheduler)
        self.climber = Climber(scheduler)
        scheduler.registerSubsystem(self)

    def periodic(self):
        pass
