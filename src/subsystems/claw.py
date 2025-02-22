from commands2 import CommandScheduler, Subsystem


class Claw(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

    def periodic(self):
        pass

    def run(self, power: float):
        pass

    def has_coral(self) -> bool:
        return False
