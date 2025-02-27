from commands2 import CommandScheduler, Subsystem


class Pivot(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.target_angle = 0.0

    def periodic(self):
        pass

    def at_angle(self) -> bool:
        return False

    def target_attainable(self) -> bool:
        return True
