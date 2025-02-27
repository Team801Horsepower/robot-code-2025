from commands2 import CommandScheduler, Subsystem


class Elevator(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.target_extension = 0.0

    def periodic(self):
        pass

    def at_extension(self) -> bool:
        return False

    def target_attainable(self) -> bool:
        return True
