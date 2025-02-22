from commands2 import CommandScheduler, Subsystem


class Climber(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

    def periodic(self):
        pass

    def run(self, power: float):
        pass
