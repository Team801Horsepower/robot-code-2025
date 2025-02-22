from commands2 import CommandScheduler, Subsystem


class TurnSignals(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.signal_left = False
        self.signal_right = False

    def periodic(self):
        pass
