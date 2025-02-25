from commands2 import CommandScheduler, Subsystem, Time


class TurnSignals(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)
        self.led = False # TODO: change to actual LED strip initialization
        self.signal_left = False
        self.signal_right = False
        self.turnSignal = False

    def periodic(self):
        if self.turnSignal:
            if Time.time % 0.5 <= 0.25:
                self.led = True
            else:
                self.led = False
        pass
