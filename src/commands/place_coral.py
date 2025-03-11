from commands2 import Command
import time

from subsystems.periscope import Periscope
import config


class PlaceCoral(Command):
    # def __init__(self, periscope: Periscope, target_level: int):
    def __init__(self, periscope: Periscope):
        self.periscope = periscope
        # self.target_level = target_level
        self.score_time = None

        self.finished = False

    def initialize(self):
        # self.periscope.arm.target = config.reef_setpoints[self.target_level]
        # self.periscope.arm.should_extend = True
        pass

    def execute(self):
        self.finished |= (
            not self.periscope.claw.has_coral()
            and self.score_time is not None
            and time.time() - self.score_time > 0.35
        )

        if self.periscope.claw.has_coral():
            if self.periscope.arm.at_target():
                self.periscope.claw.set(1)
                if self.score_time is None:
                    self.score_time = time.time()
        elif not self.finished:
            self.periscope.arm.target = config.transit_setpoint

    def isFinished(self) -> bool:
        # TODO: Should we wait until retraction is finished before saying the command is finished?
        return self.finished

    def end(self, interrupted: bool):
        self.periscope.claw.set(0)
