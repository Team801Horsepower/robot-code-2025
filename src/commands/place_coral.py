from commands2 import Command
import time

from subsystems.periscope import Periscope


class PlaceCoral(Command):
    def __init__(self, periscope: Periscope):
        self.periscope = periscope
        self.score_time = None

        self.finished = False

    def initialize(self):
        pass

    def execute(self):
        self.finished |= (
            not self.periscope.claw.has_coral()
            and self.score_time is not None
            and time.time() - self.score_time > 0.35
        )

        if self.periscope.claw.has_coral() and self.periscope.arm.at_target():
            self.periscope.claw.set(1)
            if self.score_time is None:
                self.score_time = time.time()
        # TODO: I think replacing the else with the following
        #       will fix the issue where it wasn't fully scoring
        #       the coral, but I'm writing this after the meeting
        #       ended and so it's not tested.
        # elif self.score_time is None or self.finished:
        else:
            self.periscope.claw.set(0)

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted: bool):
        self.periscope.claw.set(0)
