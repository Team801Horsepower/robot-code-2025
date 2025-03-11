from commands2 import Command

from subsystems.periscope import Periscope
import config


class CollectCoral(Command):
    def __init__(self, periscope: Periscope, left_hps: bool):
        self.periscope = periscope
        self.left_hps = left_hps

    def initialize(self):
        self.periscope.arm.target = config.source_setpoint

    def execute(self):
        # if self.periscope.claw.has_coral():
        #     if self.periscope.arm.at_target():
        #         self.periscope.claw.set(1)
        #         if self.score_time is None:
        #             self.score_time = time.time()
        # else:
        #     self.periscope.arm.target = config.transit_setpoint

        # self.finished |= (
        #     not self.periscope.claw.has_coral()
        #     and self.score_time is not None
        #     and time.time() - self.score_time > 0.35
        # )

        self.periscope.claw.set(-1)

    def isFinished(self) -> bool:
        return self.periscope.claw.has_coral()

    def end(self, interrupted: bool):
        self.periscope.claw.set(0)
