from commands2 import Command

from subsystems.periscope import Periscope
import config


class PlaceCoral(Command):
    def __init__(self, periscope: Periscope, target_level: int):
        self.periscope = periscope
        self.target_level = target_level

    def initialize(self):
        self.periscope.arm.target = config.arm_reef_presets[target_level]
        self.periscope.arm.should_extend = True

    def execute(self):
        if self.periscope.claw.has_coral():
            if self.periscope.arm.at_target():
                self.periscope.claw.run(-1)
        else:
            self.periscope.arm.target = config.arm_stow_preset

    def isFinished(self) -> bool:
        # TODO: Should we wait until retraction is finished before saying the command is finished?
        return not self.periscope.claw.has_coral()
