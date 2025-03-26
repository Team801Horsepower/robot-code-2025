from commands2 import CommandScheduler, Subsystem
from navx import AHRS

from subsystems.arm import Arm
from subsystems.claw import Claw
from subsystems.climber import Climber
from utils import time_f
import config


class Periscope(Subsystem):
    def __init__(self, scheduler: CommandScheduler, navx: AHRS):
        scheduler.registerSubsystem(self)

        self.arm = Arm(scheduler, navx)
        self.claw = Claw(scheduler)
        self.climber = Climber(scheduler)

    @time_f("periodic periscope")
    def periodic(self):
        self.arm.wrist_passthrough_allowed_by_algae = not self.claw.algae_detected()
        self.claw.limit_eject_power = self.arm.target == config.reef_setpoints[0]
        self.arm.pivot.should_power_limit = self.climber.climbed()
