from subsystems.chassis import Chassis
from subsystems.odometry import Odometry
from subsystems.turn_signals import TurnSignals
import config
import utils

from wpimath.geometry import Transform2d
from commands2 import CommandScheduler, Subsystem


class Drive(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        self.chassis = Chassis(scheduler)
        self.odometry = Odometry(scheduler)
        self.turn_signals = TurnSignals(scheduler)
        self.elevator_height = config.extension_range[0]
        self.last_filtered_vel = 0
        scheduler.registerSubsystem(self)

    def periodic(self):
        self.odometry.update(self.chassis)

    def drive(self, vel: Transform2d, field_oriented: bool = False):
        if field_oriented:
            translation = vel.translation().rotateBy(-self.odometry.rotation())
            vel = Transform2d(translation, vel.rotation())

        # Temporarily commented pending troubleshooting
        # slew_rate_limit = utils.lerp_over_table(
        #     config.drive_acc_lim, self.elevator_height
        # )[0]
        # filtered_vel = (
        #     vel.translation() / vel.translation().norm()
        # ) * self.slew_rate_limiter(slew_rate_limit, vel.translation().norm())
        # self.last_filtered_vel = vel.translation().norm()

        # self.chassis.drive(Transform2d(filtered_vel, vel.rotation()))

        self.chassis.drive(vel)

    def slew_rate_limiter(self, limit: float, velocity: float) -> float:
        return self.last_filtered_vel + utils.clamp(
            -limit, limit, velocity - self.last_filtered_vel
        )
