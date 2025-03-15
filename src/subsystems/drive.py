from subsystems.chassis import Chassis
from subsystems.odometry import Odometry
from subsystems.turn_signals import TurnSignals
import config
import utils
import time
from wpimath.geometry import Transform2d, Translation2d
from commands2 import CommandScheduler, Subsystem


class Drive(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        self.chassis = Chassis(scheduler)
        self.odometry = Odometry(scheduler)
        self.turn_signals = TurnSignals(scheduler)
        self.elevator_height = config.extension_range[0]
        self.pivot_acceleration = 0.0
        self.last_filtered_vel = Translation2d(0, 0)
        self.last_update_time = time.time()
        scheduler.registerSubsystem(self)

    def periodic(self):
        self.odometry.update(self.chassis)

    def drive(self, vel: Transform2d, field_oriented: bool = False):
        if field_oriented:
            translation = vel.translation().rotateBy(-self.odometry.rotation())
            vel = Transform2d(translation, vel.rotation())

        slew_rate_limit = (
            utils.lerp_over_table(config.drive_acc_lim, self.elevator_height)[0]
            # Pivot correction disabled for now
            # + self.pivot_acceleration * 0.1
        )
        filtered_vel = self.slew_rate_limiter(slew_rate_limit, vel.translation())
        self.last_filtered_vel = filtered_vel
        self.chassis.drive(Transform2d(filtered_vel, vel.rotation()))

    def slew_rate_limiter(self, limit: float, velocity: Translation2d) -> Translation2d:
        delta_time = time.time() - self.last_update_time
        self.last_update_time = time.time()
        delta_velocity = velocity - self.last_filtered_vel
        if delta_velocity.norm() != 0:
            delta_velocity = (
                (delta_velocity / delta_velocity.norm())
                * delta_time
                * utils.clamp(-limit, limit, delta_velocity.norm() / delta_time)
            )
        return self.last_filtered_vel + delta_velocity
