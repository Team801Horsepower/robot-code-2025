from subsystems.chassis import Chassis
from subsystems.odometry import Odometry
from subsystems.turn_signals import TurnSignals

from wpimath.geometry import Transform2d
from commands2 import CommandScheduler, Subsystem


class Drive(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        self.chassis = Chassis(scheduler)
        self.odometry = Odometry(scheduler)
        self.turn_signals = TurnSignals(scheduler)

        scheduler.registerSubsystem(self)

    def periodic(self):
        self.odometry.update(self.chassis)

    def drive(self, vel: Transform2d, field_oriented: bool = False):
        if field_oriented:
            translation = vel.translation().rotateBy(-self.odometry.rotation())
            vel = Transform2d(translation, vel.rotation())
        self.chassis.drive(vel)
